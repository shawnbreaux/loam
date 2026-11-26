"""`loam monitor`: a live web dashboard for a Loam site, and replays of recorded runs.

It subscribes to the site's Loam Wire topics, turns them into compact frames
(measured-minus-design map, machines, work windows, tasks, safety state) and

* serves them live: open http://localhost:<port>/ (Server-Sent Events, no
  JavaScript libraries, nothing leaves the machine);
* records them as JSON lines (`--record`);
* builds a self-contained replay page from a recording (`--replay run.jsonl --html out.html`).

On a simulated site it also shows the world's ground-truth score, labelled as
such; a real site has no truth topic.
"""
from __future__ import annotations

import base64
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import numpy as np

from . import wire
from .checker import Model
from .grid import SiteGrid

MAX_PX = 160          # longest side of the map sent to browsers
PAGE = os.path.join(os.path.dirname(__file__), "monitor.html")


class Frames:
    """Folds Wire messages into dashboard state and emits frames."""

    def __init__(self, model: Model, base_dir: str):
        self.g = SiteGrid(model, base_dir)
        self.m = model
        self.design, _ = self.g.design_and_existing()
        self.step = max(1, int(np.ceil(max(self.g.nx, self.g.ny) / MAX_PX)))
        finish = [z for z in model.zones.values() if z.get("role") == "finish" and z.get("tolerance")]
        self.tol = float(finish[0]["tolerance"].to(self.g.unit)) if finish else 0.1
        self.site_info = {
            "site": model.site["name"], "unit": self.g.unit, "tol": self.tol,
            "origin": [self.g.x0, self.g.y0], "cell": self.g.cell * self.step,
            "shape": [int(np.ceil(self.g.ny / self.step)), int(np.ceil(self.g.nx / self.step))],
            "zones": {n: {"role": z.get("role"), "polygon": z["area"]["polygon"]}
                      for n, z in model.zones.items() if "polygon" in (z.get("area") or {})},
            "stockpiles": {n: sp["area"]["polygon"] for n, sp in model.stockpiles.items()
                           if "polygon" in (sp.get("area") or {})},
            "boundary": self.g.boundary,
            "machines": {n: md["class"] for n, md in model.machines.items()},
        }
        self.state: dict[str, Any] = {"t": None, "machines": {}, "orders": {}, "status": None, "truth": None,
                                      "people": {}, "events": [], "map": None, "map_t": None, "estop": False,
                                      "terrain": None, "messages": 0}

    def _t(self, msg: dict) -> float | None:
        if self.state["status"] is None and "timestamp" not in msg:
            return None
        try:
            return wire.parse_ts(msg["timestamp"])
        except (KeyError, ValueError):
            return None

    def event(self, ts: float | None, text: str, level: str = "info") -> None:
        self.state["events"] = ([{"ts": ts, "text": text, "level": level}] + self.state["events"])[:60]

    def feed(self, topic: str, msg: dict) -> str | None:
        """Update state; return the kind of change for the frame, or None."""
        kind = topic.rsplit("/", 1)[-1]
        s = self.state
        ts = self._t(msg)
        s["messages"] += 1
        if ts is not None:
            s["t"] = max(s["t"] or ts, ts)
        if kind == "terrain" and msg.get("data"):
            z = wire.decode_heights(msg["data"], tuple(msg["shape"])).astype(float)
            full = z - self.design
            ok = np.isfinite(full)
            if ok.any():
                s["terrain"] = {"within": float(np.mean(np.abs(full[ok]) <= self.tol)),
                                "rms": float(np.sqrt(np.mean(full[ok] ** 2))), "mean": float(np.mean(full[ok]))}
            r = full[:: self.step, :: self.step]
            q = np.clip(np.round(r / self.tol * 10), -127, 127)
            q = np.where(np.isfinite(q), q, -128).astype(np.int8)
            s["map"] = base64.b64encode(q.tobytes()).decode("ascii")
            s["map_t"] = ts
            return "map"
        if kind == "state":
            s["machines"][msg["machine"]] = {"x": msg["pose"]["x"], "y": msg["pose"]["y"],
                                             "h": msg["pose"]["heading"], "safety": msg["safetyState"],
                                             "order": msg.get("orderId"), "progress": msg.get("progress", 0),
                                             "errors": [e["code"] for e in msg.get("errors", [])],
                                             "stopLatencyS": msg.get("stopLatencyS")}
            return "machines"
        if kind == "order":
            o = {"task": msg["task"], "op": msg["op"], "window": msg.get("window"),
                 "deposit": (msg.get("deposit") or {}).get("window"), "id": msg["orderId"]}
            s["orders"][msg["machine"]] = o
            return "machines"
        if kind == "status":
            prev = s["status"] or {}
            for name, tk in msg.get("tasks", {}).items():
                old = (prev.get("tasks") or {}).get(name, {})
                if tk.get("done") is not None and old.get("done") is None:
                    self.event(ts, f"task {name} done after {tk['passes']} passes", "good")
            for r in set(msg.get("rules", [])) - set(prev.get("rules", [])):
                self.event(ts, f"{r}: active", "warning")
            for r in set(prev.get("rules", [])) - set(msg.get("rules", [])):
                self.event(ts, f"{r}: cleared")
            if msg.get("replans", 0) > prev.get("replans", 0):
                self.event(ts, f"replan #{msg['replans']}")
            s["status"] = {k: msg.get(k) for k in ("t", "tasks", "machines", "rules", "estop", "replans", "surveys",
                                                    "probe_age_min", "abandoned", "soil", "measured", "done")}
            return "status"
        if kind == "connection":
            self.event(ts, f"{msg['machine']}: {msg['connectionState']}",
                       "info" if msg["connectionState"] == "online" else "critical")
            return "status"
        if kind == "estop":
            s["estop"] = bool(msg["pressed"])
            self.event(ts, "SITE E-STOP pressed" if msg["pressed"] else "e-stop released",
                       "critical" if msg["pressed"] else "info")
            return "status"
        if kind == "detection":
            if msg["kind"] == "clear":
                if s["people"].pop(msg["sensor"], None) is not None:
                    self.event(ts, "person left the work area")
            else:
                if msg["sensor"] not in s["people"]:
                    self.event(ts, f"{msg['kind']} detected by {msg['sensor']}", "critical")
                s["people"][msg["sensor"]] = [msg["x"], msg["y"]]
            return "machines"
        if kind == "instantAction" and msg["action"] in ("stop", "hold"):
            self.event(ts, f"{msg['machine']}: {msg['action']} ({msg['reason']})", "warning")
            return None
        if kind == "truth":
            s["truth"] = {"t": msg.get("t"), "zones": msg.get("zones"), "final": msg.get("final", False)}
            return "status"
        return None

    def frame(self, full: bool = False) -> dict:
        s = self.state
        f = {"t": s["t"], "machines": s["machines"], "orders": s["orders"], "status": s["status"],
             "truth": s["truth"], "people": s["people"], "events": s["events"][:25], "estop": s["estop"]}
        if full:
            f["map"], f["map_t"] = s["map"], s["map_t"]
        return f


    def metrics(self) -> str:
        """The site state in the Prometheus text format (for Grafana and alerting)."""
        s, st, site = self.state, self.state["status"] or {}, self.m.site["name"]
        out: list[str] = []

        def gauge(name: str, help_: str, rows: list[tuple[dict, float | None]], kind: str = "gauge") -> None:
            vals = [(lab, float(v)) for lab, v in rows if v is not None]
            if not vals:
                return
            out.append(f"# HELP loam_{name} {help_}")
            out.append(f"# TYPE loam_{name} {kind}")
            for lab, x in vals:
                labels = ",".join(f'{k}="{_esc(str(w))}"' for k, w in {"site": site, **lab}.items())
                out.append(f"loam_{name}{{{labels}}} {x:.6g}")

        gauge("messages_total", "Wire messages seen by the monitor.", [({}, s["messages"])], "counter")
        gauge("site_minutes", "Site clock, minutes since start.", [({}, st.get("t"))])
        gauge("estop_active", "1 while the site e-stop is pressed.", [({}, float(s["estop"]))])
        gauge("people_detected", "People currently detected in the work area.", [({}, len(s["people"]))])
        gauge("surveys_total", "Surveys ingested by the controller.", [({}, st.get("surveys"))], "counter")
        gauge("replans_total", "Replans by the controller.", [({}, st.get("replans"))], "counter")
        gauge("probe_age_minutes", "Age of the newest survey, site minutes.", [({}, st.get("probe_age_min"))])
        ms = s["machines"]
        gauge("machine_progress", "Progress of the current pass (0-1).", [({"machine": n}, m["progress"])
                                                                         for n, m in ms.items()])
        gauge("machine_safety", "1 for the machine's current safetyState.",
              [({"machine": n, "state": k}, float(m["safety"] == k)) for n, m in ms.items()
               for k in ("ok", "degraded", "stopped")])
        gauge("machine_stop_latency_seconds", "Worst e-stop latency the machine has reported.",
              [({"machine": n}, m.get("stopLatencyS")) for n, m in ms.items()])
        tasks = st.get("tasks") or {}
        gauge("task_passes_total", "Passes issued per task.",
              [({"task": n}, t.get("passes")) for n, t in tasks.items()], "counter")
        gauge("task_done", "1 once the task's until-condition holds.",
              [({"task": n}, float(t.get("done") is not None)) for n, t in tasks.items()])
        tr = s["terrain"]
        if tr:
            gauge("terrain_within_tolerance_ratio", "Share of measured cells within the finish tolerance.",
                  [({}, tr["within"])])
            gauge("terrain_residual_rms", f"RMS of measured minus design, {self.g.unit}.", [({}, tr["rms"])])
            gauge("terrain_residual_mean", f"Mean of measured minus design, {self.g.unit}.", [({}, tr["mean"])])
        truth = (s["truth"] or {}).get("zones") or {}
        gauge("truth_within_tolerance_ratio", "Simulated sites only: true share within tolerance per zone.",
              [({"zone": z}, (v.get("within_tolerance_pct") or 0) / 100) for z, v in truth.items()
               if isinstance(v, dict) and "within_tolerance_pct" in v])
        return "\n".join(out) + "\n"


def _esc(v: str) -> str:
    return v.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def page(site_info: dict, frames: list[dict] | None = None) -> str:
    with open(PAGE, encoding="utf-8") as f:
        html = f.read()
    data = {"site": site_info, "frames": frames}
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    return html.replace("/*LOAM_DATA*/null", blob)


def serve(model: Model, base_dir: str, broker: str, port: int, record: str | None) -> None:
    from .runtime.bus import Bus
    site = model.site["name"]
    fr = Frames(model, base_dir)
    clients: list[queue.Queue] = []
    lock = threading.Lock()
    bus = Bus(broker, f"loam-monitor-{site}-{os.getpid()}").start([f"loam/v0/{site}/#", f"sim/{site}/world/truth"])
    rec = open(record, "w", encoding="utf-8") if record else None  # noqa: SIM115

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a: Any) -> None:
            pass

        def do_GET(self) -> None:
            if self.path == "/metrics":
                with lock:
                    body = fr.metrics().encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; version=0.0.4; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q: queue.Queue = queue.Queue(maxsize=200)
                with lock:
                    q.put(fr.frame(full=True))
                    clients.append(q)
                try:
                    while True:
                        item = q.get()
                        self.wfile.write(f"data: {json.dumps(item, separators=(',', ':'))}\n\n".encode())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with lock:
                        clients.remove(q)
                return
            body = page(fr.site_info).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"loam monitor: http://127.0.0.1:{port}/  metrics: /metrics  (site {site})", flush=True)
    if rec is not None:
        rec.write(json.dumps({"site": fr.site_info}) + "\n")
    last_push, dirty_map = 0.0, False
    try:
        while True:
            changed = False
            for topic, msg in bus.drain(0.05):
                if topic == f"sim/{site}/end":
                    return
                k = fr.feed(topic, msg)
                changed = changed or k is not None
                dirty_map = dirty_map or k == "map"
            now = time.time()
            if changed and now - last_push >= 0.2:
                last_push = now
                f = fr.frame(full=dirty_map)
                if rec is not None:
                    rec.write(json.dumps(f, separators=(",", ":")) + "\n")
                with lock:
                    for q in clients:
                        if not q.full():
                            q.put(f)
                dirty_map = False
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        if rec is not None:
            rec.close()
        bus.stop()


def replay_html(recording: str, out: str, max_frames: int = 900, max_maps: int = 120, split: bool = False) -> dict:
    """Build a replay page from a `--record` file, at most `max_maps` survey maps. With `split`, `out` is a
    folder: index.html plus run.json, which the page loads (for hosts that prefer data beside the page)."""
    with open(recording, encoding="utf-8") as f:
        site = json.loads(f.readline())["site"]
        frames = [json.loads(line) for line in f if line.strip()]
    if not frames:
        raise ValueError(f"{recording}: no frames")
    maps = [i for i, fr in enumerate(frames) if fr.get("map")]
    if len(maps) > max_maps:                     # keep surveys evenly in time, always the last one
        kept = {maps[int(k * (len(maps) - 1) / (max_maps - 1))] for k in range(max_maps)}
        for i in maps:
            if i not in kept:
                frames[i] = {k: v for k, v in frames[i].items() if k not in ("map", "map_t")}
    # keep every frame with a map, then thin the rest evenly to the budget
    keep = [i for i, fr in enumerate(frames) if fr.get("map")]
    rest = [i for i in range(len(frames)) if i not in set(keep)]
    budget = max(0, max_frames - len(keep))
    if len(rest) > budget:
        rest = [rest[int(k * len(rest) / budget)] for k in range(budget)] if budget else []
    sel = sorted(set(keep) | set(rest) | {len(frames) - 1})
    if split:
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "run.json"), "w", encoding="utf-8") as f:
            json.dump({"site": site, "frames": [frames[i] for i in sel]}, f, separators=(",", ":"))
        with open(PAGE, encoding="utf-8") as f, open(os.path.join(out, "index.html"), "w", encoding="utf-8") as g:
            g.write(f.read())
    else:
        with open(out, "w", encoding="utf-8") as f:
            f.write(page(site, [frames[i] for i in sel]))
    return {"out": out, "frames": len(sel), "maps": len(keep)}
