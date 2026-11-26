"""`loam run`: the site controller. Plans passes, sends Loam Wire orders, watches state and terrain.

It is the reference simulator's planner with the world taken away: the same
task, rule, replan and safety logic (see `Sim.step`), but

* the measured surface comes from `terrain` messages, not a simulated scan;
* a pass lands in the plan when its machine reports progress 1.0, not on a timer;
* people, weather and e-stops come from site sensors;
* a machine that is offline, has lost its link, or reports a degraded or
  stopped safety state gets no new orders, and a pass it never finished is
  abandoned (its expected effect is withdrawn and the next survey shows what
  really happened).

The controller never sees the `sim/...` physics topics, so it runs unchanged
against real machines.
"""
from __future__ import annotations

import json
import math
import time
from typing import Any

import numpy as np

from .. import wire
from ..checker import Model
from ..sim import Events, Sim
from .bus import Bus, Clock

STATE_TIMEOUT_S = 3.0
HELD_PASS_MIN = 30.0      # a pass whose machine has not been in a safe, working state for this long is withdrawn


class Controller(Sim):
    def __init__(self, model: Model, base_dir: str, bus: Bus, clock: Clock, record: str | None = None,
                 log=print, max_minutes: float = 48 * 60, dt: float = 0.25):
        super().__init__(model, base_dir, events=Events.none(), log=log, max_minutes=max_minutes, dt=dt,
                         start_epoch=clock.epoch)
        self.bus, self.clock = bus, clock
        self.record = open(record, "w", encoding="utf-8") if record else None  # noqa: SIM115
        self.mstate: dict[str, dict[str, Any]] = {n: {"conn": "unknown", "safety": "unknown", "seen": 0.0,
                                                      "x": None, "y": None, "order": None, "progress": 0.0,
                                                      "ts": -1e9, "finished": None}
                                                  for n in self.machines}
        self.terrain_since = 0.0
        # Passes landed recently: (finish time, cells, change to pred, change to p_loose, flags). A survey
        # taken before a pass finished must not see that pass in the plan, or the pass counts twice.
        self.recent: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
        self.order_of: dict[str, str] = {}
        self.terrain: dict | None = None
        self.people: dict[str, tuple[float, float]] = {}
        self.rain = 0.0
        self.estop = False
        self.abandoned = 0
        self.status_seq = 0
        self.t = clock.minutes()

    # -- outgoing -------------------------------------------------------------------------
    def emit(self, topic: str, msg: dict) -> None:
        if self.record is not None:
            self.record.write(json.dumps({"topic": topic, "payload": msg}) + "\n")
        self.bus.pub(topic, msg, qos=1)

    def _issue(self, m, name, op, w, parts, speed, dist, limits, deposit_msg, target=None) -> None:
        super()._issue(m, name, op, w, parts, speed, dist, limits, deposit_msg, target)
        m.busy_until = math.inf                     # lands when the machine says so
        self.order_of[m.name] = f"{name}-{self.order_seq}"

    # -- incoming -------------------------------------------------------------------------
    def handle(self, topic: str, msg: dict) -> None:
        kind = topic.rsplit("/", 1)[-1]
        if kind == "terrain":
            self.terrain, self.terrain_since = msg, time.time()
        elif kind == "state" and msg.get("machine") in self.mstate:
            n = msg["machine"]
            s = self.mstate[n]
            ts = self._minutes(msg["timestamp"])
            if msg["safetyState"] != "ok" and s.get("unsafe_since") is None:
                s["unsafe_since"] = ts
            elif msg["safetyState"] == "ok":
                s["unsafe_since"] = None
            if msg["safetyState"] != s["safety"] and s["safety"] != "unknown":
                why = ", ".join(e["code"] for e in msg.get("errors", []))
                self.log(f"{n}: safety {msg['safetyState']}" + (f" ({why})" if why else ""))
            s.update(seen=time.time(), safety=msg["safetyState"], x=msg["pose"]["x"], y=msg["pose"]["y"],
                     order=msg.get("orderId"), progress=msg.get("progress", 0.0), ts=max(s["ts"], ts))
            if s["conn"] == "unknown":
                s["conn"] = "online"
            if n in self.inflight and msg.get("orderId") == self.order_of.get(n) and msg.get("progress", 0) >= 1:
                self.machines[n].busy_until = self.t
                s["finished"] = ts
        elif kind == "connection" and msg.get("machine") in self.mstate:
            n = msg["machine"]
            self.mstate[n]["conn"] = msg["connectionState"]
            if msg["connectionState"] != "online":
                self.log(f"{n}: {msg['connectionState']}")
                self.abandon(n, msg["connectionState"])
            else:
                self.log(f"{n}: online")
        elif kind == "detection":
            if msg["kind"] == "clear":
                self.people.pop(msg["sensor"], None)
            elif msg["kind"] == "person":
                self.people[msg["sensor"]] = (msg["x"], msg["y"])
        elif kind == "weather":
            self.rain = float(msg.get("rain", 0.0))
        elif kind == "estop":
            if bool(msg["pressed"]) != self.estop:
                self.log("SITE E-STOP pressed" if msg["pressed"] else "site e-stop released")
            self.estop = bool(msg["pressed"])

    def step(self) -> bool:
        for n in list(self.inflight):
            since = self.mstate[n].get("unsafe_since")
            if since is not None and self.t - since > HELD_PASS_MIN and not self.estop:
                self.abandon(n, f"machine not working safely for {HELD_PASS_MIN:.0f} min")
        return super().step()

    def abandon(self, n: str, why: str) -> None:
        """Withdraw a pass that will not complete: its predicted effect never reaches the plan."""
        ps = self.inflight.pop(n, None)
        if ps is None:
            return
        np.add.at(self.plan_dz.ravel(), ps.idx, -ps.dz_pred)
        np.add.at(self.plan_dloose.ravel(), ps.idx, -ps.dloose_pred)
        self.machines[n].busy_until = self.t
        self.abandoned += 1
        self.log(f"{n}: pass {self.order_of.get(n)} abandoned ({why}); the next survey shows what was done")

    def _minutes(self, stamp: str) -> float:
        return (wire.parse_ts(stamp) - self.clock.epoch) / 60.0

    def land(self, ps) -> None:
        u = np.unique(ps.idx)
        pred, pl = self.pred.ravel(), self.p_loose.ravel()
        b_pred, b_pl = pred[u].copy(), pl[u].copy()
        super().land(ps)
        fin = self.mstate[ps.machine]["finished"]
        self.recent.append((self.t if fin is None else fin, u, pred[u] - b_pred, pl[u] - b_pl))
        self.recent = [r for r in self.recent if r[0] > self.t - 60.0][-256:]

    # -- the Context the program's conditions see --------------------------------------------
    def probe(self) -> None:
        msg = self.terrain
        if msg is None:
            return
        if tuple(msg["shape"]) != (self.ny, self.nx) or not msg.get("data"):
            self.terrain = None
            self.log(f"terrain from {msg.get('probe')} ignored: shape {msg['shape']} is not the site grid")
            return
        scan_t = self._minutes(msg["timestamp"])
        # Wait until every machine with a pass in flight has reported a state at or after the scan,
        # so we know which passes the scan saw. A silent machine is not waited for long.
        waiting = [n for n in self.inflight if self.mstate[n]["ts"] < scan_t]
        if waiting and time.time() - self.terrain_since < 2.0:
            return
        self.terrain = None
        late = [r for r in self.recent if r[0] > scan_t]          # in the plan, not yet in the scan
        pred, pl = self.pred.ravel(), self.p_loose.ravel()
        flags = (self.touched_dep, self.touched_comp, self.touched_other)
        saved = []
        for _, u, dp, dl in late:
            pred[u] -= dp
            pl[u] -= dl
            saved.append([f.ravel()[u].copy() for f in flags])
            for f in flags:
                f.ravel()[u] = False
        self.ingest_survey(wire.decode_heights(msg["data"], (self.ny, self.nx)).astype(float), msg["probe"])
        for (_, u, dp, dl), fl in zip(late, saved, strict=True):
            pred[u] += dp
            pl[u] += dl
            for f, v in zip(flags, fl, strict=True):
                f.ravel()[u] |= v

    def proximity(self, a: str, b: str) -> float:
        d = [math.hypot(px - s["x"], py - s["y"]) for px, py in self.people.values()
             for s in self.mstate.values() if s["x"] is not None]
        return min(d) * self.u if d else math.inf

    def weather(self, kind: str) -> float:
        return self.rain if kind == "rain" else 0.0

    def apply_rules(self) -> dict[str, Any]:
        eff = super().apply_rules()
        if self.estop:
            eff["stop"] = True
        return eff

    def machine_ready(self, name: str) -> bool:
        s = self.mstate[name]
        fresh = time.time() - s["seen"] < STATE_TIMEOUT_S
        return s["conn"] == "online" and fresh and s["safety"] == "ok" and not self.estop

    # -- reporting ---------------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "t": round(self.t, 3),
            "tasks": {n: {"started": self.started[n], "done": self.done_at[n], "passes": self.task_passes[n]}
                      for n in self.m.tasks},
            "machines": {n: {"conn": s["conn"], "safety": s["safety"], "ready": self.machine_ready(n),
                             "order": self.order_of.get(n) if n in self.inflight else None}
                         for n, s in self.mstate.items()},
            "rules": [n for n, on in self.active_rules.items() if on]
            + (["stale-data hold"] if self.builtin_stale else []),
            "estop": self.estop, "replans": len(self.replans), "surveys": self.probe_count,
            "probe_age_min": round(self.t - self.last_probe, 2) if self.has_survey else None,
            "abandoned": self.abandoned,
            "soil": {"swell": round(self.est_swell, 3), "keep": round(self.est_keep, 3)},
            "measured": self._measured(),
            "done": all(v is not None for v in self.done_at.values()),
        }

    def _measured(self) -> dict:
        out = {}
        for n, z in self.m.zones.items():
            if z.get("role") == "finish" and self.grid.masks.get(n) is not None and self.has_survey:
                mk = self.grid.masks[n]
                tol = float(z["tolerance"].to(self.unit)) if z.get("tolerance") else 0.1
                out[n] = round(100 * float((np.abs(self.meas - self.design)[mk] <= tol).mean()), 2)
        return out

    def report(self) -> dict[str, Any]:
        r = super().report()
        r.pop("true_ground", None)           # the controller cannot know the truth; the world reports it
        r.pop("ledger_error", None)
        r["abandoned_passes"] = self.abandoned
        return r


def run_controller(model: Model, base_dir: str, broker: str, record: str | None = None,
                   max_minutes: float = 48 * 60, wait_for_clock: bool = True) -> dict:
    site = model.site["name"]
    bus = Bus(broker, f"loam-controller-{site}").start([f"loam/v0/{site}/#", f"sim/{site}/clock"])
    clock = Clock()
    if wait_for_clock:                               # a simulated site publishes its clock; a real one does not
        from .bus import wait_clock
        clock = wait_clock(bus, site, timeout=5.0)
    logs: list[str] = []

    def log(s: str) -> None:
        print(s, flush=True)
        logs.append(s)
    ctl = Controller(model, base_dir, bus, clock, record, log, max_minutes)
    last_hb = last_status = 0.0
    hb_seq = 0
    try:
        while True:
            for topic, msg in bus.drain(0.002):
                if topic.startswith(f"loam/v0/{site}/"):
                    ctl.handle(topic, msg)
            ctl.t = clock.minutes()
            done = ctl.step()
            now = time.time()
            if now - last_hb >= 1.0:                 # spec: 1 Hz safety heartbeat
                last_hb, hb_seq = now, hb_seq + 1
                bus.pub(wire.topic(site, None, "safety"), wire.safety(site, hb_seq, ts=ctl.ts()))
            if now - last_status >= 1.0 or done:
                last_status, ctl.status_seq = now, ctl.status_seq + 1
                bus.pub(wire.topic(site, None, "status"), wire.status(site, ctl.status_seq, ctl.status(), ctl.ts()),
                        retain=True)
            if done or ctl.t >= max_minutes:
                ctl.log("all tasks done" if done else "time limit reached")
                break
            time.sleep(max(0.002, clock.wall(ctl.dt) - (time.time() - now)))
        rep = ctl.report()
        rep["log"] = logs
        return rep
    finally:
        if ctl.record is not None:
            ctl.record.close()
        time.sleep(0.2)
        bus.stop()
