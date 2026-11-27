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
from .bus import Bus, Clock, tolerant

# A machine not heard from for this long gets no new orders.
STATE_TIMEOUT_S = 3.0
ORDER_RETRY_MIN = 1.0     # after a machine reports it cannot run an order, before it is given another
HELD_PASS_MIN = 30.0      # a pass whose machine has not been in a safe, working state for this long is withdrawn
# How far a machine's or probe's clock may differ from the controller's. Surveys and finished passes are put in
# order by their timestamps, so a clock that is off makes the plan count a pass twice or not at all.
MAX_CLOCK_SKEW_S = 2.0


def _r3(t: float | None) -> float | None:
    """A site time in minutes rounded to the millisecond-ish precision worth sending, or None."""
    return None if t is None else round(t, 3)


class Controller(Sim):
    """The planner connected to a live site.

    It inherits tasks, rules, replanning and the pass planner from the simulator (`Sim`) and replaces the
    simulator's make-believe with messages: orders go out over Loam Wire, and surveys, machine states and sensor
    readings come back.

    `mstate` holds what is known of each machine:

        conn          "online", "offline", "connectionBroken" or "unknown"
        safety        the machine's last reported safetyState
        seen          wall-clock time of its last state message
        x, y          last reported position, site coordinates
        order         the order it says it is running
        progress      how far through it, 0 to 1
        ts            site time of its newest state message, minutes
        finished      site time at which it reported its pass complete
        unsafe_since  site time since which it has not been "ok"
        retry_at      no new order before this site time
        skew          seconds its clock differs from the controller's (real sites only)
    """
    def __init__(self, model: Model, base_dir: str, bus: Bus, clock: Clock, record: str | None = None,
                 log=print, max_minutes: float = 48 * 60, dt: float = 0.25):
        """`bus` and `clock` are already connected. `record`, if given, is a file that receives every message sent,
        one JSON object per line.
        """
        super().__init__(model, base_dir, events=Events.none(), log=log, max_minutes=max_minutes, dt=dt,
                         start_epoch=clock.epoch)
        self.bus, self.clock = bus, clock
        self.record = open(record, "w", encoding="utf-8") if record else None  # noqa: SIM115
        self.mstate: dict[str, dict[str, Any]] = {n: {"conn": "unknown", "safety": "unknown", "seen": 0.0,
                                                      "x": None, "y": None, "order": None, "progress": 0.0,
                                                      "ts": -1e9, "finished": None}
                                                  for n in self.machines}
        self.terrain_since = 0.0
        # Passes landed recently: (finish time, cells, what the pass added to each of the planner's four
        # per-cell books, which of the three "touched" flags it set). A survey taken before a pass finished
        # must not see that pass in the plan, or the pass counts twice.
        self.recent: list[tuple[float, np.ndarray, list[np.ndarray], list[np.ndarray]]] = []
        self.order_of: dict[str, str] = {}
        self.terrain: dict | None = None
        self.people: dict[str, tuple[float, float, float]] = {}   # per sensor: x, y and how well they are known
        self.rain = 0.0
        self.estop = False
        # Order ids must not repeat when the controller is restarted in the middle of a job: a machine (or the
        # simulated world) that has already done "strip_A-5" would take a new order of that name for the old one
        # and report it finished without working. So every run of the controller tags its ids with its start time.
        self.order_tag = np.base_repr(int(time.time()), 36).lower() + "-"
        self.abandoned = 0
        self.status_seq = 0
        self.t = clock.minutes()

    # -- outgoing -------------------------------------------------------------------------
    def emit(self, topic: str, msg: dict) -> None:
        """Send one message (and write it to the record file, if there is one)."""
        if self.record is not None:
            self.record.write(json.dumps({"topic": topic, "payload": msg}) + "\n")
        self.bus.pub(topic, msg, qos=1)

    def _issue(self, m, name, op, w, parts, speed, dist, limits, deposit_msg, target=None) -> None:
        """Send an order as the simulator would, but leave its finish time open: the pass is done when the machine
        says so.
        """
        super()._issue(m, name, op, w, parts, speed, dist, limits, deposit_msg, target)
        m.busy_until = math.inf                     # lands when the machine says so
        self.order_of[m.name] = self.order_id(name)

    # -- incoming -------------------------------------------------------------------------
    def handle(self, topic: str, msg: dict) -> None:
        """Take in one Loam Wire message.

        Terrain is kept for `probe`. A machine's state updates what is known of it, completes its pass at progress
        1.0, or abandons the pass if the machine reports `orderFailed`. A lost connection abandons the pass at
        once. Detections, weather and the e-stop feed the program's rules.
        """
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
            # On a real site (real time), a state stamped far from now means that machine's clock is wrong.
            skew = time.time() - wire.parse_ts(msg["timestamp"]) if self.clock.speed == 1.0 else 0.0
            if (abs(skew) > MAX_CLOCK_SKEW_S) != (abs(s.get("skew", 0.0)) > MAX_CLOCK_SKEW_S):
                self.log(f"{n}: clock is {skew:+.1f} s from the controller's; no orders until it is synchronised"
                         if abs(skew) > MAX_CLOCK_SKEW_S else f"{n}: clock synchronised")
            s["skew"] = skew
            s.update(seen=time.time(), safety=msg["safetyState"], x=msg["pose"]["x"], y=msg["pose"]["y"],
                     order=msg.get("orderId"), progress=msg.get("progress", 0.0), ts=max(s["ts"], ts))
            if s["conn"] == "unknown":
                s["conn"] = "online"
            # Only a report about the order this controller is waiting for counts; a state left over from an earlier
            # order must not complete or fail the current one.
            mine = n in self.inflight and msg.get("orderId") == self.order_of.get(n)
            failed = next((e for e in msg.get("errors", []) if e.get("code") == "orderFailed"), None)
            if mine and failed is not None:               # the machine cannot run it: do not wait for it
                self.abandon(n, f"machine could not run it: {failed.get('text') or 'no reason given'}",
                             cooldown=ORDER_RETRY_MIN)
            elif mine and msg.get("progress", 0) >= 1:
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
            elif msg["kind"] in ("person", "unknown"):    # unknown: a sensor that cannot tell, or has failed
                self.people[msg["sensor"]] = (float(msg["x"]), float(msg["y"]), float(msg.get("radius") or 0.0))
        elif kind == "weather":
            self.rain = float(msg.get("rain", 0.0))
        elif kind == "estop":
            if bool(msg["pressed"]) != self.estop:
                self.log("SITE E-STOP pressed" if msg["pressed"] else "site e-stop released")
            self.estop = bool(msg["pressed"])

    def step(self) -> bool:
        """One planning step. First withdraw any pass whose machine has been stopped or degraded for too long (not
        during a site e-stop, which holds every machine on purpose).
        """
        for n in list(self.inflight):
            since = self.mstate[n].get("unsafe_since")
            if since is not None and self.t - since > HELD_PASS_MIN and not self.estop:
                self.abandon(n, f"machine not working safely for {HELD_PASS_MIN:.0f} min")
        return super().step()

    def abandon(self, n: str, why: str, cooldown: float = 0.0) -> None:
        """Withdraw a pass that will not complete: its predicted effect never reaches the plan.
        The machine gets no new order for `cooldown` site minutes."""
        ps = self.inflight.pop(n, None)
        if ps is None:
            return
        # Take the pass's expected change back out of the plan.
        np.add.at(self.plan_dz.ravel(), ps.idx, -ps.dz_pred)
        np.add.at(self.plan_dloose.ravel(), ps.idx, -ps.dloose_pred)
        self.machines[n].busy_until = self.t
        self.mstate[n]["retry_at"] = self.t + cooldown
        self.abandoned += 1
        self.log(f"{n}: pass {self.order_of.get(n)} abandoned ({why}); the next survey shows what was done")

    def _books(self) -> list[np.ndarray]:
        """The planner's four per-cell books as flat views: predicted height, predicted loose fill, and the fill
        and the rolling expected since the last survey (what the soil factors are learned from).
        """
        return [a.ravel() for a in (self.pred, self.p_loose, self.dep_pred, self.loose_rolled_pred)]

    def _flags(self) -> list[np.ndarray]:
        """Flat views of the three flags that say what kind of work touched each cell since the last survey."""
        return [a.ravel() for a in (self.touched_dep, self.touched_comp, self.touched_other)]

    def _minutes(self, stamp: str) -> float:
        """A wire timestamp as site minutes since the start of the job."""
        return (wire.parse_ts(stamp) - self.clock.epoch) / 60.0

    def land(self, ps) -> None:
        """Book a finished pass into the plan, and remember what it changed, in case a survey taken before it finished
        arrives afterwards.
        """
        u = np.unique(ps.idx)
        books, flags = self._books(), self._flags()
        before = [b[u].copy() for b in books]
        was = [f[u].copy() for f in flags]
        for f in flags:                                   # cleared for a moment, to see which ones this pass sets
            f[u] = False
        super().land(ps)
        own = [f[u].copy() for f in flags]
        for f, w in zip(flags, was, strict=True):
            f[u] |= w
        fin = self.mstate[ps.machine]["finished"]
        self.recent.append((self.t if fin is None else fin, u, [b[u] - b0 for b, b0 in zip(books, before, strict=True)],
                            own))
        # Bounded memory: an hour of passes, and never more than 256.
        self.recent = [r for r in self.recent if r[0] > self.t - 60.0][-256:]

    # -- the Context the program's conditions see --------------------------------------------
    def probe(self) -> None:
        """Take in the newest survey, if one is waiting.

        A scan shows the ground as it was when the scan was taken. Passes that finished after that moment are
        already in the plan but not in the scan, so they are lifted out of the plan while the survey is compared
        with it, and put back afterwards. Otherwise each would be counted twice, or read as an error in the soil
        model.
        """
        msg = self.terrain
        if msg is None:
            return
        if tuple(msg["shape"]) != (self.ny, self.nx) or not msg.get("data"):
            self.terrain = None
            self.log(f"terrain from {msg.get('probe')} ignored: shape {msg['shape']} is not the site grid")
            return
        scan_t = self._minutes(msg["timestamp"])
        if self.clock.speed == 1.0 and scan_t > self.t + MAX_CLOCK_SKEW_S / 60.0:
            # A scan cannot come from the future: the probe's clock is ahead. Using it would hide the passes
            # that finish between now and its timestamp.
            self.terrain = None
            self.log(f"terrain from {msg.get('probe')} ignored: stamped {(scan_t - self.t) * 60:.0f} s in the "
                     "future; synchronise the probe's clock")
            return
        # Wait until every machine with a pass in flight has reported a state at or after the scan,
        # so we know which passes the scan saw. A silent machine is not waited for long.
        waiting = [n for n in self.inflight if self.mstate[n]["ts"] < scan_t]
        if waiting and time.time() - self.terrain_since < 2.0:
            return
        self.terrain = None
        late = [r for r in self.recent if r[0] > scan_t]          # in the plan, not yet in the scan
        books, flags = self._books(), self._flags()
        for _, u, deltas, _ in late:
            for b, d in zip(books, deltas, strict=True):
                b[u] -= d
            for f in flags:                               # their cells are left out of this comparison
                f[u] = False
        self.ingest_survey(wire.decode_heights(msg["data"], (self.ny, self.nx)).astype(float), msg["probe"])
        # The survey closed the books on everything it saw. The late passes are still to be seen: put back what
        # each predicted, the expected fill and rolling included, or the next survey would find their effect on
        # the ground with nothing expected there and blame the soil factors.
        for _, u, deltas, own in late:
            for b, d in zip(books[:2], deltas[:2], strict=True):
                b[u] += d
            for b, d in zip(books[2:], deltas[2:], strict=True):
                b[u] = np.maximum(b[u] + d, 0.0)          # the survey zeroed these two; never leave one negative
            for f, v in zip(flags, own, strict=True):
                f[u] |= v

    def proximity(self, a: str, b: str) -> float:
        """The smallest distance between any detected person and any machine, in metres (infinite when nobody is
        detected). The arguments are the two names in the program's `proximity(a, b)`.

        A detection with a radius means "somewhere within this circle", so the distance counts from the edge of
        the circle nearest the machine: the cautious reading.
        """
        d = [max(0.0, math.hypot(px - s["x"], py - s["y"]) - r) for px, py, r in self.people.values()
             for s in self.mstate.values() if s["x"] is not None]
        return min(d) * self.u if d else math.inf

    def weather(self, kind: str) -> float:
        """The rain gauge's reading, in metres per second; other kinds of weather are not measured and read 0."""
        return self.rain if kind == "rain" else 0.0

    def apply_rules(self) -> dict[str, Any]:
        """The program's rules, plus the site e-stop, which stops everything whatever the program says."""
        eff = super().apply_rules()
        if self.estop:
            eff["stop"] = True
        return eff

    def machine_ready(self, name: str) -> bool:
        """May this machine be given an order? It must be online, heard from within 3 s, reporting "ok", not held by
        an e-stop, past any retry delay, and its clock must agree with the controller's.
        """
        s = self.mstate[name]
        fresh = time.time() - s["seen"] < STATE_TIMEOUT_S
        return s["conn"] == "online" and fresh and s["safety"] == "ok" and not self.estop \
            and self.t >= s.get("retry_at", 0.0) and abs(s.get("skew", 0.0)) <= MAX_CLOCK_SKEW_S

    # -- reporting ---------------------------------------------------------------------------
    def status(self) -> dict:
        """The site status message: tasks, machines, active rules, survey age and the measured score. The dashboard
        and `loam status` show this.
        """
        return {
            "t": round(self.t, 3),
            "tasks": {n: {"started": _r3(self.started[n]), "done": _r3(self.done_at[n]), "passes": self.task_passes[n]}
                      for n in self.m.tasks},
            "machines": {n: {"conn": s["conn"], "safety": s["safety"], "ready": self.machine_ready(n),
                             "clock_skew_s": round(s.get("skew", 0.0), 2),
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
        """Per finish zone, the percentage of cells within tolerance according to the latest survey."""
        out = {}
        for n, z in self.m.zones.items():
            if z.get("role") == "finish" and self.grid.masks.get(n) is not None and self.has_survey:
                mk = self.grid.masks[n]
                tol = float(z["tolerance"].to(self.unit)) if z.get("tolerance") else 0.1
                out[n] = round(100 * float((np.abs(self.meas - self.design)[mk] <= tol).mean()), 2)
        return out

    def report(self) -> dict[str, Any]:
        """The final report, as the simulator's but without the things only a simulated world can know."""
        r = super().report()
        r.pop("true_ground", None)           # the controller cannot know the truth; the world reports it
        r.pop("ledger_error", None)
        r["abandoned_passes"] = self.abandoned
        return r


def run_controller(model: Model, base_dir: str, broker: str, record: str | None = None,
                   max_minutes: float = 48 * 60, wait_for_clock: bool = True) -> dict:
    """Run a site from its Loam program until every task is done or `max_minutes` pass, and return the report.

    Each loop: take in the messages that have arrived, do one planning step, send the 1 Hz safety heartbeat that
    machines must keep hearing, and publish the site status. `wait_for_clock=False` is for a real site, where site
    time is simply the time of day.
    """
    site = model.site["name"]
    bus = Bus(broker, f"loam-controller-{site}").start([f"loam/v0/{site}/#", f"sim/{site}/clock"])
    clock = Clock()
    if wait_for_clock:                               # a simulated site publishes its clock; a real one does not
        from .bus import wait_clock
        clock = wait_clock(bus, site, timeout=5.0)
    logs: list[str] = []

    def log(s: str) -> None:
        """Print a log line at once, and keep it for the report."""
        print(s, flush=True)
        logs.append(s)
    ctl = Controller(model, base_dir, bus, clock, record, log, max_minutes)
    last_hb = last_status = 0.0
    hb_seq = 0
    try:
        while True:
            msgs = bus.drain(0.002)
            ctl.t = clock.minutes()                  # messages are handled, and logged, at the current time
            for topic, msg in msgs:
                with tolerant(topic):
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
            # Sleep for what is left of one planning step (`dt` site minutes of wall time).
            time.sleep(max(0.002, clock.wall(ctl.dt) - (time.time() - now)))
        rep = ctl.report()
        rep["log"] = logs
        return rep
    finally:
        if ctl.record is not None:
            ctl.record.close()
        time.sleep(0.2)
        bus.stop()
