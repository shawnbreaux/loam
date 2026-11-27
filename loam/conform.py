"""`loam conform`: does a machine keep its side of Loam Wire?

Loam drives any machine that speaks Loam Wire, whoever built it and however: natively, through one of Loam's
bridges (MAVLink, VDA 5050, ROS 2), or through a vendor's own adapter. This is the test a machine has to pass
first. It plays the controller for that one machine on the site's broker and checks the promises the rest of
Loam relies on:

    announces      it says it is online, retained, so a controller that starts later knows
    reports        it sends its state at least once a second, complete and well formed
    ready          with a controller's heartbeat present it reports itself safe to work
    works          it takes an order, shows it is working it, and reports it done once, at progress 1.0
    no repeat      it does not run the same order a second time
    stops          it stops within 2 s of a stop, stays stopped, and goes on after a resume
    heartbeat      it stops by itself within 5 s when the controller falls silent
    e-stop         it stops on the site e-stop and does not restart when the e-stop is released

**The machine really moves.** It is sent short passes across the window given. Run this with the area clear, with
no other controller on the broker, and with the machine's own protective devices working. An order still on the
machine at the end is left stopped.

This checks behaviour on the wire. It is not a safety assessment of the machine.
"""
from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Callable
from itertools import pairwise
from typing import Any

from . import wire
from .checker import Model
from .runtime.bus import Bus

STATE_GAP_S = 1.5            # the longest a machine may go without sending its state
STOP_S = 2.0                 # how soon after a stop or an e-stop it must report itself stopped
HEARTBEAT_S = 5.0            # how soon after the controller falls silent (machines allow 3 s, plus reporting)
HOLD_S = 1.5                 # how long it is watched to see that it stays stopped
STATE_KEYS: dict[str, type | tuple[type, ...]] = {
    "site": str, "machine": str, "pose": dict, "driving": bool, "progress": (int, float), "safetyState": str,
    "errors": list}
# The work an order can ask for, by the capability it needs, in the order tried.
OPS = (("finish", "finish"), ("cut", "cut"), ("spread", "spread"), ("compact", "compact"), ("haul", "haul"))


def _schema():
    """A validator for Wire messages from the schema shipped with the source, if both it and `jsonschema` are
    here; otherwise None, and only the basic shape is checked."""
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "schemas", "wire",
                        "loam-wire-0.0.1.schema.json")
    try:
        import jsonschema
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (ImportError, OSError, ValueError):
        return None
    return jsonschema.Draft202012Validator({"$ref": "#/$defs/state", "$defs": doc.get("$defs", {})}) \
        if "state" in doc.get("$defs", {}) else None


def malformed(state: Any, validator=None) -> str | None:
    """What is wrong with a state message, or None if it is well formed."""
    if not isinstance(state, dict):
        return "not a JSON object"
    for key, kind in STATE_KEYS.items():
        if not isinstance(state.get(key), kind):
            return f"'{key}' is missing or of the wrong type"
    pose = state["pose"]
    if not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k]) for k in ("x", "y", "heading")):
        return "'pose' needs finite x, y and heading"
    if state["safetyState"] not in ("ok", "degraded", "stopped") or not 0.0 <= state["progress"] <= 1.0:
        return "'safetyState' or 'progress' is out of range"
    if validator is not None:
        err = next(iter(validator.iter_errors(state)), None)
        if err is not None:
            return f"schema: {err.message[:120]}"
    return None


class _StopError(Exception):
    """Raised inside `conform` when a check fails that the later ones depend on."""


class _Site:
    """The controller's side of the wire for one machine: its states as they arrive, and the messages to send."""

    def __init__(self, model: Model, machine: str, broker: str):
        """Connect and listen to the machine's state and connection topics."""
        self.site, self.machine = model.site["name"], machine
        self.bus = Bus(broker, f"loam-conform-{os.getpid()}").start(
            [wire.topic(self.site, machine, "state"), wire.topic(self.site, machine, "connection")])
        self.seq = int(time.time())
        self.states: list[tuple[float, dict]] = []       # (when it arrived, the message)
        self.connection: dict | None = None
        self.beating, self.beat_at = True, 0.0
        self.validator = _schema()
        self.bad: str | None = None                      # the first malformed state seen

    def send(self, kind: str, msg: dict, site_wide: bool = False, retain: bool = False) -> None:
        """Publish one message to the machine (or to the whole site)."""
        self.bus.pub(wire.topic(self.site, None if site_wide else self.machine, kind), msg, qos=1, retain=retain)

    def n(self) -> int:
        """The next sequence number."""
        self.seq += 1
        return self.seq

    def pump(self, seconds: float, until: Callable[[dict], bool] | None = None) -> dict | None:
        """Take in messages for up to `seconds`, sending the heartbeat meanwhile. Returns the first state that
        satisfies `until`, or None when the time is up."""
        end = time.time() + seconds
        while True:
            if self.beating and time.time() - self.beat_at >= 0.5:
                self.beat_at = time.time()
                self.send("safety", wire.safety(self.site, self.n(), ts=time.time()), site_wide=True)
            for topic, msg in self.bus.drain(0.05):
                if topic.endswith("/connection"):
                    self.connection = msg if isinstance(msg, dict) else None
                    continue
                problem = malformed(msg, self.validator)
                if problem is not None:
                    self.bad = self.bad or problem
                    continue
                self.states.append((time.time(), msg))
                if until is not None and until(msg):
                    return msg
            if time.time() >= end:
                return None

    def order(self, order_id: str, op: str, window: list) -> dict:
        """Send a one-pass order over the window and return it."""
        msg = wire.order(self.site, self.machine, self.n(), order_id, 0, "conform", op, window,
                         {"offset": 0.0}, {}, ts=time.time())
        self.send("order", msg)
        return msg

    def action(self, action: str) -> None:
        """Send the machine an instant action."""
        self.send("instantAction", wire.instant_action(self.site, self.machine, self.n(), action, "loam conform",
                                                       ts=time.time()))


def conform(model: Model, machine: str, broker: str, window: tuple[float, float, float, float],
            timeout: float = 120.0, log: Callable[[str], None] = print) -> dict:
    """Run the checks against `machine` and return {"machine", "passed", "failed", "not_tested", "checks"}.

    `window` is (x0, y0, x1, y1) in site coordinates: a clear strip the machine may drive along. `timeout` is how
    long a pass may take. Each check is {"check", "ok", "detail"}; `ok` is None for a check that could not be
    made (the machine finished its pass before it could be stopped: give it a longer window).
    """
    if machine not in model.machines:
        raise ValueError(f"no machine named {machine!r}")
    caps = model.machines[machine].get("caps") or []
    op = next((o for cap, o in OPS if cap in caps), "haul")
    x0, y0, x1, y1 = window
    win = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    s = _Site(model, machine, broker)
    checks: list[dict] = []
    stamp = f"conform-{int(time.time())}"

    def note(name: str, ok: bool | None, detail: str) -> bool:
        """Record one check's result."""
        checks.append({"check": name, "ok": ok, "detail": detail})
        log(f"  {'PASS' if ok else 'not tested' if ok is None else 'FAIL':<10} {name}: {detail}")
        return bool(ok)

    def working(order_id: str) -> Callable[[dict], bool]:
        """A test for "this state shows the machine driving on that order"."""
        return lambda st: st.get("orderId") == order_id and st["driving"] and st["progress"] < 1.0

    def stopped(st: dict) -> bool:
        """A state that shows the machine stopped."""
        return st["safetyState"] == "stopped" and not st["driving"]

    def held(order_id: str) -> tuple[bool, float]:
        """Watch for `HOLD_S`: whether the machine stayed stopped, and how far it moved meanwhile (site units)."""
        k = len(s.states)
        s.pump(HOLD_S)
        seen = [st for _, st in s.states[k:]]
        if not seen:
            return False, 0.0
        far = max(math.hypot(st["pose"]["x"] - seen[0]["pose"]["x"], st["pose"]["y"] - seen[0]["pose"]["y"])
                  for st in seen)
        return all(stopped(st) or st.get("orderId") != order_id for st in seen), far

    try:
        # announces, reports
        s.pump(10.0, until=lambda st: s.connection is not None)
        online = (s.connection or {}).get("connectionState") == "online"
        note("announces", online, "online, retained" if online else f"no 'online' connection message: {s.connection}")
        k, t0 = len(s.states), time.time()
        s.pump(4.0)
        times = [t for t, _ in s.states[k:]]
        gaps = [b - a for a, b in zip([t0, *times], [*times, time.time()], strict=True)]
        ok = bool(times) and max(gaps) <= STATE_GAP_S and s.bad is None
        note("reports", ok, s.bad or (f"{len(times)} states in 4 s, longest gap {max(gaps):.2f} s" if times
                                      else "no state message in 4 s"))
        if not times:
            raise _StopError
        # ready
        s.action("resume")
        s.send("safety", wire.safety(s.site, s.n(), "resume", ts=time.time()), site_wide=True)
        ready = s.pump(15.0, until=lambda st: st["safetyState"] == "ok")
        if not note("ready", ready is not None, "safetyState ok" if ready else
                    f"never ok: {[e.get('code') for e in s.states[-1][1]['errors']]}"):
            raise _StopError
        # works
        first = f"{stamp}-1"
        s.order(first, op, win)
        k = len(s.states)
        began = s.pump(15.0, until=working(first))
        done = s.pump(timeout, until=lambda st: st.get("orderId") == first and st["progress"] >= 1.0)
        mine = [st["progress"] for _, st in s.states[k:] if st.get("orderId") == first]
        rising = all(b >= a - 1e-9 for a, b in pairwise(mine))
        note("works", bool(began and done and rising),
             "taken, worked and reported done at 1.0" if began and done and rising else
             "progress went backwards" if began and done else
             "never showed itself driving on the order" if not began else f"not done within {timeout:g} s")
        if not done:
            raise _StopError
        # no repeat
        s.pump(1.0)
        s.order(first, op, win)
        again = s.pump(3.0, until=working(first))
        note("no repeat", again is None, "ignored an order it had already done" if again is None else
             "ran the same orderId a second time")
        # stops
        second = f"{stamp}-2"
        s.order(second, op, win[::-1])
        if s.pump(15.0, until=working(second)) is None:
            note("stops", None, "the machine did not start the pass")
        else:
            sent = time.time()
            s.action("stop")
            hit = s.pump(STOP_S + 1.0, until=lambda st: stopped(st) or st["progress"] >= 1.0)
            if hit is None or not stopped(hit):
                note("stops", None if hit else False, "the pass finished before the stop: use a longer window"
                     if hit else f"still driving {STOP_S + 1:g} s after a stop")
            else:
                took = time.time() - sent
                stayed, far = held(second)
                s.action("resume")
                went = s.pump(10.0, until=lambda st: st.get("orderId") == second and
                              (st["driving"] or st["progress"] >= 1.0))
                note("stops", took <= STOP_S and stayed and went is not None,
                     f"stopped in {took:.2f} s, moved {far:.2f} while stopped, went on after resume"
                     if took <= STOP_S and stayed and went is not None else
                     f"took {took:.2f} s to report stopped" if took > STOP_S else
                     "did not stay stopped" if not stayed else "did not go on after resume")
            s.pump(timeout, until=lambda st: st.get("orderId") == second and st["progress"] >= 1.0)
        # heartbeat
        third = f"{stamp}-3"
        s.order(third, op, win)
        if s.pump(15.0, until=working(third)) is None:
            note("heartbeat", None, "the machine did not start the pass")
        else:
            s.beating, silent = False, time.time()
            hit = s.pump(HEARTBEAT_S + 1.0, until=lambda st: stopped(st) or st["progress"] >= 1.0)
            took = time.time() - silent
            s.beating = True
            if hit is not None and not stopped(hit):
                note("heartbeat", None, "the pass finished before the silence could tell: use a longer window")
            else:
                note("heartbeat", hit is not None and took <= HEARTBEAT_S,
                     f"stopped {took:.2f} s after the controller fell silent" if hit is not None else
                     f"still driving {HEARTBEAT_S + 1:g} s after the controller fell silent")
            s.action("resume")
            s.send("safety", wire.safety(s.site, s.n(), "resume", ts=time.time()), site_wide=True)
            s.pump(timeout, until=lambda st: st.get("orderId") == third and st["progress"] >= 1.0)
        # e-stop (last: some vehicles need a person to re-arm them afterwards)
        fourth = f"{stamp}-4"
        s.order(fourth, op, win[::-1])
        if s.pump(15.0, until=working(fourth)) is None:
            note("e-stop", None, "the machine did not start the pass")
        else:
            pressed = time.time()
            s.send("estop", wire.estop(s.site, s.n(), True, "loam conform", ts=time.time()), site_wide=True)
            hit = s.pump(STOP_S + 1.0, until=lambda st: stopped(st) or st["progress"] >= 1.0)
            took = time.time() - pressed
            s.send("estop", wire.estop(s.site, s.n(), False, "loam conform", ts=time.time()), site_wide=True)
            if hit is not None and not stopped(hit):
                note("e-stop", None, "the pass finished before the e-stop: use a longer window")
            else:
                stayed, _ = held(fourth)
                note("e-stop", hit is not None and took <= STOP_S and stayed,
                     f"stopped in {took:.2f} s and stayed stopped when the e-stop was released"
                     if hit is not None and took <= STOP_S and stayed else
                     "restarted by itself when the e-stop was released" if hit is not None and took <= STOP_S else
                     f"not stopped {STOP_S:g} s after the e-stop")
    except _StopError:
        pass
    finally:
        try:
            # Leave nothing running: the e-stop released, and the machine stopped on whatever it still holds.
            s.send("estop", wire.estop(s.site, s.n(), False, "loam conform", ts=time.time()), site_wide=True)
            s.action("stop")
            time.sleep(0.2)
        finally:
            s.bus.stop()
    return {"machine": machine, "passed": sum(c["ok"] is True for c in checks),
            "failed": sum(c["ok"] is False for c in checks),
            "not_tested": sum(c["ok"] is None for c in checks), "checks": checks}
