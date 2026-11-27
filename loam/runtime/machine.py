"""A simulated machine: one process per machine, speaking only Loam Wire to the controller.

It behaves like a machine with a Loam Wire adapter and grade control:

* takes one order at a time, drives the work window (and the haul to the
  deposit or from the source), reports state four times a second (wall);
* `stop` / site e-stop freeze it at once; `hold` lets the current pass finish;
* it stops on its own when it hears no safety heartbeat for 3 s (spec), or when
  its GNSS solution is not RTK fixed (grade control cannot work), and says so
  in `safetyState`;
* its connection status is retained, with `connectionBroken` as the last will.

When a pass completes it asks the world to move the earth (`sim/...`, the
physics stand-in) and only then reports progress 1.0.
"""
from __future__ import annotations

import math
import os
import time
from collections import deque

import numpy as np

from .. import wire
from ..checker import Model
from ..units import Quantity
from .bus import Bus, Clock, tolerant, wait_clock

# A machine stops by itself after this long without the controller's safety heartbeat (the Loam Wire rule), and
# reports its state this often.
HEARTBEAT_TIMEOUT_S = 3.0
STATE_EVERY_S = 0.25


def _centre(poly: list) -> tuple[float, float]:
    """The mean of a polygon's corners: where a machine heads for when told to go to an area."""
    p = np.asarray(poly, float)
    return float(p[:, 0].mean()), float(p[:, 1].mean())


class MachineSim:
    """One machine's side of Loam Wire: its order, its safety state and its position.

    `run_machine` drives it as a simulated machine. The vehicle bridges reuse it for everything that is the same on
    a real machine: reading orders, stop / hold / e-stop handling, the heartbeat watchdog and the state report.
    """
    def __init__(self, model: Model, name: str, broker: str, park: tuple[float, float]):
        """`park` is where the machine starts, in site coordinates. Speeds are kept in site units per site minute."""
        self.m, self.name = model, name
        self.site = model.site["name"]
        self.broker = broker
        md = model.machines[name]
        self.u = Quantity(1, model.site.get("units", "ft")).si
        self.speed = (md.get("max_speed") or Quantity(3, "mph")).si * 60 / self.u   # site units / min
        self.x, self.y, self.heading = park[0], park[1], 0.0
        self.stock_xy = {n: _centre(sp["area"]["polygon"]) for n, sp in model.stockpiles.items()
                         if "polygon" in sp.get("area", {})}
        self.order: dict | None = None
        self.path: list[tuple[float, float]] = []
        self.legs: list[float] = []
        self.duration = 0.0
        self.progress = 0.0
        self.waiting = False
        self.stopped = self.held = self.estop = False
        self.latched = False                 # e-stop released but not yet resumed (ISO 13850: no auto-restart)
        self.last_hb = time.time()
        self.gnss_bad = False
        self.seq = 0
        self.stop_latency_s: list[float] = []
        # The ids of the orders this machine has taken. An order is never run twice: a message delivered again
        # (MQTT delivers "at least once") or a controller repeating itself must not send the machine over the
        # same ground a second time.
        self.taken: deque[str] = deque(maxlen=512)
        self.max_tilt: float | None = None
        self.rolled = False
        self.stuck = 0

    # -- connection ----------------------------------------------------------
    def topic(self, kind: str) -> str:
        """This machine's topic for one kind of message."""
        return wire.topic(self.site, self.name, kind)

    def connect(self, also: tuple[str, ...] = ()) -> Bus:
        """Connect to the broker, subscribe to everything a machine listens to (and the topics in `also`), and
        announce the machine online. Its
        last will says `connectionBroken`, so the controller learns of a dropped link from the broker. `online` is
        announced again after every reconnect: otherwise a machine whose radio dropped for a few seconds would
        stay written off for the rest of the job.
        """
        will = (self.topic("connection"), wire.connection(self.site, self.name, 0, "connectionBroken"))

        def online() -> tuple[str, dict]:
            """The machine's `online` message, numbered and stamped when it is sent."""
            self.seq += 1
            return self.topic("connection"), wire.connection(self.site, self.name, self.seq, "online")

        bus = Bus(self.broker, f"loam-machine-{self.site}-{self.name}-{os.getpid()}-{time.time():.3f}", will,
                  birth=online)
        bus.start([self.topic("order"), self.topic("instantAction"), wire.topic(self.site, None, "safety"),
                   wire.topic(self.site, None, "estop"), f"sim/{self.site}/clock", f"sim/{self.site}/faults",
                   f"sim/{self.site}/world/applied/{self.name}", f"sim/{self.site}/end", *also])
        self.last_hb = time.time()
        return bus

    # -- orders ----------------------------------------------------------------
    def is_new(self, order: dict) -> bool:
        """Whether this order is one to take: not the one being worked, and not one taken before."""
        return order.get("orderId") not in self.taken

    def take(self, order: dict) -> None:
        """Accept an order and work out the path to drive, as a list of site points in `self.path`.

        The work pass runs down the middle of the order's window along its longer side (back and forth for a
        roller). A cut then hauls to where the spoil goes and returns; a fill runs from the source stockpile to the
        placement and back.
        """
        self.taken.append(order["orderId"])
        win, dep = order.get("window"), order.get("deposit") or {}
        op = order["op"]
        pts: list[tuple[float, float]] = []
        if win:
            p = np.asarray(win, float)
            (x0, y0), (x1, y1) = p.min(0), p.max(0)
            if x1 - x0 >= y1 - y0:
                a, b = (x0, (y0 + y1) / 2), (x1, (y0 + y1) / 2)
            else:
                a, b = ((x0 + x1) / 2, y0), ((x0 + x1) / 2, y1)
            pts = [a, b]
            if op == "compact":
                pts = [a, b] * max(1, int((order.get("limits") or {}).get("passes", 4)) // 2)
        if op == "cut":
            dest = (_centre(dep["window"]) if dep.get("window") else None) or self.stock_xy.get(
                dep.get("stockpile") or dep.get("overflow") or "")
            if dest:
                pts += [dest, pts[-1] if pts else dest]
        elif op in ("fill", "spread"):
            src = self.stock_xy.get(dep.get("source") or "")
            dest = _centre(dep["window"]) if dep.get("window") else None
            pts = [p for p in (src, dest, src) if p is not None]
        if not pts:
            pts = [(self.x, self.y)]
        self.path = [(self.x, self.y), *pts]
        self.legs = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:], strict=False)]
        speed = min(self.speed, (order.get("limits") or {}).get("maxSpeed", 1e9) * 60 / self.u)
        self.duration = sum(self.legs) / max(speed, 1e-9) + 0.3
        self.order, self.progress, self.waiting = order, 0.0, False

    def pose_at(self, f: float) -> None:
        """Move the machine to fraction `f` (0 to 1) of the way along its path, facing the way it is travelling."""
        total = sum(self.legs)
        if total <= 0:
            return
        d = min(f, 1.0) * total
        for (a, b), leg in zip(zip(self.path, self.path[1:], strict=False), self.legs, strict=True):
            if d <= leg or leg == self.legs[-1]:
                k = d / leg if leg > 0 else 1.0
                nx, ny = a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k
                if math.hypot(nx - self.x, ny - self.y) > 1e-6:
                    self.heading = math.atan2(ny - self.y, nx - self.x)
                self.x, self.y = nx, ny
                return
            d -= leg

    def control(self, kind: str, msg: dict, on_estop=None) -> None:
        """Apply an instantAction, safety or e-stop message. `on_estop()` runs as soon as an e-stop is
        pressed (the vehicle's hardest stop), before the stop latency is measured.

        Releasing the e-stop does not restart the machine: it stays stopped (estopReset) until a
        deliberate `resume`, site-wide on the safety topic or for this machine as an instant action."""
        if kind == "instantAction":
            a = msg["action"]
            if a == "resume":
                self.stopped = self.held = self.latched = False
            elif a == "stop":
                self.stopped = True
            elif a == "hold":
                self.held = True
        elif kind == "safety":
            self.last_hb = time.time()
            if msg["command"] == "stop":
                self.stopped = True
            elif msg["command"] == "resume":
                self.stopped = self.latched = False
        elif kind == "estop":
            pressed, was = bool(msg["pressed"]), self.estop
            self.estop = pressed
            if was and not pressed:
                self.latched = True
            if pressed and not was:
                if on_estop is not None:
                    on_estop()
                if "wall" in msg:                         # simulated sites stamp the press for latency
                    self.stop_latency_s.append(time.time() - float(msg["wall"]))

    def safety_state(self) -> tuple[str, list]:
        """The machine's safety state and the reasons for it: ("ok" | "degraded" | "stopped", errors).

        Stopped beats degraded. Reasons, in order: it has tipped over; an e-stop is pressed; an e-stop was released
        but nobody has resumed; a stop was ordered; the controller's heartbeat has not been heard for 3 s. Degraded
        means the GNSS position is not good enough to grade by.
        """
        if self.rolled:
            return "stopped", [wire.error("rollover", f"tilt {math.degrees(self.max_tilt or 0):.0f} deg")]
        if self.estop or self.latched or self.stopped or time.time() - self.last_hb > HEARTBEAT_TIMEOUT_S:
            reason = "estop" if self.estop else "estopReset" if self.latched else \
                "stop" if self.stopped else "heartbeatLost"
            return "stopped", [wire.error(reason)]
        if self.gnss_bad:
            return "degraded", [wire.error("gnssQuality", "rtk-float", "warning")]
        return "ok", []


def run_machine(model: Model, name: str, broker: str, park: tuple[float, float],
                physics_ground: tuple | None = None) -> None:
    """physics_ground = (z, x0, y0, cell) drives the machine as a PyBullet body over that ground."""
    ms = MachineSim(model, name, broker, park)
    body = None
    travelled = 0.0
    stall_xy, stall_t = (park[0], park[1]), 0.0
    if physics_ground is not None:
        from .physics import Body
        gz, gx0, gy0, gcell = physics_ground
        body = Body(gz, gx0, gy0, gcell, ms.u, model.machines[name]["class"], park)
        ms.x, ms.y, ms.heading = body.pose()[:3]
    site = ms.site
    bus = ms.connect()
    clock: Clock = wait_clock(bus, site)
    while clock.minutes() < 0:
        time.sleep(0.01)
    faults: dict = {}
    fired: set[int] = set()                       # link-loss entries already played (survives reconnects)
    last_state, last_t = 0.0, clock.minutes()
    lost_until: float | None = None
    while True:
        t = clock.minutes()
        # -- scripted faults -------------------------------------------------------------
        ms.gnss_bad = any(f["machine"] == name and f["from"] <= t < f["to"] for f in faults.get("gnss_float", []))
        for f in faults.get("crash", []):
            if f["machine"] == name and t >= f["at"]:
                bus.sever()
                # A scripted crash: die at once, with no goodbye, as a machine computer losing power would. The broker
                # then publishes its last will.
                os._exit(3)
        for k, f in enumerate(faults.get("link_loss", [])):
            if f["machine"] == name and f["at"] <= t and k not in fired:
                fired.add(k)
                bus.sever()                       # radio drops: no messages either way
                lost_until = time.time() + f.get("wall_s", 4.0)
        if lost_until is not None:
            if time.time() < lost_until:
                time.sleep(0.05)
                last_t = clock.minutes()           # a machine without its radio does not work either:
                continue                           # the heartbeat watchdog has stopped it
            lost_until = None
            bus = ms.connect()
            ms.order, ms.waiting = None, False    # never resume a pass without the controller re-issuing it
        # -- messages ------------------------------------------------------------------
        for topic, msg in bus.drain(0.02):
            with tolerant(topic):
                kind = topic.rsplit("/", 1)[-1]
                if topic == f"sim/{site}/end":
                    bus.pub(ms.topic("connection"), wire.connection(site, name, ms.seq + 1, "offline"),
                            retain=True, qos=1)
                    time.sleep(0.1)
                    bus.stop()
                    return
                if topic == f"sim/{site}/faults":
                    faults = msg
                elif kind == "order" and ms.is_new(msg):
                    ms.take(msg)
                    travelled, stall_xy, stall_t = 0.0, (ms.x, ms.y), clock.minutes()
                elif kind in ("instantAction", "safety", "estop"):
                    ms.control(kind, msg)
                elif topic.startswith(f"sim/{site}/world/applied/") and ms.waiting and ms.order \
                        and msg["orderId"] == ms.order["orderId"]:
                    ms.progress, ms.waiting = 1.0, False
                    ms.seq += 1
                    bus.pub(ms.topic("state"), _state(ms, clock, msg.get("t", t)), qos=1)
                    ms.order = None
        # -- motion --------------------------------------------------------------------
        safety, _ = ms.safety_state()
        moving = ms.order is not None and not ms.waiting and safety == "ok"
        if moving and ms.order is not None and body is not None:
            from .physics import follow
            total = sum(ms.legs)
            speed = min(ms.speed, (ms.order.get("limits") or {}).get("maxSpeed", 1e9) * 60 / ms.u) * ms.u / 60
            body.drive(follow(body, ms.path, travelled, 0.0), speed, max(0.0, (t - last_t) * 60.0), step=1 / 30)
            x, y, h, _, _ = body.pose()
            ms.max_tilt, ms.rolled = body.max_tilt, body.rolled
            travelled += math.hypot(x - ms.x, y - ms.y)
            ms.x, ms.y, ms.heading = x, y, h
            end = ms.path[-1]
            # A physical body never lands exactly on the last point, so the pass also counts as driven when the
            # machine is within 1.5 m of the end and most of the way along.
            near_end = math.hypot(end[0] - x, end[1] - y) * ms.u < 1.5
            ms.progress = min(1.0, travelled / total) if total > 0 else 1.0
            wedged = False
            if math.hypot(x - stall_xy[0], y - stall_xy[1]) * ms.u > 0.5:
                stall_xy, stall_t = (x, y), t
            elif t - stall_t > 10.0:                  # wedged for 10 site minutes: give up on the path
                ms.stuck += 1
                wedged = True
            if ms.progress >= 0.98 or (near_end and ms.progress > 0.6) or total == 0 or wedged:
                ms.progress = 1.0
        elif moving and ms.order is not None:
            ms.progress = min(1.0, ms.progress + (t - last_t) / max(ms.duration, 1e-9))
            ms.pose_at(ms.progress)
        if moving and ms.order is not None and ms.progress >= 1.0:
            ms.progress = 0.999                # 1.0 is reported only once the work is really done
            ms.waiting = True
            bus.pub(f"sim/{site}/world/apply", {"machine": name, "order": ms.order}, qos=1)
        last_t = t
        if time.time() - last_state >= STATE_EVERY_S:
            last_state = time.time()
            ms.seq += 1
            st = _state(ms, clock, t)
            st["driving"] = moving
            bus.pub(ms.topic("state"), st)
        time.sleep(0.02)


def _state(ms: MachineSim, clock: Clock, t: float) -> dict:
    """A simulated machine's state message, with its worst e-stop reaction time and, under physics, its largest tilt."""
    safety, errors = ms.safety_state()
    st = wire.state(ms.site, ms.name, ms.seq, ms.order["orderId"] if ms.order else None, round(ms.x, 3),
                    round(ms.y, 3), round(ms.heading, 4), False, round(ms.progress, 4), safety, errors, ts=clock.ts(t))
    if ms.stop_latency_s:
        st["stopLatencyS"] = round(max(ms.stop_latency_s), 4)
    if ms.max_tilt is not None:
        st["maxTiltDeg"] = round(math.degrees(ms.max_tilt), 2)
        st["stuckEvents"] = ms.stuck
    return st
