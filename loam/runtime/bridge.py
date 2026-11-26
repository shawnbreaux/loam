"""The part every vehicle bridge shares: Loam Wire in, a vehicle protocol out.

A bridge makes a real (or protocol-simulated) vehicle a Loam machine. This
loop owns the Wire side and the safety rules, so they are the same for every
protocol and the same as the simulated machines:

* an order becomes the same path the simulated machine drives (`MachineSim.take`);
* `stop`, site stop, no safety heartbeat for 3 s, or a bad GNSS fix pause the
  vehicle; `resume` lets it go on; e-stop calls the adapter's hardest stop;
* a pass is reported done (1.0) only when the vehicle has finished it and, on a
  simulated site, the world has applied it;
* a vehicle that cannot work (link lost, its own e-stop, manual mode, a fatal
  error) reports a `fault` code; the bridge holds it and reports `stopped`.

An adapter supplies the protocol: `poll()`, `pose` (site x, y, heading),
`fix_ok`, `fix_name`, `fault`, `start(path, order, max_speed_ms)`,
`pause(on)`, `estop()`, `fraction()`, `done()`, `close()`.
"""
from __future__ import annotations

import time
from typing import Protocol

from .. import wire
from ..checker import Model
from .bus import Bus, Clock, wait_clock
from .machine import STATE_EVERY_S, MachineSim


class Adapter(Protocol):
    pose: tuple[float, float, float] | None

    @property
    def fix_ok(self) -> bool: ...
    @property
    def fix_name(self) -> str: ...
    @property
    def fault(self) -> str | None: ...

    def poll(self) -> None: ...
    def start(self, path: list[tuple[float, float]], order: dict, max_speed_ms: float | None) -> None: ...
    def pause(self, on: bool) -> None: ...
    def estop(self) -> None: ...
    def fraction(self) -> float: ...
    def done(self) -> bool: ...
    def close(self) -> None: ...


def run_bridge(model: Model, name: str, broker: str, adapter: Adapter, sim_world: bool = False,
               max_wall_s: float | None = None) -> MachineSim:
    """Run until the simulated site ends, `max_wall_s` passes, or the process is stopped."""
    if name not in model.machines:
        raise ValueError(f"no machine named {name!r}")
    ms = MachineSim(model, name, broker, (0.0, 0.0))
    bus: Bus = ms.connect()
    clock = wait_clock(bus, ms.site) if sim_world else Clock(time.time())
    started, last_state = time.time(), 0.0
    try:
        while max_wall_s is None or time.time() - started < max_wall_s:
            adapter.poll()
            if adapter.pose is not None:
                ms.x, ms.y, ms.heading = adapter.pose
            ms.gnss_bad = not adapter.fix_ok
            t = clock.minutes()
            for topic, msg in bus.drain(0.02):
                kind = topic.rsplit("/", 1)[-1]
                if topic == f"sim/{ms.site}/end":
                    return ms
                if kind == "order" and (ms.order is None or msg["orderId"] != ms.order["orderId"]):
                    ms.take(msg)
                    adapter.start(ms.path, msg, (msg.get("limits") or {}).get("maxSpeed"))
                elif kind == "instantAction":
                    a = msg["action"]
                    ms.stopped = a == "stop" or (ms.stopped and a != "resume")
                    ms.held = a == "hold" or (ms.held and a != "resume")
                elif kind == "safety":
                    ms.last_hb = time.time()
                    if msg["command"] in ("stop", "resume"):
                        ms.stopped = msg["command"] == "stop"
                elif kind == "estop":
                    if msg["pressed"] and not ms.estop:
                        adapter.estop()
                        ms.stop_latency_s.append(time.time() - msg["wall"])
                    ms.estop = bool(msg["pressed"])
                elif topic.startswith(f"sim/{ms.site}/world/applied/") and ms.waiting and ms.order \
                        and msg["orderId"] == ms.order["orderId"]:
                    _finish(ms, adapter, bus, clock, t)
            if ms.order is not None and not ms.waiting:
                go = ms.safety_state()[0] == "ok" and adapter.fault is None and adapter.pose is not None
                adapter.pause(not go)
                ms.progress = min(0.999, max(ms.progress, adapter.fraction()))
                if go and adapter.done():
                    adapter.pause(True)
                    if sim_world:
                        ms.waiting = True
                        bus.pub(f"sim/{ms.site}/world/apply", {"machine": name, "order": ms.order}, qos=1)
                    else:
                        _finish(ms, adapter, bus, clock, t)
            if time.time() - last_state >= STATE_EVERY_S:
                last_state = time.time()
                ms.seq += 1
                bus.pub(ms.topic("state"), bridge_state(ms, adapter, clock, t))
            time.sleep(0.02)
        return ms
    finally:
        adapter.pause(True)
        bus.pub(ms.topic("connection"), wire.connection(ms.site, name, ms.seq + 1, "offline"), retain=True, qos=1)
        time.sleep(0.1)
        bus.stop()
        adapter.close()


def _finish(ms: MachineSim, adapter: Adapter, bus: Bus, clock: Clock, t: float) -> None:
    ms.progress, ms.waiting = 1.0, False
    ms.seq += 1
    bus.pub(ms.topic("state"), bridge_state(ms, adapter, clock, t), qos=1)
    ms.order = None


def bridge_state(ms: MachineSim, adapter: Adapter, clock: Clock, t: float) -> dict:
    safety, errors = ms.safety_state()
    fault = adapter.fault or (None if adapter.pose is not None else "vehicleNoPose")
    if fault is not None:
        safety, errors = "stopped", [*errors, {"code": fault, "detail": ""}]
    errors = [e for e in errors if e["code"] != "gnssQuality"]
    if ms.gnss_bad:
        errors.append({"code": "gnssQuality", "detail": adapter.fix_name})
    st = wire.state(ms.site, ms.name, ms.seq, ms.order["orderId"] if ms.order else None, round(ms.x, 3),
                    round(ms.y, 3), round(ms.heading, 4), False, round(ms.progress, 4), safety, errors,
                    ts=clock.ts(t))
    st["driving"] = bool(ms.order is not None and not ms.waiting and safety == "ok")
    if ms.stop_latency_s:
        st["stopLatencyS"] = round(max(ms.stop_latency_s), 4)
    return st
