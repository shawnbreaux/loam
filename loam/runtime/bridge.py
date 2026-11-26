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
  error) reports a `fault` code; the bridge holds it and reports `stopped`;
* a vehicle that cannot carry out *this order* (it refused it, navigation
  aborted, the target is out of reach) reports an `order_error`. The bridge
  holds the vehicle, reports `orderFailed` once and drops the order, so the
  controller withdraws the pass and the machine is free for the next one.

An adapter supplies the protocol: `poll()`, `pose` (site x, y, heading),
`fix_ok`, `fix_name`, `fault`, `order_error`, `start(path, order, max_speed_ms)`,
`pause(on)`, `estop()`, `fraction()`, `done()`, `close()`.
"""
from __future__ import annotations

import sys
import time
from typing import Protocol

from .. import wire
from ..checker import Model
from .bus import Bus, Clock, tolerant, wait_clock
from .machine import STATE_EVERY_S, MachineSim


class Adapter(Protocol):
    pose: tuple[float, float, float] | None

    @property
    def fix_ok(self) -> bool: ...
    @property
    def fix_name(self) -> str: ...
    @property
    def fault(self) -> str | None: ...
    @property
    def order_error(self) -> str | None: ...

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
        adapter.close()
        raise ValueError(f"no machine named {name!r}")
    ms = MachineSim(model, name, broker, (0.0, 0.0))
    bus: Bus | None = None
    try:                                     # the vehicle is held and closed on every way out
        bus = ms.connect()
        clock = wait_clock(bus, ms.site) if sim_world else Clock(time.time())
        started, last_state = time.time(), 0.0
        while max_wall_s is None or time.time() - started < max_wall_s:
            adapter.poll()
            ms.gnss_bad = not adapter.fix_ok
            t = clock.minutes()
            for topic, msg in bus.drain(0.02):
                with tolerant(topic):
                    kind = topic.rsplit("/", 1)[-1]
                    if topic == f"sim/{ms.site}/end":
                        return ms
                    if kind == "order" and (ms.order is None or msg["orderId"] != ms.order["orderId"]):
                        ms.take(msg)
                        adapter.start(ms.path, msg, (msg.get("limits") or {}).get("maxSpeed"))
                    elif kind in ("instantAction", "safety", "estop"):
                        ms.control(kind, msg, adapter.estop)
                    elif topic.startswith(f"sim/{ms.site}/world/applied/") and ms.waiting and ms.order \
                            and msg["orderId"] == ms.order["orderId"]:
                        _finish(ms, adapter, bus, clock, t)
            failed = adapter.order_error if ms.order is not None and not ms.waiting else None
            if failed is not None and ms.order is not None:
                adapter.pause(True)
                ms.seq += 1
                st = bridge_state(ms, adapter, clock, t)
                st.update(safetyState="stopped", driving=False,
                          errors=[*st["errors"], wire.error("orderFailed", failed)])
                bus.pub(ms.topic("state"), st, qos=1)
                print(f"loam: {name}: order {ms.order['orderId']} failed: {failed}", file=sys.stderr, flush=True)
                ms.order = None
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
        try:
            adapter.pause(True)
        finally:                                 # even if the vehicle link is already dead
            try:
                if bus is not None:
                    bus.pub(ms.topic("connection"), wire.connection(ms.site, name, ms.seq + 1, "offline"),
                            retain=True, qos=1)
                    time.sleep(0.1)
                    bus.stop()
            finally:
                adapter.close()


def _finish(ms: MachineSim, adapter: Adapter, bus: Bus, clock: Clock, t: float) -> None:
    ms.progress, ms.waiting = 1.0, False
    ms.seq += 1
    bus.pub(ms.topic("state"), bridge_state(ms, adapter, clock, t), qos=1)
    ms.order = None


def bridge_state(ms: MachineSim, adapter: Adapter, clock: Clock, t: float) -> dict:
    pose = adapter.pose                      # read once: with ROS it is set from another thread, and the
    if pose is not None:                     # position published must be the one "ok" was decided on
        ms.x, ms.y, ms.heading = pose
    safety, errors = ms.safety_state()
    fault = adapter.fault or (None if pose is not None else "vehicleNoPose")
    if fault is not None:
        safety, errors = "stopped", [*errors, wire.error(fault)]
    errors = [e for e in errors if e["code"] != "gnssQuality"]
    if ms.gnss_bad:
        errors.append(wire.error("gnssQuality", adapter.fix_name, "warning"))
    st = wire.state(ms.site, ms.name, ms.seq, ms.order["orderId"] if ms.order else None, round(ms.x, 3),
                    round(ms.y, 3), round(ms.heading, 4), False, round(ms.progress, 4), safety, errors,
                    ts=clock.ts(t))
    st["driving"] = bool(ms.order is not None and not ms.waiting and safety == "ok")
    if ms.stop_latency_s:
        st["stopLatencyS"] = round(max(ms.stop_latency_s), 4)
    return st
