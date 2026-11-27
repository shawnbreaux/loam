"""Loam Wire: the message layer between a Loam site controller and machines.

Modelled on VDA 5050 (orders + state over MQTT), adapted to earthmoving:
orders carry a work window and a target surface instead of a node graph.

Topics (all JSON, UTF-8):
    loam/v0/<site>/<machine>/order          controller -> machine
    loam/v0/<site>/<machine>/instantAction  controller -> machine (stop, hold, resume)
    loam/v0/<site>/<machine>/state          machine -> controller, at >= 1 Hz
    loam/v0/<site>/<machine>/connection     machine -> controller (retained, with last will)
    loam/v0/<site>/terrain                  probe -> controller (surface tile updates)
    loam/v0/<site>/safety                   controller -> all (site-wide stop/hold)
    loam/v0/<site>/estop                    e-stop radio -> all (pressed / released)
    loam/v0/<site>/detection                site sensors -> controller (people, vehicles)
    loam/v0/<site>/weather                  site sensors -> controller
    loam/v0/<site>/status                   controller -> monitors (task progress, holds)
"""
from __future__ import annotations

import base64
import time

import numpy as np

# Version of the message format, carried in every message. It changes separately from the language and the package.
WIRE_VERSION = "0.0.1"


def topic(site: str, machine: str | None, kind: str) -> str:
    """The MQTT topic for one kind of message: per machine when `machine` is given, site-wide otherwise."""
    return f"loam/v0/{site}/{machine}/{kind}" if machine else f"loam/v0/{site}/{kind}"


def header(seq: int, ts: float | None = None) -> dict:
    """The fields every message starts with: protocol version, a sequence number and a UTC timestamp.

    `ts` is seconds since 1970 (0 if not given). Timestamps carry milliseconds when they have any, because a
    simulated site runs faster than real time and events a second apart in site time can be milliseconds apart.
    """
    ts = ts if ts is not None else 0.0
    # Round to whole milliseconds, carrying into the seconds when rounding reaches 1000.
    ms = round((ts - int(ts)) * 1000)
    sec = int(ts) + ms // 1000
    stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(sec))
    return {"version": WIRE_VERSION, "headerId": seq,
            "timestamp": stamp + (f".{ms % 1000:03d}Z" if ms % 1000 else "Z")}


def parse_ts(stamp: str) -> float:
    """Header timestamp (ISO 8601 UTC, optional milliseconds) to epoch seconds."""
    import calendar
    base, _, frac = stamp.rstrip("Z").partition(".")
    return calendar.timegm(time.strptime(base, "%Y-%m-%dT%H:%M:%S")) + (float("0." + frac) if frac else 0.0)


def order(site: str, machine: str, seq: int, order_id: str, update_id: int, task: str, op: str,
          window: list | None, target: dict, limits: dict, deposit: dict | None = None, ts: float = 0,
          path: list | None = None, swath: float | None = None) -> dict:
    """A unit of work for one machine.

    window : polygon [[x, y], ...] in site coordinates where the machine works
    target : {"surface": "design"} or {"elevation": z} or {"offset": dz} — what the ground should become
    limits : {"maxCut": m, "lift": m, "passes": n, "maxSpeed": m/s} (SI units)
    deposit: optional {"window": [[x, y], ...], "lift": m} where spoil goes
    path   : optional line [[x, y], ...] in site coordinates for the middle of the tool to follow, in the order
             given, for the working part of the pass. Without it the machine runs down the middle of the window.
    swath  : with a path, the width of ground the pass works, m, centred on the path
    """
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "orderId": order_id, "orderUpdateId": update_id,
                "task": task, "op": op, "window": window, "target": target, "limits": limits})
    if deposit:
        msg["deposit"] = deposit
    if path:
        msg["path"] = path
        if swath:
            msg["swath"] = swath
    return msg


def instant_action(site: str, machine: str, seq: int, action: str, reason: str, ts: float = 0) -> dict:
    """An order to one machine that takes effect at once: `stop`, `hold`, `resume` or `slow`, with the reason for the
    log.
    """
    assert action in ("stop", "hold", "resume", "slow")
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "action": action, "reason": reason})
    return msg


def state(site: str, machine: str, seq: int, order_id: str | None, x: float, y: float, heading: float,
          driving: bool, progress: float, safety: str = "ok", errors: list | None = None, ts: float = 0) -> dict:
    """A machine's report: where it is, what it is doing and whether it is safe to work.

    `pose` is in site coordinates with heading in radians (0 along +x, counter-clockwise). `progress` is 0 to 1 for
    the current order and reaches 1 only when the pass is finished. `safetyState` is `ok`, `degraded` (keep going
    carefully is not allowed: the controller holds the machine) or `stopped`.
    """
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "orderId": order_id,
                "pose": {"x": x, "y": y, "heading": heading}, "driving": driving,
                "progress": progress, "safetyState": safety, "errors": errors or []})
    return msg


def connection(site: str, machine: str, seq: int, state: str, ts: float = 0) -> dict:
    """Whether a machine is reachable. Machines publish `online` retained and register `connectionBroken` as their
    MQTT last will, so the broker announces a machine that drops off the network.
    """
    assert state in ("online", "offline", "connectionBroken")
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "connectionState": state})
    return msg


def error(code: str, text: str = "", level: str = "fatal") -> dict:
    """An entry of a state message's `errors`. `orderFailed` means the machine cannot carry out its
    current order (`text` says why): the controller withdraws the pass and the machine is free again."""
    assert level in ("warning", "fatal")
    return {"code": code, "level": level, "text": text}


def safety(site: str, seq: int, command: str = "heartbeat", reason: str = "", ts: float = 0) -> dict:
    """Site-wide message on loam/v0/<site>/safety. Machines stop if no message arrives for 3 s."""
    assert command in ("heartbeat", "stop", "hold", "resume")
    msg = header(seq, ts)
    msg.update({"site": site, "command": command, "reason": reason})
    return msg


def encode_heights(z: np.ndarray) -> str:
    """Encode a height grid as float32 little-endian, base64 (the `float32-le-base64` encoding)."""
    return base64.b64encode(np.ascontiguousarray(z, dtype="<f4").tobytes()).decode("ascii")


def decode_heights(data: str, shape: tuple[int, int]) -> np.ndarray:
    """The inverse of `encode_heights`: a `[rows, columns]` float32 grid from its base64 text."""
    return np.frombuffer(base64.b64decode(data), dtype="<f4").reshape(shape)


def terrain(site: str, probe: str, seq: int, origin: tuple, cell: float, shape: tuple,
            encoding: str = "float32-le-base64", data: str | None = None, sigma: float | None = None,
            ts: float = 0) -> dict:
    """A survey of the ground from a probe: a height grid on the site grid.

    `origin` is the corner of the grid in site coordinates, `cell` the cell size and `shape` (rows, columns), all
    in site units; `sigma` is the probe's stated accuracy. `data` is the grid encoded as named by `encoding`.
    """
    msg = header(seq, ts)
    msg.update({"site": site, "probe": probe, "origin": list(origin), "cell": cell,
                "shape": list(shape), "encoding": encoding, "sigma": sigma, "data": data})
    return msg


# A detection radius that covers any site: "somewhere here, we cannot say where".
EVERYWHERE = 1.0e9


def detection(site: str, sensor: str, seq: int, kind: str, x: float, y: float, ts: float = 0,
              radius: float | None = None) -> dict:
    """Something a site sensor saw at (x, y) in site coordinates.

    `kind` is "person", "unknown" (the sensor cannot say who is where, for example because it has failed: treated
    as a person) or "clear", which ends that sensor's earlier detection. `radius`, in site units, says how well
    the position is known: the person is somewhere within that distance of (x, y). A camera that only knows
    "in my field of view" reports the middle of its view and its reach.
    """
    msg = header(seq, ts)
    msg.update({"site": site, "sensor": sensor, "kind": kind, "x": x, "y": y})
    if radius is not None:
        msg["radius"] = radius
    return msg


def estop(site: str, seq: int, pressed: bool, reason: str = "", ts: float = 0) -> dict:
    """The site e-stop: pressed (every machine stops at once) or released (nothing restarts until a resume)."""
    msg = header(seq, ts)
    msg.update({"site": site, "pressed": pressed, "reason": reason})
    return msg


def weather(site: str, sensor: str, seq: int, rain_m_per_s: float = 0.0, ts: float = 0) -> dict:
    """A weather reading; `rain` is a rate in metres per second."""
    msg = header(seq, ts)
    msg.update({"site": site, "sensor": sensor, "rain": rain_m_per_s})
    return msg


def status(site: str, seq: int, body: dict, ts: float = 0) -> dict:
    """Controller summary for monitors: tasks, machines, holds, replans."""
    msg = header(seq, ts)
    msg.update({"site": site, **body})
    return msg
