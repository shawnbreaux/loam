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

WIRE_VERSION = "0.0.1"


def topic(site: str, machine: str | None, kind: str) -> str:
    return f"loam/v0/{site}/{machine}/{kind}" if machine else f"loam/v0/{site}/{kind}"


def header(seq: int, ts: float | None = None) -> dict:
    ts = ts if ts is not None else 0.0
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
          window: list | None, target: dict, limits: dict, deposit: dict | None = None, ts: float = 0) -> dict:
    """A unit of work for one machine.

    window : polygon [[x, y], ...] in site coordinates where the machine works
    target : {"surface": "design"} or {"elevation": z} or {"offset": dz} — what the ground should become
    limits : {"maxCut": m, "lift": m, "passes": n, "maxSpeed": m/s} (SI units)
    deposit: optional {"window": [[x, y], ...], "lift": m} where spoil goes
    """
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "orderId": order_id, "orderUpdateId": update_id,
                "task": task, "op": op, "window": window, "target": target, "limits": limits})
    if deposit:
        msg["deposit"] = deposit
    return msg


def instant_action(site: str, machine: str, seq: int, action: str, reason: str, ts: float = 0) -> dict:
    assert action in ("stop", "hold", "resume", "slow")
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "action": action, "reason": reason})
    return msg


def state(site: str, machine: str, seq: int, order_id: str | None, x: float, y: float, heading: float,
          driving: bool, progress: float, safety: str = "ok", errors: list | None = None, ts: float = 0) -> dict:
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "orderId": order_id,
                "pose": {"x": x, "y": y, "heading": heading}, "driving": driving,
                "progress": progress, "safetyState": safety, "errors": errors or []})
    return msg


def connection(site: str, machine: str, seq: int, state: str, ts: float = 0) -> dict:
    assert state in ("online", "offline", "connectionBroken")
    msg = header(seq, ts)
    msg.update({"site": site, "machine": machine, "connectionState": state})
    return msg


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
    return np.frombuffer(base64.b64decode(data), dtype="<f4").reshape(shape)


def terrain(site: str, probe: str, seq: int, origin: tuple, cell: float, shape: tuple,
            encoding: str = "float32-le-base64", data: str | None = None, sigma: float | None = None,
            ts: float = 0) -> dict:
    msg = header(seq, ts)
    msg.update({"site": site, "probe": probe, "origin": list(origin), "cell": cell,
                "shape": list(shape), "encoding": encoding, "sigma": sigma, "data": data})
    return msg


def detection(site: str, sensor: str, seq: int, kind: str, x: float, y: float, ts: float = 0) -> dict:
    """Something a site sensor saw at (x, y) in site coordinates; kind "clear" ends earlier detections."""
    msg = header(seq, ts)
    msg.update({"site": site, "sensor": sensor, "kind": kind, "x": x, "y": y})
    return msg


def weather(site: str, sensor: str, seq: int, rain_m_per_s: float = 0.0, ts: float = 0) -> dict:
    msg = header(seq, ts)
    msg.update({"site": site, "sensor": sensor, "rain": rain_m_per_s})
    return msg


def status(site: str, seq: int, body: dict, ts: float = 0) -> dict:
    """Controller summary for monitors: tasks, machines, holds, replans."""
    msg = header(seq, ts)
    msg.update({"site": site, **body})
    return msg
