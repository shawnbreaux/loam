"""A simulated lidar probe (mast or tethered drone): scans the true ground on its rate and adds noise.

Accuracy in the program is read as a 2-sigma bound, as in the reference simulator.
Scripted outages stop it publishing, which is what makes the controller's
stale-data hold fire.
"""
from __future__ import annotations

import time

import numpy as np

from .. import wire
from ..checker import Model
from ..grid import SiteGrid
from .bus import Bus, tolerant, wait_clock


def run_probe(model: Model, base_dir: str, broker: str, seed: int = 11) -> None:
    site = model.site["name"]
    name, p = next(iter(model.probes.items()))
    g = SiteGrid(model, base_dir)
    sigma = p["accuracy"].to(g.unit) / 2
    rate = p["rate"].to("min")
    rng = np.random.default_rng(seed)
    bus = Bus(broker, f"loam-probe-{site}-{name}").start(
        [f"sim/{site}/clock", f"sim/{site}/faults", f"sim/{site}/world/scan/{name}", f"sim/{site}/end"])
    clock = wait_clock(bus, site)
    while clock.minutes() < 0:
        time.sleep(0.01)
    faults: dict = {}
    seq, last = 0, -1e9
    while True:
        t = clock.minutes()
        for topic, msg in bus.drain(0.02):
            with tolerant(topic):
                if topic == f"sim/{site}/end":
                    bus.stop()
                    return
                if topic == f"sim/{site}/faults":
                    faults = msg
                elif topic.startswith(f"sim/{site}/world/scan/"):
                    z = wire.decode_heights(msg["data"], tuple(msg["shape"])).astype(float)
                    z = z + rng.normal(0, sigma, z.shape)
                    seq += 1
                    bus.pub(wire.topic(site, None, "terrain"),
                            wire.terrain(site, name, seq, (g.x0, g.y0), g.cell, g.shape, data=wire.encode_heights(z),
                                         sigma=sigma, ts=clock.ts(msg["t"])), qos=1)
        out = any(f["from"] <= t < f["to"] for f in faults.get("probe_outage", []))
        if not out and t - last >= rate:
            last = t
            bus.pub(f"sim/{site}/world/scan", {"probe": name}, qos=1)
        time.sleep(0.01)
