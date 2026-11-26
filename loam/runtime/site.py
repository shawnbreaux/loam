"""`loam simulate-site`: start a simulated site as separate processes.

One process each for the world (true ground, physics, site sensors, clock),
the probe, and every machine. They talk over MQTT exactly as hardware would;
only the world-physics topics under `sim/` are simulation-specific.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

from .bus import Bus


def launch(path: str, broker: str, speed: float, faults_path: str | None, model, until_done: bool = True,
           timeout_min: float = 48 * 60, quiet: bool = False, physics: bool = False,
           soil_table: str | None = None) -> dict:
    site = model.site["name"]
    bus = Bus(broker, f"loam-site-launcher-{site}").start([])
    for t in [f"loam/v0/{site}/status", f"sim/{site}/world/truth", f"sim/{site}/clock", f"sim/{site}/faults",
              *(f"loam/v0/{site}/{n}/connection" for n in model.machines)]:
        bus.clear(t)                                 # nothing retained from an earlier run
    time.sleep(0.3)
    bus.stop()
    base = [sys.executable, "-m", "loam.cli", "_role", path, "--broker", broker]
    out = subprocess.DEVNULL if quiet else None
    world = subprocess.Popen([*base, "--role", "world", "--speed", str(speed)]
                             + (["--faults", faults_path] if faults_path else [])
                             + (["--soil-table", soil_table] if soil_table else []), stdout=out)
    time.sleep(1.0)                                  # the world publishes the clock first
    procs = [world, subprocess.Popen([*base, "--role", "probe"], stdout=out)]
    procs += [subprocess.Popen([*base, "--role", "machine", "--name", n, "--index", str(i)]
                               + (["--physics"] if physics else []), stdout=out)
              for i, n in enumerate(model.machines)]
    bus = Bus(broker, f"loam-site-launcher-{site}").start([f"loam/v0/{site}/status", f"sim/{site}/world/truth"])
    truth: dict = {}
    t_end = time.time() + timeout_min * 60 / speed
    try:
        while time.time() < t_end:
            for topic, msg in bus.drain(0.5):
                if topic.endswith("/truth"):
                    truth = msg
                elif until_done and msg.get("done"):
                    time.sleep(1.0)
                    bus.pub(f"sim/{site}/end", {"why": "controller reports all tasks done"}, qos=1)
                    t_end = 0
            if world.poll() is not None:
                break
        bus.pub(f"sim/{site}/end", {"why": "launcher stopping"}, qos=1)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.send_signal(signal.SIGTERM)
        for topic, msg in bus.drain(2.0):
            if topic.endswith("/truth"):
                truth = msg
        return {"truth": truth, "exit_codes": {name: p.returncode for name, p in
                                               zip(["world", "probe", *model.machines], procs, strict=True)}}
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
        bus.stop()


def role(path: str, model, a) -> None:
    from .machine import run_machine
    from .probe import run_probe
    from .world import DEFAULT_FAULTS, run_world
    base = os.path.dirname(os.path.abspath(path))
    if a.role == "world":
        faults = DEFAULT_FAULTS
        if a.faults:
            with open(a.faults, encoding="utf-8") as f:
                faults = json.load(f)
        compaction = None
        if getattr(a, "soil_table", None):
            from .chrono_soil import load_tables
            compaction = load_tables(a.soil_table)
        run_world(model, base, a.broker, a.speed, faults, compaction=compaction)
    elif a.role == "probe":
        run_probe(model, base, a.broker)
    elif a.role == "machine":
        from ..grid import SiteGrid
        g = SiteGrid(model, base)
        ground = None
        if a.physics:                                   # the machine's own copy of the ground, as surveyed
            _, z = g.design_and_existing()
            # The ground must reach every stockpile and parking spot, with room to turn: extend the
            # grid's edge heights outward to cover them plus 30 m.
            pts = [(g.x0, g.y0), (g.x1, g.y1), (g.x0, g.y0 - 10 * g.cell)]
            pts += [tuple(p) for sp in model.stockpiles.values() for p in (sp.get("area") or {}).get("polygon", [])]
            arr = np.array(pts, float)
            margin = 30.0 / g.u
            lo = np.ceil(np.maximum([g.x0, g.y0] - arr.min(0) + margin, 0) / g.cell).astype(int)
            hi = np.ceil(np.maximum(arr.max(0) - [g.x0 + g.nx * g.cell, g.y0 + g.ny * g.cell] + margin, 0)
                         / g.cell).astype(int)
            zp = np.pad(z, ((lo[1], hi[1]), (lo[0], hi[0])), mode="edge")
            ground = (zp, g.x0 - lo[0] * g.cell, g.y0 - lo[1] * g.cell, g.cell)
        park = (g.x0 + (a.index + 1) * 6 * g.cell, g.y0 - 10 * g.cell)
        run_machine(model, a.name, a.broker, park, ground)
