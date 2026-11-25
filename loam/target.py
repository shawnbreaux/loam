"""The next-pass target surface: Loam's closed loop, handed to a grade-control system.

Grade-control systems (Cat Grade with 3D, OpenGrade3D, Trimble, Topcon) hold
a blade to whatever surface they are given. Loam closes the loop around them:
survey the ground, compute the surface the next pass should leave, load it,
grade, survey again. Repeat until the program's `until` conditions hold on
measured ground.

For every cell inside a cut, fill or finish zone:

* in tolerance of design: target is the design;
* too high: cut toward design by at most `max_cut` (the tightest task limit
  covering the cell; 0.5 ft if none, as in the simulator);
* too low: build toward design by at most one `lift` (task, then zone; 8 in
  if none).

Exclusion zones (grown by their buffer) and cells outside every zone get no
target (NaN), so the machine has nothing to grade there.
"""
from __future__ import annotations

import numpy as np

from .grid import Grid, SiteGrid
from .units import FT, Quantity

DEFAULT_CUT = Quantity(0.5, "ft")
DEFAULT_LIFT = Quantity(8, "in")


def next_target(grid: SiteGrid, design: Grid, ground: Grid) -> tuple[Grid, dict]:
    m, unit = grid.m, grid.unit
    shape = grid.shape
    cut_cap, lift_cap, tol = np.full(shape, np.inf), np.full(shape, np.inf), np.full(shape, np.inf)
    covered = np.zeros(shape, bool)
    for name, z in m.zones.items():
        if z.get("role") == "exclusion":
            continue
        mk = grid.polygon_zone(name)
        covered |= mk
        tasks = [t for t in m.tasks.values() if t.get("zone") == name]
        cuts = [t["max_cut"] for t in tasks if t.get("max_cut")] or [DEFAULT_CUT]
        lifts = [t["lift"] for t in tasks if t.get("lift")] or [z.get("lift") or DEFAULT_LIFT]
        z_tol = z["tolerance"].to(unit) if z.get("tolerance") else 0.1 * FT / grid.u
        cut_cap[mk] = np.minimum(cut_cap[mk], min(q.to(unit) for q in cuts))
        lift_cap[mk] = np.minimum(lift_cap[mk], min(q.to(unit) for q in lifts))
        tol[mk] = np.minimum(tol[mk], z_tol)
    live = covered & ~grid.exclusion() & np.isfinite(ground) & np.isfinite(design)
    r = ground - design
    with np.errstate(invalid="ignore"):
        tgt = np.where(r > tol, np.maximum(design, ground - cut_cap),
                       np.where(r < -tol, np.minimum(design, ground + lift_cap), design))
    tgt = np.where(live, tgt, np.nan)
    d = np.where(live, tgt - ground, 0.0)
    area = grid.area
    info = {
        "cells": int(live.sum()),
        "in_tolerance_pct": round(100 * float((np.abs(r[live]) <= tol[live]).mean()), 2) if live.any() else 0.0,
        "cut_volume": round(float(-d[d < 0].sum() * area), 3),
        "fill_volume": round(float(d[d > 0].sum() * area), 3),
        "volume_unit": f"{unit}3",
        "max_cut": round(float(-d.min()), 4) if live.any() else 0.0,
        "max_fill": round(float(d.max()), 4) if live.any() else 0.0,
        "done": bool(live.any() and (np.abs(r[live]) <= tol[live]).all()),
    }
    return tgt, info
