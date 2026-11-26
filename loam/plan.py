"""`loam plan`: turn the grading model a project manager receives into a Loam program.

Input is the engineer's LandXML (Civil 3D, OpenRoads, Land Desktop, Trimble
Business Center ...) with an existing-ground and a finished-grade surface,
in one file or two. Output is a commented, checkable `.loam` program:

* site: units and CRS from the file, the job limits traced from the outer
  edge of the design surface, a grid sized for the job;
* zones: the job limits (finish), and derived cut and fill areas that
  re-evaluate on every survey;
* a stockpile for the earthwork balance (spoil or borrow), placed outside the
  limits and marked CHECK;
* tasks: cut to fill, top up, compact, finish, with measured `until`
  conditions; safety rules and a drift replan.

A fleet file (Loam `machine` and `probe` declarations) replaces the default
fleet. Everything the file cannot say (fleet, haul, stockpile location) is
written as an assumption with a `# CHECK:` comment for the PM to confirm.
"""
from __future__ import annotations

import datetime as _dt
import math
import os
import re

import numpy as np

from .landxml import rasterize, read_project, read_tin, tin_boundary
from .units import Quantity

# Words that designers use in surface names. They must stand alone (`EG`, `Site-FG`), so `LEGEND` does not count as
# `EG`.
DESIGN_NAMES = re.compile(r"(^|[^a-z])(fg|finish|final|proposed|prop|design|dsgn|top|grade)([^a-z]|$)", re.I)
EXISTING_NAMES = re.compile(r"(^|[^a-z])(eg|existing|exist|ex|og|original|survey|ground|topo)([^a-z]|$)", re.I)
# Round grid sizes to choose from, the default finish tolerance, and feet per site unit, each by site unit.
NICE = {"ft": [0.5, 1, 2, 2.5, 5, 10, 20], "usft": [0.5, 1, 2, 2.5, 5, 10, 20], "m": [0.1, 0.25, 0.5, 1, 2, 5]}
DEFAULT_TOL = {"ft": "0.1 ft", "usft": "0.1 usft", "m": "3 cm"}
FT_PER = {"ft": 1.0, "usft": 1200 / 3937 / 0.3048, "m": 1 / 0.3048}


def pick_surfaces(names: list[str], design: str | None, existing: str | None) -> tuple[str, str]:
    """Decide which surface in the file is the design and which the existing ground, from their names (FG, Finish,
    Proposed ... against EG, Existing, OG ...). Explicit names win; if the names do not make it clear, say so
    rather than guess.
    """
    if design and existing:
        return design, existing
    d = design or next((n for n in names if DESIGN_NAMES.search(n)), None)
    e = existing or next((n for n in names if n != d and EXISTING_NAMES.search(n)), None)
    if d is None or e is None or d == e:
        raise ValueError(f"cannot tell the design and existing surfaces apart in {names}; "
                         "name them with --design and --existing")
    return d, e


def simplify(poly: list[tuple[float, float]], tol: float) -> list[tuple[float, float]]:
    """Douglas-Peucker on a closed ring."""
    p = np.array(poly, float)
    if len(p) <= 4:
        return poly
    # A closed ring has no ends to anchor the algorithm, so split it at the point farthest from the first one and
    # simplify the two halves.
    far = int(np.argmax(np.hypot(*(p - p[0]).T)))

    def dp(a: np.ndarray) -> list[int]:
        """Douglas-Peucker on an open run of points: keep the point farthest from the straight line between the ends
        if it is more than `tol` away, and recurse on both sides. Returns the indices kept.
        """
        if len(a) < 3:
            return list(range(len(a)))
        d = a[-1] - a[0]
        n = math.hypot(*d) or 1e-12
        dist = np.abs(d[0] * (a[:, 1] - a[0, 1]) - d[1] * (a[:, 0] - a[0, 0])) / n
        k = int(np.argmax(dist))
        if dist[k] <= tol:
            return [0, len(a) - 1]
        left, right = dp(a[:k + 1]), dp(a[k:])
        return left[:-1] + [k + i for i in right]

    ring = np.vstack([p, p[:1]])
    keep = dp(ring[:far + 1]) + [far + i for i in dp(ring[far:])][1:]
    out = [(float(ring[i, 0]), float(ring[i, 1])) for i in keep[:-1]]
    return out if len(out) >= 3 else poly


def _area(p: np.ndarray) -> float:
    """The area enclosed by a ring of points (the shoelace formula)."""
    x, y = p[:, 0], p[:, 1]
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2


def _fmt(v: float) -> str:
    """A number with at most two decimals and no trailing zeros, as a person would write it in a program."""
    return f"{v:.2f}".rstrip("0").rstrip(".")


def _poly(poly: list[tuple[float, float]], indent: str) -> str:
    """A polygon as Loam source text, four points to a line, continuation lines indented by `indent`."""
    pts = [f"({_fmt(x)}, {_fmt(y)})" for x, y in poly]
    lines, row = [], []
    for p in pts:
        row.append(p)
        if len(row) == 4:
            lines.append(", ".join(row))
            row = []
    if row:
        lines.append(", ".join(row))
    return "[" + (",\n" + indent).join(lines) + "]"


DEFAULT_FLEET = """\
probe mast {{
  kind        static              # lidar on a mast or tower; use tethered_uav for a tethered drone
  sensor      lidar
  rate        5 min
  accuracy    {acc}
  coverage    site
  stale_after 15 min
}}

machine dz1 {{ class dozer;   blade 12 ft;  max_speed 3 mph }}
machine dz2 {{ class dozer;   blade 12 ft;  max_speed 3 mph }}
machine rl1 {{ class roller;  drum 7 ft;    max_speed 4 mph }}
"""


def plan(project: str, out: str, design: str | None = None, existing: str | None = None,
         existing_file: str | None = None, fleet: str | None = None, tolerance: str | None = None,
         max_cells: int = 40_000, name: str | None = None) -> dict:
    """Write a Loam program for the grading job in `project`. Returns a summary."""
    info = read_project(project)
    names = [s["name"] for s in info["surfaces"]]
    if existing_file:
        ex_info = read_project(existing_file)
        existing = existing or ex_info["surfaces"][0]["name"]
        design = design or pick_surfaces([*names, "__ex__"], design, "__ex__")[0]
    else:
        design, existing = pick_surfaces(names, design, existing)
    unit = info["unit"] or "ft"
    if unit not in NICE:
        raise ValueError(f"unsupported length unit {unit}")
    dp, dfc, _ = read_tin(project, design)
    ep, efc, _ = read_tin(existing_file or project, existing)
    # Job limits: outer edge of the design surface, simplified to about half a grid cell.
    ring = tin_boundary(dp, dfc)
    rp = np.array(ring)
    (x0, y0), (x1, y1) = rp.min(0), rp.max(0)
    area = _area(rp)
    # The finest round cell size that keeps the grid within `max_cells`, so large jobs stay fast and small ones stay
    # sharp.
    cell = next((c for c in NICE[unit] if (x1 - x0) * (y1 - y0) / c ** 2 <= max_cells), NICE[unit][-1])
    limits = simplify(ring, cell / 2)
    nx, ny = math.ceil((x1 - x0) / cell), math.ceil((y1 - y0) / cell)
    fg = rasterize(dp, dfc, x0, y0, cell, nx, ny)
    eg = rasterize(ep, efc, x0, y0, cell, nx, ny)
    if np.isnan(eg).any():
        miss = float(np.isnan(eg).mean() * 100)
        raise ValueError(f"existing surface '{existing}' leaves {miss:.1f} % of the job's grid uncovered; "
                         "the existing ground must cover the design surface's extent")
    job = np.isfinite(fg)
    d = np.where(job, fg - eg, 0.0)
    a_cell = cell * cell
    tol_txt = tolerance or DEFAULT_TOL[unit]
    num, tunit = tol_txt.lstrip("±").split()
    tol_v = Quantity(float(num), tunit).to(unit)
    # Earthwork quantities in place: only what is outside tolerance counts as work.
    cut = float(-d[d < -tol_v].sum() * a_cell)
    fill = float(d[d > tol_v].sum() * a_cell)
    # Balance with the planner's starting beliefs (swell 1.25, compaction keep 0.80): bank needed = fill / 1.0.
    net = cut - fill
    vu = "yd3" if unit != "m" else "m3"
    to_vu = FT_PER[unit] ** 3 / 27 if unit != "m" else 1.0
    cut_v, fill_v, net_v = cut * to_vu, fill * to_vu, net * to_vu
    # Stockpile: a square outside the limits' bounding box, on the side with the most room is unknown,
    # so the east side; sized for the imbalance plus 25 %.
    side = max(4 * cell, math.sqrt(max(abs(net), 1.0) * 1.25 / 2.0))   # piled 2 units high
    gap = 4 * cell
    sp = [(x1 + gap, y0), (x1 + gap + side, y0), (x1 + gap + side, y0 + side), (x1 + gap, y0 + side)]
    site_name = re.sub(r"[^a-z0-9_]", "_", (name or os.path.splitext(os.path.basename(project))[0]).lower())
    if not site_name[0].isalpha():
        site_name = "site_" + site_name
    rel = lambda p: os.path.relpath(p, os.path.dirname(os.path.abspath(out)) or ".")  # noqa: E731
    crs = f"EPSG:{info['epsg']}" if info["epsg"] else f"LOCAL:{unit}"
    lift = "8 in" if unit != "m" else "20 cm"
    max_cut = "0.5 ft" if unit != "m" else "15 cm"
    big = "0.15 ft" if unit != "m" else "5 cm"
    finish_tol = f"{_fmt(tol_v * 150)} cm" if unit == "m" else f"{_fmt(tol_v * 1.5)} {unit}"
    near = "50 ft" if unit != "m" else "15 m"
    t = tol_txt.lstrip("±")
    if fleet:
        with open(fleet, encoding="utf-8") as f:
            fleet_txt = f.read().strip() + "\n"
    else:
        fleet_txt = DEFAULT_FLEET.format(acc="0.05 ft" if unit != "m" else "1.5 cm")
    probe = re.search(r"^probe\s+(\w+)", fleet_txt, re.M)
    if not probe:
        raise ValueError("the fleet needs a probe (the site's measurement source)")
    exporting, importing = net > 0, net < 0
    acres_or_ha = area / 10000 if unit == "m" else area * FT_PER[unit] ** 2 / 43560
    today = _dt.date.today().isoformat()
    lines = [
        "loam 0.0.2",
        "",
        f"# Generated by `loam plan` on {today} from {os.path.basename(project)}"
        + (f" ({info['application']})" if info["application"] else ""),
        f"# Design surface '{design}', existing surface '{existing}'"
        + (f" from {os.path.basename(existing_file)}" if existing_file else "") + ".",
        f"# Job limits: outer edge of the design surface, {acres_or_ha:.2f} {'ha' if unit == 'm' else 'acres'},"
        f" simplified to {_fmt(cell / 2)} {unit}.",
        f"# Earthwork in place: cut {cut_v:,.0f} {vu}, fill {fill_v:,.0f} {vu}, "
        f"{'surplus' if exporting else 'deficit'} {abs(net_v):,.0f} {vu} before swell and compaction.",
        "# Lines marked CHECK are assumptions the grading file cannot tell us. Confirm them before work.",
        "",
        f"site {site_name} {{",
        f'  crs      "{crs}"' + ("" if info["epsg"] else "   # CHECK: no coordinate system in the file"),
        f"  units    {unit}",
        f'  design   "{rel(project)}" format landxml surface "{design}"',
        f'  existing "{rel(existing_file or project)}" format landxml surface "{existing}"',
        f"  boundary polygon {_poly(limits, '                   ')}",
        f"  grid     {_fmt(cell)} {unit}",
        "}",
        "",
        "# CHECK: fleet. Replace with your machines and probe (loam plan --fleet fleet.loam).",
        fleet_txt.rstrip(),
        "",
        "# The job: everything inside the design surface's edge, finished to tolerance.",
        f"zone limits   {{ role finish; area polygon {_poly(limits, '                                 ')};"
        f" tolerance ±{t} }}",
        "# Derived zones: re-evaluated on every survey, so they shrink as the work gets done.",
        f"zone cut_area  {{ role cut;  area where residual > {t};  tolerance ±{t} }}",
        f"zone fill_area {{ role fill; area where residual < -{t}; tolerance ±{t}",
        f"                 lift {lift}; compaction 95 % proctor }}",
        "",
        "# CHECK: stockpile location and quantity. "
        + ("Surplus material goes here." if exporting else "Borrow comes from here."),
        f"stockpile sp1 {{ area polygon {_poly(sp, '                          ')};",
        f"                 available {max(0.0, -net_v * 1.25) if importing else 0:.0f} {vu}; material \"site soil\" }}",
        "",
        "task cut_to_fill {",
        "  op      cut",
        "  zone    cut_area",
        "  by      class dozer",
        f"  max_cut {max_cut} per pass",
        "  spoil   -> [fill_area, sp1]       # fill low ground first; surplus to the stockpile",
        f"  until   residual(limits).high < {finish_tol}",
        "}",
        "",
        "task top_up {",
        "  op     fill",
        "  zone   fill_area",
        "  by     class dozer",
        "  source [sp1]",
        f"  lift   {lift}",
        "  after  cut_to_fill done",
        f"  until  residual(limits).low > -{finish_tol}",
        "}",
        "",
        "task compact {",
        "  op     compact",
        "  zone   limits",
        "  by     class roller",
        "  passes 4",
        "  after  cut_to_fill start",
        "  until  done(top_up) and compaction(limits) >= 95 %",
        "}",
        "",
        "task finish_grade {",
        "  op     finish",
        "  zone   limits",
        "  by     class dozer",
        "  after  cut_to_fill done",
        "  after  top_up done",
        "  source [sp1]",
        "  until  in_tolerance(limits)",
        "}",
        "",
        "rule people_near_machines {",
        f"  when proximity(person, machine) < {near}",
        "  do   stop all",
        "}",
        "",
        f"rule {probe.group(1)}_stale {{",
        f"  when age({probe.group(1)}) > 15 min",
        "  do   hold all",
        "}",
        "",
        "rule rain {",
        "  when weather(rain) > 0.25 in/h",
        "  do   pause task compact",
        "}",
        "",
        "replan drift {",
        f"  when deviation() > {big}",
        "}",
        "",
    ]
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return {"out": out, "site": site_name, "units": unit, "design": design, "existing": existing,
            "grid": f"{_fmt(cell)} {unit}", "cells": nx * ny, "limits_vertices": len(limits),
            f"cut_{vu}": round(cut_v), f"fill_{vu}": round(fill_v), "balance": "surplus" if exporting else "deficit"}
