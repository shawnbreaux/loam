"""`loam swale`: a drainage swale designed from the ground and a line drawn where it should run.

A swale is a broad, shallow channel that carries water off a field. What the person who wants one has is the
ground (a free elevation model, a drone survey, a rover walk) and a line: where the water should go. This module
turns the two into a design surface and a Loam program, as `loam level` does for a field.

1. Ground is read from a LandXML surface or a GeoTIFF elevation model. The centreline is a file (a path drawn in
   Google Earth, a GeoJSON LineString, a shapefile polyline) or a list of points.
2. The bottom of the swale is the highest line along the centreline that is at least `depth` below the ground
   everywhere and falls at least `grade` all the way, so that water never stands in it. Being the highest such
   line, it is the one that moves the least earth. It runs the way the line is drawn unless told otherwise or,
   left to choose, the way that moves less.
3. The cross-section is a flat bottom `bottom` wide with sides rising one unit for every `side` units out. It is
   measured from the nearest point of the centreline, which rounds the two ends.
4. The design is the lower of that section and the ground: a swale is cut, never filled. Where the sides have
   not met the ground within the strip worked on, the strip is widened and the design made again.

The design and the ground are written as LandXML beside the program. The program's cut and finishing tasks run
`along` the centreline, so each pass follows the swale and comes back along it.

The sides are a slope across the direction of travel. A blade that tilts, or a machine whose own grade control
takes the planned surface, can cut them. A level blade moved by one actuator cannot: it cuts the flat bottom.
"""
from __future__ import annotations

import math
import os

import numpy as np

from . import line
from .landxml import grid_tin, write_tin
from .level import Ground, Shape, _placed
from .plan import NICE, plan
from .units import Quantity

# How many times the strip is doubled in width while the sides still have not met the ground. Three doublings
# reach eight times the first guess (sides running out to twice the depth); ground that climbs away from the
# swale as steeply as its sides never meets them at all.
WIDENINGS = 3
# A cut shallower than this is no cut: it is the rounding of the files.
NO_CUT = 1e-6


def invert(ground: np.ndarray, station: np.ndarray, depth: float, grade: float) -> np.ndarray:
    """The height of the swale's bottom at each station, for water running from the first station to the last.

    `ground` is the ground along the centreline at the stations, `depth` the least depth below it and `grade` the
    least fall (rise over run). The answer is the highest line that is nowhere less than `depth` below the ground
    and nowhere falls less than `grade`: at each station, the lowest of the lines of slope `grade` that start
    `depth` under the ground at a station upstream of it.
    """
    return np.minimum.accumulate(ground - depth + grade * station) - grade * station


def _between(xs: np.ndarray, ys: np.ndarray, z: np.ndarray, px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """Heights at points inside a lattice, each from the four lattice points around it (bilinear)."""
    cell = float(xs[1] - xs[0])
    fx = np.clip((px - xs[0]) / cell, 0.0, len(xs) - 1 - 1e-9)
    fy = np.clip((py - ys[0]) / cell, 0.0, len(ys) - 1 - 1e-9)
    i, j = np.floor(fx).astype(int), np.floor(fy).astype(int)
    tx, ty = fx - i, fy - j
    return ((z[j, i] * (1 - tx) + z[j, i + 1] * tx) * (1 - ty) + (z[j + 1, i] * (1 - tx) + z[j + 1, i + 1] * tx) * ty)


def _length(text: str, unit: str, what: str) -> float:
    """A length written as a number and a unit ("4 ft", "30 cm"), in the working unit."""
    try:
        num, u = text.split()
        v = Quantity(float(num), u).to(unit)
    except (ValueError, KeyError) as e:
        raise ValueError(f"{what} reads as a number and a unit, for example '4 ft' (got {text!r})") from e
    if v < 0:
        raise ValueError(f"{what} cannot be negative")
    return float(v)


def swale(ground: str, out: str, along: list[tuple[float, float]] | None = None, line_file: str | None = None,
          surface: str | None = None, units: str | None = None, height_unit: str | None = None,
          bottom: str | None = None, depth: str | None = None, side: float = 4.0, grade: float = 0.5,
          flow: str | None = None, fleet: str | None = None, tolerance: str | None = None,
          max_cells: int = 100_000, name: str | None = None) -> dict:
    """Design a swale on `ground` along a line and write the Loam program for it to `out`. Returns a summary.

    The line is `along` (points, in the ground file's coordinates or longitude and latitude) or `line_file`.
    `bottom` and `depth` are lengths such as "4 ft" (by default 4 ft and 1 ft, or 1.2 m and 0.3 m on metric
    ground); `side` is the side slope as run over rise (4 is 4:1);
    `grade` is the least fall along the bottom in percent. `flow` is "forward" (water runs the way the line is
    drawn), "reverse", or None to take whichever moves less earth.
    """
    if (along is None) == (line_file is None):
        raise ValueError("give the swale's centreline as a file (--line) or as points (--along), one of the two")
    if side < 0 or grade < 0 or flow not in (None, "forward", "reverse"):
        raise ValueError("the side slope and the grade cannot be negative, and the flow is forward or reverse")
    if line_file:
        from .surfaces import line_from_file
        shape: Shape = (*line_from_file(line_file), line_file)
    else:
        shape = ([(float(x), float(y)) for x, y in along or []], None, "the line")
    g = Ground(ground, surface, units, height_unit, shape, "--line or --along")
    unit = g.unit
    centre = _placed(shape, g.crs)
    total = line.length(centre)
    bottom_v = _length(bottom or ("1.2 m" if unit == "m" else "4 ft"), unit, "the bottom width")
    depth_v = _length(depth or ("0.3 m" if unit == "m" else "1 ft"), unit, "the depth")
    if depth_v <= 0:
        raise ValueError("the depth must be above zero")
    cp = np.array(centre, float)
    reach = bottom_v / 2 + 2 * max(side, 1.0) * depth_v             # half the width of the strip worked on
    feature = max(bottom_v, side * depth_v)                         # the grid must show at least this much
    for _ in range(WIDENINGS + 1):
        (x0, y0), (x1, y1) = cp.min(0) - reach, cp.max(0) + reach
        cell = g.cell([(x0, y0), (x1, y1)], max_cells, "swale", "raise --max-cells, or design it in shorter pieces")
        if cell > feature / 2:
            fine = max((c for c in NICE[unit] if c <= feature / 2), default=NICE[unit][0])
            raise ValueError(f"a {cell:g} {unit} grid is too coarse for this swale (its bottom or one side is "
                             f"{feature:g} {unit} across): raise --max-cells to "
                             f"{math.ceil((x1 - x0) * (y1 - y0) / fine ** 2):,}, or design it in shorter pieces")
        # A lattice over the strip's bounding box, running just past its far sides: the site grid needs ground
        # under the whole box.
        nx, ny = math.ceil((x1 - x0) / cell) + 1, math.ceil((y1 - y0) / cell) + 1
        xs, ys = x0 + np.arange(nx) * cell, y0 + np.arange(ny) * cell
        z = g.heights(xs, ys)
        gx, gy = np.meshgrid(xs, ys)
        s, off = line.project(centre, gx, gy)
        strip = np.abs(off) <= reach
        if np.isnan(z[strip]).any():
            raise ValueError(f"the ground file leaves {100 * float(np.isnan(z[strip]).mean()):.1f} % of the "
                             f"ground within {reach:g} {unit} of the line uncovered")
        # The ground along the centreline, at points no more than a cell apart.
        dense = line.densify(centre, cell)
        st = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(dense, axis=0).T))))
        gl = _between(xs, ys, z, dense[:, 0], dense[:, 1])
        fwd = invert(gl, st, depth_v, grade / 100)
        rev = invert(gl[::-1], st[-1] - st[::-1], depth_v, grade / 100)[::-1]
        # Left to choose, water runs the way that needs less digging along the centreline.
        forward = flow == "forward" or (flow is None and float((gl - fwd).sum()) <= float((gl - rev).sum()))
        zb = fwd if forward else rev
        out_from = np.clip(np.abs(off) - bottom_v / 2, 0.0, None)   # distance outside the flat bottom
        rise = np.where(out_from > 0, out_from / side if side > 0 else np.inf, 0.0)      # upright sides: side 0
        section = np.interp(s, st, zb) + rise
        cut = np.where(strip, np.clip(z - section, 0.0, None), 0.0)
        # Done when the sides have met the ground at least a cell inside the strip's edge.
        if not (cut[np.abs(off) > reach - cell] > NO_CUT).any():
            break
        reach *= 2
    else:
        raise ValueError(f"the swale's sides have not met the ground {reach / 2:g} {unit} from its centreline: "
                         "the ground climbs away as steeply as the sides. Give a steeper side slope (a smaller "
                         "--side-slope), a shallower depth, or a line across flatter ground")
    design = np.where(strip, z - cut, np.nan)
    pts, faces = grid_tin(xs, ys, design)
    if not len(faces):
        raise ValueError(f"the swale is narrower than two grid cells ({cell:g} {unit}): raise --max-cells")
    stem = os.path.splitext(os.path.abspath(out))[0]
    os.makedirs(os.path.dirname(stem) or ".", exist_ok=True)
    design_xml = stem + "-design.xml"
    write_tin(design_xml, "swale", pts, faces, unit, epsg=g.epsg)
    ground_xml = g.written(stem, xs, ys, z, "swale")
    # What the design came to. Grades are along the bottom, in the direction the water runs.
    fall = np.diff(zb) / np.diff(st) * (-1 if forward else 1)
    end = -1 if forward else 0
    outlet = float(gl[end] - zb[end])
    deepest = float(cut.max())
    wide = float(np.abs(off[cut > NO_CUT]).max()) if (cut > NO_CUT).any() else 0.0
    a, b = (centre[0], centre[-1]) if forward else (centre[-1], centre[0])
    notes = [
        f"# Design: a swale {total:,.0f} {unit} long, bottom {bottom_v:g} {unit} wide, sides {side:g}:1, at least "
        f"{depth_v:g} {unit} deep.",
        f"# Water runs from ({a[0]:.1f}, {a[1]:.1f}) to ({b[0]:.1f}, {b[1]:.1f}); the bottom falls "
        f"{float(fall.min()) * 100:.2f} % to {float(fall.max()) * 100:.2f} %, {abs(float(zb[-1] - zb[0])):.2f} "
        f"{unit} in all.",
        f"# Deepest cut {deepest:.2f} {unit}; the cut reaches {wide:.1f} {unit} from the centreline at its widest.",
        *([f"# On the {cell:g} {unit} grid the sides rise {cell / side:.2f} {unit} from one cell to the next: for "
           "finer sides raise --max-cells."] if side > 0 else []),
        f"# CHECK: the swale ends {outlet:.2f} {unit} below the ground at its outlet. Water leaves only if the",
        "#        ground beyond is lower: a ditch, a pond, a slope.",
        "# CHECK: the sides slope across the direction of travel. They need a blade that tilts, or a machine",
        "#        with its own grade control. A level blade on one actuator cuts the flat bottom only.",
    ]
    r = plan(design_xml, out, design="swale", existing=g.name, existing_file=ground_xml, fleet=fleet,
             tolerance=tolerance, max_cells=max_cells, name=name or os.path.basename(stem), compaction=False,
             source=f"a `loam swale` design on {os.path.basename(ground)}", notes=notes, along=centre, cell=cell)
    r.update({"length": round(total, 2), "flows": "forward" if forward else "reverse",
              "grade_pct": [round(float(fall.min()) * 100, 3), round(float(fall.max()) * 100, 3)],
              f"fall_{unit}": round(abs(float(zb[-1] - zb[0])), 3), f"deepest_cut_{unit}": round(deepest, 3),
              f"cut_reaches_{unit}": round(wide, 2), f"outlet_depth_{unit}": round(outlet, 3),
              "design_file": design_xml, "ground_file": ground_xml})
    return r
