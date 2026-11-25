"""Write Loam surfaces and plans in the formats field tools load.

* LandXML TIN: Cat Grade with 3D (loaded by USB or VisionLink), Trimble,
  Topcon, Leica and most machine-control systems.
* OpenGrade3D `.agd` design grid (the OptiSurface layout OpenGrade3D reads).
* AgOpenGPS field folder: Field.txt (origin), Boundary.txt (site boundary,
  exclusion zones as inner boundaries) and TrackLines.txt (one AB line per
  work zone).

Formats were taken from the tools' own source: AgOpenGPS
(github.com/agopengps-official/AgOpenGPS, AgOpenGPS.Core streamers and
GPS/IO) and OpenGrade3D (github.com/Pat-I/OpenGrade3D, FileOpenAgdDesign).
Nothing here links to or copies their code.
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

from .geo import Frame
from .grid import Grid, SiteGrid
from .landxml import grid_tin, write_tin


def landxml(path: str, grid: SiteGrid, z: Grid, frame: Frame | None, name: str = "LOAM",
            world_units: str = "m") -> int:
    """Write a surface as a LandXML TIN, in projected world coordinates when a frame is given.

    Returns the number of triangles written.
    """
    pts, faces = grid_tin(grid.xs, grid.ys, z)
    if not len(faces):
        raise ValueError("the surface has no complete cells to triangulate")
    units = grid.unit
    if frame is not None:
        e, n = frame.local_to_projected(pts[:, 0], pts[:, 1])
        pts = np.column_stack([e, n, frame.height_to_world(pts[:, 2])])
        units = world_units
    write_tin(path, name, pts, faces, units)
    return len(faces)


def _fmt(v: float, nd: int) -> str:
    return f"{v:.{nd}f}"


def agd(path: str, grid: SiteGrid, ground: Grid, target: Grid, frame: Frame,
        benchmark: tuple[float, float] = (0.0, 0.0), step: int | None = None) -> int:
    """Write an OpenGrade3D design file. Heights are metres in the receiver's vertical datum.

    Rows are `lat,lon,existing,proposed,cutfill,code`: one MB (master benchmark),
    the site boundary as 2PER, then the grid as 3GRD. cutfill = proposed - existing
    (positive = fill); a blank proposed means "no design here". Returns the grid rows.
    """
    if step is None:
        step = max(1, round(1.0 / (grid.cell * grid.u)))          # about 1 m spacing, as OpenGrade3D advises
    to_m = lambda v: frame.height_to_world(v) * frame.world_m  # noqa: E731
    fallback = float(np.nanmean(ground))
    def ground_at(x: float, y: float) -> float:
        v = float(grid.sample(ground, np.array([x]), np.array([y]))[0])
        return v if math.isfinite(v) else fallback

    lines = ["Latitude,Longitude,Existing Elevation,Proposed Elevation,Cut Fill,Code"]
    def row(x: float, y: float, ex: float, pr: float | None, code: str) -> str:
        lat, lon = frame.local_to_wgs84(x, y)
        e = float(to_m(ex))
        if pr is None:
            return f"{float(lat):.9f},{float(lon):.9f},{_fmt(e, 3)},,,{code}"
        p = float(to_m(pr))
        return f"{float(lat):.9f},{float(lon):.9f},{_fmt(e, 3)},{_fmt(p, 3)},{_fmt(p - e, 3)},{code}"

    bx, by = benchmark
    lines.append(row(bx, by, ground_at(bx, by), None, "MB"))
    for x, y in grid.boundary or []:
        lines.append(row(x, y, ground_at(x, y), None, "2PER"))
    n = 0
    for j in range(0, grid.ny, step):
        for i in range(0, grid.nx, step):
            ex = ground[j, i]
            if not math.isfinite(ex):
                continue
            pr = target[j, i]
            lines.append(row(grid.xs[i], grid.ys[j], ex, pr if math.isfinite(pr) else None, "3GRD"))
            n += 1
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return n


def _ring(frame: Frame, poly: list) -> list[str]:
    """Polygon as AgOpenGPS easting,northing,heading lines (field origin = site (0, 0))."""
    pts = [tuple(float(v) for v in frame.local_to_en(x, y)) for x, y in poly]
    out = [str(len(pts))]
    for k, (e, n) in enumerate(pts):
        e2, n2 = pts[(k + 1) % len(pts)]
        out.append(f"{e:.3f},{n:.3f},{math.atan2(e2 - e, n2 - n) % (2 * math.pi):.5f}")
    return out


def agopengps(folder: str, grid: SiteGrid, frame: Frame, name: str | None = None) -> dict:
    """Write an AgOpenGPS field folder. Returns what was written."""
    lat0, lon0 = frame.wgs84_origin()
    if grid.boundary is None:
        raise ValueError("AgOpenGPS needs a site boundary polygon")
    name = name or grid.m.site["name"]
    os.makedirs(folder, exist_ok=True)
    stamp = time.strftime("%Y-%B-%d %I:%M:%S %p", time.localtime())
    _write(folder, "Field.txt", [stamp, "$FieldDir", name, "$Offsets", "0,0", "Convergence", "0",
                                 "StartFix", f"{lat0!r},{lon0!r}"])
    bnd = ["$Boundary", "False", *_ring(frame, grid.boundary)]
    inner = [n for n, z in grid.m.zones.items() if z.get("role") == "exclusion"]
    for zn in inner:
        bnd += ["False", *_ring(frame, grid.m.zones[zn]["area"]["polygon"])]
    _write(folder, "Boundary.txt", bnd)
    blades = [md["blade"].to(grid.unit) for md in grid.m.machines.values() if md.get("blade")]
    half = min(blades) / 2 if blades else 0.0
    trk = ["$TrackLines"]
    tracks = []
    for zn, z in grid.m.zones.items():
        if z.get("role") == "exclusion" or "polygon" not in (z.get("area") or {}):
            continue
        p = np.array(z["area"]["polygon"], float)
        (x0, y0), (x1, y1) = p.min(0), p.max(0)
        if x1 - x0 >= y1 - y0:          # run along the long side, first pass half a blade in from the edge
            a, b = (x0, y0 + half), (x1, y0 + half)
        else:
            a, b = (x0 + half, y0), (x0 + half, y1)
        ea, na = (float(v) for v in frame.local_to_en(*a))
        eb, nb = (float(v) for v in frame.local_to_en(*b))
        heading = math.atan2(eb - ea, nb - na) % (2 * math.pi)
        trk += [f"loam {zn}", f"{heading:.5f}", f"{ea:.3f},{na:.3f}", f"{eb:.3f},{nb:.3f}", "0", "2", "True", "0"]
        tracks.append(zn)
    _write(folder, "TrackLines.txt", trk)
    return {"folder": folder, "inner_boundaries": inner, "tracks": tracks}


def _write(folder: str, fname: str, lines: list[str]) -> None:
    with open(os.path.join(folder, fname), "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(lines) + "\n")
