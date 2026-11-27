"""`loam level`: a land-levelling design for a field nobody has drawn a design for.

An engineer's grading model is the input to `loam plan`. A farm field usually has none: what the farmer has is
the ground itself (a free elevation model, a drone survey, a rover walk) and a rule for what the field should
become: one plane, falling enough to drain and not enough to erode, with the cuts paying for the fills. This
module turns the one into the other.

1. Ground is read from a LandXML surface or a GeoTIFF elevation model (for example a USGS 3DEP tile, or an
   OpenDroneMap terrain model), inside an optional field boundary (GeoJSON, KML, KMZ or shapefile).
2. A plane is fitted to the ground by least squares. That is the plane that moves the least earth for its
   slope, so the fit is the starting point and the limits bend it only as far as they must.
3. The plane's grade is brought within `min_slope` to `max_slope`. With a `fall` bearing the main grade runs that
   way (the way the rows will drain) and `cross_slope` limits the grade across it.
4. The plane is raised or lowered until cut equals `cut_fill` times fill, the allowance for soil that packs
   down tighter in a fill than it lay in the ground.
5. An optional `edge` width blends the plane into the ground at the field's edge, so the field does not end in a
   bank.

The design and the ground are written as LandXML beside the program, and `loam plan` writes the program from
them. Nothing here is specific to farming: the same plane serves a building pad or a yard.
"""
from __future__ import annotations

import math
import os

import numpy as np

from .grid import polygon_mask
from .landxml import grid_tin, rasterize, read_project, read_tin, tin_boundary, write_tin
from .plan import NICE, plan
from .units import Quantity

# Cut over fill, in place. Soil cut from the ground and spread in a fill ends up denser than it lay, so a design
# that balances on paper comes up short of fill. This is a starting assumption: it depends on the soil.
CUT_FILL = 1.3
# Halvings of the bracket when solving for the plane's height: 2**-50 of a few feet is far below a rounding error.
BISECTIONS = 50
# A slope below this is treated as none at all (it has no direction worth trusting).
NO_SLOPE = 1e-9


def fit_plane(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> tuple[float, float]:
    """The slopes (a, b) of the least-squares plane z = a*x + b*y + c through the points. The constant is left out:
    the caller sets the plane's height by the earthwork balance instead."""
    xy = np.column_stack([x - x.mean(), y - y.mean()])
    (a, b), *_ = np.linalg.lstsq(xy, z - z.mean(), rcond=None)
    return float(a), float(b)


def shape_grade(x: np.ndarray, y: np.ndarray, z: np.ndarray, min_slope: float = 0.0,
                max_slope: float | None = None, fall: float | None = None,
                cross_slope: float | None = None) -> tuple[float, float]:
    """The slopes (a, b) of the plane closest to the ground whose grade is within the limits. Slopes are rise over
    run (0.002 is 0.2 %).

    Without `fall`, the best-fit plane keeps its direction and its grade is brought within `min_slope` to
    `max_slope`. With `fall` (a bearing in degrees clockwise from grid north, the way water should run), the grade
    down that bearing is held within the limits, and the grade across it is the best fit for that choice, held
    within plus or minus `cross_slope` if one is given.
    """
    hi = math.inf if max_slope is None else max_slope
    if min_slope < 0 or hi < min_slope:
        raise ValueError("slope limits must satisfy 0 <= min <= max")
    a, b = fit_plane(x, y, z)
    if fall is None:
        s = math.hypot(a, b)
        want = min(max(s, min_slope), hi)
        if s < NO_SLOPE:
            if want > 0:
                raise ValueError("the ground has no slope of its own: say which way the field should drain (--fall)")
            return 0.0, 0.0
        return a * want / s, b * want / s
    # Work in axes turned to the bearing: u runs down the fall, v to its right. A plane that falls `along` per
    # unit of u and `across` per unit of v is z = -along*u - across*v + c.
    dx, dy = math.sin(math.radians(fall)), math.cos(math.radians(fall))
    u, v = x * dx + y * dy, x * dy - y * dx
    u, v, zc = u - u.mean(), v - v.mean(), z - z.mean()
    along = min(max(-(a * dx + b * dy), min_slope), hi)
    # With the main grade fixed, the best cross grade is a one-variable fit to what is left.
    vv = float(v @ v)
    across = -float(v @ (zc + along * u)) / vv if vv > 0 else 0.0
    if cross_slope is not None:
        across = min(max(across, -abs(cross_slope)), abs(cross_slope))
    return -(along * dx + across * dy), -(along * dy - across * dx)


def balance(diff: np.ndarray, weight: np.ndarray, ratio: float) -> float:
    """The height to add to a design so that cut is `ratio` times fill.

    `diff` is design minus ground before the shift and `weight` is how much of the shift each point takes (1, or
    less in the blended edge). Raising the design only ever turns cut into fill, so cut - ratio*fill falls
    steadily as the design rises and bisection finds where it crosses zero.
    """
    if ratio <= 0:
        raise ValueError("the cut/fill ratio must be positive")
    live = weight > 0
    if not live.any():
        raise ValueError("nothing to level: the edge blend covers the whole field")
    lo, hi = float(-diff[live].max()), float(-diff[live].min())      # all cut at lo, all fill at hi

    def excess(c: float) -> float:
        """Cut minus `ratio` times fill, with the design shifted by c."""
        d = weight * (diff + c)
        return float(-d[d < 0].sum() - ratio * d[d > 0].sum())

    for _ in range(BISECTIONS):
        mid = 0.5 * (lo + hi)
        if excess(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def edge_distance(poly: list, gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest side of a polygon."""
    d2 = np.full(gx.shape, np.inf)
    for k in range(len(poly)):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % len(poly)]
        ex, ey = x2 - x1, y2 - y1
        n2 = ex * ex + ey * ey
        # The nearest point of a side is the foot of the perpendicular, or an end if the foot falls outside it.
        t = np.clip(((gx - x1) * ex + (gy - y1) * ey) / n2, 0.0, 1.0) if n2 > 0 else 0.0
        d2 = np.minimum(d2, (gx - x1 - t * ex) ** 2 + (gy - y1 - t * ey) ** 2)
    return np.sqrt(d2)


def _utm(lon: float, lat: float) -> str:
    """The WGS 84 UTM zone a point lies in, as an EPSG code: the metric grid to work in when the ground file is in
    longitude and latitude."""
    return f"EPSG:{(32600 if lat >= 0 else 32700) + int((lon + 180) // 6) % 60 + 1}"


Shape = tuple[list[tuple[float, float]], str | None, str]


def _placed(shape: Shape, work_crs: str | None) -> list[tuple[float, float]]:
    """A boundary or a line, as (points, the coordinate system its file states, what to call it in a message),
    in the working coordinates. One that states its coordinate system (KML, a shapefile with a .prj) is
    reprojected; one that does not is taken as longitude and latitude if its numbers can be nothing else, and as
    working coordinates otherwise."""
    pts, crs, label = shape
    if crs is None and work_crs and all(abs(x) <= 180 and abs(y) <= 90 for x, y in pts):
        crs = "EPSG:4326"
    if crs is None:
        return pts
    if work_crs is None:
        raise ValueError(f"{label} is in map coordinates but the ground file states no coordinate system, "
                         "so it cannot be placed on the ground")
    from .geo import _transformer
    xs, ys = _transformer(crs, work_crs).transform([p[0] for p in pts], [p[1] for p in pts])
    return [(float(a), float(b)) for a, b in zip(xs, ys, strict=True)]


class Ground:
    """A ground file opened for design work: its length unit and coordinate system, and its heights on a lattice.

    The file is a LandXML surface or a GeoTIFF elevation model. A raster in longitude and latitude is worked in
    the metric UTM zone the job lies in, since degrees are no unit to grade in.
    """

    def __init__(self, path: str, surface: str | None, units: str | None, height_unit: str | None,
                 shape: Shape | None, ask: str):
        """`shape` is the boundary or line the design is for, if there is one: it says which UTM zone a raster
        in degrees is worked in, and `ask` is the option to name when such a raster comes without it."""
        self.path, self.height_unit = path, height_unit
        self.raster = os.path.splitext(path)[1].lower() in (".tif", ".tiff")
        self.epsg: str | None = None
        self.info: dict = {}
        if self.raster:
            from .surfaces import raster_info
            self.info = info = raster_info(path)
            self.crs = info["crs"]
            if info["geographic"]:
                if shape is None:
                    raise ValueError(f"the ground file is in longitude and latitude: say which field ({ask})")
                lonlat = _placed(shape, info["crs"])            # in the raster's own degrees, to find the zone
                self.crs = _utm(float(np.mean([p[0] for p in lonlat])), float(np.mean([p[1] for p in lonlat])))
                self.unit = "m"
            else:
                self.unit = info["unit"] or units or ""
            if self.crs and self.crs.startswith("EPSG:"):
                self.epsg = self.crs.split(":")[1]
            elif info["epsg"] and not info["geographic"]:
                self.epsg = str(info["epsg"])
            self.name = "ground"
        else:
            if height_unit:
                raise ValueError("--height-unit is for raster ground; a LandXML file states its own unit")
            proj = read_project(path)
            self.unit = proj["unit"] or units or ""
            self.epsg = proj["epsg"]
            self.crs = f"EPSG:{self.epsg}" if self.epsg else None
            self.pts, self.faces, self.name = read_tin(path, surface)
        if self.unit not in NICE:
            raise ValueError("the ground file does not say its length unit: give --units ft, usft or m"
                             if not self.unit else f"unsupported length unit {self.unit}")

    def cell(self, ring: list[tuple[float, float]], max_cells: int, what: str, cure: str) -> float:
        """The finest round grid size that keeps the bounding box of `ring` within `max_cells`, so that large jobs
        stay fast and small ones stay sharp. `what` is the thing being designed and `cure` what to do about one
        too large, for the message."""
        (x0, y0), (x1, y1) = np.array(ring, float).min(0), np.array(ring, float).max(0)
        cell = next((c for c in NICE[self.unit] if (x1 - x0) * (y1 - y0) / c ** 2 <= max_cells), None)
        if cell is None:
            raise ValueError(f"the {what}'s bounding box is {(x1 - x0) * (y1 - y0) / NICE[self.unit][-1] ** 2:,.0f} "
                             f"cells at the coarsest grid ({NICE[self.unit][-1]} {self.unit}): {cure}")
        return cell

    def heights(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Ground height at every point of the lattice `xs` by `ys` (working coordinates), in the working unit;
        NaN where the file has none."""
        gx, gy = np.meshgrid(xs, ys)
        if not self.raster:
            cell = float(xs[1] - xs[0])
            return rasterize(self.pts, self.faces, xs[0] - cell / 2, ys[0] - cell / 2, cell, len(xs), len(ys))
        from .surfaces import raster_at
        if self.crs and self.info["crs"] and self.crs != self.info["crs"]:
            from .geo import _transformer
            rx, ry = _transformer(self.crs, self.info["crs"]).transform(gx, gy)
            z = raster_at(self.path, np.asarray(rx), np.asarray(ry))[0]
        else:
            z = raster_at(self.path, gx, gy)[0]
        return z * Quantity(1.0, self.height_unit or self.unit).to(self.unit)

    def written(self, stem: str, xs: np.ndarray, ys: np.ndarray, z: np.ndarray, what: str) -> str:
        """The LandXML file the program reads its existing ground from: the ground file itself, or for a raster
        the lattice heights `z` written beside the program."""
        if not self.raster:
            return self.path
        if np.isnan(z).any():
            raise ValueError(f"the ground file leaves {100 * float(np.isnan(z).mean()):.1f} % of the {what}'s "
                             "bounding box uncovered; the site grid needs ground under all of it")
        write_tin(stem + "-ground.xml", self.name, *grid_tin(xs, ys, z), self.unit, epsg=self.epsg)
        return stem + "-ground.xml"


def level(ground: str, out: str, surface: str | None = None, boundary: str | None = None,
          units: str | None = None, height_unit: str | None = None, min_slope: float = 0.0,
          max_slope: float | None = None, fall: float | None = None, cross_slope: float | None = None,
          cut_fill: float = CUT_FILL, edge: str | None = None, fleet: str | None = None,
          tolerance: str | None = None, max_cells: int = 40_000, name: str | None = None,
          compaction: bool = False) -> dict:
    """Design a levelled field from `ground` and write the Loam program for it to `out`. Returns a summary.

    Slopes are in percent. `edge` is a width such as "20 ft". The design and (for a raster ground) the ground
    are written as LandXML next to `out`.
    """
    stem = os.path.splitext(os.path.abspath(out))[0]
    shape: Shape | None = None
    if boundary:
        from .surfaces import polygon_from_file
        shape = (*polygon_from_file(boundary), boundary)
    g = Ground(ground, surface, units, height_unit, shape, "--boundary")
    raster, unit, epsg = g.raster, g.unit, g.epsg
    if shape:
        ring = _placed(shape, g.crs)
    elif raster:
        # The whole raster, less the half pixel at its rim where there is nothing to interpolate between.
        (w, s, e, n), h = g.info["bounds"], g.info["res"] * 0.501
        ring = [(w + h, s + h), (e - h, s + h), (e - h, n - h), (w + h, n - h)]
    else:
        ring = tin_boundary(g.pts, g.faces)
    rp = np.array(ring, float)
    (x0, y0), (x1, y1) = rp.min(0), rp.max(0)
    cell = g.cell(ring, max_cells, "field", "give the field's --boundary, or raise --max-cells")
    # Sample points: a lattice over the field's bounding box. The design is defined at the lattice points inside
    # the field, so it reaches to within a cell of the field's edge; the last strip keeps its ground. With a
    # boundary the lattice runs just past the far sides, because the site grid needs ground under the whole
    # bounding box. A raster used whole has nothing past its own rim, so there the lattice stops inside it.
    exact = bool(boundary) or not raster
    over = math.ceil if exact else math.floor
    nx, ny = over((x1 - x0) / cell) + 1, over((y1 - y0) / cell) + 1
    xs, ys = x0 + np.arange(nx) * cell, y0 + np.arange(ny) * cell
    gx, gy = np.meshgrid(xs, ys)
    z = g.heights(xs, ys)
    # Without a boundary the field is everything the file covers.
    field = polygon_mask(ring, xs, ys) if boundary else np.isfinite(z)
    if field.sum() < 9:
        raise ValueError("the field covers fewer than 9 grid points: check the boundary against the ground file")
    if np.isnan(z[field]).any():
        raise ValueError(f"the ground file leaves {100 * float(np.isnan(z[field]).mean()):.1f} % of the field "
                         "uncovered")
    fx, fy, fz = gx[field], gy[field], z[field]
    a, b = shape_grade(fx, fy, fz, min_slope / 100, None if max_slope is None else max_slope / 100, fall,
                       None if cross_slope is None else cross_slope / 100)
    tilt = a * (fx - fx.mean()) + b * (fy - fy.mean())
    weight = np.ones(len(fz))
    edge_v = 0.0
    if edge:
        num, eu = edge.split()
        edge_v = Quantity(float(num), eu).to(unit)
        if edge_v > 0:
            weight = np.clip(edge_distance(ring, fx, fy) / edge_v, 0.0, 1.0)
    c = balance(tilt - fz, weight, cut_fill)
    change = weight * (tilt + c - fz)                    # design minus ground, at every field point
    design = np.full(z.shape, np.nan)
    design[field] = fz + change
    pts, faces = grid_tin(xs, ys, design)
    if not len(faces):
        raise ValueError(f"the field is narrower than two grid cells ({cell:g} {unit}): check the boundary")
    design_xml = stem + "-design.xml"
    os.makedirs(os.path.dirname(stem) or ".", exist_ok=True)
    write_tin(design_xml, "level", pts, faces, unit, epsg=epsg)
    ground_xml, gname = g.written(stem, xs, ys, z, "field"), g.name
    slope = math.hypot(a, b)
    toward = math.degrees(math.atan2(-a, -b)) % 360 if slope >= NO_SLOPE else None
    deepest_cut, deepest_fill = float(max(-change.min(), 0.0)), float(max(change.max(), 0.0))
    notes = [
        "# Design: one plane fitted to the ground, " + (
            f"falling {slope * 100:.2f} % toward {toward:.0f} degrees (clockwise from grid north)."
            if toward is not None else "dead level."),
        *([f"# The plane is blended into the ground over {edge_v:g} {unit} at the field's edge."]
          if edge_v > 0 else []),
        f"# CHECK: cut/fill ratio {cut_fill:g} (cut in place over fill in place). It depends on the soil.",
        f"# Deepest cut {deepest_cut:.2f} {unit}, deepest fill {deepest_fill:.2f} {unit}."
        " CHECK: the cut against the depth of your topsoil.",
    ]
    r = plan(design_xml, out, design="level", existing=gname, existing_file=ground_xml, fleet=fleet,
             tolerance=tolerance, max_cells=max_cells, name=name or os.path.basename(stem),
             compaction=compaction, source=f"a `loam level` design of {os.path.basename(ground)}", notes=notes,
             limits=ring if exact else None)
    r.update({"slope_pct": round(slope * 100, 3), "falls_toward_deg": None if toward is None else round(toward, 1),
              "cut_fill_ratio": cut_fill, f"deepest_cut_{unit}": round(deepest_cut, 3),
              f"deepest_fill_{unit}": round(deepest_fill, 3), "design_file": design_xml, "ground_file": ground_xml})
    return r
