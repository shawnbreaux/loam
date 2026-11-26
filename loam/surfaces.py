"""Surfaces and areas from the open formats survey and design tools write.

* LandXML TIN (Civil 3D, OpenRoads, Trimble Business Center): built in.
* IFC 4.3 triangulated surfaces (IfcTriangulatedFaceSet, IfcTriangulatedIrregularNetwork), as BIM tools
  export earthworks; needs `pip install ifcopenshell` (LGPL-3.0).
* GeoTIFF elevation models (OpenDroneMap `odm_dem/dtm.tif`, QGIS, GDAL, most drone and lidar tools);
  needs `pip install rasterio` (BSD-3, bundles GDAL, MIT).
* GeoJSON polygons for `area file "x.geojson"` (QGIS exports): built in. Coordinates are read as the
  site's CRS, so export the layer in the site's CRS (QGIS: "Save Features As", CRS = project CRS).

All readers return heights in site units on the caller's grid; cells a surface does not cover are NaN.
"""
from __future__ import annotations

import json
import os
from typing import Any

import numpy as np

from .landxml import rasterize, read_tin


def tin_from_ifc(path: str, name: str | None = None) -> tuple[np.ndarray, np.ndarray, str]:
    """Points (metres, x y z) and faces of the named triangulated surface in an IFC file."""
    try:
        import ifcopenshell
        import ifcopenshell.geom
    except ImportError as e:
        raise ValueError("IFC surfaces need ifcopenshell: pip install ifcopenshell") from e
    f = ifcopenshell.open(path)
    settings = ifcopenshell.geom.settings()
    # Ask for coordinates in the model's world frame (metres), with every placement and rotation in the file applied.
    settings.set("use-world-coords", True)
    found = []
    for p in f.by_type("IfcProduct"):
        rep = getattr(p, "Representation", None)
        if rep is None:
            continue
        kinds = {it.is_a() for r in rep.Representations for it in r.Items}
        if kinds & {"IfcTriangulatedFaceSet", "IfcTriangulatedIrregularNetwork"}:
            found.append(p)
    if not found:
        raise ValueError(f"{path}: no triangulated surface (IfcTriangulatedFaceSet/IrregularNetwork)")
    if name is not None:
        match = [p for p in found if p.Name == name]
        if not match:
            raise ValueError(f"{path}: no surface named {name!r} (found {', '.join(repr(p.Name) for p in found)})")
        found = match
    geom: Any = ifcopenshell.geom              # typed loosely: its stubs differ between releases
    shape = geom.create_shape(settings, found[0])
    pts = np.asarray(shape.geometry.verts, float).reshape(-1, 3)
    faces = np.asarray(shape.geometry.faces, int).reshape(-1, 3)
    return pts, faces, found[0].Name or ""


def raster_sample(path: str, xs: np.ndarray, ys: np.ndarray, to_site: float = 1.0) -> np.ndarray:
    """Bilinear sample of a single-band elevation raster at the grid xs (columns) x ys (rows).

    Grid coordinates must be in the raster's CRS; heights are multiplied by `to_site`.
    """
    gx, gy = np.meshgrid(xs, ys)
    return raster_at(path, gx, gy)[0] * to_site


def raster_at(path: str, gx: np.ndarray, gy: np.ndarray) -> tuple[np.ndarray, str | None]:
    """Bilinear heights of a single-band raster at points in its own CRS (NaN outside or on nodata).
    Returns (heights shaped like gx, the raster's CRS as a string or None)."""
    try:
        import rasterio
    except ImportError as e:
        raise ValueError("GeoTIFF surfaces need rasterio: pip install rasterio") from e
    with rasterio.open(path) as src:
        band = src.read(1, masked=True).astype(float).filled(np.nan)
        # The raster's transform maps pixel (column, row) to map coordinates; its inverse maps map coordinates to
        # pixels.
        inv = ~src.transform
        crs = src.crs.to_string() if src.crs else None
    gx, gy = np.asarray(gx, float), np.asarray(gy, float)
    col, row = inv @ (gx.ravel(), gy.ravel())
    col, row = np.asarray(col) - 0.5, np.asarray(row) - 0.5        # pixel centres
    c0, r0 = np.floor(col).astype(int), np.floor(row).astype(int)
    tc, tr = col - c0, row - r0
    h, w = band.shape
    # Bilinear interpolation between the four pixel centres around each point; a point with any of the four off the
    # raster gets no value.
    ok = (c0 >= 0) & (r0 >= 0) & (c0 + 1 < w) & (r0 + 1 < h)
    c0, r0 = np.clip(c0, 0, w - 2), np.clip(r0, 0, h - 2)
    v = ((1 - tc) * (1 - tr) * band[r0, c0] + tc * (1 - tr) * band[r0, c0 + 1]
         + (1 - tc) * tr * band[r0 + 1, c0] + tc * tr * band[r0 + 1, c0 + 1])
    return np.where(ok, v, np.nan).reshape(gx.shape), crs


def raster_crs(path: str) -> str | None:
    """The coordinate reference system a raster declares, or None."""
    import rasterio
    with rasterio.open(path) as src:
        return src.crs.to_string() if src.crs else None


def geotiff_ground(path: str, grid, frame, src_crs: str | None = None) -> np.ndarray:
    """A georeferenced elevation raster (e.g. an OpenDroneMap DTM) sampled onto the site grid."""
    gx, gy = np.meshgrid(grid.xs, grid.ys)
    e, n = frame.local_to_projected(gx, gy)
    src = src_crs or raster_crs(path)
    if src and frame.crs and src != frame.crs:      # site points into the raster's CRS
        from .geo import _transformer
        e, n = _transformer(frame.crs, src).transform(e, n)
        e, n = np.asarray(e), np.asarray(n)
        z, _ = raster_at(path, e, n)
        _, _, z = frame.reproject(e, n, z, src)     # heights back, if both CRSs are compound
    else:
        z, _ = raster_at(path, e, n)
    return frame.height_to_local(z)


def write_geotiff(path: str, z: np.ndarray, grid, frame) -> None:
    """Write a site-grid surface (z[j, i], row 0 = south) as a float32 GeoTIFF in the frame's projected
    coordinates and height units, rotated as the site is; NaN is nodata. The CRS is tagged when known."""
    try:
        import rasterio
        from rasterio.transform import Affine
    except ImportError as e:
        raise ValueError("GeoTIFF output needs rasterio: pip install rasterio") from e
    ny, nx = z.shape
    top = grid.y0 + ny * grid.cell                       # raster row 0 is the north edge of the grid
    c, f = frame.local_to_projected(grid.x0, top)
    ex, nx_ = frame.local_to_projected(grid.x0 + grid.cell, top)
    ey, ny_ = frame.local_to_projected(grid.x0, top - grid.cell)
    # An affine transform built from where three grid corners land in projected coordinates, so a rotated site is
    # written exactly, without resampling.
    t = Affine(float(ex - c), float(ey - c), float(c), float(nx_ - f), float(ny_ - f), float(f))
    with rasterio.open(path, "w", driver="GTiff", height=ny, width=nx, count=1, dtype="float32",
                       crs=frame.crs, transform=t, nodata=np.nan, compress="deflate") as dst:
        dst.write(np.flipud(frame.height_to_world(z)).astype("float32"), 1)


def surface_grid(spec: dict, base_dir: str, x0: float, y0: float, cell: float, nx: int, ny: int,
                 u: float) -> np.ndarray:
    """A declared surface (`design`/`existing`) on a site grid, in site units."""
    path = spec["file"] if os.path.isabs(spec["file"]) else os.path.join(base_dir, spec["file"])
    fmt = spec.get("format", "landxml")
    if fmt == "landxml":
        pts, faces, _ = read_tin(path, spec.get("surface"))
        return rasterize(pts, faces, x0, y0, cell, nx, ny)
    if fmt == "ifc":
        pts, faces, _ = tin_from_ifc(path, spec.get("surface"))
        return rasterize(pts / u, faces, x0, y0, cell, nx, ny)       # IFC geometry comes out in metres
    if fmt == "geotiff":
        xs, ys = x0 + (np.arange(nx) + 0.5) * cell, y0 + (np.arange(ny) + 0.5) * cell
        return raster_sample(path, xs, ys)
    raise ValueError(f"unsupported surface format {fmt!r}")


def polygon_from_geojson(path: str) -> list[tuple[float, float]]:
    """Outer ring of the first Polygon (or MultiPolygon part) in a GeoJSON file, closing point dropped."""
    with open(path, encoding="utf-8") as f:
        g = json.load(f)
    stack = [g]
    while stack:
        o = stack.pop(0)
        t = o.get("type")
        if t == "FeatureCollection":
            stack.extend(o.get("features", []))
        elif t == "Feature":
            stack.append(o.get("geometry") or {})
        elif t in ("Polygon", "MultiPolygon"):
            ring = o["coordinates"][0] if t == "Polygon" else o["coordinates"][0][0]
            pts = [(float(p[0]), float(p[1])) for p in ring]
            if len(pts) > 1 and pts[0] == pts[-1]:
                pts = pts[:-1]
            if len(pts) < 3:
                raise ValueError(f"{path}: polygon has fewer than 3 points")
            return pts
    raise ValueError(f"{path}: no Polygon in the GeoJSON")


def resolve_area_files(model, base_dir: str) -> None:
    """Load every `area file "..."` (zones, stockpiles, boundary) into a polygon, in place."""
    holders = [z.get("area") for z in model.zones.values()]
    holders += [sp.get("area") for sp in model.stockpiles.values()]
    holders.append(model.site.get("boundary"))
    for a in holders:
        if a and "file" in a and "polygon" not in a:
            p = a["file"] if os.path.isabs(a["file"]) else os.path.join(base_dir, a["file"])
            a["polygon"] = polygon_from_geojson(p)


def site_geojson(model, grid, frame=None) -> dict:
    """The site's boundary, zones and stockpiles as a GeoJSON FeatureCollection (for QGIS): site
    coordinates, or the frame's projected coordinates when a frame is given."""
    feats = []

    def add(poly, **props):
        """Append one polygon as a GeoJSON Feature with the given properties, closing its ring as GeoJSON requires."""
        xy = np.asarray(poly, float)
        if frame is not None:
            xy = np.column_stack(frame.local_to_projected(xy[:, 0], xy[:, 1]))
        ring = [[float(a), float(b)] for a, b in xy] + [[float(xy[0, 0]), float(xy[0, 1])]]
        feats.append({"type": "Feature", "properties": props, "geometry": {"type": "Polygon", "coordinates": [ring]}})
    if grid.boundary:
        add(grid.boundary, kind="boundary", name=model.site["name"])
    for n, z in model.zones.items():
        if "polygon" in (z.get("area") or {}):
            tol = z.get("tolerance")
            add(z["area"]["polygon"], kind="zone", name=n, role=z.get("role"),
                tolerance=f"{tol.value:g} {tol.unit}" if tol is not None else None)
    for n, sp in model.stockpiles.items():
        if "polygon" in (sp.get("area") or {}):
            add(sp["area"]["polygon"], kind="stockpile", name=n)
    out: dict = {"type": "FeatureCollection", "name": model.site["name"], "features": feats}
    if frame is not None and frame.crs:
        out["crs"] = {"type": "name", "properties": {"name": frame.crs}}
    return out
