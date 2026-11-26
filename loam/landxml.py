"""Minimal LandXML TIN surface reader and writer.

Reads <Surface><Definition surfType="TIN"> with <Pnts><P id>N E Z</P> and
<Faces><F>a b c</F>, and rasterizes it onto a regular grid. LandXML lists
point coordinates as northing, easting, elevation (y x z).
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

try:  # hardened parser for untrusted files, if installed (pip install defusedxml)
    from defusedxml.ElementTree import parse as _parse
except ImportError:  # stdlib expat >= 2.4.1 already limits entity-expansion attacks
    _parse = ET.parse

import numpy as np


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def read_tin(path: str, surface: str | None = None):
    """Return (points[N,3] as x,y,z, faces[M,3] as indices, name)."""
    root = _parse(path).getroot()
    surfaces = [el for el in root.iter() if _local(el.tag) == "Surface"]
    if not surfaces:
        raise ValueError(f"{path}: no <Surface> found")
    if surface:
        matches = [s for s in surfaces if s.get("name") == surface]
        if not matches:
            names = ", ".join(repr(s.get("name")) for s in surfaces)
            raise ValueError(f"{path}: no surface named {surface!r} (found {names})")
        surf = matches[0]
    else:
        surf = surfaces[0]
    ids: dict[str, int] = {}
    pts: list[tuple[float, float, float]] = []
    faces: list[list[int]] = []
    for el in surf.iter():
        tag = _local(el.tag)
        if tag == "P":
            n, e, z = (float(v) for v in (el.text or "").split()[:3])
            ids[el.get("id", str(len(pts) + 1))] = len(pts)
            pts.append((e, n, z))
        elif tag == "F":
            if el.get("i") == "1":  # invisible face
                continue
            faces.append([ids[v] for v in (el.text or "").split()[:3]])
    if not faces:
        raise ValueError(f"{path}: surface has no faces")
    return np.array(pts, float), np.array(faces, int), surf.get("name")


def rasterize(pts: np.ndarray, faces: np.ndarray, x0: float, y0: float, cell: float,
              nx: int, ny: int) -> np.ndarray:
    """Sample a TIN at cell centres. Cells outside the TIN are NaN."""
    z = np.full((ny, nx), np.nan)
    xs = x0 + (np.arange(nx) + 0.5) * cell
    ys = y0 + (np.arange(ny) + 0.5) * cell
    for a, b, c in faces:
        (xa, ya, za), (xb, yb, zb), (xc, yc, zc) = pts[a], pts[b], pts[c]
        det = (yb - yc) * (xa - xc) + (xc - xb) * (ya - yc)
        if abs(det) < 1e-12:
            continue
        i0 = max(0, int(np.floor((min(xa, xb, xc) - x0) / cell)))
        i1 = min(nx - 1, int(np.ceil((max(xa, xb, xc) - x0) / cell)))
        j0 = max(0, int(np.floor((min(ya, yb, yc) - y0) / cell)))
        j1 = min(ny - 1, int(np.ceil((max(ya, yb, yc) - y0) / cell)))
        if i1 < i0 or j1 < j0:
            continue
        gx, gy = np.meshgrid(xs[i0:i1 + 1], ys[j0:j1 + 1])
        l1 = ((yb - yc) * (gx - xc) + (xc - xb) * (gy - yc)) / det
        l2 = ((yc - ya) * (gx - xc) + (xa - xc) * (gy - yc)) / det
        l3 = 1 - l1 - l2
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        val = l1 * za + l2 * zb + l3 * zc
        sub = z[j0:j1 + 1, i0:i1 + 1]
        sub[inside] = val[inside]
    return z


LINEAR_UNITS = {"ft": "foot", "foot": "foot", "usft": "USSurveyFoot", "m": "meter", "meter": "meter"}


def grid_tin(xs: np.ndarray, ys: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Triangulate a regular grid (z[j, i] at xs[i], ys[j]). Cells with a NaN corner get no faces."""
    ny, nx = z.shape
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel(), gy.ravel(), z.ravel()])
    k = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, d = k[:-1, :-1].ravel(), k[:-1, 1:].ravel(), k[1:, 1:].ravel(), k[1:, :-1].ravel()
    faces = np.stack([np.column_stack([a, b, c]), np.column_stack([a, c, d])], 1).reshape(-1, 3)  # per cell
    faces = faces[np.isfinite(pts[faces, 2]).all(1)]
    used = np.unique(faces)
    remap = np.full(len(pts), -1)
    remap[used] = np.arange(len(used))
    return pts[used], remap[faces]


def write_tin(path: str, name: str, pts: np.ndarray, faces: np.ndarray, units: str = "ft", digits: int = 3) -> None:
    """Write a TIN surface (pts[N] as x, y, z; faces[M] as point indices) to LandXML 1.2."""
    linear = LINEAR_UNITS.get(units)
    if linear is None:
        raise ValueError(f"no LandXML unit for {units!r}")
    imperial = linear != "meter"
    lx = ET.Element("LandXML", {"xmlns": "http://www.landxml.org/schema/LandXML-1.2", "version": "1.2"})
    u = ET.SubElement(lx, "Units")
    ET.SubElement(u, "Imperial" if imperial else "Metric",
                  {"linearUnit": linear, "areaUnit": "squareFoot" if imperial else "squareMeter",
                   "volumeUnit": "cubicYard" if imperial else "cubicMeter"})
    s = ET.SubElement(ET.SubElement(lx, "Surfaces"), "Surface", {"name": name})
    d = ET.SubElement(s, "Definition", {"surfType": "TIN"})
    pnts, fcs = ET.SubElement(d, "Pnts"), ET.SubElement(d, "Faces")
    for k, (x, y, z) in enumerate(pts):
        ET.SubElement(pnts, "P", {"id": str(k + 1)}).text = f"{y:.{digits}f} {x:.{digits}f} {z:.{digits}f}"
    for a, b, c in faces:
        ET.SubElement(fcs, "F").text = f"{a + 1} {b + 1} {c + 1}"
    ET.indent(lx)
    ET.ElementTree(lx).write(path, encoding="utf-8", xml_declaration=True)


def write_grid_tin(path: str, name: str, xs: np.ndarray, ys: np.ndarray, z: np.ndarray,
                   units: str = "ft") -> None:
    """Write a regular grid of elevations (z[j, i] at xs[i], ys[j]) as a LandXML TIN; NaN cells are left out."""
    pts, faces = grid_tin(xs, ys, z)
    if not len(faces):
        raise ValueError("nothing to write: the grid has no complete cells")
    write_tin(path, name, pts, faces, units)


UNIT_OF_LINEAR = {"foot": "ft", "USSurveyFoot": "usft", "meter": "m", "metre": "m"}


def read_project(path: str) -> dict:
    """Project-level facts of a LandXML file: length unit, coordinate system, surface names."""
    root = _parse(path).getroot()
    out: dict = {"unit": None, "epsg": None, "crs_name": None, "surfaces": [], "application": None}
    for el in root.iter():
        tag = _local(el.tag)
        if tag in ("Imperial", "Metric") and out["unit"] is None:
            out["unit"] = UNIT_OF_LINEAR.get(el.get("linearUnit", ""))
        elif tag == "CoordinateSystem":
            out["epsg"] = el.get("epsgCode") or out["epsg"]
            out["crs_name"] = el.get("name") or el.get("desc") or out["crs_name"]
        elif tag == "Application":
            out["application"] = " ".join(filter(None, (el.get("manufacturer"), el.get("name"), el.get("version"))))
        elif tag == "Surface":
            out["surfaces"].append({"name": el.get("name"), "desc": el.get("desc"),
                                    "state": el.get("state") or el.get("stateType")})
    return out


def tin_boundary(pts: np.ndarray, faces: np.ndarray) -> list[tuple[float, float]]:
    """Outer boundary of a TIN as a polygon: the longest closed loop of edges used by one face."""
    e = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    uniq, count = np.unique(e, axis=0, return_counts=True)
    edges = uniq[count == 1]
    nxt: dict[int, list[int]] = {}
    for a, b in edges:
        nxt.setdefault(int(a), []).append(int(b))
        nxt.setdefault(int(b), []).append(int(a))
    seen: set[tuple[int, int]] = set()
    best: list[int] = []
    for start in list(nxt):
        for first in nxt[start]:
            if (start, first) in seen:
                continue
            loop, prev, cur = [start], start, first
            seen.add((start, first))
            while cur != start and len(loop) <= len(edges):
                loop.append(cur)
                cand = [v for v in nxt[cur] if v != prev and (cur, v) not in seen] or \
                       [v for v in nxt[cur] if (cur, v) not in seen]
                if not cand:
                    break
                seen.add((cur, cand[0]))
                prev, cur = cur, cand[0]
            if cur == start and (not best or _area(pts[loop]) > _area(pts[best])):
                best = loop
    if not best:
        raise ValueError("surface has no closed outer boundary")
    return [(float(pts[i, 0]), float(pts[i, 1])) for i in best]


def _area(p: np.ndarray) -> float:
    x, y = p[:, 0], p[:, 1]
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2
