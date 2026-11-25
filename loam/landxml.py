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


def write_grid_tin(path: str, name: str, xs: np.ndarray, ys: np.ndarray, z: np.ndarray,
                   units: str = "foot") -> None:
    """Write a regular grid of elevations (z[j, i] at xs[i], ys[j]) as a LandXML TIN."""
    lx = ET.Element("LandXML", {"xmlns": "http://www.landxml.org/schema/LandXML-1.2", "version": "1.2"})
    u = ET.SubElement(lx, "Units")
    ET.SubElement(u, "Imperial" if units == "foot" else "Metric",
                  {"linearUnit": units, "areaUnit": "squareFoot" if units == "foot" else "squareMeter",
                   "volumeUnit": "cubicYard" if units == "foot" else "cubicMeter"})
    surfs = ET.SubElement(lx, "Surfaces")
    s = ET.SubElement(surfs, "Surface", {"name": name})
    d = ET.SubElement(s, "Definition", {"surfType": "TIN"})
    pnts = ET.SubElement(d, "Pnts")
    fcs = ET.SubElement(d, "Faces")
    ny, nx = z.shape
    def pid(i: int, j: int) -> str:
        return str(j * nx + i + 1)
    for j in range(ny):
        for i in range(nx):
            p = ET.SubElement(pnts, "P", {"id": pid(i, j)})
            p.text = f"{ys[j]:.3f} {xs[i]:.3f} {z[j, i]:.3f}"
    for j in range(ny - 1):
        for i in range(nx - 1):
            ET.SubElement(fcs, "F").text = f"{pid(i, j)} {pid(i + 1, j)} {pid(i + 1, j + 1)}"
            ET.SubElement(fcs, "F").text = f"{pid(i, j)} {pid(i + 1, j + 1)} {pid(i, j + 1)}"
    ET.indent(lx)
    ET.ElementTree(lx).write(path, encoding="utf-8", xml_declaration=True)
