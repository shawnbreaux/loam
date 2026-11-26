"""The site height grid shared by the simulator and the field adapters.

A `SiteGrid` is the raster every Loam surface lives on: cells of `cell` site
units, centred at `xs[i], ys[j]`, covering the site boundary and every zone
polygon. Arrays are indexed `[j, i]` (row = y, column = x).
"""
from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import ast as ast_
from .checker import Model
from .landxml import rasterize, read_tin
from .units import Quantity, length_unit_si

Grid = NDArray[np.float64]
Mask = NDArray[np.bool_]


def cell_eval(node: Any, actual: Grid, design: Grid, u: float) -> Any:
    """Evaluate an `area where` condition per cell (names: residual, design, actual), SI units."""
    if isinstance(node, ast_.Ident):
        return {"residual": actual - design, "design": design, "actual": actual}[node.name] * u
    if isinstance(node, ast_.Num):
        return node.value
    if isinstance(node, ast_.Qty):
        return node.q.si
    if isinstance(node, ast_.Not):
        return ~cell_eval(node.expr, actual, design, u)
    if isinstance(node, ast_.BinOp):
        a, b = cell_eval(node.left, actual, design, u), cell_eval(node.right, actual, design, u)
        ops: dict[str, Callable[[], Any]] = {
            "and": lambda: a & b, "or": lambda: a | b, "<": lambda: a < b, "<=": lambda: a <= b,
            ">": lambda: a > b, ">=": lambda: a >= b, "+": lambda: a + b, "-": lambda: a - b,
            "*": lambda: a * b, "/": lambda: a / b}
        return ops[node.op]()
    raise ValueError("unsupported 'area where' expression")


def polygon_mask(poly: list, xs: Grid, ys: Grid) -> Mask:
    """Even-odd point-in-polygon test at cell centres."""
    gx, gy = np.meshgrid(xs, ys)
    return points_in_polygon(poly, gx, gy)


def points_in_polygon(poly: list, x: Grid, y: Grid) -> Mask:
    """Even-odd point-in-polygon test for arrays of points."""
    inside = np.zeros(np.shape(x), bool)
    for k in range(len(poly)):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % len(poly)]
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        inside ^= crosses & (x < xint)
    return inside


class SiteGrid:
    """Geometry of a checked program's site, plus its polygon zone masks."""

    def __init__(self, model: Model, base_dir: str = "."):
        self.m = model
        self.base_dir = base_dir
        site = model.site
        self.unit: str = site.get("units", "ft")
        self.u = length_unit_si(self.unit)          # metres per site unit
        self.cell = (site.get("grid") or Quantity(1, self.unit)).to(self.unit)
        self.boundary: list | None = (site.get("boundary") or {}).get("polygon")
        polys = [self.boundary] if self.boundary else []
        polys += [z["area"]["polygon"] for z in model.zones.values() if "polygon" in (z.get("area") or {})]
        if not polys:
            raise ValueError("the site needs a boundary or zone polygons to size its grid")
        pts = np.array([p for poly in polys for p in poly], float)
        (self.x0, self.y0), (self.x1, self.y1) = pts.min(0), pts.max(0)
        self.nx = int(np.ceil((self.x1 - self.x0) / self.cell))
        self.ny = int(np.ceil((self.y1 - self.y0) / self.cell))
        self.xs = self.x0 + (np.arange(self.nx) + 0.5) * self.cell
        self.ys = self.y0 + (np.arange(self.ny) + 0.5) * self.cell
        self.area = self.cell ** 2
        self.masks: dict[str, Mask | None] = {
            name: polygon_mask(z["area"]["polygon"], self.xs, self.ys) if "polygon" in (z.get("area") or {}) else None
            for name, z in model.zones.items()}

    @property
    def shape(self) -> tuple[int, int]:
        return (self.ny, self.nx)

    def surface(self, spec: dict, allow_gaps: bool = False) -> Grid:
        """Rasterize a site surface (`design` or `existing`) given as a LandXML TIN."""
        if spec.get("format", "landxml") != "landxml":
            raise ValueError(f"only LandXML surfaces are supported; got format {spec.get('format')}")
        path = spec["file"] if os.path.isabs(spec["file"]) else os.path.join(self.base_dir, spec["file"])
        pts, faces, _ = read_tin(path, spec.get("surface"))
        z = rasterize(pts, faces, self.x0, self.y0, self.cell, self.nx, self.ny)
        if not allow_gaps and np.isnan(z).any():
            raise ValueError(f"{path}: surface does not cover the whole site grid")
        return z

    def design_and_existing(self) -> tuple[Grid, Grid]:
        """Design and existing ground on the grid. Where the design surface stops (outside the job's
        limits), the design is the existing ground: no work is wanted there. Existing ground must
        cover the whole grid."""
        ex = self.m.site.get("existing") or {}
        if "file" not in ex:
            raise ValueError("the site needs an existing-ground surface file")
        z = self.surface(ex)
        d = self.surface(self.m.site["design"], allow_gaps=True)
        if np.isnan(d).all():
            raise ValueError("the design surface does not overlap the site grid")
        return np.where(np.isnan(d), z, d), z

    def zone_mask(self, name: str, actual: Grid, design: Grid) -> Mask:
        """A zone's cells: its polygon, or its `area where` condition evaluated on `actual`."""
        mk = self.masks.get(name)
        if mk is not None:
            return mk
        if name in self.m.stockpiles:
            return np.zeros(self.shape, bool)
        return np.asarray(cell_eval(self.m.zones[name]["area"]["where"], actual, design, self.u), bool)

    def valid_shift(self, dj: int, di: int) -> Mask:
        v = np.ones(self.shape, bool)
        if dj > 0:
            v[:dj] = False
        elif dj < 0:
            v[dj:] = False
        if di > 0:
            v[:, :di] = False
        elif di < 0:
            v[:, di:] = False
        return v

    def exclusion(self, mask_of=None) -> Mask:
        """Cells no work may touch: exclusion zones grown by their buffer (square dilation)."""
        mask_of = mask_of or self.polygon_zone
        ex = np.zeros(self.shape, bool)
        for name, z in self.m.zones.items():
            if z.get("role") != "exclusion":
                continue
            mk = mask_of(name)
            b = int(np.ceil(z["buffer"].to(self.unit) / self.cell)) if z.get("buffer") else 0
            grown = mk.copy()
            for dj in range(-b, b + 1):
                for di in range(-b, b + 1):
                    grown |= np.roll(np.roll(mk, dj, 0), di, 1) & self.valid_shift(dj, di)
            ex |= grown
        return ex

    def polygon_zone(self, name: str) -> Mask:
        mk = self.masks.get(name)
        if mk is None:
            raise ValueError(f"zone '{name}' has a 'where' area; field adapters need polygon areas")
        return mk

    def cell_of(self, x: Grid, y: Grid) -> tuple[NDArray[np.intp], NDArray[np.intp], Mask]:
        """Column, row and in-grid flag of points in site coordinates."""
        i = np.floor((np.asarray(x) - self.x0) / self.cell).astype(np.intp)
        j = np.floor((np.asarray(y) - self.y0) / self.cell).astype(np.intp)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        return i, j, ok

    def sample(self, z: Grid, x: Grid, y: Grid) -> Grid:
        """Bilinear sample of a cell-centred grid at points; NaN outside the grid or next to a gap."""
        fx = (np.asarray(x, float) - self.xs[0]) / self.cell
        fy = (np.asarray(y, float) - self.ys[0]) / self.cell
        i0 = np.clip(np.floor(fx).astype(np.intp), 0, max(self.nx - 2, 0))
        j0 = np.clip(np.floor(fy).astype(np.intp), 0, max(self.ny - 2, 0))
        i1, j1 = np.minimum(i0 + 1, self.nx - 1), np.minimum(j0 + 1, self.ny - 1)
        tx, ty = np.clip(fx - i0, 0, 1), np.clip(fy - j0, 0, 1)
        v = ((1 - tx) * (1 - ty) * z[j0, i0] + tx * (1 - ty) * z[j0, i1]
             + (1 - tx) * ty * z[j1, i0] + tx * ty * z[j1, i1])
        half = 0.5
        out = (fx < -half) | (fx > self.nx - half) | (fy < -half) | (fy > self.ny - half)
        return np.where(out, np.nan, v)
