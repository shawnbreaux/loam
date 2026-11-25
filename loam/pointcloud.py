"""Point clouds to site height grids.

Readers stream points in bounded chunks, so a 100-million-point scan costs a
few tens of megabytes, not gigabytes:

* LAS 1.0-1.4, point formats 0-10, uncompressed. DJI Terra exports Zenmuse L2
  scans as LAS; most lidar tools can. LAZ needs `pip install laspy[lazrs]`.
* PCD (ascii and binary), as written by livox_ros_driver2 + PCL tools, the
  usual way to record a Livox Mid-360.
* Plain text "x y z" (or comma separated), one point per line.

Coordinates are taken as projected (easting, northing, height) and mapped into
the site with `Frame.projected_to_local`.
"""
from __future__ import annotations

import os
import struct
from collections.abc import Iterator
from itertools import islice

import numpy as np
from numpy.typing import NDArray

from .geo import Frame
from .grid import Grid, SiteGrid

CHUNK = 250_000                 # points per read: ~100 bytes of working memory each
Points = NDArray[np.float64]          # shape (n, 3)


# -- LAS -----------------------------------------------------------------------------
def _las_header(f) -> dict:
    head = f.read(375)
    if head[:4] != b"LASF":
        raise ValueError("not a LAS file (missing LASF signature)")
    major, minor = head[24], head[25]
    offset, = struct.unpack_from("<I", head, 96)
    fmt, rec_len = head[104], struct.unpack_from("<H", head, 105)[0]
    count, = struct.unpack_from("<I", head, 107)
    if (major, minor) >= (1, 4) and count == 0:
        count, = struct.unpack_from("<Q", head, 247)
    scale = struct.unpack_from("<3d", head, 131)
    off = struct.unpack_from("<3d", head, 155)
    if fmt & 0xC0:
        raise ValueError("compressed LAZ points; install laspy with lazrs and convert to LAS first")
    if fmt > 10:
        raise ValueError(f"unsupported LAS point format {fmt}")
    return {"version": f"{major}.{minor}", "offset": offset, "format": fmt, "rec_len": rec_len,
            "count": count, "scale": scale, "off": off}


def read_las(path: str, classes: set[int] | None = None, chunk: int = CHUNK) -> Iterator[Points]:
    """Yield (n, 3) float arrays of x, y, z, optionally keeping only some classifications."""
    with open(path, "rb") as f:
        h = _las_header(f)
        cls_at = 15 if h["format"] <= 5 else 16
        dt = np.dtype({"names": ["x", "y", "z", "c"], "formats": ["<i4", "<i4", "<i4", "u1"],
                       "offsets": [0, 4, 8, cls_at], "itemsize": h["rec_len"]})
        f.seek(h["offset"])
        left = h["count"]
        while left > 0:
            rec = np.fromfile(f, dt, min(chunk, left))
            if rec.size == 0:
                raise ValueError(f"{path}: file ends before its {h['count']} points")
            left -= rec.size
            if classes is not None:
                c = rec["c"] & 0x1F if h["format"] <= 5 else rec["c"]
                rec = rec[np.isin(c, list(classes))]
            out = np.empty((rec.size, 3))
            for k, name in enumerate("xyz"):
                out[:, k] = rec[name] * h["scale"][k] + h["off"][k]
            yield out


def write_las(path: str, pts: Points, scale: float = 0.001, classification: int = 2) -> None:
    """Write a minimal LAS 1.2, point format 0 file (for tests and exporting as-built scans)."""
    pts = np.asarray(pts, float)
    off = np.floor(pts.min(0)) if len(pts) else np.zeros(3)
    rec = np.zeros(len(pts), np.dtype([("xyz", "<i4", 3), ("i", "<u2"), ("b", "u1"), ("c", "u1"),
                                        ("a", "i1"), ("u", "u1"), ("p", "<u2")]))
    rec["xyz"] = np.round((pts - off) / scale).astype("<i4")
    rec["c"] = classification
    head = bytearray(227)
    head[0:4] = b"LASF"
    head[24], head[25] = 1, 2
    struct.pack_into("<H", head, 94, 227)
    struct.pack_into("<I", head, 96, 227)
    head[104] = 0
    struct.pack_into("<H", head, 105, 20)
    struct.pack_into("<I", head, 107, len(pts))
    struct.pack_into("<3d", head, 131, scale, scale, scale)
    struct.pack_into("<3d", head, 155, *off)
    if len(pts):
        mx, mn = pts.max(0), pts.min(0)
        struct.pack_into("<6d", head, 179, mx[0], mn[0], mx[1], mn[1], mx[2], mn[2])
    with open(path, "wb") as f:
        f.write(head)
        rec.tofile(f)


# -- PCD -----------------------------------------------------------------------------
_PCD_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
              ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("U", 8): "<u8", ("I", 8): "<i8"}


def read_pcd(path: str, chunk: int = CHUNK) -> Iterator[Points]:
    with open(path, "rb") as f:
        meta: dict[str, list[str]] = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: PCD header has no DATA line")
            words = line.decode("ascii", "replace").split()
            if not words or words[0].startswith("#"):
                continue
            meta[words[0].upper()] = words[1:]
            if words[0].upper() == "DATA":
                break
        fields = meta.get("FIELDS", [])
        if not {"x", "y", "z"} <= set(fields):
            raise ValueError(f"{path}: PCD has no x, y, z fields")
        n = int(meta.get("POINTS", meta.get("WIDTH", ["0"]))[0])
        kind = meta["DATA"][0].lower()
        if kind == "ascii":
            cols = [fields.index(c) for c in "xyz"]
            count = [int(c) for c in meta.get("COUNT", ["1"] * len(fields))]
            if any(c != 1 for c in count):
                cols = [sum(count[:i]) for i in cols]
            while True:
                rows = [ln.split() for ln in islice(f, chunk)]
                if not rows:
                    return
                yield np.array([[float(r[c]) for c in cols] for r in rows if r])
        if kind != "binary":
            raise ValueError(f"{path}: PCD DATA {kind} is not supported; save as binary or ascii")
        sizes, types = meta["SIZE"], meta["TYPE"]
        counts = meta.get("COUNT", ["1"] * len(fields))
        names = [f if f not in fields[:k] else f"_pad{k}" for k, f in enumerate(fields)]   # PCL pads with "_"
        try:
            dt = np.dtype([(name, _PCD_TYPES[(t.upper(), int(s))], (int(c),) if int(c) > 1 else ())
                           for name, s, t, c in zip(names, sizes, types, counts, strict=True)])
        except (KeyError, ValueError) as e:
            raise ValueError(f"{path}: bad PCD field description ({e})") from None
        left = n
        while left > 0:
            rec = np.fromfile(f, dt, min(chunk, left))
            if rec.size == 0:
                raise ValueError(f"{path}: file ends before its {n} points")
            left -= rec.size
            yield np.column_stack([rec["x"], rec["y"], rec["z"]]).astype(float)


def write_pcd(path: str, pts: Points) -> None:
    """Write a binary x y z float32 PCD (for tests and tools that want PCD)."""
    pts = np.asarray(pts, np.float32)
    head = (f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(head.encode("ascii"))
        pts.astype("<f4").tofile(f)


def read_xyz(path: str, chunk: int = CHUNK) -> Iterator[Points]:
    with open(path, encoding="utf-8") as f:
        while True:
            rows = []
            for ln in islice(f, chunk):
                parts = ln.replace(",", " ").split()
                if len(parts) >= 3:
                    try:
                        rows.append([float(v) for v in parts[:3]])
                    except ValueError:
                        continue          # header or comment line
            if not rows:
                return
            yield np.array(rows)


def read_points(path: str, ground_only: bool = False, chunk: int = CHUNK) -> Iterator[Points]:
    ext = os.path.splitext(path)[1].lower()
    if ext == ".las":
        return read_las(path, {2} if ground_only else None, chunk)
    if ext == ".laz":
        raise ValueError("LAZ is compressed; convert to LAS (e.g. laszip or laspy) first")
    if ext == ".pcd":
        return read_pcd(path, chunk)
    if ext in (".xyz", ".txt", ".pts", ".csv"):
        return read_xyz(path, chunk)
    raise ValueError(f"{path}: unknown point cloud type {ext!r} (use .las, .pcd or .xyz)")


# -- gridding -------------------------------------------------------------------------
class Binner:
    """Streams points into per-cell count, mean and minimum height."""

    def __init__(self, grid: SiteGrid):
        self.g = grid
        self.n = np.zeros(grid.nx * grid.ny, np.int64)
        self.s = np.zeros(grid.nx * grid.ny)
        self.lo = np.full(grid.nx * grid.ny, np.inf)
        self.seen = self.used = 0

    def add(self, x: Grid, y: Grid, z: Grid) -> None:
        self.seen += len(z)
        i, j, ok = self.g.cell_of(x, y)
        ok &= np.isfinite(z)
        k = j[ok] * self.g.nx + i[ok]
        zz = np.asarray(z, float)[ok]
        self.used += len(zz)
        self.n += np.bincount(k, minlength=self.n.size)
        self.s += np.bincount(k, zz, minlength=self.s.size)
        np.minimum.at(self.lo, k, zz)

    def result(self, stat: str = "mean", min_points: int = 1) -> tuple[Grid, NDArray[np.int64]]:
        n = self.n.reshape(self.g.shape)
        with np.errstate(invalid="ignore", divide="ignore"):
            z = (self.s / self.n if stat == "mean" else self.lo).reshape(self.g.shape)
        return np.where(n >= max(min_points, 1), z, np.nan), n


def grid_cloud(path: str, grid: SiteGrid, frame: Frame, stat: str = "mean", ground_only: bool = False,
               min_points: int = 1, chunk: int = CHUNK) -> tuple[Grid, dict]:
    """Rasterize a point cloud file onto the site grid. Returns (heights, stats)."""
    if stat not in ("mean", "min"):
        raise ValueError("stat must be 'mean' or 'min'")
    b = Binner(grid)
    for p in read_points(path, ground_only, chunk):
        x, y = frame.projected_to_local(p[:, 0], p[:, 1])
        b.add(x, y, frame.height_to_local(p[:, 2]))
    z, _ = b.result(stat, min_points)
    return z, {"points_read": int(b.seen), "points_on_grid": int(b.used),
               "cells_covered_pct": round(100 * float(np.isfinite(z).mean()), 2)}


def fill_gaps(z: Grid, max_cells: int = 2) -> Grid:
    """Fill small holes by repeated averaging of filled neighbours, at most `max_cells` cells deep."""
    z = z.copy()
    for _ in range(max_cells):
        hole = np.isnan(z)
        if not hole.any():
            break
        p = np.pad(z, 1, constant_values=np.nan)
        nb = np.stack([p[1:-1, :-2], p[1:-1, 2:], p[:-2, 1:-1], p[2:, 1:-1]])
        cnt = np.isfinite(nb).sum(0)
        with np.errstate(invalid="ignore"):
            avg = np.nansum(nb, 0) / cnt
        z = np.where(hole & (cnt > 0), avg, z)
    return z
