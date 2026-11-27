"""Point clouds to site height grids.

Readers stream points in bounded chunks, so a 100-million-point scan costs a
few tens of megabytes, not gigabytes:

* LAS 1.0-1.4, point formats 0-10, uncompressed: built in.
* LAZ (compressed LAS), as OpenDroneMap, PDAL and most lidar tools write it:
  `pip install laspy lazrs` (BSD-2 / Apache-2.0).
* PCD (ascii and binary), as written by livox_ros_driver2 + PCL tools, the
  usual way to record a Livox Mid-360.
* Plain text "x y z" (or comma separated), one point per line.

Coordinates are taken as projected (easting, northing, height) and mapped into
the site with `Frame.projected_to_local`.
"""
from __future__ import annotations

import math
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
    """Read the fixed public header of a LAS file: version, where the points start, their record layout and count, and
    the scale and offset that turn the stored integers into coordinates.

    The byte positions are those of the ASPRS LAS specification.
    """
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
        # Each point record starts with x, y, z as 32-bit integers. The classification byte sits at offset 15 in the
        # older point formats (0-5, where its low 5 bits are the class) and at 16 in the newer ones (6-10).
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
# PCD field (TYPE, SIZE) pairs to NumPy little-endian types.
_PCD_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
              ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("U", 8): "<u8", ("I", 8): "<i8"}


def read_pcd(path: str, chunk: int = CHUNK) -> Iterator[Points]:
    """Yield (n, 3) arrays of x, y, z from a Point Cloud Data file, ascii or binary. Other fields (intensity, time,
    ...) are skipped.
    """
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
    """Yield (n, 3) arrays from a text file of `x y z` lines (spaces or commas). Lines that are not three numbers,
    such as a header, are skipped.
    """
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


def read_laz(path: str, classes: set[int] | None = None, chunk: int = CHUNK) -> Iterator[Points]:
    """Yield (n, 3) arrays of x, y, z from a compressed LAZ file, optionally keeping only some classifications. Needs
    laspy with the lazrs backend.
    """
    try:
        import laspy
    except ImportError as e:
        raise ValueError("LAZ needs laspy and lazrs: pip install laspy lazrs") from e
    with laspy.open(path) as r:
        for pts in r.chunk_iterator(chunk):
            keep = np.isin(np.asarray(pts.classification), list(classes)) if classes is not None else slice(None)
            yield np.column_stack([np.asarray(pts.x)[keep], np.asarray(pts.y)[keep], np.asarray(pts.z)[keep]])


def read_points(path: str, ground_only: bool = False, chunk: int = CHUNK) -> Iterator[Points]:
    """Open a point cloud by its file extension and yield its points in chunks. `ground_only` keeps LAS/LAZ class 2
    (ground, as classified by whoever processed the scan).
    """
    ext = os.path.splitext(path)[1].lower()
    if ext == ".las":
        return read_las(path, {2} if ground_only else None, chunk)
    if ext == ".laz":
        return read_laz(path, {2} if ground_only else None, chunk)
    if ext == ".pcd":
        return read_pcd(path, chunk)
    if ext in (".xyz", ".txt", ".pts", ".csv"):
        return read_xyz(path, chunk)
    raise ValueError(f"{path}: unknown point cloud type {ext!r} (use .las, .pcd or .xyz)")


# -- gridding -------------------------------------------------------------------------
class Binner:
    """Streams points into per-cell count, mean and minimum height (and maximum, on request)."""

    def __init__(self, grid: SiteGrid, top: bool = False):
        """Start with empty cells: point count, sum of heights and lowest height per cell. With `top`, the highest
        point per cell is kept too, for finding what stands on the ground."""
        self.g = grid
        self.n = np.zeros(grid.nx * grid.ny, np.int64)
        self.s = np.zeros(grid.nx * grid.ny)
        self.lo = np.full(grid.nx * grid.ny, np.inf)
        self.hi = np.full(grid.nx * grid.ny, -np.inf) if top else None
        self.seen = self.used = 0

    def add(self, x: Grid, y: Grid, z: Grid) -> None:
        """Add a chunk of points (site coordinates). Points off the grid or without a height are counted in `seen` but
        not used.
        """
        self.seen += len(z)
        i, j, ok = self.g.cell_of(x, y)
        ok &= np.isfinite(z)
        k = j[ok] * self.g.nx + i[ok]
        zz = np.asarray(z, float)[ok]
        self.used += len(zz)
        self.n += np.bincount(k, minlength=self.n.size)
        self.s += np.bincount(k, zz, minlength=self.s.size)
        np.minimum.at(self.lo, k, zz)
        if self.hi is not None:
            np.maximum.at(self.hi, k, zz)

    def result(self, stat: str = "mean", min_points: int = 1) -> tuple[Grid, NDArray[np.int64]]:
        """The height grid: the mean height per cell (`stat="mean"`) or the lowest (`"min"`), NaN where a cell has
        fewer than `min_points` points. Also returns the point count per cell.

        The mean suits bare ground; the minimum is the starting point of the ground filter, since vegetation and
        machines only ever add height.
        """
        n = self.n.reshape(self.g.shape)
        with np.errstate(invalid="ignore", divide="ignore"):
            z = (self.s / self.n if stat == "mean" else self.lo).reshape(self.g.shape)
        return np.where(n >= max(min_points, 1), z, np.nan), n


def cloud_crs(path: str) -> str | None:
    """The CRS a LAS/LAZ file declares, when laspy and pyproj can read it."""
    if os.path.splitext(path)[1].lower() not in (".las", ".laz"):
        return None
    try:
        import laspy
        with laspy.open(path) as r:
            c = r.header.parse_crs()
    except Exception:          # laspy/pyproj missing, or no CRS record: the caller's --source-crs applies
        return None
    return c.to_string() if c is not None else None


def grid_cloud(path: str, grid: SiteGrid, frame: Frame, stat: str = "mean", ground_only: bool = False,
               min_points: int = 1, chunk: int = CHUNK, pmf: dict | None = None,
               src_crs: str | None = None, objects: bool = False) -> tuple[Grid, dict]:
    """Rasterize a point cloud file onto the site grid. Returns (heights, stats).

    With `objects` (and the ground filter), the stats also list the person-sized things standing on the ground,
    as `standing_objects` finds them, in site coordinates.

    `pmf` = parameters for `ground_surface` (or {} for its defaults): a first pass finds the lowest point
    per cell, the morphological filter turns that into a ground surface, and the second pass keeps only
    points within `dh_final` of it, so grass, machines and people do not count as ground.
    """
    if stat not in ("mean", "min"):
        raise ValueError("stat must be 'mean' or 'min'")
    if objects and pmf is None:
        raise ValueError("finding objects needs the ground filter (--ground-filter): they are what it removes")
    standing: list[dict] = []

    src = src_crs or (cloud_crs(path) if frame.crs else None)

    def points():
        """The file's points in site coordinates, chunk by chunk. Called once per pass, so the file is read twice when
        the ground filter is on and never held in memory.
        """
        for p in read_points(path, ground_only, chunk):
            e, n, h = frame.reproject(p[:, 0], p[:, 1], p[:, 2], src)
            x, y = frame.projected_to_local(e, n)
            yield x, y, frame.height_to_local(h)
    ground = None
    if pmf is not None:
        b0 = Binner(grid, top=objects)
        for x, y, z in points():
            b0.add(x, y, z)
        ground = ground_surface(b0.result("min")[0] * grid.u, grid.cell * grid.u, **pmf) / grid.u
        tol = pmf.get("dh_final", 0.15) / grid.u
        if b0.hi is not None:
            above = (b0.hi.reshape(grid.shape) - ground) * grid.u            # metres; -inf where nothing was seen
            found = standing_objects(above, grid.cell * grid.u)
            standing = [{"x": round(float(grid.x0 + cx / grid.u), 2), "y": round(float(grid.y0 + cy / grid.u), 2),
                         "r": round(r / grid.u, 2), "height_m": round(h, 2)} for cx, cy, r, h in found]
    b = Binner(grid)
    for x, y, z in points():
        if ground is not None:
            i, j, ok = grid.cell_of(x, y)
            g = np.full(len(z), np.nan)
            g[ok] = ground[j[ok], i[ok]]
            keep = np.isfinite(g) & (z <= g + tol)
            x, y, z = x[keep], y[keep], z[keep]
        b.add(x, y, z)
    z, _ = b.result(stat, min_points)
    return z, {"points_read": int(b.seen if ground is None else b0.seen), "points_on_grid": int(b.used),
               "ground_filter": "pmf" if ground is not None else ("class 2" if ground_only else "none"),
               "cells_covered_pct": round(100 * float(np.isfinite(z).mean()), 2),
               **({"objects": standing} if objects else {})}


# What counts as person-sized, in metres: something this tall, on a footprint no wider than this.
PERSON_HEIGHT_M = (0.8, 2.4)
PERSON_WIDTH_M = 1.5
STANDS_M = 0.3               # a cell holds part of an object if something in it is this far above the ground


def standing_objects(above: Grid, cell_m: float, height: tuple[float, float] = PERSON_HEIGHT_M,
                     width: float = PERSON_WIDTH_M) -> list[tuple[float, float, float, float]]:
    """Person-sized things standing on the ground, from the height of the highest point in each cell above the
    ground surface (metres). Returns (x, y, radius, height) for each, in metres from the grid's corner.

    Cells with something standing in them are grouped into objects (cells that touch, corners included, are one
    object). An object is person-sized if its footprint fits in a `width` square and its top is within `height`.
    A machine is too wide, brush and stakes too low, a pole too tall. This is shape only: a barrel on end is
    person-sized, and a person leaning on a machine is part of the machine. What it finds is a place to keep
    clear of and to look at, not an identification.
    """
    up = np.isfinite(above) & (above >= STANDS_M)
    out = []
    ny, nx = up.shape
    for j0, i0 in zip(*np.nonzero(up), strict=True):
        if not up[j0, i0]:
            continue                                     # already taken into an earlier object
        # Flood fill from this cell: collect every touching cell that holds part of an object.
        up[j0, i0] = False
        todo, cells = [(int(j0), int(i0))], []
        while todo:
            j, i = todo.pop()
            cells.append((j, i))
            for a in range(max(j - 1, 0), min(j + 2, ny)):
                for b in range(max(i - 1, 0), min(i + 2, nx)):
                    if up[a, b]:
                        up[a, b] = False
                        todo.append((a, b))
        js, is_ = np.array(cells).T
        top = float(above[js, is_].max())
        wide = (max(np.ptp(js), np.ptp(is_)) + 1) * cell_m
        if wide <= max(width, cell_m) and height[0] <= top <= height[1]:
            # The centre of the cells' bounding box, and a radius that reaches its corners.
            cx, cy = (is_.min() + is_.max() + 1) / 2 * cell_m, (js.min() + js.max() + 1) / 2 * cell_m
            r = math.hypot(np.ptp(is_) + 1, np.ptp(js) + 1) / 2 * cell_m
            out.append((float(cx), float(cy), float(r), top))
    return out


def _morph(z: Grid, w: int, op) -> Grid:
    """Square-window grey erosion (op=np.min) or dilation (op=np.max); edges padded with the edge value."""
    from numpy.lib.stride_tricks import sliding_window_view as win
    p = np.pad(z, w // 2, mode="edge")
    return op(win(op(win(p, w, axis=1), axis=-1), w, axis=0), axis=-1)


FILTER_ROUNDS = 4            # most rounds of the ground filter; one is enough unless the scan holds a wide object


def ground_surface(zmin: Grid, cell_m: float, max_window_m: float = 10.0, slope: float = 0.3,
                   dh0: float = 0.1, dh_max: float = 1.5, dh_final: float = 0.15) -> Grid:
    """Progressive morphological filter (Zhang et al., 2003) on a lowest-point grid (metres expected).

    Openings with growing windows flag cells that stand more than the slope-scaled threshold above the
    opened surface (machines, people, brush). Ground cells keep their measured height; flagged cells and
    holes are filled from the surrounding ground, so the surface is not biased low on slopes.
    `dh_final` is used by `grid_cloud` for the point pass.

    One round misses the low parts of a wide object (a dozer's blade and tracks under its cab): beside the cab
    they look like part of something wide, which only the widest window, with the largest threshold, can remove.
    So around every wide object the filter is repeated on its own result (at most `FILTER_ROUNDS` rounds, until
    nothing new is found): with the cab gone, the low parts stand alone and the small windows take them.
    Elsewhere one round is final, so ground is not worn down by the repeats.

    `slope` trades banks against low objects: at 0.3 (3:1 banks) a wide object must stand about 1.3 m
    above ground to be removed at 1 m cells, which catches machines and people seen from above; on flat
    sites a lower slope also catches low objects. Crests sharper than `slope` are taken for objects.
    Returns the ground surface."""
    z = fill_gaps(zmin, max(zmin.shape))
    if np.isnan(z).all():
        return z
    unseen = np.isnan(zmin)
    obj = np.zeros(z.shape, bool)                # every cell found to hold an object
    wide = np.zeros(z.shape, bool)               # those found by a window wider than the smallest
    near = np.ones(z.shape, bool)                # where to look: everywhere at first, then around the wide objects
    for _ in range(FILTER_ROUNDS):
        found = np.zeros(z.shape, bool)
        # Windows of 3, 5, 9, 17, ... cells. An *opening* (erosion then dilation) removes anything narrower than the
        # window; what the opening lowered by more than the threshold was an object standing on the ground. The
        # threshold grows with the window so that real slopes survive.
        cur, w_prev, k, w = z, 3, 1, 3
        while (2 * k - 1) * cell_m <= max_window_m:
            w = 2 * k + 1
            opened = _morph(_morph(cur, w, np.min), w, np.max)
            hit = (cur - opened > min(dh_max, dh0 + slope * (w - w_prev) * cell_m)) & ~obj & near
            found |= hit
            if w > 3:
                wide |= hit
            cur, w_prev, k = opened, w, k * 2
        if not found.any():
            break
        obj |= found
        z = relax(fill_gaps(np.where(obj | unseen, np.nan, zmin), max(zmin.shape)), obj | unseen)
        if not wide.any():
            break
        near = _morph(wide, w, np.max)           # within half the widest window of a wide object
    return z


def relax(z: Grid, free: NDArray[np.bool_], tol: float = 1e-4, max_iter: int = 2000) -> Grid:
    """Harmonic infill: iterate the `free` cells toward the mean of their four neighbours (fixed cells
    hold), so filled patches follow planes and ramps exactly instead of sagging toward the rim."""
    if not free.any():
        return z
    z = z.copy()
    for _ in range(max_iter):
        p = np.pad(z, 1, mode="edge")
        new = 0.25 * (p[1:-1, :-2] + p[1:-1, 2:] + p[:-2, 1:-1] + p[2:, 1:-1])
        d = np.abs(new[free] - z[free]).max()
        z[free] = new[free]
        if d < tol:
            break
    return z


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
