"""Survey points and GNSS positions: Emlid Reach and other RTK rovers.

* `read_points_csv` reads an Emlid Flow CSV export ("All columns", PNEZD or
  PENZD) or any survey CSV with easting/northing/elevation or
  latitude/longitude/elevation columns.
* `parse_gga` reads NMEA GGA sentences, which Reach receivers (and most RTK
  receivers) stream over TCP or serial; `nmea_states` turns them into Loam
  Wire `state` messages.
* `score` measures surveyed ground against the design: the field-test number.
* `idw` grids scattered survey points onto the site grid.
"""
from __future__ import annotations

import csv
import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from . import wire
from .geo import Frame
from .grid import Grid, SiteGrid, points_in_polygon

FIX_QUALITY = {0: "invalid", 1: "gps", 2: "dgps", 4: "rtk-fixed", 5: "rtk-float", 6: "dead-reckoning"}


@dataclass
class Survey:
    name: list[str]
    x: NDArray[np.float64]      # site coordinates
    y: NDArray[np.float64]
    z: NDArray[np.float64]


def _num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def read_points_csv(path: str, frame: Frame, layout: str = "PNEZD", fix_only: bool = True) -> Survey:
    """Read survey points into site coordinates.

    With a header row, columns are found by name: Easting/Northing/Elevation for
    projected data, Latitude/Longitude/Elevation when the frame has a WGS84
    origin. Rows whose solution status column says anything but FIX are dropped
    when `fix_only`. Without a header the layout is PNEZD or PENZD.
    """
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]
    if not rows:
        raise ValueError(f"{path}: no points")
    has_header = not all(_num(c) for c in rows[0][1:4])
    names: list[str] = []
    if has_header:
        col = {c.strip().lower(): k for k, c in enumerate(rows[0])}
        body = rows[1:]
        status = next((col[c] for c in ("solution status", "solution", "status") if c in col), None)
        if fix_only and status is not None:
            body = [r for r in body if r[status].strip().upper() in ("FIX", "FIXED", "RTK FIX", "")]
        get = lambda key: np.array([float(r[col[key]]) for r in body])  # noqa: E731
        if "elevation" not in col:
            raise ValueError(f"{path}: no Elevation column")
        if frame.lat0 is not None and {"latitude", "longitude"} <= col.keys():
            x, y = frame.wgs84_to_local(get("latitude"), get("longitude"))
        elif {"easting", "northing"} <= col.keys():
            x, y = frame.projected_to_local(get("easting"), get("northing"))
        else:
            raise ValueError(f"{path}: needs Easting/Northing columns, or Latitude/Longitude with --origin")
        z = frame.height_to_local(get("elevation"))
        names = [r[col["name"]] if "name" in col else str(k) for k, r in enumerate(body)]
    else:
        lay = layout.upper()
        if lay not in ("PNEZD", "PENZD"):
            raise ValueError("layout must be PNEZD or PENZD")
        a = np.array([[float(v) for v in r[1:4]] for r in rows])
        e, n = (a[:, 1], a[:, 0]) if lay == "PNEZD" else (a[:, 0], a[:, 1])
        x, y = frame.projected_to_local(e, n)
        z = frame.height_to_local(a[:, 2])
        names = [r[0] for r in rows]
    return Survey(names, np.asarray(x), np.asarray(y), np.asarray(z))


# -- NMEA ------------------------------------------------------------------------------
def _nmea_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def parse_gga(line: str) -> dict | None:
    """Parse one GGA sentence ($GPGGA, $GNGGA, ...). Returns None for other or corrupt sentences."""
    line = line.strip()
    if not _nmea_ok(line):
        return None
    f = line[1:line.index("*")].split(",")
    if not f[0].endswith("GGA") or len(f) < 12 or not f[2] or not f[4]:
        return None
    lat = int(f[2][:2]) + float(f[2][2:]) / 60
    lon = int(f[4][:3]) + float(f[4][3:]) / 60
    q = int(f[6] or 0)
    return {"time": f[1], "lat": -lat if f[3] == "S" else lat, "lon": -lon if f[5] == "W" else lon,
            "quality": q, "fix": FIX_QUALITY.get(q, str(q)), "sats": int(f[7] or 0),
            "hdop": float(f[8] or "nan"), "alt": float(f[9] or "nan"),
            "age": float(f[13]) if len(f) > 13 and f[13] else None}


def nmea_states(lines: Iterable[str], frame: Frame, site: str, machine: str,
                allow_float: bool = False, min_move: float = 0.05) -> Iterator[dict]:
    """Turn a GGA stream into Loam Wire `state` messages.

    Heading (radians, counter-clockwise from the site +x axis) comes from
    successive fixes once the antenna has moved `min_move` metres. Anything but
    an RTK fixed solution (RTK float too, unless `allow_float`) is reported with
    safetyState "degraded" and an error, so the controller holds the machine
    rather than trusting the position.
    """
    seq, heading, last = 0, 0.0, None
    for line in lines:
        g = parse_gga(line)
        if g is None:
            continue
        x, y = (float(v) for v in frame.wgs84_to_local(g["lat"], g["lon"]))
        if last is not None and math.hypot(x - last[0], y - last[1]) * frame.unit_m >= min_move:
            heading = math.atan2(y - last[1], x - last[0])
            last = (x, y)
        elif last is None:
            last = (x, y)
        ok = g["quality"] == 4 or (allow_float and g["quality"] == 5)
        seq += 1
        msg = wire.state(site, machine, seq, None, round(x, 4), round(y, 4), round(heading, 5), False, 0.0,
                         safety="ok" if ok else "degraded",
                         errors=[] if ok else [{"code": "gnssQuality", "detail": g["fix"]}])
        msg["pose"]["z"] = round(float(frame.height_to_local(g["alt"])), 4)
        msg["gnss"] = {"fix": g["fix"], "sats": g["sats"], "hdop": g["hdop"], "age": g["age"]}
        yield msg


# -- gridding and scoring ------------------------------------------------------------------
def idw(grid: SiteGrid, x: Grid, y: Grid, z: Grid, radius: float, power: float = 2.0,
        max_elems: int = 2_000_000) -> Grid:
    """Inverse-distance-weighted heights at cell centres from points within `radius` site units.

    Works in bands of rows so the cell-by-point distance block never exceeds `max_elems` floats.
    """
    out = np.full(grid.shape, np.nan)
    x, y, z = np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)
    rows_per_chunk = max(1, max_elems // max(1, grid.nx * len(x)))
    for j0 in range(0, grid.ny, rows_per_chunk):
        ys = grid.ys[j0:j0 + rows_per_chunk]
        near = np.abs(y[:, None] - ys[None, :]).min(1) <= radius
        if not near.any():
            continue
        px, py, pz = x[near], y[near], z[near]
        gx, gy = np.meshgrid(grid.xs, ys)
        d2 = (gx[..., None] - px) ** 2 + (gy[..., None] - py) ** 2
        w = np.where(d2 <= radius * radius, 1.0 / np.maximum(d2, 1e-12) ** (power / 2), 0.0)
        ws = w.sum(-1)
        with np.errstate(invalid="ignore"):
            out[j0:j0 + len(ys)] = np.where(ws > 0, (w * pz).sum(-1) / ws, np.nan)
    return out


def score(grid: SiteGrid, design: Grid, s: Survey, polygon: list, tolerance: float,
          n_boot: int = 2000, seed: int = 1) -> dict:
    """Surveyed ground against the design inside a polygon, with a bootstrap 95 % interval."""
    keep = points_in_polygon(polygon, s.x, s.y)
    err = s.z[keep] - grid.sample(design, s.x[keep], s.y[keep])
    err = err[np.isfinite(err)]
    if err.size == 0:
        raise ValueError("no surveyed points fall inside the zone")
    ok = np.abs(err) <= tolerance * (1 + 1e-9)
    rng = np.random.default_rng(seed)
    boot = np.empty(n_boot)
    for b0 in range(0, n_boot, 256):                 # bounded memory: 256 resamples at a time
        k = min(256, n_boot - b0)
        boot[b0:b0 + k] = ok[rng.integers(0, ok.size, (k, ok.size))].mean(1)
    lo, hi = np.percentile(boot, [2.5, 97.5])
    return {"points": int(err.size), "within_tolerance_pct": round(100 * ok.mean(), 2),
            "ci95_pct": [round(100 * lo, 2), round(100 * hi, 2)], "mean": round(float(err.mean()), 4),
            "p95_abs": round(float(np.percentile(np.abs(err), 95)), 4),
            "max_abs": round(float(np.abs(err).max()), 4)}
