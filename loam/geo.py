"""Placing a site's local grid on the earth.

Loam programs use site-local coordinates. Field tools want WGS84 latitude and
longitude (AgOpenGPS, OpenGrade3D, NMEA receivers) or a projected grid
(UTM, State Plane; lidar clouds and survey exports). A `Frame` ties the two
together with:

* the WGS84 position of site point (0, 0), for lat/lon tools;
* the projected easting/northing of site point (0, 0), for projected data;
* the bearing of the site's +y axis, degrees clockwise from north (grid
  north for projected data, true north for WGS84);
* a vertical offset, added to incoming heights after unit conversion.

Latitude/longitude use the same local tangent plane as AgOpenGPS
(meridian and parallel arc lengths at the origin), so an AgOpenGPS field whose
origin is site (0, 0) has exactly the site's rotated metre grid. Over a few
kilometres this plane is accurate to millimetres, which is far inside RTK
error; it is not a substitute for a proper projection over large sites.

With `crs` set (any CRS pyproj understands, e.g. "EPSG:6355" for Alabama East
US-ft, or a compound "EPSG:32616+5703" for UTM with NAVD88 heights), the
projected coordinates are in that CRS, WGS84 goes through a real projection,
and `reproject` brings data from other CRSs (including heights, when both are
compound) into it. pyproj (MIT) is needed only then.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from numpy.typing import ArrayLike, NDArray

F = NDArray[np.float64]


def m_per_deg_lat(lat: float) -> float:
    r = math.radians(lat)
    return 111132.92 - 559.82 * math.cos(2 * r) + 1.175 * math.cos(4 * r) - 0.0023 * math.cos(6 * r)


def m_per_deg_lon(lat: ArrayLike) -> F:
    r = np.radians(np.asarray(lat, float))
    return 111412.84 * np.cos(r) - 93.5 * np.cos(3 * r) + 0.118 * np.cos(5 * r)


@lru_cache(maxsize=16)
def _transformer(src: str, dst: str):
    try:
        from pyproj import Transformer
    except ImportError as e:
        raise ValueError("coordinate reference systems need pyproj: pip install pyproj") from e
    return Transformer.from_crs(src, dst, always_xy=True)


@dataclass(frozen=True)
class Frame:
    unit_m: float                    # metres per site unit
    rotation_deg: float = 0.0        # bearing of site +y, clockwise from north
    lat0: float | None = None        # WGS84 of site (0, 0)
    lon0: float | None = None
    e0: float = 0.0                  # projected coordinates of site (0, 0), in world units
    n0: float = 0.0
    world_m: float = 1.0             # metres per world unit of projected data and its heights
    dz: float = 0.0                  # site units added to incoming heights
    crs: str | None = None           # CRS of the projected coordinates (needs pyproj)

    # -- rotation between site axes and east/north metres -------------------
    def local_to_en(self, x: ArrayLike, y: ArrayLike) -> tuple[F, F]:
        """Site coordinates to metres east and north of site (0, 0)."""
        t = math.radians(self.rotation_deg)
        x, y = np.asarray(x, float) * self.unit_m, np.asarray(y, float) * self.unit_m
        return x * math.cos(t) + y * math.sin(t), -x * math.sin(t) + y * math.cos(t)

    def en_to_local(self, e: ArrayLike, n: ArrayLike) -> tuple[F, F]:
        t = math.radians(self.rotation_deg)
        e, n = np.asarray(e, float), np.asarray(n, float)
        return (e * math.cos(t) - n * math.sin(t)) / self.unit_m, (e * math.sin(t) + n * math.cos(t)) / self.unit_m

    def heading_rad(self, dx: float, dy: float) -> float:
        """Bearing in radians (0 = north, clockwise, in [0, 2π)) of a site-axis direction."""
        e, n = self.local_to_en(dx, dy)
        return math.atan2(float(e), float(n)) % (2 * math.pi)

    # -- WGS84 ------------------------------------------------------------------
    def wgs84_origin(self) -> tuple[float, float]:
        if self.lat0 is None or self.lon0 is None:
            raise ValueError("this needs the site origin in WGS84 (--origin LAT,LON)")
        return self.lat0, self.lon0

    def local_to_wgs84(self, x: ArrayLike, y: ArrayLike) -> tuple[F, F]:
        if self.crs:
            lon, lat = _transformer(self.crs, "EPSG:4326").transform(*self.local_to_projected(x, y))
            return np.asarray(lat, float), np.asarray(lon, float)
        lat0, lon0 = self.wgs84_origin()
        e, n = self.local_to_en(x, y)
        lat = lat0 + n / m_per_deg_lat(lat0)
        return lat, lon0 + e / m_per_deg_lon(lat)

    def wgs84_to_local(self, lat: ArrayLike, lon: ArrayLike) -> tuple[F, F]:
        if self.crs:
            e, n = _transformer("EPSG:4326", self.crs).transform(np.asarray(lon, float), np.asarray(lat, float))
            return self.projected_to_local(e, n)
        lat0, lon0 = self.wgs84_origin()
        lat, lon = np.asarray(lat, float), np.asarray(lon, float)
        n = (lat - lat0) * m_per_deg_lat(lat0)
        e = (lon - lon0) * m_per_deg_lon(lat)
        return self.en_to_local(e, n)

    # -- projected grids and heights ---------------------------------------------
    def projected_to_local(self, e: ArrayLike, n: ArrayLike) -> tuple[F, F]:
        return self.en_to_local((np.asarray(e, float) - self.e0) * self.world_m,
                                (np.asarray(n, float) - self.n0) * self.world_m)

    def local_to_projected(self, x: ArrayLike, y: ArrayLike) -> tuple[F, F]:
        e, n = self.local_to_en(x, y)
        return self.e0 + e / self.world_m, self.n0 + n / self.world_m

    def height_to_local(self, z: ArrayLike) -> F:
        return np.asarray(z, float) * self.world_m / self.unit_m + self.dz

    def height_to_world(self, z: ArrayLike) -> F:
        return (np.asarray(z, float) - self.dz) * self.unit_m / self.world_m

    def reproject(self, e: ArrayLike, n: ArrayLike, z: ArrayLike, src: str | None) -> tuple[F, F, F]:
        """Coordinates in CRS `src` to this frame's CRS (no-op when either is unset or they match)."""
        e, n, z = np.asarray(e, float), np.asarray(n, float), np.asarray(z, float)
        if not src or not self.crs or src == self.crs:
            return e, n, z
        e2, n2, z2 = _transformer(src, self.crs).transform(e, n, z)
        return np.asarray(e2, float), np.asarray(n2, float), np.asarray(z2, float)
