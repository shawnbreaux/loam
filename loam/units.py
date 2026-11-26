"""Units and quantities for Loam.

Every quantity is stored internally in SI base units (metres, seconds,
cubic metres, metres per second, ratio for percent) together with its
dimension, so comparisons between e.g. `0.2 ft` and `6 cm` are exact and
mixing dimensions is a type error.
"""
from __future__ import annotations

from dataclasses import dataclass

# Metres in one international foot (exact by definition).
FT = 0.3048

# unit -> (dimension, factor to SI)
UNITS: dict[str, tuple[str, float]] = {
    # length
    "mm": ("length", 0.001),
    "cm": ("length", 0.01),
    "m": ("length", 1.0),
    "in": ("length", FT / 12),
    "ft": ("length", FT),
    "usft": ("length", 1200 / 3937),        # US survey foot (State Plane grids)
    "yd": ("length", 3 * FT),
    # area
    "ft2": ("area", FT**2),
    "yd2": ("area", (3 * FT) ** 2),
    "m2": ("area", 1.0),
    "ac": ("area", 4046.8564224),
    # volume
    "ft3": ("volume", FT**3),
    "yd3": ("volume", (3 * FT) ** 3),
    "m3": ("volume", 1.0),
    # time
    "s": ("time", 1.0),
    "min": ("time", 60.0),
    "h": ("time", 3600.0),
    # speed
    "mph": ("speed", 0.44704),
    "kmh": ("speed", 1 / 3.6),
    # ratio
    "%": ("ratio", 0.01),
    # angle
    "deg": ("angle", 3.141592653589793 / 180),
}

# compound "a/b" units are built on demand: length/time -> rate
# Only these two combinations are meaningful in Loam: a height per time (rain, in/h) and a volume per time
# (production, yd3/h).
COMPOUND_DIMS = {
    ("length", "time"): "rate",
    ("volume", "time"): "flow",
}


class UnitError(ValueError):
    """A unit that does not exist, or two quantities whose dimensions do not match (a length compared with a time)."""
    pass


def lookup(unit: str) -> tuple[str, float]:
    """The dimension of a unit and the factor that converts it to SI, e.g. lookup("ft") == ("length", 0.3048).

    Compound units written "a/b" (such as "in/h" or "yd3/h") are resolved from their two parts.
    """
    if unit in UNITS:
        return UNITS[unit]
    if "/" in unit:
        num, den = unit.split("/", 1)
        if num in UNITS and den in UNITS:
            dn, fn = UNITS[num]
            dd, fd = UNITS[den]
            dim = COMPOUND_DIMS.get((dn, dd))
            if dim:
                return dim, fn / fd
    raise UnitError(f"unknown unit '{unit}'")


@dataclass(frozen=True)
class Quantity:
    """A number with its unit, exactly as the author wrote it (`0.1 ft`, `±3 cm`, `2 %`).

    The written value and unit are kept so messages can quote the program; `si` gives the value in SI units for every
    calculation and comparison.
    """
    value: float  # magnitude as written
    unit: str  # unit as written
    tol: bool = False  # written with a leading ± (tolerance band)

    @property
    def dim(self) -> str:
        """The physical dimension: length, area, volume, time, speed, rate, flow, ratio or angle."""
        return lookup(self.unit)[0]

    @property
    def si(self) -> float:
        """The value in SI units (metres, seconds, cubic metres, metres per second; a ratio for percent)."""
        return self.value * lookup(self.unit)[1]

    def to(self, unit: str) -> float:
        """The value expressed in another unit of the same dimension; a different dimension is an error."""
        dim, f = lookup(unit)
        if dim != self.dim:
            raise UnitError(f"cannot convert {self} ({self.dim}) to {unit} ({dim})")
        return self.si / f

    def __str__(self) -> str:
        """As written in a program, e.g. `±0.1 ft`."""
        v = f"{self.value:g}"
        return f"{'±' if self.tol else ''}{v} {self.unit}"


def length_unit_si(unit: str) -> float:
    """Metres per one of this length unit (0.3048 for `ft`); an error for anything that is not a length."""
    dim, f = lookup(unit)
    if dim != "length":
        raise UnitError(f"'{unit}' is not a length unit")
    return f
