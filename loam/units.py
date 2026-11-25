"""Units and quantities for Loam.

Every quantity is stored internally in SI base units (metres, seconds,
cubic metres, metres per second, ratio for percent) together with its
dimension, so comparisons between e.g. `0.2 ft` and `6 cm` are exact and
mixing dimensions is a type error.
"""
from __future__ import annotations

from dataclasses import dataclass

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
COMPOUND_DIMS = {
    ("length", "time"): "rate",
    ("volume", "time"): "flow",
}


class UnitError(ValueError):
    pass


def lookup(unit: str) -> tuple[str, float]:
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
    value: float  # magnitude as written
    unit: str  # unit as written
    tol: bool = False  # written with a leading ± (tolerance band)

    @property
    def dim(self) -> str:
        return lookup(self.unit)[0]

    @property
    def si(self) -> float:
        return self.value * lookup(self.unit)[1]

    def to(self, unit: str) -> float:
        dim, f = lookup(unit)
        if dim != self.dim:
            raise UnitError(f"cannot convert {self} ({self.dim}) to {unit} ({dim})")
        return self.si / f

    def __str__(self) -> str:
        v = f"{self.value:g}"
        return f"{'±' if self.tol else ''}{v} {self.unit}"


def length_unit_si(unit: str) -> float:
    dim, f = lookup(unit)
    if dim != "length":
        raise UnitError(f"'{unit}' is not a length unit")
    return f
