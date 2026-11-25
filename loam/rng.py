"""Portable, specified random numbers for the reference simulator.

NumPy's generators are fast but their streams are not promised to be stable
across releases, and no other implementation (e.g. the Assembly port) could
reproduce them. Loam uses a counter-based generator instead: every draw is a
pure function of (seed, counter), so it is bit-for-bit reproducible in any
language and vectorizes in NumPy.

    u64(seed, c)   = splitmix64(seed * 0x9E3779B97F4A7C15 + c)          (mod 2**64)
    uniform        = (u64 >> 11) * 2**-53                                 in [0, 1)
    normal         = sum of 12 consecutive uniforms - 6   (Irwin-Hall; mean 0, variance 1, |x| <= 6)

The Irwin-Hall normal needs no transcendental functions and its sum is taken in
a fixed order, so results are exact across platforms. Its tails are truncated at
6 sigma, which is immaterial for blade and survey noise.
"""
from __future__ import annotations

import numpy as np

GOLDEN = np.uint64(0x9E3779B97F4A7C15)
_M1 = np.uint64(0xBF58476D1CE4E5B9)
_M2 = np.uint64(0x94D049BB133111EB)


def splitmix64(x: np.ndarray) -> np.ndarray:
    x = x + GOLDEN
    x = (x ^ (x >> np.uint64(30))) * _M1
    x = (x ^ (x >> np.uint64(27))) * _M2
    return x ^ (x >> np.uint64(31))


class CounterRNG:
    def __init__(self, seed: int):
        with np.errstate(over="ignore"):
            self.key = np.uint64(seed & 0xFFFFFFFFFFFFFFFF) * GOLDEN
        self.counter = 0

    def _u64(self, n: int, start: int | None = None, step: int = 1) -> np.ndarray:
        """Raw draws for counters start, start+step, ... (n of them). Advances the counter only when
        `start` is None."""
        first = self.counter if start is None else start
        c = np.uint64(first) + np.arange(n, dtype=np.uint64) * np.uint64(step)
        if start is None:
            self.counter += n
        with np.errstate(over="ignore"):
            return splitmix64(self.key + c)

    def uniform(self, n: int) -> np.ndarray:
        return _to_unit(self._u64(n))

    def normal(self, loc: float, scale: float, shape: tuple[int, ...] | int) -> np.ndarray:
        """Normal draw i uses uniforms 12*i .. 12*i+11 of the stream, summed in that order."""
        n = int(np.prod(shape))
        base = self.counter
        s = _to_unit(self._u64(n, base, 12))
        for k in range(1, 12):  # column by column: same stream, 1/12 of the temporary memory
            s += _to_unit(self._u64(n, base + k, 12))
        self.counter += 12 * n
        s -= 6.0
        s *= scale
        s += loc
        return s.reshape(shape)


def _to_unit(u: np.ndarray) -> np.ndarray:
    return (u >> np.uint64(11)).astype(np.float64) * 2.0**-53
