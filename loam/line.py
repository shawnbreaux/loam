"""Lines on the site plan: how far along a line a point is, and how far to its side.

A task that runs `along` a line (a swale, a ditch, the edge of a road) is planned in the line's own coordinates.
The *station* of a point is the distance along the line to the point of the line nearest to it. Its *offset* is
its distance from the line, positive to the left of the direction the line is drawn in. A pass of such a task is
a lane: a strip of ground one tool-width wide at a fixed offset, between two stations.

A line is a list of (x, y) points joined by straight pieces. Lengths are in whatever unit the points are in.
"""
from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

Points = Sequence[Sequence[float]]

# A corner sharper than this is not mitred when a line is moved sideways. The mitre point of two pieces lies
# 1 / cos(half the turn) offsets from the corner; at a turn of 120 degrees that is twice the offset, and beyond
# it the point runs away. Sharper corners are cut straight across instead.
MITRE_LIMIT = 0.5            # 1 + cos(turn), which is 0.5 at 120 degrees


def _line(pts: Points) -> tuple[np.ndarray, np.ndarray]:
    """The line as an (n, 2) array with repeated points removed, and the station of each of its points."""
    p = np.asarray(pts, float).reshape(-1, 2)
    if len(p):
        p = p[np.concatenate(([True], np.hypot(*np.diff(p, axis=0).T) > 0))]
    if len(p) < 2:
        raise ValueError("a line needs at least two different points")
    return p, np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))))


def length(pts: Points) -> float:
    """The length of the line."""
    return float(_line(pts)[1][-1])


def project(pts: Points, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The station and the signed offset of each point (x, y), as arrays of the same shape as `x`.

    The nearest point of the line is the foot of a perpendicular to one of its pieces, or one of its corners.
    Past either end of the line the nearest point is that end, so the station stops there and the offset is the
    distance to the end. The offset is positive to the left of the line's direction.
    """
    p, st = _line(pts)
    x, y = np.asarray(x, float), np.asarray(y, float)
    best = np.full(x.shape, np.inf)
    s, off = np.zeros(x.shape), np.zeros(x.shape)
    for k in range(len(p) - 1):
        ex, ey = p[k + 1] - p[k]
        n = st[k + 1] - st[k]
        t = np.clip(((x - p[k, 0]) * ex + (y - p[k, 1]) * ey) / (n * n), 0.0, 1.0)
        dx, dy = x - p[k, 0] - t * ex, y - p[k, 1] - t * ey
        d2 = dx * dx + dy * dy
        near = d2 < best
        best = np.where(near, d2, best)
        s = np.where(near, st[k] + t * n, s)
        off = np.where(near, np.copysign(np.sqrt(d2), ex * dy - ey * dx), off)      # the cross product's sign
    return s, off


def piece(pts: Points, s0: float, s1: float, offset: float = 0.0) -> list[tuple[float, float]]:
    """The part of the line from station `s0` to station `s1`, moved `offset` to its left, as a list of points in
    that order (so backwards along the line when `s1` is the smaller).

    Straight pieces stay straight when moved sideways, so the result has one point per corner between the two
    stations and one at each end. Corners are mitred: the moved pieces are extended or trimmed to where they
    meet. A corner sharper than 120 degrees is cut straight across. An offset larger than the radius of a bend,
    on its inside, has no sound answer and is the caller's to avoid.
    """
    p, st = _line(pts)
    lo, hi = sorted((min(max(s0, 0.0), st[-1]), min(max(s1, 0.0), st[-1])))
    e = np.diff(p, axis=0) / np.diff(st)[:, None]              # the unit direction of each piece
    n = np.column_stack([-e[:, 1], e[:, 0]])                   # and its left-hand normal
    k0 = min(max(int(np.searchsorted(st, lo, "right")) - 1, 0), len(e) - 1)
    k1 = min(max(int(np.searchsorted(st, hi, "left")) - 1, k0), len(e) - 1)
    out = [p[k0] + e[k0] * (lo - st[k0]) + offset * n[k0]]
    for k in range(k0 + 1, k1 + 1):                            # the corners in between
        c = 1.0 + float(n[k - 1] @ n[k])
        out += [p[k] + offset * (n[k - 1] + n[k]) / c] if c > MITRE_LIMIT or not offset else [
            p[k] + offset * n[k - 1], p[k] + offset * n[k]]
    out.append(p[k1] + e[k1] * (hi - st[k1]) + offset * n[k1])
    path = [(float(q[0]), float(q[1])) for q in out]
    return path[::-1] if s1 < s0 else path


def densify(pts: Points, step: float) -> np.ndarray:
    """The line with points added so that no two neighbours are more than `step` apart, as an (n, 2) array.
    Every point of the line is kept, and each piece is divided evenly."""
    p, st = _line(pts)
    out = [p[:1]]
    for k in range(len(p) - 1):
        m = max(1, math.ceil((st[k + 1] - st[k]) / step - 1e-9))
        out.append(p[k] + (p[k + 1] - p[k]) * (np.arange(1, m + 1) / m)[:, None])
    return np.concatenate(out)
