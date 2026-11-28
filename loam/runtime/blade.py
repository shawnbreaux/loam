"""A blade held to the next-pass surface, on a machine with nobody aboard.

A grade-control system (Cat Grade with 3D, OpenGrade3D) holds a blade to a surface while a person drives. A
machine that drives itself has nobody to load that surface, so here the bridge does both: it steers the vehicle
through its own protocol and moves the blade's actuator so that the cutting edge follows the surface the next
pass should leave (`loam.target.next_target`: the latest survey, cut by at most the order's limit, filled by at
most one lift, never past design).

What the machine must offer is small, and any protocol can carry it:

* its height, as its positioning reports it (`altitude`, site units);
* one actuator output that raises and lowers the blade (`implement(pwm)`), a servo pulse width in microseconds as
  a hobby servo, a linear servo or a valve driver takes it.

Two ways to mount the height sensor, and one number tells them apart:

* **on the blade** (`travel` 0): the antenna rides up and down with the cutting edge, a fixed `height` above
  it. The edge is then measured directly, and the actuator is stepped until it is where it should be. This is how
  grade control is normally built, and it is the accurate way.
* **on the machine's body** (`travel` above 0): the edge is `height` below the antenna with the blade fully
  down, and rises by `travel` over the actuator's stroke. The edge is then worked out from the actuator's
  position, which is only as good as that geometry and the ground under the machine.

The blade works only inside the order's window, during an order that moves earth, while the machine is allowed to
move. An order that gives a `path` to follow narrows that to its `swath`: the blade is down only within half the
swath of the path, so the drive to the path's start cuts nothing. Everywhere else it is carried fully raised. A
machine that loses its height carries the blade too: the ground is never cut on a guess.

One actuator sets one height, so the edge is level across its width. Given the blade's `width`, the edge is
held at the highest point wanted anywhere under it, and where part of it is over ground that is not to be
graded, no lower than that ground. It therefore never cuts below the surface at either end, and it cannot
shape a slope across its own width: the side of a swale needs a blade that tilts, which this does not drive.

This is not a safety function. It moves one actuator; the stops are the bridge's and the vehicle's own.
"""
from __future__ import annotations

import math
from typing import Protocol

import numpy as np

from .. import line, wire
from ..grid import SiteGrid, points_in_polygon
from ..target import next_target
from ..units import FT

# Orders during which the blade is in the ground. A roller or a truck has no blade to hold.
BLADE_OPS = ("cut", "fill", "spread", "finish")
RATE = 0.5                   # most of the actuator's stroke moved in one second
GAIN_PER_FT = 2.0            # sensor on the blade: strokes per second, per foot the edge is off
DEADBAND_FT = 0.02           # an edge this close to where it should be is left alone (a quarter of an inch)
MIN_STEP_US = 2              # a new pulse width is sent only if it differs by this much from the last one sent


class Grader(Protocol):
    """What a vehicle adapter adds to be given a blade to hold: its height and one actuator."""
    #   altitude        the height to grade by, site units; None when the vehicle has none it can vouch for
    #   implement(pwm)  set the blade actuator's pulse width, microseconds
    @property
    def altitude(self) -> float | None: ...
    def implement(self, pwm: int) -> None: ...


class Surface:
    """The ground a blade works on, as its controller knows it: the design, the ground as last measured, and the
    surface the next pass should leave. It answers what a cutting edge at some place should be held to."""

    def __init__(self, grid: SiteGrid, width: float = 0.0):
        """`width`: the length of the cutting edge, across the direction of travel, in site units; 0 treats the
        edge as the one point at its middle."""
        if width < 0:
            raise ValueError("blade: width cannot be negative")
        self.grid, self.width = grid, width
        self.design, self.ground = grid.design_and_existing()
        self.target = next_target(grid, self.design, self.ground)[0]
        self.surveys = 0
        # Cells the edge has cut since their target was worked out. Their target stands until the edge has left
        # them: a pass cuts what it was given to cut, not as deep as it can chase its own cut.
        self.fresh = np.zeros(grid.shape, bool)

    def survey(self, msg: dict) -> None:
        """Take in a terrain message: measured ground replaces what was known cell by cell, and the surface the
        next pass should leave is worked out again. A survey on another grid is not this site's and is ignored."""
        shape = tuple(msg.get("shape") or ())
        if shape != self.grid.shape or not msg.get("data"):
            return
        z = wire.decode_heights(msg["data"], shape).astype(float)
        self.ground = np.where(np.isfinite(z), z, self.ground)
        self.target = next_target(self.grid, self.design, self.ground)[0]
        self.surveys += 1
        self.fresh[:] = False

    def under(self, x: float, y: float, heading: float) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """What lies under a cutting edge whose middle is at (x, y), heading `heading`: how far to the left of
        the middle each sample of the edge is, the cell each is over (as flat indices into the grid) and the
        height wanted there. None if the middle is off the grid or over ground with nothing to grade.

        The edge is sampled at its middle and, given a width, at points no more than a cell apart out to both
        ends. The height wanted is the surface the next pass should leave, or the ground as it is where nothing
        is to be graded: an edge is never taken below either. It is read between the cells' middles (bilinear),
        so a slope across the edge is a slope and not a staircase; next to ground of unknown height it is the
        cell's own value. Samples off the grid are left out.
        """
        half = math.ceil(self.width / (2 * self.grid.cell)) if self.width > 0 else 0
        across = np.arange(-half, half + 1, dtype=np.float64) * (self.width / (2 * half) if half else 0.0)
        px = np.asarray(x - across * math.sin(heading), np.float64)
        py = np.asarray(y + across * math.cos(heading), np.float64)
        i, j, ok = self.grid.cell_of(px, py)
        if not ok[half] or not math.isfinite(self.target[j[half], i[half]]):
            return None
        wanted = np.where(np.isfinite(self.target), self.target, self.ground)
        smooth = self.grid.sample(wanted, px[ok], py[ok])
        return across[ok], j[ok] * self.grid.nx + i[ok], np.where(np.isfinite(smooth), smooth, wanted[j[ok], i[ok]])

    def held(self, x: float, y: float, heading: float, slope: float = 0.0) -> float | None:
        """The lowest the middle of the edge may be held: with the edge rising `slope` for each unit to the left
        (0 for a level edge), no point of it below the height wanted under that point. None where there is
        nothing to grade."""
        found = self.under(x, y, heading)
        if found is None:
            return None
        across, _, want = found
        return float(np.nanmax(want - across * slope))

    def cut_to(self, x: float, y: float, heading: float, z: float, slope: float = 0.0) -> None:
        """Record that the edge has passed here with its middle at height `z`: the ground under it is no higher
        than the edge was. Ground the edge passed above is left as it was known, because a blade can only
        show what it cut, not what it spilled. The cells cut are marked for `refresh`."""
        found = self.under(x, y, heading)
        if found is None:
            return
        _, cells, _ = found
        # The edge's height over each cell's middle: how far that middle is to the left of the edge's own.
        j, i = np.divmod(cells, self.grid.nx)
        left = -(self.grid.xs[i] - x) * math.sin(heading) + (self.grid.ys[j] - y) * math.cos(heading)
        edge = z + left * slope
        ground = self.ground.ravel()
        lower = np.isfinite(ground[cells]) & (edge < ground[cells])
        if lower.any():
            ground[cells[lower]] = edge[lower]
            self.fresh.ravel()[cells[lower]] = True

    def refresh(self, x: float | None = None, y: float | None = None) -> None:
        """Work out the next-pass surface again for the cells the edge has cut and since left: those more than
        the edge's length (and two cells) from its middle at (x, y), or all of them if no place is given. So the
        next pass over a cell may cut deeper, and the pass in hand may not."""
        if not self.fresh.any():
            return
        left = self.fresh
        if x is not None and y is not None:
            far = max(self.width, 2 * self.grid.cell)
            left = left & (np.hypot(self.grid.xs[None, :] - x, self.grid.ys[:, None] - y) > far)
        if left.any():
            self.target = np.where(left, next_target(self.grid, self.design, self.ground)[0], self.target)
            self.fresh &= ~left


class Blade(Surface):
    """The blade of one machine: where its edge should be, and the actuator command that takes it there."""

    def __init__(self, grid: SiteGrid, height: float, pwm: tuple[int, int] = (1100, 1900), travel: float = 0.0,
                 reach: float = 0.0, rate: float = RATE, width: float = 0.0):
        """`height`: how far the reported altitude is above the cutting edge with the blade fully down (measure
        it with `loam mavlink --blade-zero`). `pwm`: the pulse widths for fully down and fully up. `travel`: how
        far the edge rises over the stroke when the height sensor is on the machine's body; 0 when it rides on
        the blade. `reach`: how far the edge is ahead of the position the vehicle reports (negative for a towed
        blade). `width`: the length of the cutting edge, across the direction of travel; 0 treats the edge as
        the one point at its middle. Lengths are in site units. `rate` is the most of the stroke moved per second.
        """
        if pwm[0] == pwm[1] or travel < 0 or rate <= 0:
            raise ValueError("blade: the two pulse widths must differ, travel cannot be negative, rate must be > 0")
        super().__init__(grid, width)
        self.height, self.pwm, self.travel, self.reach, self.rate = height, pwm, travel, reach, rate
        self.gain = GAIN_PER_FT * grid.u / FT            # per site unit
        self.deadband = DEADBAND_FT * FT / grid.u
        self.at = 1.0                    # where the actuator has been sent: 0 fully down, 1 fully up
        self.sent: int | None = None     # the last pulse width handed out
        self.t: float | None = None      # when `command` last ran

    def edge_point(self, pose: tuple[float, float, float]) -> tuple[float, float]:
        """Where the cutting edge is on the site: `reach` ahead of the vehicle's reported position."""
        x, y, heading = pose
        return x + self.reach * math.cos(heading), y + self.reach * math.sin(heading)

    def wanted(self, pose: tuple[float, float, float] | None, order: dict | None) -> float | None:
        """The elevation the cutting edge should be at now, or None if the blade is to be carried: no order, an
        order that moves no earth, an edge outside the order's window (or off the order's path by more than half
        its swath) or off the grid, or a place with nothing to grade. One actuator holds the edge level, so the
        height is the highest wanted anywhere under it (see `Surface.held`).
        """
        if pose is None or order is None or order.get("op") not in BLADE_OPS or not order.get("window"):
            return None
        bx, by = self.edge_point(pose)
        if not bool(points_in_polygon(order["window"], np.array([bx]), np.array([by]))[0]):
            return None
        if order.get("path") and order.get("swath") and len(order["path"]) > 1:
            off = line.project(order["path"], np.array([bx]), np.array([by]))[1]
            if abs(float(off[0])) > float(order["swath"]) / self.grid.u / 2:
                return None
        return self.held(bx, by, pose[2])

    def command(self, pose: tuple[float, float, float] | None, altitude: float | None, order: dict | None,
                now: float) -> int | None:
        """The pulse width to send to the actuator now, or None if it is already there.

        `order` is the order being worked, or None while the machine is not allowed to move. Call it every cycle:
        the actuator moves at most `rate` of its stroke per second, so the steps are small.
        """
        dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
        self.t = now
        want = self.wanted(pose, order)
        goal = 1.0                                       # carried
        if want is not None and altitude is not None and math.isfinite(altitude):
            edge = altitude - self.height + self.travel * self.at
            err = want - edge                            # above zero: the edge is too low, raise it
            if abs(err) <= self.deadband:
                goal = self.at
            elif self.travel > 0:
                goal = self.at + err / self.travel       # the geometry says how far to go
            else:
                goal = self.at + self.gain * err * dt    # measured at the edge: step toward it
        step = self.rate * dt
        self.at = min(max(self.at + min(max(goal - self.at, -step), step), 0.0), 1.0)
        pulse = round(self.pwm[0] + self.at * (self.pwm[1] - self.pwm[0]))
        # A step too small to matter is not sent, except the one that brings the actuator to an end of its stroke:
        # fully raised means the last microsecond too.
        small = abs(pulse - self.sent) < MIN_STEP_US if self.sent is not None else False
        if pulse == self.sent or (small and 0.0 < self.at < 1.0):
            return None
        self.sent = pulse
        return pulse
