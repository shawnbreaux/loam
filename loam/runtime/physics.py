"""Physics-driven machine motion (PyBullet), for `loam simulate-site --physics`.

Each machine process gets its own physics world: the site's existing ground as
a height field and the machine as a rigid body of its class's size and mass.
A pure-pursuit controller steers it along the order's path at no more than the
order's speed, gravity holds it to the ground, and its pose, heading, pitch and
roll come from the physics engine rather than from interpolation.

What it adds over the kinematic machine: slopes, inertia, turning limits and
tracking error. What it does not model: soil reacting to the blade (the world
process still moves the earth), track slip in loose material, or hydraulics.
"""
from __future__ import annotations

import itertools
import math

import numpy as np

try:
    import pybullet as pb
except ImportError as e:  # pragma: no cover
    raise SystemExit("--physics needs pybullet: pip install pybullet") from e

# Body size (m: length, width, height) and mass (kg) by machine class.
BODY = {"dozer": ((4.5, 2.6, 2.4), 9000.0), "roller": ((5.5, 2.3, 2.9), 11000.0),
        "excavator": ((5.0, 2.6, 2.8), 14000.0), "grader": ((8.0, 2.5, 3.2), 13000.0)}
# Pure pursuit steers at a point this far ahead on the path. Further ahead cuts corners; nearer weaves.
LOOKAHEAD_M = 3.0
MAX_YAW_RATE = 0.6          # rad/s, a small dozer counter-rotating


class Body:
    """One machine in its own PyBullet world: the site's ground as a height field, and the machine as a rigid body
    driven along its path.
    """
    def __init__(self, z: np.ndarray, x0: float, y0: float, cell: float, u: float, cls: str,
                 start: tuple[float, float]):
        """z: ground heights on the site grid (site units); cell, x0, y0 in site units; u = metres per unit."""
        self.u = u
        self.c = pb.connect(pb.DIRECT)
        pb.setGravity(0, 0, -9.81, physicsClientId=self.c)
        ny, nx = z.shape
        self.zmin = float(np.nanmin(z))
        # Heights in metres above the lowest point of the site. Cells with no data sit at that lowest point.
        hf = ((np.nan_to_num(z, nan=self.zmin) - self.zmin) * u).astype(np.float32)
        span = (float(hf.max()) if hf.size else 0.0)
        shape = pb.createCollisionShape(pb.GEOM_HEIGHTFIELD, meshScale=[cell * u, cell * u, 1.0],
                                        heightfieldData=hf.ravel(order="C").tolist(),
                                        numHeightfieldRows=nx, numHeightfieldColumns=ny,
                                        physicsClientId=self.c)
        # PyBullet centres a height field on its bounding box: shift it so site metres line up.
        self.ox, self.oy = x0 * u, y0 * u
        cx, cy = (nx - 1) * cell * u / 2, (ny - 1) * cell * u / 2
        self.ground = pb.createMultiBody(0, shape, basePosition=[self.ox + cell * u / 2 + cx,
                                                                 self.oy + cell * u / 2 + cy, span / 2],
                                         physicsClientId=self.c)
        pb.changeDynamics(self.ground, -1, lateralFriction=0.9, physicsClientId=self.c)
        (length, width, height), mass = BODY.get(cls, BODY["dozer"])
        # The undercarriage as six spheres (track rollers): it rides over the TIN's facet edges the way
        # tracks do, where a flat box would catch on every ridge.
        r = 0.45
        rollers = [[sx_ * (length / 2 - r), sy_ * (width / 2 - r), -height / 2 + r]
                   for sx_ in (-1, 0, 1) for sy_ in (-1, 1)]
        box = pb.createCollisionShapeArray([pb.GEOM_SPHERE] * 6, radii=[r] * 6, collisionFramePositions=rollers,
                                           physicsClientId=self.c)
        sx, sy = start[0] * u, start[1] * u
        self.h = height
        # Centre of mass low, as on a dozer (tracks and blade carry most of the weight). The body is driven by
        # setting its velocity, so its contact friction is kept low: friction only resists the drive and tips it.
        self.id = pb.createMultiBody(mass, box, basePosition=[sx, sy, self._ground_z(z, x0, y0, cell, start) + height],
                                     baseInertialFramePosition=[0, 0, -height * 0.35], physicsClientId=self.c)
        # Roll and pitch inertia raised 20x: wide tracks and a low, heavy undercarriage make a dozer far
        # harder to tip than a box of the same size, and the velocity drive below would otherwise rock it.
        ixx, iyy = mass * (width ** 2 + height ** 2) / 12, mass * (length ** 2 + height ** 2) / 12
        izz = mass * (length ** 2 + width ** 2) / 12
        pb.changeDynamics(self.id, -1, lateralFriction=0.15, linearDamping=0.4, angularDamping=0.95,
                          localInertiaDiagonal=[20 * ixx, 20 * iyy, izz], physicsClientId=self.c)
        for _ in range(240):                       # settle onto the ground
            pb.stepSimulation(physicsClientId=self.c)
        self.max_tilt = 0.0
        self.rolled = False                        # tipped past 45 degrees: out of service
        self.track_err: list[float] = []

    def _ground_z(self, z, x0, y0, cell, p) -> float:
        """The physics-world height (m) of the ground under site point `p`."""
        i = int(np.clip((p[0] - x0) / cell, 0, z.shape[1] - 1))
        j = int(np.clip((p[1] - y0) / cell, 0, z.shape[0] - 1))
        v = z[j, i]
        return float(((v if np.isfinite(v) else self.zmin) - self.zmin) * self.u)

    def pose(self) -> tuple[float, float, float, float, float]:
        """x, y (site units), heading (rad, CCW from +x), pitch, roll (rad)."""
        (x, y, _), q = pb.getBasePositionAndOrientation(self.id, physicsClientId=self.c)
        roll, pitch, yaw = pb.getEulerFromQuaternion(q)
        return x / self.u, y / self.u, yaw, pitch, roll

    def drive(self, target: tuple[float, float], speed_mps: float, seconds: float, step: float = 1 / 60) -> None:
        """Steer toward a site-unit target for `seconds` of physics time."""
        tx, ty = target[0] * self.u, target[1] * self.u
        n = max(1, round(seconds / step))
        pb.setTimeStep(step, physicsClientId=self.c)
        for _ in range(n):
            if self.rolled:
                return
            (x, y, _z), q = pb.getBasePositionAndOrientation(self.id, physicsClientId=self.c)
            roll, pitch, yaw = pb.getEulerFromQuaternion(q)
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            # The heading error, wrapped into -180 to +180 degrees so the machine turns the short way round.
            err = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
            yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, 2.0 * err))
            v = speed_mps * max(0.0, math.cos(err)) * min(1.0, dist / 1.0)     # slow to turn, stop at target
            lin, ang = pb.getBaseVelocity(self.id, physicsClientId=self.c)
            # Set the forward speed and turn rate, and leave vertical speed, pitch rate and roll rate to the physics:
            # that is what lets the ground tilt the machine.
            pb.resetBaseVelocity(self.id, [v * math.cos(yaw), v * math.sin(yaw), lin[2]], [ang[0], ang[1], yaw_rate],
                                 physicsClientId=self.c)
            pb.stepSimulation(physicsClientId=self.c)
            self.max_tilt = max(self.max_tilt, abs(roll), abs(pitch))
            self.rolled = self.max_tilt > math.radians(45)

    def close(self) -> None:
        """Release the physics world. Each one holds native memory until it is disconnected."""
        pb.disconnect(physicsClientId=self.c)


def follow(body: Body, path: list[tuple[float, float]], travelled: float, ds: float) -> tuple[float, float]:
    """Pure-pursuit target: the path point LOOKAHEAD ahead of the distance already travelled (site units)."""
    look = travelled + ds + LOOKAHEAD_M / body.u
    acc = 0.0
    for a, b in itertools.pairwise(path):
        leg = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + leg >= look and leg > 0:
            k = (look - acc) / leg
            return a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k
        acc += leg
    return path[-1]
