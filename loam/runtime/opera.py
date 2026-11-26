"""OPERA machines (PWRI's Open Platform for Earthwork with Robotics Autonomy, ROS 2) as Loam machines.

`loam opera` runs one bridge per machine inside a ROS 2 environment. Its safety
loop is the same as for MAVLink and VDA 5050 (see `bridge.py`). It drives two
kinds of OPERA machine:

* Carriers: the IC120 crawler dump, or anything else that runs Nav2. An order's
  path becomes a `navigate_through_poses` goal on `/<ns>/navigate_through_poses`
  in the `map` frame, with the order's speed on `/<ns>/speed_limit`. Pause
  cancels the goal and resume sends the poses that were left. With `--dump`, the
  vessel is raised and lowered (com3_msgs `SetDumpAngle`) at the deposit. Odometry
  comes from `/<ns>/odom`.
* Excavators: the ZX200. The bucket tip follows the order's target surface: the
  latest Loam terrain survey plus the order's planned change per cell, never
  below design. It does this in radial strokes across the part of the window in
  reach. Joint angles come from the machine's own URDF (`/<ns>/robot_description`
  or `--urdf`). The trajectory goes to the arm's ros2_control
  JointTrajectoryController (`/<ns>/upper_arm_controller/follow_joint_trajectory`),
  timed to OPERA's joint speed limits.

The map frame is the site grid in metres (heights too), unless `--map-offset`
places the site in it. Everything here is Apache-2.0 or BSD apart from
com3_msgs, which carries no licence: it is never shipped, and the dump is used
only when the machine already has it.
"""
from __future__ import annotations

import itertools
import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any

import numpy as np

from .. import wire
from ..checker import Model
from ..grid import SiteGrid, polygon_mask
from .bridge import run_bridge
from .bus import Bus, tolerant

# Odometry or joint states older than this (seconds) count as a lost link.
STALE_S = 1.0
NAV_RETRIES = 3               # an aborted goal is retried this many times (2 s apart) before the order fails
UNAVAILABLE_S = 10.0          # a missing action server is reported after this, and still retried
OPERA_JOINT_SPEED = {"swing_joint": 1.1, "boom_joint": 0.45, "arm_joint": 1.1, "bucket_joint": 1.1}  # rad/s


# -- kinematics --------------------------------------------------------------------------------------
def _rpy(r: float, p: float, y: float) -> np.ndarray:
    """The rotation matrix for URDF roll, pitch, yaw angles (radians, about fixed x, y, z in that order)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(axis: np.ndarray, q: float) -> np.ndarray:
    """The rotation matrix for a turn of `q` radians about a unit axis (Rodrigues' formula)."""
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


@dataclass
class Joint:
    """One joint of a URDF.

    `kind` is the URDF joint type (fixed, revolute, continuous). `xyz` and `rot` place the joint in its parent
    link. `axis` is the unit axis it turns about. `lower` and `upper` are its limits in radians.
    """
    name: str
    kind: str
    xyz: np.ndarray
    rot: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class Chain:
    """A serial chain from a URDF, base link to tip link: forward and inverse kinematics."""

    def __init__(self, urdf: str, base: str = "base_link", tip: str = "bucket_end_link"):
        """Read the joints from the URDF text and walk from the tip link back to the base link. Raises ValueError if
        the two are not connected.
        """
        root = ET.fromstring(urdf)
        by_child: dict[str, tuple[str, Joint]] = {}

        def attr(el: ET.Element | None, key: str, default: str) -> str:
            """An XML attribute, or the default when the element or the attribute is missing."""
            return el.get(key, default) if el is not None else default
        for j in root.iter("joint"):
            o, lim = j.find("origin"), j.find("limit")
            xyz = [float(v) for v in attr(o, "xyz", "0 0 0").split()]
            rpy = [float(v) for v in attr(o, "rpy", "0 0 0").split()]
            ax = np.array([float(v) for v in attr(j.find("axis"), "xyz", "1 0 0").split()])
            kind = j.get("type", "fixed")
            lo, hi = (-math.pi, math.pi) if kind == "continuous" or lim is None else \
                (float(lim.get("lower", "-3.14159")), float(lim.get("upper", "3.14159")))
            by_child[attr(j.find("child"), "link", "")] = (attr(j.find("parent"), "link", ""), Joint(
                j.get("name", ""), kind, np.array(xyz), _rpy(*rpy), ax / np.linalg.norm(ax), lo, hi))
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no chain from {base} to {tip} in the URDF")
            parent, jt = by_child[link]
            chain.append(jt)
            link = parent
        self.joints = chain[::-1]
        self.moving = [j.name for j in self.joints if j.kind in ("revolute", "continuous")]

    def frames(self, q: dict[str, float]) -> list[np.ndarray]:
        """4x4 transforms of each joint's child frame, base frame first."""
        t = np.eye(4)
        out = [t]
        for j in self.joints:
            step = np.eye(4)
            step[:3, :3] = j.rot @ (_axis_rot(j.axis, q.get(j.name, 0.0)) if j.kind != "fixed" else np.eye(3))
            step[:3, 3] = j.xyz
            t = t @ step
            out.append(t)
        return out

    def tip(self, q: dict[str, float]) -> np.ndarray:
        """The tip's position (m) in the base frame for joint angles `q`."""
        return self.frames(q)[-1][:3, 3]

    def limits(self, name: str) -> tuple[float, float]:
        """A joint's lower and upper limits (rad)."""
        j = next(j for j in self.joints if j.name == name)
        return j.lower, j.upper


class ExcavatorIK:
    """Bucket tip position plus bucket attitude, solved for swing, boom, arm and bucket within their limits."""

    def __init__(self, chain: Chain, joints=("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
                 pivot: str = "bucket_joint"):
        """`joints` are the four the solver moves. `pivot` is the joint the bucket turns about, used to measure the
        bucket's attitude.
        """
        self.c, self.names = chain, list(joints)
        self.pivot_index = 1 + [j.name for j in chain.joints].index(pivot)

    def attitude(self, q: dict[str, float]) -> float:
        """Elevation angle (rad) of the bucket pivot to tip vector: about -pi/2 is straight down."""
        fr = self.c.frames(q)
        v = fr[-1][:3, 3] - fr[self.pivot_index][:3, 3]
        return math.atan2(v[2], math.hypot(v[0], v[1]))

    def _residual(self, q, target, att):
        """How far the solution is off: the tip's position error (3 values, m) and the attitude error (rad)."""
        return np.array([*(self.c.tip(q) - target), self.attitude(q) - att])

    def solve(self, target: np.ndarray, att: float, seed: dict[str, float], tol: float = 1e-4,
              iters: int = 60) -> dict[str, float] | None:
        """Joint angles that put the bucket tip at `target` (m, base frame) with attitude `att`, starting from `seed`;
        None if there are none within the joint limits.

        Damped least squares (Levenberg-Marquardt) with a numerical Jacobian. Angles are clipped to the limits at
        every step.
        """
        q = dict(seed)
        lim = [self.c.limits(n) for n in self.names]
        for _ in range(iters):
            r = self._residual(q, target, att)
            if np.linalg.norm(r[:3]) < tol and abs(r[3]) < 1e-3:
                return q
            jac = np.zeros((4, len(self.names)))
            for k, n in enumerate(self.names):
                dq = dict(q)
                dq[n] += 1e-6
                jac[:, k] = (self._residual(dq, target, att) - r) / 1e-6
            step = np.linalg.solve(jac.T @ jac + 1e-6 * np.eye(len(self.names)), -jac.T @ r)
            for k, n in enumerate(self.names):
                q[n] = float(np.clip(q[n] + step[k], lim[k][0], lim[k][1]))
        r = self._residual(q, target, att)
        return q if np.linalg.norm(r[:3]) < tol * 10 and abs(r[3]) < 1e-2 else None

    def solve_any(self, target: np.ndarray, seed: dict[str, float], prefer: float) -> dict[str, float] | None:
        """Try the preferred bucket attitude first, then others nearby, so the tip gets there if it can."""
        for d in (0.0, 0.15, -0.15, 0.3, -0.3, 0.5, -0.5, 0.8, -0.8):
            q = self.solve(target, prefer + d, seed)
            if q is not None:
                return q
        return None


# -- frames ------------------------------------------------------------------------------------------
class MapFrame:
    """Site coordinates (site units) to the OPERA map frame (metres): scale, rotate, offset."""

    def __init__(self, unit_m: float, offset: tuple[float, float, float] = (0, 0, 0), yaw_deg: float = 0.0):
        """`unit_m` is one site unit in metres. `offset` (m) and `yaw_deg` place the site's origin and x axis in the
        map frame.
        """
        self.u, self.off, self.yaw = unit_m, np.array(offset, float), math.radians(yaw_deg)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.R = np.array([[c, -s], [s, c]])

    def to_map(self, x: float, y: float, z: float = 0.0) -> np.ndarray:
        """A site point (site units) as a map point (m)."""
        xy = self.R @ np.array([x, y]) * self.u
        return np.array([xy[0], xy[1], z * self.u]) + self.off

    def to_site(self, p: np.ndarray) -> np.ndarray:
        """A map point (m) as a site point (site units)."""
        d = np.asarray(p, float) - self.off
        xy = self.R.T @ d[:2] / self.u
        return np.array([xy[0], xy[1], d[2] / self.u])


def _yaw(q) -> float:
    """The heading (rad) of a ROS quaternion: its rotation about the vertical axis."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# -- the ROS side --------------------------------------------------------------------------------------
class Ros:
    """One rclpy node, spun in a background thread."""

    def __init__(self, name: str):
        """Start ROS 2 if this process has not, create the node, and spin it in a background thread."""
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
        except ImportError as e:
            raise ValueError("OPERA machines need ROS 2 (rclpy): run inside a ROS 2 Humble/Jazzy environment") from e
        self.rclpy = rclpy
        if not rclpy.ok():
            try:                               # Loam handles SIGINT/SIGTERM: the vehicle is held on the way out
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):   # older rclpy
                rclpy.init()
        self.node = rclpy.create_node(name)
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.node)
        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        """The background thread: deliver subscriptions and action callbacks until `close`."""
        while self.running and self.rclpy.ok():    # short waits, so close() can always stop this thread
            try:
                self.ex.spin_once(timeout_sec=0.1)
            except Exception:                       # rclpy raises several types once its context is gone
                if self.rclpy.ok():
                    raise
                break

    def close(self) -> None:
        """Stop the thread, then destroy the node and shut ROS 2 down, in that order."""
        self.running = False
        self.thread.join(timeout=5)            # the thread is out of rcl before the context goes
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class _Odom:
    """A machine's odometry: its newest pose (site coordinates), height, speed and when it arrived."""
    def __init__(self, ros: Ros, topic: str, frame: MapFrame):
        """Subscribe to a nav_msgs/Odometry topic."""
        from nav_msgs.msg import Odometry
        self.frame, self.z, self.at, self.speed = frame, 0.0, 0.0, 0.0
        self.pose: tuple[float, float, float] | None = None
        ros.node.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, m) -> None:
        """Store one odometry message. `speed` is the larger of the linear speed (m/s) and the turn rate (rad/s), so a
        machine turning on the spot does not count as still.
        """
        p = m.pose.pose
        s = self.frame.to_site(np.array([p.position.x, p.position.y, p.position.z]))
        self.pose = (float(s[0]), float(s[1]), (_yaw(p.orientation) - self.frame.yaw) % (2 * math.pi))
        tw = m.twist.twist
        self.speed = max(math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z))
        self.z, self.at = float(s[2]), time.time()

    def fresh(self) -> bool:
        """Has odometry arrived within the last second?"""
        return time.time() - self.at < STALE_S


class Carrier:
    """A Nav2 machine (OPERA IC120 and others) as a bridge adapter.

    Work passes (segments inside the order's window) go to Nav2's controller as an explicit path
    (`follow_path`), so the machine holds the work line instead of re-planning from wherever it is.
    Travel and haul segments go to `navigate_through_poses`, so Nav2 plans them."""

    def __init__(self, ros: Ros, ns: str, frame: MapFrame, dump: bool = False, dump_angle_deg: float = 60.0,
                 line_step_m: float = 0.25):
        """Create the Nav2 action clients, the speed-limit publisher and, with `dump`, the vessel client.

        `dump_angle_deg` is how far the vessel is raised. `line_step_m` is the spacing of the points on a work
        line.
        """
        from nav2_msgs.action import FollowPath, NavigateThroughPoses
        from nav2_msgs.msg import SpeedLimit
        from rclpy.action import ActionClient
        self.ros, self.ns, self.frame = ros, ns.strip("/"), frame
        self.odom = _Odom(ros, f"/{self.ns}/odom", frame)
        self.NTP, self.FP, self.SpeedLimit = NavigateThroughPoses, FollowPath, SpeedLimit
        self.nav = ActionClient(ros.node, NavigateThroughPoses, f"/{self.ns}/navigate_through_poses")
        self.follow = ActionClient(ros.node, FollowPath, f"/{self.ns}/follow_path")
        self.speed_pub = ros.node.create_publisher(SpeedLimit, f"/{self.ns}/speed_limit", 1)
        self.dumper = None
        if dump:
            from com3_msgs.action import SetDumpAngle
            self.SetDumpAngle = SetDumpAngle
            self.dumper = ActionClient(ros.node, SetDumpAngle, f"/{self.ns}/set_dump_angle")
        self.dump_angle, self.line_step = math.radians(dump_angle_deg), line_step_m
        self.steps: list[dict] = []          # {"mode": "nav" | "line", "poses": [...], "dump": bool}
        self.k = 0                           # current step
        self.left: list[np.ndarray] = []     # poses of the current step not yet reached
        self.sent = 0                        # poses in the goal on its way
        self.goal = None
        self.state = "idle"                  # idle | driving | waiting (for Nav2) | paused | dumping | done | failed
        self.waiting_since = self.tried_at = 0.0
        self.aborts = 0
        self.lock = threading.Lock()
        self.total = 1
        self.max_speed: float | None = None
        self.speed_at = 0.0

    # Position comes from odometry, which has no fix quality to report, so the fix always reads as good.
    pose = property(lambda self: self.odom.pose)
    fix_ok = property(lambda self: True)
    fix_name = property(lambda self: "odom")

    @property
    def fault(self) -> str | None:
        """`vehicleLinkLost` without fresh odometry; `vehicleNavUnavailable` when Nav2 has not taken a goal for 10 s;
        else None.
        """
        if not self.odom.fresh():
            return "vehicleLinkLost"
        if self.state == "waiting" and time.time() - self.waiting_since > UNAVAILABLE_S:
            return "vehicleNavUnavailable"
        return None

    @property
    def order_error(self) -> str | None:
        """`vehicleNavFailed` when Nav2 gave up on the order, else None."""
        return "vehicleNavFailed" if self.state == "failed" else None

    def poll(self) -> None:
        """Retry a goal Nav2 was not ready for, and repeat the speed limit once a second."""
        if self.state == "waiting" and time.time() - self.tried_at > 1.0:   # Nav2 not there yet: retry
            with self.lock:
                if self.state == "waiting":
                    self.tried_at = time.time()
                    self._send()
        # the limit is repeated: a message sent before the controller's subscription has matched is lost
        if time.time() - self.speed_at > 1.0 and self.state in ("driving", "paused", "dumping"):
            msg = self.SpeedLimit(percentage=not self.max_speed, speed_limit=float(self.max_speed or 100.0))
            msg.header.frame_id = "map"
            self.speed_pub.publish(msg)
            self.speed_at = time.time()

    def start(self, path: list[tuple[float, float]], order: dict, max_speed_ms: float | None) -> None:
        """Turn an order's path into steps: work lines inside the order's window, and navigation everywhere else.

        With a vessel, the step that ends nearest the centre of the deposit is marked to dump there. The carrier is
        left paused; the bridge releases it when it is safe to move.
        """
        self._cancel()
        win = order.get("window")
        inside = (lambda x, y: bool(polygon_mask([tuple(p) for p in win], np.array([x]), np.array([y]))[0, 0])) \
            if win else (lambda x, y: False)
        steps: list[dict] = []
        for (x0, y0), (x1, y1) in itertools.pairwise(path):
            mode = "line" if inside((x0 + x1) / 2, (y0 + y1) / 2) else "nav"   # ends sit on the window's edge
            p = self.frame.to_map(x1, y1)
            if mode == "nav" and steps and steps[-1]["mode"] == "nav":
                steps[-1]["poses"].append(p)
            else:                                   # a work line runs from its own start, not the machine
                steps.append({"mode": mode, "poses": [p], "dump": False, "start": self.frame.to_map(x0, y0)})
        dep = order.get("deposit") or {}
        if self.dumper is not None and dep.get("window") and steps:
            c = self.frame.to_map(*np.asarray(dep["window"], float).mean(0))
            flat = [(i, j, q) for i, st in enumerate(steps) for j, q in enumerate(st["poses"])]
            i, j, _ = min(flat, key=lambda t: float(np.hypot(*(t[2][:2] - c[:2]))))
            st = steps[i]
            if j + 1 < len(st["poses"]):                       # split the step after the deposit pose
                steps[i:i + 1] = [{**st, "poses": st["poses"][:j + 1]}, {**st, "poses": st["poses"][j + 1:]}]
            steps[i]["dump"] = True
        if not steps:
            steps = [{"mode": "nav", "poses": [self.frame.to_map(*path[0])], "dump": False}]
        with self.lock:
            self.steps, self.k, self.left, self.aborts = steps, 0, list(steps[0]["poses"]), 0
            self.total = sum(len(st["poses"]) + st["dump"] for st in steps)
            self.max_speed, self.speed_at = max_speed_ms, 0.0
            self.state = "paused"                     # the bridge un-pauses it when it is safe to go

    def _here(self) -> np.ndarray:
        """Where the machine is, in the map frame (before the first odometry: the next pose)."""
        return self.frame.to_map(*self.pose[:2]) if self.pose else self.left[0]

    def _send(self) -> None:
        """Send the next goal for the current step. Nav2 ends a through-poses goal as soon as the machine is
        at its *last* pose, so a path that comes back to where it started is sent in pieces."""
        while True:
            if self.k >= len(self.steps):
                self.state = "done"
                return
            st = self.steps[self.k]
            if st["mode"] == "nav":
                while self.left and np.hypot(*(self.left[0][:2] - self._here()[:2])) < 1.0:
                    self.left = self.left[1:]                       # already there
            if self.left:
                break
            if st["dump"]:
                self.state = "dumping"
                threading.Thread(target=self._dump, daemon=True).start()
                return
            self._next_step()
        here = self._here()
        if st["mode"] == "line":
            chunk = self.left[:1]
            goal, client, fb = self.FP.Goal(path=self._path([st["start"], *chunk])), self.follow, None
        else:
            chunk = [self.left[0]]
            for p in self.left[1:]:
                if np.hypot(*(p[:2] - here[:2])) < 1.0:              # comes back: next goal
                    break
                chunk.append(p)
            goal = self.NTP.Goal(poses=self._path([here, *chunk], dense=False).poses[1:])
            client, fb = self.nav, self._feedback
        if not client.server_is_ready():
            self._wait_retry()
            return
        self.state, self.sent = "driving", len(chunk)
        fut = client.send_goal_async(goal, feedback_callback=fb)
        fut.add_done_callback(self._accepted)

    def _wait_retry(self) -> None:
        """Note that Nav2 could not take the goal. `poll` tries again."""
        if self.state != "waiting":
            self.state, self.waiting_since = "waiting", time.time()
        self.tried_at = time.time()

    def _next_step(self) -> None:
        """Move on to the next step of the order."""
        self.k += 1
        self.left = list(self.steps[self.k]["poses"]) if self.k < len(self.steps) else []

    def _path(self, pts: list[np.ndarray], dense: bool = True):
        """nav_msgs/Path through `pts` (map frame), every `line_step` metres when dense, heading along it."""
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        out = Path()
        out.header.frame_id = "map"
        way = [pts[0]]
        for a, b in itertools.pairwise(pts):
            n = max(1, math.ceil(np.hypot(*(b[:2] - a[:2])) / self.line_step)) if dense else 1
            way += [a + (b - a) * (i / n) for i in range(1, n + 1)]
        for i, p in enumerate(way):
            d = way[i + 1] - p if i + 1 < len(way) else (p - way[i - 1] if i else np.zeros(3))  # face along it
            yaw = math.atan2(d[1], d[0]) if np.hypot(d[0], d[1]) > 1e-9 else 0.0
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.poses.append(ps)
        return out

    def _feedback(self, fb) -> None:
        """Nav2's progress on a through-poses goal: drop the poses it has passed."""
        with self.lock:
            if self.state == "driving":
                passed = self.sent - int(fb.feedback.number_of_poses_remaining)
                if passed > 0:
                    self.left, self.sent = self.left[passed:], self.sent - passed

    def _accepted(self, fut) -> None:
        """Nav2 has answered a goal request: keep the goal's handle, or retry if it was refused."""
        h = fut.result()
        with self.lock:
            if not h.accepted:                          # e.g. Nav2 not active yet: try again shortly
                self._wait_retry()
                return
            if self.state != "driving":                 # paused or stopped while the goal was on its way
                h.cancel_goal_async()
                return
            self.goal = h
        h.get_result_async().add_done_callback(lambda f: self._result(h, f))

    def _result(self, h, f) -> None:
        """A goal has ended. Success moves on; an abort is retried a few times, and then the order fails."""
        from action_msgs.msg import GoalStatus
        with self.lock:
            if h is not self.goal or self.state != "driving":
                return                                 # a cancelled or replaced goal
            self.goal = None
            if f.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.left, self.sent, self.aborts = self.left[self.sent:], 0, 0
                self._send()
            elif self.aborts < NAV_RETRIES:            # e.g. Nav2 still waiting for its transforms
                self.aborts += 1
                self._wait_retry()
                self.tried_at = time.time() + 1.0      # retried 2 s from now
            else:
                self.state = "failed"

    def _dump(self) -> None:
        """Raise the vessel, then lower it. Runs in its own thread, because it waits.

        It waits up to 30 s for the machine to have stood still for half a second. Any failure fails the order: a
        load that was not dumped must not be booked as placed.
        """
        still_since, end = None, time.time() + 30.0
        while time.time() < end:                       # never raise the vessel on the move
            if self.odom.speed < 0.02:
                still_since = still_since or time.time()
                if time.time() - still_since > 0.5:
                    break
            else:
                still_since = None
            time.sleep(0.05)
        else:
            with self.lock:
                self.state = "failed"
            return
        dumper = self.dumper
        ok = dumper is not None and dumper.wait_for_server(timeout_sec=30.0)
        for angle in (self.dump_angle, 0.0):
            if not ok or dumper is None:
                break
            h = _wait(dumper.send_goal_async(self.SetDumpAngle.Goal(target_angle=angle)), 10)
            ok = h is not None and h.accepted and (r := _wait(h.get_result_async(), 60)) is not None \
                and r.result.success
        with self.lock:
            if not ok:
                self.state = "failed"
                return
            self.steps[self.k]["dump"] = False
            self._next_step()
            self._send()

    def _cancel(self) -> None:
        """Cancel the goal that is running, if any."""
        if self.goal is not None:
            self.goal.cancel_goal_async()
            self.goal = None

    def pause(self, on: bool) -> None:
        """Hold: cancel the goal, and Nav2 stops the machine. Release: send the poses that are left."""
        with self.lock:
            if on and self.state in ("driving", "waiting"):
                self.state = "paused"
                self._cancel()
            elif not on and self.state == "paused":
                self._send()

    def estop(self) -> None:
        """E-stop: cancel the goal. The machine's own safety system does the actual stopping."""
        with self.lock:
            if self.state in ("driving", "waiting"):
                self.state = "paused"
            self._cancel()

    def fraction(self) -> float:
        """How far through the order, 0 to 1, by poses reached (a dump counts as one)."""
        if self.state == "done" or not self.steps:
            return 1.0 if self.state == "done" else 0.0
        done = sum(len(st["poses"]) + st["dump"] for st in self.steps[:self.k])
        done += len(self.steps[self.k]["poses"]) - len(self.left) if self.k < len(self.steps) else 0
        return min(1.0, done / max(self.total, 1))

    def done(self) -> bool:
        """Has every step been driven and dumped?"""
        return self.state == "done"

    def close(self) -> None:
        """Cancel whatever is running."""
        self._cancel()


def _wait(fut, timeout: float):
    """Wait up to `timeout` seconds for a ROS future. Returns its result, or None if it did not finish."""
    end = time.time() + timeout
    while not fut.done() and time.time() < end:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


class Excavator:
    """An OPERA excavator (ZX200) as a bridge adapter: the bucket tip follows the order's target surface."""

    def __init__(self, ros: Ros, ns: str, frame: MapFrame, model: Model, base_dir: str, broker: str,
                 urdf: str | None = None, step_m: float = 0.1, lift_m: float = 0.6, speed_scale: float = 0.5,
                 joint_speed: dict[str, float] | None = None):
        """Create the arm's trajectory client, listen to joint states, odometry and Loam terrain surveys, and load the
        design surface.

        `step_m` is the spacing of the bucket-tip points along a stroke. `lift_m` is how far above the surface the
        bucket travels between strokes. `speed_scale` is the fraction of the joint speed limits to use.
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.ros, self.ns, self.frame = ros, ns.strip("/"), frame
        self.odom = _Odom(ros, f"/{self.ns}/odom", frame)
        self.FJT = FollowJointTrajectory
        self.arm = ActionClient(ros.node, FollowJointTrajectory,
                                f"/{self.ns}/upper_arm_controller/follow_joint_trajectory")
        self.q: dict[str, float] = {}
        self.q_at = 0.0
        ros.node.create_subscription(JointState, f"/{self.ns}/joint_states", self._joints, 10)
        self.urdf = urdf
        if urdf is None:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            ros.node.create_subscription(String, f"/{self.ns}/robot_description",
                                         lambda m: setattr(self, "urdf", m.data), qos)
        self.g = SiteGrid(model, base_dir)
        self.design = self.g.surface(model.site["design"], allow_gaps=True)
        self.ground: np.ndarray | None = None                  # latest Loam terrain survey, site units
        self.bus = Bus(broker, f"loam-opera-{self.ns}-terrain-{time.time():.3f}").start(
            [wire.topic(model.site["name"], None, "terrain")])
        self.step, self.lift, self.scale = step_m / frame.u, lift_m, speed_scale
        self.vmax = {**OPERA_JOINT_SPEED, **(joint_speed or {})}
        self.ik: ExcavatorIK | None = None
        self.points: list[dict[str, float]] = []
        self.times: list[float] = []
        self.sent_from = 0
        self.goal = None
        self.started = 0.0
        self.state = "idle"
        self.waiting_since = self.tried_at = 0.0
        self.reason: str | None = None
        self.lock = threading.Lock()

    # As for the carrier: position from odometry, no fix quality.
    pose = property(lambda self: self.odom.pose)
    fix_ok = property(lambda self: True)
    fix_name = property(lambda self: "odom")

    @property
    def fault(self) -> str | None:
        """`vehicleLinkLost` without fresh odometry and joint states; `vehicleArmUnavailable` when the arm controller
        has not taken a trajectory for 10 s; else None.
        """
        if not self.odom.fresh() or time.time() - self.q_at > STALE_S:
            return "vehicleLinkLost"
        if self.state == "waiting" and time.time() - self.waiting_since > UNAVAILABLE_S:
            return "vehicleArmUnavailable"
        return None

    @property
    def order_error(self) -> str | None:
        """Why the order cannot run (for example `outOfReach` or `noTerrain`), or None."""
        return self.reason if self.state == "failed" else None

    def _joints(self, m) -> None:
        """Store the newest joint angles."""
        self.q.update(dict(zip(m.name, (float(p) for p in m.position), strict=False)))
        self.q_at = time.time()

    def poll(self) -> None:
        """Retry a trajectory the arm controller was not ready for, and take in new terrain surveys."""
        if self.state == "waiting" and time.time() - self.tried_at > 1.0:   # arm controller not there yet
            with self.lock:
                if self.state == "waiting":
                    self.tried_at = time.time()
                    self._send()
        for _, msg in self.bus.drain(0.0):
            with tolerant("terrain"):
                z = wire.decode_heights(msg["data"], tuple(msg["shape"])).astype(float)
                if z.shape == self.g.shape:
                    self.ground = z

    # -- planning -----------------------------------------------------------------------------
    def target_surface(self, order: dict) -> tuple[np.ndarray, np.ndarray]:
        """What the ground should become (site units, whole grid) and the window's cells."""
        g, u = self.g, self.frame.u
        win = order.get("window") or []
        mk = polygon_mask([tuple(p) for p in win], g.xs, g.ys)
        tgt = order.get("target") or {}
        z = np.full(g.shape, np.nan)
        if tgt.get("pattern"):
            if self.ground is None:
                raise ValueError("noTerrain")
            tile = wire.decode_heights(tgt["pattern"], tuple(tgt["shape"])).astype(float) / u
            p = np.asarray(win, float)
            i0 = round((p[:, 0].min() - g.x0) / g.cell)
            j0 = round((p[:, 1].min() - g.y0) / g.cell)
            planned = np.zeros(g.shape)
            planned[j0:j0 + tile.shape[0], i0:i0 + tile.shape[1]] = tile
            z = self.ground + planned
        elif "elevation" in tgt:
            z = np.full(g.shape, float(tgt["elevation"]) / u)
        elif "offset" in tgt:
            if self.ground is None:
                raise ValueError("noTerrain")
            z = self.ground + float(tgt["offset"]) / u
        else:
            z = self.design.copy()
        z = np.where(np.isfinite(self.design), np.maximum(z, self.design), z)     # never below design
        return z, mk

    def _height(self, surf: np.ndarray, mk: np.ndarray, x: float, y: float) -> float:
        """Bilinear target height at (x, y), or NaN outside the window's cells."""
        g = self.g
        ci, cj = round((x - g.xs[0]) / g.cell), round((y - g.ys[0]) / g.cell)
        if not (0 <= ci < g.shape[1] and 0 <= cj < g.shape[0] and mk[cj, ci]):
            return math.nan
        fi, fj = (x - g.xs[0]) / g.cell, (y - g.ys[0]) / g.cell
        i, j = math.floor(fi), math.floor(fj)
        if not (0 <= i < g.shape[1] - 1 and 0 <= j < g.shape[0] - 1):
            return math.nan
        a, b = fi - i, fj - j
        c = surf[j:j + 2, i:i + 2]
        return float((1 - b) * ((1 - a) * c[0, 0] + a * c[0, 1]) + b * ((1 - a) * c[1, 0] + a * c[1, 1]))

    def plan(self, order: dict) -> tuple[list[dict[str, float]], list[float], list[tuple | None]]:
        """Joint waypoints, their times (s) and the tip targets (site x, y, z or None for travel moves)."""
        if self.ik is None:
            if self.urdf is None:
                raise ValueError("noRobotDescription")
            self.ik = ExcavatorIK(Chain(self.urdf))
        surf, mk = self.target_surface(order)
        x0, y0, hd = self.pose
        bz = self.odom.z
        u = self.frame.u
        q = {n: self.q.get(n, 0.0) for n in self.ik.names}
        prefer = -1.2                                          # bucket tip pointing down and a little back

        def base_frame(x, y, z):                               # site point -> machine base frame (m)
            """A site point as a point in the machine's base frame (m): x forward, y left, z up."""
            dx, dy = (x - x0) * u, (y - y0) * u
            c, s = math.cos(hd), math.sin(hd)
            return np.array([c * dx + s * dy, -s * dx + c * dy, (z - bz) * u])
        cells = np.argwhere(mk)
        if not len(cells):
            raise ValueError("emptyWindow")
        pts = np.column_stack([self.g.xs[cells[:, 1]], self.g.ys[cells[:, 0]]])
        rel = np.array([base_frame(x, y, 0)[:2] for x, y in pts])
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        rad = np.hypot(rel[:, 0], rel[:, 1])
        # Strokes fan out from the swing axis like the spokes of a wheel. Their angular spacing is chosen so that, at
        # the far edge of the window, neighbouring strokes are just under one grid cell apart; each runs from the far
        # side of the window in towards the machine.
        spacing = self.g.cell * u * 0.9
        strokes = []
        da = spacing / max(rad.max(), 1.0)
        a = ang.min() - spacing / 2 / max(rad.min(), 1.0) + da / 2       # cover the outer half cells too
        while a <= ang.max() + spacing / 2 / max(rad.min(), 1.0):
            r_hi = rad[np.abs(((ang - a + math.pi) % (2 * math.pi)) - math.pi) * np.maximum(rad, 1) < spacing]
            if len(r_hi):
                strokes.append((a, r_hi.max() + spacing / 2, r_hi.min() - spacing / 2))
            a += da
        out_q, targets = [], []
        for a, r1, r0 in strokes:
            line = []
            for r in np.arange(r1, r0 - 1e-9, -self.step * u):
                bx, by = r * math.cos(a), r * math.sin(a)
                c, s = math.cos(hd), math.sin(hd)
                sx, sy = x0 + (c * bx - s * by) / u, y0 + (s * bx + c * by) / u
                h = self._height(surf, mk, sx, sy)
                if math.isfinite(h):
                    line.append((sx, sy, h, np.array([bx, by, (h - bz) * u])))
            if not line:
                continue
            # Each stroke starts and ends with the bucket lifted clear, so that it travels to the next stroke above
            # the ground. A lifted point that cannot be reached is skipped; a point on the surface that cannot be
            # reached fails the order.
            above = [line[0][3] + [0, 0, self.lift], line[-1][3] + [0, 0, self.lift]]
            seq = [(above[0], None), *[(p, (sx, sy, h)) for sx, sy, h, p in line], (above[1], None)]
            for p, tgt in seq:
                sol = self.ik.solve_any(p, q, prefer)
                if sol is None:
                    if tgt is None:
                        continue
                    raise ValueError("outOfReach")
                q = sol
                out_q.append(dict(q))
                targets.append(tgt)
        if not out_q:
            raise ValueError("outOfReach")
        times, t, prev = [], 0.0, {n: self.q.get(n, 0.0) for n in self.ik.names}
        for qq in out_q:
            # Time each point by the slowest joint: the joint that needs longest at its speed limit sets the pace.
            dt = max(abs(qq[n] - prev[n]) / (self.vmax.get(n, 0.5) * self.scale) for n in self.ik.names)
            t += max(dt, 0.05)
            times.append(t)
            prev = qq
        return out_q, times, targets

    # -- adapter ----------------------------------------------------------------------------------
    def start(self, path: list[tuple[float, float]], order: dict, max_speed_ms: float | None) -> None:
        """Plan the order. The arm is left paused, or the order is failed with the reason it cannot be planned."""
        self._cancel()
        try:
            self.points, self.times, self.targets = self.plan(order)
            self.state, self.reason, self.sent_from = "paused", None, 0
        except ValueError as e:
            self.points, self.times, self.state, self.reason = [], [], "failed", str(e)

    def _send(self) -> None:
        """Send the trajectory from the point reached so far. After a pause the arm first gets time to move back onto
        the path.
        """
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        if self.ik is None:
            self.state, self.reason = "failed", "noRobotDescription"
            return
        if not self.arm.server_is_ready():
            if self.state != "waiting":
                self.state, self.waiting_since = "waiting", time.time()
            self.tried_at = time.time()
            return
        k0 = self.sent_from
        names = self.ik.names
        held = [n for n in self.ik.c.moving if n not in names]          # e.g. bucket_end_joint: stays put
        tr = JointTrajectory(joint_names=[*names, *held])
        t0 = self.times[k0 - 1] if k0 else 0.0
        lead = 0.0
        if k0:                                                # resume: first get back onto the path
            lead = max(abs(self.points[k0][n] - self.q.get(n, 0.0)) / (self.vmax.get(n, 0.5) * self.scale)
                       for n in names) + 0.2
        for qq, t in zip(self.points[k0:], self.times[k0:], strict=True):
            sec = t - t0 + lead
            tr.points.append(JointTrajectoryPoint(positions=[*(qq[n] for n in names),
                                                             *(self.q.get(n, 0.0) for n in held)],
                                                  time_from_start=Duration(sec=int(sec),
                                                                           nanosec=int((sec % 1) * 1e9))))
        self.state, self.started, self.t_base, self.lead = "driving", time.time(), t0, lead
        fut = self.arm.send_goal_async(self.FJT.Goal(trajectory=tr))
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut) -> None:
        """The arm controller has answered a trajectory request: keep the goal's handle, or retry if it was refused."""
        h = fut.result()
        with self.lock:
            if not h.accepted:                          # e.g. the controller not active yet: try again
                if self.state != "waiting":
                    self.state, self.waiting_since = "waiting", time.time()
                self.tried_at = time.time()
                return
            if self.state != "driving":                 # paused or stopped while the goal was on its way
                h.cancel_goal_async()
                return
            self.goal = h
        h.get_result_async().add_done_callback(lambda f: self._result(h, f))

    def _result(self, h, f) -> None:
        """The trajectory has ended: done, or failed with the controller's error."""
        with self.lock:
            if h is not self.goal or self.state != "driving":
                return
            self.goal = None
            if f.result().result.error_code == 0:
                self.state, self.sent_from = "done", len(self.points)
            else:
                self.state, self.reason = "failed", "vehicleTrajectoryFailed"

    def _progress_index(self) -> int:
        """The index of the trajectory point the arm should have reached by now, going by the clock."""
        if self.state != "driving":
            return self.sent_from
        t = self.t_base + max(0.0, time.time() - self.started - self.lead)
        return int(np.searchsorted(self.times, t, side="right"))

    def _cancel(self) -> None:
        """Cancel the trajectory that is running, if any."""
        if self.goal is not None:
            self.goal.cancel_goal_async()
            self.goal = None

    def pause(self, on: bool) -> None:
        """Hold: note the point reached and cancel the trajectory, so the controller holds the arm where it is.
        Release: carry on from that point.
        """
        with self.lock:
            if on and self.state == "waiting":
                self.state = "paused"
            elif on and self.state == "driving":
                self.sent_from = min(self._progress_index(), len(self.points) - 1)
                self.state = "paused"
                self._cancel()
            elif not on and self.state == "paused" and self.points:
                self._send()

    def estop(self) -> None:
        """E-stop: hold the arm. The machine's own safety system does the actual stopping."""
        self.pause(True)

    def fraction(self) -> float:
        """How far through the trajectory, 0 to 1."""
        return min(1.0, self._progress_index() / max(len(self.points), 1)) if self.points else 0.0

    def done(self) -> bool:
        """Has the whole trajectory been run?"""
        return self.state == "done"

    def close(self) -> None:
        """Cancel whatever is running and disconnect from the broker."""
        self._cancel()
        self.bus.stop()


def run_opera(model: Model, name: str, broker: str, ns: str, kind: str, map_offset=(0.0, 0.0, 0.0),
              map_yaw_deg: float = 0.0, base_dir: str = ".", dump: bool = False, urdf: str | None = None,
              sim_world: bool = False, max_wall_s: float | None = None):
    """Bridge one OPERA machine as Loam machine `name` (see `bridge.run_bridge`)."""
    from ..units import length_unit_si
    frame = MapFrame(length_unit_si(model.site.get("units", "ft")), map_offset, map_yaw_deg)
    ros = Ros(f"loam_{name}")
    try:
        if kind == "excavator":
            adapter: Any = Excavator(ros, ns, frame, model, base_dir, broker, urdf)
        else:
            adapter = Carrier(ros, ns, frame, dump)
        return run_bridge(model, name, broker, adapter, sim_world, max_wall_s)
    finally:
        ros.close()
