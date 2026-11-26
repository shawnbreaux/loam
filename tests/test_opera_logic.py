"""The OPERA adapters' goal handling, without ROS (these run in CI).

ROS message types and action clients are replaced by small stand-ins, and the test resolves every
future by hand. That fixes the order of acceptances, feedback, results and pauses, so the races met
on a live Nav2 (a stop while a goal is on its way, a goal rejected while Nav2 starts, a path that comes
back to its start) are replayed exactly. tests/test_opera.py runs the same adapters on real ROS 2.
"""
import sys
import threading
import time
import types

import numpy as np
import pytest


# -- stand-ins for the ROS pieces the adapters use ---------------------------------------------------------
class Msg:
    """Any ROS message: an object whose fields are whatever it is given."""
    def __init__(self, **kw):
        """Set the fields."""
        self.__dict__.update(kw)


def _pose_stamped():
    """An empty geometry_msgs/PoseStamped."""
    return Msg(header=Msg(frame_id=""), pose=Msg(position=Msg(x=0.0, y=0.0, z=0.0),
                                                 orientation=Msg(x=0.0, y=0.0, z=0.0, w=1.0)))


def _action(name):
    """An action type with a Goal message, as rclpy's generated action classes have."""
    return type(name, (), {"Goal": type("Goal", (Msg,), {})})


# The ROS message modules the adapters import, each replaced by the few names they use.
STUBS = {
    "nav2_msgs": {}, "nav2_msgs.action": {"NavigateThroughPoses": _action("NTP"), "FollowPath": _action("FP"),
                                          "FollowJointTrajectory": _action("FJT")},
    "nav2_msgs.msg": {"SpeedLimit": lambda **kw: Msg(header=Msg(frame_id=""), **kw)},
    "geometry_msgs": {}, "geometry_msgs.msg": {"PoseStamped": _pose_stamped},
    "nav_msgs": {}, "nav_msgs.msg": {"Path": lambda: Msg(header=Msg(frame_id=""), poses=[])},
    "action_msgs": {}, "action_msgs.msg": {"GoalStatus": Msg(STATUS_SUCCEEDED=4, STATUS_ABORTED=6)},
    "trajectory_msgs": {}, "trajectory_msgs.msg": {"JointTrajectory": lambda **kw: Msg(points=[], **kw),
                                                   "JointTrajectoryPoint": lambda **kw: Msg(**kw)},
    "builtin_interfaces": {}, "builtin_interfaces.msg": {"Duration": lambda **kw: Msg(**kw)},
}


@pytest.fixture
def ros_stubs(monkeypatch):
    """Put the stand-in ROS message modules in place for one test, so the adapters' imports find them."""
    for name, attrs in STUBS.items():
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)


class Future:
    """The result of an asynchronous ROS call. The test decides when it completes."""
    def __init__(self):
        """Not done, no result, no callbacks."""
        self._done, self._result, self._cbs = False, None, []

    def done(self):
        """Has it completed?"""
        return self._done

    def result(self):
        """Its value (None until it completes)."""
        return self._result

    def add_done_callback(self, cb):
        """Call `cb` when it completes, or at once if it already has."""
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def resolve(self, value):
        """Complete it with `value` and run the callbacks."""
        self._done, self._result = True, value
        for cb in self._cbs:
            cb(self)


class Handle:
    """A goal handle: what an action server returns when it answers a goal request."""
    def __init__(self, accepted=True):
        """`accepted` is the server's answer."""
        self.accepted, self.cancelled, self.result_future = accepted, False, Future()

    def cancel_goal_async(self):
        """Note that the goal was cancelled."""
        self.cancelled = True

    def get_result_async(self):
        """The future that completes when the goal ends."""
        return self.result_future


class Client:
    """An action client: records goals; the test accepts, rejects and finishes them."""

    def __init__(self, ready=True):
        """`ready` says whether the action server exists yet."""
        self.ready, self.sent = ready, []

    def server_is_ready(self):
        """Is the server there?"""
        return self.ready

    def wait_for_server(self, timeout_sec=0.0):
        """Is the server there? (It never waits.)"""
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        """Record the goal and return the future for the server's answer."""
        f = Future()
        self.sent.append((goal, feedback_callback, f))
        return f

    def accept(self, k=-1, accepted=True):
        """Have the server answer goal number `k` (the latest by default), accepting or rejecting it. Returns the
        goal's handle.
        """
        h = Handle(accepted)
        self.sent[k][2].resolve(h)
        return h


def finish(h, status=4, **result):
    """End a goal with a status (4 = succeeded, 6 = aborted) and result fields."""
    h.result_future.resolve(Msg(status=status, result=Msg(**result)))


class Pub:
    """A publisher that keeps what is published."""
    def __init__(self):
        """Nothing published yet."""
        self.msgs = []

    def publish(self, m):
        """Keep the message."""
        self.msgs.append(m)


def odom(x=0.0, y=0.0, speed=0.0):
    """Fresh odometry at (x, y), moving at `speed`."""
    return types.SimpleNamespace(pose=(x, y, 0.0), z=0.0, speed=speed, at=time.time(),
                                 fresh=lambda: True)


def carrier(**kw):
    """A Carrier adapter wired to the stand-ins instead of ROS. `kw` overrides any attribute."""
    from loam.runtime.opera import Carrier, MapFrame
    c = Carrier.__new__(Carrier)
    c.__dict__.update(frame=MapFrame(1.0), dumper=None, goal=None, lock=threading.Lock(), state="idle",
                      nav=Client(), follow=Client(), speed_pub=Pub(), odom=odom(), line_step=0.25, k=0, left=[],
                      sent=0, total=1, max_speed=None, speed_at=0.0, waiting_since=0.0, tried_at=0.0,
                      dump_angle=1.0, steps=[], aborts=0)
    from nav2_msgs.action import FollowPath, NavigateThroughPoses
    from nav2_msgs.msg import SpeedLimit
    c.NTP, c.FP, c.SpeedLimit = NavigateThroughPoses, FollowPath, SpeedLimit
    c.__dict__.update(kw)
    return c


# A work window 40 m long and 10 m wide; the work line runs down its middle, y = 30.
WIN = [[60, 25], [100, 25], [100, 35], [60, 35]]


def xy(goal):
    """The (x, y) points of a goal's poses, whichever kind of goal it is."""
    poses = goal.poses if hasattr(goal, "poses") else goal.path.poses
    return [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in poses]


def test_carrier_drives_lines_and_travel_in_order(ros_stubs):
    """An order is sent as three goals in turn: travel to the work line, the work line itself as an explicit path with
    a point every 0.25 m, and the trip back. The speed limit is published.
    """
    c = carrier()
    c.start([(0, 30), (60, 30), (100, 30), (0, 30)], {"window": WIN}, 0.8)
    assert c.state == "paused" and c.nav.sent == []
    c.pause(False)                                                    # the bridge says go
    goal, fb, _ = c.nav.sent[-1]
    assert xy(goal) == [(60, 30)] and c.state == "driving"
    h = c.nav.accept()
    fb(Msg(feedback=Msg(number_of_poses_remaining=1)))                # not there yet
    assert c.left and c.fraction() == 0.0
    c.odom.pose = (60.0, 30.0, 0.0)
    finish(h)
    line = c.follow.sent[-1][0]                                       # the work line, from its own start
    pts = xy(line)
    assert pts[0] == (60, 30) and pts[-1] == (100, 30) and len(pts) == 161      # every 0.25 m
    assert all(p[1] == 30 for p in pts)
    assert c.fraction() == pytest.approx(1 / 3)
    c.poll()
    assert c.speed_pub.msgs[-1].speed_limit == 0.8 and not c.speed_pub.msgs[-1].percentage
    h = c.follow.accept()
    c.odom.pose = (100.0, 30.0, 0.0)
    finish(h)
    back = c.nav.sent[-1][0]
    assert xy(back) == [(0, 30)]
    finish(c.nav.accept())
    assert c.done() and c.fraction() == 1.0


def test_carrier_splits_a_path_that_returns_to_its_start(ros_stubs):
    """Nav2 treats a goal as finished the moment the machine is at its last pose, so a path that returns to its start
    is sent in two parts. Without a speed limit, 100 % is published.
    """
    c = carrier()
    c.odom.pose = (0.0, 0.0, 0.0)
    c.start([(0, 0), (30, 0), (30, 30), (0, 0), (30, 0)], {}, None)   # out, round, home, out again
    c.pause(False)
    assert xy(c.nav.sent[-1][0]) == [(30, 0), (30, 30)]               # stops short of "home"
    h = c.nav.accept()
    c.odom.pose = (30.0, 30.0, 0.0)
    finish(h)
    assert xy(c.nav.sent[-1][0]) == [(0, 0), (30, 0)]                 # home, then out again
    c.poll()
    assert c.speed_pub.msgs[-1].percentage and c.speed_pub.msgs[-1].speed_limit == 100.0   # no limit


def test_carrier_stop_while_a_goal_is_on_its_way(ros_stubs):
    """A stop that arrives before Nav2 has answered still wins: the goal is cancelled the moment it is accepted."""
    c = carrier()
    c.start([(0, 30), (60, 30), (100, 30)], {"window": WIN}, None)
    c.pause(False)
    c.pause(True)                                                     # stop before Nav2 has answered
    h = c.nav.accept()
    assert h.cancelled and c.state == "paused"                        # the late goal is cancelled at once
    c.pause(False)
    h2 = c.nav.accept()
    assert not h2.cancelled and c.state == "driving"
    c.estop()
    assert h2.cancelled and c.state == "paused"


def test_carrier_waits_for_nav2_and_never_fails_the_order(ros_stubs):
    """A missing or not-yet-active Nav2 is waited for and reported, not treated as failure. Only repeated aborts of
    the goal itself fail the order.
    """
    c = carrier(nav=Client(ready=False))
    c.start([(0, 30), (60, 30)], {}, None)
    c.pause(False)
    assert c.state == "waiting" and c.fault is None and c.nav.sent == []
    c.waiting_since -= 11                                             # 11 s on: reported, still retried
    assert c.fault == "vehicleNavUnavailable"
    c.nav.ready = True
    c.poll()
    assert c.nav.sent == []                                           # retries at most once a second
    c.tried_at -= 1.5
    c.poll()
    assert len(c.nav.sent) == 1 and c.state == "driving"
    c.nav.accept(accepted=False)                                      # Nav2 not active yet: rejected
    assert c.state == "waiting"
    c.tried_at -= 1.5
    c.poll()
    for _ in range(3):                                                # aborted: retried, 2 s apart
        finish(c.nav.accept(), status=6)
        assert c.state == "waiting" and c.fault is None
        c.poll()
        assert c.nav.sent[-1][2].done()                               # not sent again yet
        c.tried_at -= 2.5
        c.poll()
    finish(c.nav.accept(), status=6)                                  # still failing: a real navigation failure
    assert c.state == "failed" and c.order_error == "vehicleNavFailed" and c.fault is None
    c.odom.fresh = lambda: False
    assert c.fault == "vehicleLinkLost"


def test_carrier_dumps_only_standing_still(ros_stubs, monkeypatch):
    """The vessel is raised only once the machine has stopped moving, then lowered, and then the machine drives on."""
    from loam.runtime import opera
    dumper = Client()
    c = carrier(dumper=dumper, SetDumpAngle=_action("Dump"))
    c.odom.speed = 0.5                                                # still rolling
    c.start([(0, 30), (65, 65), (0, 30)], {"deposit": {"window": [[60, 60], [70, 60], [70, 70], [60, 70]]}},
            None)
    c.pause(False)
    c.odom.pose = (65.0, 65.0, 0.0)
    finish(c.nav.accept())
    assert c.state == "dumping"
    time.sleep(0.3)
    assert dumper.sent == []                                          # waits for a standstill
    c.odom.speed = 0.0

    def answer():
        """Play the vessel's action server: accept and finish the two goals as they arrive."""
        for _ in range(2):
            while len(dumper.sent) <= _:
                time.sleep(0.01)
            h = dumper.accept(_)
            finish(h, success=True)
    t = threading.Thread(target=answer)
    t.start()
    t.join(5)
    deadline = time.time() + 5
    while c.state == "dumping" and time.time() < deadline:
        time.sleep(0.02)
    assert [g.target_angle for g, *_ in dumper.sent] == [1.0, 0.0]   # up, then down
    assert xy(c.nav.sent[-1][0]) == [(0, 30)] and c.state == "driving"
    assert opera.UNAVAILABLE_S == 10.0


# -- the excavator's trajectory handling -----------------------------------------------------------------------
def excavator(points, times):
    """An Excavator adapter with a ready-made trajectory, wired to the stand-ins instead of ROS."""
    from test_safety import ARM

    from loam.runtime.opera import Chain, Excavator, ExcavatorIK
    ik = ExcavatorIK(Chain(ARM, tip="tip"))
    e = Excavator.__new__(Excavator)
    e.__dict__.update(ik=ik, arm=Client(), points=points, times=times, targets=[None] * len(points),
                      sent_from=0, goal=None, state="paused", reason=None, lock=threading.Lock(),
                      waiting_since=0.0, tried_at=0.0, scale=0.5, vmax={}, q={n: 0.0 for n in ik.names},
                      odom=odom(), q_at=time.time(), bus=types.SimpleNamespace(drain=lambda t: []))
    from nav2_msgs.action import FollowJointTrajectory
    e.FJT = FollowJointTrajectory
    return e


def test_excavator_pause_resume_and_waiting(ros_stubs):
    """A pause cancels the trajectory and remembers the point reached; a resume sends the rest, with time to rejoin
    the path. A missing controller is waited for; a controller error fails the order.
    """
    pts = [{"swing_joint": 0.0, "boom_joint": -0.1 * k, "arm_joint": 1.0, "bucket_joint": 0.5} for k in range(5)]
    e = excavator(pts, [1.0, 2.0, 3.0, 4.0, 5.0])
    e.pause(False)
    tr = e.arm.sent[-1][0].trajectory
    assert len(tr.points) == 5 and tr.joint_names == e.ik.names
    h = e.arm.accept()
    e.started -= 2.5                                                  # 2.5 s into the trajectory
    assert e.fraction() == pytest.approx(2 / 5)
    e.pause(True)
    assert h.cancelled and e.sent_from == 2
    e.q = dict(pts[1])                                                # where the arm stopped
    e.pause(False)
    tr2 = e.arm.sent[-1][0].trajectory
    assert len(tr2.points) == 3                                       # the rest, after a move back onto the path
    first = tr2.points[0].time_from_start
    assert first.sec + first.nanosec / 1e9 > 0.2
    finish(e.arm.accept(), error_code=0)
    assert e.done() and e.fraction() == 1.0
    w = excavator(pts, [1.0, 2.0, 3.0, 4.0, 5.0])
    w.arm.ready = False
    w.pause(False)
    assert w.state == "waiting" and w.fault is None
    w.waiting_since -= 11
    assert w.fault == "vehicleArmUnavailable"
    w.arm.ready, w.tried_at = True, w.tried_at - 1.5
    w.poll()
    w.arm.accept(accepted=False)
    assert w.state == "waiting"                                       # rejected: retried, not failed
    w.tried_at -= 1.5
    w.poll()
    finish(w.arm.accept(), error_code=-4)
    assert w.state == "failed" and w.order_error == "vehicleTrajectoryFailed" and w.fault is None
    assert np.isfinite(w.fraction())
