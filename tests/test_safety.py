"""Safety and robustness of the runtime, with no external systems (these run in CI).

* e-stop: a release never restarts a machine (ISO 13850); only a deliberate resume does
* malformed traffic on the broker is skipped, never fatal
* message order survives waiting for the clock; a failed broker connection leaves nothing behind
* MAVLink: the whole protocol path against a fake ArduPilot written with pymavlink
* OPERA: kinematics, frames, carrier step planning and excavator planning without ROS
* the monitor shuts down with live connections open; `loam resume`
"""
import json
import math
import socket
import threading
import time
import types
import urllib.request

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES

from loam import wire
from loam.checker import check, has_errors
from loam.cli import load, main
from loam.geo import Frame
from loam.parser import parse

pytest.importorskip("paho.mqtt")
from loam.runtime.bus import Bus, tolerant, wait_clock
from loam.runtime.machine import MachineSim


def free_port() -> int:
    """A TCP or UDP port that is free right now."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# -- e-stop latch ---------------------------------------------------------------------------------------
def machine() -> MachineSim:
    """The example's first machine, not connected to any broker: only its safety logic is used."""
    m = load(EXAMPLE)
    return MachineSim(m, next(iter(m.machines)), "localhost:1", (0.0, 0.0))


def test_estop_release_does_not_restart():
    """The e-stop latch, step by step.

    Pressing stops the machine. Releasing does not start it: it stays stopped as `estopReset` until someone sends a
    resume. Heartbeats do not count as a resume. The same holds for per-machine hold and stop commands.
    """
    ms = machine()
    assert ms.safety_state()[0] == "ok"
    hits = []
    ms.control("estop", {"pressed": True}, lambda: hits.append(ms.stop_latency_s[:]))  # a real radio: no `wall`
    assert hits == [[]] and ms.stop_latency_s == []          # the stop runs first; no latency without a stamp
    assert ms.safety_state() == ("stopped", [wire.error("estop")])
    ms.control("estop", {"pressed": True}, lambda: hits.append(1))
    assert len(hits) == 1                                    # a repeated press is one press
    ms.control("estop", {"pressed": False})
    assert ms.safety_state() == ("stopped", [wire.error("estopReset")])
    ms.control("safety", {"command": "heartbeat"})
    assert ms.safety_state()[1][0]["code"] == "estopReset"   # heartbeats do not resume
    ms.control("safety", {"command": "resume"})
    assert ms.safety_state()[0] == "ok"
    # the same through a per-machine instant action, with the simulated site's latency stamp
    ms.control("estop", {"pressed": True, "wall": time.time() - 0.01})
    assert 0.0 < ms.stop_latency_s[-1] < 1.0
    ms.control("estop", {"pressed": False})
    ms.control("instantAction", {"action": "hold"})
    assert ms.held and ms.safety_state()[0] == "stopped"
    ms.control("instantAction", {"action": "resume"})
    assert not ms.held and ms.safety_state()[0] == "ok"
    ms.control("instantAction", {"action": "stop"})
    assert ms.safety_state()[1][0]["code"] == "stop"
    ms.control("safety", {"command": "stop"})
    ms.control("safety", {"command": "resume"})
    assert ms.safety_state()[0] == "ok"


def test_malformed_messages_are_skipped(capsys):
    """A message with missing or wrong fields is reported and skipped; the machine carries on. Errors that are not
    about a message are not hidden.
    """
    ms = machine()
    for kind, msg in (("estop", {}), ("instantAction", {"act": "stop"}), ("safety", {"command": None})):
        with tolerant(f"loam/v0/s/{kind}"):
            ms.control(kind, msg)
    with tolerant("loam/v0/s/x"):
        {}["orderId"]
    err = capsys.readouterr().err
    assert err.count("ignored a malformed message") == 3 and "loam/v0/s/estop" in err
    with pytest.raises(RuntimeError), tolerant("t"):        # only message errors are swallowed
        raise RuntimeError("not a message error")


# -- the bus ------------------------------------------------------------------------------------------------
def test_bus_order_and_failed_connection(mqtt_broker):
    """Messages that arrive while waiting for the site clock are handed back in order, non-JSON traffic is dropped,
    and a failed connection exits with a clear message.
    """
    a = Bus(mqtt_broker, "order-a").start(["t/#", "sim/s/clock"])
    b = Bus(mqtt_broker, "order-b").start([])
    try:
        b.pub("t/1", {"n": 1}, qos=1)
        b.c.publish("t/raw", b"[1, 2]", qos=1)                # not a JSON object: dropped
        b.c.publish("t/bad", b"{oops", qos=1)                 # not JSON: dropped
        b.pub("t/2", {"n": 2}, qos=1)
        time.sleep(0.3)
        b.pub("sim/s/clock", {"t0": time.time(), "speed": 1.0, "epoch": 0.0}, retain=True, qos=1)
        time.sleep(0.3)
        b.pub("t/3", {"n": 3}, qos=1)
        time.sleep(0.3)
        wait_clock(a, "s")
        got = [m["n"] for _, m in a.drain(0.5)]
        assert got == [1, 2, 3]                               # handed back ahead of anything newer
        b.clear("sim/s/clock")
    finally:
        a.stop()
        b.stop()
    with pytest.raises(SystemExit, match="cannot reach"):
        Bus(f"127.0.0.1:{free_port()}", "nobody").start([], timeout=0.5)
    # the leak check (conftest) fails this test if the failed connection left a thread or socket behind


# -- MAVLink against a fake ArduPilot -----------------------------------------------------------------------
class FakeRover(threading.Thread):
    """Enough of ArduPilot Rover's MAVLink behaviour: heartbeats, positions (zeros until its EKF has an
    origin), GPS, the mission and fence handshakes, parameters, modes, arming; in AUTO it reaches each waypoint
    in turn."""

    def __init__(self, port: int, lat0: float, lon0: float):
        """Connect out to the bridge's UDP port, as a vehicle at (lat0, lon0) in HOLD (mode 4), disarmed."""
        super().__init__(daemon=True)
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
        self.lat, self.lon, self.origin = lat0, lon0, False
        self.armed, self.mode, self.items, self.reached = False, 4, [], 0
        self.commands, self.modes, self.running, self.beating = [], [], True, True
        self.reject = False                       # refuse the next mission upload (MAV_MISSION_ERROR)
        # Rover 4.8's defaults for the parameters the bridge sets up, and what has been uploaded as fences.
        self.params = {"FENCE_ENABLE": 0.0, "FENCE_TYPE": 6.0, "FENCE_ACTION": 1.0, "OA_TYPE": 0.0,
                       "FS_GCS_ENABLE": 0.0, "FS_ACTION": 2.0, "FS_GCS_TIMEOUT": 5.0, "MAV_GCS_SYSID": 255.0}
        self.fence, self.incoming, self.sets = [], {}, []
        self.gcs_beats, self.breach = [], False   # times of heartbeats from the bridge; "outside the fence"

    def send_state(self):
        """Send what a vehicle streams: heartbeat, position and GPS status (RTK fixed)."""
        mav = self.m.mav
        if self.beating:
            mav.heartbeat_send(10, 3, (128 if self.armed else 0) | 1, self.mode, 4)
        lat, lon = (self.lat, self.lon) if self.origin else (0.0, 0.0)
        mav.global_position_int_send(0, round(lat * 1e7), round(lon * 1e7), 0, 0, 0, 0, 0, 9000)
        mav.gps_raw_int_send(0, 6, round(self.lat * 1e7), round(self.lon * 1e7), 0, 50, 50, 0, 0, 20)
        mav.fence_status_send(int(self.breach), int(self.breach), 0, 0)

    def run(self):
        """The vehicle's loop: stream state ten times a second, answer messages, and in AUTO (mode 10) and armed, jump
        to each waypoint in turn.
        """
        last = 0.0
        while self.running:
            if time.time() - last > 0.1:
                self.send_state()
                last = time.time()
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self.handle(msg)
            if self.mode == 10 and self.armed and self.items:
                wps = [i for i in self.items if i.command == 16 and i.seq > 0]
                if self.reached < len(wps) and time.time() - last > 0.05:
                    w = wps[self.reached]
                    self.lat, self.lon = w.x / 1e7, w.y / 1e7
                    self.reached += 1
                    self.send_state()                       # a vehicle is at the waypoint before it says so
                    self.m.mav.mission_item_reached_send(w.seq)
        self.m.close()

    def handle(self, msg):
        """Answer one message from the bridge: the mission upload handshake, commands (arm and disarm are obeyed), and
        mode changes.
        """
        t = msg.get_type()
        kind = getattr(msg, "mission_type", 0)                        # 0: mission, 1: fence
        if t == "MISSION_COUNT" and self.reject:
            self.m.mav.mission_ack_send(255, 0, 1, kind)              # the old mission stays on board
        elif t == "MISSION_COUNT":
            self.incoming[kind] = [None] * msg.count
            self.m.mav.mission_request_int_send(255, 0, 0, kind)
        elif t == "MISSION_ITEM_INT":
            got = self.incoming[kind]
            got[msg.seq] = msg
            if msg.seq + 1 < len(got):
                self.m.mav.mission_request_int_send(255, 0, msg.seq + 1, kind)
            else:
                if kind == 1:
                    self.fence = got
                else:
                    self.items, self.reached = got, 0
                self.m.mav.mission_ack_send(255, 0, 0, kind)
        elif t in ("PARAM_REQUEST_READ", "PARAM_SET"):
            name = str(msg.param_id).rstrip("\x00")
            if name in self.params:                                   # an unknown parameter gets no answer
                if t == "PARAM_SET":
                    self.params[name] = float(msg.param_value)
                    self.sets.append(name)
                self.m.mav.param_value_send(name.encode(), self.params[name], 9, len(self.params), 0)
        elif t == "HEARTBEAT" and msg.type == 6:
            self.gcs_beats.append((msg.get_srcSystem(), time.time()))
        elif t == "COMMAND_LONG":
            self.commands.append((msg.command, msg.param1, msg.param2))
            if msg.command == 400:
                self.armed = msg.param1 == 1
        elif t == "SET_MODE":
            self.mode = msg.custom_mode
            self.modes.append(msg.custom_mode)


def test_mavlink_bridge_against_fake_ardupilot(mqtt_broker):
    """The MAVLink bridge from end to end against the fake vehicle.

    In order: no position until the autopilot has one; an order becomes a mission with its speed limit, is driven
    and reported done; a refused mission fails the order without the old mission being re-run; the next order
    works; the e-stop force-disarms; release leaves it stopped; losing the vehicle's heartbeat is reported.
    """
    pytest.importorskip("pymavlink")
    from test_bridges import SRC, Controller

    from loam.checker import check
    from loam.parser import parse
    from loam.runtime.mavlink import run_mavlink
    model, _ = check(parse(SRC))
    lat0, lon0 = 34.4459, -85.7197
    frame = Frame(0.3048, 0.0, lat0, lon0)
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.start()
    ctl = Controller(mqtt_broker, "yard", "dz1")
    bridge = threading.Thread(target=run_mavlink, args=(model, "dz1", mqtt_broker, f"udpin:127.0.0.1:{port}",
                                                       frame, 6, False, 60), daemon=True)
    try:
        bridge.start()
        first = ctl.wait(lambda s: True, timeout=15)
        assert any(e["code"] == "vehicleNoPose" for e in first["errors"])   # zeros are not a position
        time.sleep(0.5)
        assert ctl.latest()["safetyState"] == "stopped"                     # still none while it sends zeros
        rover.origin = True                                                  # the autopilot's EKF has an origin
        ok = ctl.wait(lambda s: s["safetyState"] == "ok", timeout=15)
        assert math.hypot(ok["pose"]["x"], ok["pose"]["y"]) < 0.5
        assert ok["pose"]["heading"] == pytest.approx(0.0, abs=1e-6)        # bearing 90 deg = site +x
        ctl.send("order", wire.order("yard", "dz1", 1, "m1", 0, "cut", "cut", [[20, -5], [60, -5], [60, 5], [20, 5]],
                                     {}, {"maxSpeed": 1.5}))
        done = ctl.wait(lambda s: s["orderId"] == "m1" and s["progress"] == 1.0, timeout=20)
        assert math.hypot(done["pose"]["x"] - 60, done["pose"]["y"]) < 0.1
        cmds = [i.command for i in rover.items]
        assert cmds[0] == 16 and cmds[1] == 178 and rover.items[1].param2 == pytest.approx(1.5)
        assert (400, 1.0, 0.0) in rover.commands and 10 in rover.modes          # armed, AUTO
        time.sleep(0.3)
        assert rover.mode == 4                                                   # HOLD when done
        # a refused mission upload: the order is never reported done on the back of the last mission,
        # the vehicle is never put in AUTO to re-run it, and the next order works again
        rover.reject, n_modes = True, len(rover.modes)
        far = [[60, -5], [160, -5], [160, 5], [60, 5]]
        ctl.send("order", wire.order("yard", "dz1", 2, "bad", 0, "cut", "cut", far, {}, {}))
        refused = ctl.wait(lambda s: s["orderId"] == "bad" and s["errors"], timeout=10)
        assert refused["safetyState"] == "stopped"
        assert wire.error("orderFailed", "vehicleMissionRejected") in refused["errors"]
        free = ctl.wait(lambda s: s["orderId"] is None, timeout=5)         # the order is dropped: free again
        assert free["safetyState"] == "ok" and free["errors"] == []
        time.sleep(1.0)
        assert all(s["progress"] < 1.0 for s in ctl.states if s["orderId"] == "bad")
        assert 10 not in rover.modes[n_modes:]
        assert math.hypot(ctl.latest()["pose"]["x"] - 60, ctl.latest()["pose"]["y"]) < 0.1    # has not moved
        rover.reject = False
        ctl.send("order", wire.order("yard", "dz1", 3, "m2", 0, "cut", "cut", far, {}, {}))
        again = ctl.wait(lambda s: s["orderId"] == "m2" and s["progress"] == 1.0, timeout=20)
        assert again["safetyState"] == "ok" and abs(again["pose"]["x"] - 160) < 0.1
        ctl.send("order", wire.order("yard", "dz1", 4, "m3", 0, "cut", "cut", [[160, -5], [60, -5], [60, 5], [160, 5]],
                                     {}, {}))
        ctl.wait(lambda s: s["orderId"] == "m3", timeout=10)
        ctl.bus.pub(wire.topic("yard", None, "estop"), {"pressed": True}, qos=1)
        ctl.wait(lambda s: s["safetyState"] == "stopped", timeout=10)
        time.sleep(0.5)
        assert (400, 0.0, 21196.0) in rover.commands                            # forced disarm
        ctl.bus.pub(wire.topic("yard", None, "estop"), {"pressed": False}, qos=1)
        reset = ctl.wait(lambda s: any(e["code"] == "estopReset" for e in s["errors"]), timeout=10)
        assert reset["safetyState"] == "stopped"
        rover.beating = False
        lost = ctl.wait(lambda s: any(e["code"] == "vehicleLinkLost" for e in s["errors"]), timeout=10)
        assert lost["safetyState"] == "stopped"
    finally:
        ctl.close(bridge)
        rover.running = False
        rover.join(5)


FENCED = """loam 0.0.2
site yard { crs "LOCAL:ft"; units ft; design "d.xml"; boundary rect (0, 0) (200, 100); grid 2 ft }
probe p { kind tethered_uav; rate 5 min; accuracy 0.05 ft; coverage site; stale_after 15 min }
machine dz1 { class dozer; blade 10 ft; max_speed 3 mph }
zone pad { role cut; area rect (0, 0) (200, 100); tolerance ±0.1 ft }
zone well { role exclusion; area rect (90, 40) (110, 60); buffer 5 ft }
stockpile sp { area rect (210, 0) (240, 30); available 0 yd3 }
task cut_pad { op cut; zone pad; by dz1; spoil -> sp; until in_tolerance(pad) }
"""


def test_offset_polygon():
    """A polygon grows by the same distance on every side, whichever way its corners are listed, including at an
    inside corner; a negative distance shrinks it.
    """
    from loam.geo import offset_polygon
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    grown = [(-2, -2), (12, -2), (12, 12), (-2, 12)]
    close = np.testing.assert_allclose
    close(offset_polygon(square, 2.0), grown, atol=1e-9)
    close(offset_polygon(square[::-1], 2.0), grown[::-1], atol=1e-9)                     # clockwise
    close(offset_polygon(square, -2.0), [(2, 2), (8, 2), (8, 8), (2, 8)], atol=1e-9)
    ell = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]                           # L-shaped
    close(offset_polygon(ell, 1.0), [(-1, -1), (11, -1), (11, 5), (5, 5), (5, 11), (-1, 11)], atol=1e-9)


def test_site_limits_become_vehicle_fences():
    """The vehicle's inclusion fence is the hull of everything the program names (here the boundary and a stockpile
    outside it), grown by a margin; each exclusion zone, grown by its buffer, is an exclusion fence. Converted
    back from latitude and longitude the corners are where the program put them. No areas, no fence.
    """
    from loam.geo import convex_hull
    from loam.runtime.mavlink import CMD_FENCE_INSIDE, CMD_FENCE_OUTSIDE, fence_items
    model, diags = check(parse(FENCED))
    assert not has_errors(diags)
    frame = Frame(0.3048, 30.0, 34.4459, -85.7197)                                        # a site turned 30 degrees

    def corners(items):
        """The fence corners back in site coordinates, with each item's command and polygon size."""
        x, y = frame.wgs84_to_local([i[7] / 1e7 for i in items], [i[8] / 1e7 for i in items])
        return [(i[2], i[3]) for i in items], np.column_stack([x, y])

    kinds, xy = corners(fence_items(model, frame, margin_m=0.0))
    assert kinds == [(CMD_FENCE_INSIDE, 5)] * 5 + [(CMD_FENCE_OUTSIDE, 4)] * 4
    want = [(0, 0), (240, 0), (240, 30), (200, 100), (0, 100), (85, 35), (115, 35), (115, 65), (85, 65)]
    assert xy == pytest.approx(np.array(want, float), abs=0.05)                           # 1e-7 degrees is 1 cm
    only_pad, _ = check(parse(FENCED.replace("stockpile sp { area rect (210, 0) (240, 30); available 0 yd3 }\n", "")
                              .replace("spoil -> sp; ", "")))
    kinds, xy = corners(fence_items(only_pad, frame))                                     # the default 10 m margin
    m = 10 / 0.3048
    assert xy[:4] == pytest.approx(np.array([(-m, -m), (200 + m, -m), (200 + m, 100 + m), (-m, 100 + m)]), abs=0.05)
    assert [i[0] for i in fence_items(model, frame)] == list(range(9))                    # items are numbered in order
    assert convex_hull([(0, 0), (4, 0), (4, 4), (0, 4), (2, 2), (2, 0), (4, 0)]) == [(0, 0), (4, 0), (4, 4), (0, 4)]
    assert convex_hull([(0, 0), (1, 1)]) == [(0, 0), (1, 1)]


def test_mavlink_setup_makes_the_vehicle_protect_itself(capsys):
    """Connecting sets the vehicle up: its failsafe watches the bridge's heartbeat, the fences are on board and
    switched on, and only parameters that differ are written (a second connection writes none). The bridge then
    sends a heartbeat every second, and reports a fence breach as a fault. Older parameter names are found; a
    vehicle that will not be set up is refused, with the link closed.
    """
    from loam.runtime.mavlink import VEHICLE_SETUP, Vehicle, fence_items
    model, _ = check(parse(FENCED))
    lat0, lon0 = 34.4459, -85.7197
    frame = Frame(0.3048, 0.0, lat0, lon0)
    fences = fence_items(model, frame)
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.start()
    try:
        v = Vehicle(f"udpin:127.0.0.1:{port}", frame, fences=fences)
        try:
            want = {names[0]: value for names, value, _ in VEHICLE_SETUP}
            assert rover.params == want and sorted(rover.sets) == sorted(n for n in want if n != "FS_ACTION")
            assert [(f.command, f.param1, f.x, f.y, f.mission_type) for f in rover.fence] == \
                [(i[2], i[3], i[7], i[8], 1) for i in fences]
            err = capsys.readouterr().err
            assert "FS_GCS_ENABLE: 0 -> 1" in err and "uploaded 2 fence polygon(s), 9 corners" in err
            end = time.time() + 3.2
            while time.time() < end:
                v.poll()
                time.sleep(0.05)
            beats = [t for sysid, t in rover.gcs_beats if sysid == 254]
            assert len(beats) >= 4 and np.diff(beats).max() < 1.5
            assert v.fault is None
            rover.breach = True
            end = time.time() + 2
            while v.fault is None and time.time() < end:
                v.poll()
                time.sleep(0.05)
            assert v.fault == "vehicleFenceBreach"
        finally:
            v.close()
        rover.sets.clear()
        Vehicle(f"udpin:127.0.0.1:{port}", frame, fences=fences).close()                  # nothing left to change
        assert rover.sets == []
        Vehicle(f"udpin:127.0.0.1:{port}", frame, fences=[]).close()                      # a site without limits:
        assert rover.sets == []                                                           # failsafe only, no upload
        rover.params["FS_TIMEOUT"] = rover.params.pop("FS_GCS_TIMEOUT") + 2               # firmware before 4.4
        rover.params["SYSID_MYGCS"] = rover.params.pop("MAV_GCS_SYSID") + 1
        Vehicle(f"udpin:127.0.0.1:{port}", frame, fences=fences).close()
        assert rover.params["FS_TIMEOUT"] == 3 and rover.params["SYSID_MYGCS"] == 254
        del rover.params["FENCE_ACTION"]                                                  # not a Rover at all
        with pytest.raises(ValueError, match="FENCE_ACTION"):
            Vehicle(f"udpin:127.0.0.1:{port}", frame, fences=fences)
    finally:
        rover.running = False
        rover.join(5)


def test_mavlink_no_vehicle_closes_the_link():
    """If no vehicle answers, the bridge says so and leaves no socket open."""
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import Vehicle
    with pytest.raises(ValueError, match="no MAVLink heartbeat"):
        Vehicle(f"udpin:127.0.0.1:{free_port()}", Frame(1.0), timeout=0.5)
    # the leak check (conftest) fails this test if the UDP socket was left open


# -- OPERA without ROS ------------------------------------------------------------------------------------------
# A small excavator arm as a URDF: swing, boom (5 m), arm (2.8 m), bucket (1.2 m).
ARM = """<robot name="arm">
 <link name="base_link"/><link name="body"/><link name="boom"/><link name="arm"/><link name="bucket"/>
 <link name="tip"/>
 <joint name="swing_joint" type="continuous"><parent link="base_link"/><child link="body"/>
  <origin xyz="0 0 1" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
 <joint name="boom_joint" type="revolute"><parent link="body"/><child link="boom"/>
  <origin xyz="0.5 0 0.5" rpy="0 0.6 0"/><axis xyz="0 1 0"/><limit lower="-1.2" upper="0.8" velocity="1"/></joint>
 <joint name="arm_joint" type="revolute"><parent link="boom"/><child link="arm"/>
  <origin xyz="5 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="0.3" upper="2.6" velocity="1"/></joint>
 <joint name="bucket_joint" type="revolute"><parent link="arm"/><child link="bucket"/>
  <origin xyz="2.8 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-1.0" upper="2.0" velocity="1"/></joint>
 <joint name="tip_joint" type="fixed"><parent link="bucket"/><child link="tip"/>
  <origin xyz="1.2 0 0" rpy="0 0 0"/></joint>
</robot>"""


def test_urdf_kinematics_and_ik():
    """The arm's geometry is read from a URDF; the solver puts the bucket tip on reachable targets within the joint
    limits and returns None for one out of reach.
    """
    from loam.runtime.opera import Chain, ExcavatorIK
    c = Chain(ARM, tip="tip")
    assert c.moving == ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
    np.testing.assert_allclose(c.tip({"boom_joint": -0.6}), [0.5 + 9.0, 0, 1.5], atol=1e-9)   # arm level
    ik = ExcavatorIK(c, pivot="bucket_joint")
    seed = {"swing_joint": 0.0, "boom_joint": 0.0, "arm_joint": 1.2, "bucket_joint": 0.5}
    for target in ([6.0, 1.0, -0.5], [4.5, -2.0, 0.0], [7.5, 0.0, -1.5]):
        q = ik.solve_any(np.array(target), seed, -1.2)
        assert q is not None and np.linalg.norm(c.tip(q) - target) < 1e-4
        for n in ("boom_joint", "arm_joint", "bucket_joint"):
            lo, hi = c.limits(n)
            assert lo - 1e-9 <= q[n] <= hi + 1e-9
    assert ik.solve_any(np.array([30.0, 0, 0]), seed, -1.2) is None                       # out of reach
    with pytest.raises(ValueError, match="no chain"):
        Chain(ARM, tip="nowhere")


def test_map_frame_round_trip():
    """Site to map and back gives the same point, and distances scale by the unit (feet to metres)."""
    from loam.runtime.opera import MapFrame
    f = MapFrame(0.3048, (100.0, -50.0, 12.0), 30.0)
    p = f.to_map(10.0, 20.0, 5.0)
    np.testing.assert_allclose(f.to_site(p), [10.0, 20.0, 5.0], atol=1e-12)
    np.testing.assert_allclose(np.hypot(*(p[:2] - [100, -50])), math.hypot(10, 20) * 0.3048)


def bare(cls, **attrs):
    """An adapter object without its ROS connections, for its planning logic."""
    o = cls.__new__(cls)
    o.__dict__.update(attrs)
    return o


def test_carrier_splits_work_lines_travel_and_dump():
    """A carrier's order becomes travel, a work line inside the window, a haul that ends with a dump, and the return.
    Back-and-forth roller passes are all work lines.
    """
    from loam.runtime.opera import Carrier, MapFrame
    f = MapFrame(1.0)
    c = bare(Carrier, frame=f, dumper=object(), goal=None, lock=threading.Lock(), state="idle")
    win = [[60, 25], [100, 25], [100, 35], [60, 35]]
    order = {"window": win, "deposit": {"window": [[60, 60], [70, 60], [70, 70], [60, 70]]}}
    c.start([(0, 0), (60, 30), (100, 30), (65, 65), (100, 30)], order, 0.8)
    modes = [(s["mode"], len(s["poses"]), s["dump"]) for s in c.steps]
    assert modes == [("nav", 1, False), ("line", 1, False), ("nav", 1, True), ("nav", 1, False)]
    np.testing.assert_allclose(c.steps[1]["start"], [60, 30, 0])                    # the line's own start
    assert c.total == 5 and c.state == "paused" and c.max_speed == 0.8
    assert c.fraction() == 0.0
    c.k, c.left = 2, []                              # two steps done and the third's pose reached
    assert c.fraction() == pytest.approx(3 / 5)
    c.state = "done"
    assert c.fraction() == 1.0 and c.done()
    c2 = bare(Carrier, frame=f, dumper=None, goal=None, lock=threading.Lock(), state="idle")
    c2.start([(70, 30), (90, 30), (70, 30), (90, 30)], {"window": win}, None)       # compaction passes
    assert [s["mode"] for s in c2.steps] == ["line", "line", "line"]


def test_excavator_plan_follows_the_target_and_never_goes_below_design(tmp_path):
    """The planned bucket-tip points lie on the target surface (checked by forward kinematics, to a millimetre), the
    target is never below design, and a relative target without a survey is refused.
    """
    from loam.grid import SiteGrid
    from loam.runtime.opera import Chain, Excavator, ExcavatorIK, MapFrame
    m = load(EXAMPLE)
    g = SiteGrid(m, EXAMPLES)
    design = g.surface(m.site["design"], allow_gaps=True)
    j, i = np.argwhere(np.isfinite(design))[len(np.argwhere(np.isfinite(design))) // 2]
    x0, y0 = float(g.xs[i]), float(g.ys[j])
    u = 0.3048 if m.site.get("units", "ft") in ("ft", "usft") else 1.0
    f = MapFrame(u)
    odom = types.SimpleNamespace(pose=(x0, y0, 0.0), z=float(design[j, i]))
    ex = bare(Excavator, frame=f, g=g, design=design, ground=design + 1.0, odom=odom, step=0.1 / u, lift=0.6,
              scale=0.5, vmax={}, q={}, ik=ExcavatorIK(Chain(ARM, tip="tip")), urdf=ARM)
    win = [[x0 + 16, y0 - 3], [x0 + 22, y0 - 3], [x0 + 22, y0 + 3], [x0 + 16, y0 + 3]]
    surf, mk = ex.target_surface({"window": win, "target": {"offset": -0.5}})
    assert mk.sum() > 10
    assert (surf[mk] >= design[mk] - 1e-12).all()                         # 1 ft above design - 0.5 m: clipped
    pts, times, targets = ex.plan({"window": win, "target": {"surface": "design"}})
    assert len(pts) == len(times) == len(targets) and np.all(np.diff(times) > 0)
    on = [(q, t) for q, t in zip(pts, targets, strict=True) if t is not None]
    assert len(on) > 20
    for q, (sx, sy, h) in on:
        tip = ex.ik.c.tip(q)
        np.testing.assert_allclose([x0 + tip[0] / u, y0 + tip[1] / u, odom.z + tip[2] / u], [sx, sy, h], atol=1e-3)
    with pytest.raises(ValueError, match="noTerrain"):
        bare(Excavator, frame=f, g=g, design=design, ground=None).target_surface(
            {"window": win, "target": {"pattern": "x", "shape": [1, 1]}})


# -- monitor and resume ----------------------------------------------------------------------------------------
def test_monitor_serves_and_shuts_down_with_a_live_connection(mqtt_broker):
    """The dashboard serves metrics and a live stream, and still shuts down cleanly while a browser is connected."""
    from loam import monitor
    m = load(EXAMPLE)
    port = free_port()
    t = threading.Thread(target=monitor.serve, args=(m, EXAMPLES, mqtt_broker, port, None, "127.0.0.1"))
    t.start()
    pub = Bus(mqtt_broker, "monitor-test").start([])
    try:
        for _ in range(50):
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/metrics", timeout=2) as r:
                    assert "loam_messages_total" in r.read().decode()
                break
            except OSError:
                time.sleep(0.1)
        live = urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=5)
        first = live.readline()
        assert first.startswith(b"data: ") and json.loads(first[6:])
        pub.pub("sim/pad_north/end", {}, qos=1)
        t.join(10)
        assert not t.is_alive()
        rest = live.read()                                                      # the stream ends: not a hang
        assert rest is not None
        live.close()
    finally:
        pub.stop()


def test_resume_command(mqtt_broker, capsys):
    """`loam resume` sends the site-wide resume, or one machine's, with the reason given; an unknown machine is an
    error.
    """
    m = load(EXAMPLE)
    site, name = m.site["name"], next(iter(m.machines))
    sub = Bus(mqtt_broker, "resume-test").start([f"loam/v0/{site}/#"])
    try:
        main(["resume", EXAMPLE, "--broker", mqtt_broker, "--json"])
        main(["resume", EXAMPLE, "--broker", mqtt_broker, "--machine", name, "--reason", "checked", "--json"])
        got = sub.drain(1.0)
        assert (f"loam/v0/{site}/safety", "resume") in [(t, x.get("command")) for t, x in got]
        act = [x for t, x in got if t.endswith("/instantAction")]
        assert act and act[0]["action"] == "resume" and act[0]["reason"] == "checked"
        first, _ = json.JSONDecoder().raw_decode(capsys.readouterr().out)
        assert first["resumed"] == "site"
        with pytest.raises(SystemExit):
            main(["resume", EXAMPLE, "--broker", mqtt_broker, "--machine", "nobody"])
    finally:
        sub.stop()


def test_sigterm_still_holds_the_vehicle(mqtt_broker, tmp_path):
    """A service manager stops a bridge with SIGTERM: it must still hold the vehicle and say it is offline."""
    pytest.importorskip("pymavlink")
    import signal
    import subprocess
    import sys

    from test_bridges import SRC, Controller
    (tmp_path / "s.loam").write_text(SRC)
    lat0, lon0 = 34.4459, -85.7197
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.origin = True
    rover.start()
    ctl = Controller(mqtt_broker, "yard", "dz1")
    conn = Bus(mqtt_broker, "conn-watch").start(["loam/v0/yard/dz1/connection"])
    proc = subprocess.Popen([sys.executable, "-m", "loam.cli", "mavlink", str(tmp_path / "s.loam"), "--machine", "dz1",
                             "--vehicle", f"udpin:127.0.0.1:{port}", "--broker", mqtt_broker, "--min-fix", "3",
                             "--origin", f"{lat0},{lon0}"], stderr=subprocess.PIPE, text=True)
    try:
        ctl.wait(lambda s: s["safetyState"] == "ok", timeout=20)
        rover.items = []
        ctl.send("order", wire.order("yard", "dz1", 1, "t1", 0, "cut", "cut",
                                     [[20, -5], [900, -5], [900, 5], [20, 5]], {}, {}))
        deadline = time.time() + 10
        while rover.mode != 10 and time.time() < deadline:      # driving in AUTO
            time.sleep(0.05)
        assert rover.mode == 10
        rover.items = []                                          # stop reaching waypoints: it is mid-pass
        proc.send_signal(signal.SIGTERM)
        _, err = proc.communicate(timeout=10)
        assert proc.returncode == 128 + signal.SIGTERM, err
        assert "Traceback" not in err
        time.sleep(0.3)
        assert rover.mode == 4                                    # HOLD: not left driving
        states = [m.get("connectionState") for _, m in conn.drain(1.0)]
        assert "offline" in states
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        conn.stop()
        ctl.close()
        rover.running = False
        rover.join(5)


def test_bridge_state_position_and_ok_come_from_one_reading():
    """With ROS the position arrives on another thread. The state must never say "ok" while still carrying
    the placeholder position from before the first reading."""
    from types import SimpleNamespace

    from loam.runtime.bridge import bridge_state
    from loam.runtime.bus import Clock
    ms = machine()
    ms.last_hb = time.time()
    vehicle = SimpleNamespace(pose=None, fault=None, order_error=None, blocked=None, fix_ok=True, fix_name="rtk-fixed")
    none = bridge_state(ms, vehicle, Clock(), 0.0)
    assert none["safetyState"] == "stopped" and wire.error("vehicleNoPose") in none["errors"]
    vehicle.pose = (12.5, 7.25, 1.0)                 # arrives after the loop's last look at it
    st = bridge_state(ms, vehicle, Clock(), 0.0)
    assert st["safetyState"] == "ok" and st["pose"] == {"x": 12.5, "y": 7.25, "heading": 1.0}
