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
from loam.cli import load, main
from loam.geo import Frame

pytest.importorskip("paho.mqtt")
from loam.runtime.bus import Bus, tolerant, wait_clock
from loam.runtime.machine import MachineSim


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# -- e-stop latch ---------------------------------------------------------------------------------------
def machine() -> MachineSim:
    m = load(EXAMPLE)
    return MachineSim(m, next(iter(m.machines)), "localhost:1", (0.0, 0.0))


def test_estop_release_does_not_restart():
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
    origin), GPS, the mission handshake, modes, arming; in AUTO it reaches each waypoint in turn."""

    def __init__(self, port: int, lat0: float, lon0: float):
        super().__init__(daemon=True)
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
        self.lat, self.lon, self.origin = lat0, lon0, False
        self.armed, self.mode, self.items, self.reached = False, 4, [], 0
        self.commands, self.modes, self.running, self.beating = [], [], True, True
        self.reject = False                       # refuse the next mission upload (MAV_MISSION_ERROR)

    def send_state(self):
        mav = self.m.mav
        if self.beating:
            mav.heartbeat_send(10, 3, (128 if self.armed else 0) | 1, self.mode, 4)
        lat, lon = (self.lat, self.lon) if self.origin else (0.0, 0.0)
        mav.global_position_int_send(0, round(lat * 1e7), round(lon * 1e7), 0, 0, 0, 0, 0, 9000)
        mav.gps_raw_int_send(0, 6, round(self.lat * 1e7), round(self.lon * 1e7), 0, 50, 50, 0, 0, 20)

    def run(self):
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
        t = msg.get_type()
        if t == "MISSION_COUNT" and self.reject:
            self.m.mav.mission_ack_send(255, 0, 1, 0)                 # the old mission stays on board
        elif t == "MISSION_COUNT":
            self.items, self.reached = [None] * msg.count, 0
            self.m.mav.mission_request_int_send(255, 0, 0, 0)
        elif t == "MISSION_ITEM_INT":
            self.items[msg.seq] = msg
            if msg.seq + 1 < len(self.items):
                self.m.mav.mission_request_int_send(255, 0, msg.seq + 1, 0)
            else:
                self.m.mav.mission_ack_send(255, 0, 0, 0)
        elif t == "COMMAND_LONG":
            self.commands.append((msg.command, msg.param1, msg.param2))
            if msg.command == 400:
                self.armed = msg.param1 == 1
        elif t == "SET_MODE":
            self.mode = msg.custom_mode
            self.modes.append(msg.custom_mode)


def test_mavlink_bridge_against_fake_ardupilot(mqtt_broker):
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


def test_mavlink_no_vehicle_closes_the_link():
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import Vehicle
    with pytest.raises(ValueError, match="no MAVLink heartbeat"):
        Vehicle(f"udpin:127.0.0.1:{free_port()}", Frame(1.0), timeout=0.5)
    # the leak check (conftest) fails this test if the UDP socket was left open


# -- OPERA without ROS ------------------------------------------------------------------------------------------
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
    vehicle = SimpleNamespace(pose=None, fault=None, order_error=None, fix_ok=True, fix_name="rtk-fixed")
    none = bridge_state(ms, vehicle, Clock(), 0.0)
    assert none["safetyState"] == "stopped" and wire.error("vehicleNoPose") in none["errors"]
    vehicle.pose = (12.5, 7.25, 1.0)                 # arrives after the loop's last look at it
    st = bridge_state(ms, vehicle, Clock(), 0.0)
    assert st["safetyState"] == "ok" and st["pose"] == {"x": 12.5, "y": 7.25, "heading": 1.0}
