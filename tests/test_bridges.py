"""Vehicle bridges: VDA 5050 3.0 robots and MAVLink (ArduPilot Rover) vehicles as Loam machines.

The VDA 5050 robot here is a small protocol-faithful stand-in. Every message the
bridge sends it is checked against the official VDA 5050 3.0 JSON schemas
(tests/vda5050, MIT), and so is every state it sends back. The MAVLink test
runs real ArduPilot Rover SITL and is skipped when no `ardurover` binary is
found (set ARDUROVER=/path/to/ardurover).
"""
import json
import math
import os
import shutil
import subprocess
import threading
import time

import numpy as np
import pytest

from loam import wire
from loam.checker import check, has_errors
from loam.geo import Frame
from loam.parser import parse

pytest.importorskip("paho.mqtt")
from loam.runtime.bus import Bus
from loam.runtime.vda5050 import iso, run_vda5050, vda_order

HERE = os.path.dirname(os.path.abspath(__file__))
FT = 0.3048
SRC = """loam 0.0.2
site yard { crs "LOCAL:ft"; units ft; design "d.xml"; boundary rect (0, 0) (200, 100); grid 2 ft }
probe p { kind static; sensor lidar; rate 5 min; accuracy 0.05 ft }
machine dz1 { class dozer; blade 10 ft; max_speed 4 mph }
zone pad { role cut; area rect (20, 20) (120, 80); tolerance ±0.1 ft }
task cut { op cut; zone pad; by class dozer; spoil -> pad; until residual(pad).high < 0.1 ft }
"""


@pytest.fixture(scope="module")
def model():
    m, d = check(parse(SRC))
    assert not has_errors(d), [x.msg for x in d]
    return m


def schemas():
    jsonschema = pytest.importorskip("jsonschema")
    out = {}
    for n in ("order", "instantActions", "state", "connection"):
        with open(os.path.join(HERE, "vda5050", f"{n}.schema"), encoding="utf-8") as f:
            out[n] = jsonschema.Draft202012Validator(json.load(f), format_checker=jsonschema.FormatChecker())
    return out


# -- a VDA 5050 robot ------------------------------------------------------------------------------
class VdaRobot(threading.Thread):
    """Drives node to node at the edge speed; obeys startPause/stopPause/cancelOrder."""

    def __init__(self, broker, map_id, x=0.0, y=0.0, speed=2.0):
        super().__init__(daemon=True)
        self.v = schemas()
        self.base = "uagv/v3/acme/R1"
        self.map_id, self.x, self.y, self.theta, self.speed = map_id, x, y, 0.0, speed
        self.nodes, self.edges, self.actions, self.instant = [], [], [], []
        self.order_id, self.last_node, self.last_seq = "", "", 0
        self.paused = self.hw_estop = False
        self.problems, self.received = [], []
        self.header, self.running = 0, True
        will = (f"{self.base}/connection", self.msg(connectionState="CONNECTION_BROKEN"))
        self.bus = Bus(broker, "vda-robot", will)
        self.bus.start([f"{self.base}/order", f"{self.base}/instantActions"])
        self.send("connection", self.msg(connectionState="ONLINE"), retain=True)

    def msg(self, **body):
        self.header += 1
        return {"headerId": self.header, "timestamp": iso(), "version": "3.0.0", "manufacturer": "acme",
                "serialNumber": "R1", **body}

    def check(self, kind, m):
        for e in self.v[kind].iter_errors(m):
            self.problems.append(f"{kind}: {e.json_path}: {e.message}")

    def send(self, kind, m, retain=False):
        self.check(kind, m)
        self.bus.pub(f"{self.base}/{kind}", m, retain=retain, qos=1)

    def state(self):
        return self.msg(
            orderId=self.order_id, orderUpdateId=0, lastNodeId=self.last_node, lastNodeSequenceId=self.last_seq,
            nodeStates=[{k: n[k] for k in ("nodeId", "sequenceId", "released")} for n in self.nodes],
            edgeStates=[{k: e[k] for k in ("edgeId", "sequenceId", "released")} for e in self.edges],
            mobileRobotPosition={"x": self.x, "y": self.y, "theta": self.theta, "mapId": self.map_id,
                                 "localized": True},
            driving=bool(self.nodes) and not self.paused, paused=self.paused, actionStates=self.actions,
            instantActionStates=self.instant, powerSupply={"stateOfCharge": 80.0, "charging": False},
            operatingMode="AUTOMATIC", errors=[],
            safetyState={"activeEmergencyStop": "MANUAL" if self.hw_estop else "NONE", "fieldViolation": False})

    def run(self):
        last = time.time()
        while self.running:
            for topic, m in self.bus.drain(0.02):
                kind = topic.rsplit("/", 1)[-1]
                self.check(kind, m)
                self.received.append((kind, m))
                if kind == "order":
                    self.order_id = m["orderId"]
                    first, *rest = m["nodes"]
                    self.last_node, self.last_seq = first["nodeId"], first["sequenceId"]
                    self.nodes, self.edges = rest, list(m["edges"])
                    self.actions = [{"actionId": a["actionId"], "actionType": a["actionType"],
                                     "actionStatus": "WAITING"} for e in m["edges"] for a in e["actions"]]
                else:
                    for a in m["actions"]:
                        self.instant.append({"actionId": a["actionId"], "actionType": a["actionType"],
                                             "actionStatus": "FINISHED"})
                        if a["actionType"] in ("startPause", "stopPause"):
                            self.paused = a["actionType"] == "startPause"
                        elif a["actionType"] == "cancelOrder":
                            self.nodes, self.edges = [], []
                            for s in self.actions:
                                if s["actionStatus"] != "FINISHED":
                                    s["actionStatus"] = "FAILED"
            dt, last = time.time() - last, time.time()
            if self.nodes and not self.paused and not self.hw_estop:
                n, e = self.nodes[0], self.edges[0]
                for s in self.actions:
                    if any(a["actionId"] == s["actionId"] for a in e["actions"]):
                        s["actionStatus"] = "RUNNING"
                tx, ty = n["nodePosition"]["x"], n["nodePosition"]["y"]
                d = math.hypot(tx - self.x, ty - self.y)
                step = min(d, e.get("maximumSpeed", self.speed) * dt)
                if d > 1e-9:
                    self.theta = math.atan2(ty - self.y, tx - self.x)
                    self.x += (tx - self.x) * step / d
                    self.y += (ty - self.y) * step / d
                if d - step < 1e-6:
                    for s in self.actions:
                        if any(a["actionId"] == s["actionId"] for a in e["actions"]):
                            s["actionStatus"] = "FINISHED"
                    self.last_node, self.last_seq = n["nodeId"], n["sequenceId"]
                    self.nodes, self.edges = self.nodes[1:], self.edges[1:]
            self.send("state", self.state())
            time.sleep(0.05)
        self.bus.stop()


class Controller:
    """Plays the Loam controller's part of the Wire: heartbeats, orders, actions, e-stop."""

    def __init__(self, broker, site, machine):
        self.site, self.machine, self.seq = site, machine, 0
        self.bus = Bus(broker, "test-controller")
        self.bus.start([wire.topic(site, machine, "state")])
        self.states, self.pending = [], []
        self.stop_hb = threading.Event()
        threading.Thread(target=self._hb, daemon=True).start()

    def _hb(self):
        while not self.stop_hb.is_set():
            self.seq += 1
            self.bus.pub(wire.topic(self.site, None, "safety"), wire.safety(self.site, self.seq))
            self.stop_hb.wait(0.5)

    def close(self, *bridges):
        """End the bridges (the site `end` message), wait for them, and leave no client behind."""
        self.stop_hb.set()
        self.bus.pub(f"sim/{self.site}/end", {}, qos=1)
        for b in bridges:
            b.join(10)
            assert not b.is_alive(), "bridge did not stop"
        self.bus.stop()

    def send(self, kind, m, topic_machine=True):
        self.bus.pub(wire.topic(self.site, self.machine if topic_machine else None, kind), m, qos=1)

    def latest(self, settle=0.5):
        """The newest state, after `settle` seconds of draining (older ones are consumed)."""
        end = time.time() + settle
        while time.time() < end:
            for _, m in self.bus.drain(0.05):
                self.states.append(m)
                self.pending.append(m)
        m, self.pending = (self.pending[-1] if self.pending else self.states[-1]), []
        return m

    def wait(self, pred, timeout=20.0):
        """The next state (in arrival order) that satisfies `pred`; earlier ones are consumed."""
        end = time.time() + timeout
        while time.time() < end:
            while self.pending:
                m = self.pending.pop(0)
                if pred(m):
                    return m
            for _, m in self.bus.drain(0.05):
                self.states.append(m)
                self.pending.append(m)
        raise AssertionError(f"timed out; last state {self.states[-1] if self.states else None}")


def loam_order(site, machine, oid, window, speed=None):
    return wire.order(site, machine, 1, oid, 0, "cut", "cut", window, {}, {"maxSpeed": speed} if speed else {})


def test_vda_order_matches_the_schema(model):
    v = schemas()
    o = loam_order("yard", "dz1", "o1", [(20, 40), (60, 40), (60, 50), (20, 50)], 1.5)
    hdr = {"headerId": 1, "timestamp": iso(), "version": "3.0.0", "manufacturer": "a", "serialNumber": "b"}
    m = vda_order(lambda **b: {**hdr, **b}, o, [(0, 0), (20, 45), (60, 45)], "yard", FT, 1.5)
    assert not list(v["order"].iter_errors(m))
    assert [n["sequenceId"] for n in m["nodes"]] == [0, 2, 4] and [e["sequenceId"] for e in m["edges"]] == [1, 3]
    assert m["nodes"][2]["nodePosition"] == {"x": round(60 * FT, 4), "y": round(45 * FT, 4), "mapId": "yard"}
    work = m["edges"][1]["actions"][0]
    assert work["actionType"] == "loam.cut" and {p["key"] for p in work["actionParameters"]} >= {"loamOrderId",
                                                                                                   "window"}
    one = vda_order(lambda **b: {**hdr, **b}, o, [(5, 5)], "yard", FT, None)
    assert one["edges"] == [] and one["nodes"][0]["actions"][0]["actionType"] == "loam.cut"
    assert not list(v["order"].iter_errors(one))


def test_vda5050_robot_runs_loam_orders(model, mqtt_broker):
    robot = VdaRobot(mqtt_broker, "yard", speed=3.0)
    robot.start()
    ctl = Controller(mqtt_broker, "yard", "dz1")
    done = {}
    bridge = threading.Thread(target=lambda: done.setdefault("ms", run_vda5050(
        model, "dz1", mqtt_broker, "acme", "R1", max_wall_s=120)), daemon=True)
    bridge.start()
    try:
        ctl.wait(lambda m: m["pose"]["x"] == 0 and m["safetyState"] == "ok")
        # 1: a pass from the robot's spot through a 40 ft window, at 1.5 m/s
        ctl.send("order", loam_order("yard", "dz1", "o1", [(20, 40), (60, 40), (60, 50), (20, 50)], 1.5))
        mid = ctl.wait(lambda m: m["orderId"] == "o1" and 0 < m["progress"] < 1 and m["driving"])
        assert mid["safetyState"] == "ok"
        end = ctl.wait(lambda m: m["orderId"] == "o1" and m["progress"] == 1.0)
        assert math.hypot(end["pose"]["x"] - 60, end["pose"]["y"] - 45) < 0.01
        assert robot.x == pytest.approx(60 * FT, abs=1e-3)
        # 2: stop pauses it, resume goes on, the robot's own e-stop holds the machine
        ctl.send("order", loam_order("yard", "dz1", "o2", [(60, 40), (160, 40), (160, 50), (60, 50)]))
        ctl.wait(lambda m: m["orderId"] == "o2" and m["driving"])
        ctl.send("instantAction", wire.instant_action("yard", "dz1", 2, "stop", "test"))
        ctl.wait(lambda m: m["safetyState"] == "stopped")
        time.sleep(0.5)
        assert robot.paused
        x_paused = robot.x
        time.sleep(0.5)
        assert robot.x == x_paused
        ctl.send("instantAction", wire.instant_action("yard", "dz1", 3, "resume", "test"))
        ctl.wait(lambda m: m["driving"])
        time.sleep(0.3)
        assert not robot.paused and robot.x > x_paused
        robot.hw_estop = True
        st = ctl.wait(lambda m: m["safetyState"] == "stopped")
        assert {"code": "vehicleEstop", "detail": ""} in st["errors"]
        robot.hw_estop = False
        ctl.wait(lambda m: m["safetyState"] == "ok")
        # 3: the site e-stop cancels the order on the robot
        ctl.bus.pub(wire.topic("yard", None, "estop"), {"pressed": True, "wall": time.time()}, qos=1)
        ctl.wait(lambda m: m["safetyState"] == "stopped" and "stopLatencyS" in m)
        time.sleep(0.5)
        assert any(k == "instantActions" and a["actionType"] == "cancelOrder"
                   for k, m in robot.received for a in m.get("actions", []))
        assert robot.nodes == [] and robot.paused
        # 4: lost robot link is reported, not driven through
        robot.bus.sever()                                     # radio drop: the broker sends its will
        robot.running = False
        st = ctl.wait(lambda m: any(e["code"] == "vehicleLinkLost" for e in m["errors"]), timeout=10)
        assert st["safetyState"] == "stopped"
    finally:
        robot.running = False
        ctl.close(bridge)
        robot.join(5)
    assert done["ms"].name == "dz1"
    assert robot.problems == [], robot.problems[:5]
    kinds = {k for k, _ in robot.received}
    assert kinds == {"order", "instantActions"}


# -- MAVLink / ArduPilot Rover SITL ---------------------------------------------------------------
def test_mission_items_and_heading(model):
    from loam.runtime.mavlink import CMD_CHANGE_SPEED, CMD_WAYPOINT, bearing_to_heading, mission_items
    f = Frame(FT, 30.0, 34.4459, -85.7197)
    path = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0)]
    items = mission_items(path, f, 1.5)
    assert [i[2] for i in items] == [CMD_WAYPOINT, CMD_CHANGE_SPEED, CMD_WAYPOINT, CMD_WAYPOINT]
    assert [i[0] for i in items] == [0, 1, 2, 3] and items[1][4] == 1.5
    lat, lon = items[2][7] / 1e7, items[2][8] / 1e7
    x, y = f.wgs84_to_local(lat, lon)
    assert math.hypot(x - 100, y) < 0.06                       # 1e-7 degree steps are ~1.1 cm (0.036 ft)
    assert bearing_to_heading(30.0, f) == pytest.approx(math.pi / 2)        # along site +y
    assert bearing_to_heading(120.0, f) == pytest.approx(0.0, abs=1e-12)    # along site +x


ARDUROVER = os.environ.get("ARDUROVER") or shutil.which("ardurover") or next(
    (p for p in ("/opt/src/ardupilot/build/sitl/bin/ardurover",) if os.path.exists(p)), None)


@pytest.mark.slow
@pytest.mark.skipif(ARDUROVER is None, reason="needs an ArduPilot Rover SITL binary (ARDUROVER)")
def test_ardupilot_rover_sitl(model, mqtt_broker, tmp_path):
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import run_mavlink
    root = os.path.dirname(os.path.dirname(os.path.dirname(ARDUROVER)))
    parms = [os.path.join(root, "..", "Tools", "autotest", "default_params", p) for p in ("rover.parm",
                                                                                         "rover-skid.parm")]
    parms = [p for p in parms if os.path.exists(p)]
    lat0, lon0 = 34.4459, -85.7197
    sitl = subprocess.Popen([ARDUROVER, "--model", "rover-skid", "--home", f"{lat0},{lon0},200,0",
                             "--speedup", "1", "-I", "3", *(["--defaults", ",".join(parms)] if parms else [])],
                            cwd=tmp_path, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    frame = Frame(FT, 0.0, lat0, lon0)
    ctl = Controller(mqtt_broker, "yard", "dz1")
    bridges = []
    try:
        time.sleep(2)
        bridge = threading.Thread(target=run_mavlink, args=(model, "dz1", mqtt_broker, "tcp:127.0.0.1:5790",
                                                           frame, 3, False, 150), daemon=True)
        bridge.start()
        bridges.append(bridge)
        first = ctl.wait(lambda m: m["safetyState"] == "ok", timeout=90)       # EKF settled, 3D fix
        assert math.hypot(first["pose"]["x"], first["pose"]["y"]) < 5          # within 5 ft of home
        ctl.send("order", loam_order("yard", "dz1", "r1", [(20, -5), (60, -5), (60, 5), (20, 5)], 2.0))
        ctl.wait(lambda m: m["orderId"] == "r1" and m["driving"], timeout=30)
        end = ctl.wait(lambda m: m["orderId"] == "r1" and m["progress"] == 1.0, timeout=90)
        assert math.hypot(end["pose"]["x"] - 60, end["pose"]["y"]) < 7         # ArduPilot WP_RADIUS 2 m (6.6 ft)
        track = np.array([[s["pose"]["x"], s["pose"]["y"]] for s in ctl.states if s["orderId"] == "r1"])
        assert np.abs(track[track[:, 0] > 25, 1]).max() < 2                    # held the work line (0.6 ft seen)
        ctl.send("order", loam_order("yard", "dz1", "r2", [(60, -5), (160, -5), (160, 5), (60, 5)], 2.0))
        ctl.wait(lambda m: m["orderId"] == "r2" and m["driving"], timeout=30)
        ctl.send("instantAction", wire.instant_action("yard", "dz1", 2, "stop", "test"))
        stop = ctl.wait(lambda m: m["safetyState"] == "stopped")
        time.sleep(4)
        held = ctl.latest()
        assert math.hypot(held["pose"]["x"] - stop["pose"]["x"], held["pose"]["y"] - stop["pose"]["y"]) < 1
        ctl.bus.pub(wire.topic("yard", None, "estop"), {"pressed": True, "wall": time.time()}, qos=1)
        ctl.wait(lambda m: "stopLatencyS" in m)
    finally:
        ctl.close(*bridges)                                     # the bridge closes its MAVLink link first
        sitl.terminate()
        sitl.wait(10)


# -- RTKLIB stream -> loam nmea -> MQTT -----------------------------------------------------------
STR2STR = os.environ.get("STR2STR") or shutil.which("str2str") or next(
    (p for p in ("/opt/src/RTKLIB/app/consapp/str2str/gcc/str2str",) if os.path.exists(p)), None)


@pytest.mark.skipif(STR2STR is None, reason="needs RTKLIB str2str (STR2STR)")
def test_rtklib_stream_to_wire(model, mqtt_broker, tmp_path):
    import socket
    import sys

    from conftest import EXAMPLES  # noqa: F401  (conftest on the path)
    from test_field import _gga
    lat0, lon0 = 34.4459, -85.7197
    f = Frame(FT, 0.0, lat0, lon0)
    pts = [(10.0 * k, 5.0) for k in range(8)]
    lines = [_gga(*f.local_to_wgs84(x, y), q=4) for x, y in pts]

    def free():
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]
    src_port, out_port = free(), free()
    srv = socket.create_server(("127.0.0.1", src_port))
    (tmp_path / "s.loam").write_text(SRC)
    relay = subprocess.Popen([STR2STR, "-in", f"tcpcli://127.0.0.1:{src_port}", "-out", f"tcpsvr://:{out_port}"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctl = Controller(mqtt_broker, "yard", "dz1")
    rx = None
    try:
        srv.settimeout(10)
        feed, _ = srv.accept()                                  # str2str connected to the receiver
        time.sleep(0.3)
        rx = subprocess.Popen([sys.executable, "-m", "loam.cli", "nmea", str(tmp_path / "s.loam"),
                               f"tcp://127.0.0.1:{out_port}", "--machine", "dz1", "--broker", mqtt_broker,
                               "--origin", f"{lat0},{lon0}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(1.5)                                         # loam connected to str2str
        for ln in lines:
            feed.sendall((ln + "\r\n").encode())
            time.sleep(0.1)
        got = []
        while len(got) < len(pts):
            got.append(ctl.wait(lambda m: True, timeout=10))
        xy = np.array([[m["pose"]["x"], m["pose"]["y"]] for m in got])
        assert np.abs(xy - np.array(pts)).max() < 0.05          # 1e-5 arc-minute steps: ~2 cm
        assert all(m["safetyState"] == "ok" for m in got)       # quality 4 = RTK fixed
        feed.close()
    finally:
        ctl.close()
        relay.terminate()
        relay.wait(5)
        if rx is not None:
            rx.terminate()
            rx.communicate(timeout=5)
        srv.close()
