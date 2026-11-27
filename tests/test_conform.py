"""`loam conform`: the check a machine has to pass to be driven by Loam, run against a machine that keeps its side
of the wire (the MAVLink bridge with a fake ArduPilot), one that does not, and one that is not there."""
import json
import threading
import time

import pytest
from test_bridges import SRC
from test_safety import FakeRover, free_port

from loam import wire
from loam.checker import check
from loam.cli import main
from loam.conform import conform, malformed
from loam.geo import Frame
from loam.parser import parse
from loam.runtime.bus import Bus

WINDOW = (20.0, -5.0, 60.0, 5.0)


class Careless(threading.Thread):
    """A machine that talks well and listens badly: it announces itself, reports its state four times a second and
    works every order it is sent for eight seconds, the same one twice if asked. Stops, the e-stop and a silent
    controller mean nothing to it."""

    def __init__(self, broker):
        """Connect as machine dz1 of site yard and announce it online."""
        super().__init__(daemon=True)
        self.bus = Bus(broker, f"careless-{time.time()}").start([wire.topic("yard", "dz1", "order")])
        self.bus.pub(wire.topic("yard", "dz1", "connection"), wire.connection("yard", "dz1", 1, "online"),
                     retain=True, qos=1)
        self.running, self.order, self.began, self.runs = True, None, 0.0, 0

    def run(self):
        """Report, and work whatever arrives."""
        seq = 1
        while self.running:
            for _, msg in self.bus.drain(0.25):
                self.order, self.began = msg, time.time()
                self.runs += 1
            seq += 1
            frac = min((time.time() - self.began) / 8.0, 1.0) if self.order else 0.0
            self.bus.pub(wire.topic("yard", "dz1", "state"),
                         wire.state("yard", "dz1", seq, self.order["orderId"] if self.order else None, 20 + 40 * frac,
                                    0.0, 0.0, bool(self.order) and frac < 1.0, frac, ts=time.time()),
                         qos=1 if frac >= 1.0 else 0)
            if frac >= 1.0:
                self.order = None

    def close(self):
        """Stop, withdraw the retained announcement and disconnect."""
        self.running = False
        self.join(5)
        self.bus.clear(wire.topic("yard", "dz1", "connection"))
        time.sleep(0.2)
        self.bus.stop()


def test_a_bridged_vehicle_conforms(mqtt_broker):
    """The MAVLink bridge driving a fake ArduPilot passes every check: it announces itself, reports, works an
    order once and only once, stops on a stop, on a silent controller and on the e-stop, and stays stopped when the
    e-stop is released."""
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import run_mavlink
    model, _ = check(parse(SRC))
    lat0, lon0 = 34.4459, -85.7197
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.origin, rover.dwell = True, 6.0
    rover.start()
    bridge = threading.Thread(target=run_mavlink, args=(model, "dz1", mqtt_broker, f"udpin:127.0.0.1:{port}",
                                                       Frame(0.3048, 0.0, lat0, lon0), 6, False, 150), daemon=True)
    lines: list[str] = []
    try:
        bridge.start()
        r = conform(model, "dz1", mqtt_broker, WINDOW, timeout=40.0, log=lines.append)
    finally:
        end = Bus(mqtt_broker, f"end-{time.time()}").start([])
        end.pub("sim/yard/end", {}, qos=1, wait=2.0)
        bridge.join(15)
        end.clear(wire.topic("yard", "dz1", "connection"))
        time.sleep(0.2)
        end.stop()
        rover.running = False
        rover.join(5)
    assert not bridge.is_alive()
    assert r["failed"] == 0 and r["not_tested"] == 0 and r["passed"] == 8, lines
    assert [c["check"] for c in r["checks"]] == ["announces", "reports", "ready", "works", "no repeat", "stops",
                                                 "heartbeat", "e-stop"]
    assert (400, 0.0, 21196.0) in rover.commands           # the e-stop reached the vehicle as a forced disarm
    assert rover.mode == 4                                  # and it is left in HOLD
    with pytest.raises(ValueError, match="no machine"):
        conform(model, "dz9", mqtt_broker, WINDOW)


def test_a_machine_that_does_not_listen_fails(mqtt_broker):
    """A machine that reports well and ignores what it is told passes the first four checks and fails the rest,
    each with the reason."""
    model, _ = check(parse(SRC))
    m = Careless(mqtt_broker)
    m.start()
    try:
        r = conform(model, "dz1", mqtt_broker, WINDOW, timeout=20.0, log=lambda line: None)
    finally:
        m.close()
    got = {c["check"]: c for c in r["checks"]}
    assert [got[k]["ok"] for k in ("announces", "reports", "ready", "works")] == [True] * 4
    assert got["no repeat"]["ok"] is False and "second time" in got["no repeat"]["detail"]
    assert got["stops"]["ok"] is False and "still driving" in got["stops"]["detail"]
    assert got["heartbeat"]["ok"] is False and got["e-stop"]["ok"] is False
    assert r["passed"] == 4 and r["failed"] == 4 and m.runs == 5


def test_no_machine_and_the_command(mqtt_broker, tmp_path, capsys):
    """With nothing answering, the first two checks fail and the rest are not attempted; the command prints the
    result and exits with status 1."""
    (tmp_path / "yard.loam").write_text(SRC)
    with pytest.raises(SystemExit) as stop:
        main(["conform", str(tmp_path / "yard.loam"), "--machine", "dz1", "--window", "20,-5,60,5", "--broker",
              mqtt_broker, "--json"])
    assert stop.value.code == 1
    r = json.loads(capsys.readouterr().out)
    assert [(c["check"], c["ok"]) for c in r["checks"]] == [("announces", False), ("reports", False)]


def test_malformed_states_are_named():
    """What is wrong with a state message, in words."""
    good = wire.state("yard", "dz1", 1, None, 1.0, 2.0, 0.0, False, 0.0, ts=time.time())
    assert malformed(good) is None
    assert malformed("hello") == "not a JSON object"
    assert "'pose'" in malformed({**good, "pose": {"x": 1.0, "y": float("nan"), "heading": 0.0}})
    assert "'driving'" in malformed({k: v for k, v in good.items() if k != "driving"})
    assert "out of range" in malformed({**good, "progress": 1.5})
    assert "out of range" in malformed({**good, "safetyState": "fine"})


def test_nav2_is_the_ros_carrier_under_its_own_name(tmp_path, capsys):
    """`loam nav2` needs a ROS 2 environment, and says so where there is none."""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pass
    else:
        pytest.skip("ROS 2 is installed here; the vehicle test covers it")
    (tmp_path / "yard.loam").write_text(SRC)
    with pytest.raises(SystemExit):
        main(["nav2", str(tmp_path / "yard.loam"), "--machine", "dz1", "--namespace", "rover"])
    assert "ROS 2" in capsys.readouterr().err
