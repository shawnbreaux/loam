"""A blade held to the next-pass surface by the bridge, for a machine with nobody aboard (`loam.runtime.blade`,
`loam mavlink --blade-servo`)."""
import math
import threading
import time
from itertools import pairwise

import numpy as np
import pytest
from test_bridges import Controller
from test_safety import FakeRover, free_port

from loam import wire
from loam.checker import check, has_errors
from loam.cli import main
from loam.geo import Frame
from loam.grid import SiteGrid
from loam.landxml import write_grid_tin
from loam.parser import parse
from loam.runtime.blade import DEADBAND_FT, Blade

# A 200 by 100 ft yard. The ground lies flat at 101 ft; the pad is to come down to 100 ft, half a foot a pass.
SRC = """loam 0.0.2
site yard { crs "LOCAL:ft"; units ft; design "d.xml"; existing "e.xml"; boundary rect (0, 0) (200, 100); grid 2 ft }
probe p { kind static; sensor lidar; rate 5 min; accuracy 0.05 ft }
machine dz1 { class dozer; blade 10 ft; max_speed 4 mph }
machine rl1 { class roller; drum 7 ft; max_speed 4 mph }
zone pad { role cut; area rect (20, 20) (120, 80); tolerance ±0.1 ft }
task cut { op cut; zone pad; by class dozer; max_cut 0.5 ft per pass; spoil -> pad;
           until residual(pad).high < 0.1 ft }
"""
WINDOW = [[20, 45], [60, 45], [60, 55], [20, 55]]
FT = 0.3048


def yard(tmp_path):
    """The yard above, with its two surfaces written out: the checked model and its grid."""
    xs, ys = np.arange(-2.0, 203.0, 4.0), np.arange(-2.0, 103.0, 4.0)
    flat = np.ones((len(ys), len(xs)))
    write_grid_tin(str(tmp_path / "d.xml"), "FG", xs, ys, 100.0 * flat, "ft")
    write_grid_tin(str(tmp_path / "e.xml"), "EG", xs, ys, 101.0 * flat, "ft")
    (tmp_path / "yard.loam").write_text(SRC)
    model, diags = check(parse(SRC))
    assert not has_errors(diags), [d.msg for d in diags]
    return model, SiteGrid(model, str(tmp_path))


def order(op="cut", window=WINDOW):
    """An order as the bridge hands it to the blade."""
    return {"orderId": "o1", "op": op, "window": window}


def test_where_the_edge_should_be(tmp_path):
    """Inside the window of an order that moves earth, the edge belongs on the next-pass surface: here half a
    foot below the ground, and after a survey shows the ground lower, on design and never below it. Anywhere else,
    and for a roller's order, the blade is carried."""
    _, g = yard(tmp_path)
    b = Blade(g, height=3.0)
    assert b.wanted((40.0, 50.0, 0.0), order()) == pytest.approx(100.5)
    assert b.wanted((40.0, 50.0, 0.0), None) is None and b.wanted(None, order()) is None
    assert b.wanted((40.0, 50.0, 0.0), order("compact")) is None
    assert b.wanted((80.0, 50.0, 0.0), order()) is None                  # past the end of the window
    assert b.wanted((40.0, 50.0, 0.0), order(window=None)) is None
    assert b.wanted((10.0, 50.0, 0.0), order(window=[[0, 45], [18, 45], [18, 55], [0, 55]])) is None   # no zone there
    assert b.wanted((500.0, 50.0, 0.0), order(window=[[400, 0], [600, 0], [600, 100], [400, 100]])) is None
    # A towed blade is behind the position the vehicle reports.
    towed = Blade(g, height=3.0, reach=-6.0)
    assert towed.wanted((24.0, 50.0, 0.0), order()) is None and towed.wanted((64.0, 50.0, 0.0), order()) == 100.5
    assert towed.edge_point((64.0, 50.0, math.pi / 2)) == pytest.approx((64.0, 44.0))
    # A survey: the pad now lies at 100.2 ft, so one more pass takes it to design, not half a foot under.
    z = np.full(g.shape, np.nan, "<f4")
    z[:, 10:60] = 100.2
    b.survey(wire.terrain("yard", "p", 1, (g.x0, g.y0), g.cell, g.shape, data=wire.encode_heights(z)))
    assert b.surveys == 1 and b.wanted((40.0, 50.0, 0.0), order()) == pytest.approx(100.0)
    b.survey(wire.terrain("yard", "p", 2, (0, 0), 1.0, (3, 3), data=wire.encode_heights(np.zeros((3, 3)))))
    b.survey({"shape": list(g.shape), "data": None})
    assert b.surveys == 1                                                 # another grid, and an empty message
    with pytest.raises(ValueError, match="pulse widths"):
        Blade(g, 3.0, pwm=(1500, 1500))


@pytest.mark.parametrize("pwm", [(1100, 1900), (1900, 1100)])
def test_a_sensor_on_the_blade_is_stepped_onto_the_surface(tmp_path, pwm):
    """With the antenna riding on the blade, the edge is measured, and the actuator is stepped until the edge is
    within a quarter of an inch of the surface. It moves no faster than half its stroke a second, sends nothing
    when nothing changes, and lifts the blade when the order ends or the height is lost. An actuator wired the
    other way round (down at the long pulse) behaves the same."""
    _, g = yard(tmp_path)
    b = Blade(g, height=3.0, pwm=pwm)
    stroke, low = 2.5, 99.8                                # the real blade: edge at 99.8 ft fully down, 2.5 ft of lift

    def altitude():
        """What the antenna on the blade reads, 3 ft above the edge, wherever the actuator has been sent."""
        frac = (b.sent - pwm[0]) / (pwm[1] - pwm[0])
        return low + stroke * frac + 3.0

    t, pose = 1000.0, (40.0, 50.0, 0.0)
    assert b.command(pose, None, order(), t) == pwm[1] and b.command(pose, None, order(), t + 0.05) is None
    sent = []
    for k in range(1, 200):                                # ten seconds at 20 Hz
        p = b.command(pose, altitude(), order(), t + 0.05 * k)
        if p is not None:
            sent.append((k, p))
    edge = altitude() - 3.0
    assert abs(edge - 100.5) <= DEADBAND_FT + 0.01 and sent[-1][0] < 150   # there, and settled
    frac = [(p - pwm[0]) / (pwm[1] - pwm[0]) for _, p in sent]
    assert max(abs(a - c) for a, c in pairwise(frac)) <= 0.5 * 0.05 + 0.003   # the rate limit
    assert frac[-1] == pytest.approx((100.5 - low) / stroke, abs=0.02)
    # The order ends: carried again within the two seconds the stroke takes. Then the height is lost mid-order.
    for k in range(200, 260):
        b.command(pose, altitude(), None, t + 0.05 * k)
    assert b.sent == pwm[1]
    for k in range(260, 300):
        b.command(pose, altitude(), order(), t + 0.05 * k)
    assert b.sent != pwm[1]
    for k in range(300, 360):
        b.command(pose, None, order(), t + 0.05 * k)
    assert b.sent == pwm[1]


def test_a_sensor_on_the_body_uses_the_geometry(tmp_path):
    """With the antenna on the machine's body the edge is worked out from the actuator's position: 3 ft below the
    antenna fully down, rising 2 ft over the stroke. The actuator goes straight to where that puts the edge on the
    surface, and to its end stop when the surface is out of reach."""
    _, g = yard(tmp_path)
    b = Blade(g, height=3.0, travel=2.0)
    pose = (40.0, 50.0, 0.0)
    for k in range(60):
        b.command(pose, 102.5, order(), 1000.0 + 0.05 * k)     # edge = 99.5 + 2 * stroke; wanted 100.5
    assert b.at == pytest.approx(0.5, abs=0.01) and abs(b.sent - 1500) <= 8
    for k in range(60, 120):
        b.command(pose, 104.0, order(), 1000.0 + 0.05 * k)     # edge = 101 + 2 * stroke: cannot get down to 100.5
    assert b.at == 0.0 and b.sent == 1100


def test_the_bridge_holds_the_blade_of_a_vehicle(tmp_path, mqtt_broker, capsys):
    """`loam mavlink --blade-servo` against the fake ArduPilot, over a real broker. Before any order the blade is
    raised. On a cut order the servo is moved to put the edge on the next-pass surface while the vehicle is in the
    window, and raised when it leaves and when the order is done. A survey published on the site's terrain topic
    changes where the edge is put on the next order. `--blade-zero` measures the blade's height first."""
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import blade_zero, run_mavlink
    model, g = yard(tmp_path)
    lat0, lon0 = 34.4459, -85.7197
    frame = Frame(FT, 0.0, lat0, lon0)
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.alt_mm, rover.origin, rover.dwell = round(102.5 * FT * 1000), True, 2.2
    rover.start()
    try:
        # The blade rests on a mark at 99.5 ft and the vehicle reads 102.5 ft: the edge is 3 ft below the antenna.
        zero = blade_zero(f"udpin:127.0.0.1:{port}", frame, 9, 1100, 99.5, seconds=1.5)
        assert zero["blade_height"] == pytest.approx(3.0, abs=0.01) and zero["spread"] < 0.01
        assert (183, 9.0, 1100.0) in rover.commands
        rover.commands.clear()
        ctl = Controller(mqtt_broker, "yard", "dz1")
        blade = Blade(g, height=3.0, travel=2.0, reach=2.0)        # the edge is 2 ft ahead of the antenna
        bridge = threading.Thread(target=run_mavlink, args=(model, "dz1", mqtt_broker, f"udpin:127.0.0.1:{port}",
                                                           frame, 6, False, 90),
                                  kwargs={"blade": blade, "blade_servo": 9}, daemon=True)
        try:
            bridge.start()
            ctl.wait(lambda s: s["safetyState"] == "ok", timeout=15)

            def pulses():
                """The blade servo's pulse widths, in the order the vehicle was sent them."""
                return [int(p) for c, n, p in rover.commands if c == 183 and n == 9.0]

            time.sleep(0.5)
            assert pulses() == [1900]                                         # raised, and told so once
            ctl.send("order", wire.order("yard", "dz1", 1, "c1", 0, "cut", "cut", WINDOW, {}, {}))
            ctl.wait(lambda s: s["orderId"] == "c1" and s["progress"] == 1.0, timeout=30)
            time.sleep(2.5)
            first = pulses()
            # In the window: down to mid stroke, which is where the geometry puts the edge at 100.5 ft. Then up.
            assert abs(min(first) - 1500) <= 8 and first[0] > 1850 and first[-1] == 1900    # within the deadband
            assert all(abs(a - c) <= 30 for a, c in pairwise(first))    # in small steps
            # A scan shows the pad at 100.2 ft: the next pass goes to design, a quarter of the stroke.
            z = np.full(g.shape, 100.2, "<f4")
            ctl.bus.pub(wire.topic("yard", None, "terrain"),
                        wire.terrain("yard", "p", 1, (g.x0, g.y0), g.cell, g.shape, data=wire.encode_heights(z)), qos=1)
            end = time.time() + 5
            while blade.surveys == 0 and time.time() < end:
                time.sleep(0.05)
            assert blade.surveys == 1
            n = len(pulses())
            back = [[60, 45], [20, 45], [20, 55], [60, 55]]
            ctl.send("order", wire.order("yard", "dz1", 2, "c2", 0, "cut", "cut", back, {}, {}))
            ctl.wait(lambda s: s["orderId"] == "c2" and s["progress"] == 1.0, timeout=30)
            time.sleep(2.5)
            second = pulses()[n:]
            assert abs(min(second) - 1300) <= 8 and second[-1] == 1900
        finally:
            ctl.close(bridge)
    finally:
        rover.running = False
        rover.join(5)
    # The command refuses a blade without its height, and a zero without the servo.
    for extra in (["--blade-servo", "9"], ["--blade-zero", "99.5"]):
        with pytest.raises(SystemExit):
            main(["mavlink", str(tmp_path / "yard.loam"), "--machine", "dz1", "--vehicle", "udpin:127.0.0.1:1",
                  "--origin", f"{lat0},{lon0}", *extra])
    assert "--blade-height" in capsys.readouterr().err


def test_the_blade_height_can_come_from_a_second_gps():
    """With `--blade-gps2` the height is that of the autopilot's second GPS, whose antenna rides on the blade:
    the vehicle's own altitude is not used, a second GPS without an RTK fix gives no height, and a height that
    stops arriving is gone within a second."""
    pytest.importorskip("pymavlink")
    from loam.runtime.mavlink import Vehicle, blade_zero
    lat0, lon0 = 34.4459, -85.7197
    frame = Frame(FT, 0.0, lat0, lon0)
    port = free_port()
    rover = FakeRover(port, lat0, lon0)
    rover.alt_mm, rover.origin, rover.gps2 = 99_000, True, (round(103.0 * FT * 1000), 6)
    rover.start()

    def watch(v, seconds):
        """Read the vehicle for a while and return its height."""
        end = time.time() + seconds
        while time.time() < end:
            v.poll()
            time.sleep(0.02)
        return v.altitude

    try:
        v = Vehicle(f"udpin:127.0.0.1:{port}", frame, blade_servo=9, blade_gps2=True)
        try:
            assert watch(v, 0.6) == pytest.approx(103.0, abs=0.01)           # the second GPS, not 99 m
            rover.gps2 = (round(103.0 * FT * 1000), 5)                       # RTK float: not good enough
            assert watch(v, 1.3) is None
            rover.gps2 = (round(102.0 * FT * 1000), 6)
            assert watch(v, 0.6) == pytest.approx(102.0, abs=0.01)
            rover.gps2 = None                                                # the antenna cable comes off
            assert watch(v, 1.3) is None
        finally:
            v.close()
        rover.gps2 = (round(103.0 * FT * 1000), 6)
        zero = blade_zero(f"udpin:127.0.0.1:{port}", frame, 9, 1100, 100.0, seconds=1.0, gps2=True)
        assert zero["blade_height"] == pytest.approx(3.0, abs=0.01)
    finally:
        rover.running = False
        rover.join(5)
