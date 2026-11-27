"""Real site inputs: people from Frigate, rain from rtl_433, scans from a folder, the e-stop command.

Frigate is played by a broker client that follows its documented messages (the fields were checked against
Frigate's source, commit 8eaae4b of November 2026). rtl_433 is the real program when it is installed: it decodes
recorded radio samples of an Ecowitt WH40 rain gauge.
"""
import json
import os
import shutil
import subprocess
import threading
import time

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES, wire_valid

from loam import sensors, wire
from loam.cli import load, main
from loam.grid import SiteGrid
from loam.pointcloud import write_las
from loam.runtime.bus import Bus, Clock
from loam.runtime.controller import Controller
from loam.runtime.machine import MachineSim
from loam.sensors import Area, Camera, People, Rain, load_areas, run_sense

CAMERAS = {"yard": Camera(Area(80.0, 50.0, 60.0), {"gate": Area(120.0, 40.0, 30.0), "dock": Area(20.0, 80.0, 15.0)}),
           "ramp": Camera(Area(150.0, 90.0, 25.0), {})}
RTL_433 = os.environ.get("RTL_433") or shutil.which("rtl_433") or next(
    (p for p in ["/opt/src/rtl_433/build/src/rtl_433"] if os.path.exists(p)), None)
SAMPLES = os.environ.get("RTL_433_SAMPLES", "/opt/src/rtl_433_tests/tests/EcoWitt-WH40")
MOSQUITTO_PUB = shutil.which("mosquitto_pub")


def view(*people, detect=True):
    """One camera's part of Frigate's `camera_activity` message: the objects it follows, each (label, zones)."""
    return {"motion": bool(people), "config": {"detect": detect, "enabled": True},
            "objects": [{"id": f"o{i}", "label": label, "stationary": False, "area": 900, "ratio": 0.4, "score": 0.8,
                         "sub_label": None, "current_zones": zones} for i, (label, zones) in enumerate(people)]}


def test_people_from_frigates_camera_picture():
    """People are placed in the mapped zones they stand in, else in their camera's area. Cars, and cameras that are
    not site cameras, are ignored. Nothing is carried over from one picture to the next.
    """
    p = People(CAMERAS)
    quiet = {"yard": view(), "ramp": view(), "office": view(("person", []))}
    p.activity = quiet
    assert p.detections() == {}
    p.activity = {**quiet, "yard": view(("person", ["gate"]), ("car", ["dock"]), ("person-verified", ["dock", "gate"]),
                                        ("person", ["unmapped_zone"]))}
    assert p.detections() == {"yard/gate": ("person", CAMERAS["yard"].zones["gate"]),
                              "yard/dock": ("person", CAMERAS["yard"].zones["dock"]),
                              "yard": ("person", CAMERAS["yard"].area)}
    p.activity = quiet
    assert p.detections() == {}


def test_a_blind_camera_is_unknown_not_empty():
    """A site camera that Frigate does not report, that has detection off, or that delivers no pictures cannot
    vouch for its area: the area is "unknown" until the camera is back.
    """
    p = People(CAMERAS)
    unknown = lambda n: ("unknown", CAMERAS[n].area)                              # noqa: E731
    assert p.detections() == {"yard": unknown("yard"), "ramp": unknown("ramp")}   # no picture from Frigate yet
    p.activity = {"yard": view(), "ramp": view(detect=False)}
    assert p.detections() == {"ramp": unknown("ramp")}
    p.activity = {"yard": view(("person", ["gate"])), "ramp": view()}
    p.stats({"cameras": {"yard": {"camera_fps": 0.0}, "ramp": {"camera_fps": 5.1}}})
    assert p.detections() == {"yard": unknown("yard")}        # its last picture still shows a person: not trusted
    p.stats({"cameras": {"yard": {"camera_fps": 5.0}, "ramp": {"camera_fps": 5.1}}})
    assert p.detections() == {"yard/gate": ("person", CAMERAS["yard"].zones["gate"])}


def test_rain_rate_from_gauge_totals():
    """The rate is the rise of the gauge's total over the window. A restart of the gauge, readings in inches, a rate
    the gauge reports itself and other people's gauges are all handled.
    """
    r = Rain("EcoWitt-WH40", "52591", window_s=600.0)
    say = lambda t, **f: r.reading({"model": "EcoWitt-WH40", "id": 52591, **f}, t)      # noqa: E731
    assert say(0, rain_mm=10.0) == 0.0                            # one reading is no rate yet
    assert say(300, rain_mm=13.0) == pytest.approx(0.003 / 300)   # 3 mm in 5 min = 36 mm/h
    assert say(301, temperature_C=20.0) is None                   # not a rain reading
    assert r.reading({"model": "Acurite-5n1", "id": 9, "rain_mm": 99.0}, 302) is None       # a neighbour's gauge
    assert say(900, rain_mm=13.0) == pytest.approx(0.0)           # the rise has left the 10-minute window
    assert say(960, rain_mm=0.2) == 0.0                           # new batteries: the total starts again
    assert say(1020, rain_mm=0.7) == pytest.approx(0.0005 / 60)
    assert say(1021, rain_mm=1.2) == pytest.approx(0.001 / 61)
    quick = Rain(window_s=600.0)                                  # two tips a second apart are not a cloudburst:
    assert quick.reading({"model": "X", "id": 1, "rain_mm": 0.0}, 0) == 0.0
    assert quick.reading({"model": "X", "id": 1, "rain_mm": 0.3}, 1) == pytest.approx(0.0003 / 60)
    first = Rain()                                                # no gauge named: the first one heard is kept
    assert first.reading({"model": "A", "id": 1, "rain_in": 1.0}, 0) == 0.0
    assert first.reading({"model": "B", "id": 2, "rain_in": 9.0}, 10) is None
    assert first.reading({"model": "A", "id": 1, "rain_in": 1.1}, 120) == pytest.approx(0.00254 / 120)
    own = Rain()
    assert own.reading({"model": "C", "id": 3, "rain_rate_mm_h": 7.2}, 0) == pytest.approx(2e-6)
    assert own.reading({"model": "C", "id": 3, "rain_rate_in_h": 1.0}, 1) == pytest.approx(0.0254 / 3600)


def test_areas_file(tmp_path):
    """The areas file is read, and a malformed one is refused with the expected shape in the message."""
    f = tmp_path / "areas.json"
    f.write_text(json.dumps({"cameras": {"yard": {"x": 80, "y": 50, "radius": 60,
                                                  "zones": {"gate": {"x": 120, "y": 40, "radius": 30}}},
                                         "ramp": {"x": 150, "y": 90, "radius": 25}}}))
    got = load_areas(str(f))
    assert got["yard"] == Camera(Area(80.0, 50.0, 60.0), {"gate": Area(120.0, 40.0, 30.0)}) and got["ramp"].zones == {}
    for bad in ({"cameras": {}}, {"cameras": {"g": {"x": 1, "y": 2}}},
                {"cameras": {"g": {"x": 1, "y": 2, "radius": 0}}},
                {"cameras": {"g": {"x": 1, "y": 2, "radius": 5, "zones": {"z": {"x": 1}}}}}, {"areas": {}}):
        f.write_text(json.dumps(bad))
        with pytest.raises(ValueError, match="cameras"):
            load_areas(str(f))


class _Out:
    """A bus that only keeps what the controller publishes."""
    def __init__(self):
        """Nothing published yet."""
        self.out = []

    def pub(self, topic, payload, retain=False, qos=0):
        """Keep the message."""
        self.out.append((topic, payload))


def test_controller_measures_from_the_edge_of_a_detection():
    """A detection with a radius is "somewhere in this circle": the distance to a machine counts from the circle's
    edge. An unknown presence over the whole site is at distance zero from everything.
    """
    c = Controller(load(EXAMPLE), EXAMPLES, _Out(), Clock(0, 60, 1796108400.0), log=lambda x: None)
    n = next(iter(c.machines))
    c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 100.0, 40.0, 0, False, 0, ts=1796108400.0))
    assert c.proximity("person", "machine") == float("inf")
    c.handle("loam/v0/s/detection", wire.detection("s", "cam/gate", 1, "person", 180.0, 40.0, radius=30.0))
    assert c.proximity("person", "machine") == pytest.approx(50 * 0.3048)        # 80 ft away, less the 30 ft radius
    c.handle("loam/v0/s/detection", wire.detection("s", "cam/near", 2, "person", 110.0, 40.0, radius=30.0))
    assert c.proximity("person", "machine") == 0.0                                # the machine is inside the circle
    c.handle("loam/v0/s/detection", wire.detection("s", "cam/near", 3, "clear", 0, 0))
    c.handle("loam/v0/s/detection", wire.detection("s", "cam/gate", 4, "clear", 0, 0))
    c.handle("loam/v0/s/detection", wire.detection("s", "cam", 5, "unknown", 0, 0, radius=wire.EVERYWHERE))
    assert c.proximity("person", "machine") == 0.0
    c.handle("loam/v0/s/detection", wire.detection("s", "cam", 6, "clear", 0, 0))
    assert c.proximity("person", "machine") == float("inf")


class _Frigate:
    """Frigate, as far as `loam sense` can tell: a retained status, and its full camera picture published whenever
    someone publishes to `frigate/onConnect`.
    """

    def __init__(self, broker):
        """Connect and start answering."""
        self.activity: dict | None = {"yard": view(), "ramp": view()}        # None: do not answer
        self.asked = 0
        self.bus = Bus(broker, f"frigate-{time.time()}", raw=True).start(["frigate/onConnect"])
        self.running = True
        self.thread = threading.Thread(target=self._answer, daemon=True)
        self.thread.start()

    def _answer(self):
        """Answer each request with the current picture, as Frigate does."""
        while self.running:
            for topic, _ in self.bus.drain(0.05):
                if topic == "frigate/onConnect":
                    self.asked += 1
                    if self.activity is not None:
                        self.bus.pub("frigate/camera_activity", self.activity)

    def available(self, state):
        """Publish Frigate's status the way it does: plain text, retained."""
        self.bus.c.publish("frigate/available", state, qos=1, retain=True)

    def close(self):
        """Stop answering, remove the retained status and disconnect."""
        self.running = False
        self.thread.join(5)
        self.bus.clear("frigate/available")
        time.sleep(0.2)
        self.bus.stop()


def test_sense_republishes_frigate_and_fails_safe(mqtt_broker, monkeypatch):
    """`loam sense` against a stand-in Frigate, end to end over a real broker.

    In order: until Frigate is known to be up, the whole site counts as unwatched, and whatever an earlier run of
    the program last said about any camera or zone is withdrawn; a person already in view when the program starts
    is found; moving between zones moves the detection; leaving clears it; a camera that stops
    delivering pictures makes its area unknown; Frigate falling silent or going offline makes the site unwatched
    again; and so does stopping the program, with a retained message.
    """
    monkeypatch.setattr(sensors, "POLL_S", 0.2)
    monkeypatch.setattr(sensors, "SILENT_S", 1.0)
    monkeypatch.setattr(sensors, "REPEAT_S", 0.4)
    model = load(EXAMPLE)
    topic = wire.topic("pad_north", None, "detection")
    yard, ramp = CAMERAS["yard"], CAMERAS["ramp"]
    fr = _Frigate(mqtt_broker)
    fr.activity = {"yard": view(("person", ["gate"])), "ramp": view()}
    listen = Bus(mqtt_broker, f"listen-{time.time()}").start([topic])
    seen: list = []
    result: dict = {}
    # The program runs for 25 s: the steps below take about 6 s, and several times that on a loaded computer.
    t = threading.Thread(target=lambda: result.update(run_sense(model, mqtt_broker, CAMERAS, max_wall_s=25.0)))

    def current(until, wait=4.0):
        """The detections in force, by sensor, once they equal `until` (or after `wait` seconds)."""
        end, state = time.time() + wait, {}
        while time.time() < end:
            seen.extend(m for _, m in listen.drain(0.1))
            state = {}
            for m in seen:
                wire_valid("detection", m)
                if m["kind"] == "clear":
                    state.pop(m["sensor"], None)
                else:
                    state[m["sensor"]] = (m["kind"], Area(m["x"], m["y"], m["radius"]))
            if state == until:
                break
        return state

    try:
        t.start()
        unwatched = {"sense": ("unknown", Area(0.0, 0.0, wire.EVERYWHERE))}
        assert current(unwatched) == unwatched                # Frigate has not said it is up
        # A run that died while a person was in view left that detection with the controller. Starting again
        # (or reconnecting) withdraws every one, so none can outlive the person.
        every = {"sense/yard", "sense/yard/gate", "sense/yard/dock", "sense/ramp"}
        end = time.time() + 5.0
        while time.time() < end and not {m["sensor"] for m in seen if m["kind"] == "clear"} >= every:
            seen.extend(m for _, m in listen.drain(0.1))
        assert {m["sensor"] for m in seen if m["kind"] == "clear"} >= every
        assert fr.asked == 0                                  # and is not asked until it has
        fr.available("online")
        at_gate = {"sense/yard/gate": ("person", yard.zones["gate"])}
        assert current(at_gate) == at_gate                    # in view before the program started
        fr.activity = {"yard": view(("person", ["dock"])), "ramp": view()}
        at_dock = {"sense/yard/dock": ("person", yard.zones["dock"])}
        assert current(at_dock) == at_dock
        fr.activity = {"yard": view(), "ramp": view(("person", []))}
        on_ramp = {"sense/ramp": ("person", ramp.area)}
        assert current(on_ramp) == on_ramp
        fr.activity = {"yard": view(), "ramp": view()}
        assert current({}) == {}
        fr.bus.pub("frigate/stats", {"cameras": {"yard": {"camera_fps": 0.0}, "ramp": {"camera_fps": 5.0}}})
        yard_blind = {"sense/yard": ("unknown", yard.area)}
        assert current(yard_blind) == yard_blind              # the yard camera has stopped
        fr.bus.pub("frigate/stats", {"cameras": {"yard": {"camera_fps": 5.0}, "ramp": {"camera_fps": 5.0}}})
        assert current({}) == {}
        fr.activity = None                                    # Frigate stops answering
        assert current(unwatched) == unwatched
        fr.activity = {"yard": view(), "ramp": view()}
        assert current({}) == {}
        fr.available("offline")                               # Frigate's last will
        assert current(unwatched) == unwatched
    finally:
        t.join(40)
        listen.stop()
        fr.close()
    assert result["detection"] >= 10 and result["weather"] == 0
    late = Bus(mqtt_broker, f"late-{time.time()}").start([topic])             # a controller that starts afterwards
    try:
        retained = [m for _, m in late.drain(2.0)]
    finally:
        late.clear(topic)
        time.sleep(0.2)
        late.stop()
    assert [(m["sensor"], m["kind"], m["radius"]) for m in retained] == [("sense", "unknown", wire.EVERYWHERE)]


def test_sense_needs_something_to_sense():
    """`loam sense` with neither cameras nor a gauge explains itself instead of starting."""
    with pytest.raises(SystemExit):
        main(["sense", EXAMPLE])


def test_rain_over_mqtt(mqtt_broker):
    """Gauge readings published the way rtl_433 publishes them come out as schema-valid weather messages; another
    gauge on the air is ignored.
    """
    topic = wire.topic("pad_north", None, "weather")
    pub, listen = Bus(mqtt_broker, f"rtl-{time.time()}").start([]), Bus(mqtt_broker, f"l-{time.time()}").start([topic])
    result: dict = {}
    t = threading.Thread(target=lambda: result.update(run_sense(
        load(EXAMPLE), mqtt_broker, rain_topic="rtl_433/+/events", gauge="EcoWitt-WH40:52591", max_wall_s=3.0)))
    try:
        t.start()
        time.sleep(0.8)
        for mm, model in [(0.0, "EcoWitt-WH40"), (55.0, "Acurite-5n1"), (4.1, "EcoWitt-WH40")]:
            pub.pub("rtl_433/site-pc/events", {"time": "2026-11-26 12:00:00", "model": model, "id": 52591,
                                               "battery_ok": 1, "rain_mm": mm, "mic": "CRC"})
            time.sleep(0.2)
        msgs = [m for _, m in listen.drain(1.5)] + [m for _, m in listen.drain(0.5)]
    finally:
        t.join(10)
        pub.stop()
        listen.stop()
    for m in msgs:
        wire_valid("weather", m)
    assert [m["rain"] for m in msgs] == [0.0, pytest.approx(0.0041 / 60)]
    assert result == {"detection": 0, "weather": 2}


@pytest.mark.skipif(not (RTL_433 and MOSQUITTO_PUB and os.path.isdir(SAMPLES)),
                    reason="needs rtl_433, mosquitto_pub and rtl_433's WH40 test samples")
def test_rtl_433_decodes_a_real_gauge_into_weather(mqtt_broker):
    """The real rtl_433 decodes two recorded radio transmissions of an Ecowitt WH40 rain gauge (totals 0.0 and
    4.1 mm); its JSON goes to the broker through `mosquitto_pub`; `loam sense` turns it into a rain rate.

    (rtl_433's own MQTT output only runs with a live radio, so a recording is piped instead:
    `rtl_433 -F json | mosquitto_pub -l -t rtl_433/<host>/events`, which also works on site.)
    """
    topic = wire.topic("pad_north", None, "weather")
    listen = Bus(mqtt_broker, f"l-{time.time()}").start([topic])
    result: dict = {}
    t = threading.Thread(target=lambda: result.update(
        run_sense(load(EXAMPLE), mqtt_broker, rain_topic="rtl_433/+/events", max_wall_s=6.0)))
    host, port = mqtt_broker.split(":")
    try:
        t.start()
        time.sleep(0.8)
        for sample in ("g003_433.92M_250k.cu8", "g022_433.92M_250k.cu8"):
            decoded = subprocess.run([RTL_433, "-r", os.path.join(SAMPLES, sample), "-F", "json"],
                                     capture_output=True, text=True, timeout=30)
            assert decoded.returncode == 0 and "EcoWitt-WH40" in decoded.stdout, decoded.stderr[-400:]
            subprocess.run([MOSQUITTO_PUB, "-h", host, "-p", port, "-t", "rtl_433/test/events", "-l"],
                           input=decoded.stdout, text=True, check=True, timeout=30)
        msgs = [m for _, m in listen.drain(3.0)] + [m for _, m in listen.drain(0.5)]
    finally:
        t.join(15)
        listen.stop()
    for m in msgs:
        wire_valid("weather", m)
    assert [m["rain"] for m in msgs] == [0.0, pytest.approx(0.0041 / 60)]


def test_terrain_watch_publishes_scans_as_they_arrive(mqtt_broker, tmp_path):
    """`loam terrain --watch`: of the scans already in the folder only the newest is published; each new scan is
    published once it has stopped changing, stamped with the file's time; an unreadable file is skipped.
    """
    model = load(EXAMPLE)
    g = SiteGrid(model, EXAMPLES)
    gx, gy = np.meshgrid(g.xs, g.ys)
    scans = tmp_path / "scans"
    scans.mkdir()

    def scan(name, height, age_s=0.0):
        """Write a flat scan at `height` ft, in site coordinates, last changed `age_s` seconds ago."""
        path = str(scans / name)
        write_las(path, np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, height)]))
        os.utime(path, (time.time() - age_s,) * 2)
        return os.path.getmtime(path)

    scan("old.las", 100.0, age_s=600)
    newest_at = scan("newest.las", 101.0, age_s=60)
    topic = wire.topic("pad_north", None, "terrain")
    listen = Bus(mqtt_broker, f"l-{time.time()}").start([topic])
    t = threading.Thread(target=main, args=(["terrain", EXAMPLE, str(scans), "--watch", "--broker", mqtt_broker,
                                              "--world-units", "ft", "--settle", "0.3", "--max-seconds", "5"],))
    try:
        t.start()
        first = [m for _, m in listen.drain(3.0)]
        (scans / "broken.las").write_bytes(b"not a point cloud")
        (scans / "notes.txt").write_text("ignored: not a scan type")
        scan("next.las", 102.0)
        later = [m for _, m in listen.drain(3.0)] + [m for _, m in listen.drain(1.0)]
    finally:
        t.join(15)
        listen.stop()
    assert len(first) == 1 and len(later) == 1
    for m, height in ((first[0], 101.0), (later[0], 102.0)):
        wire_valid("terrain", m)
        assert np.nanmax(np.abs(wire.decode_heights(m["data"], tuple(m["shape"])) - height)) < 1e-3
    assert wire.parse_ts(first[0]["timestamp"]) == pytest.approx(newest_at, abs=0.002)
    with pytest.raises(SystemExit):
        main(["terrain", EXAMPLE, str(scans), "--watch"])                     # --watch needs a broker


def test_terrain_publishes_one_scan(mqtt_broker, tmp_path):
    """`loam terrain --broker` publishes a single scan and exits."""
    model = load(EXAMPLE)
    g = SiteGrid(model, EXAMPLES)
    gx, gy = np.meshgrid(g.xs, g.ys)
    cloud = str(tmp_path / "scan.las")
    write_las(cloud, np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, 99.0)]))
    topic = wire.topic("pad_north", None, "terrain")
    listen = Bus(mqtt_broker, f"l-{time.time()}").start([topic])
    try:
        time.sleep(0.2)
        main(["terrain", EXAMPLE, cloud, "--world-units", "ft", "--broker", mqtt_broker])
        msgs = [m for _, m in listen.drain(2.0)]
    finally:
        listen.stop()
    assert len(msgs) == 1 and msgs[0]["probe"] == next(iter(model.probes))


def test_estop_command_reaches_machines_that_connect_later(mqtt_broker):
    """`loam estop` publishes a retained, schema-valid press. A machine told of it stops; a release leaves it
    stopped until someone resumes.
    """
    model = load(EXAMPLE)
    topic = wire.topic("pad_north", None, "estop")
    main(["estop", EXAMPLE, "--broker", mqtt_broker, "--reason", "person in the cut", "--json"])
    late = Bus(mqtt_broker, f"late-{time.time()}").start([topic])
    try:
        pressed = [m for _, m in late.drain(2.0)]
        main(["estop", EXAMPLE, "--broker", mqtt_broker, "--release"])
        released = [m for _, m in late.drain(2.0)]
    finally:
        late.clear(topic)
        time.sleep(0.2)
        late.stop()
    assert [m["pressed"] for m in pressed + released] == [True, False]
    assert pressed[0]["reason"] == "person in the cut" and released[0]["reason"] == "released by operator"
    for m in pressed + released:
        wire_valid("estop", m)
    ms = MachineSim(model, next(iter(model.machines)), "unused", (0.0, 0.0))
    ms.last_hb = time.time()
    ms.control("estop", pressed[0])
    assert ms.safety_state()[1][0]["code"] == "estop"
    ms.control("estop", released[0])
    assert ms.safety_state()[1][0]["code"] == "estopReset"


def test_controller_refuses_orders_to_a_machine_with_a_wrong_clock():
    """On a real site (real time), a machine whose state is stamped more than 2 s from the controller's clock gets
    no orders until its clock is right, and the status reports the difference. A simulated site, which runs
    faster than real time, is not judged this way. A survey stamped in the future is ignored.
    """
    from loam.runtime.controller import MAX_CLOCK_SKEW_S
    logs: list = []
    c = Controller(load(EXAMPLE), EXAMPLES, _Out(), Clock(), log=logs.append)             # real time, starting now
    n = next(iter(c.machines))
    c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})

    def state(offset_s):
        """The machine reports, with its clock `offset_s` seconds ahead of ours."""
        c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=time.time() + offset_s))

    state(0.0)
    assert c.machine_ready(n) and abs(c.status()["machines"][n]["clock_skew_s"]) < 0.5
    state(-(MAX_CLOCK_SKEW_S + 3))                                # its clock is 5 s behind
    assert not c.machine_ready(n) and c.status()["machines"][n]["clock_skew_s"] == pytest.approx(5.0, abs=0.5)
    state(-(MAX_CLOCK_SKEW_S + 3))
    assert sum("clock is" in line for line in logs) == 1          # said once, not four times a second
    state(MAX_CLOCK_SKEW_S + 3)                                   # or ahead
    assert not c.machine_ready(n)
    state(0.3)
    assert c.machine_ready(n) and any("clock synchronised" in line for line in logs)

    def survey(offset_s):
        """A survey of the site's own ground, stamped `offset_s` seconds from now."""
        c.t = c.clock.minutes()
        c.handle("loam/v0/s/terrain", wire.terrain("s", "p", 1, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                                   data=wire.encode_heights(c.z), ts=time.time() + offset_s))
        c.terrain_since = 0.0
        c.probe()

    survey(30.0)
    assert c.probe_count == 0 and any("in the future" in line for line in logs)
    survey(-1.0)
    assert c.probe_count == 1

    sim = Controller(load(EXAMPLE), EXAMPLES, _Out(), Clock(0, 60, 1796108400.0), log=logs.append)
    sim.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
    sim.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=1796108400.0))
    assert sim.machine_ready(n)                                   # site time is not wall time there


def test_monitor_records_every_wire_message_to_mcap(mqtt_broker, tmp_path):
    """`loam monitor --mcap` writes each Wire message, unchanged, to an MCAP file that the reference MCAP reader
    opens: one channel per topic, the message's own time as publish time, the time it was seen as log time.
    """
    reader = pytest.importorskip("mcap.reader")
    import socket

    from loam import monitor
    model = load(EXAMPLE)
    out = tmp_path / "run.mcap"
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    t = threading.Thread(target=monitor.serve, args=(model, EXAMPLES, mqtt_broker, port, None, "127.0.0.1", str(out)))
    pub = Bus(mqtt_broker, f"p-{time.time()}").start([])
    stamp = 1796108400.5
    sent = [(wire.topic("pad_north", "dz1", "state"), wire.state("pad_north", "dz1", 1, None, 3.0, 4.0, 0, False, 0,
                                                                 ts=stamp)),
            (wire.topic("pad_north", None, "detection"), wire.detection("pad_north", "cam", 2, "person", 1, 2, ts=stamp,
                                                                        radius=30.0)),
            (wire.topic("pad_north", None, "estop"), {"pressed": True}),               # the simplest e-stop radio
            (wire.topic("pad_north", "dz1", "state"), wire.state("pad_north", "dz1", 3, None, 5.0, 4.0, 0, False, 0,
                                                                 ts=stamp + 1))]
    try:
        t.start()
        time.sleep(1.0)
        before = time.time()
        for topic, msg in sent:
            pub.pub(topic, msg, qos=1)
        time.sleep(0.5)
        pub.pub("sim/pad_north/end", {}, qos=1)
        t.join(10)
    finally:
        pub.stop()
    assert not t.is_alive()
    with open(out, "rb") as f:
        got = [(ch.topic, ch.message_encoding, sch.name, json.loads(m.data), m.publish_time, m.log_time)
               for sch, ch, m in reader.make_reader(f).iter_messages()]
    assert [(g[0], g[3]) for g in got] == sent
    assert {g[1] for g in got} == {"json"} and {g[2] for g in got} == {"loam.wire"}
    assert got[0][4] == int(stamp * 1e9) and got[3][4] == int((stamp + 1) * 1e9)       # the message's own time
    assert all(before - 1 < g[5] / 1e9 < time.time() for g in got)                     # when it was seen
    assert abs(got[2][4] - got[2][5]) < 1e6                                            # no timestamp: both the same
