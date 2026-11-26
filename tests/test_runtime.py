"""loam plan, the world physics, the controller and the monitor.

The loopback test runs the real controller against the real world physics in one
process (no broker), so it is deterministic and fast. The MQTT test starts a
Mosquitto broker and every role as its own process, exactly as on a site; it is
skipped when `mosquitto` is not installed.
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import time

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES

from loam import wire
from loam.cli import load, main
from loam.landxml import grid_tin, tin_boundary, write_tin
from loam.plan import pick_surfaces, plan, simplify
from loam.sim import Events, Sim

pytest.importorskip("paho.mqtt")
from loam.monitor import Frames, replay_html
from loam.runtime.bus import Clock
from loam.runtime.controller import Controller
from loam.runtime.world import World


# -- loam plan ------------------------------------------------------------------------------
def _project(path, design_offset=0.0, eg_margin=10.0, names=("EG", "FG")):
    """One LandXML with an existing surface and a smaller design surface, as Civil 3D exports them."""
    xs, ys = np.arange(-eg_margin, 60 + eg_margin + 1, 2.0), np.arange(-eg_margin, 40 + eg_margin + 1, 2.0)
    gx, gy = np.meshgrid(xs, ys)
    eg = 100 + 0.02 * gx + 0.4 * np.sin(gx / 9) * np.cos(gy / 7)
    dx, dy = np.arange(0, 61, 2.0), np.arange(0, 41, 2.0)
    fg = np.full((dy.size, dx.size), 100.6 + design_offset)
    import xml.etree.ElementTree as ET
    write_tin(str(path) + ".a", names[0], *grid_tin(xs, ys, eg), "ft")
    write_tin(str(path) + ".b", names[1], *grid_tin(dx, dy, fg), "ft")
    ns = "http://www.landxml.org/schema/LandXML-1.2"
    ET.register_namespace("", ns)
    a, b = ET.parse(str(path) + ".a").getroot(), ET.parse(str(path) + ".b").getroot()
    a.find(f"{{{ns}}}Surfaces").append(b.find(f"{{{ns}}}Surfaces").find(f"{{{ns}}}Surface"))
    ET.ElementTree(a).write(str(path), encoding="utf-8", xml_declaration=True)
    os.remove(str(path) + ".a")
    os.remove(str(path) + ".b")


def test_plan_writes_a_checked_program(tmp_path):
    _project(tmp_path / "job.xml")
    r = plan(str(tmp_path / "job.xml"), str(tmp_path / "job.loam"))
    assert r["design"] == "FG" and r["existing"] == "EG" and r["units"] == "ft"
    m = load(str(tmp_path / "job.loam"))
    assert set(m.tasks) == {"cut_to_fill", "top_up", "compact", "finish_grade"}
    assert len(m.site["boundary"]["polygon"]) == 4                     # the design's edge, simplified
    assert m.zones["cut_area"]["area"]["where"] is not None
    text = (tmp_path / "job.loam").read_text()
    assert "CHECK:" in text and "loam 0.0.2" in text
    s = Sim(m, str(tmp_path), events=Events.none(), log=lambda x: None)
    rep = s.run()
    assert rep["finished"] and rep["true_ground"]["within_tolerance_pct"] >= 95


def test_plan_fleet_names_and_errors(tmp_path, capsys):
    _project(tmp_path / "job.xml", names=("Existing Ground", "Proposed Grade"))
    (tmp_path / "fleet.loam").write_text(
        "probe drone { kind tethered_uav; sensor lidar; rate 5 min; accuracy 0.05 ft; coverage site }\n"
        "machine d6 { class dozer; blade 13 ft; max_speed 3 mph }\n"
        "machine cs { class roller; drum 7 ft; max_speed 4 mph }\n")
    main(["plan", str(tmp_path / "job.xml"), "-o", str(tmp_path / "a.loam"), "--fleet", str(tmp_path / "fleet.loam"),
          "--tolerance", "0.05 ft", "--json"])
    r = json.loads(capsys.readouterr().out)
    assert r["check"] == "ok" and r["design"] == "Proposed Grade"
    assert "rule drone_stale" in (tmp_path / "a.loam").read_text()
    with pytest.raises(ValueError, match="cannot tell"):
        pick_surfaces(["Surface 1", "Surface 2"], None, None)
    _project(tmp_path / "small.xml", eg_margin=-10.0)                   # existing smaller than design
    with pytest.raises(SystemExit):
        main(["plan", str(tmp_path / "small.xml"), "-o", str(tmp_path / "b.loam")])
    assert "uncovered" in capsys.readouterr().err


def test_tin_boundary_and_simplify():
    pts, faces = grid_tin(np.arange(0.0, 5), np.arange(0.0, 4), np.zeros((4, 5)))
    ring = tin_boundary(pts, faces)
    assert len(ring) == 14
    assert sorted(simplify(ring, 0.1)) == [(0.0, 0.0), (0.0, 3.0), (4.0, 0.0), (4.0, 3.0)]


# -- wire ----------------------------------------------------------------------------------
def test_wire_timestamps_keep_milliseconds():
    h = wire.header(1, 1796108400.25)
    assert h["timestamp"] == "2026-12-01T07:00:00.250Z"
    assert wire.parse_ts(h["timestamp"]) == pytest.approx(1796108400.25)
    assert wire.header(2, 1796108400.0)["timestamp"] == "2026-12-01T07:00:00Z"
    assert wire.parse_ts(wire.header(3, 1796108400.9996)["timestamp"]) == pytest.approx(1796108401.0)


# -- world physics against the planner -------------------------------------------------------------
class _Replay(Sim):
    """The reference simulator, with every order also sent to the world."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.w = World(self.m, EXAMPLES)
        self.orders = {}

    def emit(self, topic, msg):
        if topic.endswith("/order"):
            self.orders[msg["machine"]] = msg

    def land(self, ps):
        super().land(ps)
        self.w.apply(self.orders[ps.machine])


def test_world_moves_the_earth_the_orders_describe():
    s = _Replay(load(EXAMPLE), EXAMPLES, events=Events.none(), log=lambda x: None)
    s.run()
    d = np.abs(s.w.z - s.z)              # independent blade-error draws, so equal only within that noise
    assert np.percentile(d, 95) < 0.15 and d.mean() < 0.05
    t = s.w.truth()
    assert t["zones"]["pad"]["within_tolerance_pct"] >= 90 and abs(t["ledger_error"]) < 1e-6   # no feedback here
    order = next(iter(s.orders.values()))
    assert s.w.apply(order)["duplicate"] and s.w.truth()["duplicate_reports"] == 1


# -- controller loopback: the real controller, the real world, no broker ---------------------------
class _FakeBus:
    def __init__(self):
        self.out = []

    def pub(self, topic, msg, **k):
        self.out.append((topic, msg))


def loopback(path, base, pass_min=3.0, scan_lag=0.0, max_min=1500.0):
    m = load(path)
    bus, clock, logs = _FakeBus(), Clock(0, 60, 1796108400.0), []
    c = Controller(m, base, bus, clock, log=logs.append)
    w = World(m, base)
    c.t, seq, last_scan = 0.0, 0, -1e9
    p = next(iter(m.probes.values()))
    rate, sigma = p["rate"].to("min"), p["accuracy"].to(c.unit) / 2
    rng = np.random.default_rng(1)
    work: dict = {}
    for n in c.machines:
        c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
    done = False
    while c.t < max_min and not done:
        for topic, msg in bus.out:
            if topic.endswith("/order"):
                work[msg["machine"]] = (msg, c.t + pass_min)
        bus.out.clear()
        for n in c.machines:
            seq += 1
            if n in work and c.t >= work[n][1]:
                o, ft = work.pop(n)
                w.apply(o)
                c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, seq, o["orderId"], 0, 0, 0, False, 1.0,
                                                             ts=clock.ts(ft)))
            else:
                oid = work[n][0]["orderId"] if n in work else None
                c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, seq, oid, 0, 0, 0, False, 0.5, ts=clock.ts(c.t)))
            c.mstate[n]["seen"] = time.time()
        if c.t - last_scan >= rate:
            last_scan = c.t
            z = w.z + rng.normal(0, sigma, w.z.shape)
            c.handle("loam/v0/s/terrain", wire.terrain("s", "p", seq, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                                       data=wire.encode_heights(z), ts=clock.ts(c.t - scan_lag)))
        done = c.step()
        c.t += 0.25
    return c, w, logs, done


def test_controller_closes_the_loop_against_the_world():
    c, w, _, done = loopback(EXAMPLE, EXAMPLES)
    assert done
    t = w.truth()
    assert t["zones"]["pad"]["within_tolerance_pct"] >= 95
    assert abs(c.est_keep - 0.70) < 0.08                         # learned compaction from surveys
    assert c.report()["finished"] and "true_ground" not in c.report()


def test_controller_withdraws_a_pass_the_machine_cannot_run():
    """`orderFailed` withdraws the pass at once (not after 30 minutes), and the machine, which is safe and
    idle again, gets its next order only after a short cooldown, so a failing order cannot spin."""
    from loam.runtime.controller import ORDER_RETRY_MIN
    m = load(EXAMPLE)
    c = Controller(m, EXAMPLES, _FakeBus(), Clock(0, 60, 1796108400.0), log=lambda x: None)
    for n in c.machines:
        c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
        c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=1796108400.0))
    c.handle("loam/v0/s/terrain", wire.terrain("s", "p", 1, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                               data=wire.encode_heights(c.z), ts=1796108400.0))
    c.t = 0.1
    c.step()
    n = next(iter(c.inflight))
    oid, before = c.order_of[n], c.plan_dz.copy()
    other = wire.state("s", n, 2, "someone-elses", 0, 0, 0, False, 0.1, "stopped",
                       [wire.error("orderFailed", "vehicleNavFailed")], ts=1796108406.0)
    c.handle(f"loam/v0/s/{n}/state", other)
    assert n in c.inflight                                        # not about this pass
    c.handle(f"loam/v0/s/{n}/state", {**other, "orderId": oid})
    assert n not in c.inflight and c.abandoned == 1
    assert np.abs(c.plan_dz).sum() < np.abs(before).sum()
    c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 3, None, 0, 0, 0, False, 0, ts=1796108407.0))
    c.t = 0.2
    assert not c.machine_ready(n)                                 # safe and idle, but cooling down
    c.step()
    assert n not in c.inflight
    c.t = 0.1 + ORDER_RETRY_MIN + 0.1
    for k in c.machines:                                          # states stay fresh
        c.mstate[k]["seen"] = time.time()
    assert c.machine_ready(n)
    c.step()
    assert n in c.inflight and c.order_of[n] != oid               # then a fresh order


def test_controller_withdraws_a_pass_from_a_lost_machine():
    m = load(EXAMPLE)
    c = Controller(m, EXAMPLES, _FakeBus(), Clock(0, 60, 1796108400.0), log=lambda x: None)
    for n in c.machines:
        c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
        c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=1796108400.0))
    c.handle("loam/v0/s/terrain", wire.terrain("s", "p", 1, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                               data=wire.encode_heights(c.z), ts=1796108400.0))
    c.t = 0.1
    c.step()
    assert c.inflight
    n = next(iter(c.inflight))
    before = c.plan_dz.copy()
    c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "connectionBroken"})
    assert n not in c.inflight and c.abandoned == 1
    assert np.abs(c.plan_dz).sum() < np.abs(before).sum()
    assert not c.machine_ready(n)
    c.handle("loam/v0/s/estop", {"pressed": True})
    assert c.apply_rules()["stop"]
    c.handle("loam/v0/s/detection", wire.detection("s", "cam", 1, "person", c.mstate["dz2"]["x"] + 10, 0))
    assert c.proximity("person", "machine") == pytest.approx(10 * 0.3048)


# -- monitor --------------------------------------------------------------------------------------
def test_monitor_frames_and_replay_page(tmp_path):
    m = load(EXAMPLE)
    fr = Frames(m, EXAMPLES)
    s = Sim(m, EXAMPLES, events=Events.none(), log=lambda x: None)
    z = s.z
    assert fr.feed("loam/v0/pad_north/terrain", wire.terrain("pad_north", "p", 1, (0, 0), 1, z.shape,
                                                            data=wire.encode_heights(z), ts=1796108400)) == "map"
    fr.feed("loam/v0/pad_north/dz1/state", wire.state("pad_north", "dz1", 2, "a-1", 5, 5, 0, True, 0.5, ts=1796108460))
    fr.feed("loam/v0/pad_north/status", wire.status("pad_north", 3, {"tasks": {"strip_A": {"done": 3.0, "passes": 9}},
                                                                     "rules": ["rain"], "replans": 1}, 1796108470))
    fr.feed("loam/v0/pad_north/estop", {"pressed": True, "timestamp": "2026-12-01T07:02:00Z"})
    f = fr.frame(full=True)
    assert f["map"] and f["machines"]["dz1"]["progress"] == 0.5 and f["estop"]
    assert [e["text"] for e in f["events"]][:2] == ["SITE E-STOP pressed", "replan #1"]
    rec = tmp_path / "rec.jsonl"
    rec.write_text(json.dumps({"site": fr.site_info}) + "\n" + json.dumps(f) + "\n" + json.dumps(fr.frame()) + "\n")
    r = replay_html(str(rec), str(tmp_path / "replay.html"))
    html = (tmp_path / "replay.html").read_text()
    assert r["frames"] == 2 and r["maps"] == 1 and '"site":"pad_north"' in html and "/*LOAM_DATA*/" not in html


def test_monitor_prometheus_metrics():
    m = load(EXAMPLE)
    fr = Frames(m, EXAMPLES)
    design = fr.design.copy()
    z = design + np.where(np.arange(design.shape[1]) % 4 == 0, 0.5, 0.01)      # a quarter of columns out
    fr.feed("loam/v0/pad_north/terrain", wire.terrain("pad_north", "p", 1, (0, 0), 1, z.shape,
                                                      data=wire.encode_heights(z), ts=1796108400))
    st = wire.state("pad_north", 'dz"1', 2, "a-1", 5, 5, 0, True, 0.25, "stopped", ts=1796108460)
    st["stopLatencyS"] = 0.004
    fr.feed('loam/v0/pad_north/dz"1/state', st)
    fr.feed("loam/v0/pad_north/status", wire.status("pad_north", 3, {"t": 42.0, "tasks": {"strip_A": {
        "done": None, "passes": 9}}, "surveys": 4, "replans": 1, "probe_age_min": 3.5}, 1796108470))
    text = fr.metrics()
    pc = pytest.importorskip("prometheus_client.parser")
    fams = {f.name: f for f in pc.text_string_to_metric_families(text)}
    val = {(x.name, tuple(sorted(x.labels.items()))): x.value for f in fams.values() for x in f.samples}

    def get(name, **lab):
        return val[(name, tuple(sorted({"site": "pad_north", **lab}.items())))]
    assert get("loam_machine_progress", machine='dz"1') == 0.25               # label escaping round-trips
    assert get("loam_machine_safety", machine='dz"1', state="stopped") == 1
    assert get("loam_machine_stop_latency_seconds", machine='dz"1') == 0.004
    assert get("loam_task_passes_total", task="strip_A") == 9 and get("loam_task_done", task="strip_A") == 0
    assert get("loam_site_minutes") == 42 and get("loam_probe_age_minutes") == 3.5
    ok = np.isfinite(z - design)
    expect = float(np.mean(np.abs((z - design)[ok]) <= fr.tol))
    assert 0.7 < expect < 0.8 and get("loam_terrain_within_tolerance_ratio") == pytest.approx(expect, rel=1e-5)
    assert fams["loam_messages"].type == "counter" and get("loam_messages_total") == 3


# -- the real thing: a broker and one process per role ------------------------------------------------
def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("mosquitto") is None and not os.path.exists("/usr/sbin/mosquitto"),
                    reason="needs the mosquitto broker")
def test_site_over_mqtt_with_faults(tmp_path):
    port = _free_port()
    broker = subprocess.Popen([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-p", str(port)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    faults = {"person": [{"from": 20, "to": 23, "near": "dz1", "distance_m": 9.0}], "probe_outage": [],
              "rain": [], "gnss_float": [{"machine": "dz2", "from": 30, "to": 36}],
              "link_loss": [{"machine": "dz1", "at": 50, "wall_s": 2.0}],
              "estop": [{"at": 70, "for_min": 2, "resume_after_min": 1}],
              "crash": [{"machine": "dz2", "at": 90}]}
    (tmp_path / "faults.json").write_text(json.dumps(faults))
    b = f"127.0.0.1:{port}"
    try:
        time.sleep(0.5)
        site = subprocess.Popen([sys.executable, "-m", "loam.cli", "simulate-site", EXAMPLE, "--broker", b,
                                 "--speed", "300", "--faults", str(tmp_path / "faults.json"), "--quiet"],
                                stdout=subprocess.PIPE, text=True)
        time.sleep(1.0)
        ctl = subprocess.run([sys.executable, "-m", "loam.cli", "run", EXAMPLE, "--broker", b, "--json"],
                             capture_output=True, text=True, timeout=600)
        out, _ = site.communicate(timeout=120)
    finally:
        broker.terminate()
        broker.wait(10)
    log = ctl.stdout[:ctl.stdout.index("{\n")]
    # releasing the e-stop (minute 72) restarts nothing; machines work again only after the resume (minute 73)
    lines = log.splitlines()
    k = next(i for i, ln in enumerate(lines) if "site e-stop released" in ln)
    first_ok = next(ln for ln in lines[k:] if "safety ok" in ln)
    h, mnt = first_ok.split("]")[0].strip("[").split(":")
    assert int(h) * 60 + float(mnt) >= 73.0, first_ok
    rep = json.loads(ctl.stdout[ctl.stdout.index("{\n"):])
    truth = json.loads(out)
    assert rep["finished"], log
    assert truth["truth"]["zones"]["pad"]["within_tolerance_pct"] >= 95
    assert abs(truth["truth"]["ledger_error"]) < 1e-6
    assert truth["exit_codes"]["dz2"] == 3                          # it really crashed
    for expect in ("rule people_near_machines: ACTIVE", "dz2: safety degraded (gnssQuality)",
                   "dz1: connectionBroken", "SITE E-STOP pressed", "dz2: connectionBroken"):
        assert expect in log, expect


def test_physics_dozer_follows_its_path():
    pytest.importorskip("pybullet")
    from loam.grid import SiteGrid
    from loam.runtime.physics import Body, follow
    m = load(EXAMPLE)
    g = SiteGrid(m, EXAMPLES)
    _, z = g.design_and_existing()
    b = Body(np.pad(z, 20, mode="edge"), g.x0 - 20, g.y0 - 20, g.cell, 0.3048, "dozer", (10, 50))
    path = [b.pose()[:2], (140, 50), (140, 20)]
    trav, last, dev = 0.0, b.pose()[:2], []
    for _ in range(200):
        b.drive(follow(b, path, trav, 0), 1.34, 1.0, step=1 / 30)
        x, y = b.pose()[:2]
        trav += float(np.hypot(x - last[0], y - last[1]))
        last = (x, y)
        if x < 135:
            dev.append(abs(y - 50))
    x, y = b.pose()[:2]
    b.close()
    assert np.hypot(x - 140, y - 20) < 3 and max(dev) < 3     # reached the end, stayed on the line (ft)
    assert np.degrees(b.max_tilt) < 15                          # did not tip
