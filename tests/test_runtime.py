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
from collections import Counter

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES, wire_valid

from loam import wire
from loam.cli import load, main
from loam.landxml import grid_tin, tin_boundary, write_tin
from loam.plan import pick_surfaces, plan, simplify
from loam.sim import Events, Sim

pytest.importorskip("paho.mqtt")
from loam.monitor import Frames, replay_html
from loam.runtime.bus import Bus, Clock
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
    """`loam plan` turns a LandXML with an existing and a design surface into a Loam program that checks clean and,
    simulated, finishes the job within tolerance.
    """
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
    """Surfaces are recognised by name ("Existing Ground", "Proposed Grade"), a fleet file supplies the machines, and
    unclear or impossible inputs are refused with a reason.
    """
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
    """The outline of a TIN is found from its edges, and a straight-sided outline simplifies to its four corners."""
    pts, faces = grid_tin(np.arange(0.0, 5), np.arange(0.0, 4), np.zeros((4, 5)))
    ring = tin_boundary(pts, faces)
    assert len(ring) == 14
    assert sorted(simplify(ring, 0.1)) == [(0.0, 0.0), (0.0, 3.0), (4.0, 0.0), (4.0, 3.0)]


# -- wire ----------------------------------------------------------------------------------
def test_wire_timestamps_keep_milliseconds():
    """Timestamps are written to the millisecond when there are any, and read back to the same instant."""
    h = wire.header(1, 1796108400.25)
    assert h["timestamp"] == "2026-12-01T07:00:00.250Z"
    assert wire.parse_ts(h["timestamp"]) == pytest.approx(1796108400.25)
    assert wire.header(2, 1796108400.0)["timestamp"] == "2026-12-01T07:00:00Z"
    assert wire.parse_ts(wire.header(3, 1796108400.9996)["timestamp"]) == pytest.approx(1796108401.0)


# -- world physics against the planner -------------------------------------------------------------
class _Replay(Sim):
    """The reference simulator, with every order also sent to the world."""

    def __init__(self, *a, **k):
        """A simulator plus a World on the same site."""
        super().__init__(*a, **k)
        self.w = World(self.m, EXAMPLES)
        self.orders = {}

    def emit(self, topic, msg):
        """Keep each machine's latest order instead of writing it out."""
        if topic.endswith("/order"):
            self.orders[msg["machine"]] = msg

    def land(self, ps):
        """When a pass lands in the simulator, give the same order to the world."""
        super().land(ps)
        self.w.apply(self.orders[ps.machine])


def test_world_moves_the_earth_the_orders_describe():
    """The world's physics, driven only by the order messages, ends up with the same ground as the reference simulator
    (within blade noise).

    This shows an order carries everything a machine needs: nothing is passed on the side. A repeated report of the
    same order moves no earth twice.
    """
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
    """A bus that only keeps what is published, so a test can play the machines."""
    def __init__(self):
        """Start with nothing published."""
        self.out = []

    def pub(self, topic, msg, **k):
        """Keep the message."""
        self.out.append((topic, msg))


def loopback(path, base, pass_min=3.0, scan_lag=0.0, max_min=1500.0):
    """Run the real controller against the real world physics in one process, stepping time by hand.

    Every pass takes `pass_min` site minutes, after which the world moves the earth and the machine reports done.
    The probe scans at the program's rate with its stated accuracy; with `scan_lag` each scan reaches the controller
    that many minutes after it was taken (stamped with the time it was taken), as when a scan takes a while to
    process. Returns the controller, the world, the log lines and whether the job finished.
    """
    m = load(path)
    bus, clock, logs = _FakeBus(), Clock(0, 60, 1796108400.0), []
    c = Controller(m, base, bus, clock, log=logs.append)
    w = World(m, base)
    c.t, seq, last_scan = 0.0, 0, -1e9
    p = next(iter(m.probes.values()))
    rate, sigma = p["rate"].to("min"), p["accuracy"].to(c.unit) / 2
    rng = np.random.default_rng(1)
    work: dict = {}
    scans: list = []                                              # taken, not yet delivered: (time, heights)
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
            scans.append((c.t, w.z + rng.normal(0, sigma, w.z.shape)))
        while scans and c.t >= scans[0][0] + scan_lag:
            taken, z = scans.pop(0)
            c.handle("loam/v0/s/terrain", wire.terrain("s", "p", seq, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                                       data=wire.encode_heights(z), ts=clock.ts(taken)))
        done = c.step()
        c.t += 0.25
    return c, w, logs, done


def test_controller_closes_the_loop_against_the_world():
    """The controller, seeing only surveys and machine states, finishes the example within tolerance on the true
    ground and learns the soil's real compaction factor (0.70; it starts believing 0.80).
    """
    c, w, _, done = loopback(EXAMPLE, EXAMPLES)
    assert done
    t = w.truth()
    assert t["zones"]["pad"]["within_tolerance_pct"] >= 95
    assert abs(c.est_keep - 0.70) < 0.08                         # learned compaction from surveys
    assert c.report()["finished"] and "true_ground" not in c.report()


def test_late_surveys_do_not_confuse_the_soil_estimates():
    """A survey that arrives two minutes after it was taken does not show the passes finished in between. Those
    passes stay in the plan, and so does the fill and rolling they are expected to show at the next survey:
    otherwise their effect would be found on the ground with nothing expected there, and blamed on the soil.
    The estimates must stay as steady as with prompt surveys, and the job must not need many more replans.
    """
    import re
    prompt, _, _, done0 = loopback(EXAMPLE, EXAMPLES)
    c, w, logs, done = loopback(EXAMPLE, EXAMPLES, scan_lag=2.0)
    assert done0 and done
    keep = [float(x) for line in logs for x in re.findall(r"keep now ([0-9.]+)", line)]
    assert keep and max(keep) - min(keep) < 0.1 and not {0.5, 1.0} & set(keep)     # was 0.50 to 0.85 before the fix
    assert abs(c.est_keep - 0.70) < 0.05 and abs(c.est_swell - 1.30) < 0.1
    assert len(c.replans) <= len(prompt.replans) + 3                              # was 13 against 6
    assert w.truth()["zones"]["pad"]["within_tolerance_pct"] >= 95


def test_idle_while_a_task_is_paused_is_not_a_stall():
    """When the only work left belongs to a task a rule has paused (rolling, in the rain), the controller says once
    what it is waiting for and does not count a replan at every survey. A task that can start because the one
    before it finished is not called a stall either: it starts with the first survey that shows the work before it.
    """
    c, _, logs, done = loopback(EXAMPLE, EXAMPLES)
    assert done
    assert not [line for line in logs if "stalled" in line and "finish_pad" in line]
    began = next(line for line in logs if "task finish_pad started" in line)
    assert float(began[4:8]) % 5 == 0                             # on a survey (every 5 min), not between two
    m = load(EXAMPLE)
    c = Controller(m, EXAMPLES, _FakeBus(), Clock(0, 60, 1796108400.0), log=logs.append)
    c.t, c.has_survey, c.last_probe, c.last_rebase = 10.0, True, 10.0, 5.0
    c.meas, c.pred = c.design.copy(), c.design.copy() + 1.0       # nothing left to move, on the measured ground
    c.loose[c.masks["C"]] = c.p_loose[c.masks["C"]] = 0.2         # but the fill in C is still to be rolled
    c.started = dict.fromkeys(c.started, 0.0)
    c.done_at.update(strip_A=5.0)
    c.rain = 1.0                                                  # the example pauses compact_C in the rain
    for n in c.machines:
        c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
        c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=1796108400.0 + 600))
        c.mstate[n]["seen"] = time.time()
    del logs[:]
    for k in range(3):                                            # three surveys go by
        c.last_probe = c.t = 10.0 + 5 * k + 5
        for n in c.machines:
            c.mstate[n]["seen"] = time.time()
        c.step()
    waiting = [line for line in logs if "waiting:" in line]
    assert len(waiting) == 1 and "compact_C is paused" in waiting[0], logs
    assert not c.replans and np.array_equal(c.pred, c.meas)       # kept on the survey, without counting replans


def test_a_new_phase_is_planned_from_a_survey_that_shows_the_last_pass():
    """When a task finishes and the next can start, the latest survey may have been taken before the last pass
    landed. It still shows the ground that pass has since cut, and planning from it would order that work again.
    So a phase that has not begun waits for the next survey: nothing is planned from the stale one, on its word or
    on the prediction's, and the prediction is not rebased on it.
    """
    c, _, _, _ = loopback(EXAMPLE, EXAMPLES, max_min=1.0)
    for name in ("strip_A", "top_up_C", "compact_C"):             # only the finishing is left
        c.done_at[name] = c.started[name] = 1.0
    for n in list(c.inflight):
        c.abandon(n, "test")
    pad = c.masks["pad"]
    west, east = pad & (c.xs[None, :] < 20), pad & (c.xs[None, :] > 60)
    c.plan_dz[:], c.plan_dloose[:], c.p_loose[:] = 0.0, 0.0, 0.0
    c.meas, c.pred = c.design.copy(), c.design.copy()
    c.meas[west | east] += 0.3                                    # the survey: two strips of the pad stand high
    c.pred[east] += 0.3                                           # the plan: a pass has since cut the western one
    c.has_survey, c.t = True, 20.0
    c.last_probe, c.last_scan, c.last_rebase, c.last_land = 18.0, 18.0, 10.0, 19.0

    def tick():
        """One planning step with every machine reporting in, idle."""
        for n in c.machines:
            c.machines[n].busy_until = 0.0
            c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 9, None, 0, 0, 0, False, 0, ts=c.clock.ts(c.t)))
            c.mstate[n]["seen"] = time.time()
        c.step()

    before = c.pred.copy()
    tick()
    tick()
    assert c.started["finish_pad"] is None and not c.inflight and not c.replans and np.array_equal(c.pred, before)
    c.meas = c.design.copy()
    c.meas[east] += 0.3                                           # the next survey shows the western strip cut
    c.t = c.last_probe = c.last_scan = 25.0
    c.bus.out.clear()
    tick()
    assert c.started["finish_pad"] == 25.0 and c.inflight
    windows = [np.asarray(msg["window"]) for topic, msg in c.bus.out if topic.endswith("/order")]
    assert windows and all(w[:, 0].min() >= 60 for w in windows)  # work on the eastern strip only


def test_finishing_never_takes_the_simulated_blade_past_design():
    """A finishing order planned on ground that has since been cut to design moves nothing there: the simulated
    site's blade aims at the planned change but stops at design, cutting or filling, within its own error."""
    m = load(EXAMPLE)
    w = World(m, EXAMPLES)
    w.z = w.design.copy()
    j, i = np.argwhere(w.g.masks["pad"])[40]
    w.z[j, i + 1] += 0.1                                          # one cell a little high, one a little low
    w.z[j, i + 2] -= 0.1
    w.stock["sp1"] = 50.0
    tile = np.array([[-0.3, -0.3, 0.3, 0.3]]) * 0.3048            # asks for far more than is there
    order = wire.order("s", "dz1", 1, "f-1", 0, "finish_pad", "finish",
                       [[w.g.x0 + (i + a) * w.g.cell, w.g.y0 + (j + b) * w.g.cell] for a, b in
                        ((0, 0), (4, 0), (4, 1), (0, 1))],
                       {"surface": "design", "pattern": wire.encode_heights(tile), "shape": [1, 4]},
                       {"step": 0.0914}, {"source": "sp1"})
    w.apply(order)
    off = (w.z - w.design)[j, i:i + 4]
    assert np.abs(off).max() < 4 * w.soil.finish_error_m / 0.3048 + 1e-9 and np.abs(off).max() < 0.1


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


def test_a_restarted_controller_does_not_reuse_order_ids(monkeypatch):
    """Two runs of the controller on the same job give their orders different ids. With the same ids, a machine
    that had already done "strip_A-1" would report the restarted controller's "strip_A-1" finished without doing
    it. (Found by restarting the controller in the middle of a simulated job: the world ignored 56 passes.)
    """
    import loam.runtime.controller as rc
    ids = []
    for start in (1796108400.0, 1796108460.0):                    # the second run starts a minute later
        monkeypatch.setattr(rc.time, "time", lambda start=start: start)
        bus = _FakeBus()
        c = Controller(load(EXAMPLE), EXAMPLES, bus, Clock(0, 60, 1796108400.0), log=lambda x: None)
        for n in c.machines:
            c.handle(f"loam/v0/s/{n}/connection", {"machine": n, "connectionState": "online"})
            c.handle(f"loam/v0/s/{n}/state", wire.state("s", n, 1, None, 0, 0, 0, False, 0, ts=1796108400.0))
        c.handle("loam/v0/s/terrain", wire.terrain("s", "p", 1, (c.x0, c.y0), c.cell, (c.ny, c.nx),
                                                   data=wire.encode_heights(c.z), ts=1796108400.0))
        c.t = 0.1
        c.step()
        sent = [m["orderId"] for t, m in bus.out if t.endswith("/order")]
        assert sent and set(sent) == set(c.order_of.values())    # what was sent is what is being waited for
        ids.append(set(sent))
    assert not ids[0] & ids[1]


def test_controller_withdraws_a_pass_from_a_lost_machine():
    """A machine whose link breaks loses its pass at once and gets no more orders. Also here: the site e-stop stops
    everything, and a detected person's distance is reported in metres.
    """
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
    """The dashboard turns Wire messages into frames and events, and a recording becomes a self-contained replay page.
    """
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
    """The /metrics text is valid Prometheus, read back here with the official parser, with correct values and with
    awkward label characters escaped.
    """
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
        """One sample's value, by metric name and labels."""
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
    """A TCP port that is free right now."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("mosquitto") is None and not os.path.exists("/usr/sbin/mosquitto"),
                    reason="needs the mosquitto broker")
def test_site_over_mqtt_with_faults(tmp_path):
    """The whole system as on a site: a broker, and the world, probe, each machine and the controller as separate
    processes.

    The fault script has a person walk up to a machine, a machine lose RTK, a link drop, the e-stop pressed and
    released, and a machine's computer die. The job must still finish within tolerance, with every event in the log
    and no machine moving between e-stop release and the supervisor's resume. Every message any of them sent on
    the Wire topics must be valid against the published schema.
    """
    port = _free_port()
    broker = subprocess.Popen([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-p", str(port)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # The person stays six site minutes: at 300 times real speed that is 1.2 s, long enough that a busy computer
    # cannot step over the whole visit between two looks (three minutes, 0.6 s, was once missed).
    faults = {"person": [{"from": 20, "to": 26, "near": "dz1", "distance_m": 9.0}], "probe_outage": [],
              "rain": [], "gnss_float": [{"machine": "dz2", "from": 30, "to": 36}],
              "link_loss": [{"machine": "dz1", "at": 50, "wall_s": 2.0}],
              "estop": [{"at": 70, "for_min": 2, "resume_after_min": 1}],
              "crash": [{"machine": "dz2", "at": 90}]}
    (tmp_path / "faults.json").write_text(json.dumps(faults))
    b = f"127.0.0.1:{port}"
    listen = None
    try:
        time.sleep(0.5)
        listen = Bus(b, "wire-check").start(["loam/v0/#"])
        site = subprocess.Popen([sys.executable, "-m", "loam.cli", "simulate-site", EXAMPLE, "--broker", b,
                                 "--speed", "300", "--faults", str(tmp_path / "faults.json"), "--quiet"],
                                stdout=subprocess.PIPE, text=True)
        time.sleep(1.0)
        ctl = subprocess.run([sys.executable, "-m", "loam.cli", "run", EXAMPLE, "--broker", b, "--json"],
                             capture_output=True, text=True, timeout=600)
        out, _ = site.communicate(timeout=120)
        sent = listen.drain(0.5)
    finally:
        if listen is not None:
            listen.stop()
        broker.terminate()
        broker.wait(10)
    kinds = Counter(topic.rsplit("/", 1)[-1] for topic, _ in sent)
    assert kinds["state"] > 100 and kinds["order"] > 50 and kinds["status"] > 10 and kinds["estop"] == 2, kinds
    for topic, msg in sent:
        wire_valid(topic.rsplit("/", 1)[-1], msg)
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


@pytest.mark.skipif(shutil.which("mosquitto") is None and not os.path.exists("/usr/sbin/mosquitto"),
                    reason="needs the mosquitto broker")
def test_machine_is_back_online_after_the_broker_restarts():
    """A machine that loses its connection (a radio drop, a restarted broker) is reported as gone by the broker.
    When it reconnects it must say `online` again, or the controller never gives it another order; and whoever was
    listening must be subscribed again without being told.
    """
    from loam.runtime.machine import MachineSim
    exe, port = shutil.which("mosquitto") or "/usr/sbin/mosquitto", _free_port()

    def start():
        """Start the broker and wait until it listens."""
        p = subprocess.Popen([exe, "-p", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    return p
            time.sleep(0.05)
        raise AssertionError("the broker did not start")

    def states(bus, wait):
        """The connection states that arrive within `wait` seconds, in order."""
        end, out = time.time() + wait, []
        while time.time() < end and "online" not in out:
            out += [m["connectionState"] for _, m in bus.drain(0.1)]
        return out

    broker = start()
    b = f"127.0.0.1:{port}"
    listen = machine = None
    try:
        listen = Bus(b, "listener").start([wire.topic("pad_north", "dz1", "connection")])
        ms = MachineSim(load(EXAMPLE), "dz1", b, (10.0, 10.0))
        machine = ms.connect()
        assert states(listen, 5.0) == ["online"]
        broker.terminate()
        broker.wait(10)
        time.sleep(1.0)
        broker = start()
        back = states(listen, 15.0)                               # both reconnect by themselves
        assert back and back[-1] == "online", back
        assert machine.connects == 2 and listen.connects == 2
    finally:
        for bus in (machine, listen):
            if bus is not None:
                bus.stop()
        broker.terminate()
        broker.wait(10)


def test_physics_dozer_follows_its_path():
    """Under rigid-body physics a dozer follows an L-shaped path over the example's real ground: it reaches the end,
    stays within 3 ft of the line and does not tip.
    """
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
