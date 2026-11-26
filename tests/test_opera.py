"""OPERA (PWRI's ROS 2 earthwork platform) machines driven through Loam Wire.

These tests start real ROS 2 graphs:

* the ZX200 arm on ros2_control mock hardware with a JointTrajectoryController;
* Nav2 (planner, Regulated Pure Pursuit, behavior-tree navigator) driving a kinematic IC120.

They are skipped unless LOAM_ROS_RUN gives the command prefix that runs a program
inside a ROS 2 Jazzy environment (e.g. "micromamba run -n ros"), and
LOAM_OPERA_WS points at a built workspace holding zx200_description and
com3_msgs from github.com/pwri-opera.
"""
import json
import math
import os
import shlex
import signal
import subprocess
import time

import numpy as np
import pytest

from loam import wire
from loam.checker import check, has_errors
from loam.grid import SiteGrid, polygon_mask
from loam.landxml import write_grid_tin
from loam.parser import parse

pytest.importorskip("paho.mqtt")
from test_bridges import Controller

from loam.runtime.opera import Chain

# How to run a program inside ROS 2, and where the built OPERA workspace is. Both come from the environment.
RUN = shlex.split(os.environ.get("LOAM_ROS_RUN", ""))
WS = os.environ.get("LOAM_OPERA_WS", "")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
need_ros = pytest.mark.skipif(not RUN or not os.path.isdir(WS), reason="needs LOAM_ROS_RUN and LOAM_OPERA_WS")
FT = 0.3048


DOMAIN = str(1 + os.getpid() % 100)            # this run's own ROS graph: no cross-talk with stray nodes


def ros(cmd: str, log=None, **kw) -> subprocess.Popen:
    """A shell command inside the ROS environment with the OPERA workspace sourced; output to `log`."""
    full = (f"source {WS}/install/setup.bash && export PYTHONPATH={ROOT}:$PYTHONPATH "
            f"ROS_DOMAIN_ID={DOMAIN} && exec {cmd}")
    if log is not None:
        with open(log, "w", encoding="utf-8") as f:           # the child keeps its own copy of the handle
            return subprocess.Popen([*RUN, "bash", "-c", full], start_new_session=True, stdout=f,
                                    stderr=subprocess.STDOUT, **kw)
    return subprocess.Popen([*RUN, "bash", "-c", full], start_new_session=True, **kw)


def stop(p: subprocess.Popen) -> None:
    """Stop a process started by `ros` and everything it started: SIGTERM to the group, then SIGKILL after 20 s."""
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def read(path) -> str:
    """A log file's text (bytes that are not valid UTF-8 are replaced, not fatal)."""
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def jsonl(path) -> list[dict]:
    """A JSON-lines file as a list of objects."""
    return [json.loads(ln) for ln in read(path).splitlines() if ln.strip()]


def wait_for(path: str, text: str, timeout: float = 90) -> None:
    """Wait until `text` appears in a log file. On timeout, fail and show the end of the log."""
    end = time.time() + timeout
    while time.time() < end:
        if os.path.exists(path) and text in read(path):
            return
        time.sleep(0.5)
    raise AssertionError(f"{text!r} not in {path}:\n" + read(path)[-3000:])


# -- the site: imperial units, a 1 % sloped design plane at about 100 m (map frame = site grid in metres)
X0, Y0 = 10 / FT, 10 / FT                       # where the excavator stands (ft)
Z0 = 100 / FT


def design(x, y):
    """The design surface (ft): a plane sloping down 1 % to the east and up 0.5 % to the north, through the
    excavator's position at 100 m.
    """
    return Z0 - 0.01 * (x - X0) + 0.005 * (y - Y0)


# The site program; each test fills in the size and the pit's corners.
SRC = """loam 0.0.2
site yard {{ crs "LOCAL:ft"; units ft; design "d.xml"; boundary rect (0, 0) ({w}, {w}); grid 1 ft }}
probe p {{ kind static; sensor lidar; rate 5 min; accuracy 0.05 ft }}
machine ex1 {{ class excavator; bucket 1.2 yd3 }}
machine dt1 {{ class truck; capacity 10 yd3 }}
zone pit {{ role cut; area rect ({x1}, {y1}) ({x2}, {y2}); tolerance ±0.1 ft }}
task dig {{ op cut; zone pit; by class excavator; spoil -> pit; until residual(pit).high < 0.1 ft }}
"""


@pytest.fixture
def site(tmp_path):
    """An 80 x 80 ft site with an excavator and a truck, and a pit 14 to 22 ft in front of the excavator. Returns
    (model, folder, pit rectangle).
    """
    w = 80
    win = (round(X0 + 14), round(Y0 - 4), round(X0 + 22), round(Y0 + 4))        # 4.3-6.7 m out, 2.4 m wide
    xs = ys = np.arange(0.0, w + 1.0)
    gx, gy = np.meshgrid(xs, ys)
    write_grid_tin(str(tmp_path / "d.xml"), "FG", xs, ys, design(gx, gy), "ft")
    src = SRC.format(w=w, x1=win[0], y1=win[1], x2=win[2], y2=win[3])
    (tmp_path / "s.loam").write_text(src)
    m, d = check(parse(src))
    assert not has_errors(d), [x.msg for x in d]
    return m, tmp_path, win


def last_joints(tmp) -> dict:
    """The arm's most recent complete joint-state record (the very last line may still be being written)."""
    return jsonl(tmp / "rig" / "joints.jsonl")[-2]


def urdf() -> str:
    """The ZX200's URDF, generated from OPERA's own xacro description."""
    p = ros("bash -c 'xacro $(ros2 pkg prefix zx200_description)/share/zx200_description/urdf/zx200.xacro'",
            stdout=subprocess.PIPE, text=True)
    out, _ = p.communicate(timeout=120)
    assert "<robot" in out
    return out


@need_ros
@pytest.mark.slow
def test_zx200_cuts_to_the_design_surface(site, mqtt_broker):
    """The excavator bridge drives OPERA's ZX200 arm through a real ros2_control trajectory controller.

    A survey reports 1 ft of material on the pit. The order says remove it. Afterwards the bucket tip's path is
    rebuilt from every recorded joint state: it never goes more than 10 mm below design, stays within 15 mm of
    design while dragging, and covers over 90 % of the pit. Midway, a stop freezes the arm and a resume finishes
    the job.
    """
    m, tmp, win = site
    g = SiteGrid(m, str(tmp))
    design_grid = g.surface(m.site["design"])
    (tmp / "zx200.urdf").write_text(urdf())
    rig = ros(f"python {HERE}/ros/excavator_rig.py {tmp}/zx200.urdf {tmp}/rig {10.0} {10.0} {100.0} 0.0",
              log=tmp / "rig.log")
    bridge = None
    ctl = Controller(mqtt_broker, "yard", "ex1")
    try:
        wait_for(str(tmp / "rig.log"), "RIG READY", 120)
        bridge = ros(f"python -m loam.cli opera {tmp}/s.loam --machine ex1 --namespace zx200 --kind excavator "
                     f"--broker {mqtt_broker}", log=tmp / "bridge.log")
        ctl.wait(lambda s: s["safetyState"] == "ok", timeout=60)
        # the survey: 1 ft of material on the window, design elsewhere
        poly = [(win[0], win[1]), (win[2], win[1]), (win[2], win[3]), (win[0], win[3])]
        mk = polygon_mask(poly, g.xs, g.ys)
        ground = np.where(mk, design_grid + 1.0, design_grid)
        ctl.bus.pub(wire.topic("yard", None, "terrain"), wire.terrain(
            "yard", "p", 1, (g.x0, g.y0), g.cell, g.shape, data=wire.encode_heights(ground)), qos=1)
        time.sleep(1.0)
        i0, i1 = np.searchsorted(g.xs, win[0]), np.searchsorted(g.xs, win[2], "right")
        j0, j1 = np.searchsorted(g.ys, win[1]), np.searchsorted(g.ys, win[3], "right")
        tile = (design_grid - ground)[j0:j1, i0:i1] * FT                     # planned change, metres
        order = wire.order("yard", "ex1", 1, "cut-1", 0, "dig", "cut", [list(p) for p in poly],
                           {"pattern": wire.encode_heights(tile), "shape": list(tile.shape)}, {})
        ctl.send("order", order)
        ctl.wait(lambda s: s["orderId"] == "cut-1" and 0.3 < s["progress"] < 1, timeout=300)
        # stop mid-cut: the arm holds still; resume: it rejoins the path and finishes
        ctl.send("instantAction", wire.instant_action("yard", "ex1", 2, "stop", "test"))
        ctl.wait(lambda s: s["safetyState"] == "stopped")
        time.sleep(1.0)
        before = last_joints(tmp)
        time.sleep(2.0)
        after = last_joints(tmp)
        assert max(abs(after[k] - before[k]) for k in before if k != "t") < 1e-3
        ctl.send("instantAction", wire.instant_action("yard", "ex1", 3, "resume", "test"))
        done = ctl.wait(lambda s: s["orderId"] == "cut-1" and s["progress"] == 1.0, timeout=600)
        assert done["safetyState"] == "ok"
    finally:
        ctl.close()
        if bridge is not None:
            stop(bridge)
        stop(rig)
    # where the bucket tip actually went: forward kinematics of every recorded joint state
    chain = Chain((tmp / "zx200.urdf").read_text())
    rows = jsonl(tmp / "rig" / "joints.jsonl")
    tips = np.array([chain.tip(r) for r in rows]) / FT + [X0, Y0, Z0]
    inside = polygon_mask(poly, g.xs, g.ys)
    xi, yj = np.round((tips[:, 0] - g.x0) / g.cell).astype(int), np.round((tips[:, 1] - g.y0) / g.cell).astype(int)
    ok = (xi >= 0) & (xi < g.shape[1]) & (yj >= 0) & (yj < g.shape[0])
    in_win = np.zeros(len(tips), bool)
    in_win[ok] = inside[yj[ok], xi[ok]]
    tgt = design(tips[:, 0], tips[:, 1])
    over = (tips[in_win, 2] - tgt[in_win]) * FT * 1000                      # mm above grade, every sample
    # dragging along the grade: moving mostly sideways, near the surface (approach and lift excluded)
    t = np.array([r["t"] for r in rows])
    v = np.gradient(tips * FT, t, axis=0)
    vxy, vz = np.hypot(v[:, 0], v[:, 1]), np.abs(v[:, 2])
    drag = in_win & (vxy > 0.02) & (vz < 0.25 * vxy) & ((tips[:, 2] - tgt) * FT < 0.1)
    err = (tips[drag, 2] - tgt[drag]) * FT * 1000
    print(f"\nZX200 tip: deepest {over.min():+.1f} mm vs grade over {in_win.sum()} samples in the window; "
          f"dragging {drag.sum()} samples: mean {err.mean():+.1f} mm, RMS {np.sqrt((err ** 2).mean()):.1f} mm, "
          f"max |e| {np.abs(err).max():.1f} mm")
    assert over.min() > -10.0                                    # never more than 10 mm below grade
    assert drag.sum() > 200
    assert np.abs(err).max() < 15.0                              # on the grade within 15 mm while dragging
    cover = {(a, b) for a, b in zip(xi[drag], yj[drag], strict=True)}
    cells = {(i, j) for j, i in np.argwhere(inside)}
    assert len(cover & cells) / len(cells) > 0.9                 # the bucket passed over the window


# -- IC120 carrier on Nav2 ------------------------------------------------------------------------------
def _seg_dist(p, a, b):
    """The distance from point `p` to the line segment from `a` to `b`."""
    ab, ap = b - a, p - a
    t = np.clip((ap @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return float(np.hypot(*(ap - t * ab)))


@need_ros
@pytest.mark.slow
def test_ic120_hauls_dumps_and_stops_on_nav2(site, mqtt_broker):
    """The carrier bridge drives a tracked dump carrier through the real Nav2 stack.

    It runs a work line, hauls to the deposit, raises and lowers the vessel there while standing still, and
    returns, never exceeding the order's 0.8 m/s. A stop halts it and a resume carries on. When odometry disappears
    it is reported stopped. Finally the recorded track must lie within 30 cm of the work line.
    """
    _, tmp, _ = site
    rig = ros(f"python {HERE}/ros/carrier_rig.py {HERE}/ros/nav2_ic120.yaml {tmp}/rig 5.0 10.0 0.0",
              log=tmp / "rig.log")
    bridge = None
    ctl = Controller(mqtt_broker, "yard", "dt1")
    try:
        wait_for(str(tmp / "rig.log"), "SIM UP", 120)       # Nav2 is still coming up: the bridge must wait for it
        bridge = ros(f"python -m loam.cli opera {tmp}/s.loam --machine dt1 --namespace ic120 --kind carrier --dump "
                     f"--broker {mqtt_broker}", log=tmp / "bridge.log")
        first = ctl.wait(lambda s: s["safetyState"] == "ok", timeout=60)
        assert math.hypot(first["pose"]["x"] - 5 / FT, first["pose"]["y"] - 10 / FT) < 0.1
        # 1: through a work window, to the deposit (dump there), and back; at most 0.8 m/s
        wa = [[60, 25], [100, 25], [100, 35], [60, 35]]                       # ft; work line y = 30 ft
        dep = [[60, 60], [70, 60], [70, 70], [60, 70]]                        # deposit centre (65, 65) ft
        order = wire.order("yard", "dt1", 1, "h-1", 0, "dig", "cut", wa, {}, {"maxSpeed": 0.8},
                           deposit={"window": dep})
        ctl.send("order", order)
        end = ctl.wait(lambda s: s["orderId"] == "h-1" and s["progress"] == 1.0, timeout=400)
        assert end["safetyState"] == "ok"
        dumps = jsonl(tmp / "rig" / "dumps.jsonl")
        assert [round(math.degrees(d["angle"])) for d in dumps] == [60, 0]
        assert all(60 <= d["x"] / FT <= 70 and 60 <= d["y"] / FT <= 70 for d in dumps)   # in the deposit window
        assert math.hypot(dumps[0]["x"] - dumps[1]["x"], dumps[0]["y"] - dumps[1]["y"]) < 0.01  # standing still
        # 2: stop: the crawler halts and stays put; resume: it carries on and finishes
        order2 = wire.order("yard", "dt1", 2, "h-2", 0, "dig", "cut", [[100, 25], [20, 25], [20, 35], [100, 35]],
                            {}, {"maxSpeed": 1.0})
        ctl.send("order", order2)
        ctl.wait(lambda s: s["orderId"] == "h-2" and 0.2 < s["progress"] < 0.9, timeout=120)
        ctl.send("instantAction", wire.instant_action("yard", "dt1", 3, "stop", "test"))
        ctl.wait(lambda s: s["safetyState"] == "stopped")
        time.sleep(3.0)
        a = ctl.latest()
        time.sleep(2.0)
        b = ctl.latest()
        assert math.hypot(b["pose"]["x"] - a["pose"]["x"], b["pose"]["y"] - a["pose"]["y"]) < 0.05
        ctl.send("instantAction", wire.instant_action("yard", "dt1", 4, "resume", "test"))
        ctl.wait(lambda s: s["orderId"] == "h-2" and s["progress"] == 1.0, timeout=300)
        # 3: no odometry: the machine is reported stopped, not driven blind
        (tmp / "rig" / "odom_off").touch()
        lost = ctl.wait(lambda s: any(e["code"] == "vehicleLinkLost" for e in s["errors"]), timeout=10)
        assert lost["safetyState"] == "stopped"
    finally:
        ctl.close()
        if bridge is not None:
            stop(bridge)
        stop(rig)
    # how closely Nav2 drove Loam's path for order 1, and its speed
    poses = jsonl(tmp / "rig" / "poses.jsonl")
    t_dump = dumps[0]["t"]
    seg = [p for p in poses if p["t"] < t_dump]
    xy = np.array([[p["x"], p["y"]] for p in seg]) / FT
    a_pt, b_pt = np.array([60.0, 30.0]), np.array([100.0, 30.0])
    k_a = int(np.argmax(np.hypot(*(xy - a_pt).T) < 2.5))      # one end of the work line (Nav2 passes a pose at 0.5 m)
    k_b = k_a + int(np.argmax(np.hypot(*(xy[k_a:] - b_pt).T) < 2.5))   # then the other
    on_line = xy[k_a:k_b + 1]
    on_line = on_line[(on_line[:, 0] >= a_pt[0]) & (on_line[:, 0] <= b_pt[0])]    # from the line's start to its end
    off = max(_seg_dist(p, a_pt, b_pt) for p in on_line) * FT
    vmax = max(abs(p["v"]) for p in seg)
    dev = np.array([_seg_dist(p, a_pt, b_pt) for p in on_line]) * FT * 100        # cm
    along = (on_line[:, 0] - a_pt[0]) * FT
    print(f"\nIC120 on Nav2: {len(on_line)} samples along the work line, max {off * 100:.1f} cm off it "
          f"(after 2 m: max {dev[along > 2].max():.1f}, RMS {np.sqrt((dev[along > 2] ** 2).mean()):.1f} cm); "
          f"top speed {vmax:.2f} m/s (limit 0.8)")
    assert len(on_line) > 50 and k_b > k_a
    assert off < 0.30                                       # Regulated Pure Pursuit on a tracked base
    assert vmax <= 0.8 + 1e-6
