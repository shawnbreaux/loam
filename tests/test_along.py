"""Work along a line: `along` on a task (lanes, a path on every order, machines and blades that follow it), the
swale designer (`loam swale`) and the curve tracks written for AgOpenGPS."""
import json
import math
from itertools import pairwise

import numpy as np
import pytest
from conftest import MINI, wire_valid
from test_level import _dem
from test_runtime import loopback

from loam import line, wire
from loam.checker import check
from loam.cli import load, main
from loam.export import CURVE_TAIL, agopengps
from loam.geo import Frame
from loam.grid import SiteGrid
from loam.landxml import read_tin, write_grid_tin
from loam.parser import parse
from loam.runtime.machine import MachineSim
from loam.runtime.world import World
from loam.sim import Events, Sim
from loam.swale import invert, swale

FT = 0.3048
# A line with one bend, a ditch along it (6 ft flat bottom a foot deep, sides 4:1) and a grader 6 ft wide.
LINE = [(20.0, 20.0), (60.0, 50.0), (100.0, 100.0)]
BLADE = 6.0


# -- the geometry ----------------------------------------------------------------------------------
def test_station_and_offset():
    """A point's station is the distance along the line to its nearest point, and its offset the distance from
    it: positive on the left of the line's direction, and measured from the end for a point past an end."""
    corner = [(0, 0), (10, 0), (10, 10)]
    assert line.length(corner) == 20.0
    s, off = line.project(corner, np.array([5.0, 5.0, 12.0, 8.0, -3.0, 10.0, 13.0]),
                          np.array([2.0, -2.0, 5.0, 5.0, 4.0, 14.0, 10.0]))
    assert s.tolist() == [5.0, 5.0, 15.0, 15.0, 0.0, 20.0, 20.0]
    assert off.tolist() == pytest.approx([2.0, -2.0, -2.0, 2.0, 5.0, 4.0, -3.0])
    # Repeated points are no part of a line, and one point is none at all.
    assert line.length([(0, 0), (0, 0), (3, 4), (3, 4)]) == 5.0
    with pytest.raises(ValueError, match="two different points"):
        line.project([(1, 1), (1, 1)], np.zeros(1), np.zeros(1))


def test_a_piece_of_a_line_moved_sideways():
    """A piece runs between two stations in the order asked, has a point at each corner between them, and moved
    sideways keeps its distance from the line: corners are mitred, and a very sharp one is cut across."""
    corner = [(0, 0), (10, 0), (10, 10)]
    assert line.piece(corner, 2, 15) == [(2.0, 0.0), (10.0, 0.0), (10.0, 5.0)]
    assert line.piece(corner, 15, 2) == [(10.0, 5.0), (10.0, 0.0), (2.0, 0.0)]
    assert line.piece(corner, -5, 99, 1.0) == [(0.0, 1.0), (9.0, 1.0), (9.0, 10.0)]         # inside the bend
    assert line.piece(corner, 0, 20, -1.0) == [(0.0, -1.0), (11.0, -1.0), (11.0, 10.0)]     # outside it
    for off in (1.5, -1.5):
        p = np.array(line.piece(LINE, 3, 110, off))
        assert line.project(LINE, p[:, 0], p[:, 1])[1] == pytest.approx(off, abs=0.02)      # the bend is gentle
    hairpin = line.piece([(0, 0), (10, 0), (0, 0.5)], 0, 30, 1.0)
    assert len(hairpin) == 4 and max(abs(x) + abs(y) for x, y in hairpin) < 12              # no runaway spike
    d = line.densify(corner, 4.0)
    assert d.tolist()[0] == [0.0, 0.0] and [10.0, 0.0] in d.tolist() and d.tolist()[-1] == [10.0, 10.0]
    assert len(d) == 7 and np.hypot(*np.diff(d, axis=0).T).max() <= 4.0


# -- the planner -----------------------------------------------------------------------------------
def ditch(tmp_path, along=True, extra=""):
    """A 120 ft square of flat ground at 100 ft with a ditch designed along LINE, and a program that cuts it with
    one grader. The work zone is the ground within 9 ft of the line. Returns the checked model."""
    xs = ys = np.arange(0, 121, 2.0)
    gx, gy = np.meshgrid(xs, ys)
    off = line.project(LINE, gx, gy)[1]
    ground = np.full(gx.shape, 100.0)
    write_grid_tin(str(tmp_path / "eg.xml"), "EG", xs, ys, ground, "ft")
    write_grid_tin(str(tmp_path / "fg.xml"), "FG", xs, ys,
                   np.minimum(ground, 99.0 + np.clip(np.abs(off) - 3, 0, None) / 4), "ft")
    e0, e1 = np.subtract(LINE[0], LINE[1]), np.subtract(LINE[-1], LINE[-2])
    longer = [tuple(LINE[0] + 9 * e0 / np.hypot(*e0)), *LINE[1:-1], tuple(LINE[-1] + 9 * e1 / np.hypot(*e1))]
    edge = line.piece(longer, 0, 1e9, 9.0) + line.piece(longer, 1e9, 0, -9.0)
    ring = ", ".join(f"({x:.2f}, {y:.2f})" for x, y in edge)
    pts = ", ".join(f"({x:g}, {y:g})" for x, y in LINE)
    (tmp_path / "ditch.loam").write_text(f"""loam 0.0.2
site s {{ crs "LOCAL:ft"; units ft; design "fg.xml" format landxml surface "FG"
         existing "eg.xml" format landxml surface "EG"; boundary rect (0, 0) (120, 120); grid 2 ft }}
probe mast {{ kind static; sensor lidar; rate 5 min; accuracy 0.02 ft; coverage site; stale_after 15 min }}
machine gr1 {{ class grader; caps [cut, finish]; blade {BLADE:g} ft; max_speed 3 mph }}
zone strip {{ role finish; area polygon [{ring}]; tolerance ±0.1 ft }}
stockpile sp {{ area rect (100, 0) (118, 18); available 0 yd3 }}
{extra}
task dig {{
  op cut; zone strip; by gr1; max_cut 0.4 ft per pass
  {f"along [{pts}]" if along else ""}
  spoil -> sp
  until in_tolerance(strip)
}}
""")
    return load(str(tmp_path / "ditch.loam"))


def run(tmp_path, model):
    """Run a model in the reference simulator. Returns the simulator, its report and the orders it sent."""
    sim = Sim(model, str(tmp_path), events=Events.none(), log=lambda x: None)
    rep = sim.run(wire_path=str(tmp_path / "wire.jsonl"))
    with open(tmp_path / "wire.jsonl", encoding="utf-8") as f:
        sent = [json.loads(ln) for ln in f]
    return sim, rep, [m["payload"] for m in sent if m["topic"].endswith("/order")]


def lane_cells(sim, order):
    """The cells an order plans to change, with each one's station and offset from LINE."""
    tile = wire.decode_heights(order["target"]["pattern"], tuple(order["target"]["shape"]))
    j, i = np.nonzero(tile)
    p = np.asarray(order["window"], float)
    i, j = i + round((p[:, 0].min() - sim.x0) / sim.cell), j + round((p[:, 1].min() - sim.y0) / sim.cell)
    s, off = line.project(LINE, sim.xs[i], sim.ys[j])
    return i, j, s, off


def test_passes_follow_the_line_and_come_back(tmp_path):
    """Every pass of a task with `along` is a lane: its path is the line moved a whole number of tool widths to
    one side, its order changes only ground within half a tool width of that path, and it starts from the end
    nearer to where the last one stopped, so the ditch is worked along and back. The ground decides the rest: the
    task ends when the survey says the ditch is cut, and nothing is cut below design."""
    sim, rep, orders = run(tmp_path, ditch(tmp_path))
    assert rep["finished"] and rep["true_ground"]["within_tolerance_pct"] >= 95
    assert abs(sim.ledger_error()) < 1e-6 and float((sim.design - sim.z).max()) < 0.1
    assert len(orders) > 12 and all(o["op"] == "cut" and o["swath"] == pytest.approx(BLADE * FT) for o in orders)
    lanes, backwards = set(), 0
    for prev, o in zip([None, *orders], orders, strict=False):
        p = np.asarray(o["path"])
        s, off = line.project(LINE, p[:, 0], p[:, 1])
        lane = round(float(np.median(off)) / BLADE)
        lanes.add(lane)
        assert off == pytest.approx(lane * BLADE, abs=0.05)              # on the line, or whole widths beside it
        assert abs(s[-1] - s[0]) <= 30.05                                # one push long at most
        backwards += s[-1] < s[0]
        _, _, _, cell_off = lane_cells(sim, o)
        assert cell_off.size and np.abs(cell_off - lane * BLADE).max() <= BLADE / 2 + 1e-6
        if prev is not None:
            end = prev["path"][-1]
            assert math.dist(end, o["path"][0]) <= math.dist(end, o["path"][-1]) + 1e-6
    assert lanes == {-1, 0, 1} and 0 < backwards < len(orders)          # both directions, all three lanes
    # The same ground cut twice running is cut in opposite directions.
    assert any(a["path"] == b["path"][::-1] for a, b in pairwise(orders))


def test_without_a_line_orders_are_strips_as_before(tmp_path):
    """A task with no `along` is planned in straight strips, and its orders carry no path."""
    _, rep, orders = run(tmp_path, ditch(tmp_path, along=False))
    assert rep["finished"] and orders and not any("path" in o or "swath" in o for o in orders)


def test_a_lane_keeps_clear_of_an_exclusion(tmp_path):
    """No lane is planned over an exclusion zone: the ditch is cut on both sides of a marked utility crossing and
    never inside it."""
    m = ditch(tmp_path, extra="zone gas { role exclusion; area rect (40, 28) (48, 48) }")
    sim, _, orders = run(tmp_path, m)
    stations = []
    for o in orders:
        i, j, s, _ = lane_cells(sim, o)
        assert not sim.excl[j, i].any()
        p = line.densify(o["path"], 0.5) if len(o["path"]) > 1 else np.asarray(o["path"], float)
        assert not ((p[:, 0] > 40) & (p[:, 0] < 48) & (p[:, 1] > 28) & (p[:, 1] < 48)).any()
        stations += s.tolist()
    crossing = line.project(LINE, np.array([44.0]), np.array([38.0]))[0][0]
    assert min(stations) < crossing - 10 and max(stations) > crossing + 10
    assert np.allclose(sim.z[sim.excl], 100.0)


def test_only_tasks_that_follow_a_line_may_have_one():
    """`along` on a task that cannot follow it is flagged, not silently dropped."""
    src = MINI.replace("spoil -> A;", "spoil -> A; along [(0, 0), (10, 10)];")
    assert not [d for d in check(parse(src))[1] if "along" in d.msg]
    roller = src.replace("op cut", "op compact").replace("class dozer }", "class dozer; caps [cut, compact] }")
    assert [d.level for d in check(parse(roller))[1] if "'along' is ignored" in d.msg] == ["warning"]
    assert "at least two points" in " ".join(d.msg for d in check(parse(src.replace(", (10, 10)", "")))[1])


# -- the machine's side ----------------------------------------------------------------------------
def test_a_machine_drives_the_path_it_is_given(tmp_path):
    """A machine follows an order's path in the order given, then hauls and comes back to the path's end. An
    order without one is driven down the middle of its window. Both forms are valid Loam Wire."""
    m = ditch(tmp_path)
    _, _, orders = run(tmp_path, m)
    bent = next(o for o in orders if len(o["path"]) == 3)
    wire_valid("order", bent)
    with pytest.raises(Exception, match="0 is less than or equal to the minimum"):
        wire_valid("order", {**bent, "swath": 0})
    ms = MachineSim(m, "gr1", "unused", (0.0, 0.0))
    ms.take(bent)
    path = [tuple(p) for p in bent["path"]]
    assert ms.path == [(0.0, 0.0), *path, (109.0, 9.0), path[-1]]        # start, the lane, the stockpile, back
    ms.pose_at(sum(ms.legs[:2]) / sum(ms.legs))
    assert (ms.x, ms.y) == pytest.approx(path[1])                        # at the bend when that far along
    plain = {k: v for k, v in bent.items() if k not in ("path", "swath")}
    wire_valid("order", plain)
    ms.take({**plain, "orderId": "other"})
    (x0, y0), (x1, y1) = np.min(plain["window"], 0), np.max(plain["window"], 0)
    assert ms.path[1][0] in (x0, (x0 + x1) / 2) and ms.path[1][1] in (y0, (y0 + y1) / 2)


def test_the_world_cuts_only_the_lane(tmp_path):
    """The simulated site moves earth from the order alone, and for a lane that is the lane's ground: the cells
    of the order's window outside the lane are left as they were."""
    m = ditch(tmp_path)
    sim, _, orders = run(tmp_path, m)
    w = World(m, str(tmp_path))
    o = next(o for o in orders if len(o["path"]) == 3)                   # a lane round the bend
    before = w.z.copy()
    w.apply(o)
    i, j, _, _ = lane_cells(sim, o)
    changed = np.zeros(w.z.shape, bool)
    changed[j, i] = True
    assert (w.z[changed] < before[changed]).mean() > 0.9 and np.array_equal(w.z[~changed], before[~changed])
    planned = wire.decode_heights(o["target"]["pattern"], tuple(o["target"]["shape"]))
    assert float((before - w.z).sum()) == pytest.approx(-float(planned.sum()) / FT, rel=0.1)


# -- the swale designer ----------------------------------------------------------------------------
def test_the_bottom_is_the_highest_line_that_drains():
    """Along any ground, the bottom is at least the asked depth under it, falls at least the asked grade the whole
    way, and could not be higher: every point of it either touches the depth limit or lies on the least grade
    running down from one that does."""
    rng = np.random.default_rng(3)
    st = np.concatenate(([0.0], np.cumsum(rng.uniform(0.5, 2.0, 300))))
    g = 50 - 0.004 * st + 0.6 * np.sin(st / 17) + rng.normal(0, 0.05, st.size)
    zb = invert(g, st, 1.0, 0.005)
    assert (zb <= g - 1.0 + 1e-12).all() and zb[0] == pytest.approx(g[0] - 1.0)
    fall = -np.diff(zb) / np.diff(st)
    assert fall.min() >= 0.005 - 1e-12
    touching = np.isclose(zb, g - 1.0)
    assert 3 < touching.sum() < st.size and (np.isclose(fall, 0.005) | touching[1:]).all()
    # Ground that already falls faster than the grade is simply followed, a constant depth down.
    assert invert(40 - 0.02 * st, st, 1.5, 0.005) == pytest.approx(40 - 0.02 * st - 1.5)


def _field(path, cross=0.002):
    """Ground falling gently to the east with low humps, and `cross` of rise to the north, on a 2 ft lattice, as a
    LandXML surface. Returns the lattice and the height function."""
    xs, ys = np.arange(0, 301, 2.0), np.arange(0, 201, 2.0)
    h = lambda x, y: 100 - 0.004 * x + cross * y + 0.3 * np.sin(x / 40) * np.cos(y / 33)  # noqa: E731
    write_grid_tin(str(path), "EG", xs, ys, h(*np.meshgrid(xs, ys)), "ft")
    return xs, ys, h


CENTRE = [(40.0, 60.0), (150.0, 100.0), (260.0, 90.0)]


def test_swale_designs_a_channel_and_its_program_finishes(tmp_path):
    """The design is the ground with a channel cut into it: a flat bottom on the drained line, sides at the asked
    slope, rounded ends, and nothing filled. The program cuts and finishes along the centreline and the simulated
    job ends in tolerance."""
    _, _, h = _field(tmp_path / "eg.xml")
    r = swale(str(tmp_path / "eg.xml"), str(tmp_path / "sw.loam"), along=CENTRE, bottom="4 ft", depth="1 ft")
    assert r["flows"] == "forward" and r["length"] == pytest.approx(line.length(CENTRE), abs=0.01)
    assert r["grade_pct"][0] >= 0.5 - 1e-6 and r["outlet_depth_ft"] >= 1.0 and r["fill_yd3"] == 0 and r["cut_yd3"] > 50
    pts, _, _ = read_tin(r["design_file"], "swale")
    s, off = line.project(CENTRE, pts[:, 0], pts[:, 1])
    assert np.abs(off).max() <= 10.0 + 1e-9                               # the strip: the bottom and twice the sides
    ground = h(pts[:, 0], pts[:, 1])
    assert (pts[:, 2] <= ground + 0.005).all()                           # the file's ground, between its points
    dense = line.densify(CENTRE, 1.0)
    st = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(dense, axis=0).T))))
    zb = np.interp(s, st, invert(h(dense[:, 0], dense[:, 1]), st, 1.0, 0.005))
    want = np.minimum(ground, zb + np.clip(np.abs(off) - 2.0, 0, None) / 4.0)
    assert pts[:, 2] == pytest.approx(want, abs=0.01)                    # the lattice ground is not the exact one
    flat = np.abs(off) <= 2.0
    assert flat.sum() > 300 and float((ground - pts[:, 2])[flat].min()) >= 1.0 - 0.01
    assert r["deepest_cut_ft"] == pytest.approx(float((ground - pts[:, 2]).max()), abs=0.01)
    text = (tmp_path / "sw.loam").read_text()
    assert text.count("(40, 60), (150, 100), (260, 90)]") == 2 and "blade that tilts" in text
    assert max(len(ln) for ln in text.splitlines()) <= 120
    m = load(r["out"])
    assert m.tasks["cut_to_fill"]["along"] == CENTRE == m.tasks["finish_grade"]["along"]
    _, rep, orders = run(tmp_path, m)
    assert rep["finished"] and rep["true_ground"]["within_tolerance_pct"] >= 95
    assert {o["op"] for o in orders} == {"cut", "finish"} and all(len(o["path"]) >= 2 for o in orders)


def test_the_controller_cuts_a_swale_on_the_simulated_site(tmp_path):
    """The live controller and the simulated site, exchanging only orders, machine states and surveys, finish the
    swale within tolerance: an order's path and pattern carry everything the site needs."""
    _field(tmp_path / "eg.xml")
    r = swale(str(tmp_path / "eg.xml"), str(tmp_path / "sw.loam"), along=CENTRE)
    _, w, _, done = loopback(r["out"], str(tmp_path))
    t = w.truth()
    assert done and t["zones"]["limits"]["within_tolerance_pct"] >= 95 and abs(t["ledger_error"]) < 1e-6
    assert float((w.design - w.z).max()) < 0.1                           # no gouge below the design


def test_water_runs_the_cheaper_way_unless_told(tmp_path):
    """The ground falls east. A line drawn downhill drains forward, the same line drawn uphill drains in reverse
    and gives the same swale, and forcing the water uphill is possible and costs a much deeper cut."""
    _field(tmp_path / "eg.xml")
    g, out = str(tmp_path / "eg.xml"), str(tmp_path / "a.loam")
    down = swale(g, out, along=CENTRE)
    up = swale(g, out, along=CENTRE[::-1])
    assert (down["flows"], up["flows"]) == ("forward", "reverse")
    assert up["deepest_cut_ft"] == pytest.approx(down["deepest_cut_ft"], abs=0.02)
    assert up["grade_pct"] == pytest.approx(down["grade_pct"], abs=0.02) and up["cut_yd3"] == down["cut_yd3"]
    forced = swale(g, out, along=CENTRE[::-1], flow="forward")
    assert forced["flows"] == "forward" and forced["deepest_cut_ft"] > down["deepest_cut_ft"] + 1.0
    assert "Water runs from (260.0, 90.0) to (40.0, 60.0)" in (tmp_path / "a.loam").read_text()


def test_the_strip_widens_until_the_sides_meet_the_ground(tmp_path):
    """On a hillside the uphill side runs far before it meets the ground, so the strip is widened to hold it. Where
    the hill is as steep as the sides they never meet, and that is said. Upright sides need no room at all."""
    east = [(120.0, 100.0), (180.0, 100.0)]
    _field(tmp_path / "hill.xml", cross=0.15)
    r = swale(str(tmp_path / "hill.xml"), str(tmp_path / "h.loam"), along=east)
    # Uphill, the side gains 0.25 - 0.15 a foot on the ground: 10 ft of run for each foot of depth, past the
    # first strip (10 ft) and the second (20 ft).
    assert 10 < r["cut_reaches_ft"] < 19 and r["cut_reaches_ft"] == pytest.approx(2 + 10 * r["deepest_cut_ft"], rel=0.2)
    pts, _, _ = read_tin(r["design_file"], "swale")
    assert np.abs(line.project(east, pts[:, 0], pts[:, 1])[1]).max() == pytest.approx(20.0, abs=0.5)
    _field(tmp_path / "steep.xml", cross=0.26)
    with pytest.raises(ValueError, match="climbs away"):
        swale(str(tmp_path / "steep.xml"), str(tmp_path / "s.loam"), along=east)
    upright = swale(str(tmp_path / "steep.xml"), str(tmp_path / "s.loam"), along=east, side=0.0)
    assert upright["cut_reaches_ft"] <= 2.0 + 0.5


def test_swale_refuses_what_it_cannot_design(tmp_path, capsys):
    """One centreline, a real depth, readable lengths, ground under the line and a grid fine enough to show the
    channel: each is checked and said plainly."""
    _field(tmp_path / "eg.xml")
    g, out = str(tmp_path / "eg.xml"), str(tmp_path / "x.loam")
    for kw, why in (({}, "one of the two"), ({"along": CENTRE, "line_file": "a.kml"}, "one of the two"),
                    ({"along": CENTRE, "depth": "0 ft"}, "above zero"), ({"along": CENTRE, "bottom": "wide"}, "'4 ft'"),
                    ({"along": CENTRE, "bottom": "-1 ft"}, "negative"), ({"along": CENTRE, "side": -1.0}, "negative"),
                    ({"along": CENTRE, "flow": "up"}, "forward or reverse"),
                    ({"along": [(40.0, 60.0), (40.0, 60.0)]}, "two different points"),
                    ({"along": [(250.0, 100.0), (400.0, 100.0)]}, "uncovered"),
                    ({"along": CENTRE, "max_cells": 2000}, "too coarse.*raise --max-cells to 3,600"),
                    ({"along": CENTRE, "max_cells": 5}, "raise --max-cells")):
        with pytest.raises(ValueError, match=why):
            swale(g, out, **kw)
    with pytest.raises(SystemExit):
        main(["swale", g, "-o", out, "--along", "40,60 150,100", "--depth", "deep"])
    assert "loam swale:" in capsys.readouterr().err
    main(["swale", g, "-o", out, "--along", "40,60 150,100 260,90", "--side-slope", "3", "--grade", "1", "--json"])
    r = json.loads(capsys.readouterr().out)
    assert r["check"] == "ok" and r["grade_pct"][0] >= 1.0 - 1e-6 and "sides 3:1" in (tmp_path / "x.loam").read_text()


def test_a_centreline_from_a_file(tmp_path):
    """The line may be a path drawn in Google Earth, a GeoJSON LineString or a shapefile polyline."""
    from loam.surfaces import line_from_file
    ll = [(-85.70, 34.70), (-85.699, 34.7005), (-85.698, 34.7004)]
    (tmp_path / "p.kml").write_text(
        '<?xml version="1.0"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark><LineString>'
        "<coordinates>" + " ".join(f"{a},{b},0" for a, b in ll) + "</coordinates></LineString></Placemark>"
        "</Document></kml>")
    assert line_from_file(str(tmp_path / "p.kml")) == (ll, "EPSG:4326")
    feature = {"type": "Feature", "geometry": {"type": "MultiLineString", "coordinates": [CENTRE, [[0, 0], [1, 1]]]}}
    (tmp_path / "p.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": [feature]}))
    assert line_from_file(str(tmp_path / "p.geojson")) == (CENTRE, None)
    (tmp_path / "poly.geojson").write_text(json.dumps({"type": "Polygon", "coordinates": [[[0, 0], [1, 0], [1, 1]]]}))
    with pytest.raises(ValueError, match="no LineString"):
        line_from_file(str(tmp_path / "poly.geojson"))
    (tmp_path / "one.geojson").write_text(json.dumps({"type": "LineString", "coordinates": [[0, 0]]}))
    with pytest.raises(ValueError, match="fewer than 2"):
        line_from_file(str(tmp_path / "one.geojson"))
    # The designer takes the file in place of points, and the result is the same swale.
    _field(tmp_path / "eg.xml")
    a = swale(str(tmp_path / "eg.xml"), str(tmp_path / "a.loam"), along=CENTRE)
    b = swale(str(tmp_path / "eg.xml"), str(tmp_path / "b.loam"), line_file=str(tmp_path / "p.geojson"))
    assert (a["cut_yd3"], a["deepest_cut_ft"]) == (b["cut_yd3"], b["deepest_cut_ft"])
    shapefile = pytest.importorskip("shapefile")
    with open(tmp_path / "l.shp", "wb") as shp, open(tmp_path / "l.shx", "wb") as shx, \
            open(tmp_path / "l.dbf", "wb") as dbf:                        # pyshp's own writer leaves one open
        w = shapefile.Writer(shp=shp, shx=shx, dbf=dbf)
        w.field("name", "C")
        w.line([CENTRE])
        w.record("swale")
        w.close()
    assert line_from_file(str(tmp_path / "l.shp")) == (CENTRE, None)


def test_swale_on_an_elevation_model_in_degrees(tmp_path):
    """The free-data path: an elevation model in longitude and latitude and a path drawn in Google Earth. The
    swale is designed in the metric UTM zone the line lies in and the program carries that zone."""
    pyproj = pytest.importorskip("pyproj")
    _dem(str(tmp_path / "dem.tif"), "EPSG:4326", -85.70, 34.70, n=300, res=1e-5)       # about 0.9 by 1.1 m cells
    ll = [(-85.6992, 34.7008), (-85.6984, 34.7014), (-85.6976, 34.7016)]
    (tmp_path / "p.kml").write_text('<kml xmlns="http://www.opengis.net/kml/2.2"><LineString><coordinates>'
                                    + " ".join(f"{a},{b}" for a, b in ll) + "</coordinates></LineString></kml>")
    r = swale(str(tmp_path / "dem.tif"), str(tmp_path / "s.loam"), line_file=str(tmp_path / "p.kml"))
    to_utm = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32616", always_xy=True)
    want = [to_utm.transform(a, b) for a, b in ll]
    m = load(r["out"])
    assert r["units"] == "m" and m.site["crs"] == "EPSG:32616" and r["ground_file"].endswith("s-ground.xml")
    assert np.asarray(m.tasks["cut_to_fill"]["along"]) == pytest.approx(np.asarray(want), abs=0.006)
    assert r["length"] == pytest.approx(line.length(want), abs=0.01) and 0.3 <= r["deepest_cut_m"] < 2.0
    text = (tmp_path / "s.loam").read_text()
    assert "bottom 1.2 m wide" in text and "grid     0.5 m" in text and "rise 0.12 m from one cell" in text


# -- guidance lines for AgOpenGPS ------------------------------------------------------------------
def _tracks(path):
    """TrackLines.txt read the way AgOpenGPS reads it (TrackFiles.Load): a list of (name, heading, A, B, mode,
    points), each point (easting, northing, heading)."""
    with open(path, encoding="utf-8") as f:
        tl = f.read().splitlines()
    assert tl[0].startswith("$")
    out, i = [], 1
    while i < len(tl):
        a, b = (tuple(float(v) for v in tl[i + k].split(",")) for k in (2, 3))
        assert float(tl[i + 4]) == 0 and tl[i + 6] == "True"
        n = int(tl[i + 7])
        out.append((tl[i], float(tl[i + 1]), a, b, int(tl[i + 5]),
                    np.array([[float(v) for v in ln.split(",")] for ln in tl[i + 8:i + 8 + n]])))
        i += 8 + n
    return out


def test_a_line_becomes_an_agopengps_curve(tmp_path):
    """A task's line is written as one AgOpenGPS curve track, laid out as AgOpenGPS lays out a curve drawn on its
    own map: points no more than 1.6 m apart with a straight tail at each end, each heading taken from the
    point's two neighbours, the track heading the mean of the curve's, and A and B at its ends."""
    _field(tmp_path / "eg.xml")
    r = swale(str(tmp_path / "eg.xml"), str(tmp_path / "sw.loam"), along=CENTRE)
    f = Frame(FT, 20.0, 34.70, -85.70)
    out = agopengps(str(tmp_path / "aog"), SiteGrid(load(r["out"]), str(tmp_path)), f)
    assert out["curves"] == ["cut_to_fill"]                              # the finishing task shares the line
    tracks = _tracks(tmp_path / "aog" / "TrackLines.txt")
    assert [(t[0], t[4]) for t in tracks] == [("loam limits", 2), ("loam cut_to_fill", 4)]
    _, heading, a, b, _, pts = tracks[1]
    e, n = f.local_to_en(*np.array(CENTRE).T)
    assert a == pytest.approx((e[0], n[0]), abs=1e-3) and b == pytest.approx((e[-1], n[-1]), abs=1e-3)
    gap = np.hypot(*np.diff(pts[:, :2], axis=0).T)
    assert gap.max() <= 1.6 and gap[:CURVE_TAIL] == pytest.approx(1.0, abs=2e-3)
    assert gap[-CURVE_TAIL:] == pytest.approx(1.0, abs=2e-3)
    body = pts[CURVE_TAIL:-CURVE_TAIL]
    assert tuple(body[0, :2]) == pytest.approx(a, abs=1e-3) and tuple(body[-1, :2]) == pytest.approx(b, abs=1e-3)
    for corner in zip(e[1:-1], n[1:-1], strict=True):                    # the line's own corners are kept
        assert np.hypot(*(body[:, :2] - corner).T).min() < 2e-3
    # CABCurve.CalculateHeadings: the first toward the second, the last from the one before, the rest from the
    # neighbour behind to the neighbour ahead; clockwise from north, 0 to 2 pi.
    step = np.concatenate([pts[1:2, :2] - pts[:1, :2], pts[2:, :2] - pts[:-2, :2], pts[-1:, :2] - pts[-2:-1, :2]])
    turn = (pts[:, 2] - np.arctan2(step[:, 0], step[:, 1]) + np.pi) % (2 * np.pi) - np.pi
    assert np.abs(turn).max() < 2e-3 and (pts[:, 2] >= 0).all() and (pts[:, 2] < 2 * np.pi).all()
    inner = np.arctan2(*np.gradient(body[:, :2], axis=0).T)
    assert heading == pytest.approx(math.atan2(np.sin(inner).mean(), np.cos(inner).mean()) % (2 * math.pi), abs=1e-3)
    # The tails run straight on from the ends.
    assert np.ptp(np.unwrap(pts[:CURVE_TAIL, 2])) < 1e-3 and np.ptp(np.unwrap(pts[-CURVE_TAIL:, 2])) < 1e-3
