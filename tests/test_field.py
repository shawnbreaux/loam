"""Field adapters: georeferencing, point clouds, surveys, NMEA, targets and tool exports.

Readers for the AgOpenGPS and OpenGrade3D files below follow those tools' own
parsers line for line (AgOpenGPS TrackFiles.Load / FieldPlaneFiles.LoadOrigin /
BoundaryStreamer.Read; OpenGrade3D FileOpenAgdDesign), so a file that passes
here is one the tools will accept.
"""
import json
import math
import os
import struct
import tracemalloc

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES

from loam import wire
from loam.checker import check, has_errors
from loam.cli import main
from loam.export import agd, agopengps, landxml
from loam.geo import Frame, m_per_deg_lat, m_per_deg_lon
from loam.grid import SiteGrid
from loam.landxml import rasterize, read_tin, write_grid_tin
from loam.parser import parse
from loam.pointcloud import Binner, fill_gaps, grid_cloud, read_las, read_pcd, read_xyz, write_las, write_pcd
from loam.survey import idw, nmea_states, parse_gga, read_points_csv, score
from loam.target import next_target

# A site origin (latitude, longitude) and matching projected coordinates (easting, northing) for the tests.
LAT0, LON0 = 34.4459, -85.7197          # DeKalb County, AL
E0, N0 = 600000.0, 3812000.0

# A small metric site: a cut zone (W), a fill zone (E) and an exclusion zone (X) inside E.
SRC = """loam 0.0.1
site t {{ crs "LOCAL:m"; units m; design "d.xml" format landxml surface "FG"
          existing "e.xml" format landxml surface "EG"; boundary rect (0, 0) (40, 20); grid 1 m }}
probe p {{ kind static; sensor lidar; rate 5 min; accuracy 2 cm }}
machine dz {{ class dozer; blade 3 m }}
zone W {{ role cut;  area rect (0, 0) (20, 20);  tolerance ±3 cm }}
zone E {{ role fill; area rect (20, 0) (40, 20); tolerance ±3 cm; lift 20 cm }}
zone X {{ role exclusion; area rect (30, 8) (34, 12); buffer {buf} }}
task cutW  {{ op cut;  zone W; by class dozer; max_cut 10 cm per pass; spoil -> E; until residual(W).high < 3 cm }}
task fillE {{ op fill; zone E; by class dozer; until residual(E).low > -3 cm }}
"""


def model_for(tmp_path, design_z, existing_z, buf="1 m"):
    """Write design and existing surfaces (functions of x and y, metres) as LandXML files and return the checked model
    and its grid. `buf` is the exclusion zone's buffer.
    """
    xs, ys = np.arange(0.0, 41.0), np.arange(0.0, 21.0)
    gx, gy = np.meshgrid(xs, ys)
    write_grid_tin(str(tmp_path / "d.xml"), "FG", xs, ys, design_z(gx, gy), "m")
    write_grid_tin(str(tmp_path / "e.xml"), "EG", xs, ys, existing_z(gx, gy), "m")
    m, diags = check(parse(SRC.format(buf=buf)))
    assert not has_errors(diags), [d.msg for d in diags]
    return m, SiteGrid(m, str(tmp_path))


@pytest.fixture
def site(tmp_path):
    """A 40 x 20 m site with a flat design at 100 m: the west half 0.25 m too high, the east half 0.5 m too low.
    Returns (model, grid, folder).
    """
    # design flat at 100 m; west half 0.25 m high, east half 0.5 m low
    m, g = model_for(tmp_path, lambda x, y: np.full_like(x, 100.0),
                     lambda x, y: np.where(x < 20, 100.25, 99.5))
    return m, g, tmp_path


# -- georeferencing -------------------------------------------------------------------------
@pytest.mark.parametrize("rot", [0.0, 30.0, -112.5])
def test_frame_round_trips(rot):
    """Site coordinates convert to latitude/longitude and to projected coordinates and back to the same values, at
    several site rotations.
    """
    f = Frame(0.3048, rot, LAT0, LON0, E0, N0, 1200 / 3937, dz=1.5)
    x, y = np.array([0.0, 100.0, -40.0, 1234.5]), np.array([0.0, 50.0, 300.0, -77.0])
    lat, lon = f.local_to_wgs84(x, y)
    np.testing.assert_allclose(f.wgs84_to_local(lat, lon), (x, y), atol=1e-6)
    e, n = f.local_to_projected(x, y)
    np.testing.assert_allclose(f.projected_to_local(e, n), (x, y), atol=1e-9)
    np.testing.assert_allclose(f.height_to_local(f.height_to_world([10.0, 20.0])), [10.0, 20.0])


def test_frame_rotation_convention():
    """The rotation is the compass bearing of the site's +y axis: at 90 degrees, +y points east."""
    f = Frame(1.0, 90.0)                        # site +y points east
    np.testing.assert_allclose(f.local_to_en(0, 1), (1, 0), atol=1e-12)
    np.testing.assert_allclose(f.local_to_en(1, 0), (0, -1), atol=1e-12)
    assert f.heading_rad(0, 1) == pytest.approx(math.pi / 2)


def test_frame_matches_agopengps_local_plane():
    """AgOpenGPS: northing = dlat * m_per_deg_lat(origin); easting = dlon * m_per_deg_lon(point lat)."""
    f = Frame(1.0, 0.0, LAT0, LON0)
    lat, lon = f.local_to_wgs84(123.4, -56.7)
    north = (lat - LAT0) * m_per_deg_lat(LAT0)
    east = (lon - LON0) * m_per_deg_lon(lat)
    assert (float(east), float(north)) == pytest.approx((123.4, -56.7), abs=1e-6)
    with pytest.raises(ValueError, match="WGS84"):
        Frame(1.0).local_to_wgs84(0, 0)


# -- point clouds ------------------------------------------------------------------------------
def test_las_round_trip_chunks_and_classes(tmp_path):
    """Points written as LAS read back within the file's 0.5 mm resolution, in chunks of any size, and can be filtered
    by class (2 = ground).
    """
    rng = np.random.default_rng(3)
    pts = np.column_stack([E0 + rng.uniform(0, 50, 1000), N0 + rng.uniform(0, 50, 1000), rng.uniform(90, 110, 1000)])
    write_las(str(tmp_path / "a.las"), pts)
    got = np.concatenate(list(read_las(str(tmp_path / "a.las"), chunk=97)))
    np.testing.assert_allclose(got, pts, atol=6e-4)
    assert sum(len(c) for c in read_las(str(tmp_path / "a.las"), {2})) == 1000
    assert sum(len(c) for c in read_las(str(tmp_path / "a.las"), {6})) == 0


def test_las_14_point_count_and_errors(tmp_path):
    """A LAS 1.4 file with only the 64-bit point count is read correctly. Compressed, non-LAS and cut-off files are
    refused with a reason.
    """
    p = tmp_path / "a.las"
    write_las(str(p), np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
    raw = bytearray(p.read_bytes())
    head, body = raw[:227], raw[227:]
    head = head + bytearray(375 - 227)                  # LAS 1.4 header is 375 bytes
    head[25] = 4
    struct.pack_into("<H", head, 94, 375)
    struct.pack_into("<I", head, 96, 375)
    struct.pack_into("<I", head, 107, 0)                # legacy count zeroed, as 1.4 writers may do
    struct.pack_into("<Q", head, 247, 2)
    (tmp_path / "b.las").write_bytes(bytes(head + body))
    np.testing.assert_allclose(np.concatenate(list(read_las(str(tmp_path / "b.las")))), [[1, 2, 3], [4, 5, 6]])
    head[104] |= 0x80                                   # LAZ flag
    (tmp_path / "c.las").write_bytes(bytes(head + body))
    with pytest.raises(ValueError, match="LAZ"):
        list(read_las(str(tmp_path / "c.las")))
    (tmp_path / "d.las").write_bytes(b"nope" + bytes(400))
    with pytest.raises(ValueError, match="LASF"):
        list(read_las(str(tmp_path / "d.las")))
    (tmp_path / "e.las").write_bytes(bytes(raw[:-10]))
    with pytest.raises(ValueError, match="ends before"):
        list(read_las(str(tmp_path / "e.las"), chunk=1))


def test_pcd_binary_ascii_and_xyz(tmp_path):
    """PCD files (binary, ASCII, with extra fields and padding) and plain x y z text files are read; compressed PCD is
    refused.
    """
    pts = np.array([[1.5, 2.5, 3.5], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    write_pcd(str(tmp_path / "b.pcd"), pts)
    np.testing.assert_allclose(np.concatenate(list(read_pcd(str(tmp_path / "b.pcd"), chunk=2))), pts)
    (tmp_path / "a.pcd").write_text(
        "# livox\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n" + "".join(f"{a} {b} {c} 9\n" for a, b, c in pts))
    np.testing.assert_allclose(np.concatenate(list(read_pcd(str(tmp_path / "a.pcd")))), pts)
    # binary with an intensity field and PCL "_" padding between other fields
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("_", "u1", (4,)), ("z", "<f4"), ("_2", "u1", (4,))])
    rec = np.zeros(3, dt)
    rec["x"], rec["y"], rec["z"] = pts.T
    (tmp_path / "p.pcd").write_bytes(
        b"FIELDS x y _ z _\nSIZE 4 4 1 4 1\nTYPE F F U F U\nCOUNT 1 1 4 1 4\nPOINTS 3\nDATA binary\n" + rec.tobytes())
    np.testing.assert_allclose(np.concatenate(list(read_pcd(str(tmp_path / "p.pcd")))), pts)
    (tmp_path / "c.pcd").write_text("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1\nDATA binary_compressed\n")
    with pytest.raises(ValueError, match="binary_compressed"):
        list(read_pcd(str(tmp_path / "c.pcd")))
    (tmp_path / "p.xyz").write_text("x,y,z\n1,2,3\n# note\n4 5 6\n")
    np.testing.assert_allclose(np.concatenate(list(read_xyz(str(tmp_path / "p.xyz")))), [[1, 2, 3], [4, 5, 6]])


def test_binner_and_fill(site):
    """Points are averaged per grid cell, points outside the site are dropped, cells with too few points are left
    empty, and small holes are filled from their neighbours.
    """
    _, g, _ = site
    b = Binner(g)
    b.add(np.array([0.5, 0.6, 1.5, -5.0]), np.array([0.5, 0.4, 0.5, 0.5]), np.array([1.0, 3.0, 7.0, 99.0]))
    z, n = b.result("mean")
    assert z[0, 0] == 2.0 and z[0, 1] == 7.0 and n.sum() == 3 and np.isnan(z[5, 5])
    assert b.result("min")[0][0, 0] == 1.0
    assert np.isnan(b.result("mean", min_points=2)[0][0, 1])
    hole = np.ones((5, 5))
    hole[2, 2] = np.nan
    assert fill_gaps(hole)[2, 2] == 1.0
    assert np.isnan(fill_gaps(np.full((3, 3), np.nan))).all()


def test_grid_cloud_recovers_surface(site):
    """A surface written as a point cloud in projected coordinates, on a rotated site, comes back onto the grid within
    1 mm.
    """
    m, g, tmp = site
    ex = g.surface(m.site["existing"])
    f = Frame(1.0, 25.0, None, None, E0, N0)
    gx, gy = np.meshgrid(g.xs, g.ys)
    e, n = f.local_to_projected(gx.ravel(), gy.ravel())
    write_las(str(tmp / "s.las"), np.column_stack([e, n, ex.ravel()]))
    z, info = grid_cloud(str(tmp / "s.las"), g, f, chunk=101)
    np.testing.assert_allclose(z, ex, atol=1e-3)
    assert info["cells_covered_pct"] == 100.0 and info["points_read"] == ex.size


# -- surveys and NMEA ---------------------------------------------------------------------------
def _gga(lat, lon, q=4, alt=100.0):
    """An NMEA GGA sentence for a position, with a valid checksum. `q` is the fix quality (4 = RTK fixed, 5 = RTK
    float).
    """
    def dm(v, w):
        """Degrees as NMEA writes them: whole degrees then decimal minutes."""
        d = int(abs(v))
        return f"{d:0{w}d}{(abs(v) - d) * 60:08.5f}"
    body = (f"GNGGA,120000.00,{dm(lat, 2)},{'N' if lat >= 0 else 'S'},{dm(lon, 3)},{'E' if lon >= 0 else 'W'},"
            f"{q},18,0.7,{alt:.3f},M,-30.0,M,1.0,0000")
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def test_parse_gga_and_checksum():
    """A GGA sentence is read to within 1e-7 degrees; a wrong checksum, another sentence type or garbage gives None."""
    g = parse_gga(_gga(LAT0, LON0))
    assert g["fix"] == "rtk-fixed" and g["lat"] == pytest.approx(LAT0, abs=1e-7)
    assert g["lon"] == pytest.approx(LON0, abs=1e-7) and g["alt"] == 100.0 and g["age"] == 1.0
    bad = _gga(LAT0, LON0)[:-2] + "00"
    assert parse_gga(bad) is None
    assert parse_gga("$GNRMC,1*00") is None and parse_gga("garbage") is None


def test_nmea_states_heading_and_quality():
    """A stream of positions becomes machine states: heading from the direction of travel, and RTK float reported as
    degraded.
    """
    f = Frame(1.0, 0.0, LAT0, LON0)
    pts = [(0, 0), (0, 0.01), (0, 1), (1, 2)]
    lines = [_gga(*(float(v) for v in f.local_to_wgs84(x, y))) for x, y in pts] + ["$junk*00"]
    lines.append(_gga(*(float(v) for v in f.local_to_wgs84(1, 2)), q=5))
    out = list(nmea_states(lines, f, "t", "dz"))
    assert len(out) == 5
    assert out[2]["pose"]["heading"] == pytest.approx(math.pi / 2, abs=1e-3)       # moved +y
    assert out[3]["pose"]["heading"] == pytest.approx(math.pi / 4, abs=0.02)       # GGA rounds to ~2 cm
    assert out[0]["safetyState"] == "ok" and out[4]["safetyState"] == "degraded"
    assert out[4]["errors"][0] == wire.error("gnssQuality", "rtk-float", "warning")
    assert out[0]["pose"]["z"] == pytest.approx(100.0)


def test_read_points_csv_layouts(tmp_path):
    """Survey point files are read in the layouts receivers export: Emlid's named columns, and unnamed PNEZD or PENZD.
    Float solutions are dropped unless asked for.
    """
    f = Frame(0.3048, 0.0, LAT0, LON0, E0, N0, 1.0)
    lat, lon = f.local_to_wgs84(100.0, 50.0)
    e, n = f.local_to_projected(100.0, 50.0)
    (tmp_path / "emlid.csv").write_text(
        "Name,Code,Easting,Northing,Elevation,Longitude,Latitude,Solution status\n"
        f"p1,,{e},{n},30.48,{float(lon)!r},{float(lat)!r},FIX\n"
        f"p2,,{e},{n},99,{float(lon)!r},{float(lat)!r},FLOAT\n")
    s = read_points_csv(str(tmp_path / "emlid.csv"), f)
    assert s.name == ["p1"] and s.x[0] == pytest.approx(100, abs=1e-5) and s.z[0] == pytest.approx(100.0)
    s = read_points_csv(str(tmp_path / "emlid.csv"), Frame(0.3048, 0.0, None, None, E0, N0), fix_only=False)
    assert len(s.z) == 2 and s.y[0] == pytest.approx(50, abs=1e-6)
    (tmp_path / "p.csv").write_text(f"1,{n},{e},30.48,shot\n")
    assert read_points_csv(str(tmp_path / "p.csv"), f).x[0] == pytest.approx(100, abs=1e-6)
    (tmp_path / "q.csv").write_text(f"1,{e},{n},30.48,shot\n")
    assert read_points_csv(str(tmp_path / "q.csv"), f, "PENZD").y[0] == pytest.approx(50, abs=1e-6)
    (tmp_path / "r.csv").write_text("Name,Foo,Elevation\na,1,2\n")
    with pytest.raises(ValueError, match="Easting"):
        read_points_csv(str(tmp_path / "r.csv"), f)


def test_idw_and_score(site):
    """Sparse survey points are spread onto the grid by inverse-distance weighting, and a zone is scored against its
    tolerance with a 95 % confidence interval.
    """
    m, g, _ = site
    design = g.surface(m.site["design"])
    z = idw(g, np.array([10.5, 30.5]), np.array([10.5, 10.5]), np.array([1.0, 3.0]), radius=50, max_elems=50)
    assert z[10, 10] == pytest.approx(1.0) and z[10, 30] == pytest.approx(3.0) and 1 < z[10, 20] < 3
    assert np.isnan(idw(g, np.array([10.5]), np.array([10.5]), np.array([1.0]), radius=2)[0, 39])
    from loam.survey import Survey
    xs, ys = np.meshgrid(np.arange(1.0, 39), np.arange(1.0, 19))
    good = Survey([], xs.ravel(), ys.ravel(), np.full(xs.size, 100.01))
    poly = m.zones["W"]["area"]["polygon"]
    r = score(g, design, good, poly, 0.03)
    assert r["within_tolerance_pct"] == 100.0 and r["ci95_pct"] == [100.0, 100.0]
    half = Survey([], good.x, good.y, np.where(np.arange(good.x.size) % 2 == 0, 100.0, 100.1))
    r = score(g, design, half, poly, 0.03)
    assert 40 < r["within_tolerance_pct"] < 60 and r["ci95_pct"][0] < 50 < r["ci95_pct"][1]
    with pytest.raises(ValueError, match="inside"):
        score(g, design, Survey([], np.array([500.0]), np.array([500.0]), np.array([1.0])), poly, 0.03)


# -- next-pass target --------------------------------------------------------------------------
def test_next_target_limits_and_exclusion(site):
    """The next-pass target cuts at most the allowed depth, fills at most one lift, leaves exclusion zones and their
    buffer blank, and reaches design after a few passes.
    """
    m, g, _ = site
    design, ground = g.surface(m.site["design"]), g.surface(m.site["existing"])
    t, info = next_target(g, design, ground)
    assert t[10, 5] == pytest.approx(100.15)          # 0.25 high, cut capped at 10 cm
    assert t[3, 25] == pytest.approx(99.7)            # 0.5 low, built up one 20 cm lift
    assert np.isnan(t[10, 31])                        # exclusion zone
    assert np.isnan(t[10, 29]) and np.isfinite(t[10, 28])   # 1 m buffer reaches one cell further
    assert info["max_cut"] == pytest.approx(0.1) and info["max_fill"] == pytest.approx(0.2)
    assert not info["done"] and info["in_tolerance_pct"] == 0.0
    near = np.where(np.isfinite(ground), design + 0.02, np.nan)
    t2, info2 = next_target(g, design, near)
    assert info2["done"] and np.nanmax(np.abs(t2 - design)) == 0
    # repeated passes converge to design
    z = ground.copy()
    for _ in range(5):
        t, info = next_target(g, design, z)
        z = np.where(np.isfinite(t), t, z)
    assert info["done"]


# -- exports -----------------------------------------------------------------------------------
def test_landxml_site_and_projected(site):
    """An exported LandXML surface reads back to the same heights, has a hole where the exclusion zone is, and can be
    written in projected coordinates and feet.
    """
    m, g, tmp = site
    design, ground = g.surface(m.site["design"]), g.surface(m.site["existing"])
    t, _ = next_target(g, design, ground)
    landxml(str(tmp / "s.xml"), g, t, None)
    pts, faces, name = read_tin(str(tmp / "s.xml"))
    back = rasterize(pts, faces, g.x0, g.y0, g.cell, g.nx, g.ny)      # samples land on the vertices
    both = np.isfinite(t) & np.isfinite(back)
    assert both.sum() > 0.9 * np.isfinite(t).sum()
    np.testing.assert_allclose(back[both], t[both], atol=1e-4)
    assert name == "LOAM" and len(faces) < 2 * (g.nx - 1) * (g.ny - 1)    # exclusion left a hole
    f = Frame(1.0, 40.0, None, None, E0, N0, 0.3048, dz=0.0)
    landxml(str(tmp / "p.xml"), g, design, f, "FG", "ft")
    pts2, _, _ = read_tin(str(tmp / "p.xml"))
    x, _ = f.projected_to_local(pts2[:, 0], pts2[:, 1])
    assert x.min() == pytest.approx(g.xs[0], abs=1e-3) and pts2[0, 2] == pytest.approx(100 / 0.3048, abs=1e-3)
    assert "foot" in (tmp / "p.xml").read_text()
    with pytest.raises(ValueError, match="no complete cells"):
        landxml(str(tmp / "n.xml"), g, np.full(g.shape, np.nan), None)


def _read_agd(path):
    """OpenGrade3D FileOpenAgdDesign, restated: skip the header, split on commas, code in column 5."""
    pts = []
    with open(path) as f:
        f.readline()
        for line in f:
            w = line.rstrip("\n").split(",")
            code = w[5].strip()
            if code in ("MB", "0MB", "2PER") or code.startswith("2SUBZONE"):
                pts.append((float(w[0]), float(w[1]), float(w[2]), -999.0, code))
            elif code == "3GRD":
                if w[2].strip() in ("", "0"):
                    continue
                prop = -999.0 if w[3].strip() in ("", "0") or w[4].strip() == "" else float(w[3])
                pts.append((float(w[0]), float(w[1]), float(w[2]), prop, code))
    return pts


def test_agd_matches_opengrade3d_parser(site):
    """The AGD export, read back the way OpenGrade3D itself reads it, has one master benchmark, four perimeter points
    and a grid of existing and proposed heights.
    """
    m, g, tmp = site
    design, ground = g.surface(m.site["design"]), g.surface(m.site["existing"])
    t, _ = next_target(g, design, ground)
    f = Frame(1.0, 0.0, LAT0, LON0)
    n = agd(str(tmp / "d.agd"), g, ground, t, f, step=2)
    pts = _read_agd(str(tmp / "d.agd"))
    codes = [p[4] for p in pts]
    assert codes[0] == "MB" and codes.count("MB") == 1 and codes.count("2PER") == 4
    assert codes.count("3GRD") == n == 20 * 10
    lat, lon = f.local_to_wgs84(g.xs[2], g.ys[0])
    grd = [p for p in pts if p[4] == "3GRD"]
    assert grd[1][0] == pytest.approx(float(lat), abs=1e-8) and grd[1][1] == pytest.approx(float(lon), abs=1e-8)
    assert grd[1][2] == pytest.approx(100.25) and grd[1][3] == pytest.approx(100.15)
    assert sum(p[3] == -999.0 for p in grd) > 0                   # exclusion cells carry no design
    assert agd(str(tmp / "e.agd"), g, ground, t, f) == g.nx * g.ny   # 1 m grid -> step 1


def test_agopengps_folder_matches_its_parsers(site):
    """The AgOpenGPS field folder, read back the way AgOpenGPS reads it, has the origin, the boundary rings and one
    guidance line per zone.
    """
    _, g, tmp = site
    f = Frame(1.0, 15.0, LAT0, LON0)
    out = agopengps(str(tmp / "aog"), g, f)
    assert out["inner_boundaries"] == ["X"] and out["tracks"] == ["W", "E"]
    lines = (tmp / "aog" / "Field.txt").read_text().splitlines()
    k = lines.index("StartFix")                              # FieldPlaneFiles.LoadOrigin
    assert tuple(float(v) for v in lines[k + 1].split(",")) == (LAT0, LON0)
    b = (tmp / "aog" / "Boundary.txt").read_text().splitlines()
    assert b[0] == "$Boundary"
    rings, i = [], 1
    while i < len(b):                                        # BoundaryStreamer.Read
        assert b[i] in ("True", "False")
        cnt = int(b[i + 1])
        rings.append([tuple(float(v) for v in ln.split(",")) for ln in b[i + 2:i + 2 + cnt]])
        i += 2 + cnt
    assert len(rings) == 2 and len(rings[0]) == 4
    assert rings[0][1][:2] == pytest.approx(tuple(float(v) for v in f.local_to_en(40, 0)), abs=1e-3)
    tl = (tmp / "aog" / "TrackLines.txt").read_text().splitlines()
    assert tl[0].startswith("$")                             # TrackFiles.Load
    tracks, i = [], 1
    while i < len(tl):
        name, heading = tl[i], float(tl[i + 1])
        a = tuple(float(v) for v in tl[i + 2].split(","))
        bb = tuple(float(v) for v in tl[i + 3].split(","))
        nudge, mode, vis, cnt = float(tl[i + 4]), int(tl[i + 5]), tl[i + 6], int(tl[i + 7])
        assert mode == 2 and vis == "True" and cnt == 0 and nudge == 0
        assert heading == pytest.approx(math.atan2(bb[0] - a[0], bb[1] - a[1]) % (2 * math.pi), abs=1e-3)
        tracks.append(name)
        i += 8 + cnt
    assert tracks == ["loam W", "loam E"]
    with pytest.raises(ValueError, match="WGS84"):
        agopengps(str(tmp / "x"), g, Frame(1.0))


# -- command line ----------------------------------------------------------------------------
def test_cli_field_commands(tmp_path, capsys):
    """The field commands run on the example: `terrain`, `export` to every format, `score` and `nmea`."""
    from loam.cli import load
    model = load(EXAMPLE)
    g = SiteGrid(model, EXAMPLES)
    ex = g.surface(model.site["existing"])
    gx, gy = np.meshgrid(g.xs, g.ys)
    frame = ["--origin", f"{LAT0},{LON0}", "--world-origin", f"{E0},{N0}", "--rotation", "10"]
    f = Frame(0.3048, 10.0, LAT0, LON0, E0, N0)
    e, n = f.local_to_projected(gx.ravel(), gy.ravel())
    cloud = str(tmp_path / "scan.las")
    write_las(cloud, np.column_stack([e, n, ex.ravel() * 0.3048]))
    main(["terrain", EXAMPLE, cloud, *frame, "--wire", str(tmp_path / "w.jsonl"), "--landxml",
          str(tmp_path / "scan.xml")])
    assert "cells_covered_pct: 100.0" in capsys.readouterr().out
    msg = json.loads((tmp_path / "w.jsonl").read_text())
    assert msg["topic"] == "loam/v0/pad_north/terrain" and msg["payload"]["shape"] == [g.ny, g.nx]
    for to, out in (("landxml", "n.xml"), ("agd", "n.agd")):
        main(["export", EXAMPLE, "--to", to, "-o", str(tmp_path / out), "--ground", cloud, *frame, "--json"])
        r = json.loads(capsys.readouterr().out)
        assert r["target"]["max_cut"] == pytest.approx(0.5, abs=1e-3)
    main(["export", EXAMPLE, "--to", "landxml", "--surface", "design", "-o", str(tmp_path / "fg.xml"), "--json"])
    assert json.loads(capsys.readouterr().out)["coordinates"] == "site"
    main(["export", EXAMPLE, "--to", "agopengps", "-o", str(tmp_path / "aog"), *frame])
    assert os.path.exists(tmp_path / "aog" / "TrackLines.txt")
    capsys.readouterr()
    survey = tmp_path / "s.csv"
    pe, pn = f.local_to_projected(80.0, 50.0)
    survey.write_text(f"Name,Easting,Northing,Elevation\na,{float(pe)},{float(pn)},{30.0}\n")
    main(["score", EXAMPLE, str(survey), "--zone", "pad", *frame, "--json"])
    assert json.loads(capsys.readouterr().out)["points"] == 1
    main(["export", EXAMPLE, "--to", "landxml", "-o", str(tmp_path / "v.xml"), "--ground", str(survey),
          *frame, "--json"])
    assert json.loads(capsys.readouterr().out)["ground"]["source"] == "survey"
    nmea = tmp_path / "gga.txt"
    nmea.write_text(_gga(LAT0, LON0) + "\n")
    main(["nmea", EXAMPLE, str(nmea), "--machine", "dz1", *frame, "-o", str(tmp_path / "s.jsonl")])
    assert json.loads((tmp_path / "s.jsonl").read_text())["topic"] == "loam/v0/pad_north/dz1/state"
    with pytest.raises(SystemExit):                     # agd needs --origin
        main(["export", EXAMPLE, "--to", "agd", "-o", str(tmp_path / "z.agd")])


@pytest.mark.slow
def test_grid_cloud_memory_is_bounded(site):
    """Three million points are gridded in under 40 MB: the cloud is read in chunks and never held whole."""
    _, g, tmp = site
    n = 3_000_000
    rng = np.random.default_rng(0)
    path = str(tmp / "big.las")
    write_las(path, np.column_stack([rng.uniform(0, 40, n), rng.uniform(0, 20, n), rng.uniform(99, 101, n)]))
    tracemalloc.start()
    grid_cloud(path, g, Frame(1.0), chunk=250_000)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    assert peak < 40e6, f"peak {peak / 1e6:.1f} MB for {n} points"
