"""`loam level`: a levelling design from the ground alone, and the field-boundary files it reads."""
import json
import math
import zipfile

import numpy as np
import pytest

from loam.cli import load, main
from loam.landxml import read_project, read_tin, write_grid_tin
from loam.level import balance, edge_distance, fit_plane, level, shape_grade
from loam.plan import plan
from loam.sim import Events, Sim

SQUARE = [(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0)]


def _points(seed=0, n=400):
    """Scattered points over a 300 by 200 field."""
    rng = np.random.default_rng(seed)
    return rng.uniform(0, 300, n), rng.uniform(0, 200, n), rng


def _ground(path, unit="ft", rough=0.35):
    """About an acre of gently falling ground with humps and one low spot, as a LandXML surface 'EG'. Returns the
    lattice and heights."""
    xs, ys = np.arange(0, 221, 2.0), np.arange(0, 201, 2.0)
    gx, gy = np.meshgrid(xs, ys)
    z = (100 + 0.006 * gx - 0.003 * gy + rough * np.sin(gx / 31) * np.cos(gy / 27)
         - 0.5 * np.exp(-((gx - 120) ** 2 + (gy - 90) ** 2) / 1500))
    write_grid_tin(str(path), "EG", xs, ys, z, unit)
    return xs, ys, z


def _fall(a, b):
    """Grade (rise over run) and the bearing water runs, for plane slopes (a, b)."""
    return math.hypot(a, b), math.degrees(math.atan2(-a, -b)) % 360


def test_the_fit_recovers_a_plane_and_limits_keep_its_direction():
    """Ground that is a plane is fitted exactly. A grade outside the limits is brought to the nearest limit without
    turning the plane, and a grade inside them is left alone."""
    x, y, _ = _points()
    z = 50 + 0.004 * x - 0.003 * y
    assert fit_plane(x, y, z) == pytest.approx((0.004, -0.003), abs=1e-12)
    assert shape_grade(x, y, z, 0.001, 0.01) == pytest.approx((0.004, -0.003), abs=1e-12)
    for lo, hi, want in ((0.0, 0.002, 0.002), (0.008, 0.01, 0.008)):
        s, toward = _fall(*shape_grade(x, y, z, lo, hi))
        assert s == pytest.approx(want) and toward == pytest.approx(_fall(0.004, -0.003)[1])


def test_a_fall_bearing_sets_the_main_grade_and_limits_the_cross_grade():
    """With a bearing, the grade down that bearing is held within the limits, even where the ground runs the other
    way, and the cross grade is the best fit unless it is limited too."""
    x, y, rng = _points(1)
    z = 50 - 0.004 * x + 0.001 * y + rng.normal(0, 0.05, len(x))       # falls east at 0.4 %, north-south 0.1 %
    a, b = shape_grade(x, y, z, 0.001, 0.002, fall=90)                 # drain east, at most 0.2 %
    assert -a == pytest.approx(0.002) and b == pytest.approx(0.001, abs=4e-4)
    a, b = shape_grade(x, y, z, 0.001, 0.002, fall=90, cross_slope=0.0005)
    assert -a == pytest.approx(0.002) and abs(b) == pytest.approx(0.0005)
    a, b = shape_grade(x, y, z, 0.001, 0.002, fall=270)                # ask it to drain uphill: least grade west
    assert a == pytest.approx(0.001)
    # A fixed main grade changes the best cross grade when the field is skewed; the answer is the least-squares
    # one, checked against a direct solve.
    x2 = x + 0.8 * y
    z2 = 50 - 0.004 * x2 + 0.001 * y + rng.normal(0, 0.05, len(x))
    a, b = shape_grade(x2, y, z2, 0.0, 0.002, fall=90)
    yc = y - y.mean()
    best = float(yc @ ((z2 - z2.mean()) + 0.002 * (x2 - x2.mean())) / (yc @ yc))
    assert -a == pytest.approx(0.002) and b == pytest.approx(best)


def test_slope_limits_that_cannot_be_met_are_refused():
    """Level ground has no direction to drain in, so a minimum grade needs a bearing; limits must make sense."""
    x, y, _ = _points()
    flat = np.full(len(x), 10.0)
    assert shape_grade(x, y, flat) == (0.0, 0.0)
    with pytest.raises(ValueError, match="--fall"):
        shape_grade(x, y, flat, 0.001)
    assert shape_grade(x, y, flat, 0.001, fall=180) == pytest.approx((0.0, 0.001), abs=1e-5)
    with pytest.raises(ValueError, match="limits"):
        shape_grade(x, y, flat, 0.01, 0.001)


@pytest.mark.parametrize("ratio", [0.8, 1.0, 1.3, 2.0])
def test_balance_meets_the_cut_fill_ratio(ratio):
    """The shift makes cut equal to the ratio times fill, with every point counted by its weight."""
    rng = np.random.default_rng(3)
    diff, weight = rng.normal(0.4, 1.0, 5000), rng.uniform(0, 1, 5000)
    d = weight * (diff + balance(diff, weight, ratio))
    cut, fill = -d[d < 0].sum(), d[d > 0].sum()
    assert cut / fill == pytest.approx(ratio, rel=1e-6)
    with pytest.raises(ValueError, match="positive"):
        balance(diff, weight, 0.0)
    with pytest.raises(ValueError, match="edge blend"):
        balance(diff, np.zeros(5000), ratio)


def test_edge_distance_is_the_distance_to_the_nearest_side():
    """Checked on a rectangle, where the answer is known, inside and outside."""
    gx, gy = np.array([50.0, 10.0, 99.0, 50.0, 110.0]), np.array([40.0, 70.0, 5.0, 79.5, 90.0])
    assert edge_distance(SQUARE, gx, gy) == pytest.approx([40.0, 10.0, 1.0, 0.5, math.hypot(10, 10)])


def test_level_designs_a_plane_that_balances_and_the_program_finishes(tmp_path):
    """From a LandXML ground alone: the design is one plane within the slope limits, its cut is 1.3 times its
    fill, and the program written for it checks clean, has no compaction and, simulated, finishes in tolerance."""
    _, _, z = _ground(tmp_path / "ground.xml")
    r = level(str(tmp_path / "ground.xml"), str(tmp_path / "field.loam"), min_slope=0.2, max_slope=0.5)
    assert r["units"] == "ft" and r["slope_pct"] == pytest.approx(0.5) and r["existing"] == "EG"
    pts, _, name = read_tin(r["design_file"])
    assert name == "level" and r["ground_file"].endswith("ground.xml")
    a, b = fit_plane(pts[:, 0], pts[:, 1], pts[:, 2])
    flat = pts[:, 2] - a * pts[:, 0] - b * pts[:, 1]
    assert np.ptp(flat) < 2e-3                                        # a plane, to the file's millimetre rounding
    assert _fall(a, b) == pytest.approx((0.005, r["falls_toward_deg"]), abs=0.06)
    # The design points are lattice points of the ground file here, so the balance can be checked exactly.
    d = pts[:, 2] - _height(z, pts)
    assert -d[d < 0].sum() / d[d > 0].sum() == pytest.approx(1.3, rel=0.01)
    assert r["deepest_cut_ft"] == pytest.approx(-d.min(), abs=0.01)
    m = load(str(tmp_path / "field.loam"))
    assert set(m.tasks) == {"cut_to_fill", "top_up", "finish_grade"} and set(m.machines) == {"dz1", "dz2"}
    assert "compaction" not in m.zones["fill_area"]
    text = (tmp_path / "field.loam").read_text()
    assert "cut/fill ratio 1.3" in text and "topsoil" in text and "hold all" in text
    assert max(len(ln) for ln in text.splitlines()) <= 120
    rep = Sim(m, str(tmp_path), events=Events.none(), log=lambda x: None).run()
    assert rep["finished"] and rep["true_ground"]["within_tolerance_pct"] >= 95


def _height(z, pts):
    """Ground height at design points that are lattice points of `_ground` (2 ft apart, from the origin)."""
    i, j = np.rint(pts[:, 0] / 2).astype(int), np.rint(pts[:, 1] / 2).astype(int)
    assert np.allclose(pts[:, 0], i * 2, atol=1e-6) and np.allclose(pts[:, 1], j * 2, atol=1e-6)
    return z[j, i]


def test_the_edge_blend_meets_the_ground_at_the_boundary(tmp_path):
    """With an edge width, the design equals the ground at the field's edge and is the plane beyond that width."""
    _, _, z = _ground(tmp_path / "ground.xml")
    (tmp_path / "field.geojson").write_text(json.dumps({"type": "Polygon", "coordinates": [[
        [20, 20], [200, 20], [200, 180], [20, 180], [20, 20]]]}))
    r = level(str(tmp_path / "ground.xml"), str(tmp_path / "field.loam"), boundary=str(tmp_path / "field.geojson"),
              edge="30 ft", max_cells=10_000)
    pts, _, _ = read_tin(r["design_file"])
    ring = [(20.0, 20.0), (200.0, 20.0), (200.0, 180.0), (20.0, 180.0)]
    dist = edge_distance(ring, pts[:, 0], pts[:, 1])
    change = pts[:, 2] - _height(z, pts)
    assert np.abs(change[dist < 0.5]).max() < 0.02                    # the edge keeps its ground
    inner = dist >= 30
    a, b = fit_plane(pts[inner, 0], pts[inner, 1], pts[inner, 2])
    assert np.ptp(pts[inner, 2] - a * pts[inner, 0] - b * pts[inner, 1]) < 2e-3
    assert np.abs(change).max() > 0.2 and "blended into the ground over 30 ft" in (tmp_path / "field.loam").read_text()
    m = load(str(tmp_path / "field.loam"))
    assert [tuple(p) for p in m.site["boundary"]["polygon"]] == ring  # the surveyed boundary, not a staircase


def _dem(path, crs, e0, n0, n=240, res=1.0, scale=1.0):
    """A square elevation raster with its south-west corner at (e0, n0): tilted ground with humps. `scale`
    multiplies the heights (metres to feet, say)."""
    rasterio = pytest.importorskip("rasterio")
    from rasterio.transform import from_origin
    x, y = np.arange(n) + 0.5, n - (np.arange(n) + 0.5)
    gx, gy = np.meshgrid(x, y)
    z = 380 + 0.004 * gx - 0.002 * gy + 0.25 * np.sin(gx / 23) * np.cos(gy / 31)
    with rasterio.open(path, "w", driver="GTiff", height=n, width=n, count=1, dtype="float32", crs=crs,
                       transform=from_origin(e0, n0 + n * res, res, res)) as dst:
        dst.write((z * scale).astype("float32"), 1)


def _shp(path, rings=None, point=None):
    """A shapefile holding one polygon (rings, the first the outer one) or one point. The three files are opened and
    closed here because pyshp's own writer leaves one open."""
    shapefile = pytest.importorskip("shapefile")
    stem = str(path)[:-4]
    with open(stem + ".shp", "wb") as shp, open(stem + ".shx", "wb") as shx, open(stem + ".dbf", "wb") as dbf:
        w = shapefile.Writer(shp=shp, shx=shx, dbf=dbf)
        w.field("name", "C")
        if point:
            w.point(*point)
        else:
            w.poly(rings)
        w.record("north field")
        w.close()


def _kml(path, lonlat):
    """A KML file with one polygon."""
    ring = " ".join(f"{a:.9f},{b:.9f},0" for a, b in [*lonlat, lonlat[0]])
    path.write_text('<?xml version="1.0"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark>'
                    "<Polygon><outerBoundaryIs><LinearRing><coordinates>" + ring + "</coordinates></LinearRing>"
                    "</outerBoundaryIs></Polygon></Placemark></Document></kml>")


def test_level_from_an_elevation_raster_and_a_kml_boundary(tmp_path, capsys):
    """The free-data path: a projected elevation raster (as USGS 3DEP 1 m tiles are) and a field traced in Google
    Earth. The boundary is placed from latitude and longitude, the program carries the raster's CRS, and the
    ground is written out for the site grid."""
    pyproj = pytest.importorskip("pyproj")
    e0, n0 = 640000.0, 3846000.0
    _dem(tmp_path / "dem.tif", "EPSG:26916", e0, n0)
    ring = [(e0 + 40, n0 + 50), (e0 + 200, n0 + 60), (e0 + 190, n0 + 190), (e0 + 50, n0 + 170)]
    back = pyproj.Transformer.from_crs("EPSG:26916", "EPSG:4326", always_xy=True)
    _kml(tmp_path / "field.kml", [back.transform(*p) for p in ring])
    main(["level", str(tmp_path / "dem.tif"), "--boundary", str(tmp_path / "field.kml"), "-o",
          str(tmp_path / "out" / "field.loam"), "--max-slope", "0.3", "--json"])
    r = json.loads(capsys.readouterr().out)
    assert r["check"] == "ok" and r["units"] == "m" and r["slope_pct"] == pytest.approx(0.3)
    assert r["falls_toward_deg"] == pytest.approx(math.degrees(math.atan2(-0.004, 0.002)) % 360, abs=2)
    m = load(r["out"])
    assert m.site["crs"] == "EPSG:26916"
    assert np.allclose(m.site["boundary"]["polygon"], ring, atol=0.01)
    assert read_project(r["ground_file"])["epsg"] == "26916" and read_project(r["design_file"])["unit"] == "m"
    # The same field with the boundary as GeoJSON in longitude and latitude, recognised by its numbers.
    (tmp_path / "field.geojson").write_text(json.dumps({"type": "Feature", "geometry": {
        "type": "Polygon", "coordinates": [[list(back.transform(*p)) for p in [*ring, ring[0]]]]}}))
    r2 = level(str(tmp_path / "dem.tif"), str(tmp_path / "g" / "f.loam"), boundary=str(tmp_path / "field.geojson"),
               max_slope=0.3)
    assert r2["cut_m3"] == pytest.approx(r["cut_m3"], abs=5) and r2["slope_pct"] == r["slope_pct"]
    # Heights in feet on a metre grid: say so, and the design comes out the same.
    _dem(tmp_path / "ft.tif", "EPSG:26916", e0, n0, scale=1 / 0.3048)
    r3 = level(str(tmp_path / "ft.tif"), str(tmp_path / "h" / "f.loam"), boundary=str(tmp_path / "field.kml"),
               max_slope=0.3, height_unit="ft")
    assert r3["cut_m3"] == pytest.approx(r["cut_m3"], abs=5) and r3["slope_pct"] == pytest.approx(0.3)


def test_level_from_a_raster_in_degrees_or_with_no_boundary(tmp_path):
    """A raster in longitude and latitude (as 3DEP 1/3 arc-second tiles are) is worked in the UTM zone the field
    lies in. A small raster with no boundary is levelled whole (a drone survey of just the field)."""
    pytest.importorskip("pyproj")
    lon0, lat0, d = -85.70, 34.75, 1e-5                              # about a metre a pixel
    _dem(tmp_path / "geo.tif", "EPSG:4269", lon0, lat0, n=300, res=d)
    ll = [(lon0 + 60 * d, lat0 + 60 * d), (lon0 + 220 * d, lat0 + 60 * d), (lon0 + 220 * d, lat0 + 200 * d),
          (lon0 + 60 * d, lat0 + 200 * d)]
    _kml(tmp_path / "field.kml", ll)
    with zipfile.ZipFile(tmp_path / "field.kmz", "w") as zf:        # Google Earth saves KMZ by default
        zf.write(tmp_path / "field.kml", "doc.kml")
    r = level(str(tmp_path / "geo.tif"), str(tmp_path / "a" / "f.loam"), boundary=str(tmp_path / "field.kmz"))
    m = load(r["out"])
    assert m.site["crs"] == "EPSG:32616" and r["units"] == "m"       # UTM zone 16 north
    xy = np.array(m.site["boundary"]["polygon"])
    sides = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)             # east-west and north-south, in metres
    assert sides[0] == pytest.approx(160 * d * 111320 * math.cos(math.radians(lat0)), rel=0.01)
    assert sides[1] == pytest.approx(140 * d * 110950, rel=0.01)
    with pytest.raises(ValueError, match="which field"):
        level(str(tmp_path / "geo.tif"), str(tmp_path / "b" / "f.loam"))
    _dem(tmp_path / "odm.tif", "EPSG:32616", 640000.0, 3846000.0, n=120)
    r = level(str(tmp_path / "odm.tif"), str(tmp_path / "c" / "f.loam"), max_cells=4000)
    m = load(r["out"])
    assert r["grid"] == "2 m" and len(m.site["boundary"]["polygon"]) == 4
    xy = np.array(m.site["boundary"]["polygon"])
    assert xy.min() >= 640000.5 and np.ptp(xy[:, 0]) >= 116          # all of it but the rim


def test_level_refuses_what_it_cannot_place(tmp_path, capsys):
    """Clear reasons, one line each: no unit, a boundary off the ground, a field too big for the grid, a boundary
    in map coordinates on ground that has none."""
    pytest.importorskip("pyproj")
    _dem(tmp_path / "bare.tif", None, 0.0, 0.0, n=60)
    with pytest.raises(SystemExit):
        main(["level", str(tmp_path / "bare.tif"), "-o", str(tmp_path / "x.loam")])
    assert "--units" in capsys.readouterr().err
    r = level(str(tmp_path / "bare.tif"), str(tmp_path / "u" / "x.loam"), units="ft")
    assert r["units"] == "ft" and "LOCAL:ft" in (tmp_path / "u" / "x.loam").read_text()
    _kml(tmp_path / "far.kml", [(-85.7, 34.75), (-85.699, 34.75), (-85.699, 34.751)])
    with pytest.raises(ValueError, match="no coordinate system"):
        level(str(tmp_path / "bare.tif"), str(tmp_path / "x.loam"), units="ft", boundary=str(tmp_path / "far.kml"))
    _dem(tmp_path / "dem.tif", "EPSG:26916", 640000.0, 3846000.0, n=60)
    with pytest.raises(ValueError, match="uncovered"):
        level(str(tmp_path / "dem.tif"), str(tmp_path / "x.loam"), boundary=str(tmp_path / "far.kml"))
    with pytest.raises(ValueError, match="--boundary"):
        level(str(tmp_path / "dem.tif"), str(tmp_path / "x.loam"), max_cells=100)
    _ground(tmp_path / "ground.xml")
    with pytest.raises(ValueError, match="raster"):
        level(str(tmp_path / "ground.xml"), str(tmp_path / "x.loam"), height_unit="m")


def test_reading_a_raster_takes_only_the_window_it_needs(tmp_path, monkeypatch):
    """Sampling a corner of a large elevation tile reads that corner, not the tile, and gives the same heights as
    the whole tile would."""
    rasterio = pytest.importorskip("rasterio")
    from loam.surfaces import raster_at
    _dem(tmp_path / "tile.tif", "EPSG:26916", 0.0, 0.0, n=1200)
    gx, gy = np.meshgrid(np.linspace(30.3, 90.7, 40), np.linspace(1100.2, 1150.9, 30))
    read = []
    real = rasterio.io.DatasetReader.read

    def spy(self, *a, **k):
        """Record the shape of every block read from the file."""
        out = real(self, *a, **k)
        read.append(out.shape)
        return out
    monkeypatch.setattr(rasterio.io.DatasetReader, "read", spy)
    z, crs = raster_at(str(tmp_path / "tile.tif"), gx, gy)
    assert crs == "EPSG:26916" and read == [(53, 63)]
    want = 380 + 0.004 * gx - 0.002 * gy + 0.25 * np.sin(gx / 23) * np.cos(gy / 31)
    assert np.allclose(z, want, atol=2e-3)
    # Points off the tile have no height, alone or mixed with points on it.
    z, _ = raster_at(str(tmp_path / "tile.tif"), np.array([-5.0, 600.0, 5000.0]), np.array([600.0, 600.0, 600.0]))
    assert np.isnan(z[0]) and np.isfinite(z[1]) and np.isnan(z[2])
    assert np.isnan(raster_at(str(tmp_path / "tile.tif"), np.array([5000.0]), np.array([9000.0]))[0]).all()


def test_field_boundaries_from_kml_and_shapefile(tmp_path):
    """The outer ring of the first polygon, closing point dropped, with the coordinate system the file states."""
    from loam.surfaces import polygon_from_file, polygon_from_kml
    ll = [(-85.7, 34.75), (-85.699, 34.75), (-85.699, 34.751), (-85.7, 34.751)]
    _kml(tmp_path / "f.kml", ll)
    assert polygon_from_file(str(tmp_path / "f.kml")) == (ll, "EPSG:4326")
    (tmp_path / "empty.kml").write_text('<kml xmlns="http://www.opengis.net/kml/2.2"><Placemark/></kml>')
    (tmp_path / "broken.kml").write_text("<kml><Polygon>")
    (tmp_path / "words.kml").write_text("<kml><Polygon><coordinates>north of the barn</coordinates></Polygon></kml>")
    (tmp_path / "two.kml").write_text("<kml><Polygon><coordinates>1,2 3,4 1,2</coordinates></Polygon></kml>")
    with zipfile.ZipFile(tmp_path / "none.kmz", "w") as zf:
        zf.writestr("readme.txt", "no kml here")
    for name, why in (("empty.kml", "no Polygon"), ("broken.kml", "not a KML"), ("words.kml", "unreadable"),
                      ("two.kml", "fewer than 3"), ("none.kmz", "no .kml inside")):
        with pytest.raises(ValueError, match=why):
            polygon_from_kml(str(tmp_path / name))
    pytest.importorskip("shapefile")
    ring = [(640040.0, 3846050.0), (640050.0, 3846170.0), (640190.0, 3846190.0), (640200.0, 3846060.0)]
    _shp(tmp_path / "gate.shp", point=(1, 2))                         # not a polygon: skipped
    with pytest.raises(ValueError, match="no polygon"):
        polygon_from_file(str(tmp_path / "gate.shp"))
    hole = [(640100.0, 3846100.0), (640110.0, 3846100.0), (640110.0, 3846110.0)]
    _shp(tmp_path / "clu.shp", [[*ring, ring[0]], [*hole, hole[0]]])  # a field with a pond: the outer ring is used
    assert polygon_from_file(str(tmp_path / "clu.shp")) == (ring, None)
    (tmp_path / "clu.prj").write_text('GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,'
                                      '298.257223563]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]')
    assert polygon_from_file(str(tmp_path / "clu.shp"))[1].startswith("GEOGCS")
    (tmp_path / "junk.shp").write_bytes(b"not a shapefile")
    with pytest.raises(ValueError, match="not a shapefile"):
        polygon_from_file(str(tmp_path / "junk.shp"))


def test_a_shapefile_with_a_prj_places_the_field(tmp_path):
    """A shapefile in latitude and longitude, as the FSA publishes field boundaries, is reprojected by its .prj."""
    pyproj = pytest.importorskip("pyproj")
    pytest.importorskip("shapefile")
    e0, n0 = 640000.0, 3846000.0
    _dem(tmp_path / "dem.tif", "EPSG:26916", e0, n0)
    ring = [(e0 + 40, n0 + 50), (e0 + 50, n0 + 170), (e0 + 190, n0 + 190), (e0 + 200, n0 + 60)]
    back = pyproj.Transformer.from_crs("EPSG:26916", "EPSG:4269", always_xy=True)
    _shp(tmp_path / "clu.shp", [[back.transform(*p) for p in [*ring, ring[0]]]])
    (tmp_path / "clu.prj").write_text(pyproj.CRS("EPSG:4269").to_wkt("WKT1_ESRI"))
    r = level(str(tmp_path / "dem.tif"), str(tmp_path / "o" / "f.loam"), boundary=str(tmp_path / "clu.shp"))
    assert np.allclose(load(r["out"]).site["boundary"]["polygon"], ring, atol=0.01)


def test_area_files_in_a_program_may_be_shapefiles_but_not_kml(tmp_path):
    """A zone's `area file` is in site coordinates: a shapefile exported in the site's CRS works as GeoJSON does, and
    a KML, which is always latitude and longitude, is refused with the reason."""
    pytest.importorskip("shapefile")
    from loam.grid import SiteGrid
    _ground(tmp_path / "ground.xml")
    level(str(tmp_path / "ground.xml"), str(tmp_path / "field.loam"))
    ring = [(40.0, 40.0), (40.0, 120.0), (160.0, 120.0), (160.0, 40.0)]
    _shp(tmp_path / "wet.shp", [[*ring, ring[0]]])
    text = (tmp_path / "field.loam").read_text()
    zone = 'zone wet {{ role exclusion; area file "{}" }}\n'
    (tmp_path / "shp.loam").write_text(text + zone.format("wet.shp"))
    g = SiteGrid(load(str(tmp_path / "shp.loam")), str(tmp_path))
    assert g.masks["wet"].sum() * g.area == pytest.approx(120 * 80, rel=0.02)
    _kml(tmp_path / "wet.kml", [(-85.7, 34.75), (-85.699, 34.75), (-85.699, 34.751)])
    (tmp_path / "kml.loam").write_text(text + zone.format("wet.kml"))
    with pytest.raises(ValueError, match="site coordinates"):
        SiteGrid(load(str(tmp_path / "kml.loam")), str(tmp_path))


def test_a_fleet_without_dozers_gets_its_tasks_by_capability(tmp_path, capsys):
    """A tractor-drawn scraper and a land plane (a grader, to Loam) do a farm's levelling. The tasks are then
    given by capability, the stale rule follows the fleet's own probe, and the job still finishes. A pad is
    levelled with compaction on request."""
    _ground(tmp_path / "ground.xml")
    (tmp_path / "fleet.loam").write_text(
        "probe rover1 { kind rover; sensor lidar; rate 10 min; accuracy 0.05 ft; coverage site; stale_after 30 min }\n"
        "machine sc1 { class scraper; blade 10 ft; max_speed 5 mph }\n"
        "machine lp1 { class grader; blade 12 ft; max_speed 4 mph }\n")
    main(["level", str(tmp_path / "ground.xml"), "-o", str(tmp_path / "farm.loam"), "--fleet",
          str(tmp_path / "fleet.loam"), "--fall", "300", "--min-slope", "0.2", "--max-slope", "0.4",
          "--cross-slope", "0.1", "--edge", "15 ft", "--json"])
    r = json.loads(capsys.readouterr().out)
    assert r["check"] == "ok" and 0.4 <= r["slope_pct"] <= math.hypot(0.4, 0.1) + 1e-3
    text = (tmp_path / "farm.loam").read_text()
    assert "by      cap cut" in text and "by     cap spread" in text and "by     cap finish" in text
    assert "age(rover1) > 30 min" in text and "class dozer" not in text
    m = load(str(tmp_path / "farm.loam"))
    rep = Sim(m, str(tmp_path), events=Events.none(), log=lambda x: None).run()
    assert rep["finished"] and rep["true_ground"]["within_tolerance_pct"] >= 95
    r = level(str(tmp_path / "ground.xml"), str(tmp_path / "pad.loam"), compaction=True)
    m = load(r["out"])
    assert "compact" in m.tasks and "rl1" in m.machines
    assert "pause task compact" in (tmp_path / "pad.loam").read_text()
    # `loam plan` keeps writing for dozers and a roller by default, and takes the same option.
    r = plan(r["design_file"], str(tmp_path / "p.loam"), design="level", existing="EG",
             existing_file=str(tmp_path / "ground.xml"), compaction=False)
    assert "compact" not in load(r["out"]).tasks
