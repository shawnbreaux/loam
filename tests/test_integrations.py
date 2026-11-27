"""Optional integrations: LAZ and ground filtering, CRSs (pyproj), GeoTIFF (rasterio), IFC 4.3
(IfcOpenShell), GeoJSON areas (QGIS), and a cross-check of the ground filter against PDAL. Each test skips when its
FOSS extra or tool is not installed."""
import json
import os
import shutil
import subprocess

import numpy as np
import pytest

from loam.checker import check, has_errors
from loam.cli import main
from loam.geo import Frame
from loam.grid import SiteGrid
from loam.landxml import write_grid_tin
from loam.parser import parse
from loam.pointcloud import grid_cloud, ground_surface, write_las

E0, N0 = 600000.0, 3812000.0                 # UTM 16N, DeKalb County, AL
UTM = "EPSG:32616"
SP = "EPSG:26929"                            # NAD83 / Alabama East (m)

# A small metric site whose design file, format, boundary and zone area are filled in by each test.
SRC = """loam 0.0.2
site t {{ crs "LOCAL:m"; units m; design "{design}" format {fmt} surface "FG"
          boundary {boundary}; grid 1 m }}
probe p {{ kind static; sensor lidar; rate 5 min; accuracy 2 cm }}
machine dz {{ class dozer; blade 3 m }}
zone W {{ role cut; area {area}; tolerance ±3 cm }}
task cutW {{ op cut; zone W; by class dozer; until residual(W).high < 3 cm }}
"""


def design(x, y):
    """The design surface used throughout: a plane sloping up 1 % to the east and 2 % to the north, from 100 m."""
    return 100 + 0.01 * x + 0.02 * y


def site(tmp_path, fmt="landxml", design_file="d.xml", boundary="rect (0, 0) (40, 20)", area="rect (0, 0) (20, 20)"):
    """A checked 40 x 20 m site and its grid. By default the design is written as LandXML; with another `fmt` the test
    supplies the design file itself.
    """
    if fmt == "landxml":
        xs, ys = np.arange(0.0, 41.0), np.arange(0.0, 21.0)
        gx, gy = np.meshgrid(xs, ys)
        write_grid_tin(str(tmp_path / design_file), "FG", xs, ys, design(gx, gy), "m")
    m, diags = check(parse(SRC.format(design=design_file, fmt=fmt, boundary=boundary, area=area)))
    assert not has_errors(diags), [d.msg for d in diags]
    return m, SiteGrid(m, str(tmp_path))


# -- ground filter ------------------------------------------------------------------------------
@pytest.mark.parametrize("name,surface", [
    ("gentle", lambda x, y: 0.05 * x + 0.02 * y + 0.3 * np.sin(x / 9)),
    ("3:1 bank", lambda x, y: np.clip((x - 20) / 3, 0, 8)),
    ("swale", lambda x, y: 0.6 * np.cos(y / 6)),
])
def test_ground_filter_removes_machines_and_brush(name, surface):
    """The ground filter recovers bare earth from a scan with brush (15 % of cells, 0.3 m high), a parked machine (3 m
    high) and gaps, on a gentle slope, a 3:1 bank and a swale.
    """
    c = 0.5
    y, x = np.mgrid[0:120, 0:120] * c
    g = surface(x, y)
    rng = np.random.default_rng(1)
    z = g + np.abs(rng.normal(0, 0.02, g.shape))
    brush = rng.random(g.shape) < 0.15
    z[brush] += 0.3
    machine = np.zeros(g.shape, bool)
    machine[40:50, 60:72] = True                 # 6 m x 5 m, 3 m tall
    z[machine] += 3.0
    z[rng.random(g.shape) < 0.05] = np.nan
    e = ground_surface(z, c) - g
    assert np.sqrt(np.mean(e[~brush & ~machine] ** 2)) < 0.025          # sensor noise is 0.02
    assert np.abs(e[machine]).max() < 0.15
    assert np.mean(np.abs(e[brush]) < 0.1) > 0.998


def test_ground_filter_removes_the_low_parts_of_a_machine(monkeypatch):
    """A dozer seen from above is not one block: the scanner also hits its blade and tracks, well below the cab.
    The filter removes all of it, where a single round leaves the low parts in the ground as a 0.9 m bump.
    """
    import loam.pointcloud as pc
    y, x = np.mgrid[0:60, 0:60] * 1.0
    g = 0.03 * x + 0.01 * y
    rng = np.random.default_rng(3)
    z = g + np.abs(rng.normal(0, 0.02, g.shape))
    body = np.zeros(g.shape, bool)
    body[20:25, 30:36] = True                    # 6 m x 5 m: cab and engine deck, 2 to 3 m
    low = np.zeros(g.shape, bool)
    low[20:25, 36] = True                        # the blade across the front, 0.9 m
    low[20, 30:36] = low[24, 30:36] = True       # the tracks along both sides, 0.6 m
    z[body] += rng.uniform(2.0, 3.0, g.shape)[body]
    z[low] = g[low] + np.where(x[low] == 36, 0.9, 0.6)
    e = ground_surface(z, 1.0) - g
    assert np.abs(e[body | low]).max() < 0.1 and np.sqrt(np.mean(e[~(body | low)] ** 2)) < 0.025
    monkeypatch.setattr(pc, "FILTER_ROUNDS", 1)
    assert np.abs((ground_surface(z, 1.0) - g)[low]).max() > 0.5


def test_ground_filter_degenerate():
    """An empty scan stays empty, and flat ground comes back unchanged."""
    assert np.isnan(ground_surface(np.full((5, 5), np.nan), 1.0)).all()
    flat = np.full((9, 9), 2.0)
    np.testing.assert_array_equal(ground_surface(flat, 1.0), flat)


# PDAL's program: named by the environment, on the PATH, or where tools/integration-envs.sh puts it.
PDAL = os.environ.get("PDAL") or shutil.which("pdal") or next(
    (c for c in (os.path.expanduser("~/loam-envs/mm/envs/pdal/bin/pdal"),) if os.path.exists(c)), None)


@pytest.mark.skipif(PDAL is None, reason="needs PDAL (PDAL=/path/to/pdal)")
def test_ground_filter_agrees_with_pdal_smrf(tmp_path):
    """Loam's ground filter against an independent one: PDAL's Simple Morphological Filter (Pingel et al., 2013).

    One scan of a sloping site with a parked machine, two people and brush is cleaned twice: by Loam's own filter,
    and by PDAL, whose ground classification Loam then reads with `--ground-only`. Both must match the true ground,
    and each other, so a site can use either: the built-in filter with no extra software, or PDAL when a surveyor
    wants a filter with a published record.
    """
    _, g = site(tmp_path)
    frame = Frame(1.0)
    rng = np.random.default_rng(7)
    x, y = rng.uniform(0, 40, 200_000), rng.uniform(0, 20, 200_000)
    z = design(x, y) + rng.normal(0, 0.01, x.size)
    machine = (x > 25) & (x < 31) & (y > 5) & (y < 10)
    z[machine] += rng.uniform(1.5, 3.0, machine.sum())
    people = ((np.hypot(x - 8, y - 6) < 0.3) | (np.hypot(x - 14, y - 15) < 0.3))
    z[people] += rng.uniform(0.3, 1.8, people.sum())
    brush = (rng.random(x.size) < 0.15) & ~machine & ~people
    z[brush] += rng.uniform(0.2, 0.6, brush.sum())
    raw, classified = tmp_path / "scan.las", tmp_path / "classified.las"
    write_las(str(raw), np.column_stack([x, y, z]), classification=1)           # 1 = not yet classified
    # The same height tolerance as Loam's filter (0.15 m); the rest are PDAL's defaults.
    done = subprocess.run([PDAL, "translate", str(raw), str(classified), "smrf", "--filters.smrf.threshold=0.15"],
                          capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stderr

    truth = design(*np.meshgrid(g.xs, g.ys))
    unfiltered, _ = grid_cloud(str(raw), g, frame)
    ours, info = grid_cloud(str(raw), g, frame, pmf={})
    theirs, pdal_info = grid_cloud(str(classified), g, frame, ground_only=True)
    assert info["ground_filter"] == "pmf" and pdal_info["ground_filter"] == "class 2"
    both = np.isfinite(ours) & np.isfinite(theirs)
    rms = [float(np.sqrt(np.nanmean(d ** 2))) for d in (ours - truth, theirs - truth, (ours - theirs)[both])]
    print(f"\nground filters on one scan: unfiltered up to {np.nanmax(unfiltered - truth):.2f} m high; "
          f"Loam {rms[0] * 1000:.1f} mm RMS from the truth, PDAL SMRF {rms[1] * 1000:.1f} mm, "
          f"{rms[2] * 1000:.1f} mm between them; cells with ground: Loam {np.isfinite(ours).sum()}, "
          f"PDAL {np.isfinite(theirs).sum()} of {ours.size}")
    assert np.nanmax(unfiltered - truth) > 0.5                       # the machine is in the raw surface
    assert np.nanmax(np.abs(ours - truth)) < 0.03 and np.nanmax(np.abs(theirs - truth)) < 0.03
    assert rms[2] < 0.005 and both.mean() > 0.95


# -- LAZ + CRS ------------------------------------------------------------------------------------
def write_laz(path, e, n, z, cls, crs=None):
    """Write points as compressed LAS 1.4 (LAZ) with classes and, optionally, a coordinate reference system."""
    laspy = pytest.importorskip("laspy")
    pytest.importorskip("lazrs")
    h = laspy.LasHeader(point_format=6, version="1.4")
    h.scales, h.offsets = np.array([0.001] * 3), np.array([e.min(), n.min(), z.min()])
    if crs:
        pyproj = pytest.importorskip("pyproj")
        h.add_crs(pyproj.CRS.from_user_input(crs))
    las = laspy.LasData(h)
    las.x, las.y, las.z, las.classification = e, n, z, cls
    las.write(str(path))


def test_laz_with_ground_filter_and_crs(tmp_path):
    """A drone scan in one coordinate system (UTM) is gridded onto a site defined in another (Alabama East state
    plane).

    Unfiltered, the machine's roof shows up in the surface. With the ground filter, or using the file's own ground
    classification, the surface matches the truth within 2 cm.
    """
    pyproj = pytest.importorskip("pyproj")
    _, g = site(tmp_path)
    frame = Frame(1.0, 0.0, None, None, *pyproj.Transformer.from_crs(UTM, SP, always_xy=True)
                  .transform(E0, N0), 1.0, 0.0, SP)
    rng = np.random.default_rng(2)
    x, y = rng.uniform(0, 40, 200_000), rng.uniform(0, 20, 200_000)
    z = design(x, y) + rng.normal(0, 0.01, x.size)
    on_machine = (x > 25) & (x < 31) & (y > 5) & (y < 10)
    z[on_machine] += rng.uniform(1.5, 3.0, on_machine.sum())   # a dozer's top, seen from a drone
    e, n = frame.local_to_projected(x, y)                   # site -> Alabama East -> the cloud's UTM
    e, n = pyproj.Transformer.from_crs(SP, UTM, always_xy=True).transform(e, n)
    path = tmp_path / "scan.laz"
    write_laz(path, np.asarray(e), np.asarray(n), z, np.where(on_machine, 1, 2), UTM)

    truth = design(*np.meshgrid(g.xs, g.ys))
    raw, info = grid_cloud(str(path), g, frame, stat="mean")
    assert info["ground_filter"] == "none" and np.nanmax(raw - truth) > 0.5
    filt, info = grid_cloud(str(path), g, frame, stat="mean", pmf={})
    assert info["ground_filter"] == "pmf"
    assert np.isnan(filt).sum() <= 30                         # only the 6 m x 5 m under the machine is unseen
    assert np.nanmax(np.abs(filt - truth)) < 0.02             # reprojected exactly, machine points gone
    cls2, _ = grid_cloud(str(path), g, frame, stat="mean", ground_only=True)
    assert np.nanmax(np.abs(cls2 - truth)) < 0.02


def test_frame_crs_wgs84_and_reproject():
    """With a named coordinate system, pyproj does the conversions: to latitude/longitude and back, and between
    systems.
    """
    pytest.importorskip("pyproj")
    f = Frame(0.3048, 0.0, None, None, E0, N0, 1.0, 0.0, UTM)
    lat, lon = f.local_to_wgs84([0.0, 1000.0], [0.0, 2000.0])
    np.testing.assert_allclose(f.wgs84_to_local(lat, lon), ([0, 1000], [0, 2000]), atol=1e-6)
    assert abs(lat[0] - 34.4447) < 1e-3 and abs(lon[0] + 85.9115) < 1e-3     # 100 km east of 87° W
    e, n, z = f.reproject([E0], [N0], [10.0], None)
    assert (e[0], n[0], z[0]) == (E0, N0, 10.0)
    e, n, _ = f.reproject(*Frame(1, crs=SP).reproject([E0], [N0], [0.0], None), None)
    assert e[0] == E0


# -- GeoTIFF -------------------------------------------------------------------------------------
@pytest.mark.parametrize("rot", [0.0, 30.0])
def test_geotiff_export_reads_back_exactly(tmp_path, rot):
    """A surface exported as GeoTIFF reads back to the same heights, also on a rotated site and with heights in feet."""
    pytest.importorskip("rasterio")
    from loam.surfaces import geotiff_ground, write_geotiff
    _, g = site(tmp_path)
    frame = Frame(1.0, rot, None, None, E0, N0, 0.3048, 1.5, UTM if rot else None)
    z = design(*np.meshgrid(g.xs, g.ys))
    write_geotiff(str(tmp_path / "s.tif"), z, g, frame)
    back = geotiff_ground(str(tmp_path / "s.tif"), g, frame)
    inside = np.isfinite(back)
    assert inside.mean() > 0.85
    np.testing.assert_allclose(back[inside], z[inside], atol=1e-4)     # float32 at ~330 ft


def test_geotiff_dem_in_another_crs_and_as_design(tmp_path, capsys):
    """A terrain model in UTM at 0.25 m pixels is sampled correctly onto a state-plane site, and an OpenDroneMap
    project folder is accepted as the ground for `export`.
    """
    pytest.importorskip("rasterio")
    pyproj = pytest.importorskip("pyproj")
    import rasterio
    from rasterio.transform import Affine
    _, g = site(tmp_path)
    e0, n0 = pyproj.Transformer.from_crs(UTM, SP, always_xy=True).transform(E0, N0)
    frame = Frame(1.0, 0.0, None, None, e0, n0, 1.0, 0.0, SP)
    to_site = pyproj.Transformer.from_crs(UTM, SP, always_xy=True)      # a DTM in UTM, 0.25 m pixels
    ce, cn = np.meshgrid(E0 - 10 + 0.125 + np.arange(280) * 0.25, N0 + 30 - 0.125 - np.arange(240) * 0.25)
    se, sn = to_site.transform(ce, cn)
    dem = design(np.asarray(se) - e0, np.asarray(sn) - n0).astype("float32")
    with rasterio.open(tmp_path / "dtm.tif", "w", driver="GTiff", width=280, height=240, count=1,
                       dtype="float32", crs=UTM, transform=Affine(0.25, 0.0, E0 - 10, 0.0, -0.25, N0 + 30)) as d:
        d.write(dem, 1)
    from loam.surfaces import geotiff_ground
    z = geotiff_ground(str(tmp_path / "dtm.tif"), g, frame)
    np.testing.assert_allclose(z, design(*np.meshgrid(g.xs, g.ys)), atol=2e-3)

    odm = tmp_path / "odm" / "odm_dem"                       # an OpenDroneMap project folder
    odm.mkdir(parents=True)
    (tmp_path / "dtm.tif").rename(odm / "dtm.tif")
    (tmp_path / "t.loam").write_text(SRC.format(design="d.xml", fmt="landxml",
                                                boundary="rect (0, 0) (40, 20)", area="rect (0, 0) (20, 20)"))
    main(["export", str(tmp_path / "t.loam"), "--to", "geotiff", "-o", str(tmp_path / "next.tif"),
          "--ground", str(tmp_path / "odm"), "--crs", SP, "--world-origin", f"{e0},{n0}", "--json"])
    assert '"source": "geotiff"' in capsys.readouterr().out


def test_geotiff_design_surface(tmp_path):
    """A GeoTIFF can be the design surface of a site."""
    pytest.importorskip("rasterio")
    from loam.surfaces import write_geotiff
    _, g = site(tmp_path)
    write_geotiff(str(tmp_path / "fg.tif"), design(*np.meshgrid(g.xs, g.ys)), g, Frame(1.0))
    _, g2 = site(tmp_path, fmt="geotiff", design_file="fg.tif")
    d2 = g2.surface(g2.m.site["design"], allow_gaps=True)
    ok = np.isfinite(d2)
    assert ok.mean() > 0.85
    np.testing.assert_allclose(d2[ok], design(*np.meshgrid(g.xs, g.ys))[ok], atol=1e-4)


# -- IFC 4.3 -------------------------------------------------------------------------------------
def write_ifc(path, step=4.0):
    """Write a minimal IFC 4.3 file holding the design plane as a triangulated surface named FG."""
    pytest.importorskip("ifcopenshell")
    import ifcopenshell.api as api
    f = api.run("project.create_file", version="IFC4X3")
    proj = api.run("root.create_entity", f, ifc_class="IfcProject", name="t")
    api.run("unit.assign_unit", f, length={"is_metric": True, "raw": "METERS"})
    ctx = api.run("context.add_context", f, context_type="Model")
    body = api.run("context.add_context", f, context_type="Model", context_identifier="Body",
                   target_view="MODEL_VIEW", parent=ctx)
    s = api.run("root.create_entity", f, ifc_class="IfcSite", name="S")
    api.run("aggregate.assign_object", f, relating_object=proj, products=[s])
    el = api.run("root.create_entity", f, ifc_class="IfcGeographicElement", name="FG")
    api.run("spatial.assign_container", f, relating_structure=s, products=[el])
    api.run("geometry.edit_object_placement", f, product=el)
    xs, ys = np.arange(0, 41.0, step), np.arange(0, 21.0, step)
    gx, gy = np.meshgrid(xs, ys)
    pts = [(float(a), float(b), float(design(a, b))) for a, b in zip(gx.ravel(), gy.ravel(), strict=True)]
    nx, tris = len(xs), []
    for j in range(len(ys) - 1):
        for i in range(nx - 1):
            a = j * nx + i
            tris += [(a + 1, a + 2, a + nx + 2), (a + 1, a + nx + 2, a + nx + 1)]
    fs = f.createIfcTriangulatedFaceSet(f.createIfcCartesianPointList3D(pts), None, None, tris, None)
    api.run("geometry.assign_representation", f, product=el,
            representation=f.createIfcShapeRepresentation(body, "Body", "Tessellation", [fs]))
    f.write(str(path))


def test_ifc_design_surface(tmp_path):
    """An IFC 4.3 surface can be the design surface of a site; asking for a name the file does not have is an error."""
    write_ifc(tmp_path / "fg.ifc")
    _, g = site(tmp_path, fmt="ifc", design_file="fg.ifc")
    d = g.surface(g.m.site["design"])                       # a plane: exact on every cell
    np.testing.assert_allclose(d, design(*np.meshgrid(g.xs, g.ys)), atol=1e-9)
    from loam.surfaces import tin_from_ifc
    with pytest.raises(ValueError, match="no surface named"):
        tin_from_ifc(str(tmp_path / "fg.ifc"), "EG")


# -- GeoJSON areas (QGIS) ---------------------------------------------------------------------------
def test_geojson_area_files_and_export(tmp_path):
    """A zone's area can come from a GeoJSON polygon (drawn in QGIS, for example), and the site's zones export as
    GeoJSON in world coordinates.
    """
    poly = {"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {}, "geometry": {
        "type": "Polygon", "coordinates": [[[0, 0], [20, 0], [20, 20], [0, 20], [0, 0]]]}}]}
    (tmp_path / "w.geojson").write_text(json.dumps(poly))
    m, g = site(tmp_path, area='file "w.geojson"')
    assert m.zones["W"]["area"]["polygon"] == [(0, 0), (20, 0), (20, 20), (0, 20)]
    from loam.surfaces import polygon_from_geojson, site_geojson
    fc = site_geojson(m, g, Frame(1.0, 0.0, None, None, E0, N0, 1.0, 0.0, UTM))
    assert fc["crs"]["properties"]["name"] == UTM
    assert {f["properties"]["kind"] for f in fc["features"]} == {"boundary", "zone"}
    assert fc["features"][1]["geometry"]["coordinates"][0][2] == [E0 + 20, N0 + 20]
    (tmp_path / "bad.geojson").write_text(json.dumps({"type": "Point", "coordinates": [0, 0]}))
    with pytest.raises(ValueError, match="no Polygon"):
        polygon_from_geojson(str(tmp_path / "bad.geojson"))


def test_cli_exports(tmp_path):
    """`loam export` writes GeoJSON and GeoTIFF from the command line."""
    pytest.importorskip("rasterio")
    site(tmp_path)
    (tmp_path / "t.loam").write_text(SRC.format(design="d.xml", fmt="landxml",
                                                boundary="rect (0, 0) (40, 20)", area="rect (0, 0) (20, 20)"))
    prog = str(tmp_path / "t.loam")
    main(["export", prog, "--to", "geojson", "-o", str(tmp_path / "s.geojson"), "--json"])
    assert len(json.loads((tmp_path / "s.geojson").read_text())["features"]) == 2
    main(["export", prog, "--to", "geotiff", "--surface", "design", "-o", str(tmp_path / "s.tif"), "--json"])
    from loam.surfaces import raster_at
    z, crs = raster_at(str(tmp_path / "s.tif"), np.array([10.5]), np.array([5.5]))
    assert crs is None and abs(z[0] - design(10.5, 5.5)) < 1e-4
