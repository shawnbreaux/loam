"""The `loam` command line.

    Understand a program     check, explain, compile
    Try it on a model        sim (the reference simulator)
    Start from a design      plan (LandXML grading model -> Loam program)
    Work with field data     export, terrain, score, nmea
    Run a site               run (controller), monitor (dashboard), estop, resume (after a stop)
    Connect site sensors     sense (people from cameras, rain from a gauge), terrain --watch (scans as they arrive)
    Simulate a site          simulate-site, soil-calibrate
    Connect real machines    mavlink, vda5050, opera

Run `loam <command> --help` for a command's options. Heavy or optional modules are imported inside the
commands that need them, so `loam check` starts fast and works with nothing but numpy installed.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import TextIO

from .checker import check, has_errors
from .compiler import explain, manifest
from .lexer import LoamSyntaxError
from .parser import parse_file


def load(path: str):
    """Parse and check a `.loam` file. Diagnostics go to stderr; the process exits with status 2 for a syntax or file
    error and 1 for a program that does not check.
    """
    try:
        prog = parse_file(path)
    except LoamSyntaxError as e:
        print(f"{e}", file=sys.stderr)
        sys.exit(2)
    except OSError as e:
        print(f"loam: {e}", file=sys.stderr)
        sys.exit(2)
    model, diags = check(prog)
    for d in diags:
        print(d.format(path), file=sys.stderr)
    if has_errors(diags):
        sys.exit(1)
    return model


# commands that run until stopped: a service manager's SIGTERM must still run their cleanup (hold the
# vehicle, cancel its goal, report it offline), which Python's default SIGTERM action skips
LONG_RUNNING = {"run", "simulate-site", "monitor", "_role", "mavlink", "vda5050", "opera", "nmea", "sense", "terrain"}


def _stop_cleanly_on_sigterm() -> None:
    """Turn SIGTERM into a normal exit, so `finally` blocks run. Only possible on the main thread; elsewhere (tests
    calling `main` from a thread) it does nothing.
    """
    import signal
    import threading

    def stop(signum, frame):
        """Exit with the conventional status for death by signal (128 + signal number)."""
        raise SystemExit(128 + signum)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, stop)


def main(argv=None) -> None:
    """The `loam` command: parse the arguments, load the program and run the chosen sub-command. `argv` defaults to
    the process arguments; tests pass a list.
    """
    ap = argparse.ArgumentParser(prog="loam", description="Loam: an open language for programmable earthmoving")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, hlp in (("check", "parse and check a program"), ("explain", "describe a program in plain English"),
                      ("compile", "write the execution manifest as JSON"), ("sim", "run the closed-loop simulator")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("file")
        if name == "compile":
            p.add_argument("-o", "--out", help="output file (default: stdout)")
        if name == "sim":
            p.add_argument("--open-loop", action="store_true", help="one survey, no feedback (for comparison)")
            p.add_argument("--no-events", action="store_true", help="no scripted people, outage or rain")
            p.add_argument("--seed", type=int, default=7)
            p.add_argument("--plot", help="write before/after PNG")
            p.add_argument("--wire", help="write Loam Wire messages as JSON lines")
            p.add_argument("--json", action="store_true", help="print the report as JSON")
            p.add_argument("--quiet", action="store_true", help="no event log")
    add_field_commands(sub)
    a = ap.parse_args(argv)
    if a.cmd == "plan":
        cmd_plan(a)
        return
    model = load(a.file)
    if a.cmd in LONG_RUNNING:
        _stop_cleanly_on_sigterm()
    if a.cmd in FIELD_COMMANDS:
        try:
            FIELD_COMMANDS[a.cmd](model, a)
        except (ValueError, OSError) as e:
            print(f"loam {a.cmd}: {e}", file=sys.stderr)
            sys.exit(1)
        return
    if a.cmd == "check":
        print(f"{a.file}: ok ({len(model.tasks)} tasks, {len(model.machines)} machines, {len(model.rules)} rules)")
    elif a.cmd == "explain":
        print(explain(model))
    elif a.cmd == "compile":
        text = json.dumps(manifest(model), indent=2)
        if a.out:
            with open(a.out, "w") as f:
                f.write(text + "\n")
            print(f"wrote {a.out}")
        else:
            print(text)
    elif a.cmd == "sim":
        from .sim import Events, Sim
        sim = Sim(model, base_dir=os.path.dirname(os.path.abspath(a.file)), seed=a.seed, open_loop=a.open_loop,
                  events=Events.none() if a.no_events else None,
                  log=(lambda *_: None) if a.quiet or a.json else print)
        rep = sim.run(wire_path=a.wire)
        if a.plot:
            sim.plot(a.plot, f"{model.site['name']} ({rep['mode']})")
        if a.json:
            print(json.dumps(rep, indent=2))
        else:
            print_report(rep)


# -- field adapters ------------------------------------------------------------------
def _pair(text: str) -> tuple[float, float]:
    """Two comma-separated numbers, e.g. "34.4459,-85.7197"."""
    a, b = (float(v) for v in text.split(","))
    return a, b


def _frame_args(p: argparse.ArgumentParser) -> None:
    """The options, shared by every command that reads or writes real-world coordinates, that say where the site is on
    the earth.
    """
    g = p.add_argument_group("placing the site on the earth")
    g.add_argument("--origin", type=_pair, metavar="LAT,LON", help="WGS84 position of site point (0, 0)")
    g.add_argument("--world-origin", type=_pair, metavar="E,N", default=(0.0, 0.0),
                   help="projected easting,northing of site point (0, 0) (for clouds, surveys, LandXML out)")
    g.add_argument("--world-units", default="m", choices=["m", "ft", "usft"],
                   help="units of projected coordinates and heights in files (default m)")
    g.add_argument("--rotation", type=float, default=0.0, metavar="DEG",
                   help="bearing of the site +y axis, degrees clockwise from north")
    g.add_argument("--dz", type=float, default=0.0, help="site units added to incoming heights")
    g.add_argument("--crs", help="CRS of the projected coordinates, e.g. EPSG:6355 (needs pyproj)")
    g.add_argument("--source-crs", help="CRS of the input file when it differs from --crs "
                   "(GeoTIFF and LAZ declare their own)")


def _frame(grid, a):
    """The coordinate frame described by those options (see geo.Frame)."""
    from .geo import Frame
    from .units import length_unit_si
    lat0, lon0 = a.origin if a.origin else (None, None)
    return Frame(grid.u, a.rotation, lat0, lon0, a.world_origin[0], a.world_origin[1],
                 length_unit_si(a.world_units), a.dz, getattr(a, "crs", None))


def _grid(model, a):
    """The site grid of a program; relative file names in it are looked up next to the `.loam` file."""
    from .grid import SiteGrid
    return SiteGrid(model, os.path.dirname(os.path.abspath(a.file)))


def _is_survey_csv(path: str) -> bool:
    """A .csv is a survey (Emlid and the like) if it has an Elevation column or is headerless PNEZD/PENZD."""
    with open(path, encoding="utf-8-sig") as f:
        first = f.readline().strip().lower()
    cols = [c.strip() for c in first.split(",")]
    if any(ch.isalpha() for ch in "".join(cols[1:])):
        return "elevation" in cols
    return len(cols) >= 4


def load_ground(path: str, grid, frame, a):
    """Measured ground on the site grid from 'existing', a survey CSV, a point cloud, a GeoTIFF DEM or an
    OpenDroneMap project folder."""
    import os

    import numpy as np

    from .pointcloud import fill_gaps, grid_cloud
    from .survey import idw, read_points
    if path == "existing":
        ex = grid.m.site.get("existing") or {}
        if "file" not in ex:
            raise ValueError("the site declares no existing surface")
        return grid.surface(ex, allow_gaps=True), {"source": "existing"}
    if path.lower().endswith(".geojson") or (path.lower().endswith(".csv") and _is_survey_csv(path)):
        s = read_points(path, frame, a.layout)
        z = idw(grid, s.x, s.y, s.z, a.radius / grid.u)
        return z, {"source": "survey", "points": len(s.z),
                   "cells_covered_pct": round(100 * float(np.isfinite(z).mean()), 2)}
    src = getattr(a, "source_crs", None)
    pmf: dict | None = {} if getattr(a, "ground_filter", False) else None
    if os.path.isdir(path):                              # OpenDroneMap project: DTM, else filtered cloud
        dtm = os.path.join(path, "odm_dem", "dtm.tif")
        laz = os.path.join(path, "odm_georeferencing", "odm_georeferenced_model.laz")
        if os.path.exists(dtm):
            path = dtm
        elif os.path.exists(laz):
            path, pmf = laz, pmf if pmf is not None else {}
        else:
            raise ValueError(f"{path}: no odm_dem/dtm.tif or odm_georeferencing/odm_georeferenced_model.laz")
    if path.lower().endswith((".tif", ".tiff")):
        from .surfaces import geotiff_ground
        z = fill_gaps(geotiff_ground(path, grid, frame, src), a.fill)
        return z, {"source": "geotiff", "file": os.path.basename(path),
                   "cells_covered_pct": round(100 * float(np.isfinite(z).mean()), 2)}
    z, info = grid_cloud(path, grid, frame, a.stat, a.ground_only, pmf=pmf, src_crs=src)
    return fill_gaps(z, a.fill), {"source": "point cloud", **info}


def _ground_args(p: argparse.ArgumentParser) -> None:
    """The options, shared by every command that reads measured ground, that say where it comes from and how to grid
    it.
    """
    p.add_argument("--stat", choices=["mean", "min"], default="mean", help="per-cell statistic for clouds")
    p.add_argument("--ground-only", action="store_true", help="LAS/LAZ: keep only class 2 (ground) points")
    p.add_argument("--ground-filter", action="store_true",
                   help="clouds: drop machines, people and brush with a morphological ground filter")
    p.add_argument("--fill", type=int, default=2, metavar="CELLS", help="fill cloud holes up to this deep")
    p.add_argument("--radius", type=float, default=3.0, metavar="M",
                   help="survey CSV: interpolation radius in metres (default 3)")
    p.add_argument("--layout", default="PNEZD", choices=["PNEZD", "PENZD"], help="survey CSV without a header")


def add_field_commands(sub) -> None:
    """Declare every sub-command beyond the four basic ones, with its options."""
    p = sub.add_parser("plan", help="write a Loam program from a grading model (LandXML existing + design)")
    p.add_argument("file", help="LandXML file with the design (and usually the existing) surface")
    p.add_argument("-o", "--out", required=True, help="the .loam program to write")
    p.add_argument("--design", help="design surface name (default: guessed, e.g. FG, Proposed)")
    p.add_argument("--existing", help="existing surface name (default: guessed, e.g. EG, Existing)")
    p.add_argument("--existing-file", help="existing ground in a separate LandXML file")
    p.add_argument("--fleet", help="Loam machine and probe declarations (default: 2 dozers, roller, mast lidar)")
    p.add_argument("--tolerance", help="finish tolerance, e.g. '0.1 ft' or '3 cm'")
    p.add_argument("--max-cells", type=int, default=40_000, help="grid size limit (default 40000 cells)")
    p.add_argument("--name", help="site name (default: from the file name)")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("export", help="write a surface or field files for grade-control and guidance tools")
    p.add_argument("file")
    p.add_argument("--to", required=True, choices=["landxml", "agd", "agopengps", "geotiff", "geojson"],
                   help="landxml: Cat Grade 3D and most machine control; agd: OpenGrade3D; agopengps: field "
                        "folder; geotiff: surface raster (QGIS, GDAL); geojson: boundary, zones, stockpiles")
    p.add_argument("-o", "--out", required=True, help="output file (folder for agopengps)")
    p.add_argument("--surface", choices=["design", "next"], default="next",
                   help="design, or the next-pass target computed from --ground (default)")
    p.add_argument("--ground", default="existing",
                   help="measured ground: 'existing', a survey .csv or .geojson, or a .las/.pcd/.xyz cloud")
    p.add_argument("--benchmark", type=_pair, default=(0.0, 0.0), metavar="X,Y",
                   help="agd: site position of the master benchmark")
    p.add_argument("--json", action="store_true")
    _frame_args(p)
    _ground_args(p)
    p = sub.add_parser("terrain", help="grid a lidar point cloud onto the site and emit a Wire terrain message")
    p.add_argument("file")
    p.add_argument("cloud")
    p.add_argument("--probe", default=None, help="probe name for the message (default: the program's first)")
    p.add_argument("--wire", help="append the terrain message to this JSON-lines file")
    p.add_argument("--landxml", help="also write the gridded scan as a LandXML surface (site coordinates)")
    p.add_argument("--broker", help="publish the terrain message to this MQTT broker, for a running controller")
    p.add_argument("--watch", action="store_true",
                   help="CLOUD is a folder: publish every scan that appears in it, until stopped (needs --broker)")
    p.add_argument("--settle", type=float, default=2.0, metavar="S",
                   help="with --watch: a file counts as complete once it has not changed for S seconds")
    p.add_argument("--max-seconds", type=float, help=argparse.SUPPRESS)       # for tests: stop --watch after this
    _frame_args(p)
    _ground_args(p)
    p = sub.add_parser("score", help="score a survey against the design inside a zone")
    p.add_argument("file")
    p.add_argument("survey")
    p.add_argument("--zone", required=True)
    p.add_argument("--layout", default="PNEZD", choices=["PNEZD", "PENZD"])
    p.add_argument("--json", action="store_true")
    _frame_args(p)
    p = sub.add_parser("run", help="run the site controller over MQTT (real machines or a simulated site)")
    p.add_argument("file")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--record", help="write every message the controller sends as JSON lines")
    p.add_argument("--real-time", action="store_true", help="no simulated clock: run at wall-clock speed")
    p.add_argument("--max-minutes", type=float, default=48 * 60)
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("simulate-site", help="start a simulated world, probe and machines as separate processes")
    p.add_argument("file")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--speed", type=float, default=600.0, help="site minutes per wall minute (default 600)")
    p.add_argument("--faults", help="JSON fault script (default: person, outage, rain, GNSS, link loss, e-stop)")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--physics", action="store_true", help="drive machines as PyBullet bodies (try --speed 60)")
    p.add_argument("--soil-table", help="roller compaction curves from `loam soil-calibrate` (Project Chrono)")
    p = sub.add_parser("mavlink", help="drive an ArduPilot Rover (or any MAVLink ground vehicle) as a Loam machine")
    p.add_argument("file")
    p.add_argument("--machine", required=True, help="the Loam machine this vehicle is")
    p.add_argument("--vehicle", required=True, metavar="URL", help="pymavlink connection, e.g. udpin:0.0.0.0:14550, "
                   "tcp:127.0.0.1:5760, /dev/ttyACM0")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--min-fix", type=int, default=6, choices=range(0, 7), metavar="0-6",
                   help="lowest GPS fix type to work on (6 RTK fixed, 5 float, 3 3D; default 6)")
    p.add_argument("--sim-world", action="store_true", help="part of `loam simulate-site`: its clock and world")
    _frame_args(p)
    p.add_argument("--no-vehicle-setup", action="store_true",
                   help="leave the vehicle's own parameters and fences alone (default: upload the site boundary and "
                        "exclusion zones as fences and switch on its ground-station failsafe)")
    p = sub.add_parser("vda5050", help="drive a VDA 5050 3.0 mobile robot as a Loam machine")
    p.add_argument("file")
    p.add_argument("--machine", required=True, help="the Loam machine this robot is")
    p.add_argument("--manufacturer", required=True)
    p.add_argument("--serial", required=True, help="the robot's serialNumber")
    p.add_argument("--interface", default="uagv", help="first MQTT topic level (default uagv)")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--sim-world", action="store_true", help="part of `loam simulate-site`: its clock and world")
    p = sub.add_parser("resume", help="restart machines after an e-stop or a stop: the deliberate human step")
    p.add_argument("file")
    p.add_argument("--machine", help="one machine (default: the whole site)")
    p.add_argument("--reason", default="resumed by operator")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("estop", help="press (or release) the site e-stop from the supervisor's computer")
    p.add_argument("file")
    p.add_argument("--release", action="store_true", help="release it; machines stay stopped until `loam resume`")
    p.add_argument("--reason", help="why (default: pressed, or released, by operator)")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("sense", help="site sensors as Wire messages: people from Frigate cameras, rain from rtl_433")
    p.add_argument("file")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--areas", help='people: JSON file placing Frigate\'s cameras and zones on the site, '
                                   '{"cameras": {"yard": {"x": 80, "y": 50, "radius": 60, "zones": {...}}}}')
    p.add_argument("--frigate-prefix", default="frigate", help="Frigate's MQTT topic prefix")
    p.add_argument("--rain", nargs="?", const="rtl_433/+/events", metavar="TOPIC",
                   help="rain: rtl_433's events topic (default rtl_433/+/events)")
    p.add_argument("--gauge", metavar="MODEL[:ID]", help="the rain gauge to use (default: the first one heard)")
    p.add_argument("--name", default="sense", help="this sensor's name in the messages")
    p.add_argument("--max-seconds", type=float, help=argparse.SUPPRESS)       # for tests
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("opera", help="drive an OPERA (ROS 2) machine as a Loam machine: IC120 carrier, ZX200")
    p.add_argument("file")
    p.add_argument("--machine", required=True, help="the Loam machine this is")
    p.add_argument("--namespace", required=True, help="the machine's ROS namespace, e.g. ic120 or zx200")
    p.add_argument("--kind", choices=["carrier", "excavator"], default="carrier")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--dump", action="store_true", help="carrier: raise the vessel at the deposit (com3_msgs)")
    p.add_argument("--urdf", help="excavator: URDF file (default: /<ns>/robot_description)")
    p.add_argument("--map-offset", default="0,0,0", metavar="X,Y,Z", help="site origin in the map frame (m)")
    p.add_argument("--map-yaw", type=float, default=0.0, help="site +x axis in the map frame (deg)")
    p.add_argument("--sim-world", action="store_true", help="part of `loam simulate-site`: its clock and world")
    p = sub.add_parser("soil-calibrate", help="roller compaction per pass from Project Chrono's SCM soil model")
    p.add_argument("file")
    p.add_argument("--machine", required=True, help="the roller (its drum width comes from the program)")
    p.add_argument("--mass", required=True, help="operating mass, e.g. 10t or 22000lb")
    p.add_argument("--diameter", required=True, help="drum diameter, e.g. 1.5m or 59in")
    p.add_argument("--passes", type=int, default=8)
    p.add_argument("--soil", help="JSON of SCM soil parameters (default: an illustrative sandy loam)")
    p.add_argument("-o", "--out", required=True)
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("monitor", help="live web dashboard for a site, or a replay page from a recording")
    p.add_argument("file")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--host", default="127.0.0.1",
                   help="address to listen on; 0.0.0.0 lets other machines (a Prometheus server) reach it")
    p.add_argument("--record", help="record dashboard frames as JSON lines")
    p.add_argument("--mcap", help="record every Wire message to this MCAP file (for Lichtblick, PlotJuggler)")
    p.add_argument("--replay", help="a recording to turn into a self-contained page (with --html)")
    p.add_argument("--html", help="where to write the replay page")
    p.add_argument("--split", action="store_true", help="write a folder: index.html + run.json")
    p = sub.add_parser("_role", help=argparse.SUPPRESS)
    p.add_argument("file")
    p.add_argument("--role", required=True, choices=["world", "probe", "machine"])
    p.add_argument("--soil-table")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--speed", type=float, default=600.0)
    p.add_argument("--faults")
    p.add_argument("--name")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--physics", action="store_true")
    p = sub.add_parser("nmea", help="turn a GGA stream into Wire state messages")
    p.add_argument("file")
    p.add_argument("input", nargs="?", default="-",
                   help="NMEA file, - for stdin, or tcp://host:port (RTKLIB str2str/rtkrcv, RTKBase, a receiver)")
    p.add_argument("--machine", required=True)
    p.add_argument("-o", "--out", help="write JSON lines here (default stdout)")
    p.add_argument("--broker", help="publish the states to this MQTT broker instead of writing them")
    _frame_args(p)


def cmd_plan(a) -> None:
    """`loam plan`: write a Loam program from a grading model, then check what was written."""
    from .plan import plan
    try:
        r = plan(a.file, a.out, a.design, a.existing, a.existing_file, a.fleet, a.tolerance, a.max_cells, a.name)
    except (ValueError, OSError) as e:
        print(f"loam plan: {e}", file=sys.stderr)
        sys.exit(1)
    load(r["out"])          # parses and checks; exits with the diagnostics if the program is wrong
    r["check"] = "ok"
    _show(r, a.json)


def cmd_export(model, a) -> None:
    """`loam export`: write the design, or the next-pass target computed from measured ground, in a format field tools
    load (LandXML, OpenGrade3D, AgOpenGPS, GeoTIFF, GeoJSON).
    """
    from . import export
    from .target import next_target
    grid = _grid(model, a)
    frame = _frame(grid, a)
    design = grid.surface(model.site["design"], allow_gaps=True)
    out: dict = {"to": a.to, "out": a.out}
    if a.to == "agopengps":
        out.update(export.agopengps(a.out, grid, frame))
    elif a.to == "geojson":
        import json as _json

        from .surfaces import site_geojson
        fc = site_geojson(model, grid, frame)
        with open(a.out, "w") as fh:
            _json.dump(fc, fh)
        out["features"] = len(fc["features"])
    else:
        ground = design
        if a.surface != "design" or a.to == "agd":
            ground, out["ground"] = load_ground(a.ground, grid, frame, a)
        if a.surface == "design":
            surf = design
        else:
            surf, ti = next_target(grid, design, ground)
            out["target"] = ti
        if a.to == "landxml":
            world = frame if (a.world_origin != (0.0, 0.0) or a.rotation or a.dz) else None
            out["triangles"] = export.landxml(a.out, grid, surf, world,
                                              "FG" if a.surface == "design" else "LOAM_NEXT", a.world_units)
            out["coordinates"] = "projected" if world else "site"
        elif a.to == "geotiff":
            from .surfaces import write_geotiff
            write_geotiff(a.out, surf, grid, frame)
            out["cells"] = int(surf.size)
        else:
            out["grid_points"] = export.agd(a.out, grid, ground, surf, frame, a.benchmark)
    _show(out, a.json)


SCAN_TYPES = (".las", ".laz", ".pcd", ".xyz", ".csv", ".geojson", ".tif", ".tiff")


def cmd_terrain(model, a) -> None:
    """`loam terrain`: grid measured ground onto the site and hand it on as a Loam Wire terrain message (to a file
    and/or a broker) and/or a LandXML surface. With `--watch`, do so for every scan that appears in a folder.
    """
    import json as _json

    from . import wire
    from .landxml import write_grid_tin
    grid = _grid(model, a)
    frame = _frame(grid, a)
    site = model.site["name"]
    probe = a.probe or next(iter(model.probes), "probe")
    topic = wire.topic(site, None, "terrain")

    def message(path: str, seq: int):
        """Grid one scan. The message is stamped with the file's time: when the scan was taken, not when it was
        processed. The controller needs that to know which passes the scan already shows.
        """
        z, info = load_ground(path, grid, frame, a)
        return z, info, wire.terrain(site, probe, seq, (grid.x0, grid.y0), grid.cell, grid.shape,
                                     data=wire.encode_heights(z), ts=os.path.getmtime(path))

    if a.watch:
        if not a.broker or not os.path.isdir(a.cloud):
            raise ValueError("--watch needs --broker and a folder to watch")
        _watch_scans(a, topic, message)
        return
    z, info, msg = message(a.cloud, 1)
    if a.wire:
        with open(a.wire, "a", encoding="utf-8") as f:
            f.write(_json.dumps({"topic": topic, "payload": msg}) + "\n")
        info["wire"] = a.wire
    if a.landxml:
        write_grid_tin(a.landxml, "SCAN", grid.xs, grid.ys, z, grid.unit)
        info["landxml"] = a.landxml
    if a.broker:
        from .runtime.bus import Bus
        bus = Bus(a.broker, f"loam-terrain-{os.getpid()}").start([])
        try:
            bus.pub(topic, msg, qos=1, wait=10.0)
        finally:
            bus.stop()
        info["published"] = topic
    _show(info, False)


def _watch_scans(a, topic: str, message) -> None:
    """Publish each scan file that appears in the folder `a.cloud`, oldest first, until stopped.

    This is how a mast lidar or a drone-mapping computer feeds a running site: it writes scan files into a
    folder, by whatever means it has. Of the files already there at the start only the newest is published: it
    is the current ground. A file is taken once it has stopped changing for `a.settle` seconds, since the writer
    may still be writing it. A file that cannot be read is reported and skipped, never retried.
    """
    from .runtime.bus import Bus

    def scans() -> dict[str, float]:
        """The scan files in the folder now, with their modification times."""
        out = {}
        for e in os.scandir(a.cloud):
            if e.is_file() and e.name.lower().endswith(SCAN_TYPES):
                out[e.path] = e.stat().st_mtime
        return out

    done = scans()
    newest = max(done, key=lambda path: done[path], default=None)
    if newest is not None:
        del done[newest]
    bus = Bus(a.broker, f"loam-terrain-{os.getpid()}").start([])
    seq = int(time.time())
    end = time.time() + a.max_seconds if a.max_seconds else float("inf")
    try:
        while time.time() < end:
            for path, mtime in sorted(scans().items(), key=lambda kv: kv[1]):
                if done.get(path) == mtime or time.time() - mtime < a.settle:
                    continue
                done[path] = mtime
                seq += 1
                try:
                    _, info, msg = message(path, seq)
                except (ValueError, OSError) as e:
                    print(f"loam terrain: skipped {path}: {e}", file=sys.stderr, flush=True)
                    continue
                bus.pub(topic, msg, qos=1, wait=10.0)
                print(f"published {os.path.basename(path)} ({info.get('source', 'scan')})", flush=True)
            for path in set(done) - set(scans()):               # forget files that were removed
                del done[path]
            time.sleep(0.5)
    finally:
        bus.stop()


def cmd_score(model, a) -> None:
    """`loam score`: how much of a zone a survey shows within tolerance of the design. This is the number a field test
    is judged by.
    """
    from .survey import read_points, score
    grid = _grid(model, a)
    frame = _frame(grid, a)
    z = model.zones.get(a.zone)
    if z is None or "polygon" not in (z.get("area") or {}):
        raise ValueError(f"no polygon zone named {a.zone!r}")
    tol = z["tolerance"].to(grid.unit) if z.get("tolerance") else 0.1 / (grid.u / 0.3048)
    design = grid.surface(model.site["design"], allow_gaps=True)
    s = read_points(a.survey, frame, a.layout)
    r = score(grid, design, s, z["area"]["polygon"], tol)
    r.update({"zone": a.zone, "tolerance": tol, "units": grid.unit})
    _show(r, a.json)


def cmd_nmea(model, a) -> None:
    """`loam nmea`: turn a GNSS receiver's GGA sentences (file, stdin or TCP) into machine state messages, written as
    JSON lines or published to MQTT.
    """
    import json as _json

    from . import wire
    from .survey import nmea_states
    grid = _grid(model, a)
    frame = _frame(grid, a)
    src: TextIO
    if a.input.startswith("tcp://"):
        import socket
        host, _, port = a.input[6:].rpartition(":")
        conn = socket.create_connection((host or "localhost", int(port)), timeout=30)
        src = conn.makefile("r", encoding="ascii", errors="replace")
    else:
        src = sys.stdin if a.input == "-" else open(a.input, encoding="ascii", errors="replace")  # noqa: SIM115
    bus = None
    if a.broker:
        from .runtime.bus import Bus
        bus = Bus(a.broker, f"loam-nmea-{a.machine}-{os.getpid()}").start([])
    dst = open(a.out, "w", encoding="utf-8") if a.out else sys.stdout  # noqa: SIM115
    try:
        topic = wire.topic(model.site["name"], a.machine, "state")
        for msg in nmea_states(src, frame, model.site["name"], a.machine):
            if bus is not None:
                bus.pub(topic, msg)
            else:
                dst.write(_json.dumps({"topic": topic, "payload": msg}) + "\n")
                dst.flush()
    finally:
        if src is not sys.stdin:
            src.close()
        if dst is not sys.stdout:
            dst.close()
        if bus is not None:
            bus.stop()


def _show(d: dict, as_json: bool) -> None:
    """Print a result: as JSON, or as `key: value` lines with one level of nesting."""
    if as_json:
        print(json.dumps(d, indent=2))
        return
    for k, v in d.items():
        if isinstance(v, dict):
            print(f"{k}:")
            for k2, v2 in v.items():
                print(f"  {k2}: {v2}")
        else:
            print(f"{k}: {v}")


def cmd_run(model, a) -> None:
    """`loam run`: run the site controller against the machines on an MQTT broker until every task is done."""
    from .runtime.controller import run_controller
    r = run_controller(model, os.path.dirname(os.path.abspath(a.file)), a.broker, a.record, a.max_minutes,
                       wait_for_clock=not a.real_time)
    r.pop("log", None)
    if a.json:
        print(json.dumps(r, indent=2))


def cmd_simulate_site(model, a) -> None:
    """`loam simulate-site`: start a simulated world, probe and machines as separate processes and print the true
    result when the job ends.
    """
    from .runtime.site import launch
    r = launch(a.file, a.broker, a.speed, a.faults, model, quiet=a.quiet, physics=a.physics,
               soil_table=a.soil_table)
    print(json.dumps(r, indent=2))


def cmd_mavlink(model, a) -> None:
    """`loam mavlink`: make an ArduPilot Rover (or other MAVLink ground vehicle) one of the program's machines."""
    from .runtime.mavlink import run_mavlink
    run_mavlink(model, a.machine, a.broker, a.vehicle, _frame(_grid(model, a), a), a.min_fix, a.sim_world,
                setup=not a.no_vehicle_setup)


def cmd_vda5050(model, a) -> None:
    """`loam vda5050`: make a VDA 5050 mobile robot one of the program's machines."""
    from .runtime.vda5050 import run_vda5050
    run_vda5050(model, a.machine, a.broker, a.manufacturer, a.serial, a.interface, a.sim_world)


MASS_UNITS = {"kg": 1.0, "t": 1000.0, "lb": 0.45359237, "ton": 907.18474}     # ton = US short ton


def _si(text: str, units: dict[str, float]) -> float:
    """'10t', '22000 lb', '1.5m' -> SI, for CLI options."""
    import re
    m = re.fullmatch(r"\s*([0-9]*\.?[0-9]+)\s*([a-z]+)\s*", text)
    if not m or m.group(2) not in units:
        raise ValueError(f"{text!r}: expected a number and one of {', '.join(sorted(units))}")
    return float(m.group(1)) * units[m.group(2)]


def cmd_soil_calibrate(model, a) -> None:
    """`loam soil-calibrate`: compute a roller's settlement per pass with Project Chrono and save it for `simulate-
    site --soil-table`. The output file holds one table per roller; an existing file is added to.
    """
    import json as _json

    from .runtime.chrono_soil import SoilParams, calibrate
    from .units import UNITS, Quantity
    m = model.machines.get(a.machine)
    if m is None or m.get("class") != "roller":
        raise ValueError(f"{a.machine!r} is not a roller in this program")
    if "drum" not in m:
        raise ValueError(f"roller {a.machine!r} needs a drum width (drum 84 in)")
    width = m["drum"].si if isinstance(m["drum"], Quantity) else float(m["drum"])
    soil = SoilParams()
    if a.soil:
        with open(a.soil, encoding="utf-8") as f:
            soil = SoilParams(**_json.load(f))
    table = calibrate(_si(a.mass, MASS_UNITS), width, _si(a.diameter, {u: f for u, (d, f) in UNITS.items()
                                                                      if d == "length"}), soil, a.passes)
    tables = {}
    try:
        with open(a.out, encoding="utf-8") as f:
            tables = _json.load(f)
    except FileNotFoundError:
        pass
    if "settlement_m" in tables:
        tables = {}
    tables[a.machine] = table
    with open(a.out, "w", encoding="utf-8") as f:
        _json.dump(tables, f, indent=1)
    _show({"machine": a.machine, **{k: table[k] for k in ("settlement_m", "first_pass_sinkage_m",
                                                         "bekker_wong_sinkage_m")}}, a.json)


def cmd_resume(model, a) -> None:
    """`loam resume`: the deliberate restart after an e-stop or stop, for the whole site or one machine. Releasing an
    e-stop restarts nothing by itself.
    """
    from . import wire
    from .runtime.bus import Bus
    site = model.site["name"]
    if a.machine and a.machine not in model.machines:
        raise ValueError(f"no machine named {a.machine!r}")
    bus = Bus(a.broker, f"loam-resume-{os.getpid()}").start([])
    try:
        seq = int(time.time())
        if a.machine:
            bus.pub(wire.topic(site, a.machine, "instantAction"),
                    wire.instant_action(site, a.machine, seq, "resume", a.reason, ts=time.time()), qos=1)
        else:
            bus.pub(wire.topic(site, None, "safety"), wire.safety(site, seq, "resume", a.reason, ts=time.time()),
                    qos=1)
        time.sleep(0.3)                                  # let the QoS 1 publish complete
    finally:
        bus.stop()
    _show({"resumed": a.machine or "site", "reason": a.reason}, a.json)


def cmd_estop(model, a) -> None:
    """`loam estop`: press or release the site e-stop over the broker.

    Every machine stops at once on a press, without waiting for the controller. A release restarts nothing: the
    machines stay stopped until `loam resume`. The message is retained, so a machine that connects during an
    e-stop learns of it. This is a software stop over a radio link; it does not replace the hard-wired stop
    button on each machine.
    """
    from . import wire
    from .runtime.bus import Bus
    site = model.site["name"]
    reason = a.reason or ("released by operator" if a.release else "pressed by operator")
    bus = Bus(a.broker, f"loam-estop-{os.getpid()}").start([])
    try:
        bus.pub(wire.topic(site, None, "estop"),
                wire.estop(site, int(time.time()), not a.release, reason, ts=time.time()), qos=1, retain=True,
                wait=5.0)
    finally:
        bus.stop()
    _show({"estop": "released" if a.release else "pressed", "reason": reason}, a.json)


def cmd_sense(model, a) -> None:
    """`loam sense`: republish people seen by Frigate's cameras and rain measured by an rtl_433 gauge as Loam Wire
    detection and weather messages, until stopped.
    """
    from .sensors import load_areas, run_sense
    sent = run_sense(model, a.broker, load_areas(a.areas) if a.areas else None, a.frigate_prefix, a.rain, a.gauge,
                     a.name, a.max_seconds)
    _show({"sent": sent}, a.json)


def cmd_opera(model, a) -> None:
    """`loam opera`: make an OPERA (ROS 2) machine one of the program's machines."""
    from .runtime.opera import run_opera
    off = tuple(float(v) for v in a.map_offset.split(","))
    if len(off) != 3:
        raise ValueError("--map-offset takes X,Y,Z")
    urdf = None
    if a.urdf:
        with open(a.urdf, encoding="utf-8") as f:
            urdf = f.read()
    run_opera(model, a.machine, a.broker, a.namespace, a.kind, off, a.map_yaw,
              os.path.dirname(os.path.abspath(a.file)), a.dump, urdf, a.sim_world)


def cmd_monitor(model, a) -> None:
    """`loam monitor`: serve the live dashboard, or build a replay page from a recording."""
    from . import monitor
    if a.replay:
        if not a.html:
            raise ValueError("--replay needs --html OUT")
        _show(monitor.replay_html(a.replay, a.html, split=a.split), False)
        return
    monitor.serve(model, os.path.dirname(os.path.abspath(a.file)), a.broker, a.port, a.record, a.host, a.mcap)


def cmd_role(model, a) -> None:
    """Internal: one process of a simulated site (world, probe or machine), started by `simulate-site`."""
    from .runtime.site import role
    role(a.file, model, a)


# Sub-command name to the function that runs it. Their errors (ValueError, OSError) are printed as one line, without a
# traceback.
FIELD_COMMANDS = {"run": cmd_run, "simulate-site": cmd_simulate_site, "mavlink": cmd_mavlink, "vda5050": cmd_vda5050,
                  "soil-calibrate": cmd_soil_calibrate, "opera": cmd_opera,
                  "resume": cmd_resume, "estop": cmd_estop, "sense": cmd_sense,
                  "_role": cmd_role, "monitor": cmd_monitor,
                  "export": cmd_export, "terrain": cmd_terrain, "score": cmd_score, "nmea": cmd_nmea}


def print_report(r: dict) -> None:
    """Print a simulator report for people: time, volumes, passes per task, and the finish zone as the program saw it
    and as it truly is.
    """
    u = r["units"]
    print()
    print(f"== {r['mode']} result after {r['minutes']} min ({'all tasks done' if r['finished'] else 'NOT finished'})")
    print(f"cut {r['cut_bank_yd3']} yd3 bank, imported {r['imported_bank_yd3']} yd3 bank, "
          f"{r['probes']} surveys, {r['replans']} replans")
    me = r["material_estimates"]
    print(f"material: planner ended at swell {me['swell']} / keep {me['keep']} "
          f"(true {me['true_swell']} / {me['true_keep']})")
    for n, t in r["tasks"].items():
        print(f"  {n:<12} passes {t['passes']:>4}  done at {t['done_min']} min")
    tg, sg = r["true_ground"], r["as_seen_by_program"]
    print(f"zone {r['zone']} (±{r['tolerance']:g} {u}):")
    for label, s in (("as the program saw it", sg), ("true ground", tg)):
        print(f"  {label:<22}: p95 |error| {s['p95_abs']} {u}, {s['within_tolerance_pct']}% of cells in tolerance, "
              f"mean {s['mean']:+} {u}")
    print(f"volume ledger error: {r['ledger_error']:.2e} {u}3")


if __name__ == "__main__":
    main()
