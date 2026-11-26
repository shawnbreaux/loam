"""Command line: loam check | explain | compile | sim | export | terrain | score | nmea"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import TextIO

from .checker import check, has_errors
from .compiler import explain, manifest
from .lexer import LoamSyntaxError
from .parser import parse_file


def load(path: str):
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


def main(argv=None) -> None:
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
    a, b = (float(v) for v in text.split(","))
    return a, b


def _frame_args(p: argparse.ArgumentParser) -> None:
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
    from .geo import Frame
    from .units import length_unit_si
    lat0, lon0 = a.origin if a.origin else (None, None)
    return Frame(grid.u, a.rotation, lat0, lon0, a.world_origin[0], a.world_origin[1],
                 length_unit_si(a.world_units), a.dz, getattr(a, "crs", None))


def _grid(model, a):
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
    from .survey import idw, read_points_csv
    if path == "existing":
        ex = grid.m.site.get("existing") or {}
        if "file" not in ex:
            raise ValueError("the site declares no existing surface")
        return grid.surface(ex, allow_gaps=True), {"source": "existing"}
    if path.lower().endswith(".csv") and _is_survey_csv(path):
        s = read_points_csv(path, frame, a.layout)
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
    p.add_argument("--stat", choices=["mean", "min"], default="mean", help="per-cell statistic for clouds")
    p.add_argument("--ground-only", action="store_true", help="LAS/LAZ: keep only class 2 (ground) points")
    p.add_argument("--ground-filter", action="store_true",
                   help="clouds: drop machines, people and brush with a morphological ground filter")
    p.add_argument("--fill", type=int, default=2, metavar="CELLS", help="fill cloud holes up to this deep")
    p.add_argument("--radius", type=float, default=3.0, metavar="M",
                   help="survey CSV: interpolation radius in metres (default 3)")
    p.add_argument("--layout", default="PNEZD", choices=["PNEZD", "PENZD"], help="survey CSV without a header")


def add_field_commands(sub) -> None:
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
                   help="measured ground: 'existing', a survey .csv, or a .las/.pcd/.xyz cloud")
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
    p = sub.add_parser("vda5050", help="drive a VDA 5050 3.0 mobile robot as a Loam machine")
    p.add_argument("file")
    p.add_argument("--machine", required=True, help="the Loam machine this robot is")
    p.add_argument("--manufacturer", required=True)
    p.add_argument("--serial", required=True, help="the robot's serialNumber")
    p.add_argument("--interface", default="uagv", help="first MQTT topic level (default uagv)")
    p.add_argument("--broker", default="localhost:1883")
    p.add_argument("--sim-world", action="store_true", help="part of `loam simulate-site`: its clock and world")
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
    p.add_argument("--record", help="record dashboard frames as JSON lines")
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


def cmd_terrain(model, a) -> None:
    import json as _json

    from . import wire
    from .landxml import write_grid_tin
    grid = _grid(model, a)
    frame = _frame(grid, a)
    z, info = load_ground(a.cloud, grid, frame, a)
    probe = a.probe or next(iter(model.probes), "probe")
    if a.wire:
        msg = wire.terrain(model.site["name"], probe, 1, (grid.x0, grid.y0), grid.cell, grid.shape,
                           data=wire.encode_heights(z), ts=os.path.getmtime(a.cloud))
        with open(a.wire, "a", encoding="utf-8") as f:
            f.write(_json.dumps({"topic": wire.topic(model.site["name"], None, "terrain"), "payload": msg}) + "\n")
        info["wire"] = a.wire
    if a.landxml:
        write_grid_tin(a.landxml, "SCAN", grid.xs, grid.ys, z, grid.unit)
        info["landxml"] = a.landxml
    _show(info, False)


def cmd_score(model, a) -> None:
    from .survey import read_points_csv, score
    grid = _grid(model, a)
    frame = _frame(grid, a)
    z = model.zones.get(a.zone)
    if z is None or "polygon" not in (z.get("area") or {}):
        raise ValueError(f"no polygon zone named {a.zone!r}")
    tol = z["tolerance"].to(grid.unit) if z.get("tolerance") else 0.1 / (grid.u / 0.3048)
    design = grid.surface(model.site["design"], allow_gaps=True)
    s = read_points_csv(a.survey, frame, a.layout)
    r = score(grid, design, s, z["area"]["polygon"], tol)
    r.update({"zone": a.zone, "tolerance": tol, "units": grid.unit})
    _show(r, a.json)


def cmd_nmea(model, a) -> None:
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
    from .runtime.controller import run_controller
    r = run_controller(model, os.path.dirname(os.path.abspath(a.file)), a.broker, a.record, a.max_minutes,
                       wait_for_clock=not a.real_time)
    r.pop("log", None)
    if a.json:
        print(json.dumps(r, indent=2))


def cmd_simulate_site(model, a) -> None:
    from .runtime.site import launch
    r = launch(a.file, a.broker, a.speed, a.faults, model, quiet=a.quiet, physics=a.physics,
               soil_table=a.soil_table)
    print(json.dumps(r, indent=2))


def cmd_mavlink(model, a) -> None:
    from .runtime.mavlink import run_mavlink
    run_mavlink(model, a.machine, a.broker, a.vehicle, _frame(_grid(model, a), a), a.min_fix, a.sim_world)


def cmd_vda5050(model, a) -> None:
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
                                                         "bekker_wong_sinkage_m")}}, a)


def cmd_opera(model, a) -> None:
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
    from . import monitor
    if a.replay:
        if not a.html:
            raise ValueError("--replay needs --html OUT")
        _show(monitor.replay_html(a.replay, a.html, split=a.split), False)
        return
    monitor.serve(model, os.path.dirname(os.path.abspath(a.file)), a.broker, a.port, a.record)


def cmd_role(model, a) -> None:
    from .runtime.site import role
    role(a.file, model, a)


FIELD_COMMANDS = {"run": cmd_run, "simulate-site": cmd_simulate_site, "mavlink": cmd_mavlink, "vda5050": cmd_vda5050,
                  "soil-calibrate": cmd_soil_calibrate, "opera": cmd_opera,
                  "_role": cmd_role, "monitor": cmd_monitor,
                  "export": cmd_export, "terrain": cmd_terrain, "score": cmd_score, "nmea": cmd_nmea}


def print_report(r: dict) -> None:
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
