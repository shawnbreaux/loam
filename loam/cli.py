"""Command line: loam check | explain | compile | sim | export | terrain | score | nmea"""
from __future__ import annotations

import argparse
import json
import os
import sys

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


def _frame(grid, a):
    from .geo import Frame
    from .units import length_unit_si
    lat0, lon0 = a.origin if a.origin else (None, None)
    return Frame(grid.u, a.rotation, lat0, lon0, a.world_origin[0], a.world_origin[1],
                 length_unit_si(a.world_units), a.dz)


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
    """Measured ground on the site grid from 'existing', a survey CSV or a point cloud."""
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
    z, info = grid_cloud(path, grid, frame, a.stat, a.ground_only)
    return fill_gaps(z, a.fill), {"source": "point cloud", **info}


def _ground_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--stat", choices=["mean", "min"], default="mean", help="per-cell statistic for clouds")
    p.add_argument("--ground-only", action="store_true", help="LAS: keep only class 2 (ground) points")
    p.add_argument("--fill", type=int, default=2, metavar="CELLS", help="fill cloud holes up to this deep")
    p.add_argument("--radius", type=float, default=3.0, metavar="M",
                   help="survey CSV: interpolation radius in metres (default 3)")
    p.add_argument("--layout", default="PNEZD", choices=["PNEZD", "PENZD"], help="survey CSV without a header")


def add_field_commands(sub) -> None:
    p = sub.add_parser("export", help="write a surface or field files for grade-control and guidance tools")
    p.add_argument("file")
    p.add_argument("--to", required=True, choices=["landxml", "agd", "agopengps"],
                   help="landxml: Cat Grade 3D and most machine control; agd: OpenGrade3D; agopengps: field folder")
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
    p = sub.add_parser("nmea", help="turn a GGA stream (file or stdin) into Wire state messages")
    p.add_argument("file")
    p.add_argument("input", nargs="?", default="-", help="NMEA file, or - for stdin (e.g. piped from nc)")
    p.add_argument("--machine", required=True)
    p.add_argument("-o", "--out", help="write JSON lines here (default stdout)")
    _frame_args(p)


def cmd_export(model, a) -> None:
    from . import export
    from .target import next_target
    grid = _grid(model, a)
    frame = _frame(grid, a)
    design = grid.surface(model.site["design"], allow_gaps=True)
    out: dict = {"to": a.to, "out": a.out}
    if a.to == "agopengps":
        out.update(export.agopengps(a.out, grid, frame))
    else:
        ground, gi = load_ground(a.ground, grid, frame, a)
        out["ground"] = gi
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
    src = sys.stdin if a.input == "-" else open(a.input, encoding="ascii", errors="replace")  # noqa: SIM115
    dst = open(a.out, "w", encoding="utf-8") if a.out else sys.stdout  # noqa: SIM115
    try:
        topic = wire.topic(model.site["name"], a.machine, "state")
        for msg in nmea_states(src, frame, model.site["name"], a.machine):
            dst.write(_json.dumps({"topic": topic, "payload": msg}) + "\n")
            dst.flush()
    finally:
        if src is not sys.stdin:
            src.close()
        if dst is not sys.stdout:
            dst.close()


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


FIELD_COMMANDS = {"export": cmd_export, "terrain": cmd_terrain, "score": cmd_score, "nmea": cmd_nmea}


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
