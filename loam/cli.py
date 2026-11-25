"""Command line: loam check | explain | compile | sim"""
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
    a = ap.parse_args(argv)
    model = load(a.file)
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
