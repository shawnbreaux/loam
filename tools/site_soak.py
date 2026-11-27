#!/usr/bin/env python3
"""Run a whole simulated Loam site as on a real one, and watch what each process does to the computer.

A broker, `loam simulate-site` (world, lidar probe, one process per machine), `loam run` (the controller) and
`loam monitor` (dashboard, metrics, MCAP log) are started as separate processes. While the job runs, every
process's memory, open files and threads are sampled every two seconds and the monitor is polled like a Prometheus
server would. Optionally the broker or the controller is stopped and started again in the middle of the job.

    python tools/site_soak.py out/ --speed 6                              # a 35-minute job
    python tools/site_soak.py out/ --speed 60 --event broker-restart@45   # restart the broker 45 s in
    python tools/site_soak.py out/ --speed 60 --event controller-kill@60  # kill -9 the controller, start it again

Afterwards it prints, and writes to out/soak.json: whether the job finished and how well, each process's memory at
the start, a quarter of the way in and at the end with its growth over the second half, every warning or error
any process logged, and whether every recorded Wire message is valid against the published schema (needs the
`mcap` and `jsonschema` packages; skipped without them). The exit code is 0 only if the job finished in tolerance,
nothing was logged as an error, no message was invalid and no process was left running.

Needs Linux (it reads /proc) and the `mosquitto` broker. Slower is a harder test: messages go out at wall-clock
rates, so `--speed 6` sends a hundred times as many per site minute as the default 600.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLAGGED = re.compile(r"Traceback|Error|Warning|Exception|unclosed", re.I)
HARMLESS = ("Address family not supported",)         # Mosquitto on a computer without IPv6


def free_port() -> int:
    """A TCP port nothing is listening on."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def tree(pid: int) -> list[int]:
    """`pid` and every process descended from it."""
    kids: dict[int, list[int]] = {}
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                with open(f"/proc/{p}/stat", encoding="utf-8") as f:
                    kids.setdefault(int(f.read().rsplit(")", 1)[1].split()[1]), []).append(int(p))
            except OSError:                              # it ended while we looked
                pass
    out, todo = [], [pid]
    while todo:
        q = todo.pop()
        out.append(q)
        todo += kids.get(q, [])
    return out


def sample(pid: int) -> tuple[str, int, int, int] | None:
    """A process's command line, resident memory in kB, open files and threads; None if it has gone."""
    try:
        with open(f"/proc/{pid}/status", encoding="utf-8") as f:
            status = f.read()
        with open(f"/proc/{pid}/cmdline", encoding="utf-8") as f:
            cmd = f.read().replace("\0", " ")
        rss, threads = re.search(r"VmRSS:\s+(\d+)", status), re.search(r"Threads:\s+(\d+)", status)
        if rss is None or threads is None:
            return None
        return cmd, int(rss.group(1)), len(os.listdir(f"/proc/{pid}/fd")), int(threads.group(1))
    except OSError:
        return None


def label(cmd: str) -> str | None:
    """A short name for a process of the site: "broker", "run", "monitor", "machine dz1", ...; None for others."""
    if "mosquitto" in cmd:
        return "broker"
    if "loam.cli " not in cmd:
        return None
    args = cmd.split("loam.cli ", 1)[1].split()
    if args[0] != "_role":
        return args[0]
    role = args[args.index("--role") + 1] if "--role" in args else "role"
    return f"{role} {args[args.index('--name') + 1]}" if "--name" in args else role


def check_wire(mcap_path: str) -> tuple[int, dict[str, int]] | None:
    """Validate every message in the MCAP log against the Wire schema: (messages, {problem: count}), or None if the
    packages for it are missing or the log cannot be read.
    """
    try:
        import jsonschema
        from mcap.reader import make_reader
    except ImportError:
        return None
    with open(os.path.join(ROOT, "schemas", "wire", "loam-wire-0.0.1.schema.json"), encoding="utf-8") as f:
        defs = json.load(f)["$defs"]
    checkers: dict[str, jsonschema.Draft202012Validator] = {}
    n, bad = 0, {}
    try:
        with open(mcap_path, "rb") as f:
            for _, channel, message in make_reader(f).iter_messages():
                n += 1
                kind = channel.topic.rsplit("/", 1)[-1]
                if kind not in defs:
                    bad[f"{kind}: no schema"] = bad.get(f"{kind}: no schema", 0) + 1
                    continue
                if kind not in checkers:
                    checkers[kind] = jsonschema.Draft202012Validator({"$defs": defs, "$ref": f"#/$defs/{kind}"})
                for e in checkers[kind].iter_errors(json.loads(message.data)):
                    key = f"{kind}: {e.message[:90]}"
                    bad[key] = bad.get(key, 0) + 1
    except Exception as e:
        return n, {f"log unreadable: {e!r}"[:120]: 1}
    return n, bad


def main() -> int:
    """Run the site, report, and return the exit code."""
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("out", help="folder for logs, recordings and soak.json")
    ap.add_argument("--site", default=os.path.join(ROOT, "examples", "pad_north.loam"))
    ap.add_argument("--speed", default="60", help="site minutes per wall minute (default 60)")
    ap.add_argument("--event", help="broker-restart@S, controller-restart@S or controller-kill@S: S seconds in")
    ap.add_argument("--down", type=float, default=4.0, help="seconds the broker or controller stays down")
    ap.add_argument("--limit", type=float, default=7200.0, help="give up after this many seconds")
    a = ap.parse_args()
    exe = shutil.which("mosquitto") or "/usr/sbin/mosquitto"
    if not os.path.exists(exe) or not os.path.isdir("/proc"):
        sys.exit("site_soak needs Linux and the mosquitto broker")
    os.makedirs(a.out, exist_ok=True)
    loam = [sys.executable, "-m", "loam.cli"]
    port, http = free_port(), free_port()
    b = f"127.0.0.1:{port}"

    def log(name: str):
        """A log file in the output folder, opened for writing."""
        return open(os.path.join(a.out, name), "w", encoding="utf-8")

    def start_broker(logname: str) -> subprocess.Popen:
        """Start Mosquitto on the run's port."""
        return subprocess.Popen([exe, "-p", str(port)], stdout=subprocess.DEVNULL, stderr=log(logname))

    def start_controller(n: int) -> subprocess.Popen:
        """Start `loam run`; `n` numbers its output files."""
        return subprocess.Popen([*loam, "run", a.site, "--broker", b, "--record", f"{a.out}/orders{n}.jsonl", "--json"],
                                stdout=log(f"run{n}.out"), stderr=log(f"run{n}.log"), cwd=ROOT)

    procs = {"broker": start_broker("broker.log")}
    time.sleep(0.5)
    procs["monitor"] = subprocess.Popen(
        [*loam, "monitor", a.site, "--broker", b, "--port", str(http), "--mcap", f"{a.out}/run.mcap",
         "--record", f"{a.out}/frames.jsonl"], stdout=log("monitor.log"), stderr=subprocess.STDOUT, cwd=ROOT)
    procs["site"] = subprocess.Popen([*loam, "simulate-site", a.site, "--broker", b, "--speed", a.speed, "--quiet"],
                                     stdout=log("site.out"), stderr=log("site.log"), cwd=ROOT)
    time.sleep(1.5)
    procs["run"] = start_controller(1)
    t0 = time.time()
    series: dict[str, list[tuple[float, int, int, int]]] = {}
    http_errors: list[str] = []
    stop = threading.Event()

    def watch() -> None:
        """Sample every process of the site and poll the monitor until told to stop."""
        k = 0
        while not stop.is_set():
            now = round(time.time() - t0, 1)
            for p in list(procs.values()):
                for pid in tree(p.pid):
                    s = sample(pid)
                    name = label(s[0]) if s else None
                    if s and name:
                        series.setdefault(f"{name} #{pid}", []).append((now, *s[1:]))
            if k % 3 == 2:
                for path in ("/metrics", "/"):
                    try:
                        with urllib.request.urlopen(f"http://127.0.0.1:{http}{path}", timeout=3) as r:
                            r.read()
                    except OSError as e:
                        http_errors.append(f"{now} s {path}: {e!r}"[:120])
            k += 1
            stop.wait(2.0)

    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()
    notes: list[str] = []
    codes: dict[str, object] = {}
    try:
        if a.event:
            what, at = a.event.split("@")
            time.sleep(float(at))
            if what == "broker-restart":
                procs["broker"].terminate()
                procs["broker"].wait(10)
                time.sleep(a.down)
                procs["broker"] = start_broker("broker2.log")
            elif what in ("controller-restart", "controller-kill"):
                procs["run"].send_signal(signal.SIGTERM if what == "controller-restart" else signal.SIGKILL)
                codes["run (first)"] = procs["run"].wait(20)
                time.sleep(a.down)
                procs["run"] = start_controller(2)
            else:
                sys.exit(f"unknown event {what!r}")
            notes.append(f"{what} at {at} s, down for {a.down:g} s")
        try:
            codes["run"] = procs["run"].wait(a.limit)
        except subprocess.TimeoutExpired:
            codes["run"] = "did not finish"
        wall = time.time() - t0
        try:
            codes["site"] = procs["site"].wait(120)
        except subprocess.TimeoutExpired:
            codes["site"] = "did not finish"
    finally:
        stop.set()
        watcher.join(5)
        for name, sig in (("run", signal.SIGTERM), ("site", signal.SIGTERM), ("monitor", signal.SIGINT)):
            if procs[name].poll() is None:
                procs[name].send_signal(sig)
        for name in ("run", "site", "monitor"):
            try:
                rc = procs[name].wait(20)
            except subprocess.TimeoutExpired:
                procs[name].kill()
                rc = "killed"
            codes.setdefault(name, rc)
        time.sleep(0.5)
        left = [s[0] for p in os.listdir("/proc") if p.isdigit() and (s := sample(int(p))) and b in s[0]
                and "loam.cli" in s[0]]
        procs["broker"].terminate()
        procs["broker"].wait(10)

    # -- the report --------------------------------------------------------------------------------
    truth: dict = {}
    try:
        with open(f"{a.out}/site.out", encoding="utf-8") as f:
            truth = json.load(f).get("truth", {})
    except (OSError, ValueError):
        pass
    zones = truth.get("zones", {})
    in_tol = min((z["within_tolerance_pct"] for z in zones.values()), default=0.0)
    print(f"{wall:.0f} s wall, {truth.get('t', 0):.0f} site minutes; exit codes {codes}"
          + (f"; {', '.join(notes)}" if notes else ""))
    print(f"true ground in tolerance: {in_tol:.1f} %; passes applied {truth.get('passes_applied')}, "
          f"orders the world had already seen {truth.get('duplicate_reports')}, "
          f"volume ledger error {truth.get('ledger_error', float('nan')):.1e}")
    print(f"\n{'process':28s} {'samples':>7s} {'MB first':>9s} {'at 25 %':>8s} {'last':>7s} "
          f"{'kB/min over 2nd half':>21s} {'files':>9s} {'threads':>8s}")
    rows = {}
    for name, s in sorted(series.items()):
        if len(s) < 8:
            continue
        t, rss = [r[0] for r in s], [r[1] for r in s]
        h = len(s) // 2
        mt, mr = sum(t[h:]) / len(t[h:]), sum(rss[h:]) / len(rss[h:])
        den = sum((x - mt) ** 2 for x in t[h:])
        slope = 60 * sum((x - mt) * (y - mr) for x, y in zip(t[h:], rss[h:], strict=True)) / den if den else 0.0
        rows[name] = {"first_mb": rss[0] / 1024, "quarter_mb": rss[len(s) // 4] / 1024, "last_mb": rss[-1] / 1024,
                      "slope_kb_per_min": slope, "files": [min(r[2] for r in s), max(r[2] for r in s)],
                      "threads": [min(r[3] for r in s), max(r[3] for r in s)]}
        r = rows[name]
        print(f"{name:28s} {len(s):7d} {r['first_mb']:9.1f} {r['quarter_mb']:8.1f} {r['last_mb']:7.1f} "
              f"{slope:21.1f} {r['files'][0]:4d}-{r['files'][1]:<4d} {r['threads'][0]:3d}-{r['threads'][1]:<3d}")
    flagged: dict[str, int] = {}
    for name in sorted(os.listdir(a.out)):
        if name.endswith(".log"):
            with open(os.path.join(a.out, name), encoding="utf-8", errors="replace") as f:
                for line in f:
                    if FLAGGED.search(line) and not any(h in line for h in HARMLESS):
                        key = f"{name}: {re.sub(r'[0-9]+', 'N', line.strip())[:110]}"
                        flagged[key] = flagged.get(key, 0) + 1
    print(f"\nlogged warnings and errors: {sum(flagged.values())}")
    for key, n in list(flagged.items())[:15]:
        print(f"  {n:5d} x {key}")
    wire = check_wire(f"{a.out}/run.mcap")
    if wire is None:
        print("Wire messages not checked (needs the mcap and jsonschema packages)")
    else:
        print(f"Wire messages recorded: {wire[0]}, invalid against the schema: {sum(wire[1].values())}")
        for key, n in list(wire[1].items())[:10]:
            print(f"  {n:5d} x {key}")
    if http_errors[2:]:                                  # the first poll or two can beat the monitor's start
        print(f"monitor did not answer {len(http_errors)} times: {http_errors[:3]}")
    if left:
        print(f"left running: {left}")
    ok = (codes.get("run") == 0 and codes.get("site") == 0 and in_tol >= 95 and not flagged and not left
          and not (wire and wire[1]))
    with open(f"{a.out}/soak.json", "w", encoding="utf-8") as f:
        json.dump({"ok": ok, "wall_s": wall, "exit_codes": {k: str(v) for k, v in codes.items()}, "truth": truth,
                   "processes": rows, "flagged": flagged, "wire": wire, "left_running": left, "notes": notes,
                   "series": series}, f)
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
