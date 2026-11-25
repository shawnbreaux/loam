"""Monte Carlo study of open-loop vs closed-loop execution of examples/pad_north.loam.

    python examples/study.py [--seeds 50] [--jobs 8]

Writes results/study.json, results/study.md and results/study.png. Every run is
seeded, so the study is exactly reproducible. Confidence intervals are 95 %
percentile bootstraps (10 000 resamples) of the mean across seeds.

Experiments
  A  headline: default soil and planner beliefs, scripted events on, N seeds per mode
  B  2x2 factorial: planner beliefs (wrong/right) x execution noise (on/off), no events
  C  sensitivity: true compaction factor swept while the planner keeps believing 0.80 (swell held equal)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from loam.checker import check  # noqa: E402
from loam.parser import parse_file  # noqa: E402
from loam.sim import Beliefs, Events, Sim, Soil  # noqa: E402

MODEL = None


def _model():
    global MODEL
    if MODEL is None:
        MODEL, _ = check(parse_file(os.path.join(HERE, "pad_north.loam")))
    return MODEL


def run(job: tuple) -> dict:
    exp, seed, open_loop, soil, beliefs, events = job
    r = Sim(_model(), base_dir=HERE, seed=seed, open_loop=open_loop, soil=Soil(**soil), beliefs=Beliefs(**beliefs),
            events=Events() if events else Events.none(), log=lambda *a: None).run()
    return {"exp": exp, "seed": seed, "mode": r["mode"], "soil": soil, "beliefs": beliefs,
            "in_tol": r["true_ground"]["within_tolerance_pct"], "p95": r["true_ground"]["p95_abs"],
            "mean": r["true_ground"]["mean"], "seen_in_tol": r["as_seen_by_program"]["within_tolerance_pct"],
            "minutes": r["minutes"], "import": r["imported_bank_yd3"], "finished": r["finished"],
            "ledger": r["ledger_error"], "replans": r["replans"]}


def boot_ci(x: list[float], n: int = 10_000, seed: int = 0) -> tuple[float, float, float]:
    a = np.asarray(x, float)
    rng = np.random.default_rng(seed)   # analysis only; does not affect simulated results
    means = a[rng.integers(0, a.size, (n, a.size))].mean(1)
    return float(a.mean()), float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def fmt(x: list[float], digits: int = 1) -> str:
    m, lo, hi = boot_ci(x)
    return f"{m:.{digits}f} [{lo:.{digits}f}, {hi:.{digits}f}]"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    a = ap.parse_args()
    soil, bel = asdict(Soil()), asdict(Beliefs())
    quiet_soil = {**soil, "cut_error": 0.0, "finish_error_m": 0.0}
    jobs = [("A", s, ol, soil, bel, True) for s in range(a.seeds) for ol in (True, False)]
    nb = max(10, a.seeds // 2)
    for noise in (True, False):
        for right in (False, True):
            so = soil if noise else quiet_soil
            be = {"swell": so["swell"], "keep": so["keep"]} if right else bel
            jobs += [(f"B:{'noise' if noise else 'quiet'}:{'right' if right else 'wrong'}", s, ol, so, be, False)
                     for s in range(nb) for ol in (True, False)]
    keeps = [0.60, 0.65, 0.70, 0.75, 0.80, 0.85]
    nc = max(10, a.seeds // 5)
    # Swell held at the planner's belief so only the compaction factor is wrong (what matters is swell x keep).
    jobs += [(f"C:{k:.2f}", s, ol, {**soil, "swell": bel["swell"], "keep": k}, bel, False) for k in keeps
             for s in range(nc) for ol in (True, False)]
    with ProcessPoolExecutor(a.jobs) as ex:
        rows = list(ex.map(run, jobs, chunksize=4))
    assert all(abs(r["ledger"]) < 1e-6 for r in rows), "volume ledger violated"

    out = os.path.join(HERE, "..", "results")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "study.json"), "w", encoding="utf-8") as f:
        json.dump({"seeds": a.seeds, "rows": rows}, f, indent=1)

    def sel(exp: str, mode: str, key: str) -> list[float]:
        return [r[key] for r in rows if r["exp"] == exp and r["mode"] == mode]

    md = ["# Simulation study: open loop vs closed loop", "",
          f"Program `examples/pad_north.loam`; {a.seeds} seeds per mode in A, {nb} in B, {nc} per level in C.",
          "Values are means with 95 % bootstrap confidence intervals. \"In tolerance\" is the share of pad cells",
          "within ±0.1 ft on the *true* ground; \"program saw\" is the same share on what the program measured",
          "(closed loop) or predicted (open loop).", "",
          "## A. Headline (default soil, planner beliefs wrong, scripted events)", "",
          "| Metric | Open loop | Closed loop |", "| --- | --- | --- |"]
    for label, key, d in (("In tolerance, true ground (%)", "in_tol", 1),
                          ("In tolerance, as the program saw it (%)", "seen_in_tol", 1),
                          ("95th pct abs error, true ground (ft)", "p95", 3),
                          ("Mean error, true ground (ft)", "mean", 3),
                          ("Fill imported (yd³ bank)", "import", 1),
                          ("Simulated time (min)", "minutes", 1)):
        md.append(f"| {label} | {fmt(sel('A', 'open-loop', key), d)} | {fmt(sel('A', 'closed-loop', key), d)} |")
    fin = {m: sum(sel("A", m, "finished")) for m in ("open-loop", "closed-loop")}
    md += ["", f"Runs that finished every task: open {fin['open-loop']}/{a.seeds}, "
               f"closed {fin['closed-loop']}/{a.seeds}.", "",
           "## B. Where the gap comes from (2×2 factorial, no events)", "",
           "| Execution noise | Planner beliefs | Open loop in tol. (%) | Closed loop in tol. (%) |",
           "| --- | --- | --- | --- |"]
    for noise in ("noise", "quiet"):
        for right in ("wrong", "right"):
            e = f"B:{noise}:{right}"
            md.append(f"| {'on' if noise == 'noise' else 'off'} | {right} | {fmt(sel(e, 'open-loop', 'in_tol'))} | "
                      f"{fmt(sel(e, 'closed-loop', 'in_tol'))} |")
    md += ["", "## C. Sensitivity to the true compaction factor (planner believes 0.80)", "",
           "True swell is set to the planner's belief (1.25) so that only compaction is mis-estimated.", "",
           "| True keep | Open loop in tol. (%) | Closed loop in tol. (%) |", "| --- | --- | --- |"]
    for k in keeps:
        e = f"C:{k:.2f}"
        md.append(f"| {k:.2f} | {fmt(sel(e, 'open-loop', 'in_tol'))} | {fmt(sel(e, 'closed-loop', 'in_tol'))} |")
    md += ["", "All runs passed the volume-ledger check (|error| < 1e-6 ft³).", "",
           "These are results of a simplified simulator with assumed soil factors. They describe how the",
           "control loop behaves under the stated model, not field performance. See docs/SIMULATOR.md."]
    with open(os.path.join(out, "study.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.4, 3.8), constrained_layout=True)
    try:
        for mode, mk in (("open-loop", "o-"), ("closed-loop", "s-")):
            stats = [boot_ci(sel(f"C:{k:.2f}", mode, "in_tol")) for k in keeps]
            m = [s[0] for s in stats]
            ax.plot(keeps, m, mk, label=mode)
            ax.fill_between(keeps, [s[1] for s in stats], [s[2] for s in stats], alpha=0.2)
        ax.axvline(0.80, ls="--", lw=1, color="grey")
        ax.text(0.803, 75, "planner's belief", fontsize=8, color="grey")
        ax.set_xlabel("true compaction factor (compacted / loose thickness)")
        ax.set_ylabel("pad within ±0.1 ft, true ground (%)")
        ax.set_ylim(50, 101)
        ax.legend(loc="center left")
        ax.set_title("Belief error vs grade accuracy (95 % CI)", fontsize=10)
        fig.savefig(os.path.join(out, "study.png"), dpi=130)
    finally:
        plt.close(fig)
    print("\n".join(md))


if __name__ == "__main__":
    main()
