"""Run pad_north.loam open-loop and closed-loop and plot the results side by side.

    python examples/compare.py   ->  examples/pad_north_compare.png
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import matplotlib

from loam.checker import check
from loam.parser import parse_file
from loam.sim import Sim

matplotlib.use("Agg")
import matplotlib.pyplot as plt

here = os.path.dirname(os.path.abspath(__file__))
model, _ = check(parse_file(os.path.join(here, "pad_north.loam")))
runs = {}
for mode in (True, False):
    s = Sim(model, base_dir=here, open_loop=mode, log=lambda *a: None)
    runs["open" if mode else "closed"] = (s, s.run())

s0 = runs["closed"][0]
ext = [s0.x0, s0.x0 + s0.nx * s0.cell, s0.y0, s0.y0 + s0.ny * s0.cell]
tol = runs["closed"][1]["tolerance"]
fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), constrained_layout=True)
im0 = axes[0].imshow(s0.z_start - s0.design, origin="lower", extent=ext, cmap="RdBu_r", vmin=-3, vmax=3)
axes[0].set_title("Before: existing ground minus design", fontsize=10)
fig.colorbar(im0, ax=axes[0], label="ft", shrink=0.85)
for ax, key, label in ((axes[1], "open", "Open loop: one survey, no feedback"),
                       (axes[2], "closed", "Closed loop: overhead probe every 5 min")):
    s, r = runs[key]
    im = ax.imshow(s.z - s.design, origin="lower", extent=ext, cmap="RdBu_r", vmin=-0.4, vmax=0.4)
    pct = r["true_ground"]["within_tolerance_pct"]
    ax.set_title(f"{label}\n{pct}% of pad within ±{tol:g} ft (true ground)", fontsize=10)
fig.colorbar(im, ax=axes[1:], label="ft from design (red high, blue low)", shrink=0.85)
for ax in axes:
    ax.set_xlabel("ft")
fig.suptitle("Same Loam program, same machines, same soil: feedback is what gets the pad to grade", fontsize=12)
out = os.path.join(here, "pad_north_compare.png")
fig.savefig(out, dpi=120)
print(f"wrote {out}")
for k, (_, r) in runs.items():
    print(k, r["true_ground"], "program saw", r["as_seen_by_program"])
