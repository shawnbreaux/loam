# Reference simulator: model, assumptions and validation

`loam/sim.py` runs a checked Loam program against a simulated site. It exists to
test the language's semantics (closed-loop completion, rules, replanning) and to
give implementers a conformance target. It is **not** a production planner and
its soil parameters are illustrative, not calibrated to field data.

## State

The site is a regular grid of `nx × ny` cells of side `c` (the program's `grid`),
in site units. All grids are float64.

| Grid | Meaning |
| --- | --- |
| `design` | Target surface, rasterized from the design TIN |
| `z` | True ground. Starts as the existing-ground TIN |
| `loose` | True thickness of placed, not-yet-rolled fill |
| `meas` | Latest probe survey: `z + ε`, with ε ~ N(0, σ²) per cell |
| `pred` | Planner's belief about current ground |
| `p_loose` | Planner's belief about unrolled fill thickness |
| `plan_dz`, `plan_dloose` | Changes committed by in-flight passes, not yet landed |

Peak memory is about 160 bytes per cell (measured: 41.6 MB for 256 000 cells).

## Material model

For bank (in-place) volume `B` dug by a pass:

* Loose volume placed = `B · s`, where `s` is the swell factor (truth `Soil.swell`, belief `est_swell`).
* Rolling a loose lift of thickness `L` leaves `L · k`, where `k` is the compaction keep factor
  (truth `Soil.keep`, belief `est_keep`).
* Net: bank volume to finished fill is `s · k` (truth 1.30 × 0.70 = 0.91; initial belief 1.25 × 0.80 = 1.00).

Execution error:

* Cut depth `d` becomes `d · max(0, 1 + N(0, cut_error²))`.
* A finishing pass adds N(0, finish_error²) per cell.

Probe accuracy `a` is read as a 2σ bound, so σ = a / 2.

## Passes

Each order is a `Pass` whose effect is computed when the order is issued and
applied when it lands (`busy_until`). Consequences:

* A survey never sees work in progress.
* A `stop` rule extends `busy_until`, so ground is frozen while stopped (tested).
* The planner plans against `plan_view = pred + plan_dz − (p_loose + plan_dloose)(1 − est_keep)`,
  so concurrent machines do not plan the same fill twice.
* Work windows of in-flight passes are reserved; two passes never overlap (tested).

| Op | Planning | Landing |
| --- | --- | --- |
| cut | Best 30 ft × blade window by positive residual; depth ≤ `max_cut` | Lower `z`; place spoil in the lowest cells of each `spoil` destination in order, one lift at most; any surplus goes to a stockpile or a windrow |
| fill | Load ≤ ½ blade × segment from the first `source` stockpile with material; place in the lowest cells | Raise `z` and `loose` by the true loose volume |
| compact | Window over cells whose fill has landed | Truth: `z −= loose(1 − k)`; belief likewise with `est_keep`. Recomputed at landing from the fill actually present |
| finish | Window by \|residual\| > ½ tolerance; step ≤ 0.3 ft per cell | Cuts go to a windrow; fills draw from the windrow, then from the task's `source`. Fill is reduced if material runs short |

## Feedback

* **Surveys** every probe `rate`, except during scripted outages. The first survey seeds `pred`.
* **`deviation()`** is the 95th percentile of `|meas − pred|` over cells worked since the previous survey.
* **Drift replan** (when a `replan` condition holds): `pred ← meas`. Soil factors are then re-estimated
  from cells that saw exactly one kind of work between the last two surveys (at least 200 cells):
  * swell: `est_swell ← est_swell · Σ(meas − prev_meas) / Σ(expected rise)` over deposit-only cells
  * keep: `est_keep ← 1 − Σ(prev_meas − meas) / Σ(rolled loose)` over roll-only cells
* **Stall replan** (built in): when every machine is idle and an eligible task is unfinished,
  rebase on the latest survey once per survey.
* **Built-in stale hold**: no new orders while any probe is older than its `stale_after`.
* **Task completion**: `until` is evaluated on `meas`, never on `pred` (open loop: on `pred`).
  A task with work still in flight is never closed.

## Randomness

All noise comes from `loam/rng.py`, a counter-based SplitMix64 generator with
Irwin–Hall normals (sum of 12 uniforms − 6). It is fully specified, so results
are bit-for-bit reproducible across platforms, NumPy versions and the planned
Assembly port. The reference values are pinned in the tests.

## Invariants checked by the test suite

| Invariant | Test |
| --- | --- |
| Volume ledger: every change to `z` is booked; \|booked − actual\| < 1e-6 | `test_volume_ledger_balances` |
| Same seed gives an identical report | `test_deterministic_for_a_seed` |
| The program model is not mutated by a run | `test_model_is_not_mutated` |
| In-flight work windows never overlap | `test_inflight_windows_never_overlap` |
| Ground does not change during a `stop` | `test_ground_is_frozen_during_a_stop` |
| Exclusion zones plus buffer are never touched | `test_exclusion_zone_is_never_touched` |
| Control: with correct beliefs and no execution noise, open loop is unbiased and meets the completion rule | `test_control_nothing_unknown_means_open_loop_is_unbiased` |
| No memory or file handles retained across runs | `test_no_memory_or_file_handles_retained` |

## Known limitations

* **One probe**, with uniform Gaussian error and no occlusion. Real lidar has shadows, dust and range-dependent error.
* **Simple soil.** Two scalar factors, uniform across the site, constant over time; no moisture, no
  material types, no slope stability.
* **Simple machines.** A pass is a rectangle cut or fill with an average speed; no blade-load limit
  beyond segment size, no turning geometry, no traction, no fuel. Travel time is straight-line.
* **Scripted events.** People, outages and rain are fixed windows. `proximity()` returns the scripted
  distance whatever its arguments.
* **Compaction** is an intelligent-compaction coverage stand-in (85–95 %), not a density model.
* **Single-threaded fleet logic**, with a 15 s tick. Order latency and radio loss are not modelled.

None of these limitations affects the property the study tests: whether conditions evaluated on
measured ground reach tolerance when the planner's material model is wrong.
