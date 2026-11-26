# Simulation study: open loop vs closed loop

Program `examples/pad_north.loam`; 50 seeds per mode in A, 25 in B, 10 per level in C.
Values are means with 95 % bootstrap confidence intervals. "In tolerance" is the share of pad cells
within ±0.1 ft on the *true* ground; "program saw" is the same share on what the program measured
(closed loop) or predicted (open loop).

## A. Headline (default soil, planner beliefs wrong, scripted events)

| Metric | Open loop | Closed loop |
| --- | --- | --- |
| In tolerance, true ground (%) | 60.8 [60.7, 60.9] | 98.5 [98.4, 98.6] |
| In tolerance, as the program saw it (%) | 95.5 [95.4, 95.6] | 97.4 [97.2, 97.5] |
| 95th pct abs error, true ground (ft) | 0.250 [0.250, 0.251] | 0.068 [0.067, 0.069] |
| Mean error, true ground (ft) | -0.061 [-0.062, -0.061] | -0.001 [-0.001, -0.000] |
| Fill imported (yd³ bank) | 37.7 [37.6, 37.9] | 86.5 [85.8, 87.2] |
| Simulated time (min) | 86.1 [86.0, 86.3] | 118.0 [118.0, 118.0] |

Runs that finished every task: open 50/50, closed 50/50.

## B. Where the gap comes from (2×2 factorial, no events)

| Execution noise | Planner beliefs | Open loop in tol. (%) | Closed loop in tol. (%) |
| --- | --- | --- | --- |
| on | wrong | 60.8 [60.7, 61.0] | 98.3 [98.1, 98.5] |
| on | right | 92.2 [92.1, 92.3] | 99.0 [98.8, 99.1] |
| off | wrong | 64.1 [64.0, 64.2] | 98.2 [97.9, 98.6] |
| off | right | 95.5 [95.4, 95.6] | 97.3 [97.0, 97.7] |

## C. Sensitivity to the true compaction factor (planner believes 0.80)

True swell is set to the planner's belief (1.25) so that only compaction is mis-estimated.

| True keep | Open loop in tol. (%) | Closed loop in tol. (%) |
| --- | --- | --- |
| 0.60 | 50.4 [50.2, 50.6] | 98.9 [98.6, 99.1] |
| 0.65 | 52.2 [52.0, 52.4] | 98.7 [98.4, 99.0] |
| 0.70 | 56.2 [56.0, 56.4] | 98.9 [98.6, 99.3] |
| 0.75 | 68.6 [68.4, 68.8] | 98.7 [98.5, 98.8] |
| 0.80 | 92.3 [92.1, 92.5] | 98.8 [98.0, 99.5] |
| 0.85 | 68.7 [68.4, 69.0] | 98.5 [98.2, 98.8] |

All runs passed the volume-ledger check (|error| < 1e-6 ft³).

These are results of a simplified simulator with assumed soil factors. They describe how the
control loop behaves under the stated model, not field performance. See docs/SIMULATOR.md.
