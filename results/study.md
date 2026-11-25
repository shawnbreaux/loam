# Simulation study: open loop vs closed loop

Program `examples/pad_north.loam`; 50 seeds per mode in A, 25 in B, 10 per level in C.
Values are means with 95 % bootstrap confidence intervals. "In tolerance" is the share of pad cells
within ±0.1 ft on the *true* ground; "program saw" is the same share on what the program measured
(closed loop) or predicted (open loop).

## A. Headline (default soil, planner beliefs wrong, scripted events)

| Metric | Open loop | Closed loop |
| --- | --- | --- |
| In tolerance, true ground (%) | 61.7 [61.6, 61.7] | 99.7 [99.7, 99.8] |
| In tolerance, as the program saw it (%) | 96.8 [96.8, 96.8] | 98.5 [98.5, 98.6] |
| 95th pct abs error, true ground (ft) | 0.252 [0.251, 0.252] | 0.062 [0.062, 0.062] |
| Mean error, true ground (ft) | -0.069 [-0.069, -0.069] | 0.002 [0.002, 0.003] |
| Fill imported (yd³ bank) | 32.2 [32.2, 32.3] | 85.4 [84.8, 85.9] |
| Simulated time (min) | 98.4 [98.1, 98.7] | 133.9 [133.3, 134.4] |

Runs that finished every task: open 50/50, closed 50/50.

## B. Where the gap comes from (2×2 factorial, no events)

| Execution noise | Planner beliefs | Open loop in tol. (%) | Closed loop in tol. (%) |
| --- | --- | --- | --- |
| on | wrong | 61.7 [61.6, 61.8] | 99.3 [99.2, 99.5] |
| on | right | 93.1 [93.0, 93.2] | 99.6 [99.5, 99.7] |
| off | wrong | 65.3 [65.3, 65.4] | 99.2 [98.8, 99.5] |
| off | right | 96.8 [96.8, 96.8] | 97.4 [96.9, 97.9] |

## C. Sensitivity to the true compaction factor (planner believes 0.80)

True swell is set to the planner's belief (1.25) so that only compaction is mis-estimated.

| True keep | Open loop in tol. (%) | Closed loop in tol. (%) |
| --- | --- | --- |
| 0.60 | 51.2 [51.1, 51.4] | 99.7 [99.5, 99.9] |
| 0.65 | 53.0 [52.9, 53.2] | 99.5 [99.3, 99.7] |
| 0.70 | 57.1 [56.9, 57.2] | 99.7 [99.5, 99.9] |
| 0.75 | 69.5 [69.3, 69.7] | 99.7 [99.6, 99.8] |
| 0.80 | 93.2 [93.0, 93.3] | 99.8 [99.7, 99.9] |
| 0.85 | 69.5 [69.4, 69.7] | 99.8 [99.7, 99.9] |

All runs passed the volume-ledger check (|error| < 1e-6 ft³).

These are results of a simplified simulator with assumed soil factors. They describe how the
control loop behaves under the stated model, not field performance. See docs/SIMULATOR.md.
