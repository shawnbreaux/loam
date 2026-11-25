# Loam

**An open language for programmable, closed-loop earthmoving.**

Loam is to a job site roughly what G-code is to a CNC machine, with one difference that drives the whole
design: soil is not a rigid workpiece. A Loam program states the finished ground, the machines, the
sensors and the safety rules, and says *when the work is done* in terms of **measured** ground. A runtime
turns it into passes for each machine and keeps checking the real ground, for example from a tethered
drone, until the site is in tolerance.

```loam
loam 0.0.1

site pad_north {
  crs    "EPSG:6447"
  units  ft
  design "pad_design.xml" format landxml surface "FG"
}

probe overhead { kind tethered_uav; sensor lidar; rate 5 min; accuracy 0.05 ft }
machine dz1    { class dozer; blade 12 ft }

zone A { role cut;  area rect (0, 0) (85, 100);   tolerance ±0.1 ft }
zone C { role fill; area rect (85, 0) (160, 100); lift 8 in }

task strip_A {
  op      cut
  zone    A
  by      class dozer
  max_cut 0.5 ft per pass
  spoil   -> C
  until   residual(A).high < 0.15 ft
}

rule people { when proximity(person, machine) < 50 ft; do stop all }
replan drift { when deviation() > 0.15 ft }
```

| Document | Contents |
| --- | --- |
| [SPEC.md](SPEC.md) | Language, execution semantics, Loam Wire messages, safety model |
| [docs/SIMULATOR.md](docs/SIMULATOR.md) | Reference simulator: model, equations, invariants, limitations |
| [results/study.md](results/study.md) | Monte Carlo study, 420 seeded runs |
| [schemas/wire](schemas/wire) | JSON Schema for Loam Wire (controller ↔ machines ↔ probes over MQTT) |

## Quick start

```bash
pip install -e ".[plot]"                 # Python ≥ 3.10; numpy (matplotlib only for plots)

loam check   examples/pad_north.loam      # parse + units, references, cycles, capabilities
loam explain examples/pad_north.loam      # the program in plain English
loam compile examples/pad_north.loam -o plan.json    # execution manifest and MQTT topics
loam sim     examples/pad_north.loam --wire messages.jsonl --plot result.png
loam sim     examples/pad_north.loam --open-loop     # same program, one survey, no feedback
```

## What the simulation shows

`examples/study.py` runs the example program with 50 seeds per mode. The simulated soil swells and
compacts differently than the planner initially assumes (net 0.91 vs 1.00 bank-to-fill), and blades
cut with 6 % random error. The table gives means with 95 % bootstrap confidence intervals:

| Pad cells within ±0.1 ft | Open loop (one survey) | Closed loop (survey every 5 min) |
| --- | --- | --- |
| On the true ground | 61.7 % [61.6, 61.7] | 99.7 % [99.7, 99.8] |
| As the program believed | 96.8 % [96.8, 96.8] | 98.5 % [98.5, 98.6] |

The open-loop run reports success while 38 % of the pad is out of tolerance. A 2×2 factorial separates
the causes. Belief error accounts for most of the gap (open loop 93.1 % → 61.7 %); execution noise
accounts for the rest. With correct beliefs and noiseless blades, open loop is unbiased and meets its
completion rule. That control is checked in the test suite, so the gap cannot be an artefact of the
simulator favouring feedback.

![Sensitivity to compaction belief error](results/study.png)

These are results of a simplified model with assumed soil factors, not field measurements; see
[docs/SIMULATOR.md](docs/SIMULATOR.md) for every assumption and limitation.

## Field tools

Loam works with the equipment on a real test site through files and simple streams, with no vendor SDKs:
LandXML next-pass surfaces for **Cat Grade with 3D** (and Trimble, Topcon, Leica), `.agd` design grids for
**OpenGrade3D**, field folders for **AgOpenGPS**, survey CSVs from **Emlid Reach**, NMEA from any RTK
receiver, and LAS/PCD scans from a **DJI Zenmuse L2** or **Livox Mid-360**. See
[docs/FIELD.md](docs/FIELD.md) for the commands, the coordinate setup and what cannot be connected
(Cat Command, closed retrofit kits).

```bash
loam terrain site.loam scan.las --world-origin 600000,3812000          # grid a lidar scan
loam export  site.loam --to landxml -o next.xml --ground scan.las --world-origin 600000,3812000
loam score   site.loam rover.csv --zone pad --world-origin 600000,3812000
```

## Repository layout

| Path | Purpose |
| --- | --- |
| `loam/lexer.py`, `loam/parser.py` | Tokenizer and recursive-descent parser; errors carry line:column |
| `loam/checker.py` | Units and dimensions, references, dependency cycles, capability coverage |
| `loam/evaluate.py` | Evaluates `when` / `until` conditions against a site context (SI units) |
| `loam/compiler.py` | JSON manifest and plain-English explanation |
| `loam/landxml.py` | LandXML TIN reader/writer and rasterizer |
| `loam/sim.py` | Closed-loop reference simulator with a volume ledger |
| `loam/rng.py` | Specified counter-based RNG, reproducible in any language |
| `loam/wire.py` | Loam Wire message builders |
| `loam/grid.py`, `loam/geo.py` | The site height grid; placing it on WGS84 and projected coordinates |
| `loam/pointcloud.py` | Streaming LAS and PCD readers (DJI L2, Livox Mid-360) and gridding |
| `loam/survey.py` | Emlid/RTK survey CSV, NMEA GGA to Wire state, scoring against design |
| `loam/target.py`, `loam/export.py` | Next-pass surface; LandXML (Cat Grade 3D), OpenGrade3D `.agd`, AgOpenGPS field folder |
| `examples/` | Example program and surfaces, comparison plot, Monte Carlo study |
| `tests/` | 108 tests: language, I/O, simulator invariants, field adapters, memory bounds, property-based parser fuzzing |

## Development

```bash
pip install -e ".[dev]"
pytest -q --cov=loam          # 108 tests, about 40 s
ruff check loam tests examples
mypy loam
python examples/study.py      # about 1 min on 2 cores; rewrites results/
```

CI runs the tests on Python 3.10 to 3.13, plus lint, type checks and a reproducibility check of the
example surfaces. Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md).

## Status and safety

v0.0.1 is a first public draft; the language and wire format will change. **Loam does not make a
machine safe.** Machines directed by Loam need their own certified safety systems and a site risk
assessment (ISO 17757). The reference simulator is a test tool, not a planner for real equipment.

## License

Code: [Apache 2.0](LICENSE), which includes an explicit patent grant. Specification: [CC BY 4.0](LICENSE-SPEC).
If you use Loam in research, please cite it (see [CITATION.cff](CITATION.cff)).
