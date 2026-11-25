# Changelog

All notable changes are recorded here. The project follows [Semantic Versioning](https://semver.org/);
before 1.0 any minor release may change the language.

## [Unreleased]

Field adapters for testing on real equipment (see docs/FIELD.md).

- `loam export --to landxml | agd | agopengps`: next-pass target surface (or the design) for Cat Grade
  with 3D and other machine control, OpenGrade3D design grids, and AgOpenGPS field folders.
- `loam terrain`: streaming LAS 1.0-1.4 and PCD readers (DJI Zenmuse L2, Livox Mid-360) gridded onto
  the site, as a Wire terrain message or an as-built LandXML surface. Memory stays near 25 MB.
- `loam score`: Emlid Flow / RTK survey CSVs scored against the design, with a bootstrap 95 % interval.
- `loam nmea`: NMEA GGA from an RTK receiver to Wire `state` messages; non-fixed solutions are flagged.
- Site placement on WGS84 and projected grids (`--origin`, `--world-origin`, `--rotation`), matching
  AgOpenGPS's local plane.
- `usft` (US survey foot) length unit. Wire `pose.heading` convention documented.
- The simulator's grid code moved to `loam/grid.py`; simulator output is byte-identical.

## [0.0.1] - 2026-11-25

First public draft.

- Language: `site`, `probe`, `machine`, `zone`, `stockpile`, `task`, `rule` and `replan` declarations;
  mandatory units; measured completion conditions (`until`); safety rules.
- Checker: units and dimensions, references, dependency cycles, capability coverage; diagnostics with
  line and column, in source order.
- Loam Wire 0.0.1: JSON Schema for order, instantAction, state, connection, terrain and safety messages.
- Reference simulator: in-flight passes that land on completion, a volume ledger, soil-factor
  calibration on replan, built-in stale-data hold and stall replan, exclusion zones.
- Portable counter-based RNG (SplitMix64, Irwin-Hall normals) with pinned reference values.
- Monte Carlo study (`examples/study.py`) with bootstrap confidence intervals, a 2×2 factorial
  and a sensitivity sweep.
