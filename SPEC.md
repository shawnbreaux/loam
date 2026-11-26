# Loam: An Open Language for Programmable Earthmoving

Specification v0.0.1 (draft), 2026-11-25. Licensed under CC BY 4.0 (see `LICENSE-SPEC`).

## Purpose and principles

Loam is an open, vendor-neutral language for writing a program for a whole earthmoving site. It describes the finished ground, the machines and sensors on hand, the work to be done and the safety rules. A planner turns that into passes for each machine, then checks the real ground against the goal from overhead sensing until the site is in tolerance.

It fills the gap between what is already standardized (design surfaces in LandXML and IFC, server-to-server site data in ISO/TS 15143-4) and the proprietary pass planners inside each vendor's autonomy stack.

| CNC | Loam |
| --- | --- |
| CAD part model | `site` + `design` (a LandXML or IFC surface) |
| Stock and fixtures | `zone`, `stockpile`, exclusion areas |
| Machine and tool table | `machine` declarations with capabilities |
| CAM operations | `task` blocks (cut, fill, spread, compact, trench, haul) |
| Touch probe | `probe` (tethered drone lidar, machine-mounted sensors, rovers) |
| Interlocks | `rule` blocks |
| G-code sent to the controller | Loam Wire orders over MQTT |

Design principles:

1. **Goals, not toolpaths.** A program says what the ground must become and the limits on getting there. The planner chooses the passes. Soil swells, shrinks and moves, so fixed toolpaths written in advance would be wrong by the second pass.
2. **Closed loop by default.** Every task ends on a measured condition (`until residual(A) < 0.2 ft`), never on a pass count alone.
3. **Units are mandatory.** Every length, rate and volume carries its unit. A bare number where a quantity is expected is an error.
4. **Readable by an operator.** A foreman should be able to read a `.loam` file and say what the site will do.
5. **Safety is layered, never delegated.** Machine-level safety and hardware e-stops always win. Loam rules add site-level holds on top and can never switch a machine's own protections off.
6. **Vendor-neutral.** Any machine that speaks Loam Wire can take orders, whatever its brand or autonomy kit.

License: the specification is published under CC BY 4.0 and the reference implementation under Apache 2.0, whose explicit patent grant and patent-retaliation clause protect implementers. Publishing the spec openly, with a date, also puts the ideas on the public record as prior art.

## Core model

A Loam program is a set of declarations. Order in the file doesn't matter; dependencies are stated with `after`.

| Declaration | What it describes | Key fields |
| --- | --- | --- |
| `site` | The job: coordinate system, units, finished surface, boundary | `crs`, `units`, `datum`, `design`, `existing`, `boundary`, `grid` |
| `probe` | A source of measured ground surface | `kind`, `sensor`, `rate`, `accuracy`, `coverage`, `stale_after`, `endpoint` |
| `machine` | One physical machine and what it can do | `class`, `caps`, tool size (`blade`, `bucket`, `drum`, `capacity`, `width`), `max_speed`, `endpoint` |
| `zone` | An area of the site with a role | `role` (cut, fill, finish, exclusion), `area`, `tolerance`, `lift`, `compaction`, `buffer` |
| `stockpile` | A place to put or take material | `area`, `capacity`, `available`, `material` |
| `task` | Work to be done, ending on a measured condition | `op`, `zone`, `by`, `max_cut`, `lift`, `passes`, `depth`, `along`, `spoil`, `source`, `after`, `until`, `priority` |
| `rule` | A site-level safety or quality condition and its response | `when`, `do` |
| `replan` | When the planner must recompute passes | `when` |

**The ground is a height grid.** The site is a raster of cells (`grid 1 ft` by default). Each cell holds three values: `design` (target elevation), `actual` (latest probe measurement) and `residual = actual - design`. A positive residual means material still to remove; a negative one means material still to place.

**Design surfaces need not cover the site.** Outside the design surface the design is the existing ground, so no work is wanted there; the existing surface must cover the whole site grid.

**Zones can be drawn or derived.** A zone is either a polygon (`area rect (...) (...)`, `area polygon [...]` or `area file "x.geojson"`) or computed from the grid (`area where residual > 0.1 ft`). Derived zones are re-evaluated after every probe update, so a cut zone shrinks as it is cut.

**Machines are matched by capability.** A task names a class (`by class dozer`), a specific machine (`by dz1`) or a capability (`by cap compact`). The planner assigns any idle machine that matches.

**Tasks are goals with limits.** A task's `until` is the done condition. Its limits (`max_cut 0.5 ft per pass`, `lift 8 in`, `passes 4`) bound how each pass may be planned. The planner may split a task into as many passes as it needs. `spoil -> [C, sp1]` lists destinations in order: fill `C` first, overflow to `sp1`. `after T start` waits for task `T` to begin; `after T` or `after T done` waits for it to finish.

## Syntax reference

A file starts with `loam 0.0.1`, then declarations of the form `kind name { field value ... }`. Fields end at a newline or `;`, so one-line blocks use semicolons: `machine dz1 { class dozer; blade 12 ft }`. Comments start with `#`.

```
program    = header { decl } ;
header     = "loam" VERSION ;          (* loam 0.0.1 or loam 0.0.2 *)
decl       = KIND NAME "{" { stmt ( NEWLINE | ";" ) } "}" ;
KIND       = "site" | "probe" | "machine" | "zone" | "stockpile"
           | "task" | "rule" | "replan" ;
stmt       = KEY ( expr                       (* when, until *)
                 | "where" expr               (* area where ... *)
                 | { atom } ) ;
atom       = quantity | NUMBER | STRING | IDENT | list | point | "->" atom ;
quantity   = [ "±" ] NUMBER UNIT ;
list       = "[" [ atom { "," atom } ] "]" ;
point      = "(" NUMBER "," NUMBER ")" ;
expr       = or ;  or = and { "or" and } ;  and = not { "and" not } ;
not        = "not" not | cmp ;
cmp        = sum [ ( "<" | "<=" | ">" | ">=" | "==" | "!=" ) sum ] ;
sum        = prod { ( "+" | "-" ) prod } ;
prod       = unary { ( "*" | "/" ) unary } ;
postfix    = primary { "(" args ")" | "." IDENT } ;
```

**Units.** Every quantity carries a unit, and the checker rejects a bare number where a quantity belongs. Coordinates in points are the exception: they are always in the site's units and CRS.

| Dimension | Units |
| --- | --- |
| Length | `mm` `cm` `m` `in` `ft` `usft` (US survey foot) `yd` |
| Area | `ft2` `yd2` `m2` `ac` |
| Volume | `ft3` `yd3` `m3` |
| Time | `s` `min` `h` |
| Speed | `mph` `kmh` |
| Rate | any length/time, e.g. `in/h` |
| Ratio | `%` |

**Areas.** `rect (x1, y1) (x2, y2)`, `polygon [(x, y), ...]`, `file "area.geojson"`, or `where <cell condition>` using the cell values `residual`, `design` and `actual`.

**Condition functions.** These are the only names a `when` or `until` condition may use.

| Function | Returns | Meaning |
| --- | --- | --- |
| `residual(Z)` | length | 95th percentile of \|actual − design\| in zone Z. Also `.high` (95th percentile, signed), `.low` (5th percentile, signed), `.max`, `.min`, `.mean` |
| `in_tolerance(Z)` | true/false | `residual(Z)` is within Z's ± tolerance |
| `volume_remaining(Z)` | volume | Absolute cut plus fill still to do in Z |
| `compaction(Z)` | ratio | Measured compaction of placed fill in Z |
| `age(P)` | time | Time since probe P last delivered a surface |
| `proximity(a, b)` | length | Closest distance between `person`, `machine`, `vehicle` or named things |
| `weather(rain \| wind)` | rate or speed | Current site weather reading |
| `done(T)`, `progress(T)` | true/false, ratio | Task state |
| `deviation()` | length | 95th percentile difference between the measured and predicted surface over ground worked since the last survey |
| `elapsed()` | time | Time since the program started |

Comparisons convert both sides to SI and use a relative epsilon of 1e-9, so `compaction(C) >= 95 %` behaves as written.

**Actions** (rules only): `stop`, `hold`, `pause`, `resume`, `slow <speed>`, `alert "message"`. Each takes a target: `all`, a machine name, `task <name>`, `class <class>` or a list.

**Checker guarantees.** A program that passes `loam check` has: exactly one site; no duplicate names; every reference resolved to the right kind; every quantity in the right dimension; true/false conditions in every `when` and `until`; no `after ... done` cycles; and at least one declared machine able to do each task.

## Closed-loop execution

A Loam runtime repeats one loop for every new survey, and every decision in it is made from measured ground, not from what the plan expected.

```
 1 Probe surveys ──► 2 Compare to design ──► 3 Apply rules
       ▲                                          │
       │ machines move the earth                  ▼
 6 Send orders ◄── 5 Replan if needed ◄── 4 Close finished tasks
```

Rules run before anything else in each cycle, so a safety stop is never queued behind planning work.

1. **Measured, not predicted.** `until`, `when` and `replan` conditions are evaluated on the latest probe surface. The planner may keep a predicted surface between surveys to plan passes, but it never closes a task on a prediction.
2. **Eligible and satisfied means done.** A task is done when its `after` dependencies are met and its `until` holds, even if it never ran a pass. A fill zone that is already at grade needs no work.
3. **Replan on drift.** When a `replan` condition holds, typically `deviation() > 0.15 ft`, the planner rebases its prediction on the measured surface. It also re-estimates material factors (swell, compaction shrink) from ground worked since the last survey.
4. **Replan on stall.** If a task is eligible and not done, but the planner has no work for it, the runtime must rebase on the latest survey before trying again. This is built in and needs no declaration.
5. **Stale data stops work.** If no probe has delivered within its `stale_after` time (default 3 × `rate`), a conforming runtime must hold new orders, whether or not the program has a rule for it.
6. **Orders are small.** Each order covers one pass in one work window, so a replan never has to unwind a long queued sequence.

## Loam Wire: machine messages

Loam Wire is the JSON-over-MQTT layer that carries orders to machines and state back, so any machine with a Loam Wire adapter can join a site. Its shape follows VDA 5050 (orders, state and connection over MQTT), with the node-and-edge route swapped for a work window and a target surface. The JSON Schema is in `schemas/wire/loam-wire-0.0.1.schema.json`.

| Topic | Direction | Purpose |
| --- | --- | --- |
| `loam/v0/<site>/<machine>/order` | controller → machine | One pass of work: task, op, window, target, limits, where spoil goes |
| `loam/v0/<site>/<machine>/instantAction` | controller → machine | `stop`, `hold`, `resume`, `slow`, applied immediately, ahead of any order |
| `loam/v0/<site>/<machine>/state` | machine → controller | Pose, tool pose, progress, safety state, errors, at 1 Hz or faster |
| `loam/v0/<site>/<machine>/connection` | machine → controller | `online`, `offline`, `connectionBroken`, retained and set as the MQTT last will |
| `loam/v0/<site>/terrain` | probe → controller | Measured surface tiles: origin, cell size, shape, σ, encoded heights |
| `loam/v0/<site>/safety` | controller → all | Site-wide stop or hold, plus a 1 Hz heartbeat |
| `loam/v0/<site>/estop` | e-stop radio → all | `pressed` true/false. Machines stop at once without waiting for the controller. Releasing it restarts nothing: machines stay stopped (`estopReset`) until a `resume` (ISO 13850) |
| `loam/v0/<site>/detection` | site sensors → controller | A person or vehicle seen at (x, y); `kind: "clear"` ends it. Feeds `proximity()` |
| `loam/v0/<site>/weather` | site sensors → controller | Rain rate (m/s). Feeds `weather(rain)` |
| `loam/v0/<site>/status` | controller → monitors | Tasks, machines, active rules, replans, measured % in tolerance (retained) |

Conventions:

- Positions are in the site CRS and units. Every other quantity is SI: metres, seconds, m/s, m³.
- `pose.heading` is in radians, counter-clockwise from the site +x axis (the VDA 5050 convention). A state may add `pose.z` (site units) and a `gnss` object (`fix`, `sats`, `hdop`, `age`).
- A new `orderId` replaces the machine's current order; a higher `orderUpdateId` amends it. A machine never executes an `orderId` twice, and after losing its link it drops the current order rather than resume it unasked.
- **Targets and spread patterns.** Cut and finish orders carry `target.pattern`: the planned height change per grid cell over the window (metres, negative = cut, `float32-le-base64`, row-major from the window's low corner, `target.shape` = rows, cols). Grade control holds the blade to it and never cuts below design. Deposits carry `deposit.pattern` the same way: the planned loose thickness per cell, with `volume` (m³ in the pattern), `planned` (all loose spoil of a cut, m³; the rest goes to `overflow`) or `bank` (m³ to take from `source`).
- **Done means done.** A machine reports `progress: 1.0` for an order only once the work has really been done, and stamps that state with the time it finished. The controller matches each survey's timestamp against these, so a survey taken before a pass finished never counts that pass twice.
- Timestamps are ISO 8601 UTC with optional milliseconds (`2026-12-01T07:00:00.250Z`).
- **Fail-safe heartbeat:** a machine that hears no `safety` heartbeat for 3 s stops on its own and reports `safetyState: "stopped"`.
- Every message carries `version`, `headerId` (increasing per sender) and a UTC `timestamp`.

Example order:

```json
{
  "version": "0.0.1", "headerId": 2, "timestamp": "2026-12-01T07:00:00Z",
  "site": "pad_north", "machine": "dz1",
  "orderId": "strip_A-1", "orderUpdateId": 0,
  "task": "strip_A", "op": "cut",
  "window": [[0, 0], [30, 0], [30, 12], [0, 12]],
  "target": {"surface": "design"},
  "limits": {"maxCut": 0.1524, "maxSpeed": 1.341},
  "deposit": {"zone": "C", "lift": 0.2032}
}
```

## Safety model

Loam is a planning and coordination layer. It adds site-level holds on top of a machine's own safety system and can never weaken that system.

| Layer | Owner | Can Loam override it? |
| --- | --- | --- |
| Hardware e-stop, all-stop and remote stop | Machine and site hardware | Never |
| Machine perception and obstacle stop | Autonomy kit or OEM | Never |
| Fail-safe heartbeat (stop after 3 s of silence) | Loam Wire adapter on the machine | Never |
| Built-in runtime holds (stale probe data) | Loam runtime | Never; programs cannot remove them |
| Program `rule` blocks | Site program author | They only add restrictions |

Requirements on a conforming runtime:

1. Rules are evaluated every cycle, before task checks or planning, and on every new machine state or probe message.
2. `stop` sends an `instantAction` to each targeted machine at once. When the condition clears, the runtime sends `resume`; it never resends a stale order.
3. There is no `do` action that disables a stop, raises a speed limit above a machine's `max_speed`, or enters an `exclusion` zone.
4. Exclusion zones (`role exclusion`, with an optional `buffer`) are never placed inside an order's work window or travel path.
5. Every rule activation, clearance and instant action is logged with its timestamp, for after-incident review.

The target safety standard for machines and systems built on Loam is [ISO 17757:2019](https://www.iso.org/standard/76126.html) (earth-moving and mining, autonomous and semi-autonomous machine system safety). Loam does not certify anything by itself; a site using Loam still needs the risk assessment that standard calls for.

## Worked example

See `examples/pad_north.loam`: a 160 × 100 ft building pad with two dozers, a roller, a tethered lidar
drone and a stockpile. `examples/study.py` runs it 50 times in each mode, against soil that swells 30 %
and keeps 70 % of its loose thickness when rolled, while the planner starts out assuming 25 % and 80 %.
Figures are means with 95 % bootstrap confidence intervals; full results are in `results/study.md`.

| Pad within ±0.1 ft | Open loop (one survey) | Closed loop (survey every 5 min) |
| --- | --- | --- |
| As the program saw it | 96.8 % [96.8, 96.8] | 98.5 % [98.5, 98.6] |
| True ground | 61.7 % [61.6, 61.7] | 99.7 % [99.7, 99.8] |

The model and its limitations are documented in `docs/SIMULATOR.md`. The numbers describe how the loop
behaves under that model, not field performance.

## Interoperability and roadmap

| Standard | How Loam uses it |
| --- | --- |
| [LandXML](https://highways.dot.gov/federal-lands/pddm/dpg/data-exchange-standards) | Design and existing surfaces (`format landxml`); the reference tool reads TIN surfaces |
| [IFC 4.3](https://www.autodesk.com/blogs/aec/2023/05/12/whats-new-in-openbim-and-infrastructure-ifc-4-3-for-civil-3d/) | Design and existing surfaces (`format ifc`): triangulated face sets and TINs, through IfcOpenShell |
| GeoTIFF | Surfaces (`format geotiff`) and measured ground (an OpenDroneMap DTM, say); `loam export --to geotiff` |
| GeoJSON | Areas (`area file "x.geojson"`), drawn in QGIS; `loam export --to geojson` |
| [VDA 5050 3.0](https://github.com/VDA5050/VDA5050) | Template for Loam Wire, and `loam vda5050` drives VDA 5050 robots as Loam machines |
| [MAVLink 2](https://mavlink.io/) | `loam mavlink` drives ArduPilot Rover and other MAVLink ground vehicles as Loam machines |
| [OPERA](https://github.com/pwri-opera) (ROS 2) | `loam opera` drives OPERA machines: Nav2 carriers and the ZX200 excavator |
| [ISO/TS 15143-4:2025](https://www.iso.org/standard/74485.html) | Server-to-server design and as-built exchange. It excludes machine-to-server and machine-to-machine traffic; Loam Wire covers that |
| [ISO 23725:2024](https://www.iso.org/standard/76769.html) | Fleet-to-autonomy dispatch for mining haul trucks; a model for Loam's task assignment |
| [ISO 17757:2019](https://www.iso.org/standard/76126.html) | Safety requirements for the autonomous machines Loam directs |

Roadmap:

- [ ] **0.0.2 language:** trench and haul tasks in the simulator, multiple probes fused by accuracy, machine-mounted probes, `deposit` windows
- [x] **0.0.3 tooling:** IFC 4.3 and GeoTIFF surfaces, LAZ point clouds with ground filtering, CRSs, GeoJSON areas
- [x] **Runtime:** a reference MQTT controller that runs a `.loam` file against Loam Wire machines; MAVLink and VDA 5050 bridges
- [ ] **Tooling:** Assembly port (x86-64 and ARM64), a `loam fmt` formatter, editor syntax highlighting
- [x] **0.0.4 bridges:** OPERA on ROS 2 (Nav2 carriers, ZX200 excavator), Project Chrono soil physics
- [ ] **Conformance suite:** test programs and message traces any runtime or machine adapter must pass
- [ ] **Governance:** public repository, an RFC process for language changes, versioned spec releases

On patents: the Apache 2.0 license gives every user an explicit patent license from each contributor, and it ends that license for anyone who sues over the code. Publishing this spec with a date also puts its methods on the public record as prior art. Specific implementations can still be patented by others, so have a patent attorney review this before you rely on it.
