# Changelog

All notable changes are recorded here. The project follows [Semantic Versioning](https://semver.org/);
before 1.0 any minor release may change the language.

## [0.0.5] - 2026-11-26

A review for leaks and errors, and CI fixed. CI had failed since 0.0.3: an unused `type: ignore` in
the lint job, and coverage at 77 % once the integration tests (ArduPilot, ROS 2, Chrono) skip there.

Safety
- Releasing the site e-stop no longer restarts machines (ISO 13850). They stay stopped (`estopReset`)
  until a deliberate resume. The new `loam resume` command issues it, site-wide or for one machine. A
  simulated site's fault script resumes `resume_after_min` after release.
- Bridges and site processes run their cleanup on SIGTERM (systemd, Docker): the vehicle is held, its
  goal cancelled and the machine reported offline. Before, Python's default action killed them on
  the spot.
- A real e-stop radio message (`pressed` only, no simulation timestamp) no longer crashes a bridge.
- Malformed messages on the broker are logged and skipped by every site process; before, one bad
  message could crash a machine, the controller, the world or the monitor.
- The stop latency is measured after the vehicle's stop command, not before.
- An order a vehicle cannot run is now a first-class failure (`orderFailed`). Before:
  - ArduPilot refusing a mission upload left the previous, finished mission's counters in place, so the
    new order was reported done without the vehicle moving;
  - a VDA 5050 robot that never took an order sat as "driving, ok, 0 %" for good;
  - a failed OPERA order left the machine "stopped" until an order that could never come.
  Now the bridge holds the vehicle, reports the failure once and drops the order; the controller
  withdraws the pass at once and re-plans, with a one-minute cooldown for that machine.
- Bridge shutdown finishes (machine reported offline, vehicle link closed) even if the last command to
  a dead vehicle link raises.

Wire
- State errors are `{code, level, text}`, as the schema always said; the code had been sending
  `{code, detail}`. The schema now rejects unknown error fields, and a test holds the two together.

Leaks and errors
- MQTT: a failed broker connection left paho's network thread and sockets behind, and a refusal
  raised a bare `ConnectionRefusedError`; both now exit cleanly. Waiting for the site clock no longer
  reorders the messages that arrived meanwhile.
- The monitor: it now shuts down at the end of a simulated site (it never subscribed to it). Live
  dashboard connections are woken and closed on shutdown, with a 15 s keep-alive to notice vanished
  browsers. The listening socket is closed, and nothing is left behind when the port is busy.
  `--host` lets a remote Prometheus scrape `/metrics`. Replay of long recordings is no longer quadratic.
- Site launcher: every child process is reaped (no zombies), and none is left running if start-up
  fails part-way.
- Bridges: the vehicle connection is closed on every exit path (no broker, no clock, unknown
  machine, no MAVLink heartbeat).
- OPERA: the ROS executor thread stops before its context is shut down, and Loam keeps
  SIGINT/SIGTERM. An aborted Nav2 goal is retried 3 times, 2 s apart, before the order fails, since
  Nav2 also aborts while it is still starting up.
- Controller: incoming messages were handled with the clock one loop behind (it was read only after
  them), so log stamps and time-based decisions lagged by a planning cycle. Found by CI on Python 3.13.
- CLI: `soil-calibrate` and `resume` always printed JSON; they now honour `--json`.

Tests and CI
- Every test fails if it leaves a thread, child process or open file behind (tests/conftest.py).
- New offline tests:
  - the e-stop latch;
  - malformed traffic and message order;
  - the whole MAVLink path against a fake ArduPilot written with pymavlink;
  - SIGTERM holding the vehicle, checked against the unfixed code;
  - OPERA's goal handling with ROS stand-ins (the races met on live Nav2, replayed exactly);
  - refused orders on MAVLink and VDA 5050, and the controller withdrawing them, each seen to fail
    on the unfixed code first;
  - the monitor and `loam resume`.
- CI: actions checkout and setup-python v7 (Node 24), `apt-get update` before installing Mosquitto,
  RTKLIB pinned to the tested commit, job timeouts, least-privilege permissions, superseded runs
  cancelled, pip cache. Coverage in CI is 86 %; with every integration live it is 89 %.
- Packaging: SPDX licence (`Apache-2.0`) and `license-files`, replacing the deprecated table.

Code comments
- All the code is commented for outside readers: every module, class and function says what it is
  for, with units, and the reasoning behind non-obvious steps and constants is written beside them.
  That covers the package, the dashboard page, the tests (each says what it proves), the examples
  and the setup script. No behaviour changed: for every Python file, the syntax tree with docstrings
  removed is identical to the one before.

## [0.0.4] - 2026-11-26

Project Chrono soil physics and OPERA machines on ROS 2. Both are FOSS; see docs/INTEGRATIONS.md.

- `loam opera`: PWRI's OPERA machines as Loam machines.
  - Carriers (IC120, or any Nav2 machine): work passes go to Nav2's `follow_path` so the machine holds the
    work line; travel goes to `navigate_through_poses`. With `--dump`, the vessel is raised only once the
    machine stands still.
  - The ZX200 excavator cuts to the order's target surface: inverse kinematics from its own URDF, a
    trajectory for its ros2_control arm controller, never below design.
  - Measured: the ZX200 held the grade to 0.3 mm RMS and never went as much as 1 mm below it, on
    ros2_control. An IC120 on Nav2 Jazzy held the work line to 1.2 cm RMS at its 0.8 m/s limit.
- `loam soil-calibrate`: roller compaction per pass from Project Chrono's SCM soil model. A towed drum
  stays within 1.25–1.31x of the Bekker–Wong closed form, mesh-converged. `simulate-site --soil-table`
  makes the simulated world compact loose fill by that curve.
- Bridges: no "ok" before the vehicle has reported a position (`vehicleNoPose`). A vehicle's action
  server that is missing or not yet active is waited for and retried (reported after 10 s), never
  turned into a failed order. MAVLink: positions are ignored until ArduPilot's EKF has an origin.
- `tools/integration-envs.sh`: conda environments for Chrono and ROS 2, plus PWRI's OPERA packages.

## [0.0.3] - 2026-11-25

Open-source integrations, all optional and all under OSI-approved licences (see docs/INTEGRATIONS.md).
The core still needs numpy only; the language stays at 0.0.2 (`format geotiff` is a new value).

- Vehicles: `loam mavlink` drives ArduPilot Rover (any MAVLink 2 ground vehicle) as a Loam machine:
  orders become missions, stop is HOLD, e-stop is a forced disarm, GNSS fix is gated. Tested against
  ArduPilot Rover SITL: 1.17 ft from the end of the work line, held within 0.62 ft of it, 9 ms e-stop.
  `loam vda5050` is the VDA 5050 3.0 fleet-control side for one robot, checked against the official
  schemas. Both share one bridge loop, so the safety rules are those of the simulated machines.
- Positioning: `loam nmea` reads `tcp://host:port` (RTKLIB str2str/rtkrcv, RTKBase, receivers) and
  publishes to MQTT with `--broker`. Tested through RTKLIB's str2str.
- Survey data: LAZ (laspy, lazrs); `--ground-filter`, a progressive morphological ground filter with
  harmonic infill (2 cm on clean ground, machines removed); OpenDroneMap project folders; GeoTIFF DEMs.
- Coordinates: `--crs` and `--source-crs` (pyproj): real projections, reprojection of clouds and
  rasters, heights too with compound CRSs.
- Design data: IFC 4.3 surfaces (IfcOpenShell), GeoTIFF surfaces, GeoJSON area files (QGIS);
  `loam export --to geotiff | geojson`, georeferenced and rotated as the site is.
- `loam monitor` serves Prometheus `/metrics` (Grafana): machines, tasks, safety, survey age, share
  of measured terrain in tolerance.
- Fixes: MQTT clients now release paho's wake-up sockets on stop (a machine reconnecting on a flaky
  radio no longer accumulates descriptors); `export --surface design` no longer needs measured ground.

## [0.0.2] - 2026-11-25

From a grading file to a running, monitored site, tested end to end over MQTT (see docs/RUNTIME.md).

- `loam plan`: writes a checked Loam program from the LandXML a project manager receives (existing
  and finished-grade surfaces, one file or two): job limits traced from the design surface, derived cut
  and fill zones, earthwork balance, stockpile, tasks, safety rules. Assumptions are marked `CHECK`.
- `loam run`: the site controller over MQTT. Same planner as the reference simulator; surveys come from
  terrain messages, passes land when machines report them done, and survey and completion times are
  reconciled so no pass counts twice. Withdraws passes from machines that drop off.
- `loam simulate-site`: a world, a lidar probe and one process per machine speaking Loam Wire, with
  scripted faults (person near a machine, lidar outage, rain, GNSS float, radio loss, e-stop, crash).
- `loam monitor`: live web dashboard (measured-vs-design map, machines, tasks, events) and
  self-contained replay pages of recorded runs.
- Wire: `estop`, `detection`, `weather` and `status` topics; per-cell `target.pattern` for cut and finish
  orders and `deposit.pattern` for spoil and fill; millisecond timestamps; idempotent orders.
- Planner fixes found by the end-to-end tests: a roller compacts everything under its window; no new
  passes once a task's `until` already holds on measured ground. Study numbers regenerated.
- Language 0.0.2: `usft` unit; design surfaces may stop short of the site (outside them, no work).
- Field adapters for testing on real equipment (see docs/FIELD.md):
  - `loam export --to landxml | agd | agopengps`: next-pass target surface (or the design) for Cat
    Grade with 3D and other machine control, OpenGrade3D design grids, and AgOpenGPS field folders.
  - `loam terrain`: streaming LAS 1.0-1.4 and PCD readers (DJI Zenmuse L2, Livox Mid-360) gridded onto
    the site, as a Wire terrain message or an as-built LandXML surface. Memory stays near 25 MB.
  - `loam score`: Emlid Flow / RTK survey CSVs scored against the design, with a bootstrap 95 % interval.
  - `loam nmea`: NMEA GGA from an RTK receiver to Wire `state` messages; non-fixed solutions are flagged.
  - Site placement on WGS84 and projected grids (`--origin`, `--world-origin`, `--rotation`), matching
    AgOpenGPS's local plane.
- The simulator's grid code moved to `loam/grid.py`.

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
