# Changelog

All notable changes are recorded here. The project follows [Semantic Versioning](https://semver.org/);
before 1.0 any minor release may change the language.

## [0.0.11] - 2026-11-28

Added
- **`loam blade`: a grade box that works alone, beside AgOpenGPS.** It reads an RTK antenna on the blade,
  works out how far the cutting edge is above the surface the next pass should leave, and sends that to an
  OpenGrade valve board: the Nano board on USB or OpenGrade3D's Teensy board on the network. No controller,
  broker or survey is needed: the ground is followed from the blade's own cuts, one `max_cut` a pass, down to
  the design. Without an RTK-fixed position the blade is lifted and left alone. `--with-agopengps` grades
  only while AgOpenGPS says it is steering.
- **Valves tuned by measurement.** `loam blade --calibrate` pulses the valve with the blade in the air,
  measures where it starts to move, its speed, its delay and the antenna's noise, and sets the board's eight
  numbers from them. It recognises an open-and-shut valve, swapped coils, a work switch left off, and a valve
  too quick for the board.
- **Tilt.** A second valve board on the tilt cylinder (`--tilt-valve`) lays the edge on the cross slope of the
  wanted surface, which the sides of a swale need.
- **The AgOpenGPS family's messages** (`loam/aog.py`): the network frame, what AgOpenGPS says while it steers,
  the `$PANDA` and `$PAOGI` position sentences, and both OpenGrade valve boards. `loam nmea` reads `$PANDA` and
  `$PAOGI`, uses the heading they carry, and listens on UDP (`udp://:9999`). Positions arriving on one port
  from two senders (a tractor's GPS board and a blade's) are refused until the one to hear is named
  (`udp://SENDER:PORT`), never mixed.
- New optional extra `serial` (pyserial, BSD-3-Clause), for a valve board or receiver on a serial port.

Fixed
- **Loam Wire schema: the `gnss` object of a state.** The specification allowed it and `loam nmea` sent it,
  but the published JSON Schema refused it. The schema now defines it. A value the receiver left empty is
  sent as `null`; it was `NaN`, which is not JSON.
- **A blade left one microsecond short of raised.** `loam mavlink --blade-servo` does not send a step smaller
  than two microseconds, and that included the last step to the top of the stroke. The step that reaches an
  end of the stroke is now always sent.

Not yet tested on hardware: `loam blade` has run only against stand-ins for the boards, a simulated valve and
cylinder, and a simulated antenna.

## [0.0.10] - 2026-11-27

Added
- **Work along a line.** `along [(x, y), ...]` on a cut or finish task is now planned: each pass is a lane (the
  line, or the line moved a whole number of tool widths to one side), a machine works its way along the line
  and comes back along it, and a lane stops short of an exclusion. The line says where to drive; the measured
  ground still says how deep each pass cuts and when the task is done. Until now `along` was read and ignored.
- **Loam Wire: `path` and `swath` on an order.** The line the middle of the tool follows, and the width the
  pass works. Every machine and bridge follows a path it is given; an order without one is driven as before.
- **`loam swale`.** A drainage swale from the ground and a line drawn where the water should go: the bottom is
  the highest line that drains (a least depth, a least fall), with a flat bottom, sloped sides and rounded
  ends. The line may be a path from Google Earth, a GeoJSON LineString, a shapefile polyline or typed points.
  The program's passes follow the centreline.
- **AgOpenGPS curve tracks.** `loam export --to agopengps` writes a task's line as a curve track, laid out as
  AgOpenGPS (source of 2026-11-14) writes a curve drawn on its own map.
- The blade controller takes the blade's width from the program and holds a level edge at the highest point
  wanted under it. On a pass along a line the blade is down only within half a tool width of the path.

Fixed
- **A pass ordered twice after a stale survey.** When a task finished and the next could start, the planner
  rebased its prediction on the latest survey even if that survey was taken before the last pass landed. The
  survey still showed the ground that pass had cut, so its work was planned again: in the simulated site a
  finishing pass then went a third of a foot below design. The planner now rebases only on a survey taken
  since the last pass landed, and a task that has not begun waits for one. Found while testing swales; it
  affected every program.
- The simulated site's finishing passes stop at design, as the specification says grade control does.

Not built
- Blade tilt. The sides of a swale need a blade that tilts or a machine with its own grade control. Loam's
  blade controller moves one actuator. Neither OpenGrade3D nor ArduPilot's u-blox driver offers what a tilt
  loop needs (a tilt output; the blade's roll from two antennas), so there is nothing sound to build on yet.

## [0.0.9] - 2026-11-27

Added
- **A blade held with nobody aboard.** `loam mavlink --blade-servo N` has the bridge hold a vehicle's blade to
  the next-pass surface through one servo output, with the height from the autopilot's own position
  (`--blade-zero` measures the geometry), or from its second GPS when that antenna rides on the blade
  (`--blade-gps2`). Until now a person had to load each surface into a grade-control system.
- **`loam conform`.** Plays the controller for one machine and checks eight promises of Loam Wire: announces,
  reports, ready, works an order, never repeats one, stops, stops when the controller falls silent, and stays
  stopped when the e-stop is released. For any machine, whoever built it.
- **`loam nav2`.** Any ROS 2 vehicle that runs Nav2 as a Loam machine (the carrier of `loam opera`, under its
  own name).
- docs/FIELD.md: "Connecting a machine", with the four ways in and which machines on the market are open.
- **People located by what they carry.** `loam sense --tags people.json` reports each listed person as a circle
  from the phone's GPS (OwnTracks), the Wi-Fi signal the site's access points hear (OpenWrt's hostapd table, or
  single readings), or a Meshtastic tracker. The circle grows at walking pace while the device is silent. An
  access point on a machine works as a proximity sensor. `loam wifi-fit` measures the signal model from a walk,
  and the model is also learnt on site from people whose place is known.
- **Phones matched with lidar and cameras.** A camera with ground calibration points places each person it
  sees on the site; `loam terrain --objects` publishes the person-sized things in each scan; any other detector
  can publish to `sightings/<name>`. Every sighting is a keep-out. On a site declared `everyone_tagged`, a
  phone's wide circle becomes the place its owner is seen, and a phone heard where nobody is seen, or more
  phones than people seen, keeps the wide circle.
- **`loam level`.** A levelling design from the ground alone: a best-fit plane within slope limits, cut
  balanced against fill, optionally blended into the ground at the edge. Ground from LandXML or a GeoTIFF
  elevation model (USGS 3DEP, OpenDroneMap); boundary from KML, KMZ, shapefile or GeoJSON.
- `loam plan` writes programs without compaction (`compaction=False`, used for cropland) and for fleets without
  dozers (tasks by capability).
- Shapefile area files (`area file "zone.shp"`, optional extra `shp`).
- Broker: a `trackers` account for phones and access points, which can report positions and nothing else.

Changed
- Reading a GeoTIFF reads only the block of pixels needed, not the whole tile (a 3DEP tile is 100 million
  pixels).
- The stale-data rule `loam plan` writes follows the probe's own `stale_after`.

Fixed
- **A machine ran an order twice if it was sent twice.** The spec says a machine never executes an order id
  twice, and MQTT delivers "at least once". The simulated machines and the bridges forgot an order once it was
  done, so a repeated message sent them over the same ground again. They now remember the ids they have taken.
  Found by `loam conform`.

Found while testing
- Wi-Fi signal strength cannot place a person across a site: three access points 120 m apart leave a circle
  about 170 m in radius. It is tight only close to an access point.
- The default Wi-Fi model is a guess. On a simulated site where signal carries farther than it assumes, its
  circles held the phone 8 % of the time; the model learnt on that site, 99 %.

## [0.0.8] - 2026-11-27

Found by running a complete job on the example site as separate processes over a broker, for up to
36 minutes at a time, while sampling every process, restarting the broker and the controller in the
middle, and checking every recorded message against the schema. See docs/RUNTIME.md, section 6.

Fixed
- **A machine stayed written off after a reconnect.** When its link dropped, the broker reported it as
  `connectionBroken`; when it reconnected, nothing said `online` again, so the controller never gave it
  another order. After a broker restart no machine came back and the job never finished. Machines now
  announce themselves on every connect.
- **A restarted controller reused its order ids.** Machines, which never execute an id twice, reported the
  new orders finished without doing them (56 passes in one run). Every run of the controller now tags its
  ids with its start time.
- **A lost "clear" held the site for good.** A detection, a "rain stopped" or an e-stop release published
  while a subscriber was reconnecting never reached it. `loam sense` now repeats everything it sees every
  5 s, `clear` included, and withdraws every stale detection when it starts or reconnects; the simulated
  sensors do the same, and the simulated e-stop is retained.
- **Soil factors swung to their limits when surveys arrived late.** A pass that finished after a survey
  was taken kept its place in the plan but lost the fill and rolling it was expected to show, so the next
  survey blamed the soil. With surveys two minutes late: 12 replans down to 5, and no estimate at a limit
  (was 6 of 26).
- **The Wire schema did not match the messages.** It listed five safety states of which only two are
  used, lacked `degraded` and `stopLatencyS`, and had no definition for `status`: 405 of 6,116 recorded
  messages were invalid. The schema is corrected, and the end-to-end test now validates every message.
- **Unbounded memory while the broker is unreachable.** Messages waiting to be sent grew without limit
  (37 MB to 69 MB in a simulated five-hour outage, and rising). Now capped at 2,000 messages.
- "Stalled" in the log for two things that are not stalls: a task that can start because the one before it
  finished (now "can start: planning from the latest survey"), and waiting for a task a rule has paused
  (now one "waiting" line, and no replan counted at each survey).
- The simulated rain gauge sent bursts of hundreds of messages every 5 site minutes at low speeds.
- Log times before the start of a job read `[-1:44.6]` for minus 15 minutes; now `[-00:15.4]`.
- The list of rule changes kept for the report is bounded.
- `simulate-site` defaults to 60× (was 600×): above about 100× the messages' own delays distort the job.

Added
- `tools/site_soak.py`: runs the whole simulated site, samples every process's memory, files and threads,
  can restart the broker or kill the controller mid-job, and validates the recorded messages.
- `Bus(birth=...)`: a message published on every connect, the counterpart of the last will.
- The Wire schema defines `status`.

## [0.0.7] - 2026-11-27

The secured broker of 0.0.6, made to work where a site server actually runs it.

- **`loam broker`** writes the whole secured Mosquitto setup for a site from its program: a certificate
  authority and broker certificate, a random password per client, the access rules and an `install.sh`.
  It replaces the hand-run `openssl` and `mosquitto_passwd` steps and `examples/broker/`.
- **Works on Ubuntu's confined Mosquitto.** The packaged broker may read only `/etc/mosquitto` (AppArmor)
  and reported the 0.0.6 example, run from any other folder, as "Unable to open config file".
  `install.sh` puts each file where the profile allows, owned as Mosquitto requires. CI now sets up the
  packaged, confined service this way and runs the security tests against it.
- **Tighter access rules.** Each machine gets its own rules. Before, the machine rules were a pattern for
  every user, so the camera system's account could have reported as a machine of its own name.
- Passwords are hashed in Mosquitto's format by Loam, so none appears on a command line.
- A pressed e-stop survives a restart of the packaged broker (tested).

## [0.0.6] - 2026-11-26

What a first field test needs beyond the vehicle bridges: machines that protect themselves, real
sensors, a secured broker and a record of the run. Every new piece is FOSS and optional; the core
still needs numpy only.

Safety
- **ArduPilot protects itself.** `loam mavlink` uploads the working area and the exclusion zones as
  fences, has the rover plan around them, and switches on its ground-station failsafe, watched by the
  bridge's own 1 Hz heartbeat. On SITL the rover went around an exclusion zone it had driven straight
  through before, and held 4.3 s after the bridge was killed. `--no-vehicle-setup` leaves the vehicle
  alone. A fence breach is reported as `vehicleFenceBreach`. The bridge now requires MAVLink 2.
- **Protective stops.** A stop by Nav2's Collision Monitor is reported (`stopped`, `vehicleObstacle`
  warning) and no longer counts against the order: the carrier carries on when the way is clear.
  Before, the aborted goals could fail the order.
- **A secured broker.** `--broker mqtts://user@host` with TLS 1.2+, certificate and host-name checks,
  and passwords from the environment or a file, never from the command line. `examples/broker/` holds a
  Mosquitto configuration with access rules: a machine can read only its own orders and cannot publish
  one. A refused login is reported in words.
- **Clocks.** On a real site the controller sends no orders to a machine whose clock is more than 2 s
  off its own, and ignores a survey stamped in the future. Before, a wrong clock silently broke the
  matching of scans to finished passes.
- **What sensors cannot see is unsafe.** Detections can cover an area (`radius`) and can say `unknown`:
  a blind camera, a silent Frigate or a stopped `loam sense` counts as a person there.
- `loam estop` presses and releases the site e-stop. The message is retained, so machines that connect
  later still get it.

Added
- `loam sense`: people from Frigate's cameras (MIT) and rain from an rtl_433 gauge (GPL-2.0), both over
  MQTT, as Wire `detection` and `weather` messages.
- `loam terrain --broker` publishes a scan to a running controller; `--watch` does so for every scan
  written to a folder.
- `loam monitor --mcap`: every Wire message in one MCAP file for Lichtblick or PlotJuggler; a
  `machine_clock_skew_seconds` metric.
- Survey points from QField and QGIS (GeoJSON) wherever a survey CSV is accepted.
- The Wire schema defines `detection`, `weather` and `estop`.

Fixed
- **Ground filter.** A machine's low parts (blade, tracks) stayed in the filtered ground as a bump of
  up to 0.9 m, because next to the cab they passed for part of something wide. Around wide objects the
  filter is now repeated until it finds nothing new. Bare ground is unchanged. Found by checking the
  filter against PDAL's SMRF, which is now a test: the two agree to 0.4 mm RMS.
- A failed TLS handshake or refused login no longer leaves a socket open.

Checked and left out, with reasons in docs/INTEGRATIONS.md: EMQX, Foxglove Studio and Isaac Sim (not
FOSS); Fields2Cover and AgIsoStack++ (not needed yet). OpenGrade3D 4.0.2 reads the `.agd` files
unchanged.

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
