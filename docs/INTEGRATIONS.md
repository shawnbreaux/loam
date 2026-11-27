# Integrations

Loam's core needs only numpy. Everything below is an optional extra, and every
dependency is FOSS under an OSI-approved licence. Loam talks to GPL programs
(ArduPilot, QGIS, RTKLIB's apps) through their file formats and protocols, so it
never links them.

| Integration | What Loam does with it | Install | Licence | Tested against |
| --- | --- | --- | --- | --- |
| ArduPilot Rover (MAVLink 2) | `loam mavlink`: a vehicle is a Loam machine | `.[mavlink]` | pymavlink LGPL-3.0 | ArduPilot Rover SITL |
| OPERA (PWRI, ROS 2) | `loam opera`: IC120 carriers on Nav2, the ZX200 excavator's arm | ROS 2 env | Apache-2.0 | Nav2 and ros2_control |
| Nav2 Collision Monitor | `loam opera` reports its stops; the order carries on afterwards | ROS 2 env | Apache-2.0 | Nav2 Jazzy, simulated scanner |
| Project Chrono | `loam soil-calibrate`: roller compaction per pass from SCM soil | conda env | BSD-3 | Bekker–Wong closed form |
| VDA 5050 3.0 robots | `loam vda5050`: a robot is a Loam machine | `.[run]` | paho EPL-2.0/BSD-3 | official VDA 5050 3.0 JSON schemas |
| openTCS | A VDA 5050 fleet control; see below | — | MIT | — |
| RTKLIB / RTKBase | `loam nmea tcp://host:port` → Wire states, `--broker` to publish | `.[run]` | BSD-2 (RTKLIB) | RTKLIB `str2str` relay |
| OpenDroneMap | `--ground <project folder>`: its DTM, else its LAZ with the ground filter | `.[raster,lidar]` | AGPL-3.0 (ODM, files only) | DTM layout |
| LAZ (laspy, lazrs) | Compressed point clouds wherever LAS works | `.[lidar]` | BSD-2, MIT | laspy round trip |
| Ground filter | `--ground-filter`: removes machines, people and brush from clouds | core | Apache-2.0 | synthetic terrain; PDAL's SMRF |
| PDAL | Classify ground with PDAL, then `--ground-only` | PDAL itself | BSD-3 | PDAL 2.10 `filters.smrf` |
| QField / QGIS surveys | `--ground points.geojson`, `loam score points.geojson` | core | GPL (files only) | GeoJSON points |
| Frigate | `loam sense --areas`: people seen by site cameras become `detection` messages | `.[run]` | MIT (Frigate, MQTT only) | Frigate's MQTT messages (source, v0.19-dev) |
| rtl_433 | `loam sense --rain`: a wireless rain gauge becomes `weather` messages | `.[run]` | GPL-2.0 (rtl_433, MQTT only) | rtl_433 decoding recorded gauges |
| Mosquitto with TLS | `--broker mqtts://user@host`: encrypted, named clients, per-machine access rules | `.[run]` | EPL-2.0 / EDL-1.0 | Mosquitto with `examples/broker/` |
| MCAP / Lichtblick | `loam monitor --mcap`: every Wire message in one log file | `.[mcap]` | MIT (Lichtblick MPL-2.0) | mcap's reader |
| pyproj | `--crs` / `--source-crs`: real projections and reprojection | `.[crs]` | MIT | UTM ↔ Alabama East |
| GDAL / rasterio | GeoTIFF surfaces and ground; `export --to geotiff` | `.[raster]` | BSD-3 (GDAL MIT) | write → read back |
| QGIS | GeoJSON area files; `export --to geojson` | core | GPL (QGIS, files only) | GeoJSON round trip |
| IfcOpenShell | IFC 4.3 design and existing surfaces (`format ifc`) | `.[ifc]` | LGPL-3.0 | IFC4X3 built in the test |
| Prometheus / Grafana | `loam monitor` serves `/metrics` | core | Apache-2.0 / AGPL-3.0 | prometheus_client's parser |
| chrony + gpsd | Site clocks set from GNSS; the controller checks them | system | GPL-2.0 / BSD-2 (not linked) | controller's clock check |

`pip install "loam-lang[all]"` installs every extra.

## Vehicles

All three vehicle bridges (MAVLink, VDA 5050, OPERA) share one loop (`loam/runtime/bridge.py`). The stop, e-stop,
heartbeat and GNSS rules are therefore the same as for the simulated machines.

- **Orders.** An order becomes the path the simulated machine drives: through the work window, then the haul.
- **Stopping.** `stop`, a site stop, 3 s without a safety heartbeat, or a poor fix pause the vehicle.
- **E-stop.** The site e-stop triggers the vehicle's hardest stop. Releasing it restarts nothing: the
  machine stays stopped (`estopReset`) until `loam resume`.
- **Process stop.** On SIGTERM (systemd, Docker) or Ctrl-C the bridge holds the vehicle, cancels its
  goal and reports itself offline before it exits.
- **Malformed messages** on the broker are logged and skipped; they cannot crash a bridge.
- **Protective stops.** A vehicle halted by its own protective device (Nav2's Collision Monitor) is reported as `stopped` with a `vehicleObstacle` warning. The bridge does not pause or cancel anything: when the way is clear the vehicle carries on with the same order.
- **Vehicle faults.** A vehicle that can't work reports a `fault` and the machine shows `stopped`. Causes: link lost, its own e-stop, manual mode, a fatal error.
- **Orders a vehicle can't run.** A refused mission, an order the robot never took, aborted navigation or a target out of reach is reported once as an `orderFailed` error, with the reason in `text`. The bridge holds the vehicle and drops the order. The controller withdraws the pass at once and gives that machine its next order a minute later. A failed order is never counted as done, and the vehicle is never sent back onto its previous mission.
- **Completion.** A pass counts as done only when the vehicle has finished it.

The bridges steer only. Blade height stays with the machine's own grade control,
which takes the surfaces from `loam export`.

### ArduPilot Rover

```
loam mavlink site.loam --machine dz1 --vehicle udpin:0.0.0.0:14550 --origin 34.4459,-85.7197 --broker host:1883
```

Each order is uploaded as a mission: a DO_CHANGE_SPEED plus WGS84 waypoints.
Pause and resume map to HOLD and AUTO. The site e-stop is a forced disarm, so a
person has to re-arm. `--min-fix` sets the lowest fix the vehicle may work on:
6 is RTK fixed, the default; 5 is float; 3 is a 3D fix, as SITL reports.

Measured on ArduPilot Rover SITL (skid steer), in
`tests/test_bridges.py::test_ardupilot_rover_sitl`:

- **Finish:** 1.17 ft from the end of the work line (WP_RADIUS is 2 m).
- **Line holding:** stayed within 0.62 ft of the line through the window.
- **Stop:** no drift while held.
- **E-stop:** disarmed 9 ms after the site e-stop.

These figures come from SITL's simulated GPS, not RTK in the field.

**The vehicle protects itself.** Before the first order, `loam mavlink` sets the vehicle up so that the site's
limits hold even if Loam or the radio fails. It prints every parameter it changes; the changes are stored on the
vehicle. `--no-vehicle-setup` leaves the vehicle as it is.

- **Fences.** The working area is uploaded as an inclusion fence: the convex hull of the boundary, zones and
  stockpiles, grown by 10 m for turning room. Each exclusion zone, grown by its buffer, is uploaded as an
  exclusion fence. The vehicle holds at a breach (`FENCE_ACTION` 2) and plans its routes around the fences
  (`OA_TYPE` 2). A breach is reported as `vehicleFenceBreach`.
- **Failsafe.** The bridge sends a ground-station heartbeat once a second. Without it for 3 s the vehicle holds
  (`FS_GCS_ENABLE`, `FS_ACTION` 2, `FS_GCS_TIMEOUT` 3). The failsafe watches the bridge (`MAV_GCS_SYSID` 254),
  not an operator's laptop on the same link.
- **Refusal.** If the vehicle does not accept a parameter or the fences, the bridge does not start.

Measured on SITL, in `tests/test_bridges.py::test_ardupilot_fences_and_failsafe_sitl`, with a haul whose straight
line crosses an exclusion zone:

- **Fences:** the rover went around the zone and came no closer than 12.3 ft to its buffered edge. Without the
  setup it drove straight through the zone.
- **Failsafe:** with the bridge killed outright (SIGKILL), the rover was in HOLD 4.3 s later and moved 0.06 ft
  after that.

### VDA 5050 3.0

```
loam vda5050 site.loam --machine dz1 --manufacturer acme --serial R1 --broker host:1883
```

Loam is the fleet control on `uagv/v3/<manufacturer>/<serial>/...`. The robot's
map is the site grid in metres, with `mapId` set to the site name.

- **Orders.** Each order becomes nodes and edges. The work goes on the work edge as a `loam.<op>` action, carrying the Loam order id, window and limits.
- **Pause and e-stop.** Pause and resume are `startPause` and `stopPause`. The site e-stop sends `cancelOrder` plus `startPause`.
- **Robot faults.** The robot's own e-stop, manual mode or a CRITICAL/FATAL error holds the machine.

The test robot checks every message the bridge sends against the official
schemas, which are vendored in `tests/vda5050` (MIT).

openTCS (from 7.x) is a VDA 5050 fleet control in its own right. A robot answers
to one fleet control at a time, so take it out of openTCS while Loam runs the
job. Loam and openTCS can share a site and a broker.

### OPERA (ROS 2)

OPERA is the Public Works Research Institute's open platform for earthwork autonomy (github.com/pwri-opera,
Apache-2.0). `loam opera` runs inside the machine's ROS 2 environment (Humble or Jazzy):

```
loam opera site.loam --machine dt1 --namespace ic120 --kind carrier --dump --broker host:1883
loam opera site.loam --machine ex1 --namespace zx200 --kind excavator --broker host:1883
```

**Carriers** (the IC120 crawler dump, or any machine running Nav2) take each segment differently:

- **Work passes**, the segments inside the order's window, go to Nav2's controller as an explicit path
  (`/<ns>/follow_path`). The machine holds the work line instead of re-planning from wherever it happens to be.
- **Travel and haul segments** go to `/<ns>/navigate_through_poses`, so Nav2 plans them. A path that comes back
  to where it started is sent in pieces, because Nav2 ends a through-poses goal as soon as the machine is at
  its last pose.
- **Speed:** the order's speed goes to `/<ns>/speed_limit`, repeated every second so a late subscriber still
  gets it.
- **Dumping:** with `--dump`, the vessel is raised and lowered at the deposit (com3_msgs `SetDumpAngle`), and
  only once odometry shows the machine standing still.

**Excavators** (the ZX200) cut to the target surface:

- **What the bucket follows:** the order's target surface, which is the latest Loam terrain survey plus the
  order's planned change per cell, never below design.
- **How:** in radial strokes across the part of the window within reach.
- **Joint angles** come from the machine's own URDF (`/<ns>/robot_description`).
- **The trajectory** goes to the arm's ros2_control controller (`/<ns>/upper_arm_controller`), timed to OPERA's
  joint speed limits. Joints the plan doesn't move, such as `bucket_end_joint`, are held.

On both machine kinds, pause cancels the goal and resume sends what was left. An action server that is
missing or not yet active is waited for and retried, and reported after 10 s (`vehicleNavUnavailable`,
`vehicleArmUnavailable`). An aborted Nav2 goal is retried 3 times, 2 s apart, before the order is
reported as failed (`vehicleNavFailed`), because Nav2 also aborts while it is still starting up. The map frame is the site grid
in metres; `--map-offset` and `--map-yaw` place the site in it.

Measured in `tests/test_opera.py`, a site in feet driven over Loam Wire:

- **ZX200 on OPERA's URDF:** ros2_control's JointTrajectoryController on mock hardware cut a window from 1 ft
  above grade. While dragging, the bucket tip held the design surface to 0.3 mm RMS (under 1 mm worst). It never
  went as much as 1 mm below grade, including after a stop and resume mid-cut.
- **IC120 on Nav2 Jazzy:** NavFn, Regulated Pure Pursuit and the behaviour-tree navigator drove a kinematic
  crawler.
  - **Work line:** held to 1.2 cm RMS (5.4 cm worst) after a 2 m entry.
  - **Speed:** capped at the order's 0.8 m/s.
  - **Dump:** inside the deposit window, standing still.
  - **Stop:** held position within 5 cm.
  - **Lost odometry:** reported as stopped.

**Nav2's Collision Monitor** is the machine's own last check against driving into something, fed by its
scanners. Loam does not replace it and does not act for it. Give it a `state_topic`, which is off by default, so
that `loam opera` can say why the machine stands still:

```yaml
collision_monitor:
  ros__parameters:
    cmd_vel_in_topic: cmd_vel_nav          # from Nav2's controller
    cmd_vel_out_topic: cmd_vel             # to the machine
    state_topic: collision_monitor_state   # read by `loam opera`
```

While the monitor's action is STOP, the machine is reported as `stopped` with a `vehicleObstacle` warning, and
Nav2 goals that abort in that time do not count against the order. Measured in
`tests/test_opera.py::test_ic120_stops_for_an_obstacle_and_says_why`, with a simulated scanner seeing an
obstacle 2.5 m ahead of a carrier at 0.8 m/s:

- **Stop:** at rest 0.73 m further on, short of the obstacle, and reported with `vehicleObstacle`.
- **Held:** no movement while the obstacle stayed.
- **Afterwards:** the same order resumed and finished, with no error left.

These are the platform's own kinematics and controllers. The vehicle dynamics are a simple model, not a real
machine.

`com3_msgs` (OPERA's vessel and swing actions) carries no licence. It is never shipped; the dump is used only
where the machine already has it.

## Soil physics (Project Chrono)

```
loam soil-calibrate site.loam --machine rl1 --mass 11t --diameter 1.5m -o soil.json
loam simulate-site site.loam --soil-table soil.json
```

Chrono's SCM terrain (Bekker–Wong pressure-sinkage, Mohr–Coulomb and Janosi–Hanamoto shear, plastic memory)
rolls the roller's drum back and forth over loose soil. It records the settlement after each pass. The
simulated world then compacts loose fill by that curve, capped at the lift's full compaction, rather than all
at once. So a light roller, or too few passes, leaves fill high, and the closed loop has to catch it.

**Accuracy against theory.** A towed drum sinks 1.25–1.31× the Bekker–Wong closed form for a rigid wheel. That
holds over twice the load, half the width and twice the mesh resolution, and the result is mesh-converged
within 1 %. The closed form is itself an approximation for towed wheels.

**The example table** (`examples/soil_rl1.json`): an 11 t roller with a 7 ft drum settles 80 mm on the first
pass and 156 mm after eight. Each pass adds less than the one before.

**How the check was run.** Loading has to be quasi-static: a load dropped onto SCM soil overshoots, and the
plastic memory keeps the overshoot.

The default soil parameters are illustrative. Measure them on site (plate-sinkage or bevameter test) for real
work. Calibration needs pychrono; running a site from a saved table does not.

## Setting up Chrono and ROS 2

pip cannot install pychrono or ROS 2. `tools/integration-envs.sh` builds both as conda environments with
micromamba (conda-forge, projectchrono and RoboStack channels). It also builds a workspace with OPERA's
`zx200_description`, `ic120_description` and `com3_msgs`, and prints the environment variables the tests use.

## Site sensors

`loam sense` turns what two free programs find into Loam Wire messages. Both publish on the site's MQTT broker;
nothing of theirs is linked into Loam.

```
loam sense site.loam --areas cameras.json --rain --broker mqtts://sensors@site-server
```

**People (Frigate).** Frigate records the site's cameras and detects people. The areas file places each camera,
and each zone drawn in its picture, on the site as a circle, in site units:

```json
{"cameras": {"yard": {"x": 80, "y": 50, "radius": 60,
                      "zones": {"gate": {"x": 120, "y": 40, "radius": 30}}}}}
```

- A person in a mapped zone is reported in that zone's circle, otherwise in the camera's. A camera cannot say
  where in the circle the person stands, so `proximity()` measures from the circle's nearest edge.
- **What is unknown is unsafe.** A camera that delivers no pictures, or has detection switched off, makes its
  circle `unknown`. If Frigate goes silent for 6 s or reports itself offline, or `loam sense` stops for any
  reason, the whole site is `unknown` (a retained last will). A rule such as
  `when proximity(person, machine) < 50 ft do stop all` then holds the machines until the cameras watch again.
- Frigate reports a camera's frame rate in its stats message, every 60 s by default. Set `mqtt: stats_interval: 15`
  (the lowest it accepts) so that a dead camera is noticed sooner.
- Cameras miss people in dust, glare and darkness. This is supervision, not a safety function (ISO 13849). The
  e-stop and each machine's own protective devices stay in charge.

**Rain (rtl_433).** rtl_433 decodes inexpensive wireless weather stations with a USB radio receiver:

```
rtl_433 -F "mqtts://site-server:8883,user=rtl433,pass=…"
```

`--rain` reads its events on rtl_433's default topic (`--rain TOPIC` for another) and publishes the rain rate, from the gauge's own
rate if it sends one, else from its running total over the last 10 minutes. `--gauge Model[:id]` picks one gauge
when several are in range; otherwise the first one heard is used. Tested by decoding recordings of a real gauge
(EcoWitt WH40) with rtl_433 itself.

**The e-stop from a keyboard.** `loam estop site.loam` presses the site e-stop and `--release` releases it. The
message is retained, so a machine that connects later still gets it. Releasing restarts nothing; that takes
`loam resume`. This does not replace a hardware e-stop.

**Scans as they arrive.** `loam terrain site.loam scans/ --watch --broker …` publishes every scan that appears
in a folder, once the file has stopped changing (`--settle`, 2 s by default), stamped with the file's time.

## Positioning

`loam nmea` reads GGA from a file, from stdin, or from `tcp://host:port`. RTKLIB's
`str2str` and `rtkrcv`, RTKBase and most receivers serve that. With
`--broker`, it publishes Wire states straight to MQTT. Anything other than an
RTK fixed solution is reported as `degraded`, so the controller holds the
machine.

- **Base station.** RTKBase (AGPL-3.0) turns a receiver and a Raspberry Pi into an NTRIP base and serves NMEA to
  `loam nmea`. Loam uses it over the network only.
- **Network corrections in Alabama.** ALDOT runs a statewide CORS network. Sources disagree on whether an account
  is free, so ask ALDOT before planning on it.
- **Clocks.** Every Wire message is stamped with its sender's clock, and scans are matched to finished passes by
  time. On a real site the controller compares each machine's state stamps with its own clock and sends no
  orders to a machine more than 2 s off (`machine_clock_skew_seconds` in the metrics, `clock_skew_s` in the
  status). A scan stamped more than 2 s in the future is ignored. Run chrony on every computer; where there is
  no internet, feed it from the GNSS receiver through gpsd, with the receiver's PPS pulse if it has one:

  ```
  # /etc/chrony/chrony.conf on the site server (gpsd running)
  refclock SHM 0 refid GPS precision 1e-1 offset 0.0 delay 0.2
  refclock PPS /dev/pps0 lock GPS refid PPS
  allow 192.168.0.0/16
  local stratum 10
  ```

  The machines' computers then name the site server as their only `server`.

## Survey data

```
loam export site.loam --to landxml -o next.xml --ground odm_project/ --crs EPSG:6355 \
     --world-origin E,N --world-units usft
```

- **OpenDroneMap.** A project folder is read from `odm_dem/dtm.tif`. Without a DTM, Loam reads `odm_georeferencing/odm_georeferenced_model.laz` and applies the ground filter.
- **Point clouds.** LAS and LAZ class 2 (`--ground-only`) is used when the cloud is already classified. Otherwise `--ground-filter` applies a progressive morphological filter (Zhang et al. 2003) in two passes: the lowest point per cell gives the ground surface, then only points within 15 cm of it are kept. Ground cells keep their measured heights. Cells holding machines or brush are filled by harmonic infill, so slopes are not biased low.
- **Machines seen from above.** A scanner also hits a machine's blade and tracks, well below its cab. Around
  every wide object the filter is repeated on its own result until it finds nothing new, so the low parts go
  too. In a trial of 12 dozers parked on rolling ground with a berm, one round left a 0.9 m bump under every
  one; the repeated filter left the ground within 0.1 m under 11, and the 0.9 m bump under the one parked on the
  berm's crest. Bare ground is not changed by the repeats.
- **Checked against PDAL.** On one scan with a machine, two people and brush, Loam's filter and PDAL's SMRF
  (Pingel et al. 2013, same 15 cm tolerance) gave surfaces 0.4 mm RMS apart, 0.8 mm and 1.6 mm RMS from the
  true ground (`tests/test_integrations.py::test_ground_filter_agrees_with_pdal_smrf`). To use PDAL's filter
  instead: `pdal translate scan.las ground.las smrf`, then `--ground ground.las --ground-only`.
- **Survey points from QField or QGIS.** A GeoJSON file of points (`--ground points.geojson`,
  `loam score site.loam points.geojson --zone pad`) is read like a survey CSV. The height is each point's third coordinate,
  else its `elevation`, `z` or `height` property. Plain GeoJSON is longitude and latitude; a file QGIS wrote in a projected CRS says so and is read as easting and northing.
- **Filter accuracy, synthetic terrain.** On clean ground the error is the sensor noise (2 cm). A machine on a 3:1 bank is filled to within 0.12 m. Brush is removed except at break lines (2 cells in 2,133).
- **Filter limits.** At slope 0.3, a wide object has to stand about 1.3 m proud at 1 m cells to be removed. That catches machines and people seen from above. On flat sites, lower `slope` to catch lower objects. Crests sharper than `slope` (a windrow, the top of a steep stockpile) are taken for objects; those cells come back unseen, not at a wrong height. A step under 1.3 m on a 3:1 bank, such as a blade resting over the edge, stays in the surface: keep parked machines off banks when scanning.
- **CRS.** `--crs` places the projected coordinates; without it, the tangent plane in `docs/FIELD.md` is used. LAZ and GeoTIFF files declare their own CRS. `--source-crs` overrides that, and a compound CRS (e.g. `EPSG:32616+5703`) also transforms heights.

## Surfaces and areas

`format landxml`, `format ifc` and `format geotiff` are all accepted for
`design` and `existing`.

- **IFC.** IFC surfaces are read through IfcOpenShell in world coordinates, in metres.
- **GeoTIFF.** Rasters are sampled bilinearly in the site CRS.
- **Area files.** Draw zones in QGIS and save them as GeoJSON in the site CRS. Use them with `area file "zone.geojson"`.
- **Exports.** `loam export --to geojson` writes the boundary, zones and stockpiles. `--to geotiff` writes the design or next-pass surface, georeferenced and rotated as the site is.

## Metrics

`loam monitor` serves `/metrics` in the Prometheus text format. It listens on 127.0.0.1 unless given
`--host`; use `--host 0.0.0.0` for a Prometheus server on another machine. It reports:

- machine progress, safety state and stop latency;
- task passes and completion;
- surveys, replans and survey age;
- people detected and the e-stop state;
- the share of the latest measured terrain within the finish tolerance, plus residual RMS and mean;
- each machine's clock difference from the controller;
- on simulated sites, the true share per zone.

```yaml
scrape_configs:
  - job_name: loam
    scrape_interval: 5s
    static_configs: [{ targets: ["monitor-host:8080"] }]
```

**A flight recorder.** `loam monitor site.loam --mcap run.mcap` writes every Wire message it sees to one MCAP
file: one channel per topic, JSON, logged at the time it arrived and stamped with the time the sender gave it.
Lichtblick (MPL-2.0, the open continuation of Foxglove Studio) and PlotJuggler open it. Each topic is given a
permissive schema, so a message that breaks the Wire schema is still recorded.

## Left out

- **TimescaleDB.** Its community edition is under the TSL, which is not OSI-approved. Prometheus covers metrics.
- **EMQX.** From version 5.9 it is under the Business Source License. Mosquitto (EPL-2.0) is the broker Loam is
  tested with.
- **Foxglove Studio.** Closed since version 2. Lichtblick is its open continuation.
- **Isaac Sim.** Its source is public but it needs NVIDIA's Omniverse Kit, which is not FOSS. OPERA's own
  simulator (OperaSim-AGX) needs Unity and AGX Dynamics, also not FOSS. PyBullet and Project Chrono cover
  simulation.
- **Fields2Cover / opennav_coverage** (BSD-3, Apache-2.0). Coverage path planners for fields. Loam's planner
  already cuts each task into blade-wide strips over its window, so a second planner would add a C++ dependency
  and nothing else. Worth another look for headland turns.
- **AgIsoStack++** (MIT). An ISOBUS stack in C++. It would matter for commanding implements behind a tractor;
  the tractor route goes through AgOpenGPS and OpenGrade3D, which Loam feeds by file.
