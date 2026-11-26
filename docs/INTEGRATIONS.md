# Integrations

Loam's core needs only numpy. Everything below is an optional extra, and every
dependency is FOSS under an OSI-approved licence. Loam talks to GPL programs
(ArduPilot, QGIS, RTKLIB's apps) through their file formats and protocols, so it
never links them.

| Integration | What Loam does with it | Install | Licence | Tested against |
| --- | --- | --- | --- | --- |
| ArduPilot Rover (MAVLink 2) | `loam mavlink`: a vehicle is a Loam machine | `.[mavlink]` | pymavlink LGPL-3.0 | ArduPilot Rover SITL |
| OPERA (PWRI, ROS 2) | `loam opera`: IC120 carriers on Nav2, the ZX200 excavator's arm | ROS 2 env | Apache-2.0 | Nav2 and ros2_control |
| Project Chrono | `loam soil-calibrate`: roller compaction per pass from SCM soil | conda env | BSD-3 | Bekker–Wong closed form |
| VDA 5050 3.0 robots | `loam vda5050`: a robot is a Loam machine | `.[run]` | paho EPL-2.0/BSD-3 | official VDA 5050 3.0 JSON schemas |
| openTCS | A VDA 5050 fleet control; see below | — | MIT | — |
| RTKLIB / RTKBase | `loam nmea tcp://host:port` → Wire states, `--broker` to publish | `.[run]` | BSD-2 (RTKLIB) | RTKLIB `str2str` relay |
| OpenDroneMap | `--ground <project folder>`: its DTM, else its LAZ with the ground filter | `.[raster,lidar]` | AGPL-3.0 (ODM, files only) | DTM layout |
| LAZ (laspy, lazrs) | Compressed point clouds wherever LAS works | `.[lidar]` | BSD-2, MIT | laspy round trip |
| Ground filter | `--ground-filter`: removes machines, people and brush from clouds | core | Apache-2.0 | synthetic terrain |
| pyproj | `--crs` / `--source-crs`: real projections and reprojection | `.[crs]` | MIT | UTM ↔ Alabama East |
| GDAL / rasterio | GeoTIFF surfaces and ground; `export --to geotiff` | `.[raster]` | BSD-3 (GDAL MIT) | write → read back |
| QGIS | GeoJSON area files; `export --to geojson` | core | GPL (QGIS, files only) | GeoJSON round trip |
| IfcOpenShell | IFC 4.3 design and existing surfaces (`format ifc`) | `.[ifc]` | LGPL-3.0 | IFC4X3 built in the test |
| Prometheus / Grafana | `loam monitor` serves `/metrics` | core | Apache-2.0 / AGPL-3.0 | prometheus_client's parser |

`pip install "loam-lang[all]"` installs every extra.

## Vehicles

All three vehicle bridges (MAVLink, VDA 5050, OPERA) share one loop (`loam/runtime/bridge.py`). The stop, e-stop,
heartbeat and GNSS rules are therefore the same as for the simulated machines.

- **Orders.** An order becomes the path the simulated machine drives: through the work window, then the haul.
- **Stopping.** `stop`, a site stop, 3 s without a safety heartbeat, or a poor fix pause the vehicle.
- **E-stop.** The site e-stop triggers the vehicle's hardest stop.
- **Vehicle faults.** A vehicle that can't work reports a `fault` and the machine shows `stopped`. Causes: link lost, its own e-stop, manual mode, a fatal error.
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

On both machine kinds, pause cancels the goal and resume sends what was left. The map frame is the site grid
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

## Positioning

`loam nmea` reads GGA from a file, from stdin, or from `tcp://host:port`. RTKLIB's
`str2str` and `rtkrcv`, RTKBase and most receivers serve that. With
`--broker`, it publishes Wire states straight to MQTT. Anything other than an
RTK fixed solution is reported as `degraded`, so the controller holds the
machine.

## Survey data

```
loam export site.loam --to landxml -o next.xml --ground odm_project/ --crs EPSG:6355 \
     --world-origin E,N --world-units usft
```

- **OpenDroneMap.** A project folder is read from `odm_dem/dtm.tif`. Without a DTM, Loam reads `odm_georeferencing/odm_georeferenced_model.laz` and applies the ground filter.
- **Point clouds.** LAS and LAZ class 2 (`--ground-only`) is used when the cloud is already classified. Otherwise `--ground-filter` applies a progressive morphological filter (Zhang et al. 2003) in two passes: the lowest point per cell gives the ground surface, then only points within 15 cm of it are kept. Ground cells keep their measured heights. Cells holding machines or brush are filled by harmonic infill, so slopes are not biased low.
- **Filter accuracy, synthetic terrain.** On clean ground the error is the sensor noise (2 cm). A machine on a 3:1 bank is filled to within 0.12 m. Brush is removed except at break lines (2 cells in 2,133).
- **Filter limits.** At slope 0.3, a wide object has to stand about 1.3 m proud at 1 m cells to be removed. That catches machines and people seen from above. On flat sites, lower `slope` to catch lower objects.
- **CRS.** `--crs` places the projected coordinates; without it, the tangent plane in `docs/FIELD.md` is used. LAZ and GeoTIFF files declare their own CRS. `--source-crs` overrides that, and a compound CRS (e.g. `EPSG:32616+5703`) also transforms heights.

## Surfaces and areas

`format landxml`, `format ifc` and `format geotiff` are all accepted for
`design` and `existing`.

- **IFC.** IFC surfaces are read through IfcOpenShell in world coordinates, in metres.
- **GeoTIFF.** Rasters are sampled bilinearly in the site CRS.
- **Area files.** Draw zones in QGIS and save them as GeoJSON in the site CRS. Use them with `area file "zone.geojson"`.
- **Exports.** `loam export --to geojson` writes the boundary, zones and stockpiles. `--to geotiff` writes the design or next-pass surface, georeferenced and rotated as the site is.

## Metrics

`loam monitor` serves `/metrics` in the Prometheus text format. It reports:

- machine progress, safety state and stop latency;
- task passes and completion;
- surveys, replans and survey age;
- people detected and the e-stop state;
- the share of the latest measured terrain within the finish tolerance, plus residual RMS and mean;
- on simulated sites, the true share per zone.

```yaml
scrape_configs:
  - job_name: loam
    scrape_interval: 5s
    static_configs: [{ targets: ["monitor-host:8080"] }]
```

## Left out

- **TimescaleDB.** Its community edition is under the TSL, which is not OSI-approved. Prometheus covers metrics.
