# Field tools

Loam talks to field equipment through files and simple streams, so it works with the tools you already have and does not need anyone's SDK. This page shows what each tool exchanges with Loam and the commands that do it.

## Compatibility

| Tool | What it does on the test site | How Loam connects | Command |
|---|---|---|---|
| **Cat Grade with 3D** (dozers) | Holds the blade to a design surface | Loam writes the next-pass surface as LandXML; load it by USB or VisionLink | `loam export --to landxml` |
| Trimble, Topcon, Leica machine control | Same | LandXML, as above | `loam export --to landxml` |
| **OpenGrade3D** | Open-source blade control on a tractor | Loam writes the next-pass surface as an `.agd` design grid | `loam export --to agd` |
| **AgOpenGPS** | Open-source autosteer | Loam writes a field folder: origin, boundary, exclusion zones, one AB line per work zone, one curve per task that runs `along` a line | `loam export --to agopengps` |
| **OpenGrade valve boards** (Nano on USB, Teensy on Ethernet) | Move a blade's hydraulic valve | Loam is the grade box: it sends the board how far the edge is from the next-pass surface, and can tune the board by measuring the valve | `loam blade` |
| **AgOpenGPS GPS boards** (`$PANDA`, `$PAOGI`) | Position, heading and roll | Loam reads their sentences from serial, TCP or the UDP port they broadcast to | `loam blade --gnss`, `loam nmea udp://:9999` |
| **Emlid Reach RS3** (rover) | Truth surveys | Loam reads Emlid Flow CSV exports (All columns, PNEZD, PENZD) | `loam score`, `--ground survey.csv` |
| Emlid Reach (or any RTK receiver) on a machine | Live position | Loam reads NMEA GGA (file, stdin, TCP or UDP) and emits Wire `state` messages | `loam nmea` |
| **QField** (phone or tablet) or QGIS | Survey points | Loam reads GeoJSON points | `loam score`, `--ground points.geojson` |
| **Livox Mid-360** on a mast | Overhead scans | Record with livox_ros_driver2 and save PCD (or LAS); Loam grids each scan as it is written | `loam terrain --watch`, `--ground scan.pcd` |
| IP cameras with **Frigate** | Sees people on the site; calibrated to the ground, says where each one stands | Loam reads Frigate's MQTT messages | `loam sense --areas` |
| Phones with **OwnTracks** | Each person's GPS position | Loam reads OwnTracks' MQTT messages | `loam sense --tags` |
| Wi-Fi access points (**OpenWrt**, others) | Which phones are on the site, and near which access point | Loam reads hostapd's client table, or single signal readings | `loam sense --tags`, `loam wifi-fit` |
| **Meshtastic** trackers | A person's position where there is no phone signal | Loam reads the gateway's JSON over MQTT | `loam sense --tags` |
| **USGS 3DEP**, OpenDroneMap, any DEM | The ground of a field | Loam reads the GeoTIFF | `loam level`, `--ground dem.tif` |
| **Google Earth**, FSA field boundaries, **farmOS** | The field's boundary, or the line of a swale | Loam reads KML, KMZ, shapefile, GeoJSON | `loam level --boundary`, `loam swale --line` |
| Wireless rain gauge with **rtl_433** | Rain rate | Loam reads rtl_433's MQTT events | `loam sense --rain` |
| **DJI Zenmuse L2** | Drone scans | Process in DJI Terra, export LAS; Loam grids it | `loam terrain`, `--ground scan.las` |
| Any lidar or photogrammetry tool | Scans | LAS 1.0–1.4 (uncompressed), PCD, or x y z text | same |

### What Loam cannot connect to

These have no public interface that third-party software can use to command the machine, so Loam can only feed them design files:

- **Cat Command** (remote control). It is a closed radio link between Cat's console or station and the machine. Loam cannot send it steering commands.
- **Teleo** retrofit kits are closed. **Bluelight** kits (rollers and haul trucks) are closed too, but its maker says it will write an integration and an API for a customer. Loam Wire and `loam conform` are what to hand them.
- **Bobcat** remote control (the radio kit and the MaxControl iPhone app) and the handsets of remote trench rollers. Built to run with nobody aboard, and closed.
- **DJI flight control.** Fly with DJI's own apps; Loam only reads the scans.
- **Livox SDK2 live stream.** Loam reads recordings (PCD/LAS), not the live UDP stream. `loam terrain --watch` publishes each recording as soon as it is written.
- **Cat VisionLink API.** It carries machine hours, location and fuel (ISO 15143-3), not control. A reader is possible later; it isn't needed for grading.

## Connecting a machine

Loam drives any machine that is open to being driven. There are four ways in, and a test that tells you whether
a machine is through.

| The machine has | Use | What Loam needs from it |
|---|---|---|
| An **ArduPilot** (or other MAVLink) autopilot | `loam mavlink` | Position, missions, HOLD and AUTO. With `--blade-servo`, one servo output for the blade |
| **ROS 2 with Nav2** | `loam nav2` | A `navigate_through_poses` action, odometry, a `map` frame |
| A **VDA 5050** interface (factory and yard robots) | `loam vda5050` | VDA 5050 3.0 over MQTT |
| Its **own controller**, by you or its maker | Speak Loam Wire directly | JSON over MQTT, as [SPEC.md](../SPEC.md) and the JSON Schema define it |

The fourth is the general one. A machine subscribes to its orders, reports its state four times a second, and
obeys the stops. The reference is `loam/runtime/machine.py`, a few hundred lines.

**Check it before you trust it:**

```
loam conform site.loam --machine dz1 --window 20,-5,60,5 --broker mqtts://controller@site-server
```

`loam conform` plays the controller for that one machine and checks eight promises: it announces itself, reports
well-formed state at least once a second, reports itself ready, works an order and reports it done once, never
runs an order twice, stops within 2 s of a stop and goes on after a resume, stops by itself within 5 s when the
controller falls silent, and stops on the e-stop without restarting when it is released. **The machine drives
short passes across the window, so clear the area**, and run it with no other controller on the broker. Any
maker can run it against their own machine; passing it is what "works with Loam" means.

### A blade with nobody aboard

A machine that drives itself has nobody to load a design into its grade control. `loam mavlink --blade-servo N`
has the bridge hold the blade itself: every cycle it works out where the cutting edge should be (the latest
survey, cut by at most the task's limit, never below design), reads the vehicle's height, and moves one servo
output.

```
# once: blade resting on a mark of known elevation, to measure how far the antenna is above the edge
loam mavlink site.loam --machine dz1 --vehicle /dev/ttyACM0 --origin LAT,LON --blade-servo 9 --blade-zero 101.25

loam mavlink site.loam --machine dz1 --vehicle /dev/ttyACM0 --origin LAT,LON --broker mqtts://dz1@site-server \
     --blade-servo 9 --blade-height 3.02
```

- **Put an RTK antenna on the blade**, above the cutting edge. The edge is then measured, not worked out, and
  the actuator is stepped until it is within a quarter of an inch. Make it the autopilot's second GPS and pass
  `--blade-gps2`: the first GPS stays on the body and steers, the second gives the blade's height.
- A height older than a second, or from a second GPS without an RTK fix, is treated as no height.
- The actuator must go to a position on command: a servo, or a linear actuator with position feedback and a
  controller that takes a servo pulse. A plain up-down actuator will not do.
- With the antenna on the machine's body, give `--blade-travel` (how far the edge rises over the actuator's
  stroke). The edge is then worked out from the actuator's position, which is only as good as that geometry.
- `--blade-reach` says how far the edge is ahead of the position the vehicle reports (negative for a towed blade).
- The blade is in the ground only inside the order's window, during an order that moves earth, while the machine
  may move. Otherwise, and whenever the height is lost, it is carried fully raised. On a pass along a line it is
  down only within half a tool width of the path, so the drive to the start of a lane cuts nothing.
- One actuator holds the whole edge level. The edge's length is the machine's `blade` in the program, and the
  edge is held at the highest point wanted anywhere under it. So neither end cuts deeper than its own ground
  allows, and an end that hangs over ground outside the job stays on top of that ground. **It cannot shape a
  slope across its own width**: the sides of a swale need a blade that tilts, which `loam blade` (below) drives
  through a second valve board and `loam mavlink` does not.
- The servo output can drive a hobby or linear servo directly, or a hydraulic valve through a driver that takes
  a servo pulse.

Tested against a fake ArduPilot over a real broker. It has not moved a real blade yet.

### A grade box beside AgOpenGPS: `loam blade`

AgOpenGPS steers a tractor; `loam blade` holds the blade behind it. It needs no controller, no broker and no
survey: one program on the tractor's computer reads an RTK antenna on the blade, works out how far the cutting
edge is above the surface the next pass should leave, and sends that number, in millimetres, to an **OpenGrade
valve board**. The board's firmware moves the valve, and it acts only while its work switch is on, so the
person on the tractor keeps the last word.

```
# 1. once: the cutting edge standing level on ground of known site elevation
loam blade field.loam --valve serial:/dev/ttyUSB0 --gnss udp://:9999 --origin LAT,LON --box box.json --zero 101.25

# 2. once per valve: blade in the air near the middle of its travel, work switch on, everyone clear
loam blade field.loam --valve serial:/dev/ttyUSB0 --gnss udp://:9999 --origin LAT,LON --box box.json --calibrate lift

# 3. grade, only while AgOpenGPS is steering
loam blade field.loam --valve serial:/dev/ttyUSB0 --gnss udp://:9999 --origin LAT,LON --box box.json --with-agopengps
```

**What it talks to.**

| Part | Which | Link |
|---|---|---|
| Valve board | OpenGrade's Nano board (USB) | `--valve serial:PORT` (needs `pip install "loam-lang[serial]"`) |
| Valve board | OpenGrade3D's Teensy board (Ethernet), with the blade's antennas on it | `--valve udp://ADDRESS --gnss board` |
| Blade antenna | Any AgOpenGPS GPS board (`$PANDA`, `$PAOGI`), or any receiver that sends GGA | `--gnss serial:PORT`, `tcp://HOST:PORT`, `udp://:PORT` or `udp://SENDER:PORT` |
| AgOpenGPS | AgIO's own messages, heard on the network; nothing is sent to it | `--with-agopengps` |

Both valve boards drive one valve section through a motor driver (pulse width and direction), so any 12 V
valve that driver can power will do: proportional, or plain open-and-shut.

**Valves are measured, not looked up.** A valve's standard size (ISO 4401-03, sold as D03, CETOP 3 or NG6;
ISO 4401-05, sold as D05, CETOP 5 or NG10) says how it bolts on and roughly how much oil it can pass. How fast
your blade moves also depends on the cylinder, the pump and the hoses, so no table by valve size can give the
board's settings. `--calibrate` measures them instead. With the blade in the air it pulses the valve up for a
second and down for a second at stronger and stronger duties, watches the antenna, and finds for each
direction the duty at which the blade starts to move and how much speed each further step adds, the valve's
delay, and the antenna's noise. From those it sets the board's eight numbers: least and greatest duty,
deadband (two and a half times the noise, at least 3 mm), and a gain that makes the error die away over twice
the delay. It takes about a minute, stops early once the blade moves 150 mm/s both ways, gives the blade one
pulse back if it has wandered 200 mm, and gives up at 600 mm. It says so if the blade does not move (the work
switch is off, or there is no oil), if it moves the wrong way (the coils are swapped), and if the valve is too
quick for the board to place the blade finely (fit a flow restrictor). An open-and-shut valve is recognised:
the board is set to switch it fully on outside a deadband as wide as the blade coasts.

**The box file** (`--box`) keeps what `--zero` and `--calibrate` measured, as JSON you can read. The settings
in it are sent to the board every time grading starts.

**Several passes with no survey.** The ground starts as the program's `existing` surface. Wherever the edge
passes below the ground as it is known, the ground is now there, so each pass may cut one `max_cut` deeper than
the last, down to the design and never past it. A piece of ground keeps its target until the edge has left it
by more than its own length, so one pass cannot chase its own cut down. **Fill is not followed**: a blade
cannot tell what it spilled, so low ground stays as last known and the edge is carried over it.

**Tilt.** `--tilt-valve` names a second valve board, on the tilt cylinder (`--calibrate tilt` tunes it). It
needs the blade's roll, so two antennas (`$PAOGI`, or the Teensy board's own pair), and `--mast`, the antenna's
true height over the edge, so that the lean of the mast is taken out of the position. The edge is tilted to the
straight line that best fits the wanted surface under it, at most 0.35 across (19 degrees), and its middle is
held just high enough that neither end cuts deeper than wanted. This is what the sides of a swale need. Two
Teensy boards cannot share a network (the firmware gives every one the same address), so with a Teensy on the
lift put a Nano on the tilt.

**When it will not cut.**

- No RTK-fixed position from the blade's antenna in the last half second (`--allow-float` accepts RTK float):
  the blade is lifted for a second and a half, then left alone. `--hold-on-loss` leaves it where it is.
- Outside every work zone, or where the task is done: the edge is carried half a foot above the ground.
- With `--with-agopengps`, whenever AgOpenGPS has not said in the last 2 s that it is steering. The status
  line shows the pass AgOpenGPS is on.
- With one antenna, until the tractor has moved 0.2 m: the edge's direction is the direction of travel.
- The board's work switch is off, or the board has heard nothing for 0.28 s (Nano); both are the firmware's own.
- Positions arriving on one UDP port from two senders, such as the tractor's GPS board and the blade's: it
  stops and asks for `--gnss udp://SENDER:PORT`.

**Limits.** Tested against a stand-in for each board that follows the published firmware's arithmetic, a
simulated cylinder and valve, and a noisy, late antenna, over real serial and network ports. **It has not
moved a real blade.** Not tested on Windows. The first passes on a real machine should be made in the air,
with `--zero` given an elevation a foot too low.

### Machines, by how open they are

Checked in November 2026. "Open" means outside software can command it without the maker's private agreement.

| Machine | Nobody aboard? | Open to Loam? | Way in |
|---|---|---|---|
| A hydrostatic two-lever machine with an ArduPilot and servos on its levers (the zero-turn mower conversions of the ArduPilot community) | Yes | Yes | `loam mavlink` |
| **Amiga** robots (farm-ng, now Bonsai Robotics) | Yes | Yes: a published SDK and API | Its SDK or ROS bridge; `loam nav2` once Nav2 runs on it |
| **AgileX Bunker Pro** tracked chassis | Yes | Yes: CAN protocol, open-source ROS 2 driver | `loam nav2` |
| OPERA machines (PWRI, Japan) | Yes | Yes | `loam opera` |
| Cat soil compactor with a **Bluelight** kit | Yes | On request: its maker says it will write an integration and an API | Their API to Loam Wire |
| Dozers with the **Polymath Robotics** and Hard-Line kit | Yes | A commercial SDK and API | Their API to Loam Wire |
| **Bobcat** loaders with the Radio Remote Control kit or MaxControl | Yes | No: a closed radio link and an iPhone app | None |
| **Cat Command**, **Teleo** | Yes | No | None |
| Remote trench rollers (Wacker Neuson RTLx-SC3, Bomag BMP 8500) | Yes | No: an infrared or radio handset | None |
| Any machine with **Cat Grade with 3D** | No: a person steers | Files only | `loam export --to landxml` |

No rental yard offers a machine in the first four rows. The machines that can be rented and run with nobody
aboard are closed, except where a kit maker opens an API.

## Placing the site on the earth

Loam programs use site-local coordinates. Every field command takes the same options to tie them to real coordinates:

| Option | Meaning |
|---|---|
| `--origin LAT,LON` | WGS84 position of site point (0, 0). Needed for AgOpenGPS, OpenGrade3D and NMEA. |
| `--world-origin E,N` | Projected easting and northing of site point (0, 0), e.g. UTM 16N or Alabama East State Plane. Needed for LAS/PCD clouds and projected survey exports. |
| `--world-units m\|ft\|usft` | Units of projected coordinates and heights in files. State Plane in Alabama is often US survey feet. |
| `--rotation DEG` | Bearing of the site's +y axis, degrees clockwise from north. |
| `--dz VALUE` | Site units added to incoming heights (a datum or benchmark correction). |

Set the RTK base over a control point, survey site (0, 0) and one point along the site's +y axis, and you have all of these.

Latitude and longitude use the same local tangent plane as AgOpenGPS, so an AgOpenGPS field whose origin is site (0, 0) lines up with the site grid exactly. Over a 20-acre site the plane is accurate to well under a millimetre.

**Heights must share one vertical datum.** Cat Grade, OpenGrade3D and the Reach all compare blade height with the design. Use the same datum (for example NAVD88 through the receiver's geoid model) for the design, the scans and the receivers, or correct with `--dz`.

## The closed loop with a grade-control system

A grade-control system holds the blade to whatever surface it has loaded. Loam closes the loop around it by computing what the *next* pass should leave, from the latest measured ground:

```
loam export site.loam --to landxml -o next.xml --ground scan.las --world-origin 600000,3812000
```

For every cell in a cut, fill or finish zone: in tolerance → design; too high → cut toward design by at most `max_cut` (0.5 ft if no task sets it); too low → build up by at most one `lift` (8 in if none). Exclusion zones and their buffers get no surface at all, so the machine has nothing to grade there. The command prints the cut and fill volumes and `done: True` once every cell is in tolerance.

Each cycle:

1. Scan (mast lidar) or survey (rover).
2. `loam export ... --ground <scan>` writes the next-pass surface.
3. Load it into the machine (USB stick for Cat Grade, file open in OpenGrade3D).
4. Grade one pass over the zone.
5. Repeat until `done: True`, then `loam score` with an independent rover survey.

On a Cat machine, LandXML must be in the same projected coordinate system as the machine's site calibration: pass `--world-origin` (and `--rotation`) and the file is written in those coordinates.

## People on foot

Cameras see people without knowing who they are. Phones know who they are without knowing well where. `loam sense`
takes both and reports every person as a circle the machines keep clear of.

```
loam sense site.loam --areas cameras.json --tags people.json --origin LAT,LON --broker mqtts://sensors@site-server
```

### What a person carries: the tags file

```json
{"people": {"sam": {"owntracks": "trackers/sam", "wifi": "a4:83:e7:12:34:56", "meshtastic": "!a1b2c3d4"},
            "ana": {"wifi": "a4:83:e7:00:00:02"}},
 "wifi": {"aps": {"barn": {"x": 0, "y": 0, "bssid": "74:ac:b9:00:11:22"},
                  "well": {"x": 400, "y": 0},
                  "dz1-ap": {"machine": "dz1"}},
          "rssi_1m": -40, "exponent": 2.7, "sigma_db": 6},
 "sightings": {"everyone_tagged": true, "watch": {"mast": 10}, "fixtures": [{"x": 300, "y": 300, "r": 4}]}}
```

| Source | What it needs | What it gives |
|---|---|---|
| **OwnTracks** (phone app, Android and iPhone) | MQTT mode, the site broker, user `trackers`. The phone then publishes to `owntracks/trackers/<device>` | The phone's GPS position and its accuracy: usually a circle of 5 to 15 m |
| **Wi-Fi signal strength** | Each access point reports the clients it hears to `wifi/<ap>`. No app on the phone | A distance from each access point, roughly. See below for how roughly |
| **Meshtastic** tracker | A gateway node with MQTT and JSON output switched on | The tracker's GPS position, where there is no phone signal |

A person is the tightest circle any source gives, grown by walking pace (2.5 m/s, `speed_m_s`) for every second
since the fix. A phone that goes quiet on the site therefore becomes a circle that grows until the machines stop.

- **Access points.** On OpenWrt, send hostapd's own client table as it is:
  `while sleep 2; do ubus call hostapd.phy0-ap0 get_clients | mosquitto_pub -s -t wifi/barn …; done`.
  Any other access point or controller (UniFi's Network application lists each client's signal and access point)
  can publish single observations instead: `{"mac": "a4:83:e7:12:34:56", "rssi": -61}`, or a measured Wi-Fi RTT
  range as `{"mac": …, "range_m": 12.4, "std_m": 1.0}`. The UniFi route is not tested here.
- **An access point on a machine** (`{"machine": "dz1"}`) is a proximity sensor: its place is wherever that
  machine reports itself, and a strong signal there means a person is near that machine.
- **Phones use a private Wi-Fi address** for each network. Read it in the phone's Wi-Fi settings for the site's
  network, and set it to fixed, not rotating.
- **Leave machine operators out of the file.** An operator's phone is a person a few feet from a machine, and the
  proximity rule would stop that machine.

### How precise Wi-Fi is, and is not

Signal strength is a poor ruler. With the default model and three access points 120 m apart, the circle that
holds the phone 99 % of the time has a median radius of about 170 m: it says the phone is on the site, not where.
It is tight only near an access point (about 18 m at 5 m), which is why one on a machine is useful.

The default model is also only a guess about your site. On a simulated site where the signal carries farther than
the default assumes, the default's circles held the phone 8 % of the time. So measure it:

```
loam wifi-fit walk.csv          # distance_m,rssi_dbm from a walk away from an access point
```

or let it be learnt. Whenever a person's place is known to within 10 m from another source (a phone's GPS, or a
person pinned to a sighting) while an access point hears the phone, that is a sample. After 40 samples the model
is refitted, and again at every 20 more (`"wifi": {"learn": false}` switches this off). In the tests the learnt
model's circles hold the phone 99 % of the time on a site the default gets badly wrong, and are half the size on a
site with steady signal. A learnt model is not trusted outside the distances it was taught at.

### Putting phones together with lidar and cameras

The precise sensors report *sightings*: where a person-sized thing stands, without a name.

| Sighting source | Precision | Set up |
|---|---|---|
| **A camera calibrated to the ground** | About a foot near the camera, a few feet across a pad | Add `"ground": [[px, py, x, y], …]` to the camera in the areas file: four or more ground marks, each as its pixel in Frigate's detect picture and its site coordinates. Flat ground only |
| **A lidar scan** | The site grid's cell | `loam terrain … --ground-filter --objects` publishes the person-sized things in every scan (0.8 to 2.4 m tall, no wider than 1.5 m) |
| **Anything else** (a drone's detector) | Its own | Publish `{"t": seconds, "people": [{"x", "y", "r"}], "objects": […]}` to `sightings/<name>`, in site units, or `{"lat", "lon", "acc"}` with the accuracy in metres. Each message is the whole picture: what is not in it is gone |

Every sighting is reported as a small keep-out circle of its own (`sense/seen/1` … `sense/seen/12`), whatever
else is known. What happens to the phones' wide circles depends on one site rule.

**`"everyone_tagged": true`: everyone on foot carries a listed phone or tracker.** Then the two are matched:

- a person whose circle holds exactly one sighting is that sighting. The detection `sense/tag/sam` moves to where
  the camera or lidar sees sam, a few feet wide;
- a person whose circle holds several is one of them, and is not reported separately;
- a person whose circle holds none keeps the wide circle. A phone is heard and nobody is seen where it could be:
  someone is behind a machine or in a blind spot;
- when more phones are heard in a place than things are seen there, everyone concerned keeps the wide circle,
  because nobody can say which of them is the one not seen.

One person seen by both a camera and a lidar counts once. In 3,000 random test sites with hidden people, double
sightings and posts, no person was ever left outside every reported circle.

**Without that rule** (the default) nothing a phone says is narrowed by a sighting, because a visitor who is seen
could stand in for a worker who is not. Phones and sightings are reported side by side.

Three more things to know:

- **A lidar sees shapes, not people.** A shape counts toward the head count only if it has moved in the last
  10 minutes (`fixture_s`), so a post or a barrel never accounts for a phone. It is still a keep-out. List the
  shapes that belong to the site under `fixtures` and they are not reported at all.
- **A source the site relies on** goes under `watch` with the seconds it may be silent. Longer than that, and the
  whole site is `unknown`, as with a dead camera.
- **A person leaning on a machine** is part of the machine to a lidar. The access point on the machine is what
  covers that case.

All of this is supervision, not a safety function. A phone can be left in the truck, a camera can be blinded by
dust. The e-stop and each machine's own protective devices stay in charge.

## A field with no design: `loam level`

`loam plan` starts from an engineer's grading model. A farm field usually has none. `loam level` writes the design
from the ground itself: one plane, with enough fall to drain and the cuts paying for the fills.

```
loam level dem.tif --boundary north_field.kml -o north.loam --min-slope 0.1 --max-slope 0.3 --edge "20 ft"
loam level survey.xml -o pad.loam --fall 135 --cross-slope 0.1 --compaction      # a building pad, rolled
```

| Input | Where it comes from |
|---|---|
| Ground as a **GeoTIFF** | USGS 3DEP elevation (free; 1 m tiles are in UTM metres), an OpenDroneMap terrain model from your own drone, any GIS |
| Ground as **LandXML** | A surveyor, or `loam terrain --landxml` from your own scan |
| Boundary as **KML or KMZ** | Trace the field in Google Earth and save it |
| Boundary as a **shapefile** | FSA common land units, farm management software. Needs `pip install pyshp` |
| Boundary as **GeoJSON** | QGIS, a farmOS export |

What it does:

1. fits a plane to the ground inside the boundary by least squares, which is the plane that moves the least earth;
2. brings its grade within `--min-slope` and `--max-slope` (percent). With `--fall` (a bearing, degrees clockwise
   from north) the main grade runs that way and `--cross-slope` limits the grade across it;
3. raises or lowers the plane until cut is `--cut-fill` times fill (1.3 by default), because soil spread in a fill
   ends up denser than it lay in the ground. **Ask your NRCS office what ratio they use for your soil**;
4. with `--edge`, blends the plane into the ground over that width, so the field does not end in a bank;
5. writes the design (and, from a raster, the ground) as LandXML next to the program, and the program itself.

The summary gives the deepest cut. **Compare it with the depth of your topsoil** before any work: where the cut
goes through it, raise the slope limits or level the field in two benches.

- Cropland is not rolled: the program has no compaction task, and rain holds all work. `--compaction` puts the
  roller back for a pad or a yard.
- A fleet without dozers works. Give `--fleet` a scraper and a land plane (`class grader`) and the tasks go to
  whatever machines can cut, spread and finish.
- Boundaries in latitude and longitude are placed on the ground file by its coordinate system. A ground file in
  degrees is worked in the UTM zone it lies in. A raster whose heights are in a different unit from its map
  needs `--height-unit`.
- The design reaches to within one grid cell of the boundary.

## A swale along a line: `loam swale`

A swale carries water off a field. `loam swale` designs one from the ground and a line drawn where the water
should go, and writes a program whose passes follow that line.

```
loam swale dem.tif --line swale.kml -o swale.loam --bottom "4 ft" --depth "1 ft" --side-slope 4 --grade 0.5
loam swale survey.xml --along "40,60 150,100 260,90" -o swale.loam --flow forward
```

The line is a path drawn in Google Earth (KML, KMZ), a GeoJSON LineString, a shapefile polyline, or points
typed on the command line. What it does:

1. finds the bottom: the highest line along the centreline that is at least `--depth` below the ground
   everywhere and falls at least `--grade` percent the whole way. Water never stands in it, and no line that
   drains moves less earth. Water runs the way that needs less digging, or the way `--flow` says;
2. shapes the channel: a flat bottom `--bottom` wide, sides rising one unit for every `--side-slope` units
   out, rounded ends. The design is the lower of that and the ground: a swale is cut, never filled;
3. widens the strip it works on until the sides have met the ground, as they must on a hillside;
4. writes the design (and, from a raster, the ground) as LandXML next to the program, and the program itself.

The program's cut and finishing tasks carry `along [...]`, the centreline. Each pass is then a lane: the
centreline, or the centreline moved a whole number of tool widths to one side, one push long. A machine works
its way along the swale and comes back along it in the other direction. The line says where to drive. The
measured ground still says how deep each pass cuts and when the swale is done.

- **Check the outlet.** The summary gives how far below the ground the swale ends. Water leaves only if the
  ground beyond is lower: a ditch, a pond, a slope.
- **The sides need a blade that tilts**, or a machine whose own grade control takes the surface (`loam export
  --to landxml`). A level blade on one actuator cuts the flat bottom.
- The summary gives the grade of the bottom. Where it is steep the water will cut the channel deeper: ask your
  NRCS office what your soil carries bare and under grass.
- The site grid must show the channel: at least two cells across the bottom or one side. A long swale drawn
  diagonally needs many cells; the message says how many (`--max-cells`), or design it in shorter pieces.
- `along` works on any cut or finish task, not only on a swale: give it the line of a ditch, a road's edge, a
  terrace.

For AgOpenGPS, `loam export --to agopengps` writes the line as a curve track. AgOpenGPS lays its passes one
implement width apart on both sides of a curve, which are the lanes Loam plans for a tool of that width.

## Commands

```
# Mast scan to the site grid; also as a Wire terrain message and an as-built surface
loam terrain site.loam scan.pcd --world-origin E,N --wire terrain.jsonl --landxml asbuilt.xml

# The same for every scan that appears in a folder, sent to the running controller, with the people in it
loam terrain site.loam scans/ --watch --ground-filter --objects --broker mqtts://sensors@site-server

# Next-pass surface for Cat Grade 3D (projected LandXML)
loam export site.loam --to landxml -o next.xml --ground scan.las --world-origin E,N --world-units usft

# Design (not next-pass) surface, site coordinates
loam export site.loam --to landxml --surface design -o design.xml

# Next-pass design grid for OpenGrade3D, master benchmark at site (0, 0)
loam export site.loam --to agd -o next.agd --ground scan.las --origin LAT,LON --world-origin E,N

# AgOpenGPS field folder: copy it into AgOpenGPS's Fields directory
loam export site.loam --to agopengps -o Fields/pad_north --origin LAT,LON

# Score a rover survey against the design inside a zone (bootstrap 95 % interval)
loam score site.loam rover.csv --zone pad --world-origin E,N

# Live machine position from a Reach streaming NMEA over TCP
nc reach.local 9001 | loam nmea site.loam --machine dz1 --origin LAT,LON

# The same from an AgOpenGPS GPS board, which broadcasts $PANDA or $PAOGI to UDP port 9999
loam nmea site.loam udp://:9999 --machine tr1 --origin LAT,LON --broker mqtts://tr1@site-server

# A grade box beside AgOpenGPS (zero and calibrate first; see "A grade box beside AgOpenGPS")
loam blade field.loam --valve serial:/dev/ttyUSB0 --gnss udp://:9999 --origin LAT,LON --box box.json --with-agopengps
```

Reading and gridding stream in chunks of 250,000 points, so memory stays near 25 MB however large the scan is (measured on 5 million points; the test suite checks 3 million). Gridding runs at roughly 20 million points a second on a laptop.

## File formats

Formats were taken from each tool's own source code, and the tests parse Loam's output with line-for-line restatements of those parsers:

- **AgOpenGPS** (`github.com/agopengps-official/AgOpenGPS`): `Field.txt` (`StartFix` origin), `Boundary.txt` (outer ring, then exclusion zones as inner rings; easting, northing, heading in radians), `TrackLines.txt` (AB tracks, mode 2; curve tracks, mode 4, written as AgOpenGPS writes a curve drawn on its own map: points at most a metre apart, a straight tail of 99 one-metre points at each end, each point's heading from its two neighbours).
- **OpenGrade3D** (`github.com/Pat-I/OpenGrade3D`): `.agd` rows `lat,lon,existing,proposed,cutfill,code` with codes `MB` (master benchmark), `2PER` (perimeter) and `3GRD` (grid). Heights are metres. `cutfill` is proposed minus existing (positive = fill); OpenGrade3D steers on the proposed height. A blank proposed height means "no design here". The grid is thinned to about 1 m spacing, as OpenGrade3D recommends.
- **LandXML 1.2** TIN surfaces, which Cat VisionLink lists among its accepted design files alongside Trimble formats.
- **Emlid Flow** CSV: columns found by name (`Easting`, `Northing`, `Elevation`, `Latitude`, `Longitude`, and a solution-status column if present; only FIX rows are kept). Headerless PNEZD and PENZD also work.
- **NMEA GGA**, and the `$PANDA` and `$PAOGI` sentences of AgOpenGPS GPS boards: checksums are verified. Anything but an RTK fixed solution is reported in the Wire state as `degraded` with a `gnssQuality` error, so a controller holds the machine instead of trusting the position. The state carries a `gnss` object with what the receiver said about its fix; a field the receiver left empty is `null`.
- **AgOpenGPS family messages** (`loam/aog.py`): the network frame `0x80 0x81 source pgn length data crc`; an OpenGrade valve board is sent a *cut value*, 100 on grade and one step per millimetre the edge is too high (to 199) or too low (to 1), and eight settings (gain up and down, least and greatest duty up and down, damping, deadband).
