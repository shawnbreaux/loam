# Field tools

Loam talks to field equipment through files and simple streams, so it works with the tools you already have and does not need anyone's SDK. This page shows what each tool exchanges with Loam and the commands that do it.

## Compatibility

| Tool | What it does on the test site | How Loam connects | Command |
|---|---|---|---|
| **Cat Grade with 3D** (dozers) | Holds the blade to a design surface | Loam writes the next-pass surface as LandXML; load it by USB or VisionLink | `loam export --to landxml` |
| Trimble, Topcon, Leica machine control | Same | LandXML, as above | `loam export --to landxml` |
| **OpenGrade3D** | Open-source blade control on a tractor | Loam writes the next-pass surface as an `.agd` design grid | `loam export --to agd` |
| **AgOpenGPS** | Open-source autosteer | Loam writes a field folder: origin, boundary, exclusion zones, one AB line per work zone | `loam export --to agopengps` |
| **Emlid Reach RS3** (rover) | Truth surveys | Loam reads Emlid Flow CSV exports (All columns, PNEZD, PENZD) | `loam score`, `--ground survey.csv` |
| Emlid Reach (or any RTK receiver) on a machine | Live position | Loam reads NMEA GGA (TCP or serial) and emits Wire `state` messages | `loam nmea` |
| **QField** (phone or tablet) or QGIS | Survey points | Loam reads GeoJSON points | `loam score`, `--ground points.geojson` |
| **Livox Mid-360** on a mast | Overhead scans | Record with livox_ros_driver2 and save PCD (or LAS); Loam grids each scan as it is written | `loam terrain --watch`, `--ground scan.pcd` |
| IP cameras with **Frigate** | Sees people on the site; calibrated to the ground, says where each one stands | Loam reads Frigate's MQTT messages | `loam sense --areas` |
| Phones with **OwnTracks** | Each person's GPS position | Loam reads OwnTracks' MQTT messages | `loam sense --tags` |
| Wi-Fi access points (**OpenWrt**, others) | Which phones are on the site, and near which access point | Loam reads hostapd's client table, or single signal readings | `loam sense --tags`, `loam wifi-fit` |
| **Meshtastic** trackers | A person's position where there is no phone signal | Loam reads the gateway's JSON over MQTT | `loam sense --tags` |
| **USGS 3DEP**, OpenDroneMap, any DEM | The ground of a field | Loam reads the GeoTIFF | `loam level`, `--ground dem.tif` |
| **Google Earth**, FSA field boundaries, **farmOS** | The field's boundary | Loam reads KML, KMZ, shapefile, GeoJSON | `loam level --boundary` |
| Wireless rain gauge with **rtl_433** | Rain rate | Loam reads rtl_433's MQTT events | `loam sense --rain` |
| **DJI Zenmuse L2** | Drone scans | Process in DJI Terra, export LAS; Loam grids it | `loam terrain`, `--ground scan.las` |
| Any lidar or photogrammetry tool | Scans | LAS 1.0–1.4 (uncompressed), PCD, or x y z text | same |

### What Loam cannot connect to

These have no public interface that third-party software can use to command the machine, so Loam can only feed them design files:

- **Cat Command** (remote control). It is a closed radio link between Cat's console or station and the machine. Loam cannot send it steering commands.
- **Teleo** and **Bluelight** retrofit kits. Both are closed. Ask each vendor whether they will accept orders over an API; Loam Wire is designed for exactly that.
- **DJI flight control.** Fly with DJI's own apps; Loam only reads the scans.
- **Livox SDK2 live stream.** Loam reads recordings (PCD/LAS), not the live UDP stream. `loam terrain --watch` publishes each recording as soon as it is written.
- **Cat VisionLink API.** It carries machine hours, location and fuel (ISO 15143-3), not control. A reader is possible later; it isn't needed for grading.

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
```

Reading and gridding stream in chunks of 250,000 points, so memory stays near 25 MB however large the scan is (measured on 5 million points; the test suite checks 3 million). Gridding runs at roughly 20 million points a second on a laptop.

## File formats

Formats were taken from each tool's own source code, and the tests parse Loam's output with line-for-line restatements of those parsers:

- **AgOpenGPS** (`github.com/agopengps-official/AgOpenGPS`): `Field.txt` (`StartFix` origin), `Boundary.txt` (outer ring, then exclusion zones as inner rings; easting, northing, heading in radians), `TrackLines.txt` (AB tracks, mode 2).
- **OpenGrade3D** (`github.com/Pat-I/OpenGrade3D`): `.agd` rows `lat,lon,existing,proposed,cutfill,code` with codes `MB` (master benchmark), `2PER` (perimeter) and `3GRD` (grid). Heights are metres. `cutfill` is proposed minus existing (positive = fill); OpenGrade3D steers on the proposed height. A blank proposed height means "no design here". The grid is thinned to about 1 m spacing, as OpenGrade3D recommends.
- **LandXML 1.2** TIN surfaces, which Cat VisionLink lists among its accepted design files alongside Trimble formats.
- **Emlid Flow** CSV: columns found by name (`Easting`, `Northing`, `Elevation`, `Latitude`, `Longitude`, and a solution-status column if present; only FIX rows are kept). Headerless PNEZD and PENZD also work.
- **NMEA GGA**: checksums are verified. Anything but an RTK fixed solution is reported in the Wire state as `degraded` with a `gnssQuality` error, so a controller holds the machine instead of trusting the position.
