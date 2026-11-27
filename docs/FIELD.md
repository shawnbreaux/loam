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
| IP cameras with **Frigate** | Sees people on the site | Loam reads Frigate's MQTT messages | `loam sense --areas` |
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

## Commands

```
# Mast scan to the site grid; also as a Wire terrain message and an as-built surface
loam terrain site.loam scan.pcd --world-origin E,N --wire terrain.jsonl --landxml asbuilt.xml

# The same for every scan that appears in a folder, sent to the running controller
loam terrain site.loam scans/ --watch --ground-filter --broker mqtts://sensors@site-server

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
