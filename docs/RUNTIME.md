# Running a site: plan, controller, machines, monitor

Loam goes from the grading file a project manager receives to a monitored site with machines
taking orders. This page describes the parts, how to run them, and what the end-to-end tests showed.

```
 grading model (LandXML: existing + design)
        │  loam plan
        ▼
 site program (.loam, checked)          loam monitor ──► web dashboard (live, or replay)
        │  loam run                           ▲
        ▼                                     │ status, orders, state, terrain
 ┌─────────────── MQTT broker (Mosquitto) ────┴──────────────────────────────┐
 │ loam/v0/<site>/<machine>/order ▼     state, connection ▲                    │
 │ loam/v0/<site>/terrain ▲   safety ▼   estop / detection / weather ▲         │
 └──────┬──────────────┬──────────────┬──────────────┬───────────────────────┘
        │              │              │              │
     dozer 1        dozer 2        roller         lidar probe        (real machines with a Loam Wire
     process        process        process        process             adapter, or `loam simulate-site`)
        └──────────────┴── sim/<site>/world/... ─────┘
                 world process: true ground, soil physics, site sensors, clock
                 (simulation only; the controller never sees these topics)
```

## 1. Plan: `loam plan`

```bash
loam plan job.xml -o job.loam                   # existing + design in one LandXML
loam plan design.xml --existing-file topo.xml -o job.loam --fleet my_fleet.loam --tolerance "3 cm"
```

It reads the units and coordinate system from the file and picks the design and existing surfaces by
name (`FG`, `Proposed`, `EG`, `Existing`, ...), or takes `--design` / `--existing`. Then it:

- traces the job limits from the outer edge of the design surface and simplifies them to half a grid
  cell; outside the design surface no work is wanted;
- sizes the grid to stay under `--max-cells` (40,000 by default);
- writes a finish zone for the limits, plus derived cut and fill zones (`area where residual > tol`)
  that shrink as the work gets done;
- works out the earthwork balance and places a stockpile for the surplus or the borrow;
- writes the tasks (cut to fill, top up, compact, finish) with measured `until` conditions, safety
  rules and a drift replan;
- marks everything the file cannot tell it, such as fleet, stockpile location and coordinate system,
  with `# CHECK:`, and checks the program before returning.

A fleet file is plain Loam: `machine` and `probe` declarations.

## 2. Control: `loam run`

```bash
loam run job.loam --broker site-server:1883 --record orders.jsonl
```

The controller is the reference simulator's planner with the world taken away. The task, rule, replan
and safety logic is the same code, `Sim.step`. What changes:

| | Reference simulator | `loam run` |
| --- | --- | --- |
| Measured surface | simulated scan | `terrain` messages |
| A pass lands | after its planned duration | when the machine reports `progress: 1.0` |
| People, rain, e-stop | scripted | `detection`, `weather`, `estop` messages, from `loam sense` and `loam estop` or your own sensors |
| Clocks | one clock | a machine whose clock is more than 2 s off the controller's gets no orders |
| A machine drops off | n/a | pass withdrawn; the machine gets no orders until it is back online, fresh, and `safetyState: ok` |
| A machine stays unsafe | n/a | after 30 site minutes not `ok` (outside an e-stop) its pass is withdrawn, e.g. after a rollover |

Two details matter on a real network.

- **Scans and completions are matched by time.** A machine reports completion only after the work is
  done, stamped with the time it finished. Before using a scan, the controller waits until every
  machine with a pass in flight has reported a state at or after the scan time (or 2 s go by). It then
  takes out of its plan any pass that finished after the scan. Without this, a scan that already
  shows a pass the controller has not yet booked counts that pass twice; that was the first bug the
  end-to-end tests found.
- **Done means done.** A task whose `until` already holds on measured ground gets no new passes, so
  its last passes can land and it can close.

## 3. Simulated site: `loam simulate-site`

```bash
mosquitto -p 1883 &
loam simulate-site job.loam --speed 300 --faults faults.json &
loam monitor job.loam --port 8080 --record run.jsonl &     # open http://127.0.0.1:8080/
loam run job.loam
```

The launcher starts the world, one probe and one process per machine. Each machine:

- takes one order at a time and drives the work window and the haul;
- stops at once on `stop`, a site e-stop, a missing safety heartbeat (3 s) or a GNSS solution that is
  not RTK fixed;
- stays stopped when the e-stop is released (`estopReset`): resetting an e-stop must not restart a
  machine (ISO 13850). A person restarts the site with `loam resume job.loam`, or one machine with
  `--machine`, which sends `resume` on the safety topic or as an instant action;
- on SIGTERM (systemd, Docker), as on Ctrl-C, holds its vehicle and reports itself offline before it exits;
- reports state four times a second;
- has a retained connection status with `connectionBroken` as its last will;
- never executes an order twice, and drops its current order if its radio comes back after a loss.

With `--physics`, each machine is a PyBullet rigid body steered by pure pursuit over the site's ground:
a six-roller undercarriage, a low centre of mass, and roll and pitch inertia raised the way wide tracks
resist tipping. Its pose, heading and tilt then come from the physics engine. If it tips past 45° it
reports `safetyState: stopped` with a `rollover` error and stays out of service. Use `--speed 30` or less, because physics is CPU-bound. Gazebo is not used: its and ROS 2's
package servers were not reachable from the build environment, so that stage remains to be done on a
machine that can install them.

### Faults (`--faults`, site minutes)

`resume_after_min` is the simulated supervisor: a site-wide `resume` that many minutes after the
e-stop is released. Leave it out and the site stays stopped, as a real one would until someone resumes it.

```json
{"person":       [{"from": 42, "to": 45, "near": "dz1", "distance_m": 9.0}],
 "probe_outage": [{"from": 95, "to": 118}],
 "rain":         [{"from": 150, "to": 175, "rate_m_per_s": 2.8e-06}],
 "gnss_float":   [{"machine": "dz2", "from": 60, "to": 70}],
 "link_loss":    [{"machine": "dz1", "at": 130, "wall_s": 4.0}],
 "estop":        [{"at": 200, "for_min": 3, "resume_after_min": 1}],
 "crash":        [{"machine": "dz2", "at": 170}]}
```

## Securing the broker

On a real site, anyone who can reach an open broker can publish an order. Run the broker encrypted, with a
name and password for every client and with access rules. `examples/broker/` holds a Mosquitto configuration
and the rules:

| User | May |
| --- | --- |
| `controller` | read and write everything under `loam/v0/` |
| `supervisor` | read everything; publish the site stop, `resume` and the e-stop |
| `sensors` | publish detections, weather, terrain and the e-stop; read the camera system's and the rain gauge's topics |
| a machine, logged in under its machine name | read its own orders and the site's safety topics; publish its own state |

A machine cannot publish an order, not even to itself, and cannot speak for another machine.

Create the site's certificate authority and the broker's certificate once. Current TLS libraries refuse a
certificate authority without the key-usage extension, so keep the `-addext` lines:

```bash
openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -subj "/CN=Loam site CA" \
  -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=critical,keyCertSign,cRLSign" \
  -keyout ca.key -out ca.crt
openssl req -newkey rsa:2048 -nodes -subj "/CN=site-server" -keyout server.key -out server.csr
printf "subjectAltName=DNS:site-server,IP:192.168.1.10\nkeyUsage=digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\nsubjectKeyIdentifier=hash\nauthorityKeyIdentifier=keyid\n" > san.cnf
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -days 825 -extfile san.cnf -out server.crt
mosquitto_passwd -c passwords controller      # then, without -c: supervisor, sensors, dz1, ...
chmod 600 server.key passwords acl
mosquitto -c mosquitto.conf
```

On Ubuntu the packaged Mosquitto is confined by AppArmor and reads files only under `/etc/mosquitto`:
put the configuration in `/etc/mosquitto/conf.d/`, the other files beside it, and name them by their full paths.

`subjectAltName` must hold the name or address the clients connect to. Keep `ca.key` off the site server. Every
client gets a copy of `ca.crt` and connects with `mqtts://`:

```bash
export LOAM_BROKER_CA=ca.crt LOAM_BROKER_PASSWORD_FILE=/etc/loam/password
loam run job.loam --broker mqtts://controller@site-server
loam mavlink job.loam --machine dz1 --vehicle … --broker mqtts://dz1@site-server
```

- The password comes from `LOAM_BROKER_PASSWORD` or the file named by `LOAM_BROKER_PASSWORD_FILE`. A password
  in the address is refused, because every user of the computer can read it in the process list.
- `LOAM_BROKER_USER` names the user when the address does not. `LOAM_BROKER_CERT` and `LOAM_BROKER_KEY` add a
  client certificate. Without `LOAM_BROKER_CA` the system's certificate store is used.
- TLS 1.2 or later, and the broker's certificate and host name are always checked.
- A wrong password or a certificate that does not match is reported in words, not as a timeout.

Tested against Mosquitto with exactly these files (`tests/test_security.py`): a machine reads only its own
orders, and an order it tries to publish is not delivered.

## 4. Monitor: `loam monitor`

The dashboard shows:

- the latest survey as measured-minus-design: red is high and needs cutting, blue is low and needs fill;
- machines with heading, work windows and deposit windows, and people detected;
- tasks, machine link and safety state, rules in force, replans and the event log;
- on a simulated site, the world's ground-truth score, labelled as such.

It uses Server-Sent Events, has no JavaScript dependencies, and nothing leaves the machine.
`loam monitor job.loam --replay run.jsonl --html replay.html` builds a self-contained replay page
(`--split` writes `index.html` + `run.json` instead). `--mcap run.mcap` records every Wire message for
Lichtblick or PlotJuggler (see [INTEGRATIONS.md](INTEGRATIONS.md)).

## 5. What the end-to-end tests showed

The test files are real design surfaces:

- the Autodesk Land Desktop "subdivision" sample (FG, feet);
- a Civil 3D design model from a Norwegian project ("00201-31B Total topp", metres).

Both come from public GitHub repositories under open licences. **No public file pairs a design with a
matching existing-ground survey** (the subdivision sample's EG sits about 1,000 ft below its FG), so
existing ground was synthesized: the design plus smooth undulation. Results are on the world's true
ground. Everything runs at 300× real time unless noted, with the full fault script above.

| Job | Machines | Faults | Site time | In tolerance (true ground) | Replans |
| --- | --- | --- | --- | --- | --- |
| `examples/pad_north.loam`, 160 × 100 ft, ±0.1 ft | 2 dozers, roller | all 7 | 240 min | 97.3 % | 18 |
| subdivision lots, 679 yd³ cut, ±0.1 ft | 2 dozers, roller | all 7 (dz2 crashes at 170 min) | 37.4 h | 95.8 % | 5 |
| Total topp, 195 m³ cut, ±3 cm | 2 dozers, roller | all 7 | 279 min | 97.6 % | 2 |
| `pad_north`, physics machines (PyBullet), 30× | 2 dozers, roller | all 7 | 339 min | 97.9 % | 6 |
| `pad_north`, loopback (no broker, deterministic) | 2 dozers, roller | none | 420 min | 97.8 % | 6 |

In every run:

- every fault was handled: the stop rule fired, the GNSS hold applied, the stale-data hold covered the
  lidar outage, the lost pass was withdrawn, compaction paused for rain, all machines stopped on the
  e-stop, and the crashed machine's work moved to the remaining dozer;
- the volume ledger closed to about 1e-11;
- the e-stop reached every machine within about 1 ms of wall time on a local broker (a radio link adds
  its own latency; measure it on site);
- every process exited cleanly, apart from the one that was crashed on purpose (exit code 3).

The completion rule (`in_tolerance` = 95 % of cells as measured) is what ends a job. The cells still
out at the end cluster along the job's edge, where the design ties into existing ground.

### Bugs the end-to-end tests found and fixed

1. **Double-counted passes.** A scan could include a pass whose completion the controller had not yet
   received. Fixed by time-stamped completions and scan reconciliation (above).
2. **Machines reported 1.0 too early.** They reported completion before the earth had actually moved.
   They now report 0.999 until then.
3. **Spoil spread differently from the plan.** Orders gave only a deposit rectangle, so the machine
   spread spoil in a different place from the one planned. Orders now carry per-cell patterns
   (`target.pattern`, `deposit.pattern`).
4. **Roller modelled on too few cells.** The planner modelled the roller compacting only the cells
   ready when it planned the pass. A roller compacts everything it drives over.
5. **Tasks that never closed.** With one dozer left, a task whose `until` already held kept getting
   noise-driven passes and never closed.
6. **Retained state from an earlier run.** A retained "done" status from a previous run ended the next
   one at once. The launcher now clears retained topics.
7. **Lost messages at start-up.** Messages that arrived in the same batch as the clock were dropped,
   including the fault script.
8. **Fault replay.** A lost-radio fault replayed on every reconnect. Person detections flooded the
   broker.
9. **A silent rollover.** In the first physics run a dozer body rolled over and kept crawling
   upside down, and nothing reported it. Rollovers are now detected and reported, the body was made
   much harder to tip, and the controller withdraws a pass whose machine stays unsafe for 30 minutes.
10. **Speed limit for testing.** Above about 600× real time on two cores, wall-clock latency becomes
   many site minutes and planning degrades. The tests use 300×. With `--physics` at 300×, the
   machines' heartbeats went stale and every machine stopped itself, which is the correct fail-safe
   behaviour.

### Limits

- Soil physics is the simulator's simple model: swell, compaction keep, blade error. There is no
  slip, no material flow and no stability modelling.
- The world applies a pass's effect at completion. Real ground changes during the pass.
- Physics machines model motion only. The blade's interaction with soil stays in the world process.
- One probe per site. Lidar occlusion by machines is not modelled.
- People are located only as well as the cameras allow: a detection is a circle, not a point, and cameras
  miss people. `loam sense` is supervision, not a certified safety function.
- Results are simulation results, not field measurements.
