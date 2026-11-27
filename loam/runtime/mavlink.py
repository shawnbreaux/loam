"""ArduPilot Rover (and any MAVLink ground vehicle) as a Loam machine.

`loam mavlink` runs one bridge per vehicle. Loam Wire on one side, MAVLink 2
on the other (UDP, TCP or serial, as pymavlink accepts). ArduPilot is GPL and
is never linked: the bridge speaks the protocol only, through pymavlink
(LGPL-3.0).

Orders become missions: the bridge works out the same path the simulated
machine drives (work window, haul to the deposit or from the source), turns
it into WGS84 waypoints through the site frame, uploads it with a
DO_CHANGE_SPEED for the order's speed limit, and starts it in AUTO. The pass
is done when the last waypoint is reached.

Safety mapping:

* `stop`, site stop, or no safety heartbeat for 3 s → HOLD; `resume` → AUTO;
* e-stop → forced disarm (the vehicle needs re-arming by a person);
* GNSS fix below `min_fix` (6 = RTK fixed) → HOLD, `safetyState` degraded;
* no vehicle heartbeat for 3 s → `safetyState` stopped (vehicleLinkLost).

By default the bridge steers only, and blade height stays with the machine's
own grade control, which takes the surfaces from `loam export`. A vehicle with
nobody aboard can instead have its blade held by the bridge (`--blade-servo`,
see `blade.py`): the edge follows the next-pass surface, moved through one of
the autopilot's servo outputs (MAV_CMD_DO_SET_SERVO), with the height taken
from the autopilot's own position, or from its second GPS when that one's
antenna rides on the blade (`--blade-gps2`). A height older than a second, or
from a second GPS without an RTK fix, is no height: the blade is carried.
"""
from __future__ import annotations

import math
import os
import sys
import time
from typing import Any

# pymavlink chooses its protocol version when it is first imported: ask for MAVLink 2 before that can happen.
os.environ.setdefault("MAVLINK20", "1")

from ..checker import Model
from ..geo import Frame, convex_hull, offset_polygon
from ..units import length_unit_si
from .blade import Blade
from .bridge import run_bridge
from .machine import HEARTBEAT_TIMEOUT_S

# Numbers from the MAVLink common message set and ArduPilot Rover: GPS_FIX_TYPE names, command ids, the coordinate
# frame for waypoints, Rover's mode numbers, and the magic value that forces a disarm.
FIX_NAMES = {0: "no-gps", 1: "no-fix", 2: "2d", 3: "3d", 4: "dgps", 5: "rtk-float", 6: "rtk-fixed"}
CMD_WAYPOINT, CMD_CHANGE_SPEED, CMD_SET_SERVO, CMD_ARM = 16, 178, 183, 400
FRAME_REL_ALT = 3                     # MAV_FRAME_GLOBAL_RELATIVE_ALT (the INT variant takes the same id)
MODE_HOLD, MODE_AUTO = 4, 10          # ArduPilot Rover custom modes
FORCE_DISARM = 21196
HEIGHT_FRESH_S = 1.0                  # a height older than this is not graded by


# Fences travel by the same upload protocol as missions, told apart by the mission type.
MISSION, FENCE = 0, 1                 # MAV_MISSION_TYPE_MISSION, MAV_MISSION_TYPE_FENCE
CMD_FENCE_INSIDE, CMD_FENCE_OUTSIDE = 5001, 5002   # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION / _EXCLUSION
GCS_SYSTEM = 254                      # the MAVLink system id this bridge speaks as
# What `Vehicle.setup` asks of an ArduPilot Rover, so that the vehicle protects itself when Loam, the radio or
# this bridge fails. Each entry: the parameter (and its name in older firmware), the value, and what it does.
VEHICLE_SETUP: list[tuple[tuple[str, ...], float, str]] = [
    (("FENCE_ENABLE",), 1, "fences on"),
    (("FENCE_TYPE",), 4, "polygon fences"),
    (("FENCE_ACTION",), 2, "hold at a fence breach"),
    (("OA_TYPE",), 2, "plan routes around fences (Dijkstra)"),
    (("FS_GCS_ENABLE",), 1, "ground-station failsafe on"),
    (("FS_ACTION",), 2, "hold on failsafe"),
    (("FS_GCS_TIMEOUT", "FS_TIMEOUT"), 3, "seconds without this bridge's heartbeat before the failsafe"),
    (("MAV_GCS_SYSID", "SYSID_MYGCS"), GCS_SYSTEM, "the failsafe watches this bridge, not an operator's laptop"),
]


def bearing_to_heading(bearing_deg: float, frame: Frame) -> float:
    """Compass bearing (degrees clockwise from north) to a site heading (radians from +x, CCW)."""
    return (math.pi / 2 - math.radians(bearing_deg - frame.rotation_deg)) % (2 * math.pi)


def _e7(deg: float) -> int:
    """Degrees as the integer MAVLink sends: degrees times ten million, rounded half up (about 1 cm on the ground)."""
    return math.floor(float(deg) * 1e7 + 0.5)


def mission_items(path: list[tuple[float, float]], frame: Frame, speed_ms: float | None) -> list[tuple]:
    """MISSION_ITEM_INT fields (seq, frame, command, p1..p4, lat*1e7, lon*1e7, alt) for a site path.
    Item 0 is ArduPilot's home slot, which it overwrites; waypoints follow, skipping the start point."""
    lat, lon = frame.local_to_wgs84([p[0] for p in path], [p[1] for p in path])
    items: list[tuple] = [(0, 0, CMD_WAYPOINT, 0, 0, 0, 0, _e7(lat[0]), _e7(lon[0]), 0)]
    if speed_ms:
        items.append((len(items), FRAME_REL_ALT, CMD_CHANGE_SPEED, 1, speed_ms, -1, 0, 0, 0, 0))
    for la, lo in zip(lat[1:], lon[1:], strict=True):
        items.append((len(items), FRAME_REL_ALT, CMD_WAYPOINT, 0, 0, 0, 0,
                      _e7(la), _e7(lo), 0))
    return items


def fence_items(model: Model, frame: Frame, margin_m: float = 10.0) -> list[tuple]:
    """The site's limits as MAVLink fence items, in the same tuple layout as `mission_items`.

    * One inclusion polygon, which the vehicle may not leave: the convex hull of the site boundary, every zone
      and every stockpile, grown by `margin_m` metres for turning room. Stockpiles often lie outside the stated
      boundary, so the boundary alone would fence a machine off from its own work.
    * One exclusion polygon per exclusion zone, grown by the zone's buffer, which the vehicle may not enter.

    Each item is one corner and carries the number of corners of its polygon. A program that names no areas at
    all gives no fence.
    """
    unit = model.site.get("units", "ft")
    areas = [(model.site.get("boundary") or {}).get("polygon")]
    areas += [(z.get("area") or {}).get("polygon") for z in [*model.zones.values(), *model.stockpiles.values()]]
    corners = [p for poly in areas if poly for p in poly]
    polys: list[tuple[int, list]] = []
    if len(corners) >= 3:
        polys.append((CMD_FENCE_INSIDE, offset_polygon(convex_hull(corners), margin_m / length_unit_si(unit))))
    for z in model.zones.values():
        if z.get("role") == "exclusion" and "polygon" in (z.get("area") or {}):
            buffer = z["buffer"].to(unit) if z.get("buffer") is not None else 0.0
            polys.append((CMD_FENCE_OUTSIDE, offset_polygon(z["area"]["polygon"], buffer)))
    items: list[tuple] = []
    for cmd, poly in polys:
        lat, lon = frame.local_to_wgs84([p[0] for p in poly], [p[1] for p in poly])
        for la, lo in zip(lat, lon, strict=True):
            items.append((len(items), 0, cmd, len(poly), 0, 0, 0, _e7(la), _e7(lo), 0))     # frame 0: MAV_FRAME_GLOBAL
    return items


class Vehicle:
    """A MAVLink ground vehicle as a bridge adapter: telemetry, missions, HOLD/AUTO, arming."""

    def __init__(self, url: str, frame: Frame, min_fix: int = 6, timeout: float = 30.0,
                 fences: list[tuple] | None = None, blade_servo: int | None = None, blade_gps2: bool = False):
        """Connect and wait up to `timeout` seconds for the vehicle's heartbeat.

        `url` is a pymavlink connection string (for example `udpin:0.0.0.0:14550` or `/dev/ttyUSB0`). `frame`
        places the site grid on the Earth. `min_fix` is the lowest GNSS fix type good enough to work by (6 is RTK
        fixed, centimetre level). With `fences` (from `fence_items`; may be empty) the vehicle is set up to
        protect itself: see `setup`. With None its parameters and fences are left alone. `blade_servo` is the
        number of the autopilot's servo output that moves the blade, if the bridge is to hold it; with
        `blade_gps2` the blade's height is read from the autopilot's second GPS (its antenna on the blade)
        and not from the vehicle's own position.
        """
        try:
            from pymavlink import mavutil
        except ImportError as e:
            raise ValueError("MAVLink vehicles need pymavlink: pip install pymavlink") from e
        self.frame, self.min_fix = frame, min_fix
        if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
            # Something imported pymavlink before this module could ask for MAVLink 2. Fences and their
            # acknowledgements only exist in MAVLink 2, so do not run half-protected.
            raise ValueError("pymavlink is running MAVLink 1; set MAVLINK20=1 in the environment")
        # The bridge identifies itself as a ground station (system 254), component 191 (MAV_COMP_ID_ONBOARD_COMPUTER).
        self.m: Any = mavutil.mavlink_connection(url, source_system=GCS_SYSTEM, source_component=191)
        if self.m.wait_heartbeat(timeout=timeout) is None:
            self.m.close()
            raise ValueError(f"no MAVLink heartbeat on {url}")
        self.pose: tuple[float, float, float] | None = None
        self.height: float | None = None        # the height to grade by, in site units, and when it was reported
        self.height_at = 0.0
        self.blade_servo, self.blade_gps2 = blade_servo, blade_gps2
        self.fix, self.armed, self.mode, self.reached = 0, False, -1, -1
        self.n_wp, self.paused, self.stopped_hard = 0, True, False
        self.sent_at = 0.0
        self.failed: str | None = None          # why the current order cannot run, until the next order
        self.last_hb = time.time()
        self.beat_at = 0.0                      # when this bridge last sent its own heartbeat
        self.breached = False                   # the vehicle reports it is outside its fences
        # GLOBAL_POSITION_INT, GPS_RAW_INT, FENCE_STATUS, and GPS2_RAW when the blade's antenna is the second GPS
        for msg_id, hz in ((33, 10), (24, 5), (162, 2), *(((124, 10),) if blade_gps2 else ())):
            self.command(511, msg_id, 1e6 / hz)                # MAV_CMD_SET_MESSAGE_INTERVAL
        if fences is not None:
            try:
                self.setup(fences)
            except ValueError:
                self.m.close()
                raise

    # -- adapter -------------------------------------------------------------------------
    @property
    def altitude(self) -> float | None:
        """The height to grade by, in site units, or None if none has been reported within the last second."""
        return self.height if time.time() - self.height_at <= HEIGHT_FRESH_S else None

    def _height(self, alt_mm: float) -> None:
        """Take in a height reported in millimetres above mean sea level."""
        self.height = float(self.frame.height_to_local(alt_mm / 1000.0 / self.frame.world_m))
        self.height_at = time.time()

    @property
    def fix_ok(self) -> bool:
        """Is the GNSS fix good enough to grade by?"""
        return self.fix >= self.min_fix

    @property
    def fix_name(self) -> str:
        """The fix type in words, for the state message."""
        return FIX_NAMES.get(self.fix, str(self.fix))

    @property
    def fault(self) -> str | None:
        """`vehicleLinkLost` when the vehicle's own heartbeat has not been heard for 3 s; `vehicleFenceBreach`
        while it reports being outside its fences (it holds there until a person drives it back); else None.
        """
        if time.time() - self.last_hb >= HEARTBEAT_TIMEOUT_S:
            return "vehicleLinkLost"
        return "vehicleFenceBreach" if self.breached else None

    @property
    def order_error(self) -> str | None:
        """Why the current order cannot run (the mission was refused), or None."""
        return self.failed

    @property
    def blocked(self) -> None:
        """Never: an ArduPilot vehicle does not report protective stops of its own."""
        return None

    def poll(self) -> None:
        """Read every MAVLink message that has arrived: heartbeat (armed, mode), position, fix type, waypoint reached,
        fence status. Also send this bridge's own heartbeat when one is due.
        """
        self._beat()
        while (msg := self.m.recv_match(blocking=False)) is not None:
            t = msg.get_type()
            # Only the vehicle's own heartbeat counts. Type 6 is a ground station, such as an operator's laptop on the
            # same link.
            if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != 6:
                self.last_hb = time.time()
                self.armed = bool(msg.base_mode & 128)
                self.mode = msg.custom_mode
            elif t == "GLOBAL_POSITION_INT":
                if msg.lat == 0 and msg.lon == 0:      # ArduPilot sends zeros until the EKF has an origin
                    continue
                x, y = self.frame.wgs84_to_local(msg.lat / 1e7, msg.lon / 1e7)
                h = bearing_to_heading(msg.hdg / 100.0, self.frame) if msg.hdg != 65535 else \
                    (self.pose[2] if self.pose else 0.0)
                self.pose = (float(x), float(y), h)
                if not self.blade_gps2:
                    self._height(msg.alt)              # as the autopilot's position estimate has it
            elif t == "GPS2_RAW" and self.blade_gps2 and msg.fix_type >= self.min_fix:
                self._height(msg.alt)                  # the antenna on the blade, and only on a fix good enough
            elif t == "GPS_RAW_INT":
                self.fix = msg.fix_type
            elif t == "MISSION_ITEM_REACHED":
                self.reached = msg.seq
            elif t == "FENCE_STATUS":
                self.breached = bool(msg.breach_status)

    def start(self, path: list[tuple[float, float]], order: dict, max_speed_ms: float | None) -> None:
        """Upload the order as a mission. The vehicle is in HOLD for the upload (which blocks for up to
        10 s). If it refuses or does not answer, the last mission is still on board: nothing of it may
        count toward this order, and the vehicle must not be put in AUTO to run it again."""
        self.n_wp, self.reached, self.paused, self.failed = 0, -1, True, None
        self.set_mode(MODE_HOLD)
        try:
            items = mission_items(path, self.frame, max_speed_ms)
            self.upload(items)
        except ValueError as e:
            self.failed = "vehicleMissionRejected"
            print(f"loam: {order.get('orderId')}: {e}", file=sys.stderr, flush=True)
            return
        self.n_wp = len(items) - 1
        if not self.armed and not self.stopped_hard:
            self.arm(True)

    def pause(self, on: bool) -> None:
        """Hold the vehicle (`on`) or let it run its mission.

        The command is repeated once a second until the vehicle's heartbeat shows the mode, because a single
        MAVLink command can be lost. AUTO needs the vehicle armed, so a resume arms first and sets the mode on a
        later call. A vehicle stopped by an e-stop is never re-armed from here.
        """
        on = on or self.n_wp == 0               # no mission of ours on board: HOLD is the only mode
        want = MODE_HOLD if on else MODE_AUTO
        if on != self.paused or (self.mode != want and time.time() - self.sent_at > 1.0):
            self.sent_at = time.time()          # resend until the vehicle's heartbeat shows the mode
            if on or self.armed:
                self.set_mode(MODE_HOLD if on else MODE_AUTO)
            elif not self.stopped_hard:
                self.arm(True)                    # AUTO needs arming; the next poll sets the mode
                return
            self.paused = on

    def estop(self) -> None:
        """Emergency stop: HOLD and a forced disarm. From then on only a person at the vehicle can arm it."""
        self.stopped_hard = True
        self.set_mode(MODE_HOLD)
        self.arm(False, force=True)

    def fraction(self) -> float:
        """How far through the mission, 0 to 1, by waypoints reached."""
        return max(self.reached, 0) / max(self.n_wp, 1)

    def done(self) -> bool:
        """Has the last waypoint of this order's mission been reached?"""
        return self.n_wp > 0 and self.reached >= self.n_wp

    def close(self) -> None:
        """Close the MAVLink connection."""
        self.m.close()

    def implement(self, pwm: int) -> None:
        """Move the blade's actuator: set the blade servo output to a pulse width in microseconds."""
        if self.blade_servo is not None:
            self.command(CMD_SET_SERVO, self.blade_servo, pwm)

    # -- MAVLink -------------------------------------------------------------------------
    def command(self, cmd: int, *p: float) -> None:
        """Send a COMMAND_LONG with up to seven parameters (the rest are zero)."""
        q = [*p, *[0.0] * (7 - len(p))]
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *q)

    def upload(self, items: list[tuple], timeout: float = 10.0, kind: int = MISSION) -> None:
        """Upload a mission (or, with `kind=FENCE`, the fences) with the MAVLink mission protocol: send the count,
        then answer each request for an item until the vehicle acknowledges.

        Raises ValueError if the vehicle rejects the upload or does not finish within `timeout` seconds.
        """
        m = self.m
        what = "fence" if kind == FENCE else "mission"
        m.mav.mission_count_send(m.target_system, m.target_component, len(items), kind)
        end = time.time() + timeout
        while time.time() < end:
            self._beat()
            msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True,
                               timeout=0.5)
            if msg is None or getattr(msg, "mission_type", MISSION) != kind:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type != 0:
                    raise ValueError(f"vehicle rejected the {what} (MAV_MISSION_RESULT {msg.type})")
                return
            s, fr, cmd, p1, p2, p3, p4, x, y, z = items[msg.seq]
            m.mav.mission_item_int_send(m.target_system, m.target_component, s, fr, cmd, 0, 1,
                                        p1, p2, p3, p4, x, y, z, kind)
        raise ValueError(f"{what} upload timed out")

    def _beat(self) -> None:
        """Send this bridge's heartbeat once a second, as a ground station (type 6, no autopilot: 8).

        The vehicle's ground-station failsafe watches for it: if this bridge or the radio link dies, the vehicle
        holds by itself (see `setup`).
        """
        if time.time() - self.beat_at >= 1.0:
            self.beat_at = time.time()
            self.m.mav.heartbeat_send(6, 8, 0, 0, 0)

    def param(self, names: tuple[str, ...], value: float | None = None, timeout: float = 2.0) -> tuple[str, float]:
        """Read a parameter, or set it when `value` is given; returns its name and the value the vehicle reports.

        `names` lists the parameter's name in current firmware first, then older names. Raises ValueError if the
        vehicle answers to none of them.
        """
        m = self.m
        for name in names:
            if value is None:
                m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
            else:
                m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), 9)  # REAL32
            end = time.time() + timeout
            while time.time() < end:
                self._beat()
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if msg is not None and str(msg.param_id).rstrip("\x00") == name:
                    return name, float(msg.param_value)
        raise ValueError(f"the vehicle does not answer for parameter {' / '.join(names)}")

    def setup(self, fences: list[tuple]) -> None:
        """Make the vehicle protect itself, and say what was changed.

        * Its ground-station failsafe is switched on and pointed at this bridge: without the bridge's heartbeat
          for 3 s (bridge crashed, radio lost), the vehicle holds.
        * `fences` are uploaded and switched on: the vehicle holds if it leaves the working area or enters an
          exclusion zone, and plans its routes around them, whatever mission it is given.

        Parameters that already have the wanted value are not touched. The changes are stored on the vehicle
        and stay after this program ends. Raises ValueError if the vehicle refuses any of it: better not to
        start than to run unprotected.
        """
        for names, want, why in VEHICLE_SETUP:
            if not fences and names[0] in ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "OA_TYPE"):
                continue                                  # the program states no boundary and no exclusion zone
            name, have = self.param(names)
            if abs(have - want) > 1e-6:
                name, now = self.param((name,), want)
                if abs(now - want) > 1e-6:
                    raise ValueError(f"the vehicle did not accept {name} = {want:g} (it reports {now:g})")
                print(f"loam: vehicle parameter {name}: {have:g} -> {want:g} ({why})", file=sys.stderr, flush=True)
        if fences:
            self.upload(fences, kind=FENCE)
            n, i = 0, 0
            while i < len(fences):                        # each corner carries its polygon's corner count
                n, i = n + 1, i + int(fences[i][3])
            print(f"loam: uploaded {n} fence polygon(s), {len(fences)} corners", file=sys.stderr, flush=True)

    def set_mode(self, mode: int) -> None:
        """Ask for an ArduPilot custom mode (the 1 says the number is a custom mode)."""
        self.m.mav.set_mode_send(self.m.target_system, 1, mode)

    def arm(self, on: bool, force: bool = False) -> None:
        """Arm or disarm the motors. `force` disarms even while moving."""
        self.command(CMD_ARM, 1 if on else 0, FORCE_DISARM if force else 0)


def run_mavlink(model: Model, name: str, broker: str, url: str, frame: Frame, min_fix: int = 6,
                sim_world: bool = False, max_wall_s: float | None = None, setup: bool = True,
                blade: Blade | None = None, blade_servo: int | None = None, blade_gps2: bool = False):
    """Bridge one MAVLink vehicle as Loam machine `name` (see `bridge.run_bridge`).

    With `setup` (the default) the vehicle is first given the site's fences and its failsafe is switched on; see
    `Vehicle.setup`. Without, its own configuration is trusted as it is. With `blade` and `blade_servo` the
    bridge also holds the vehicle's blade to the next-pass surface through that servo output, by the height of
    the vehicle or, with `blade_gps2`, of the second GPS antenna on the blade.
    """
    fences = fence_items(model, frame) if setup else None
    vehicle = Vehicle(url, frame, min_fix, fences=fences, blade_servo=blade_servo if blade else None,
                      blade_gps2=bool(blade) and blade_gps2)
    return run_bridge(model, name, broker, vehicle, sim_world, max_wall_s, blade=(blade, vehicle) if blade else None)


def blade_zero(url: str, frame: Frame, servo: int, down_pwm: int, elevation: float, seconds: float = 3.0,
               gps2: bool = False, min_fix: int = 0) -> dict:
    """Measure a blade's `height` for `--blade-height`: lower the blade onto a mark of known `elevation` (site
    units), read the vehicle's altitude for a few seconds, and return how far it is above the cutting edge.

    Returns {"blade_height", "altitude", "spread", "samples"}; `spread` is the highest reading minus the lowest,
    which says how steady the height is. `gps2` reads the second GPS, as `--blade-gps2` does.
    """
    v = Vehicle(url, frame, min_fix=min_fix, blade_servo=servo, blade_gps2=gps2)
    try:
        v.implement(down_pwm)
        got: list[float] = []
        end, seen = time.time() + seconds, 0.0
        while time.time() < end:
            v.poll()
            if v.height is not None and v.height_at != seen:        # a new reading
                seen = v.height_at
                got.append(v.height)
            time.sleep(0.05)
    finally:
        v.close()
    if not got:
        raise ValueError("the vehicle reported no position: wait for its position estimate and try again")
    mean = sum(got) / len(got)
    return {"blade_height": round(mean - elevation, 4), "altitude": round(mean, 4),
            "spread": round(max(got) - min(got), 4), "samples": len(got)}
