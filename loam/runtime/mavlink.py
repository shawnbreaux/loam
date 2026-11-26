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

The bridge steers only. Blade height stays with the machine's own grade
control, which takes the surfaces from `loam export`.
"""
from __future__ import annotations

import math
import os
import sys
import time
from typing import Any

from ..checker import Model
from ..geo import Frame
from .bridge import run_bridge
from .machine import HEARTBEAT_TIMEOUT_S

# Numbers from the MAVLink common message set and ArduPilot Rover: GPS_FIX_TYPE names, command ids, the coordinate
# frame for waypoints, Rover's mode numbers, and the magic value that forces a disarm.
FIX_NAMES = {0: "no-gps", 1: "no-fix", 2: "2d", 3: "3d", 4: "dgps", 5: "rtk-float", 6: "rtk-fixed"}
CMD_WAYPOINT, CMD_CHANGE_SPEED, CMD_ARM = 16, 178, 400
FRAME_REL_ALT = 3                     # MAV_FRAME_GLOBAL_RELATIVE_ALT (the INT variant takes the same id)
MODE_HOLD, MODE_AUTO = 4, 10          # ArduPilot Rover custom modes
FORCE_DISARM = 21196


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


class Vehicle:
    """A MAVLink ground vehicle as a bridge adapter: telemetry, missions, HOLD/AUTO, arming."""

    def __init__(self, url: str, frame: Frame, min_fix: int = 6, timeout: float = 30.0):
        """Connect and wait up to `timeout` seconds for the vehicle's heartbeat.

        `url` is a pymavlink connection string (for example `udpin:0.0.0.0:14550` or `/dev/ttyUSB0`). `frame`
        places the site grid on the Earth. `min_fix` is the lowest GNSS fix type good enough to work by (6 is RTK
        fixed, centimetre level).
        """
        try:
            from pymavlink import mavutil
        except ImportError as e:
            raise ValueError("MAVLink vehicles need pymavlink: pip install pymavlink") from e
        os.environ.setdefault("MAVLINK20", "1")
        self.frame, self.min_fix = frame, min_fix
        # The bridge identifies itself as a ground station (system 254), component 191 (MAV_COMP_ID_ONBOARD_COMPUTER).
        self.m: Any = mavutil.mavlink_connection(url, source_system=254, source_component=191)
        if self.m.wait_heartbeat(timeout=timeout) is None:
            self.m.close()
            raise ValueError(f"no MAVLink heartbeat on {url}")
        self.pose: tuple[float, float, float] | None = None
        self.fix, self.armed, self.mode, self.reached = 0, False, -1, -1
        self.n_wp, self.paused, self.stopped_hard = 0, True, False
        self.sent_at = 0.0
        self.failed: str | None = None          # why the current order cannot run, until the next order
        self.last_hb = time.time()
        for msg_id, hz in ((33, 10), (24, 5)):                 # GLOBAL_POSITION_INT, GPS_RAW_INT
            self.command(511, msg_id, 1e6 / hz)                # MAV_CMD_SET_MESSAGE_INTERVAL

    # -- adapter -------------------------------------------------------------------------
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
        """`vehicleLinkLost` when the vehicle's own heartbeat has not been heard for 3 s, else None."""
        return None if time.time() - self.last_hb < HEARTBEAT_TIMEOUT_S else "vehicleLinkLost"

    @property
    def order_error(self) -> str | None:
        """Why the current order cannot run (the mission was refused), or None."""
        return self.failed

    def poll(self) -> None:
        """Read every MAVLink message that has arrived: heartbeat (armed, mode), position, fix type, waypoint reached.
        """
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
            elif t == "GPS_RAW_INT":
                self.fix = msg.fix_type
            elif t == "MISSION_ITEM_REACHED":
                self.reached = msg.seq

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

    # -- MAVLink -------------------------------------------------------------------------
    def command(self, cmd: int, *p: float) -> None:
        """Send a COMMAND_LONG with up to seven parameters (the rest are zero)."""
        q = [*p, *[0.0] * (7 - len(p))]
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *q)

    def upload(self, items: list[tuple], timeout: float = 10.0) -> None:
        """Upload a mission with the MAVLink mission protocol: send the count, then answer each request for an item
        until the vehicle acknowledges.

        Raises ValueError if the vehicle rejects the mission or does not finish within `timeout` seconds.
        """
        m = self.m
        m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
        end = time.time() + timeout
        while time.time() < end:
            msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True,
                               timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type != 0:
                    raise ValueError(f"vehicle rejected the mission (MAV_MISSION_RESULT {msg.type})")
                return
            s, fr, cmd, p1, p2, p3, p4, x, y, z = items[msg.seq]
            m.mav.mission_item_int_send(m.target_system, m.target_component, s, fr, cmd, 0, 1,
                                        p1, p2, p3, p4, x, y, z, 0)
        raise ValueError("mission upload timed out")

    def set_mode(self, mode: int) -> None:
        """Ask for an ArduPilot custom mode (the 1 says the number is a custom mode)."""
        self.m.mav.set_mode_send(self.m.target_system, 1, mode)

    def arm(self, on: bool, force: bool = False) -> None:
        """Arm or disarm the motors. `force` disarms even while moving."""
        self.command(CMD_ARM, 1 if on else 0, FORCE_DISARM if force else 0)


def run_mavlink(model: Model, name: str, broker: str, url: str, frame: Frame, min_fix: int = 6,
                sim_world: bool = False, max_wall_s: float | None = None):
    """Bridge one MAVLink vehicle as Loam machine `name` (see `bridge.run_bridge`)."""
    return run_bridge(model, name, broker, Vehicle(url, frame, min_fix), sim_world, max_wall_s)
