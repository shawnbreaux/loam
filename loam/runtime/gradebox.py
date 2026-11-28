"""`loam blade`: a grade box that works alone, beside AgOpenGPS or beside nothing.

AgOpenGPS steers the tractor; this holds the blade. It needs no controller, no broker and no survey. It reads
the position of an antenna on the blade, works out how far the cutting edge is above the surface the next pass
should leave, and sends that one number, in millimetres, to an OpenGrade valve board (see `loam.aog`). The
board's own firmware turns it into valve movement, and the board acts only while its work switch is on, so the
person on the machine keeps the last word.

**Where the edge is.** The antenna is on a mast above the middle of the edge. Its altitude less `height` is
the edge's elevation, where `height` is measured once with the edge standing on ground of known elevation
(`--zero`), which also takes up any difference between the receiver's height datum and the site's. With two
antennas the board reports the blade's roll, and the lean of the mast is taken out: a mast `mast` long leaning
by the roll puts the antenna `mast x sin(roll)` to one side of the edge's middle and `mast x (1 - cos(roll))`
lower than when it stands upright.

**Where it should be.** The next-pass surface is the one the rest of Loam uses: toward the design by no more
than the task's `max_cut`, never past it. The edge is a straight line as long as the machine's `blade`, level or
at the measured roll, and its middle is held just high enough that no point of it is below what is wanted under
that point. Where there is nothing to grade the edge is carried clear of the ground.

**Several passes with no survey.** A blade shows what it cut: wherever the edge has passed below the ground
as it was known, the ground is now there. So each pass lowers the known ground and the next pass may cut
`max_cut` deeper, down to the design. A pass is over for a piece of ground once the edge has left it by more
than its own length; until then that ground keeps the target it had. Fill is different. A blade cannot tell
what it spilled, so low ground stays as last known until a survey says otherwise, and the edge is held one
`lift` above it, or at the design if that is lower.

**Tilt.** A second valve board on the tilt cylinder is sent the same kind of number: how many millimetres the
right-hand end of the edge is above where the wanted cross slope would put it. The wanted cross slope is the
straight line that best fits the surface under the edge.

**What is unknown is unsafe.** Without an RTK-fixed position newer than half a second the blade is lifted for
a moment and then left alone: the ground is never cut on a guess. With `--with-agopengps` the edge is in the
ground only while AgOpenGPS says it is steering.

**Tuning by measurement.** `Calibration` moves the blade in the air through the board, measures what the valve
does, and works out the board's eight settings from that, in place of tuning them by hand. See its description.
"""
from __future__ import annotations

import contextlib
import json
import math
import os
import socket
import statistics
import time
from collections.abc import Callable
from typing import Any, NamedTuple

import numpy as np

from .. import aog
from ..aog import Fix, Settings
from ..geo import Frame
from ..units import FT
from .blade import Surface

FIX_FRESH_S = 0.5            # a position older than this is no position
LIFT_S = 1.5                 # how long the blade is lifted when the position is lost while it was grading
CARRY_FT = 0.5               # how far above the ground the edge is carried where there is nothing to grade
MAX_SLOPE = 0.35             # the steepest cross slope asked of the tilt (rise over run; about 19 degrees)
MOVE_M = 0.2                 # with one antenna, the heading is the direction of the last move this long
PERIOD_S = 0.1               # how often the boards are written to; the Nano stops its valve after 0.28 s of silence
AOG_FRESH_S = 2.0            # AgOpenGPS is taken to be steering only if it said so this recently


class Command(NamedTuple):
    """What to send the boards now, and why."""
    lift: int                    # the cut value for the lift board
    tilt: int | None             # and for the tilt board, if there is one
    state: str                   # "grade", "carry", "lift" (position lost), "hold"
    high: float | None           # how far the edge is above where it should be, site units


class GradeBox:
    """The reasoning of the grade box, free of any input or output: positions in, cut values out."""

    def __init__(self, surface: Surface, frame: Frame, height: float, mast: float = 0.0, reach: float = 0.0,
                 heading_offset: float = 0.0, roll_zero: float = 0.0, roll_sign: int = 1, tilt: bool = False,
                 allow_float: bool = False, lift_on_loss: bool = True):
        """`height`: the antenna's altitude less the edge's elevation with the blade level, site units.
        `mast`: the antenna's true height above the edge, for the lean correction (0 for none, and for a board
        that already corrects its position for roll). `reach`: how far the edge is ahead of the antenna.
        `heading_offset`: degrees added to a two-antenna heading to get the direction of travel. `roll_zero`:
        the roll reported with the blade level, degrees. `roll_sign`: -1 if the board's roll is positive with
        the left side down. `tilt`: there is a tilt board. `allow_float`: grade on an RTK float position too.
        """
        self.surface, self.frame, self.height, self.mast, self.reach = surface, frame, height, mast, reach
        self.heading_offset, self.roll_zero, self.roll_sign, self.tilt = heading_offset, roll_zero, roll_sign, tilt
        self.allow_float, self.lift_on_loss = allow_float, lift_on_loss
        self.u = surface.grid.u
        self.carry = CARRY_FT * FT / self.u
        self.heading: float | None = None             # direction of travel, radians from the site's +x axis
        self.last: tuple[float, float] | None = None  # where the heading was last taken from, with one antenna
        self.lost: float | None = None                # when the position was lost while grading
        self.grading = False

    def roll(self, fix: Fix) -> float:
        """The blade's roll in radians, positive with the right side down; 0 if the position carries none."""
        return 0.0 if fix.roll is None else math.radians(self.roll_sign * (fix.roll - self.roll_zero))

    def pose(self, fix: Fix) -> tuple[float, float, float, float] | None:
        """Where the middle of the cutting edge is: site x and y, its elevation, and how much it rises for each
        unit to its left. None until the direction of travel is known."""
        x, y = (float(v) for v in self.frame.wgs84_to_local(fix.lat, fix.lon))
        if fix.heading is not None:
            self.heading = self.frame.heading_of(fix.heading + self.heading_offset)
        elif self.last is None:
            self.last = (x, y)
        elif math.hypot(x - self.last[0], y - self.last[1]) * self.u >= MOVE_M:
            self.heading, self.last = math.atan2(y - self.last[1], x - self.last[0]), (x, y)
        if self.heading is None:
            return None
        roll, h = self.roll(fix), self.heading
        # A mast leaning with the right side down carries the antenna to the right of the edge's middle.
        x += self.reach * math.cos(h) - self.mast * math.sin(roll) * math.sin(h)
        y += self.reach * math.sin(h) + self.mast * math.sin(roll) * math.cos(h)
        z = fix.alt / self.u - self.height + self.mast * (1 - math.cos(roll))
        return x, y, z, math.tan(roll)

    def step(self, fix: Fix | None, age: float, now: float, allowed: bool = True) -> Command:
        """The command for this moment. `fix` is the latest position and `age` how many seconds ago it came;
        `allowed` is False while something outside says the blade must stay out of the ground."""
        level = None if not self.tilt else aog.ON_GRADE
        good = fix is not None and age <= FIX_FRESH_S and (fix.quality == 4 or (self.allow_float and fix.quality == 5))
        at = self.pose(fix) if good and fix is not None else None
        if at is None:
            # No position to work from. If the edge was in the ground, lift it for a moment; then leave it alone.
            if self.grading and self.lost is None:
                self.lost = now
            self.grading = False
            lifting = self.lift_on_loss and self.lost is not None and now - self.lost < LIFT_S
            return Command(aog.cut_value(-99) if lifting else aog.ON_GRADE, level, "lift" if lifting else "hold", None)
        self.lost = None
        x, y, z, slope = at
        s = self.surface
        s.cut_to(x, y, self.heading or 0.0, z, slope)
        s.refresh(x, y)
        found = s.under(x, y, self.heading or 0.0) if allowed else None
        if found is None:
            # Nothing to grade here: carry the edge level and clear of the ground, if the ground is known.
            self.grading = False
            i, j, ok = s.grid.cell_of(np.array([x]), np.array([y]))
            ground = float(s.ground[j[0], i[0]]) if ok[0] else math.nan
            if not math.isfinite(ground):
                return Command(aog.ON_GRADE, level, "hold", None)
            high = z - (ground + self.carry)
            return Command(self._mm(high), self._tilt(slope, 0.0), "carry", high)
        self.grading = True
        across, _, want = found
        high = z - float(np.max(want - across * slope))
        wanted = 0.0
        if self.tilt and len(across) > 2:
            wanted = float(np.clip(np.polyfit(across, want, 1)[0], -MAX_SLOPE, MAX_SLOPE))
        return Command(self._mm(high), self._tilt(slope, wanted), "grade", high)

    def _mm(self, high: float) -> int:
        """The cut value for an edge `high` site units above where it should be."""
        return aog.cut_value(high * self.u * 1000)

    def _tilt(self, slope: float, wanted: float) -> int | None:
        """The cut value for the tilt board: how far the right-hand end of the edge is above where the wanted
        cross slope would put it. The end is half the edge's length to the right of the middle, so it is
        higher than wanted when the edge rises less to the left than it should."""
        if not self.tilt:
            return None
        return self._mm(self.surface.width / 2 * (wanted - slope))


# -- tuning by measurement -------------------------------------------------------------------------
LEVELS = tuple(range(30, 256, 15))       # the valve duties tried, in 255ths, from gentle to full
SETTLE_S, PULSE_S, REST_S, SKIP_S = 2.0, 1.0, 0.5, 0.3
FAST_MM_S = 150.0                        # once both directions move this fast, no stronger duty is tried
MIN_TAU_S = 0.3                          # the quickest response asked of the loop


class Calibration:
    """Measures a valve through its board and works out the board's settings.

    With the blade in the air and the board's work switch on, the valve is driven at one fixed duty at a time,
    up for a second and down for a second, at stronger and stronger duties, while the antenna shows what the
    blade did. A board drives its valve at exactly its `min` setting when its gain is zero, so a duty is chosen
    by sending settings with the gain at zero and `min` at that duty. From the speeds measured:

    * the *onset* in each direction is the duty at which the blade starts to move (where the straight line
      through the speeds that did move crosses zero), and becomes the board's `min`;
    * the *speed per duty* is that line's slope;
    * the *delay* is how long after a command the blade is first seen to move, less the time it took to
      cover the distance at which movement can be told from noise;
    * the *noise* is the scatter of the height while the blade is still.

    The settings follow. The deadband is two and a half times the noise, at least 3 mm, so that noise alone does
    not work the valve. The gain is chosen for the response time: the board commands a speed in proportion to
    the error, which makes the error decay with time constant 1 / (gain x speed per duty). That time constant is
    set to twice the delay (and no less than 0.3 s), which leaves a phase margin of about 60 degrees for a
    loop that is an integrator with a delay. The damping term of the firmware is switched off, so the board
    follows exactly the law this is designed for. `max` is the strongest duty tried. A valve that is either
    shut or open is recognised and set up differently (see `settings`).

    Feed it the measured value every cycle with `step`; it returns the cut value to send and, when they change,
    the settings to send first. It is finished when `done` is true; `settings()` and `report()` give the result.
    The value is in millimetres and must fall when the board lowers: the edge's height for the lift, the height
    of the right-hand end over the middle for the tilt.
    """

    def __init__(self, gain_unit: int = 1, levels: tuple[int, ...] = LEVELS, travel: float = 200.0):
        """`gain_unit`: how many steps of the board's gain setting make one 255th of duty per millimetre (1 for
        the Nano, 4 for the Teensy). `travel`: how far, in millimetres, the blade may wander from where it
        started before it is given one pulse back; at three times that the calibration is abandoned."""
        self.gain_unit, self.levels, self.travel = gain_unit, levels, travel
        self.i = 0                                         # the level being tried
        self.phase = "settle"                              # "settle", "up", "down" or "rest"
        self.queue: list[str] = []                         # the phases still to come at this level
        self.returned = False                              # this level has had its pulse back
        self.t0: float | None = None                       # when the phase in hand began
        self.samples: list[tuple[float, float]] = []       # (time, value) of the phase in hand
        self.noise, self.start, self.value = 0.0, 0.0, 0.0
        self.speeds: dict[int, dict[int, float]] = {1: {}, -1: {}}    # by direction (1 up), duty -> mm/s
        self.delays: list[float] = []
        self.backwards = 0                                 # pulses that moved the blade the wrong way
        self.done = False

    def _duty(self, duty: int) -> Settings:
        """Settings that make the board drive the valve at exactly `duty` whenever it is off grade. The deadband
        is one millimetre and not zero: with none at all, the Nano's firmware lifts at rest."""
        return Settings(0, 0, duty, duty, duty, duty, 0, 1)

    def step(self, value: float | None, now: float) -> tuple[int, Settings | None]:
        """Take one measurement (None if there is none this cycle) and say what to send: the cut value, and
        first the settings if they are to change."""
        if self.done:
            return aog.ON_GRADE, None
        if self.t0 is None:
            self.t0 = now
            return aog.ON_GRADE, self._duty(self.levels[0])
        if value is not None:
            self.samples.append((now, value))
            self.value = value
        t = now - self.t0
        if self.phase == "settle":
            if t < SETTLE_S:
                return aog.ON_GRADE, None
            vals = [v for _, v in self.samples]
            if len(vals) < 5:
                raise ValueError("no height came from the blade while it stood still: check the receiver")
            self.noise, self.start = statistics.pstdev(vals), statistics.fmean(vals)
            self.queue = ["up", "rest", "down", "rest"]
        elif self.phase == "rest":
            if t < REST_S:
                return aog.ON_GRADE, None
        else:
            if t < PULSE_S:
                return aog.cut_value(-10 if self.phase == "up" else 10), None
            self._measure(1 if self.phase == "up" else -1)
        away = self.value - self.start
        if abs(away) > 3 * self.travel:
            raise ValueError(f"the blade has moved {abs(away):.0f} mm from where it started: begin with it near "
                             "the middle of its travel, well clear of the ground")
        new = None
        if not self.queue:
            duty = self.levels[self.i]
            fast = all(abs(self.speeds[d].get(duty, 0.0)) >= FAST_MM_S for d in (1, -1))
            if abs(away) > self.travel and not self.returned:
                self.queue, self.returned = ["up" if away < 0 else "down", "rest"], True
            elif self.i + 1 < len(self.levels) and not fast:
                self.i, self.returned, self.queue = self.i + 1, False, ["up", "rest", "down", "rest"]
                new = self._duty(self.levels[self.i])
            else:
                self.done = True
                return aog.ON_GRADE, self.settings()
        self.phase, self.t0, self.samples = self.queue.pop(0), now, []
        return aog.ON_GRADE, new

    def _measure(self, direction: int) -> None:
        """The speed of the pulse just ended (a straight line through its later samples), and, if the blade
        moved, how long it took to start."""
        t, v = (np.array(c, float) for c in zip(*self.samples, strict=True)) if self.samples else (np.zeros(0),) * 2
        late = t - t[0] >= SKIP_S if len(t) else np.zeros(0, bool)
        if late.sum() < 3:
            return
        speed = float(np.polyfit(t[late], v[late], 1)[0])
        duty = self.levels[self.i]
        # A repeat at the same duty (a pulse back toward the start) is averaged with the first.
        seen = self.speeds[direction].get(duty)
        self.speeds[direction][duty] = speed if seen is None else (seen + speed) / 2
        clear = 4 * self.noise + 2.0                       # a move this large is no longer noise
        self.backwards += direction * speed < -2 * clear / PULSE_S
        if self.backwards >= 2:
            raise ValueError("the blade moves the wrong way: the valve's two coils are swapped, at the valve or at "
                             "the driver")
        moved = np.flatnonzero(np.abs(v - v[0]) > clear)
        if moved.size and abs(speed) > clear / PULSE_S:
            self.delays.append(max(float(t[moved[0]] - t[0]) - clear / abs(speed), 0.0))

    def _fit(self, direction: int) -> dict:
        """One direction, from the pulses that moved: the onset duty, the top speed in mm/s, and the speed per
        255th of duty, which is None for a valve that is either shut or open (on/off)."""
        still = max(3.0, 6 * self.noise / (PULSE_S - SKIP_S))
        pts = sorted((d, direction * s) for d, s in self.speeds[direction].items())
        moving = [(d, s) for d, s in pts if s > still]
        way = "up" if direction > 0 else "down"
        if len(moving) < 2:
            raise ValueError(f"the blade did not move {way}: check that the board's work switch is on, that the "
                             "valve is plumbed and wired, and that the tractor is giving it oil")
        top = max(s for _, s in moving)
        slope, cut = np.polyfit([d for d, _ in moving], [s for _, s in moving], 1)
        # A proportional valve is faster at stronger duties. One that moves the blade as fast at the first duty
        # that moves it as at the last is an on/off valve, whatever is driving it.
        if moving[-1][1] < 1.3 * moving[0][1] or slope <= 0:
            one = statistics.fmean(s for _, s in moving)             # its one speed, as well as it was measured
            return {"onset": float(moving[0][0]), "mm_s": round(one, 1), "mm_s_per_duty": None}
        quiet = max((d for d, s in pts if s <= still and d < moving[0][0]), default=0)
        return {"onset": round(float(min(max(-cut / slope, quiet), moving[0][0])), 1), "mm_s": round(top, 1),
                "mm_s_per_duty": round(float(slope), 3)}

    def report(self) -> dict:
        """What was measured: the noise (mm), the delay (s), each direction's onset and speeds, the response time
        designed for, and whether the valve is on/off or too quick for the board's smallest step of gain."""
        up, down = self._fit(1), self._fit(-1)
        delay = statistics.median(self.delays) if self.delays else 0.0
        tau = max(2 * delay, MIN_TAU_S)
        on_off = up["mm_s_per_duty"] is None or down["mm_s_per_duty"] is None
        quickest = math.inf if on_off else min(
            self.gain_unit / (d["mm_s_per_duty"] * self._gain(d["mm_s_per_duty"], tau)) for d in (up, down))
        return {"noise_mm": round(self.noise, 2), "delay_s": round(delay, 3), "response_s": round(tau, 3),
                "up": up, "down": down, "strongest_duty": self.levels[self.i], "on_off": on_off,
                "too_quick": bool(quickest < 0.75 * tau)}

    def _gain(self, per_duty: float, tau: float) -> int:
        """The board's gain setting for a response time `tau`: a whole number of its steps, never less than one."""
        return max(1, math.floor(self.gain_unit / (per_duty * tau)))

    def settings(self) -> Settings:
        """The board settings that follow from what was measured (see the class description).

        An on/off valve has no speed to choose, so it gets no gain: it is opened fully whenever the edge is
        outside the deadband. The blade then coasts for the length of the delay after the valve is told to shut,
        so the deadband is that distance plus the noise allowance, and the blade stops short of the line in place
        of crossing it and hunting.
        """
        r = self.report()
        top = r["strongest_duty"]
        quiet = 2.5 * r["noise_mm"]
        if r["on_off"]:
            coast = max(r["up"]["mm_s"], r["down"]["mm_s"]) * r["delay_s"]
            return Settings(0, 0, top, top, top, top, 0, min(max(math.ceil(coast + quiet), 3), 99))
        gain = [self._gain(r[way]["mm_s_per_duty"], r["response_s"]) for way in ("up", "down")]
        return Settings(gain[0], gain[1], math.ceil(r["up"]["onset"]), math.ceil(r["down"]["onset"]), top, top, 0,
                        min(max(math.ceil(quiet), 3), 20))


# -- inputs and outputs ----------------------------------------------------------------------------
class Link:
    """A serial port, a TCP connection or a UDP socket, read without waiting.

    `serial:PORT[:BAUD]` (needs pyserial), `tcp://HOST:PORT`, or `udp://HOST[:PORT[:LISTEN]]`: datagrams are
    sent to HOST:PORT and received on the LISTEN port from anyone, because the family's boards answer to the
    whole subnet. `udp://:LISTEN` only listens. A link opened to `hear` only (an antenna's sentences) reads
    `udp://HOST:LISTEN` as: listen on that port and keep only what HOST sends. `senders` are the addresses that
    have sent sentences.
    """

    def __init__(self, spec: str, port: int = aog.MODULE_PORT, listen: int = aog.HUB_PORT, hear: bool = False):
        """Open the link `spec` names. `port` and `listen` are the UDP ports to send to and to hear on when the
        spec leaves them out: the AgOpenGPS family's 8888 and 9999."""
        self.spec = spec
        self.only: str | None = None                  # the one sender a UDP listener keeps, if it was named
        self.senders: set[str] = set()
        self.serial: Any = None
        self.sock: socket.socket | None = None
        self.to: tuple[str, int] | None = None
        if spec.startswith("serial:"):
            try:
                import serial
            except ImportError as e:
                raise ValueError("a serial port needs pyserial: pip install pyserial") from e
            head, _, tail = spec[7:].rpartition(":")
            name, baud = (head, int(tail)) if head and tail.isdigit() else (spec[7:], aog.BAUD)
            try:
                self.serial = serial.Serial(name, baud, timeout=0, write_timeout=0.2)
            except serial.SerialException as e:
                raise ValueError(f"cannot open {name}: {e}") from e
        elif spec.startswith("tcp://"):
            host, _, p = spec[6:].rpartition(":")
            self.sock = socket.create_connection((host or "localhost", int(p)), timeout=10)
            self.sock.setblocking(False)
        elif spec.startswith("udp://"):
            parts = spec[6:].split(":")
            if len(parts) == 2 and not parts[0]:
                listen = int(parts[1])                                # udp://:LISTEN
            elif hear:
                if len(parts) != 2:
                    raise ValueError(f"{spec}: to listen, udp://:PORT or udp://SENDER:PORT")
                self.only, listen = socket.gethostbyname(parts[0]), int(parts[1])
            else:
                self.to = (parts[0], int(parts[1]) if len(parts) > 1 else port)
                listen = int(parts[2]) if len(parts) > 2 else listen
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.sock.bind(("", listen))
                self.sock.setblocking(False)
            except OSError as e:
                self.sock.close()                                     # a socket that cannot be used is not left open
                raise ValueError(f"cannot listen on UDP port {listen}: {e}") from e
        else:
            raise ValueError(f"{spec}: a link is serial:PORT, tcp://HOST:PORT or udp://HOST[:PORT]")
        self.datagrams = spec.startswith("udp://")
        self.buf = b""

    def read(self) -> list[bytes]:
        """Everything that has arrived: whole datagrams from a UDP socket, one chunk from a stream."""
        out: list[bytes] = []
        try:
            if self.serial is not None:
                chunk = self.serial.read(4096)
                return [chunk] if chunk else []
            while self.sock is not None and self.datagrams:
                chunk, sender = self.sock.recvfrom(4096)
                if chunk.startswith(b"$"):
                    self.senders.add(sender[0])
                if self.only is None or sender[0] == self.only:
                    out.append(chunk)
            while self.sock is not None and len(out) <= 64:           # a stream: enough for one turn
                chunk = self.sock.recv(4096)
                if not chunk:                                         # closed by the far end
                    break
                out.append(chunk)
        except (BlockingIOError, InterruptedError):
            pass
        return out

    def lines(self) -> list[str]:
        """The whole text lines that have arrived (a line cut short waits for its end)."""
        self.buf += b"".join(self.read())
        *whole, self.buf = self.buf.split(b"\n")
        self.buf = self.buf[-4096:]
        return [w.decode("ascii", "replace").strip() for w in whole]

    def write(self, data: bytes) -> None:
        """Send bytes; a UDP link that only listens sends nothing."""
        if self.serial is not None:
            self.serial.write(data)
        elif self.to is not None and self.sock is not None:
            self.sock.sendto(data, self.to)
        elif self.sock is not None and not self.datagrams:
            self.sock.sendall(data)

    def close(self) -> None:
        """Close the port or socket. A serial port is first given time to send what it was handed, so that the
        last message (the valve at rest, or the settings a calibration ends with) is not cut off."""
        if self.serial is not None:
            with contextlib.suppress(OSError):
                self.serial.flush()
        for end in (self.serial, self.sock):
            if end is not None:
                end.close()


class Board:
    """An OpenGrade valve board on a link: a Nano on a serial port, or a Teensy on the network."""

    def __init__(self, spec: str):
        """Open the link to the board. A `udp://` link means the Teensy board, any other the Nano."""
        self.link = Link(spec)
        self.teensy = self.link.datagrams
        self.gain_unit = 4 if self.teensy else 1
        self.state: dict | None = None           # what the board last said of itself
        self.heard = -math.inf                   # and when
        self.fix: Fix | None = None              # the Teensy's own position
        self.fix_at = -math.inf

    def poll(self, now: float) -> None:
        """Take in whatever the board has sent: its state, and from a Teensy its position."""
        if not self.teensy:
            for text in self.link.lines():
                state = aog.nano_status(text)
                if state is not None:
                    self.state, self.heard = state, now
            return
        for msg in self.link.read():
            got = aog.unframe(msg)
            if got is None or got[0] != aog.BLADE_BOARD:
                continue
            if got[1] == 0xE1 and (state := aog.teensy_status(got[2])) is not None:
                self.state, self.heard = state, now
            elif got[1] == 0xD1 and (fix := aog.teensy_position(got[2])) is not None:
                self.fix, self.fix_at = fix, now

    def send(self, cut: int) -> None:
        """Send a cut value."""
        self.link.write(aog.teensy_data(cut) if self.teensy else aog.nano_data(cut))

    def tune(self, s: Settings) -> None:
        """Send settings. The board keeps them in its permanent memory."""
        self.link.write(aog.teensy_settings(s) if self.teensy else aog.nano_settings(s))

    def close(self) -> None:
        """Leave the valve at rest and close the link."""
        with contextlib.suppress(OSError):
            self.send(aog.ON_GRADE)
        self.link.close()


def read_box(path: str | None) -> dict:
    """The grade box's file: what `--zero` and `--calibrate` measured. Empty if there is none yet."""
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_box(path: str, box: dict) -> None:
    """Write the grade box's file, whole, so that a crash cannot leave half of one."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(box, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def run_blade(surface: Surface, frame: Frame, valve: str, gnss: str, box_path: str | None = None,
              tilt_valve: str | None = None, zero: float | None = None, calibrate: str | None = None,
              mast: float = 0.0, reach: float = 0.0, heading_offset: float = 0.0, roll_invert: bool = False,
              allow_float: bool = False, lift_on_loss: bool = True, agopengps: int | None = None,
              max_wall_s: float | None = None, log: Callable[[str], None] = print,
              levels: tuple[int, ...] = LEVELS) -> dict:
    """Run the grade box until interrupted (or for `max_wall_s`). Returns a summary.

    `valve` and `tilt_valve` are board links, `gnss` a link that carries position sentences or the word
    "board" for a Teensy valve board's own antenna. `zero` (an elevation) or `calibrate` ("lift" or "tilt")
    make this a one-off measurement that is written to the box file and ends. `agopengps` is the UDP port to
    hear AgOpenGPS on, to grade only while it steers. `levels` are the valve duties a calibration tries.
    """
    box = read_box(box_path)
    if (zero is not None or calibrate) and not box_path:
        raise ValueError("--zero and --calibrate write what they measure to the box file: give --box FILE")
    if calibrate == "tilt" and not tilt_valve:
        raise ValueError("--calibrate tilt needs the tilt board (--tilt-valve)")
    if zero is None and not calibrate and "height" not in box:
        raise ValueError("the grade box has not been zeroed: stand the cutting edge on ground of known elevation "
                         "and run with --zero ELEVATION and --box FILE")
    if gnss == "board" and not valve.startswith("udp://"):
        raise ValueError("--gnss board is for a Teensy valve board (udp://...), which carries the antenna")
    if tilt_valve and valve.startswith("udp://") and tilt_valve.startswith("udp://"):
        # Every Teensy valve board has the same address and answers on the same port; two cannot be told apart.
        raise ValueError("two network valve boards cannot share a network: put the tilt on a Nano board (serial:PORT)")
    ends: list[Board | Link] = []
    try:
        lift = Board(valve)
        ends.append(lift)
        tilt = Board(tilt_valve) if tilt_valve else None
        ends += [tilt] if tilt else []
        feed = None if gnss == "board" else Link(gnss, hear=True)
        ends += [feed] if feed else []
        hub = Link(f"udp://:{agopengps}") if agopengps else None
        ends += [hub] if hub else []
        gb = GradeBox(surface, frame, box.get("height", 0.0), mast, reach, heading_offset, box.get("roll_zero", 0.0),
                      -1 if roll_invert else 1, tilt is not None, allow_float, lift_on_loss)
        tuned = tilt if calibrate == "tilt" else lift            # the board a calibration works on
        cal = Calibration(tuned.gain_unit, levels) if calibrate and tuned else None
        for board, key in ((lift, "lift"), (tilt, "tilt")):
            if board is not None and cal is None and key in box:
                board.tune(Settings(*box[key]["settings"]))
        fix: Fix | None = None
        fix_at = last_sent = steering_at = -math.inf
        track: dict | None = None
        zeros: list[tuple[float, float]] = []                    # (height, roll) of each position while zeroing
        t_start = last_say = time.monotonic()
        sent = grading = 0
        cmd = Command(aog.ON_GRADE, None, "hold", None)
        while max_wall_s is None or time.monotonic() - t_start < max_wall_s:
            now = time.monotonic()
            was = fix_at
            for board in (lift, tilt):
                if board is not None:
                    board.poll(now)
            if feed is not None:
                for text in feed.lines():
                    got = aog.position(text)
                    if got is not None:
                        fix, fix_at = got, now
                if feed.only is None and len(feed.senders) > 1:
                    # The tractor's receiver and the blade's, on one port: which is which cannot be guessed.
                    raise ValueError(f"positions are arriving from {' and '.join(sorted(feed.senders))}: say which "
                                     f"is the blade's antenna (--gnss udp://ADDRESS:PORT)")
            elif lift.fix is not None:
                fix, fix_at = lift.fix, lift.fix_at
            if hub is not None:
                for msg in hub.read():
                    said = aog.unframe(msg)
                    if said is None or said[0] != aog.AGIO:
                        continue
                    if said[1] == 0xFE and (aog.steer_data(said[2]) or {}).get("steering"):
                        steering_at = now
                    elif said[1] == 0xF4:
                        track = aog.guidance(said[2])
            fresh = fix_at != was                                # a new position came this time round
            if not fresh and now - last_sent < PERIOD_S:
                time.sleep(0.01)                                 # nothing new, and the boards have heard lately
                continue
            last_sent = now
            age = now - fix_at
            rtk = fix is not None and age <= FIX_FRESH_S and fix.quality == 4
            if zero is not None:
                # Zeroing: thirty good positions with the edge standing on ground of the elevation given.
                lift.send(aog.ON_GRADE)
                if rtk and fresh and fix is not None:
                    zeros.append((fix.alt / gb.u - zero, fix.roll or 0.0))
                if len(zeros) >= 30:
                    box["height"] = round(statistics.fmean(z for z, _ in zeros), 4)
                    box["roll_zero"] = round(statistics.fmean(r for _, r in zeros), 3)
                    box["zeroed"] = time.strftime("%Y-%m-%d %H:%M")
                    write_box(box_path or "", box)
                    log(f"zeroed: the antenna is {box['height']:.3f} {surface.grid.unit} above the edge; "
                        f"level roll {box['roll_zero']:.2f} degrees")
                    return {"height": box["height"], "roll_zero": box["roll_zero"]}
                if now - t_start > 30:
                    raise ValueError("zeroing needs a steady RTK-fixed position from the blade's antenna")
            elif cal is not None and tuned is not None and calibrate:
                value = None
                if rtk and fresh and fix is not None:
                    # Millimetres that fall when the board lowers: the antenna's height, or for the tilt the
                    # height of the right-hand end of the edge over its middle.
                    value = fix.alt * 1000 if calibrate == "lift" else (
                        -surface.width / 2 * math.tan(gb.roll(fix)) * gb.u * 1000)
                if not rtk and now - t_start > 5:
                    raise ValueError("calibration needs a steady RTK-fixed position from the blade's antenna")
                cut, new = cal.step(value, now)
                if new is not None:
                    tuned.tune(new)
                tuned.send(cut)
                if cal.done:
                    box[calibrate] = {"settings": list(cal.settings()), "measured": cal.report(),
                                      "calibrated": time.strftime("%Y-%m-%d %H:%M")}
                    write_box(box_path or "", box)
                    log(f"calibrated the {calibrate} valve: {json.dumps(box[calibrate]['measured'])}")
                    if box[calibrate]["measured"]["too_quick"] or box[calibrate]["settings"][7] > 20:
                        log("the valve moves the blade faster than this board can place it finely: fit a flow "
                            "restrictor, or expect the blade to hunt or to stop well short of grade")
                    return box[calibrate]
            else:
                cmd = gb.step(fix, age, now, allowed=hub is None or now - steering_at <= AOG_FRESH_S)
                lift.send(cmd.lift)
                if tilt is not None and cmd.tilt is not None:
                    tilt.send(cmd.tilt)
                sent += 1
                grading += cmd.state == "grade"
            if now - last_say >= 1.0:
                last_say = now
                st = lift.state or {}
                off = "" if cmd.high is None else f" {cmd.high * gb.u * 1000:+.0f} mm"
                log(f"{cmd.state}{off}  valve {'up' if st.get('up') else 'down' if st.get('down') else 'still'} "
                    f"{st.get('duty', '-')}  switch {'on' if st.get('switch') else 'OFF' if st else 'no board'}"
                    + (f"  pass {track['track']}" if track and track["valid"] else ""))
        return {"sent": sent, "grading": grading, "state": cmd.state, "board": lift.state}
    finally:
        for end in ends:
            end.close()
