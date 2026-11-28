"""The wire formats of the AgOpenGPS family, so that Loam can work beside it and drive its boards.

AgOpenGPS steers a tractor. Around it is a family of open boards and programs that share a few simple formats.
This module reads and writes those formats and does nothing else: no sockets, no ports, no timing. Each layout
was read from the project's own source and is checked in the tests against bytes built by hand.

**The frame.** Every message on the network is `0x80 0x81 source pgn length data... crc`, where the crc is the
low byte of the sum of everything from `source` to the last data byte. AgIO, the family's hub, sends to the
boards on UDP port 8888 (broadcast on the boards' subnet) and the boards answer on port 9999.

**What AgOpenGPS says while it works** (source 0x7F, read from AgOpenGPS `docs/pgn-protocol.md` and its steer
firmware, November 2026): the guidance context (0xF4: which pass the tractor is on and how far apart the passes
are), the steer data (0xFE: speed, and whether autosteer is engaged) and the steer board's reply (0xFD: its
switches). Loam only listens to these. A steer or machine board obeys whoever writes to it, and while
AgOpenGPS is running that is AgOpenGPS.

**Position.** GPS boards of the family send `$PANDA` (one antenna and an inertial unit) or `$PAOGI` (two
antennas) in place of the standard GGA sentence. Both carry the position, the height, a heading and a roll.
In PAOGI the heading and roll are degrees. In PANDA they are tenths of a degree, and 65535 means none.

**The OpenGrade valve boards** (read from OpenGrade3D `ArduinoCode`, commit 48cbd89 of November 2026). A valve
board lifts or lowers a blade from one number, the *cut value*: 100 is on grade, and each step is a millimetre
the edge is too high (above 100, lower it) or too low (below 100, lift it), from 1 to 199. The board turns that
into a PWM duty for a valve driver with eight settings (see `Settings`), and it acts only while its own work
switch is on. Two boards exist:

* a Nano on USB serial at 115200 baud. Data is the two bytes 0x7F 0xFA and six more (the cut value, a blade
  offset of 100 for none, four spare); settings are 0x7F 0xF8 and the eight setting bytes. It answers each
  data message with a line of eight comma-separated numbers. Silent for about 0.3 s, it stops the valve.
* a Teensy on the network, which also carries the blade's own GPS. It takes source 0x61 messages on port
  8888: 0xBA (a target altitude in millimetres, the cut value, the blade offset) and 0xB8 (the settings). It
  sends source 0x6A messages to port 9999: 0xE1 (its state) and 0xD1 (its position, heading and roll).
"""
from __future__ import annotations

import struct
from typing import NamedTuple

from .survey import _nmea_ok

AGIO, OPENGRADE, BLADE_BOARD = 0x7F, 0x61, 0x6A      # who a frame is from: the hub, OpenGrade3D, its Teensy board
MODULE_PORT, HUB_PORT = 8888, 9999                   # boards listen on the first and answer on the second
ON_GRADE = 100                                       # the cut value of a blade that is where it should be
NO_TARGET_MM = 22_000_000                            # a target altitude at or above 20 000 000 mm means "none"
BAUD = 115200                                        # the Nano valve board's serial speed
KNOT_KMH = 1.852


def frame(source: int, pgn: int, data: bytes) -> bytes:
    """A family message: the two header bytes, who it is from, what it is, its data and its checksum."""
    body = bytes([source, pgn, len(data)]) + bytes(data)
    return b"\x80\x81" + body + bytes([sum(body) & 0xFF])


def unframe(msg: bytes, check: bool = True) -> tuple[int, int, bytes] | None:
    """(source, pgn, data) of one message, or None if it is not one: too short, the wrong header, a length that
    runs past the end, or (unless `check` is off) a checksum that does not match. Some boards send their fixed
    hello messages with a constant in place of the checksum, which is what `check=False` is for."""
    if len(msg) < 6 or msg[0] != 0x80 or msg[1] != 0x81 or len(msg) < msg[4] + 6:
        return None
    end = 5 + msg[4]
    if check and sum(msg[2:end]) & 0xFF != msg[end]:
        return None
    return msg[2], msg[3], bytes(msg[5:end])


# -- what AgOpenGPS says while it works ------------------------------------------------------------
def guidance(data: bytes) -> dict | None:
    """The guidance context (pgn 0xF4): the pass the tractor is on and the spacing of the passes.

    `track` counts passes from the guidance line: 0 on the line, positive to one side, negative to the other.
    `left` and `right` are the passes beside the tractor as it is heading now. `spacing` is the distance between
    passes in metres. `valid` is False when no guidance line is active, and the rest then means nothing.
    """
    if len(data) < 12:
        return None
    seq, flags, ref, track, left, right, swath = struct.unpack_from("<BBHhhhH", data)
    return {"seq": seq, "valid": bool(flags & 1), "same_way": bool(flags & 2), "curve": bool(flags & 4),
            "line": ref, "track": track, "left": left, "right": right, "spacing": swath / 1000.0}


def steer_data(data: bytes) -> dict | None:
    """The steer data AgOpenGPS sends its steer board about ten times a second (pgn 0xFE): the speed in km/h,
    whether autosteer is engaged, and the wheel angle asked for in degrees."""
    if len(data) < 5:
        return None
    speed, status, angle = struct.unpack_from("<HBh", data)
    return {"speed_kmh": speed * 0.1, "steering": bool(status & 1), "angle": angle * 0.01}


def steer_reply(data: bytes) -> dict | None:
    """The steer board's reply (pgn 0xFD): the wheel angle in degrees and its two switches."""
    if len(data) < 7:
        return None
    angle, _, _, switches = struct.unpack_from("<hhhB", data)
    return {"angle": angle * 0.01, "work_switch": bool(switches & 1), "steer_switch": bool(switches & 2)}


# -- position ---------------------------------------------------------------------------------------
class Fix(NamedTuple):
    """One position of an antenna. Angles are degrees; `heading` is clockwise from north; `roll` is positive
    with the right side down. `quality` is the GGA fix quality: 4 is RTK fixed, 5 RTK float."""
    lat: float
    lon: float
    alt: float                     # metres, as the receiver gives it
    quality: int
    heading: float | None = None
    roll: float | None = None
    speed_kmh: float | None = None
    age: float | None = None       # seconds since the last correction


def _degrees(text: str, hemisphere: str) -> float:
    """An NMEA angle, degrees and decimal minutes run together (ddmm.mmmm or dddmm.mmmm), as signed degrees."""
    point = text.index(".") if "." in text else len(text)
    value = float(text[:point - 2] or 0) + float(text[point - 2:]) / 60
    return -value if hemisphere in ("S", "W") else value


def _number(text: str) -> float | None:
    """A sentence field as a number, or None if it is empty or not one."""
    try:
        return float(text)
    except ValueError:
        return None


def position(sentence: str) -> Fix | None:
    """A position from one `$PAOGI`, `$PANDA` or GGA sentence; None for any other sentence, a damaged one, or
    one with no position in it. GGA carries no heading or roll."""
    sentence = sentence.strip()
    if not _nmea_ok(sentence):
        return None
    f = sentence[1:sentence.index("*")].split(",")
    family = f[0] in ("PAOGI", "PANDA")
    if not (family or f[0].endswith("GGA")) or len(f) < 10 or not f[2] or not f[4] or not f[9]:
        return None
    try:
        lat, lon, alt, quality = _degrees(f[2], f[3]), _degrees(f[4], f[5]), float(f[9]), int(f[6] or 0)
    except ValueError:
        return None
    if not family:
        return Fix(lat, lon, alt, quality, age=_number(f[13]) if len(f) > 13 else None)
    f += [""] * (14 - len(f))
    heading, roll, knots = _number(f[12]), _number(f[13]), _number(f[11])
    if f[0] == "PANDA":                      # tenths of a degree from the inertial unit; 65535 when it has none
        if heading is None or heading >= 65535:
            heading = roll = None
        else:
            heading, roll = heading / 10, None if roll is None else roll / 10
    return Fix(lat, lon, alt, quality, heading, roll, None if knots is None else knots * KNOT_KMH, _number(f[10]))


# -- the OpenGrade valve boards ---------------------------------------------------------------------
class Settings(NamedTuple):
    """The eight numbers a valve board is tuned with, as the bytes it is sent.

    The valve is driven at `min` duty as soon as the edge is more than `deadband` millimetres off, and at `gain`
    more for each further millimetre, up to `max`; up and down have their own three, because a cylinder is
    faster one way than the other. Duties are in 255ths of full. `gain` is in 255ths per millimetre on the Nano
    and in quarters of that on the Teensy (`gain_unit`). `damping` scales a braking term the firmware applies as
    the edge closes on its target; at 0 it applies none and the law is exactly the one above.
    """
    gain_up: int = 5
    gain_down: int = 5
    min_up: int = 50
    min_down: int = 50
    max_up: int = 255
    max_down: int = 255
    damping: int = 0
    deadband: int = 5

    def packed(self) -> bytes:
        """The settings in the order both boards take them."""
        return bytes(min(max(int(v), 0), 255) for v in self)


def cut_value(high_mm: float) -> int:
    """The cut value for an edge `high_mm` millimetres above where it should be (negative: below it)."""
    return int(min(max(round(ON_GRADE + high_mm), 1), 199))


def nano_data(cut: int) -> bytes:
    """The Nano valve board's data message for a cut value."""
    return bytes([0x7F, 0xFA, cut, ON_GRADE, 0, 0, 0, 0])


def nano_settings(s: Settings) -> bytes:
    """The Nano valve board's settings message. The board writes the settings to its permanent memory each time
    it gets one, so it is sent when they change and not every cycle."""
    return bytes([0x7F, 0xF8]) + s.packed()


def _state(flags: int, duty: int, cut: int) -> dict:
    """A valve board's state from its status byte: which way it is driving the valve, and whether its work
    switch is on (bit 2 clear) and it has not been taken over by hand (bit 3 set)."""
    return {"up": bool(flags & 1), "down": bool(flags & 2), "switch": not flags & 4, "auto": bool(flags & 8),
            "duty": duty, "cut": cut}


def nano_status(text: str) -> dict | None:
    """The state in a line the Nano valve board answers with: its status byte, the duty it is driving the valve
    at (0 to 255) and the cut value it last took, then five numbers Loam has no use for."""
    try:
        flags, duty, cut = (int(float(v)) for v in text.strip().split(",")[:3])
    except ValueError:
        return None
    return _state(flags, duty, cut) if text.count(",") >= 7 else None


def teensy_data(cut: int, target_mm: int | None = None) -> bytes:
    """The Teensy valve board's data message. With `target_mm`, the altitude in millimetres its own antenna should
    be at, the board works out the cut value itself from every position it measures, between one message and
    the next. Without it the board uses the cut value given."""
    return frame(OPENGRADE, 0xBA, struct.pack("<iBBBB", NO_TARGET_MM if target_mm is None else int(target_mm),
                                              cut, ON_GRADE, 0, 0))


def teensy_settings(s: Settings) -> bytes:
    """The Teensy valve board's settings message."""
    return frame(OPENGRADE, 0xB8, s.packed())


def teensy_status(data: bytes) -> dict | None:
    """The state in the Teensy valve board's 0xE1 message. Its duty is in 255ths, like the Nano's."""
    return _state(data[0], data[1], data[2]) if len(data) >= 3 else None


def teensy_position(data: bytes) -> Fix | None:
    """The position in the Teensy valve board's 0xD1 message, or None while it has none.

    The board sends latitude and longitude as the receiver's own digits, degrees and decimal minutes run
    together, times 10^9; an altitude in millimetres; and the heading and roll of its two antennas times 100.
    A value out of range is the board's way of saying it has none: 2 x 10^13 for the position, 400 degrees for
    the heading, 320 for the roll.
    """
    if len(data) < 42:
        return None
    lon, lat, alt, dual, single, roll, speed = struct.unpack_from("<qqiHHhh", data)
    age, quality = struct.unpack_from("<hB", data, 38)

    def degrees(v: int) -> float:
        """Degrees from degrees-and-minutes run together and scaled by 10^9."""
        whole = int(abs(v) // 100_000_000_000)
        return (whole + (abs(v) / 1e9 - whole * 100) / 60) * (1 if v >= 0 else -1)

    if abs(lon) > 18_100 * 10 ** 9 or abs(lat) > 9_100 * 10 ** 9 or alt >= 20_000_000:
        return None
    heading = dual / 100 if dual <= 36_100 else single / 100 if single <= 36_100 else None
    return Fix(degrees(lat), degrees(lon), alt / 1000, quality, heading, roll / 100 if abs(roll) <= 18_100 else None,
               speed / 100 if 0 <= speed <= 30_000 else None, age / 100)
