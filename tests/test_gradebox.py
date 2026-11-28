"""The AgOpenGPS family's formats (`loam.aog`) and the grade box that drives its valve boards (`loam blade`).

No valve board, cylinder or receiver is at hand, so the box is tested against `Rig`: a board that follows the
control law the OpenGrade firmware documents, a valve with a dead zone and a delay, and an antenna with noise."""
import contextlib
import json
import math
import os
import random
import socket
import struct
import threading
import time
from collections import deque

import numpy as np
import pytest
from test_blade import yard

from loam import aog
from loam.aog import Fix, Settings
from loam.cli import load, main
from loam.geo import Frame
from loam.grid import SiteGrid
from loam.landxml import write_grid_tin
from loam.runtime.blade import Surface
from loam.runtime.gradebox import LIFT_S, Calibration, Command, GradeBox, Link, read_box, run_blade

FT = 0.3048
FRAME = Frame(FT, 0.0, 34.70, -85.70)       # the yard's (0, 0) on the earth; its +y axis is north
MAST = 6.0                                  # ft: the antenna above the cutting edge
DATUM = 200.0                               # m: the receiver's heights above the site's
HEIGHT = MAST + DATUM / FT                  # what zeroing should find: altitude less elevation, site units
EAST = 90.0                                 # the compass bearing of the yard's +x axis


def free_port() -> int:
    """A UDP port that is free right now. (UDP and TCP ports are separate: one found free for TCP may be taken
    for UDP.)"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# -- the formats -----------------------------------------------------------------------------------
def sentence(body: str) -> str:
    """An NMEA sentence with its checksum."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def dm(deg: float, width: int) -> str:
    """Degrees as NMEA writes them: degrees and decimal minutes run together."""
    d = int(abs(deg))
    return f"{d:0{width}d}{(abs(deg) - d) * 60:012.9f}"


def paogi(fix: Fix) -> str:
    """The `$PAOGI` sentence a two-antenna board of the family would send for a position."""
    return sentence(f"PAOGI,120000.00,{dm(fix.lat, 2)},{'N' if fix.lat >= 0 else 'S'},{dm(fix.lon, 3)},"
                    f"{'E' if fix.lon >= 0 else 'W'},{fix.quality},14,0.8,{fix.alt:.4f},1.0,"
                    f"{(fix.speed_kmh or 0) / 1.852:.3f},{fix.heading:.2f},{fix.roll or 0:.2f},0.00,0.00")


def test_the_family_frame():
    """A frame is two header bytes, the sender, the message number, the length, the data and the low byte of
    their sum from the sender on. AgIO's hello carries a constant where the sum belongs, as its source has it."""
    msg = aog.frame(0x7F, 0xFE, bytes([65, 0, 1, 0xBB, 0xFE, 0, 0, 0]))
    crc = (0x7F + 0xFE + 8 + 65 + 1 + 0xBB + 0xFE) & 255
    assert msg == bytes([0x80, 0x81, 0x7F, 0xFE, 8, 65, 0, 1, 0xBB, 0xFE, 0, 0, 0, crc])
    assert aog.unframe(msg) == (0x7F, 0xFE, bytes([65, 0, 1, 0xBB, 0xFE, 0, 0, 0]))
    hello = bytes([0x80, 0x81, 0x7F, 200, 3, 56, 0, 0, 0x47])
    assert aog.unframe(hello) is None and aog.unframe(hello, check=False) == (0x7F, 200, bytes([56, 0, 0]))
    for bad in (b"", msg[:-1], b"\x81\x80" + msg[2:], msg[:4] + bytes([40]) + msg[5:], msg[:-1] + b"\x00"):
        assert aog.unframe(bad) is None


def test_what_agopengps_says_while_it_works():
    """The guidance context, the steer data and the steer board's reply, each from bytes laid out as AgOpenGPS
    documents them."""
    # Pass 3 of line 2, heading against the line, on a curve, passes 3.048 m apart.
    g = aog.guidance(bytes([7, 0b101, 2, 0, 3, 0, 4, 0, 2, 0]) + struct.pack("<H", 3048))
    assert g == {"seq": 7, "valid": True, "same_way": False, "curve": True, "line": 2, "track": 3, "left": 4,
                 "right": 2, "spacing": 3.048}
    assert aog.guidance(struct.pack("<BBHhhhH", 0, 0, 0, -2, -1, -3, 0))["track"] == -2
    assert aog.guidance(b"\x00") is None
    # 6.5 km/h, autosteer on, wheels asked 3.25 degrees left (negative).
    assert aog.steer_data(bytes([65, 0, 1, 0xBB, 0xFE])) == pytest.approx(
        {"speed_kmh": 6.5, "steering": True, "angle": -3.25})
    assert aog.steer_data(bytes([65, 0, 0, 0, 0]))["steering"] is False and aog.steer_data(b"\x00") is None
    r = aog.steer_reply(struct.pack("<hhhBB", 125, 9999, 8888, 0b10, 40))
    assert r == pytest.approx({"angle": 1.25, "work_switch": False, "steer_switch": True})
    assert aog.steer_reply(b"") is None


def test_position_sentences():
    """`$PAOGI` gives heading and roll in degrees, `$PANDA` in tenths with 65535 for none, GGA gives neither.
    A damaged sentence, one of another kind and one with no position are no position."""
    where = "3442.1234560,N,08542.6543210,W"
    lat, lon = 34 + 42.123456 / 60, -(85 + 42.654321 / 60)
    f = aog.position(sentence(f"PAOGI,120000.00,{where},4,14,0.8,254.321,1.2,2.500,91.25,-3.50,0.1,0.0"))
    assert f == pytest.approx(Fix(lat, lon, 254.321, 4, 91.25, -3.5, 2.5 * 1.852, 1.2))
    p = aog.position(sentence("PANDA,120000.00,3442.1234560,S,08542.6543210,E,5,12,0.9,10.5,2.0,0.0,1805,-25,0,0"))
    assert (p.lat, p.lon, p.quality, p.heading, p.roll) == pytest.approx((-lat, -lon, 5, 180.5, -2.5))
    none = aog.position(sentence("PANDA,120000.00,3442.1234560,N,08542.6543210,W,4,12,0.9,10.5,2.0,0.0,65535,0,0,0"))
    assert none.heading is None and none.roll is None and none.quality == 4
    g = aog.position(sentence("GNGGA,120000.00,3442.1234560,N,08542.6543210,W,4,12,0.9,254.3,M,-30.1,M,1.0,0000"))
    assert (g.alt, g.quality, g.heading, g.roll, g.age) == (254.3, 4, None, None, 1.0)
    good = sentence("PAOGI,120000.00,3442.1234560,N,08542.6543210,W,4,14,0.8,254.321,1.2,2.5,91.25,-3.50,0.1,0.0")
    for bad in (good[:-2] + "00", good.replace("PAOGI", "GNVTG"), "garbage", "",
                sentence("PAOGI,120000.00,,,,,0,0,,,,,,,,"), sentence("PAOGI,1,x,N,y,W,4,1,1,z,1,1,1,1,1,1")):
        assert aog.position(bad) is None


def test_valve_board_messages():
    """What goes to the two valve boards and what comes back, byte for byte as their firmware reads and writes
    it. A cut value is 100 on grade and one more for each millimetre the edge is too high, from 1 to 199."""
    assert [aog.cut_value(v) for v in (0, 7.4, -12.6, 500, -500)] == [100, 107, 87, 199, 1]
    s = Settings(6, 4, 62, 78, 165, 165, 0, 5)
    assert aog.nano_data(107) == bytes([0x7F, 0xFA, 107, 100, 0, 0, 0, 0])
    assert aog.nano_settings(s) == bytes([0x7F, 0xF8, 6, 4, 62, 78, 165, 165, 0, 5])
    assert Settings(300, -4, 1, 1, 1, 1, 1, 1).packed()[:2] == bytes([255, 0])
    # The Nano answers: status, duty, cut value, then five more. Status bit 2 set means its work switch is off.
    assert aog.nano_status("137,57,107,100,512,0,100,0.00\r") == {
        "up": True, "down": False, "switch": True, "auto": True, "duty": 57, "cut": 107}
    assert aog.nano_status("134,0,100,100,512,0,100,0.00")["switch"] is False
    assert aog.nano_status("OG machine controler") is None and aog.nano_status("1,2,3") is None
    # The Teensy takes a target altitude (none: 22 000 000 mm), the cut value and a blade offset of 100.
    data = aog.unframe(aog.teensy_data(93))
    assert data == (0x61, 0xBA, struct.pack("<i", 22_000_000) + bytes([93, 100, 0, 0]))
    assert aog.unframe(aog.teensy_settings(s)) == (0x61, 0xB8, bytes([6, 4, 62, 78, 165, 165, 0, 5]))
    assert aog.teensy_status(bytes([0b1010, 80, 120, 100, 0, 0, 0, 0])) == {
        "up": False, "down": True, "switch": True, "auto": True, "duty": 80, "cut": 120}
    assert aog.teensy_status(b"\x00") is None


def teensy_fix(fix: Fix | None) -> bytes:
    """The Teensy board's position message for a position, built from its documented layout: longitude and
    latitude as degrees-and-minutes digits times 10^9, altitude in mm, heading and roll times 100."""
    if fix is None:
        return aog.frame(0x6A, 0xD1, struct.pack("<qqiHHhhHhhhhhBB", 2 * 10 ** 13, 2 * 10 ** 13, 21_000_000, 0, 0, 0,
                                                 0, 40000, 32000, 32000, 32000, 0, 0, 0, 0))
    digits = lambda v: round(math.copysign(int(abs(v)) * 100 + (abs(v) - int(abs(v))) * 60, v) * 1e9)  # noqa: E731
    return aog.frame(0x6A, 0xD1, struct.pack(
        "<qqiHHhhHhhhhhBB", digits(fix.lon), digits(fix.lat), round(fix.alt * 1000),
        40000 if fix.heading is None else round(fix.heading * 100), 9000,
        32000 if fix.roll is None else round(fix.roll * 100), round((fix.speed_kmh or 0) * 100),
        40000, 32000, 32000, 32000, 80, 100, fix.quality, 14))


def test_the_teensy_boards_own_position():
    """The Teensy valve board reports its antenna's position in its own packing, with out-of-range values where
    it has nothing to report."""
    got = aog.unframe(teensy_fix(Fix(34.7020576, -85.7109054, 254.321, 4, 91.25, -3.5, 4.63)))
    assert got[:2] == (0x6A, 0xD1) and len(got[2]) == 42
    f = aog.teensy_position(got[2])
    assert f == pytest.approx(Fix(34.7020576, -85.7109054, 254.321, 4, 91.25, -3.5, 4.63, 1.0), abs=1e-9)
    one = aog.teensy_position(aog.unframe(teensy_fix(Fix(-34.5, 85.25, 10.0, 5)))[2])
    assert (one.lat, one.lon, one.heading) == pytest.approx((-34.5, 85.25, 90.0)) and one.roll is None   # one antenna
    assert aog.teensy_position(aog.unframe(teensy_fix(None))[2]) is None and aog.teensy_position(b"\x00" * 10) is None


def test_loam_nmea_reads_the_familys_gps_boards():
    """`loam nmea` makes machine state from `$PANDA` and `$PAOGI` as it does from GGA, and uses the heading the
    sentence carries, which a machine standing still also has. The boards send to a UDP port of the whole subnet."""
    from conftest import wire_valid

    from loam.survey import nmea_states, udp_lines
    lat, lon = (float(v) for v in FRAME.local_to_wgs84(60.0, 50.0))
    still = [paogi(Fix(lat, lon, 254.3, 4, 0.0, 1.5))] * 2 + [paogi(Fix(lat, lon, 254.3, 5, 180.0, 0.0)),
                                                              "$GNVTG,junk*00"]
    out = list(nmea_states(still, FRAME, "yard", "tr1"))
    assert len(out) == 3 and all(wire_valid("state", m) is None for m in out)
    assert (out[0]["pose"]["x"], out[0]["pose"]["y"]) == pytest.approx((60.0, 50.0), abs=1e-3)
    assert out[0]["pose"]["heading"] == pytest.approx(math.pi / 2, abs=1e-4) and out[0]["safetyState"] == "ok"   # north
    assert out[2]["pose"]["heading"] == pytest.approx(-math.pi / 2, abs=1e-4) and out[2]["safetyState"] == "degraded"
    assert out[0]["gnss"] == {"fix": "rtk-fixed", "sats": None, "hdop": None, "age": 1.0}
    # A receiver may leave a field empty. What it left out is null in the message, since JSON has no NaN.
    bare = sentence(f"GNGGA,120000.00,{dm(lat, 2)},N,{dm(-lon, 3)},W,4,,,254.3,M,,M,,")
    (one,) = nmea_states([bare], FRAME, "yard", "tr1")
    wire_valid("state", one)
    assert one["gnss"] == {"fix": "rtk-fixed", "sats": 0, "hdop": None, "age": None}
    assert "NaN" not in json.dumps(one, allow_nan=False)
    port = free_port()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        lines = udp_lines(port, idle_s=0.5)
        got: list[str] = []

        def send() -> None:
            """Two datagrams as a board sends them, the second with two sentences in it."""
            time.sleep(0.1)
            for datagram in (still[0] + "\r\n", still[2] + "\r\n" + still[3] + "\r\n"):
                tx.sendto(datagram.encode(), ("127.0.0.1", port))

        t = threading.Thread(target=send)
        t.start()
        got = list(lines)                                       # ends after half a second of silence
        t.join()
    assert got == [still[0], still[2], still[3]]
    # Two boards on one port are not read as one machine: it is an error, unless the one to hear is named.
    udp = (socket.AF_INET, socket.SOCK_DGRAM)
    with socket.socket(*udp) as one, socket.socket(*udp) as two:
        try:
            two.bind(("127.0.0.2", 0))                          # a second address; Linux has all of 127/8
        except OSError:
            return
        for only, want in ((None, None), ("127.0.0.1", [still[0]] * 2)):
            heard = udp_lines(port, idle_s=0.5, only=only)

            def both() -> None:
                """A sentence from each of two addresses, twice."""
                time.sleep(0.1)
                for sock in (one, two, one, two):
                    sock.sendto((still[0] if sock is one else still[2]).encode(), ("127.0.0.1", port))

            t = threading.Thread(target=both)
            t.start()
            try:
                if want is None:
                    with pytest.raises(ValueError, match=r"from 127.0.0.1 and 127.0.0.2: say which"):
                        list(heard)
                else:
                    assert list(heard) == want
            finally:
                t.join()


# -- a blade to test against -----------------------------------------------------------------------
class Rig:
    """A blade on a hydraulic cylinder, its valve board and its antenna.

    The board follows the law the OpenGrade firmware documents: no drive inside the deadband, `min` duty at its
    edge and `gain` more per millimetre beyond, up to `max`, and nothing at all unless its work switch is on
    and it has heard a cut value in the last 0.28 s. The valve passes no oil below an onset duty and then moves
    the blade at a speed in proportion to the duty above it (or, for an on/off valve, at one speed), `delay`
    seconds after it is told. The antenna reports the blade's height `lag` seconds late with Gaussian noise.
    `z` is in site units and falls when the board lowers.
    """

    def __init__(self, z: float, onset=(62, 78), speed=(1.5, 2.2), delay=0.12, lag=0.05, noise=2.0, teensy=False,
                 switch=True, swapped=False, on_off=False, seed=1):
        """`z`: where the edge starts, feet. `onset`: the duties (up, down) at which the valve starts to pass oil.
        `speed`: mm/s gained per duty step beyond it. `delay`: seconds before the valve answers. `lag`, `noise`:
        how late and how rough (mm) the antenna's height is. `switch`: the work switch is on. `swapped`: the
        coils are wired the wrong way round. `on_off`: the valve has one speed, whatever the duty."""
        self.z, self.onset, self.speed, self.delay, self.lag, self.noise = z, onset, speed, delay, lag, noise
        self.teensy, self.switch, self.swapped, self.on_off = teensy, switch, swapped, on_off
        self.s, self.cut, self.heard, self.t = Settings(), aog.ON_GRADE, -math.inf, 0.0
        self.sent: deque = deque()            # (time, duty) on their way to the valve
        self.was: deque = deque([(0.0, z)])   # (time, height) on their way to the antenna
        self.duty, self.rng, self.tunes = 0.0, random.Random(seed), 0
        self.gaps: list[float] = []           # how long the board waited between each two cut values

    def take(self, data: bytes) -> None:
        """A message from the grade box: a cut value or settings, in the board's own format."""
        if self.teensy:
            got = aog.unframe(data)
            kind, body = (got[1], got[2]) if got and got[0] == 0x61 else (None, b"")
            cut, settings = (body[4], None) if kind == 0xBA else (None, body if kind == 0xB8 else None)
        else:
            cut = data[2] if data[:2] == b"\x7f\xfa" else None
            settings = data[2:10] if data[:2] == b"\x7f\xf8" else None
        if cut is not None:
            if math.isfinite(self.heard):
                self.gaps.append(self.t - self.heard)
            self.cut, self.heard = cut, self.t
        if settings is not None:
            self.s, self.tunes = Settings(*settings), self.tunes + 1

    def law(self) -> float:
        """The duty the board drives the valve at, in 255ths, positive to lift."""
        if not self.switch or self.t - self.heard > 0.28:
            return 0.0
        s, e, unit = self.s, self.cut - aog.ON_GRADE, 4 if self.teensy else 1
        beyond = abs(e) - s.deadband
        if e == 0 or beyond < 0 or (self.teensy and beyond == 0):
            return 0.0
        if e > 0:
            return -min(s.min_down + beyond * s.gain_down / unit, s.max_down)
        return min(s.min_up + beyond * s.gain_up / unit, s.max_up)

    def advance(self, dt: float) -> None:
        """Let `dt` seconds pass."""
        self.t += dt
        self.sent.append((self.t, self.law()))
        while self.sent and self.sent[0][0] <= self.t - self.delay:
            self.duty = self.sent.popleft()[1]
        way = 0 if self.duty > 0 else 1
        over = abs(self.duty) - self.onset[way]
        v = 0.0 if over < 0 else self.speed[way] * (60.0 if self.on_off else over)      # mm/s
        v *= (1 if self.duty > 0 else -1) * (-1 if self.swapped else 1)
        self.z += v * dt / 1000 / FT
        self.was.append((self.t, self.z))
        while len(self.was) > 1 and self.was[1][0] <= self.t - self.lag:
            self.was.popleft()

    def seen(self) -> float:
        """The height the antenna reports now, site units."""
        return self.was[0][1] + self.rng.gauss(0, self.noise) / 1000 / FT

    def fix(self, x: float, y: float, bearing: float | None = EAST, quality: int = 4, roll: float | None = None,
            mast: float = MAST) -> Fix:
        """The position the antenna reports with the middle of the edge at site (x, y)."""
        lat, lon = FRAME.local_to_wgs84(x, y)
        return Fix(float(lat), float(lon), (self.seen() + mast) * FT + DATUM, quality, bearing, roll)

    def status(self) -> bytes:
        """The board's own report of itself, in its format."""
        d = self.law()
        flags = (d > 0) | (d < 0) << 1 | (not self.switch) << 2 | 1 << 3 | (0 if self.teensy else 1 << 7)
        if self.teensy:
            return aog.frame(0x6A, 0xE1, bytes([flags, round(abs(d)), self.cut, 100, 128, 0, 0, 0]))
        return f"{flags},{round(abs(d))},{self.cut},100,512,0,100,0.00\r\n".encode()


def simulate(rig: Rig, seconds: float, each, dt: float = 0.01, period: float = 0.1) -> None:
    """Run a rig in simulated time, calling `each(t)` ten times a second: where the grade box has its turn."""
    for k in range(round(seconds / dt)):
        rig.advance(dt)
        if k % round(period / dt) == 0:
            each(rig.t)


def calibrated(rig: Rig, **kw) -> Calibration:
    """Calibrate a rig's valve in simulated time, as `loam blade --calibrate lift` does in real time."""
    cal = Calibration(4 if rig.teensy else 1, **kw)

    def turn(t: float) -> None:
        """One step of the calibration: what the antenna reads goes in, a cut value and any new settings come out."""
        if not cal.done:
            cut, new = cal.step(rig.seen() * FT * 1000, t)
            if new is not None:
                rig.take(aog.teensy_settings(new) if rig.teensy else aog.nano_settings(new))
            rig.take(aog.teensy_data(cut) if rig.teensy else aog.nano_data(cut))

    simulate(rig, 90, turn)
    assert cal.done
    return cal


# -- tuning by measurement -------------------------------------------------------------------------
@pytest.mark.parametrize("teensy", [False, True])
def test_calibration_measures_the_valve_and_sets_the_board(teensy):
    """Driving the valve at one duty after another, up and down, recovers where it starts to pass oil, how fast
    the blade moves per duty, the delay and the noise, each way. The settings that follow place the blade from
    80 mm off to within the deadband in a few seconds without crossing the line by more than the noise, on
    either board. The blade ends the calibration near where it began, and the board's permanent memory is
    written once per duty tried, not every cycle."""
    rig = Rig(100.0, teensy=teensy)
    cal = calibrated(rig)
    r = cal.report()
    assert r["up"]["onset"] == pytest.approx(62, abs=6) and r["down"]["onset"] == pytest.approx(78, abs=6)
    assert r["up"]["mm_s_per_duty"] == pytest.approx(1.5, rel=0.1)
    assert r["down"]["mm_s_per_duty"] == pytest.approx(2.2, rel=0.1)
    assert r["delay_s"] == pytest.approx(0.12 + 0.05 + 0.05, abs=0.07) and r["noise_mm"] == pytest.approx(2.0, rel=0.4)
    assert not r["on_off"] and not r["too_quick"] and r["strongest_duty"] <= 180       # stopped once it was fast
    assert abs(rig.z - 100.0) * FT * 1000 < 250 and rig.tunes <= len(cal.speeds[1]) + 3
    s = cal.settings()
    assert s == rig.s and s.damping == 0 and 3 <= s.deadband <= 8 and s.max_up == r["strongest_duty"]
    assert abs(s.min_up - 62) <= 7 and abs(s.min_down - 78) <= 7
    # The time constant asked for is twice the delay; a whole number of gain steps comes as near as it can.
    for gain, k in ((s.gain_up, 1.5), (s.gain_down, 2.2)):
        assert r["response_s"] * 0.9 <= cal.gain_unit / (k * gain) <= (4.0 if not teensy else 1.4) * r["response_s"]
    for start in (80.0, -80.0):
        target = rig.z - start / 1000 / FT                  # the edge starts `start` mm above its target
        errs: list[float] = []

        def turn(t: float, target=target, errs=errs) -> None:
            """Send the board how high the edge reads over the target, and note how high it truly is."""
            high = (rig.seen() - target) * FT * 1000
            rig.take(aog.teensy_data(aog.cut_value(high)) if teensy else aog.nano_data(aog.cut_value(high)))
            errs.append((rig.z - target) * FT * 1000)

        simulate(rig, 6, turn)
        assert abs(errs[-1]) <= s.deadband + 4 and np.abs(errs[-20:]).max() <= s.deadband + 6      # there, and still
        assert min(e * math.copysign(1, start) for e in errs) > -(s.deadband + 6)                   # no hunting across


def test_calibration_of_an_on_off_valve():
    """A valve that is either shut or open has no speed to choose. It is recognised by moving the blade no
    faster at strong duties than at weak ones, and gets no gain and a deadband wide enough for the blade to
    coast to a stop inside it. With that the blade stops short of the line and stays there."""
    rig = Rig(100.0, on_off=True, speed=(1.0, 1.0), onset=(100, 100))          # 60 mm/s either way, or nothing
    cal = calibrated(rig)
    r, s = cal.report(), cal.settings()
    assert r["on_off"] and r["up"]["mm_s_per_duty"] is None and r["up"]["mm_s"] == pytest.approx(60, rel=0.1)
    assert (s.gain_up, s.gain_down) == (0, 0) and s.min_up == s.max_up == r["strongest_duty"] == 255
    assert s.deadband == pytest.approx(60 * r["delay_s"] + 2.5 * r["noise_mm"], abs=2) and s.deadband > 12
    target, errs = rig.z - 0.3, []

    def turn(t: float) -> None:
        """Send the board how high the edge reads over the target, and note how high it truly is."""
        rig.take(aog.nano_data(aog.cut_value((rig.seen() - target) * FT * 1000)))
        errs.append((rig.z - target) * FT * 1000)

    simulate(rig, 6, turn)
    assert -6 <= errs[-1] <= s.deadband + 3 and max(errs[-30:]) - min(errs[-30:]) < 1e-6     # stopped, and still
    assert errs[-1] == min(errs)                                                             # it never came back


def test_calibration_says_what_is_wrong():
    """A board whose switch is off, a valve plumbed backwards, a receiver that says nothing and a blade that
    runs away are each named, not turned into settings."""
    with pytest.raises(ValueError, match=r"did not move up.*work switch"):
        calibrated(Rig(100.0, switch=False)).settings()
    backwards = Rig(100.0, swapped=True)
    with pytest.raises(ValueError, match=r"wrong way.*coils are swapped"):
        calibrated(backwards)
    assert backwards.t < 25 and abs(backwards.z - 100.0) * FT * 1000 < 150       # found out early, and gently
    silent = Calibration()
    silent.step(None, 0.0)
    with pytest.raises(ValueError, match="no height came"):
        silent.step(None, 3.0)
    with pytest.raises(ValueError, match="near the middle of its travel"):
        calibrated(Rig(100.0, onset=(40, 250), speed=(3.0, 0.1)))             # lifts, and never comes back down
    # A valve far quicker than the board's smallest gain step can use is flagged.
    quick = calibrated(Rig(100.0, onset=(40, 40), speed=(12.0, 12.0)), levels=(45, 50, 55))
    assert quick.report()["too_quick"] and quick.settings().gain_up == 1


# -- the grade box ---------------------------------------------------------------------------------
TUNED = Settings(1, 1, 62, 78, 165, 165, 0, 5)       # what calibration gives the rig's valve on a Nano
TEENSY_TUNED = Settings(4, 4, 62, 78, 165, 165, 0, 5)   # and on a Teensy, whose gain steps are a quarter the size


def box(tmp_path, **kw) -> tuple[GradeBox, Surface]:
    """A grade box for the yard: a pad from x 20 to 120 and y 20 to 80 with a foot to cut, a 10 ft edge."""
    _, g = yard(tmp_path)
    s = Surface(g, 10.0)
    return GradeBox(s, FRAME, HEIGHT, MAST, **kw), s


def drive(gb: GradeBox, rig: Rig, x0: float, x1: float, y: float = 50.0, speed: float = 3.0, **fix) -> list:
    """Drive the blade from x0 to x1 along a line of constant y at `speed` ft/s with the grade box holding it.
    Returns (x, the edge's true elevation, the command) for each turn of the box."""
    rig.take(aog.nano_settings(TUNED))
    out: list[tuple[float, float, Command]] = []
    way = 1 if x1 >= x0 else -1
    t0 = rig.t

    def turn(t: float) -> None:
        """The machine has driven on; the grade box reads the fix there and the board gets its command."""
        x = x0 + way * speed * (t - t0)
        cmd = gb.step(rig.fix(x, y, EAST if way > 0 else 270.0, **fix), 0.0, t)
        rig.take(aog.nano_data(cmd.lift))
        out.append((x, rig.z, cmd))

    simulate(rig, abs(x1 - x0) / speed, turn)
    return out


def test_the_blade_is_cut_down_to_design_pass_by_pass(tmp_path):
    """With no survey and no controller, the grade box takes the pad to design in passes. The first pass holds the
    edge half a foot (the task's `max_cut`) under the ground. A blade shows what it cut, so the second pass knows
    the ground is lower and holds the edge at design, and a third stays there: the edge is never taken below it.
    Off the pad the edge is carried half a foot clear of the ground."""
    gb, s = box(tmp_path)
    rig = Rig(101.5)
    first = drive(gb, rig, 5, 135)
    on = [(z, c) for x, z, c in first if 40 <= x <= 115]
    assert {c.state for x, _, c in first if x < 14 or x > 126} == {"carry"} and {c.state for _, c in on} == {"grade"}
    assert np.allclose([z for x, z, _ in first if x < 12 or x > 132], 101.5, atol=0.03)
    # The board stops driving within its deadband, 5 mm (0.016 ft), so the edge settles a little high.
    assert 100.495 <= np.mean([z for z, _ in on]) <= 100.525 and np.ptp([z for z, _ in on]) < 0.05
    i, j, _ = s.grid.cell_of(np.array([60.0, 60.0, 60.0]), np.array([50.0, 46.0, 58.0]))
    assert s.ground[j, i] == pytest.approx([100.5, 100.5, 101.0], abs=0.04)           # under the edge, and beside it
    second = drive(gb, rig, 135, 5)
    assert 100.0 <= np.mean([z for x, z, _ in second if 25 <= x <= 100]) <= 100.06     # half a foot more, less the 5 mm
    third = drive(gb, rig, 5, 135)
    low = min(z for run in (first, second, third) for _, z, _ in run)
    assert 100.0 <= np.mean([z for x, z, _ in third if 40 <= x <= 115]) <= 100.03 and low > 100 - 0.04
    assert float(np.nanmin(s.ground)) > 100 - 0.05
    # The pass in hand does not chase its own cut: ground under the edge keeps its target until the edge has left.
    fresh, _ = box(tmp_path)
    still = Rig(101.5)
    still.take(aog.nano_settings(TUNED))
    simulate(still, 8, lambda t: still.take(aog.nano_data(fresh.step(still.fix(60, 50), 0.0, t).lift)))
    assert 100.49 <= still.z <= 100.53 and fresh.surface.fresh.any()       # half a foot down, and no further
    fresh.step(still.fix(90, 50), 0.0, 9.0)                      # gone 30 ft: now that ground may be cut again
    i, j, _ = fresh.surface.grid.cell_of(np.array([60.0]), np.array([50.0]))
    assert fresh.surface.target[j[0], i[0]] == pytest.approx(100.016, abs=0.02) and not fresh.surface.fresh[j[0], i[0]]


def test_without_a_good_position_the_ground_is_not_cut(tmp_path):
    """An RTK float position, a stale one or none at all is no position. If the edge was in the ground it is
    lifted for a moment and then left alone; if it was not, it is left alone. With one antenna there is no
    direction until the machine has moved, and until then the blade is not moved either."""
    gb, _ = box(tmp_path)
    rig = Rig(101.5)
    assert {c.state for _, _, c in drive(gb, rig, 40, 70)} == {"grade"}
    lost = rig.t
    for fix, age in ((rig.fix(70, 50, quality=5), 0.0), (rig.fix(70, 50), 0.6), (None, 0.0)):
        gb.lost, gb.grading = None, True
        cmd = gb.step(fix, age, lost)
        assert (cmd.state, cmd.lift, cmd.high) == ("lift", 1, None)
        assert gb.step(fix, age, lost + LIFT_S - 0.1).state == "lift"
        assert gb.step(fix, age, lost + LIFT_S + 0.1) == Command(100, None, "hold", None)
    assert gb.step(rig.fix(70, 50), 0.0, lost + 5).state == "grade" and gb.lost is None      # and back to work
    floating, _ = box(tmp_path, allow_float=True, lift_on_loss=False)
    assert floating.step(rig.fix(70, 50, quality=5), 0.0, 0.0).state == "grade"
    floating.grading = True
    assert floating.step(None, 0.0, 1.0) == Command(100, None, "hold", None)                 # asked to hold, not lift
    single, _ = box(tmp_path)
    assert single.step(rig.fix(70, 50, bearing=None), 0.0, 0.0).state == "hold"
    assert single.step(rig.fix(70.5, 50, bearing=None), 0.0, 0.1).state == "hold"            # 0.15 m: not yet
    assert single.step(rig.fix(71, 50, bearing=None), 0.0, 0.2).state == "grade"
    assert single.heading == pytest.approx(0.0, abs=1e-6)                                    # east, from the move
    # Told to stay out of the ground (AgOpenGPS is not steering), it carries.
    assert gb.step(rig.fix(70, 50), 0.0, lost + 6, allowed=False).state == "carry"


def test_where_the_edge_is_under_a_leaning_mast(tmp_path):
    """With the right side down, a mast carries the antenna to the right of the edge's middle and lower than when
    it stands upright; both are taken out. The edge is ahead of the antenna by `reach`, and a two-antenna heading
    is turned by `heading_offset` into the direction of travel."""
    gb, _ = box(tmp_path, reach=2.0)
    lat, lon = (float(v) for v in FRAME.local_to_wgs84(60.0, 50.0))
    roll = 10.0
    x, y, z, slope = gb.pose(Fix(lat, lon, (100.0 + MAST) * FT + DATUM, 4, EAST, roll))
    lean = math.radians(roll)
    # Heading east, right is south: the antenna is south of the middle, so the middle is north of the antenna.
    assert (x, y) == pytest.approx((62.0, 50.0 + MAST * math.sin(lean)), abs=1e-6)
    assert z == pytest.approx(100.0 + MAST * (1 - math.cos(lean)), abs=1e-6) and slope == pytest.approx(math.tan(lean))
    turned, _ = box(tmp_path, heading_offset=90.0, roll_zero=2.0, roll_sign=-1)
    x, y, z, slope = turned.pose(Fix(lat, lon, (100.0 + MAST) * FT + DATUM, 4, 0.0, -8.0))     # antennas side by side
    assert turned.heading == pytest.approx(0.0, abs=1e-9) and slope == pytest.approx(math.tan(lean))


def test_a_tilting_blade_follows_a_slope_across_it(tmp_path):
    """On ground that should slope across the direction of travel, the tilt board is sent how far the right-hand
    end of the edge is from the wanted cross slope, and the lift keeps every point of the edge, at whatever
    slope it has at that moment, on or above the surface. The edge ends on the slope, and where the surface is
    level again, level."""
    _, g = yard(tmp_path)
    s = Surface(g, 10.0)
    gx, gy = np.meshgrid(g.xs, g.ys)
    s.design = np.where(gx < 70, 100.0 + 0.1 * (gy - 50), 100.0)          # x < 70: rising 1 in 10 to the north
    s.ground = s.design + 0.3
    s.fresh[:] = True
    s.refresh()
    gb = GradeBox(s, FRAME, HEIGHT, MAST, tilt=True)
    # The tilt rig's height is that of the right-hand end of the edge over its middle.
    lift, tilt = Rig(101.5), Rig(0.0, onset=(50, 50), speed=(1.0, 1.0), noise=0.5, seed=2)
    lift.take(aog.nano_settings(TUNED))
    tilt.take(aog.nano_settings(TUNED._replace(min_up=50, min_down=50)))
    rows = []

    def turn(t: float) -> None:
        """Drive east; the grade box commands the lift board and the tilt board from one fix."""
        x = 30 + 1.5 * t
        slope = -tilt.z / 5.0                                              # the right end is 5 ft from the middle
        # The mast leans with the blade: the antenna is to the right (south) of the edge's middle, and lower.
        fix = lift.fix(x, 50.0 - MAST * math.sin(math.atan(slope)), roll=math.degrees(math.atan(slope)))
        fix = fix._replace(alt=fix.alt - MAST * (1 - math.cos(math.atan(slope))) * FT)
        cmd = gb.step(fix, 0.0, t)
        lift.take(aog.nano_data(cmd.lift))
        tilt.take(aog.nano_data(cmd.tilt))
        across = np.linspace(-5, 5, 11)                                    # along the edge, north positive
        plane = 100.0 + (0.1 * across if x < 70 else 0.0)
        rows.append((x, slope, float((lift.z + across * slope - plane).min()), cmd))

    for k in range(4000):
        lift.advance(0.01), tilt.advance(0.01)
        if k % 10 == 0:
            turn(lift.t)
    on_slope = [r for r in rows if 45 <= r[0] <= 65]
    assert np.mean([r[1] for r in on_slope]) == pytest.approx(0.1, abs=0.006)             # the edge lies on the slope
    assert np.mean([r[2] for r in on_slope]) == pytest.approx(0.016, abs=0.02)            # and on the design
    level = [r for r in rows if 80 <= r[0] <= 88]
    assert np.mean([r[1] for r in level]) == pytest.approx(0.0, abs=0.006)
    assert min(r[2] for r in rows if r[3].state == "grade" and not 68 < r[0] < 72) > -0.03   # never under the design
    assert all(r[3].tilt is not None for r in rows)
    # With no tilt board the same ground is worked level, at the height of its highest point under the edge.
    flat = GradeBox(s, FRAME, HEIGHT, MAST)
    lat, lon = (float(v) for v in FRAME.local_to_wgs84(50.0, 50.0))
    cmd = flat.step(Fix(lat, lon, (101.0 + MAST) * FT + DATUM, 4, EAST), 0.0, 0.0)
    assert cmd.tilt is None and cmd.high == pytest.approx(101.0 - (100.0 + 0.1 * 4.0), abs=0.11)


# -- on real ports ---------------------------------------------------------------------------------
class Bench(threading.Thread):
    """A rig running in real time behind real ports: a Nano on a pseudo-terminal with its antenna on a TCP port,
    or a Teensy on UDP. The blade is driven east from `x` at `speed` ft/s."""

    def __init__(self, rig: Rig, x: float = 40.0, speed: float = 0.0, hub: int | None = None, steering: bool = True,
                 tilt: Rig | None = None):
        """`tilt`: the rig of the blade's tilt cylinder, whose height is the right-hand end of a 10 ft edge over
        its middle; the antenna then reports the roll that goes with it."""
        super().__init__(daemon=True)
        self.rig, self.x, self.speed, self.hub, self.steering, self.tilt = rig, x, speed, hub, steering, tilt
        self.halt = threading.Event()
        if rig.teensy:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.bind(("127.0.0.1", 0))
            self.udp.setblocking(False)
            self.listen = free_port()                         # where the grade box listens for the board
            self.valve, self.gnss = f"udp://127.0.0.1:{self.udp.getsockname()[1]}:{self.listen}", "board"
        else:
            self.master, slave = os.openpty()
            os.set_blocking(self.master, False)
            self.slave = slave
            self.server = socket.create_server(("127.0.0.1", 0))
            self.server.setblocking(False)
            self.conn: socket.socket | None = None
            self.valve = f"serial:{os.ttyname(slave)}"
            self.gnss = f"tcp://127.0.0.1:{self.server.getsockname()[1]}"

    def run(self) -> None:
        """Be the board and the receiver in real time until stopped: take what `loam blade` sends, move the
        cylinder, and report status and position at the rates the real boards do."""
        rig, last, said, buf = self.rig, time.monotonic(), 0.0, b""
        if not rig.teensy:
            os.write(self.master, b"OG machine controler\r\nv 2.0.1\r\n")       # what the Nano prints as it starts
        while not self.halt.is_set():
            now = time.monotonic()
            rig.advance(now - last)
            self.x += self.speed * (now - last)
            last = now
            with contextlib.suppress(BlockingIOError):
                while rig.teensy:
                    rig.take(self.udp.recv(256))
                buf += os.read(self.master, 4096)
            while len(buf) >= 2 and buf[:2] not in (b"\x7f\xfa", b"\x7f\xf8"):
                buf = buf[1:]
            while len(buf) >= (8 if buf[1:2] == b"\xfa" else 10):
                n = 8 if buf[1] == 0xFA else 10
                rig.take(buf[:n])
                if n == 8:
                    os.write(self.master, rig.status())
                buf = buf[n:]
            if not rig.teensy and self.conn is None:
                with contextlib.suppress(BlockingIOError):
                    self.conn = self.server.accept()[0]
            if now - said >= 0.1:
                said = now
                roll = None if self.tilt is None else math.degrees(math.atan(-self.tilt.z / 5.0))
                fix = rig.fix(self.x, 50.0, roll=roll, mast=0.0 if self.tilt else MAST)   # tilting: no mast to lean
                if rig.teensy:
                    to = ("127.0.0.1", self.listen)
                    self.udp.sendto(teensy_fix(fix), to)
                    self.udp.sendto(rig.status(), to)
                    if self.hub:
                        self.udp.sendto(aog.frame(0x7F, 0xFE, bytes([30, 0, int(self.steering), 0, 0, 0, 0, 0])),
                                        ("127.0.0.1", self.hub))
                        self.udp.sendto(aog.frame(0x7F, 0xF4, struct.pack("<BBHhhhH", 1, 3, 1, -2, -3, -1, 3048)),
                                        ("127.0.0.1", self.hub))
                elif self.conn is not None:
                    try:
                        self.conn.sendall((paogi(fix) + "\r\n").encode())
                    except OSError:                              # the grade box hung up; wait for the next one
                        self.conn.close()
                        self.conn = None
            time.sleep(0.004)

    def stop(self) -> None:
        """End the thread and close every port and socket it opened."""
        self.halt.set()
        self.join(5)
        if self.rig.teensy:
            self.udp.close()
        else:
            for fd in (self.master, self.slave):
                os.close(fd)
            for s in (self.conn, self.server):
                if s is not None:
                    s.close()


def test_links_carry_bytes_without_waiting(tmp_path):
    """A link reads what has arrived and returns at once; a line cut short waits for its end; a bad address is
    said plainly."""
    port, listen = free_port(), free_port()
    a, b = Link(f"udp://127.0.0.1:{port}:{listen}"), Link(f"udp://127.0.0.1:{listen}:{port}")
    only = Link(f"udp://:{free_port()}")
    try:
        assert a.read() == [] and a.lines() == []
        a.write(b"one"), a.write(b"two"), only.write(b"nothing: this link only listens")
        deadline = time.monotonic() + 2
        got: list[bytes] = []
        while len(got) < 2 and time.monotonic() < deadline:
            got += b.read()
        assert got == [b"one", b"two"]
        b.write(b"$A*00\r\n$B")
        deadline = time.monotonic() + 2
        lines: list[str] = []
        while not lines and time.monotonic() < deadline:
            lines += a.lines()
        assert lines == ["$A*00"] and a.buf == b"$B"
    finally:
        for end in (a, b, only):
            end.close()
    for spec, why in (("usb:1", "serial:PORT, tcp://HOST:PORT or udp://"), (f"serial:{tmp_path}/none", "cannot open")):
        with pytest.raises(ValueError, match=why):
            Link(spec)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as taken:      # a port someone holds for themselves
        taken.bind(("", 0))
        with pytest.raises(ValueError, match="cannot listen on UDP port"):
            Link(f"udp://:{taken.getsockname()[1]}")                     # and the socket tried is not left open


def test_two_antennas_on_one_port_must_be_told_apart(tmp_path):
    """The tractor's GPS board and the blade's both send their sentences to one port of the whole subnet. The
    box will not guess which is the blade's: it stops and asks, and when the blade's is named it hears only that
    one."""
    yard(tmp_path)
    (tmp_path / "box.json").write_text(json.dumps({"height": HEIGHT}))
    surface = Surface(SiteGrid(load(str(tmp_path / "yard.loam")), str(tmp_path)), 10.0)
    lat, lon = (float(v) for v in FRAME.local_to_wgs84(50.0, 50.0))
    blade, tractor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    halt = threading.Event()
    port, valve = 0, ""

    def talk() -> None:
        """Both receivers at 10 Hz: the blade's is 101 ft up, the tractor's 8 ft higher on the cab."""
        while not halt.wait(0.1):
            for sock, alt in ((blade, 101.0), (tractor, 109.0)):
                sock.sendto((paogi(Fix(lat, lon, (alt + MAST) * FT + DATUM, 4, EAST, 0.0)) + "\r\n").encode(),
                            ("127.0.0.1", port))

    t = threading.Thread(target=talk)
    try:
        try:
            tractor.bind(("127.0.0.2", 0))          # a second address on this computer; Linux has all of 127/8
        except OSError:
            pytest.skip("needs a second loopback address")
        blade.bind(("127.0.0.1", 0))
        # Ports are picked once both senders hold theirs, so that none can be handed out twice.
        port, valve = free_port(), f"udp://127.0.0.1:{free_port()}:{free_port()}"   # a valve board nobody is
        t.start()
        with pytest.raises(ValueError, match=r"arriving from 127.0.0.1 and 127.0.0.2: say which"):
            run_blade(surface, FRAME, valve, f"udp://:{port}", str(tmp_path / "box.json"), max_wall_s=3,
                      log=lambda m: None)
        said: list[str] = []
        r = run_blade(surface, FRAME, valve, f"udp://127.0.0.1:{port}", str(tmp_path / "box.json"), max_wall_s=1.5,
                      log=said.append)
        # Heard as the blade's alone: the edge reads the same few inches high every time, never 8 ft more.
        high = [float(line.split()[1]) for line in said]
        assert r["state"] == "grade" and high and max(high) - min(high) < 5 and 0 < high[0] < 400
        with pytest.raises(ValueError, match="udp://:PORT or udp://SENDER:PORT"):
            Link("udp://127.0.0.1", hear=True)
    finally:
        halt.set()
        if t.is_alive():
            t.join(5)
        blade.close(), tractor.close()


def test_loam_blade_on_a_nano_board(tmp_path, capsys):
    """The whole of `loam blade` against a Nano valve board on a serial port and an antenna on a TCP port: zero the
    box with the edge standing on the ground, calibrate the valve in the air, then grade. Each step keeps what it
    found in the box file, the board is tuned once at the start of grading, and the board hears from the box
    every tenth of a second."""
    pytest.importorskip("serial")
    yard(tmp_path)
    program, boxfile = str(tmp_path / "yard.loam"), str(tmp_path / "box.json")
    rig = Rig(101.0, noise=1.0)                                  # the edge standing on the ground, 101 ft
    bench = Bench(rig)
    bench.start()
    common = ["blade", program, "--valve", bench.valve, "--gnss", bench.gnss, "--origin", "34.70,-85.70", "--mast", "6"]
    try:
        with pytest.raises(SystemExit):
            main([*common, "--box", boxfile])                    # not zeroed yet
        assert "has not been zeroed" in capsys.readouterr().err
        main([*common, "--box", boxfile, "--zero", "101"])
        assert read_box(boxfile)["height"] == pytest.approx(HEIGHT, abs=0.02) and "zeroed:" in capsys.readouterr().out
        rig.z = 103.0                                            # raised clear of the ground for calibration
        _, g = yard(tmp_path)
        said: list[str] = []
        r = run_blade(Surface(g, 10.0), FRAME, bench.valve, bench.gnss, boxfile, calibrate="lift", mast=MAST,
                      levels=(70, 120, 170), log=said.append)
        assert r["measured"]["up"]["onset"] == pytest.approx(62, abs=12) and not r["measured"]["on_off"]
        deadline = time.monotonic() + 2                         # the board reads its port a moment after it is written
        while list(rig.s) != read_box(boxfile)["lift"]["settings"] and time.monotonic() < deadline:
            time.sleep(0.01)
        assert read_box(boxfile)["lift"]["settings"] == list(rig.s) and "calibrated the lift valve" in " ".join(said)
        tunes, rig.z, bench.x, bench.speed, rig.heard = rig.tunes, 101.5, 40.0, 2.0, -math.inf
        rig.gaps.clear()                                        # from here on: the gaps of the grading run alone
        main([*common, "--box", boxfile, "--seconds", "5"])
        out = capsys.readouterr().out
        assert "grading:" in out and "switch on" in out and "grade" in out
        assert rig.z == pytest.approx(100.516, abs=0.04) and rig.tunes == tunes + 1
        # The board hears from the box every tenth of a second, well inside its watchdog's 0.28 s. This computer
        # keeps no promises about time (the stand-in board shares it, and a pause of either reads as a gap), so
        # the usual gap is held to the design and a single long one is allowed.
        gaps = sorted(rig.gaps)
        assert len(gaps) > 30 and gaps[len(gaps) // 2] < 0.12 and gaps[-2] < 0.28, gaps[-3:]
    finally:
        bench.stop()
    box = json.loads((tmp_path / "box.json").read_text())
    assert set(box) == {"height", "roll_zero", "zeroed", "lift"} and not os.path.exists(boxfile + ".tmp")


def test_loam_blade_on_a_teensy_board_beside_agopengps(tmp_path, capsys):
    """A Teensy valve board on the network brings its own antenna. Beside AgOpenGPS the box grades while
    AgOpenGPS says it is steering, shows which pass the tractor is on, and carries the blade when it is not."""
    yard(tmp_path)
    program, boxfile = str(tmp_path / "yard.loam"), str(tmp_path / "box.json")
    (tmp_path / "box.json").write_text(json.dumps({"height": HEIGHT, "lift": {"settings": list(TEENSY_TUNED)}}))
    for steering, want in ((True, 100.5), (False, 101.5)):
        hub = free_port()
        rig = Rig(101.5, noise=1.0, teensy=True)
        bench = Bench(rig, speed=2.0, hub=hub, steering=steering)
        bench.start()
        try:
            main(["blade", program, "--valve", bench.valve, "--gnss", "board", "--box", boxfile, "--mast", "6",
                  "--origin", "34.70,-85.70", "--with-agopengps", str(hub), "--seconds", "4"])
        finally:
            bench.stop()
        out = capsys.readouterr().out
        assert rig.z == pytest.approx(want, abs=0.05) and rig.s == TEENSY_TUNED
        assert ("pass -2" in out) and (("grade" in out) == steering)
        assert ("state: " + ("grade" if steering else "carry")) in out
    for valve, more, why in (("serial:/dev/null", [], "--gnss board is for a Teensy"),
                             ("udp://127.0.0.1", ["--tilt-valve", "udp://127.0.0.2"], "put the tilt on a Nano board"),
                             ("udp://127.0.0.1", ["--calibrate", "tilt"], "needs the tilt board"),
                             ("udp://127.0.0.1", ["--machine", "nobody"], "has no `blade` length")):
        with pytest.raises(SystemExit):
            main(["blade", program, "--valve", valve, "--gnss", "board", "--box", boxfile, "--origin", "34.7,-85.7",
                  *more])
        assert why in capsys.readouterr().err


def test_loam_blade_with_a_tilt_board(tmp_path, capsys):
    """The full blade: a Teensy board with two antennas lifts, and a Nano board on the tilt cylinder is tuned by
    measurement and then tilts the edge onto a design that slopes across the direction of travel."""
    pytest.importorskip("serial")
    yard(tmp_path)
    xs, ys = np.arange(-2.0, 203.0, 4.0), np.arange(-2.0, 103.0, 4.0)
    for name, surface, above in (("d.xml", "FG", 0.0), ("e.xml", "EG", 0.3)):      # rising 1 in 10 to the north
        write_grid_tin(str(tmp_path / name), surface, xs, ys, 100.0 + 0.1 * (np.meshgrid(xs, ys)[1] - 50) + above, "ft")
    program, boxfile = str(tmp_path / "yard.loam"), str(tmp_path / "box.json")
    (tmp_path / "box.json").write_text(json.dumps({"height": DATUM / FT, "lift": {"settings": list(TEENSY_TUNED)}}))
    lift = Rig(100.8, noise=1.0, teensy=True)
    tilt = Rig(0.0, onset=(50, 50), speed=(1.0, 1.0), noise=0.3, seed=2)
    benches = [Bench(lift, speed=0.0, tilt=tilt), Bench(tilt)]
    for b in benches:
        b.start()
    try:
        r = run_blade(Surface(SiteGrid(load(program), str(tmp_path)), 10.0), FRAME, benches[0].valve, "board", boxfile,
                      tilt_valve=benches[1].valve, calibrate="tilt", levels=(60, 110, 160), log=lambda m: None)
        assert r["measured"]["down"]["onset"] == pytest.approx(50, abs=12) and not r["measured"]["on_off"]
        tilt.z, benches[0].speed = 0.0, 1.5
        main(["blade", program, "--valve", benches[0].valve, "--gnss", "board", "--tilt-valve", benches[1].valve,
              "--box", boxfile, "--origin", "34.70,-85.70", "--seconds", "7"])
    finally:
        for b in benches:
            b.stop()
    assert "grading:" in capsys.readouterr().out and set(read_box(boxfile)) == {"height", "lift", "tilt"}
    slope = -tilt.z / 5.0
    assert slope == pytest.approx(0.1, abs=0.01)                                # the edge lies on the slope
    assert lift.z == pytest.approx(100.0 + 5 * (0.1 - slope), abs=0.06)         # and its middle on the design
