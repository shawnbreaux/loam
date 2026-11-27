"""Real site sensors as Loam Wire messages: people from cameras, rain from a wireless gauge.

Two free programs do the sensing. Each publishes what it finds on the site's MQTT broker; `loam sense` listens
and republishes it as the `detection` and `weather` messages a Loam controller understands. Nothing of theirs is
linked into Loam.

    people   Frigate (MIT licence): a camera recorder that detects people. Each site camera, and each zone drawn
             in its picture, is mapped to an *area* on the site: a centre and a radius.
    rain     rtl_433 (GPL-2.0): decodes the radio messages of inexpensive wireless weather stations.

A camera cannot say where in its picture's footprint a person stands, so a detection covers the whole area, and
the controller measures distances from the area's nearest edge.

What is not known is treated as unsafe:

* a site camera that delivers no pictures, or has detection switched off, makes its whole area "unknown";
* if Frigate stops answering, or this program stops, a detection of kind "unknown" covering the whole site is
  published.

A rule such as `when proximity(person, machine) < 50 ft do stop all` then stops the machines until the cameras
are watching again.

This is supervision, not a safety function in the sense of ISO 13849: cameras miss people in dust, glare and
darkness. The e-stop and each machine's own protective devices stay in charge of safety.
"""
from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from typing import NamedTuple

from . import wire
from .checker import Model
from .runtime.bus import Bus, tolerant

# Frigate is asked for its full picture this often (it answers at most once a second) ...
POLL_S = 1.5
# ... and after this long without an answer the cameras count as not watching.
SILENT_S = 6.0
# Detections are repeated this often, so a controller that starts late learns of them.
REPEAT_S = 5.0
# A camera delivering fewer pictures per second than this has stopped (Frigate's own threshold for "unusable").
MIN_FPS = 0.1


class Area(NamedTuple):
    """A circle on the site, in site coordinates and units: where the people one camera or zone sees can be."""
    x: float
    y: float
    radius: float


class Camera(NamedTuple):
    """A site camera: the area its whole picture covers, and the areas of the zones drawn in the picture."""
    area: Area
    zones: dict[str, Area]


def load_areas(path: str) -> dict[str, Camera]:
    """Read the areas file, which names the site's cameras as Frigate names them:

        {"cameras": {"yard": {"x": 80, "y": 50, "radius": 60,
                              "zones": {"gate": {"x": 120, "y": 40, "radius": 30}}}}}

    Coordinates and radii are in the site's units. Zones are optional. Cameras Frigate has that are not listed
    here (the office door, say) are ignored.
    """
    def area(a: dict) -> Area:
        """One circle; refuses anything that is not three finite numbers with a radius above zero."""
        out = Area(float(a["x"]), float(a["y"]), float(a["radius"]))
        if not (out.radius > 0 and math.isfinite(out.x + out.y + out.radius)):
            raise ValueError("x, y and radius must be finite, the radius above 0")
        return out

    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    try:
        cameras = {str(n): Camera(area(c), {str(z): area(a) for z, a in (c.get("zones") or {}).items()})
                   for n, c in doc["cameras"].items()}
        if not cameras:
            raise ValueError("no cameras")
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        raise ValueError(f'{path}: expected {{"cameras": {{name: {{"x", "y", "radius", "zones"}}}}}} ({e})') from e
    return cameras


class People:
    """Where people may be, from Frigate's own picture of each site camera.

    Frigate publishes that picture (`camera_activity`: for every camera, the objects it is following now and the
    zones each is in) whenever it is asked. This keeps the latest one, and each camera's picture rate from
    Frigate's statistics, and turns them into detections. Nothing is remembered between pictures, so a person
    who was in view before this program started is found, and a missed message cannot leave a person behind.
    """

    def __init__(self, cameras: dict[str, Camera]):
        """`cameras` is the areas file: the site's cameras and zones as circles on the site."""
        self.cameras = cameras
        self.activity: dict = {}                    # Frigate's latest camera_activity message
        self.fps: dict[str, float] = {}             # pictures per second each camera delivers, once known

    def stats(self, msg: dict) -> None:
        """Take in Frigate's statistics message, for each camera's picture rate."""
        self.fps = {n: float(c.get("camera_fps", 0.0)) for n, c in msg["cameras"].items()}

    def detections(self) -> dict[str, tuple[str, Area]]:
        """The detections the latest picture calls for, by name ("camera" or "camera/zone"): each a kind and an area.

        A camera that Frigate does not report, that has detection switched off or that delivers no pictures
        cannot say its area is empty: the area is "unknown". Otherwise each person is in the areas of the mapped
        zones they stand in, or, outside every mapped zone, somewhere in the camera's area.
        """
        out: dict[str, tuple[str, Area]] = {}
        for name, cam in self.cameras.items():
            status = self.activity.get(name)
            if (not isinstance(status, dict) or not (status.get("config") or {}).get("detect")
                    or self.fps.get(name, 1.0) < MIN_FPS):
                out[name] = ("unknown", cam.area)
                continue
            for obj in status.get("objects") or []:
                # A recognised person is labelled "person-verified".
                if str(obj.get("label")).removesuffix("-verified") != "person":
                    continue
                zones = [z for z in obj.get("current_zones") or [] if z in cam.zones]
                for z in zones:
                    out[f"{name}/{z}"] = ("person", cam.zones[z])
                if not zones:
                    out[name] = ("person", cam.area)
        return out


class Rain:
    """Rain rate from a wireless gauge's readings.

    Most gauges report the total rain since they were switched on, counted by a tipping bucket. The rate is the
    increase of that total over the last `window_s` seconds: long enough to smooth single tips of the bucket,
    short enough to notice that rain has started or stopped. A few gauges report a rate themselves.
    """

    def __init__(self, model: str | None = None, device_id: str | None = None, window_s: float = 600.0):
        """`model` and `device_id` pick one gauge (rtl_433 hears the neighbours' weather stations too); with
        neither, the first gauge heard is used from then on.
        """
        self.model, self.device_id, self.window = model, device_id, window_s
        self.totals: deque[tuple[float, float]] = deque()         # (time, total rain in metres), oldest first
        self.last_heard = 0.0

    def reading(self, msg: dict, now: float) -> float | None:
        """Take in one decoded radio message. Returns the rain rate in m/s, or None if the message is not a rain
        reading from the chosen gauge.
        """
        mm_h = msg.get("rain_rate_mm_h", msg["rain_rate_in_h"] * 25.4 if "rain_rate_in_h" in msg else None)
        total = msg.get("rain_mm", msg["rain_in"] * 25.4 if "rain_in" in msg else None)
        if mm_h is None and total is None:
            return None
        if self.model is None:                                    # adopt the first gauge heard
            self.model, self.device_id = str(msg.get("model")), str(msg.get("id"))
        ours = str(msg.get("model")) == self.model and self.device_id in (None, str(msg.get("id")))
        if not ours:
            return None
        self.last_heard = now
        if mm_h is not None:
            return max(0.0, float(mm_h)) / 1000.0 / 3600.0
        metres = float(total or 0.0) / 1000.0
        if self.totals and metres < self.totals[-1][1]:           # the gauge restarted (new batteries): start again
            self.totals.clear()
        self.totals.append((now, metres))
        while len(self.totals) > 1 and self.totals[1][0] <= now - self.window:
            self.totals.popleft()                                 # keep one reading at or before the window's start
        (t0, m0), (t1, m1) = self.totals[0], self.totals[-1]
        # Spread the increase over at least a minute, so two readings a few seconds apart do not read as a deluge.
        return (m1 - m0) / max(t1 - t0, 60.0) if len(self.totals) > 1 else 0.0


def run_sense(model: Model, broker: str, cameras: dict[str, Camera] | None = None, frigate_prefix: str = "frigate",
              rain_topic: str | None = None, gauge: str | None = None, name: str = "sense",
              max_wall_s: float | None = None) -> dict:
    """Run until stopped (or for `max_wall_s`): republish Frigate's people as detections and a gauge's rain as
    weather. Returns how many messages of each kind were sent.

    `cameras` (the areas file) switches people on; `frigate_prefix` is Frigate's MQTT topic prefix. `rain_topic`
    (rtl_433's events topic, e.g. `rtl_433/+/events`) switches rain on; `gauge` is "MODEL" or "MODEL:ID".
    `name` is this sensor's name in the messages.
    """
    if not cameras and not rain_topic:
        raise ValueError("nothing to sense: give --areas and/or --rain")
    site = model.site["name"]
    topic = wire.topic(site, None, "detection")
    seq = int(time.time())
    everywhere = Area(0.0, 0.0, wire.EVERYWHERE)

    def detection(sensor: str, kind: str, area: Area, ts: float) -> dict:
        """One detection message from this program."""
        return wire.detection(site, sensor, seq, kind, area.x, area.y, ts=ts, radius=area.radius)

    people = People(cameras) if cameras else None
    rain = Rain(*(gauge.split(":", 1) if gauge else ())) if rain_topic else None
    f = frigate_prefix
    subs = [f"{f}/available", f"{f}/camera_activity", f"{f}/stats"] if people else []
    subs += [rain_topic] if rain_topic else []
    # If this program dies, the broker publishes "the cameras are not watching" for it: an unknown presence over
    # the whole site, retained so that a controller that starts later still sees it.
    will = (topic, detection(name, "unknown", everywhere, 0.0)) if people else None
    bus = Bus(broker, f"loam-sense-{site}-{os.getpid()}", will=will, raw=True).start(subs)
    sent = {"detection": 0, "weather": 0}
    # What is out now, by sensor name. `name` itself stands for "not watching". It starts there, so that the
    # first moment of watching sends a "clear" for it: a controller may still hold the retained one an earlier
    # run left behind.
    published: dict[str, tuple[str, Area]] = {name: ("unknown", everywhere)} if people else {}
    frigate_up, asked_at, answered_at, repeated_at, rain_sent = False, 0.0, 0.0, 0.0, (0.0, -1.0)
    end = time.time() + max_wall_s if max_wall_s else math.inf
    try:
        if people:
            bus.clear(topic)                   # the retained "not watching" of an earlier run
        while time.time() < end:
            for t, msg in bus.drain(0.2):
                with tolerant(t):
                    now = time.time()
                    if people and t == f"{f}/available":
                        frigate_up = msg == "online"
                    elif people and t == f"{f}/camera_activity":
                        people.activity, answered_at = dict(msg), now
                    elif people and t == f"{f}/stats":
                        people.stats(msg)
                    elif rain and isinstance(msg, dict):
                        rate = rain.reading(msg, now)
                        # Send changes at once, and a reading a minute so the controller knows the gauge is alive.
                        if rate is not None and (rate != rain_sent[1] or now - rain_sent[0] >= 60.0):
                            seq += 1
                            bus.pub(wire.topic(site, None, "weather"), wire.weather(site, name, seq, rate, ts=now))
                            sent["weather"] += 1
                            rain_sent = (now, rate)
            if not people:
                continue
            now = time.time()
            if frigate_up and now - asked_at >= POLL_S:
                bus.poke(f"{f}/onConnect")                 # Frigate answers with camera_activity
                asked_at = now
            watching = frigate_up and now - answered_at < SILENT_S
            want = ({f"{name}/{k}": v for k, v in people.detections().items()} if watching
                    else {name: ("unknown", everywhere)})
            repeat = now - repeated_at >= REPEAT_S
            for sensor in sorted(set(want) | set(published)):
                if sensor not in want:
                    seq += 1
                    bus.pub(topic, wire.detection(site, sensor, seq, "clear", 0.0, 0.0, ts=now), qos=1)
                elif want[sensor] != published.get(sensor) or repeat:
                    seq += 1
                    bus.pub(topic, detection(sensor, *want[sensor], now), qos=1)
                else:
                    continue
                sent["detection"] += 1
            published = want
            if repeat:
                repeated_at = now
    finally:
        if people:
            # Stopping on purpose is still "not watching". Retained, like the last will a crash would leave.
            bus.pub(topic, detection(name, "unknown", everywhere, time.time()), qos=1, retain=True, wait=2.0)
        bus.stop()
    return sent
