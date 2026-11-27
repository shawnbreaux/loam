"""Real site sensors as Loam Wire messages: people from cameras, rain from a wireless gauge.

Two free programs do the sensing. Each publishes what it finds on the site's MQTT broker; `loam sense` listens
and republishes it as the `detection` and `weather` messages a Loam controller understands. Nothing of theirs is
linked into Loam.

    people   Frigate (MIT licence): a camera recorder that detects people. Each site camera, and each zone drawn
             in its picture, is mapped to an *area* on the site: a centre and a radius.
    rain     rtl_433 (GPL-2.0): decodes the radio messages of inexpensive wireless weather stations.

A camera cannot by itself say where in its picture's footprint a person stands, so a detection covers the whole
area, and the controller measures distances from the area's nearest edge. A camera that has been *calibrated to
the ground* can: four or more points whose place is known both in the picture and on the site fix the mapping
from one to the other (a homography; it holds where the ground is close to flat). The point where a person's box
meets the ground is then a place on the site, a metre or so wide, and it is handed to `loam.tags` to be matched
with the phones the site hears.

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

import numpy as np

from . import wire
from .checker import Model
from .runtime.bus import Bus, tolerant
from .tags import Tags
from .units import length_unit_si

# Frigate is asked for its full picture this often (it answers at most once a second) ...
POLL_S = 1.5
# ... and after this long without an answer the cameras count as not watching.
SILENT_S = 6.0
# Detections are repeated this often, so a controller that starts late learns of them.
REPEAT_S = 5.0
TAG_REPEAT_S = 1.0           # a person located by a phone or tracker is reported again this often
# A camera delivering fewer pictures per second than this has stopped (Frigate's own threshold for "unusable").
MIN_FPS = 0.1
MAX_SEEN = 12                # sightings reported one by one; any beyond that share one circle
FOOT_PX = 3.0                # how well the foot of a person's box is known in the picture, pixels, at least
FOOT_FLOOR_M = 0.5           # a person seen by a camera is never placed closer than this


class Area(NamedTuple):
    """A circle on the site, in site coordinates and units: where the people one camera or zone sees can be."""
    x: float
    y: float
    radius: float


class Ground:
    """A camera's mapping from picture to ground, from points known in both: pixel (px, py) is site (x, y)."""

    def __init__(self, pairs: list):
        """`pairs` are [px, py, x, y], four or more, no three of them in a line. Pixels are those of the picture
        Frigate detects on (its `detect` width and height), since that is what its boxes are measured in."""
        p = np.array(pairs, float)
        if p.ndim != 2 or p.shape[1] != 4 or len(p) < 4 or not np.isfinite(p).all():
            raise ValueError("ground needs four or more [px, py, x, y] points")
        # The direct linear transform: each pair gives two equations in the nine numbers of the mapping, and the
        # singular vector of the smallest singular value solves them. Both sides are first shifted and scaled to
        # be of order one, without which the equations are too ill-conditioned to trust.
        self.a, self.b = self._norm(p[:, :2]), self._norm(p[:, 2:])
        q = np.column_stack([p[:, :2], np.ones(len(p))]) @ self.a.T
        w = np.column_stack([p[:, 2:], np.ones(len(p))]) @ self.b.T
        rows = []
        for (u, v, _), (x, y, _) in zip(q, w, strict=True):
            rows += [[-u, -v, -1, 0, 0, 0, u * x, v * x, x], [0, 0, 0, -u, -v, -1, u * y, v * y, y]]
        sv = np.linalg.svd(np.array(rows))
        if sv[1][7] < 1e-6 * sv[1][0]:                   # eight independent equations are needed for the nine
            raise ValueError("ground points must not all lie on one line")
        self.h = np.linalg.inv(self.b) @ sv[2][-1].reshape(3, 3) @ self.a
        # In front of the camera the third coordinate has one sign; beyond the horizon it has the other.
        depth = np.column_stack([p[:, :2], np.ones(len(p))]) @ self.h[2]
        if not (np.all(depth > 0) or np.all(depth < 0)):
            raise ValueError("ground points are not all on the ground in front of the camera")
        self.h = self.h * np.sign(depth[0])
        back = np.array([self.at(u, v) for u, v in p[:, :2]])
        err = float(np.abs(back - p[:, 2:]).max())
        if err > 0.02 * float(np.ptp(p[:, 2:], axis=0).max()):
            raise ValueError(f"ground points do not fit one flat ground (worst misfit {err:.2f} site units): "
                             "check them, or the ground is not flat enough to calibrate")

    @staticmethod
    def _norm(pts: np.ndarray) -> np.ndarray:
        """The matrix that moves points to their centre and scales them to a mean distance of sqrt(2) from it."""
        c = pts.mean(0)
        s = math.sqrt(2) / max(float(np.hypot(*(pts - c).T).mean()), 1e-12)
        return np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])

    def at(self, px: float, py: float) -> tuple[float, float] | None:
        """The place on the ground seen at a pixel, or None for a pixel at or above the horizon."""
        x, y, w = self.h @ (px, py, 1.0)
        return (float(x / w), float(y / w)) if w > 1e-9 else None

    def person(self, box: list) -> Area | None:
        """Where a person stands, from their box [x1, y1, x2, y2]: the middle of its lower edge, with a radius that
        covers what the box cannot tell (where across its width the feet are, and the lower edge to a few
        pixels). None if any of that is beyond the horizon."""
        x1, y1, x2, y2 = (float(v) for v in box)
        dv = max(FOOT_PX, 0.03 * abs(y2 - y1))
        cx = 0.5 * (x1 + x2)
        foot = self.at(cx, y2)
        rim = [self.at(x1, y2), self.at(x2, y2), self.at(cx, y2 - dv), self.at(cx, y2 + dv)]
        if foot is None or any(r is None for r in rim):
            return None
        return Area(foot[0], foot[1], max(math.hypot(r[0] - foot[0], r[1] - foot[1]) for r in rim if r))


class Camera(NamedTuple):
    """A site camera: the area its whole picture covers, the areas of the zones drawn in the picture, and its
    mapping from picture to ground if it has been calibrated."""
    area: Area
    zones: dict[str, Area]
    ground: Ground | None = None


def load_areas(path: str) -> dict[str, Camera]:
    """Read the areas file, which names the site's cameras as Frigate names them:

        {"cameras": {"yard": {"x": 80, "y": 50, "radius": 60,
                              "zones": {"gate": {"x": 120, "y": 40, "radius": 30}}}}}

    Coordinates and radii are in the site's units. Zones are optional. Cameras Frigate has that are not listed
    here (the office door, say) are ignored. A camera may add `"ground": [[px, py, x, y], ...]`: four or more
    points of the ground known both in its picture and on the site (see `Ground`); people it sees are then placed
    where they stand.
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
        cameras = {str(n): Camera(area(c), {str(z): area(a) for z, a in (c.get("zones") or {}).items()},
                                  Ground(c["ground"]) if c.get("ground") else None)
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
        # Whether people seen by calibrated cameras are placed on the ground (and reported by `sightings`), or
        # reported like any other camera's, by area. The caller switches it on when it reads the sightings.
        self.placing = False

    def stats(self, msg: dict) -> None:
        """Take in Frigate's statistics message, for each camera's picture rate."""
        self.fps = {n: float(c.get("camera_fps", 0.0)) for n, c in msg["cameras"].items()}

    def names(self) -> list[str]:
        """Every name a detection can carry: each camera, and each of its mapped zones as "camera/zone"."""
        return [n for cam, c in self.cameras.items() for n in (cam, *(f"{cam}/{z}" for z in c.zones))]

    def _watching(self, name: str) -> dict | None:
        """Frigate's status of a camera, if it is watching: reported, detecting, and delivering pictures."""
        status = self.activity.get(name)
        ok = (isinstance(status, dict) and (status.get("config") or {}).get("detect")
              and self.fps.get(name, 1.0) >= MIN_FPS)
        return status if ok else None

    def _stands(self, cam: Camera, obj: dict) -> Area | None:
        """Where on the ground a person seen by a calibrated camera stands, if that can be told."""
        box = obj.get("box")
        if not (self.placing and cam.ground and isinstance(box, list) and len(box) == 4):
            return None
        return cam.ground.person(box)

    def sightings(self) -> dict[str, list[Area]]:
        """For each calibrated camera that is watching: where each person it sees stands. A camera that sees
        nobody is listed with nothing, which is how a person who has walked out of its picture is forgotten."""
        out: dict[str, list[Area]] = {}
        for name, cam in self.cameras.items():
            status = self._watching(name) if cam.ground else None
            if status is None:
                continue
            people = [o for o in status.get("objects") or []
                      if str(o.get("label")).removesuffix("-verified") == "person"]
            out[name] = [a for a in (self._stands(cam, o) for o in people) if a is not None]
        return out

    def detections(self) -> dict[str, tuple[str, Area]]:
        """The detections the latest picture calls for, by name ("camera" or "camera/zone"): each a kind and an area.

        A camera that Frigate does not report, that has detection switched off or that delivers no pictures
        cannot say its area is empty: the area is "unknown". Otherwise each person is in the areas of the mapped
        zones they stand in, or, outside every mapped zone, somewhere in the camera's area. A person a calibrated
        camera can place on the ground is left to `sightings`.
        """
        out: dict[str, tuple[str, Area]] = {}
        for name, cam in self.cameras.items():
            status = self._watching(name)
            if status is None:
                out[name] = ("unknown", cam.area)
                continue
            for obj in status.get("objects") or []:
                # A recognised person is labelled "person-verified".
                if str(obj.get("label")).removesuffix("-verified") != "person":
                    continue
                if self._stands(cam, obj) is not None:
                    continue                             # placed on the ground: reported by `sightings`
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


def _tag_detections(tags: Tags, name: str, now: float, everywhere: Area) -> dict[str, tuple[str, Area]]:
    """The detections that people's phones and the sightings call for now: named people, things seen that are
    nobody in particular (the first `MAX_SEEN` one by one, any more under one circle that holds them all), and
    "unknown" over the whole site for each relied-upon source that is silent."""
    def area(c: tuple[float, float, float]) -> Area:
        """A circle as a detection's area, rounded to what a message needs."""
        return Area(round(c[0], 2), round(c[1], 2), round(c[2], 2))

    who, others = tags.fused(now, 1.5 * TAG_REPEAT_S)
    out = {f"{name}/tag/{p}": ("person", area(c)) for p, c in who.items()}
    others.sort()
    for k, c in enumerate(others[:MAX_SEEN], 1):
        out[f"{name}/seen/{k}"] = ("person", area(c))
    rest = others[MAX_SEEN:]
    if rest:
        cx, cy = sum(c[0] for c in rest) / len(rest), sum(c[1] for c in rest) / len(rest)
        out[f"{name}/seen/more"] = ("person", area((cx, cy, max(math.hypot(c[0] - cx, c[1] - cy) + c[2]
                                                                  for c in rest))))
    for source in tags.silent(now):
        out[f"{name}/watch/{source}"] = ("unknown", everywhere)
    return out


def run_sense(model: Model, broker: str, cameras: dict[str, Camera] | None = None, frigate_prefix: str = "frigate",
              rain_topic: str | None = None, gauge: str | None = None, name: str = "sense",
              max_wall_s: float | None = None, tags: Tags | None = None) -> dict:
    """Run until stopped (or for `max_wall_s`): republish Frigate's people as detections and a gauge's rain as
    weather. Returns how many messages of each kind were sent.

    `cameras` (the areas file) switches people on; `frigate_prefix` is Frigate's MQTT topic prefix. `rain_topic`
    (rtl_433's events topic, e.g. `rtl_433/+/events`) switches rain on; `gauge` is "MODEL" or "MODEL:ID".
    `tags` (the tags file, see `loam.tags`) switches on people located by their phones and trackers, and
    sightings from lidar and other sources. Cameras calibrated to the ground feed the same sightings.
    `name` is this sensor's name in the messages.

    Detections are named `<name>/<camera>[/<zone>]` for a camera's area, `<name>/tag/<person>` for a person
    located by name, `<name>/seen/<n>` for a person or person-sized thing seen and not named, and
    `<name>/watch/<source>` for a sightings source the site relies on that has fallen silent.
    """
    if not cameras and not rain_topic and not tags:
        raise ValueError("nothing to sense: give --areas, --tags and/or --rain")
    site = model.site["name"]
    topic = wire.topic(site, None, "detection")
    seq = int(time.time())
    everywhere = Area(0.0, 0.0, wire.EVERYWHERE)

    def detection(sensor: str, kind: str, area: Area, ts: float) -> dict:
        """One detection message from this program."""
        return wire.detection(site, sensor, seq, kind, area.x, area.y, ts=ts, radius=area.radius)

    people = People(cameras) if cameras else None
    if people and any(c.ground for c in people.cameras.values()):
        # Calibrated cameras place people on the ground; the tags take those places in, with or without phones.
        people.placing = True
        tags = tags or Tags({"sightings": {}}, None, length_unit_si(model.site.get("units", "ft")),
                            (0.0, 0.0, 0.0, 0.0), site)
    rain = Rain(*(gauge.split(":", 1) if gauge else ())) if rain_topic else None
    f = frigate_prefix
    subs = [f"{f}/available", f"{f}/camera_activity", f"{f}/stats"] if people else []
    subs += [rain_topic] if rain_topic else []
    subs += tags.topics() if tags else []
    detecting = bool(people or tags)
    # If this program dies, the broker publishes "nobody is watching" for it: an unknown presence over the whole
    # site, retained so that a controller that starts later still sees it.
    will = (topic, detection(name, "unknown", everywhere, 0.0)) if detecting else None
    bus = Bus(broker, f"loam-sense-{site}-{os.getpid()}", will=will, raw=True).start(subs)
    sent = {"detection": 0, "weather": 0}
    # What is out now, by sensor name. `name` itself stands for "not watching".
    published: dict[str, tuple[str, Area] | None] = {}
    frigate_up, asked_at, answered_at, repeated_at, rain_sent = False, 0.0, 0.0, 0.0, (0.0, -1.0)
    end = time.time() + max_wall_s if max_wall_s else math.inf
    connects = 0
    # Every sensor name this program uses: itself, each camera and zone, each person with a phone or tracker.
    everything = {name} if detecting else set()
    everything |= {f"{name}/{k}" for k in people.names()} if people else set()
    if tags:
        everything |= {f"{name}/tag/{p}" for p in tags.names()} | {f"{name}/watch/{s}" for s in tags.watch}
        everything |= {f"{name}/seen/{k}" for k in (*range(1, MAX_SEEN + 1), "more")}
    tag_want: dict[str, tuple[str, Area]] = {}
    tag_at = 0.0
    try:
        while time.time() < end:
            if detecting and bus.connects != connects:
                # Just connected, or connected again after the link dropped. The broker may hold a retained "not
                # watching" (from an earlier run, or published as our last will a moment ago): delete it. And
                # nothing a controller knows from before can be trusted to be current, so everything is sent
                # afresh: `published` starts as "not watching" plus every camera and zone with nothing known,
                # and the loop below sends a "clear" for each one that is not in what the cameras show now.
                connects = bus.connects
                bus.clear(topic)
                published = {**dict.fromkeys(everything), **({name: ("unknown", everywhere)} if people else {})}
            for t, msg in bus.drain(0.2):
                with tolerant(t):
                    now = time.time()
                    if people and t == f"{f}/available":
                        frigate_up = msg == "online"
                    elif people and t == f"{f}/camera_activity":
                        people.activity, answered_at = dict(msg), now
                        if tags and people.placing:
                            for cam, stood in people.sightings().items():
                                tags.seen.put(cam, now, [(a.x, a.y, a.radius, True) for a in stood], now)
                            tags.fresh = True
                    elif people and t == f"{f}/stats":
                        people.stats(msg)
                    elif tags and tags.feed(t, msg, now):
                        pass
                    elif rain and isinstance(msg, dict):
                        rate = rain.reading(msg, now)
                        # Send changes at once, and a reading a minute so the controller knows the gauge is alive.
                        if rate is not None and (rate != rain_sent[1] or now - rain_sent[0] >= 60.0):
                            seq += 1
                            bus.pub(wire.topic(site, None, "weather"), wire.weather(site, name, seq, rate, ts=now))
                            sent["weather"] += 1
                            rain_sent = (now, rate)
            if not detecting:
                continue
            now = time.time()
            want: dict[str, tuple[str, Area]] = {}
            if people:
                if frigate_up and now - asked_at >= POLL_S:
                    bus.poke(f"{f}/onConnect")             # Frigate answers with camera_activity
                    asked_at = now
                watching = frigate_up and now - answered_at < SILENT_S
                want = ({f"{name}/{k}": v for k, v in people.detections().items()} if watching
                        else {name: ("unknown", everywhere)})
            if tags and (tags.fresh or now - tag_at >= TAG_REPEAT_S):
                # Each person as a circle that already allows for the walking they can do before the next report.
                tags.fresh, tag_at = False, now
                tag_want = _tag_detections(tags, name, now, everywhere)
            want.update(tag_want)
            repeat = now - repeated_at >= REPEAT_S
            # A repeat states everything again, the "clear" of every camera and zone that shows nobody included:
            # a controller that missed one message (it reconnected a moment late) is put right within seconds.
            for sensor in sorted(set(want) | set(published) | (everything if repeat else set())):
                if sensor not in want:
                    seq += 1
                    bus.pub(topic, wire.detection(site, sensor, seq, "clear", 0.0, 0.0, ts=now), qos=1)
                elif want[sensor] != published.get(sensor) or repeat:
                    seq += 1
                    bus.pub(topic, detection(sensor, *want[sensor], now), qos=1)
                else:
                    continue
                sent["detection"] += 1
            published = dict(want)
            if repeat:
                repeated_at = now
    finally:
        if detecting:
            # Stopping on purpose is still "not watching". Retained, like the last will a crash would leave.
            bus.pub(topic, detection(name, "unknown", everywhere, time.time()), qos=1, retain=True, wait=2.0)
        bus.stop()
    return sent
