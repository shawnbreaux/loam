"""People located by what they carry: a phone or a tracker, as circles the machines must keep clear of.

Three free ways to locate a person, each over the site's MQTT broker, none linked into Loam:

* **OwnTracks** (phone app, open source): the phone's own GPS position and its accuracy, straight to the broker.
* **Wi-Fi**: the site's access points report how strongly they hear each phone. That gives a distance, roughly, and
  so a circle round the access point; where two access points hear the phone, the overlap of their circles.
  OpenWrt's `ubus call hostapd.<interface> get_clients` output is understood as it is. An access point may ride
  on a machine; its place is then wherever that machine reports itself.
* **Meshtastic** (LoRa trackers, open firmware): the tracker's GPS position, relayed by a gateway as JSON.

None of these says where a person *is*. Each says where they were, to within some distance, some seconds ago. So a
person is always a circle: the uncertainty of the fix, grown by how far someone can walk in the time since
(`speed_m_s`). The freshest, tightest source wins. A phone that falls silent on the site therefore becomes a circle
that grows until it covers the site and the machines stop: what is unknown is unsafe.

Wi-Fi signal strength is a poor ruler: the same distance can read 10 dB apart from one moment to the next. The
circle's radius is the distance at which the signal would still be this strong with `K_SIGMA` standard deviations of
luck (`sigma_db`), so it is large: 18 m when the phone is 5 m away, 65 m at 20 m, over 250 m at 50 m, with the
default model. Across a site it therefore says little more than "this phone is here somewhere"; three access points
120 m apart still leave a circle about 170 m in radius. It is sharp only close to an access point, which is why one
carried by a machine is useful: a strong signal there means a person is near that machine. A measured range (Wi-Fi
RTT, `range_m`) is used as it is. Measure the model on your own site with `loam wifi-fit`.

**Fusion.** A phone says *who* is on the site and roughly where. A lidar scan, or a camera calibrated to the
ground, says exactly where a person-sized thing stands, and not who it is, and it is blind behind a machine. The
two are put together (`account`):

* every sighting is a small keep-out circle of its own, always;
* on a site where everyone on foot carries a listed phone or tracker (`sightings.everyone_tagged`), a person whose
  own circle holds a sighting is taken to be one of the things seen there, and stops being a wide circle. If it
  holds exactly one, that sighting is the person, to the sensor's precision;
* a person whose circle holds no sighting keeps the wide circle: a phone is heard and nobody is seen where it could
  be. The same when more phones are heard in a place than things are seen there. That head count is what neither
  sensor can do alone.

Without that site rule nothing a phone says is narrowed by a sighting, because a visitor who is seen could stand in
for a worker who is not. Both are then reported side by side.

A lidar reports shapes, not people. A shape counts toward the head count only if it has moved in the last
`fixture_s`: a post or a barrel never accounts for a phone. Sightings come from calibrated cameras (see
`loam.sensors`), from scans (`loam terrain --objects`) and from anything else that publishes to the sightings
topic.

Positions that are known well (a phone's GPS, a person pinned to a sighting) are also used to measure the Wi-Fi
model on the site as it runs (`wifi.learn`), so the Wi-Fi circles become as tight as the site's own radio
conditions allow.

This is supervision, not a safety function: a phone can be left in the truck. It supplements cameras and the rules
of the site; it replaces neither.
"""
from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from typing import Any, NamedTuple

WALK_M_S = 2.5               # how fast a person can leave the place they were last located, m/s
K_SIGMA = 2.5                # Wi-Fi: standard deviations of signal fading the radius allows for (99.4 % one-sided)
WIFI_FRESH_S = 15.0          # Wi-Fi: an access point's report older than this is no longer used
GPS_FLOOR_M = 5.0            # a phone's or tracker's GPS is never trusted closer than this
CELL_M = 150.0               # "connected to this access point" with no signal strength: within this distance
GONE_M = 100.0               # located this far outside the site: off the site, and not reported
MESH_CELL_M = 23905787.925   # Meshtastic: size of a position cell at 0 precision bits; halves with every bit

SEEN_KEEP_S = 30.0           # a sighting whose source sends nothing newer is forgotten after this long
SEEN_FLOOR_M = 0.5           # a sighting is never trusted closer than this
STILL_M = 1.5                # an object seen within this distance of where it was has not moved
FIXTURE_S = 600.0            # an object that has not moved for this long is part of the site, not a person
LEARN_M = 10.0               # a position known this closely can teach the Wi-Fi model
LEARN_MIN = 40               # samples before the Wi-Fi model is first refitted, then at every LEARN_MIN // 2 more
LEARN_KEEP = 400             # samples kept; older ones are forgotten, so the model follows the site
SIGMA_FLOOR_DB = 3.0         # a learned fading is never trusted to be smaller than this
EXPONENT_RANGE = (1.8, 4.5)  # a learned path-loss exponent outside this is a bad fit, and is not used

Circle = tuple[float, float, float]                      # x, y, radius


class Sighting(NamedTuple):
    """A person-sized thing seen at a place: a circle in site units, the source that saw it, and whether it may
    account for a phone (a detector called it a person, or it is a shape that has moved lately)."""
    x: float
    y: float
    radius: float
    counts: bool
    source: str = ""


class Seen:
    """What the precise sensors see, source by source. Each source sends whole pictures: everything it sees now. A
    newer picture replaces the older one, so a thing no longer seen is gone, and nothing is kept from a source
    that has fallen silent for `SEEN_KEEP_S`.

    A shape (as against a detected person) is followed from picture to picture only to know whether it has moved:
    each remembers where it was first seen and since when. Shapes in a source's first picture are taken to have
    stood there for ever.
    """

    def __init__(self, u: float, speed_m_s: float = WALK_M_S, fixture_s: float = FIXTURE_S):
        """`u` is metres per site unit."""
        self.u, self.speed, self.fixture_s = u, speed_m_s, fixture_s
        # source: (time, [(x, y, radius, is a person, anchor x, anchor y, at the anchor since)])
        self.pictures: dict[str, tuple[float, list[tuple[float, float, float, bool, float, float, float]]]] = {}
        self.heard: dict[str, float] = {}                # source: when its last picture arrived

    def put(self, source: str, t: float, things: Sequence[tuple[float, float, float, bool]], now: float) -> None:
        """A source's new picture, taken at `t`: (x, y, radius, is a person) for each thing, in site units."""
        self.heard[source] = now
        before = self.pictures.get(source)
        still = STILL_M / self.u
        out = []
        for x, y, r, person in things:
            ax, ay, since = x, y, (t if before is not None else -math.inf)
            # The nearest thing in the previous picture, if close enough, is this thing a moment ago.
            was = min(before[1], key=lambda o: math.hypot(o[0] - x, o[1] - y), default=None) if before else None
            if (was is not None and math.hypot(was[0] - x, was[1] - y) <= still
                    and math.hypot(was[4] - x, was[5] - y) <= still):
                ax, ay, since = was[4], was[5], was[6]            # still where it was first seen
            out.append((x, y, max(r, SEEN_FLOOR_M / self.u), person, ax, ay, since))
        self.pictures[source] = (t, out)

    def now(self, now: float, ahead_s: float = 0.0) -> list[Sighting]:
        """Every thing seen, as circles grown by the walking possible since its picture was taken and until
        `ahead_s` from now."""
        out = []
        for source in [s for s, at in self.heard.items() if now - at > SEEN_KEEP_S]:
            del self.heard[source], self.pictures[source]
        for source, (t, things) in self.pictures.items():
            grow = self.speed * (max(now - t, 0.0) + ahead_s) / self.u
            out += [Sighting(x, y, r + grow, person or now - since <= self.fixture_s, source)
                    for x, y, r, person, _, _, since in things]
        return out


def account(people: dict[str, Circle], seen: Sequence[Sighting]) -> tuple[dict[str, Circle], list[Circle]]:
    """Put people located by what they carry together with things seen, on a site where everyone on foot carries
    a listed phone or tracker. Returns the people to report, each as a circle, and the sightings to report as
    people nobody could name.

    Two sources may see the same person, so sightings from different sources whose circles touch are first taken
    as one *thing* (which can only undercount what is seen, never overcount). A thing is a *candidate* for a
    person if one of its sightings lies in that person's circle and counts (see `Sighting`). Then:

    * people are matched to candidates, one each. Where everyone in a group of people sharing candidates finds
      one, they are all accounted for: each is among the things seen. A person with one candidate is that
      thing, and takes its tightest circle if that is the smaller. A person with several is not reported at all,
      because every one of the candidates is reported anyway.
    * where the matching leaves someone out, more people are known to be there than things are seen, and nobody
      can say who is the one not seen: everyone sharing candidates with the one left out keeps their own circle.
    * a person with no candidate keeps their own circle.

    Every sighting that did not become a named person is returned as a circle of its own.

    Why this is sound on such a site: each thing seen is one of the listed people, and lies in that person's own
    circle, so that person is among those sharing it. A group with someone hidden therefore has fewer things than
    people, and cannot be matched in full. What it rests on is the site's rule (an unlisted visitor who is seen
    can stand in for a listed person who is not) and on each person's circle really holding them.
    """
    # Things: sightings joined across sources where their circles touch.
    thing = list(range(len(seen)))

    def root(k: int) -> int:
        """The thing a sighting belongs to (union-find, with path halving)."""
        while thing[k] != k:
            thing[k] = thing[thing[k]]
            k = thing[k]
        return k

    for i, a in enumerate(seen):
        for j in range(i + 1, len(seen)):
            b = seen[j]
            if a.source != b.source and math.hypot(a.x - b.x, a.y - b.y) <= a.radius + b.radius:
                thing[root(i)] = root(j)
    cand = {p: sorted({root(k) for k, s in enumerate(seen)
                       if s.counts and math.hypot(s.x - x, s.y - y) <= r + s.radius})
            for p, (x, y, r) in people.items()}
    owner: dict[int, str] = {}                           # thing: the person matched to it

    def place(p: str, tried: set[int]) -> bool:
        """Find `p` a thing, moving people already matched to another of theirs if need be (Kuhn's augmenting
        path for bipartite matching)."""
        for k in cand[p]:
            if k in tried:
                continue
            tried.add(k)
            if k not in owner or place(owner[k], tried):
                owner[k] = p
                return True
        return False

    unseen = [p for p in people if cand[p] and not place(p, set())]
    # Everyone who shares a candidate, directly or through others, with a person left out is in doubt too.
    doubt, todo = set(unseen), list(unseen)
    while todo:
        mine = set(cand[todo.pop()])
        for q in people:
            if q not in doubt and mine & set(cand[q]):
                doubt.add(q)
                todo.append(q)
    out: dict[str, Circle] = {}
    pinned: set[int] = set()
    for p, own in people.items():
        if not cand[p] or p in doubt:
            out[p] = own
        elif len(cand[p]) == 1:
            best = min((s for k, s in enumerate(seen) if root(k) == cand[p][0]), key=lambda s: s.radius)
            out[p] = (best.x, best.y, best.radius) if best.radius < own[2] else own
            pinned.add(cand[p][0])
    return out, [(s.x, s.y, s.radius) for k, s in enumerate(seen) if root(k) not in pinned]


def wifi_range(rssi: float, rssi_1m: float = -40.0, exponent: float = 2.7, sigma_db: float = 6.0,
               k: float = K_SIGMA) -> float:
    """The distance in metres within which a phone heard at `rssi` dBm almost certainly is.

    The log-distance model: signal = `rssi_1m` - 10 * `exponent` * log10(distance) + fading, where the fading is
    random with standard deviation `sigma_db`. The answer is the distance at which the signal would still be `rssi`
    with `k` standard deviations of fading in its favour.
    """
    return float(10 ** ((rssi_1m - rssi + k * sigma_db) / (10 * exponent)))


def fit_path_loss(samples: Sequence[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit the Wi-Fi model to a walk: `samples` are (distance in metres, signal in dBm) pairs.

    Returns (rssi_1m, exponent, sigma_db): a straight line of signal against log10(distance), and the scatter round
    it. Needs at least 5 samples over a range of distances.
    """
    pts = [(math.log10(d), r) for d, r in samples if d > 0]
    if len(pts) < 5 or max(p[0] for p in pts) - min(p[0] for p in pts) < 0.3:
        raise ValueError("fitting the Wi-Fi model needs at least 5 samples, the farthest twice as far as the nearest")
    n = len(pts)
    mx, my = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    slope = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
    rssi_1m = my - slope * mx
    sigma = math.sqrt(sum((p[1] - (rssi_1m + slope * p[0])) ** 2 for p in pts) / max(n - 2, 1))
    return rssi_1m, -slope / 10.0, sigma


def smallest_circle(circles: Sequence[Circle]) -> Circle:
    """A circle that contains everything common to all of `circles`, as small as this simple method finds.

    Each circle says "the person is in here". The person is then in the smallest of them, and, where two overlap
    like a lens, in the circle whose diameter is the lens's chord. The smallest of all those candidates is returned.
    Circles that do not overlap at all contradict each other; then the smallest single circle is the answer, since
    there is no telling which is wrong.
    """
    best = min(circles, key=lambda c: c[2])
    for i, (x1, y1, r1) in enumerate(circles):
        for x2, y2, r2 in circles[i + 1:]:
            d = math.hypot(x2 - x1, y2 - y1)
            if d >= r1 + r2 or d <= abs(r1 - r2) or d == 0.0:
                continue                                 # apart (a contradiction), or one inside the other
            a = (d * d + r1 * r1 - r2 * r2) / (2 * d)    # from the first centre to the chord, along the line of centres
            if not 0.0 <= a <= d:
                continue                                 # the chord is not between the centres: no gain
            h = math.sqrt(max(r1 * r1 - a * a, 0.0))     # half the chord
            if h < best[2]:
                best = (x1 + a * (x2 - x1) / d, y1 + a * (y2 - y1) / d, h)
    return best


class Tags:
    """The people named in a tags file, where each was last located by each source, and what the precise sensors
    see."""

    def __init__(self, cfg: dict, frame: Any, u: float, extent: tuple[float, float, float, float], site: str = ""):
        """`cfg` is the tags file (see docs/FIELD.md). `frame` places the site on the earth, for the sources that
        report latitude and longitude. `u` is metres per site unit; `extent` is the site's bounding box (x0, y0,
        x1, y1) in site units. `site` is the site's name, needed when an access point rides on a machine.
        """
        self.frame, self.u, self.extent, self.site = frame, u, extent, site
        self.speed = float(cfg.get("speed_m_s", WALK_M_S))
        wifi = cfg.get("wifi") or {}
        self.model = (float(wifi.get("rssi_1m", -40.0)), float(wifi.get("exponent", 2.7)),
                      float(wifi.get("sigma_db", 6.0)))
        self.wifi_topic = str(wifi.get("topic", "wifi/+"))
        self.learn = bool(wifi.get("learn", True))
        self.given = self.model                          # the model as the tags file gave it
        self.taught: tuple[float, float] | None = None   # nearest and farthest distance the learnt model rests on
        self.samples: deque[tuple[float, float]] = deque(maxlen=LEARN_KEEP)       # (distance m, signal dBm)
        self.learned = 0                                 # samples taken in all, to know when to refit
        sight = cfg.get("sightings")
        self.sight_topic = str((sight or {}).get("topic", "sightings/+"))
        # Sources the site relies on, and how long each may be silent before nobody is watching.
        self.watch = {str(k): float(v) for k, v in ((sight or {}).get("watch") or {}).items()}
        self.seen = Seen(u, self.speed, float((sight or {}).get("fixture_s", FIXTURE_S)))
        # The site's rule that everyone on foot carries a listed phone or tracker. Only then may a sighting stand
        # in for a person's own circle (see `account`).
        self.everyone_tagged = bool((sight or {}).get("everyone_tagged", False))
        # Places where a person-sized shape is known to be a thing (a post, a tank): shapes there are not reported.
        self.fixtures = [(float(f["x"]), float(f["y"]), float(f.get("r") or STILL_M / u))
                         for f in (sight or {}).get("fixtures") or []]
        self.started: float | None = None                # when the first message of any kind was fed
        self.owntracks = str(cfg.get("owntracks_topic", "owntracks")).rstrip("/")
        self.mesh = str(cfg.get("meshtastic_topic", "msh")).rstrip("/")
        self.aps: dict[str, tuple[float, float]] = {}        # where each fixed access point stands
        self.riding: dict[str, str] = {}                     # access point: the machine that carries it
        self.poses: dict[str, tuple[float, float, float]] = {}   # machine: (time heard, x, y)
        self.by_bssid: dict[str, str] = {}
        for name, ap in (wifi.get("aps") or {}).items():
            if ap.get("machine"):
                self.riding[name] = str(ap["machine"])
            else:
                self.aps[name] = (float(ap["x"]), float(ap["y"]))
            if ap.get("bssid"):
                self.by_bssid[str(ap["bssid"]).lower()] = name
        # Who carries what: each lookup leads from a source's own name for the device to the person.
        self.by_phone: dict[str, str] = {}
        self.by_mac: dict[str, str] = {}
        self.by_node: dict[int, str] = {}
        for person, ids in (cfg.get("people") or {}).items():
            if ids.get("owntracks"):
                self.by_phone[str(ids["owntracks"]).strip("/")] = person
            if ids.get("wifi"):
                self.by_mac[str(ids["wifi"]).lower()] = person
            if ids.get("meshtastic") is not None:
                node = ids["meshtastic"]
                self.by_node[int(str(node).lstrip("!"), 16) if isinstance(node, str) else int(node)] = person
        if not (self.by_phone or self.by_mac or self.by_node) and sight is None:
            raise ValueError("the tags file names no person with an owntracks, wifi or meshtastic id, and no "
                             "sightings")
        if self.by_mac and not (self.aps or self.riding):
            raise ValueError("people have wifi ids, but the tags file places no access points (wifi.aps)")
        if (self.by_phone or self.by_node) and getattr(frame, "lat0", None) is None:
            raise ValueError("phones and trackers report latitude and longitude: say where the site is (--origin)")
        self.fixes: dict[str, dict[str, tuple[float, float, float, float]]] = {}   # person: source: (t, x, y, r)
        self.fresh = False                               # a fix has arrived since the caller last reset this
        self.heard: dict[str, dict[str, tuple[float, float]]] = {}                 # mac: access point: (t, range m)

    def topics(self) -> list[str]:
        """The MQTT topics to subscribe to for the sources the tags file uses."""
        out = []
        if self.by_phone:
            out.append(f"{self.owntracks}/+/+")
        if self.by_mac:
            out.append(self.wifi_topic)
        if self.by_node:
            out.append(f"{self.mesh}/#")
        if self.riding:
            out.append(f"loam/v0/{self.site}/+/state")
        out.append(self.sight_topic)
        return out

    def names(self) -> list[str]:
        """Every person in the tags file."""
        return sorted({*self.by_phone.values(), *self.by_mac.values(), *self.by_node.values()})

    def _fix(self, person: str, source: str, t: float, x: float, y: float, r_m: float) -> None:
        """Remember a fix: time received, site position, radius in site units."""
        self.fixes.setdefault(person, {})[source] = (t, x, y, r_m / self.u)
        self.fresh = True

    def _latlon(self, person: str, source: str, lat: float, lon: float, r_m: float, t: float) -> None:
        """Remember a fix given as latitude and longitude."""
        x, y = self.frame.wgs84_to_local(lat, lon)
        self._fix(person, source, t, float(x), float(y), max(r_m, GPS_FLOOR_M))

    def feed(self, topic: str, msg: Any, now: float) -> bool:
        """Take in one message. Returns whether it was one of ours."""
        if not isinstance(msg, dict):
            return False
        if topic.startswith(self.owntracks + "/"):
            person = self.by_phone.get(topic[len(self.owntracks) + 1:])
            if person is None or msg.get("_type") != "location":
                return person is not None
            # The fix is as old as the phone says (it may have been queued while out of coverage), never newer
            # than now.
            t = min(now, float(msg.get("tst", now)))
            self._latlon(person, "owntracks", float(msg["lat"]), float(msg["lon"]), float(msg.get("acc") or 0.0), t)
            ap = self.by_bssid.get(str(msg.get("BSSID", "")).lower())
            at = self._where(ap, now) if ap is not None else None
            if at is not None:                           # the phone also says which access point it is on
                self._fix(person, "bssid", t, *at, CELL_M)
            return True
        if topic.startswith(self.mesh + "/"):
            person = self.by_node.get(int(msg.get("from", -1))) if str(msg.get("from", "")).lstrip("-").isdigit() \
                else None
            p = msg.get("payload")
            if person is None or msg.get("type") != "position" or not isinstance(p, dict) or "latitude_i" not in p:
                return person is not None
            bits = int(p.get("precision_bits") or 32)    # a channel may blur positions to a grid
            cell = MESH_CELL_M / 2 ** bits if bits < 32 else 0.0
            t = min(now, float(p.get("time") or now))
            self._latlon(person, "meshtastic", p["latitude_i"] * 1e-7, p["longitude_i"] * 1e-7, cell, t)
            return True
        if self.riding and topic.startswith(f"loam/v0/{self.site}/") and topic.endswith("/state"):
            pose = msg.get("pose")
            if isinstance(pose, dict) and msg.get("machine") in self.riding.values():
                self.poses[msg["machine"]] = (now, float(pose["x"]), float(pose["y"]))
            return True
        # Both of these topics end in the name of the sender: "wifi/+" is heard as "wifi/barn".
        head = self.wifi_topic.split("+")[0]
        ap = topic[len(head):]
        if topic.startswith(head) and (ap in self.aps or ap in self.riding):
            self._wifi(ap, msg, now)
            return True
        head = self.sight_topic.split("+")[0]
        if topic.startswith(head) and topic[len(head):] and "/" not in topic[len(head):]:
            self._sightings(topic[len(head):], msg, now)
            return True
        return False

    def _sightings(self, source: str, msg: dict, now: float) -> None:
        """One source's picture of the site: `{"t": seconds, "people": [...], "objects": [...]}`. Each entry is
        `{"x", "y", "r"}` in site units or `{"lat", "lon", "acc"}` with the accuracy in metres. "people" are what a
        detector called a person; "objects" are person-sized shapes."""
        things = []
        for key, person in (("people", True), ("objects", False)):
            for o in msg.get(key) or []:
                if "lat" in o:
                    if getattr(self.frame, "lat0", None) is None:
                        raise ValueError("a sighting in latitude and longitude: say where the site is (--origin)")
                    x, y = (float(v) for v in self.frame.wgs84_to_local(float(o["lat"]), float(o["lon"])))
                    r = float(o.get("acc") or 0.0) / self.u
                else:
                    x, y, r = float(o["x"]), float(o["y"]), float(o.get("r") or 0.0)
                known = not person and any(math.hypot(x - fx, y - fy) <= fr for fx, fy, fr in self.fixtures)
                if math.isfinite(x + y + r) and not known:
                    things.append((x, y, r, person))
        self.seen.put(source, min(now, float(msg.get("t") or now)), things, now)
        self.fresh = True

    def silent(self, now: float) -> list[str]:
        """The sources the site relies on (`sightings.watch`) that have sent no picture for longer than they may:
        nobody is watching there."""
        if self.started is None:
            self.started = now
        return sorted(s for s, limit in self.watch.items() if now - self.seen.heard.get(s, self.started) > limit)

    def _learn(self, person: str, at: tuple[float, float], rssi: float, now: float) -> None:
        """Use one signal reading as a sample for the Wi-Fi model, if the person's place is known well enough from
        another source, and refit the model when enough new samples have come in."""
        fixes = [(x, y, (r + self.speed * (now - t) / self.u) * self.u) for src, (t, x, y, r)
                 in self.fixes.get(person, {}).items() if src not in ("wifi", "bssid")]
        if not fixes:
            return
        x, y, r_m = min(fixes, key=lambda f: f[2])
        d = math.hypot(x - at[0], y - at[1]) * self.u
        if r_m > LEARN_M or d < 2 * r_m:                 # not known closely enough for this distance
            return
        self.samples.append((d, rssi))
        self.learned += 1
        if len(self.samples) >= LEARN_MIN and self.learned % (LEARN_MIN // 2) == 0:
            try:
                rssi_1m, exponent, sigma = fit_path_loss(self.samples)
            except ValueError:
                return                                   # all at one distance so far
            if EXPONENT_RANGE[0] <= exponent <= EXPONENT_RANGE[1]:
                self.model = (rssi_1m, exponent, max(sigma, SIGMA_FLOOR_DB))
                self.taught = (min(s[0] for s in self.samples), max(s[0] for s in self.samples))

    def _range(self, rssi: float) -> float:
        """The distance within which a phone heard at `rssi` almost certainly is, by the model in use.

        A learnt model is a line through samples taken between two distances, and says nothing sure outside them.
        Nearer than the nearest sample the answer is therefore never less than that nearest distance. A signal so
        weak that the phone is most likely beyond the farthest sample gets the larger of the learnt model's
        answer and that of the tags file's own model.
        """
        r = wifi_range(rssi, *self.model)
        if self.taught is not None:
            r = max(r, self.taught[0])
            if wifi_range(rssi, *self.model[:2], sigma_db=0.0) > self.taught[1]:
                r = max(r, wifi_range(rssi, *self.given))
        return r

    def fused(self, now: float, ahead_s: float = 0.0) -> tuple[dict[str, Circle], list[Circle]]:
        """People and sightings put together: the people to report, by name, and the things seen that are nobody
        in particular. Where everyone on the site is tagged they are matched (see `account`), and a person pinned
        to one sighting is remembered there, as a fix of its own. Otherwise both are reported as they are.
        """
        seen = self.seen.now(now, ahead_s)
        own = self.located(now, ahead_s)
        if not self.everyone_tagged:
            return own, [(s.x, s.y, s.radius) for s in seen]
        people, others = account(own, seen)
        grow = self.speed * ahead_s / self.u
        for p, c in people.items():
            if c != own[p]:                              # pinned: keep it as a fix, without the look-ahead
                self.fixes[p]["seen"] = (now, c[0], c[1], max(c[2] - grow, 0.0))
        return people, others

    def _where(self, ap: str, now: float) -> tuple[float, float] | None:
        """Where an access point is now: where it stands, or where the machine carrying it last said it was. None
        if that machine has not been heard from lately, since then the access point could be anywhere.
        """
        if ap in self.aps:
            return self.aps[ap]
        t, x, y = self.poses.get(self.riding.get(ap, ""), (-math.inf, 0.0, 0.0))
        return (x, y) if now - t <= WIFI_FRESH_S else None

    def _wifi(self, ap: str, msg: dict, now: float) -> None:
        """One access point's report: every client it hears (hostapd's `clients` table), or a single
        `{"mac", "rssi"}` or `{"mac", "range_m"}` observation.
        """
        seen: dict[str, float] = {}
        signal: dict[str, float] = {}
        clients = msg.get("clients")
        if isinstance(clients, dict):
            for mac, c in clients.items():
                if isinstance(c, dict) and isinstance(c.get("signal"), (int, float)):
                    signal[mac.lower()] = float(c["signal"])
            for mac, by_ap in self.heard.items():        # a full list: who is not on it is no longer heard here
                if mac not in seen:
                    by_ap.pop(ap, None)
        elif msg.get("mac"):
            if isinstance(msg.get("range_m"), (int, float)):                       # a measured range (Wi-Fi RTT)
                seen[str(msg["mac"]).lower()] = float(msg["range_m"]) + K_SIGMA * float(msg.get("std_m", 1.0))
            elif isinstance(msg.get("rssi"), (int, float)):
                signal[str(msg["mac"]).lower()] = float(msg["rssi"])
        here = self._where(ap, now)
        for mac, rssi in signal.items():
            if self.learn and here is not None and mac in self.by_mac:
                self._learn(self.by_mac[mac], here, rssi, now)
            seen[mac] = self._range(rssi)
        for mac, r in seen.items():
            person = self.by_mac.get(mac)
            if person is None:
                continue
            by_ap = self.heard.setdefault(mac, {})
            by_ap[ap] = (now, r)
            circles = [(*at, r_m / self.u) for a, (t, r_m) in by_ap.items()
                       if now - t <= WIFI_FRESH_S and (at := self._where(a, now)) is not None]
            if circles:
                x, y, rad = smallest_circle(circles)
                self._fix(person, "wifi", now, x, y, rad * self.u)

    def located(self, now: float, ahead_s: float = 0.0) -> dict[str, Circle]:
        """Where each person is, as far as anyone knows: {person: (x, y, radius)} in site units.

        Each source's last fix is grown by how far a person can have walked since, and until `ahead_s` from now
        (the caller's next report); the tightest is taken. A person never located is not listed, and neither is
        one last located well off the site.
        """
        x0, y0, x1, y1 = self.extent
        out: dict[str, Circle] = {}
        for person, by_source in self.fixes.items():
            pin = by_source.get("seen")
            if pin is not None:
                # A person pinned to a sighting is believed only while what they carry does not say otherwise: a
                # newer fix from the device whose circle the pinned place cannot be in ends it.
                reach = pin[3] + self.speed * (now - pin[0]) / self.u
                if any(t > pin[0] and math.hypot(x - pin[1], y - pin[2]) > r + self.speed * (now - t) / self.u + reach
                       for src, (t, x, y, r) in by_source.items() if src != "seen"):
                    del by_source["seen"]
            grown = [(x, y, r + self.speed * (now - t + ahead_s) / self.u) for t, x, y, r in by_source.values()]
            # The latest word on where the person was: if the whole of that circle was well off the site, they
            # have left, and are not followed further.
            _, lx, ly, lr = max(by_source.values(), key=lambda f: f[0])
            off = (math.hypot(max(x0 - lx, 0.0, lx - x1), max(y0 - ly, 0.0, ly - y1)) - lr) * self.u
            if off > GONE_M:
                continue
            out[person] = min(grown, key=lambda c: c[2])
        return out
