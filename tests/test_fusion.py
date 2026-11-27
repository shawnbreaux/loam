"""People's phones put together with what lidar and cameras see (`loam.tags.account` and what feeds it): the head
count, cameras calibrated to the ground, person-sized things in a scan, and the Wi-Fi model learnt on the site.
"""
import json
import math
import os
import threading
import time

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES, wire_valid

import loam.sensors as sensors
from loam import wire
from loam.cli import load, main
from loam.geo import Frame
from loam.grid import SiteGrid
from loam.pointcloud import grid_cloud, standing_objects, write_las
from loam.runtime.bus import Bus
from loam.sensors import Area, Camera, Ground, People, load_areas, run_sense
from loam.tags import FIXTURE_S, SEEN_KEEP_S, Seen, Sighting, Tags, account, wifi_range

FT = 0.3048
ORIGIN = (34.5712, -85.6790)
MAC = "a4:83:e7:12:34:56"


def see(x, y, r=0.5, counts=True, source="cam"):
    """A sighting."""
    return Sighting(x, y, r, counts, source)


def test_one_person_one_sighting_is_that_person():
    """A sighting inside a person's circle, and nobody else it could be: the person is there, to the sensor's
    precision. Outside the circle it is someone else, and the person stays where the phone says."""
    sam = (50.0, 50.0, 40.0)
    assert account({"sam": sam}, [see(60, 45)]) == ({"sam": (60, 45, 0.5)}, [])
    assert account({"sam": sam}, [see(150, 45)]) == ({"sam": sam}, [(150, 45, 0.5)])
    assert account({"sam": sam}, []) == ({"sam": sam}, [])
    assert account({}, [see(1, 2)]) == ({}, [(1, 2, 0.5)])
    # A sighting vaguer than the phone does not replace it, and is not reported twice.
    assert account({"sam": (50.0, 50.0, 3.0)}, [see(51, 50, r=8.0)]) == ({"sam": (50.0, 50.0, 3.0)}, [])


def test_more_phones_than_people_seen_keeps_everyone_wide():
    """Two phones where one person is seen: one of the two is not seen, and nobody can say which. Both keep their
    own circles and the sighting stands by itself."""
    sam, ana = (50.0, 50.0, 40.0), (60.0, 50.0, 40.0)
    assert account({"sam": sam, "ana": ana}, [see(55, 50)]) == ({"sam": sam, "ana": ana}, [(55, 50, 0.5)])
    # The doubt spreads through shared candidates: lee could be either sighting, and one of sam and ana's is
    # therefore not certain either.
    lee = (150.0, 50.0, 100.0)
    people = {"sam": (50.0, 50.0, 10.0), "ana": (250.0, 50.0, 10.0), "lee": lee}
    who, others = account(people, [see(50, 52), see(250, 52)])
    assert who == people and len(others) == 2


def test_several_candidates_account_for_a_person_without_naming_them():
    """A person whose circle holds two sightings, with enough sightings to go round, is one of them: no wide
    circle, and no name on either. The person with only one candidate is named."""
    people = {"sam": (50.0, 50.0, 5.0), "ana": (60.0, 50.0, 60.0)}
    who, others = account(people, [see(51, 50), see(100, 50)])
    assert who == {"sam": (51, 50, 0.5)} and others == [(100, 50, 0.5)]


def test_shapes_that_have_not_moved_account_for_nobody():
    """A post inside a person's circle is still a place to keep clear of, and does not stand in for the person."""
    sam = (50.0, 50.0, 40.0)
    assert account({"sam": sam}, [see(60, 45, counts=False, source="mast")]) == ({"sam": sam}, [(60, 45, 0.5)])


def test_one_person_seen_by_two_sensors_is_counted_once():
    """A camera and a lidar that see the same person give two sightings and one thing. Counted twice, they would
    account for a second phone whose owner nobody sees."""
    sam, ana = (50.0, 50.0, 40.0), (55.0, 50.0, 40.0)
    both = [see(60, 45, r=0.8, source="cam"), see(60.4, 45.2, r=0.5, source="mast")]
    who, others = account({"sam": sam, "ana": ana}, both)
    assert who == {"sam": sam, "ana": ana} and len(others) == 2
    # Alone, the person takes the tighter of the two, and neither is reported again.
    assert account({"sam": sam}, both) == ({"sam": (60.4, 45.2, 0.5)}, [])
    # The same source seeing two things close together sees two.
    two = [see(60, 45, source="mast"), see(60.6, 45, source="mast")]
    assert account({"sam": sam, "ana": ana}, two) == ({}, [(60, 45, 0.5), (60.6, 45, 0.5)])


def test_nobody_is_ever_left_without_a_circle():
    """The property the head count must have, on a site where everyone is tagged: whoever is seen or hidden,
    every person's true position is inside a circle that is reported. Tried on thousands of random sites with
    hidden people, people seen by two sensors, and posts."""
    rng = np.random.default_rng(11)
    pinned = wide = 0
    for _ in range(3000):
        n = int(rng.integers(1, 7))
        truth = rng.uniform(0, 120, (n, 2))
        people, seen = {}, []
        for k, (x, y) in enumerate(truth):
            r = float(rng.uniform(4, 90))
            ang, off = rng.uniform(0, 2 * math.pi), r * math.sqrt(rng.uniform(0, 0.9))
            people[f"p{k}"] = (x + off * math.cos(ang), y + off * math.sin(ang), r)
            for source, p in (("cam", 0.5), ("mast", 0.4)):
                if rng.random() < p:
                    dx, dy = rng.uniform(-0.3, 0.3, 2)
                    seen.append(see(x + dx, y + dy, 0.5, True, source))
        seen += [see(*rng.uniform(0, 120, 2), 0.5, False, "mast") for _ in range(int(rng.integers(0, 4)))]
        order = rng.permutation(len(seen))
        seen = [seen[i] for i in order]
        who, others = account(people, seen)
        for k, (x, y) in enumerate(truth):
            circles = ([who[f"p{k}"]] if f"p{k}" in who else []) + others
            assert any(math.hypot(x - cx, y - cy) <= cr + 1e-9 for cx, cy, cr in circles)
        pinned += sum(1 for p, c in who.items() if c != people[p])
        wide += sum(1 for p, c in who.items() if c == people[p])
    print(f"\n3000 random sites: {pinned} people pinned to a sighting, {wide} left with their own circle")
    assert pinned > 1500 and wide > 1500


def test_what_is_seen_ages_moves_and_is_forgotten():
    """A source's newer picture replaces its older one. What was seen grows with the time since. A shape counts as
    a possible person only after it has moved, and stops counting when it has stood still too long; a detected
    person always counts. A source that sends nothing more is forgotten."""
    s = Seen(1.0)
    t0 = 1_800_000_000.0
    s.put("mast", t0, [(10.0, 10.0, 0.2, False), (40.0, 10.0, 0.5, False)], t0)
    first = s.now(t0)
    assert [(a.x, a.radius, a.counts, a.source) for a in first] == [(10.0, 0.5, False, "mast"),
                                                                    (40.0, 0.5, False, "mast")]
    assert s.now(t0 + 2, ahead_s=1.0)[0].radius == pytest.approx(0.5 + 2.5 * 3.0)     # 2 s old, 1 s ahead
    # The second thing walks off in small steps; the first stays. Only the walker counts.
    for k in range(1, 6):
        s.put("mast", t0 + k, [(10.0, 10.1, 0.5, False), (40.0 + 0.6 * k, 10.0, 0.5, False)], t0 + k)
    post, walker = s.now(t0 + 5)
    assert not post.counts and walker.counts and walker.x == pytest.approx(43.0)
    # It stops, and after standing still for the limit it is taken for a thing again.
    s.put("mast", t0 + 6, [(10.0, 10.0, 0.5, False), (43.0, 10.0, 0.5, False)], t0 + 6)
    late = t0 + FIXTURE_S + 20
    s.put("mast", late, [(10.0, 10.0, 0.5, False), (43.0, 10.0, 0.5, False)], late)
    assert [a.counts for a in s.now(late)] == [False, False]
    # A thing that appears where nothing was has moved there; a person a detector names always counts.
    s.put("mast", late + 1, [(10.0, 10.0, 0.5, False), (70.0, 40.0, 0.5, False)], late + 1)
    s.put("cam", late + 1, [(20.0, 20.0, 0.5, True)], late + 1)
    assert [(a.x, a.counts) for a in s.now(late + 1)] == [(10.0, False), (70.0, True), (20.0, True)]
    s.put("mast", late + 2, [], late + 2)                              # the lidar sees nothing now
    assert [a.source for a in s.now(late + 2)] == ["cam"]
    assert s.now(late + 2 + SEEN_KEEP_S) == [] and s.pictures == {"mast": (late + 2, [])}


# -- cameras calibrated to the ground ---------------------------------------------------------------------------
CAM_AT = np.array([80.0, -30.0, 25.0])                           # a camera on a pole south of the pad, in feet


def pixel(x, y, z=0.0):
    """Where a point of the site appears in the camera's 1280 x 720 picture: a pinhole camera at `CAM_AT`,
    looking north and 15 degrees down, so the far edge of the pad and some sky above it are in the picture."""
    tilt = math.radians(15)
    fwd = np.array([0.0, math.cos(tilt), -math.sin(tilt)])
    right = np.array([1.0, 0.0, 0.0])
    down = np.cross(fwd, right)
    p = np.array([x, y, z]) - CAM_AT
    return 640 + 900 * (p @ right) / (p @ fwd), 360 + 900 * (p @ down) / (p @ fwd)


def box(x, y, tall=5.8, wide=1.6):
    """Frigate's box round a person standing at (x, y): from the feet to the head, as wide as the shoulders."""
    (ul, vf), (ur, _), (_, vh) = pixel(x - wide / 2, y), pixel(x + wide / 2, y), pixel(x, y, tall)
    return [round(ul), round(vh), round(ur), round(vf)]


MARKS = [(20.0, 10.0), (140.0, 10.0), (150.0, 90.0), (10.0, 90.0), (80.0, 50.0)]     # surveyed ground marks
GROUND = [[*pixel(x, y), x, y] for x, y in MARKS]


def test_a_calibrated_camera_places_a_person_on_the_ground():
    """Five ground marks seen in the picture fix the mapping. A person's box then gives where they stand to about
    a foot near the camera and a few feet across the pad, with a radius that says so; a box above the horizon
    gives nothing."""
    g = Ground(GROUND)
    for x, y in ((35.0, 20.0), (120.0, 75.0), (80.0, 5.0)):
        assert g.at(*pixel(x, y)) == pytest.approx((x, y), abs=1e-6)
        a = g.person(box(x, y))
        err = math.hypot(a.x - x, a.y - y)
        assert err <= a.radius and 0.7 < a.radius < 6 and err < 0.02 * (y + 40)       # worse farther away
    assert g.person(box(120.0, 75.0)).radius > g.person(box(80.0, 5.0)).radius
    assert g.at(640, 10) is None and g.person([600, 0, 660, 20]) is None              # sky
    with pytest.raises(ValueError, match="four or more"):
        Ground(GROUND[:3])
    with pytest.raises(ValueError, match="one line"):
        Ground([[*pixel(x, 40.0), x, 40.0] for x in (10.0, 50.0, 90.0, 130.0)])
    bent = [list(p) for p in GROUND]
    bent[4][2] += 15.0                                                               # one mark surveyed wrong
    with pytest.raises(ValueError, match="do not fit"):
        Ground(bent)
    # A pixel above the horizon, paired with where the mapping would put it (behind the camera): it fits the
    # arithmetic and is no place on the ground.
    x, y, w = g.h @ (640.0, 60.0, 1.0)
    with pytest.raises(ValueError, match="in front of the camera"):
        Ground([*GROUND, [640.0, 60.0, x / w, y / w]])


def view(*boxes, detect=True):
    """One camera's part of Frigate's `camera_activity`: a person for each box."""
    return {"motion": bool(boxes), "config": {"detect": detect, "enabled": True},
            "objects": [{"id": f"o{i}", "label": "person", "box": b, "current_zones": []} for i, b in enumerate(boxes)]}


def test_people_are_placed_only_when_someone_reads_the_places(tmp_path):
    """A calibrated camera's people are sightings once placing is switched on; until then, and for a person it
    cannot place, they are reported as before, somewhere in the camera's area. A camera that is not watching sees
    nothing and its area is unknown."""
    (tmp_path / "areas.json").write_text(json.dumps({"cameras": {
        "pad": {"x": 80, "y": 50, "radius": 95, "ground": GROUND}, "gate": {"x": 200, "y": 0, "radius": 30}}}))
    cams = load_areas(str(tmp_path / "areas.json"))
    assert cams["pad"].ground is not None and cams["gate"].ground is None
    p = People(cams)
    p.activity = {"pad": view(box(35.0, 20.0), [600, 0, 660, 20]), "gate": view([10, 10, 50, 90])}
    whole = {"pad": ("person", cams["pad"].area), "gate": ("person", cams["gate"].area)}
    assert p.detections() == whole and p.sightings() == {"pad": []}
    p.placing = True
    (a,) = p.sightings()["pad"]
    assert math.hypot(a.x - 35, a.y - 20) < 1.0
    assert p.detections() == whole                       # the one in the sky is still "somewhere in view"
    p.activity["pad"] = view(box(35.0, 20.0))
    assert p.detections() == {"gate": ("person", cams["gate"].area)}
    p.activity["pad"] = view(box(35.0, 20.0), detect=False)
    assert p.sightings() == {} and p.detections()["pad"] == ("unknown", cams["pad"].area)
    (tmp_path / "bad.json").write_text(json.dumps({"cameras": {"pad": {"x": 0, "y": 0, "radius": 9,
                                                                       "ground": GROUND[:2]}}}))
    with pytest.raises(ValueError, match="four or more"):
        load_areas(str(tmp_path / "bad.json"))


# -- the tags file's sightings ----------------------------------------------------------------------------------
def tagged(**sightings):
    """Sam, known only by Wi-Fi at the barn's access point, on a 400 ft site."""
    cfg = {"people": {"sam": {"wifi": MAC}}, "wifi": {"aps": {"barn": {"x": 0.0, "y": 0.0}}, "learn": False},
           "sightings": sightings}
    return Tags(cfg, Frame(FT, 0.0, *ORIGIN), FT, (0.0, 0.0, 400.0, 400.0))


def test_a_sighting_narrows_a_phone_only_where_everyone_is_tagged():
    """The same phone and the same sighting, with and without the site's rule. Without it both are reported. With
    it the person is the sighting, stays pinned while seen, and goes back to the phone's wide circle when the
    phone is heard somewhere the pinned place cannot be."""
    now = 1_800_000_000.0
    heard = {"clients": {MAC: {"signal": -62}}}
    spot = {"t": now, "people": [{"x": 60.0, "y": 30.0, "r": 1.0}]}
    wide = wifi_range(-62) / FT
    t = tagged()
    assert t.feed("wifi/barn", heard, now) and t.feed("sightings/drone", spot, now)
    assert t.fused(now) == ({"sam": (0.0, 0.0, pytest.approx(wide))}, [(60.0, 30.0, pytest.approx(0.5 / FT))])
    t = tagged(everyone_tagged=True)
    t.feed("wifi/barn", heard, now)
    assert t.fused(now) == ({"sam": (0.0, 0.0, pytest.approx(wide))}, [])            # heard, and nobody seen
    t.feed("sightings/drone", spot, now)
    assert t.fused(now) == ({"sam": (60.0, 30.0, pytest.approx(0.5 / FT))}, [])
    assert t.fixes["sam"]["seen"][1:3] == (60.0, 30.0)
    # The drone moves on (an empty picture). Sam is remembered where last seen, growing at walking pace.
    t.feed("sightings/drone", {"t": now + 4, "people": []}, now + 4)
    (x, y, r), = t.fused(now + 4)[0].values()
    assert (x, y) == (60.0, 30.0) and r == pytest.approx((0.5 + 2.5 * 4) / FT)
    # Then the phone is heard right at the well, 400 ft from where sam was pinned 6 s ago: the pin is dropped.
    t2 = Tags({"people": {"sam": {"wifi": MAC}}, "sightings": {"everyone_tagged": True},
               "wifi": {"aps": {"barn": {"x": 0.0, "y": 0.0}, "well": {"x": 400.0, "y": 0.0}}, "learn": False}},
              Frame(FT, 0.0, *ORIGIN), FT, (0.0, 0.0, 400.0, 400.0))
    t2.feed("wifi/barn", heard, now)
    t2.feed("sightings/drone", spot, now)
    assert t2.fused(now)[0]["sam"][:2] == (60.0, 30.0)
    t2.feed("sightings/drone", {"people": []}, now + 5)
    t2.feed("wifi/barn", {"clients": {}}, now + 6)
    t2.feed("wifi/well", {"clients": {MAC: {"signal": -42}}}, now + 6)
    who, _ = t2.fused(now + 6)
    assert who["sam"][:2] == (400.0, 0.0) and "seen" not in t2.fixes["sam"]


def test_sightings_messages_fixtures_and_silent_sources():
    """A sightings picture may be in site coordinates or latitude and longitude, people or shapes. Shapes at a
    listed fixture are not reported. A source the site relies on that falls silent is named."""
    now = 1_800_000_000.0
    t = tagged(watch={"mast": 10}, fixtures=[{"x": 300.0, "y": 300.0, "r": 4.0}])
    assert t.silent(now) == [] and t.silent(now + 11) == ["mast"]                    # never heard from at all
    lat, lon = (float(v) for v in Frame(FT, 0.0, *ORIGIN).local_to_wgs84(120.0, 80.0))
    msg = {"t": now + 11, "objects": [{"x": 301.0, "y": 299.0, "r": 1.0}, {"x": 50.0, "y": 50.0}],
           "people": [{"lat": lat, "lon": lon, "acc": 2.0}, {"x": float("nan"), "y": 1.0}]}
    assert t.feed("sightings/mast", msg, now + 12) and t.fresh
    assert t.silent(now + 12) == [] and t.silent(now + 23) == ["mast"]
    got = sorted(t.seen.now(now + 11))
    assert [(round(s.x), round(s.y), s.counts) for s in got] == [(50, 50, False), (120, 80, True)]
    assert got[1].radius == pytest.approx(2.0 / FT) and got[0].radius == pytest.approx(0.5 / FT)
    assert not t.feed("sightings/mast/extra", msg, now) and not t.feed("other/mast", msg, now)
    # With no way to place latitude and longitude, such a sighting is an error, not a guess.
    bare = Tags({"sightings": {}}, None, FT, (0.0, 0.0, 0.0, 0.0))
    assert bare.names() == [] and bare.topics() == ["sightings/+"]
    with pytest.raises(ValueError, match="--origin"):
        bare.feed("sightings/drone", {"people": [{"lat": lat, "lon": lon}]}, now)
    with pytest.raises(ValueError, match="no sightings"):
        Tags({}, None, FT, (0.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("site,true", [("steady", (-40.0, 2.7, 3.0)), ("loud", (-30.0, 2.0, 6.0))])
def test_the_wifi_model_is_learnt_from_people_whose_place_is_known(site, true):
    """A phone that reports its own GPS position while the access points hear it is a walk with a tape measure,
    all day. After enough of it the model is the site's own, and the circles hold the phone 97 % of the time.

    On a site where the signal fades as the default model expects but wavers less ("steady"), the circles
    become smaller. On a site where the signal carries farther than the default expects ("loud"), the default
    model's circles are too small to hold the phone, and the learnt model puts that right.
    """
    rng = np.random.default_rng(5)
    cfg = {"people": {"sam": {"owntracks": "sam/phone", "wifi": MAC}},
           "wifi": {"aps": {"barn": {"x": 0.0, "y": 0.0}, "well": {"x": 400.0, "y": 0.0}}}}
    frame = Frame(FT, 0.0, *ORIGIN)
    t = Tags(cfg, frame, FT, (0.0, 0.0, 400.0, 400.0))
    start = t.model
    now = 1_800_000_000.0

    def signal(x, y, ap):
        """What an access point hears from a phone at a site point, under the true model."""
        d = max(math.hypot(x - ap[0], y - ap[1]) * FT, 1.0)
        return true[0] - 10 * true[1] * math.log10(d) + rng.normal(0, true[2])

    def predicted(model, d):
        """The signal a model expects at d metres."""
        return model[0] - 10 * model[1] * math.log10(d)

    for k in range(300):
        x, y = rng.uniform(0, 400, 2)
        lat, lon = (float(v) for v in frame.local_to_wgs84(x, y))
        t.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 4, "tst": now + k}, now + k)
        for name, ap in t.aps.items():
            t.feed(f"wifi/{name}", {"clients": {MAC: {"signal": signal(x, y, ap)}}}, now + k)
    assert t.model != start and len(t.samples) > 200 and abs(t.model[2] - true[2]) < 1.0
    assert all(abs(predicted(t.model, d) - predicted(true, d)) < 2.5 for d in (20, 50, 120))
    assert t.taught is not None and 10 <= t.taught[0] < 20 and t.taught[1] > 100
    held = held_default = 0
    radii, default = [], []
    for _ in range(3000):
        x, y = rng.uniform(0, 400, 2)
        rssi = signal(x, y, (0.0, 0.0))
        radii.append(t._range(rssi))
        default.append(wifi_range(rssi, *start))
        held += math.hypot(x, y) * FT <= radii[-1]
        held_default += math.hypot(x, y) * FT <= default[-1]
    print(f"\n{site} site, learnt model {tuple(round(v, 2) for v in t.model)}: holds the phone {held / 30:.1f} % "
          f"of the time, median radius {np.median(radii):.0f} m; the default model {held_default / 30:.1f} %, "
          f"{np.median(default):.0f} m")
    assert held / 3000 >= 0.97
    if site == "steady":
        assert np.median(radii) < 0.7 * np.median(default)
    else:
        assert held_default / 3000 < 0.9
    # Outside the distances it was taught at, a learnt model is not trusted to be tighter: a fit that happened to
    # come out wrong there (this one would put a phone 2 m away within 1 m) is held to the nearest sample, and
    # beyond the farthest sample to the tags file's own model.
    t.model = (-52.0, 1.85, 3.0)
    assert wifi_range(-50.0, *t.model) < 2 and t._range(-50.0) == t.taught[0]
    assert t._range(-120.0) == max(wifi_range(-120.0, *t.model), wifi_range(-120.0, *start)) > t.taught[1]
    # A vague position teaches nothing, and neither does learning switched off.
    n = len(t.samples)
    lat, lon = (float(v) for v in frame.local_to_wgs84(200.0, 200.0))
    t.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 60, "tst": now + 900}, now + 900)
    t.feed("wifi/barn", {"clients": {MAC: {"signal": -70}}}, now + 900)
    assert len(t.samples) == n
    cfg["wifi"]["learn"] = False
    off = Tags(cfg, frame, FT, (0.0, 0.0, 400.0, 400.0))
    lat, lon = (float(v) for v in frame.local_to_wgs84(300.0, 300.0))
    off.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 4, "tst": now}, now)
    off.feed("wifi/barn", {"clients": {MAC: {"signal": -70}}}, now)
    assert not off.samples and off.model == start


# -- person-sized things in a scan ------------------------------------------------------------------------------
def test_standing_objects_are_person_sized_things():
    """On a half-metre grid: a person is found where they stand; two people side by side are one wider thing,
    still person-sized; a machine is too wide, a stake too low, a pole too tall, and grass is not standing at all.
    """
    above = np.full((40, 60), 0.02)
    above[10, 10] = 1.75                                   # a person
    above[20, 30:32] = (1.7, 1.6)                          # two people shoulder to shoulder
    above[28:34, 40:52] = 2.9                              # a dozer
    above[5, 50] = 0.6                                     # a stake
    above[35, 5] = 4.5                                     # a light pole
    above[30, 20] = 0.25                                   # grass
    above[0, 0] = np.nan                                   # not seen
    found = sorted(standing_objects(above, 0.5))
    assert [(x, y, round(h, 2)) for x, y, _, h in found] == [(5.25, 5.25, 1.75), (15.5, 10.25, 1.7)]
    assert found[0][2] == pytest.approx(math.hypot(0.5, 0.5) / 2) and found[1][2] > found[0][2]
    assert standing_objects(np.full((3, 3), np.nan), 0.5) == []


def test_a_scan_gives_the_people_standing_in_it(tmp_path):
    """A lidar scan of the example pad with a person and a machine on it: the ground comes out clean, and the
    person is listed where they stand, in site coordinates."""
    g = SiteGrid(load(EXAMPLE), EXAMPLES)
    rng = np.random.default_rng(3)
    x, y = rng.uniform(g.x0, g.x1, 400_000), rng.uniform(g.y0, g.y1, 400_000)
    z = 100 + 0.01 * x + rng.normal(0, 0.02, x.size)
    person = np.hypot(x - 42.0, y - 61.0) < 0.9
    z[person] += rng.uniform(0.5, 5.8, person.sum())
    machine = (x > 100) & (x < 115) & (y > 20) & (y < 30)
    z[machine] += rng.uniform(4.0, 9.0, machine.sum())
    path = str(tmp_path / "scan.las")
    write_las(path, np.column_stack([x, y, z]))
    frame = Frame(FT, world_m=FT)
    ground, info = grid_cloud(path, g, frame, pmf={}, objects=True)
    assert np.nanmax(np.abs(ground - (100 + 0.01 * np.meshgrid(g.xs, g.ys)[0]))) < 0.5
    (o,) = info["objects"]
    assert math.hypot(o["x"] - 42.0, o["y"] - 61.0) < 1.0 and 1.0 < o["r"] < 2.5 and 1.4 < o["height_m"] < 1.8
    assert "objects" not in grid_cloud(path, g, frame, pmf={})[1]
    with pytest.raises(ValueError, match="ground filter"):
        grid_cloud(path, g, frame, objects=True)


def test_terrain_publishes_the_objects_of_a_scan(mqtt_broker, tmp_path, capsys):
    """`loam terrain --objects` sends, beside the terrain message, a sightings picture stamped with the scan's time
    that `loam sense` reads as it is."""
    g = SiteGrid(load(EXAMPLE), EXAMPLES)
    rng = np.random.default_rng(3)
    x, y = rng.uniform(g.x0, g.x1, 300_000), rng.uniform(g.y0, g.y1, 300_000)
    z = 100 + rng.normal(0, 0.02, x.size)
    person = np.hypot(x - 42.0, y - 61.0) < 0.9
    z[person] += rng.uniform(0.5, 5.8, person.sum())
    cloud = str(tmp_path / "scan.las")
    write_las(cloud, np.column_stack([x, y, z]))
    listen = Bus(mqtt_broker, f"l-{time.time()}").start(["sightings/+", wire.topic("pad_north", None, "terrain")])
    try:
        time.sleep(0.2)
        main(["terrain", EXAMPLE, cloud, "--world-units", "ft", "--ground-filter", "--objects", "--probe", "mast",
              "--broker", mqtt_broker])
        got = dict(listen.drain(3.0))
    finally:
        listen.stop()
    assert "objects" in capsys.readouterr().out
    pic = got["sightings/mast"]
    assert pic["t"] == pytest.approx(os.path.getmtime(cloud)) and len(pic["objects"]) == 1
    t = Tags({"sightings": {}}, None, FT, (0.0, 0.0, 0.0, 0.0))
    assert t.feed("sightings/mast", pic, time.time())
    (s,) = t.seen.now(time.time())
    assert math.hypot(s.x - 42.0, s.y - 61.0) < 1.0 and not s.counts and s.source == "mast"
    with pytest.raises(SystemExit):
        main(["terrain", EXAMPLE, cloud, "--world-units", "ft", "--objects", "--broker", mqtt_broker])


# -- all of it, over a broker -----------------------------------------------------------------------------------
class _Frigate:
    """Stands in for Frigate: answers `frigate/onConnect` with the current camera picture."""

    def __init__(self, broker):
        """Connect, say Frigate is up, and start answering."""
        self.activity = {"pad": view()}
        self.bus = Bus(broker, f"frigate-{time.time()}", raw=True).start(["frigate/onConnect"])
        self.bus.c.publish("frigate/available", "online", qos=1, retain=True)
        self.running = True
        self.thread = threading.Thread(target=self._answer, daemon=True)
        self.thread.start()

    def _answer(self):
        """Answer each request with the current picture."""
        while self.running:
            for topic, _ in self.bus.drain(0.05):
                if topic == "frigate/onConnect":
                    self.bus.pub("frigate/camera_activity", self.activity)

    def close(self):
        """Stop answering, remove the retained status and disconnect."""
        self.running = False
        self.thread.join(5)
        self.bus.clear("frigate/available")
        time.sleep(0.2)
        self.bus.stop()


def test_sense_pins_a_phone_to_the_person_a_camera_sees(mqtt_broker, monkeypatch):
    """`loam sense` with a calibrated camera and a tags file, over a real broker, on a site where everyone is
    tagged. A phone heard only by Wi-Fi is a circle hundreds of feet wide; when the camera sees one person inside
    it, the detection becomes that person's place, a few feet wide, under the person's name; a second person
    nobody can name gets a circle of their own; when the camera sees nobody the circle is wide again; and a
    relied-upon lidar that has never reported makes the site unwatched."""
    monkeypatch.setattr(sensors, "POLL_S", 0.2)
    monkeypatch.setattr(sensors, "TAG_REPEAT_S", 0.3)
    monkeypatch.setattr(sensors, "REPEAT_S", 1.0)
    model = load(EXAMPLE)
    cams = {"pad": Camera(Area(80.0, 50.0, 95.0), {}, Ground(GROUND))}
    tags = Tags({"people": {"sam": {"wifi": MAC}}, "wifi": {"aps": {"barn": {"x": 0.0, "y": 0.0}}},
                 "sightings": {"everyone_tagged": True, "watch": {"mast": 2.0}}},
                Frame(FT, 0.0, *ORIGIN), FT, (0.0, 0.0, 160.0, 100.0), "pad_north")
    topic = wire.topic("pad_north", None, "detection")
    fr = _Frigate(mqtt_broker)
    listen = Bus(mqtt_broker, f"listen-{time.time()}").start([topic])
    ap = Bus(mqtt_broker, f"ap-{time.time()}").start([])
    seen: list = []
    t = threading.Thread(target=lambda: run_sense(model, mqtt_broker, cams, tags=tags, max_wall_s=14.0))

    def state(want, wait=5.0):
        """The detections in force, by sensor, once `want(state)` holds (or after `wait` seconds)."""
        end, now = time.time() + wait, {}
        while time.time() < end:
            seen.extend(m for _, m in listen.drain(0.1))
            now = {}
            for m in seen:
                wire_valid("detection", m)
                if m["kind"] == "clear":
                    now.pop(m["sensor"], None)
                else:
                    now[m["sensor"]] = m
            if want(now):
                break
        return now

    def hear():
        """The barn's access point reports the phone, faintly."""
        ap.pub("wifi/barn", {"clients": {MAC: {"signal": -70}}})

    try:
        t.start()
        time.sleep(0.8)
        hear()
        s = state(lambda d: "sense/tag/sam" in d)
        assert s["sense/tag/sam"]["radius"] > 140 and (s["sense/tag/sam"]["x"], s["sense/tag/sam"]["y"]) == (0, 0)
        fr.activity = {"pad": view(box(35.0, 20.0))}
        hear()
        s = state(lambda d: d.get("sense/tag/sam", {}).get("radius", 999) < 30)
        sam = s["sense/tag/sam"]
        assert math.hypot(sam["x"] - 35, sam["y"] - 20) < 1.5 and sam["radius"] < 30 and sam["kind"] == "person"
        assert not [k for k in s if k.startswith("sense/seen/") or k == "sense/pad"]
        # A second person walks in. Two things seen, one phone: sam is one of them, and both are kept clear of.
        fr.activity = {"pad": view(box(35.0, 20.0), box(120.0, 70.0))}
        hear()
        s = state(lambda d: "sense/seen/2" in d)
        near = [m for k, m in s.items() if k.startswith("sense/")
                and min(math.hypot(m["x"] - 35, m["y"] - 20), math.hypot(m["x"] - 120, m["y"] - 70)) < 4]
        assert len(near) == 2 and all(m["radius"] < 40 for m in near)
        # After 2 s with no word from the lidar the site relies on, nobody is watching.
        s = state(lambda d: "sense/watch/mast" in d)
        assert s["sense/watch/mast"]["kind"] == "unknown" and s["sense/watch/mast"]["radius"] == wire.EVERYWHERE
        ap.pub("sightings/mast", {"objects": []})
        s = state(lambda d: "sense/watch/mast" not in d)
        assert "sense/watch/mast" not in s
        # Everybody leaves the picture; sam's phone is still heard. Wide again, growing from the last place seen.
        fr.activity = {"pad": view()}
        s = state(lambda d: not [k for k in d if k.startswith("sense/seen/")]
                  and d.get("sense/tag/sam", {}).get("radius", 0) > 5)
        assert not [k for k in s if k.startswith("sense/seen/")] and s["sense/tag/sam"]["radius"] > 5
    finally:
        t.join(20)
        listen.stop()
        ap.stop()
        fr.close()
