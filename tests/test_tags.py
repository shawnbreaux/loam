"""People located by their phones and trackers (`loam sense --tags`): OwnTracks, Wi-Fi signal strength at the
site's access points, and Meshtastic. The message shapes follow the projects' own sources: OwnTracks' booklet
(tech/json.md), hostapd's ubus `get_clients` in OpenWrt, and Meshtastic's MeshPacketSerializer.
"""
import json
import math
import threading
import time

import numpy as np
import pytest
from conftest import EXAMPLE, wire_valid

from loam import wire
from loam.cli import load, main
from loam.geo import Frame
from loam.runtime.bus import Bus
from loam.sensors import run_sense
from loam.tags import CELL_M, GONE_M, K_SIGMA, Tags, fit_path_loss, smallest_circle, wifi_range

FT = 0.3048
ORIGIN = (34.5712, -85.6790)                                   # site (0, 0), for the sources that report lat/lon
CFG = {"people": {"sam": {"owntracks": "sam/phone", "wifi": "A4:83:E7:12:34:56", "meshtastic": "!a1b2c3d4"},
                  "ana": {"wifi": "a4:83:e7:00:00:02"}},
       "wifi": {"aps": {"barn": {"x": 0.0, "y": 0.0, "bssid": "74:AC:B9:00:11:22"},
                        "well": {"x": 400.0, "y": 0.0}, "gate": {"x": 0.0, "y": 400.0}}}}


def tags(cfg=CFG):
    """The example tags on a site in feet, 400 ft square."""
    return Tags(cfg, Frame(FT, 0.0, *ORIGIN), FT, (0.0, 0.0, 400.0, 400.0))


def latlon(x, y):
    """The latitude and longitude of a site point."""
    lat, lon = Frame(FT, 0.0, *ORIGIN).local_to_wgs84(x, y)
    return float(lat), float(lon)


def test_wifi_model_and_its_fit():
    """Signal strength gives a distance the phone is almost certainly within, which grows fast as the signal fades;
    and a walk with a phone gives the model back.
    """
    assert wifi_range(-40, sigma_db=0.0) == pytest.approx(1.0)               # the reference: 1 m
    near, mid, far = (wifi_range(r) for r in (-59, -75, -85))
    assert 14 < near < 20 and 60 < mid < 75 and 150 < far < 180              # 5, 20 and 46 m away, typically
    rng = np.random.default_rng(4)
    d = rng.uniform(2, 150, 400)
    walk = [(float(x), float(-38 - 10 * 3.1 * math.log10(x) + rng.normal(0, 5.0))) for x in d]
    rssi_1m, exponent, sigma = fit_path_loss(walk)
    assert abs(rssi_1m + 38) < 1.5 and abs(exponent - 3.1) < 0.1 and abs(sigma - 5.0) < 0.5
    with pytest.raises(ValueError, match="at least 5 samples"):
        fit_path_loss([(10.0, -60.0)] * 9)                                   # one distance tells nothing


def test_smallest_circle_holds_the_overlap():
    """Whatever point lies in all the circles also lies in the answer, and the answer is never larger than the
    smallest circle. Two access points 100 m apart that each hear a phone within 60 m pin it to 33 m.
    """
    x, y, r = smallest_circle([(0, 0, 60), (100, 0, 60)])
    assert (x, y) == pytest.approx((50, 0)) and r == pytest.approx(math.sqrt(60 ** 2 - 50 ** 2))
    assert smallest_circle([(0, 0, 60), (10, 0, 20)]) == (10, 0, 20)         # one inside the other
    assert smallest_circle([(0, 0, 10), (100, 0, 20)]) == (0, 0, 10)         # apart: no telling which is wrong
    rng = np.random.default_rng(1)
    for _ in range(300):
        p = rng.uniform(-50, 50, 2)                                          # a point all the circles contain
        circles = []
        for _ in range(int(rng.integers(1, 5))):
            c = p + rng.uniform(-80, 80, 2)
            circles.append((float(c[0]), float(c[1]), float(np.hypot(*(c - p)) + rng.uniform(0, 40))))
        x, y, r = smallest_circle(circles)
        assert r <= min(c[2] for c in circles) + 1e-9
        q = p + rng.uniform(-120, 120, (400, 2))                             # other points: those in every circle
        inside = np.all([np.hypot(q[:, 0] - cx, q[:, 1] - cy) <= cr for cx, cy, cr in circles], axis=0)
        assert np.all(np.hypot(q[inside, 0] - x, q[inside, 1] - y) <= r + 1e-9)


def test_wifi_circles_hold_the_person_under_the_model():
    """With signal fading as the model assumes (6 dB standard deviation), the circle reported for a phone contains
    the phone 97 % of the time or more, whether one, two or three access points hear it. The price is its size.
    """
    rng = np.random.default_rng(2)
    aps = [(0.0, 0.0), (120.0, 0.0), (0.0, 120.0)]
    for heard in (1, 2, 3):
        held, radii = 0, []
        for _ in range(4000):
            p = rng.uniform(0, 120, 2)
            circles = []
            for ax, ay in aps[:heard]:
                d = max(float(np.hypot(p[0] - ax, p[1] - ay)), 1.0)
                rssi = -40 - 27 * math.log10(d) + rng.normal(0, 6.0)
                circles.append((ax, ay, wifi_range(rssi)))
            x, y, r = smallest_circle(circles)
            held += np.hypot(p[0] - x, p[1] - y) <= r
            radii.append(r)
        print(f"\n{heard} access point(s): person inside the circle {held / 40:.1f} % of the time; "
              f"median radius {np.median(radii):.0f} m")
        assert held / 4000 >= 0.97
    assert K_SIGMA == 2.5


def test_each_source_places_a_person():
    """A phone's own position, the access point it is on, the access points that hear it and a LoRa tracker each
    put the person in a circle on the site; the tightest is used.
    """
    t, now = tags(), 1_800_000_000.0
    assert t.topics() == ["owntracks/+/+", "wifi/+", "msh/#", "sightings/+"] and t.names() == ["ana", "sam"]
    assert t.located(now) == {}                                              # nobody has been located yet
    lat, lon = latlon(250.0, 100.0)
    fix = {"_type": "location", "lat": lat, "lon": lon, "acc": 8, "tst": int(now), "tid": "sm",
           "BSSID": "74:ac:b9:00:11:22"}
    assert t.feed("owntracks/sam/phone", fix, now) and t.fresh
    assert not t.feed("owntracks/someone/else", fix, now)                    # not in the tags file
    assert t.feed("owntracks/sam/phone", {"_type": "lwt"}, now)              # ours, and nothing to learn from
    x, y, r = t.located(now)["sam"]
    assert (x, y) == pytest.approx((250.0, 100.0), abs=0.05) and r == pytest.approx(8 / FT)
    assert t.fixes["sam"]["bssid"][1:] == pytest.approx((0.0, 0.0, CELL_M / FT))   # and it is on the barn's Wi-Fi
    # Thirty seconds of silence: the person can have walked 75 m.
    assert t.located(now + 30)["sam"][2] == pytest.approx((8 + 2.5 * 30) / FT)
    # The access points hear the phone meanwhile (hostapd's own report): a fresher, and now tighter, circle.
    report = {"freq": 2437, "clients": {"a4:83:e7:12:34:56": {"auth": True, "assoc": True, "signal": -50},
                                        "de:ad:be:ef:00:01": {"signal": -40}}}
    assert t.feed("wifi/well", report, now + 30)
    x, y, r = t.located(now + 30)["sam"]
    assert (x, y) == (400.0, 0.0) and r == pytest.approx(wifi_range(-50) / FT)
    assert t.feed("wifi/gate", {"mac": "A4:83:E7:12:34:56", "rssi": -72}, now + 31)   # a second one: the overlap
    assert t.located(now + 31)["sam"][2] < wifi_range(-50) / FT + 2.5 / FT
    assert t.feed("wifi/well", {"clients": {}}, now + 32)                    # no longer on that access point
    assert "well" not in t.heard["a4:83:e7:12:34:56"]
    # A measured range (Wi-Fi RTT) is used as it is, with its own uncertainty.
    assert t.feed("wifi/barn", {"mac": "a4:83:e7:00:00:02", "range_m": 12.0, "std_m": 1.0}, now)
    assert t.located(now)["ana"] == pytest.approx((0.0, 0.0, (12.0 + 2.5) / FT))
    # A tracker on a channel that blurs positions to 19 bits is only known to 45 m.
    lat, lon = latlon(100.0, 300.0)
    mesh = {"from": 0xA1B2C3D4, "type": "position", "sender": "!gateway1", "timestamp": int(now) + 40,
            "payload": {"latitude_i": round(lat * 1e7), "longitude_i": round(lon * 1e7), "time": int(now) + 40,
                        "precision_bits": 19}}
    assert t.feed("msh/US/2/json/LongFast/!gateway1", mesh, now + 40)
    assert t.fixes["sam"]["meshtastic"][3] == pytest.approx(45.6 / FT, rel=0.01)
    assert not t.feed("frigate/stats", {"cameras": {}}, now) and not t.feed("wifi/shed", report, now)


def test_an_access_point_on_a_machine_is_a_proximity_sensor():
    """A machine that carries an access point hears a phone strongly only when the person is close to it. The
    circle is then round the machine, wherever the machine says it is, and small; with the machine's position
    unknown, its access point places nobody.
    """
    cfg = {"people": {"sam": {"wifi": "a4:83:e7:12:34:56"}}, "wifi": {"aps": {"dz1-ap": {"machine": "dz1"}}}}
    t = Tags(cfg, Frame(FT, 0.0, *ORIGIN), FT, (0.0, 0.0, 400.0, 400.0), "pad_north")
    now = 1_800_000_000.0
    assert t.topics() == ["wifi/+", "loam/v0/pad_north/+/state", "sightings/+"]
    heard = {"clients": {"a4:83:e7:12:34:56": {"signal": -52}}}
    assert t.feed("wifi/dz1-ap", heard, now) and t.located(now) == {}        # where is the machine?
    assert t.feed("loam/v0/pad_north/dz1/state", wire.state("pad_north", "dz1", 1, None, 150.0, 80.0, 0.0, True, 0.2,
                                                           ts=now), now)
    t.feed("wifi/dz1-ap", heard, now + 1)
    x, y, r = t.located(now + 1)["sam"]
    assert (x, y) == (150.0, 80.0) and r * FT == pytest.approx(wifi_range(-52)) and r * FT < 12
    t.feed("loam/v0/pad_north/dz1/state", wire.state("pad_north", "dz1", 2, None, 180.0, 80.0, 0.0, True, 0.3,
                                                     ts=now + 2), now + 2)
    t.feed("wifi/dz1-ap", heard, now + 2)
    assert t.located(now + 2)["sam"][:2] == (180.0, 80.0)                    # it moves with the machine
    t.feed("wifi/dz1-ap", heard, now + 60)                                   # the machine has gone quiet
    assert t.fixes["sam"]["wifi"][0] == now + 2


def test_a_person_who_left_the_site_is_not_followed():
    """Someone last located well outside the site has left: they are not reported, and their silence afterwards
    does not grow into a circle that stops the machines. Someone who falls silent on the site does.
    """
    t, now = tags(), 1_800_000_000.0
    away = GONE_M / FT + 80
    lat, lon = latlon(400.0 + away, 200.0)
    t.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 10, "tst": int(now)}, now)
    assert "sam" not in t.located(now) and "sam" not in t.located(now + 3600)
    lat, lon = latlon(380.0, 200.0)                                          # back at the gate
    t.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 10, "tst": int(now) + 60},
           now + 60)
    assert t.located(now + 60)["sam"][2] == pytest.approx(10 / FT)
    assert t.located(now + 60 + 600)["sam"][2] * FT > 1500                   # ten silent minutes: could be anywhere
    # A fix that was queued in the phone and arrives late is as old as the phone says.
    t.feed("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 10, "tst": int(now) + 700},
           now + 760)
    assert t.located(now + 760)["sam"][2] == pytest.approx((10 + 2.5 * 60) / FT)


def test_tags_file_mistakes_are_reported():
    """A tags file that cannot work says why."""
    frame = Frame(FT, 0.0, *ORIGIN)
    with pytest.raises(ValueError, match="names no person"):
        Tags({"people": {}}, frame, FT, (0, 0, 1, 1))
    with pytest.raises(ValueError, match="places no access points"):
        Tags({"people": {"sam": {"wifi": "aa:bb:cc:dd:ee:ff"}}}, frame, FT, (0, 0, 1, 1))
    with pytest.raises(ValueError, match="--origin"):
        Tags({"people": {"sam": {"owntracks": "sam/phone"}}}, Frame(FT), FT, (0, 0, 1, 1))


def test_sense_reports_people_by_their_phones(mqtt_broker, monkeypatch):
    """`loam sense --tags` over a real broker: a phone's position becomes a person detection with the fix's
    uncertainty as its radius; while the phone is silent the circle grows with every report; the access point that
    hears it gives a tighter one; and when the program stops, nobody is known to be watched.
    """
    import loam.sensors as sensors
    monkeypatch.setattr(sensors, "TAG_REPEAT_S", 0.3)
    monkeypatch.setattr(sensors, "REPEAT_S", 1.0)
    model = load(EXAMPLE)
    topic = wire.topic("pad_north", None, "detection")
    listen = Bus(mqtt_broker, f"listen-{time.time()}").start([topic])
    phone = Bus(mqtt_broker, f"phone-{time.time()}").start([])
    result: dict = {}
    t = threading.Thread(target=lambda: result.update(run_sense(model, mqtt_broker, tags=tags(), max_wall_s=8.0)))
    seen: list = []

    def sam(wait=3.0, since=0):
        """Detections of sam received from now on, for up to `wait` seconds or until there is one."""
        end = time.time() + wait
        while time.time() < end and not [m for m in seen[since:]
                                         if m["sensor"] == "sense/tag/sam" and m["kind"] != "clear"]:
            seen.extend(m for _, m in listen.drain(0.1))
        return [m for m in seen[since:] if m["sensor"] == "sense/tag/sam" and m["kind"] != "clear"]

    try:
        t.start()
        time.sleep(1.0)
        lat, lon = latlon(120.0, 60.0)
        phone.pub("owntracks/sam/phone", {"_type": "location", "lat": lat, "lon": lon, "acc": 6,
                                          "tst": int(time.time())}, qos=1)
        first = sam()
        assert first and first[0]["kind"] == "person"
        wire_valid("detection", first[0])
        assert (first[0]["x"], first[0]["y"]) == pytest.approx((120.0, 60.0), abs=0.1)
        assert 6 / FT <= first[0]["radius"] < (6 + 2.5 * 3.0) / FT            # the fix, plus the walk until the next
        time.sleep(1.5)                                                      # the phone says nothing more
        seen.extend(m for _, m in listen.drain(0.1))
        later = sam(since=len(seen))
        assert later[-1]["radius"] > first[0]["radius"] + 2.5 * 1.0 / FT      # the circle has grown
        n = len(seen)
        phone.pub("wifi/barn", {"clients": {"a4:83:e7:12:34:56": {"signal": -45}}})
        tight = sam(since=n)
        assert tight and (tight[-1]["x"], tight[-1]["y"]) == (0.0, 0.0)       # round the barn's access point
        assert tight[-1]["radius"] < later[-1]["radius"]
        assert {m["sensor"] for m in seen if m["kind"] == "clear"} >= {"sense", "sense/tag/ana"}
    finally:
        t.join(15)
        seen.extend(m for _, m in listen.drain(0.5))
        listen.stop()
        phone.stop()
    assert result["detection"] > 5
    last = [m for m in seen if m["sensor"] == "sense"][-1]
    assert last["kind"] == "unknown" and last["radius"] == wire.EVERYWHERE    # stopped: nobody is watching


def test_wifi_fit_command(tmp_path, capsys):
    """`loam wifi-fit` reads a walk and prints the model for the tags file."""
    rng = np.random.default_rng(3)
    lines = ["distance_m,rssi"] + [f"{d:.1f},{-41 - 26 * math.log10(d) + rng.normal(0, 4):.0f}"
                                   for d in rng.uniform(2, 120, 200)]
    (tmp_path / "walk.csv").write_text("\n".join(lines))
    main(["wifi-fit", str(tmp_path / "walk.csv"), "--json"])
    out = json.loads(capsys.readouterr().out)
    assert out["samples"] == 200 and abs(out["rssi_1m"] + 41) < 2 and abs(out["exponent"] - 2.6) < 0.15
    assert abs(out["sigma_db"] - 4) < 0.6 and out["radius_m"]["at -60 dBm"] < out["radius_m"]["at -80 dBm"]
