"""The broker connection: TLS, passwords and access rules, with the setup that `loam broker` writes.

A real Mosquitto runs with exactly the files `loam broker` wrote and its `install.sh` installed, so the setup in
the documentation is the one that is tested. Three tests run against a broker installed on the system instead
(the CI job `broker`: Ubuntu's packaged Mosquitto, confined by AppArmor); they skip unless LOAM_INSTALLED_BROKER
names the folder `loam broker` wrote for it.
"""
import fnmatch
import getpass
import os
import re
import shutil
import socket
import stat
import subprocess
import sys
import time

import pytest
from conftest import MOSQUITTO, ROOT

from loam import wire
from loam.broker import ROLES, password_line, site_broker
from loam.runtime.bus import Broker, Bus, parse_broker

OPENSSL = shutil.which("openssl")
INSTALLED = os.environ.get("LOAM_INSTALLED_BROKER")          # folder written by `loam broker` and installed
RESTART = os.environ.get("LOAM_INSTALLED_BROKER_RESTART")    # command that restarts that broker
# What Ubuntu's AppArmor profile lets Mosquitto read under /etc (apparmor 5.0, /etc/apparmor.d/mosquitto).
CONFINED_TO = ("/etc/mosquitto/*", "/etc/mosquitto/conf.d/**", "/etc/mosquitto/certs/**",
               "/etc/mosquitto/ca_certificates/**")


def test_broker_addresses():
    """Addresses parse to host, port, TLS and user; the standard port follows the scheme; a password in the address
    is refused.
    """
    assert parse_broker("localhost:1883") == Broker("localhost", 1883, False, None)
    assert parse_broker(":1884") == Broker("localhost", 1884, False, None)
    assert parse_broker("mqtt://10.0.0.5") == Broker("10.0.0.5", 1883, False, None)
    assert parse_broker("mqtts://dz1@site.example") == Broker("site.example", 8883, True, "dz1")
    assert parse_broker("mqtts://site.example:9000/") == Broker("site.example", 9000, True, None)
    for bad, why in [("mqtts://dz1:hunter2@site", "LOAM_BROKER_PASSWORD"), ("http://site", "mqtts://"),
                     ("site:port", "must be a number")]:
        with pytest.raises(SystemExit, match=why):
            parse_broker(bad)


def test_password_file_and_bad_certificate_paths(tmp_path):
    """The password can come from a file (first line); unreadable files are reported, and nothing is left open."""
    secret = tmp_path / "pw"
    secret.write_text("from-file\nignored\n")
    bus = Bus("mqtt://dz1@localhost:1", "t", env={"LOAM_BROKER_PASSWORD_FILE": str(secret)})
    assert bus.c._password == b"from-file"
    bus._close_wakeup()
    with pytest.raises(SystemExit, match="password file"):
        Bus("mqtt://dz1@localhost:1", "t", env={"LOAM_BROKER_PASSWORD_FILE": str(tmp_path / "missing")})
    with pytest.raises(SystemExit, match="TLS certificates"):
        Bus("mqtts://localhost:1", "t", env={"LOAM_BROKER_CA": str(tmp_path / "missing.crt")})


@pytest.fixture
def secure_broker(tmp_path, example_model):
    """Mosquitto on a free port, set up the way a site server is: `loam broker` writes the files, its `install.sh`
    installs them (into a temporary folder standing in for /etc/mosquitto), and Mosquitto's main configuration
    includes `conf.d`, as the packaged one does. Yields (port, the folder `loam broker` wrote).
    """
    if not (MOSQUITTO and OPENSSL):
        pytest.skip("needs mosquitto and openssl")
    out, etc = str(tmp_path / "broker"), str(tmp_path / "etc" / "mosquitto")
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    site_broker(example_model, out, "localhost", etc=etc, port=port)
    me = getpass.getuser()
    done = subprocess.run(["sh", os.path.join(out, "install.sh"), me], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    # Started by root, Mosquitto switches to the user `mosquitto`, who cannot read a temporary folder; keep the
    # user who runs the tests. (Ignored when the tests do not run as root.)
    main = tmp_path / "mosquitto.conf"
    main.write_text(f"user {me}\ninclude_dir {etc}/conf.d\n")
    proc = subprocess.Popen([MOSQUITTO, "-c", str(main)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                            text=True)
    try:
        for _ in range(100):
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    break
            if proc.poll() is not None:
                why = proc.stderr.read()
                if "Unable to open config file" in why and not os.environ.get("CI"):
                    # Ubuntu's Mosquitto is confined by AppArmor to /etc/mosquitto and cannot read a test folder.
                    # CI lifts the confinement for these tests (.github/workflows/ci.yml), so there this is an
                    # error; its `broker` job tests the confined broker with the files in their real place.
                    pytest.skip("this Mosquitto may not read files outside /etc/mosquitto (AppArmor)")
                raise AssertionError(why)
            time.sleep(0.05)
        yield port, out
    finally:
        proc.terminate()
        proc.wait(5)
        log = proc.stderr.read()
        proc.stderr.close()
    # Mosquitto warns about secret files it will one day refuse (readable by others, or owned by someone else)
    # and about access rules that apply to every user.
    assert "Future versions will refuse" not in log and "Warning: ACL" not in log, log


def client(port, out, user, subs=(), password=None, host="localhost"):
    """A connected client logged in as `user` over TLS, set up as the documentation says: the site's CA
    certificate and the client's own password file. `password` overrides the file (to try a wrong one).
    """
    env = {"LOAM_BROKER_CA": os.path.join(out, "ca.crt")}
    if password is None:
        env["LOAM_BROKER_PASSWORD_FILE"] = os.path.join(out, "clients", f"{user}.password")
    else:
        env["LOAM_BROKER_PASSWORD"] = password
    return Bus(f"mqtts://{user}@{host}:{port}", f"test-{user}-{time.time()}", env=env).start(list(subs), timeout=5)


def got(bus, topic, wait=1.5):
    """The messages that arrive on `topic` within `wait` seconds."""
    end, out = time.time() + wait, []
    while time.time() < end:
        out += [m for t, m in bus.drain(0.1) if t == topic]
        if out:
            break
    return out


def test_tls_and_passwords(secure_broker):
    """Over TLS with the right password a message gets through. A wrong password, an untrusted certificate, a
    certificate for another host name and a connection without TLS are all refused, each with its reason.
    """
    port, out = secure_broker
    ca = os.path.join(out, "ca.crt")
    with open(os.path.join(out, "clients", "dz1.password"), encoding="utf-8") as f:
        secret = {"LOAM_BROKER_PASSWORD": f.read().strip()}
    order = wire.topic("s", "dz1", "order")
    ctl, dz1 = client(port, out, "controller"), client(port, out, "dz1", [order])
    try:
        time.sleep(0.3)
        ctl.pub(order, {"n": 1}, qos=1)
        assert got(dz1, order) == [{"n": 1}]
    finally:
        ctl.stop()
        dz1.stop()
    with pytest.raises(SystemExit, match="refused the connection"):
        client(port, out, "dz1", password="wrong")
    with pytest.raises(SystemExit, match="refused the connection"):          # no name at all
        Bus(f"mqtts://localhost:{port}", "anon", env={"LOAM_BROKER_CA": ca}).start([], timeout=5)
    with pytest.raises(SystemExit, match="CERTIFICATE_VERIFY_FAILED"):       # the system does not trust this CA
        Bus(f"mqtts://dz1@localhost:{port}", "noca", env=secret).start([], timeout=5)
    with pytest.raises(SystemExit, match="cannot reach"):                    # plain MQTT to the TLS port
        Bus(f"mqtt://dz1@localhost:{port}", "plain", env=secret).start([], timeout=2)


def lanes(port, out, host="localhost"):
    """Check the access rules on a running broker: a machine can report its own state and read its own orders,
    and nothing more. It cannot order another machine (or itself), speak for another machine, or read another
    machine's orders. The supervisor can stop and resume but not order; sensors can report but not command;
    the camera system cannot pose as a machine. Phones and access points can say where a device is and nothing
    else: only the site's own sensors can publish what a lidar or camera saw.
    """
    t = lambda machine, kind: wire.topic("s", machine, kind)                 # noqa: E731
    ctl = client(port, out, "controller", ["loam/v0/#"], host=host)
    dz1 = client(port, out, "dz1", [t("dz1", "order"), t("dz2", "order"), t(None, "safety"), t(None, "estop")],
                 host=host)
    dz2 = client(port, out, "dz2", [t("dz2", "order")], host=host)
    sup, sen = client(port, out, "supervisor", host=host), client(port, out, "sensors", host=host)
    cam = client(port, out, "frigate", [t(None, "estop"), "frigate/#"], host=host)
    ears = client(port, out, "sensors", ["owntracks/#", "wifi/#", "sightings/#"], host=host)
    trk = client(port, out, "trackers", ["loam/v0/#", "owntracks/#", "sightings/#"], host=host)
    try:
        time.sleep(0.3)
        # what must work
        dz1.pub(t("dz1", "state"), {"ok": 1}, qos=1)
        assert got(ctl, t("dz1", "state")) == [{"ok": 1}]
        ctl.pub(t("dz1", "order"), {"order": 1}, qos=1)
        ctl.pub(t(None, "safety"), {"hb": 1}, qos=1)
        assert got(dz1, t("dz1", "order")) == [{"order": 1}]
        sup.pub(t(None, "estop"), {"pressed": True}, qos=1)
        assert got(dz1, t(None, "estop")) == [{"pressed": True}]
        sen.pub(t(None, "detection"), {"kind": "person"}, qos=1)
        assert got(ctl, t(None, "detection")) == [{"kind": "person"}]
        trk.pub("owntracks/sam/phone", {"lat": 34.5}, qos=1)
        assert got(ears, "owntracks/sam/phone") == [{"lat": 34.5}]
        trk.pub("wifi/barn", {"clients": {}}, qos=1)
        assert got(ears, "wifi/barn") == [{"clients": {}}]
        sen.pub("sightings/mast", {"objects": []}, qos=1)
        assert got(ears, "sightings/mast") == [{"objects": []}]
        # what must not
        trk.pub("sightings/mast", {"forged": 9}, qos=1)                      # a phone cannot say what a lidar saw
        trk.pub(t(None, "detection"), {"forged": 10}, qos=1)                 # nor report a detection
        dz1.pub(t("dz2", "order"), {"forged": 1}, qos=1)                     # order another machine
        dz1.pub(t("dz1", "order"), {"forged": 2}, qos=1)                     # order itself
        dz1.pub(t("dz2", "state"), {"forged": 3}, qos=1)                     # speak for another machine
        dz1.pub(t(None, "safety"), {"forged": 4}, qos=1)                     # fake the controller's heartbeat
        sup.pub(t("dz2", "order"), {"forged": 5}, qos=1)                     # the supervisor does not plan
        sen.pub(t("dz2", "instantAction"), {"forged": 6}, qos=1)             # a sensor commands nothing
        cam.pub(t("frigate", "state"), {"forged": 7}, qos=1)                 # a role is not a machine
        cam.pub(t(None, "detection"), {"forged": 8}, qos=1)                  # nor a Loam sensor
        ctl.pub(t("dz2", "order"), {"order": 2}, qos=1)                      # the one genuine order
        assert got(dz2, t("dz2", "order")) == [{"order": 2}]
        time.sleep(0.5)
        seen = ctl.drain() + dz2.drain() + dz1.drain() + ears.drain()
        assert not trk.drain()                                               # and it is told nothing at all
        assert not [tp for tp, _ in cam.drain() if tp.startswith("loam/")]  # it was sent the e-stop above: not for it
        assert not [m for _, m in seen if "forged" in m], seen
        assert not [m for tp, m in seen if tp == t("dz2", "order") and m != {"order": 2}]
    finally:
        for b in (ctl, dz1, dz2, sup, sen, cam, ears, trk):
            b.stop()


def test_access_rules_keep_machines_in_their_lane(secure_broker):
    """The access rules `loam broker` writes hold on a real Mosquitto (see `lanes`)."""
    lanes(*secure_broker)


def test_broker_setup(tmp_path, example_model):
    """`loam broker` writes a folder that is closed to other users, names only files the confined Mosquitto may
    read, and can be rerun: the certificate authority and the passwords already handed out are kept, and a new
    machine gets a password.
    """
    if not OPENSSL:
        pytest.skip("needs openssl")
    out = str(tmp_path / "b")
    first = site_broker(example_model, out, "site-server", ["192.168.1.10", "site.example"])
    assert first["ca_created"] and first["users"] == [*ROLES, "dz1", "dz2", "rl1"] == first["new"]
    mode = lambda name: stat.S_IMODE(os.stat(os.path.join(out, name)).st_mode)      # noqa: E731
    assert mode("") == 0o700 and mode("clients") == 0o700
    for secret in ("ca.key", "server.key", "passwords", "acl", "clients/dz1.password"):
        assert mode(secret) == 0o600, secret
    names = subprocess.run([OPENSSL, "x509", "-in", os.path.join(out, "server.crt"), "-noout", "-ext",
                            "subjectAltName"], capture_output=True, text=True).stdout
    for name in ("DNS:site-server", "IP Address:192.168.1.10", "DNS:site.example", "DNS:localhost",
                 "IP Address:127.0.0.1"):
        assert name in names
    # Every file the configuration names and every place install.sh writes is one the confined Mosquitto may read.
    def text(name):
        """The contents of one file in the folder."""
        with open(os.path.join(out, name), encoding="utf-8") as f:
            return f.read()
    conf, install = text("loam.conf"), text("install.sh")
    targets = re.findall(r"^\w+file (\S+)$", conf, re.M) + re.findall(r'^install -m .* "\$etc(/[^"]+)"$', install, re.M)
    assert len(targets) == 4 + 6
    for path in targets:
        path = path if path.startswith("/etc") else "/etc/mosquitto" + path
        allowed = [g for g in CONFINED_TO if fnmatch.fnmatch(path, g) and ("**" in g or path.count("/") == 3)]
        assert allowed, path
    assert "listener 8883\n" in conf and "allow_anonymous false" in conf and "1883" not in conf

    kept = lambda: (text("ca.crt"), text("clients/dz1.password"))                   # noqa: E731
    before, old_cert = kept(), text("server.crt")
    example_model.machines["ex9"] = dict(example_model.machines["dz1"])
    try:
        again = site_broker(example_model, out, "site-server")
        assert not again["ca_created"] and again["new"] == ["ex9"] and kept() == before
        assert text("server.crt") != old_cert                                       # reissued
        assert text("passwords").count("\nex9:$7$101$") == 1
        example_model.machines["sensors"] = example_model.machines.pop("ex9")
        with pytest.raises(SystemExit, match="reserved"):
            site_broker(example_model, out, "site-server")
    finally:
        example_model.machines.pop("ex9", None)
        example_model.machines.pop("sensors", None)


def test_password_lines_are_mosquittos(tmp_path):
    """A password line is what `mosquitto_passwd` writes: PBKDF2-HMAC-SHA512, 101 rounds, a fresh 12-byte salt.
    (That a real Mosquitto accepts them is `test_tls_and_passwords`.)
    """
    import base64
    import hashlib
    a, b = password_line("dz1", "s3cret"), password_line("dz1", "s3cret")
    user, _, kind, rounds, salt, digest = re.split(r"[:$]", a)
    assert (user, kind, rounds) == ("dz1", "7", "101") and a != b
    raw = base64.b64decode(salt)
    assert len(raw) == 12
    assert base64.b64decode(digest) == hashlib.pbkdf2_hmac("sha512", b"s3cret", raw, 101, 64)
    tool = shutil.which("mosquitto_passwd")
    if tool:                                                 # the same fields as the tool's own line
        pw = tmp_path / "pw"
        subprocess.run([tool, "-b", "-c", str(pw), "dz1", "s3cret"], check=True, capture_output=True)
        theirs = re.split(r"[:$]", pw.read_text().strip())
        assert theirs[:4] == ["dz1", "", "7", "101"] and [len(x) for x in theirs[4:]] == [len(salt), len(digest)]


def test_broker_command(tmp_path, capsys):
    """`loam broker` says what it made and what to do with it."""
    if not OPENSSL:
        pytest.skip("needs openssl")
    from loam.cli import main
    out = str(tmp_path / "b")
    main(["broker", os.path.join(ROOT, "examples", "pad_north.loam"), "--host", "site-server", "--out", out])
    said = capsys.readouterr().out
    assert "9 clients" in said and f"sudo sh {out}/install.sh" in said and "mqtts://<name>@site-server" in said
    assert os.path.exists(os.path.join(out, "install.sh"))


# -- a broker installed on this computer (the CI job `broker`) -------------------------------------

installed = pytest.mark.skipif(not INSTALLED, reason="needs LOAM_INSTALLED_BROKER (a folder written by "
                                                    "`loam broker --host localhost` and installed)")


@installed
def test_installed_broker_serves_only_tls_and_named_clients():
    """The system's Mosquitto, set up by `sudo sh install.sh`: clients get in with their password files over TLS,
    a wrong password is refused, and nothing listens without TLS any more.
    """
    order = wire.topic("s", "dz1", "order")
    ctl, dz1 = client(8883, INSTALLED, "controller"), client(8883, INSTALLED, "dz1", [order])
    try:
        time.sleep(0.3)
        ctl.pub(order, {"n": 1}, qos=1)
        assert got(dz1, order) == [{"n": 1}]
    finally:
        ctl.stop()
        dz1.stop()
    with pytest.raises(SystemExit, match="refused the connection"):
        client(8883, INSTALLED, "dz1", password="wrong")
    with socket.socket() as s:
        assert s.connect_ex(("127.0.0.1", 1883)) != 0        # the package's default listener is gone


@installed
def test_installed_broker_access_rules():
    """The access rules hold on the system's Mosquitto too (see `lanes`)."""
    lanes(8883, INSTALLED)


@installed
@pytest.mark.skipif(not RESTART, reason="needs LOAM_INSTALLED_BROKER_RESTART (the command that restarts it)")
def test_installed_broker_keeps_the_estop_over_a_restart():
    """`loam estop` reaches the secured broker as the supervisor, and a pressed e-stop is still there for a machine
    that connects after the broker has been restarted (the package keeps retained messages on disk).
    """
    site = os.path.join(ROOT, "examples", "pad_north.loam")
    env = {**os.environ, "LOAM_BROKER_CA": os.path.join(INSTALLED, "ca.crt"),
           "LOAM_BROKER_PASSWORD_FILE": os.path.join(INSTALLED, "clients", "supervisor.password")}
    cmd = [sys.executable, "-m", "loam.cli", "estop", site, "--broker", "mqtts://supervisor@localhost"]
    topic = wire.topic("pad_north", None, "estop")
    try:
        assert subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30).returncode == 0
        assert subprocess.run(RESTART, shell=True, timeout=60).returncode == 0
        dz1 = client(8883, INSTALLED, "dz1", [topic])
        try:
            seen = got(dz1, topic, wait=5.0)
        finally:
            dz1.stop()
        assert seen and seen[0]["pressed"] is True
    finally:
        subprocess.run([*cmd, "--release"], env=env, capture_output=True, timeout=30)

