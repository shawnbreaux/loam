"""The broker connection: TLS, passwords, and the access rules in examples/broker.

A real Mosquitto runs with the shipped example configuration, so the configuration in the documentation
is the one that is tested. Certificates are made on the spot with the `openssl` command.
"""
import getpass
import os
import shutil
import socket
import subprocess
import time

import pytest
from conftest import MOSQUITTO, ROOT

from loam import wire
from loam.runtime.bus import Broker, Bus, parse_broker

OPENSSL = shutil.which("openssl")
PASSWD = shutil.which("mosquitto_passwd")
USERS = {"controller": "c-secret", "dz1": "d-secret", "dz2": "e-secret", "supervisor": "s-secret",
         "sensors": "n-secret"}


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


def _openssl(*args, cwd):
    """Run one openssl command quietly; fail the test with its output if it fails."""
    r = subprocess.run([OPENSSL, *args], cwd=cwd, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


@pytest.fixture
def secure_broker(tmp_path):
    """Mosquitto with examples/broker/mosquitto.conf and its access rules, on a free port, with a fresh certificate
    authority and one password per user. Yields (port, path of the CA certificate).
    """
    if not (MOSQUITTO and OPENSSL and PASSWD):
        pytest.skip("needs mosquitto, mosquitto_passwd and openssl")
    d = tmp_path / "broker"
    d.mkdir()
    # A site certificate authority, and a broker certificate it signs for the name clients will connect to.
    # The extensions are the ones docs/RUNTIME.md gives: current TLS libraries reject a CA without key usage.
    _openssl("req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "2", "-subj", "/CN=Loam test CA",
             "-addext", "basicConstraints=critical,CA:TRUE", "-addext", "keyUsage=critical,keyCertSign,cRLSign",
             "-keyout", "ca.key", "-out", "ca.crt", cwd=d)
    _openssl("req", "-newkey", "rsa:2048", "-nodes", "-subj", "/CN=localhost", "-keyout", "server.key",
             "-out", "server.csr", cwd=d)
    (d / "san.cnf").write_text("subjectAltName=DNS:localhost,IP:127.0.0.1\nkeyUsage=digitalSignature,keyEncipherment\n"
                               "extendedKeyUsage=serverAuth\nsubjectKeyIdentifier=hash\n"
                               "authorityKeyIdentifier=keyid\n")
    _openssl("x509", "-req", "-in", "server.csr", "-CA", "ca.crt", "-CAkey", "ca.key", "-CAcreateserial",
             "-days", "2", "-extfile", "san.cnf", "-out", "server.crt", cwd=d)
    for i, (user, password) in enumerate(USERS.items()):
        subprocess.run([PASSWD, "-b", *(["-c"] if i == 0 else []), "passwords", user, password], cwd=d, check=True,
                       capture_output=True)
    shutil.copy(os.path.join(ROOT, "examples", "broker", "acl"), d / "acl")
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    with open(os.path.join(ROOT, "examples", "broker", "mosquitto.conf"), encoding="utf-8") as f:
        conf = f.read().replace("listener 8883", f"listener {port} 127.0.0.1")
    # Started by root, Mosquitto switches to the user `mosquitto`, who cannot read a temporary folder; keep the
    # user who runs the tests. (Ignored when the tests do not run as root.)
    (d / "mosquitto.conf").write_text(f"user {getpass.getuser()}\n{conf}")
    for name in ("server.key", "passwords", "acl"):          # Mosquitto refuses world-readable secrets
        os.chmod(d / name, 0o600)
    proc = subprocess.Popen([MOSQUITTO, "-c", "mosquitto.conf"], cwd=d, stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE, text=True)
    try:
        for _ in range(100):
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    break
            if proc.poll() is not None:
                why = proc.stderr.read()
                if "Unable to open config file" in why:
                    # Ubuntu's Mosquitto package is confined by AppArmor to /etc/mosquitto and cannot read a
                    # test folder. CI lifts the confinement (.github/workflows/ci.yml).
                    pytest.skip("this Mosquitto may not read files outside /etc/mosquitto (AppArmor)")
                raise AssertionError(why)
            time.sleep(0.05)
        yield port, str(d / "ca.crt")
    finally:
        proc.terminate()
        proc.wait(5)
        proc.stderr.close()


def client(port, ca, user, subs=(), password=None):
    """A connected client logged in as `user` over TLS."""
    env = {"LOAM_BROKER_PASSWORD": USERS[user] if password is None else password, "LOAM_BROKER_CA": ca}
    return Bus(f"mqtts://{user}@localhost:{port}", f"test-{user}-{time.time()}", env=env).start(list(subs), timeout=5)


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
    port, ca = secure_broker
    order = wire.topic("s", "dz1", "order")
    ctl, dz1 = client(port, ca, "controller"), client(port, ca, "dz1", [order])
    try:
        time.sleep(0.3)
        ctl.pub(order, {"n": 1}, qos=1)
        assert got(dz1, order) == [{"n": 1}]
    finally:
        ctl.stop()
        dz1.stop()
    with pytest.raises(SystemExit, match="refused the connection"):
        client(port, ca, "dz1", password="wrong")
    with pytest.raises(SystemExit, match="refused the connection"):          # no name at all
        Bus(f"mqtts://localhost:{port}", "anon", env={"LOAM_BROKER_CA": ca}).start([], timeout=5)
    with pytest.raises(SystemExit, match="CERTIFICATE_VERIFY_FAILED"):       # the system does not trust this CA
        Bus(f"mqtts://dz1@localhost:{port}", "noca", env={"LOAM_BROKER_PASSWORD": USERS["dz1"]}).start([], timeout=5)
    with pytest.raises(SystemExit, match="cannot reach"):                    # plain MQTT to the TLS port
        Bus(f"mqtt://dz1@localhost:{port}", "plain", env={"LOAM_BROKER_PASSWORD": USERS["dz1"]}).start([], timeout=2)


def test_access_rules_keep_machines_in_their_lane(secure_broker):
    """With the example access rules a machine can report its own state and read its own orders, and nothing more:
    it cannot order another machine (or itself), speak for another machine, or read another machine's orders.
    The supervisor can stop and resume but not order; sensors can report but not command.
    """
    port, ca = secure_broker
    t = lambda machine, kind: wire.topic("s", machine, kind)                 # noqa: E731
    ctl = client(port, ca, "controller", ["loam/v0/#"])
    dz1 = client(port, ca, "dz1", [t("dz1", "order"), t("dz2", "order"), t(None, "safety"), t(None, "estop")])
    dz2 = client(port, ca, "dz2", [t("dz2", "order")])
    sup, sen = client(port, ca, "supervisor"), client(port, ca, "sensors")
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
        # what must not
        dz1.pub(t("dz2", "order"), {"forged": 1}, qos=1)                     # order another machine
        dz1.pub(t("dz1", "order"), {"forged": 2}, qos=1)                     # order itself
        dz1.pub(t("dz2", "state"), {"forged": 3}, qos=1)                     # speak for another machine
        dz1.pub(t(None, "safety"), {"forged": 4}, qos=1)                     # fake the controller's heartbeat
        sup.pub(t("dz2", "order"), {"forged": 5}, qos=1)                     # the supervisor does not plan
        sen.pub(t("dz2", "instantAction"), {"forged": 6}, qos=1)             # a sensor commands nothing
        ctl.pub(t("dz2", "order"), {"order": 2}, qos=1)                      # the one genuine order
        assert got(dz2, t("dz2", "order")) == [{"order": 2}]
        time.sleep(0.5)
        seen = ctl.drain() + dz2.drain() + dz1.drain()
        assert not [m for _, m in seen if "forged" in m], seen
        assert not [m for tp, m in seen if tp == t("dz2", "order") and m != {"order": 2}]
    finally:
        for b in (ctl, dz1, dz2, sup, sen):
            b.stop()
