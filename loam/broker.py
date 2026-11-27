"""`loam broker`: everything a Mosquitto broker needs to serve one site securely, in one folder.

On a real site anyone who can reach an open broker can publish an order. This writes, for the machines named in
the site program: a certificate authority and a broker certificate (made with the `openssl` command), a password
per client, the access rules, a Mosquitto configuration and an `install.sh` that puts them where the packaged
Mosquitto looks for them.

The file layout is not a matter of taste. On Ubuntu, Mosquitto is confined by AppArmor and can read only

    /etc/mosquitto/*   /etc/mosquitto/conf.d/**   /etc/mosquitto/certs/**   /etc/mosquitto/ca_certificates/**

and it reports a file anywhere else as "Unable to open config file". Mosquitto reads its configuration as root
and everything else after it has switched to the user `mosquitto`, so the key, the passwords and the rules must
belong to that user; it warns about (and will one day refuse) such files when anyone else can read them.

Passwords are written in Mosquitto's own format (`mosquitto_passwd`: PBKDF2-HMAC-SHA512, 101 rounds) here, so
that no password ever appears on a command line.
"""
from __future__ import annotations

import base64
import hashlib
import ipaddress
import os
import secrets
import shlex
import shutil
import subprocess
from collections.abc import Sequence
from typing import Any

ETC = "/etc/mosquitto"
CA_DAYS, SERVER_DAYS = 3650, 825     # the authority lasts ten years; the broker's certificate is reissued by a rerun
ROLES = ("controller", "supervisor", "sensors", "frigate", "rtl433")       # the clients that are not machines

ACL = """\
# Who may publish and read what on a Loam site (written by `loam broker`; rerun it rather than editing).
# Each machine logs in under its own machine name. Whatever is not allowed here is refused.
#
# Without these rules any client could publish an order to any machine.

# The controller runs the site: it may publish and read everything under loam/.
user controller
topic readwrite loam/v0/#

# The supervisor's laptop (dashboard, `loam resume`, `loam estop`).
user supervisor
topic read loam/v0/#
topic write loam/v0/+/safety
topic write loam/v0/+/+/instantAction
topic write loam/v0/+/estop

# Site sensors (`loam sense`, `loam terrain`, the e-stop radio) report; they command nothing.
# They read the camera system's and the weather receiver's own topics.
user sensors
topic write loam/v0/+/detection
topic write loam/v0/+/weather
topic write loam/v0/+/terrain
topic write loam/v0/+/estop
topic read frigate/#
topic write frigate/onConnect
topic read rtl_433/#

# The camera system and the weather receiver publish under their own prefixes only.
user frigate
topic readwrite frigate/#
user rtl433
topic write rtl_433/#

# Every machine: read its own orders and the site-wide safety topics, report its own state.
# A machine cannot publish an order, not even to itself, and cannot speak for another machine.
"""

MACHINE_ACL = """user {name}
topic read loam/v0/+/{name}/order
topic read loam/v0/+/{name}/instantAction
topic read loam/v0/+/safety
topic read loam/v0/+/estop
topic write loam/v0/+/{name}/state
topic write loam/v0/+/{name}/connection
"""

CONF = """\
# Loam site broker (written by `loam broker`; rerun it rather than editing this file).
# Encrypted, every client named, each allowed only its own topics. A listener defined here replaces Mosquitto's
# default one, so nothing is served without TLS.
listener {port}
certfile {etc}/certs/loam-server.crt
keyfile {etc}/certs/loam-server.key
tls_version tlsv1.2
allow_anonymous false
password_file {etc}/loam-passwords
acl_file {etc}/loam-acl
"""

INSTALL = """\
#!/bin/sh
# Installs this site's broker files where the packaged Mosquitto may read them, and restarts it.
# Written by `loam broker`. Usage:  sudo sh install.sh [user Mosquitto runs as; default mosquitto]
set -eu
here=$(cd "$(dirname "$0")" && pwd)
etc={etc_sh}
owner=${{1:-mosquitto}}
install -d "$etc/conf.d" "$etc/certs" "$etc/ca_certificates"
install -m 644 "$here/ca.crt" "$etc/ca_certificates/loam-ca.crt"
install -m 644 "$here/server.crt" "$etc/certs/loam-server.crt"
install -m 644 "$here/loam.conf" "$etc/conf.d/loam.conf"
# Mosquitto reads these three after it has given up root, and wants them closed to everyone else.
install -m 600 "$here/server.key" "$etc/certs/loam-server.key"
install -m 600 "$here/passwords" "$etc/loam-passwords"
install -m 600 "$here/acl" "$etc/loam-acl"
if [ "$(id -u)" = 0 ]; then
    chown "$owner:" "$etc/certs/loam-server.key" "$etc/loam-passwords" "$etc/loam-acl"
fi
if [ "$etc" = /etc/mosquitto ] && [ -d /run/systemd/system ]; then
    systemctl restart mosquitto || {{ tail -n 20 /var/log/mosquitto/mosquitto.log >&2; exit 1; }}
    echo "Mosquitto restarted with the Loam site configuration (port {port})."
else
    echo "Installed in $etc. Restart Mosquitto to use it (port {port})."
fi
"""


def password_line(user: str, password: str) -> str:
    """One line of a Mosquitto password file: `user:$7$rounds$salt$hash`, as `mosquitto_passwd` writes it."""
    salt = os.urandom(12)
    digest = hashlib.pbkdf2_hmac("sha512", password.encode(), salt, 101, 64)
    return f"{user}:$7$101${base64.b64encode(salt).decode()}${base64.b64encode(digest).decode()}"


def _openssl(*args: str, cwd: str) -> None:
    """Run one `openssl` command in `cwd`, with new files closed to other users; exit with its message if it fails."""
    exe = shutil.which("openssl")
    if exe is None:
        raise SystemExit("loam broker needs the `openssl` command to make certificates")
    done = subprocess.run([exe, *args], cwd=cwd, capture_output=True, text=True, umask=0o077)
    if done.returncode:
        raise SystemExit(f"openssl {args[0]} failed: {done.stderr.strip()}")


def _secret(path: str, text: str) -> None:
    """Write a file that only its owner can read, from the first byte on."""
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), "w", encoding="utf-8") as f:
        f.write(text)
    os.chmod(path, 0o600)                        # also when the file was already there, more open


def _name(text: str) -> str:
    """A certificate name entry for a host name or an IP address."""
    try:
        return f"IP:{ipaddress.ip_address(text)}"
    except ValueError:
        return f"DNS:{text}"


def site_broker(model: Any, out: str, host: str, addresses: Sequence[str] = (), etc: str = ETC,
                port: int = 8883) -> dict[str, Any]:
    """Write the broker folder `out` for the site program `model`; return what was made.

    `host` is the name (or address) clients will connect to; `addresses` are further names or addresses of the
    same computer. Each must be in the certificate, or clients refuse the connection. `etc` is where
    `install.sh` will put the files and what the configuration names.

    A rerun keeps the certificate authority and every existing password, so the clients already set up go on
    working; it adds a password and access rules for each new machine and reissues the broker's certificate.
    Returns `{"out", "host", "port", "users", "new", "ca_created"}`.
    """
    users = [*ROLES, *model.machines]
    clash = sorted(set(ROLES) & set(model.machines))
    if clash:
        raise SystemExit(f"machine name(s) {', '.join(clash)} are reserved for the broker's own users; rename them")
    os.makedirs(os.path.join(out, "clients"), mode=0o700, exist_ok=True)
    os.chmod(out, 0o700)                         # the folder holds the authority's key and every password

    ca_created = not (os.path.exists(os.path.join(out, "ca.key")) and os.path.exists(os.path.join(out, "ca.crt")))
    if ca_created:
        # Current TLS libraries reject an authority without the key-usage extension.
        _openssl("req", "-x509", "-newkey", "rsa:3072", "-nodes", "-days", str(CA_DAYS),
                 "-subj", f"/CN=Loam site CA ({model.site['name']})", "-addext", "basicConstraints=critical,CA:TRUE",
                 "-addext", "keyUsage=critical,keyCertSign,cRLSign", "-keyout", "ca.key", "-out", "ca.crt", cwd=out)
    names = list(dict.fromkeys(_name(n) for n in (host, *addresses, "localhost", "127.0.0.1")))
    with open(os.path.join(out, "server.ext"), "w", encoding="utf-8") as f:
        f.write(f"subjectAltName={','.join(names)}\nkeyUsage=digitalSignature,keyEncipherment\n"
                "extendedKeyUsage=serverAuth\nsubjectKeyIdentifier=hash\nauthorityKeyIdentifier=keyid\n")
    _openssl("req", "-newkey", "rsa:2048", "-nodes", "-subj", f"/CN={host}", "-keyout", "server.key",
             "-out", "server.csr", cwd=out)
    _openssl("x509", "-req", "-in", "server.csr", "-CA", "ca.crt", "-CAkey", "ca.key", "-CAcreateserial",
             "-days", str(SERVER_DAYS), "-extfile", "server.ext", "-out", "server.crt", cwd=out)
    for leftover in ("server.csr", "server.ext"):
        os.remove(os.path.join(out, leftover))
    for public in ("ca.crt", "server.crt"):                  # certificates are not secrets
        os.chmod(os.path.join(out, public), 0o644)

    new, lines = [], []
    for user in users:
        path = os.path.join(out, "clients", f"{user}.password")
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                password = f.readline().strip()
        else:
            password = secrets.token_urlsafe(24)
            _secret(path, password + "\n")
            new.append(user)
        lines.append(password_line(user, password))
    _secret(os.path.join(out, "passwords"), "\n".join(lines) + "\n")
    _secret(os.path.join(out, "acl"), ACL + "".join(MACHINE_ACL.format(name=m) for m in model.machines))
    for name, text in (("loam.conf", CONF), ("install.sh", INSTALL)):
        with open(os.path.join(out, name), "w", encoding="utf-8") as f:
            f.write(text.format(etc=etc, etc_sh=shlex.quote(etc), port=port))
    return {"out": out, "host": host, "port": port, "users": users, "new": new, "ca_created": ca_created}
