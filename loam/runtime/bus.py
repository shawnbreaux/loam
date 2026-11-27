"""A small MQTT client and the shared simulation clock.

Messages are JSON. Callbacks run on paho's network thread, so every process
here pushes incoming messages onto a queue and handles them on its own thread.

The simulated site runs faster than real time. The world publishes a retained
clock message `sim/<site>/clock` = {"t0": wall seconds, "speed": factor,
"epoch": site epoch seconds}; every process converts wall time to site minutes
with it. A real site has no clock topic and runs at speed 1.

A broker address is "host:port", or a URL: `mqtt://host:1883` (plain) or
`mqtts://user@host:8883` (TLS). Secrets never go on the command line, where
every user of the computer could read them with `ps`; they come from the
environment:

    LOAM_BROKER_USER           user name (if the URL has none)
    LOAM_BROKER_PASSWORD       password, or
    LOAM_BROKER_PASSWORD_FILE  a file holding it (systemd credentials, Docker secrets)
    LOAM_BROKER_CA             CA certificate that signed the broker's (default: the system's)
    LOAM_BROKER_CERT, LOAM_BROKER_KEY   this client's certificate and key, for mutual TLS
"""
from __future__ import annotations

import contextlib
import json
import os
import queue
import socket
import ssl
import sys
import threading
import time
from collections.abc import Iterator, Mapping
from typing import Any, NamedTuple

try:
    import paho.mqtt.client as mqtt
except ImportError as e:  # pragma: no cover
    raise SystemExit("the Loam runtime needs paho-mqtt: pip install 'loam[run]'") from e


class Broker(NamedTuple):
    """Where the broker is and how to reach it: host, port, whether to use TLS, and the user name (or None)."""
    host: str
    port: int
    tls: bool
    user: str | None


def parse_broker(text: str) -> Broker:
    """A broker address as a `Broker`.

    Accepts "host:port", `mqtt://[user@]host[:port]` and `mqtts://[user@]host[:port]`. The host defaults to
    localhost; the port to 1883, or 8883 with TLS (the two standard MQTT ports). A password in the address is
    refused: command lines are visible to every user of the computer.
    """
    scheme, sep, rest = text.partition("://")
    if not sep:
        scheme, rest = "mqtt", text
    if scheme not in ("mqtt", "mqtts"):
        raise SystemExit(f"broker address {text!r}: use host:port, mqtt://host or mqtts://host")
    user, at, hostport = rest.rpartition("@")
    if ":" in user:
        raise SystemExit("put the broker password in LOAM_BROKER_PASSWORD or LOAM_BROKER_PASSWORD_FILE, "
                         "not in the address")
    host, _, port = hostport.rstrip("/").partition(":")
    tls = scheme == "mqtts"
    try:
        return Broker(host or "localhost", int(port or (8883 if tls else 1883)), tls, user if at else None)
    except ValueError:
        raise SystemExit(f"broker address {text!r}: the port must be a number") from None


def _password(env: Mapping[str, str]) -> str | None:
    """The broker password from the environment: LOAM_BROKER_PASSWORD, else the first line of the file named by
    LOAM_BROKER_PASSWORD_FILE.
    """
    if env.get("LOAM_BROKER_PASSWORD"):
        return env["LOAM_BROKER_PASSWORD"]
    path = env.get("LOAM_BROKER_PASSWORD_FILE")
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return f.readline().rstrip("\r\n")
    except OSError as e:
        raise SystemExit(f"cannot read the broker password file: {e}") from e


class Bus:
    """One MQTT connection with a receive queue, JSON publishing and an optional last will."""

    def __init__(self, broker: str, client_id: str, will: tuple[str, dict] | None = None, raw: bool = False,
                 env: Mapping[str, str] | None = None):
        """Prepare a connection (it is opened by `start`).

        `client_id` must be unique on the broker. `will` is a (topic, message) pair the broker publishes for us,
        retained, if this client vanishes without saying goodbye: how a machine that loses its radio link gets
        reported. With `raw`, payloads that are not JSON objects are delivered too, as text: other programs'
        status topics ("online", "offline") are like that. `env` supplies the credentials (see the module
        docstring); it defaults to the process environment.
        """
        if env is None:
            env = os.environ
        self.host, self.port, tls, user = parse_broker(broker)
        self.raw = raw
        self.refused: str | None = None               # why the broker turned us away, if it did
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)
        user = user or env.get("LOAM_BROKER_USER")
        if user:
            self.c.username_pw_set(user, _password(env))
        if tls:
            # Verify the broker's certificate and its host name; never fall back to an unverified connection.
            # TLS 1.2 is the oldest version accepted.
            try:
                ctx = ssl.create_default_context(cafile=env.get("LOAM_BROKER_CA") or None)
                ctx.minimum_version = ssl.TLSVersion.TLSv1_2
                if env.get("LOAM_BROKER_CERT"):
                    ctx.load_cert_chain(env["LOAM_BROKER_CERT"], env.get("LOAM_BROKER_KEY") or None)
            except (OSError, ssl.SSLError) as e:
                self._close_wakeup()
                raise SystemExit(f"cannot load the broker TLS certificates: {e}") from e
            self.c.tls_set_context(ctx)
        self.inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.pushback: list[tuple[str, Any]] = []     # handed back by the consumer; drained first, in order
        self.subs: list[str] = []
        self.connected = threading.Event()
        if will:
            self.c.will_set(will[0], json.dumps(will[1]), qos=1, retain=True)
        self.c.on_message = self._on_message
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a: self.connected.clear()
        self.c.reconnect_delay_set(1, 2)

    def _on_connect(self, client, userdata, flags, rc, props=None) -> None:
        """Runs on every connect and reconnect: subscribe again, since the broker forgets a clean session's
        subscriptions. A refusal is kept for `start` to report.
        """
        if getattr(rc, "is_failure", False):          # wrong password, not authorised, ...
            self.refused = str(rc)
            return
        for t in self.subs:
            client.subscribe(t, qos=1)
        self.connected.set()

    def _on_message(self, client, userdata, msg) -> None:
        """Runs on paho's network thread: decode and queue the message for the owner's own thread."""
        with contextlib.suppress(ValueError, UnicodeDecodeError):   # not ours: ignore malformed traffic
            try:
                payload = json.loads(msg.payload)
            except ValueError:
                if not self.raw:
                    raise
                payload = msg.payload.decode("utf-8")
            if isinstance(payload, dict) or self.raw:               # every Loam message is a JSON object
                self.inbox.put((msg.topic, payload))

    def start(self, subs: list[str], timeout: float = 10.0) -> Bus:
        """Connect, subscribe to `subs` and start the network thread. Exits the process with a clear message if the
        broker cannot be reached within `timeout` seconds.
        """
        self.subs = subs
        try:
            self.c.connect(self.host, self.port, keepalive=5)
        except OSError as e:                          # refused, unreachable, unknown host, TLS handshake failed
            # paho does not close the socket when the TLS handshake fails; it is left to the garbage collector.
            # Close it now: the socket is a local variable of the frames the error passed through.
            tb = e.__traceback__
            while tb is not None:
                for v in list(tb.tb_frame.f_locals.values()):
                    if isinstance(v, socket.socket):
                        v.close()
                tb = tb.tb_next
            self._close_wakeup()
            raise SystemExit(f"cannot reach the MQTT broker at {self.host}:{self.port}: {e}") from e
        self.c.loop_start()
        end = time.time() + timeout
        while not self.connected.wait(0.05) and self.refused is None and time.time() < end:
            pass
        if not self.connected.is_set():
            self.stop()                               # no network thread or sockets left behind
            if self.refused:
                raise SystemExit(f"the MQTT broker at {self.host}:{self.port} refused the connection: {self.refused}")
            raise SystemExit(f"cannot reach the MQTT broker at {self.host}:{self.port}: no answer")
        return self

    def pub(self, topic: str, payload: dict, retain: bool = False, qos: int = 0, wait: float = 0.0) -> None:
        """Publish a message as compact JSON.

        `qos=0` is fire-and-forget, right for state that is repeated anyway; `qos=1` is delivered at least once,
        for orders and one-off events. `retain` makes the broker keep the message for clients that subscribe later.
        `wait` blocks up to that many seconds until the broker has the message: for a program that publishes
        and exits.
        """
        info = self.c.publish(topic, json.dumps(payload, separators=(",", ":")), qos=qos, retain=retain)
        if wait:
            with contextlib.suppress(RuntimeError, ValueError):     # not connected: there is nothing to wait for
                info.wait_for_publish(wait)

    def poke(self, topic: str) -> None:
        """Publish an empty message: how some programs are asked to report."""
        self.c.publish(topic, b"", qos=0)

    def clear(self, topic: str) -> None:
        """Delete a retained message."""
        self.c.publish(topic, b"", qos=1, retain=True)

    def drain(self, timeout: float = 0.0) -> list[tuple[str, Any]]:
        """Every message received so far, oldest first. With a `timeout`, wait up to that long for the first one, so a
        loop built on `drain` does not spin.
        """
        out, self.pushback = self.pushback, []
        if out:
            timeout = 0.0
        try:
            out.append(self.inbox.get(timeout=timeout) if timeout else self.inbox.get_nowait())
            while True:
                out.append(self.inbox.get_nowait())
        except queue.Empty:
            pass
        return out

    def stop(self) -> None:
        """Disconnect cleanly (no last will is sent) and release the network thread and sockets."""
        self.c.disconnect()                     # the network loop sends DISCONNECT, then exits
        self.c.loop_stop()
        sock = self.c.socket()                  # left open if paho was part-way through a reconnect
        if sock is not None:
            sock.close()
        self._close_wakeup()

    def _close_wakeup(self) -> None:
        """paho keeps a wake-up socket pair until the client is garbage-collected; close it now, so a
        process that reconnects many times (a machine on a flaky radio) does not accumulate descriptors."""
        reset = getattr(self.c, "_reset_sockets", None)
        if reset is not None:
            reset(sockpair_only=True)

    def sever(self) -> None:
        """Drop the connection without a clean disconnect, so the broker sends the last will."""
        self.c.loop_stop()
        sock = self.c.socket()
        if sock is not None:
            sock.close()
        self._close_wakeup()


@contextlib.contextmanager
def tolerant(topic: str) -> Iterator[None]:
    """Handle one message; a malformed one (missing or ill-typed fields) is reported and skipped, so
    nobody on the broker can crash a site process with a bad message."""
    try:
        yield
    except (KeyError, TypeError, ValueError, AttributeError, IndexError) as e:
        print(f"loam: ignored a malformed message on {topic}: {e!r}", file=sys.stderr, flush=True)


class Clock:
    """The site clock: converts wall time to *site minutes* since the job started.

    A simulated site runs `speed` times faster than real time, and every process must agree on the time, so they
    all build their clock from the one message the world publishes. `epoch` is the calendar time of minute 0, used
    for message timestamps.
    """
    def __init__(self, t0: float | None = None, speed: float = 1.0, epoch: float | None = None):
        """By default: a real-time clock that starts now."""
        self.t0 = time.time() if t0 is None else t0
        self.speed = speed
        self.epoch = self.t0 if epoch is None else epoch

    @classmethod
    def from_msg(cls, m: dict) -> Clock:
        """The clock described by a `sim/<site>/clock` message."""
        return cls(m["t0"], m["speed"], m["epoch"])

    def msg(self) -> dict:
        """This clock as a message other processes can build theirs from."""
        return {"t0": self.t0, "speed": self.speed, "epoch": self.epoch}

    def minutes(self) -> float:
        """Site minutes since the job started."""
        return (time.time() - self.t0) * self.speed / 60.0

    def ts(self, minutes: float | None = None) -> float:
        """A site time (now, unless `minutes` is given) as seconds since 1970, for message timestamps."""
        return self.epoch + 60.0 * (self.minutes() if minutes is None else minutes)

    def wall(self, minutes: float) -> float:
        """Wall seconds for a span of site minutes."""
        return minutes * 60.0 / self.speed


def wait_clock(bus: Bus, site: str, timeout: float = 30.0) -> Clock:
    """Block until the retained clock message arrives (subscribe to sim/<site>/clock first)."""
    end, others = time.time() + timeout, []
    clock: Clock | None = None
    try:
        while clock is None and time.time() < end:
            for topic, msg in bus.drain(0.2):
                if topic == f"sim/{site}/clock" and clock is None:
                    clock = Clock.from_msg(msg)
                else:
                    others.append((topic, msg))
    finally:
        bus.pushback = others + bus.pushback   # hand everything else back, ahead of anything newer
    if clock is not None:
        return clock
    raise SystemExit(f"no clock on sim/{site}/clock; is `loam simulate-site` running?")
