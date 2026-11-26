"""A small MQTT client and the shared simulation clock.

Messages are JSON. Callbacks run on paho's network thread, so every process
here pushes incoming messages onto a queue and handles them on its own thread.

The simulated site runs faster than real time. The world publishes a retained
clock message `sim/<site>/clock` = {"t0": wall seconds, "speed": factor,
"epoch": site epoch seconds}; every process converts wall time to site minutes
with it. A real site has no clock topic and runs at speed 1.
"""
from __future__ import annotations

import contextlib
import json
import queue
import sys
import threading
import time
from collections.abc import Iterator
from typing import Any

try:
    import paho.mqtt.client as mqtt
except ImportError as e:  # pragma: no cover
    raise SystemExit("the Loam runtime needs paho-mqtt: pip install 'loam[run]'") from e


def parse_broker(text: str) -> tuple[str, int]:
    """A broker address "host:port" as (host, port); the host defaults to localhost and the port to 1883, the standard
    MQTT port.
    """
    host, _, port = text.partition(":")
    return host or "localhost", int(port or 1883)


class Bus:
    """One MQTT connection with a receive queue, JSON publishing and an optional last will."""

    def __init__(self, broker: str, client_id: str, will: tuple[str, dict] | None = None):
        """Prepare a connection (it is opened by `start`).

        `client_id` must be unique on the broker. `will` is a (topic, message) pair the broker publishes for us,
        retained, if this client vanishes without saying goodbye: how a machine that loses its radio link gets
        reported.
        """
        self.host, self.port = parse_broker(broker)
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)
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
        subscriptions.
        """
        for t in self.subs:
            client.subscribe(t, qos=1)
        self.connected.set()

    def _on_message(self, client, userdata, msg) -> None:
        """Runs on paho's network thread: decode and queue the message for the owner's own thread."""
        with contextlib.suppress(ValueError, UnicodeDecodeError):   # not ours: ignore malformed traffic
            payload = json.loads(msg.payload)
            if isinstance(payload, dict):                           # every Loam message is a JSON object
                self.inbox.put((msg.topic, payload))

    def start(self, subs: list[str], timeout: float = 10.0) -> Bus:
        """Connect, subscribe to `subs` and start the network thread. Exits the process with a clear message if the
        broker cannot be reached within `timeout` seconds.
        """
        self.subs = subs
        try:
            self.c.connect(self.host, self.port, keepalive=5)
        except OSError as e:                          # refused, unreachable, unknown host
            self._close_wakeup()
            raise SystemExit(f"cannot reach the MQTT broker at {self.host}:{self.port}: {e}") from e
        self.c.loop_start()
        if not self.connected.wait(timeout):
            self.stop()                               # no network thread or sockets left behind
            raise SystemExit(f"cannot reach the MQTT broker at {self.host}:{self.port}")
        return self

    def pub(self, topic: str, payload: dict, retain: bool = False, qos: int = 0) -> None:
        """Publish a message as compact JSON.

        `qos=0` is fire-and-forget, right for state that is repeated anyway; `qos=1` is delivered at least once,
        for orders and one-off events. `retain` makes the broker keep the message for clients that subscribe later.
        """
        self.c.publish(topic, json.dumps(payload, separators=(",", ":")), qos=qos, retain=retain)

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
