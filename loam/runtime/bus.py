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
import threading
import time
from typing import Any

try:
    import paho.mqtt.client as mqtt
except ImportError as e:  # pragma: no cover
    raise SystemExit("the Loam runtime needs paho-mqtt: pip install 'loam[run]'") from e


def parse_broker(text: str) -> tuple[str, int]:
    host, _, port = text.partition(":")
    return host or "localhost", int(port or 1883)


class Bus:
    """One MQTT connection with a receive queue, JSON publishing and an optional last will."""

    def __init__(self, broker: str, client_id: str, will: tuple[str, dict] | None = None):
        self.host, self.port = parse_broker(broker)
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)
        self.inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.subs: list[str] = []
        self.connected = threading.Event()
        if will:
            self.c.will_set(will[0], json.dumps(will[1]), qos=1, retain=True)
        self.c.on_message = self._on_message
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a: self.connected.clear()
        self.c.reconnect_delay_set(1, 2)

    def _on_connect(self, client, userdata, flags, rc, props=None) -> None:
        for t in self.subs:
            client.subscribe(t, qos=1)
        self.connected.set()

    def _on_message(self, client, userdata, msg) -> None:
        with contextlib.suppress(ValueError, UnicodeDecodeError):   # not ours: ignore malformed traffic
            self.inbox.put((msg.topic, json.loads(msg.payload)))

    def start(self, subs: list[str], timeout: float = 10.0) -> Bus:
        self.subs = subs
        self.c.connect(self.host, self.port, keepalive=5)
        self.c.loop_start()
        if not self.connected.wait(timeout):
            raise SystemExit(f"cannot reach the MQTT broker at {self.host}:{self.port}")
        return self

    def pub(self, topic: str, payload: dict, retain: bool = False, qos: int = 0) -> None:
        self.c.publish(topic, json.dumps(payload, separators=(",", ":")), qos=qos, retain=retain)

    def clear(self, topic: str) -> None:
        """Delete a retained message."""
        self.c.publish(topic, b"", qos=1, retain=True)

    def drain(self, timeout: float = 0.0) -> list[tuple[str, Any]]:
        out = []
        try:
            out.append(self.inbox.get(timeout=timeout) if timeout else self.inbox.get_nowait())
            while True:
                out.append(self.inbox.get_nowait())
        except queue.Empty:
            pass
        return out

    def stop(self) -> None:
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


class Clock:
    def __init__(self, t0: float | None = None, speed: float = 1.0, epoch: float | None = None):
        self.t0 = time.time() if t0 is None else t0
        self.speed = speed
        self.epoch = self.t0 if epoch is None else epoch

    @classmethod
    def from_msg(cls, m: dict) -> Clock:
        return cls(m["t0"], m["speed"], m["epoch"])

    def msg(self) -> dict:
        return {"t0": self.t0, "speed": self.speed, "epoch": self.epoch}

    def minutes(self) -> float:
        return (time.time() - self.t0) * self.speed / 60.0

    def ts(self, minutes: float | None = None) -> float:
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
        for item in others:                    # hand everything else back to the caller, in order
            bus.inbox.put(item)
    if clock is not None:
        return clock
    raise SystemExit(f"no clock on sim/{site}/clock; is `loam simulate-site` running?")
