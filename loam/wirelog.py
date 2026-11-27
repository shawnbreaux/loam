"""A flight recorder: every Loam Wire message, as it was seen, in one MCAP file.

MCAP (MIT licence) is the log format of ROS 2 bags. A recording made here opens in the free log viewers
(Lichtblick, PlotJuggler) on the same time line as a machine's own ROS 2 bag, which is how a field test is
reviewed afterwards: what was ordered, what each machine reported, what the sensors saw, and when.

Each MQTT topic becomes one MCAP channel carrying the messages as JSON, unchanged.
"""
from __future__ import annotations

import json
import time
from typing import Any

from . import __version__, wire


class McapLog:
    """An MCAP file being written."""

    def __init__(self, path: str):
        """Open `path` for writing. Needs the `mcap` package: pip install 'loam-lang[mcap]'."""
        try:
            from mcap.writer import Writer
        except ImportError as e:
            raise ValueError("recording to MCAP needs the mcap package: pip install 'loam-lang[mcap]'") from e
        self.f = open(path, "wb")  # noqa: SIM115  (closed in `close`)
        self.w: Any = Writer(self.f)
        self.w.start(profile="", library=f"loam {__version__}")
        self.channels: dict[str, int] = {}
        self.count = 0
        # Viewers want a schema per channel. Loam's messages are plain JSON objects; their exact shapes are in
        # schemas/wire, which a viewer does not need in order to plot the fields.
        self.schema = self.w.register_schema(name="loam.wire", encoding="jsonschema", data=b'{"type":"object"}')

    def add(self, topic: str, msg: dict, now: float | None = None) -> None:
        """Record one message.

        The log time is when it was seen (`now`, seconds since 1970); the publish time is the message's own
        timestamp when it has one. The difference between the two is the delay on the way, or a wrong clock.
        """
        if topic not in self.channels:
            self.channels[topic] = self.w.register_channel(topic=topic, message_encoding="json",
                                                           schema_id=self.schema)
        seen = time.time() if now is None else now
        try:
            sent = wire.parse_ts(msg["timestamp"])
        except (KeyError, TypeError, ValueError):
            sent = seen
        self.w.add_message(channel_id=self.channels[topic], log_time=int(seen * 1e9), publish_time=int(sent * 1e9),
                           data=json.dumps(msg, separators=(",", ":")).encode(), sequence=self.count)
        self.count += 1

    def close(self) -> None:
        """Write the file's index and close it. A file that was not closed can still be read up to the last
        complete chunk.
        """
        try:
            self.w.finish()
        finally:
            self.f.close()
