"""Shared test setup: the example program, a private MQTT broker, and the leak check.

Run the tests with `python -m pytest tests/`. Tests that need a program that is not
installed (Mosquitto, ArduPilot, ROS 2, Project Chrono, RTKLIB) skip themselves and
say what they need.
"""
import contextlib
import functools
import gc
import json
import os
import re
import shutil
import socket
import subprocess
import threading
import time

import pytest

from loam.checker import check, has_errors
from loam.parser import parse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES = os.path.join(ROOT, "examples")
EXAMPLE = os.path.join(EXAMPLES, "pad_north.loam")

# The smallest complete Loam program. Tests change one thing in it and look at what the checker says.
MINI = """loam 0.0.1
site s { crs "LOCAL:ft"; units ft; design "d.xml" }
probe p { kind uav; rate 10 min; accuracy 0.1 ft }
machine d1 { class dozer; blade 10 ft }
zone A { role cut; area rect (0, 0) (10, 10) }
task t { op cut; zone A; by class dozer; spoil -> A; until residual(A) < 0.2 ft }
"""


def read(path: str) -> str:
    """A text file's contents."""
    with open(path, encoding="utf-8") as f:
        return f.read()


def errors(src: str) -> list[str]:
    """The checker's error messages for a Loam program given as text."""
    _, diags = check(parse(src))
    return [d.msg for d in diags if d.level == "error"]


@pytest.fixture(scope="session")
def example_src() -> str:
    """The text of the example program, `examples/pad_north.loam`."""
    return read(EXAMPLE)


@pytest.fixture(scope="session")
def example_model(example_src):
    """The example program, parsed and checked. Tests must not change it: it is shared by the whole run."""
    model, diags = check(parse(example_src))
    assert not has_errors(diags), [d.msg for d in diags]
    return model


MOSQUITTO = shutil.which("mosquitto") or ("/usr/sbin/mosquitto" if os.path.exists("/usr/sbin/mosquitto") else None)


@functools.cache
def _wire_validator(kind: str):
    """A validator for one message kind of the published Loam Wire JSON Schema (read once per kind)."""
    jsonschema = pytest.importorskip("jsonschema")
    with open(os.path.join(ROOT, "schemas", "wire", "loam-wire-0.0.1.schema.json"), encoding="utf-8") as f:
        defs = json.load(f)["$defs"]
    assert kind in defs, f"the Wire schema does not define {kind!r}"
    return jsonschema.Draft202012Validator({"$defs": defs, "$ref": f"#/$defs/{kind}"})


def wire_valid(kind: str, payload: dict) -> None:
    """Check a message against one definition in the published Loam Wire JSON Schema; raises if it does not fit."""
    _wire_validator(kind).validate(payload)


@pytest.fixture
def mqtt_broker():
    """A private Mosquitto broker on a free port; yields 'host:port'. Skips when Mosquitto is absent."""
    if MOSQUITTO is None:
        pytest.skip("needs the mosquitto broker")
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    proc = subprocess.Popen([MOSQUITTO, "-p", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                break
        time.sleep(0.05)
    yield f"127.0.0.1:{port}"
    proc.terminate()
    proc.wait(5)


# -- leak check: every test must leave no thread, child process or open file behind ------------------------
# Read-only library data that a library opens once and keeps for the life of the process is not a leak.
CACHES = re.compile(r"(proj\.db|\.(ttf|otf|ttc))$")


def _resources() -> tuple[dict, dict, dict]:
    """What this process holds right now: open files (by descriptor), child processes and live threads."""
    fds = {}
    if os.path.isdir("/proc/self/fd"):
        for f in os.listdir("/proc/self/fd"):
            with contextlib.suppress(OSError):
                fds[f] = os.readlink(f"/proc/self/fd/{f}")
    out = subprocess.run(["ps", "-o", "pid=,args=", "--ppid", str(os.getpid())], capture_output=True, text=True,
                         check=False).stdout
    kids = {ln.split()[0]: ln.strip()[:100] for ln in out.splitlines() if ln.strip() and "ps -o" not in ln}
    threads = {t.ident: t.name for t in threading.enumerate() if t.is_alive()}
    return fds, kids, threads


def _new(before: tuple, after: tuple) -> list[str]:
    """What is held `after` that was not held `before`, as lines for the failure message."""
    fds = sorted({v for k, v in after[0].items() if k not in before[0] and not CACHES.search(v)})
    kids = sorted(v for k, v in after[1].items() if k not in before[1])
    threads = sorted(v for k, v in after[2].items() if k not in before[2])
    return [f"{what}: {items}" for what, items in (("files", fds), ("processes", kids), ("threads", threads))
            if items]


@pytest.fixture(autouse=True)
def no_leaks(request):
    """Runs around every test: fail it if it leaves a thread, a child process or an open file behind.

    Earthmoving controllers run for days, so a leak per order is a fault, not a nuisance. Set LOAM_LEAKCHECK=0 to
    switch the check off (for example under a debugger, which opens files of its own).
    """
    if os.environ.get("LOAM_LEAKCHECK", "1") == "0":
        yield
        return
    gc.collect()
    before = _resources()
    yield
    for _ in range(20):                       # shutdown threads and child processes may take a moment
        gc.collect()
        left = _new(before, _resources())
        if not left:
            return
        time.sleep(0.1)
    pytest.fail(f"{request.node.nodeid} left resources behind: " + "; ".join(left), pytrace=False)
