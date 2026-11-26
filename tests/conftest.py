import os
import shutil
import socket
import subprocess
import time

import pytest

from loam.checker import check, has_errors
from loam.parser import parse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES = os.path.join(ROOT, "examples")
EXAMPLE = os.path.join(EXAMPLES, "pad_north.loam")

MINI = """loam 0.0.1
site s { crs "LOCAL:ft"; units ft; design "d.xml" }
probe p { kind uav; rate 10 min; accuracy 0.1 ft }
machine d1 { class dozer; blade 10 ft }
zone A { role cut; area rect (0, 0) (10, 10) }
task t { op cut; zone A; by class dozer; spoil -> A; until residual(A) < 0.2 ft }
"""


def read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def errors(src: str) -> list[str]:
    _, diags = check(parse(src))
    return [d.msg for d in diags if d.level == "error"]


@pytest.fixture(scope="session")
def example_src() -> str:
    return read(EXAMPLE)


@pytest.fixture(scope="session")
def example_model(example_src):
    model, diags = check(parse(example_src))
    assert not has_errors(diags), [d.msg for d in diags]
    return model


MOSQUITTO = shutil.which("mosquitto") or ("/usr/sbin/mosquitto" if os.path.exists("/usr/sbin/mosquitto") else None)


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
