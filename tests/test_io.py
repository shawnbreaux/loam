"""LandXML, wire messages, CLI."""
import json
import os
import subprocess
import sys

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES, ROOT, read

from loam import wire
from loam.landxml import rasterize, read_tin, write_grid_tin

SCHEMA = os.path.join(ROOT, "schemas", "wire", "loam-wire-0.0.1.schema.json")


def test_tin_reproduces_a_plane_exactly(tmp_path):
    xs, ys = np.arange(0, 51, 10.0), np.arange(0, 31, 10.0)
    gx, gy = np.meshgrid(xs, ys)
    plane = 100 + 0.02 * gx - 0.01 * gy
    path = str(tmp_path / "p.xml")
    write_grid_tin(path, "FG", xs, ys, plane)
    pts, faces, name = read_tin(path, "FG")
    assert name == "FG" and len(faces) == 2 * 5 * 3
    z = rasterize(pts, faces, 0, 0, 1.0, 50, 30)
    cx, cy = np.meshgrid(np.arange(50) + 0.5, np.arange(30) + 0.5)
    np.testing.assert_allclose(z, 100 + 0.02 * cx - 0.01 * cy, atol=2e-3)   # file stores 3 decimals


def test_tin_errors(tmp_path):
    path = str(tmp_path / "p.xml")
    write_grid_tin(path, "FG", np.array([0.0, 1]), np.array([0.0, 1]), np.zeros((2, 2)))
    with pytest.raises(ValueError, match="no surface named"):
        read_tin(path, "EG")
    pts, faces, _ = read_tin(path)
    assert np.isnan(rasterize(pts, faces, 0, 0, 1.0, 3, 3)).any()   # grid larger than the TIN


def _validate(kind, payload):
    jsonschema = pytest.importorskip("jsonschema")
    with open(SCHEMA, encoding="utf-8") as f:
        defs = json.load(f)["$defs"]
    jsonschema.validate(payload, {"$defs": defs, "$ref": f"#/$defs/{kind}"})


def test_every_builder_matches_schema():
    _validate("order", wire.order("s", "m", 1, "o", 0, "t", "cut", [[0, 0], [1, 0], [1, 1]],
                                  {"surface": "design"}, {"maxCut": 0.1}))
    _validate("instantAction", wire.instant_action("s", "m", 2, "stop", "person"))
    _validate("state", wire.state("s", "m", 3, None, 1.0, 2.0, 0.0, False, 0.0))
    _validate("connection", wire.connection("s", "m", 4, "online"))
    _validate("safety", wire.safety("s", 5))
    z = np.arange(6, dtype=float).reshape(2, 3)
    msg = wire.terrain("s", "p", 6, (0, 0), 1.0, z.shape, data=wire.encode_heights(z), sigma=0.01)
    _validate("terrain", msg)
    np.testing.assert_array_equal(wire.decode_heights(msg["data"], (2, 3)), z)


def test_sample_wire_log_matches_schema():
    n = 0
    with open(os.path.join(EXAMPLES, "pad_north_wire.jsonl"), encoding="utf-8") as f:
        for line in f:
            msg = json.loads(line)
            _validate(msg["topic"].rsplit("/", 1)[1], msg["payload"])
            n += 1
    assert n > 100


class Result:
    def __init__(self, returncode, stdout, stderr):
        self.returncode, self.stdout, self.stderr = returncode, stdout, stderr


def run_cli(*args):
    """Run the CLI in-process (so coverage sees it) and capture exit code and output."""
    import contextlib
    import io

    from loam.cli import main
    out, err = io.StringIO(), io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            main(list(args))
        except SystemExit as e:
            code = int(e.code or 0)
    return Result(code, out.getvalue(), err.getvalue())


def test_cli_entry_point_runs_as_module():
    r = subprocess.run([sys.executable, "-m", "loam.cli", "check", EXAMPLE], capture_output=True, text=True,
                       cwd=ROOT, check=False)
    assert r.returncode == 0 and "ok" in r.stdout


def test_cli_exit_codes(tmp_path):
    assert run_cli("check", EXAMPLE).returncode == 0
    bad = tmp_path / "bad.loam"
    bad.write_text(read(EXAMPLE).replace("rate        5 min", "rate        5"))
    r = run_cli("check", str(bad))
    assert r.returncode == 1 and ":20:15: error: probe rate needs a unit" in r.stderr
    bad.write_text("loam 0.0.1\nsite {")
    assert run_cli("check", str(bad)).returncode == 2
    assert run_cli("check", str(tmp_path / "missing.loam")).returncode == 2


def test_cli_compile_and_sim(tmp_path):
    out = tmp_path / "plan.json"
    assert run_cli("compile", EXAMPLE, "-o", str(out)).returncode == 0
    assert json.loads(out.read_text())["task_order"][0] == "strip_A"
    r = run_cli("sim", EXAMPLE, "--json", "--wire", str(tmp_path / "w.jsonl"))
    assert r.returncode == 0 and json.loads(r.stdout)["finished"]
    r = run_cli("sim", EXAMPLE, "--no-events", "--plot", str(tmp_path / "p.png"))
    assert r.returncode == 0 and "volume ledger error" in r.stdout and (tmp_path / "p.png").stat().st_size > 1000
    assert "Work, in dependency order" in run_cli("explain", EXAMPLE).stdout
    assert json.loads(run_cli("compile", EXAMPLE).stdout)["loam"] == "0.0.1"
