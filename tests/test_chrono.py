"""Roller compaction from Project Chrono's SCM soil model.

The world-side tests use small hand-made tables and need no Chrono. The physics
tests run Chrono itself: in this interpreter if pychrono imports, or in the
interpreter named by LOAM_CHRONO_PYTHON (e.g. a conda env with pychrono).
They are skipped when neither is available.
"""
import json
import os
import subprocess
import sys

import numpy as np
import pytest
from conftest import EXAMPLE, EXAMPLES

from loam.cli import load, main
from loam.runtime.chrono_soil import SoilParams, bekker_wheel_sinkage, load_tables, settlement
from loam.runtime.world import World

TABLE = {"settlement_m": [0.010, 0.018, 0.024, 0.028]}


def world(compaction=None):
    w = World(load(EXAMPLE), EXAMPLES, compaction=compaction)
    w.loose[:] = 0.0
    return w


def compact(w, oid, passes, machine="rl1"):
    g = w.g
    win = [[g.x0, g.y0], [g.x0 + 10 * g.cell, g.y0], [g.x0 + 10 * g.cell, g.y0 + 10 * g.cell],
           [g.x0, g.y0 + 10 * g.cell]]
    return w.apply({"orderId": oid, "op": "compact", "machine": machine, "window": win, "target": {},
                    "limits": {"passes": passes}})


def test_settlement_table_is_clamped_not_extrapolated():
    assert settlement(TABLE, 0) == 0.0
    assert settlement(TABLE, 2) == 0.018
    assert settlement(TABLE, 9) == 0.028


def test_world_compacts_by_the_roller_curve():
    w = world({"rl1": TABLE})
    u = w.u
    mk = np.zeros(w.g.shape, bool)
    mk[:10, :10] = True
    w._add(mk, np.full(int(mk.sum()), 0.2 / u), loose=True)        # an 8-inch-ish loose lift (0.2 m)
    z0 = w.z.copy()
    out = compact(w, "c1", 2)
    np.testing.assert_allclose((z0 - w.z)[mk] * u, 0.018, atol=1e-12)              # 2 passes: 18 mm
    assert out["settled_mean"] == pytest.approx(0.018)
    compact(w, "c2", 2)
    np.testing.assert_allclose((z0 - w.z)[mk] * u, 0.028, atol=1e-12)              # 4 passes: 28 mm
    assert (w.rolled[mk] == 4).all()
    # a heavy curve hits the lift's full compaction (keep 0.70 -> 60 mm of 200 mm) and no more
    w2 = world({"*": {"settlement_m": [0.05, 0.09, 0.12]}})
    w2._add(mk, np.full(int(mk.sum()), 0.2 / u), loose=True)
    z0 = w2.z.copy()
    compact(w2, "h1", 3)
    np.testing.assert_allclose((z0 - w2.z)[mk] * u, 0.2 * (1 - w2.soil.keep), atol=1e-12)
    assert (w2.loose[mk] == 0).all()
    # new loose material on top starts its rolling over
    w._add(mk, np.full(int(mk.sum()), 0.1 / u), loose=True)
    assert (w.rolled[mk] == 0).all()


def test_world_without_tables_is_unchanged():
    w = world()
    u = w.u
    mk = np.zeros(w.g.shape, bool)
    mk[:10, :10] = True
    w._add(mk, np.full(int(mk.sum()), 0.2 / u), loose=True)
    z0 = w.z.copy()
    compact(w, "c1", 1)
    np.testing.assert_allclose((z0 - w.z)[mk] * u, 0.2 * (1 - w.soil.keep), atol=1e-12)


def test_tables_file_and_cli_errors(tmp_path, capsys):
    (tmp_path / "one.json").write_text(json.dumps(TABLE))
    assert load_tables(str(tmp_path / "one.json")) == {"*": TABLE}
    with open(os.path.join(EXAMPLES, "soil_rl1.json"), encoding="utf-8") as f:
        ex = json.load(f)
    rl1 = load_tables(os.path.join(EXAMPLES, "soil_rl1.json"))["rl1"]
    assert rl1 == ex["rl1"] and rl1["model"].startswith("Project Chrono SCM")
    s = rl1["settlement_m"]
    assert (np.diff(s) > 0).all()                                                    # every pass settles more
    inc = np.diff([0.0, *s])
    assert (np.diff(inc) < 0).all()                                                  # by less each time
    assert 1.2 < rl1["first_pass_sinkage_m"] / rl1["bekker_wong_sinkage_m"] < 1.4
    for machine, mass, why in (("dz1", "10t", "not a roller"), ("rl1", "10 tonnes", "expected a number")):
        with pytest.raises(SystemExit):
            main(["soil-calibrate", EXAMPLE, "--machine", machine, "--mass", mass, "--diameter", "1.5m", "-o",
                  str(tmp_path / "x.json")])
        assert why in capsys.readouterr().err


def test_bekker_wong_closed_form():
    s = SoilParams()
    z = bekker_wheel_sinkage(10_000 * 9.81, 2.1, 1.5, s)
    assert z == pytest.approx(0.0579, abs=5e-4)
    assert bekker_wheel_sinkage(20_000 * 9.81, 2.1, 1.5, s) > z > bekker_wheel_sinkage(10_000 * 9.81, 4.2, 1.5, s)


# -- Chrono itself ------------------------------------------------------------------------------------
def chrono_python():
    try:
        import pychrono  # noqa: F401
        return sys.executable
    except ImportError:
        return os.environ.get("LOAM_CHRONO_PYTHON")


CHRONO = chrono_python()
PROBE = """
import json, sys
from loam.runtime.chrono_soil import drum_passes, bekker_wheel_sinkage, SoilParams
out = {}
for name, kw in json.loads(sys.argv[1]).items():
    r = drum_passes(**kw)
    r["bekker"] = bekker_wheel_sinkage(kw["mass_kg"] * 9.81, kw["width_m"], kw["diameter_m"], SoilParams())
    out[name] = r
print(json.dumps(out))
"""


@pytest.mark.slow
@pytest.mark.skipif(CHRONO is None, reason="needs pychrono (or LOAM_CHRONO_PYTHON)")
def test_chrono_drum_matches_bekker_wong_and_compacts_less_each_pass():
    cases = {"10t": {"mass_kg": 10000, "width_m": 2.1, "diameter_m": 1.5, "passes": 3},
             "20t": {"mass_kg": 20000, "width_m": 2.1, "diameter_m": 1.5, "passes": 1},
             "narrow": {"mass_kg": 10000, "width_m": 1.0, "diameter_m": 1.5, "passes": 1}}
    env = {**os.environ, "PYTHONPATH": os.path.dirname(EXAMPLES)}
    r = subprocess.run([CHRONO, "-c", PROBE, json.dumps(cases)], capture_output=True, text=True, timeout=900,
                       env=env, check=True)
    out = json.loads(r.stdout.strip().splitlines()[-1])
    for name, v in out.items():
        ratio = v["first_pass_sinkage_m"] / v["bekker"]
        assert 1.15 < ratio < 1.40, (name, ratio)                          # 1.25-1.31 seen; see module doc
    assert out["20t"]["first_pass_sinkage_m"] > out["10t"]["first_pass_sinkage_m"]
    assert out["narrow"]["first_pass_sinkage_m"] > out["10t"]["first_pass_sinkage_m"]
    s = out["10t"]["settlement_m"]
    inc = np.diff([0.0, *s])
    assert (inc > 0).all() and inc[1] < inc[0] and inc[2] < inc[1]
