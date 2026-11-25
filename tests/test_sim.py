"""Simulator invariants and behaviour. These are properties of the model, not tuned outputs."""
import gc
import os
import tracemalloc

import numpy as np
import pytest
from conftest import EXAMPLES

from loam.checker import check, has_errors
from loam.parser import parse
from loam.sim import Beliefs, Events, Sim, Soil


def quiet(model, **kw):
    return Sim(model, base_dir=EXAMPLES, log=lambda *a: None, **kw)


@pytest.mark.parametrize("open_loop", [False, True])
def test_volume_ledger_balances(example_model, open_loop):
    r = quiet(example_model, open_loop=open_loop).run()
    assert abs(r["ledger_error"]) < 1e-6


def test_deterministic_for_a_seed(example_model):
    a, b = quiet(example_model, seed=3).run(), quiet(example_model, seed=3).run()
    assert a == b
    assert quiet(example_model, seed=4).run()["true_ground"] != a["true_ground"]


def test_model_is_not_mutated(example_model):
    import copy
    before = copy.deepcopy(example_model)
    quiet(example_model).run()
    assert repr(before) == repr(example_model)


def test_inflight_windows_never_overlap(example_model):
    class Checked(Sim):
        def plan_pass(self, m, name, speed_cap):
            ok = super().plan_pass(m, name, speed_cap)
            ws = [p.window for p in self.inflight.values() if p.window is not None]
            for i, a in enumerate(ws):
                for b in ws[i + 1:]:
                    assert not (a[0].start < b[0].stop and b[0].start < a[0].stop
                                and a[1].start < b[1].stop and b[1].start < a[1].stop)
            return ok
    assert Checked(example_model, base_dir=EXAMPLES, log=lambda *a: None).run()["finished"]


def test_ground_is_frozen_during_a_stop(example_model):
    class Watch(Sim):
        def _loop(self):
            self.frozen = []
            orig = self.apply_rules

            def rules():
                eff = orig()
                if eff["stop"]:
                    self.frozen.append(self.z.copy())
                return eff
            self.apply_rules = rules
            super()._loop()
    s = Watch(example_model, base_dir=EXAMPLES, log=lambda *a: None)
    s.run()
    assert len(s.frozen) > 4
    for z in s.frozen[1:]:
        np.testing.assert_array_equal(z, s.frozen[0])


def test_exclusion_zone_is_never_touched(example_src):
    src = example_src.replace(
        "stockpile sp1", "zone well { role exclusion; area rect (20, 40) (24, 44); buffer 6 ft }\nstockpile sp1")
    model, diags = check(parse(src))
    assert not has_errors(diags)
    s = quiet(model)
    s.run()
    assert s.excl.sum() == (4 + 12) ** 2
    np.testing.assert_array_equal(s.z[s.excl], s.z_start[s.excl])


def test_control_nothing_unknown_means_open_loop_is_unbiased(example_model):
    """Falsification check: with correct soil beliefs and noiseless blades there is nothing for feedback to
    correct. Open loop must then be unbiased and meet the program's own completion rule on the true ground,
    so any open-loop failure elsewhere is caused by the belief error, not by the simulator."""
    soil = Soil(cut_error=0.0, finish_error_m=0.0)
    s = quiet(example_model, open_loop=True, soil=soil, beliefs=Beliefs(soil.swell, soil.keep),
              events=Events.none())
    r = s.run()
    belief = s.pred - s.p_loose * (1 - s.est_keep)
    assert abs(float((s.z - belief).mean())) < 0.005                 # site units (ft)
    assert r["true_ground"]["within_tolerance_pct"] >= 95.0          # in_tolerance(pad) holds on true ground


def test_feedback_corrects_execution_noise_alone(example_model):
    soil = Soil()
    right = Beliefs(swell=soil.swell, keep=soil.keep)
    o, c = (quiet(example_model, open_loop=ol, soil=soil, beliefs=right, events=Events.none()).run()
            for ol in (True, False))
    assert c["true_ground"]["within_tolerance_pct"] > o["true_ground"]["within_tolerance_pct"]


def test_closed_loop_recovers_soil_factors(example_model):
    r = quiet(example_model).run()
    me = r["material_estimates"]
    assert r["finished"] and r["true_ground"]["within_tolerance_pct"] > 95
    assert abs(me["keep"] - me["true_keep"]) < 0.03 and abs(me["swell"] - me["true_swell"]) < 0.05


def test_no_memory_or_file_handles_retained(example_model, tmp_path):
    fd_dir = f"/proc/{os.getpid()}/fd"
    fds = len(os.listdir(fd_dir)) if os.path.isdir(fd_dir) else None
    quiet(example_model).run()                       # warm numpy's internal caches
    gc.collect()
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    for seed in range(8):
        quiet(example_model, seed=seed).run(wire_path=str(tmp_path / "w.jsonl"))
    gc.collect()
    grown = tracemalloc.get_traced_memory()[0] - base
    tracemalloc.stop()
    assert grown < 200_000, f"{grown} bytes retained after 8 runs"
    if fds is not None:
        assert len(os.listdir(fd_dir)) == fds


def test_wire_file_closed_on_error(example_model, tmp_path):
    class Boom(Sim):
        def plan_pass(self, *a):
            raise RuntimeError("boom")
    s = Boom(example_model, base_dir=EXAMPLES, log=lambda *a: None)
    with pytest.raises(RuntimeError):
        s.run(wire_path=str(tmp_path / "w.jsonl"))
    assert s.wire_out is None


REFERENCE_U64 = [14232521865600346940, 9648886400068060533, 13633754720362554755]


def test_rng_reference_values():
    """Pinned values: any implementation of the spec'd generator must reproduce these exactly."""
    from loam.rng import CounterRNG
    r = CounterRNG(7)
    assert [int(v) for v in r._u64(3)] == REFERENCE_U64
    n = CounterRNG(1).normal(0.0, 1.0, 100_000)
    assert abs(n.mean()) < 0.01 and abs(n.std() - 1) < 0.01 and np.abs(n).max() <= 6
