"""Lexer, parser, checker and evaluator."""
import numpy as np
import pytest
from conftest import MINI, errors
from hypothesis import given, settings
from hypothesis import strategies as st

from loam.checker import check
from loam.compiler import explain, manifest
from loam.evaluate import Stat
from loam.lexer import LoamSyntaxError, tokenize
from loam.parser import parse
from loam.units import Quantity, UnitError, lookup


def test_example_checks_clean(example_model):
    assert example_model.order == ["strip_A", "top_up_C", "compact_C", "finish_pad"]
    assert "strip_A" in explain(example_model)
    topics = manifest(example_model)["topics"]["machines"]["dz1"]
    assert topics["order"] == "loam/v0/pad_north/dz1/order"


def test_minimal_program_is_clean():
    assert errors(MINI) == []


def test_version_header():
    assert parse(MINI).version == "0.0.1"
    assert any("unsupported Loam version" in e for e in errors(MINI.replace("loam 0.0.1", "loam 0.1")))
    with pytest.raises(LoamSyntaxError, match="must start with"):
        parse("site s { }")


@pytest.mark.parametrize("src, line, col", [
    ('loam 0.0.1\nsite s {\n  crs "x\n}', 3, 7),             # unterminated string
    ("loam 0.0.1\nsite s { crs @ }", 2, 14),                  # bad character
    ("loam 0.0.1\nsite s { crs \"x\"", 2, 17),                # missing }
    ("loam 0.0.1\nwidget w { }", 2, 1),                       # unknown kind
    ("loam 0.0.1\ntask t { until a < b < c }", 2, 22),        # chained comparison
])
def test_syntax_error_positions(src, line, col):
    with pytest.raises(LoamSyntaxError) as e:
        parse(src)
    assert (e.value.line, e.value.col) == (line, col)


def test_lists_span_lines_and_allow_trailing_comma():
    prog = parse('loam 0.0.1\nzone z { role cut; area polygon [\n (0, 0),\n (1, 0),\n (1, 1),\n] }')
    assert len(prog.decls[0].get("area").args[1].items) == 3


@pytest.mark.parametrize("change, message", [
    (("accuracy 0.1 ft", "accuracy 0.1"), "needs a unit"),
    (("rate 10 min", "rate 10 ft"), "must be a time"),
    (("residual(A) < 0.2 ft", "residual(A) < 5 min"), "cannot compare"),
    (("residual(A) < 0.2 ft", "residual(A) < 0.2"), "bare number"),
    (("zone A;", "zone B;"), "no zone named 'B'"),
    (("by class dozer", "by class roller"), "needs a roller"),
    (("class dozer;", "class bulldozer;"), "unknown machine class"),
    (("role cut;", "role cut; colour red;"), "unknown field 'colour'"),
    (("kind uav;", ""), "missing required field 'kind'"),
    (("machine d1", "machine A"), "already declared"),
    (("until residual(A) < 0.2 ft", "until residual(A)"), "true/false"),
    (("residual(A) < 0.2 ft", "slope(A) < 0.2 ft"), "unknown function"),
    (("residual(A) < 0.2 ft", "residual(A).p50 < 0.2 ft"), "unknown statistic"),
    (("area rect (0, 0) (10, 10)", "area circle (0, 0)"), "unknown area form"),
    (("area rect (0, 0) (10, 10)", "area polygon [(0, 0), (1, 1)]"), "at least 3 points"),
    (("area rect (0, 0) (10, 10)", "area rect (0, 0)"), "two corner points"),
    (("area rect (0, 0) (10, 10)", "area file 3"), "needs a path"),
    (("area rect (0, 0) (10, 10)", "area where residual"), "true/false"),
    (("spoil -> A;", "spoil A;"), "spoil reads"),
    (("by class dozer", "by dozers please"), "by reads"),
    (("by class dozer", "by cap fly"), "unknown capability"),
    (("spoil -> A;", "spoil -> A; passes 2.5;"), "whole number"),
    (("spoil -> A;", "spoil -> A; priority high;"), "must be a number"),
    (("spoil -> A;", "spoil -> A; after t;"), "cannot run after itself"),
    (("spoil -> A;", "spoil -> A; after nothing soon;"), "after reads"),
    (("spoil -> A;", "spoil -> A; max_cut 0.5 ft per lane;"), "per pass"),
    (("op cut;", "op trench;"), "needs 'along"),
    (("op cut;", "op dig;"), "unknown task op"),
    (("units ft;", "units furlongs;"), "unknown site units"),
    (("crs \"LOCAL:ft\";", "crs 5;"), "quoted string"),
    (("design \"d.xml\"", "design \"d.xml\" format dwg"), "unknown surface format"),
    (("design \"d.xml\"", "design \"d.xml\" colour red"), "expected 'format"),
    (("blade 10 ft", "blade ±10 ft"), "only allowed on tolerances"),
    (("blade 10 ft", "blade 10 ft; caps cut"), "caps needs a list"),
    (("kind uav;", "kind kite;"), "unknown probe kind"),
])
def test_checker_errors(change, message):
    errs = errors(MINI.replace(*change))
    assert any(message in e for e in errs), errs


RULE = "rule r {{ when {cond}; do {act} }}\n"


@pytest.mark.parametrize("cond, act, message", [
    ("age(p) > 15 min", "explode all", "unknown action"),
    ("age(p) > 15 min", "alert 5", "needs a message"),
    ("age(p) > 15 min", "slow all", "needs a speed"),
    ("age(p) > 15 min", "stop", "needs a target"),
    ("age(p) > 15 min", "stop task nope", "no task named"),
    ("age(p) > 15 min", "stop d1 d1", "could not read"),
    ("proximity(person, ghost) < 5 ft", "stop all", "proximity() arguments"),
    ("weather(rain) > 5 ft", "stop all", "rate (in/h) or a speed"),
    ("age(p) > 15 min and 3 ft", "stop all", "joins true/false"),
    ("not 3 ft", "stop all", "'not' needs"),
    ("age(p) + 3 ft > 1 min", "stop all", "cannot add"),
    ("age(p) * age(p) > 1 min", "stop all", "plain number"),
    ("\"x\" > 1 min", "stop all", "strings cannot"),
    ("age(t) > 1 min", "stop all", "is a task, expected a probe"),
    ("residual(A) > 1 ft or done(t)", "pause [t, d1]", None),
    ("weather(wind) > 30 mph", "slow class dozer 3 mph", None),
])
def test_rule_checks(cond, act, message):
    errs = errors(MINI + RULE.format(cond=cond, act=act))
    if message is None:
        assert errs == []
    else:
        assert any(message in e for e in errs), errs


def test_warnings_are_reported():
    from loam.checker import check
    _, diags = check(parse(MINI.replace("spoil -> A; ", "").replace("blade 10 ft", "blade 10 ft; caps [haul]")
                           .replace("by class dozer", "by d1")))
    warns = [d.msg for d in diags if d.level == "warning"]
    assert any("no spoil destination" in w for w in warns) and any("no 'cut' capability" in w for w in warns)


def test_dependency_cycle():
    src = MINI.replace("spoil -> A;", "spoil -> A; after u;") + \
        "task u { op cut; zone A; by class dozer; after t; until residual(A) < 0.1 ft }\n"
    assert any("cycle" in e for e in errors(src))


def test_diagnostics_are_in_source_order():
    _, diags = check(parse(MINI.replace("blade 10 ft", "blade 10").replace("rate 10 min", "rate 10")))
    assert [(d.line, d.col) for d in diags] == sorted((d.line, d.col) for d in diags)


def test_units():
    assert Quantity(8, "in").to("ft") == pytest.approx(2 / 3)
    assert Quantity(1, "yd3").to("ft3") == pytest.approx(27)
    assert Quantity(0.25, "in/h").dim == "rate"
    assert Quantity(95, "%").si == pytest.approx(0.95)
    with pytest.raises(UnitError):
        lookup("furlong")
    with pytest.raises(UnitError):
        Quantity(1, "ft").to("min")


def test_stat_attributes():
    s = Stat(np.array([-0.3, -0.1, 0.0, 0.1, 0.2]))
    assert (s.get("max"), s.get("min")) == (0.2, -0.3)
    assert s.get("mean") == pytest.approx(-0.02)
    assert s.get("low") < 0 < s.get("high")
    assert s.get("p95") == pytest.approx(np.percentile([0.3, 0.1, 0, 0.1, 0.2], 95))


def test_comparisons_absorb_unit_rounding():
    from loam.evaluate import Context, evaluate

    class Ctx(Context):
        def compaction(self, zone):
            return 0.95
    cond = parse(MINI.replace("until residual(A) < 0.2 ft", "until compaction(A) >= 95 %")).decls[-1].get("until")
    assert evaluate(cond.args[0], Ctx()) is True


TOKENS = st.sampled_from(["loam", "0.0.1", "site", "task", "zone", "s", "{", "}", "(", ")", "[", "]", ",", ";",
                          "\n", "->", "<", ">=", "±", "0.5", "ft", "min", "%", '"x"', "until", "when", "and", "#c",
                          "residual", ".", "rect", "where", "area"])


@settings(max_examples=400, deadline=None)
@given(st.lists(TOKENS, max_size=40))
def test_parser_and_checker_never_crash(tokens):
    """Any input either parses or raises LoamSyntaxError; the checker never raises."""
    src = "loam 0.0.1\n" + " ".join(tokens)
    try:
        prog = parse(src)
    except LoamSyntaxError:
        return
    check(prog)


def test_tokenizer_scientific_notation():
    assert [t.text for t in tokenize("1.5e3 2E-2")][:2] == ["1.5e3", "2E-2"]
