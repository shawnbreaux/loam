# Contributing to Loam

Thanks for helping. Loam aims to be small, correct and easy to review, so changes are judged on those terms.

## Ground rules

- **Language changes go through an RFC.** Open an issue titled `RFC: <change>` describing the syntax,
  semantics, checker rules and how a runtime must behave. Changes to SPEC.md land together with the
  checker and tests that enforce them.
- **Every behaviour has a test.** Bug fixes include a test that fails before the fix. Simulator
  changes must keep every invariant in `tests/test_sim.py`; if you change the model, update
  docs/SIMULATOR.md in the same pull request.
- **No unexplained numbers.** Any result quoted in docs comes from a script in this repository, with
  seeds, and is regenerated in the same pull request (`python examples/study.py`).
- **Keep it small.** Prefer deleting code to adding it. New runtime dependencies need a strong reason;
  the core depends only on NumPy.
- **Safety-relevant changes** (rules, holds, Loam Wire stop and heartbeat semantics) need a second
  reviewer and must never weaken an existing guarantee.

## Before you open a pull request

```bash
pip install -e ".[dev]"
ruff check loam tests examples
mypy loam
pytest -q --cov=loam
```

By contributing you agree that your contribution is licensed under Apache 2.0 (code) and CC BY 4.0
(specification text).
