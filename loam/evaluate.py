"""Evaluate Loam conditions against a live site context.

All quantities are compared in SI units. A `Context` supplies the measured
world: residual grids per zone, probe ages, proximity, weather and task state.
"""
from __future__ import annotations

import numpy as np

from . import ast as A


class Stat:
    """Residual statistics for one zone, built from signed residuals in metres.

    residual(Z) on its own means residual(Z).p95.
      .p95  95th percentile of |residual|
      .high 95th percentile of residual (how far above grade the high spots are)
      .low  5th percentile of residual (how far below grade the low spots are)
      .max / .min / .mean  signed extremes and mean
    """

    def __init__(self, r: np.ndarray):
        r = r[np.isfinite(r)]
        self.r = r if r.size else np.zeros(1)

    def get(self, attr: str = "p95") -> float:
        r = self.r
        return {
            "p95": lambda: float(np.percentile(np.abs(r), 95)),
            "high": lambda: float(np.percentile(r, 95)),
            "low": lambda: float(np.percentile(r, 5)),
            "max": lambda: float(r.max()),
            "min": lambda: float(r.min()),
            "mean": lambda: float(r.mean()),
        }[attr]()


class Context:
    """Interface the evaluator calls. Override in a simulator or a live runtime."""

    def residual(self, zone: str) -> np.ndarray: raise NotImplementedError
    def tolerance(self, zone: str) -> float: raise NotImplementedError
    def volume_remaining(self, zone: str) -> float: raise NotImplementedError
    def compaction(self, zone: str) -> float: raise NotImplementedError
    def age(self, probe: str) -> float: raise NotImplementedError
    def proximity(self, a: str, b: str) -> float: raise NotImplementedError
    def weather(self, kind: str) -> float: raise NotImplementedError
    def progress(self, task: str) -> float: raise NotImplementedError
    def done(self, task: str) -> bool: raise NotImplementedError
    def deviation(self) -> float: raise NotImplementedError
    def elapsed(self) -> float: raise NotImplementedError


def in_tolerance(ctx: Context, zone: str) -> bool:
    """A zone is in tolerance when 95% of its cells are within its ± tolerance."""
    return Stat(ctx.residual(zone)).get("p95") <= ctx.tolerance(zone) + 1e-12


def evaluate(node, ctx: Context):
    if isinstance(node, A.Num):
        return node.value
    if isinstance(node, A.Qty):
        return node.q.si
    if isinstance(node, A.Call):
        f, args = node.fn, node.args
        name = lambda i: args[i].name  # noqa: E731
        if f == "residual":
            return Stat(ctx.residual(name(0)))
        if f == "in_tolerance":
            return in_tolerance(ctx, name(0))
        if f == "volume_remaining":
            return ctx.volume_remaining(name(0))
        if f == "compaction":
            return ctx.compaction(name(0))
        if f == "age":
            return ctx.age(name(0))
        if f == "proximity":
            return ctx.proximity(name(0), name(1))
        if f == "weather":
            return ctx.weather(name(0))
        if f == "progress":
            return ctx.progress(name(0))
        if f == "done":
            return ctx.done(name(0))
        if f == "deviation":
            return ctx.deviation()
        if f == "elapsed":
            return ctx.elapsed()
        raise ValueError(f"unknown function {f}")
    if isinstance(node, A.Attr):
        return evaluate(node.obj, ctx).get(node.name)
    if isinstance(node, A.Not):
        return not truth(evaluate(node.expr, ctx))
    if isinstance(node, A.BinOp):
        if node.op == "and":
            return truth(evaluate(node.left, ctx)) and truth(evaluate(node.right, ctx))
        if node.op == "or":
            return truth(evaluate(node.left, ctx)) or truth(evaluate(node.right, ctx))
        a = scalar(evaluate(node.left, ctx))
        b = scalar(evaluate(node.right, ctx))
        # unit conversion leaves rounding noise (95 % -> 0.9500000000000001); compare with a tiny epsilon
        eps = 1e-9 * max(1.0, abs(a), abs(b))
        return {
            "<": lambda: a < b - eps, "<=": lambda: a <= b + eps, ">": lambda: a > b + eps,
            ">=": lambda: a >= b - eps, "==": lambda: abs(a - b) <= eps, "!=": lambda: abs(a - b) > eps,
            "+": lambda: a + b, "-": lambda: a - b, "*": lambda: a * b, "/": lambda: a / b,
        }[node.op]()
    raise ValueError(f"cannot evaluate {node!r}")


def scalar(v):
    return v.get("p95") if isinstance(v, Stat) else v


def truth(v) -> bool:
    return bool(v)
