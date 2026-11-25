"""Closed-loop reference simulator for Loam programs.

A teaching and conformance tool, not a production planner. It runs a checked
Loam program against a gridded site whose soil does not behave the way the
planner assumes, and evaluates the program's `until`, `rule` and `replan`
conditions on noisy probe measurements of the true ground. `open_loop=True`
runs the same program with one survey and no feedback, for comparison.

Model summary (full description in docs/SIMULATOR.md):

* Ground truth `z` and planner belief `pred` are height grids in site units.
* Every order is an in-flight `Pass`. Its effect is decided when the order is
  issued and lands on the ground when the pass completes, so surveys never see
  unfinished work and a `stop` delays the effect.
* Material factors: dug bank volume swells by `Soil.swell`; rolling keeps
  `Soil.keep` of the loose thickness. The planner starts from `Beliefs` and
  re-estimates both from surveys when a `replan` fires.
* Every change to `z` is booked in a volume ledger; `ledger_error()` must stay
  at rounding level, and the test suite checks it.
"""
from __future__ import annotations

import contextlib
import json
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import IO, Any

import numpy as np
from numpy.typing import NDArray

from . import ast as ast_
from . import wire
from .checker import OP_CAP, Model
from .evaluate import Context, evaluate
from .grid import SiteGrid, polygon_mask  # noqa: F401  (polygon_mask re-exported)
from .rng import CounterRNG
from .units import FT, Quantity

Grid = NDArray[np.float64]
Mask = NDArray[np.bool_]
Window = tuple[slice, slice]


@dataclass(frozen=True)
class Soil:
    """Ground truth. All dimensional values in metres."""
    swell: float = 1.30            # loose volume / bank volume when dug
    keep: float = 0.70             # compacted thickness / loose thickness when rolled
    cut_error: float = 0.06        # relative std-dev of blade cut depth
    finish_error_m: float = 0.006  # std-dev of a finishing pass


@dataclass(frozen=True)
class Beliefs:
    """What the planner assumes before it has measured anything."""
    swell: float = 1.25
    keep: float = 0.80


@dataclass(frozen=True)
class PlannerConfig:
    segment_m: float = 30 * FT          # length of one dozer push
    turnaround_min: float = 0.3
    finish_step_m: float = 0.3 * FT     # most a finishing pass moves a cell
    work_fraction: float = 0.5          # ignore cells within this share of tolerance
    min_calibration_cells: int = 200


@dataclass(frozen=True)
class Events:
    """Scripted disturbances, minutes from start: (start, end)."""
    person: tuple[float, float] = (42.0, 45.0)
    person_distance_m: float = 30 * FT
    probe_outage: tuple[float, float] = (95.0, 118.0)
    rain: tuple[float, float] = (150.0, 175.0)
    rain_m_per_s: float = 0.4 * 0.0254 / 3600

    @staticmethod
    def none() -> Events:
        return Events(person=(-1, -1), probe_outage=(-1, -1), rain=(-1, -1))


def in_window(t: float, w: tuple[float, float]) -> bool:
    return w[0] <= t < w[1]


@dataclass
class Machine:
    name: str
    cls: str
    caps: list[str]
    width: float           # site units
    speed: float           # site units per minute
    busy_until: float = 0.0
    passes: dict[str, int] = field(default_factory=dict)
    busy_min: float = 0.0


@dataclass
class Pass:
    """An issued order. Arrays are sparse: flat cell indices plus values."""
    machine: str
    task: str
    op: str
    window: Window | None
    idx: NDArray[np.intp]
    dz_true: Grid          # change to true ground (cut/fill/finish; compaction is resolved at completion)
    dz_pred: Grid          # change the planner expects
    dloose_true: Grid
    dloose_pred: Grid


class Sim(Context):
    def __init__(self, model: Model, base_dir: str = ".", seed: int = 7, open_loop: bool = False,
                 soil: Soil | None = None, beliefs: Beliefs | None = None, events: Events | None = None,
                 config: PlannerConfig | None = None, dt: float = 0.25, max_minutes: float = 24 * 60,
                 log: Callable[[str], Any] = print, start_epoch: float = 1796108400.0):  # 2026-12-01 07:00 UTC
        if len(model.probes) > 1:
            raise ValueError("the reference simulator models one probe; fusing several is on the roadmap")
        self.m = model
        self.rng = CounterRNG(seed)
        self.open_loop = open_loop
        self.soil, self.bel = soil or Soil(), beliefs or Beliefs()
        self.ev, self.cfg = events or Events(), config or PlannerConfig()
        self.dt, self.max_minutes, self.log_fn, self.start_epoch = dt, max_minutes, log, start_epoch
        self.t = 0.0
        self.wire_out: IO[str] | None = None
        self.wire_seq = 0
        site = model.site
        self.site_name: str = site["name"]
        g = self.grid = SiteGrid(model, base_dir)
        self.unit, self.u, self.cell = g.unit, g.u, g.cell
        self.x0, self.y0, self.nx, self.ny, self.xs, self.ys, self.area = g.x0, g.y0, g.nx, g.ny, g.xs, g.ys, g.area
        x1, y1 = g.x1, g.y1
        ex = site.get("existing") or {}
        if "file" not in ex:
            raise ValueError("the simulator needs an existing-ground surface file as ground truth")
        self.design = g.surface(site["design"])
        self.z = g.surface(ex)
        self.z_start = self.z.copy()
        zeros = lambda: np.zeros_like(self.z)  # noqa: E731
        self.loose = zeros()
        for name, zd in model.zones.items():
            if "polygon" not in (zd.get("area") or {}) and "where" not in (zd.get("area") or {}):
                raise ValueError(f"zone '{name}': the simulator supports polygon and 'where' areas")
        self.masks: dict[str, Mask | None] = g.masks
        self.stock: dict[str, float] = {}
        self.stock_xy: dict[str, tuple[float, float]] = {}
        for name, sp in model.stockpiles.items():
            v = sp.get("available") or sp.get("capacity")
            self.stock[name] = v.to(f"{self.unit}3") if v else 0.0
            poly = np.array(sp["area"]["polygon"]) if "polygon" in sp.get("area", {}) else np.array([[x1, y1]])
            cx, cy = poly.mean(0)
            self.stock_xy[name] = (float(cx), float(cy))
        self.machines: dict[str, Machine] = {}
        for name, md in model.machines.items():
            w = md.get("blade") or md.get("drum") or md.get("width") or Quantity(8, "ft")
            sp = md.get("max_speed") or Quantity(3, "mph")
            self.machines[name] = Machine(name, md["class"], md["caps"], w.to(self.unit), sp.si * 60 / self.u)
        self.inflight: dict[str, Pass] = {}
        # planner state
        self.meas: Grid = self.z.copy()
        self.prev_meas: Grid | None = None
        self.has_survey = False
        self.last_probe, self.last_rebase, self.last_dev = -1e9, -1e9, 0.0
        self.pred, self.p_loose = zeros(), zeros()
        self.plan_dz, self.plan_dloose = zeros(), zeros()   # in-flight changes the planner has committed to
        self.est_swell, self.est_keep = self.bel.swell, self.bel.keep
        self.touched_dep = np.zeros_like(self.z, bool)
        self.touched_comp = np.zeros_like(self.z, bool)
        self.touched_other = np.zeros_like(self.z, bool)
        self.loose_rolled_pred, self.dep_pred = zeros(), zeros()
        self.excl = self._exclusion()
        # bookkeeping
        self.started: dict[str, float | None] = dict.fromkeys(model.tasks)
        self.done_at: dict[str, float | None] = dict.fromkeys(model.tasks)
        self.task_passes = dict.fromkeys(model.tasks, 0)
        self.active_rules = dict.fromkeys(model.rules, False)
        self.builtin_stale = False
        self.rule_log: list[tuple[float, str, bool]] = []
        self.replans: list[float] = []
        self.ledger = 0.0                # booked change of true ground volume, site units^3
        self.windrow = 0.0               # bank volume cut by finishing passes, not yet reused
        self.cut_bank = self.import_bank = self.export_bank = 0.0
        self.probe_count = self.order_seq = 0

    # -- setup -----------------------------------------------------------
    def _exclusion(self) -> Mask:
        return self.grid.exclusion(self.mask)

    # -- small helpers ---------------------------------------------------
    def log(self, msg: str) -> None:
        h, mm = divmod(self.t, 60)
        self.log_fn(f"[{int(h):02d}:{mm:04.1f}] {msg}")

    def emit(self, topic: str, msg: dict) -> None:
        if self.wire_out is not None:
            self.wire_out.write(json.dumps({"topic": topic, "payload": msg}) + "\n")

    def ts(self) -> float:
        return self.start_epoch + self.t * 60

    def seq(self) -> int:
        self.wire_seq += 1
        return self.wire_seq

    def mask(self, zone: str) -> Mask:
        mk = self.masks.get(zone)
        if mk is not None:
            return mk
        if zone in self.m.stockpiles:
            return np.zeros((self.ny, self.nx), bool)
        return np.asarray(self._cell_eval(self.m.zones[zone]["area"]["where"]), bool)

    def _cell_eval(self, node: Any) -> Any:
        """Evaluate an `area where` condition per cell (names: residual, design, actual), SI units."""
        if isinstance(node, ast_.Ident):
            surf = self.checked_surface()
            return {"residual": surf - self.design, "design": self.design, "actual": surf}[node.name] * self.u
        if isinstance(node, ast_.Num):
            return node.value
        if isinstance(node, ast_.Qty):
            return node.q.si
        if isinstance(node, ast_.Not):
            return ~self._cell_eval(node.expr)
        if isinstance(node, ast_.BinOp):
            a, b = self._cell_eval(node.left), self._cell_eval(node.right)
            ops: dict[str, Callable[[], Any]] = {
                "and": lambda: a & b, "or": lambda: a | b, "<": lambda: a < b, "<=": lambda: a <= b,
                ">": lambda: a > b, ">=": lambda: a >= b, "+": lambda: a + b, "-": lambda: a - b,
                "*": lambda: a * b, "/": lambda: a / b}
            return ops[node.op]()
        raise ValueError("unsupported 'area where' expression in the simulator")

    # -- surfaces --------------------------------------------------------
    def plan_view(self) -> Grid:
        """Finished surface the planner expects once in-flight passes land and loose fill is rolled."""
        return self.pred + self.plan_dz - (self.p_loose + self.plan_dloose) * (1 - self.est_keep)

    def checked_surface(self) -> Grid:
        """Surface the program's conditions are evaluated on: measured, or (open loop) predicted."""
        if self.open_loop:
            return self.pred - self.p_loose * (1 - self.est_keep)
        return self.meas

    # -- Context interface (SI units) -----------------------------------
    def residual(self, zone: str) -> Grid:
        return (self.checked_surface() - self.design)[self.mask(zone)] * self.u

    def tolerance(self, zone: str) -> float:
        tol = self.m.zones[zone].get("tolerance") or Quantity(0.1, "ft")
        return float(tol.si)

    def volume_remaining(self, zone: str) -> float:
        r = (self.checked_surface() - self.design)[self.mask(zone)]
        return float(np.abs(r).sum() * self.area) * self.u ** 3

    def compaction(self, zone: str) -> float:
        # Stand-in for an intelligent-compaction roller's coverage map: 95 % once every
        # loose lift in the zone has been rolled, falling linearly to 85 % with unrolled area.
        mk = self.mask(zone)
        if self.open_loop:
            frac = float((self.p_loose[mk] > 1e-9).mean()) if mk.any() else 0.0
        else:
            frac = float((self.loose[mk] > 1e-9).mean()) if mk.any() else 0.0
        return 0.95 - 0.10 * frac

    def age(self, probe: str) -> float:
        return 0.0 if self.open_loop else (self.t - self.last_probe) * 60.0

    def proximity(self, a: str, b: str) -> float:
        # Scripted: the only people on site are the event's visitor.
        return self.ev.person_distance_m if in_window(self.t, self.ev.person) else float("inf")

    def weather(self, kind: str) -> float:
        return self.ev.rain_m_per_s if kind == "rain" and in_window(self.t, self.ev.rain) else 0.0

    def progress(self, task: str) -> float:
        return 1.0 if self.done_at[task] is not None else 0.0

    def done(self, task: str) -> bool:
        return self.done_at[task] is not None

    def deviation(self) -> float:
        return self.last_dev * self.u

    def elapsed(self) -> float:
        return self.t * 60.0

    # -- surveys and replanning -----------------------------------------
    def probe(self) -> None:
        if not self.m.probes:
            return
        p = next(iter(self.m.probes.values()))
        if self.has_survey and (self.open_loop or in_window(self.t, self.ev.probe_outage)
                                or self.t - self.last_probe < p["rate"].to("min") - 1e-9):
            return
        sigma = p["accuracy"].to(self.unit) / 2   # accuracy is read as a 2-sigma (95 %) bound
        self.prev_meas = self.meas if self.has_survey else None
        self.meas = self.z + self.rng.normal(0, sigma, self.z.shape)
        self.last_probe, self.probe_count = self.t, self.probe_count + 1
        self.emit(wire.topic(self.site_name, None, "terrain"),
                  wire.terrain(self.site_name, p["name"], self.seq(), (self.x0, self.y0), self.cell,
                               (self.ny, self.nx), sigma=sigma, ts=self.ts()))
        if not self.has_survey:
            self.has_survey = True
            self.pred[:] = self.meas
            self.log(f"{p['name']}: first survey, {self.nx}x{self.ny} cells at {self.cell:g} {self.unit}")
            return
        touched = self.touched_dep | self.touched_comp | self.touched_other
        self.last_dev = float(np.percentile(np.abs(self.meas - self.pred)[touched], 95)) if touched.any() else 0.0
        if any(evaluate(r["when"], self) for r in self.m.replans.values()):
            self.replan()
        for a in (self.touched_dep, self.touched_comp, self.touched_other):
            a[:] = False
        self.loose_rolled_pred[:] = 0
        self.dep_pred[:] = 0

    def replan(self, reason: str | None = None) -> None:
        """Rebase the prediction on the latest survey; on a drift replan also re-estimate soil factors
        from cells that saw exactly one kind of work since the previous survey."""
        notes = []
        if reason is None and self.prev_meas is not None:
            n_min = self.cfg.min_calibration_cells
            only_dep = self.touched_dep & ~self.touched_comp & ~self.touched_other
            expected = self.dep_pred[only_dep].sum()
            if only_dep.sum() > n_min and expected > 1e-9:
                rise = (self.meas - self.prev_meas)[only_dep].sum()
                self.est_swell = float(np.clip(self.est_swell * rise / expected, 0.9, 1.8))
                notes.append(f"swell now {self.est_swell:.2f}")
            only_comp = self.touched_comp & ~self.touched_dep & ~self.touched_other
            rolled = self.loose_rolled_pred[only_comp].sum()
            if only_comp.sum() > n_min and rolled > 1e-9:
                drop = (self.prev_meas - self.meas)[only_comp].sum()
                self.est_keep = float(np.clip(1 - drop / rolled, 0.5, 1.0))
                notes.append(f"compaction keep now {self.est_keep:.2f}")
        self.pred[:] = self.meas
        self.last_rebase = self.last_probe
        self.replans.append(self.t)
        why = reason or f"measured vs predicted p95 {self.last_dev:.2f} {self.unit}"
        self.log(f"replan #{len(self.replans)}: {why}" + (f"; {', '.join(notes)}" if notes else ""))

    # -- rules -----------------------------------------------------------
    def apply_rules(self) -> dict[str, Any]:
        eff: dict[str, Any] = {"stop": False, "hold": False, "paused": set(), "slow": None}
        # Built-in hold (spec, Safety model): stale probe data blocks new orders, rule or no rule.
        stale = any(self.age(n) > p["stale_after"].si for n, p in self.m.probes.items())
        if stale != self.builtin_stale:
            self.builtin_stale = stale
            self.log("built-in hold: probe data is stale" if stale else "built-in hold cleared: probe data fresh")
        eff["hold"] = stale
        for name, r in self.m.rules.items():
            on = bool(evaluate(r["when"], self))
            act = r["do"]
            if on != self.active_rules[name]:
                self.active_rules[name] = on
                self.rule_log.append((self.t, name, on))
                self.log(f"rule {name}: {'ACTIVE -> ' + act['verb'] if on else 'cleared'}")
                if act["verb"] in ("stop", "hold"):
                    for mn in self.targets(act.get("target")):
                        self.emit(wire.topic(self.site_name, mn, "instantAction"),
                                  wire.instant_action(self.site_name, mn, self.seq(),
                                                      act["verb"] if on else "resume", name, ts=self.ts()))
            if not on:
                continue
            tgt = act.get("target") or {}
            if act["verb"] == "stop":
                eff["stop"] = True
            elif act["verb"] == "hold" or (act["verb"] == "pause" and tgt.get("all")):
                eff["hold"] = True
            elif act["verb"] == "pause":
                eff["paused"].update([tgt["task"]] if tgt.get("task") else tgt.get("names", []))
            elif act["verb"] == "slow":
                eff["slow"] = act.get("speed")
        return eff

    def targets(self, tgt: dict | None) -> list[str]:
        tgt = tgt or {}
        if tgt.get("all"):
            return list(self.machines)
        if tgt.get("class"):
            return [n for n, m in self.machines.items() if m.cls == tgt["class"]]
        return [n for n in tgt.get("names", []) if n in self.machines]

    # -- tasks -----------------------------------------------------------
    def eligible(self, name: str, paused: set[str]) -> bool:
        if self.done_at[name] is not None or name in paused:
            return False
        for dep in self.m.tasks[name].get("after", []):
            ref = self.done_at if dep["when"] == "done" else self.started
            if ref[dep["task"]] is None:
                return False
        return True

    def fits(self, m: Machine, name: str) -> bool:
        by = self.m.tasks[name].get("by") or {}
        if by.get("machine"):
            return bool(by["machine"] == m.name)
        if by.get("class"):
            return bool(by["class"] == m.cls)
        return (by.get("cap") or OP_CAP.get(self.m.tasks[name]["op"])) in m.caps

    def check_done(self) -> None:
        """A task is done once it is eligible to run and its `until` holds, even if it never ran."""
        if not self.has_survey:
            return
        for name, t in self.m.tasks.items():
            if self.done_at[name] is None and any(p.task == name for p in self.inflight.values()):
                continue  # never close a task with work still landing
            ready = self.started[name] is not None or self.eligible(name, set())
            if self.done_at[name] is None and ready and evaluate(t["until"], self):
                self.done_at[name] = self.t
                self.log(f"task {name} DONE after {self.task_passes[name]} passes")

    # -- planning one pass -----------------------------------------------
    def _windows(self, mk: Mask, width: float, seg: float) -> list[Window]:
        wr, wc = max(1, round(width / self.cell)), max(1, round(seg / self.cell))
        rows, cols = np.flatnonzero(mk.any(1)), np.flatnonzero(mk.any(0))
        if rows.size == 0:
            return []
        return [(slice(r, min(r + wr, self.ny)), slice(c, min(c + wc, self.nx)))
                for r in range(rows[0], rows[-1] + 1, wr)
                for c in range(cols[0], cols[-1] + 1, max(1, wc // 2))]

    def _best_window(self, score: Grid, mk: Mask, width: float, seg: float) -> Window | None:
        """Highest-scoring free window: not reserved by an in-flight pass and clear of exclusions."""
        busy = [p.window for p in self.inflight.values() if p.window is not None]
        best, best_s = None, 0.0
        for w in self._windows(mk, width, seg):
            if self.excl[w].any() or any(_overlap(w, b) for b in busy):
                continue
            s = float((score[w] * mk[w]).sum())
            if s > best_s:
                best, best_s = w, s
        return best

    def _place(self, zone: str, lift: float, loose_pred: float) -> tuple[NDArray[np.intp], Grid]:
        """Choose cells for `loose_pred` of loose material: lowest planned cells first, at most one lift."""
        mk = self.mask(zone) & ~self.excl
        need = np.clip((self.design - self.plan_view()) / max(self.est_keep, 1e-3), 0, lift)
        flat = np.where(mk, need, 0).ravel()
        order = np.argsort(-flat, kind="stable")
        cum = np.cumsum(flat[order]) * self.area
        k = int(np.searchsorted(cum, loose_pred))
        idx = order[:k + 1] if k < flat.size else order
        amt = flat[idx].copy()
        if k < flat.size:
            amt[-1] = max(0.0, (loose_pred - (cum[k - 1] if k > 0 else 0.0)) / self.area)
            amt[-1] = min(amt[-1], flat[idx[-1]])
        keep = amt > 0
        return idx[keep], amt[keep]

    def _zone_lift(self, zone: str) -> float:
        return float((self.m.zones[zone].get("lift") or Quantity(8, "in")).to(self.unit))

    def plan_pass(self, m: Machine, name: str, speed_cap: Quantity | None) -> bool:
        t = self.m.tasks[name]
        op, zone = t["op"], t.get("zone")
        mk = self.mask(zone) if zone else np.zeros((self.ny, self.nx), bool)
        tol = self.m.zones[zone]["tolerance"].to(self.unit) if zone else 0.1 * FT / self.u
        res = self.plan_view() - self.design
        seg = self.cfg.segment_m / self.u
        thr = tol * self.cfg.work_fraction
        speed = min(m.speed, speed_cap.si * 60 / self.u) if speed_cap is not None else m.speed
        limits: dict[str, Any] = {}
        deposit_msg: dict[str, Any] | None = None
        dist = 0.0
        w: Window | None = None
        parts: list[tuple[NDArray[np.intp], Grid, Grid, Grid, Grid]] = []   # idx, dz_t, dz_p, dl_t, dl_p
        if op == "cut":
            w = self._best_window(np.where(res > thr, res, 0), mk, m.width, seg)
            if w is None:
                return False
            max_cut = t["max_cut"].to(self.unit) if t.get("max_cut") else 0.5 * FT / self.u
            sub = np.where(mk[w] & (res[w] > thr), np.clip(res[w], 0, max_cut), 0)
            d_true = sub * np.clip(1 + self.rng.normal(0, self.soil.cut_error, sub.shape), 0, None)
            widx = self._window_idx(w)
            parts.append((widx, -d_true.ravel(), -sub.ravel(), np.zeros(widx.size), np.zeros(widx.size)))
            bank_pred, bank_true = sub.sum() * self.area, d_true.sum() * self.area
            self.cut_bank += bank_true
            loose_pred, scale = bank_pred * self.est_swell, (bank_true * self.soil.swell) / max(
                bank_pred * self.est_swell, 1e-12)
            centre: tuple[float, float] | None = None
            for dest in t.get("spoil") or []:
                if loose_pred <= 1e-12:
                    break
                if dest in self.m.zones:
                    idx, amt = self._place(dest, self._zone_lift(dest), loose_pred)
                    if idx.size:
                        parts.append((idx, amt * scale, amt, amt * scale, amt))
                        loose_pred -= amt.sum() * self.area
                        centre = centre or self._centre(idx)
                        deposit_msg = {"zone": dest, "lift": round(self._zone_lift(dest) * self.u, 4)}
                elif dest in self.stock:
                    back = loose_pred * scale / self.soil.swell
                    self.stock[dest] += back
                    self.export_bank += back
                    centre = centre or self.stock_xy[dest]
                    deposit_msg = deposit_msg or {"stockpile": dest}
                    loose_pred = 0.0
            if loose_pred > 1e-12:   # nowhere left to put it: leave it on the cut as a windrow
                self.windrow += loose_pred * scale / self.soil.swell
            if centre is not None:
                dist = 2 * float(np.hypot(centre[0] - self.xs[w[1]].mean(), centre[1] - self.ys[w[0]].mean()))
            limits["maxCut"] = round(max_cut * self.u, 4)
        elif op in ("fill", "spread"):
            lift = (t.get("lift") or Quantity(self._zone_lift(zone), self.unit)).to(self.unit)
            if (np.clip(self.design - self.plan_view(), 0, None) * mk).sum() * self.area < 0.05 * m.width * seg:
                return False
            src = next((s for s in t.get("source") or [] if self.stock.get(s, 0) > 0), None)
            if src is None:
                return False
            load_bank = min(self.stock[src], 0.5 * m.width * seg / self.est_swell)
            idx, amt = self._place(zone, lift, load_bank * self.est_swell)
            if not idx.size:
                return False
            used_bank = amt.sum() * self.area / self.est_swell
            self.stock[src] -= used_bank
            self.import_bank += used_bank
            scale = self.soil.swell / self.est_swell
            parts.append((idx, amt * scale, amt, amt * scale, amt))
            sx, sy = self.stock_xy[src]
            cx, cy = self._centre(idx)
            dist = 2 * float(np.hypot(cx - sx, cy - sy))
            limits["lift"] = round(lift * self.u, 4)
            deposit_msg = {"zone": zone, "lift": round(lift * self.u, 4), "source": src}
        elif op == "compact":
            # Roll only lifts that have landed: cells with fill still in flight wait for the next pass.
            ready = (self.p_loose > 1e-9) & ~(self.plan_dloose > 1e-9)
            w = self._best_window(ready.astype(float), mk, m.width, 1e9)
            if w is None:
                return False
            passes = t.get("passes") or 4
            sel = (mk[w] & ready[w]).ravel()
            lp = self.p_loose[w].ravel()[sel]
            n = int(sel.sum())
            # Expected effect, held in plan_dz/plan_dloose until landing; `land` recomputes it from the
            # loose material actually present then.
            parts.append((self._window_idx(w)[sel], np.zeros(n), -lp * (1 - self.est_keep), np.zeros(n), -lp))
            dist = (passes - 1) * (w[1].stop - w[1].start) * self.cell
            limits["passes"] = passes
        elif op == "finish":
            w = self._best_window(np.where(np.abs(res) > thr, np.abs(res), 0), mk, m.width, 1.5 * seg)
            if w is None:
                return False
            widx = self._window_idx(w)
            r = res[w].ravel()
            sel = mk[w].ravel() & (np.abs(r) > thr)
            step = np.where(sel, np.clip(r, -self.cfg.finish_step_m / self.u, self.cfg.finish_step_m / self.u), 0)
            err = np.where(sel, self.rng.normal(0, self.soil.finish_error_m / self.u, r.shape), 0)
            cut_t, fill_t = np.clip(step + err, 0, None), np.clip(-(step + err), 0, None)
            # Finishing fill is blade-compacted in place: bank volume needed = fill / (swell * keep).
            need = fill_t.sum() * self.area / (self.soil.swell * self.soil.keep)
            self.windrow += cut_t.sum() * self.area
            draw = min(need, self.windrow)
            self.windrow -= draw
            for s in t.get("source") or []:   # top up from the task's declared sources, in order
                take = min(self.stock.get(s, 0.0), need - draw)
                self.stock[s] -= take
                self.import_bank += take
                draw += take
            fill_t *= draw / need if need > 1e-12 else 0.0
            parts.append((widx, fill_t - cut_t, -step, np.zeros(widx.size), np.zeros(widx.size)))
        else:
            return False
        self._issue(m, name, op, w, parts, speed, dist, limits, deposit_msg)
        return True

    def _window_idx(self, w: Window) -> NDArray[np.intp]:
        r = np.arange(w[0].start, w[0].stop)
        c = np.arange(w[1].start, w[1].stop)
        return (r[:, None] * self.nx + c[None, :]).ravel()

    def _centre(self, idx: NDArray[np.intp]) -> tuple[float, float]:
        j, i = np.divmod(idx, self.nx)
        return float(self.xs[i].mean()), float(self.ys[j].mean())

    def _issue(self, m: Machine, name: str, op: str, w: Window | None, parts: list, speed: float,
               dist: float, limits: dict, deposit_msg: dict | None) -> None:
        idx = np.concatenate([p[0] for p in parts]).astype(np.intp)
        cat = [np.concatenate([p[k] for p in parts]).astype(float) for k in range(1, 5)]
        ps = Pass(m.name, name, op, w, idx, *cat)
        np.add.at(self.plan_dz.ravel(), idx, ps.dz_pred)
        np.add.at(self.plan_dloose.ravel(), idx, ps.dloose_pred)
        self.inflight[m.name] = ps
        length = (w[1].stop - w[1].start) * self.cell if w is not None else self.cfg.segment_m / self.u
        dur = (length + dist) / max(speed, 1e-9) + self.cfg.turnaround_min
        m.busy_until, m.busy_min = self.t + dur, m.busy_min + dur
        m.passes[name] = m.passes.get(name, 0) + 1
        self.task_passes[name] += 1
        if self.started[name] is None:
            self.started[name] = self.t
            self.log(f"task {name} started ({m.name})")
        self.order_seq += 1
        limits["maxSpeed"] = round(speed * self.u / 60, 3)
        self.emit(wire.topic(self.site_name, m.name, "order"),
                  wire.order(self.site_name, m.name, self.seq(), f"{name}-{self.order_seq}", 0, name, op,
                             self._window_poly(w) if w is not None else None, {"surface": "design"},
                             limits, deposit_msg, ts=self.ts()))

    def _window_poly(self, w: Window) -> list[list[float]]:
        x0, x1 = self.x0 + w[1].start * self.cell, self.x0 + w[1].stop * self.cell
        y0, y1 = self.y0 + w[0].start * self.cell, self.y0 + w[0].stop * self.cell
        return [[round(float(x), 2), round(float(y), 2)] for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]

    def land(self, ps: Pass) -> None:
        """A pass completes: its effect reaches the true ground and the planner's prediction."""
        z, loose = self.z.ravel(), self.loose.ravel()
        np.add.at(self.plan_dz.ravel(), ps.idx, -ps.dz_pred)        # release the commitment
        np.add.at(self.plan_dloose.ravel(), ps.idx, -ps.dloose_pred)
        if ps.op == "compact":
            # Rolling acts on whatever loose fill is there now, in truth and in the planner's belief.
            dz_true = -loose[ps.idx] * (1 - self.soil.keep)
            z[ps.idx] += dz_true
            self.ledger += float(dz_true.sum()) * self.area
            loose[ps.idx] = 0.0
            p_loose, pred = self.p_loose.ravel(), self.pred.ravel()
            self.loose_rolled_pred.ravel()[ps.idx] += p_loose[ps.idx]
            pred[ps.idx] -= p_loose[ps.idx] * (1 - self.est_keep)
            p_loose[ps.idx] = 0.0
            self.touched_comp.ravel()[ps.idx] = True
            return
        np.add.at(z, ps.idx, ps.dz_true)
        np.add.at(loose, ps.idx, ps.dloose_true)
        self.ledger += float(ps.dz_true.sum()) * self.area
        dep = ps.dloose_pred > 0
        np.add.at(self.dep_pred.ravel(), ps.idx[dep], ps.dloose_pred[dep])
        self.touched_dep.ravel()[ps.idx[dep]] = True
        self.touched_other.ravel()[ps.idx[~dep & (ps.dz_pred != 0)]] = True
        np.add.at(self.pred.ravel(), ps.idx, ps.dz_pred)
        np.add.at(self.p_loose.ravel(), ps.idx, ps.dloose_pred)

    # -- main loop -------------------------------------------------------
    def run(self, wire_path: str | None = None) -> dict[str, Any]:
        """Run to completion or `max_minutes`. Optionally write Loam Wire messages as JSON lines."""
        with open(wire_path, "w", encoding="utf-8") if wire_path else contextlib.nullcontext() as f:
            self.wire_out = f
            try:
                self._loop()
            finally:
                self.wire_out = None
        return self.report()

    def _loop(self) -> None:
        self.probe()
        self.check_done()
        while self.t < self.max_minutes:
            for name, ps in list(self.inflight.items()):
                if self.machines[name].busy_until <= self.t:
                    self.land(ps)
                    del self.inflight[name]
            self.probe()
            eff = self.apply_rules()
            if eff["stop"]:
                for name in self.inflight:
                    self.machines[name].busy_until += self.dt
            elif not eff["hold"]:
                for m in self.machines.values():
                    if m.name in self.inflight:
                        continue
                    for name in self.m.order:
                        if self.eligible(name, eff["paused"]) and self.fits(m, name) \
                                and self.plan_pass(m, name, eff["slow"]):
                            break
            self.check_done()
            if all(v is not None for v in self.done_at.values()):
                break
            # Built-in stall rule (spec): an eligible, unfinished task with no work in the plan forces
            # a rebase on the latest survey.
            if not (self.inflight or eff["stop"] or eff["hold"] or self.open_loop) \
                    and self.last_probe > self.last_rebase:
                pending = [n for n in self.m.tasks if self.done_at[n] is None and self.eligible(n, eff["paused"])]
                if pending:
                    self.replan(reason=f"stalled: {', '.join(pending)} not done but nothing left in the plan")
            self.t += self.dt

    # -- results ---------------------------------------------------------
    def ledger_error(self) -> float:
        """Booked volume change minus actual change of true ground (site units^3); ~0 if conserved."""
        return float(self.ledger - (self.z - self.z_start).sum() * self.area)

    def report(self) -> dict[str, Any]:
        yd3 = (3 * FT / self.u) ** 3
        finish = [n for n, z in self.m.zones.items() if z.get("role") == "finish"]
        zone = finish[-1] if finish else next(iter(self.m.zones))
        mk = self.mask(zone)
        tol = float(self.m.zones[zone]["tolerance"].to(self.unit))
        true_r = (self.z - self.design)[mk]
        seen_r = (self.checked_surface() - self.design)[mk]

        def stats(r: Grid) -> dict[str, float]:
            return {"p95_abs": round(float(np.percentile(np.abs(r), 95)), 4),
                    "mean": round(float(r.mean()), 4),
                    "within_tolerance_pct": round(float((np.abs(r) <= tol).mean() * 100), 2)}

        return {
            "mode": "open-loop" if self.open_loop else "closed-loop",
            "minutes": round(self.t, 2),
            "finished": all(v is not None for v in self.done_at.values()),
            "tasks": {n: {"passes": self.task_passes[n], "started_min": self.started[n],
                          "done_min": self.done_at[n]} for n in self.m.tasks},
            "machines": {n: {"passes": m.passes, "busy_min": round(m.busy_min, 2)} for n, m in self.machines.items()},
            "cut_bank_yd3": round(self.cut_bank / yd3, 2),
            "imported_bank_yd3": round(self.import_bank / yd3, 2),
            "exported_bank_yd3": round(self.export_bank / yd3, 2),
            "probes": self.probe_count,
            "replans": len(self.replans),
            "material_estimates": {"swell": round(self.est_swell, 4), "keep": round(self.est_keep, 4),
                                   "true_swell": self.soil.swell, "true_keep": self.soil.keep},
            "rule_events": [{"min": t, "rule": n, "active": on} for t, n, on in self.rule_log],
            "zone": zone, "tolerance": tol, "units": self.unit,
            "true_ground": stats(true_r),
            "as_seen_by_program": stats(seen_r),
            "ledger_error": self.ledger_error(),
        }

    def plot(self, path: str, title: str = "") -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), constrained_layout=True)
        try:
            ext = (self.x0, self.x0 + self.nx * self.cell, self.y0, self.y0 + self.ny * self.cell)
            for ax, data, ttl, lim in ((axes[0], self.z_start - self.design, "Before: existing minus design", 3.0),
                                       (axes[1], self.z - self.design, "After: true ground minus design", 0.4)):
                im = ax.imshow(data, origin="lower", extent=ext, cmap="RdBu_r", vmin=-lim, vmax=lim)
                ax.set_title(ttl, fontsize=10)
                ax.set_xlabel(self.unit)
                fig.colorbar(im, ax=ax, label=self.unit, shrink=0.85)
            if title:
                fig.suptitle(title, fontsize=11)
            fig.savefig(path, dpi=120)
        finally:
            plt.close(fig)


def _overlap(a: Window, b: Window) -> bool:
    return (a[0].start < b[0].stop and b[0].start < a[0].stop
            and a[1].start < b[1].stop and b[1].start < a[1].stop)
