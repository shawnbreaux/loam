"""The simulated site: true ground, soil physics, site sensors and the clock.

The world is the only process that knows the true ground. Machines tell it when
a pass is complete (`sim/<site>/world/apply`) and it moves the earth; the probe
asks it for a scan (`sim/<site>/world/scan`) and adds its own noise. Nothing on
`sim/...` is part of Loam Wire: it stands in for physics, and a real site has
none of it. The controller never subscribes to it.

Soil truth differs from the planner's beliefs exactly as in the reference
simulator (swell 1.30 vs 1.25, compaction keep 0.70 vs 0.80, blade cut error),
and faults are scripted in site minutes (see `DEFAULT_FAULTS`).

With roller tables from `loam soil-calibrate` (Project Chrono SCM), compaction
follows the roller's settlement-per-pass curve, capped at full compaction of the
loose lift, instead of happening all at once.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from .. import wire
from ..checker import Model
from ..grid import SiteGrid, polygon_mask
from ..sim import Soil
from .bus import Bus, Clock, tolerant

# Site minutes. Machines read their own entries; the world runs the site-level ones.
SENSOR_REPEAT_S = 5.0        # a simulated sensor repeats an unchanged "clear" this often (wall seconds)

DEFAULT_FAULTS: dict[str, Any] = {
    "person": [{"from": 42, "to": 45, "near": "dz1", "distance_m": 9.0}],
    "probe_outage": [{"from": 95, "to": 118}],
    "rain": [{"from": 150, "to": 175, "rate_m_per_s": 0.4 * 0.0254 / 3600}],
    "gnss_float": [{"machine": "dz2", "from": 60, "to": 70}],
    "link_loss": [{"machine": "dz1", "at": 130, "wall_s": 4.0}],
    "estop": [{"at": 200, "for_min": 3, "resume_after_min": 1}],   # a person resumes after a walk-around
    "crash": [],                      # e.g. {"machine": "dz2", "at": 240}: the process dies
}


def active(spans: list[dict], t: float) -> dict | None:
    """The scripted fault that is under way at site minute `t`, or None."""
    return next((s for s in spans if s["from"] <= t < s["to"]), None)


class World:
    """The true state of the simulated site: the ground, the loose soil lying on it, and the stockpiles.

    Volumes are in site units cubed. "Bank" volume is soil measured in the ground, before digging; "loose" volume
    is the same soil after digging, when it takes up more room (the swell factor). Everything here is hidden from
    the controller, which only sees what the probe measures.
    """
    def __init__(self, model: Model, base_dir: str, seed: int = 7, soil: Soil | None = None,
                 compaction: dict[str, dict] | None = None):
        """Start from the existing ground. `soil` is the true soil behaviour; `compaction` holds per-machine roller
        tables from `loam soil-calibrate`; `seed` fixes the random cut and finish errors.
        """
        self.g = SiteGrid(model, base_dir)
        self.m = model
        self.design, self.z = self.g.design_and_existing()
        self.z0 = self.z.copy()
        # Height of loose (placed, not yet compacted) soil in each cell. Rolling squeezes it down.
        self.loose = np.zeros_like(self.z)
        self.rolled = np.zeros(self.z.shape, int)       # roller passes over the loose lift so far
        self.compaction = compaction or {}
        self.soil = soil or Soil()
        self.rng = np.random.default_rng(seed)
        self.u, self.area = self.g.u, self.g.area
        self.stock: dict[str, float] = {}
        for name, sp in model.stockpiles.items():
            v = sp.get("available") or sp.get("capacity")
            self.stock[name] = v.to(f"{self.g.unit}3") if v else 0.0
        # Windrow: soil a blade leaves in a ridge beside its pass. It is on site but in nobody's plan until finishing
        # uses it.
        self.windrow = 0.0                  # bank volume left lying on site
        # The ledger is a bookkeeping check: the sum of every change made through `_add` must equal the change in the
        # ground itself (see `truth`).
        self.ledger = 0.0                   # booked change of ground volume
        self.applied = 0
        self.seen_orders: set[str] = set()
        self.duplicates = 0

    def _mask(self, poly: list | None) -> np.ndarray:
        """The grid cells inside a polygon given as a list of points (none, for no polygon)."""
        if not poly:
            return np.zeros(self.g.shape, bool)
        return polygon_mask([tuple(p) for p in poly], self.g.xs, self.g.ys)

    def _add(self, mk: np.ndarray, dz: np.ndarray, loose: bool = False) -> None:
        """Change the ground height of the cells `mk` by `dz`, and book the volume in the ledger. `loose=True` marks
        it as newly placed loose soil.
        """
        self.z[mk] += dz
        if loose:
            self.loose[mk] += dz
            self.rolled[mk] = 0                          # new loose material: its rolling starts over
        self.ledger += float(dz.sum()) * self.area

    def place(self, mk: np.ndarray, lift: float, loose_vol: float) -> float:
        """Spread loose material into the lowest cells of `mk`, overbuilt for compaction, at most
        one lift thick. Returns the loose volume placed."""
        # Fill is placed higher than design because rolling will press it down: the height still needed, divided by
        # the planner's 0.8 keep factor, and no more than one lift.
        need = np.clip((self.design - self.z) / 0.8, 0, lift)
        flat = np.where(mk, need, 0).ravel()
        # Deepest holes first, until the load runs out; the last cell takes what is left.
        order = np.argsort(-flat, kind="stable")
        cum = np.cumsum(flat[order]) * self.area
        k = int(np.searchsorted(cum, loose_vol))
        amt = np.zeros(flat.size)
        if k >= flat.size or cum[-1] <= loose_vol:
            amt[order] = flat[order]
        else:
            amt[order[:k]] = flat[order[:k]]
            amt[order[k]] = loose_vol - (cum[k - 1] if k else 0.0)
        amt = amt.reshape(self.g.shape)
        sel = amt > 0
        self._add(sel, amt[sel], loose=True)
        return float(amt.sum() * self.area)

    def spread(self, dep: dict, loose_vol: float) -> float:
        """Spread `loose_vol` to the deposit's planned pattern, scaled to what is really there."""
        tile = wire.decode_heights(dep["pattern"], tuple(dep["shape"])).astype(float) / self.u
        total = float(tile.sum() * self.area)
        if total <= 0 or loose_vol <= 0:
            return 0.0
        p = np.asarray(dep["window"], float)
        i0 = round((p[:, 0].min() - self.g.x0) / self.g.cell)
        j0 = round((p[:, 1].min() - self.g.y0) / self.g.cell)
        r, c = tile.shape
        amt = tile * (loose_vol / total)
        mk = np.zeros(self.g.shape, bool)
        mk[j0:j0 + r, i0:i0 + c] = amt > 0
        self._add(mk, amt[amt > 0], loose=True)
        return loose_vol

    def apply(self, order: dict) -> dict:
        """A machine finished a pass: move the earth it moved."""
        op, u = order["op"], self.u
        if order["orderId"] in self.seen_orders:      # physics happens once, however often it is reported
            self.duplicates += 1
            return {"orderId": order["orderId"], "op": op, "duplicate": True}
        self.seen_orders.add(order["orderId"])
        lim, dep = order.get("limits") or {}, order.get("deposit") or {}
        thr = float((order.get("target") or {}).get("tolerance", 0.0)) / u
        win = self._mask(order.get("window"))
        r = self.z - self.design
        tgt = order.get("target") or {}
        planned = None                       # planned change per cell (m), where the order gives one
        if tgt.get("pattern") and order.get("window"):
            tile = wire.decode_heights(tgt["pattern"], tuple(tgt["shape"])).astype(float) / u
            p = np.asarray(order["window"], float)
            i0 = round((p[:, 0].min() - self.g.x0) / self.g.cell)
            j0 = round((p[:, 1].min() - self.g.y0) / self.g.cell)
            planned = np.zeros(self.g.shape)
            planned[j0:j0 + tile.shape[0], i0:i0 + tile.shape[1]] = tile
        out: dict[str, Any] = {"orderId": order["orderId"], "op": op}
        if op == "cut":
            if planned is not None:          # grade control cuts the planned depth, never below design
                sel = planned < 0
                d = -planned[sel] * np.clip(1 + self.rng.normal(0, self.soil.cut_error, int(sel.sum())), 0, None)
                d = np.minimum(d, np.clip(r[sel], 0, None))
            else:
                sel = win & (r > thr)
                # Without a planned pattern the blade takes off whatever is above design, up to the deepest cut
                # allowed in one pass (default 6 in).
                d = np.clip(r[sel], 0, lim.get("maxCut", 0.1524) / u)
                d *= np.clip(1 + self.rng.normal(0, self.soil.cut_error, d.shape), 0, None)
            self._add(sel, -d)
            self.loose[sel] = np.maximum(self.loose[sel] - d, 0)
            bank = float(d.sum() * self.area)
            # Dug soil swells. It goes, in order of preference, to the planned deposit, to the stockpile, and
            # otherwise stays on site as windrow.
            loose = bank * self.soil.swell
            if dep.get("pattern"):
                share = float(dep["volume"]) / max(float(dep.get("planned", dep["volume"])), 1e-12)
                loose -= self.spread(dep, loose * min(share, 1.0))
            elif dep.get("window"):
                loose -= self.place(self._mask(dep["window"]), dep.get("lift", 0.2032) / u, loose)
            sp = dep.get("stockpile") or dep.get("overflow")
            if loose > 1e-12 and sp in self.stock:
                self.stock[sp] += loose / self.soil.swell
                loose = 0.0
            self.windrow += max(loose, 0.0) / self.soil.swell
            out["bank"] = bank
        elif op in ("fill", "spread"):
            src = str(dep.get("source"))
            if dep.get("pattern") and "bank" in dep:
                bank = min(self.stock.get(src, 0.0), float(dep["bank"]) / u ** 3)
                placed = self.spread(dep, bank * self.soil.swell)
            else:
                have = self.stock.get(src, 0.0) * self.soil.swell
                want = float(dep.get("volume", 0.0)) / u ** 3
                placed = self.place(self._mask(dep.get("window")), dep.get("lift", 0.2032) / u, min(have, want))
            if src in self.stock:
                self.stock[src] -= placed / self.soil.swell
            out["bank"] = placed / self.soil.swell
        elif op == "compact":
            # Rolling only changes cells that have loose soil. Without a roller table the whole lift compacts at once
            # to the true keep factor.
            sel = win & (self.loose > 1e-9)
            table = self.compaction.get(order.get("machine", "")) or self.compaction.get("*")
            if table is None:
                self._add(sel, -self.loose[sel] * (1 - self.soil.keep))
                self.loose[sel] = 0.0
            else:
                from .chrono_soil import settlement
                k = int(lim.get("passes", 4))
                k0 = self.rolled[sel]
                inc = np.array([settlement(table, a + k) - settlement(table, a) for a in k0]) / u
                room = self.loose[sel] * (1 - self.soil.keep)        # what is left to compress in the lift
                dz = np.minimum(inc, room)
                self._add(sel, -dz)
                self.loose[sel] -= dz / (1 - self.soil.keep)
                self.loose[sel & (self.loose < 1e-9)] = 0.0
                self.rolled[sel] += k
                out["settled_mean"] = float(dz.mean() * u) if dz.size else 0.0
        elif op == "finish":
            # A finishing pass moves the surface at most one step (default 0.3 ft) towards design, with a small
            # random error. What it cuts joins the windrow; what it fills is drawn from the windrow first, then
            # the stockpile.
            step = lim.get("step", 0.0914) / u
            if planned is not None:
                # Grade control aims the blade at the planned change but never past design: a cut stops there,
                # and so does a fill. The random error is the blade's, about that aim.
                sel = planned != 0
                aim = np.clip(planned[sel], np.minimum(-r[sel], 0.0), np.maximum(-r[sel], 0.0))
                dz = aim + self.rng.normal(0, self.soil.finish_error_m / u, int(sel.sum()))
            else:
                sel = win & (np.abs(r) > thr)
                dz = np.clip(-r[sel], -step, step) + self.rng.normal(0, self.soil.finish_error_m / u, int(sel.sum()))
            cut, fill = np.clip(-dz, 0, None), np.clip(dz, 0, None)
            self.windrow += float(cut.sum() * self.area)
            need = float(fill.sum() * self.area) / (self.soil.swell * self.soil.keep)
            draw = min(need, self.windrow)
            self.windrow -= draw
            src = str(dep.get("source"))
            if src in self.stock and need > draw:
                take = min(self.stock[src], need - draw)
                self.stock[src] -= take
                draw += take
            fill *= draw / need if need > 1e-12 else 0.0
            self._add(sel, fill - cut)
        self.applied += 1
        return out

    def truth(self) -> dict:
        """Ground-truth score of the finish zones, for the test report only."""
        res: dict[str, Any] = {}
        for name, zd in self.m.zones.items():
            if zd.get("role") != "finish" or name not in self.g.masks or self.g.masks[name] is None:
                continue
            mk = self.g.masks[name]
            tol = zd["tolerance"].to(self.g.unit) if zd.get("tolerance") else 0.1
            e = (self.z - self.design)[mk]
            res[name] = {"within_tolerance_pct": round(100 * float((np.abs(e) <= tol + 1e-9).mean()), 2),
                         "p95_abs": round(float(np.percentile(np.abs(e), 95)), 4), "tolerance": tol}
        return {"zones": res, "ledger_error": float(self.ledger - (self.z - self.z0).sum() * self.area),
                "stockpiles": {k: round(v, 2) for k, v in self.stock.items()}, "windrow": round(self.windrow, 2),
                "passes_applied": self.applied, "duplicate_reports": self.duplicates}


def run_world(model: Model, base_dir: str, broker: str, speed: float, faults: dict, seed: int = 7,
              truth_every_min: float = 5.0, stop_after_min: float = 48 * 60,
              compaction: dict[str, dict] | None = None) -> None:
    """Run the simulated world until the site ends or `stop_after_min` site minutes pass.

    It publishes the site clock and the fault script, moves earth when a machine reports a finished pass, answers
    the probe's scan requests, and plays the site's sensors (person detection, rain gauge, e-stop button) from the
    fault script. Every `truth_every_min` it publishes the true score for the test report, and a final one when it
    stops.
    """
    site = model.site["name"]
    w = World(model, base_dir, seed, compaction=compaction)
    bus = Bus(broker, f"loam-world-{site}").start([f"sim/{site}/world/#", f"loam/v0/{site}/+/state",
                                                     f"sim/{site}/end"])
    clock = Clock(time.time() + 3.0, speed, 1796108400.0)  # starts in 3 s, at 2026-12-01 07:00 UTC
    bus.pub(f"sim/{site}/clock", clock.msg(), retain=True, qos=1)
    bus.pub(f"sim/{site}/faults", faults, retain=True, qos=1)
    poses: dict[str, tuple[float, float]] = {}
    seq, last_truth, estop_on = 0, -1e9, False
    resumed: set[int] = set()
    person_out: str | None = None                            # what the camera last reported: "person" or "clear"
    person_sent = 0.0                                        # wall time of that report
    rain_out: float | None = None                            # the rain rate the gauge last reported
    rain_sent = -1e9                                         # site minute of that report
    try:
        while clock.minutes() < stop_after_min:
            t = clock.minutes()
            for topic, msg in bus.drain(0.02):
                with tolerant(topic):
                    if topic == f"sim/{site}/end":
                        return
                    if topic.endswith("/state"):
                        poses[msg["machine"]] = (msg["pose"]["x"], msg["pose"]["y"])
                    elif topic == f"sim/{site}/world/apply":
                        res = w.apply(msg["order"])
                        res["t"] = t
                        bus.pub(f"sim/{site}/world/applied/{msg['machine']}", res, qos=1)
                    elif topic == f"sim/{site}/world/scan":
                        bus.pub(f"sim/{site}/world/scan/{msg['probe']}",
                                {"data": wire.encode_heights(w.z), "shape": list(w.g.shape), "t": t}, qos=1)
            # Site sensors: a camera that sees a person near a machine, a rain gauge, an e-stop button. Each
            # reports a change at once and then keeps repeating what it sees, as a real sensor must: a message can
            # be lost (the broker restarts, the controller connects a moment after it was sent), and a lost
            # "clear" or "the rain has stopped" would hold the site for good.
            now = time.time()
            p = active(faults.get("person", []), t) or {}
            seen = p.get("near") in poses
            if seen and now - person_sent >= 0.5:            # while in view: twice a second (wall)
                person_sent, person_out = now, "person"
                seq += 1
                x, y = poses[p["near"]]
                d = p.get("distance_m", 9.0) / w.u
                bus.pub(f"loam/v0/{site}/detection",
                        wire.detection(site, "camera1", seq, "person", x + d, y, ts=clock.ts(t)), qos=1)
            elif not seen and (person_out == "person"
                               or (person_out == "clear" and now - person_sent >= SENSOR_REPEAT_S)):
                person_sent, person_out = now, "clear"       # gone: at once, then every few seconds
                seq += 1
                bus.pub(f"loam/v0/{site}/detection",
                        wire.detection(site, "camera1", seq, "clear", 0, 0, ts=clock.ts(t)), qos=1)
            r = active(faults.get("rain", []), t)
            rate = (r or {}).get("rate_m_per_s", 0.0) if r else 0.0
            # The gauge reports when the rain rate changes, and every 5 site minutes, rain or none.
            if rate != rain_out or t - rain_sent >= 5.0:
                rain_out, rain_sent = rate, t
                seq += 1
                bus.pub(f"loam/v0/{site}/weather",
                        wire.weather(site, "gauge1", seq, rain_m_per_s=rate, ts=clock.ts(t)), qos=1)
            e = next((s for s in faults.get("estop", []) if s["at"] <= t < s["at"] + s["for_min"]), None)
            if bool(e) != estop_on:
                seq += 1
                # Retained, like `loam estop`: a machine that connects during an e-stop must learn of it.
                bus.pub(f"loam/v0/{site}/estop", {"site": site, "pressed": bool(e), "ts": clock.ts(t),
                                                  "wall": time.time()}, qos=1, retain=True)
                estop_on = bool(e)
            # releasing the e-stop restarts nothing (ISO 13850): the site supervisor resumes the machines
            r = next((s for s in faults.get("estop", []) if s.get("resume_after_min") is not None
                      and s["at"] + s["for_min"] + s["resume_after_min"] <= t and id(s) not in resumed), None)
            if r is not None:
                resumed.add(id(r))
                seq += 1
                bus.pub(f"loam/v0/{site}/safety", wire.safety(site, seq, "resume", "e-stop reset, site checked",
                                                              ts=clock.ts(t)), qos=1)
            if t - last_truth >= truth_every_min:
                last_truth = t
                bus.pub(f"sim/{site}/world/truth", {"t": t, **w.truth()}, retain=True)
            time.sleep(0.005)
    finally:
        bus.pub(f"sim/{site}/world/truth", {"t": clock.minutes(), "final": True, **w.truth()}, retain=True, qos=1)
        time.sleep(0.2)
        bus.stop()

