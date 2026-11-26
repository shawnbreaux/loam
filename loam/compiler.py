"""Compile a checked Loam model into an execution manifest (JSON) and
a plain-English explanation."""
from __future__ import annotations

from . import ast as A
from . import wire
from .checker import Model
from .units import Quantity


def q(v):
    """Serialize a Quantity as {value, unit, si}."""
    if isinstance(v, Quantity):
        out = {"value": v.value, "unit": v.unit, "si": round(v.si, 9), "dim": v.dim}
        if v.tol:
            out["tolerance"] = True
        return out
    return v


def ser(obj):
    """Make a checked model JSON-ready: quantities become {value, unit, si}, conditions become their source text, and
    private keys (those starting with `_`) are dropped.
    """
    if isinstance(obj, Quantity):
        return q(obj)
    if isinstance(obj, dict):
        return {k: ser(v) for k, v in obj.items() if not k.startswith("_")}
    if isinstance(obj, (list, tuple)):
        return [ser(v) for v in obj]
    if isinstance(obj, (A.BinOp, A.Call, A.Attr, A.Not, A.Num, A.Qty, A.Ident)):
        return {"expr": A.render(obj)}
    return obj


def manifest(model: Model) -> dict:
    """The execution manifest: everything a controller needs to run the job, as one JSON-ready dict.

    It holds the checked site, probes, machines, zones, stockpiles, tasks (with `task_order`, the order their
    dependencies allow), rules and replans, plus the MQTT topics each party publishes and listens on.
    """
    site = model.site["name"]
    return {
        "loam": model.version,
        "wire": wire.WIRE_VERSION,
        "site": ser(model.site),
        "probes": ser(model.probes),
        "machines": ser(model.machines),
        "zones": ser(model.zones),
        "stockpiles": ser(model.stockpiles),
        "tasks": ser(model.tasks),
        "task_order": model.order,
        "rules": ser(model.rules),
        "replans": ser(model.replans),
        "topics": {
            "terrain": wire.topic(site, None, "terrain"),
            "safety": wire.topic(site, None, "safety"),
            "machines": {m: {k: wire.topic(site, m, k) for k in ("order", "instantAction", "state", "connection")}
                         for m in model.machines},
        },
    }


def by_text(by: dict | None) -> str:
    """Who may do a task, in words: a named machine, any machine of a class, or any machine with a capability."""
    if not by:
        return "any suitable machine"
    if by.get("machine"):
        return by["machine"]
    if by.get("class"):
        return f"any {by['class']}"
    return f"any machine that can {by['cap']}"


def explain(model: Model) -> str:
    """The program in plain English, for the people on site who will not read Loam.

    One line per probe, zone and task (in dependency order, with what each waits for and when it is finished), then
    the safety rules and replan triggers.
    """
    s = model.site
    surf = f" (surface {s['design']['surface']})" if s["design"].get("surface") else ""
    lines = [f"Site '{s['name']}' in {s.get('crs')} ({s.get('units')}), finished surface from "
             f"{s['design']['file']}{surf}."]
    for p in model.probes.values():
        lines.append(f"Probe '{p['name']}': {p.get('kind', '?').replace('_', ' ')} {p.get('sensor', '')} "
                     f"every {p['rate']}, ±{p['accuracy'].value:g} {p['accuracy'].unit}; "
                     f"stale after {p['stale_after']}.")
    lines.append("Machines: " + ", ".join(f"{n} ({m['class']})" for n, m in model.machines.items()) + ".")
    for z in model.zones.values():
        extra = []
        if z.get("lift"):
            extra.append(f"lifts of {z['lift']}")
        if z.get("compaction"):
            extra.append(f"compacted to {z['compaction']}")
        lines.append(f"Zone '{z['name']}' is a {z['role']} area, tolerance {z['tolerance']}"
                     + (f", {', '.join(extra)}" if extra else "") + ".")
    lines.append("")
    lines.append("Work, in dependency order:")
    for i, n in enumerate(model.order, 1):
        t = model.tasks[n]
        parts = [f"{i}. {n}: {t['op']}"]
        if t.get("zone"):
            parts.append(f"zone {t['zone']}")
        parts.append(f"with {by_text(t.get('by'))}")
        if t.get("max_cut"):
            parts.append(f"at most {t['max_cut']} per pass")
        if t.get("spoil"):
            parts.append(f"spoil to {' then '.join(t['spoil'])}")
        if t.get("source"):
            parts.append(f"material from {', '.join(t['source'])}")
        if t.get("passes"):
            parts.append(f"{t['passes']} roller passes per lift")
        deps = [f"{d['task']} is {'started' if d['when'] in ('start', 'lift') else 'done'}" for d in t.get("after", [])]
        if deps:
            parts.append("once " + " and ".join(deps))
        lines.append(", ".join(parts) + f"; finished when {A.render(t['until'])}.")
    if model.rules:
        lines.append("")
        lines.append("Safety and site rules:")
        for r in model.rules.values():
            act = r["do"]
            tgt = act.get("target") or {}
            tt = "everything" if tgt.get("all") else (f"task {tgt['task']}" if tgt.get("task")
                                                      else f"class {tgt.get('class')}" if tgt.get("class")
                                                      else ", ".join(tgt.get("names", [])))
            lines.append(f"- {r['name']}: when {A.render(r['when'])}, {act['verb']} {tt}".rstrip() + ".")
    for rp in model.replans.values():
        lines.append(f"Replan '{rp['name']}' when {A.render(rp['when'])}.")
    return "\n".join(lines)
