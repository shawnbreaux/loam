"""Semantic checks for Loam programs.

`check(program)` returns a `Model` (resolved declarations, quantities kept
with their units) plus a list of diagnostics. A program is valid when no
diagnostic has level "error".
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from . import ast as A
from .units import Quantity, UnitError

SUPPORTED_VERSIONS = {"0.0.1"}

ENUMS = {
    "probe.kind": {"tethered_uav", "uav", "machine", "rover", "static"},
    "probe.sensor": {"lidar", "photogrammetry", "radar", "track"},
    "machine.class": {"dozer", "excavator", "loader", "grader", "roller", "truck", "scraper", "ctl"},
    "caps": {"cut", "push", "spread", "dig", "load", "trench", "haul", "compact", "finish"},
    "zone.role": {"cut", "fill", "finish", "exclusion"},
    "task.op": {"cut", "fill", "spread", "compact", "trench", "haul", "finish", "load"},
    "site.units": {"ft", "m"},
    "design.format": {"landxml", "grid", "ifc"},
    "action": {"stop", "hold", "pause", "resume", "slow", "alert"},
    "subject": {"person", "machine", "vehicle"},
    "weather": {"rain", "wind"},
}

DEFAULT_CAPS = {
    "dozer": ["cut", "push", "spread", "finish"],
    "excavator": ["dig", "load", "trench", "cut"],
    "loader": ["load", "push", "haul"],
    "grader": ["finish", "spread"],
    "roller": ["compact"],
    "truck": ["haul"],
    "scraper": ["cut", "haul", "spread"],
    "ctl": ["cut", "push", "spread", "finish", "load"],
}

# which capability a task op needs
OP_CAP = {"cut": "cut", "fill": "spread", "spread": "spread", "compact": "compact",
          "trench": "trench", "haul": "haul", "finish": "finish", "load": "load"}

# function name -> (argument kinds, result type)
FUNCTIONS = {
    "residual": (["zone"], "stat"),
    "in_tolerance": (["zone"], "bool"),
    "volume_remaining": (["zone"], "volume"),
    "compaction": (["zone"], "ratio"),
    "age": (["probe"], "time"),
    "proximity": (["subject", "subject"], "length"),
    "weather": (["weather"], "any_rate"),
    "progress": (["task"], "ratio"),
    "done": (["task"], "bool"),
    "deviation": ([], "length"),
    "elapsed": ([], "time"),
}
EXAMPLE_UNIT = {"length": "ft", "time": "min", "volume": "yd3", "speed": "mph", "ratio": "%"}
STAT_ATTRS = {"p95", "high", "low", "max", "min", "mean"}


@dataclass
class Diagnostic:
    level: str  # error | warning
    msg: str
    line: int
    col: int

    def format(self, src: str) -> str:
        return f"{src}:{self.line}:{self.col}: {self.level}: {self.msg}"


@dataclass
class Model:
    version: str
    site: dict = field(default_factory=dict)
    probes: dict = field(default_factory=dict)
    machines: dict = field(default_factory=dict)
    zones: dict = field(default_factory=dict)
    stockpiles: dict = field(default_factory=dict)
    tasks: dict = field(default_factory=dict)
    rules: dict = field(default_factory=dict)
    replans: dict = field(default_factory=dict)
    order: list = field(default_factory=list)  # tasks in dependency order


class Checker:
    def __init__(self, prog: A.Program):
        self.prog = prog
        self.diags: list[Diagnostic] = []
        self.model = Model(version=prog.version)
        self.names: dict[str, A.Decl] = {}

    # -- reporting -------------------------------------------------------
    def error(self, msg: str, pos: A.Pos) -> None:
        self.diags.append(Diagnostic("error", msg, pos.line, pos.col))

    def warn(self, msg: str, pos: A.Pos) -> None:
        self.diags.append(Diagnostic("warning", msg, pos.line, pos.col))

    # -- value helpers ---------------------------------------------------
    def one(self, s: A.Stmt, n: int = 1) -> bool:
        if len(s.args) != n:
            self.error(f"'{s.key}' takes {n} value{'s' if n > 1 else ''}, got {len(s.args)}", s.pos)
            return False
        return True

    def qty(self, node, dim: str | tuple, what: str, allow_tol: bool = False):
        dims = (dim,) if isinstance(dim, str) else dim
        if isinstance(node, A.Num):
            example = EXAMPLE_UNIT.get(dims[0], "")
            self.error(f"{what} needs a unit ({' or '.join(dims)}), e.g. '{node.value:g} {example}'", node.pos)
            return None
        if not isinstance(node, A.Qty):
            self.error(f"{what} must be a quantity with a unit", node.pos)
            return None
        try:
            d = node.q.dim
        except UnitError as e:
            self.error(str(e), node.pos)
            return None
        if d not in dims:
            self.error(f"{what} must be a {' or '.join(dims)}, but '{node.q}' is a {d}", node.pos)
            return None
        if node.q.tol and not allow_tol:
            self.error("'±' is only allowed on tolerances", node.pos)
        return node.q

    def enum(self, node, key: str, what: str):
        if not isinstance(node, A.Ident):
            self.error(f"{what} must be one of: {', '.join(sorted(ENUMS[key]))}", node.pos)
            return None
        if node.name not in ENUMS[key]:
            self.error(f"unknown {what} '{node.name}'; expected one of: {', '.join(sorted(ENUMS[key]))}",
                       node.pos)
            return None
        return node.name

    def string(self, node, what: str):
        if not isinstance(node, A.Str):
            self.error(f"{what} must be a quoted string", node.pos)
            return None
        return node.value

    def ref(self, node, kinds: tuple, what: str):
        if not isinstance(node, A.Ident):
            self.error(f"{what} must name a {' or '.join(kinds)}", node.pos)
            return None
        d = self.names.get(node.name)
        if d is None:
            self.error(f"{what}: no {' or '.join(kinds)} named '{node.name}'", node.pos)
            return None
        if d.kind not in kinds:
            self.error(f"{what}: '{node.name}' is a {d.kind}, expected a {' or '.join(kinds)}", node.pos)
            return None
        return node.name

    def area(self, s: A.Stmt):
        a = s.args
        if not a or not isinstance(a[0], A.Ident):
            self.error("area must be 'polygon [...]', 'rect (x, y) (x, y)', 'file \"...\"' "
                       "or 'where <condition>'", s.pos)
            return None
        form = a[0].name
        if form == "polygon":
            if len(a) != 2 or not isinstance(a[1], A.ListLit) or not all(isinstance(p, A.Point) for p in a[1].items):
                self.error("polygon needs a list of points: polygon [(x, y), (x, y), (x, y)]", s.pos)
                return None
            pts = [(p.x, p.y) for p in a[1].items]
            if len(pts) < 3:
                self.error("a polygon needs at least 3 points", s.pos)
                return None
            return {"polygon": pts}
        if form == "rect":
            if len(a) != 3 or not all(isinstance(p, A.Point) for p in a[1:]):
                self.error("rect needs two corner points: rect (x1, y1) (x2, y2)", s.pos)
                return None
            (x1, y1), (x2, y2) = (a[1].x, a[1].y), (a[2].x, a[2].y)
            x1, x2 = sorted((x1, x2))
            y1, y2 = sorted((y1, y2))
            return {"polygon": [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]}
        if form == "file":
            if len(a) != 2 or not isinstance(a[1], A.Str):
                self.error('file needs a path: file "boundary.geojson"', s.pos)
                return None
            return {"file": a[1].value}
        if form == "where":
            t = self.expr_type(a[1], frozenset({"cell"}))
            if t and t != "bool":
                self.error("'area where' needs a true/false condition", s.pos)
            return {"where": a[1]}
        self.error(f"unknown area form '{form}'", a[0].pos)
        return None

    # -- expressions -----------------------------------------------------
    def expr_type(self, node: Any, scope: frozenset[str] = frozenset()) -> str | None:
        """Return the type of an expression: a dimension name, 'bool',
        'stat', 'number', or None after reporting an error."""
        if isinstance(node, A.Num):
            return "number"
        if isinstance(node, A.Qty):
            try:
                return node.q.dim
            except UnitError as e:
                self.error(str(e), node.pos)
                return None
        if isinstance(node, A.Ident):
            if "cell" in scope and node.name in ("residual", "design", "actual"):
                return "length"
            self.error(f"'{node.name}' is not a value here; did you mean a function like residual({node.name})?",
                       node.pos)
            return None
        if isinstance(node, A.Str):
            self.error("strings cannot be used in conditions", node.pos)
            return None
        if isinstance(node, A.Call):
            spec = FUNCTIONS.get(node.fn)
            if spec is None:
                self.error(f"unknown function '{node.fn}'; known: {', '.join(sorted(FUNCTIONS))}", node.pos)
                return None
            kinds, result = spec
            if len(node.args) != len(kinds):
                self.error(f"{node.fn}() takes {len(kinds)} argument(s), got {len(node.args)}", node.pos)
                return None
            for arg, kind in zip(node.args, kinds, strict=True):
                if kind == "zone":
                    self.ref(arg, ("zone", "stockpile"), f"{node.fn}()")
                elif kind == "probe":
                    self.ref(arg, ("probe",), f"{node.fn}()")
                elif kind == "task":
                    self.ref(arg, ("task",), f"{node.fn}()")
                elif kind == "subject":
                    if not (isinstance(arg, A.Ident) and (arg.name in ENUMS["subject"]
                                                          or arg.name in self.names)):
                        self.error(f"{node.fn}() arguments must be person, machine, vehicle or a declared name",
                                   arg.pos)
                elif kind == "weather":
                    self.enum(arg, "weather", "weather input")
            return result
        if isinstance(node, A.Attr):
            t = self.expr_type(node.obj, scope)
            if t != "stat":
                if t is not None:
                    self.error(f"'.{node.name}' only applies to residual(...)", node.pos)
                return None
            if node.name not in STAT_ATTRS:
                self.error(f"unknown statistic '.{node.name}'; use one of {', '.join(sorted(STAT_ATTRS))}", node.pos)
                return None
            return "length"
        if isinstance(node, A.Not):
            t = self.expr_type(node.expr, scope)
            if t and t != "bool":
                self.error("'not' needs a true/false condition", node.pos)
            return "bool"
        if isinstance(node, A.BinOp):
            lt = self.expr_type(node.left, scope)
            rt = self.expr_type(node.right, scope)
            if lt is None or rt is None:
                return None
            lt = "length" if lt == "stat" else lt
            rt = "length" if rt == "stat" else rt
            if node.op in ("and", "or"):
                if lt != "bool" or rt != "bool":
                    self.error(f"'{node.op}' joins true/false conditions", node.pos)
                return "bool"
            if node.op in ("<", "<=", ">", ">=", "==", "!="):
                if "any_rate" in (lt, rt):
                    other = rt if lt == "any_rate" else lt
                    if other not in ("rate", "speed"):
                        self.error("weather() compares against a rate (in/h) or a speed (mph)", node.pos)
                    return "bool"
                if (lt == "number") != (rt == "number"):
                    dim = rt if lt == "number" else lt
                    self.error(f"comparing a {dim} to a bare number: add a unit", node.pos)
                    return "bool"
                if lt != rt:
                    self.error(f"cannot compare a {lt} with a {rt}", node.pos)
                return "bool"
            if node.op in ("+", "-"):
                if lt != rt and "number" not in (lt, rt):
                    self.error(f"cannot add a {lt} and a {rt}", node.pos)
                return lt if lt != "number" else rt
            if node.op in ("*", "/"):
                if "number" in (lt, rt):
                    return lt if lt != "number" else rt
                self.error("only multiplying or dividing by a plain number is supported", node.pos)
                return None
        self.error("unsupported expression", getattr(node, "pos", A.Pos(0, 0)))
        return None

    def condition(self, s: A.Stmt):
        t = self.expr_type(s.args[0])
        if t is not None and t != "bool":
            self.error(f"'{s.key}' needs a true/false condition, e.g. residual(A) < 0.2 ft", s.pos)
        return s.args[0]

    # -- declarations ----------------------------------------------------
    def run(self) -> tuple[Model, list[Diagnostic]]:
        if self.prog.version not in SUPPORTED_VERSIONS:
            self.error(f"unsupported Loam version {self.prog.version}; this checker supports "
                       f"{', '.join(sorted(SUPPORTED_VERSIONS))}", A.Pos(1, 1))
        for d in self.prog.decls:
            if d.name in self.names:
                prev = self.names[d.name]
                self.error(f"'{d.name}' is already declared as a {prev.kind} at line {prev.pos.line}", d.pos)
            else:
                self.names[d.name] = d
        sites = self.prog.of("site")
        if len(sites) != 1:
            self.error(f"a program needs exactly one site, found {len(sites)}",
                       sites[1].pos if len(sites) > 1 else A.Pos(1, 1))
        kind_order = ["site", "probe", "machine", "zone", "stockpile", "task", "rule", "replan"]
        for d in sorted(self.prog.decls, key=lambda d: kind_order.index(d.kind)):
            seen: dict[str, A.Stmt] = {}
            for s in d.stmts:
                if s.key in seen and s.key not in ("after",):
                    self.error(f"'{s.key}' is set twice in {d.kind} '{d.name}'", s.pos)
                seen[s.key] = s
            getattr(self, f"decl_{d.kind}")(d)
        self.check_tasks_graph()
        self.diags.sort(key=lambda d: (d.line, d.col))
        return self.model, self.diags

    def unknown_keys(self, d: A.Decl, allowed: set) -> None:
        for s in d.stmts:
            if s.key not in allowed:
                self.error(f"unknown field '{s.key}' in {d.kind}; allowed: {', '.join(sorted(allowed))}", s.pos)

    def required(self, d: A.Decl, keys: list) -> None:
        for k in keys:
            if d.get(k) is None:
                self.error(f"{d.kind} '{d.name}' is missing required field '{k}'", d.pos)

    def decl_site(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"crs", "units", "datum", "design", "boundary", "grid", "existing"})
        self.required(d, ["crs", "units", "design"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("crs")) and self.one(s):
            out["crs"] = self.string(s.args[0], "crs")
        if (s := d.get("units")) and self.one(s):
            out["units"] = self.enum(s.args[0], "site.units", "site units")
        if (s := d.get("datum")) and self.one(s):
            out["datum"] = self.string(s.args[0], "datum")
        if (s := d.get("design")):
            out["design"] = self.surface_source(s)
        if (s := d.get("existing")):
            if len(s.args) == 1 and isinstance(s.args[0], A.Ident):
                out["existing"] = {"probe": self.ref(s.args[0], ("probe",), "existing")}
            else:
                out["existing"] = self.surface_source(s)
        if (s := d.get("boundary")):
            s2 = A.Stmt("area", s.args, s.pos)
            out["boundary"] = self.area(s2)
        if (s := d.get("grid")) and self.one(s):
            out["grid"] = self.qty(s.args[0], "length", "grid cell size")
        self.model.site = out

    def surface_source(self, s: A.Stmt):
        a = s.args
        if not a or not isinstance(a[0], A.Str):
            self.error(f'{s.key} needs a file path: {s.key} "surface.xml" format landxml', s.pos)
            return None
        out: dict[str, Any] = {"file": a[0].value, "format": "landxml"}
        rest = a[1:]
        i = 0
        while i < len(rest):
            k = rest[i]
            if not isinstance(k, A.Ident) or k.name not in ("format", "surface") or i + 1 >= len(rest):
                self.error(f"expected 'format <name>' or 'surface \"name\"' after the {s.key} path", k.pos)
                return out
            v = rest[i + 1]
            if k.name == "format":
                out["format"] = self.enum(v, "design.format", "surface format")
            else:
                out["surface"] = self.string(v, "surface name")
            i += 2
        if out.get("format") == "ifc":
            self.warn("IFC surfaces are part of the spec but not yet read by this reference tool", s.pos)
        return out

    def decl_probe(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"kind", "sensor", "rate", "accuracy", "coverage", "stale_after", "endpoint"})
        self.required(d, ["kind", "rate", "accuracy"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("kind")) and self.one(s):
            out["kind"] = self.enum(s.args[0], "probe.kind", "probe kind")
        if (s := d.get("sensor")) and self.one(s):
            out["sensor"] = self.enum(s.args[0], "probe.sensor", "sensor")
        if (s := d.get("rate")) and self.one(s):
            out["rate"] = self.qty(s.args[0], "time", "probe rate")
        if (s := d.get("accuracy")) and self.one(s):
            out["accuracy"] = self.qty(s.args[0], "length", "probe accuracy")
        if (s := d.get("stale_after")) and self.one(s):
            out["stale_after"] = self.qty(s.args[0], "time", "stale_after")
        elif out.get("rate"):
            r = out["rate"]
            out["stale_after"] = Quantity(r.value * 3, r.unit)
        if (s := d.get("coverage")):
            if len(s.args) == 1 and isinstance(s.args[0], A.Ident) and s.args[0].name == "site":
                out["coverage"] = "site"
            else:
                out["coverage"] = self.area(A.Stmt("area", s.args, s.pos))
        else:
            out["coverage"] = "site"
        if (s := d.get("endpoint")) and self.one(s):
            out["endpoint"] = self.string(s.args[0], "endpoint")
        self.model.probes[d.name] = out

    def decl_machine(self, d: A.Decl) -> None:
        tool_keys = {"blade": "length", "bucket": "volume", "drum": "length", "capacity": "volume",
                     "width": "length"}
        self.unknown_keys(d, {"class", "caps", "max_speed", "endpoint", *tool_keys})
        self.required(d, ["class"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("class")) and self.one(s):
            out["class"] = self.enum(s.args[0], "machine.class", "machine class")
        if (s := d.get("caps")):
            if self.one(s) and isinstance(s.args[0], A.ListLit):
                out["caps"] = [self.enum(c, "caps", "capability") for c in s.args[0].items]
            else:
                self.error("caps needs a list, e.g. caps [cut, push]", s.pos)
        elif out.get("class"):
            out["caps"] = list(DEFAULT_CAPS[out["class"]])
        for k, dim in tool_keys.items():
            if (s := d.get(k)) and self.one(s):
                out[k] = self.qty(s.args[0], dim, k)
        if (s := d.get("max_speed")) and self.one(s):
            out["max_speed"] = self.qty(s.args[0], "speed", "max_speed")
        if (s := d.get("endpoint")) and self.one(s):
            out["endpoint"] = self.string(s.args[0], "endpoint")
        self.model.machines[d.name] = out

    def decl_zone(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"role", "area", "tolerance", "lift", "compaction", "buffer"})
        self.required(d, ["role", "area"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("role")) and self.one(s):
            out["role"] = self.enum(s.args[0], "zone.role", "zone role")
        if (s := d.get("area")):
            out["area"] = self.area(s)
        if (s := d.get("tolerance")) and self.one(s):
            q = self.qty(s.args[0], "length", "tolerance", allow_tol=True)
            if q and not q.tol:
                self.warn("write tolerances with ±, e.g. ±0.1 ft", s.pos)
            out["tolerance"] = q
        else:
            out["tolerance"] = Quantity(0.1, "ft", tol=True)
        if (s := d.get("lift")) and self.one(s):
            out["lift"] = self.qty(s.args[0], "length", "lift")
        if (s := d.get("compaction")) and s.args:
            out["compaction"] = self.qty(s.args[0], "ratio", "compaction")
            if len(s.args) == 2 and isinstance(s.args[1], A.Ident):
                out["compaction_method"] = s.args[1].name
            elif len(s.args) > 2:
                self.error("compaction takes a percent and an optional method, e.g. 95 % proctor", s.pos)
        if (s := d.get("buffer")) and self.one(s):
            out["buffer"] = self.qty(s.args[0], "length", "buffer")
            if out.get("role") != "exclusion":
                self.warn("buffer only affects exclusion zones", s.pos)
        if out.get("role") == "fill" and "lift" not in out:
            self.warn(f"fill zone '{d.name}' has no lift thickness; planners will use 8 in", d.pos)
        self.model.zones[d.name] = out

    def decl_stockpile(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"area", "capacity", "material", "available"})
        self.required(d, ["area"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("area")):
            out["area"] = self.area(s)
        if (s := d.get("capacity")) and self.one(s):
            out["capacity"] = self.qty(s.args[0], "volume", "capacity")
        if (s := d.get("available")) and self.one(s):
            out["available"] = self.qty(s.args[0], "volume", "available")
        if (s := d.get("material")) and self.one(s):
            out["material"] = self.string(s.args[0], "material")
        self.model.stockpiles[d.name] = out

    def decl_task(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"op", "zone", "by", "max_cut", "lift", "passes", "spoil", "source",
                              "after", "until", "priority", "depth", "along", "speed"})
        self.required(d, ["op", "by", "until"])
        out: dict[str, Any] = {"name": d.name, "after": []}
        if (s := d.get("op")) and self.one(s):
            out["op"] = self.enum(s.args[0], "task.op", "task op")
        if (s := d.get("zone")) and self.one(s):
            out["zone"] = self.ref(s.args[0], ("zone",), "zone")
        elif out.get("op") not in ("haul", "trench"):
            self.error(f"task '{d.name}' needs a zone", d.pos)
        if (s := d.get("by")):
            out["by"] = self.by(s, out.get("op"))
        if (s := d.get("max_cut")) and s.args:
            out["max_cut"] = self.qty(s.args[0], "length", "max_cut")
            tail = [a.name if isinstance(a, A.Ident) else "?" for a in s.args[1:]]
            if tail not in ([], ["per", "pass"]):
                self.error("max_cut reads '<depth> per pass', e.g. max_cut 0.5 ft per pass", s.pos)
        if (s := d.get("lift")) and self.one(s):
            out["lift"] = self.qty(s.args[0], "length", "lift")
        if (s := d.get("depth")) and self.one(s):
            out["depth"] = self.qty(s.args[0], "length", "depth")
        if (s := d.get("speed")) and self.one(s):
            out["speed"] = self.qty(s.args[0], "speed", "speed")
        if (s := d.get("passes")) and self.one(s):
            if isinstance(s.args[0], A.Num) and s.args[0].value >= 1 and s.args[0].value == int(s.args[0].value):
                out["passes"] = int(s.args[0].value)
            else:
                self.error("passes must be a whole number of passes", s.pos)
        if (s := d.get("priority")) and self.one(s):
            if isinstance(s.args[0], A.Num):
                out["priority"] = s.args[0].value
            else:
                self.error("priority must be a number", s.pos)
        if (s := d.get("spoil")):
            if self.one(s) and isinstance(s.args[0], A.Arrow):
                tg = s.args[0].target
                items = tg.items if isinstance(tg, A.ListLit) else [tg]
                out["spoil"] = [self.ref(i, ("zone", "stockpile"), "spoil") for i in items]
            else:
                self.error("spoil reads '-> <zone or stockpile>' or '-> [first, overflow, ...]'", s.pos)
        if (s := d.get("source")):
            items = s.args[0].items if (len(s.args) == 1 and isinstance(s.args[0], A.ListLit)) else s.args
            out["source"] = [self.ref(i, ("zone", "stockpile"), "source") for i in items]
        if (s := d.get("along")):
            pts = s.args[0].items if self.one(s) and isinstance(s.args[0], A.ListLit) else []
            if len(pts) >= 2 and all(isinstance(p, A.Point) for p in pts):
                out["along"] = [(p.x, p.y) for p in pts]
            else:
                self.error("along needs a list of at least two points", s.pos)
        for s in d.all("after"):
            if not s.args:
                self.error("after needs a task name", s.pos)
                continue
            t = self.ref(s.args[0], ("task",), "after")
            mode = "done"
            if len(s.args) == 2:
                if isinstance(s.args[1], A.Ident) and s.args[1].name in ("start", "done"):
                    mode = s.args[1].name
                else:
                    self.error("after reads 'after <task> [start | done]'", s.pos)
            elif len(s.args) > 2:
                self.error("after reads 'after <task> [start | done]'", s.pos)
            if t == d.name:
                self.error("a task cannot run after itself", s.pos)
            out["after"].append({"task": t, "when": mode})
        if (s := d.get("until")):
            out["until"] = self.condition(s)
        if out.get("op") == "trench" and "along" not in out:
            self.error("a trench task needs 'along [(x, y), ...]'", d.pos)
        if out.get("op") == "cut" and "spoil" not in out:
            self.warn(f"cut task '{d.name}' has no spoil destination; the planner will choose one", d.pos)
        self.model.tasks[d.name] = out

    def by(self, s: A.Stmt, op: str | None):
        a = s.args
        if len(a) == 1 and isinstance(a[0], A.Ident):
            name = self.ref(a[0], ("machine",), "by")
            if name and op and (m := self.model.machines.get(name)) and OP_CAP.get(op) not in (m.get("caps") or []):
                self.warn(f"machine '{name}' has no '{OP_CAP[op]}' capability for a {op} task", s.pos)
            return {"machine": name}
        if len(a) == 2 and isinstance(a[0], A.Ident) and a[0].name in ("class", "cap"):
            if a[0].name == "class":
                return {"class": self.enum(a[1], "machine.class", "machine class")}
            return {"cap": self.enum(a[1], "caps", "capability")}
        self.error("by reads 'by <machine>', 'by class <class>' or 'by cap <capability>'", s.pos)
        return None

    def decl_rule(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"when", "do", "priority"})
        self.required(d, ["when", "do"])
        out: dict[str, Any] = {"name": d.name}
        if (s := d.get("when")):
            out["when"] = self.condition(s)
        if (s := d.get("do")):
            out["do"] = self.action(s)
        self.model.rules[d.name] = out

    def action(self, s: A.Stmt):
        a = s.args
        if not a or not isinstance(a[0], A.Ident):
            self.error("do needs an action: stop, hold, pause, resume, slow or alert", s.pos)
            return None
        verb = self.enum(a[0], "action", "action")
        rest = a[1:]
        out: dict[str, Any] = {"verb": verb}
        if verb == "alert":
            if len(rest) != 1 or not isinstance(rest[0], A.Str):
                self.error('alert needs a message: do alert "probe offline"', s.pos)
            else:
                out["message"] = rest[0].value
            return out
        if verb == "slow":
            if rest and isinstance(rest[-1], (A.Qty, A.Num)):
                out["speed"] = self.qty(rest[-1], "speed", "slow speed")
                rest = rest[:-1]
            else:
                self.error("slow needs a speed limit, e.g. do slow all 3 mph", s.pos)
        out["target"] = self.target(rest, s)
        return out

    def target(self, rest: list, s: A.Stmt):
        if not rest:
            self.error("action needs a target: all, a machine, 'task <name>', 'class <class>' or a list", s.pos)
            return None
        if len(rest) == 1 and isinstance(rest[0], A.Ident) and rest[0].name == "all":
            return {"all": True}
        if len(rest) == 2 and isinstance(rest[0], A.Ident) and rest[0].name in ("task", "class"):
            if rest[0].name == "task":
                return {"task": self.ref(rest[1], ("task",), "target")}
            return {"class": self.enum(rest[1], "machine.class", "machine class")}
        if len(rest) == 1 and isinstance(rest[0], A.ListLit):
            return {"names": [self.ref(i, ("task", "machine"), "target") for i in rest[0].items]}
        if len(rest) == 1 and isinstance(rest[0], A.Ident):
            return {"names": [self.ref(rest[0], ("task", "machine"), "target")]}
        self.error("could not read the action target", s.pos)
        return None

    def decl_replan(self, d: A.Decl) -> None:
        self.unknown_keys(d, {"when"})
        self.required(d, ["when"])
        if (s := d.get("when")):
            self.model.replans[d.name] = {"name": d.name, "when": self.condition(s)}

    # -- task graph ------------------------------------------------------
    def check_tasks_graph(self) -> None:
        tasks = self.model.tasks
        state: dict[str, int] = {}
        order: list[str] = []

        def visit(n: str, path: list[str]) -> None:
            if state.get(n) == 2:
                return
            if state.get(n) == 1:
                cyc = [*path[path.index(n):], n]
                d = self.names[n]
                self.error(f"task dependency cycle: {' -> '.join(cyc)}", d.pos)
                return
            state[n] = 1
            for dep in tasks[n].get("after", []):
                if dep["task"] in tasks and dep["when"] in ("done",):
                    visit(dep["task"], [*path, n])
            state[n] = 2
            order.append(n)

        for n in tasks:
            visit(n, [])
        self.model.order = order
        if not tasks:
            self.warn("program has no tasks", A.Pos(1, 1))
        if not self.model.probes:
            self.warn("program has no probe: tasks cannot be checked against measured ground", A.Pos(1, 1))
        # every task needs at least one machine able to do it
        for name, t in tasks.items():
            by = t.get("by") or {}
            if by.get("class") and not any(m.get("class") == by["class"] for m in self.model.machines.values()):
                self.error(f"task '{name}' needs a {by['class']} but none is declared", self.names[name].pos)
            if by.get("cap") and not any(by["cap"] in (m.get("caps") or []) for m in self.model.machines.values()):
                self.error(f"task '{name}' needs capability '{by['cap']}' but no machine has it",
                           self.names[name].pos)


def check(prog: A.Program) -> tuple[Model, list[Diagnostic]]:
    return Checker(prog).run()


def has_errors(diags: list[Diagnostic]) -> bool:
    return any(d.level == "error" for d in diags)
