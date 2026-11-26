"""AST node types for Loam."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union

from .units import Quantity


@dataclass
class Pos:
    """Where something starts in the source text (line and column, counted from 1), for error messages."""
    line: int
    col: int


@dataclass
class Num:
    """A bare number, e.g. the `4` in `passes 4`."""
    value: float
    pos: Pos


@dataclass
class Qty:
    """A number with a unit, e.g. `0.3 ft` or `±0.1 ft` (see units.Quantity)."""
    q: Quantity
    pos: Pos


@dataclass
class Str:
    """A quoted string, e.g. a file name: `"pad_north_design.xml"`."""
    value: str
    pos: Pos


@dataclass
class Ident:
    """A name: of a zone, machine, task or probe the program declares, or a keyword such as `cut` or `dozer`."""
    name: str
    pos: Pos


@dataclass
class ListLit:
    """A bracketed list, e.g. `[spoil_north, spoil_south]`."""
    items: list
    pos: Pos


@dataclass
class Point:
    """A coordinate pair `(x, y)` in site units."""
    x: float
    y: float
    pos: Pos


@dataclass
class Arrow:
    """`-> target`: where material goes (`spoil -> stockpile`) or what a rule acts on."""
    target: Node
    pos: Pos


@dataclass
class Call:
    """A function in a condition, e.g. `residual(pad)` or `age(mast)`. The functions are listed in evaluate.py."""
    fn: str
    args: list
    pos: Pos


@dataclass
class Attr:
    """A statistic picked from a function's result, e.g. the `.high` in `residual(pad).high`."""
    obj: Node
    name: str
    pos: Pos


@dataclass
class BinOp:
    """Two expressions joined by an operator: comparison (`<`, `>=`, ...), arithmetic, or `and` / `or`."""
    op: str
    left: Node
    right: Node
    pos: Pos


@dataclass
class Not:
    """`not <condition>`."""
    expr: Node
    pos: Pos


# Anything that can appear as a value or inside a condition.
Node = Union[Num, Qty, Str, Ident, ListLit, Point, Arrow, Call, Attr, BinOp, Not]  # noqa: UP007 (runtime alias, py3.10)


@dataclass
class Stmt:
    """One line inside a declaration: a keyword and what follows it.

    `tolerance ±0.1 ft` is Stmt("tolerance", [Qty(...)]). The parser does not decide what the arguments mean; the
    checker does.
    """
    key: str
    args: list
    pos: Pos


@dataclass
class Decl:
    """One top-level block: `zone pad { ... }`, `machine dz1 { ... }`, `task cut_pad { ... }` and so on.

    `kind` is the leading keyword, `name` the name the author gave, `stmts` the lines between the braces.
    """
    kind: str
    name: str
    stmts: list[Stmt]
    pos: Pos

    def get(self, key: str) -> Stmt | None:
        """The first statement with this keyword, or None."""
        for s in self.stmts:
            if s.key == key:
                return s
        return None

    def all(self, key: str) -> list[Stmt]:
        """Every statement with this keyword (some, like `after`, may repeat)."""
        return [s for s in self.stmts if s.key == key]


@dataclass
class Program:
    """A parsed file: the language version it asks for (`loam 0.0.2`) and its declarations in source order."""
    version: str
    decls: list[Decl] = field(default_factory=list)
    src: str = "<input>"

    def of(self, kind: str) -> list[Decl]:
        """All declarations of one kind, e.g. every `zone`."""
        return [d for d in self.decls if d.kind == kind]

    def find(self, name: str) -> Decl | None:
        """The declaration with this name, of any kind, or None."""
        for d in self.decls:
            if d.name == name:
                return d
        return None


def render(node) -> str:
    """Turn an expression node back into Loam source text."""
    if isinstance(node, Num):
        return f"{node.value:g}"
    if isinstance(node, Qty):
        return str(node.q)
    if isinstance(node, Str):
        return f'"{node.value}"'
    if isinstance(node, Ident):
        return node.name
    if isinstance(node, ListLit):
        return "[" + ", ".join(render(i) for i in node.items) + "]"
    if isinstance(node, Point):
        return f"({node.x:g}, {node.y:g})"
    if isinstance(node, Arrow):
        return "-> " + render(node.target)
    if isinstance(node, Call):
        return f"{node.fn}(" + ", ".join(render(a) for a in node.args) + ")"
    if isinstance(node, Attr):
        return f"{render(node.obj)}.{node.name}"
    if isinstance(node, BinOp):
        return f"{render(node.left)} {node.op} {render(node.right)}"
    if isinstance(node, Not):
        return f"not {render(node.expr)}"
    raise TypeError(node)
