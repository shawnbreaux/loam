"""AST node types for Loam."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union

from .units import Quantity


@dataclass
class Pos:
    line: int
    col: int


@dataclass
class Num:
    value: float
    pos: Pos


@dataclass
class Qty:
    q: Quantity
    pos: Pos


@dataclass
class Str:
    value: str
    pos: Pos


@dataclass
class Ident:
    name: str
    pos: Pos


@dataclass
class ListLit:
    items: list
    pos: Pos


@dataclass
class Point:
    x: float
    y: float
    pos: Pos


@dataclass
class Arrow:
    target: Node
    pos: Pos


@dataclass
class Call:
    fn: str
    args: list
    pos: Pos


@dataclass
class Attr:
    obj: Node
    name: str
    pos: Pos


@dataclass
class BinOp:
    op: str
    left: Node
    right: Node
    pos: Pos


@dataclass
class Not:
    expr: Node
    pos: Pos


Node = Union[Num, Qty, Str, Ident, ListLit, Point, Arrow, Call, Attr, BinOp, Not]  # noqa: UP007 (runtime alias, py3.10)


@dataclass
class Stmt:
    key: str
    args: list
    pos: Pos


@dataclass
class Decl:
    kind: str
    name: str
    stmts: list[Stmt]
    pos: Pos

    def get(self, key: str) -> Stmt | None:
        for s in self.stmts:
            if s.key == key:
                return s
        return None

    def all(self, key: str) -> list[Stmt]:
        return [s for s in self.stmts if s.key == key]


@dataclass
class Program:
    version: str
    decls: list[Decl] = field(default_factory=list)
    src: str = "<input>"

    def of(self, kind: str) -> list[Decl]:
        return [d for d in self.decls if d.kind == kind]

    def find(self, name: str) -> Decl | None:
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
