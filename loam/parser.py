"""Recursive-descent parser for Loam.

Grammar (EBNF; see SPEC.md for the full reference):

    program    = { NL } header { NL } { decl { NL } } EOF ;
    header     = "loam" VERSION ;          (* e.g. loam 0.0.1 *)
    decl       = KIND NAME "{" { NL | ";" } { stmt ( NL | ";" ) { NL | ";" } } "}" ;
    KIND       = "site" | "probe" | "machine" | "zone" | "stockpile"
               | "task" | "rule" | "replan" ;
    stmt       = KEY ( expr                     (* for KEY in when, until *)
                     | { atom } ) ;
    atom       = quantity | NUMBER | STRING | IDENT | list | point | "->" atom ;
    quantity   = [ "±" ] NUMBER UNIT ;
    UNIT       = IDENT | "%" | IDENT "/" IDENT ;
    list       = "[" [ atom { "," atom } ] "]" ;
    point      = "(" NUMBER "," NUMBER ")" ;
    expr       = or ;   or = and { "or" and } ;   and = not { "and" not } ;
    not        = "not" not | cmp ;
    cmp        = sum [ ( "<" | "<=" | ">" | ">=" | "==" | "!=" ) sum ] ;
    sum        = prod { ( "+" | "-" ) prod } ;
    prod       = unary { ( "*" | "/" ) unary } ;
    unary      = "-" unary | postfix ;
    postfix    = primary { "(" [ expr { "," expr } ] ")" | "." IDENT } ;
    primary    = quantity | NUMBER | STRING | IDENT | list | "(" expr ")" ;
"""
from __future__ import annotations

from . import ast as A
from .lexer import LoamSyntaxError, Token, tokenize
from .units import Quantity, UnitError, lookup

# The eight kinds of declaration, the statement keywords whose argument is a condition, and the comparison operators.
KINDS = {"site", "probe", "machine", "zone", "stockpile", "task", "rule", "replan"}
EXPR_KEYS = {"when", "until"}
CMP_OPS = {"<", "<=", ">", ">=", "==", "!="}


class Parser:
    """Turns the token list into a Program, one method per grammar rule above.

    Each method consumes the tokens of its rule and returns the node it built; `self.i` is the next token to read.
    Nothing is looked up or type-checked here: a name is just a name until the checker runs.
    """
    def __init__(self, text: str, src: str = "<input>"):
        """Tokenize `text`; `src` is the file name for error messages."""
        self.src = src
        self.toks = tokenize(text, src)
        self.i = 0
        self.depth = 0  # bracket depth: newlines are ignored inside brackets

    # -- token helpers ---------------------------------------------------
    @property
    def tok(self) -> Token:
        """The token being looked at."""
        return self.toks[self.i]

    def peek(self, k: int = 1) -> Token:
        """The token `k` places ahead, without consuming anything (the last token, EOF, if that is past the end)."""
        return self.toks[min(self.i + k, len(self.toks) - 1)]

    def err(self, msg: str, tok: Token | None = None) -> LoamSyntaxError:
        """A syntax error at the current token, or at `tok`."""
        t = tok or self.tok
        return LoamSyntaxError(msg, t.line, t.col, self.src)

    def advance(self) -> Token:
        """Consume the current token and return it. Inside brackets, line ends that follow are skipped, so lists and
        conditions can span lines.
        """
        t = self.tok
        self.i += 1
        if self.depth > 0:
            self.skip_nl()
        return t

    def skip_nl(self) -> None:
        """Skip blank lines."""
        while self.tok.kind == "NL":
            self.i += 1

    def is_sym(self, s: str) -> bool:
        """Whether the current token is the symbol `s`."""
        return self.tok.kind == "SYM" and self.tok.text == s

    def is_kw(self, s: str) -> bool:
        """Whether the current token is the word `s`."""
        return self.tok.kind == "IDENT" and self.tok.text == s

    def expect_sym(self, s: str) -> Token:
        """Consume the symbol `s`, or fail saying what was found instead."""
        if not self.is_sym(s):
            raise self.err(f"expected '{s}', found {self.describe(self.tok)}")
        return self.advance()

    def expect_ident(self, what: str = "name") -> Token:
        """Consume a name, or fail; `what` describes what was wanted, for the message."""
        if self.tok.kind != "IDENT":
            raise self.err(f"expected {what}, found {self.describe(self.tok)}")
        return self.advance()

    @staticmethod
    def describe(t: Token) -> str:
        """A token as it should appear in an error message."""
        if t.kind == "NL":
            return "end of line"
        if t.kind == "EOF":
            return "end of file"
        return f"'{t.text}'"

    def pos(self, t: Token | None = None) -> A.Pos:
        """The source position of a token (the current one by default)."""
        t = t or self.tok
        return A.Pos(t.line, t.col)

    def at_stmt_end(self) -> bool:
        """Whether the statement is over: a line end, `;`, the closing `}` or the end of the file."""
        return self.tok.kind in ("NL", "EOF") or self.is_sym(";") or self.is_sym("}")

    # -- program ---------------------------------------------------------
    def parse(self) -> A.Program:
        """The whole program: the `loam <version>` header, then declarations until the end of the file."""
        self.skip_nl()
        if not self.is_kw("loam"):
            raise self.err("a Loam program must start with 'loam <version>', e.g. 'loam 0.0.1'")
        self.advance()
        if self.tok.kind != "NUM":
            raise self.err("expected a version number after 'loam', e.g. 'loam 0.0.1'")
        # "0.0.1" lexes as NUM "0.0" + NUM ".1": join the pieces on this line
        parts = [self.advance().text]
        while self.tok.kind == "NUM" and self.tok.text.startswith("."):
            parts.append(self.advance().text)
        version = "".join(parts)
        prog = A.Program(version=version, src=self.src)
        self.skip_nl()
        while self.tok.kind != "EOF":
            prog.decls.append(self.decl())
            self.skip_nl()
        return prog

    def decl(self) -> A.Decl:
        """One `kind name { statements }` block."""
        t = self.tok
        if t.kind != "IDENT" or t.text not in KINDS:
            raise self.err(f"expected a declaration ({', '.join(sorted(KINDS))}), found {self.describe(t)}")
        self.advance()
        name = self.expect_ident(f"a name for this {t.text}").text
        self.expect_sym("{")
        stmts: list[A.Stmt] = []
        while True:
            while self.tok.kind == "NL" or self.is_sym(";"):
                self.i += 1
            if self.is_sym("}"):
                self.i += 1
                break
            if self.tok.kind == "EOF":
                raise self.err(f"missing '}}' to close {t.text} '{name}' opened at line {t.line}")
            stmts.append(self.stmt())
            if not (self.tok.kind == "NL" or self.is_sym(";") or self.is_sym("}")):
                raise self.err(f"unexpected {self.describe(self.tok)} at end of statement")
        return A.Decl(t.text, name, stmts, self.pos(t))

    def stmt(self) -> A.Stmt:
        """One statement: a keyword and its arguments.

        `when` and `until` take a condition (an expression). `area where <condition>` defines a zone by a condition on
        the ground. Everything else takes a run of simple values up to the end of the line.
        """
        kt = self.expect_ident("a field name")
        key = kt.text
        args: list = []
        if key in EXPR_KEYS:
            if self.at_stmt_end():
                raise self.err(f"'{key}' needs a condition")
            args.append(self.expr())
        elif key == "area" and self.is_kw("where"):
            wt = self.advance()
            if self.at_stmt_end():
                raise self.err("'area where' needs a condition")
            args.append(A.Ident("where", self.pos(wt)))
            args.append(self.expr())
        else:
            while not self.at_stmt_end():
                args.append(self.atom())
        return A.Stmt(key, args, self.pos(kt))

    # -- atoms -----------------------------------------------------------
    def unit_here(self) -> str | None:
        """If the current tokens spell a unit, consume and return it."""
        t = self.tok
        if t.kind == "SYM" and t.text == "%":
            self.advance()
            return "%"
        if t.kind != "IDENT":
            return None
        # Try the longer unit first, so `in/h` is one unit and not `in` followed by a division.
        # compound a/b
        if self.peek().kind == "SYM" and self.peek().text == "/" and self.peek(2).kind == "IDENT":
            cand = f"{t.text}/{self.peek(2).text}"
            try:
                lookup(cand)
                for _ in range(3):
                    self.advance()
                return cand
            except UnitError:
                pass
        try:
            lookup(t.text)
        except UnitError:
            return None
        self.advance()
        return t.text

    def number(self, neg: bool = False) -> A.Num | A.Qty:
        """A number, with its unit if one follows (`0.3 ft` is one quantity; `4` alone is a plain number)."""
        t = self.advance()
        v = float(t.text)
        if neg:
            v = -v
        unit = self.unit_here()
        if unit:
            return A.Qty(Quantity(v, unit), self.pos(t))
        return A.Num(v, self.pos(t))

    def atom(self):
        """One simple value: a quantity, number, string, name, list, point or `-> target`."""
        t = self.tok
        if t.kind == "NUM":
            return self.number()
        if self.is_sym("-") and self.peek().kind == "NUM":
            self.advance()
            return self.number(neg=True)
        if self.is_sym("±"):
            self.advance()
            if self.tok.kind != "NUM":
                raise self.err("expected a number after '±'")
            n = self.number()
            if not isinstance(n, A.Qty):
                raise self.err("a ± tolerance needs a unit, e.g. ±0.1 ft", t)
            return A.Qty(Quantity(n.q.value, n.q.unit, tol=True), self.pos(t))
        if t.kind == "STR":
            self.advance()
            return A.Str(t.text, self.pos(t))
        if t.kind == "IDENT":
            self.advance()
            return A.Ident(t.text, self.pos(t))
        if self.is_sym("["):
            return self.list_lit(self.atom)
        if self.is_sym("("):
            return self.point()
        if self.is_sym("->"):
            self.advance()
            return A.Arrow(self.atom(), self.pos(t))
        raise self.err(f"unexpected {self.describe(t)}")

    def list_lit(self, item):
        """A `[a, b, c]` list. `item` parses one element, so lists work both in statements and in conditions. A trailing
        comma is allowed.
        """
        t = self.tok
        self.depth += 1
        self.advance()
        items = []
        if not self.is_sym("]"):
            items.append(item())
            while self.is_sym(","):
                self.advance()
                if self.is_sym("]"):
                    break  # trailing comma
                items.append(item())
        self.depth -= 1
        if not self.is_sym("]"):
            raise self.err(f"expected ',' or ']', found {self.describe(self.tok)}")
        self.advance()
        return A.ListLit(items, self.pos(t))

    def signed_num(self) -> float:
        """A coordinate: a number, possibly negative."""
        neg = False
        if self.is_sym("-"):
            neg = True
            self.advance()
        if self.tok.kind != "NUM":
            raise self.err("expected a coordinate number")
        v = float(self.advance().text)
        return -v if neg else v

    def point(self) -> A.Point:
        """An `(x, y)` coordinate pair."""
        t = self.tok
        self.depth += 1
        self.advance()
        x = self.signed_num()
        self.expect_sym(",")
        y = self.signed_num()
        self.depth -= 1
        self.expect_sym(")")
        return A.Point(x, y, self.pos(t))

    # -- expressions -----------------------------------------------------
    def expr(self):
        """A condition or arithmetic expression.

        The methods below are layered from loosest to tightest binding: or, and, not, comparison, + -, * /,
        unary minus, calls and attributes.
        """
        return self.or_()

    def or_(self):
        """`a or b or ...`"""
        left = self.and_()
        while self.is_kw("or"):
            t = self.advance()
            left = A.BinOp("or", left, self.and_(), self.pos(t))
        return left

    def and_(self):
        """`a and b and ...`"""
        left = self.not_()
        while self.is_kw("and"):
            t = self.advance()
            left = A.BinOp("and", left, self.not_(), self.pos(t))
        return left

    def not_(self):
        """`not a`"""
        if self.is_kw("not"):
            t = self.advance()
            return A.Not(self.not_(), self.pos(t))
        return self.cmp()

    def cmp(self):
        """One comparison. `a < b < c` is refused rather than guessed at: write `a < b and b < c`."""
        left = self.sum()
        if self.tok.kind == "SYM" and self.tok.text in CMP_OPS:
            t = self.advance()
            left = A.BinOp(t.text, left, self.sum(), self.pos(t))
            if self.tok.kind == "SYM" and self.tok.text in CMP_OPS:
                raise self.err("chained comparisons are not allowed; use 'and'")
        return left

    def sum(self):
        """Addition and subtraction, left to right."""
        left = self.prod()
        while self.is_sym("+") or self.is_sym("-"):
            t = self.advance()
            left = A.BinOp(t.text, left, self.prod(), self.pos(t))
        return left

    def prod(self):
        """Multiplication and division, left to right."""
        left = self.unary()
        while self.is_sym("*") or self.is_sym("/"):
            t = self.advance()
            left = A.BinOp(t.text, left, self.unary(), self.pos(t))
        return left

    def unary(self):
        """A leading minus: a negative number, or `0 - x` for anything else."""
        if self.is_sym("-"):
            t = self.advance()
            if self.tok.kind == "NUM":
                return self.number(neg=True)
            return A.BinOp("-", A.Num(0, self.pos(t)), self.unary(), self.pos(t))
        return self.postfix()

    def postfix(self):
        """A value followed by any number of calls `f(...)` and attributes `.name`, as in `residual(pad).high`."""
        node = self.primary()
        while True:
            if self.is_sym("(") and isinstance(node, A.Ident):
                t = self.tok
                self.depth += 1
                self.advance()
                args = []
                if not self.is_sym(")"):
                    args.append(self.expr())
                    while self.is_sym(","):
                        self.advance()
                        args.append(self.expr())
                self.depth -= 1
                self.expect_sym(")")
                node = A.Call(node.name, args, node.pos)
            elif self.is_sym("."):
                t = self.advance()
                name = self.expect_ident("an attribute name").text
                node = A.Attr(node, name, self.pos(t))
            else:
                return node

    def primary(self):
        """The innermost value: a quantity, number, string, name, list or a parenthesised expression."""
        t = self.tok
        if t.kind == "NUM":
            return self.number()
        if t.kind == "STR":
            self.advance()
            return A.Str(t.text, self.pos(t))
        if t.kind == "IDENT":
            if t.text in ("and", "or", "not"):
                raise self.err(f"'{t.text}' needs something before it")
            self.advance()
            return A.Ident(t.text, self.pos(t))
        if self.is_sym("["):
            return self.list_lit(self.expr)
        if self.is_sym("("):
            self.depth += 1
            self.advance()
            e = self.expr()
            self.depth -= 1
            self.expect_sym(")")
            return e
        raise self.err(f"expected a value, found {self.describe(t)}")


def parse(text: str, src: str = "<input>") -> A.Program:
    """Parse Loam source text into a Program. Raises LoamSyntaxError."""
    return Parser(text, src).parse()


def parse_file(path: str) -> A.Program:
    """Parse a `.loam` file."""
    with open(path, encoding="utf-8") as f:
        return parse(f.read(), path)
