"""Tokenizer for Loam source text."""
from __future__ import annotations

from dataclasses import dataclass


class LoamSyntaxError(Exception):
    """Text that is not valid Loam, with the file, line and column where the problem is."""
    def __init__(self, msg: str, line: int, col: int, src: str = "<input>"):
        """`msg` is the bare explanation; the exception text adds `file:line:col:` in front, the form editors and
        compilers use.
        """
        super().__init__(f"{src}:{line}:{col}: {msg}")
        self.msg, self.line, self.col, self.src = msg, line, col, src


@dataclass
class Token:
    """One word of a program: a number, a string, a name, a symbol, a line end or the end of the file.

    `line` and `col` (both counted from 1) say where it started, so every later error can point at the source.
    """
    kind: str  # NUM STR IDENT SYM NL EOF
    text: str
    line: int
    col: int

    def __repr__(self) -> str:  # pragma: no cover
        """For debugging: kind, text and position."""
        return f"{self.kind}({self.text!r})@{self.line}:{self.col}"


# Two-character symbols come first, so `<=` is never read as `<` followed by `=`.
SYMBOLS = ["->", "<=", ">=", "==", "!=", "±", "{", "}", "[", "]", "(", ")",
           ",", ";", "<", ">", ".", "%", "+", "-", "*", "/"]


def tokenize(text: str, src: str = "<input>") -> list[Token]:
    """Split Loam source text into tokens.

    Comments run from `#` to the end of the line and are dropped. Line ends are kept as `NL` tokens because a newline
    can end a statement. `src` is the file name used in error messages. Raises LoamSyntaxError for an unterminated
    string or a character Loam does not use.
    """
    toks: list[Token] = []
    i, line, col = 0, 1, 1
    n = len(text)

    def add(kind: str, s: str, ln: int, c: int) -> None:
        """Append one token."""
        toks.append(Token(kind, s, ln, c))

    while i < n:
        ch = text[i]
        if ch == "#":
            while i < n and text[i] != "\n":
                i += 1
            continue
        if ch == "\n":
            add("NL", "\n", line, col)
            i += 1
            line += 1
            col = 1
            continue
        if ch in " \t\r":
            i += 1
            col += 1
            continue
        # A string: everything up to the closing quote; a backslash keeps the next character as it is (so \" is a
        # quote inside a string).
        if ch == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == "\n":
                    raise LoamSyntaxError("unterminated string", line, col, src)
                if text[j] == "\\" and j + 1 < n:
                    j += 1
                buf.append(text[j])
                j += 1
            if j >= n:
                raise LoamSyntaxError("unterminated string", line, col, src)
            add("STR", "".join(buf), line, col)
            col += j + 1 - i
            i = j + 1
            continue
        # A number: digits with at most one decimal point. A dot is part of the number only when a digit follows, so
        # the `.` in `residual(pad).high` stays a symbol.
        if ch.isdigit() or (ch == "." and i + 1 < n and text[i + 1].isdigit()):
            j = i
            seen_dot = False
            while j < n and (text[j].isdigit() or (text[j] == "." and not seen_dot
                                                     and j + 1 < n and text[j + 1].isdigit())):
                if text[j] == ".":
                    seen_dot = True
                j += 1
            # scientific notation
            exp_digit = j + 1 < n and (text[j + 1].isdigit()
                                       or (text[j + 1] in "+-" and j + 2 < n and text[j + 2].isdigit()))
            if j < n and text[j] in "eE" and exp_digit:
                j += 2
                while j < n and text[j].isdigit():
                    j += 1
            add("NUM", text[i:j], line, col)
            col += j - i
            i = j
            continue
        # A name or keyword. Keywords are not special here; the parser decides what a word means from where it stands.
        if ch.isalpha() or ch == "_":
            j = i
            while j < n and (text[j].isalnum() or text[j] == "_"):
                j += 1
            add("IDENT", text[i:j], line, col)
            col += j - i
            i = j
            continue
        for s in SYMBOLS:
            if text.startswith(s, i):
                add("SYM", s, line, col)
                i += len(s)
                col += len(s)
                break
        else:
            raise LoamSyntaxError(f"unexpected character {ch!r}", line, col, src)
    # Every program ends with a line end and an EOF token, so the parser never has to check for running off the end.
    add("NL", "\n", line, col)
    add("EOF", "", line, col)
    return toks
