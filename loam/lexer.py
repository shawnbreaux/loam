"""Tokenizer for Loam source text."""
from __future__ import annotations

from dataclasses import dataclass


class LoamSyntaxError(Exception):
    def __init__(self, msg: str, line: int, col: int, src: str = "<input>"):
        super().__init__(f"{src}:{line}:{col}: {msg}")
        self.msg, self.line, self.col, self.src = msg, line, col, src


@dataclass
class Token:
    kind: str  # NUM STR IDENT SYM NL EOF
    text: str
    line: int
    col: int

    def __repr__(self) -> str:  # pragma: no cover
        return f"{self.kind}({self.text!r})@{self.line}:{self.col}"


SYMBOLS = ["->", "<=", ">=", "==", "!=", "±", "{", "}", "[", "]", "(", ")",
           ",", ";", "<", ">", ".", "%", "+", "-", "*", "/"]


def tokenize(text: str, src: str = "<input>") -> list[Token]:
    toks: list[Token] = []
    i, line, col = 0, 1, 1
    n = len(text)

    def add(kind: str, s: str, ln: int, c: int) -> None:
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
    add("NL", "\n", line, col)
    add("EOF", "", line, col)
    return toks
