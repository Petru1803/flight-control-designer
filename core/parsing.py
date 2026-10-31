"""Parse MATLAB-style matrix text, e.g. "[-0.02 0.12 0 -9.81; -0.2 -0.5 67.9 0]".

Supports: brackets optional, rows separated by ';' or newlines, elements by
spaces or commas, scientific notation (3.88e-6, 3.88*10^-6), simple
expressions (-g*cos(0.08), pi/180, 9.81*sin(0)) and named constants supplied
by the caller (g, TAS, V, ...). Comments starting with % or # are ignored.
"""
import math
import re

import numpy as np


class MatrixParseError(ValueError):
    pass


_ALLOWED_FUNCS = {
    "sin": math.sin, "cos": math.cos, "tan": math.tan, "sqrt": math.sqrt,
    "exp": math.exp, "log": math.log, "abs": abs, "deg2rad": math.radians,
    "rad2deg": math.degrees, "asin": math.asin, "acos": math.acos,
    "atan": math.atan, "pi": math.pi,
}

_TOKEN_SPLIT = re.compile(r"[,\s]+")


def _eval_element(tok, consts):
    expr = tok.replace("^", "**")
    if not re.fullmatch(r"[0-9A-Za-z_\.\+\-\*/\(\)\s]*", expr):
        raise MatrixParseError(f"Invalid characters in element '{tok}'")
    names = dict(_ALLOWED_FUNCS)
    names.update({k: float(v) for k, v in (consts or {}).items()})
    names["i"] = names["j"] = 1j
    try:
        val = eval(expr, {"__builtins__": {}}, names)  # restricted namespace
    except Exception as e:  # noqa: BLE001
        raise MatrixParseError(f"Cannot evaluate element '{tok}': {e}") from None
    if isinstance(val, complex):
        if abs(val.imag) > 1e-14:
            raise MatrixParseError(f"Complex value not allowed: '{tok}'")
        val = val.real
    return float(val)


def _split_elements(row):
    """Split a row into elements. A sign preceded by whitespace starts a new
    element (MATLAB rule: '[1 -2]' is two elements, '[1 - 2]' is one)."""
    row = row.strip()
    if not row:
        return []
    # collapse 'a - b' (spaces both sides) into 'a-b' so it stays one element
    row = re.sub(r"(?<=[0-9A-Za-z_\)\.])\s+([\+\-\*/\^])\s+", r"\1", row)
    row = re.sub(r"\s*([\*/\^])\s*", r"\1", row)
    row = re.sub(r"\(\s*", "(", row)
    row = re.sub(r"\s*\)", ")", row)
    return [t for t in _TOKEN_SPLIT.split(row) if t]


def parse_matrix(text, consts=None, name="matrix"):
    if text is None:
        raise MatrixParseError(f"{name}: empty")
    lines = []
    for line in str(text).splitlines():
        line = re.split(r"[%#]", line, maxsplit=1)[0]
        line = line.replace("...", " ")
        lines.append(line)
    body = " ; ".join(lines)
    # strip a leading 'A = ' assignment and brackets
    body = re.sub(r"^\s*[A-Za-z_]\w*\s*=", "", body)
    body = body.replace("[", " ").replace("]", " ")
    rows = [r for r in body.split(";") if r.strip()]
    if not rows:
        raise MatrixParseError(f"{name}: empty")
    data = []
    for r in rows:
        toks = _split_elements(r)
        if toks:
            data.append([_eval_element(t, consts) for t in toks])
    ncols = {len(r) for r in data}
    if len(ncols) != 1:
        raise MatrixParseError(
            f"{name}: rows have different lengths {sorted(ncols)}")
    return np.array(data, dtype=float)


def format_matrix(M, prec=6):
    M = np.atleast_2d(np.asarray(M, dtype=float))
    rows = ["  ".join(f"{v: .{prec}g}" for v in r) for r in M]
    return "[" + ";\n ".join(rows) + "]"
