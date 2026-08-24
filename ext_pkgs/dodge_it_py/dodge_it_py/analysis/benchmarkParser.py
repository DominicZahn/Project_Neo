import re
import math
from dataclasses import dataclass

_ZERO_SHORTHAND = r"0"
_NUM2 = r"-?\d{2}"
_STANDALONE_NUM = rf"(?:{_ZERO_SHORTHAND}|{_NUM2})"
 
_PI_OR_NUM = rf"(?:-?pi\d*|{_STANDALONE_NUM})"
 
CODE_RE = re.compile(
    rf"^C(?P<center>(?:{_NUM2}){{3}})"
    rf"R(?P<radius>{_STANDALONE_NUM})"
    rf"L(?P<lower>{_PI_OR_NUM})"
    rf"U(?P<upper>{_PI_OR_NUM})"
    rf"V(?P<velocity>{_STANDALONE_NUM})$"
)
 
def _parsePlainNumber(token: str) -> float:
    if token == "0":
        return 0.0
 
    sign = -1.0 if token.startswith("-") else 1.0
    digits = token.lstrip("-")
 
    if len(digits) != 2:
        raise ValueError(f"Expected exactly 2 digits (or bare '0'), got {token!r}")
 
    return sign * float(f"{digits[0]}.{digits[1]}")
 
 
def _parsePiOrNumber(token: str) -> float:
    sign = -1.0 if token.startswith("-") else 1.0
    body = token.lstrip("-")
 
    if body.startswith("pi"):
        rest = body[2:]
        if rest == "":
            return sign * math.pi
        return sign * math.pi / int(rest)
 
    return _parsePlainNumber(token)
 
 
_CENTER_COMPONENT_RE = re.compile(r"-?\d{2}")
 
 
def _parseCenter(token: str) -> list[float]:
    parts = _CENTER_COMPONENT_RE.findall(token)
 
    if len(parts) != 3 or "".join(parts) != token:
        raise ValueError(f"Could not cleanly split center token into 3 components: {token!r}")
 
    return [_parsePlainNumber(p) for p in parts]
 

@dataclass
class BenchmarkData:
    center: list
    radius: float
    lower: float
    upper: float
    velocity: float
    raw: str
 
 
def parseBenchmarkName(name: str) -> BenchmarkData:
    m = CODE_RE.match(name)
    if not m:
        raise ValueError(f"Name does not match expected pattern: {name!r}")
 
    return BenchmarkData(
        center=_parseCenter(m.group("center")),
        radius=_parsePlainNumber(m.group("radius")),
        lower=_parsePiOrNumber(m.group("lower")),
        upper=_parsePiOrNumber(m.group("upper")),
        velocity=_parsePlainNumber(m.group("velocity")),
        raw=name,
    )