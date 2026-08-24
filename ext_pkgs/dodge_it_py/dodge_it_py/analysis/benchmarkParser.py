import re
import math
from dataclasses import dataclass
from pathlib import Path
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import numpy.typing as npt

_ZERO_SHORTHAND = r"0"
_NUM = r"-?\d+"
_STANDALONE_NUM = rf"(?:{_ZERO_SHORTHAND}|{_NUM})"
_PI_OR_NUM = rf"(?:-?pi\d*|{_STANDALONE_NUM})"
_CENTER_COMPONENT_RE = re.compile(r"-?\d{2}")
 
CODE_RE = re.compile(
    rf"^C(?P<center>(?:{_NUM}){{3}})"
    rf"R(?P<radius>{_STANDALONE_NUM})"
    rf"L(?P<lower>{_PI_OR_NUM})"
    rf"U(?P<upper>{_PI_OR_NUM})"
    rf"V(?P<velocity>{_STANDALONE_NUM})"
    rf"D(?P<dsafe>{_NUM})$",
    re.VERBOSE
)

RUN_DIR_RE = re.compile(r"^(\d+)$", re.VERBOSE)

@dataclass
class RunData:
    solverDict: dict[str,int|float|npt.NDArray]
    x: npt.NDArray | None
    u: npt.NDArray | None

@dataclass
class BenchmarkData:
    center: list
    radius: float
    lower: float
    upper: float
    velocity: float
    dsafe: float
    rawName: str
    runDataDict: dict[int,RunData]
 
def _parsePlainNumber(token: str) -> float:
    if token == "0":
        return 0.0
 
    sign = -1.0 if token.startswith("-") else 1.0
    digits = token.lstrip("-")
 
    return sign * float(f"{digits[0]}.{digits[1:]}")
 
def _parsePiOrNumber(token: str) -> float:
    sign = -1.0 if token.startswith("-") else 1.0
    body = token.lstrip("-")
 
    if body.startswith("pi"):
        rest = body[2:]
        if rest == "":
            return sign * math.pi
        return sign * math.pi / int(rest)
 
    return _parsePlainNumber(token)
 
def _parseCenter(token: str) -> list[float]:
    parts = _CENTER_COMPONENT_RE.findall(token)
 
    if len(parts) != 3 or "".join(parts) != token:
        raise ValueError(f"Could not cleanly split center token into 3 components: {token!r}")
 
    return [_parsePlainNumber(p) for p in parts]
 
def parseBenchmarkData(benchmarkDir: Path) -> BenchmarkData:
    m = CODE_RE.match(benchmarkDir.name)
    if not m:
        raise ValueError(f"Name does not match expected pattern: {benchmarkDir.name!r}")
 
    return BenchmarkData(
        center=_parseCenter(m.group("center")),
        radius=_parsePlainNumber(m.group("radius")),
        lower=_parsePiOrNumber(m.group("lower")),
        upper=_parsePiOrNumber(m.group("upper")),
        velocity=_parsePlainNumber(m.group("velocity")),
        dsafe=_parsePlainNumber(m.group("dsafe")),
        rawName=benchmarkDir.name,
        runDataDict=_parseRunDataAll(benchmarkDir),
    )
 
def _parseRunDataSingle(projectDir: Path) -> tuple[int, RunData | None]:
    projectNumber = int(projectDir.name)
    solverPath = projectDir / "solver.json"
    statePath = projectDir / "x.txt"
    controlPath = projectDir / "u.txt"
 
    if not solverPath.is_file():
        print(f"[bold orange3][WARN][/bold orange3] no solver.json in {projectDir} found")
        return projectNumber, None

    try:
        with solverPath.open("r", encoding="utf-8") as f:
            solverDict = json.load(f)
        stateArr = np.loadtxt(statePath) if statePath.is_file() else None
        controlArr = np.loadtxt(controlPath) if controlPath.is_file() else None
        return projectNumber, RunData(solverDict, stateArr, controlArr)
    except (json.JSONDecodeError, OSError) as e:
        print(f"[bold red][ERROR][/bold red] failed to read: {e}")
        return projectNumber, None
 
def _parseRunDataAll(parentDir: Path, max_workers: int | None = None) -> dict[int, RunData]:
    runDirs = [
        p for p in parentDir.iterdir()
        if p.is_dir() and RUN_DIR_RE.match(p.name)
    ]
 
    results: dict[int, RunData] = {}

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(_parseRunDataSingle, d): d
            for d in runDirs
        }
        for future in as_completed(futures):
            runNumber, data = future.result()
            if data is not None:
                results[runNumber] = data
 
    return results