import re
import math
from dataclasses import dataclass
from pathlib import Path
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import numpy.typing as npt

RUN_DIR_RE = re.compile(r"^(\d+)$", re.VERBOSE)

@dataclass
class RunData:
    solverDict: dict[str,int|float|npt.NDArray]
    x: npt.NDArray | None
    u: npt.NDArray | None

@dataclass
class BenchmarkData:
    center: list[float]
    radius: list[float]
    lower: float
    upper: float
    velocity: float
    dsafe: float
    rawName: str
    runDataDict: dict[int,RunData]

def parseBenchmarkData(benchmarkDir: Path) -> BenchmarkData:
    infoPath = benchmarkDir / "info.json"
    with infoPath.open("r", encoding="utf-8") as f:
        infoDict = json.load(f)

    return BenchmarkData(
        center=infoDict["center"],
        radius=infoDict["radius"],
        lower=infoDict["lower"],
        upper=infoDict["upper"],
        velocity=infoDict["velocity"],
        dsafe=infoDict["dsafe"],
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