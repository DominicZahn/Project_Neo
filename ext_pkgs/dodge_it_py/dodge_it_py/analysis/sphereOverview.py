import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import json
import sys
import re
from rich import print
from rich.table import Table
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse

from ext_pkgs.dodge_it_py.dodge_it_py.sample import SemiSphere
from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import BenchmarkData, parseBenchmarkName

STATUS_CODES = {
    0: "Success",
    1: "NaN detected",
    2: "Maximum number of iterations reached",
    3: "Minimum step size reached",
    4: "QP solver failed",
    5: "Problem unbounded",
    6: "Solver timeout",
    7: "QP scaling could not satisfy bounds"
}

VERTS_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/verts.txt"
FACES_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/faces.txt"

PROJECT_DIR_RE = re.compile(r"^(\d+)$")
 
def parseSingle(projectDir: Path) -> tuple[int, dict | None]:
    projectNumber = int(projectDir.name)
    path = projectDir / "solver.json"
 
    if not path.is_file():
        print(f"[bold orange3][WARN][/bold orange3] no solver.json in {projectDir} found")
        return projectNumber, None
 
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return projectNumber, data
    except (json.JSONDecodeError, OSError) as e:
        print(f"[bold red][ERROR][/bold red] Failed to read {path}: {e}")
        return projectNumber, None
 
 
def parseAll(parent_dir: str | Path, max_workers: int | None = None) -> dict[int, dict]:
    parent = Path(parent_dir)
 
    project_dirs = [
        p for p in parent.iterdir()
        if p.is_dir() and PROJECT_DIR_RE.match(p.name)
    ]
 
    results: dict[int, dict] = {}

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(parseSingle, d): d
            for d in project_dirs
        }
        for future in as_completed(futures):
            project_number, data = future.result()
            if data is not None:
                results[project_number] = data
 
    return results

def printSummary(projectDataDict : dict[int, dict], output : Path) -> None:
    # Solver Status Table    
    statusList = np.array([d["status"] for i,d in projectDataDict.items()])
    n = statusList.size
    table = Table(title="Solver Status Summary")
    for c in STATUS_CODES.values(): table.add_column(c)
    counts = [np.sum(statusList == k) for k in STATUS_CODES.keys()]
    countsStr = [f"{c} ({round(c/n*100.0,2)}%)" for c in counts]
    table.add_row(*countsStr)
    print(table)

    # safe summary to json
    solverSummaryFile = open(f"{output}/solverSummary.json", "w+")
    json.dump(dict(zip(STATUS_CODES.values(), countsStr)),
              solverSummaryFile)
    solverSummaryFile.close()

def draw(center : list[float],
         velocity : float,
         radius : float,
         projectDataDict : dict[int, dict],
         out : Path) -> None:
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111, projection="3d")

    # draw projectiles
    ax.scatter(*center, color="black", marker="x")
    pts = np.hstack([np.array(d["projectile_position"]) for i,d in projectDataDict.items()])
    colorList = ["green" if d["status"] == 0 else "darkorange" for i,d in projectDataDict.items()]
    ax.scatter(*pts, c=colorList)

    # draw collision SDF
    verts = np.loadtxt(VERTS_FILE)
    faces = np.loadtxt(FACES_FILE).astype(int)
    mesh = Poly3DCollection(verts[faces], alpha=0.25)
    ax.add_collection3d(mesh)

    # draw projectile trajectories
    errorMask = [d["status"] != 0 for i,d in projectDataDict.items()]
    for p0 in pts.transpose()[errorMask]:
        v = p0 - center
        v /= np.linalg.norm(v)
        v *= -velocity * 2.5
        p1 = p0 + v
        pStack = np.vstack((p0,p1)).transpose()
        ax.plot(*pStack, c="gray", alpha=0.2)

    scale = 1.1
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Semi-sphere Overview")
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim((center[0]-radius)*scale, (center[0]+radius)*scale)
    ax.set_ylim((center[1]-radius)*scale, (center[1]+radius)*scale)
    ax.set_zlim(0, 2.5)
    plt.tight_layout()

    # iso view
    isoFile = f"{out}/iso.png"
    plt.savefig(isoFile, dpi=150)
    print(f"Plot saved to {isoFile}")
    # top view
    topFile = f"{out}/top.png"
    ax.view_init(elev=90, azim=0, roll=0)
    plt.savefig(topFile, dpi=150)
    print(f"Plot saved to {topFile}")
    # front view
    frontFile = f"{out}/front.png"
    ax.view_init(elev=0, azim=0, roll=0)
    plt.savefig(frontFile, dpi=150)
    print(f"Plot saved to {frontFile}")
    # side view
    sideFile = f"{out}/side.png"
    ax.view_init(elev=0, azim=90, roll=0)
    plt.savefig(sideFile, dpi=150)
    print(f"Plot saved to {sideFile}")

    printSummary(projectDataDict, out)

def main(path : Path) -> int:
    benchmarkData = parseBenchmarkName(path.name)
    d = parseAll(path)
    if len(d) == 0:
        print("[bold red][ERROR][/bold red] input directory is empty")
        return -1
    draw(benchmarkData.center,
         benchmarkData.velocity,
         benchmarkData.radius,
         d,
         path)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", "-p", required=True, type=Path)
    args = parser.parse_args()
    assert(args.path.exists() and args.path.is_dir())

    sys.exit(main(args.path))
