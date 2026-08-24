import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import json
import sys
from rich import print
from rich.table import Table
from pathlib import Path
import argparse

from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import BenchmarkData, parseBenchmarkData

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
    isoFile = f"{out}/iso.pdf"
    plt.savefig(isoFile)
    print(f"Plot saved to {isoFile}")
    # top view
    topFile = f"{out}/top.pdf"
    ax.view_init(elev=90, azim=0, roll=0)
    plt.savefig(topFile)
    print(f"Plot saved to {topFile}")
    # front view
    frontFile = f"{out}/front.pdf"
    ax.view_init(elev=0, azim=0, roll=0)
    plt.savefig(frontFile)
    print(f"Plot saved to {frontFile}")
    # side view
    sideFile = f"{out}/side.pdf"
    ax.view_init(elev=0, azim=90, roll=0)
    plt.savefig(sideFile)
    print(f"Plot saved to {sideFile}")

    printSummary(projectDataDict, out)

def main(path : Path) -> int:
    benchmarkData = parseBenchmarkData(path)
    if len(benchmarkData.runDataDict) == 0:
        print("[bold red][ERROR][/bold red] input directory is empty")
        return -1
    
    draw(benchmarkData.center,
         benchmarkData.velocity,
         benchmarkData.radius,
         {i: rd.solverDict for i,rd in benchmarkData.runDataDict.items()},
         path)
    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", "-p", required=True, type=Path)
    args = parser.parse_args()
    assert(args.path.exists() and args.path.is_dir())

    sys.exit(main(args.path))
