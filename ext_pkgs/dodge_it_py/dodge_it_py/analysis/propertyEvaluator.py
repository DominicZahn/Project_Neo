import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import sys
from pathlib import Path
import argparse
from rich import print
from rich.table import Table
import casadi as c

from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import parseBenchmarkData, BenchmarkData
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N

COLOR_VALUES_d = "red"
COLOR_CUT_d = "blue"
COLOR_MIN_d = "black"
ANNOTE_MARGIN_Y_d = 0.02
ANNOTE_MARGIN_X_d = 0.05
def drawDistance(ax : Axes,
                 fig : Figure,
                 t : npt.NDArray,
                 d_arr : npt.NDArray,
                 d_safe : float,
                 outPath : Path) -> None:
    
    for d in d_arr:
        ax.plot(t, d, color=COLOR_VALUES_d, alpha=0.1)

    d_arrMean = np.nanmean(d_arr, axis=0)
    d_min = np.nanmin(d_arr, axis=0)
    d_max = np.nanmax(d_arr, axis=0)
    ax.plot(t, d_arrMean, color=COLOR_VALUES_d)
    ax.fill_between(t, d_min, d_max, alpha=0.2, color=COLOR_VALUES_d)
    xyMin = (t[np.argmin(d_min)], np.min(d_min))
    ax.annotate(f"minimal distance: {round(np.min(d_min),2)} m",
                xy=xyMin,
                xytext=(ANNOTE_MARGIN_X_d,xyMin[1]+ANNOTE_MARGIN_Y_d))
    ax.scatter(*xyMin, marker=".", c=COLOR_MIN_d)
    ax.plot([0,Tf], [np.min(d_min), np.min(d_min)], color=COLOR_MIN_d, alpha=0.5)
    ax.plot([0,Tf], [d_safe, d_safe], color=COLOR_CUT_d, alpha=1)
    ax.annotate(f"safety distance: {round(d_safe,2)} m",
                xy=(ANNOTE_MARGIN_X_d,d_safe),
                xytext=(ANNOTE_MARGIN_X_d,d_safe+ANNOTE_MARGIN_Y_d),
                color=COLOR_CUT_d)

    fig.set_tight_layout(True)
    ax.set_title("Distance between Robot and Projetile")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("robot-projectile distance [m]")
    ax.set_xlim(0,Tf)
    ax.set_ylim(0,np.max(d_max))

    fig.savefig(str(outPath), format="pdf")

def main(path : Path) -> int:
    benchmarkData = parseBenchmarkData(path)
    h1 = H1Wrapper_v2(q0='knees_bend_0.4_straight',
                      dynamicJoints=DYNAMIC_JOINT_NAMES,
                      visualization=False)

    fig = plt.figure(figsize=(7,7))
    axDistance = fig.add_subplot(111)

    d_all = np.full((len(benchmarkData.runDataDict),N), np.nan)

    t = np.linspace(0.0, Tf, N)
    for i,rd in benchmarkData.runDataDict.items():
        # finish setup
        p = c.SX(rd.solverDict["projectile_position"])
        v = c.SX([0.0, 0.0, benchmarkData.velocity])
        projObj = projectile.linear(h1.t, p, v)
        h1.setCollision(projObj)

        # load states and controls
        nq = h1.model.nq
        assert(type(nq) is int)
        if rd.u is None or rd.x is None:
            continue
        u = rd.u.reshape((-1,nq-6))
        tau = np.hstack((np.zeros((N,6)), u)) # add floating base torques
        x = rd.x.reshape((-1,nq*2))
        q = x[:,nq:]
        qdot = x[:,:nq]

        # properties
        f_d = c.Function("f_d", [h1.q, h1.t], [h1.cProjectileRobotDistance(h1.t)])
        f_ZMP = c.Function("f_ZMP", [h1.q, h1.qdot, h1.tau], [h1.ZMP])

        #           distance
        f_d_map = f_d.map(N, "thread")
        d = f_d_map(q[:-1].transpose(), t).toarray()
        assert(type(d) is np.ndarray)
        d = d.flatten()
        d_all[i] = d

    drawDistance(
        axDistance,
        fig,
        t,
        d_all,
        benchmarkData.dsafe,
        path / "distance.pdf")

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=Path, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path))