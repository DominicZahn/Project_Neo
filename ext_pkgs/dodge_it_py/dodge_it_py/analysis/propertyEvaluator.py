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
from ext_pkgs.dodge_it_py.dodge_it_py.stability import PolygonOfSupport
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N

COLOR_VALUES_d = "red"
COLOR_CUT_d = "blue"
COLOR_MIN_d = "black"
ANNOTE_MARGIN_Y_d = 0.02
ANNOTE_MARGIN_X_d = 0.05
def drawDistance(t : npt.NDArray,
                 d_arr : npt.NDArray,
                 d_safe : float,
                 outPath : Path) -> None:
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111)

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
    ax.set_ylim(0,np.max(d_max)*1.1)
    fig.savefig(str(outPath), format="pdf")

COLOR_ZMP = "red"
def drawZMP(zmp_arr : npt.NDArray,
            PoS : PolygonOfSupport,
            outPath : Path) -> None:
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111)

    xy = zmp_arr.transpose()[:2]
    ax.scatter(xy[1], xy[0], marker=".", c=COLOR_ZMP, alpha=0.2)
    p00, p10, p11, p01 = PoS.get_corners()
    anker = (float(p00[1]), float(p00[0]))
    diag = c.DM(p11-p00)
    posVis = plt.Rectangle(anker,
                           float(diag[1]),
                           float(diag[0]),
                           facecolor="black", alpha=0.2)
    ax.add_patch(posVis)
    ax.scatter((0), (0), marker="+", c="black")

    fig.set_tight_layout(True)
    ax.set_aspect(1)
    ax.set_title("Zero Moment Point (ZMP) in XY-Plane")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.savefig(str(outPath), format="pdf")

COLOR_VALUES_STABILITY = "red"
COLOR_CUT_STABILITY = "blue"
COLOR_MAX_STABILITY = "black"
ANNOTE_MARGIN_Y_STABILITY = 0.02
ANNOTE_MARGIN_X_STABILITY = 0.05
def drawStability(stab_arr : npt.NDArray,
                  stab_safe : float,
                  t : npt.NDArray,
                  outPath : Path) -> None:
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111)
    t = t[1:N-1]
    stab_arr = stab_arr[:,1:N-1]
    for s in stab_arr:
        ax.plot(t, s, COLOR_VALUES_STABILITY, alpha=0.1)

    s_arrMean = np.nanmean(stab_arr, axis=0)
    s_min = np.nanmin(stab_arr, axis=0)
    s_max = np.nanmax(stab_arr, axis=0)
    ax.plot(t, s_arrMean, color=COLOR_VALUES_STABILITY)
    ax.fill_between(t, s_min, s_max, alpha=0.2, color=COLOR_VALUES_STABILITY)
    xyMax = (t[np.argmax(s_max)], np.max(s_max))
    ax.annotate(f"peak stability: {round(np.max(s_max),2)}",
                xy=xyMax,
                xytext=(ANNOTE_MARGIN_X_STABILITY,xyMax[1]+ANNOTE_MARGIN_Y_STABILITY))
    ax.scatter(*xyMax, marker=".", c=COLOR_MIN_d)
    ax.plot([0,Tf], [np.max(s_max), np.max(s_max)], color=COLOR_MAX_STABILITY, alpha=0.5)
    ax.plot([0,Tf], [stab_safe, stab_safe], color=COLOR_CUT_STABILITY, alpha=1)
    ax.annotate(f"PoS limit: {round(stab_safe,2)}",
                xy=(ANNOTE_MARGIN_X_STABILITY,stab_safe),
                xytext=(ANNOTE_MARGIN_X_STABILITY,stab_safe+ANNOTE_MARGIN_Y_STABILITY),
                color=COLOR_CUT_STABILITY)

    fig.set_tight_layout(True)
    ax.set_title("Stability as normalized quadratic distance to center")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("stability")
    ax.set_xlim(0,Tf)
    ax.set_ylim(0.0, 1.1)
    fig.savefig(str(outPath), format="pdf")

def main(path : Path) -> int:
    benchmarkData = parseBenchmarkData(path)
    h1 = H1Wrapper_v2(q0='knees_bend_0.4_straight',
                      dynamicJoints=DYNAMIC_JOINT_NAMES,
                      visualization=False)
    d_arr = np.full((len(benchmarkData.runDataDict),N), np.nan)
    zmp_arr = np.full((len(benchmarkData.runDataDict),N,3), np.nan)
    stab_arr = np.full((len(benchmarkData.runDataDict),N), np.nan)

    PoS = PolygonOfSupport()
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
        qdot = x[:,nq:]
        q = x[:,:nq]

        # properties
        #           distance
        f_d = c.Function("f_d", [h1.q, h1.t], [h1.cProjectileRobotDistance(h1.t)])
        f_d_map = f_d.map(N, "thread")
        d = f_d_map(q[:-1].transpose(), t).toarray()
        assert(type(d) is np.ndarray)
        d = d.flatten()
        d_arr[i] = d

        #           ZMP
        f_ZMP = c.Function("f_ZMP", [h1.q, h1.qdot, h1.tau], [h1.ZMP])
        f_ZMP_map = f_ZMP.map(N, "thread")
        zmp = f_ZMP_map(q[:-1].transpose(), qdot[:-1].transpose(), tau.transpose()).toarray()
        assert(type(d) is np.ndarray)
        zmp_arr[i] = zmp.transpose()

        #           stability
        f_stability = c.Function("f_stability",
                                 [h1.q, h1.qdot, h1.tau],
                                 [PoS.stability_centerDistParable(h1.ZMP)])
        f_stability_map = f_stability.map(N, "thread")
        stab = f_stability_map(q[:-1].transpose(), qdot[:-1].transpose(), tau.transpose()).toarray()
        stab_arr[i] = stab.transpose().flatten()

    drawDistance(t,
                 d_arr,
                 benchmarkData.dsafe,
                 path / "distance.pdf")

    drawZMP(zmp_arr[:,1:N-1,:], PoS, path / "zmp.pdf")
    drawStability(stab_arr[:,1:N-1], 0.8, t[1:N-1], path / "stability.pdf")

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=Path, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path))