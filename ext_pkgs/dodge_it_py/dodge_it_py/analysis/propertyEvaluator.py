import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import json
import sys
from pathlib import Path
import argparse
import re
from concurrent.futures import ThreadPoolExecutor, as_completed
from rich import print
from rich.table import Table
import casadi as c

from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import parseBenchmarkData, BenchmarkData
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N

def main(path : Path) -> int:
    benchmarkData = parseBenchmarkData(path)
    h1 = H1Wrapper_v2(q0='knees_bend_0.4_straight',
                      dynamicJoints=DYNAMIC_JOINT_NAMES,
                      visualization=False)

    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111)

    d_all = np.full((len(benchmarkData.runDataDict),N), np.nan)

    COLOR_VALUES_d = "red"
    COLOR_CUT_d = "blue"

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

        # test properties
        f_d = c.Function("f_d", [h1.q, h1.t], [h1.cProjectileRobotDistance(h1.t)])
        f_ZMP = c.Function("f_ZMP", [h1.q, h1.qdot, h1.tau], [h1.ZMP])

        f_d_map = f_d.map(N, "thread")
        d = f_d_map(q[:-1].transpose(), t).toarray()
        assert(type(d) is np.ndarray)
        d = d.flatten()
        if d.any() <= 0.0:
            breakpoint()
        d_all[i] = d

        ax.plot(t, d, color=COLOR_VALUES_d, alpha=0.1)

    d_allMean = np.nanmean(d_all, axis=0)
    d_min = np.nanmin(d_all, axis=0)
    d_max = np.nanmax(d_all, axis=0)
    ax.plot(t, d_allMean, color=COLOR_VALUES_d)
    ax.fill_between(t, d_min, d_max, alpha=0.2, color=COLOR_VALUES_d)
    ax.plot([0,Tf], [benchmarkData.dsafe, benchmarkData.dsafe], color=COLOR_CUT_d, alpha=1, linewidth=2)
    fig.set_tight_layout(True)
    ax.set_title("Distance between Robot and Projetile")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("robot-projectile distance [m]")

    outPath = path / "distance.pdf"
    fig.savefig(str(outPath), format="pdf")

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=Path, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path))