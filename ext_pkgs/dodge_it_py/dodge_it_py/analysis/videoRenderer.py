import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import sys
from pathlib import Path
import argparse
from rich import print
from rich.table import Table
import casadi as c

from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import parseBenchmarkData, BenchmarkData
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2, generateVideoFromFrames, HeadlessData
from ext_pkgs.dodge_it_py.dodge_it_py.stability import PolygonOfSupport
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N

def main(path : Path, nVideos : int) -> int:
    
    benchmarkData = parseBenchmarkData(path)
    t = np.linspace(0.0, Tf, N)
    for i,rd in benchmarkData.runDataDict.items():
        if i % nVideos != 0:
            continue
        if rd.u is None or rd.x is None:
            print(f"[bold orange3][WARNING][/] Skipped {i}")
            continue

        pathDir = str(path/(str(i).zfill(5)))
        headlessData = HeadlessData(pathDir,
                                    np.array([-0.05, -0.1, 0.7]),
                                    np.array([0.0, 0.0, -1.0]),
                                    (1080,1920))
    
        h1 = H1Wrapper_v2(q0='knees_bend_0.4_straight',
                          dynamicJoints=DYNAMIC_JOINT_NAMES,
                          showCollisionSDF=True,
                          visualization=headlessData)
        nq = h1.model.nq
        assert(type(nq) is int)
        p = c.SX(rd.solverDict["projectile_position"])
        v = c.SX(rd.solverDict["projectile_velocity"])
        projObj = projectile.linear(h1.t, p, v)
        h1.setCollision(projObj)

        # load states and controls
        u = rd.u.reshape((-1,nq-6))
        tau = np.hstack((np.zeros((N,6)), u)) # add floating base torques
        x = rd.x.reshape((-1,nq*2))
        qdot = x[:,nq:]
        q = x[:,:nq]

        #           generate videos
        if not (Path(pathDir)/"frames").exists():
            print(f"[bold green][INFO][/] Generating frames for {i}.")
            h1.visualizeJointTrajecotry(q, qdot, tau, t, 1.0)
            generateVideoFromFrames(pathDir, N, Tf)
        h1.closeHeadless()
        del h1
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--nVideos", type=int, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path, args.nVideos))