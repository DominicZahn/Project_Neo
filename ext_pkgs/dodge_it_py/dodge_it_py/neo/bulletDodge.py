import sys
import argparse
from pathlib import Path
from time import sleep
import casadi as c
import numpy as np
import numpy.typing as npt
from rich import print

from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.analysis.benchmarkParser import parseBenchmarkData, BenchmarkData


def generateCameraTrajectory(
             center : npt.NDArray,
             radius : float,
             startAngle : float,
             endAngle : float,
             N : int,
             zOffset : float) -> npt.NDArray:
    angles = np.linspace(startAngle, endAngle, N)
    p = np.zeros((N,3)) + center
    p[:, 0] += np.sin(angles) * radius
    p[:, 1] += np.cos(angles) * radius
    p[:, 2] += np.linspace(0, zOffset, N)
    return p

def main(path : Path,
         runId : int,
         bulletFrameStart : int,
         bulletFrameEnd : int) -> int:
    benchmarkData = parseBenchmarkData(path)
    rd = benchmarkData.runDataDict[runId]
    if rd.u is None or rd.x is None:
        print(f"[bold red][ERROR][/] Run {runId} has no control and state information.")
        return -1

    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=DYNAMIC_JOINT_NAMES,
        visualization=True,
        showCollisionSDF=True,
    )

    h1._vis.viewer["/Lights/SpotLight"].set_property("visible", True)
    h1._vis.viewer["/meshcat/zmp"].set_property("visible", False)
    h1._vis.viewer["/Axes"].set_property("visible", False)
    h1._vis.viewer["/Grid"].set_property("visible", False)
    pos = np.array([-6, 1, 0])
    assert(h1.model.nq)
    nq =  h1.model.nq
    p = c.SX(rd.solverDict["projectile_position"])
    v = c.SX(rd.solverDict["projectile_velocity"])
    h1.setCollision(projectile.linear(h1.t, p, v))
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq), 0.0)

    # load states and controls
    t = np.linspace(0, Tf, N)
    u = rd.u.reshape((-1,nq-6))
    tau = np.hstack((np.zeros((N,6)), u)) # add floating base torques
    x = rd.x.reshape((-1,nq*2))
    qdot = x[:,nq:]
    q = x[:,:nq]

    # ---------------------
    camPosTraj = generateCameraTrajectory(
        np.array([-3, 0, 0.5]),
        2.5,
        np.pi / 4,
        2*np.pi + np.pi / 4,
        100,
        0
    )
    input("START")
    # first part of motion
    h1.movePitchCamera(camPosTraj[0], -np.pi / 8)
    h1.visualizeJointTrajecotry(
        q[:bulletFrameEnd],
        qdot[:bulletFrameEnd],
        tau[:bulletFrameEnd],
        t[:bulletFrameEnd],
    )
    # bullet time
    for pos in camPosTraj:
        h1.movePitchCamera(pos, -np.pi / 8)
        sleep(0.05)
    # second half of motion
    h1.movePitchCamera(camPosTraj[-1], -np.pi / 8)
    h1.visualizeJointTrajecotry(
        q[bulletFrameEnd:],
        qdot[bulletFrameEnd:],
        tau[bulletFrameEnd:],
        t[bulletFrameEnd:],
    )

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", "-p", type=Path, required=True)
    parser.add_argument("--runId", "-i", type=int, required=True)
    parser.add_argument("--bulletFrameStart", "-bfs", type=int, required=True)
    parser.add_argument("--bulletFrameEnd", "-bfe", type=int, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path, args.runId, args.bulletFrameStart, args.bulletFrameEnd))