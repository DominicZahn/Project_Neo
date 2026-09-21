import sys
import argparse
from pathlib import Path
from time import sleep
import casadi as c
import numpy as np
import numpy.typing as npt
from scipy.interpolate import make_interp_spline
from rich import print

from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2, generateVideoFromFrames, HeadlessData
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

def interpolate(arr : npt.NDArray, start : int, end : int, invSlowMoFactor : int):
    segment = arr[start:end]
    n = segment.shape[0]
    x_old = np.arange(n)
    x_new = np.linspace(0, n - 1, n * invSlowMoFactor)
    spline = make_interp_spline(x_old, segment, k=1, axis=0)
    return spline(x_new)

def main(path : Path,
         runId : int,
         bulletFrameStart : int,
         bulletFrameEnd : int,
         invSlowMoFactor : int) -> int:
    benchmarkData = parseBenchmarkData(path)
    rd = benchmarkData.runDataDict[runId]
    if rd.u is None or rd.x is None:
        print(f"[bold red][ERROR][/] Run {runId} has no control and state information.")
        return -1

    headlessData = HeadlessData(str(path/str(runId).zfill(5)),
                            camPos=None,
                            camLookAt=None,
                            resolution=(1080,1920))


    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=DYNAMIC_JOINT_NAMES,
        # visualization=True,
        visualization=headlessData,
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
    # first part of motion
    print("[bold green][INFO][/] APPROACH PHASE")
    h1.movePitchCamera(camPosTraj[0], -np.pi / 8)
    h1.visualizeJointTrajecotry(
        q[:bulletFrameStart],
        qdot[:bulletFrameStart],
        tau[:bulletFrameStart],
        t[:bulletFrameStart],
    )
    # bullet time
    print("[bold green][INFO][/] SLOWMO PHASE")
    fps = N / Tf
    for pos, qBullet, qdotBullet, tauBullet, tBullet in zip(
        camPosTraj,
        interpolate(q, bulletFrameStart, bulletFrameEnd, invSlowMoFactor),
        interpolate(qdot, bulletFrameStart, bulletFrameEnd, invSlowMoFactor),
        interpolate(tau, bulletFrameStart, bulletFrameEnd, invSlowMoFactor),
        interpolate(t, bulletFrameStart, bulletFrameEnd, invSlowMoFactor)
        ):
        h1.movePitchCamera(pos, -np.pi / 8)
        h1.visualizeJointConfig(qBullet, qdotBullet, tauBullet, tBullet)
        sleep(invSlowMoFactor/fps)
    # second half of motion
    print("[bold green][INFO][/] RECOVER PHASE")
    h1.movePitchCamera(camPosTraj[-1], -np.pi / 8)
    h1.visualizeJointTrajecotry(
        q[bulletFrameEnd:],
        qdot[bulletFrameEnd:],
        tau[bulletFrameEnd:],
        t[bulletFrameEnd:],
    )
    h1.closeHeadless()
    del h1
    generateVideoFromFrames(headlessData.dir, N, Tf)

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", "-p", type=Path, required=True)
    parser.add_argument("--runId", "-i", type=int, required=True)
    parser.add_argument("--bulletFrameStart", "-bfs", type=int, required=True)
    parser.add_argument("--bulletFrameEnd", "-bfe", type=int, required=True)
    parser.add_argument("--invSlowMoFactor", "-ism", type=int, required=True)
    args = parser.parse_args()
    sys.exit(main(args.path, args.runId, args.bulletFrameStart, args.bulletFrameEnd, args.invSlowMoFactor))