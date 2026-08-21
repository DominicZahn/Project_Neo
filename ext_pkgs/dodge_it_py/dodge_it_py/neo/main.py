import sys
import numpy as np
from pathlib import Path
import casadi as c
import subprocess
import argparse

from ext_pkgs.dodge_it_py.dodge_it_py.neo.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2, HeadlessData
from ext_pkgs.dodge_it_py.dodge_it_py.ocpDebugger import OcpDebugger
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile

def mainInteractive(showCollision : bool,
                    initalValues : bool,
                    headless : bool) -> int:
    dynamicJointNames = [
        # 'left_hip_yaw_joint',
        # 'right_hip_yaw_joint',
        # 'left_hip_roll_joint',
        # 'right_hip_roll_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        # 'left_ankle_roll_joint',
        # 'right_ankle_roll_joint',
        'torso_joint',
    ]

    headlessData = HeadlessData("/home/robot/ws/_tmp/",
                                np.array([-0.5, 0.1, 0.5]),
                                np.array([0.0, 0.0, 0.0]),
                                (1920,1080))
    
    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=dynamicJointNames,
        showCollisionSDF=showCollision,
        headlessData=headlessData if headless else None)
    h1.setCollision(
        projectile.linear(h1.t,
                          c.SX([0.9 ,0 , 1.6]),
                          c.SX([-0.6, 0., 0.])))
    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq), 0.0)
    
    Tf = 2.5
    N = 120

    ocp = OCP(h1, Tf, N, loadInitalValues=initalValues)
    status = ocp.solve(plot=True)

    while True:
        print(h1._vis.viewer.url())
        print(h1.model)

        x = ocp.getSimX()
        q = x[:,:nq]
        qdot = x[:,nq:]
        tau = ocp.getSimU(floatBase=True)
        
        t = ocp.solver.get_flat('p')
        t = t.reshape((N+1,7))[:,-1]

        if status != 0:
            print("------------------- ❌ NO CONVERGENCE -------------------")
        else:
            print("------------------- ✅ CONVERGENCE -------------------")

        key = input("Press RETURN for visalization\n q to exit\n s to save controls and states\n c to save joints in csv\n d to start ocp debugger\n")
        if key == 'q':
            break
        elif key == 's':
            ocp.saveAsInitalValues()
        elif key == 'c':
            h1.saveJointTrajectory(
                q,
                Path("jointTrajectory.csv")
            )
        elif key == 'd':
            debugger = OcpDebugger(ocp.solver)
            debugger.run()           
        else:
            h1.visualizeJointTrajecotry(q, qdot, tau, t, 1.0)
            if headless:
                subprocess.run(["ffmpeg",
                                "-i", f"{headlessData.dir}/%05d.png",
                                "-framerate", str(N/Tf),
                                f"{headlessData.dir}/_video.gif"])

    return status

def main():
    parser = argparse.ArgumentParser()
    interactiveGroup = parser.add_argument_group("interactive mode")
    interactiveGroup.add_argument("--showCollision",
                                  action="store_true",
                                  required=False)
    interactiveGroup.add_argument("--initalValues",
                                  action="store_true",
                                  required=False)
    interactiveGroup.add_argument("--headless",
                                  action="store_true",
                                  required=False)
    benchmarkGroup = parser.add_argument_group("benchmark mode")
    benchmarkGroup.add_argument("--benchmark",
                                action="store_true",
                                required=False)
    args = parser.parse_args()

    sys.exit(mainInteractive(args.showCollision, args.initalValues, args.headless))
