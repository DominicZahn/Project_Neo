import sys
import numpy as np
from pathlib import Path

from ext_pkgs.dodge_it_py.dodge_it_py.neo.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.neo.H1Wrapper_v2 import H1Wrapper_v2
from ext_pkgs.dodge_it_py.dodge_it_py.ocpDebugger import OcpDebugger

def main(args=None) -> int:
    dynamicJointNames = [
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        'left_ankle_roll_joint',
        'right_ankle_roll_joint'
    ]
    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4',
        dynamicJoints=dynamicJointNames)
    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq))
    
    Tf = 0.5
    N = 33

    ocp = OCP(h1, Tf, N)
    status = ocp.solve(plot=True)

    while True:
        print(h1._vis.viewer.url())
        print(h1.model)

        x = ocp.getSimX()
        q = x[:,:nq]
        qdot = x[:,nq:]
        tau = ocp.getSimU(floatBase=True)
        t = ocp.getSimT()

        if status != 0:
            print("------------------- ❌ NO CONVERGENCE -------------------")
        else:
            print("------------------- 🎉 CONVERGENCE 🎉 -------------------")

        key = input("Press RETURN for visalization\n q to exit\n s to save controls and states\n c to save joints in csv\n d to start ocp debugger\n")
        if key == 'q':
            break
        elif key == 's':
            ocp.save()
        elif key == 'c':
            h1.saveJointTrajectory(
                q,
                Path("jointTrajectory.csv")
            )
        elif key == 'd':
            debugger = OcpDebugger(ocp.solver)
            debugger.run()           
        else:
            h1.visualizeJointTrajecotry(q, qdot, tau, t, 1)

    return status

if __name__ == "__main__":
    sys.exit(main())