import sys
import numpy as np
import numpy.typing as npt
import casadi as c

from ext_pkgs.dodge_it_py.dodge_it_py.neo.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.neo.H1Wrapper_v2 import H1Wrapper_v2

def main(args=None) -> int:
    dynamicJointNames = [
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        # 'left_ankle_roll_joint',
        # 'right_ankle_roll_joint'
    ]
    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4',
        dynamicJoints=dynamicJointNames)
    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq))
    
    Tf = 2.0
    N = 33

    ocp = OCP(h1, Tf, N)
    status = ocp.solve(plot=True)
    if status != 0:
        return status

    while True:
        print(h1._vis.viewer.url())
        print(h1.model)

        x = ocp.getSimX()
        q = x[:,:nq]
        qdot = x[:,nq:]
        tau = ocp.getSimU()
        t = ocp.getSimT()

        key = input("Press RETURN for visalization; q to exit; s to save controls and states\n")
        if key == 'q':
            break
        elif key == 's':
            ocp.save()
        else:
            h1.visualizeJointTrajecotry(q, qdot, tau, t, 1)

    return 0

if __name__ == "__main__":
    sys.exit(main())