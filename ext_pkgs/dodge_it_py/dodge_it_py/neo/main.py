import sys
import numpy as np

from ext_pkgs.dodge_it_py.dodge_it_py.neo.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.neo.H1Wrapper_v2 import H1Wrapper_v2

def main(args=None) -> int:
    dynamicJointNames = [
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint'
    ]

    Tf = 5.0
    N = 33

    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4',
        dynamicJoints=dynamicJointNames)
    h1.visualize(h1.q0)

    ocp = OCP(h1, Tf, N)
    status = ocp.solve(plot=True)
    if status != 0:
        return status

    input("Press any key to rerun...")

    return 0

if __name__ == "__main__":
    sys.exit(main())