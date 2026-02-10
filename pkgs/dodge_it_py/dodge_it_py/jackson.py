import numpy as np
import casadi as c
import pinocchio.casadi as cpin
from acados_template import (
    AcadosOcpSolver
)

import time

from pkgs.dodge_it_py.dodge_it_py.h1wrapper import *
from pkgs.dodge_it_py.dodge_it_py.ocp_def import OCP

def print_joints(cmodel, cdata):
    for name, oMi in zip(cmodel.names, cdata.oMi):
        print(name, '', oMi.translation.T)

def main(args=None) -> int:
    dynamic_joint_names = [
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        'torso_joint'
    ]
    h1 = H1Wrapper()
    h1.fixJoints(
        dynamic_joint_names,
        dynamic_representation=True)
    
    mirrors_res = h1.mirrorJoints('left_hip_pitch_joint', 'right_hip_pitch_joint')
    if not mirrors_res:
        return -1

    names = list(h1.robot.model.names)
    q0 = np.zeros(len(names)-1)
    q0[1] = 1.0
    q0 = h1.reduced2Mirrored(np.array(q0))
    print(q0)
    print(names)
    print(h1.jointNames(reduced=True))
    while True:
        h1.publishJoints(
            q0,
            names)
        time.sleep(3)

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
