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

    # ---------------- H1 ---------------
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
    mirrors_res &= h1.mirrorJoints('left_knee_joint', 'right_knee_joint')
    mirrors_res &= h1.mirrorJoints('left_ankle_pitch_joint', 'right_ankle_pitch_joint')
    
    if not mirrors_res:
        return -1
    names_reduced = h1.jointNames(reduced=True)[1:] # no universe
    q0 = np.zeros(len(names_reduced))

    # --------------- OCP -----------------
    Tf = 1.0
    N = 33
    nq = len(h1.jointNames())
    ocp = OCP(h1, dict(zip(names_reduced, q0)))
    solver = ocp.solve(Tf, N)

    # --------------- vis -----------------
    for n in range(N):
        qi = solver.get(n, 'x')[:nq]
        h1.publishJoints(
            h1.reduced2Mirrored(qi),
            h1.jointNames())
        print(qi)
        time.sleep(Tf / N)

    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    main()
