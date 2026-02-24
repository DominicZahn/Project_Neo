import numpy as np
import casadi as c
import pinocchio.casadi as cpin
from acados_template import (
    AcadosOcpSolver
)

import time

from pkgs.dodge_it_py.dodge_it_py.h1wrapper import *
from pkgs.dodge_it_py.dodge_it_py.bob.ocp_def import OCP

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
        'right_ankle_pitch_joint'
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
    # q0[h1.getJointId('left_hip_pitch_joint')-1] = -1.5
    # q0[h1.getJointId('left_knee_joint')-1] = 1
    # q0[h1.getJointId('left_ankle_pitch_joint')-1] = -0.2
    q0[h1.getJointId('left_hip_pitch_joint')-1] = -0.2
    q0[h1.getJointId('left_knee_joint')-1] = 0.4
    q0[h1.getJointId('left_ankle_pitch_joint')-1] = -0.2



    # --------------- OCP -----------------
    Tf = 30.0
    N = 33*int(Tf)
    nq = h1.get_nq(reduced=True)
    ocp = OCP(h1, dict(zip(names_reduced, q0)), Tf, N)
    solver = ocp.solve(True)

    # --------------- vis -----------------
    for n in range(N):
        qi = solver.get(n, 'x')[:nq]
        head_height_f = c.Function('head_height_f', [h1.get_q(True)], [h1.get_head_pos()])
        head_height = head_height_f(qi)[2] # type: ignore
        lam = solver.get(n, 'lam')
        t = solver.get(n, 'p')
        h1.visualizeJoints(
            h1.reduced2mirrored(qi).tolist(),
            h1.jointNames(reduced=False)[1:])
        print(t, head_height, lam)
        time.sleep(Tf / N)
        # breakpoint()
    print('Cost:', solver.get_cost())

    breakpoint()

    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    main()
