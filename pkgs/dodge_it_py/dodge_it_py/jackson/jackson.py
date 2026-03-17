import numpy as np
import casadi as c
import pinocchio.casadi as cpin
from acados_template import (
    AcadosOcpSolver
)

import matplotlib.pyplot as plt

import time

from pkgs.dodge_it_py.dodge_it_py.h1wrapper import *
from pkgs.dodge_it_py.dodge_it_py.jackson.ocp_def import OCP

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
    q0[h1.getJointId('left_hip_pitch_joint')-1] = -0.15 # x2
    q0[h1.getJointId('left_knee_joint')-1] = 0.4 # x1
    q0[h1.getJointId('left_ankle_pitch_joint')-1] = -0.226 # x0

    # --------------- OCP -----------------
    Tf = 30
    N = 33*int(Tf)
    nq = h1.get_nq(reduced=True)
    ocp = OCP(h1, dict(zip(names_reduced, q0)), Tf, N)
    solver = ocp.solve(True)

    # --------------- vis -----------------
    f_cost_stability = c.Function('f_cost_stability', [h1.get_q(True),h1.get_qdot(True), h1.get_qddot(True)], [ocp.cost_stability])
    f_cost_head = c.Function('f_cost_head', [h1.get_q(True), ocp.t], [ocp.cost_head])
    # f_cost_reg = c.Function('f_cost_reg', [h1.get_q(True), h1.get_qddot(True)], [ocp.cost_reg])

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title('Cost Graph (without weights)')
    ax.set_xlabel('t')
    ax.set_ylabel('Cost')
    line_stability, = ax.plot([], [], label='stability')
    line_head, = ax.plot([], [], label='head')
    # line_reg, = ax.plot([], [], label='regularisation')
    ax.legend()

    while rclpy.ok():
        h1.init_gazebo(dict(zip(h1.jointNames(reduced=False)[1:], h1.reduced2mirrored(q0))))
        t_data = []
        cost_stability_data = []
        cost_head_data = []
        # cost_reg_data = []
        for n in range(N):
            qi = solver.get(n, 'x')[:nq]
            qdoti = solver.get(n, 'x')[nq:]
            qddoti = solver.get(n, 'u')
            lam = solver.get(n, 'lam')
            t = solver.get(n, 'p')
            h1.visualize(
                h1.reduced2mirrored(qi).tolist(),
                h1.jointNames(reduced=False)[1:],
                h1.reduced2mirrored(qdoti).tolist(),
                h1.reduced2mirrored(qddoti).tolist())
            print(t, lam)

            # visualize cost in plot
            t_data.append(float(t))
            cost_stability = f_cost_stability(c.SX(qi), c.SX(qdoti), c.SX(qddoti))
            cost_stability_data.append(float(cost_stability)) # type: ignore
            cost_head = f_cost_head(c.SX(qi), c.SX(t))
            cost_head_data.append(float(cost_head)) # type: ignore
            # cost_reg = f_cost_reg(c.SX(qi), c.SX(qddoti))
            # cost_reg_data.append(float(cost_reg))

            line_stability.set_data(t_data, cost_stability_data)
            line_head.set_data(t_data, cost_head_data)
            # line_reg.set_data(t_data, cost_reg_data)
            ax.relim()
            ax.autoscale_view()

            if n % 10 == 0:
                plt.pause(Tf / N)
            else:
                time.sleep(Tf / N)
        print('Total Cost:', solver.get_cost())

        input("Press any key to rerun...")

    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    main()