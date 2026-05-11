import numpy as np
import casadi as c
import pinocchio.casadi as cpin
from acados_template import (
    AcadosOcpSolver
)

import matplotlib.pyplot as plt

import time

from ext_pkgs.dodge_it_py.dodge_it_py.h1wrapper import *
from ext_pkgs.dodge_it_py.dodge_it_py.john.ocp_def import OCP

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
    h1 = H1Wrapper(inverseDynamics=False)
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
    q0[h1.getJointId('left_ankle_pitch_joint',True)] = -0.226 # x0
    q0[h1.getJointId('left_knee_joint',True)] = 0.4 # x1
    q0[h1.getJointId('left_hip_pitch_joint',True)] = -0.15 # x2

    # --------------- OCP -----------------
    Tf = 1.0
    N = 100
    nq = h1.get_nq(reduced=True)
    ocp = OCP(h1, dict(zip(names_reduced, q0)), Tf, N)
    solver = ocp.solve(True)
    if solver.status != 0:
        rclpy.shutdown()
        print('Total Cost:', solver.get_cost())
        return -1

    # --------------- vis -----------------
    f_cons_stability = c.Function('f_cons_stability', [h1.get_q(True),h1.get_qdot(True), h1.get_tau(True)], [ocp.cons_stability])
    f_cost_head = c.Function('f_cost_head', [h1.get_q(True), ocp.t], [ocp.cost_head])
    f_cost_reg = c.Function('f_cost_reg', [h1.get_tau(True)], [ocp.cost_reg])

    f_aba_qddot = c.Function('f_aba_addot', [h1.get_q(True), h1.get_qdot(True), h1.get_tau(True)], [h1.get_qddot(True)])

    plt.ion()
    ax_cost = plt.subplot(211)
    ax_cost.set_title('Cost Graph (with weights)')
    ax_cost.set_xlabel('t [s]')
    ax_cost.set_ylabel('Cost')
    line_cost_head, = ax_cost.plot([], [], label='head')
    line_cost_reg, = ax_cost.plot([], [], label='regularisation')
    ax_cost.legend()
    
    ax_cons = plt.subplot(212, sharex=ax_cost)
    ax_cons.set_title('Constraints')
    ax_cons.set_xlabel('t [s]')
    ax_cons.set_ylabel('unitless constraint')
    line_cons_stability, = ax_cons.plot([], [], label='stability')
    ax_cons.legend()

    while rclpy.ok():
        # h1.init_gazebo(dict(zip(h1.jointNames(reduced=False)[1:], h1.reduced2mirrored(q0))))
        t_data = []
        cons_stability_data = []
        cost_head_data = []
        cost_reg_data = []
        for n in range(N):
            qi = solver.get(n, 'x')[:nq]
            qdoti = solver.get(n, 'x')[nq:]
            taui = solver.get(n, 'u')
            qddoti = c.DM(f_aba_qddot(c.SX(qi), c.SX(qdoti), c.SX(taui))).toarray()
            # lam = solver.get(n, 'lam')
            # print(lam)
            t = solver.get(n, 'p')
            h1.visualize(
                h1.reduced2mirrored(qi).tolist(),
                h1.jointNames(reduced=False)[1:],
                h1.reduced2mirrored(qdoti).tolist(),
                h1.reduced2mirrored(qddoti).tolist(),
                h1.reduced2mirrored(taui).tolist())

            # visualize in plot
            t_data.append(float(t))
            cons_stability = float(f_cons_stability(c.SX(qi), c.SX(qdoti), c.SX(taui))) # type: ignore
            cons_stability_data.append(cons_stability)
            line_cons_stability.set_data(t_data, cons_stability_data)
            ax_cons.relim()
            ax_cons.autoscale_view()

            cost_head = float(f_cost_head(c.SX(qi), c.SX(t))) * ocp.w_cost_head # type: ignore
            cost_head_data.append(cost_head)
            cost_reg = float(f_cost_reg(c.SX(taui))) * ocp.w_cost_reg # type: ignore
            cost_reg_data.append(cost_reg)
            line_cost_head.set_data(t_data, cost_head_data)
            line_cost_reg.set_data(t_data, cost_reg_data)
            
            ax_cost.relim()
            ax_cost.autoscale_view()

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