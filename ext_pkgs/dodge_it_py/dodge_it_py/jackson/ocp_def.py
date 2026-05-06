import numpy as np
import casadi as c
import matplotlib.pyplot as plt
import sys

from ext_pkgs.dodge_it_py.dodge_it_py.h1wrapper import H1Wrapper

from acados_template import (
    plot_trajectories,
    AcadosOcp,
    AcadosModel,
    AcadosOcpCost,
    AcadosOcpConstraints,
    AcadosOcpSolver,
)


class OCP:
    def __init__(self, h1: H1Wrapper, q0: dict[str, float] | None, Tf: float = 1.0, N: int = 33):
        self.h1 = h1
        self.Tf = Tf
        self.N = N
        if q0:
            self.q0_map = q0
        else:
            self.q0_map = {}

        nq = self.h1.get_nq(reduced=True)
        q0_vec = np.zeros((nq))
        for name, value in self.q0_map.items():
            id = self.h1.getJointId(name) - 1  # no universe
            print(id, name, value)
            q0_vec[id] = value
        self._q0 = q0_vec

        self._variable_def()

    def _variable_def(self):
        self.x = c.vertcat(
            self.h1.get_q(),
            self.h1.get_qdot())
        self.xdot = c.vertcat(
            self.h1.get_qdot(),
            self.h1.get_qddot())
        self.u = self.h1.get_qddot()
        self.t = c.SX.sym('t',1)

    def _cost(self, ac_model: AcadosModel) -> AcadosOcpCost:
        ocp_cost = AcadosOcpCost()
        ocp_cost.cost_type = "NONLINEAR_LS"

        # stability
        self.cost_stability = self.h1._PoS.stability_centerDistParable(self.h1._ZMP)

        # head
        amplitude = 0.5
        f_head_h = c.Function('f_head_h',[self.h1.get_q(True)], [self.h1.get_head_pos()])
        head_pos0 = f_head_h(self._q0)
        if head_pos0 is None:
            print('ERROR: Head pos is broken in _cost(...)')
            sys.exit(-1)
        t = ac_model.p
        if type(t) is not c.SX:
            print('ERROR: time t is broken in _cost(...)')
            sys.exit(-1)
        dz = -(amplitude / self.Tf) * t
        desired_head_pos = c.vertcat(
            head_pos0[0],
            head_pos0[1],
            head_pos0[2] + dz
        )
        head_diff = self.h1.get_head_pos() - desired_head_pos
        self.cost_head = head_diff[2]**2

        # regularisation
        self.cost_reg = c.dot(self.u,self.u)

        # combinded
        self.w_cost_stability = 1
        self.w_cost_head = 0.5
        self.w_reg = 10**-2

        ac_model.cost_y_expr = self.w_cost_stability * self.cost_stability + self.w_cost_head * self.cost_head + self.w_reg * self.cost_reg
        ocp_cost.yref = 0.0
        ocp_cost.W = np.eye(1)

        return ocp_cost

    def _constraints(self, ac_model: AcadosModel) -> AcadosOcpConstraints:
        nq = self.h1.get_nq(reduced=True)
        ocp_cons = AcadosOcpConstraints()

        # initial
        ocp_cons.x0 = np.hstack((self._q0, np.zeros(nq)))

        # path
        #      joint limit
        qub = self.h1.get_upperPosLimit()
        qlb = self.h1.get_lowerPosLimit()
        max_velocity = 100 # [rad/s]
        max_acceleration = 100 # [rad/s²]
        max_tau_knee = 360 # [Nm]
        max_tau_hip = 220 # [Nm]
        max_tau_waist = 220 # [Nm]
        max_tau_ankle_pitch = 130 # [Nm] (26mm / 30mm)*2*75Nm

        ocp_cons.idxbx = np.arange(2 * nq)
        ocp_cons.ubx = np.hstack((qub, np.full(nq, max_velocity)))
        ocp_cons.lbx = np.hstack((qlb, np.full(nq, -max_velocity)))
        ocp_cons.idxbu = np.arange(nq)
        ocp_cons.ubu = np.full(nq, max_acceleration)
        ocp_cons.lbu = np.full(nq, -max_acceleration)
        
        hip_id = self.h1.getJointId('left_hip_pitch_joint')-1
        knee_id = self.h1.getJointId('left_knee_joint')-1
        ankle_id = self.h1.getJointId('left_ankle_pitch_joint')-1
        max_tau = np.zeros(nq)
        max_tau[hip_id] = max_tau_hip
        max_tau[knee_id] = max_tau_knee
        max_tau[ankle_id] = max_tau_ankle_pitch
        
        #       stability constraint and torque constraints
        ac_model.con_h_expr = c.vertcat(
            self.h1._ZMP[2],  # y-coord
            self.h1.get_tau(),
            )
        ocp_cons.uh = np.hstack((
            self.h1._PoS.yu,
            max_tau
            ))
        ocp_cons.lh = np.hstack((
            self.h1._PoS.yl,
            -max_tau
            ))

        # terminal
        #       limit velocity to end
        ocp_cons.idxbx_e = np.arange(nq,2*nq)
        ocp_cons.ubx_e = np.full(nq, 0.001)
        ocp_cons.lbx_e = np.full(nq, -0.001)

        return ocp_cons
    
    def set_time_var(self, solver : AcadosOcpSolver) -> AcadosOcpSolver:
        # set time parameter
        time_step = self.Tf / self.N
        time_arr = np.arange(self.N+1)*time_step
        solver.set_flat('p', time_arr)
        return solver
    
    def set_inital_guess(self, solver : AcadosOcpSolver, N : int) -> AcadosOcpSolver:
        # interpolate hip
        nq = self.h1.get_nq(reduced=True)
        x_arr = np.zeros((2*nq, N+1))
        x_arr[:,:nq] = self._q0
        hip_id = self.h1.getJointId('left_hip_pitch_joint')-1
        w = np.arange(0, 1, 1/(N+1))
        x_arr[hip_id,:] = self._q0[hip_id]*w+1.55*(1-w)
        solver.set_flat('x', x_arr.flatten())

        return solver


    def solve(self, plot=False) -> AcadosOcpSolver:
        ocp = AcadosOcp()
        ac_model = AcadosModel()
        ac_model.name = "h1_2"
        ac_model.x = self.x
        ac_model.u = self.u
        ac_model.f_expl_expr = self.xdot
        ac_model.p = self.t

        ocp.cost = self._cost(ac_model)
        ocp.constraints = self._constraints(ac_model)
        ocp.parameter_values = np.array([0])

        ocp.model = ac_model
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.tf = self.Tf
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.print_level = 1  # full verbosity
        ocp.solver_options.nlp_solver_max_iter = 1000
        ocp.solver_options.nlp_solver_tol_stat = float(10**-3)
        # ocp.solver_options.tol = float(10**-3)
        ocp.solver_options.qp_solver_iter_max = 100

        solver = AcadosOcpSolver(ocp)
        solver = self.set_time_var(solver)
        # solver = self.set_inital_guess(solver, self.N)
        solver.solve()
        solver.print_statistics()
        qp_dict = solver.qp_diagnostics('FULL_HESSIAN')
        qp_prt_keys = ["max_eigv_global", "min_eigv_global", "condition_number_global"]
        for k in qp_prt_keys:
            print(k+": ", str(qp_dict[k]))

        if plot:
            simX = np.zeros((self.N + 1, self.x.size1()))
            simU = np.zeros((self.N, self.u.size1()))
            for i in range(self.N+1):
                simX[i, :] = solver.get(i, "x")
                if i < self.N:
                    simU[i, :] = solver.get(i, "u")

            plot_trajectories(
                x_traj_list=[simX],
                u_traj_list=[simU],
                time_traj_list=[np.linspace(0, self.Tf, self.N + 1)],
                labels_list=["OCP result"],
                idxbu=ocp.constraints.idxbu,
                lbu=ocp.constraints.lbu,
                ubu=ocp.constraints.ubu,
                X_ref=None,
                U_ref=None,
                x_min=None,
                x_max=None,
                single_column=True,
                x_labels=["$q_0$","$q_1$","$q_2$","$\dot q_0$","$\dot q_1$","$\dot q_2$"], # type: ignore
                u_labels=["$\ddot q_0$","$\ddot q_1$","$\ddot q_2$"], # type: ignore
                show_legend=False,
                show_plot=False, # plot but stay interactie
            )
            plt.ion()
            plt.pause(1)

        return solver
