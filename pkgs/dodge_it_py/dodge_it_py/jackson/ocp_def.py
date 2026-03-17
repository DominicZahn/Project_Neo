import numpy as np
import casadi as c
import matplotlib.pyplot as plt
import sys

from pkgs.dodge_it_py.dodge_it_py.h1wrapper import H1Wrapper

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
        self.cost_reg = c.dot(self.h1.get_tau(),self.h1.get_tau())

        # combinded
        self.w_cost_stability = 10**4
        self.w_cost_head = 10**2
        self.w_reg = 10**-5

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
        max_velocity = 0.05 # [m/s]
        max_acceleration = 5 # [m/s²]
        # max_velocity = 0.3 # [m/s]
        # max_acceleration = 30 # [m/s²]
        # max_velocity = 1e9
        # max_acceleration = 1e9
        max_tau = 220 # [Nm] (motor max)

        ocp_cons.idxbx = np.arange(2 * nq)
        ocp_cons.ubx = np.hstack((qub, np.full(nq, max_velocity)))
        ocp_cons.lbx = np.hstack((qlb, np.full(nq, -max_velocity)))
        ocp_cons.idxbu = np.arange(nq)
        ocp_cons.ubu = np.full(nq, max_acceleration)
        ocp_cons.lbu = np.full(nq, -max_acceleration)

        #       stability constraint
        ac_model.con_h_expr = c.vertcat(
            self.h1._ZMP[0],  # y-coord
            self.h1.get_tau(),
            )
        ocp_cons.uh = np.hstack((
            self.h1._PoS.yu,
            np.full(nq, max_tau),            
            ))
        ocp_cons.lh = np.hstack((
            self.h1._PoS.yl,
            np.full(nq, -max_tau),
            ))
        
        # terminal
        #       limit velocity to end
        # ocp_cons.idxbx_e = np.arange(nq,2*nq)
        # ocp_cons.ubx_e = np.full(nq, 0.01)
        # ocp_cons.lbx_e = np.full(nq, -0.01)

        return ocp_cons

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
        # ocp.solver_options.nlp_solver_max_iter = 1000

        solver = AcadosOcpSolver(ocp)

        # set time parameter
        time_step = self.Tf / self.N
        time_arr = np.arange(self.N+1)*time_step
        solver.set_flat('p', time_arr)

        solver.solve()
        solver.print_statistics()

        if plot:
            simX = np.zeros((self.N + 1, self.x.size1()))
            simU = np.zeros((self.N, self.u.size1()))
            for i in range(self.N):
                simX[i, :] = solver.get(i, "x")
                simU[i, :] = solver.get(i, "u")
            simX[self.N, :] = solver.get(self.N, "x")

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
                show_plot=False, # plot but stay interactie
            )
            plt.ion()
            plt.pause(1)

        return solver
