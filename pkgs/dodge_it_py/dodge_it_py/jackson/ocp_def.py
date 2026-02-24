import numpy as np
import casadi as c

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
    def __init__(self, h1: H1Wrapper, q0: dict[str, float] | None):
        self.h1 = h1
        if q0:
            self.q0_map = q0
        else:
            self.q0_map = {}
        self._variable_def()

    def _variable_def(self):
        self.x = c.vertcat(self.h1.get_q(), self.h1.get_qdot())
        self.xdot = c.vertcat(self.h1.get_qdot(), self.h1.get_qddot())
        self.u = self.h1.get_qddot()
        self.p = c.SX.sym('t',1)


    def _cost(self, ac_model: AcadosModel) -> AcadosOcpCost:
        ocp_cost = AcadosOcpCost()
        ocp_cost.cost_type = "NONLINEAR_LS"

        cost_stability = self.h1.stability()
        ac_model.cost_y_expr = 10**2 * cost_stability
        ocp_cost.yref = 0.0
        ocp_cost.W = np.eye(1)

        return ocp_cost

    def _constraints(self, ac_model: AcadosModel) -> AcadosOcpConstraints:
        nq = self.h1.get_nq(reduced=True)
        ocp_cons = AcadosOcpConstraints()

        #   head
        head_pos = self.h1.get_head_pos()
        ac_model.con_h_expr = head_pos[2]

        # initial
        """
        q0_vec = np.zeros((nq))
        for name, value in self.q0_map.items():
            id = self.h1.getJointId(name) - 1  # no universe
            print(id, name, value)
            q0_vec[id] = value

        ocp_cons.x0 = np.hstack((q0_vec, np.zeros(nq)))
        """
        # path
        #      joint limit
        """
        qub = self.h1.get_upperPosLimit()
        qlb = self.h1.get_lowerPosLimit()
        max_velocity = 0.1 # [m/s]
        max_acceleration = 0.1 # [m/s²]

        ocp_cons.idxbx = np.arange(2 * nq)
        ocp_cons.ubx = np.hstack((qub, np.full(nq, max_velocity)))
        ocp_cons.lbx = np.hstack((qlb, np.full(nq, -max_velocity)))
        ocp_cons.idxbu = np.arange(nq)
        ocp_cons.ubu = np.full(nq, max_acceleration)
        ocp_cons.lbu = np.full(nq, -max_acceleration)
        """

        return ocp_cons

    def solve(self, Tf: float = 1.0, N: int = 33, plot=False) -> AcadosOcpSolver:
        ocp = AcadosOcp()
        ac_model = AcadosModel()
        ac_model.name = "h1_2"
        ac_model.x = self.x
        ac_model.u = self.u
        ac_model.f_expl_expr = self.xdot

        ocp.cost = self._cost(ac_model)
        ocp.constraints = self._constraints(ac_model)

        ocp.model = ac_model
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.tf = Tf
        ocp.solver_options.N_horizon = N
        ocp.solver_options.print_level = 1  # full verbosity
        # ocp.solver_options.nlp_solver_max_iter = 1000

        solver = AcadosOcpSolver(ocp)
        breakpoint()
        for i in range(N):
            desired_head_height = 0.2*np.sin(i/(2*np.pi*
                                                N))+1.5
            head_margin = 0.01
            solver.constraints_set(i, 'lh', np.array([desired_head_height-head_margin]))
            solver.constraints_set(i, 'uh', np.array([desired_head_height+head_margin]))
        solver.solve()
        solver.print_statistics()

        if plot:
            simX = np.zeros((N + 1, self.x.size1()))
            simU = np.zeros((N, self.u.size1()))
            for i in range(N):
                simX[i, :] = solver.get(i, "x")
                simU[i, :] = solver.get(i, "u")
            simX[N, :] = solver.get(N, "x")

            plot_trajectories(
                x_traj_list=[simX],
                u_traj_list=[simU],
                time_traj_list=[np.linspace(0, Tf, N + 1)],
                labels_list=["OCP result"],
                idxbu=ocp.constraints.idxbu,
                lbu=ocp.constraints.lbu,
                ubu=ocp.constraints.ubu,
                X_ref=None,
                U_ref=None,
                x_min=None,
                x_max=None,
            )

        return solver
