import casadi as c
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt

from ext_pkgs.dodge_it_py.dodge_it_py.neo.H1Wrapper_v2 import H1Wrapper_v2
from ext_pkgs.dodge_it_py.dodge_it_py.stability import PolygonOfSupport, zmp_centroidal

from acados_template import (
    plot_trajectories,
    AcadosOcp,
    AcadosModel,
    AcadosOcpCost,
    AcadosOcpConstraints,
    AcadosOcpSolver,
    AcadosOcpOptions
)

class OCP:
    def __init__(self, h1 : H1Wrapper_v2, Tf : float, N : int):
        self.h1 = h1
        self.Tf = Tf
        self.N = N

        self.ocp = AcadosOcp()
        self.ocp.model = self._model()
        self.ocp.cost = self._cost()
        self.ocp.constraints = self._constraints()

        self.solver = self._solver()
        # self.solver = self._initalValues()
    
    def _model(self) -> AcadosModel:
        model = AcadosModel()
        model.name = "h1_2"
        model.x = c.vertcat(self.h1.q, self.h1.qdot)
        model.u = self.h1.tau
        model.f_expl_expr = c.vertcat(self.h1.qdot, self.h1.qddot)
        return model

    def _cost(self) -> AcadosOcpCost:
        cost = AcadosOcpCost()
        cost.cost_type_e = 'NONLINEAR_LS'
        cost.cost_discretization_e = 'INTEGRATOR' # GNRK

        # head
        f_headPos = c.Function('f_headPos', [self.h1.q], [self.h1.headPos])
        headPosDesired = f_headPos(self.h1.q0)
        assert(type(headPosDesired) is c.DM)
        headPosDesired = headPosDesired.toarray() - np.array([[0],[0],[0.4]])
        self.ocp.model.cost_y_expr_e = f_headPos(self.h1.q)
        cost.yref_e = headPosDesired
        cost.W_e = np.eye(3)

        return cost
    
    def _constraints(self) -> AcadosOcpConstraints:
        nq = self.h1.model.nq
        assert(nq is not None)
        cons = AcadosOcpConstraints()

        # inital
        cons.x0 = np.hstack((self.h1.q0, np.zeros(nq)))

        # path
        #       state limits
        q_ub = self.h1.model.upperPositionLimit
        q_lb = self.h1.model.lowerPositionLimit
        assert(q_ub is not None and q_lb is not None)
        q_ub[:6] = np.full(6, 10)
        q_lb[:6] = np.full(6, -10)
        # qdot_ub = self.h1.model.upperVelocityLimit
        # qdot_lb = self.h1.model.lowerVelocityLimit
        qdot_ub = np.full(nq, 2)
        qdot_lb = np.full(nq, -2)
        assert(qdot_ub is not None and qdot_lb is not None)
        cons.ubx = np.hstack((q_ub, qdot_ub))
        cons.lbx = np.hstack((q_lb, qdot_lb))
        cons.idxbx = np.arange(2*nq)

        #       control limits
        max_tau_knee = 360 # [Nm]
        max_tau_hip = 220 # [Nm]
        max_tau_waist = 220 # [Nm]
        max_tau_ankle_pitch = 130 # [Nm] (26mm / 30mm)*2*75Nm
        hip_pitch_i = self.h1.qId('left_hip_pitch_joint')
        # hip_roll_i = self.h1.qId('left_hip_roll_joint')
        knee_i = self.h1.qId('left_knee_joint')
        ankle_i = self.h1.qId('left_ankle_pitch_joint')
        max_tau = np.zeros(nq)
        max_tau[hip_pitch_i] = max_tau_hip
        # max_tau[hip_roll_i] = max_tau_hip
        max_tau[knee_i] = max_tau_knee
        max_tau[ankle_i] = max_tau_ankle_pitch

        cons.idxbu = np.arange(nq)
        cons.ubu = max_tau
        cons.lbu = -max_tau

        #       nonlinear constraints (h)
        self.ocp.model.con_h_expr = c.SX()
        cons.uh = np.array([])
        cons.lh = np.array([])
        #           remove foot drifting
        epsFoot = 0.01
        for feetFrame in self.h1.feetFrames:
            id = self.h1.model.getFrameId(feetFrame)
            func = c.Function(
                "f_"+feetFrame,
                [self.h1.q],
                [self.h1.cdata.oMf[id].translation])
            feetFramePos0 = c.SX(func(self.h1.q0))
            d = func(self.h1.q) - feetFramePos0
            self.ocp.model.con_h_expr = c.vertcat(
                self.ocp.model.con_h_expr,
                c.dot(d,d)
            )
            cons.uh = np.append(cons.uh, epsFoot)
            cons.lh = np.append(cons.lh, -epsFoot)

        #           stability constraint
        useablePoS = 0.8
        PoS = PolygonOfSupport()
        stabilityConstraint = PoS.stability_centerDistParable(self.h1.ZMP)
        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            stabilityConstraint
        )
        cons.uh = np.append(cons.uh, useablePoS)
        cons.lh = np.append(cons.lh, -0.1)

        #           planar friction
        fricCoeff = 0.5
        FLz = self.h1.cdata.lambda_c[2]
        FLx_cons = (FLz * fricCoeff)**2 - self.h1.cdata.lambda_c[0]**2
        FLy_cons = (FLz * fricCoeff)**2 - self.h1.cdata.lambda_c[1]**2
        FRz = self.h1.cdata.lambda_c[8]
        FRx_cons = (FRz * fricCoeff)**2 - self.h1.cdata.lambda_c[6]**2
        FRy_cons = (FRz * fricCoeff)**2 - self.h1.cdata.lambda_c[7]**2

        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            FLx_cons,
            FLy_cons,
            FRx_cons,
            FRy_cons
        )
        cons.uh = np.append(
            cons.uh, np.full(4, 10**6) # emulate unconstrainted
        )
        cons.lh = np.append(
            cons.lh, np.zeros(4)
        )

        # terminal
        #       limit end velocity
        cons.idxbx_e = np.arange(nq, nq+6)
        cons.ubx_e = np.full(6, 0.1)
        cons.lbx_e = np.full(6, -0.1)

        return cons

    def _solver(self) -> AcadosOcpSolver:
        options = AcadosOcpOptions()
        options.integrator_type = 'ERK'
        options.N_horizon = self.N
        options.tf = self.Tf
        options.print_level = 3
        options.nlp_solver_max_iter = 1000
        # options.nlp_solver_type = 'SQP_WITH_FEASIBLE_QP'
        options.qp_solver_iter_max = 100
        options.tol = float(10**-3)
        options.qp_solver_tol_ineq = float(10**-6)
        
        self.ocp.solver_options = options
        solver = AcadosOcpSolver(
            self.ocp,
            generate=False,
            build=False 
        )
        return solver
    
    def _initalValues(self):
        # controls u
#        u_itConst = np.array([
#            0.3, -0.1, 328, 0.1, -10, -0.05,
#            -0.3, 0.1, 328, 0.2, -10, -0.05
#        ])
#        u = np.tile(u_itConst, self.N)
#        self.solver.set_flat('u', u)

        # states x
        assert(self.h1.model.nq)
        x_itConst = np.concatenate([
            self.h1.q0,
            np.zeros(self.h1.model.nq)
        ])
        x = np.tile(x_itConst, self.N+1)
        self.solver.set_flat('x', x)

        return self.solver

    def solve(self, plot=False) -> int:
        self.status = self.solver.solve()
        self.solver.print_statistics()
        print('Total Cost:', self.solver.get_cost())

        if plot:
            plot_trajectories(
                x_traj_list=[self.getSimX()],
                u_traj_list=[self.getSimU()],
                time_traj_list=[self.getSimT()],
                labels_list=['OCP result'],
                fig_filename='/home/robot/ws/neo_ocp_fig.png'
            )

        return self.status

    def getSimX(self) -> npt.NDArray:
        assert(type(self.ocp.model.x) is c.SX)
        return np.reshape(
            self.solver.get_flat('x'),
            (-1, self.ocp.model.x.size1()))
    
    def getSimU(self) -> npt.NDArray:
        assert(type(self.ocp.model.u) is c.SX)
        return np.reshape(
            self.solver.get_flat('u'),
            (-1, self.ocp.model.u.size1()))
    
    def getSimT(self) -> npt.NDArray:
        return np.linspace(0, self.Tf, self.N + 1)
