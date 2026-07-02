import casadi as c
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from pathlib import Path

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

xFILE = '/home/robot/ws/neo_x.txt'
uFILE = '/home/robot/ws/neo_u.txt'

class OCP:
    def __init__(self, h1 : H1Wrapper_v2, Tf : float, N : int):
        self.h1 = h1
        self.Tf = Tf
        self.N = N

        self.ocp = AcadosOcp()
        self.ocp.model = self._model()
        self._stability()
        self.ocp.cost = self._cost()
        self.ocp.constraints = self._constraints()

        self.solver = self._solver()
        # self.solver = self._initalValues(0.0)

        assert(self.h1.model.nq)
        x_itConst = np.concatenate([
            self.h1.q0,
            np.zeros(self.h1.model.nq)
        ])
        x = np.tile(x_itConst, self.N+1)
        self.solver.set_flat('x', x)
    
    def _model(self) -> AcadosModel:
        model = AcadosModel()
        model.name = "h1_2"
        model.x = c.vertcat(self.h1.q, self.h1.qdot)
        model.u = self.h1.tau[6:]
        
        # set torques for floating base to 0Nm
        model.p = self.h1.tau[:6]
        self.ocp.parameter_values = np.zeros(6)

        model.f_expl_expr = c.vertcat(self.h1.qdot, self.h1.qddot)

        return model
    
    def _stability(self) -> None:
        PoS = PolygonOfSupport()
        self.stabilityConstraint = PoS.stability_centerDistParable(self.h1.ZMP)
       
    def _cost(self) -> AcadosOcpCost:
        cost = AcadosOcpCost()

        # Lagrange
        cost.cost_type = 'NONLINEAR_LS'
        cost.cost_discretization = 'INTEGRATOR' # GNRKA

        #       stability
#        self.ocp.model.cost_y_expr = self.stabilityConstraint
#        cost.yref = 0.0
#        cost.W = np.eye(1)

        #       effort
#        self.ocp.model.cost_y_expr = self.ocp.model.u
#        nu = self.ocp.model.u.size1()
#        cost.yref = np.zeros(nu)
#        cost.W = np.eye(nu)*10**-12


        # Mayer
        cost.cost_type_e = 'NONLINEAR_LS'
        cost.cost_discretization_e = 'INTEGRATOR' # GNRK

        #       head
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
        q_ub[:6] = np.full(6, 2)
        q_lb[:6] = np.full(6, -2)
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
        rhip_pitch_i = self.h1.qId('right_hip_pitch_joint')
        lhip_pitch_i = self.h1.qId('left_hip_pitch_joint')
        # rhip_yaw_i = self.h1.qId('right_hip_yaw_joint')
        # lhip_yaw_i = self.h1.qId('left_hip_yaw_joint')
        rknee_i = self.h1.qId('right_knee_joint')
        lknee_i = self.h1.qId('left_knee_joint')
        rankle_pitch_i = self.h1.qId('right_ankle_pitch_joint')
        lankle_pitch_i = self.h1.qId('left_ankle_pitch_joint')
        # rankle_roll_i = self.h1.qId('right_ankle_roll_joint')
        # lankle_roll_i = self.h1.qId('left_ankle_roll_joint')
        max_tau = np.zeros(nq)
        max_tau[rhip_pitch_i] = max_tau_hip
        max_tau[lhip_pitch_i] = max_tau_hip
        # max_tau[rhip_yaw_i] = max_tau_hip
        # max_tau[lhip_yaw_i] = max_tau_hip
        max_tau[rknee_i] = max_tau_knee
        max_tau[lknee_i] = max_tau_knee
        max_tau[rankle_pitch_i] = max_tau_ankle_pitch
        max_tau[lankle_pitch_i] = max_tau_ankle_pitch
        # max_tau[rankle_roll_i] = max_tau_ankle_pitch
        # max_tau[lankle_roll_i] = max_tau_ankle_pitch

        max_tau = max_tau[6:]

        cons.idxbu = np.arange(max_tau.size)
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
            # cons.lh = np.append(cons.lh, 0)
            cons.lh = np.append(cons.lh, -epsFoot)

        #           stability constraint
        useablePoS = 0.8
        cons.uh = np.append(cons.uh, useablePoS)
        cons.lh = np.append(cons.lh, -0.1)
        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            self.stabilityConstraint
        )
 

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
            cons.uh, np.full(4, 10**12) # emulate unconstrainted
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
        # options.hessian_approx = 'EXACT'
        options.globalization_line_search_use_sufficient_descent = 1
        options.globalization = 'FUNNEL_L1PEN_LINESEARCH'
        options.hpipm_mode = 'ROBUST'
        options.qp_solver_iter_max = 10**2
        options.tol = float(10**-3)
        options.nlp_solver_tol_eq = float(10**-2)
        options.qp_solver_tol_ineq = float(10**-5)

        self.ocp.solver_options = options
        self.ocp.solver_options.store_iterates = True
        solver = AcadosOcpSolver(
            self.ocp,
            generate=False,
            build=False
        )
        return solver
    
    def _initalValues(self, rngPertubation : float = 0.0):
        assert(self.h1.model.nq)
        if not Path(xFILE).exists() or not Path(uFILE).exists():
            return self.solver

        xInit = np.loadtxt('/home/robot/ws/neo_x.txt')
        xInit = xInit.flatten()
        uInit = np.loadtxt('/home/robot/ws/neo_u.txt')
        uInit = uInit.flatten()

        # pertubation
        if rngPertubation > 0.0:
            xInit += np.random.rand(xInit.size) * rngPertubation
            uInit += np.random.rand(uInit.size) * rngPertubation

        self.solver.set_flat('x', xInit)
        self.solver.set_flat('u', uInit)

        return self.solver

    def solve(self, plot=False) -> int:
        self.status = self.solver.solve()
        self.solver.print_statistics()
        print('Total Cost:', self.solver.get_cost())

        if plot:
            plot_trajectories(
                x_traj_list=[self.getSimX()],
                u_traj_list=[self.getSimU(floatBase=False)],
                time_traj_list=[self.getSimT()],
                labels_list=['OCP result'],
                fig_filename='/home/robot/ws/neo_ocp_fig.png',
                show_plot=False
            )

        return self.status

    def getSimX(self) -> npt.NDArray:
        assert(type(self.ocp.model.x) is c.SX)
        return np.reshape(
            self.solver.get_flat('x'),
            (-1, self.ocp.model.x.size1()))
    
    def getSimU(self, floatBase : bool) -> npt.NDArray:
        assert(type(self.ocp.model.u) is c.SX)
        
        uJoints = np.reshape(
            self.solver.get_flat('u'),
            (-1, self.ocp.model.u.size1()))
        
        if not floatBase:
            return uJoints
        
        uBase = self.solver.get_flat('p')
        uBase = np.reshape(uBase, (-1, 6))
        return np.concatenate([uBase[:-1], uJoints], axis=1)

    
    def getSimT(self) -> npt.NDArray:
        return np.linspace(0, self.Tf, self.N + 1)
    
    def save(self):
        print('[INFO] Loaded inital values from files.')
        x = self.solver.get_flat('x')
        u = self.solver.get_flat('u')
        np.savetxt(xFILE, x)
        np.savetxt(uFILE, u)
