import casadi as c
import numpy as np
import numpy.typing as npt
from pathlib import Path

from ext_pkgs.dodge_it_py.dodge_it_py.neo.H1Wrapper_v2 import H1Wrapper_v2
from ext_pkgs.dodge_it_py.dodge_it_py.stability import PolygonOfSupport

from acados_template import (
    plot_trajectories,
    AcadosOcp,
    AcadosModel,
    AcadosOcpCost,
    AcadosOcpConstraints,
    AcadosOcpSolver,
    AcadosOcpOptions,
)

xFILE = '/home/robot/ws/neo_x.txt'
uFILE = '/home/robot/ws/neo_u.txt'

class OCP:
    def __init__(self, h1 : H1Wrapper_v2, Tf : float, N : int, loadInitalValues=False):
        self.h1 = h1
        self.Tf = Tf
        self.N = N

        self.ocp = AcadosOcp()
        self.ocp.model = self._model()
        self._stability()
        self.ocp.cost = self._cost()
        self.ocp.constraints = self._constraints()
        self._codeGenOptions()
        self.solver = self._solver()

        if loadInitalValues:
            self.solver = self.loadInitalValues(xFILE, uFILE)
        else:
            self.solver = self.loadInitalValues("", "")
    
    def _model(self) -> AcadosModel:
        model = AcadosModel()
        model.name = "h1_2"
        model.x = c.vertcat(self.h1.q, self.h1.qdot)
        model.u = self.h1.tau[6:]

        # parameter
        #           set torques for floating base to 0Nm
        model.p = self.h1.tau[:6]
        self.ocp.parameter_values = np.zeros(6)
        #           add timestamp at intervals
        model.p = c.vertcat(model.p, self.h1.t)
        self.ocp.parameter_values = np.hstack((self.ocp.parameter_values, 0.0))

        model.f_expl_expr = c.vertcat(self.h1.qdot, self.h1.qddot)

        return model
    
    def _stability(self) -> None:
        PoS = PolygonOfSupport()
        self.stabilityConstraint = PoS.stability_centerDistParable(self.h1.ZMP)
       
    def _cost(self) -> AcadosOcpCost:
        nq = self.h1.model.nq
        assert(nq is not None)

        cost = AcadosOcpCost()

        # Lagrange
        cost.cost_type = 'NONLINEAR_LS'
        cost.cost_discretization = 'INTEGRATOR' # GNRKA
        x = self.ocp.model.x
        u = self.ocp.model.u
        costTau = c.dot(u,u) * 1e-9
        costStability = self.stabilityConstraint
        self.ocp.model.cost_y_expr = costTau + costStability
        cost.yref = 0.0
        cost.W = np.eye(1) / self.N
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
        cons.ubx = q_ub[6:]
        cons.lbx = q_lb[6:]
        cons.idxbx = np.arange(nq)[6:]

        #       control limits
        max_tau_knee = 360 # [Nm]
        max_tau_hip = 220 # [Nm]
        max_tau_waist = 220 # [Nm]
        max_tau_ankle_pitch = 130 # [Nm] (26mm / 30mm)*2*75Nm
        rhip_pitch_i = self.h1.qId('right_hip_pitch_joint')
        lhip_pitch_i = self.h1.qId('left_hip_pitch_joint')
        # rhip_yaw_i = self.h1.qId('right_hip_yaw_joint')
        # lhip_yaw_i = self.h1.qId('left_hip_yaw_joint')
        # rhip_roll_i = self.h1.qId('right_hip_roll_joint')
        # lhip_roll_i = self.h1.qId('left_hip_roll_joint')
        rknee_i = self.h1.qId('right_knee_joint')
        lknee_i = self.h1.qId('left_knee_joint')
        rankle_pitch_i = self.h1.qId('right_ankle_pitch_joint')
        lankle_pitch_i = self.h1.qId('left_ankle_pitch_joint')
        # rankle_roll_i = self.h1.qId('right_ankle_roll_joint')
        # lankle_roll_i = self.h1.qId('left_ankle_roll_joint')
        # torso_i = self.h1.qId('torso_joint')
        max_tau = np.zeros(nq)
        max_tau[rhip_pitch_i] = max_tau_hip
        max_tau[lhip_pitch_i] = max_tau_hip
        # max_tau[rhip_yaw_i] = max_tau_hip
        # max_tau[lhip_yaw_i] = max_tau_hip
        # max_tau[rhip_roll_i] = max_tau_hip
        # max_tau[lhip_roll_i] = max_tau_hip
        max_tau[rknee_i] = max_tau_knee
        max_tau[lknee_i] = max_tau_knee
        max_tau[rankle_pitch_i] = max_tau_ankle_pitch
        max_tau[lankle_pitch_i] = max_tau_ankle_pitch
        # max_tau[rankle_roll_i] = max_tau_ankle_pitch
        # max_tau[lankle_roll_i] = max_tau_ankle_pitch
        # max_tau[torso_i] = max_tau_waist

        max_tau = max_tau[6:]

        cons.idxbu = np.arange(max_tau.size)
        cons.ubu = max_tau
        cons.lbu = -max_tau

        #       nonlinear constraints (h)
        #           stability constraint
        useablePoS = 0.8
        cons.uh = np.append(cons.uh, useablePoS)
        cons.lh = np.append(cons.lh, -0.1)
        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            self.stabilityConstraint
        )

        #           dodge constraint
        d_safe = 0.1
        t = self.h1.t
        d = self.h1.cObjectRobotDistance(t)
        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            d
        )
        cons.uh = np.append(cons.uh, float(1e6)) # emulate unconstrainted
        cons.lh = np.append(cons.lh, d_safe)

        #           planar friction
        fricCoeff = 0.5
        FLz = self.h1.cdata.lambda_c[2]
        FL_cons = (FLz * fricCoeff)**2 - (self.h1.cdata.lambda_c[0]**2 + self.h1.cdata.lambda_c[1]**2)
        FRz = self.h1.cdata.lambda_c[8]
        FR_cons = (FRz * fricCoeff)**2 - (self.h1.cdata.lambda_c[6]**2 + self.h1.cdata.lambda_c[7]**2)

        self.ocp.model.con_h_expr = c.vertcat(
            self.ocp.model.con_h_expr,
            FLz,
            FRz,
            FL_cons,
            FR_cons
        )
        cons.uh = np.append(
            cons.uh, np.full(4, 10**6) # emulate unconstrainted
        )
        cons.lh = np.append(
            cons.lh, np.full(4, 10**1) # safety margin
        )

        # terminal
        #       limit end velocity
        q0_eps = 0.1
        v_stat = 0.1
        qdot_stat = 0.1
        cons.idxbx_e = np.arange(0, 2*nq)
        cons.ubx_e = np.hstack((
            self.h1.q0 + q0_eps,            # nq
            np.full(6, qdot_stat),          # 6
            np.full(nq-6, v_stat)           # np-6
        ))
        cons.lbx_e = np.hstack((
            self.h1.q0 - q0_eps,            # nq
            np.full(6, -v_stat),            # 6
            np.full(nq-6, -qdot_stat)       # np-6
        ))

        # set slacks to 0
        nsh = cons.lh.size
        cons.lsh = np.zeros(nsh)
        cons.ush = np.zeros(nsh)
        cons.idxsh = np.arange(nsh)
        self.ocp.cost.zl = 100 * np.ones((nsh,))
        self.ocp.cost.Zl = 0 * np.ones((nsh,))
        self.ocp.cost.zu = 100 * np.ones((nsh,))
        self.ocp.cost.Zu = 0 * np.ones((nsh,))
        return cons

    def _solver(self) -> AcadosOcpSolver:
        options = AcadosOcpOptions()
        options.integrator_type = 'ERK'
        # options.integrator_type = 'IRK'
        options.N_horizon = self.N
        options.tf = self.Tf
        options.print_level = 2
        options.nlp_solver_max_iter = 1000 # 300
        # options.nlp_solver_type = 'SQP_WITH_FEASIBLE_QP'
        # options.hessian_approx = 'EXACT'
        # options.globalization_line_search_use_sufficient_descent = 1
        # options.globalization = 'MERIT_BACKTRACKING'
        options.globalization = 'FUNNEL_L1PEN_LINESEARCH'
        options.hpipm_mode = 'ROBUST'
        options.qp_solver_iter_max = int(1e3)
        options.tol = float(1e-3)
        options.nlp_solver_tol_ineq = float(1e-3)
        options.nlp_solver_tol_stat = float(1e-3)
        options.sim_method_num_steps = 50

        self.ocp.solver_options = options
        self.ocp.solver_options.store_iterates = True
        solver = AcadosOcpSolver(
            self.ocp,
            generate=False,
            build=False
        )
        # set paramters
        time_arr = np.linspace(0, self.Tf, self.N+1)
        p = np.vstack((
            np.zeros((6,self.N+1)),       # floating base base torque
            np.array([time_arr])))        # time at intervalls
        solver.set_flat('p', p.flatten())
        return solver
    
    def _codeGenOptions(self):
        self.ocp.code_gen_options.ext_fun_compile_flags = "-O0"
    
    def loadInitalValues(self,
                         stateFile : str = "",
                         controlFile : str = "",
                         rngPertubation : float = 0.0):
        assert(self.h1.model.nq)

        nu = self.ocp.model.u.size()[0]
        nx = self.ocp.model.x.size()[0]
        uInit = np.full(nu*(self.N), float(1e-3))
        xInit = np.zeros(((self.N+1), nx))
        xInit[:,:int(nx/2)] = self.h1.q0[None,:]
        xInit = xInit.flatten()

        if len(stateFile)>0 and len(controlFile)>0 and Path(stateFile).exists() and Path(controlFile).exists():
            print('[INFO] Loaded inital values from files.')
            xInit = np.loadtxt(stateFile)
            xInit = xInit.flatten()
            uInit = np.loadtxt(controlFile)
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
        uBase = np.reshape(uBase, (-1, 7))
        return np.concatenate([uBase[:self.N,:6], uJoints], axis=1)

    
    def getSimT(self) -> npt.NDArray:
        return np.linspace(0, self.Tf, self.N + 1)
    
    def save(self):
        print('[INFO] Saved inital values to files.')
        x = self.solver.get_flat('x')
        u = self.solver.get_flat('u')
        np.savetxt(xFILE, x)
        np.savetxt(uFILE, u)