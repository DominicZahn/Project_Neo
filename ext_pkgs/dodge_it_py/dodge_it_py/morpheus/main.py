import sys, ctypes, os, json
import numpy as np
import numpy.typing as npt
from pathlib import Path
import casadi as c
import argparse
from rich import print

from acados_template import AcadosOcpSolver

from ext_pkgs.dodge_it_py.dodge_it_py.morpheus.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2, HeadlessData, generateVideoFromFrames
from ext_pkgs.dodge_it_py.dodge_it_py.ocpDebugger import OcpDebugger
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile # just for visuals

# --------------------------------- GLOBAL SETTINGS ---------------------------------
xINITAL_VALUES_FILE = '/home/robot/ws/initalValues_x.txt'
uINITAL_VALUES_FILE = '/home/robot/ws/initalValues_u.txt'

DYNAMIC_JOINT_NAMES = [
    # 'left_hip_yaw_joint',
    # 'right_hip_yaw_joint',
    # 'left_hip_roll_joint',
    # 'right_hip_roll_joint',
    'left_hip_pitch_joint',
    'right_hip_pitch_joint',
    'left_knee_joint',
    'right_knee_joint',
    'left_ankle_pitch_joint',
    'right_ankle_pitch_joint',
    # 'left_ankle_roll_joint',
    # 'right_ankle_roll_joint',
    'torso_joint',
    # 'left_shoulder_roll_joint',
    # 'left_shoulder_pitch_joint',
    # 'left_shoulder_yaw_joint',
    # 'left_elbow_joint',
    # 'right_shoulder_roll_joint',
    # 'right_shoulder_pitch_joint',
    # 'right_shoulder_yaw_joint',
    # 'right_elbow_joint',
]

Tf = 2.0
N = 60
# ------------------------------- HELPER FUNCS --------------------------------------
@staticmethod
def extractVarsFromSolver(ocp : OCP) -> tuple[npt.NDArray, npt.NDArray, npt.NDArray]:
    """return x, u, t"""
    x = ocp.getSimX()
    u = ocp.getSimU(floatBase=True)
    t = ocp.solver.get_flat('p')
    t = t.reshape((N+1,7))[:,-1]
    return (x, u, t)

def saveReport(solver : AcadosOcpSolver,
               projectilePos : c.SX,
               projectileVel : c.SX,
               file : str) -> None:
    status = solver.get_status()
    nlpIter = solver.get_stats("nlp_iter")
    cost = solver.get_cost()
    res_stat, res_eq, ress_ineq, res_comp = solver.get_residuals()

    path = Path(file)

    obj = {
        "status": status,
        "nlp_iter": nlpIter,
        "cost": cost,
        "res_stat": res_stat,
        "res_eq": res_eq,
        "res_ineq": ress_ineq,
        "res_comp": res_comp,
        "projectile_position" : c.DM(projectilePos).toarray().tolist(),
        "projectile_velocity" : c.DM(projectileVel).toarray().tolist()
    }
    with open(path, mode="w+", encoding="utf-8") as f:
        json.dump(obj, f)
    f.close()

# -----------------------------------------------------------------------------------

def mainInteractive(h_diff : float,
                    initalValues : bool,
                    visualize : str) -> int:

    if visualize == "interactive": vis = True
    elif visualize == "headless" : vis = HeadlessData("/home/robot/ws/_tmp/",
                                                      np.array([-0.05, -0.1, 0.7]),
                                                      np.array([0.0, 0.0, -1.0]),
                                                      (1080,1920))
    else: vis = False

    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=DYNAMIC_JOINT_NAMES,
        showCollisionSDF=False,
        visualization=vis)
    h1.setCollision(
        projectile.linear(h1.t,
                          c.SX([1.6 ,0 , 1.2]),
                          c.SX([-0.5, 0., 0.])))

    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq), 0.0)
    if initalValues:
        ocp = OCP(h1, Tf, N, h_diff, xINITAL_VALUES_FILE, uINITAL_VALUES_FILE)
    else:
        ocp = OCP(h1, Tf, N, h_diff, "", "")
    status = ocp.solve(plot=True)

    while True:
        print(h1.model)

        x, tau, t = extractVarsFromSolver(ocp)
        q = x[:,:nq]
        qdot = x[:,nq:]

        if status != 0:
            print("------------------- ❌ NO CONVERGENCE -------------------")
        else:
            print("------------------- ✅ CONVERGENCE -------------------")

        key = input("Press RETURN for visalization\n q to exit\n s to save controls and states\n c to save joints in csv\n d to start ocp debugger\n")
        if key == 'q':
            break
        elif key == 's':
            ocp.saveAsInitalValues(xINITAL_VALUES_FILE, uINITAL_VALUES_FILE)
        elif key == 'c':
            h1.saveJointTrajectory(
                q,
                Path("jointTrajectory.csv")
            )
        elif key == 'd':
            debugger = OcpDebugger(ocp.solver)
            debugger.run()
        else:
            h1.visualizeJointTrajecotry(q, qdot, tau, t, 1.0)
            if visualize is type(HeadlessData):
                generateVideoFromFrames(visualize.dir, N, Tf)
    return status

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--initalValues",
                        action="store_true",
                        required=False)
    parser.add_argument("--visualization",
                        type=str,
                        choices=("interactive", "headless", "off"),
                        default="interactive",
                        required=False)
    parser.add_argument("--diff",
                        type=float,
                        default=0.4,
                        required=False)
    args = parser.parse_args()

    sys.exit(mainInteractive(args.diff, args.initalValues, args.visualization))