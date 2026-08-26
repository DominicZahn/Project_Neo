import sys, ctypes, os, json
import numpy as np
import numpy.typing as npt
from pathlib import Path
import casadi as c
import subprocess
import argparse
from rich import print

from acados_template import AcadosOcpSolver

from ext_pkgs.dodge_it_py.dodge_it_py.neo.ocp_def import OCP
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2, HeadlessData
from ext_pkgs.dodge_it_py.dodge_it_py.ocpDebugger import OcpDebugger
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile
from ext_pkgs.dodge_it_py.dodge_it_py.sample import SemiSphere, SemiEllipsoid

# --------------------------------- GLOBAL SETTINGS ---------------------------------
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
]

Tf = 2.5
N = 120
# ------------------------------- HELPER FUNCS --------------------------------------
@staticmethod
def extractVarsFromSolver(ocp : OCP) -> tuple[npt.NDArray, npt.NDArray, npt.NDArray]:
    """return x, u, t"""
    x = ocp.getSimX()
    u = ocp.getSimU(floatBase=True)
    t = ocp.solver.get_flat('p')
    t = t.reshape((N+1,7))[:,-1]
    return (x, u, t)

def generateVideoFromFrames(headlessData : HeadlessData):
        subprocess.run(["ffmpeg",
                    "-i", f"{headlessData.dir}/frames/%05d.png",
                    "-framerate", str(N/Tf),
                    "-pix_fmt", "rgb8",
                    f"{headlessData.dir}/video.gif"])

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

def mainInteractive(showCollision : bool,
                    initalValues : bool,
                    visualize : str) -> int:

    xINITAL_VALUES_FILE = '/home/robot/ws/initalValues_x.txt'
    uINITAL_VALUES_FILE = '/home/robot/ws/initalValues_u.txt'

    if visualize == "interactive": vis = True
    elif visualize == "headless" : vis = HeadlessData("/home/robot/ws/_tmp/",
                                                      np.array([-0.05, -0.1, 0.7]),
                                                      np.array([0.0, 0.0, -1.0]),
                                                      (1080,1920))
    else: vis = False

    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=DYNAMIC_JOINT_NAMES,
        showCollisionSDF=showCollision,
        visualization=vis)
    h1.setCollision(
        projectile.linear(h1.t,
                          c.SX([0.9 ,0 , 1.6]),
                          c.SX([-0.6, 0., 0.])))
    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq), 0.0)
    if initalValues:
        ocp = OCP(h1, Tf, N, "", "")
    else:
        ocp = OCP(h1, Tf, N, xINITAL_VALUES_FILE, uINITAL_VALUES_FILE)
    status = ocp.solve(plot=True)

    while True:
        print(h1._vis.viewer.url())
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
                generateVideoFromFrames(visualize)
    return status

def mainBenchmark(sampleCount : int):
    # center on head
#    shape = SemiSphere(
#        (0.0, -0.2, 1.4),
#        0.7,
#        -np.pi/6,   # 30°
#        np.pi/6     # 30°
#    )
    s = 2.0     # scale factor to move ellipsoid away
    hule = SemiEllipsoid(
        (0.05, 0.0, 1.25),
        (0.2*s, 0.35*s, 0.5*s),
        -0.1, 0.4 
    )
    projectilePosArr, normalArr = hule.sample(sampleCount)
    projectileVelArr = -normalArr * 0.5

    for i in range(sampleCount):
        dirPath = f"/home/robot/ws/benchmarks/{str(i).zfill(5)}/"
        headlessData = HeadlessData(dirPath,
                                np.array([-0.05, -0.1, 0.7]),
                                np.array([0.0, 0.0, -1.0]),
                                (1080,1920))
        
        h1 = H1Wrapper_v2(
            q0='knees_bend_0.4_straight',
            dynamicJoints=DYNAMIC_JOINT_NAMES,
            showCollisionSDF=True,
            visualization=False)

        try:
            projectilePos = c.SX(projectilePosArr[i])
            projectileVel = c.SX(projectileVelArr[i])
            # projectilePos = c.SX([0.9 , 0, 1.6])
            # projectileVel = c.SX([-0.6, 0, 0])
        
            h1.setCollision(
                projectile.linear(h1.t,
                                  projectilePos,
                                  projectileVel))
    
            ocp = OCP(h1, Tf, N)
            status = ocp.solve(plot=True)
            if not Path(dirPath).exists():
                Path(dirPath).mkdir()
            
            saveReport(ocp.solver, projectilePos, projectileVel, f"{dirPath}solver.json")
            
            if status == 0:
                ocp.saveAsInitalValues(f"{dirPath}x.txt", f"{dirPath}u.txt")
                print("[INFO] rendering frames")
                x, tau, t = extractVarsFromSolver(ocp)
                assert(h1.model.nq)
                nq =  h1.model.nq
                q = x[:,:nq]
                qdot = x[:,nq:]
    
                # h1.visualizeJointTrajecotry(q, qdot, tau, t, 1.0)
                # generateVideoFromFrames(headlessData)

            del ocp
            
            color = "bold green" if status == 0 else "bold orange1"
            print(f"[{color}][INFO][/{color}] run {i} complete")
        finally:
            # h1.closeHeadless()
            del h1

            libgomp = ctypes.CDLL("libgomp.so.1")
            libgomp.omp_set_num_threads(os.cpu_count())
            assert(libgomp.omp_get_max_threads() == os.cpu_count())

def main():
    parser = argparse.ArgumentParser()
    interactiveGroup = parser.add_argument_group("interactive mode")
    interactiveGroup.add_argument("--showCollision",
                                  action="store_true",
                                  required=False)
    interactiveGroup.add_argument("--initalValues",
                                  action="store_true",
                                  required=False)
    interactiveGroup.add_argument("--visualization",
                                  type=str,
                                  choices=("interactive", "headless", "off"),
                                  default="interactive",
                                  required=False)
    benchmarkGroup = parser.add_argument_group("benchmark mode")
    benchmarkGroup.add_argument("--benchmarkMode",
                                action="store_true",
                                required=False)
    benchmarkGroup.add_argument("--samples", type=int, required=False, default=64)
    args = parser.parse_args()

    if not args.benchmarkMode:
        sys.exit(
            mainInteractive(
                args.showCollision,
                args.initalValues,
                args.visualization))
    else:
        sys.exit(mainBenchmark(args.samples))