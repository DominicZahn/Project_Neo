import argparse, sys
import casadi as c
from pathlib import Path
import numpy as np
import numpy.typing as npt
from rich import print

from ext_pkgs.dodge_it_py.dodge_it_py.collisionSDF import CollisionSDF
from ext_pkgs.dodge_it_py.dodge_it_py.H1Wrapper_v2 import H1Wrapper_v2
from ext_pkgs.dodge_it_py.dodge_it_py.neo.main import DYNAMIC_JOINT_NAMES, Tf, N
import ext_pkgs.dodge_it_py.dodge_it_py.projectile as projectile

def main() -> int:
    h1 = H1Wrapper_v2(
        q0='knees_bend_0.4_straight',
        dynamicJoints=DYNAMIC_JOINT_NAMES,
        visualization=True,
        showCollisionSDF=True,
    )
    h1._vis.viewer["/Lights/SpotLight"].set_property("visible", True)
    h1._vis.viewer["/meshcat/zmp"].set_property("visible", False)
    h1._vis.viewer["/Axes"].set_property("visible", False)
    pos = np.array([-1.0, -1.0, -3.0])
    lookAt = np.array([0.0, 0.0, 0.7])
    h1._moveCamera(pos, lookAt)
    assert(h1.model.nq)
    nq =  h1.model.nq
    h1.setCollision(
        projectile.linear(h1.t,
                      c.SX([0, 0, 100]),
                      c.SX([0, 0, 0])))
    h1.visualizeJointConfig(h1.q0, np.zeros(nq), np.zeros(nq), 0.0)

    input("WAIT FOR RETURN")

    return 0

if __name__ == "__main__":

    sys.exit(main())