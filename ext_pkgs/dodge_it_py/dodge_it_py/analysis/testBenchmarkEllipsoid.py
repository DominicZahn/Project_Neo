import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from ext_pkgs.dodge_it_py.dodge_it_py.sample import SemiEllipsoid, SemiSphere

VERTS_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/verts.txt"
FACES_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/faces.txt"

def main(args) -> int:
    return 0

if __name__ == "__main__":
    v = 0.5     # simulated velocity
    Tf = 2.5    # simulation time frame
    s = 2.0     # scale factor to move ellipsoid away
    c = (0.05, 0.0, 1.25)
    r = (0.2*s, 0.35*s, 0.5*s)
    shape = SemiEllipsoid(
        c,
        r,
        -0.1, 0.4 
    )
    points, normals = shape.sampleFibonacciThomson(500, 0)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(*points.transpose(), s=4, alpha=0.6)
    ptsCentered = points - np.array(c)
    v = 2*(np.sum(ptsCentered**2/np.array(r)**4, axis=1))**(3/2) / (np.sum(ptsCentered**2/np.array(r)**6, axis=1)) / Tf
    normals *= -v[:,None] * Tf
    ax.quiver(points[:, 0], points[:, 1], points[:, 2],
              normals[:, 0], normals[:, 1], normals[:, 2],
              normalize=False, color="gray", alpha=0.1)

    verts = np.loadtxt(VERTS_FILE)
    faces = np.loadtxt(FACES_FILE).astype(int)
    mesh = Poly3DCollection(verts[faces], alpha=0.25)
    ax.add_collection3d(mesh)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1,1,1])
    ax.set_xlim(-1.0,1.0)
    ax.set_ylim(-1.0,1.0)
    ax.set_zlim(0.0,2.0)
    ax.set_xticks([-1.0,-0.5,0.0,0.5,1.0])
    ax.set_yticks([-1.0,-0.5,0.0,0.5,1.0])
    ax.set_zticks([])

    out = "/home/robot/ws/test"
    # iso view
    isoFile = f"{out}_iso.pdf"
    fig.tight_layout(pad=0)
    plt.savefig(isoFile)
    print(f"Plot saved to {isoFile}")
    # top view
    topFile = f"{out}_top.pdf"
    ax.view_init(elev=90, azim=0, roll=0)
    fig.tight_layout(pad=0)
    plt.savefig(topFile)
    print(f"Plot saved to {topFile}")
    # front view
    frontFile = f"{out}_front.pdf"
    ax.view_init(elev=0, azim=0, roll=0)
    fig.tight_layout(pad=0)
    plt.savefig(frontFile)
    print(f"Plot saved to {frontFile}")
    # side view
    sideFile = f"{out}_side.pdf"
    ax.view_init(elev=0, azim=90, roll=0)
    fig.tight_layout(pad=0)
    plt.savefig(sideFile)
    print(f"Plot saved to {sideFile}")