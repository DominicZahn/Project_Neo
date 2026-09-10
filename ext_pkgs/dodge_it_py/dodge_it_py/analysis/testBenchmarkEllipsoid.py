import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from ext_pkgs.dodge_it_py.dodge_it_py.sample import SemiEllipsoid, SemiSphere
from ext_pkgs.dodge_it_py.dodge_it_py.analysis.solverOverview import XLIM, YLIM, ZLIM, XTICKS, YTICKS, ZTICKS, RATIO, FIGSIZE

VERTS_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/verts.txt"
FACES_FILE = "/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/analysis/faces.txt"

if __name__ == "__main__":
    Tf = 2.5    # simulation time frame
    # c = (0.00, 0.0, 1.25)
    # r = (0.1, 0.25, 0.5)
    c = (0.00, 0.0, 1.25)
    r = (0.1, 0.25, 0.5)
    d = 1.0     # start to robot distance
    N = int(sys.argv[1])
    assert(0 < N)
    shape = SemiEllipsoid(
        c,
        r,
        # -0.1, 0.4 
        0.05, 0.45
    )
    #       robot ellipsoid hull
    points, normals = shape.sampleFibonacciThomson(N, 0)
    # points, normals = shape.sampleFibonacci(N)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(*points.transpose(), s=4)

    #       object starting postions
#    pC = points - np.array(c)
#    a0 = np.linalg.norm(normals, axis=1)**2
#    a1 = 2*(normals[:,0]*pC[:,0] + normals[:,1]*pC[:,1] + normals[:,2]*pC[:,2])
#    a2 = np.linalg.norm(pC, axis=1)**2 - d**2
#    disc = a1**2-4*a0*a2
#    assert((disc > 0).all())
#    scale0 = (-a1+np.sqrt(disc))/(2*a0)
#    scale = scale0
#    startPoints = points + normals * scale[:,None]
    startPoints = points + normals * d

    startPointsCent = startPoints - np.array(c)
    rc = np.array(r) + d
    v = 2*(np.sum(startPointsCent**2/rc**4, axis=1))**(3/2) / (np.sum(startPointsCent**2/rc**6, axis=1)) / Tf
    normals *= -v[:,None] * Tf
   
    ax.scatter(*startPoints.transpose(), s=4, c="red")
    ax.quiver(startPoints[:, 0], startPoints[:, 1], startPoints[:, 2],
              normals[:, 0], normals[:, 1], normals[:, 2],
              arrow_length_ratio=0.0, normalize=False, color="gray", alpha=0.1)

    verts = np.loadtxt(VERTS_FILE)
    faces = np.loadtxt(FACES_FILE).astype(int)
    mesh = Poly3DCollection(verts[faces], alpha=0.25)
    ax.add_collection3d(mesh)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect(RATIO)
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_zlim(*ZLIM)
    ax.set_xticks(XTICKS)
    ax.set_yticks(YTICKS)
    ax.set_zticks(ZTICKS)

    out = "/home/robot/ws/test"
    # iso view
    isoFile = f"{out}_iso.pdf"
    fig.tight_layout(pad=0)
    plt.savefig(isoFile, pad_inches=0.0)
    print(f"Plot saved to {isoFile}")
    # top view
    topFile = f"{out}_top.pdf"
    ax.view_init(elev=90, azim=0, roll=0)
    fig.tight_layout(pad=0)
    ax.set_zticks([])
    plt.savefig(topFile, pad_inches=0.0)
    ax.set_zticks(ZTICKS)
    print(f"Plot saved to {topFile}")
    # front view
    frontFile = f"{out}_front.pdf"
    ax.view_init(elev=0, azim=0, roll=0)
    fig.tight_layout(pad=0)
    ax.set_xticks([])
    plt.savefig(frontFile, pad_inches=0.0)
    ax.set_xticks(XTICKS)
    print(f"Plot saved to {frontFile}")
    # side view
    sideFile = f"{out}_side.pdf"
    ax.view_init(elev=0, azim=90, roll=0)
    ax.set_yticks([])
    fig.tight_layout(pad=0)
    plt.savefig(sideFile, pad_inches=0.0)
    ax.set_yticks(YTICKS)
    print(f"Plot saved to {sideFile}")