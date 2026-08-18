from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
from skimage.measure import marching_cubes
import casadi as c
from pinocchio.visualize import MeshcatVisualizer
from meshcat.geometry import TriangularMeshGeometry, MeshLambertMaterial

def cRotMatFromEuler(xyz : c.SX) -> c.SX:
    x, y, z = xyz[0], xyz[1], xyz[2]

    Rx = c.vertcat(c.horzcat(1, 0, 0),
                   c.horzcat(0, c.cos(x), -c.sin(x)),
                   c.horzcat(0, c.sin(x),  c.cos(x)))

    Ry = c.vertcat(c.horzcat( c.cos(y), 0, c.sin(y)),
                   c.horzcat(0, 1, 0),
                   c.horzcat(-c.sin(y), 0, c.cos(y)))

    Rz = c.vertcat(c.horzcat(c.cos(z), -c.sin(z), 0),
                   c.horzcat(c.sin(z),  c.cos(z), 0),
                   c.horzcat(0, 0, 1))
    return Rx @ Ry @ Rz

def cHomogenousMat(R : c.SX, t : c.SX) -> c.SX:
    mat = c.vertcat(
        c.horzcat(R, t),
        c.horzcat(c.SX.zeros(1, 3), c.SX.ones(1, 1)))
    return c.SX(mat)

class CollisionSDF:
    def __init__(self, radius : npt.NDArray, q0 : npt.NDArray):
        self.vis = None
        self.radius = radius
        self.q = q0
        self._threadCount = 12

        p_sym = c.SX.sym("p", 3)
        q_sym = c.SX.sym("q", q0.size)
        
        self._dFunc = c.Function("dFunc",
                                 [p_sym, q_sym],
                                 [self.cDistanceFunc(p_sym, q_sym)])
        self._mappedFuncCache = {}

        self._resolution = 48
        self._margin = 1.6

    @staticmethod
    def _smoothMin(d0 : c.SX, d1 : c.SX, k : c.SX) -> c.SX:
        """
        Adapted from Inigo Quilez (https://iquilezles.org/articles/smin/)
        """
        k *= c.log10(2.0)
        x = d1 - d0
        return d0 + x / (1.0 - 2**(x/k))

    def cDistanceFunc(self, p : c.SX, q : c.SX) -> c.SX:
        R = cRotMatFromEuler(q[3:6])
        worldRobotTrans = cHomogenousMat(R.T, -R.T @ q[:3])
        p_robot = worldRobotTrans @ c.vertcat(p, c.SX(1.0))
        d = CollisionSDF._cEllipsoid(p_robot[:3], c.SX(self.radius))
        return d

    def enableVis(self, vis : MeshcatVisualizer, resolution : int = 48, margin : float = 1.6):
        """
        resolution: grid points per axis for the marching-cubes sampling of
                    cDistanceFunc. Total evaluations = resolution**3.
        margin:     half-width of the sampled cube, as a multiple of
                    max(radius). Must be large enough to fully contain the
                    zero level set (the SDF surface), including margin for
                    orientation, or the isosurface extraction finds nothing.
        """
        self.vis = vis
        self._resolution = resolution
        self._margin = margin
        self._displayRedraw()

    def disableVis(self):
        self.vis = None

    def updateJoints(self, q : npt.NDArray):
        self.q = q
        if self.vis:
            self._displayRedraw()

    def _sdfGrid(self) -> tuple[npt.NDArray, float, npt.NDArray]:
        """
        Evaluate cDistanceFunc on a regular grid around the
        robot.
        @return values reshaped to (R,R,R), voxel spacing, grid origin
        """
        half = float(np.max(self.radius) * self._margin)
        center = self.q[:3]
        axis = np.linspace(-half, half, self._resolution)
        gx, gy, gz = np.meshgrid(axis, axis, axis, indexing="ij")
        pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=0) + center[:, None]  # (3, N)

        N = pts.shape[1]
        dFuncMap = self._mappedFuncCache.get(N)
        if dFuncMap is None:
            dFuncMap = self._dFunc.map(N, "thread", self._threadCount)
            self._mappedFuncCache[N] = dFuncMap

        qRep = np.tile(self.q.reshape(-1, 1), (1, N))
        d = np.array(dFuncMap(pts, qRep)).reshape(self._resolution,
                                                  self._resolution,
                                                  self._resolution)
        spacing = axis[1] - axis[0]
        origin = center - half
        return d, spacing, origin

    def _displayRedraw(self):
        assert self.vis is not None
        grid, spacing, origin = self._sdfGrid()

        if grid.min() > 0 or grid.max() < 0:
            self.vis.viewer["collision"].delete()
            return

        verts, faces, _, _ = marching_cubes(grid, level=0.0,
                                             spacing=(spacing, spacing, spacing))
        verts += origin

        self.vis.viewer["collision"].set_object(
            TriangularMeshGeometry(verts, faces),
            MeshLambertMaterial(color=0x4287f5, opacity=0.55, transparent=True)
        )
    
    @staticmethod
    def _cEllipsoid(p : c.SX, r : c.SX) -> c.SX:
        """
        Adapted from Inigo Quilez (https://iquilezles.org/articles/distfunctions/)
        """
        k0 = c.norm_2(p/r)
        k1 = c.norm_2(p/(r*r))
        return k0*(k0-1.0)/k1