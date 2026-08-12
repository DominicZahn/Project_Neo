from concurrent.futures import ThreadPoolExecutor
import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation
import casadi as c
from pinocchio import SE3
from pinocchio.visualize import MeshcatVisualizer
from meshcat.geometry import PointCloud

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
        self._threadCount = 22
        self.pool = ThreadPoolExecutor(self._threadCount)

    def cDistanceFunc(self, p : c.SX, q : c.SX) -> c.SX:
        R = cRotMatFromEuler(q[3:6])
        worldRobotTrans = cHomogenousMat(R.T, -R.T @ q[:3])
        p_robot = worldRobotTrans @ c.vertcat(p, c.SX(1.0))
        d = CollisionSDF._cEllipsoid(p_robot[:3], c.SX(self.radius))
        return d

    def distanceFunc(self, p : npt.NDArray) -> float:
        # transform p to robot cs
        R = Rotation.from_euler("xyz", self.q[3:6], degrees=False)
        R = R.as_matrix().transpose()
        worldRobotTrans = SE3(
            rotation=R,
            translation=-R @ self.q[:3]
        ).homogeneous
        p_robot = worldRobotTrans @ np.hstack([p, [1.0]])

        # calcualte distance in robot cs
        d = CollisionSDF._ellipsoid(p_robot[:3], self.radius)
        return d

    def enableVis(self, vis : MeshcatVisualizer, pointCount : int = int(1e2)):
        self.vis = vis
        self._pointCount = pointCount
        
        # start points
        self.pts = np.random.random((3,self._pointCount))*2
        self.pts[0:2,] -= 1

        self._displayRedraw()

    def disableVis(self):
        self.vis = None

    def updateJoints(self, q : npt.NDArray):
        self.q = q
        if self.vis:
            self._displayRedraw()

    def _singleRay(self, i : int) -> None:
        eps = 1e-10
        d_max = eps / 2
        p = self.pts[:,i]
        # numerical gradient calculation
        grad = np.zeros(3)
        for j in range(3):
            delta = np.zeros(3)
            delta[j] = eps
            d_pos = self.distanceFunc(p + delta)
            d_neg = self.distanceFunc(p - delta)
            grad[j] = (d_pos - d_neg) / (2*eps)
        
        # walk gradient for shortest distance
        d = self.distanceFunc(p)
        self.pts[:,i] -= grad*d
        self._minStep[i] = d < d_max

    def _raymarch(self) -> npt.NDArray:
        max_iter = 200
        
        self._minStep = np.full(self._pointCount, False)
        func = lambda i: self._singleRay(i)
        for _ in range(max_iter):
            self.pool.map(
                func,
                range(self._pointCount),
                chunksize=int(self._pointCount / self._threadCount))
            if np.sum(self._minStep) < self._pointCount * 0.001:
                break
        return self.pts

    def _displayRedraw(self):
        pts = self._raymarch()
        colors = np.repeat(float(0.6), self._pointCount)
        assert(self.vis is not None)
        self.vis.viewer["collision"].set_object(
            PointCloud(position=pts, color=colors, size=0.01)
        )

    @staticmethod
    def _ellipsoid(p : npt.NDArray, r : npt.NDArray) -> float:
        k0 = np.linalg.norm(p/r)
        k1 = np.linalg.norm(p/(r*r))
        assert(k1 != 0)
        return float(k0*(k0-1.0)/k1)
    
    @staticmethod
    def _cEllipsoid(p : c.SX, r : c.SX) -> c.SX:
        k0 = c.norm_2(p/r)
        k1 = c.norm_2(p/(r*r))
        return k0*(k0-1.0)/k1
