import casadi as c
import numpy as np

import pinocchio.casadi as cpin

class PolygonOfSupport():
    """
    01--y--11        
    |       |
    x       x
    |       |
    00--y--10
    """
    def __init__(self) -> None:
        # self.yl = 0.0339
        # self.yu = -0.363
        self.xl = -0.07 # -0.0875
        self.xu = 0.175 
        self.yl = -0.20
        self.yu = 0.20
        self._center = c.SX([
            (self.xu+self.xl)/2,
            (self.yu+self.yl)/2,
            0.0
        ])
    
    def get_corners(self) -> list[c.SX]:
        """
        00, 10, 11, 01
        """
        return [
            c.SX([self.xl,self.yl,0.0]),
            c.SX([self.xl,self.yu,0.0]),
            c.SX([self.xu,self.yu,0.0]),
            c.SX([self.xu,self.yl,0.0])
        ]
    
    def get_center(self) -> c.SX:
        return self._center

    def stability_centerDistParable(self, p : c.SX) -> c.SX:
        xp, yp = p[0], p[1]
        xc, yc = self._center[0], self._center[1]
        xSize, ySize = self.xl-xc, self.yl-yc
        xVal = (xp-xc)**2/xSize**2
        yVal = (yp-yc)**2/ySize**2
        return xVal + yVal
    
    def stability_centerDist(self, p : c.SX) -> c.SX:
        d = self._center - p
        return c.dot(d,d)
    
    def stability_minEdgeY(self, p : c.SX) -> c.SX:
        xp = p[1]
        yp = p[0]
        xc = self._center[1]
        yc = self._center[0]
        # dx0 = (xp-self._x0)/(xc-self._x0)
        # dx1 = (xp-self._x1)/(xc-self._x1)
        dy0 = (yp-self.xl)/(yc-self.xl)-1
        dy1 = (yp-self.xu)/(yc-self.xu)-1
        return c.fmax(dy0, dy1)

# --------------------------------------------

def zmp_centroidal(
        cmodel : cpin.Model,
        cdata : cpin.Data,
        q : c.SX,
        qdot : c.SX,
        qddot : c.SX) -> c.SX:
    zmp_z = 0.0
    g = cmodel.gravity.linear[2]
    M = cpin.computeTotalMass(cmodel,cdata)
    CoM = cpin.centerOfMass(
        cmodel,
        cdata,
        q,
        qdot,
        qddot
    )
    dPdL = cpin.computeCentroidalMomentumTimeVariation(
        cmodel,
        cdata,
        q,
        qdot,
        qddot
    )
    dP = dPdL.linear
    dL = dPdL.angular

    ZMP = c.SX([0,0,0])
    zmp_num = M*g+dP[2]
    ZMP[0] = (M*g*CoM[0]+zmp_z*dP[0]-dL[1]) / zmp_num
    ZMP[1] = (M*g*CoM[1]+zmp_z*dP[2]+dL[0]) / zmp_num
    ZMP[2] = zmp_z
    return ZMP

def zmp_full(
        cmodel : cpin.Model,
        cdata : cpin.Data,
        q : c.SX,
        qdot : c.SX,
        qddot : c.SX,
        contactFrameIds : list[int]) -> c.SX:
    x_denum, x_num = c.SX(0.0), c.SX(0.0)
    y_denum, y_num = c.SX(0.0), c.SX(0.0)
    pz = 0.0
    x_num, y_num, denum = 0.0, 0.0, 0.0
    for i in range(len(contactFrameIds)):
        j = 6*i
        F = cdata.lambda_c[j:j+3]
        tau = cdata.lambda_c[j+3:j+6]
        pFoot = cdata.oMf[contactFrameIds[i]].translation
        x_num += -tau[1]-(pFoot[2]-pz)*F[0]+pFoot[0]*F[2]
        y_num += tau[0]-(pFoot[2]-pz)*F[1]+pFoot[1]*F[2]
        denum += F[2]
    px = x_num / denum
    py = y_num / denum
    zmp = c.SX([0,0,0])
    zmp[0], zmp[1], zmp[2] = px, py, pz
    return zmp

def zmp_approx(cmodel : cpin.Model, cdata : cpin.Data) -> c.SX:
    # ignoring angular accelerations
    g = 9.181
    zmp_z = 0.0
    zmp_x_den = 0.0
    zmp_y_den = 0.0
    zmp_num = 0.0
    cpin.updateFramePlacements(cmodel, cdata)
    for id in range(1,len(cmodel.frames)):
        frame = cmodel.frames[id]
        m = frame.inertia.mass
        a = cpin.getFrameAcceleration(
            cmodel,
            cdata,
            id
        ).linear
        com = cdata.oMf[id].translation
        zmp_x_den += m*((a[2]+g)*com[0]-(com[2]-zmp_z)*a[0])
        zmp_num += m*(a[2]+g)
        zmp_y_den += m*((a[2]+g)*com[1]-(com[2]-zmp_z)*a[1])

    ZMP = c.SX([0,0,0])
    ZMP[0] = zmp_x_den / zmp_num
    ZMP[1] = zmp_y_den / zmp_num
    ZMP[2] = zmp_z
    return ZMP
