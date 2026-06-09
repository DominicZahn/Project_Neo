import casadi as c
import numpy as np

class PolygonOfSupport():
    """
    01---11        
    |     |
    |     |
    00---10
    """
    def __init__(self) -> None:
        self.xl = 0.0339
        self.xu = -0.363
        self.yl = -0.07 # -0.0875
        self.yu = 0.175 
        self._center = c.SX([
            (self.yu+self.yl)/2,
            (self.xu+self.xl)/2,
            0.0
        ])
    
    def get_corners(self) -> list[c.SX]:
        """
        00, 10, 11, 01
        """
        return [
            c.SX([self.xl,self.yl,0.0]),
            c.SX([self.xu,self.yl,0.0]),
            c.SX([self.xu,self.yu,0.0]),
            c.SX([self.xl,self.yu,0.0])
        ]
    
    def get_center(self) -> c.SX:
        return self._center

    def stability_centerDistParable(self, p : c.SX) -> c.SX:
        yp = p[0]
        yc = self._center[0]
        return (yp-yc)**2/(self.yl-yc)**2
    
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
        dy0 = (yp-self.yl)/(yc-self.yl)-1
        dy1 = (yp-self.yu)/(yc-self.yu)-1
        return c.fmax(dy0, dy1)

