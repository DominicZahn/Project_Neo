import numpy as np

g = 9.81

class WalkPatternGenerators:
    def __init__(self,
                 CoM0 : np.ndarray,
                 T_sup : float,
                 px0 : float,
                 py0 : float,
                 sx : np.ndarray,
                 sy : np.ndarray):
        assert(len(sx) == len(sy))
        self._step_count = len(sx)
        self._CoM0 = CoM0
        self._T_sup = T_sup
        self.px = np.zeros(self._step_count)
        self.px_mod = px0
        self.py = np.zeros(self._step_count)
        self.py_mod = py0
        self.sx = sx
        self.sy = sy
    def __del__(self):
        pass

    def calc_pattern(self):
        T_C = 0
        S = 0
        C = 0
        z_c = self._CoM0[2]

        T = 0
        n = 0
        px = np.zeros(self._step_count)
        py = np.zeros(self._step_count)
        x =  np.zeros(self._step_count)
        y = np.zeros(self._step_count)
        vx = np.zeros(self._step_count)
        vy = np.zeros(self._step_count)
        # xd = np.zeros(self._step_count)
        # yd = np.zeros(self._step_count)
        # dxd = np.zeros(self._step_count)
        # dyd = np.zeros(self._step_count)
        while n < self._step_count:
            # Step 3: (eq.4)
            ddx = g/z_c * (x-self.px_mod)
            # Step 4:
            T += self._T_sup
            n += 1
            # Step 5: (eq.1)
            px[n] = px[n-1] + self.sx[n]
            py[n] = py[n-1] - (-1)**n*self.sy[n]
            # Step 6: (eq.2, eq.3)
            x[n] = self.sx[n+1]/2
            y[n] = (-1)**n*self.sy[n+1]/2
            vx[n] = (C+1) / (T_C*S) * x[n]
            vy[n] = (C-1) / (T_C*S) * y[n]
            # Step 7: (eq.5)
            xd = px[n] + x[n]
            dxd = vx[n]
            yd = py[n] + y[n]
            dyd = vy[n]
            # Step 8: (eq.6)

