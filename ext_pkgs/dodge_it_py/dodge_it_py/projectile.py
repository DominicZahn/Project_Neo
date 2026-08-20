import casadi as c

@staticmethod
def linear(t : c.SX,
           p : c.SX = c.SX([0.4 ,0. , 1.5]),
           v : c.SX = c.SX([-0.3, 0., 0.])) -> c.Function:
    return c.Function("objPosFunc", [t], [p+v*t])