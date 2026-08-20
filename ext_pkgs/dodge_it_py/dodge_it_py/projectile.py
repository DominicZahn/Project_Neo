import casadi as c

@staticmethod
def linear(t : c.SX,
           p : c.SX,
           v : c.SX) -> c.Function:
    return c.Function("objPosFunc", [t], [p+v*t])