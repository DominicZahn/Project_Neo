#!/usr/bin/env python3
"""Build data.csv for jointplot.sty from the optimal-control output files.

  u.txt : controls (joint torques in Nm), one value per line, N samples x NU values
  x.txt : states, one value per line, (N+1) samples x NX values
          NX = 2*NQ: first half positions (NBASE floating-base coords + NU joint
          angles), second half velocities.

Output columns: k, q0..q{NU-1}, tau0..tau{NU-1}
  * the NBASE base coordinates are skipped, only the actuated joint angles are kept
  * there is one torque fewer than states, so the last torque is repeated once to
    give every row a value (the plot does not draw that duplicate point).

Usage:  python3 make_data.py u.txt x.txt [-o data.csv] [--nu 7] [--nx 26]
"""

import argparse
import numpy as np


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("u", help="controls file (torques)")
    p.add_argument("x", help="states file (angles, then velocities)")
    p.add_argument("-o", "--out", default="data.csv")
    p.add_argument("--nu", type=int, default=7, help="number of actuated joints")
    p.add_argument(
        "--nx", type=int, default=26, help="state dimension (2 * #positions)"
    )
    a = p.parse_args()

    u = np.loadtxt(a.u)
    x = np.loadtxt(a.x)
    nq = a.nx // 2
    nbase = nq - a.nu
    if a.nx % 2 or nbase < 0:
        p.error(f"--nx={a.nx} is inconsistent with --nu={a.nu}")
    if u.size % a.nu or x.size % a.nx:
        p.error(
            f"file sizes ({u.size}, {x.size}) are not multiples of nu={a.nu}, nx={a.nx}"
        )

    u = u.reshape(-1, a.nu)
    x = x.reshape(-1, a.nx)
    if x.shape[0] != u.shape[0] + 1:
        p.error(
            f"expected N+1 states for N controls, got {x.shape[0]} and {u.shape[0]}"
        )

    q = x[:, nbase:nq]  # joint angles only
    tau = np.vstack([u, u[-1:]])  # repeat last torque
    k = np.arange(x.shape[0])

    header = ",".join(
        ["k"] + [f"q{i}" for i in range(a.nu)] + [f"tau{i}" for i in range(a.nu)]
    )
    np.savetxt(
        a.out,
        np.column_stack([k, q, tau]),
        delimiter=",",
        header=header,
        comments="",
        fmt="%.8g",
    )
    print(f"wrote {a.out}: {x.shape[0]} rows ({u.shape[0]} intervals), {a.nu} joints")


if __name__ == "__main__":
    main()
