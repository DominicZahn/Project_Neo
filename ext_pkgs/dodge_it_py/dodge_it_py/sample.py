import matplotlib.pyplot as plt
import matplotlib.cm as cm
# print(plt.style.available)
# plt.style.use("l3") https://github.com/niess/mplstyle-l3
import sys, json
from pathlib import Path
from rich import print

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from scipy.spatial import cKDTree
from scipy.spatial.distance import pdist
from scipy import special, integrate
from scipy.stats import chi2

import nlopt

GOLDEN_ANGLE = np.pi * (3.0 - np.sqrt(5.0))
SAMPLES_DIR = "/home/robot/ws/samples/"

@staticmethod
def draw(out : str, # path + prefix
         center : npt.NDArray,
         radius : npt.NDArray,
         points : npt.NDArray,
         normals : npt.NDArray | None = None) -> None:
    showNormales = normals is not None
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=4, alpha=0.6)
    if showNormales:
        ax.quiver(points[:, 0], points[:, 1], points[:, 2],
                  normals[:, 0], normals[:, 1], normals[:, 2],
                  length=0.2, normalize=True, color="tab:red", linewidth=0.6, alpha=0.7)
    cx, cy, cz = center
    a, b, c = radius
    extent = max(a, b, c)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([a, b, c])
    ax.set_xlim(cx - extent, cx + extent)
    ax.set_ylim(cy - extent, cy + extent)
    ax.set_zlim(cz - extent, cz + extent)
    plt.tight_layout()

    # iso view
    isoFile = f"{out}_iso.pdf"
    plt.savefig(isoFile)
    print(f"Plot saved to {isoFile}")
    # top view
    topFile = f"{out}_top.pdf"
    ax.view_init(elev=90, azim=0, roll=0)
    plt.savefig(topFile)
    print(f"Plot saved to {topFile}")
    # front view
    frontFile = f"{out}_front.pdf"
    ax.view_init(elev=0, azim=0, roll=0)
    plt.savefig(frontFile)
    print(f"Plot saved to {frontFile}")
    # side view
    sideFile = f"{out}_side.pdf"
    ax.view_init(elev=0, azim=90, roll=0)
    plt.savefig(sideFile)
    print(f"Plot saved to {sideFile}")


@staticmethod
def calcMinDistances(pts : npt.NDArray) -> npt.NDArray:
    tree = cKDTree(pts)
    distances, idxs = tree.query(pts, k=2, workers=-1)
    return distances[:,1] # remove own 0 distance and flatten

@staticmethod
def calcNeighbourStdDistances(pts : npt.NDArray, n : int = 4) -> npt.NDArray:
    tree = cKDTree(pts)
    distances, idxs = tree.query(pts, k=5, workers=-1)
    return np.std(distances[:,1:], axis=1)


@staticmethod
def evaluate(pts : npt.NDArray) -> None:
    # distances = calcMinDistances(pts)
    uniformity = calcNeighbourStdDistances(pts, 4)

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    sc = ax.scatter(*pts.transpose(), c=uniformity, cmap="plasma", s=4, alpha=0.6)
    fig.colorbar(sc, ax=ax)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(0,2)
    plt.tight_layout()
    out_path = "ellipsoid_eval.pdf"
    plt.savefig(out_path)
    print(f"Plot saved to {out_path}")



@dataclass
class SemiSphere:
    """Definition of a semi-sphere (spherical zone).

    Attributes:
        center: (cx, cy, cz) center of the sphere.
        radius: sphere radius (> 0).
        lower_angle_deg:
        upper_angle_deg:
    """

    center : tuple[float,float,float] = (0,0,0)
    radius : float = 1.0
    lowerAltitude : float = -np.pi/2
    upperAltitude : float = np.pi/2

    def __post_init__(self):
        assert(self.radius > 0.0)
        assert(self.lowerAltitude < self.upperAltitude)
        assert(-np.pi/2 <= self.lowerAltitude <= np.pi/2)
        assert(-np.pi/2 <= self.upperAltitude <= np.pi/2)

    def zClamp(self) -> tuple[float, float]:
        """return upper and lower z coordinate cut off"""
        zUpper = self.radius * np.sin(self.upperAltitude)
        zLower = self.radius * np.sin(self.lowerAltitude)
        return zLower, zUpper

    def sample(self, samples : int) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Generate points evenly spread (by surface area) on the spherical zone described by `shape`, using a deterministic Fibonacci-lattice grid.
    
        returns points,normals
        """
        assert(samples > 0)

        cx, cy, cz = self.center
        r = self.radius
        zLower, zUpper = self.zClamp()
    
        i = np.arange(samples)
    
        # Spread z linearly across the zone's range. This is the equivalent,
        # for a restricted zone, of the classic Fibonacci-sphere trick of
        # spacing z linearly from -1 to 1 across the full sphere.
        if samples == 1:
            z = np.array([(zLower + zUpper) / 2.0])
        else:
            z = zLower + i * (zUpper - zLower) / (samples - 1)
    
        # Azimuth advances by the golden angle each step -> even angular
        # spread with no repeating pattern/seam, at any ring size.
        theta = i * GOLDEN_ANGLE
    
        ringRadius = np.sqrt(np.clip(r**2 - z**2, 0.0, None))
        x = ringRadius * np.cos(theta)
        y = ringRadius * np.sin(theta)
    
        points = np.column_stack((x + cx, y + cy, z + cz))
        normals = np.column_stack((x,y,z)) / r
        return points, normals

    def draw(self, samples : int) -> None:
        points, normals = self.sample(samples)
    
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            s=4, alpha=0.6)
        ax.quiver(
            points[:, 0], points[:, 1], points[:, 2],
            normals[:, 0], normals[:, 1], normals[:, 2],
            length=0.2, normalize=True, color="tab:red", linewidth=0.6, alpha=0.7,
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Semi-sphere")
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlim(-1,1)
        ax.set_ylim(-1,1)
        ax.set_zlim(0,2)
        plt.tight_layout()
        out_path = "sphere_samples.png"
        plt.savefig(out_path, dpi=150)
        print(f"Plot saved to {out_path}")

@dataclass
class SemiEllipsoid:
    """Axis-aligned ellipsoidal zone sampled with gradient rejection."""

    center : tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius : tuple[float, float, float] = (1.0, 1.0, 1.0)
    relLowerZ : float = np.nan
    relUpperZ : float = np.nan

    def __post_init__(self):
        assert(len(self.radius) == 3)
        assert(all(v > 0.0 for v in self.radius))
        assert(self.relLowerZ < self.relUpperZ)
        assert(-self.radius[2] <= self.relLowerZ and self.relLowerZ <= self.radius[2])
        assert(-self.radius[2] <= self.relUpperZ and self.relUpperZ <= self.radius[2])

    def sampleLatLong(self, samples : int) -> tuple[npt.NDArray, npt.NDArray]:
        """
        This method only works for square sample counts!
        """
        nLatLong = np.sqrt(samples)
        assert(np.mod(nLatLong,1)==0)
        nLatLong = int(nLatLong)
 
        cx, cy, cz = self.center
        rx, ry, rz = self.radius

        lowerAltitude = np.sin(self.relLowerZ / rz)
        upperAltitude = np.sin(self.relUpperZ / rz)
        lat = np.linspace(lowerAltitude, upperAltitude, nLatLong)
        lon = np.linspace(0.0, 2.0 * np.pi, nLatLong, endpoint=False)
        latGrid, lonGrid = np.meshgrid(lat, lon, indexing="ij")
 
        rho = np.cos(latGrid)
        xLocal = (rx * rho * np.cos(lonGrid)).ravel()
        yLocal = (ry * rho * np.sin(lonGrid)).ravel()
        zLocal = (rz * np.sin(latGrid)).ravel()
 
        points = np.column_stack((xLocal + cx, yLocal + cy, zLocal + cz))
 
        normalDir = np.column_stack((xLocal / rx**2, yLocal / ry**2, zLocal / rz**2))
        normals = normalDir / np.linalg.norm(normalDir, axis=1, keepdims=True)
 
        return points, normals


    def sampleFibonacci(self, samples : int) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Using a Fibonacci-lattice grid which is scaled by the radius properties of the ellipsoid.
        """
        assert(samples > 0)
        r = np.array(self.radius)
        i = np.arange(samples)
    
        # Spread z linearly across the zone's range. This is the equivalent,
        # for a restricted zone, of the classic Fibonacci-sphere trick of
        # spacing z linearly from -1 to 1 across the full sphere.
        zLowerSphere = self.relLowerZ / self.radius[2]
        zUpperSphere = self.relUpperZ / self.radius[2]
        if samples == 1:
            z = np.array([(zLowerSphere + zUpperSphere) / 2.0])
        else:
            z = zLowerSphere + i * (zUpperSphere - zLowerSphere) / (samples - 1)
    
        # Azimuth advances by the golden angle each step -> even angular
        # spread with no repeating pattern/seam, at any ring size.
        theta = i * GOLDEN_ANGLE
    
        ringRadius = np.sqrt(np.clip(1.0 - z**2, 0.0, None))
        x = ringRadius * np.cos(theta)
        y = ringRadius * np.sin(theta)
    
        points = np.column_stack((x, y, z)) * self.radius
        normals = points / r**2
        points += np.array(self.center)
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        return points, normals

    @staticmethod
    def obj(pts, grad):
        pts = pts.reshape(-1, 3)
        n = pts.shape[0]
        diff = pts[:, None, :] - pts[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        np.fill_diagonal(dist, np.inf)
        energy = 0.5 * np.sum(1.0 / dist)
        if grad.size > 0:
            g = -np.sum(diff / dist[:, :, None]**3, axis=1)
            grad[:] = g.flatten()
        print("obj:", energy)
        return energy
    @staticmethod
    def consZUpper(res, pts, grad, zUpper):
        pts = pts.reshape(-1, 3)
        n = pts.shape[0]
        res[:] = pts[:, 2] - zUpper
        if grad.size > 0:
            grad[:] = 0.0
            for k in range(n):
                grad[k, 3*k+2] = 1.0
    @staticmethod
    def consZLower(res, pts, grad, zLower):
        pts = pts.reshape(-1, 3)
        n = pts.shape[0]
        res[:] = zLower - pts[:, 2]
        if grad.size > 0:
            grad[:] = 0.0
            for k in range(n):
                grad[k, 3*k+2] = -1.0
    @staticmethod
    def consEllipsoid(res, pts, grad, center, radius):
        pts = pts.reshape(-1, 3)
        n = pts.shape[0]
        rel = (pts - center) / radius**2
        res[:] = np.sum(rel * (pts - center), axis=1) - 1.0
        if grad.size > 0:
            grad[:] = 0.0
            for k in range(n):
                grad[k, 3*k:3*k+3] = 2.0 * rel[k]

    def sampleFibonacciThomson(self,
                      samples : int,
                      seed : int) -> tuple[npt.NDArray, npt.NDArray]:

        # load buffer file if applicable
        sampleFile = Path(SAMPLES_DIR) / f"{seed}_{samples}.json"
        if sampleFile.exists():
            with open(sampleFile, "r") as f:
                data = json.load(f)
                keysAvailable = "sampleCount" in data.keys() and "seed" in data.keys() and "points" in data.keys() and "normals" in data.keys()
                if keysAvailable:
                    fittingConfig = data["sampleCount"] == samples and data["seed"] == seed
                    if fittingConfig:
                        pts = np.array(data["points"])
                        assert(type(pts) is np.ndarray)
                        normals = np.array(data["normals"])
                        assert(type(normals) is np.ndarray)
                        print("[INFO] loaded buffered samples")
                        return (pts, normals)
         
        ptsInit, _ = self.sampleFibonacci(samples)
        r = np.array(self.radius)
        c = np.array(self.center)

        # optimize distance
        nlopt.srand(seed)
        ptsInit = ptsInit.flatten()
        opt = nlopt.opt(nlopt.LD_SLSQP, ptsInit.size)
        opt.set_exceptions_enabled(False)
        opt.set_xtol_abs(1e-2)
        opt.set_ftol_abs(1e-2)
        opt.set_min_objective(SemiEllipsoid.obj)
        opt.add_inequality_mconstraint(lambda res, x, grad: SemiEllipsoid.consZUpper(res, x, grad, self.relUpperZ+self.center[2]), np.zeros(int(ptsInit.size/3)))
        opt.add_inequality_mconstraint(lambda res, x, grad: SemiEllipsoid.consZLower(res, x, grad, self.relLowerZ+self.center[2]), np.zeros(int(ptsInit.size/3)))
        opt.add_equality_mconstraint(lambda res, x, grad: SemiEllipsoid.consEllipsoid(res, x, grad, c, r), np.full(int(ptsInit.size/3), 1e-6))
        ptsOpt = opt.optimize(ptsInit)
        assert(type(ptsOpt) is np.ndarray)
        res = opt.last_optimize_result()
        if res < 0: 
            if res != nlopt.ROUNDOFF_LIMITED:
                print(f"[bold red][ERROR] samples could not be generated on the ellipsoid with seed {seed}")
                assert(False)
            else:
                print("[bold orange3][WARNING] sample optimization was stopped due to limited round off")
        ptsOpt = ptsOpt.reshape(-1,3)

        # calculate normals to new points
        normals = (ptsOpt-c) / r**2 
        normals /= np.linalg.norm(normals, axis=1)[:,None]

        # write to buffer file
        sampleFile = Path(SAMPLES_DIR) / f"{seed}_{samples}.json"
        with open(sampleFile, "w") as f:
            data = dict({
                "seed" : seed,
                "sampleCount" : samples,
                "points" : ptsOpt.tolist(),
                "normals" : normals.tolist()
            })
            json.dump(data, f)
        return (ptsOpt, normals)

    def sampleGradientRejection(self, samples : int,
                                rng : np.random.Generator,
                                batchOversample : float = 2.0) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Generate points uniformly distributed (by surface area) on this ellipsoid's
        zone using the gradient rejection algorithm of Chen & Glotzer, generalised
        to any ellipsoid by Marples & Williams (2024), Sect. 5.1 / Algorithm 1.

        Trial points are drawn uniformly (by area) on the corresponding zone of the
        unit sphere -- a uniform z coordinate combined with a uniform azimuthal
        angle gives a uniform sample on the sphere surface (Archimedes' hat-box
        theorem) -- then scaled anisotropically onto the ellipsoid (line 4 of
        Algorithm 1). Trial points are accepted with probability
        P(accept) = min(a,b,c) * sqrt(x^2/a^4 + y^2/b^4 + z^2/c^4) (Eq. 44), which
        is the ratio of the surface gradient magnitude to its ellipsoid-wide
        maximum, correcting for the distortion introduced by the anisotropic
        scaling. Sampling repeats in batches until `samples` points are accepted.

        returns points, normals
        """
        assert samples > 0

        a, b, c = self.radius
        r = np.array(self.radius)
        cx, cy, cz = self.center
        gMin = min(a, b, c)  # Eq. 43: gmax = 1 / min(a,b,c), so P(accept) = gMin * g(x,y,z)

        zLo = self.relLowerZ / c
        zHi = self.relUpperZ / c

        acceptedLocal = np.empty((0, 3))
        while acceptedLocal.shape[0] < samples:
            need = samples - acceptedLocal.shape[0]
            batch = max(int(np.ceil(need * batchOversample)), need)

            # Uniform trial points on the unit-sphere zone.
            z = rng.uniform(zLo, zHi, size=batch)
            az = rng.uniform(0.0, 2.0 * np.pi, size=batch)
            ringRadius = np.sqrt(np.clip(1.0 - z ** 2, 0.0, None))
            spherePts = np.column_stack((ringRadius * np.cos(az), ringRadius * np.sin(az), z))

            # Anisotropic scaling onto the ellipsoid surface (trial points).
            trialPts = spherePts * r

            # Gradient rejection step (Algorithm 1, lines 5-9 / Eq. 44).
            pAccept = gMin * np.sqrt(np.sum((trialPts / r ** 2) ** 2, axis=1))
            u = rng.uniform(0.0, 1.0, size=batch)
            acceptedLocal = np.vstack((acceptedLocal, trialPts[u <= pAccept]))

        acceptedLocal = acceptedLocal[:samples]
        points = acceptedLocal + np.array([cx, cy, cz])
        normalDir = acceptedLocal / r ** 2
        normals = normalDir / np.linalg.norm(normalDir, axis=1, keepdims=True)
        return points, normals

    # ------------------------------------------------------------------
    # Uniformity checking, following Marples & Williams (2024), "Patch area
    # and uniform sampling on the surface of any ellipsoid", Numer. Algorithms
    # 95:1801-1827. Points are binned into a (theta, phi) grid of patches; each
    # patch's true surface area is computed analytically (sphere/spheroid) or
    # via numerical integration of the elliptic-integral formula (general
    # triaxial ellipsoid), giving an area-weighted expected count per patch.
    # A chi-squared goodness-of-fit statistic (observed vs. expected counts)
    # is then used as a single uniformity score, exactly as in Sect. 3/6 of
    # the paper (a value near/below scipy.stats.chi2.ppf(0.95, dof) supports
    # uniformity; larger values indicate the sample is not uniform).
    # ------------------------------------------------------------------

    @staticmethod
    def _ellipticE(psi: float, k: float) -> float:
        """Legendre's incomplete elliptic integral of the second kind, E(psi, k) (Eq. 14)."""
        return special.ellipeinc(psi, k ** 2)  # scipy takes parameter m = k**2

    @staticmethod
    def _patchAreaSpheroid(a : float, c : float, theta0 : float, theta1 : float,
                            phi0 : float, phi1 : float) -> float:
        """Closed-form patch area for a spheroid/sphere (a == b), Eqs. 23, 30 and 31."""
        dphi = phi1 - phi0
        if np.isclose(a, c):
            # Sphere, Eq. 23.
            return abs(a * a * (np.cos(theta0) - np.cos(theta1)) * dphi)

        if a > c:
            # Oblate spheroid, Eq. 30.
            q = np.sqrt(a ** 2 / c ** 2 - 1.0)

            def F(theta):
                ct = np.cos(theta)
                return np.arcsinh(q * ct) + q * ct * np.sqrt(1.0 + q ** 2 * ct ** 2)

            area = a * c * dphi / (2.0 * q) * (F(theta0) - F(theta1))
        else:
            # Prolate spheroid, Eq. 31.
            qbar = np.sqrt(1.0 - a ** 2 / c ** 2)

            def F(theta):
                ct = np.cos(theta)
                return np.arcsin(qbar * ct) + qbar * ct * np.sqrt(1.0 + qbar ** 2 * ct ** 2)

            area = a * c * dphi / (2.0 * qbar) * (F(theta0) - F(theta1))
        return abs(area)

    @staticmethod
    def _patchAreaTriaxial(a : float, b : float, c : float, theta0 : float, theta1 : float,
                            phi0 : float, phi1 : float) -> float:
        """Triaxial ellipsoid patch area via numerical integration of Eq. 13 (b > a) or
        Eq. 15 (a > b). Uses scipy's elliptic integral directly rather than the
        Carlson-symmetric form of Eq. 18, since scipy's E(psi, k) is well defined for
        any psi and doesn't suffer the sign ambiguity noted in the paper for phi > pi/2."""
        xi0, xi1 = np.cos(theta0), np.cos(theta1)
        xiLo, xiHi = min(xi0, xi1), max(xi0, xi1)

        if a > b:
            # Eq. 15: gamma/kappa defined from (b, c); avoids kappa becoming imaginary.
            def gam(xi):
                return np.sqrt(1.0 + xi ** 2 * (b ** 2 / c ** 2 - 1.0))

            def integrand(xi):
                kappa = np.sqrt(np.clip((1.0 - xi ** 2) * (1.0 - b ** 2 / a ** 2), 0.0, None)) / gam(xi)
                return gam(xi) * (
                    SemiEllipsoid._ellipticE(phi1 - np.pi / 2.0, kappa)
                    - SemiEllipsoid._ellipticE(phi0 - np.pi / 2.0, kappa)
                )

            area, _ = integrate.quad(integrand, xiLo, xiHi)
            return a * c * abs(area)
        else:
            # Eq. 13: g/k defined from (a, c); used when b >= a.
            def g(xi):
                return np.sqrt(1.0 + xi ** 2 * (a ** 2 / c ** 2 - 1.0))

            def integrand(xi):
                k = np.sqrt(np.clip((1.0 - xi ** 2) * (1.0 - a ** 2 / b ** 2), 0.0, None)) / g(xi)
                return g(xi) * (SemiEllipsoid._ellipticE(phi1, k) - SemiEllipsoid._ellipticE(phi0, k))

            area, _ = integrate.quad(integrand, xiLo, xiHi)
            return b * c * abs(area)

    @staticmethod
    def calcPatchArea(a : float, b : float, c : float, theta0 : float, theta1 : float,
                       phi0 : float, phi1 : float) -> float:
        """
        Surface area of the ellipsoidal patch bounded by theta in [theta0, theta1]
        and phi in [phi0, phi1], for an ellipsoid with semi-axes a (x), b (y) and
        c (z, the polar axis) -- see Fig. 1 of Marples & Williams (2024). Uses the
        closed-form spheroid/sphere expressions when a == b, and numerical
        (elliptic-integral) quadrature for the general triaxial case.
        """
        if np.isclose(a, b):
            return SemiEllipsoid._patchAreaSpheroid(a, c, theta0, theta1, phi0, phi1)
        return SemiEllipsoid._patchAreaTriaxial(a, b, c, theta0, theta1, phi0, phi1)

    def calcUniformDistScore(self, nTheta : int = 30, nPhi : int = 60) -> float:
        return float(chi2.ppf(0.95, nTheta * nPhi - 1))

    def calcUniformity(self, pts : npt.NDArray, nTheta : int = 30, nPhi : int = 60) -> float:
        """
        Quantify how uniformly `pts` (points on this ellipsoid's surface, e.g. as
        returned by `sample` or `sampleLatLong`) are spread over the zone, by
        binning them into an nTheta x nPhi grid of (theta, phi) patches spanning
        this zone and comparing observed vs. area-weighted expected counts per
        patch, following Sect. 3 and 6 of Marples & Williams (2024).

        Returns the chi-squared statistic (a single float) as the uniformity
        score. Lower is more uniform; values at or below
        `scipy.stats.chi2.ppf(0.95, nTheta * nPhi - 1)` are consistent with a
        uniform distribution, per the paper's goodness-of-fit test.
        """
        assert pts.shape[0] > 0
        a, b, c = self.radius
        cx, cy, cz = self.center
        N = pts.shape[0]

        # Recover each point's (theta, phi) on the underlying unit sphere by
        # undoing the per-axis scaling and translation used in sample()/sampleLatLong().
        local = (pts - np.array([cx, cy, cz])) / np.array([a, b, c])
        local = local / np.linalg.norm(local, axis=1, keepdims=True)
        theta = np.arccos(np.clip(local[:, 2], -1.0, 1.0))
        phi = np.mod(np.arctan2(local[:, 1], local[:, 0]), 2.0 * np.pi)

        # Theoretical theta bounds of this zone (mirrors the z -> theta mapping
        # used in sample()/zClamp()).
        zLowerSphere = self.relLowerZ / c
        zUpperSphere = self.relUpperZ / c
        thetaLower = np.arccos(np.clip(zUpperSphere, -1.0, 1.0))
        thetaUpper = np.arccos(np.clip(zLowerSphere, -1.0, 1.0))

        thetaEdges = np.linspace(thetaLower, thetaUpper, nTheta + 1)
        phiEdges = np.linspace(0.0, 2.0 * np.pi, nPhi + 1)

        counts, _, _ = np.histogram2d(
            theta, phi,
            bins=[thetaEdges, phiEdges],
            range=[[thetaLower, thetaUpper], [0.0, 2.0 * np.pi]],
        )

        areas = np.empty((nTheta, nPhi))
        for i in range(nTheta):
            for j in range(nPhi):
                areas[i, j] = self.calcPatchArea(
                    a, b, c,
                    thetaEdges[i], thetaEdges[i + 1],
                    phiEdges[j], phiEdges[j + 1],
                )

        expected = N * areas / areas.sum()
        mask = expected > 0
        chiSquared = np.sum((counts[mask] - expected[mask]) ** 2 / expected[mask])
        return float(chiSquared)

    def draw(self, samples : int, showNormales : bool = True) -> None:
        pts, normals = self.sampleFibonacci(samples)
        draw("/home/robot/ws/ellipsoid",
             self.center,
             self.radius,
             pts,
             normals if showNormales else None)


if __name__ == "__main__":
    samples = int(sys.argv[1])
    assert(samples > 0)
    s = 2.0
    shape = SemiEllipsoid(
        (0.05, 0.0, 1.25),
        (0.2*s, 0.35*s, 0.5*s),
        -0.1, 0.4 
    )

    # DEBUG
    pts, normals = shape.sampleFibonacciThomson(samples, 0)
    # 

    rng = np.random.Generator(np.random.PCG64())
    nTheta, nPhi = 3, 6
    pts, normals = shape.sampleGradientRejection(samples, rng, 1)
    draw("/home/robot/ws/gradientRejection",
         shape.center,
         shape.radius,
         pts)
    uniformity = shape.calcUniformity(pts, nTheta, nPhi)
    print("gradient rejection:", uniformity)

    pts, normal = shape.sampleFibonacci(samples)
    draw("/home/robot/ws/naiveApproach",
         shape.center,
         shape.radius,
         pts)
    uniformity = shape.calcUniformity(pts, nTheta, nPhi)
    print("naive approach:", uniformity)
    print("uniform distribution cut-off:", shape.calcUniformDistScore(nTheta, nPhi))