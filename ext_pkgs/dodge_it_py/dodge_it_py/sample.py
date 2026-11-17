import matplotlib.pyplot as plt
import sys, json
from pathlib import Path
from rich import print

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from scipy.spatial import cKDTree
from scipy import special, integrate
from scipy.stats import chi2
from matplotlib.colors import TwoSlopeNorm

import nlopt

GOLDEN_ANGLE = np.pi * (3.0 - np.sqrt(5.0))
SAMPLES_DIR = "/home/robot/ws/samples/"


@staticmethod
def draw(
    out: str,  # path + prefix
    center: npt.NDArray,
    radius: npt.NDArray,
    points: npt.NDArray,
    normals: npt.NDArray | None = None,
) -> None:
    showNormales = normals is not None
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=4, alpha=0.6)
    if showNormales:
        ax.quiver(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            normals[:, 0],
            normals[:, 1],
            normals[:, 2],
            length=0.2,
            normalize=True,
            color="tab:red",
            linewidth=0.6,
            alpha=0.7,
        )
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
def calcMinDistances(pts: npt.NDArray) -> npt.NDArray:
    tree = cKDTree(pts)
    distances, idxs = tree.query(pts, k=2, workers=-1)
    return distances[:, 1]  # remove own 0 distance and flatten


@staticmethod
def calcNeighbourStdDistances(pts: npt.NDArray, n: int = 4) -> npt.NDArray:
    tree = cKDTree(pts)
    distances, idxs = tree.query(pts, k=5, workers=-1)
    return np.std(distances[:, 1:], axis=1)


@staticmethod
def evaluate(pts: npt.NDArray) -> None:
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
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 2)
    plt.tight_layout()
    out_path = "ellipsoid_eval.pdf"
    plt.savefig(out_path)
    print(f"Plot saved to {out_path}")


@dataclass
class SemiEllipsoid:
    """Axis-aligned ellipsoidal zone sampled with gradient rejection."""

    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: tuple[float, float, float] = (1.0, 1.0, 1.0)
    relLowerZ: float = np.nan
    relUpperZ: float = np.nan

    def __post_init__(self):
        assert len(self.radius) == 3
        assert all(v > 0.0 for v in self.radius)
        assert self.relLowerZ < self.relUpperZ
        assert -self.radius[2] <= self.relLowerZ and self.relLowerZ <= self.radius[2]
        assert -self.radius[2] <= self.relUpperZ and self.relUpperZ <= self.radius[2]

    def sampleFibonacci(self, samples: int) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Using a Fibonacci-lattice grid which is scaled by the radius properties of the ellipsoid.
        """
        assert samples > 0
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
        print(f"obj: {energy:.4} | grad: {np.linalg.norm(grad):.4}")
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

    def sampleFibonacciThomson(
        self, samples: int, seed: int
    ) -> tuple[npt.NDArray, npt.NDArray]:

        # load buffer file if applicable
        sampleFile = Path(SAMPLES_DIR) / f"{seed}_{samples}.json"
        if sampleFile.exists():
            with open(sampleFile, "r") as f:
                data = json.load(f)
                keysAvailable = (
                    "sampleCount" in data.keys()
                    and "seed" in data.keys()
                    and "points" in data.keys()
                    and "normals" in data.keys()
                )
                if keysAvailable:
                    fittingConfig = (
                        data["sampleCount"] == samples and data["seed"] == seed
                    )
                    if fittingConfig:
                        pts = np.array(data["points"])
                        assert type(pts) is np.ndarray
                        normals = np.array(data["normals"])
                        assert type(normals) is np.ndarray
                        print("[INFO] loaded buffered samples")
                        return (pts, normals)

        ptsInit, _ = self.sampleFibonacci(samples)
        r = np.array(self.radius)
        c = np.array(self.center)

        # optimize distance
        nlopt.srand(seed)
        ptsInit = ptsInit.flatten()
        opt = nlopt.opt(nlopt.LD_SLSQP, ptsInit.size)
        # opt = nlopt.opt(nlopt.LD_AUGLAG, ptsInit.size)
        # localOpt = nlopt.opt(nlopt.LN_BOBYQA, ptsInit.size)
        # localOpt.set_xtol_abs(1e-8)
        # localOpt.set_ftol_abs(1e-8)
        # localOpt.set_param("tlog", 1e-8) # gradient tolerance
        # localOpt.set_vector_storage(10)
        # opt.set_local_optimizer(localOpt)

        opt.set_exceptions_enabled(False)
        opt.set_xtol_abs(1e-8)
        opt.set_ftol_abs(1e-8)
        opt.set_min_objective(SemiEllipsoid.obj)
        opt.add_inequality_mconstraint(lambda res, x, grad: SemiEllipsoid.consZUpper(res, x, grad, self.relUpperZ+self.center[2]), np.zeros(int(ptsInit.size/3)))
        opt.add_inequality_mconstraint(lambda res, x, grad: SemiEllipsoid.consZLower(res, x, grad, self.relLowerZ+self.center[2]), np.zeros(int(ptsInit.size/3)))
        opt.add_equality_mconstraint(lambda res, x, grad: SemiEllipsoid.consEllipsoid(res, x, grad, c, r), np.full(int(ptsInit.size/3), 1e-6))
        ptsOpt = opt.optimize(ptsInit)
        print(f"[bold blue][INFO] Code: {opt.last_optimize_result()} {opt.get_errmsg()}")
        assert type(ptsOpt) is np.ndarray
        res = opt.last_optimize_result()
        if res < 0:
            if res != nlopt.ROUNDOFF_LIMITED:
                print(
                    f"[bold red][ERROR] samples could not be generated on the ellipsoid with seed {seed}"
                )
                assert False
            else:
                print(
                    "[bold orange3][WARNING] sample optimization was stopped due to limited round off"
                )
        ptsOpt = ptsOpt.reshape(-1, 3)

        # calculate normals to new points
        normals = 2 * (ptsOpt - c) / r**2
        normals /= np.linalg.norm(normals, axis=1)[:, None]

        # write to buffer file
        sampleFile = Path(SAMPLES_DIR) / f"{seed}_{samples}.json"
        with open(sampleFile, "w") as f:
            data = dict(
                {
                    "seed": seed,
                    "sampleCount": samples,
                    "points": ptsOpt.tolist(),
                    "normals": normals.tolist(),
                }
            )
            json.dump(data, f)
        return (ptsOpt, normals)

    def sampleGradientRejection(
        self, samples: int, rng: np.random.Generator, batchOversample: float = 2.0
    ) -> tuple[npt.NDArray, npt.NDArray]:
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
        gMin = min(
            a, b, c
        )  # Eq. 43: gmax = 1 / min(a,b,c), so P(accept) = gMin * g(x,y,z)

        zLo = self.relLowerZ / c
        zHi = self.relUpperZ / c

        acceptedLocal = np.empty((0, 3))
        while acceptedLocal.shape[0] < samples:
            need = samples - acceptedLocal.shape[0]
            batch = max(int(np.ceil(need * batchOversample)), need)

            # Uniform trial points on the unit-sphere zone.
            z = rng.uniform(zLo, zHi, size=batch)
            az = rng.uniform(0.0, 2.0 * np.pi, size=batch)
            ringRadius = np.sqrt(np.clip(1.0 - z**2, 0.0, None))
            spherePts = np.column_stack(
                (ringRadius * np.cos(az), ringRadius * np.sin(az), z)
            )

            # Anisotropic scaling onto the ellipsoid surface (trial points).
            trialPts = spherePts * r

            # Gradient rejection step (Algorithm 1, lines 5-9 / Eq. 44).
            pAccept = gMin * np.sqrt(np.sum((trialPts / r**2) ** 2, axis=1))
            u = rng.uniform(0.0, 1.0, size=batch)
            acceptedLocal = np.vstack((acceptedLocal, trialPts[u <= pAccept]))

        acceptedLocal = acceptedLocal[:samples]
        points = acceptedLocal + np.array([cx, cy, cz])
        normalDir = acceptedLocal / r**2
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
        return special.ellipeinc(psi, k**2)  # scipy takes parameter m = k**2

    @staticmethod
    def calcPatchArea(
        a: float,
        b: float,
        c: float,
        theta0: float,
        theta1: float,
        phi0: float,
        phi1: float,
    ) -> float:
        """Triaxial ellipsoid patch area via numerical integration of Eq. 13 (b > a) or
        Eq. 15 (a > b). Uses scipy's elliptic integral directly rather than the
        Carlson-symmetric form of Eq. 18, since scipy's E(psi, k) is well defined for
        any psi and doesn't suffer the sign ambiguity noted in the paper for phi > pi/2."""
        xi0, xi1 = np.cos(theta0), np.cos(theta1)
        xiLo, xiHi = min(xi0, xi1), max(xi0, xi1)

        if a > b:
            # Eq. 15: gamma/kappa defined from (b, c); avoids kappa becoming imaginary.
            def gam(xi):
                return np.sqrt(1.0 + xi**2 * (b**2 / c**2 - 1.0))

            def integrand(xi):
                kappa = np.sqrt(
                    np.clip((1.0 - xi**2) * (1.0 - b**2 / a**2), 0.0, None)
                ) / gam(xi)
                return gam(xi) * (
                    SemiEllipsoid._ellipticE(phi1 - np.pi / 2.0, kappa)
                    - SemiEllipsoid._ellipticE(phi0 - np.pi / 2.0, kappa)
                )

            area, _ = integrate.quad(integrand, xiLo, xiHi)
            return a * c * abs(area)
        else:
            # Eq. 13: g/k defined from (a, c); used when b >= a.
            def g(xi):
                return np.sqrt(1.0 + xi**2 * (a**2 / c**2 - 1.0))

            def integrand(xi):
                k = np.sqrt(
                    np.clip((1.0 - xi**2) * (1.0 - a**2 / b**2), 0.0, None)
                ) / g(xi)
                return g(xi) * (
                    SemiEllipsoid._ellipticE(phi1, k)
                    - SemiEllipsoid._ellipticE(phi0, k)
                )

            area, _ = integrate.quad(integrand, xiLo, xiHi)
            return b * c * abs(area)

    def calcUniformDistScore(self, nTheta: int = 30, nPhi: int = 60) -> float:
        return float(chi2.ppf(0.95, nTheta * nPhi - 1))

    def calcUniformity(self, pts: npt.NDArray, nTheta: int, nPhi: int, areas : npt.NDArray) -> float:
        """
        Quantify how uniformly `pts` (points on this ellipsoid's surface, e.g. as
        returned by `sample` or `sampleLatLong`) are spread over the zone, by
        binning them into an nTheta x nPhi grid of (theta, phi) patches spanning
        this zone and comparing observed vs. area-weighted expected counts per
        patch, following Sect. 3 and 6 of Marples & Williams (2024).

        Returns the chi-squared statistic as the uniformity
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
            theta,
            phi,
            bins=[thetaEdges, phiEdges],
            range=[[thetaLower, thetaUpper], [0.0, 2.0 * np.pi]],
        )

        expected = N * areas / areas.sum()
        mask = expected > 0
        chiSquared = np.sum((counts[mask] - expected[mask]) ** 2 / expected[mask])
        return float(chiSquared)

    def calcPatchAreaGrid(self, nTheta: int, nPhi: int) -> npt.NDArray:
        """
        Areas (calcPatchArea, Eq. 13/15) of the nTheta x nPhi (theta, phi) patch grid
        spanning this zone, shape (nTheta, nPhi). Expensive for fine grids; compute it
        once and pass it to calcRSD via `areas`.
        """
        a, b, c = self.radius
        thetaLower = np.arccos(np.clip(self.relUpperZ / c, -1.0, 1.0))
        thetaUpper = np.arccos(np.clip(self.relLowerZ / c, -1.0, 1.0))
        thetaEdges = np.linspace(thetaLower, thetaUpper, nTheta + 1)
        phiEdges = np.linspace(0.0, 2.0 * np.pi, nPhi + 1)
        areas = np.empty((nTheta, nPhi))
        for i in range(nTheta):
            for j in range(nPhi):
                areas[i, j] = self.calcPatchArea(
                    a, b, c,
                    thetaEdges[i], thetaEdges[i + 1],
                    phiEdges[j], phiEdges[j + 1],
                )
        return areas

    def calcRSD(
        self,
        pts: npt.NDArray,
        nTheta: int,
        nPhi: int,
        areas: npt.NDArray | None = None,
    ) -> float:
        """
        Relative standard deviation (RSD) of Marples & Williams (2024), Sect. 3:
        the points are binned into the nTheta x nPhi patch grid, the number of
        points per patch area is computed for every patch, and the RSD is the
        standard deviation of that distribution divided by its mean. Small values
        indicate a uniform sample. Returned in percent, as in Tables 1-3 of the paper.

        The paper does not state whether the population or sample standard
        deviation is used; the population one (ddof=0) is used here (the difference
        is negligible for thousands of patches).

        `areas` may be given (from calcPatchAreaGrid) to avoid recomputing them.
        """
        assert pts.shape[0] > 0
        a, b, c = self.radius
        cx, cy, cz = self.center

        # (theta, phi) of every point, as in calcUniformity
        local = (pts - np.array([cx, cy, cz])) / np.array([a, b, c])
        local = local / np.linalg.norm(local, axis=1, keepdims=True)
        theta = np.arccos(np.clip(local[:, 2], -1.0, 1.0))
        phi = np.mod(np.arctan2(local[:, 1], local[:, 0]), 2.0 * np.pi)

        thetaLower = np.arccos(np.clip(self.relUpperZ / c, -1.0, 1.0))
        thetaUpper = np.arccos(np.clip(self.relLowerZ / c, -1.0, 1.0))
        thetaEdges = np.linspace(thetaLower, thetaUpper, nTheta + 1)
        phiEdges = np.linspace(0.0, 2.0 * np.pi, nPhi + 1)
        counts, _, _ = np.histogram2d(theta, phi, bins=[thetaEdges, phiEdges])

        if areas is None:
            areas = self.calcPatchAreaGrid(nTheta, nPhi)
        assert areas.shape == (nTheta, nPhi)

        pointsPerArea = counts / areas
        return float(100.0 * np.std(pointsPerArea) / np.mean(pointsPerArea))

    def sampleNaive(
        self, samples: int, rng: np.random.Generator
    ) -> tuple[npt.NDArray, npt.NDArray]:
        """
        'Naive scaling method' of Marples & Williams (2024), Sect. 5: uniform points
        on the unit-sphere zone are scaled anisotropically onto the ellipsoid with
        NO rejection step, so the result is not uniform by surface area (Fig. 3a/b).
        Same trial-point generation as in sampleGradientRejection.
        """
        assert samples > 0
        r = np.array(self.radius)
        zLo = self.relLowerZ / self.radius[2]
        zHi = self.relUpperZ / self.radius[2]
        z = rng.uniform(zLo, zHi, size=samples)
        az = rng.uniform(0.0, 2.0 * np.pi, size=samples)
        ringRadius = np.sqrt(np.clip(1.0 - z**2, 0.0, None))
        local = np.column_stack((ringRadius * np.cos(az), ringRadius * np.sin(az), z)) * r
        normalDir = local / r**2
        normals = normalDir / np.linalg.norm(normalDir, axis=1, keepdims=True)
        return local + np.array(self.center), normals

    def drawDistribution(
        self,
        out: str,  # path + prefix, e.g. "/home/robot/ws/gradientRejection"
        pts: npt.NDArray,
        areas: npt.NDArray,
        title: str | None = None,
        areaWeighted: bool = False,
    ) -> tuple[npt.NDArray, npt.NDArray]:
        """
        areaWeighted=True:  area-weighted view in the spirit of Fig. 3 (b)/(d): colour is
        observed / expected count per patch, O_i / E_i with E_i from Eq. 34, i.e.
        points per patch area relative to the uniform expectation (1 = uniform).
        The paper does not state the exact normalisation of its (b)/(d) colour bar.

        Recreates Fig. 3 (a)/(c) of Marples & Williams (2024): the ellipsoid zone is
        divided into an nTheta x nPhi grid of (theta, phi) patches (Fig. 1 / Eq. 1-3),
        points are counted per patch, and every patch is drawn on the surface and
        coloured by its point count, with a colour bar on the side.

        The patch areas (calcPatchArea, Eq. 13/15) are used to compute the expected
        counts and the chi-squared statistic (Eq. 33/34) of this very grid, which is
        printed next to the critical value.

        Returns (counts, areas), both of shape (nTheta, nPhi).
        """
        assert pts.shape[0] > 0
        a, b, c = self.radius
        cx, cy, cz = self.center
        N = pts.shape[0]

        # (theta, phi) of every point, as in calcUniformity
        local = (pts - np.array([cx, cy, cz])) / np.array([a, b, c])
        local = local / np.linalg.norm(local, axis=1, keepdims=True)
        theta = np.arccos(np.clip(local[:, 2], -1.0, 1.0))
        phi = np.mod(np.arctan2(local[:, 1], local[:, 0]), 2.0 * np.pi)

        # patch limits of this zone (theta decreases with z)
        thetaLower = np.arccos(np.clip(self.relUpperZ / c, -1.0, 1.0))
        thetaUpper = np.arccos(np.clip(self.relLowerZ / c, -1.0, 1.0))
        thetaEdges = np.linspace(thetaLower, thetaUpper, nTheta + 1)
        phiEdges = np.linspace(0.0, 2.0 * np.pi, nPhi + 1)

        # 1. count the points per patch
        counts, _, _ = np.histogram2d(theta, phi, bins=[thetaEdges, phiEdges])

        # patch areas (existing triaxial patch-area function) -> expected counts
        expected = N * areas / areas.sum()
        mask = expected > 0

        # 2. colour every patch by its count
        if areaWeighted:
            values = np.divide(counts, expected, out=np.zeros_like(counts), where=mask)
            cmap = plt.get_cmap("magma")
            norm = TwoSlopeNorm(vcenter=1.0, vmin=0.0, vmax=max(2.0, float(values.max())))
            cbarLabel = "points per patch area / uniform expectation (O/E)"
        else:
            values = counts
            cmap = plt.get_cmap("magma")
            norm = plt.Normalize(vmin=0.0, vmax=float(counts.max()))
            cbarLabel = "points per patch"
        faceColors = cmap(norm(values))
        faceColors[...,3] = 0.3 # facecolor alpha

        # patch corner vertices on the ellipsoid (Eq. 1-3)
        T, P = np.meshgrid(thetaEdges, phiEdges, indexing="ij")
        X = cx + a * np.sin(T) * np.cos(P)
        Y = cy + b * np.sin(T) * np.sin(P)
        Z = cz + c * np.cos(T)

        fig = plt.figure(figsize=(7, 5))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot_surface(
            X, Y, Z,
            facecolors=faceColors,
            rstride=1,
            cstride=1,
            shade=False,
            edgecolor=(1.0, 0.2, 0.2, 1.0),
            linewidth=0.4,
            antialiased=False,
        )

        # 3. colour map legend (as in Fig. 3)
        mappable = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array([])
        cbar = fig.colorbar(mappable, ax=ax, shrink=0.6, pad=0.08)
        cbar.set_label(cbarLabel)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(cx - a, cx + a)
        ax.set_ylim(cy - b, cy + b)
        ax.set_zlim(cz + self.relLowerZ, cz + self.relUpperZ)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_major_locator(plt.MaxNLocator(4))  # avoid overlapping tick labels
        ax.set_box_aspect([2 * a, 2 * b, self.relUpperZ - self.relLowerZ])  # true scale
        ax.view_init(elev=30, azim=-60)
        ax.set_title(f"{title or 'point distribution'} ({nTheta}x{nPhi} patches, N={N})")
        plt.tight_layout()

        f = f"{out}{'_perArea' if areaWeighted else ''}.pdf"
        plt.savefig(f)
        print(f"Plot saved to {f}")
        plt.close(fig)
        return counts, areas


if __name__ == "__main__":
    samples = int(sys.argv[1])
    assert samples > 0
    center = (0.00, 0.0, 1.3)
    radius = (0.1, 0.25, 0.5)
    shape = SemiEllipsoid(center, radius, 0.05, 0.45)

    rng = np.random.Generator(np.random.PCG64())
    nTheta, nPhi = 180, 360
    areas = shape.calcPatchAreaGrid(nTheta, nPhi)

    # pts, normal = shape.sampleFibonacci(samples)
    #       Fibonacci Thomson
    ptsFT, normalFT = shape.sampleFibonacciThomson(samples, 0)
    # draw("/home/robot/ws/fibonacciThomson", shape.center, shape.radius, ptsFT)
    print("Fibonacci-Thomson approach:", shape.calcUniformity(ptsFT, nTheta, nPhi, areas))
    print("Fibonacci-Thomson approach RSD [%]:", shape.calcRSD(ptsFT, nTheta, nPhi, areas))
    shape.drawDistribution("/home/robot/ws/distFibonacciThomson", ptsFT, areas, "Fibonacci-Thomson", True)

    #       Rejection Sampling
    ptsRej, normalsRej = shape.sampleGradientRejection(samples, rng, 1)
    # draw("/home/robot/ws/gradientRejection", shape.center, shape.radius, ptsRej)
    print("gradient rejection:", shape.calcUniformity(ptsRej, nTheta, nPhi, areas))
    print("gradient rejection RSD [%]:", shape.calcRSD(ptsRej, nTheta, nPhi, areas))
    shape.drawDistribution("/home/robot/ws/distGradientRejection", ptsRej, areas, "Gradient Rejection", True)

    print("-----------------------------------------------------------------------")
    print("uniform distribution cut-off:", shape.calcUniformDistScore(nTheta, nPhi))