import matplotlib.pyplot as plt

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import numpy.typing as npt

GOLDEN_ANGLE = np.pi * (3.0 - np.sqrt(5.0))

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

    def zClamp(self) -> Tuple[float, float]:
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
        ax.set_title("Semi-sphere sample grid")
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
        assert len(self.radius) == 3
        assert all(v > 0.0 for v in self.radius)
        assert self.relLowerZ < self.relUpperZ
        assert -self.radius[2] <= self.relLowerZ <= self.radius[2]
        assert -self.radius[2] <= self.relUpperZ <= self.radius[2]

    def sample(self, samples: int) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Using a Fibonacci-lattice grid which is scaled by the radius properties of the ellipsoid.
        """
        assert(samples > 0)

        c = np.array(self.center)
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
    
        points = np.column_stack((x, y, z)) * self.radius + c
        normals = points / r**2
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        return points, normals


    def draw(self, samples: int) -> None:
        points, normals = self.sample(samples)
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=4, alpha=0.6)
        ax.quiver(points[:, 0], points[:, 1], points[:, 2],
                  normals[:, 0], normals[:, 1], normals[:, 2],
                  length=0.2, normalize=True, color="tab:red", linewidth=0.6, alpha=0.7)
        cx, cy, cz = self.center
        a, b, c = self.radius
        extent = max(a, b, c)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Semi-ellipsoid")
        ax.set_box_aspect([a, b, c])
        ax.set_xlim(cx - extent, cx + extent)
        ax.set_ylim(cy - extent, cy + extent)
        ax.set_zlim(cz - extent, cz + extent)
        plt.tight_layout()

        out = "/home/robot/ws/ellipsoid"
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

if __name__ == "__main__":
    SemiEllipsoid(
        center=(0.0, 0.0, 1.0),
        radius=(1.0, 0.5, 1.8),
        relLowerZ=-1.0,
        relUpperZ=1.0,
    ).draw(500)