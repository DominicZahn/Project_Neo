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

if __name__ == "__main__":
    SemiSphere(
        center=(0.0, 0.0, 1.0),
        radius=1.0,
        lowerAltitude=-np.pi/7,
        upperAltitude=np.pi/7,
    ).draw(2000)