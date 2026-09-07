"""Bolt-pattern geometry for a rigid payload attach interface.

Coordinate convention
----------------------
- The interface is modeled as a rigid plate lying in the x-y plane.
- +z is the interface normal (out of the joint).
- Each bolt is an idealized point fastener at coordinates (x_i, y_i) in
  the interface plane, in meters.
- The bolt-group centroid is computed as the simple average of bolt
  coordinates (equal bolt stiffness is assumed everywhere in Milestone 1,
  so the centroid is also the elastic center of the group).
- Group section properties (radii, polar moment J, Ix_group, Iy_group,
  Ixy_group) are computed relative to the bolt-group centroid, because
  applied interface moments are referenced about the bolt-group centroid
  (see loads.InterfaceLoad). Original (as-specified) coordinates are
  retained unchanged for reporting.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

Coordinate = Tuple[float, float]


def _validate_coordinates(coordinates: Sequence[Coordinate], allow_duplicates: bool) -> Tuple[Coordinate, ...]:
    coords = tuple((float(x), float(y)) for x, y in coordinates)

    if len(coords) < 2:
        raise ValueError(f"BoltPattern requires at least 2 bolts, got {len(coords)}.")

    for i, (x, y) in enumerate(coords):
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Bolt {i} has non-finite coordinates: ({x}, {y}).")

    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    if max(xs) == min(xs) and max(ys) == min(ys):
        raise ValueError("All bolts are coincident at the same point; a degenerate bolt pattern is not allowed.")

    if not allow_duplicates:
        seen = set()
        for i, c in enumerate(coords):
            if c in seen:
                raise ValueError(
                    f"Duplicate bolt coordinate {c} at index {i}. "
                    "Pass allow_duplicates=True to intentionally allow coincident bolts."
                )
            seen.add(c)

    return coords


@dataclass(frozen=True)
class BoltPattern:
    """A validated, immutable bolt-group geometry.

    Parameters
    ----------
    coordinates:
        Sequence of (x, y) bolt coordinates in meters, in the interface
        plane. Copied defensively into an immutable tuple of tuples.
    allow_duplicates:
        If False (default), raises ValueError on coincident bolt
        coordinates. Set True to intentionally allow duplicates (e.g.
        doubled-up fasteners at one nominal location).
    """

    coordinates: Tuple[Coordinate, ...]
    allow_duplicates: bool = False

    def __init__(self, coordinates: Sequence[Coordinate], allow_duplicates: bool = False):
        validated = _validate_coordinates(coordinates, allow_duplicates)
        object.__setattr__(self, "coordinates", validated)
        object.__setattr__(self, "allow_duplicates", allow_duplicates)

    # -- basic counts / centroid -------------------------------------------------

    @property
    def n_bolts(self) -> int:
        return len(self.coordinates)

    @property
    def centroid(self) -> Coordinate:
        n = self.n_bolts
        cx = sum(x for x, _ in self.coordinates) / n
        cy = sum(y for _, y in self.coordinates) / n
        return (cx, cy)

    @property
    def centered_coordinates(self) -> Tuple[Coordinate, ...]:
        """Bolt coordinates translated so the centroid is at the origin."""
        cx, cy = self.centroid
        return tuple((x - cx, y - cy) for x, y in self.coordinates)

    # -- section properties (centroid-relative) ----------------------------------

    @property
    def radii(self) -> Tuple[float, ...]:
        """Radial distance of each bolt from the bolt-group centroid."""
        return tuple(math.hypot(x, y) for x, y in self.centered_coordinates)

    @property
    def J(self) -> float:
        """Polar second-moment-like quantity sum(r_i^2), centroid-relative."""
        return sum(x * x + y * y for x, y in self.centered_coordinates)

    @property
    def Ix_group(self) -> float:
        """sum(y_i^2), centroid-relative."""
        return sum(y * y for _, y in self.centered_coordinates)

    @property
    def Iy_group(self) -> float:
        """sum(x_i^2), centroid-relative."""
        return sum(x * x for x, _ in self.centered_coordinates)

    @property
    def Ixy_group(self) -> float:
        """sum(x_i*y_i), centroid-relative."""
        return sum(x * y for x, y in self.centered_coordinates)


def circular_pattern(n_bolts: int, radius: float, angular_offset: float = 0.0, center: Coordinate = (0.0, 0.0)) -> BoltPattern:
    """Bolts evenly spaced on a circle of the given radius.

    x_i = cx + R*cos(theta_i)
    y_i = cy + R*sin(theta_i)
    theta_i = angular_offset + 2*pi*i/n_bolts

    Parameters
    ----------
    n_bolts: integer, >= 2.
    radius: bolt-circle radius in meters, > 0.
    angular_offset: rotation of the first bolt, radians.
    center: (x, y) location of the pattern center, meters. Defaults to
        the origin, which is also then the bolt-group centroid.
    """
    if not isinstance(n_bolts, int) or isinstance(n_bolts, bool):
        raise ValueError(f"n_bolts must be an integer, got {type(n_bolts).__name__}.")
    if n_bolts < 2:
        raise ValueError(f"n_bolts must be >= 2, got {n_bolts}.")
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError(f"radius must be finite and > 0, got {radius}.")
    if not math.isfinite(angular_offset):
        raise ValueError(f"angular_offset must be finite, got {angular_offset}.")
    cx, cy = center
    if not (math.isfinite(cx) and math.isfinite(cy)):
        raise ValueError(f"center must be finite, got {center}.")

    coords = []
    for i in range(n_bolts):
        theta = angular_offset + 2.0 * math.pi * i / n_bolts
        coords.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    return BoltPattern(coords)


def rectangular_pattern(a: float, b: float, center: Coordinate = (0.0, 0.0)) -> BoltPattern:
    """Four bolts at the corners of an a x b rectangle: (+/-a/2, +/-b/2).

    Parameters
    ----------
    a: full width along x, meters, > 0.
    b: full height along y, meters, > 0.
    center: (x, y) location of the rectangle center, meters.
    """
    if not math.isfinite(a) or a <= 0:
        raise ValueError(f"a must be finite and > 0, got {a}.")
    if not math.isfinite(b) or b <= 0:
        raise ValueError(f"b must be finite and > 0, got {b}.")
    cx, cy = center
    if not (math.isfinite(cx) and math.isfinite(cy)):
        raise ValueError(f"center must be finite, got {center}.")

    half_a, half_b = a / 2.0, b / 2.0
    coords = [
        (cx + half_a, cy + half_b),
        (cx - half_a, cy + half_b),
        (cx - half_a, cy - half_b),
        (cx + half_a, cy - half_b),
    ]
    return BoltPattern(coords)
