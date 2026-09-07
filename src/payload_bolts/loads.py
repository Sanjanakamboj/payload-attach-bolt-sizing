"""Applied interface loads for a rigid payload attach flange.

Sign convention
---------------
- Fx, Fy: in-plane shear resultants (N), acting in the interface x-y plane.
- Fz: axial/normal resultant (N). Positive Fz is defined as TENSION
  (pulling the payload away from the structure, +z direction). Fz may be
  negative (compression); Milestone 1 does not clip or reinterpret sign.
- Mx, My: overturning moments (N*m) about the x and y axes, respectively.
- Mz: in-plane torsional moment (N*m) about the interface normal (+z),
  positive by the right-hand rule (counterclockwise when viewed from +z
  looking down at the x-y plane).
- All loads (forces and moments) are referenced about the bolt-group
  centroid (see geometry.BoltPattern.centroid), not necessarily the
  origin of the coordinate system used to define bolt locations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class InterfaceLoad:
    """Applied interface force/moment resultant, referenced about the
    bolt-group centroid. Units: N and N*m. All components may be zero;
    all components must be finite.
    """

    Fx: float = 0.0
    Fy: float = 0.0
    Fz: float = 0.0
    Mx: float = 0.0
    My: float = 0.0
    Mz: float = 0.0

    def __post_init__(self):
        for name in ("Fx", "Fy", "Fz", "Mx", "My", "Mz"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"InterfaceLoad.{name} must be finite, got {value}.")


def inertial_force(mass: float, acceleration_g: float, g0: float = 9.80665) -> float:
    """Quasi-static inertial force (N) from a mass (kg) under a load
    factor expressed in units of standard gravity g0 (m/s^2).

    force = mass * acceleration_g * g0

    This is a standalone convenience helper for translating a launch
    load factor into a force. It is intentionally NOT wired into the
    bolt-group solver: the solver consumes InterfaceLoad in N / N*m only,
    and load-factor assumptions must not be hidden inside it.
    """
    if not math.isfinite(mass) or mass < 0:
        raise ValueError(f"mass must be finite and >= 0, got {mass}.")
    if not math.isfinite(acceleration_g):
        raise ValueError(f"acceleration_g must be finite, got {acceleration_g}.")
    if not math.isfinite(g0) or g0 <= 0:
        raise ValueError(f"g0 must be finite and > 0, got {g0}.")
    return mass * acceleration_g * g0
