"""Rigid-interface bolt-group load distribution and equilibrium verification.

This module implements an elastic rigid-plate / equal-bolt-stiffness bolt
group model:

- The payload interface plate is treated as rigid.
- All bolts have equal stiffness (no stiffness-weighted load sharing).
- Direct in-plane shear (Fx, Fy) splits equally across all bolts.
- Torsional shear from Mz distributes proportional to each bolt's radial
  distance from the bolt-group centroid (classic "torsion of bolt groups"
  formula).
- Axial/tensile bolt load from Fz and overturning moments (Mx, My)
  distributes linearly with bolt coordinates (plane-sections-remain-plane
  rigid-body rotation of the attach flange), solved generally from the
  bolt-group coordinate sums -- not a formula specialized to circular
  patterns.

Explicitly NOT modeled here (deferred to later milestones): bolt
preload, friction/slip load sharing, joint separation/contact
redistribution, prying, bearing, pull-through, thread effects, fatigue,
or any bolt strength / margin-of-safety calculation. Per Milestone 1
convention, negative axial results are signed compression-side loads,
not clipped to zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .geometry import BoltPattern
from .loads import InterfaceLoad


@dataclass(frozen=True)
class BoltLoadResult:
    """Per-bolt load result. Coordinates (x, y) are the ORIGINAL
    (as-specified) bolt coordinates, for reporting. radius/angle are
    centroid-relative (radius = distance from bolt-group centroid, angle
    = atan2 of centroid-relative coordinates)."""

    index: int
    x: float
    y: float
    Vx_direct: float
    Vy_direct: float
    Vx_torsion: float
    Vy_torsion: float
    Vx_total: float
    Vy_total: float
    shear_resultant: float
    axial_direct: float
    axial_moment: float
    axial_total: float
    radius: float
    angle: float


@dataclass(frozen=True)
class EquilibriumCheck:
    """Independent recovery of group-level loads from the per-bolt
    results, and the residual against the applied InterfaceLoad.

    Recovery is computed directly from the BoltLoadResult list -- it does
    not call back into the distribution routine that produced them, so
    this is a conceptually independent verification path.
    """

    Fx_recovered: float
    Fy_recovered: float
    Fz_recovered: float
    Mx_recovered: float
    My_recovered: float
    Mz_recovered: float

    Fx_residual: float
    Fy_residual: float
    Fz_residual: float
    Mx_residual: float
    My_residual: float
    Mz_residual: float

    max_abs_residual: float


@dataclass(frozen=True)
class BoltGroupResult:
    pattern: BoltPattern
    load: InterfaceLoad
    bolts: Tuple[BoltLoadResult, ...]
    equilibrium: EquilibriumCheck

    # -- governing-bolt identification (deterministic: ties -> lowest index) --

    def max_shear_bolt(self) -> BoltLoadResult:
        return min(self.bolts, key=lambda b: (-b.shear_resultant, b.index))

    def max_tensile_bolt(self) -> BoltLoadResult:
        return min(self.bolts, key=lambda b: (-b.axial_total, b.index))

    def max_abs_axial_bolt(self) -> BoltLoadResult:
        return min(self.bolts, key=lambda b: (-abs(b.axial_total), b.index))


def _solve_axial_coefficients(xs, ys, Fz: float, Mx: float, My: float, n: int) -> Tuple[float, float, float]:
    """Solve for (c0, cx, cy) in T_i = c0 + cx*x_i + cy*y_i such that:

        sum(T_i)          = Fz
        sum(y_i * T_i)     = Mx
        sum(-x_i * T_i)    = My

    using the general 3x3 coefficient system built from bolt-coordinate
    sums (not a formula specialized to any particular pattern shape):

        [ n,    Sx,   Sy  ] [c0]   [ Fz]
        [ Sy,   Sxy,  Syy ] [cx] = [ Mx]
        [-Sx,  -Sxx, -Sxy ] [cy]   [ My]

    where Sx=sum(x_i), Sy=sum(y_i), Sxx=sum(x_i^2), Syy=sum(y_i^2),
    Sxy=sum(x_i*y_i).
    """
    Sx = sum(xs)
    Sy = sum(ys)
    Sxx = sum(x * x for x in xs)
    Syy = sum(y * y for y in ys)
    Sxy = sum(x * y for x, y in zip(xs, ys))

    A = np.array(
        [
            [n, Sx, Sy],
            [Sy, Sxy, Syy],
            [-Sx, -Sxx, -Sxy],
        ],
        dtype=float,
    )
    b = np.array([Fz, Mx, My], dtype=float)

    if abs(np.linalg.det(A)) < 1e-12:
        raise ValueError(
            "Axial moment-distribution coefficient system is singular "
            "(degenerate bolt geometry, e.g. all bolts collinear through "
            "the centroid along one axis). Cannot solve for a unique "
            "linear axial distribution."
        )

    c0, cx, cy = np.linalg.solve(A, b)
    return float(c0), float(cx), float(cy)


def distribute_loads(pattern: BoltPattern, load: InterfaceLoad) -> BoltGroupResult:
    """Distribute an InterfaceLoad (referenced about the bolt-group
    centroid) to individual bolts under the rigid, equal-bolt-stiffness
    model described in this module's docstring.
    """
    n = pattern.n_bolts
    original = pattern.coordinates
    centered = pattern.centered_coordinates
    xs = [c[0] for c in centered]
    ys = [c[1] for c in centered]

    J = pattern.J
    if J <= 0:
        raise ValueError(
            "Bolt-group polar moment J is zero; cannot distribute a "
            "torsional moment Mz across a degenerate bolt group."
        )

    # direct in-plane shear: equal split
    Vx_direct = load.Fx / n
    Vy_direct = load.Fy / n

    # axial coefficients from Fz, Mx, My
    c0, cx, cy = _solve_axial_coefficients(xs, ys, load.Fz, load.Mx, load.My, n)

    bolts = []
    for i in range(n):
        x_orig, y_orig = original[i]
        xi, yi = xs[i], ys[i]

        # torsional shear from Mz, proportional to radius from centroid
        Vx_torsion = -load.Mz * yi / J
        Vy_torsion = load.Mz * xi / J

        Vx_total = Vx_direct + Vx_torsion
        Vy_total = Vy_direct + Vy_torsion
        shear_resultant = math.hypot(Vx_total, Vy_total)

        axial_direct = c0
        axial_moment = cx * xi + cy * yi
        axial_total = axial_direct + axial_moment

        bolts.append(
            BoltLoadResult(
                index=i,
                x=x_orig,
                y=y_orig,
                Vx_direct=Vx_direct,
                Vy_direct=Vy_direct,
                Vx_torsion=Vx_torsion,
                Vy_torsion=Vy_torsion,
                Vx_total=Vx_total,
                Vy_total=Vy_total,
                shear_resultant=shear_resultant,
                axial_direct=axial_direct,
                axial_moment=axial_moment,
                axial_total=axial_total,
                radius=math.hypot(xi, yi),
                angle=math.atan2(yi, xi),
            )
        )

    bolts = tuple(bolts)
    equilibrium = check_equilibrium(pattern, load, bolts)
    return BoltGroupResult(pattern=pattern, load=load, bolts=bolts, equilibrium=equilibrium)


def check_equilibrium(pattern: BoltPattern, load: InterfaceLoad, bolts: Tuple[BoltLoadResult, ...]) -> EquilibriumCheck:
    """Independently recover group-level loads from per-bolt results and
    compare to the applied InterfaceLoad. Uses centroid-relative
    coordinates (recomputed here, independent of any coordinates cached
    inside `bolts`) and simple summation -- a separate code path from
    `distribute_loads`, not a call back into it.
    """
    centered = pattern.centered_coordinates

    Fx_rec = sum(b.Vx_total for b in bolts)
    Fy_rec = sum(b.Vy_total for b in bolts)
    Fz_rec = sum(b.axial_total for b in bolts)

    Mx_rec = sum(yi * b.axial_total for (xi, yi), b in zip(centered, bolts))
    My_rec = sum(-xi * b.axial_total for (xi, yi), b in zip(centered, bolts))
    Mz_rec = sum(xi * b.Vy_total - yi * b.Vx_total for (xi, yi), b in zip(centered, bolts))

    Fx_res = Fx_rec - load.Fx
    Fy_res = Fy_rec - load.Fy
    Fz_res = Fz_rec - load.Fz
    Mx_res = Mx_rec - load.Mx
    My_res = My_rec - load.My
    Mz_res = Mz_rec - load.Mz

    residuals = [Fx_res, Fy_res, Fz_res, Mx_res, My_res, Mz_res]
    max_abs_residual = max(abs(r) for r in residuals)

    return EquilibriumCheck(
        Fx_recovered=Fx_rec,
        Fy_recovered=Fy_rec,
        Fz_recovered=Fz_rec,
        Mx_recovered=Mx_rec,
        My_recovered=My_rec,
        Mz_recovered=Mz_rec,
        Fx_residual=Fx_res,
        Fy_residual=Fy_res,
        Fz_residual=Fz_res,
        Mx_residual=Mx_res,
        My_residual=My_res,
        Mz_residual=Mz_res,
        max_abs_residual=max_abs_residual,
    )
