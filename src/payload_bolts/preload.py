"""First-order preloaded-joint closure and friction-slip screening
(Milestone 3).

This module consumes the already-verified Milestone 1 per-bolt loads
(`axial_total`, `shear_resultant` from `payload_bolts.solver`) and adds a
preliminary preloaded-joint layer on top. It does **not** modify or
recompute:

- Milestone 1 bolt-group geometry, direct/torsional shear, overturning
  axial distribution, or equilibrium verification;
- Milestone 2 bolt tensile/shear strength margins, the quadratic
  interaction criterion, or its governing-mode tie-break convention.

Engineering question answered here: for the selected payload attach
bolt pattern, how much preload is required to keep the joint closed and
prevent interface slip under the representative launch loads?

Scope: explicit bolt preload input, a single scalar bolt/member
load-fraction `C`, external tensile-load sharing between bolt and
clamped joint, joint-separation screening, remaining clamp force,
friction capacity from remaining clamp force, local interface-slip
screening, and analytical required-preload equations. All results here
are **preliminary joint-closure/slip screening margins**, not
certification margins, and this module does NOT check bolt preload
against proof/yield strength, does NOT convert preload to installation
torque, and does NOT model embedment or thermal preload loss.

Signed external axial-load policy (mirrors Milestone 2)
--------------------------------------------------------
Milestone 1's signed `axial_total` per bolt is never mutated. Only the
separating (tensile) portion of it creates separation/slip demand:

    T_sep,i = max(axial_total_i, 0.0)

A bolt with axial_total_i <= 0 has no separation demand at that location
from the group-mechanics load path (it is on the compression side); this
does not mean the joint is somehow "extra safe" there beyond what the
preload alone already provides, it simply means this module does not
add any additional separating demand for that bolt.

Local friction/slip approximation
----------------------------------
The local per-bolt friction-capacity check assumes each bolt station's
surrounding clamped area supplies friction in proportion to that
station's own remaining clamp force,
`V_fric_cap,i = mu * n_interfaces * max(F_clamp_remaining,i, 0)`,
compared against that bolt's own Milestone 1 shear demand
`V_i = shear_resultant_i` (which already reflects both direct shear and
Mz torsional shear). This is a preliminary load-sharing approximation,
not a detailed contact-pressure/friction analysis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .solver import BoltGroupResult


@dataclass(frozen=True)
class PreloadState:
    """Explicit bolt preload input. Preload is an explicit engineering
    input in Milestone 3 -- it is NOT derived from an installation
    torque here.

    preload_per_bolt: N, must be finite and > 0.
    label: optional free-text description (e.g. "selected", "0.5x required").
    """

    preload_per_bolt: float
    label: str = ""

    def __post_init__(self):
        if not math.isfinite(self.preload_per_bolt) or self.preload_per_bolt <= 0:
            raise ValueError(f"preload_per_bolt must be finite and > 0, got {self.preload_per_bolt}.")


@dataclass(frozen=True)
class JointStiffness:
    """Equivalent linear bolt/member stiffness for preload load sharing.

    These are PRELIMINARY equivalent stiffnesses for a first-order
    bolted-joint load-fraction model -- not derived from detailed
    flange/washer/thread flexibility, and not claimed to be sourced.

    bolt_stiffness, member_stiffness: N/m, both finite and > 0.

    C = bolt_stiffness / (bolt_stiffness + member_stiffness) is the
    fraction of an external separating tensile load carried as
    additional bolt load before joint separation (0 < C < 1 whenever
    both stiffnesses are finite and positive).
    """

    bolt_stiffness: float
    member_stiffness: float

    def __post_init__(self):
        for name in ("bolt_stiffness", "member_stiffness"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"JointStiffness.{name} must be finite and > 0, got {value}.")

    @property
    def C(self) -> float:
        return self.bolt_stiffness / (self.bolt_stiffness + self.member_stiffness)


@dataclass(frozen=True)
class FrictionModel:
    """Illustrative interface friction model. `friction_coefficient` is
    a preliminary/illustrative value unless explicitly sourced by the
    caller from a real material-pair/coating test; this module never
    labels a default value as sourced.

    friction_coefficient (mu): dimensionless, finite and > 0.
    number_of_faying_surfaces: integer >= 1 (single-shear joint = 1).
    """

    friction_coefficient: float
    number_of_faying_surfaces: int = 1

    def __post_init__(self):
        if not math.isfinite(self.friction_coefficient) or self.friction_coefficient <= 0:
            raise ValueError(
                f"friction_coefficient must be finite and > 0, got {self.friction_coefficient}."
            )
        n = self.number_of_faying_surfaces
        if not isinstance(n, int) or isinstance(n, bool) or n < 1:
            raise ValueError(f"number_of_faying_surfaces must be an integer >= 1, got {n!r}.")


@dataclass(frozen=True)
class BoltPreloadResult:
    """Per-bolt preloaded-joint screening result."""

    index: int
    axial_total: float  # signed external axial load, Milestone 1, unchanged
    separating_demand: float  # T_sep,i = max(axial_total, 0)
    preload: float  # F_preload
    load_fraction: float  # C
    additional_bolt_load: float  # Delta_F_b,i = C * T_sep,i
    total_bolt_tension: float  # F_preload + Delta_F_b,i
    clamp_force_reduction: float  # Delta_F_m,i = (1-C) * T_sep,i
    remaining_clamp_force: float  # F_preload - Delta_F_m,i (unclipped, signed)
    separation_margin: Optional[float]
    separation_pass: bool
    shear_demand: float  # V_i, Milestone 1 shear_resultant, unchanged
    friction_capacity: float  # V_fric_cap,i
    slip_margin: Optional[float]
    slip_pass: bool


@dataclass(frozen=True)
class BoltPreloadGroupResult:
    """Group-level preloaded-joint screening result.

    Overall `passed` is PASS only if there is no local separation and no
    local slip at any bolt. Milestone 2 bolt-strength pass/fail is a
    separate, independently reported mechanism and is never folded into
    this flag.
    """

    bolts: Tuple[BoltPreloadResult, ...]
    load_fraction: float
    total_preload: float
    total_remaining_clamp_force: float  # sum of unclipped remaining clamp force
    total_effective_clamp_force: float  # sum of max(remaining, 0), used for friction
    all_locations_closed: bool
    all_locations_no_slip: bool
    governing_separation_bolt_index: int
    governing_slip_bolt_index: int
    min_separation_margin: Optional[float]
    min_slip_margin: Optional[float]
    passed: bool

    def governing_separation_bolt(self) -> BoltPreloadResult:
        return self.bolts[self.governing_separation_bolt_index]

    def governing_slip_bolt(self) -> BoltPreloadResult:
        return self.bolts[self.governing_slip_bolt_index]


@dataclass(frozen=True)
class RequiredPreloadResult:
    """Analytical required-preload screening result (NOT a torque
    specification -- see module docstring)."""

    separation_required: float
    separation_governing_bolt: int
    slip_required: float
    slip_governing_bolt: int
    overall_required: float
    overall_governing_bolt: int
    overall_governing_constraint: str  # "separation" | "slip"


def _select_governing_min(values: Sequence[Optional[float]]) -> int:
    """Index of the smallest applicable (non-None) value; None is
    treated as +inf (no constraint). Ties -> lowest index."""
    return min(range(len(values)), key=lambda i: (values[i] if values[i] is not None else math.inf, i))


def _select_governing_max(values: Sequence[float]) -> int:
    """Index of the largest value. Ties -> lowest index."""
    return min(range(len(values)), key=lambda i: (-values[i], i))


def _assess_single_bolt_preload(
    index: int, axial_total: float, shear_demand: float, preload_per_bolt: float, C: float, friction: FrictionModel
) -> BoltPreloadResult:
    T_sep = max(axial_total, 0.0)

    additional_bolt_load = C * T_sep
    total_bolt_tension = preload_per_bolt + additional_bolt_load

    clamp_force_reduction = (1.0 - C) * T_sep
    remaining_clamp_force = preload_per_bolt - clamp_force_reduction  # unclipped, signed

    if T_sep > 0.0:
        separation_margin = preload_per_bolt / ((1.0 - C) * T_sep) - 1.0
    else:
        separation_margin = None
    separation_pass = remaining_clamp_force >= 0.0

    clamp_force_effective = max(remaining_clamp_force, 0.0)
    friction_capacity = friction.friction_coefficient * friction.number_of_faying_surfaces * clamp_force_effective

    if shear_demand > 0.0:
        if friction_capacity > 0.0:
            slip_margin = friction_capacity / shear_demand - 1.0
        else:
            # Zero remaining clamp with nonzero shear demand: explicit
            # finite fail convention (no NaN/inf), see module docstring.
            slip_margin = -1.0
        slip_pass = slip_margin >= 0.0
    else:
        slip_margin = None
        slip_pass = True

    return BoltPreloadResult(
        index=index,
        axial_total=axial_total,
        separating_demand=T_sep,
        preload=preload_per_bolt,
        load_fraction=C,
        additional_bolt_load=additional_bolt_load,
        total_bolt_tension=total_bolt_tension,
        clamp_force_reduction=clamp_force_reduction,
        remaining_clamp_force=remaining_clamp_force,
        separation_margin=separation_margin,
        separation_pass=separation_pass,
        shear_demand=shear_demand,
        friction_capacity=friction_capacity,
        slip_margin=slip_margin,
        slip_pass=slip_pass,
    )


def assess_preloaded_joint(
    group_load_result: BoltGroupResult,
    preload: PreloadState,
    stiffness: JointStiffness,
    friction: FrictionModel,
) -> BoltPreloadGroupResult:
    """Assess preliminary preloaded-joint closure and local friction/slip
    for every bolt in a Milestone 1 `BoltGroupResult`, given a uniform
    per-bolt preload, joint stiffness (load fraction C), and friction
    model. Reuses the Milestone 1 per-bolt loads unchanged; does not
    recompute bolt-group mechanics or Milestone 2 strength results.
    """
    C = stiffness.C
    per_bolt = tuple(
        _assess_single_bolt_preload(b.index, b.axial_total, b.shear_resultant, preload.preload_per_bolt, C, friction)
        for b in group_load_result.bolts
    )

    n = len(per_bolt)
    total_preload = preload.preload_per_bolt * n
    total_remaining_clamp_force = sum(b.remaining_clamp_force for b in per_bolt)
    total_effective_clamp_force = sum(max(b.remaining_clamp_force, 0.0) for b in per_bolt)

    all_locations_closed = all(b.separation_pass for b in per_bolt)
    all_locations_no_slip = all(b.slip_pass for b in per_bolt)

    sep_margins = [b.separation_margin for b in per_bolt]
    slip_margins = [b.slip_margin for b in per_bolt]
    gov_sep_idx = _select_governing_min(sep_margins)
    gov_slip_idx = _select_governing_min(slip_margins)

    return BoltPreloadGroupResult(
        bolts=per_bolt,
        load_fraction=C,
        total_preload=total_preload,
        total_remaining_clamp_force=total_remaining_clamp_force,
        total_effective_clamp_force=total_effective_clamp_force,
        all_locations_closed=all_locations_closed,
        all_locations_no_slip=all_locations_no_slip,
        governing_separation_bolt_index=gov_sep_idx,
        governing_slip_bolt_index=gov_slip_idx,
        min_separation_margin=sep_margins[gov_sep_idx],
        min_slip_margin=slip_margins[gov_slip_idx],
        passed=all_locations_closed and all_locations_no_slip,
    )


def required_preload(
    group_load_result: BoltGroupResult, stiffness: JointStiffness, friction: FrictionModel
) -> RequiredPreloadResult:
    """Analytically derive the minimum per-bolt preload that prevents
    local separation and local slip at every bolt location, from:

        F_preload_required_sep  = max_i[(1-C) * T_sep,i]
        F_preload_required_slip = max_i[(1-C) * T_sep,i + V_i / (mu*n_interfaces)]
        F_preload_required      = max(F_preload_required_sep, F_preload_required_slip)

    The slip expression already contains the separation term for the
    same bolt, so slip_required >= separation_required always; ties
    between the two constraints are reported as "slip" (documented
    convention -- slip is the more conservative/inclusive check).
    """
    C = stiffness.C
    mu_n = friction.friction_coefficient * friction.number_of_faying_surfaces

    T_sep = [max(b.axial_total, 0.0) for b in group_load_result.bolts]
    sep_terms = [(1.0 - C) * t for t in T_sep]
    slip_terms = [sep_terms[i] + group_load_result.bolts[i].shear_resultant / mu_n for i in range(len(T_sep))]

    sep_idx = _select_governing_max(sep_terms)
    slip_idx = _select_governing_max(slip_terms)
    sep_val = sep_terms[sep_idx]
    slip_val = slip_terms[slip_idx]

    if slip_val >= sep_val:
        overall_val, overall_idx, constraint = slip_val, slip_idx, "slip"
    else:
        overall_val, overall_idx, constraint = sep_val, sep_idx, "separation"

    return RequiredPreloadResult(
        separation_required=sep_val,
        separation_governing_bolt=sep_idx,
        slip_required=slip_val,
        slip_governing_bolt=slip_idx,
        overall_required=overall_val,
        overall_governing_bolt=overall_idx,
        overall_governing_constraint=constraint,
    )


def apply_preload_factor(required_preload_value: float, preload_factor: float = 1.0) -> float:
    """Scale a required-preload value by an explicit design reserve
    factor (>= 1.0). Kept separate from `required_preload()` so the
    bare analytical requirement and any applied design margin are never
    conflated."""
    if not math.isfinite(preload_factor) or preload_factor < 1.0:
        raise ValueError(f"preload_factor must be finite and >= 1.0, got {preload_factor}.")
    if not math.isfinite(required_preload_value) or required_preload_value < 0.0:
        raise ValueError(f"required_preload_value must be finite and >= 0, got {required_preload_value}.")
    return required_preload_value * preload_factor
