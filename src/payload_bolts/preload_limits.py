"""Preload feasibility, proof/yield screening, and installation-preload
window (Milestone 4).

This module closes the gap left open by Milestone 3: M3 computes the
preload *required* to prevent joint separation and interface slip, but
explicitly does not check whether that preload is structurally feasible
for the selected bolt, nor does it define an allowable *installation*
preload range. This module adds that layer on top of the unchanged
Milestone 1-3 results.

It does **not** recompute or modify:

- Milestone 1 bolt-group geometry, direct/torsional shear, overturning
  axial distribution, or equilibrium verification;
- Milestone 2 bolt tensile/shear strength margins, the quadratic
  interaction criterion, or its governing-mode tie-break convention;
- Milestone 3 load-fraction C, separation/slip screening, or the
  analytical required-preload equations (`required_preload`,
  `assess_preloaded_joint` are called here exactly as-is, never
  reimplemented).

Scope: bolt proof/yield load from the tensile stress area already
established in Milestone 2 (`BoltSection.tensile_area` -- no new/
conflicting stress-area convention is introduced here); an allowable
installation-preload ceiling expressed as a fraction of proof load; a
deterministic installation-preload scatter/loss allowance that inflates
the Milestone 3 required preload into a minimum installation target; the
resulting installation-preload window (may be infeasible -- never
forced to be feasible); classification of a selected preload against
that window; and a maximum in-service bolt-tension screen against
proof/yield load using the Milestone 3 closed-joint load-sharing
formula, restricted to bolts that remain closed.

This is a conceptual/preliminary joint-sizing screen, **not** a torque
specification, qualification procedure, certification analysis, or
detailed threaded-joint design. Explicitly NOT modeled here (deferred):
torque-tension relationship / nut factor / torque coefficient /
lubrication / thread friction / under-head friction; preload
relaxation/embedment; thermal preload change; fatigue; prying; bearing;
thread stripping; nonlinear joint opening beyond the linear closed-joint
model already used in Milestone 3; proof testing; certification.

Source audit (see README.md / this module for the summary; sources
actually inspected this milestone):

- mechanicalc.com, "Bolted Joint Analysis" (readable technical
  reference) -- preload recommended as a fraction of proof load
  (~50% non-permanent/reusable, ~75% semi-permanent, ~90% permanent
  connections); tensile stress-area formula for threaded fasteners;
  torque-based preload uncertainty (~+/-25% for a hand torque wrench on
  a lubricated fastener, vs ~+/-3-5% for elongation/load-sensing
  methods) and the torque-coefficient variables (thread friction,
  under-head/collar friction) that make torque an inherently indirect,
  scattered preload indicator.
- engineeringlibrary.org, "Preloaded Bolted Joint Analysis Methodology
  (NASA)" (readable technical reference) -- fasteners commonly
  preloaded to "65 to 90 percent of yield strength"; the joint
  load-fraction/stiffness-ratio concept `phi = Kb/(Kb+Kj)`, which is
  algebraically the same construct as Milestone 3's `C =
  k_b/(k_b+k_m)`, corroborating the Milestone 3 formulation; a
  hand-torque-wrench preload uncertainty of "+/-25 percent" (reducible
  to "+/-5 percent" with load-sensing/instrumented methods).
- Web-search-derived summary attributed to Shigley, *Mechanical
  Engineering Design* (9th ed., p. 442) -- for reused/non-permanent
  connections, recommended preload `Fi = 0.75*Fp` (Fp = proof load);
  proof strength approximately 0.85x tensile yield strength. This was
  NOT read from the primary text directly (only a search-engine
  summary was available); it is reported here as a secondary,
  corroborating data point, not an independently verified primary-source
  quote.
- NASA-STD-5020 was located (PDF) but returned as unreadable binary
  content in this environment and its specific numeric requirements
  were therefore **not** independently verified; it is not cited for
  any specific numeric value in this module. Only the readable sources
  above are used to justify the illustrative baseline assumptions
  below.

Illustrative baseline assumptions chosen after this audit (all labeled
illustrative, not claimed to be sourced fastener-grade allowables):

- `eta_proof = 0.75` (mid-range of the ~50-90% proof-load fraction cited
  above; matches the cited reused/non-permanent-connection convention).
- `scatter_allowance (delta_F) = 0.10` (10%), representing a moderately
  controlled installation method -- well inside the ~+/-25% hand-torque-
  wrench uncertainty cited above, and above the ~3-5% achievable with
  elongation/load-sensing methods. This is a deterministic installation
  allowance, **not** a statistical confidence interval.
- Illustrative `BoltStrengthLimits`: proof strength 830 MPa, yield
  strength 970 MPa (ratio ~0.86, consistent with the cited ~0.85 proof-
  to-yield ratio). These are distinct from, and not to be confused
  with, Milestone 2's `BoltMaterial.tensile_allowable` (800 MPa), which
  is a simple working-stress allowable for M2's margin checks, not a
  proof or yield strength -- the two properties are never conflated in
  this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from .preload import FrictionModel, JointStiffness, PreloadState, assess_preloaded_joint, required_preload
from .solver import BoltGroupResult
from .strength import BoltSection


@dataclass(frozen=True)
class BoltStrengthLimits:
    """Illustrative bolt proof/yield strength properties for Milestone 4
    preload feasibility screening.

    This is a DISTINCT property set from Milestone 2's `BoltMaterial`
    (tensile_allowable/shear_allowable), which represents a simple
    working-stress allowable for M2's margin checks -- not a proof or
    yield strength. The two are never conflated or substituted for one
    another in this module.

    proof_strength (S_p): Pa, finite, > 0.
    yield_strength (S_y): Pa, optional. If supplied, must be finite,
        > 0, and >= proof_strength (proof strength is conventionally a
        fraction, ~0.85, of yield strength per the source audit above;
        this module enforces S_y >= S_p as the physically consistent
        ordering unless a caller has an independently justified reason
        to model otherwise, in which case this dataclass should not be
        used as-is).
    """

    name: str
    proof_strength: float
    yield_strength: Optional[float] = None

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("BoltStrengthLimits.name must be a non-empty string.")
        if not math.isfinite(self.proof_strength) or self.proof_strength <= 0:
            raise ValueError(
                f"BoltStrengthLimits.proof_strength must be finite and > 0, got {self.proof_strength}."
            )
        if self.yield_strength is not None:
            if not math.isfinite(self.yield_strength) or self.yield_strength <= 0:
                raise ValueError(
                    f"BoltStrengthLimits.yield_strength must be finite and > 0, got {self.yield_strength}."
                )
            if self.yield_strength < self.proof_strength:
                raise ValueError(
                    "BoltStrengthLimits.yield_strength must be >= proof_strength "
                    f"(got yield_strength={self.yield_strength}, proof_strength={self.proof_strength})."
                )


@dataclass(frozen=True)
class PreloadLimitResult:
    """Proof/yield load basis for a given bolt section and strength
    limits: F = S * A_t, using the EXACT tensile stress area already
    established by Milestone 2's `BoltSection.tensile_area` (no new or
    conflicting stress-area convention is introduced here)."""

    bolt_section: BoltSection
    strength_limits: BoltStrengthLimits
    proof_load: float  # F_proof = S_p * A_t
    yield_load: Optional[float]  # F_yield = S_y * A_t, or None


def compute_preload_limits(section: BoltSection, limits: BoltStrengthLimits) -> PreloadLimitResult:
    """F_proof = S_p * A_t; F_yield = S_y * A_t if yield_strength given."""
    proof = limits.proof_strength * section.tensile_area
    yld = limits.yield_strength * section.tensile_area if limits.yield_strength is not None else None
    return PreloadLimitResult(bolt_section=section, strength_limits=limits, proof_load=proof, yield_load=yld)


class PreloadFeasibilityStatus(str, Enum):
    """Deterministic Milestone 4 feasibility statuses.

    Governing-status priority (most severe first) when more than one
    diagnostic applies simultaneously -- see `PreloadFeasibilityResult`:

        NO_INSTALLATION_WINDOW
        SELECTED_PRELOAD_TOO_LOW
        SELECTED_PRELOAD_TOO_HIGH
        PROOF_LIMIT_EXCEEDED_IN_SERVICE
        FEASIBLE

    `INVALID_INPUT` is reserved for API completeness; this module
    validates inputs via exceptions at construction time rather than
    returning this status from a normal assessment call.
    """

    FEASIBLE = "FEASIBLE"
    NO_INSTALLATION_WINDOW = "NO_INSTALLATION_WINDOW"
    SELECTED_PRELOAD_TOO_LOW = "SELECTED_PRELOAD_TOO_LOW"
    SELECTED_PRELOAD_TOO_HIGH = "SELECTED_PRELOAD_TOO_HIGH"
    PROOF_LIMIT_EXCEEDED_IN_SERVICE = "PROOF_LIMIT_EXCEEDED_IN_SERVICE"
    INVALID_INPUT = "INVALID_INPUT"


_STATUS_PRIORITY = (
    PreloadFeasibilityStatus.NO_INSTALLATION_WINDOW,
    PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_LOW,
    PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_HIGH,
    PreloadFeasibilityStatus.PROOF_LIMIT_EXCEEDED_IN_SERVICE,
    PreloadFeasibilityStatus.FEASIBLE,
)


@dataclass(frozen=True)
class InstallationPreloadWindow:
    """Allowable installation-preload window.

    target_min = F_required / (1 - scatter_allowance)   [guarantees the
        Milestone 3 required preload is met even at the minimum end of
        the deterministic installation scatter/loss allowance]
    target_max = eta_proof * F_proof                     [proof-based
        installation ceiling]

    A feasible window exists only if target_min <= target_max.
    `window_width` is NEVER clipped at zero -- a negative width is
    reported honestly as an infeasible window, not silently corrected.
    """

    required_preload: float  # F_required, unchanged from Milestone 3
    scatter_allowance: float  # delta_F
    target_min: float  # F_required / (1 - delta_F)
    limits: PreloadLimitResult
    eta_proof: float
    target_max: float  # eta_proof * F_proof
    window_width: float  # target_max - target_min, unclipped (may be negative)
    window_width_normalized: Optional[float]  # window_width / target_min, or None if target_min <= 0
    upper_margin: Optional[float]  # target_max / target_min - 1, or None if target_min <= 0
    feasible: bool  # target_min <= target_max


def min_installation_preload(required_preload_value: float, scatter_allowance: float) -> float:
    """F_target,min = F_required / (1 - delta_F).

    Requires required_preload_value finite and >= 0, and
    0 <= scatter_allowance < 1 (deterministic installation allowance,
    not a statistical confidence interval).
    """
    if not math.isfinite(required_preload_value) or required_preload_value < 0.0:
        raise ValueError(
            f"required_preload_value must be finite and >= 0, got {required_preload_value}."
        )
    if not math.isfinite(scatter_allowance) or not (0.0 <= scatter_allowance < 1.0):
        raise ValueError(f"scatter_allowance must satisfy 0 <= delta_F < 1, got {scatter_allowance}.")
    return required_preload_value / (1.0 - scatter_allowance)


def max_installation_preload(limits_result: PreloadLimitResult, eta_proof: float) -> float:
    """F_target,max = eta_proof * F_proof. Requires 0 < eta_proof < 1."""
    if not math.isfinite(eta_proof) or not (0.0 < eta_proof < 1.0):
        raise ValueError(f"eta_proof must satisfy 0 < eta_proof < 1, got {eta_proof}.")
    return eta_proof * limits_result.proof_load


def installation_preload_window(
    required_preload_value: float,
    section: BoltSection,
    strength_limits: BoltStrengthLimits,
    eta_proof: float,
    scatter_allowance: float,
) -> InstallationPreloadWindow:
    """Build the Milestone 4 installation-preload window from a
    Milestone 3 required-preload value, a Milestone 2 bolt section (for
    its EXACT tensile stress area), illustrative proof/yield strength
    limits, a proof-load installation fraction `eta_proof`, and a
    deterministic preload scatter/loss allowance `scatter_allowance`.

    The window is reported honestly -- if target_min > target_max, the
    window is INFEASIBLE and `window_width` is reported as its true
    (negative) value, never clipped to zero.
    """
    limits_result = compute_preload_limits(section, strength_limits)
    target_min = min_installation_preload(required_preload_value, scatter_allowance)
    target_max = max_installation_preload(limits_result, eta_proof)

    width = target_max - target_min
    if target_min > 0.0:
        width_normalized = width / target_min
        upper_margin = target_max / target_min - 1.0
    else:
        width_normalized = None
        upper_margin = None

    return InstallationPreloadWindow(
        required_preload=required_preload_value,
        scatter_allowance=scatter_allowance,
        target_min=target_min,
        limits=limits_result,
        eta_proof=eta_proof,
        target_max=target_max,
        window_width=width,
        window_width_normalized=width_normalized,
        upper_margin=upper_margin,
        feasible=(target_min <= target_max),
    )


def classify_selected_preload(window: InstallationPreloadWindow, selected_preload: float) -> PreloadFeasibilityStatus:
    """Classify a selected per-bolt preload against the installation
    window ONLY (no in-service proof check here -- see
    `assess_preload_feasibility` for the combined governing status)."""
    if not math.isfinite(selected_preload) or selected_preload <= 0.0:
        raise ValueError(f"selected_preload must be finite and > 0, got {selected_preload}.")
    if not window.feasible:
        return PreloadFeasibilityStatus.NO_INSTALLATION_WINDOW
    if selected_preload < window.target_min:
        return PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_LOW
    if selected_preload > window.target_max:
        return PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_HIGH
    return PreloadFeasibilityStatus.FEASIBLE


def _max_total_tension_among_closed(bolts) -> Tuple[float, int, bool]:
    """Governing (max total_bolt_tension, index, all_closed_valid) among
    bolts that remain closed (separation_pass). Ties -> lowest index.
    If NO bolt remains closed (pathological), fall back to the maximum
    over ALL bolts and flag the closed-joint model as invalid there,
    per the requirement to never silently apply the closed-joint linear
    load-sharing formula outside its valid (closed-joint) regime."""
    closed = [b for b in bolts if b.separation_pass]
    pool = closed if closed else list(bolts)
    best_val = max(b.total_bolt_tension for b in pool)
    tied = [b for b in pool if b.total_bolt_tension == best_val]
    governing = min(tied, key=lambda b: b.index)
    return governing.total_bolt_tension, governing.index, bool(closed)


@dataclass(frozen=True)
class PreloadFeasibilityResult:
    """Combined Milestone 4 result: installation window, selected-preload
    classification, and maximum in-service bolt-force proof/yield screen.

    `status` is the single governing status (see
    `PreloadFeasibilityStatus` priority order). `diagnostics` lists
    every applicable status simultaneously (window classification plus,
    independently, the in-service proof exceedance flag if present), so
    no information is hidden behind the single governing flag.
    """

    window: InstallationPreloadWindow
    selected_preload: float
    window_status: PreloadFeasibilityStatus  # window-only classification
    max_in_service_bolt_force: float
    max_in_service_bolt_index: int
    in_service_model_valid: bool  # False if no bolt remained closed at selected_preload
    proof_reserve: Optional[float]  # F_proof / max_in_service_bolt_force - 1
    yield_load: Optional[float]
    yield_reserve: Optional[float]  # F_yield / max_in_service_bolt_force - 1, or None
    in_service_exceeds_proof: bool
    diagnostics: Tuple[PreloadFeasibilityStatus, ...]
    status: PreloadFeasibilityStatus  # single governing status


def assess_preload_feasibility(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    selected_preload: float,
) -> PreloadFeasibilityResult:
    """End-to-end Milestone 4 assessment for a single bolt candidate.

    Internally reuses, unchanged:
      - `required_preload()` (Milestone 3) for F_required;
      - `assess_preloaded_joint()` (Milestone 3) for the maximum
        in-service bolt tension F_bolt = F_preload + C*T_sep at the
        SELECTED preload, restricted to bolts that remain closed (the
        closed-joint linear load-sharing formula is not applied outside
        its valid regime -- see `_max_total_tension_among_closed`).

    Does not recompute Milestone 1-3 mechanics.
    """
    required = required_preload(group_load_result, stiffness, friction)
    window = installation_preload_window(
        required.overall_required, bolt_section, strength_limits, eta_proof, scatter_allowance
    )
    window_status = classify_selected_preload(window, selected_preload)

    preload_state = PreloadState(preload_per_bolt=selected_preload, label="M4 candidate")
    group_result = assess_preloaded_joint(group_load_result, preload_state, stiffness, friction)
    max_force, max_index, in_service_valid = _max_total_tension_among_closed(group_result.bolts)

    f_proof = window.limits.proof_load
    f_yield = window.limits.yield_load
    proof_reserve = f_proof / max_force - 1.0 if max_force > 0.0 else None
    yield_reserve = (f_yield / max_force - 1.0) if (f_yield is not None and max_force > 0.0) else None
    in_service_exceeds_proof = max_force > f_proof or (f_yield is not None and max_force > f_yield)

    diagnostics: List[PreloadFeasibilityStatus] = [window_status]
    if in_service_exceeds_proof:
        diagnostics.append(PreloadFeasibilityStatus.PROOF_LIMIT_EXCEEDED_IN_SERVICE)

    status = next(s for s in _STATUS_PRIORITY if s in diagnostics)

    return PreloadFeasibilityResult(
        window=window,
        selected_preload=selected_preload,
        window_status=window_status,
        max_in_service_bolt_force=max_force,
        max_in_service_bolt_index=max_index,
        in_service_model_valid=in_service_valid,
        proof_reserve=proof_reserve,
        yield_load=f_yield,
        yield_reserve=yield_reserve,
        in_service_exceeds_proof=in_service_exceeds_proof,
        diagnostics=tuple(diagnostics),
        status=status,
    )
