"""Torque-to-preload sizing, installation scatter, and installation-
robustness screening (Milestone 6).

Milestones 3-5 reason directly in terms of achieved bolt preload
(`PreloadState.preload_per_bolt`), without asking how that preload
would actually be installed. This module introduces a transparent
nut-factor torque-preload model, asks how friction (nut-factor)
uncertainty maps into achieved-preload uncertainty for a FIXED applied
torque, and determines whether a single installation torque can
robustly keep the achieved preload inside the Milestone 4/5 target
preload window for every nut-factor value in a declared uncertainty
range.

This is still a reduced-order CONCEPTUAL joint-sizing study. It is
explicitly NOT a production torque specification, a qualification
procedure, a statistical process-capability analysis, a detailed
threaded-contact model, or a certified installation requirement.

It does NOT recompute or modify Milestone 1-5 mechanics:
`BoltGroupResult`, `BoltGroupStrengthResult`, `RequiredPreloadResult`,
`InstallationPreloadWindow` / `PreloadFeasibilityResult`, and
`BoltSelectionResult` / `BoltCandidateTradeResult` are all consumed
exactly as produced by their own modules. Nothing here modifies the
Milestone 3-5 required-preload or preload-window equations.

Source audit (sources actually inspected this milestone)
----------------------------------------------------------
- **engineeringlibrary.org, "Preloaded Bolted Joint Analysis
  Methodology (NASA)"** (readable technical reference; the same source
  already cited in Milestone 4 for the joint stiffness-ratio concept)
  -- states the torque-preload relation directly as
  `Po = (T / K*D) * (1.0 +/- u)`, i.e. algebraically the classical
  nut-factor form `T = K * F * D` used below (`u` there is a preload-
  uncertainty fraction, not reused here -- Milestone 6 instead
  propagates a K range directly, per this milestone's explicit
  guidance). It gives illustrative nut-factor ranges: lubricated
  fasteners K ~= 0.11-0.15, unlubricated fasteners K ~= 0.2, and
  states "for a hand-operated torque wrench used on a lubricated
  fastener, preload uncertainty is +/-25 percent," reducible to "+/-5
  percent" with instrumented/load-sensing methods (consistent with the
  same figures already used for Milestone 4's `scatter_allowance`).
- **Web-search-derived summary of general nut-factor engineering
  references** (Machine Design magazine and similar; not a single
  primary document read directly -- reported as a secondary,
  corroborating data point): dry/as-received steel fasteners commonly
  K ~= 0.20; lubricated (engine oil, machine oil, anti-seize, wax-based
  thread compound) commonly K ~= 0.15-0.17; well-lubricated joints
  reported near K ~= 0.12 in some published tables; preload uncertainty
  commonly cited as +/-35% unlubricated, +/-25% lubricated.

Illustrative baseline assumptions chosen after this audit (all labeled
illustrative, not claimed to be a sourced hardware/lubrication
specification):

- Baseline nut-factor range `K_min=0.15, K_nom=0.20, K_max=0.25` --
  spanning from the cited lubricated-to-unlubricated range (0.11-0.20)
  up through a conservative margin above the cited dry value (0.20),
  matching this milestone's own suggested illustrative family.
- Sensitivity families: "narrow" K=[0.18,0.22] (representing tighter,
  better-controlled friction/lubrication practice), "wide"
  K=[0.12,0.28] (representing looser installation control), alongside
  the nominal [0.15,0.25] baseline.

Classical nut-factor model and its limitations
------------------------------------------------
`T = K * F * d` is a first-order, empirically-fitted torque-tension
relation, NOT a first-principles thread-mechanics model: it lumps
thread friction, under-head/collar friction, lubrication condition,
surface finish, and installation-method effects into a single
dimensionless coefficient K. It is friction-dominated (small changes in
lubrication or surface condition can shift K substantially, as the
sourced ranges above show) and does not directly measure achieved
preload -- torque control is fundamentally an INDIRECT preload control
method. This module treats K uncertainty deterministically (a declared
[K_min, K_max] range), per this milestone's explicit scope guidance --
it is NOT a statistical process-capability or reliability analysis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .preload import FrictionModel, JointStiffness, PreloadState, required_preload
from .preload_limits import (
    BoltStrengthLimits,
    InstallationPreloadWindow,
    installation_preload_window,
)
from .preload_limits import _max_total_tension_among_closed as _max_closed_tension
from .solver import BoltGroupResult
from .strength import BoltSection


# ---------------------------------------------------------------------------
# Nut-factor model and the T = K*F*d relation
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class NutFactorModel:
    """Illustrative, deterministic nut-factor range for torque-to-
    preload mapping. NOT a statistical distribution -- `k_min`/`k_max`
    are treated as a declared deterministic bound, per this module's
    scope (see module docstring).

    k_min, k_nom, k_max: dimensionless, finite, > 0, with
    `k_min <= k_nom <= k_max` (a zero-width range, `k_min == k_nom ==
    k_max`, is permitted and represents a hypothetical zero-uncertainty
    case).
    """

    k_min: float
    k_nom: float
    k_max: float

    def __post_init__(self):
        for name in ("k_min", "k_nom", "k_max"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"NutFactorModel.{name} must be finite and > 0, got {value}.")
        if not (self.k_min <= self.k_nom <= self.k_max):
            raise ValueError(
                "NutFactorModel requires k_min <= k_nom <= k_max, got "
                f"k_min={self.k_min}, k_nom={self.k_nom}, k_max={self.k_max}."
            )

    @property
    def uncertainty_ratio(self) -> float:
        """K_max / K_min."""
        return self.k_max / self.k_min


def torque_from_preload(preload: float, k: float, diameter: float) -> float:
    """T = K * F * d. Requires preload >= 0, k > 0, diameter > 0."""
    if not math.isfinite(preload) or preload < 0.0:
        raise ValueError(f"preload must be finite and >= 0, got {preload}.")
    if not math.isfinite(k) or k <= 0.0:
        raise ValueError(f"k must be finite and > 0, got {k}.")
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ValueError(f"diameter must be finite and > 0, got {diameter}.")
    return k * preload * diameter


def preload_from_torque(torque: float, k: float, diameter: float) -> float:
    """F = T / (K * d). Requires torque >= 0, k > 0, diameter > 0."""
    if not math.isfinite(torque) or torque < 0.0:
        raise ValueError(f"torque must be finite and >= 0, got {torque}.")
    if not math.isfinite(k) or k <= 0.0:
        raise ValueError(f"k must be finite and > 0, got {k}.")
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ValueError(f"diameter must be finite and > 0, got {diameter}.")
    return torque / (k * diameter)


# ---------------------------------------------------------------------------
# Nominal torque mapping (informational only -- NOT robust to K
# uncertainty; see RobustTorqueWindowResult below for the central M6
# result)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class NominalTorqueWindow:
    """Nominal torque mapping of the Milestone 4/5 target preload window
    at a single fixed K_nom. This is ONLY a nominal mapping -- it is
    NOT a guaranteed/robust torque window, because K uncertainty
    changes the achieved preload for any fixed applied torque (see
    `RobustTorqueWindowResult`)."""

    k_nom: float
    diameter: float
    target_min: float
    target_max: float
    torque_min_nom: float  # K_nom * d * target_min
    torque_max_nom: float  # K_nom * d * target_max


def nominal_torque_window(window: InstallationPreloadWindow, k_nom: float, diameter: float) -> NominalTorqueWindow:
    return NominalTorqueWindow(
        k_nom=k_nom,
        diameter=diameter,
        target_min=window.target_min,
        target_max=window.target_max,
        torque_min_nom=torque_from_preload(window.target_min, k_nom, diameter),
        torque_max_nom=torque_from_preload(window.target_max, k_nom, diameter),
    )


# ---------------------------------------------------------------------------
# Robust torque window -- the central Milestone 6 result
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RobustTorqueWindowResult:
    """Torque window that is robust to the FULL declared nut-factor
    range [k_min, k_max] -- i.e. a single applied torque T in
    [torque_robust_min, torque_robust_max] guarantees the achieved
    preload F = T/(K*d) stays within [target_min, target_max] for
    EVERY K in [k_min, k_max]:

        torque_robust_min = k_max * d * target_min
        torque_robust_max = k_min * d * target_max

    A robust window exists only if torque_robust_min <=
    torque_robust_max. `width` is NEVER clipped at zero -- a negative
    width is reported honestly as an infeasible robust window, not
    silently corrected. Equivalent feasibility identity (independently
    verified in tests):

        k_max / k_min <= target_max / target_min
    """

    nut_factor: NutFactorModel
    diameter: float
    target_min: float
    target_max: float
    torque_robust_min: float
    torque_robust_max: float
    width: float  # torque_robust_max - torque_robust_min, unclipped
    feasible: bool
    preload_window_ratio: float  # target_max / target_min
    k_uncertainty_ratio: float  # k_max / k_min


def robust_torque_window(window: InstallationPreloadWindow, nut_factor: NutFactorModel, diameter: float) -> RobustTorqueWindowResult:
    torque_robust_min = torque_from_preload(window.target_min, nut_factor.k_max, diameter)
    torque_robust_max = torque_from_preload(window.target_max, nut_factor.k_min, diameter)
    width = torque_robust_max - torque_robust_min
    preload_window_ratio = window.target_max / window.target_min if window.target_min > 0.0 else math.inf
    return RobustTorqueWindowResult(
        nut_factor=nut_factor,
        diameter=diameter,
        target_min=window.target_min,
        target_max=window.target_max,
        torque_robust_min=torque_robust_min,
        torque_robust_max=torque_robust_max,
        width=width,
        feasible=(torque_robust_min <= torque_robust_max),
        preload_window_ratio=preload_window_ratio,
        k_uncertainty_ratio=nut_factor.uncertainty_ratio,
    )


# ---------------------------------------------------------------------------
# Statuses
# ---------------------------------------------------------------------------


class TorqueInstallationStatus(str, Enum):
    FEASIBLE = "FEASIBLE"
    NO_ROBUST_TORQUE_WINDOW = "NO_ROBUST_TORQUE_WINDOW"
    TORQUE_TOO_LOW = "TORQUE_TOO_LOW"
    TORQUE_TOO_HIGH = "TORQUE_TOO_HIGH"
    PRELOAD_BELOW_TARGET = "PRELOAD_BELOW_TARGET"
    PRELOAD_ABOVE_TARGET = "PRELOAD_ABOVE_TARGET"
    PROOF_LIMIT_EXCEEDED = "PROOF_LIMIT_EXCEEDED"
    INVALID_INPUT = "INVALID_INPUT"


# ---------------------------------------------------------------------------
# Nominal torque target + achieved-preload spread + in-service proof carry-forward
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TorqueInstallationResult:
    """End-to-end Milestone 6 result: robust torque window, a
    predeclared-rule nominal torque target (midpoint of the robust
    window) when one exists, the achieved-preload spread across the
    full nut-factor range at that torque, and the Milestone 3/4
    in-service bolt-force / proof-reserve carry-forward evaluated at
    the highest achieved preload (occurring at k_min) -- the bounding
    case for a proof-limit exceedance.

    `nominal_torque` and the achieved-preload/in-service fields are
    `None` when no robust torque window exists -- no torque target is
    ever invented in that case.
    """

    robust_window: RobustTorqueWindowResult
    nominal_torque: Optional[float]
    achieved_preload_at_k_min: Optional[float]  # highest achieved preload (lowest friction)
    achieved_preload_at_k_nom: Optional[float]
    achieved_preload_at_k_max: Optional[float]  # lowest achieved preload (highest friction)
    all_within_target_window: Optional[bool]
    max_in_service_bolt_force: Optional[float]
    max_in_service_bolt_index: Optional[int]
    in_service_model_valid: Optional[bool]
    proof_load: Optional[float]
    proof_reserve: Optional[float]
    status: TorqueInstallationStatus
    message: str


def assess_torque_installation(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    nut_factor: NutFactorModel,
) -> TorqueInstallationResult:
    """Central Milestone 6 assessment for ONE candidate bolt.

    Internally reuses, unchanged: `required_preload()` (Milestone 3),
    `installation_preload_window()` (Milestone 4) for the target
    preload bounds, and the Milestone 3 closed-joint total-bolt-tension
    formula (via `assess_preloaded_joint`, imported at call time to
    avoid a module-level circular import) for the in-service carry-
    forward. Does not recompute Milestone 1-5 mechanics.

    Predeclared rule (declared before evaluating any candidate; see
    README "Milestone 6" and module docstring): if a robust torque
    window exists, the nominal torque target is the ARITHMETIC MIDPOINT
    of [torque_robust_min, torque_robust_max]. This guarantees (proven
    in the module docstring / verified in tests) that the achieved
    preload at EVERY K in [k_min, k_max] -- not only at the extremes --
    falls within [target_min, target_max].
    """
    required = required_preload(group_load_result, stiffness, friction)
    window = installation_preload_window(
        required.overall_required, bolt_section, strength_limits, eta_proof, scatter_allowance
    )
    diameter = bolt_section.nominal_diameter
    robust = robust_torque_window(window, nut_factor, diameter)

    if not robust.feasible:
        return TorqueInstallationResult(
            robust_window=robust,
            nominal_torque=None,
            achieved_preload_at_k_min=None,
            achieved_preload_at_k_nom=None,
            achieved_preload_at_k_max=None,
            all_within_target_window=None,
            max_in_service_bolt_force=None,
            max_in_service_bolt_index=None,
            in_service_model_valid=None,
            proof_load=None,
            proof_reserve=None,
            status=TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW,
            message=(
                f"No torque exists that robustly keeps the achieved preload within "
                f"[{window.target_min:,.1f}, {window.target_max:,.1f}] N for every K in "
                f"[{nut_factor.k_min:.3f}, {nut_factor.k_max:.3f}] "
                f"(K_max/K_min={nut_factor.uncertainty_ratio:.4f} > "
                f"preload-window ratio={robust.preload_window_ratio:.4f}). No target torque is proposed."
            ),
        )

    t_nom = 0.5 * (robust.torque_robust_min + robust.torque_robust_max)
    f_at_kmin = preload_from_torque(t_nom, nut_factor.k_min, diameter)
    f_at_knom = preload_from_torque(t_nom, nut_factor.k_nom, diameter)
    f_at_kmax = preload_from_torque(t_nom, nut_factor.k_max, diameter)

    all_within = all(
        (window.target_min <= f <= window.target_max) for f in (f_at_kmin, f_at_knom, f_at_kmax)
    )

    # In-service proof carry-forward at the HIGHEST achieved preload
    # (occurring at k_min), the bounding case for a proof exceedance --
    # exactly one preload term, no double-counting.
    from .preload import assess_preloaded_joint

    preload_state = PreloadState(preload_per_bolt=f_at_kmin, label="M6 torque-installed (K_min bound)")
    group_result = assess_preloaded_joint(group_load_result, preload_state, stiffness, friction)
    max_force, max_index, in_service_valid = _max_closed_tension(group_result.bolts)

    f_proof = window.limits.proof_load
    proof_reserve = f_proof / max_force - 1.0 if max_force > 0.0 else None
    proof_violated = max_force > f_proof

    if proof_violated:
        status = TorqueInstallationStatus.PROOF_LIMIT_EXCEEDED
        message = (
            f"Robust torque window exists (T in [{robust.torque_robust_min:.2f}, "
            f"{robust.torque_robust_max:.2f}] N*m) and nominal torque {t_nom:.2f} N*m "
            f"keeps achieved preload within target bounds, BUT the resulting maximum "
            f"in-service bolt force ({max_force:,.1f} N, bolt {max_index}) exceeds the "
            f"proof load ({f_proof:,.1f} N) at the K_min (lowest-friction) bound."
        )
    elif not all_within:
        # Should not occur given the midpoint-rule proof above; reported
        # explicitly rather than silently trusted.
        status = TorqueInstallationStatus.PRELOAD_ABOVE_TARGET
        message = (
            "Achieved preload at one or more K values fell outside the target window "
            "at the nominal torque -- unexpected given the robust-window construction; "
            "reported explicitly rather than silently accepted."
        )
    else:
        status = TorqueInstallationStatus.FEASIBLE
        message = (
            f"Robust torque window T in [{robust.torque_robust_min:.2f}, "
            f"{robust.torque_robust_max:.2f}] N*m exists; nominal torque {t_nom:.2f} N*m "
            f"keeps achieved preload within [{window.target_min:,.1f}, {window.target_max:,.1f}] N "
            f"for every K in [{nut_factor.k_min:.3f}, {nut_factor.k_max:.3f}], and the resulting "
            f"maximum in-service bolt force ({max_force:,.1f} N) remains below the proof load "
            f"({f_proof:,.1f} N)."
        )

    return TorqueInstallationResult(
        robust_window=robust,
        nominal_torque=t_nom,
        achieved_preload_at_k_min=f_at_kmin,
        achieved_preload_at_k_nom=f_at_knom,
        achieved_preload_at_k_max=f_at_kmax,
        all_within_target_window=all_within,
        max_in_service_bolt_force=max_force,
        max_in_service_bolt_index=max_index,
        in_service_model_valid=in_service_valid,
        proof_load=f_proof,
        proof_reserve=proof_reserve,
        status=status,
        message=message,
    )


# ---------------------------------------------------------------------------
# Historical M3 preload back-calculation (diagnostic only)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TorqueBackCalculation:
    """Torque required to produce a GIVEN preload (e.g. the historical
    Milestone 3 selected preload) at k_min/k_nom/k_max. A diagnostic
    only -- never labeled a production torque specification."""

    preload: float
    diameter: float
    nut_factor: NutFactorModel
    torque_at_k_min: float
    torque_at_k_nom: float
    torque_at_k_max: float


def back_calculate_torque(preload: float, diameter: float, nut_factor: NutFactorModel) -> TorqueBackCalculation:
    return TorqueBackCalculation(
        preload=preload,
        diameter=diameter,
        nut_factor=nut_factor,
        torque_at_k_min=torque_from_preload(preload, nut_factor.k_min, diameter),
        torque_at_k_nom=torque_from_preload(preload, nut_factor.k_nom, diameter),
        torque_at_k_max=torque_from_preload(preload, nut_factor.k_max, diameter),
    )
