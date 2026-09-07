"""Direct-preload verification methods and installation-control trade
(Milestone 7).

Milestone 6 showed that torque-only control of the Milestone 5 selected
10 mm candidate has NO robust torque window at the illustrative
baseline nut-factor range -- because torque is only an INDIRECT,
friction-dominated proxy for preload. This module asks the natural
follow-up question: can a method that measures or controls preload more
directly (elongation, ultrasonic time-of-flight, load-sensing washer,
instrumented/strain-gauged bolt, turn-of-nut) achieve a preload-control
window that fits inside the established Milestone 4/5 preload target
window, without relying on nut-factor/friction assumptions at all?

This is still a reduced-order CONCEPTUAL screening study. It is
explicitly NOT a production work instruction, a calibration procedure,
a statistical process-capability study, a qualification plan, or a
certified tightening specification.

It does NOT recompute or modify Milestone 1-6 mechanics:
`BoltGroupResult`, `BoltGroupStrengthResult`, `RequiredPreloadResult`,
`InstallationPreloadWindow` / `PreloadFeasibilityResult`,
`BoltSelectionResult`, and Milestone 6's torque-only
`RobustTorqueWindowResult` / `TorqueInstallationResult` are all
consumed or reported exactly as produced by their own modules --
torque-only uncertainty is NEVER recomputed using this module's
epsilon-based error model; the two models remain distinct throughout.

Source audit (sources actually inspected this milestone)
----------------------------------------------------------
- **engineeringlibrary.org, "Preloaded Bolted Joint Analysis
  Methodology (NASA)"** (readable primary technical reference; the same
  source already cited in Milestones 4 and 6) -- gives a DIRECT,
  SOURCED quote for instrumented/load-sensing bolts: "the preload
  uncertainty factor may be reduced to +/-5 percent," contrasted with
  "+/-25 percent" for a hand-operated torque wrench on a lubricated
  fastener (the figure already used for Milestone 6's torque-only
  reference). This is the ONLY method in this module with a directly
  read, sourced accuracy figure (`INSTRUMENTED_BOLT`, epsilon=0.05,
  `is_illustrative=False`).
- **Web-search-derived summary of bolt-preload-measurement engineering
  references** (not a single primary document read directly --
  reported as secondary, corroborating/illustrative data points only):
  preload-indicating washers commonly cited near +/-10%; bolt elongation
  measurement commonly cited near +/-5%; ultrasonic time-of-flight
  measurement reported across a wide range depending on instrument and
  geometry -- from as tight as ~1% for simple-geometry fasteners under
  controlled lab conditions, to <3% for some "smart bolt" applications,
  up to a more general/conservative ~10% figure; simple torque-only
  measurement corroborated at "no better than +/-30%" in one summarized
  source, consistent with the +/-25% figure already used for Milestone
  6. Given the wide reported spread for ultrasonic measurement and the
  absence of a directly-read primary source pinning down a specific
  instrument/geometry, this module deliberately adopts the MORE
  CONSERVATIVE end of that reported range (+/-10%) as its illustrative
  baseline for `ULTRASONIC`, rather than the more optimistic ~1-3%
  figures also reported -- this choice is stated explicitly rather than
  silently picking the number that makes the method look best.
- **Turn-of-nut / angle-controlled tightening**: no credible,
  independently-read source giving a specific achievable preload
  accuracy was found during this audit (its accuracy depends heavily on
  thread pitch, elastic/plastic torque-angle behavior, and joint
  stiffness in ways this project has no established basis to quantify
  responsibly). Per this milestone's explicit guidance, this method is
  therefore marked `METHOD_NOT_QUANTIFIED` rather than assigned a
  fabricated accuracy figure -- a transparent omission is used instead
  of an invented number, exactly as thread stripping was left
  unmodeled in Milestone 5.

Core deterministic error model
---------------------------------
For a method commanding/verifying a NOMINAL target preload `F_target`
with a deterministic fractional accuracy/error band `epsilon` (`0 <=
epsilon < 1`):

    F_achieved,min = F_target * (1 - epsilon)
    F_achieved,max = F_target * (1 + epsilon)

A target is robustly acceptable only if BOTH achieved bounds stay
inside the inherited Milestone 4/5 preload window
`[F_window,min, F_window,max]`, which rearranges to a feasible COMMAND
window:

    F_command,min = F_window,min / (1 - epsilon)
    F_command,max = F_window,max / (1 + epsilon)

A robust direct-preload-control window exists only if `F_command,min
<= F_command,max`; width is NEVER clipped -- reported honestly negative
when infeasible. The equivalent feasibility identity (independently
verified in tests) is:

    (1 + epsilon) / (1 - epsilon)  <=  F_window,max / F_window,min

and the maximum admissible SYMMETRIC deterministic fractional error the
established preload window can tolerate at all is:

    epsilon_max = (R - 1) / (R + 1),   R = F_window,max / F_window,min

This is a deterministic screening construct, exactly analogous in
spirit to Milestone 6's nut-factor ratio condition but for a directly
declared measurement/control error rather than a friction-driven torque
coefficient -- it is NOT a statistical process-capability analysis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from .preload import FrictionModel, JointStiffness, PreloadState, required_preload
from .preload_limits import BoltStrengthLimits, installation_preload_window
from .preload_limits import _max_total_tension_among_closed as _max_closed_tension
from .solver import BoltGroupResult
from .strength import BoltSection


# ---------------------------------------------------------------------------
# Method / status enums
# ---------------------------------------------------------------------------


class PreloadVerificationMethod(str, Enum):
    TORQUE_ONLY = "TORQUE_ONLY"  # M6 reference row only; not assessed by this module's epsilon model
    BOLT_ELONGATION = "BOLT_ELONGATION"
    ULTRASONIC = "ULTRASONIC"
    LOAD_SENSING_WASHER = "LOAD_SENSING_WASHER"
    INSTRUMENTED_BOLT = "INSTRUMENTED_BOLT"
    TURN_OF_NUT = "TURN_OF_NUT"


class VerificationStatus(str, Enum):
    FEASIBLE = "FEASIBLE"
    NO_VERIFIED_PRELOAD_WINDOW = "NO_VERIFIED_PRELOAD_WINDOW"
    CALIBRATION_REQUIRED = "CALIBRATION_REQUIRED"
    METHOD_NOT_QUANTIFIED = "METHOD_NOT_QUANTIFIED"
    PROOF_LIMIT_EXCEEDED = "PROOF_LIMIT_EXCEEDED"
    INVALID_INPUT = "INVALID_INPUT"


# ---------------------------------------------------------------------------
# Method accuracy models
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VerificationAccuracyModel:
    """One preload-verification method's deterministic accuracy model.

    relative_error (epsilon): dimensionless fractional error band,
        `0 <= epsilon < 1`, or `None` if the method is
        `METHOD_NOT_QUANTIFIED` (no credible accuracy figure available;
        see module docstring).
    source_basis: short text identifying the source (or "illustrative,
        not independently sourced this milestone") -- never left blank.
    is_illustrative: True unless `relative_error` was read directly from
        a primary source (only `INSTRUMENTED_BOLT` in this module).
    measurement_type: "direct" or "indirect" -- what physical quantity
        is actually sensed vs. inferred.
    what_is_measured: short description.
    limitation: short description of the method's main limitation.
    """

    method: PreloadVerificationMethod
    relative_error: Optional[float]
    source_basis: str
    is_illustrative: bool
    measurement_type: str
    what_is_measured: str
    limitation: str

    def __post_init__(self):
        if self.relative_error is not None:
            if not math.isfinite(self.relative_error) or not (0.0 <= self.relative_error < 1.0):
                raise ValueError(
                    f"VerificationAccuracyModel.relative_error must be None or satisfy "
                    f"0 <= epsilon < 1, got {self.relative_error}."
                )
        if not isinstance(self.source_basis, str) or not self.source_basis.strip():
            raise ValueError("VerificationAccuracyModel.source_basis must be a non-empty string.")


# Baseline illustrative/sourced method accuracy models (see module
# docstring for the full source audit behind each figure).
BOLT_ELONGATION_MODEL = VerificationAccuracyModel(
    method=PreloadVerificationMethod.BOLT_ELONGATION,
    relative_error=0.05,
    source_basis="illustrative, corroborated by a web-search summary of bolt-elongation "
    "measurement references (not independently primary-sourced this milestone)",
    is_illustrative=True,
    measurement_type="direct",
    what_is_measured="physical bolt length change under load (micrometer or dial gauge)",
    limitation="requires access to both bolt ends and an accurate unstressed reference length",
)

ULTRASONIC_MODEL = VerificationAccuracyModel(
    method=PreloadVerificationMethod.ULTRASONIC,
    relative_error=0.10,
    source_basis="illustrative, DELIBERATELY CONSERVATIVE choice within a web-search-summarized "
    "range (~1% simple-geometry lab conditions to ~10% general use); not independently "
    "primary-sourced this milestone",
    is_illustrative=True,
    measurement_type="direct",
    what_is_measured="ultrasonic time-of-flight change through the bolt under load",
    limitation="requires calibration to the specific bolt/material and a clean acoustic "
    "coupling; achievable accuracy is highly instrument/geometry dependent",
)

LOAD_SENSING_WASHER_MODEL = VerificationAccuracyModel(
    method=PreloadVerificationMethod.LOAD_SENSING_WASHER,
    relative_error=0.10,
    source_basis="illustrative, corroborated by a web-search summary of load-indicating-washer "
    "references (not independently primary-sourced this milestone)",
    is_illustrative=True,
    measurement_type="direct",
    what_is_measured="local clamp-force-induced deformation/strain at the washer",
    limitation="washer hysteresis/creep and seating repeatability are not modeled here",
)

INSTRUMENTED_BOLT_MODEL = VerificationAccuracyModel(
    method=PreloadVerificationMethod.INSTRUMENTED_BOLT,
    relative_error=0.05,
    source_basis="SOURCED: engineeringlibrary.org NASA preloaded-joint methodology page, "
    '"the preload uncertainty factor may be reduced to +/-5 percent" for instrumented/'
    "load-sensing bolts",
    is_illustrative=False,
    measurement_type="direct",
    what_is_measured="strain-gauge bridge output on the bolt body, calibrated to load",
    limitation="strain-gauge bridge electronics/calibration drift not modeled here",
)

TURN_OF_NUT_MODEL = VerificationAccuracyModel(
    method=PreloadVerificationMethod.TURN_OF_NUT,
    relative_error=None,
    source_basis="NOT QUANTIFIED: no credible independently-read source pinning down an "
    "achievable accuracy was found; angle-controlled tightening accuracy depends heavily "
    "on thread pitch and elastic/plastic torque-angle behavior not established in this "
    "project (see module docstring)",
    is_illustrative=True,
    measurement_type="indirect",
    what_is_measured="nut rotation angle past snug-tight, inferring elongation via thread pitch",
    limitation="accuracy strongly depends on joint stiffness and the elastic/plastic "
    "torque-angle curve, neither of which is modeled in this project",
)

ALL_METHOD_MODELS = (
    BOLT_ELONGATION_MODEL,
    ULTRASONIC_MODEL,
    LOAD_SENSING_WASHER_MODEL,
    INSTRUMENTED_BOLT_MODEL,
    TURN_OF_NUT_MODEL,
)


# ---------------------------------------------------------------------------
# Core error-model math
# ---------------------------------------------------------------------------


def achieved_preload_bounds(target: float, epsilon: float) -> tuple:
    """F_achieved,min = F_target*(1-epsilon), F_achieved,max = F_target*(1+epsilon)."""
    if not math.isfinite(target) or target < 0.0:
        raise ValueError(f"target must be finite and >= 0, got {target}.")
    if not math.isfinite(epsilon) or not (0.0 <= epsilon < 1.0):
        raise ValueError(f"epsilon must satisfy 0 <= epsilon < 1, got {epsilon}.")
    return target * (1.0 - epsilon), target * (1.0 + epsilon)


def _validate_window(window_min: float, window_max: float) -> None:
    if not math.isfinite(window_min) or window_min <= 0.0:
        raise ValueError(f"window_min must be finite and > 0, got {window_min}.")
    if not math.isfinite(window_max) or window_max <= 0.0:
        raise ValueError(f"window_max must be finite and > 0, got {window_max}.")
    if window_max < window_min:
        raise ValueError(f"window_max ({window_max}) must be >= window_min ({window_min}).")


def epsilon_max_for_window(window_min: float, window_max: float) -> float:
    """epsilon_max = (R - 1) / (R + 1), R = window_max/window_min. The
    maximum admissible SYMMETRIC deterministic fractional
    measurement/control error the preload window can tolerate at all."""
    _validate_window(window_min, window_max)
    R = window_max / window_min
    return (R - 1.0) / (R + 1.0)


@dataclass(frozen=True)
class VerifiedPreloadWindow:
    """Feasible COMMAND-preload window for ONE method's deterministic
    error band epsilon, against the inherited Milestone 4/5 preload
    window. `width` is NEVER clipped -- a negative width is reported
    honestly as infeasible."""

    window_min: float
    window_max: float
    epsilon: float
    command_min: float  # window_min / (1 - epsilon)
    command_max: float  # window_max / (1 + epsilon)
    width: float  # command_max - command_min, unclipped
    feasible: bool
    window_ratio: float  # window_max / window_min
    equivalent_error_ratio: float  # (1+epsilon) / (1-epsilon)
    epsilon_max: float  # (R-1)/(R+1)


def command_window(window_min: float, window_max: float, epsilon: float) -> VerifiedPreloadWindow:
    _validate_window(window_min, window_max)
    if not math.isfinite(epsilon) or not (0.0 <= epsilon < 1.0):
        raise ValueError(f"epsilon must satisfy 0 <= epsilon < 1, got {epsilon}.")
    command_min = window_min / (1.0 - epsilon)
    command_max = window_max / (1.0 + epsilon)
    width = command_max - command_min
    return VerifiedPreloadWindow(
        window_min=window_min,
        window_max=window_max,
        epsilon=epsilon,
        command_min=command_min,
        command_max=command_max,
        width=width,
        feasible=(command_min <= command_max),
        window_ratio=window_max / window_min,
        equivalent_error_ratio=(1.0 + epsilon) / (1.0 - epsilon),
        epsilon_max=epsilon_max_for_window(window_min, window_max),
    )


# ---------------------------------------------------------------------------
# Per-method feasibility assessment (nominal target, achieved spread,
# in-service proof carry-forward)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MethodFeasibilityResult:
    accuracy: VerificationAccuracyModel
    verified_window: Optional[VerifiedPreloadWindow]  # None if METHOD_NOT_QUANTIFIED
    nominal_target: Optional[float]
    achieved_min: Optional[float]
    achieved_max: Optional[float]
    all_within_window: Optional[bool]
    max_in_service_bolt_force: Optional[float]
    max_in_service_bolt_index: Optional[int]
    proof_load: Optional[float]
    proof_reserve: Optional[float]
    status: VerificationStatus
    message: str


def assess_method(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    accuracy: VerificationAccuracyModel,
) -> MethodFeasibilityResult:
    """Assess ONE preload-verification method against the inherited
    Milestone 4/5 preload window for a given bolt candidate. Reuses
    `required_preload()` and `installation_preload_window()` exactly
    (Milestones 3/4), and the Milestone 3 closed-joint total-bolt-
    tension formula (via `assess_preloaded_joint`) for the in-service
    carry-forward, identically to Milestone 6's pattern -- never
    recomputed with a different convention.

    Predeclared rule (declared before evaluating any method): if a
    feasible command window exists, the nominal target is the
    ARITHMETIC MIDPOINT of [command_min, command_max]. This guarantees
    (by the same monotonic-bracketing argument as Milestone 6) that
    F_achieved,min and F_achieved,max both fall within
    [window_min, window_max].
    """
    required = required_preload(group_load_result, stiffness, friction)
    window = installation_preload_window(
        required.overall_required, bolt_section, strength_limits, eta_proof, scatter_allowance
    )

    if accuracy.relative_error is None:
        return MethodFeasibilityResult(
            accuracy=accuracy,
            verified_window=None,
            nominal_target=None,
            achieved_min=None,
            achieved_max=None,
            all_within_window=None,
            max_in_service_bolt_force=None,
            max_in_service_bolt_index=None,
            proof_load=None,
            proof_reserve=None,
            status=VerificationStatus.METHOD_NOT_QUANTIFIED,
            message=(
                f"{accuracy.method.value}: no credible accuracy figure available; "
                f"{accuracy.source_basis}. No numeric feasibility result is forced."
            ),
        )

    if not window.feasible:
        # The INHERITED Milestone 4/5 preload window is already
        # infeasible (target_min > target_max) before any measurement
        # error is even considered -- no verification method can fix a
        # prior-milestone infeasibility. Report this honestly rather
        # than passing an inverted window into `command_window` (which
        # validates ordering and would otherwise raise).
        return MethodFeasibilityResult(
            accuracy=accuracy,
            verified_window=None,
            nominal_target=None,
            achieved_min=None,
            achieved_max=None,
            all_within_window=None,
            max_in_service_bolt_force=None,
            max_in_service_bolt_index=None,
            proof_load=None,
            proof_reserve=None,
            status=VerificationStatus.NO_VERIFIED_PRELOAD_WINDOW,
            message=(
                f"{accuracy.method.value}: the inherited Milestone 4/5 preload window itself is "
                f"already infeasible (target_min={window.target_min:,.1f} N > "
                f"target_max={window.target_max:,.1f} N) -- no preload-verification method can "
                f"resolve a prior-milestone infeasibility."
            ),
        )

    epsilon = accuracy.relative_error
    verified = command_window(window.target_min, window.target_max, epsilon)

    if not verified.feasible:
        return MethodFeasibilityResult(
            accuracy=accuracy,
            verified_window=verified,
            nominal_target=None,
            achieved_min=None,
            achieved_max=None,
            all_within_window=None,
            max_in_service_bolt_force=None,
            max_in_service_bolt_index=None,
            proof_load=None,
            proof_reserve=None,
            status=VerificationStatus.NO_VERIFIED_PRELOAD_WINDOW,
            message=(
                f"{accuracy.method.value}: at epsilon={epsilon:.3f}, no target preload exists "
                f"whose achieved range [{1-epsilon:.2f}x,{1+epsilon:.2f}x] stays within "
                f"[{window.target_min:,.1f}, {window.target_max:,.1f}] N "
                f"(command window width {verified.width:,.1f} N; requires epsilon <= "
                f"{verified.epsilon_max:.4f})."
            ),
        )

    f_nom = 0.5 * (verified.command_min + verified.command_max)
    f_min, f_max = achieved_preload_bounds(f_nom, epsilon)
    all_within = (window.target_min <= f_min) and (f_max <= window.target_max)

    from .preload import assess_preloaded_joint

    preload_state = PreloadState(preload_per_bolt=f_max, label=f"M7 {accuracy.method.value} (achieved max)")
    group_result = assess_preloaded_joint(group_load_result, preload_state, stiffness, friction)
    max_force, max_index, _valid = _max_closed_tension(group_result.bolts)

    f_proof = window.limits.proof_load
    proof_reserve = f_proof / max_force - 1.0 if max_force > 0.0 else None
    proof_violated = max_force > f_proof

    if proof_violated:
        status = VerificationStatus.PROOF_LIMIT_EXCEEDED
        message = (
            f"{accuracy.method.value}: feasible command window exists and nominal target "
            f"{f_nom:,.1f} N keeps achieved preload within target bounds, BUT the resulting "
            f"maximum in-service bolt force ({max_force:,.1f} N, bolt {max_index}) exceeds the "
            f"proof load ({f_proof:,.1f} N)."
        )
    elif not all_within:
        status = VerificationStatus.CALIBRATION_REQUIRED
        message = (
            f"{accuracy.method.value}: achieved preload at the midpoint target fell outside the "
            "window -- unexpected given the command-window construction; reported explicitly."
        )
    else:
        status = VerificationStatus.FEASIBLE
        message = (
            f"{accuracy.method.value}: feasible command window [{verified.command_min:,.1f}, "
            f"{verified.command_max:,.1f}] N (width {verified.width:,.1f} N); nominal target "
            f"{f_nom:,.1f} N keeps achieved preload within [{window.target_min:,.1f}, "
            f"{window.target_max:,.1f}] N, and the resulting maximum in-service bolt force "
            f"({max_force:,.1f} N) remains below the proof load ({f_proof:,.1f} N)."
        )

    return MethodFeasibilityResult(
        accuracy=accuracy,
        verified_window=verified,
        nominal_target=f_nom,
        achieved_min=f_min,
        achieved_max=f_max,
        all_within_window=all_within,
        max_in_service_bolt_force=max_force,
        max_in_service_bolt_index=max_index,
        proof_load=f_proof,
        proof_reserve=proof_reserve,
        status=status,
        message=message,
    )


# ---------------------------------------------------------------------------
# Group-level method-comparison trade result
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class InstallationMethodTradeResult:
    """Comparison of every quantified method (in the fixed
    `ALL_METHOD_MODELS` order) for one bolt candidate. Torque-only (M6)
    is deliberately NOT included here -- it is reported as a separate
    reference alongside this table, never recomputed with this
    module's epsilon model (see module docstring)."""

    bolt_section: BoltSection
    results: tuple  # Tuple[MethodFeasibilityResult, ...], fixed ALL_METHOD_MODELS order
    any_feasible: bool
    feasible_methods: tuple  # Tuple[PreloadVerificationMethod, ...]


def evaluate_installation_methods(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    methods: Optional[tuple] = None,
) -> InstallationMethodTradeResult:
    """Evaluate every method in `methods` (default: `ALL_METHOD_MODELS`,
    in that fixed, deterministic order) for one bolt candidate."""
    models = methods if methods is not None else ALL_METHOD_MODELS
    results = tuple(
        assess_method(group_load_result, bolt_section, strength_limits, stiffness, friction, eta_proof, scatter_allowance, m)
        for m in models
    )
    feasible: List[PreloadVerificationMethod] = [r.accuracy.method for r in results if r.status == VerificationStatus.FEASIBLE]
    return InstallationMethodTradeResult(
        bolt_section=bolt_section,
        results=results,
        any_feasible=len(feasible) > 0,
        feasible_methods=tuple(feasible),
    )
