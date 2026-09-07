"""Hardware installation-architecture trade closure (Milestone 8).

Milestones 5-7 left the conceptual hardware decision open: Milestone 5
selected 10 mm as the smallest conceptual bolt passing strength +
preload-window + local-joint screening; Milestone 6 found 10 mm has NO
robust torque-only installation window at the illustrative baseline
nut-factor range, while 12 mm does; Milestone 7 found 10 mm DOES have a
feasible window under several direct preload-verification methods.
This module closes the resulting hardware trade by comparing two full
installation ARCHITECTURES -- not just bolt sizes -- across mass,
installation-control robustness, in-service proof reserve, local-joint
packaging margin, and a normalized (non-dollar) installation-complexity
index, using explicit, transparent, deterministic criteria declared
before any architecture is evaluated.

This is a conceptual reduced-order engineering TRADE STUDY. It is
explicitly NOT a procurement analysis, flight-hardware selection,
supplier quotation, manufacturing plan, certification activity,
life-cycle cost model, or qualification plan.

This module does NOT recompute or modify Milestone 1-7 mechanics:
`BoltGroupResult`, `BoltGroupStrengthResult`, `RequiredPreloadResult`,
`InstallationPreloadWindow`/`PreloadFeasibilityResult`,
`BoltSelectionResult`/`BoltCandidateTradeResult` (bearing/edge/
spacing), `RobustTorqueWindowResult`/`TorqueInstallationResult`, and
`MethodFeasibilityResult`/`InstallationMethodTradeResult` are all
consumed exactly as produced by their own modules.

Source audit (sources actually inspected this milestone)
----------------------------------------------------------
- **mechanicalc.com, "Fastener Size Tables"** (readable primary
  technical reference) -- gives exact ISO metric hex bolt/nut/washer
  dimensions used directly (at their stated maximum/nominal values) as
  the sourced geometry inputs for this milestone's mass model:

    M10 x 1.5: head width-across-flats 16.00 mm, head height 6.63 mm;
        hex nut width-across-flats 16.00 mm, thickness 8.40 mm; flat
        washer OD 28.00 mm, thickness 2.80 mm.
    M12 x 1.75: head width-across-flats 18.00 mm, head height 7.76 mm;
        hex nut width-across-flats 18.00 mm, thickness 10.80 mm; flat
        washer OD 34.00 mm, thickness 3.50 mm.

  These are used as a lookup for exactly the two diameters this
  milestone compares -- NOT extrapolated to other diameters via a
  fitted formula.
- **Steel density, 7850 kg/m^3**: a standard, widely-known structural/
  carbon-steel density value (not independently re-derived from a
  single primary document this milestone; used here as a conventional
  engineering constant, consistent with the illustrative alloy-steel
  fastener already assumed for strength/proof properties in Milestones
  2/4, but kept as a SEPARATE, explicitly labeled input -- mass density
  is never inferred from proof strength).
- **Web-search-derived summary of the NASA Systems Engineering
  Handbook's trade-study guidance** (not a single primary document read
  directly) -- corroborates a weighted decision-matrix approach
  (criteria in rows, alternatives in columns, declared weights) as a
  standard, legitimate systems-engineering trade methodology; this
  motivates this module's predeclared, fixed-BEFORE-evaluation
  complexity-index weighting (see `COMPLEXITY_WEIGHTS` below), applied
  only as a secondary/illustrative index alongside -- never instead of
  -- an explicit Pareto/trade-table comparison across the raw metrics.

No real procurement price or cost data were found or used. Per this
milestone's explicit scope guidance, NO dollar cost figure is produced
anywhere in this module -- only a normalized, unitless, deterministic
"installation-complexity index" built from small integer sub-scores on
a declared 0-3 scale (see `ComplexityIndex`).

Mass model
------------
This is a REDUCED-ORDER, sourced-geometry conceptual mass proxy for
ONE installed fastener SET (bolt + nut + one flat washer), NOT a
detailed CAD-accurate fastener model:

    V_shank  = pi/4 * d^2 * L_bolt
    V_head   = A_hex(W_head) * H_head                      (solid)
    V_nut    = A_hex(W_nut) * t_nut  -  pi/4 * d^2 * t_nut  (threaded bore subtracted)
    V_washer = pi/4 * (OD_washer^2 - d^2) * t_washer        (annulus)
    V_total  = V_shank + V_head + V_nut + V_washer
    m        = rho * V_total

where `A_hex(W) = (sqrt(3)/2) * W^2` is the area of a regular hexagon
in terms of its width across flats `W`. `L_bolt = grip_length +
thread_allowance(d)`, with `grip_length` shared between candidates
(twice the Milestone 5 illustrative plate thickness, i.e. two clamped
plates) and `thread_allowance(d) = 1.0*d` (a common nut-engagement
rule of thumb, scaled with diameter -- the SAME rule applied to both
candidates, not tuned to favor either). Thread-root diameter reduction
along the shank is NOT modeled (consistent with this project's
established idealized full-diameter-shank convention from Milestone 2
onward); this is a full assembled-fastener-set mass proxy (shank +
head + nut + washer), not a shank-only mass -- explicitly NOT labeled
as such.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional

from .joint_local_checks import (
    EdgeDistanceCheckResult,
    JointGeometry,
    PlateMaterial,
    SpacingCheckResult,
    assess_bearing,
    assess_edge_distance,
    assess_spacing,
)
from .preload import FrictionModel, JointStiffness, required_preload
from .preload_limits import BoltStrengthLimits, installation_preload_window
from .preload_verification import (
    INSTRUMENTED_BOLT_MODEL,
    MethodFeasibilityResult,
    VerificationStatus,
    assess_method,
)
from .solver import BoltGroupResult
from .strength import BoltMaterial, BoltSection, assess_bolt_group_strength, circular_unthreaded_bolt
from .torque_preload import NutFactorModel, TorqueInstallationResult, TorqueInstallationStatus, assess_torque_installation

# ---------------------------------------------------------------------------
# Sourced ISO metric fastener geometry (see module docstring for the audit)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FastenerGeometry:
    """Sourced (or, if noted, illustrative) ISO metric hex bolt/nut/
    washer geometry for ONE specific nominal diameter. All dimensions
    in meters."""

    nominal_diameter: float
    head_width_across_flats: float
    head_height: float
    nut_width_across_flats: float
    nut_thickness: float
    washer_outer_diameter: float
    washer_thickness: float
    source_basis: str


GEOMETRY_M10 = FastenerGeometry(
    nominal_diameter=0.010,
    head_width_across_flats=0.01600,
    head_height=0.00663,
    nut_width_across_flats=0.01600,
    nut_thickness=0.00840,
    washer_outer_diameter=0.02800,
    washer_thickness=0.00280,
    source_basis="SOURCED: mechanicalc.com Fastener Size Tables, M10 x 1.5 (max/nominal values)",
)

GEOMETRY_M12 = FastenerGeometry(
    nominal_diameter=0.012,
    head_width_across_flats=0.01800,
    head_height=0.00776,
    nut_width_across_flats=0.01800,
    nut_thickness=0.01080,
    washer_outer_diameter=0.03400,
    washer_thickness=0.00350,
    source_basis="SOURCED: mechanicalc.com Fastener Size Tables, M12 x 1.75 (max/nominal values)",
)

FASTENER_GEOMETRY_BY_DIAMETER_MM: Dict[float, FastenerGeometry] = {10.0: GEOMETRY_M10, 12.0: GEOMETRY_M12}

DEFAULT_STEEL_DENSITY = 7850.0  # kg/m^3, standard structural/carbon-steel density (illustrative for this alloy)
DEFAULT_THREAD_ALLOWANCE_FACTOR = 1.0  # thread_allowance = factor * d


def _hex_area(width_across_flats: float) -> float:
    return (math.sqrt(3.0) / 2.0) * width_across_flats**2


@dataclass(frozen=True)
class FastenerMassModel:
    """Per-installed-fastener-set mass result (bolt + nut + one washer).
    NOT a shank-only mass -- see module docstring for the full volume
    breakdown, which is reported here for auditability."""

    geometry: FastenerGeometry
    density: float
    grip_length: float
    thread_allowance: float
    bolt_length: float
    volume_shank: float
    volume_head: float
    volume_nut: float
    volume_washer: float
    volume_total: float
    mass: float


def compute_fastener_mass(
    geometry: FastenerGeometry,
    grip_length: float,
    density: float = DEFAULT_STEEL_DENSITY,
    thread_allowance_factor: float = DEFAULT_THREAD_ALLOWANCE_FACTOR,
) -> FastenerMassModel:
    if not math.isfinite(grip_length) or grip_length <= 0.0:
        raise ValueError(f"grip_length must be finite and > 0, got {grip_length}.")
    if not math.isfinite(density) or density <= 0.0:
        raise ValueError(f"density must be finite and > 0, got {density}.")
    if not math.isfinite(thread_allowance_factor) or thread_allowance_factor < 0.0:
        raise ValueError(f"thread_allowance_factor must be finite and >= 0, got {thread_allowance_factor}.")

    d = geometry.nominal_diameter
    thread_allowance = thread_allowance_factor * d
    bolt_length = grip_length + thread_allowance

    v_shank = math.pi / 4.0 * d**2 * bolt_length
    v_head = _hex_area(geometry.head_width_across_flats) * geometry.head_height
    v_nut = _hex_area(geometry.nut_width_across_flats) * geometry.nut_thickness - math.pi / 4.0 * d**2 * geometry.nut_thickness
    v_washer = math.pi / 4.0 * (geometry.washer_outer_diameter**2 - d**2) * geometry.washer_thickness
    v_total = v_shank + v_head + v_nut + v_washer

    return FastenerMassModel(
        geometry=geometry,
        density=density,
        grip_length=grip_length,
        thread_allowance=thread_allowance,
        bolt_length=bolt_length,
        volume_shank=v_shank,
        volume_head=v_head,
        volume_nut=v_nut,
        volume_washer=v_washer,
        volume_total=v_total,
        mass=density * v_total,
    )


@dataclass(frozen=True)
class ArchitectureMassResult:
    per_fastener: FastenerMassModel
    n_bolts: int
    group_mass: float  # n_bolts * per_fastener.mass


def compute_group_mass(mass_model: FastenerMassModel, n_bolts: int) -> ArchitectureMassResult:
    if not isinstance(n_bolts, int) or isinstance(n_bolts, bool) or n_bolts < 1:
        raise ValueError(f"n_bolts must be an integer >= 1, got {n_bolts!r}.")
    return ArchitectureMassResult(per_fastener=mass_model, n_bolts=n_bolts, group_mass=n_bolts * mass_model.mass)


# ---------------------------------------------------------------------------
# Normalized installation-complexity index (NOT a dollar cost)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ComplexityIndex:
    """Normalized, unitless, deterministic installation-complexity
    index. Sub-scores use a fixed 0-3 discrete scale (0=none/basic,
    1=modest, 2=moderate, 3=high), and weights are declared in
    `COMPLEXITY_WEIGHTS` BEFORE any architecture is evaluated. This is
    an engineering proxy, NOT a dollar cost."""

    hardware_complexity: int  # special instrumentation hardware required
    measurement_complexity: int  # per-bolt measurement required
    calibration_complexity: int  # calibration requirement
    process_step_complexity: int  # extra installation/inspection steps
    total_score: float  # weighted sum, using COMPLEXITY_WEIGHTS


# Predeclared BEFORE evaluating any architecture (see module docstring
# source audit / NASA SEH decision-matrix motivation). Never tuned
# after seeing which architecture wins.
COMPLEXITY_WEIGHTS: Dict[str, float] = {
    "hardware": 1.0,
    "measurement": 1.0,
    "calibration": 1.0,
    "process": 1.0,
}


def _validate_subscore(name: str, value: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not (0 <= value <= 3):
        raise ValueError(f"{name} sub-score must be an integer in [0, 3], got {value!r}.")


def compute_complexity_index(
    hardware_complexity: int, measurement_complexity: int, calibration_complexity: int, process_step_complexity: int
) -> ComplexityIndex:
    _validate_subscore("hardware_complexity", hardware_complexity)
    _validate_subscore("measurement_complexity", measurement_complexity)
    _validate_subscore("calibration_complexity", calibration_complexity)
    _validate_subscore("process_step_complexity", process_step_complexity)
    total = (
        COMPLEXITY_WEIGHTS["hardware"] * hardware_complexity
        + COMPLEXITY_WEIGHTS["measurement"] * measurement_complexity
        + COMPLEXITY_WEIGHTS["calibration"] * calibration_complexity
        + COMPLEXITY_WEIGHTS["process"] * process_step_complexity
    )
    return ComplexityIndex(
        hardware_complexity=hardware_complexity,
        measurement_complexity=measurement_complexity,
        calibration_complexity=calibration_complexity,
        process_step_complexity=process_step_complexity,
        total_score=total,
    )


# Predeclared, fixed sub-scores for the two primary architectures (see
# README "Milestone 8" for the rationale behind each value). Declared
# BEFORE the architecture table is generated; never tuned afterward.
COMPLEXITY_10MM_DIRECT_VERIFICATION = compute_complexity_index(
    hardware_complexity=2,  # instrumented bolts or equivalent sensing hardware required
    measurement_complexity=2,  # per-bolt measurement/readout required
    calibration_complexity=2,  # sensor calibration required
    process_step_complexity=1,  # one additional verification step vs. torque-only
)

COMPLEXITY_12MM_TORQUE_CONTROL = compute_complexity_index(
    hardware_complexity=0,  # standard torque wrench only
    measurement_complexity=0,  # no per-bolt measurement device
    calibration_complexity=1,  # torque wrench itself still requires periodic calibration
    process_step_complexity=0,  # standard single-step torque application
)


# ---------------------------------------------------------------------------
# Installation architectures
# ---------------------------------------------------------------------------


class InstallationArchitecture(str, Enum):
    TEN_MM_DIRECT_VERIFICATION = "TEN_MM_DIRECT_VERIFICATION"
    TWELVE_MM_TORQUE_CONTROL = "TWELVE_MM_TORQUE_CONTROL"
    TEN_MM_TORQUE_ONLY = "TEN_MM_TORQUE_ONLY"  # comparison-only
    TWELVE_MM_DIRECT_VERIFICATION = "TWELVE_MM_DIRECT_VERIFICATION"  # comparison-only


class GateStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    NOT_APPLICABLE = "NOT_APPLICABLE"


@dataclass(frozen=True)
class MandatoryGateResult:
    """Pass/fail against the seven predeclared Milestone 8 mandatory
    gates. An architecture enters the trade only if ALL applicable
    gates pass."""

    m2_strength: bool
    m4_preload_window: bool
    m5_bearing: bool
    m5_edge_distance: bool
    m5_spacing: bool
    installation_control_robust: bool
    in_service_proof: bool

    @property
    def all_pass(self) -> bool:
        return all(
            (
                self.m2_strength,
                self.m4_preload_window,
                self.m5_bearing,
                self.m5_edge_distance,
                self.m5_spacing,
                self.installation_control_robust,
                self.in_service_proof,
            )
        )


@dataclass(frozen=True)
class ArchitectureTradeResult:
    """Combined Milestone 2-8 result for ONE installation architecture."""

    architecture: InstallationArchitecture
    nominal_diameter_mm: float
    bolt_section: BoltSection
    gates: MandatoryGateResult
    mass: ArchitectureMassResult
    edge_distance: EdgeDistanceCheckResult
    spacing: SpacingCheckResult
    bearing_governing_margin: Optional[float]
    proof_reserve: Optional[float]
    tolerance_ratio: float  # preload-window ratio (direct) or K-uncertainty ratio (torque)
    complexity: ComplexityIndex
    torque_result: Optional[TorqueInstallationResult]
    verification_result: Optional[MethodFeasibilityResult]
    admissible: bool


def _mandatory_gates_direct(
    strength_passed: bool,
    window_feasible: bool,
    bearing_passed: bool,
    edge_passed: bool,
    spacing_passed: bool,
    verification: MethodFeasibilityResult,
) -> MandatoryGateResult:
    return MandatoryGateResult(
        m2_strength=strength_passed,
        m4_preload_window=window_feasible,
        m5_bearing=bearing_passed,
        m5_edge_distance=edge_passed,
        m5_spacing=spacing_passed,
        installation_control_robust=(verification.status == VerificationStatus.FEASIBLE),
        in_service_proof=(verification.status == VerificationStatus.FEASIBLE and verification.proof_reserve is not None and verification.proof_reserve >= 0.0),
    )


def _mandatory_gates_torque(
    strength_passed: bool,
    window_feasible: bool,
    bearing_passed: bool,
    edge_passed: bool,
    spacing_passed: bool,
    torque: TorqueInstallationResult,
) -> MandatoryGateResult:
    torque_ok = torque.status == TorqueInstallationStatus.FEASIBLE
    proof_ok = torque_ok and torque.proof_reserve is not None and torque.proof_reserve >= 0.0
    return MandatoryGateResult(
        m2_strength=strength_passed,
        m4_preload_window=window_feasible,
        m5_bearing=bearing_passed,
        m5_edge_distance=edge_passed,
        m5_spacing=spacing_passed,
        installation_control_robust=torque_ok,
        in_service_proof=proof_ok,
    )


def evaluate_architecture(
    architecture: InstallationArchitecture,
    group_load_result: BoltGroupResult,
    bolt_material: BoltMaterial,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    plate: PlateMaterial,
    geometry_screen: JointGeometry,
    grip_length: float,
    density: float = DEFAULT_STEEL_DENSITY,
    nut_factor: Optional[NutFactorModel] = None,
    complexity: Optional[ComplexityIndex] = None,
) -> ArchitectureTradeResult:
    """Evaluate ONE installation architecture against the seven
    predeclared Milestone 8 mandatory gates, reusing Milestone 2-7
    results exactly (never recomputed with a different convention)."""
    if architecture in (InstallationArchitecture.TEN_MM_DIRECT_VERIFICATION, InstallationArchitecture.TEN_MM_TORQUE_ONLY):
        d_mm = 10.0
    else:
        d_mm = 12.0
    section = circular_unthreaded_bolt(d_mm / 1000.0)

    strength = assess_bolt_group_strength(group_load_result, section, bolt_material)
    required = required_preload(group_load_result, stiffness, friction)
    window = installation_preload_window(required.overall_required, section, strength_limits, eta_proof, scatter_allowance)
    bearing = assess_bearing(group_load_result, d_mm / 1000.0, geometry_screen.plate_thickness, plate)
    edge = assess_edge_distance(group_load_result, d_mm / 1000.0, geometry_screen)
    spacing = assess_spacing(group_load_result, d_mm / 1000.0, geometry_screen)

    fastener_geometry = FASTENER_GEOMETRY_BY_DIAMETER_MM[d_mm]
    mass_model = compute_fastener_mass(fastener_geometry, grip_length, density)
    group_mass = compute_group_mass(mass_model, group_load_result.pattern.n_bolts)

    is_direct = architecture in (
        InstallationArchitecture.TEN_MM_DIRECT_VERIFICATION,
        InstallationArchitecture.TWELVE_MM_DIRECT_VERIFICATION,
    )

    if is_direct:
        verification = assess_method(
            group_load_result, section, strength_limits, stiffness, friction, eta_proof, scatter_allowance,
            INSTRUMENTED_BOLT_MODEL,
        )
        gates = _mandatory_gates_direct(strength.passed, window.feasible, bearing.passed, edge.passed, spacing.passed, verification)
        proof_reserve = verification.proof_reserve
        tolerance_ratio = verification.verified_window.window_ratio if verification.verified_window else (
            window.target_max / window.target_min if window.target_min > 0 else math.inf
        )
        torque_result = None
    else:
        if nut_factor is None:
            raise ValueError("nut_factor is required for a torque-control architecture.")
        torque_result = assess_torque_installation(
            group_load_result, section, strength_limits, stiffness, friction, eta_proof, scatter_allowance, nut_factor
        )
        gates = _mandatory_gates_torque(strength.passed, window.feasible, bearing.passed, edge.passed, spacing.passed, torque_result)
        proof_reserve = torque_result.proof_reserve
        tolerance_ratio = nut_factor.uncertainty_ratio
        verification = None

    if complexity is None:
        complexity = COMPLEXITY_10MM_DIRECT_VERIFICATION if is_direct else COMPLEXITY_12MM_TORQUE_CONTROL

    return ArchitectureTradeResult(
        architecture=architecture,
        nominal_diameter_mm=d_mm,
        bolt_section=section,
        gates=gates,
        mass=group_mass,
        edge_distance=edge,
        spacing=spacing,
        bearing_governing_margin=bearing.governing_margin,
        proof_reserve=proof_reserve,
        tolerance_ratio=tolerance_ratio,
        complexity=complexity,
        torque_result=torque_result,
        verification_result=verification,
        admissible=gates.all_pass,
    )


# ---------------------------------------------------------------------------
# Pareto comparison and predeclared selection rule
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ParetoComparison:
    """Dominance comparison between two architectures across the trade
    axes: lower mass is better, higher tolerance_ratio is better, higher
    proof_reserve is better, higher edge/spacing margin is better, lower
    complexity is better. `a_dominates_b` is True only if A is at least
    as good as B on EVERY axis and strictly better on at least one."""

    better_mass: str  # "a" | "b" | "tie"
    better_tolerance: str
    better_proof_reserve: str
    better_packaging: str
    better_complexity: str
    a_dominates_b: bool
    b_dominates_a: bool
    nondominated: bool


def compare_pareto(a: ArchitectureTradeResult, b: ArchitectureTradeResult) -> ParetoComparison:
    def _cmp(a_val: float, b_val: float, lower_is_better: bool) -> str:
        if a_val == b_val:
            return "tie"
        a_better = (a_val < b_val) if lower_is_better else (a_val > b_val)
        return "a" if a_better else "b"

    a_edge_margin = a.edge_distance.min_ratio - a.edge_distance.criterion
    b_edge_margin = b.edge_distance.min_ratio - b.edge_distance.criterion
    a_spacing_margin = a.spacing.min_ratio - a.spacing.criterion
    b_spacing_margin = b.spacing.min_ratio - b.spacing.criterion
    a_packaging = min(a_edge_margin, a_spacing_margin)
    b_packaging = min(b_edge_margin, b_spacing_margin)

    better_mass = _cmp(a.mass.group_mass, b.mass.group_mass, lower_is_better=True)
    better_tolerance = _cmp(a.tolerance_ratio, b.tolerance_ratio, lower_is_better=False)
    a_pr = a.proof_reserve if a.proof_reserve is not None else -math.inf
    b_pr = b.proof_reserve if b.proof_reserve is not None else -math.inf
    better_proof_reserve = _cmp(a_pr, b_pr, lower_is_better=False)
    better_packaging = _cmp(a_packaging, b_packaging, lower_is_better=False)
    better_complexity = _cmp(a.complexity.total_score, b.complexity.total_score, lower_is_better=True)

    axes = [better_mass, better_tolerance, better_proof_reserve, better_packaging, better_complexity]
    a_dominates = all(x in ("a", "tie") for x in axes) and any(x == "a" for x in axes)
    b_dominates = all(x in ("b", "tie") for x in axes) and any(x == "b" for x in axes)

    return ParetoComparison(
        better_mass=better_mass,
        better_tolerance=better_tolerance,
        better_proof_reserve=better_proof_reserve,
        better_packaging=better_packaging,
        better_complexity=better_complexity,
        a_dominates_b=a_dominates,
        b_dominates_a=b_dominates,
        nondominated=not a_dominates and not b_dominates,
    )


class TradeSelectionStatus(str, Enum):
    PREFER_A = "PREFER_A"
    PREFER_B = "PREFER_B"
    NEITHER_ADMISSIBLE = "NEITHER_ADMISSIBLE"
    ONLY_A_ADMISSIBLE = "ONLY_A_ADMISSIBLE"
    ONLY_B_ADMISSIBLE = "ONLY_B_ADMISSIBLE"


@dataclass(frozen=True)
class TradeSelectionResult:
    """Result of applying the PREDECLARED Milestone 8 decision rule
    (see README "Predeclared decision rule" / module docstring) to two
    admissible-or-not architectures. The rule is fixed and documented
    BEFORE any architecture table is computed; it is never tuned after
    seeing results.

    Predeclared rule: prefer architecture A (10 mm + direct
    verification) over B (12 mm + torque control) if AND ONLY IF:
      1. both A and B pass all seven mandatory gates (if only one
         passes, that one is preferred regardless of the rest; if
         neither passes, NEITHER_ADMISSIBLE);
      2. A's total hardware mass is lower than B's;
      3. A's installation-tolerance reserve below its own epsilon_max
         (i.e. epsilon_max - epsilon_used) is at least
         `MIN_TOLERANCE_RESERVE` (declared below);
      4. A's complexity index remains at or below
         `MAX_COMPLEXITY_THRESHOLD` (declared below).
    Otherwise prefer B.
    """

    pareto: ParetoComparison
    status: TradeSelectionStatus
    reason: str


# Predeclared thresholds (see module docstring / README). Never tuned
# after seeing which architecture wins.
MIN_TOLERANCE_RESERVE = 0.05  # epsilon_max - epsilon_used must be >= 5 percentage points
MAX_COMPLEXITY_THRESHOLD = 6.0  # architecture A's complexity index must not exceed this


def apply_trade_decision_rule(
    architecture_a: ArchitectureTradeResult,
    architecture_b: ArchitectureTradeResult,
    epsilon_max_a: float,
    epsilon_used_a: float,
) -> TradeSelectionResult:
    pareto = compare_pareto(architecture_a, architecture_b)

    if not architecture_a.admissible and not architecture_b.admissible:
        return TradeSelectionResult(pareto=pareto, status=TradeSelectionStatus.NEITHER_ADMISSIBLE, reason="Neither architecture passes all seven mandatory gates.")
    if not architecture_a.admissible:
        return TradeSelectionResult(pareto=pareto, status=TradeSelectionStatus.ONLY_B_ADMISSIBLE, reason="Architecture A fails a mandatory gate; B is the only admissible architecture.")
    if not architecture_b.admissible:
        return TradeSelectionResult(pareto=pareto, status=TradeSelectionStatus.ONLY_A_ADMISSIBLE, reason="Architecture B fails a mandatory gate; A is the only admissible architecture.")

    tolerance_reserve = epsilon_max_a - epsilon_used_a
    mass_ok = architecture_a.mass.group_mass < architecture_b.mass.group_mass
    reserve_ok = tolerance_reserve >= MIN_TOLERANCE_RESERVE
    complexity_ok = architecture_a.complexity.total_score <= MAX_COMPLEXITY_THRESHOLD

    if mass_ok and reserve_ok and complexity_ok:
        return TradeSelectionResult(
            pareto=pareto,
            status=TradeSelectionStatus.PREFER_A,
            reason=(
                f"Both admissible; A has lower mass ({architecture_a.mass.group_mass*1000:.1f} g < "
                f"{architecture_b.mass.group_mass*1000:.1f} g), tolerance reserve "
                f"{tolerance_reserve*100:.1f}pp >= {MIN_TOLERANCE_RESERVE*100:.0f}pp, and complexity "
                f"{architecture_a.complexity.total_score:.1f} <= {MAX_COMPLEXITY_THRESHOLD:.1f}."
            ),
        )
    reasons = []
    if not mass_ok:
        reasons.append(f"A's mass ({architecture_a.mass.group_mass*1000:.1f} g) is not lower than B's ({architecture_b.mass.group_mass*1000:.1f} g)")
    if not reserve_ok:
        reasons.append(f"A's tolerance reserve ({tolerance_reserve*100:.1f}pp) is below the {MIN_TOLERANCE_RESERVE*100:.0f}pp threshold")
    if not complexity_ok:
        reasons.append(f"A's complexity ({architecture_a.complexity.total_score:.1f}) exceeds the threshold ({MAX_COMPLEXITY_THRESHOLD:.1f})")
    return TradeSelectionResult(
        pareto=pareto,
        status=TradeSelectionStatus.PREFER_B,
        reason="Both admissible, but the predeclared rule for A was not satisfied: " + "; ".join(reasons) + ".",
    )
