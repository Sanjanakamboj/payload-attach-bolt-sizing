"""Bolt-size trade, local joint failure-mode screening, and preliminary
hardware selection (Milestone 5).

Resolves the engineering decision exposed by Milestone 4: Milestone 2
found 8 mm to be the smallest bolt passing its tensile/shear/interaction
strength screen, but Milestone 4 showed that same 8 mm bolt has NO
feasible installation-preload window under illustrative proof-strength
and installation-scatter assumptions. This module adds a transparent
conceptual bolt-size trade combining, for each candidate diameter:

1. Milestone 2 strength pass/fail (reused exactly, never recomputed);
2. Milestone 3 required preload (reused exactly);
3. Milestone 4 installation-preload-window feasibility (reused exactly);
4. a new local bearing-stress screen at the plate/hole interface;
5. a new edge-distance geometry screen;
6. a new bolt-spacing geometry screen;
7. thread stripping, EXPLICITLY NOT MODELED (see below) rather than a
   fabricated formula.

This is a reduced-order CONCEPTUAL sizing study, not a certification
analysis, and not a real-hardware qualification. It does NOT model
fatigue, prying, nonlinear plate flexibility, detailed flange bending,
detailed fastener torque, lubrication/nut-factor effects, thermal
preload, embedment/relaxation, detailed contact FEA, fracture mechanics,
net-section rupture, or bearing/tear-out interaction beyond the simple
bearing-stress screen implemented here.

It does NOT recompute or modify Milestone 1-4 mechanics: `BoltGroupResult`
(direct/torsional shear, axial distribution, equilibrium),
`BoltGroupStrengthResult` (M2 tensile/shear/interaction), or
`RequiredPreloadResult` / `InstallationPreloadWindow` /
`PreloadFeasibilityResult` (M3/M4) are all consumed exactly as produced
by their own modules.

Source audit (sources actually inspected this milestone)
----------------------------------------------------------
- **mechanicalc.com, "Lug Analysis"** (readable technical reference,
  presenting the Bruhn/Air-Force-Method approach to pin-loaded lugs) --
  bearing area `A_br = D_p * t` (pin diameter times lug thickness),
  giving the classical bearing-stress convention
  `sigma_bearing = P / (D_p * t)` used below; and the edge-distance
  regime split `e/D < 1.5` (hole close to the edge -- shear-out/hoop
  tension can govern) versus `e/D >= 1.5` (bearing becomes the likely
  critical mode). This directly motivates the `(e/d)_min = 1.5` baseline
  screening threshold below.
- **Web-search-derived summary of general edge-distance industry
  practice** (not a single primary document read directly): a
  "1.5D" edge distance was historical 1950s metallic-structure practice;
  post-Comet-disaster aerospace practice commonly moved to "2D"
  (measured from hole center) for sheet-metal structure. Used here only
  as a secondary corroborating data point for the sensitivity range
  {1.5, 2.0, 2.5}, not as a certified requirement.
- **Web-search-derived summary of AISC 360-22** (general structural-
  steel bolted-connection standard, NOT aerospace-specific -- used here
  only because no aerospace-specific spacing source could be
  independently read): minimum center-to-center bolt spacing "shall not
  be less than 2-2/3 bolt diameters (2.67d)," with "3d" cited as
  preferred/constructability spacing. Used here as the illustrative
  `(s/d)_min = 3.0` baseline (with 2.67 noted as the cited minimum).
- **Web-search-derived summary of thread-stripping shear-area formulas**
  (Unified/inch-thread and ISO-metric/VDI-2230 forms) -- both require
  detailed thread-geometry parameters (pitch diameters, thread class of
  fit, effective engagement fraction) that are not established anywhere
  else in this project and cannot be transparently and confidently
  reproduced from a search summary alone. Per this milestone's explicit
  scope guidance, thread stripping is therefore NOT modeled here (see
  `ThreadStripCheckResult` / `THREAD_CHECK_NOT_MODELED` below) rather
  than approximated with a fabricated formula. One rule-of-thumb noted
  in the same search summary -- for equal-tensile-strength mating
  threads, stripping is avoided if the thread shear area is at least
  about 2x the tensile stress area at roughly 1x-1.5x diameter of
  engagement -- is recorded here purely as engineering context, not as
  an implemented check.

Illustrative baseline local-joint assumptions introduced this milestone
(none tied to any real payload adapter):

- Plate/lug bearing material: `PlateMaterial(bearing_allowable=400 MPa)`
  (illustrative -- distinct from Milestone 2's `BoltMaterial` and
  Milestone 4's `BoltStrengthLimits`, which are bolt properties, not
  plate/lug bearing properties).
- Plate thickness `t = 8 mm`, an illustrative conceptual flange
  thickness (order-of-magnitude comparable to the candidate bolt
  diameters, a common proportion for adapter-ring flanges).
- A circular plate boundary concentric with the Milestone 1 bolt
  pattern's own center, of illustrative outer radius `R_plate = bolt
  circle radius + 0.05 m` (a 50 mm illustrative edge margin) -- a
  simplification appropriate ONLY for this project's circular bolt
  pattern; this module does not attempt a general polygon-boundary
  edge-distance calculation.
- `(e/d)_min = 1.5` (source: mechanicalc.com Air-Force-Method
  bearing/shear-out transition, see above).
- `(s/d)_min = 3.0` (source: AISC 360-22 summary, general structural
  steel practice, not aerospace-specific -- explicitly labeled as such).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from .preload import FrictionModel, JointStiffness
from .preload_limits import BoltStrengthLimits, PreloadFeasibilityResult, assess_preload_feasibility
from .solver import BoltGroupResult
from .strength import BoltGroupStrengthResult, BoltMaterial, BoltSection, assess_bolt_group_strength


# ---------------------------------------------------------------------------
# Material / geometry inputs
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlateMaterial:
    """Illustrative plate/lug bearing property. DISTINCT from Milestone
    2's `BoltMaterial` and Milestone 4's `BoltStrengthLimits`, which are
    bolt (not plate/lug) properties.

    bearing_allowable: Pa, finite, > 0. Illustrative unless explicitly
        tied to a sourced material specification by the caller.
    """

    name: str
    bearing_allowable: float

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("PlateMaterial.name must be a non-empty string.")
        if not math.isfinite(self.bearing_allowable) or self.bearing_allowable <= 0:
            raise ValueError(
                f"PlateMaterial.bearing_allowable must be finite and > 0, got {self.bearing_allowable}."
            )


@dataclass(frozen=True)
class JointGeometry:
    """Illustrative local-joint geometry for Milestone 5 bearing/edge/
    spacing screening.

    plate_thickness: t, meters, finite, > 0.
    plate_center: (x, y), meters, finite. Center of the illustrative
        circular plate boundary. This is a simplification appropriate
        for THIS project's circular bolt pattern only (see module
        docstring) -- not a general polygon-boundary model.
    plate_outer_radius: meters, finite, > 0. Must be > the bolt-pattern's
        maximum bolt radius from plate_center for any bolt to have a
        positive edge distance (not enforced here; a non-positive edge
        distance is reported honestly by `assess_edge_distance`, not
        silently rejected at construction).
    edge_distance_min_ratio: (e/d)_min screening threshold, finite, > 0.
    spacing_min_ratio: (s/d)_min screening threshold, finite, > 0.
    """

    plate_thickness: float
    plate_center: Tuple[float, float]
    plate_outer_radius: float
    edge_distance_min_ratio: float
    spacing_min_ratio: float

    def __post_init__(self):
        if not math.isfinite(self.plate_thickness) or self.plate_thickness <= 0:
            raise ValueError(
                f"JointGeometry.plate_thickness must be finite and > 0, got {self.plate_thickness}."
            )
        cx, cy = self.plate_center
        if not (math.isfinite(cx) and math.isfinite(cy)):
            raise ValueError(f"JointGeometry.plate_center must be finite, got {self.plate_center}.")
        if not math.isfinite(self.plate_outer_radius) or self.plate_outer_radius <= 0:
            raise ValueError(
                f"JointGeometry.plate_outer_radius must be finite and > 0, got {self.plate_outer_radius}."
            )
        if not math.isfinite(self.edge_distance_min_ratio) or self.edge_distance_min_ratio <= 0:
            raise ValueError(
                "JointGeometry.edge_distance_min_ratio must be finite and > 0, got "
                f"{self.edge_distance_min_ratio}."
            )
        if not math.isfinite(self.spacing_min_ratio) or self.spacing_min_ratio <= 0:
            raise ValueError(
                f"JointGeometry.spacing_min_ratio must be finite and > 0, got {self.spacing_min_ratio}."
            )


class CheckStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    NOT_APPLICABLE = "NOT_APPLICABLE"


# ---------------------------------------------------------------------------
# Bearing screen
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PerBoltBearing:
    index: int
    bearing_load: float  # F_bearing,i = shear_resultant,i (Milestone 1 in-plane shear, NOT axial/tensile)
    hole_diameter: float  # idealized == candidate nominal diameter (no clearance modeled)
    plate_thickness: float
    bearing_stress: float  # sigma = F_bearing / (d_h * t)
    bearing_margin: Optional[float]  # allowable/demand - 1, None if demand == 0


@dataclass(frozen=True)
class BearingCheckResult:
    """Bearing-stress screen for one candidate diameter, using the
    Milestone 1 per-bolt IN-PLANE SHEAR resultant (never the axial/
    tensile load) as the local bearing demand: `sigma_bearing =
    F_bearing / (d_h * t)`, `d_h` idealized equal to the candidate
    nominal diameter (no hole-clearance allowance modeled -- consistent
    with Milestone 2's idealized shank-based candidate convention)."""

    bolts: Tuple[PerBoltBearing, ...]
    plate: PlateMaterial
    governing_bolt_index: int
    governing_margin: Optional[float]
    passed: bool


def assess_bearing(
    group_load_result: BoltGroupResult, nominal_diameter: float, plate_thickness: float, plate: PlateMaterial
) -> BearingCheckResult:
    if not math.isfinite(nominal_diameter) or nominal_diameter <= 0:
        raise ValueError(f"nominal_diameter must be finite and > 0, got {nominal_diameter}.")
    if not math.isfinite(plate_thickness) or plate_thickness <= 0:
        raise ValueError(f"plate_thickness must be finite and > 0, got {plate_thickness}.")

    per_bolt = []
    for b in group_load_result.bolts:
        f_bearing = b.shear_resultant  # in-plane shear resultant, NOT axial/tensile load
        sigma = f_bearing / (nominal_diameter * plate_thickness)
        margin = (plate.bearing_allowable / sigma - 1.0) if sigma > 0.0 else None
        per_bolt.append(
            PerBoltBearing(
                index=b.index,
                bearing_load=f_bearing,
                hole_diameter=nominal_diameter,
                plate_thickness=plate_thickness,
                bearing_stress=sigma,
                bearing_margin=margin,
            )
        )
    per_bolt = tuple(per_bolt)

    def _sort_key(pb: PerBoltBearing):
        m = pb.bearing_margin if pb.bearing_margin is not None else math.inf
        return (m, pb.index)

    governing = min(per_bolt, key=_sort_key)
    passed = all(pb.bearing_margin is None or pb.bearing_margin >= 0.0 for pb in per_bolt)

    return BearingCheckResult(
        bolts=per_bolt,
        plate=plate,
        governing_bolt_index=governing.index,
        governing_margin=governing.bearing_margin,
        passed=passed,
    )


# ---------------------------------------------------------------------------
# Edge-distance screen (GEOMETRY SCREEN ONLY -- not a tear-out strength
# calculation)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PerBoltEdgeDistance:
    index: int
    edge_distance: float  # e_i = plate_outer_radius - |bolt_i - plate_center|, meters (may be <= 0)
    ratio: float  # e_i / d
    status: CheckStatus


@dataclass(frozen=True)
class EdgeDistanceCheckResult:
    """Geometric edge-distance SCREEN ONLY (e/d vs. a threshold) -- this
    is NOT a detailed tear-out or net-section strength calculation."""

    bolts: Tuple[PerBoltEdgeDistance, ...]
    min_ratio: float
    criterion: float
    governing_bolt_index: int
    passed: bool


def assess_edge_distance(
    group_load_result: BoltGroupResult, nominal_diameter: float, geometry: JointGeometry
) -> EdgeDistanceCheckResult:
    if not math.isfinite(nominal_diameter) or nominal_diameter <= 0:
        raise ValueError(f"nominal_diameter must be finite and > 0, got {nominal_diameter}.")

    cx, cy = geometry.plate_center
    per_bolt = []
    for b in group_load_result.bolts:
        dist_to_center = math.hypot(b.x - cx, b.y - cy)
        e = geometry.plate_outer_radius - dist_to_center
        ratio = e / nominal_diameter
        status = CheckStatus.PASS if ratio >= geometry.edge_distance_min_ratio else CheckStatus.FAIL
        per_bolt.append(PerBoltEdgeDistance(index=b.index, edge_distance=e, ratio=ratio, status=status))
    per_bolt = tuple(per_bolt)

    governing = min(per_bolt, key=lambda pb: (pb.ratio, pb.index))
    passed = all(pb.status == CheckStatus.PASS for pb in per_bolt)

    return EdgeDistanceCheckResult(
        bolts=per_bolt,
        min_ratio=governing.ratio,
        criterion=geometry.edge_distance_min_ratio,
        governing_bolt_index=governing.index,
        passed=passed,
    )


# ---------------------------------------------------------------------------
# Spacing screen (GEOMETRY SCREEN ONLY -- not a group tear-out / net-
# section analysis)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PairSpacing:
    index_a: int
    index_b: int
    spacing: float  # meters
    ratio: float  # s / d
    status: CheckStatus


@dataclass(frozen=True)
class SpacingCheckResult:
    """Pairwise bolt center-to-center spacing SCREEN ONLY (s/d vs. a
    threshold) -- NOT a detailed group tear-out / net-section
    calculation."""

    pairs: Tuple[PairSpacing, ...]
    min_ratio: float
    criterion: float
    governing_pair: Tuple[int, int]
    passed: bool


def assess_spacing(group_load_result: BoltGroupResult, nominal_diameter: float, geometry: JointGeometry) -> SpacingCheckResult:
    if not math.isfinite(nominal_diameter) or nominal_diameter <= 0:
        raise ValueError(f"nominal_diameter must be finite and > 0, got {nominal_diameter}.")

    bolts = group_load_result.bolts
    pairs = []
    for a, b in itertools.combinations(bolts, 2):
        s = math.hypot(a.x - b.x, a.y - b.y)
        ratio = s / nominal_diameter
        status = CheckStatus.PASS if ratio >= geometry.spacing_min_ratio else CheckStatus.FAIL
        # Deterministic pair ordering regardless of input bolt order.
        idx_a, idx_b = (a.index, b.index) if a.index < b.index else (b.index, a.index)
        pairs.append(PairSpacing(index_a=idx_a, index_b=idx_b, spacing=s, ratio=ratio, status=status))
    pairs = tuple(sorted(pairs, key=lambda p: (p.index_a, p.index_b)))

    governing = min(pairs, key=lambda p: (p.ratio, p.index_a, p.index_b))
    passed = all(p.status == CheckStatus.PASS for p in pairs)

    return SpacingCheckResult(
        pairs=pairs,
        min_ratio=governing.ratio,
        criterion=geometry.spacing_min_ratio,
        governing_pair=(governing.index_a, governing.index_b),
        passed=passed,
    )


# ---------------------------------------------------------------------------
# Thread stripping -- EXPLICITLY NOT MODELED (see module docstring)
# ---------------------------------------------------------------------------


class ThreadCheckStatus(str, Enum):
    THREAD_CHECK_NOT_MODELED = "THREAD_CHECK_NOT_MODELED"


@dataclass(frozen=True)
class ThreadStripCheckResult:
    """Placeholder result stating explicitly that thread stripping is
    NOT modeled in Milestone 5 (see module docstring source audit for
    why: a source-verified formula could not be established with
    confidence from the thread-geometry information available to this
    project). `passed` is intentionally `None` (neither PASS nor FAIL)
    -- this check is never used to admit or reject a candidate."""

    status: ThreadCheckStatus
    reason: str
    passed: Optional[bool] = None


def thread_strip_not_modeled() -> ThreadStripCheckResult:
    return ThreadStripCheckResult(
        status=ThreadCheckStatus.THREAD_CHECK_NOT_MODELED,
        reason=(
            "A source-verified thread-stripping shear-area formula requires detailed "
            "thread-geometry parameters (pitch diameters, thread class of fit, effective "
            "engagement fraction) not established elsewhere in this project. Rather than "
            "approximate with an unverified formula, this check is explicitly not modeled "
            "in Milestone 5; see the module docstring source audit."
        ),
    )


# ---------------------------------------------------------------------------
# Per-candidate trade result and predeclared selection rule
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BoltCandidateTradeResult:
    """Combined Milestone 2-5 result for ONE candidate bolt diameter."""

    bolt_section: BoltSection
    strength: BoltGroupStrengthResult  # Milestone 2, reused exactly
    preload_feasibility: PreloadFeasibilityResult  # Milestone 3/4, reused exactly
    bearing: BearingCheckResult
    edge_distance: EdgeDistanceCheckResult
    spacing: SpacingCheckResult
    thread_strip: ThreadStripCheckResult
    admissible: bool
    admissibility_reasons: Tuple[str, ...]  # which predeclared criteria failed, if any


def _is_window_feasible_and_selected_ok(pf: PreloadFeasibilityResult) -> bool:
    """Predeclared M4 admissibility criterion: the installation window
    must be feasible AND the M3 selected preload must classify as
    FEASIBLE against it (not too low, not too high, not
    NO_INSTALLATION_WINDOW). PROOF_LIMIT_EXCEEDED_IN_SERVICE is treated
    as a separate diagnostic already folded into `pf.status` priority,
    so checking `pf.status == FEASIBLE` covers all M4 gating conditions
    in one predeclared test."""
    from .preload_limits import PreloadFeasibilityStatus

    return pf.window.feasible and pf.status == PreloadFeasibilityStatus.FEASIBLE


def evaluate_candidate_trade(
    group_load_result: BoltGroupResult,
    nominal_diameter: float,
    bolt_section: BoltSection,
    bolt_material: BoltMaterial,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    preload_factor: float,
    plate: PlateMaterial,
    geometry: JointGeometry,
) -> BoltCandidateTradeResult:
    """Evaluate ONE candidate diameter against the predeclared Milestone
    5 admissibility rule (see `select_bolt_candidate`), reusing
    Milestone 2 strength and Milestone 3/4 preload feasibility exactly,
    unchanged, and adding the new bearing/edge/spacing local-joint
    screens."""
    strength = assess_bolt_group_strength(group_load_result, bolt_section, bolt_material)

    from .preload import required_preload

    required = required_preload(group_load_result, stiffness, friction)
    selected_preload = required.overall_required * preload_factor
    preload_feasibility = assess_preload_feasibility(
        group_load_result, bolt_section, strength_limits, stiffness, friction,
        eta_proof, scatter_allowance, selected_preload,
    )

    bearing = assess_bearing(group_load_result, nominal_diameter, geometry.plate_thickness, plate)
    edge_distance = assess_edge_distance(group_load_result, nominal_diameter, geometry)
    spacing = assess_spacing(group_load_result, nominal_diameter, geometry)
    thread_strip = thread_strip_not_modeled()

    reasons: List[str] = []
    if not strength.passed:
        reasons.append("M2 strength FAIL")
    if not _is_window_feasible_and_selected_ok(preload_feasibility):
        reasons.append(f"M4 preload feasibility {preload_feasibility.status.value}")
    if not bearing.passed:
        reasons.append("bearing MS < 0")
    if not edge_distance.passed:
        reasons.append("edge-distance screen FAIL")
    if not spacing.passed:
        reasons.append("spacing screen FAIL")
    # Thread stripping is NOT modeled -- per the predeclared rule it is
    # never a gating criterion (see module docstring / rule statement).

    admissible = len(reasons) == 0

    return BoltCandidateTradeResult(
        bolt_section=bolt_section,
        strength=strength,
        preload_feasibility=preload_feasibility,
        bearing=bearing,
        edge_distance=edge_distance,
        spacing=spacing,
        thread_strip=thread_strip,
        admissible=admissible,
        admissibility_reasons=tuple(reasons),
    )


@dataclass(frozen=True)
class BoltSelectionResult:
    """Group-level Milestone 5 result: every candidate's trade result,
    plus the predeclared-rule selection outcome.

    Predeclared selection rule (declared BEFORE evaluating candidates,
    never tuned after seeing results):

        A candidate is ADMISSIBLE only if ALL of:
          1. Milestone 2 strength passes;
          2. the Milestone 4 installation-preload window is feasible AND
             the Milestone 3 selected preload classifies as FEASIBLE
             against it;
          3. bearing margin of safety >= 0;
          4. the edge-distance screen passes;
          5. the spacing screen passes.
        (Thread stripping is NOT modeled and is therefore never a
        gating criterion -- see `ThreadStripCheckResult`.)

        Among ADMISSIBLE candidates, SELECT the smallest nominal
        diameter. Ties (should not occur for distinct diameters) break
        by lowest diameter value. This is a conceptual MINIMUM-SIZE
        rule, not a claim of global optimality. If no candidate is
        admissible, `selected` is `None` and `no_feasible_candidate` is
        `True`.
    """

    candidates: Tuple[BoltCandidateTradeResult, ...]  # in increasing nominal-diameter order
    selected_index: Optional[int]  # index into `candidates`, or None
    no_feasible_candidate: bool

    def selected(self) -> Optional[BoltCandidateTradeResult]:
        return None if self.selected_index is None else self.candidates[self.selected_index]


def select_bolt_candidate(
    group_load_result: BoltGroupResult,
    candidates: List[Tuple[float, BoltSection]],
    bolt_material: BoltMaterial,
    strength_limits: BoltStrengthLimits,
    stiffness: JointStiffness,
    friction: FrictionModel,
    eta_proof: float,
    scatter_allowance: float,
    preload_factor: float,
    plate: PlateMaterial,
    geometry: JointGeometry,
) -> BoltSelectionResult:
    """Evaluate every (nominal_diameter, BoltSection) candidate via the
    predeclared admissibility rule above and select the smallest
    admissible diameter, in increasing-diameter order (stable for
    input-order ties)."""
    ordered = sorted(candidates, key=lambda c: c[0])
    results = tuple(
        evaluate_candidate_trade(
            group_load_result, d, section, bolt_material, strength_limits, stiffness, friction,
            eta_proof, scatter_allowance, preload_factor, plate, geometry,
        )
        for d, section in ordered
    )

    selected_index = None
    for i, r in enumerate(results):
        if r.admissible:
            selected_index = i
            break

    return BoltSelectionResult(
        candidates=results, selected_index=selected_index, no_feasible_candidate=(selected_index is None)
    )
