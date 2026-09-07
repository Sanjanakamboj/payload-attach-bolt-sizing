"""payload_bolts: rigid-interface bolt-group load distribution and
preliminary bolt strength sizing.

Milestone 1 scope: bolt-pattern geometry, applied interface loads, rigid
elastic bolt-group load distribution (direct shear, torsional shear,
axial/overturning), and independent equilibrium verification.

Milestone 2 scope: preliminary bolt tensile/shear strength screening on
top of the unchanged Milestone 1 loads -- bolt material/section
representation, tensile/shear stress, separate tensile/shear margins,
an illustrative quadratic tension-shear interaction check, deterministic
governing-bolt/mode identification, and candidate bolt-size comparison.

Milestone 3 scope: a first-order preloaded-joint closure and
friction-slip screen on top of the unchanged Milestone 1/2 results --
explicit bolt preload, a bolt/member load-fraction C, joint-separation
screening, remaining clamp force, local friction-capacity/slip
screening, deterministic governing bolt/constraint identification, and
analytical required-preload equations (not a torque specification).

Milestone 4 scope: preload feasibility, proof/yield screening, and an
installation-preload window on top of the unchanged Milestone 1-3
results -- illustrative bolt proof/yield strength limits reusing the
exact Milestone 2 tensile stress area, a proof-load-fraction
installation ceiling, a deterministic installation preload
scatter/loss allowance applied to the Milestone 3 required preload,
the resulting installation-preload window (honestly reported as
infeasible when it is), classification of a selected preload against
that window, and a maximum in-service bolt-tension screen against
proof/yield load using the Milestone 3 closed-joint load-sharing
formula. Not a torque specification, qualification procedure,
certification analysis, or detailed threaded-joint design.

Milestone 5 scope: a transparent conceptual bolt-size trade on top of
the unchanged Milestone 1-4 results -- for each candidate diameter,
Milestone 2 strength and Milestone 3/4 preload-window feasibility are
reused exactly and combined with new local-joint screens: bearing
stress at the plate/hole interface (using the Milestone 1 in-plane
shear resultant, never the axial/tensile load), an edge-distance
geometry screen, and a bolt-spacing geometry screen. Thread stripping
is explicitly NOT modeled (no source-verified formula could be
established from the thread-geometry information available to this
project) rather than approximated. A predeclared, deterministic
admissibility rule and smallest-admissible-diameter selection rule
resolve whether the Milestone 2 minimum-strength 8 mm bolt remains a
defensible conceptual choice once preload feasibility and local-joint
screening are added.

Milestone 6 scope: a transparent nut-factor torque-to-preload model on
top of the unchanged Milestone 1-5 results, for the Milestone 5
selected 10 mm conceptual candidate -- the classical `T = K*F*d`
relation and its inverse, a deterministic (non-statistical) nut-factor
uncertainty range, the resulting "robust torque window" (a torque
range that keeps achieved preload inside the Milestone 4/5 target
window for EVERY nut factor in the declared range, honestly reported
infeasible when it is), a predeclared midpoint nominal-torque rule,
achieved-preload spread across the nut-factor range, an in-service
proof-load carry-forward at the bounding (highest) achieved preload,
a diagnostic torque back-calculation for the historical Milestone 3
selected preload, and a bolt-size torque-feasibility comparison. Not a
production torque specification, qualification procedure, statistical
process-capability analysis, detailed threaded-contact model, or
certified installation requirement.

Explicitly out of scope through Milestone 6 (deferred to later
milestones): detailed thread friction / under-head friction modeled
separately, lubrication-specific coefficients, torque-angle
tightening, bolt-elongation or ultrasonic preload measurement,
hydraulic tensioning, prevailing-torque effects, preload
relaxation/embedment, thermal preload change, fatigue, prying,
nonlinear plate flexibility, detailed flange bending, detailed
bearing/tear-out interaction, net-section rupture, nonlinear contact
FEA, fracture mechanics, thread stripping (explicitly not modeled, see
Milestone 5), installation process-capability analysis, proof testing,
detailed fastener standards/database lookup, structural optimization,
and certification/qualification.
"""

from .geometry import BoltPattern, circular_pattern, rectangular_pattern
from .loads import InterfaceLoad, inertial_force
from .solver import (
    BoltLoadResult,
    BoltGroupResult,
    EquilibriumCheck,
    distribute_loads,
)
from .strength import (
    BoltMaterial,
    BoltSection,
    BoltStrengthResult,
    BoltGroupStrengthResult,
    NoFeasibleCandidateError,
    circular_unthreaded_bolt,
    assess_bolt_group_strength,
    evaluate_candidates,
    select_smallest_passing_bolt,
)
from .preload import (
    PreloadState,
    JointStiffness,
    FrictionModel,
    BoltPreloadResult,
    BoltPreloadGroupResult,
    RequiredPreloadResult,
    assess_preloaded_joint,
    required_preload,
    apply_preload_factor,
)
from .preload_limits import (
    BoltStrengthLimits,
    PreloadLimitResult,
    InstallationPreloadWindow,
    PreloadFeasibilityStatus,
    PreloadFeasibilityResult,
    compute_preload_limits,
    min_installation_preload,
    max_installation_preload,
    installation_preload_window,
    classify_selected_preload,
    assess_preload_feasibility,
)
from .joint_local_checks import (
    PlateMaterial,
    JointGeometry,
    CheckStatus,
    PerBoltBearing,
    BearingCheckResult,
    PerBoltEdgeDistance,
    EdgeDistanceCheckResult,
    PairSpacing,
    SpacingCheckResult,
    ThreadCheckStatus,
    ThreadStripCheckResult,
    BoltCandidateTradeResult,
    BoltSelectionResult,
    assess_bearing,
    assess_edge_distance,
    assess_spacing,
    thread_strip_not_modeled,
    evaluate_candidate_trade,
    select_bolt_candidate,
)
from .torque_preload import (
    NutFactorModel,
    NominalTorqueWindow,
    RobustTorqueWindowResult,
    TorqueInstallationStatus,
    TorqueInstallationResult,
    TorqueBackCalculation,
    torque_from_preload,
    preload_from_torque,
    nominal_torque_window,
    robust_torque_window,
    assess_torque_installation,
    back_calculate_torque,
)

__all__ = [
    "BoltPattern",
    "circular_pattern",
    "rectangular_pattern",
    "InterfaceLoad",
    "inertial_force",
    "BoltLoadResult",
    "BoltGroupResult",
    "EquilibriumCheck",
    "distribute_loads",
    "BoltMaterial",
    "BoltSection",
    "BoltStrengthResult",
    "BoltGroupStrengthResult",
    "NoFeasibleCandidateError",
    "circular_unthreaded_bolt",
    "assess_bolt_group_strength",
    "evaluate_candidates",
    "select_smallest_passing_bolt",
    "PreloadState",
    "JointStiffness",
    "FrictionModel",
    "BoltPreloadResult",
    "BoltPreloadGroupResult",
    "RequiredPreloadResult",
    "assess_preloaded_joint",
    "required_preload",
    "apply_preload_factor",
    "BoltStrengthLimits",
    "PreloadLimitResult",
    "InstallationPreloadWindow",
    "PreloadFeasibilityStatus",
    "PreloadFeasibilityResult",
    "compute_preload_limits",
    "min_installation_preload",
    "max_installation_preload",
    "installation_preload_window",
    "classify_selected_preload",
    "assess_preload_feasibility",
    "PlateMaterial",
    "JointGeometry",
    "CheckStatus",
    "PerBoltBearing",
    "BearingCheckResult",
    "PerBoltEdgeDistance",
    "EdgeDistanceCheckResult",
    "PairSpacing",
    "SpacingCheckResult",
    "ThreadCheckStatus",
    "ThreadStripCheckResult",
    "BoltCandidateTradeResult",
    "BoltSelectionResult",
    "assess_bearing",
    "assess_edge_distance",
    "assess_spacing",
    "thread_strip_not_modeled",
    "evaluate_candidate_trade",
    "select_bolt_candidate",
    "NutFactorModel",
    "NominalTorqueWindow",
    "RobustTorqueWindowResult",
    "TorqueInstallationStatus",
    "TorqueInstallationResult",
    "TorqueBackCalculation",
    "torque_from_preload",
    "preload_from_torque",
    "nominal_torque_window",
    "robust_torque_window",
    "assess_torque_installation",
    "back_calculate_torque",
]

__version__ = "0.1.0"
