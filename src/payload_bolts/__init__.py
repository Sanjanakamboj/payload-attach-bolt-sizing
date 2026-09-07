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

Explicitly out of scope through Milestone 4 (deferred to later
milestones): torque-to-preload conversion / nut factor / torque
coefficient / lubrication / thread friction / under-head friction,
preload relaxation/embedment, thermal preload change, fatigue, prying,
bearing, tear-out, pull-through, thread stripping, nonlinear joint
opening beyond the linear closed-joint model, proof testing,
detailed fastener standards/database lookup, structural optimization,
and final portfolio figures.
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
]

__version__ = "0.1.0"
