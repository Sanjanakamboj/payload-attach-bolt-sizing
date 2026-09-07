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

Explicitly out of scope through Milestone 2 (deferred to later
milestones): preload/torque, friction load sharing, joint slip/
separation, bearing, tear-out, pull-through, prying, thread stripping,
fatigue, detailed fastener standards/database lookup, structural
optimization, and final portfolio figures.
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
]

__version__ = "0.1.0"
