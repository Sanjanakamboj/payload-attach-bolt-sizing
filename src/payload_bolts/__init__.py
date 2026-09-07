"""payload_bolts: rigid-interface bolt-group load distribution.

Milestone 1 scope: bolt-pattern geometry, applied interface loads, rigid
elastic bolt-group load distribution (direct shear, torsional shear,
axial/overturning), and independent equilibrium verification.

Explicitly out of scope for Milestone 1 (deferred to later milestones):
bolt material strength allowables, preload/torque, friction load sharing,
joint slip/separation, bearing, tear-out, pull-through, prying, thread
effects, fatigue, fastener standards, and margin-of-safety sizing.
"""

from .geometry import BoltPattern, circular_pattern, rectangular_pattern
from .loads import InterfaceLoad, inertial_force
from .solver import (
    BoltLoadResult,
    BoltGroupResult,
    EquilibriumCheck,
    distribute_loads,
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
]

__version__ = "0.1.0"
