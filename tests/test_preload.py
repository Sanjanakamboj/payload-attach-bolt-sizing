"""Milestone 3 tests: first-order preloaded-joint closure and
friction-slip screening.

These tests exercise payload_bolts.preload in isolation and against
Milestone 1 BoltGroupResult objects, and confirm Milestone 1 and
Milestone 2 behavior is unaffected (see test_milestone1_milestone2_regression
at the bottom).
"""

import math

import pytest

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadState,
    assess_bolt_group_strength,
    assess_preloaded_joint,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    required_preload,
)
from payload_bolts.preload import apply_preload_factor

# ---------------------------------------------------------------------------
# shared fixtures / helpers
# ---------------------------------------------------------------------------

FRICTION = FrictionModel(friction_coefficient=0.2)
STIFFNESS = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)  # C = 0.2


def _sanity_group_result():
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
    return distribute_loads(pattern, load)


# ---------------------------------------------------------------------------
# A-E. JointStiffness / C
# ---------------------------------------------------------------------------


def test_joint_stiffness_validation():
    with pytest.raises(ValueError):
        JointStiffness(bolt_stiffness=0.0, member_stiffness=1e8)
    with pytest.raises(ValueError):
        JointStiffness(bolt_stiffness=1e8, member_stiffness=-1.0)
    with pytest.raises(ValueError):
        JointStiffness(bolt_stiffness=float("nan"), member_stiffness=1e8)


def test_C_hand_calculation():
    s = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)
    assert s.C == pytest.approx(0.2)


def test_C_strictly_between_zero_and_one():
    for kb, km in [(1.0, 1e9), (1e9, 1.0), (5.0, 5.0)]:
        s = JointStiffness(bolt_stiffness=kb, member_stiffness=km)
        assert 0.0 < s.C < 1.0


def test_higher_member_stiffness_lowers_C():
    base = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)
    stiffer_member = JointStiffness(bolt_stiffness=1e8, member_stiffness=8e8)
    assert stiffer_member.C < base.C


def test_higher_bolt_stiffness_raises_C():
    base = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)
    stiffer_bolt = JointStiffness(bolt_stiffness=2e8, member_stiffness=4e8)
    assert stiffer_bolt.C > base.C


# ---------------------------------------------------------------------------
# F-K. preload load sharing
# ---------------------------------------------------------------------------


def test_pure_separating_load_hand_calc():
    # 4-bolt symmetric pattern, pure Fz -> every bolt identical T_sep.
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0)  # T=10kN/bolt
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=20_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    C = STIFFNESS.C
    assert b0.separating_demand == pytest.approx(10_000.0)
    assert b0.additional_bolt_load == pytest.approx(C * 10_000.0)
    assert b0.total_bolt_tension == pytest.approx(20_000.0 + C * 10_000.0)
    assert b0.clamp_force_reduction == pytest.approx((1 - C) * 10_000.0)
    assert b0.remaining_clamp_force == pytest.approx(20_000.0 - (1 - C) * 10_000.0)


def test_zero_separating_load():
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad())  # all zero
    preload = PreloadState(preload_per_bolt=15_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    for b in group.bolts:
        assert b.separating_demand == 0.0
        assert b.additional_bolt_load == 0.0
        assert b.clamp_force_reduction == 0.0
        assert b.remaining_clamp_force == pytest.approx(15_000.0)
        assert b.separation_margin is None
        assert b.separation_pass is True


def test_compression_side_external_load_does_not_reduce_clamp():
    # Construct a bolt with negative axial_total (compression side).
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=5000.0, Mx=50000.0)
    result = distribute_loads(pattern, load)
    compression_bolts = [b for b in result.bolts if b.axial_total < 0]
    assert compression_bolts, "expected at least one compression-side bolt"

    preload = PreloadState(preload_per_bolt=10_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    for b_ml1 in compression_bolts:
        b = group.bolts[b_ml1.index]
        assert b.separating_demand == 0.0
        assert b.clamp_force_reduction == 0.0
        assert b.remaining_clamp_force == pytest.approx(10_000.0)  # full preload retained
        # signed load preserved for reporting
        assert b.axial_total == pytest.approx(b_ml1.axial_total)


def test_bolt_load_increment_equals_C_times_T_sep():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=20_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    for b in group.bolts:
        assert b.additional_bolt_load == pytest.approx(C * b.separating_demand)


def test_clamp_reduction_equals_1_minus_C_times_T_sep():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=20_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    for b in group.bolts:
        assert b.clamp_force_reduction == pytest.approx((1 - C) * b.separating_demand)


def test_force_balance_additional_bolt_load_plus_member_unloading():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=20_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    for b in group.bolts:
        assert b.additional_bolt_load + b.clamp_force_reduction == pytest.approx(b.separating_demand)


# ---------------------------------------------------------------------------
# L-Q. separation
# ---------------------------------------------------------------------------


def test_separation_exact_boundary():
    C = STIFFNESS.C
    T_sep = 10_000.0
    preload_val = (1 - C) * T_sep  # boundary: remaining clamp == 0 exactly
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=4 * T_sep)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.remaining_clamp_force == pytest.approx(0.0, abs=1e-6)
    assert b0.separation_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.separation_pass is True  # boundary included in PASS


def test_separation_below_boundary_fails():
    C = STIFFNESS.C
    T_sep = 10_000.0
    preload_val = (1 - C) * T_sep * 0.9  # below requirement
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=4 * T_sep)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.remaining_clamp_force < 0.0
    assert b0.separation_margin < 0.0
    assert b0.separation_pass is False


def test_separation_above_boundary_passes():
    C = STIFFNESS.C
    T_sep = 10_000.0
    preload_val = (1 - C) * T_sep * 1.1
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=4 * T_sep)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.remaining_clamp_force > 0.0
    assert b0.separation_margin > 0.0
    assert b0.separation_pass is True


def test_zero_separating_demand_passes():
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad())
    preload = PreloadState(preload_per_bolt=5000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.all_locations_closed is True


def test_required_separation_preload_hand_calc():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    T_sep = [max(b.axial_total, 0.0) for b in result.bolts]
    expected = max((1 - C) * t for t in T_sep)
    assert req.separation_required == pytest.approx(expected, rel=1e-9)


def test_group_required_preload_is_max_local_requirement():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    mu_n = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces
    manual_slip_terms = [
        (1 - C) * max(b.axial_total, 0.0) + b.shear_resultant / mu_n for b in result.bolts
    ]
    assert req.slip_required == pytest.approx(max(manual_slip_terms), rel=1e-9)
    assert req.overall_required == pytest.approx(max(req.separation_required, req.slip_required), rel=1e-9)


# ---------------------------------------------------------------------------
# R-Y. friction / slip
# ---------------------------------------------------------------------------


def test_friction_model_validation():
    with pytest.raises(ValueError):
        FrictionModel(friction_coefficient=0.0)
    with pytest.raises(ValueError):
        FrictionModel(friction_coefficient=-0.2)
    with pytest.raises(ValueError):
        FrictionModel(friction_coefficient=0.2, number_of_faying_surfaces=0)
    with pytest.raises(ValueError):
        FrictionModel(friction_coefficient=0.2, number_of_faying_surfaces=1.5)


def test_local_friction_capacity_hand_calc():
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fx=8000.0))  # V=2kN/bolt, no axial
    preload = PreloadState(preload_per_bolt=10_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    expected_capacity = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces * 10_000.0
    assert b0.friction_capacity == pytest.approx(expected_capacity)
    assert b0.shear_demand == pytest.approx(2000.0)


def test_slip_exact_boundary():
    preload_val = 10_000.0
    V = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces * preload_val  # boundary
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fx=4 * V))
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.slip_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.slip_pass is True


def test_slip_below_boundary_fails():
    preload_val = 10_000.0
    V = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces * preload_val * 1.1
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fx=4 * V))
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.slip_margin < 0.0
    assert b0.slip_pass is False


def test_slip_above_boundary_passes():
    preload_val = 10_000.0
    V = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces * preload_val * 0.9
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fx=4 * V))
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.slip_margin > 0.0
    assert b0.slip_pass is True


def test_zero_shear_passes():
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fz=40_000.0))  # axial only, no shear
    preload = PreloadState(preload_per_bolt=20_000.0)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    for b in group.bolts:
        assert b.shear_demand == 0.0
        assert b.slip_margin is None
        assert b.slip_pass is True


def test_zero_remaining_clamp_and_nonzero_shear_fails_cleanly():
    # Preload fully consumed by separation demand (remaining clamp == 0),
    # while shear demand is nonzero -> friction capacity 0, slip fails
    # with the documented explicit margin = -1 convention (no NaN/inf).
    C = STIFFNESS.C
    T_sep = 10_000.0
    preload_val = (1 - C) * T_sep  # remaining clamp == 0 exactly
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=4 * T_sep, Fx=4000.0)
    result = distribute_loads(pattern, load)
    preload = PreloadState(preload_per_bolt=preload_val)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    b0 = group.bolts[0]
    assert b0.remaining_clamp_force == pytest.approx(0.0, abs=1e-6)
    assert b0.friction_capacity == pytest.approx(0.0, abs=1e-6)
    assert b0.shear_demand > 0.0
    assert b0.slip_margin == pytest.approx(-1.0)
    assert b0.slip_pass is False
    assert math.isfinite(b0.slip_margin)


def test_mz_generated_shear_included_via_milestone1_shear_demand():
    # Pure Mz produces per-bolt shear (Milestone 1) with zero axial
    # anywhere -> this shear demand must show up as slip shear_demand.
    pattern = circular_pattern(8, 0.5)
    result = distribute_loads(pattern, InterfaceLoad(Mz=12000.0))
    preload = PreloadState(preload_per_bolt=100.0)  # deliberately small -> slip likely
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    for b_ml1, b in zip(result.bolts, group.bolts):
        assert b.shear_demand == pytest.approx(b_ml1.shear_resultant)
    assert any(b.shear_demand > 0 for b in group.bolts)


# ---------------------------------------------------------------------------
# Z-AD. required preload
# ---------------------------------------------------------------------------


def test_required_slip_preload_hand_calc():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    mu_n = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces
    terms = [(1 - C) * max(b.axial_total, 0.0) + b.shear_resultant / mu_n for b in result.bolts]
    idx = max(range(len(terms)), key=lambda i: terms[i])
    assert req.slip_required == pytest.approx(terms[idx], rel=1e-9)


def test_overall_required_preload_hand_calc():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    assert req.overall_required == pytest.approx(max(req.separation_required, req.slip_required), rel=1e-9)
    assert req.overall_governing_constraint in ("separation", "slip")


def test_required_preload_satisfies_all_bolts():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    preload = PreloadState(preload_per_bolt=req.overall_required)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.all_locations_closed is True
    assert group.all_locations_no_slip is True
    assert group.passed is True
    # boundary: at least one bolt is exactly at margin 0 (the governing one)
    assert group.min_slip_margin == pytest.approx(0.0, abs=1e-6) or group.min_separation_margin == pytest.approx(
        0.0, abs=1e-6
    )


def test_slightly_lower_preload_fails_active_constraint():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    preload = PreloadState(preload_per_bolt=req.overall_required * 0.99)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.passed is False


def test_slightly_higher_preload_passes():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    preload = PreloadState(preload_per_bolt=req.overall_required * 1.01)
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.passed is True


# ---------------------------------------------------------------------------
# AE-AJ. sensitivity / determinism
# ---------------------------------------------------------------------------


def test_required_slip_preload_decreases_with_mu():
    result = _sanity_group_result()
    mus = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40]
    required = [required_preload(result, STIFFNESS, FrictionModel(friction_coefficient=mu)).slip_required for mu in mus]
    for i in range(len(required) - 1):
        assert required[i + 1] < required[i]


def test_separation_required_preload_decreases_as_C_increases():
    result = _sanity_group_result()
    kb = 1e8
    Cs = [0.10, 0.20, 0.30, 0.40]
    required = []
    for C in Cs:
        km = kb * (1 - C) / C
        stiffness = JointStiffness(bolt_stiffness=kb, member_stiffness=km)
        required.append(required_preload(result, stiffness, FRICTION).separation_required)
    for i in range(len(required) - 1):
        assert required[i + 1] < required[i]


def test_preload_sensitivity_monotonic_margin_improvement():
    result = _sanity_group_result()
    req = required_preload(result, STIFFNESS, FRICTION)
    factors = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]
    min_slip_margins = []
    min_sep_margins = []
    for f in factors:
        preload = PreloadState(preload_per_bolt=req.overall_required * f)
        group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
        min_slip_margins.append(group.min_slip_margin)
        min_sep_margins.append(group.min_separation_margin)
    for i in range(len(factors) - 1):
        assert min_slip_margins[i + 1] > min_slip_margins[i]
        assert min_sep_margins[i + 1] > min_sep_margins[i]
    # boundary at factor 1.0 (index 2): governing constraint at margin ~0
    assert min(abs(min_slip_margins[2]), abs(min_sep_margins[2])) < 1e-6


def test_deterministic_governing_separation_bolt():
    # symmetric pattern, pure Fz -> all bolts tied -> lowest index wins
    pattern = circular_pattern(6, 0.4)
    result = distribute_loads(pattern, InterfaceLoad(Fz=6000.0))
    preload = PreloadState(preload_per_bolt=100.0)  # force separation everywhere, all tied
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.governing_separation_bolt_index == 0


def test_deterministic_governing_slip_bolt():
    pattern = circular_pattern(6, 0.4)
    result = distribute_loads(pattern, InterfaceLoad(Fx=6000.0))  # equal shear, no axial -> tied
    preload = PreloadState(preload_per_bolt=1.0)  # force slip everywhere, all tied
    group = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert group.governing_slip_bolt_index == 0


def test_repeated_assessment_deterministic():
    result = _sanity_group_result()
    preload = PreloadState(preload_per_bolt=30_000.0)
    g1 = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    g2 = assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert g1 == g2


def test_apply_preload_factor_validation_and_hand_calc():
    assert apply_preload_factor(1000.0, 1.0) == pytest.approx(1000.0)
    assert apply_preload_factor(1000.0, 1.5) == pytest.approx(1500.0)
    with pytest.raises(ValueError):
        apply_preload_factor(1000.0, 0.9)  # must be >= 1.0
    with pytest.raises(ValueError):
        apply_preload_factor(-1.0, 1.0)


def test_preload_state_validation():
    with pytest.raises(ValueError):
        PreloadState(preload_per_bolt=0.0)
    with pytest.raises(ValueError):
        PreloadState(preload_per_bolt=-5.0)
    with pytest.raises(ValueError):
        PreloadState(preload_per_bolt=float("inf"))


# ---------------------------------------------------------------------------
# AK-AM. prior-milestone regression
# ---------------------------------------------------------------------------


def test_milestone1_result_not_mutated_by_preload_assessment():
    result = _sanity_group_result()
    before_bolts = tuple(result.bolts)
    before_eq = result.equilibrium
    preload = PreloadState(preload_per_bolt=30_000.0)
    assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)
    assert result.bolts == before_bolts
    assert result.equilibrium == before_eq


def test_milestone2_strength_result_unaffected_by_preload_module():
    result = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.008)
    mat = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
    strength_before = assess_bolt_group_strength(result, sec, mat)

    preload = PreloadState(preload_per_bolt=30_000.0)
    assess_preloaded_joint(result, preload, STIFFNESS, FRICTION)  # Milestone 3 call

    strength_after = assess_bolt_group_strength(result, sec, mat)
    assert strength_before == strength_after
    assert strength_after.governing_bolt_index == 1
    assert strength_after.governing_mode == "interaction"
    assert strength_after.passed is True
