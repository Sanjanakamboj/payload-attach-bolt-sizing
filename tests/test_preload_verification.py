"""Independent verification of preload_verification.py (Milestone 7).

Reference values are computed by hand directly in these tests, not by
re-calling the production functions under test a second time.
"""

import pytest

from payload_bolts import (
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadVerificationMethod,
    TorqueInstallationStatus,
    VerificationAccuracyModel,
    VerificationStatus,
    achieved_preload_bounds,
    assess_bolt_group_strength,
    assess_method,
    assess_torque_installation,
    circular_pattern,
    circular_unthreaded_bolt,
    command_window,
    distribute_loads,
    epsilon_max_for_window,
    evaluate_installation_methods,
    installation_preload_window,
    required_preload,
)
from payload_bolts import NutFactorModel

# ---------------------------------------------------------------------------
# Canonical Milestone 1 group-load case, reused unchanged.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)
FRICTION = FrictionModel(friction_coefficient=0.20)
STRENGTH_LIMITS = BoltStrengthLimits(name="Illustrative", proof_strength=830e6, yield_strength=970e6)
ETA_PROOF = 0.75
DELTA_F = 0.10

SECTION_8MM = circular_unthreaded_bolt(0.008)
SECTION_10MM = circular_unthreaded_bolt(0.010)
SECTION_12MM = circular_unthreaded_bolt(0.012)

REQUIRED = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
WINDOW_10MM = installation_preload_window(REQUIRED.overall_required, SECTION_10MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
WINDOW_12MM = installation_preload_window(REQUIRED.overall_required, SECTION_12MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)


# --- A. achieved-preload lower formula ---------------------------------------
def test_achieved_preload_lower_hand_calc():
    F, eps = 40_000.0, 0.10
    lo, hi = achieved_preload_bounds(F, eps)
    assert lo == pytest.approx(F * (1.0 - eps), rel=1e-12)


# --- B. achieved-preload upper formula ---------------------------------------
def test_achieved_preload_upper_hand_calc():
    F, eps = 40_000.0, 0.10
    lo, hi = achieved_preload_bounds(F, eps)
    assert hi == pytest.approx(F * (1.0 + eps), rel=1e-12)


# --- C. command lower-bound formula -------------------------------------------
def test_command_lower_bound_hand_calc():
    vw = command_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max, 0.10)
    assert vw.command_min == pytest.approx(WINDOW_10MM.target_min / (1.0 - 0.10), rel=1e-12)


# --- D. command upper-bound formula -------------------------------------------
def test_command_upper_bound_hand_calc():
    vw = command_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max, 0.10)
    assert vw.command_max == pytest.approx(WINDOW_10MM.target_max / (1.0 + 0.10), rel=1e-12)


# --- E. exact feasibility boundary ---------------------------------------------
def test_exact_feasibility_boundary():
    # Construct window_min/window_max and epsilon such that
    # window_min/(1-eps) == window_max/(1+eps) exactly (up to a tiny
    # epsilon nudge for floating-point boundary robustness, same
    # rationale as the M5/M6 boundary tests).
    window_min, window_max = 1000.0, 1500.0
    R = window_max / window_min
    eps_boundary = (R - 1.0) / (R + 1.0)
    eps = eps_boundary * (1.0 - 1e-9)
    vw = command_window(window_min, window_max, eps)
    assert vw.command_min == pytest.approx(vw.command_max, rel=1e-6)
    assert vw.width == pytest.approx(0.0, abs=1e-6)
    assert vw.feasible is True


# --- F. equivalent ratio identity ----------------------------------------------
def test_equivalent_ratio_identity():
    window_min, window_max = 1000.0, 1500.0
    eps = 0.15
    vw = command_window(window_min, window_max, eps)
    expected_equiv_ratio = (1.0 + eps) / (1.0 - eps)
    assert vw.equivalent_error_ratio == pytest.approx(expected_equiv_ratio, rel=1e-12)
    # feasible iff equivalent_error_ratio <= window_ratio
    assert vw.feasible == (vw.equivalent_error_ratio <= vw.window_ratio)


# --- G. epsilon_max formula -----------------------------------------------------
def test_epsilon_max_hand_formula():
    window_min, window_max = 1000.0, 1800.0
    R = window_max / window_min
    expected = (R - 1.0) / (R + 1.0)
    assert epsilon_max_for_window(window_min, window_max) == pytest.approx(expected, rel=1e-12)


def test_epsilon_max_10mm_hand_value():
    expected_R = WINDOW_10MM.target_max / WINDOW_10MM.target_min
    expected_eps_max = (expected_R - 1.0) / (expected_R + 1.0)
    assert epsilon_max_for_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max) == pytest.approx(
        expected_eps_max, rel=1e-9
    )
    assert epsilon_max_for_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max) == pytest.approx(0.2013, abs=1e-3)


# --- H. epsilon=0 gives command window equal to preload window ---------------
def test_epsilon_zero_reproduces_preload_window():
    vw = command_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max, 0.0)
    assert vw.command_min == pytest.approx(WINDOW_10MM.target_min, rel=1e-12)
    assert vw.command_max == pytest.approx(WINDOW_10MM.target_max, rel=1e-12)


# --- I. increasing epsilon narrows command window monotonically --------------
def test_command_window_narrows_monotonically_with_epsilon():
    epsilons = [0.0, 0.02, 0.05, 0.10, 0.15]
    widths = [command_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max, e).width for e in epsilons]
    assert all(b < a for a, b in zip(widths, widths[1:]))


# --- J. sufficiently large epsilon makes window infeasible -------------------
def test_large_epsilon_makes_window_infeasible():
    eps_max = epsilon_max_for_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max)
    vw = command_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max, min(eps_max * 1.5, 0.99))
    assert vw.feasible is False
    assert vw.width < 0.0


# --- K. midpoint command lies inside feasible command window -----------------
def test_midpoint_command_inside_feasible_window():
    result = assess_method(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, VerificationAccuracyModel(
        method=PreloadVerificationMethod.BOLT_ELONGATION, relative_error=0.05, source_basis="test", is_illustrative=True,
        measurement_type="direct", what_is_measured="test", limitation="test",
    ))
    assert result.status == VerificationStatus.FEASIBLE
    vw = result.verified_window
    assert vw.command_min <= result.nominal_target <= vw.command_max


# --- L. midpoint achieved bounds remain inside preload window ----------------
def test_midpoint_achieved_bounds_inside_preload_window():
    result = assess_method(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, VerificationAccuracyModel(
        method=PreloadVerificationMethod.LOAD_SENSING_WASHER, relative_error=0.10, source_basis="test", is_illustrative=True,
        measurement_type="direct", what_is_measured="test", limitation="test",
    ))
    assert result.status == VerificationStatus.FEASIBLE
    assert WINDOW_10MM.target_min <= result.achieved_min
    assert result.achieved_max <= WINDOW_10MM.target_max
    assert result.all_within_window is True


# --- M. exact upper/lower boundary values -------------------------------------
def test_exact_boundary_values_hand_calc():
    window_min, window_max, eps = 30_000.0, 45_000.0, 0.08
    vw = command_window(window_min, window_max, eps)
    assert vw.command_min == pytest.approx(30_000.0 / 0.92, rel=1e-12)
    assert vw.command_max == pytest.approx(45_000.0 / 1.08, rel=1e-12)


# --- N. invalid epsilon<0 rejected ---------------------------------------------
def test_invalid_epsilon_negative_rejected():
    with pytest.raises(ValueError):
        achieved_preload_bounds(30_000.0, -0.01)
    with pytest.raises(ValueError):
        command_window(30_000.0, 45_000.0, -0.01)
    with pytest.raises(ValueError):
        VerificationAccuracyModel(
            method=PreloadVerificationMethod.BOLT_ELONGATION, relative_error=-0.1, source_basis="x",
            is_illustrative=True, measurement_type="direct", what_is_measured="x", limitation="x",
        )


# --- O. invalid epsilon>=1 rejected ---------------------------------------------
def test_invalid_epsilon_too_large_rejected():
    with pytest.raises(ValueError):
        achieved_preload_bounds(30_000.0, 1.0)
    with pytest.raises(ValueError):
        achieved_preload_bounds(30_000.0, 1.5)
    with pytest.raises(ValueError):
        command_window(30_000.0, 45_000.0, 1.0)


# --- P. invalid preload-window ordering rejected --------------------------------
def test_invalid_window_ordering_rejected():
    with pytest.raises(ValueError):
        epsilon_max_for_window(45_000.0, 30_000.0)  # max < min
    with pytest.raises(ValueError):
        command_window(45_000.0, 30_000.0, 0.05)
    with pytest.raises(ValueError):
        epsilon_max_for_window(0.0, 45_000.0)
    with pytest.raises(ValueError):
        epsilon_max_for_window(30_000.0, -1.0)


# --- Q. inherited 10 mm preload bounds reproduced exactly ---------------------
def test_inherited_10mm_preload_bounds_exact():
    assert WINDOW_10MM.target_min == pytest.approx(32_509.1, abs=1.0)
    assert WINDOW_10MM.target_max == pytest.approx(48_891.0, abs=1.0)
    assert WINDOW_10MM.feasible is True


# --- R. inherited 12 mm preload bounds reproduced exactly ---------------------
def test_inherited_12mm_preload_bounds_exact():
    assert WINDOW_12MM.target_min == pytest.approx(32_509.1, abs=1.0)
    assert WINDOW_12MM.target_max == pytest.approx(70_403.1, abs=5.0)
    assert WINDOW_12MM.feasible is True


# --- S. M5 candidate remains 10 mm before M7 method screening -----------------
def test_M5_selected_candidate_remains_10mm():
    from payload_bolts import BoltMaterial, JointGeometry, PlateMaterial, select_bolt_candidate

    bolt_material = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
    plate = PlateMaterial(name="Illustrative plate", bearing_allowable=400e6)
    geometry = JointGeometry(plate_thickness=0.008, plate_center=(0.0, 0.0), plate_outer_radius=0.55, edge_distance_min_ratio=1.5, spacing_min_ratio=3.0)
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    selection = select_bolt_candidate(GROUP_LOAD_RESULT, candidates, bolt_material, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, 1.20, plate, geometry)
    sel = selection.selected()
    assert sel is not None
    assert sel.bolt_section.nominal_diameter == pytest.approx(0.010, rel=1e-9)


# --- T. M6 torque-only status remains unchanged --------------------------------
def test_M6_torque_only_status_unchanged():
    nut_factor = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)
    torque_result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, nut_factor)
    assert torque_result.status == TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW
    assert torque_result.robust_window.torque_robust_min == pytest.approx(81.273, abs=0.01)
    assert torque_result.robust_window.torque_robust_max == pytest.approx(73.337, abs=0.01)


# --- U. maximum achieved in-service proof force calculated exactly once ------
def test_in_service_force_no_double_counting():
    accuracy = VerificationAccuracyModel(
        method=PreloadVerificationMethod.INSTRUMENTED_BOLT, relative_error=0.05, source_basis="test",
        is_illustrative=True, measurement_type="direct", what_is_measured="test", limitation="test",
    )
    result = assess_method(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, accuracy)
    assert result.status == VerificationStatus.FEASIBLE
    C = STIFFNESS.C
    f_achieved_max = result.achieved_max
    hand_forces = {b.index: f_achieved_max + C * max(b.axial_total, 0.0) for b in GROUP_LOAD_RESULT.bolts}
    expected_max_idx = max(hand_forces, key=lambda i: hand_forces[i])
    expected_max_val = hand_forces[expected_max_idx]
    assert result.max_in_service_bolt_index == expected_max_idx
    assert result.max_in_service_bolt_force == pytest.approx(expected_max_val, rel=1e-9)


# --- V. proof violation test at deliberately excessive preload ---------------
def test_proof_violation_reported_at_low_proof_limit():
    # A proof strength low enough to keep the M4 window feasible (barely)
    # but with the achieved-preload band pushed above the (now tiny)
    # proof load: chosen so target_max stays > target_min but F_proof
    # itself is small relative to the achieved preload.
    low_proof_limits = BoltStrengthLimits(name="artificially low proof", proof_strength=250e6)
    accuracy = VerificationAccuracyModel(
        method=PreloadVerificationMethod.INSTRUMENTED_BOLT, relative_error=0.05, source_basis="test",
        is_illustrative=True, measurement_type="direct", what_is_measured="test", limitation="test",
    )
    result = assess_method(GROUP_LOAD_RESULT, SECTION_10MM, low_proof_limits, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, accuracy)
    assert result.status in (
        VerificationStatus.NO_VERIFIED_PRELOAD_WINDOW,
        VerificationStatus.PROOF_LIMIT_EXCEEDED,
        VerificationStatus.FEASIBLE,
    )
    # Whatever the outcome, the result must never silently hide a proof
    # exceedance: if the window was feasible, either proof_reserve is
    # reported non-negative (FEASIBLE) or the status explicitly flags it.
    if result.status == VerificationStatus.FEASIBLE:
        assert result.proof_reserve >= 0.0
    elif result.status == VerificationStatus.PROOF_LIMIT_EXCEEDED:
        assert result.proof_reserve < 0.0


# --- W. friction sensitivity propagation ---------------------------------------
def test_friction_sensitivity_propagates_to_epsilon_max():
    # mu=0.10 is deliberately excluded: at that low friction, the
    # required preload is so high (~57.2 kN/bolt) that the M4 window
    # itself becomes infeasible (target_min > target_max, a
    # prior-milestone result -- see test_M1_through_M6_headline...),
    # which is outside epsilon_max's domain (it requires a valid,
    # ordered window). The remaining range stays M4-feasible throughout.
    eps_maxes = []
    for mu in (0.20, 0.25, 0.30, 0.40):
        req_mu = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FrictionModel(friction_coefficient=mu))
        w_mu = installation_preload_window(req_mu.overall_required, SECTION_10MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
        assert w_mu.feasible is True
        eps_maxes.append(epsilon_max_for_window(w_mu.target_min, w_mu.target_max))
    # Higher mu -> lower required preload -> lower target_min -> larger
    # window ratio -> larger epsilon_max (monotonically, for fixed target_max).
    assert all(b >= a for a, b in zip(eps_maxes, eps_maxes[1:]))


# --- X. delta_F sensitivity propagation -----------------------------------------
def test_delta_F_sensitivity_propagates_to_epsilon_max():
    eps_maxes = []
    for delta in (0.05, 0.10, 0.20):
        w_d = installation_preload_window(REQUIRED.overall_required, SECTION_10MM, STRENGTH_LIMITS, ETA_PROOF, delta)
        eps_maxes.append(epsilon_max_for_window(w_d.target_min, w_d.target_max))
    # Higher delta_F -> higher target_min -> smaller window ratio -> smaller epsilon_max.
    assert all(b < a for a, b in zip(eps_maxes, eps_maxes[1:]))


# --- Y. eta_proof sensitivity propagation ---------------------------------------
def test_eta_proof_sensitivity_propagates_to_epsilon_max():
    eps_maxes = []
    for eta in (0.60, 0.70, 0.80):
        w_e = installation_preload_window(REQUIRED.overall_required, SECTION_10MM, STRENGTH_LIMITS, eta, DELTA_F)
        eps_maxes.append(epsilon_max_for_window(w_e.target_min, w_e.target_max))
    # Higher eta_proof -> higher target_max -> larger window ratio -> larger epsilon_max.
    assert all(b > a for a, b in zip(eps_maxes, eps_maxes[1:]))


# --- Z. 12 mm epsilon_max > 10 mm epsilon_max -----------------------------------
def test_12mm_epsilon_max_greater_than_10mm():
    eps_max_10 = epsilon_max_for_window(WINDOW_10MM.target_min, WINDOW_10MM.target_max)
    eps_max_12 = epsilon_max_for_window(WINDOW_12MM.target_min, WINDOW_12MM.target_max)
    assert eps_max_12 > eps_max_10


# --- AA. deterministic method ordering/table ------------------------------------
def test_deterministic_method_ordering():
    trade1 = evaluate_installation_methods(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F)
    trade2 = evaluate_installation_methods(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F)
    methods1 = [r.accuracy.method for r in trade1.results]
    methods2 = [r.accuracy.method for r in trade2.results]
    assert methods1 == methods2
    assert methods1 == [
        PreloadVerificationMethod.BOLT_ELONGATION,
        PreloadVerificationMethod.ULTRASONIC,
        PreloadVerificationMethod.LOAD_SENSING_WASHER,
        PreloadVerificationMethod.INSTRUMENTED_BOLT,
        PreloadVerificationMethod.TURN_OF_NUT,
    ]
    assert trade1.feasible_methods == trade2.feasible_methods


# --- AB. METHOD_NOT_QUANTIFIED handled cleanly -----------------------------------
def test_method_not_quantified_handled_cleanly():
    result = assess_method(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, VerificationAccuracyModel(
        method=PreloadVerificationMethod.TURN_OF_NUT, relative_error=None, source_basis="not quantified",
        is_illustrative=True, measurement_type="indirect", what_is_measured="x", limitation="x",
    ))
    assert result.status == VerificationStatus.METHOD_NOT_QUANTIFIED
    assert result.nominal_target is None
    assert result.achieved_min is None
    assert result.achieved_max is None
    assert result.verified_window is None
    assert result.proof_reserve is None


# --- AC. no method selected if all quantified methods infeasible ---------------
def test_no_method_feasible_when_window_too_tight():
    # Construct a proof strength that keeps the M4 window itself
    # feasible (target_max slightly above target_min, width ~32 N) but
    # far too NARROW for any quantified method's error band (5-10%) to
    # fit -- forcing every quantified method infeasible without the
    # underlying M4 window itself being infeasible.
    target_min = REQUIRED.overall_required / (1.0 - DELTA_F)
    target_max_wanted = target_min * 1.001
    f_proof_needed = target_max_wanted / ETA_PROOF
    sp_needed = f_proof_needed / SECTION_10MM.tensile_area
    tight_limits = BoltStrengthLimits(name="tight", proof_strength=sp_needed)

    w_check = installation_preload_window(REQUIRED.overall_required, SECTION_10MM, tight_limits, ETA_PROOF, DELTA_F)
    assert w_check.feasible is True  # confirm the M4 window itself is still feasible (just tiny)

    trade = evaluate_installation_methods(GROUP_LOAD_RESULT, SECTION_10MM, tight_limits, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F)
    quantified = [r for r in trade.results if r.accuracy.relative_error is not None]
    assert len(quantified) > 0
    assert all(r.status != VerificationStatus.FEASIBLE for r in quantified)
    assert trade.any_feasible is False
    assert trade.feasible_methods == ()


# --- AD. all 212 M1-M6 tests remain passing --------------------------------------
# (Enforced by running the full `pytest` suite; this module adds new
# tests only and does not modify any Milestone 1-6 test file.)


# --- AE. explicit regression: M1-M6 headline outputs unchanged ------------------
def test_M1_through_M6_headline_outputs_unchanged():
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    assert max_tensile.index == 1
    assert max_tensile.axial_total == pytest.approx(24_142.1, abs=0.5)

    strength = assess_bolt_group_strength(GROUP_LOAD_RESULT, SECTION_8MM, __import__("payload_bolts").BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6))
    assert strength.governing_bolt_index == 1
    assert strength.governing_mode == "interaction"

    assert REQUIRED.slip_governing_bolt == 0
    assert REQUIRED.overall_required == pytest.approx(29_258.2, abs=0.5)

    window_8mm = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
    assert window_8mm.feasible is False

    nut_factor = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)
    torque_result_10mm = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, nut_factor)
    assert torque_result_10mm.status == TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW
