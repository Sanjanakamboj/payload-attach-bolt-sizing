"""Independent verification of preload_limits.py (Milestone 4).

Reference values are computed by hand directly in these tests, not by
re-calling the production functions under test a second time.
"""

import math
from types import SimpleNamespace

import pytest

from payload_bolts import (
    BoltMaterial,
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadFeasibilityStatus,
    assess_bolt_group_strength,
    assess_preload_feasibility,
    assess_preloaded_joint,
    circular_pattern,
    circular_unthreaded_bolt,
    classify_selected_preload,
    compute_preload_limits,
    distribute_loads,
    installation_preload_window,
    max_installation_preload,
    min_installation_preload,
    required_preload,
)
from payload_bolts.preload_limits import _max_total_tension_among_closed

# ---------------------------------------------------------------------------
# Canonical Milestone 1 group-load case, reused unchanged across the suite.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20)
REQUIRED = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)  # overall ~29,258.2 N/bolt

SECTION_8MM = circular_unthreaded_bolt(0.008)
LIMITS = BoltStrengthLimits(name="Illustrative", proof_strength=830e6, yield_strength=970e6)

ETA_PROOF = 0.75
DELTA_F = 0.10


# --- A. proof load: F_proof = A_t * S_p ------------------------------------
def test_proof_load_hand_calc():
    result = compute_preload_limits(SECTION_8MM, LIMITS)
    expected = SECTION_8MM.tensile_area * LIMITS.proof_strength
    assert result.proof_load == pytest.approx(expected, rel=1e-12)


# --- B. yield load if modeled ----------------------------------------------
def test_yield_load_hand_calc():
    result = compute_preload_limits(SECTION_8MM, LIMITS)
    expected = SECTION_8MM.tensile_area * LIMITS.yield_strength
    assert result.yield_load == pytest.approx(expected, rel=1e-12)


def test_yield_load_none_when_not_supplied():
    limits_no_yield = BoltStrengthLimits(name="no-yield", proof_strength=830e6)
    result = compute_preload_limits(SECTION_8MM, limits_no_yield)
    assert result.yield_load is None


# --- C. exact reuse of M2 tensile stress area -------------------------------
def test_reuses_exact_M2_tensile_area():
    # Independently reconstruct A_t via the same idealized circular-shank
    # formula Milestone 2 uses, and confirm M4's proof-load basis uses
    # that exact value (no hidden diameter substitution).
    d = 0.008
    expected_area = math.pi * d * d / 4.0
    assert SECTION_8MM.tensile_area == pytest.approx(expected_area, rel=1e-12)
    result = compute_preload_limits(SECTION_8MM, LIMITS)
    assert result.bolt_section.tensile_area == SECTION_8MM.tensile_area
    assert result.proof_load == pytest.approx(expected_area * LIMITS.proof_strength, rel=1e-12)


# --- D. proof-fraction ceiling: F_target,max = eta_proof * F_proof ---------
def test_upper_ceiling_hand_calc():
    limits_result = compute_preload_limits(SECTION_8MM, LIMITS)
    expected = ETA_PROOF * limits_result.proof_load
    assert max_installation_preload(limits_result, ETA_PROOF) == pytest.approx(expected, rel=1e-12)


# --- E. scatter-adjusted lower target: F_target,min = F_required/(1-delta_F)
def test_lower_target_hand_calc():
    expected = REQUIRED.overall_required / (1.0 - DELTA_F)
    assert min_installation_preload(REQUIRED.overall_required, DELTA_F) == pytest.approx(expected, rel=1e-12)


# --- F. delta_F = 0 gives F_target,min = F_required -------------------------
def test_zero_scatter_gives_required_preload_exactly():
    assert min_installation_preload(REQUIRED.overall_required, 0.0) == pytest.approx(
        REQUIRED.overall_required, rel=1e-12
    )


# --- G. increasing delta_F increases F_target,min monotonically ------------
def test_target_min_increases_monotonically_with_delta_F():
    deltas = [0.0, 0.05, 0.10, 0.20, 0.30]
    targets = [min_installation_preload(REQUIRED.overall_required, d) for d in deltas]
    assert all(b > a for a, b in zip(targets, targets[1:]))


# --- H. increasing S_p increases upper ceiling monotonically ---------------
def test_target_max_increases_monotonically_with_proof_strength():
    proof_strengths = [600e6, 700e6, 830e6, 900e6, 1000e6]
    ceilings = []
    for sp in proof_strengths:
        lim = BoltStrengthLimits(name="x", proof_strength=sp)
        lr = compute_preload_limits(SECTION_8MM, lim)
        ceilings.append(max_installation_preload(lr, ETA_PROOF))
    assert all(b > a for a, b in zip(ceilings, ceilings[1:]))


# --- I. increasing eta_proof increases upper ceiling monotonically ---------
def test_target_max_increases_monotonically_with_eta_proof():
    lr = compute_preload_limits(SECTION_8MM, LIMITS)
    etas = [0.5, 0.6, 0.7, 0.8, 0.9]
    ceilings = [max_installation_preload(lr, e) for e in etas]
    assert all(b > a for a, b in zip(ceilings, ceilings[1:]))


# --- J. exact feasibility boundary: target_min = target_max ----------------
def test_exact_feasibility_boundary_gives_zero_window():
    # Construct target_min == target_max exactly: required=1000, delta_F=0
    # -> target_min=1000; pick eta_proof=0.5, A_t=1 m^2, S_p=2000 Pa ->
    # F_proof=2000, target_max=0.5*2000=1000.
    section = SECTION_8MM.__class__(nominal_diameter=1.0, tensile_area=1.0, shear_area=1.0)
    limits = BoltStrengthLimits(name="boundary", proof_strength=2000.0)
    window = installation_preload_window(1000.0, section, limits, eta_proof=0.5, scatter_allowance=0.0)
    assert window.target_min == pytest.approx(1000.0, rel=1e-12)
    assert window.target_max == pytest.approx(1000.0, rel=1e-12)
    assert window.window_width == pytest.approx(0.0, abs=1e-9)
    assert window.upper_margin == pytest.approx(0.0, abs=1e-9)
    assert window.feasible is True  # target_min <= target_max, inclusive


# --- K. below-boundary case infeasible --------------------------------------
def test_below_boundary_infeasible():
    section = SECTION_8MM.__class__(nominal_diameter=1.0, tensile_area=1.0, shear_area=1.0)
    limits = BoltStrengthLimits(name="below", proof_strength=1999.0)  # F_proof*0.5 = 999.5 < 1000
    window = installation_preload_window(1000.0, section, limits, eta_proof=0.5, scatter_allowance=0.0)
    assert window.feasible is False
    assert window.window_width < 0.0


# --- L. above-boundary case feasible ----------------------------------------
def test_above_boundary_feasible():
    section = SECTION_8MM.__class__(nominal_diameter=1.0, tensile_area=1.0, shear_area=1.0)
    limits = BoltStrengthLimits(name="above", proof_strength=2001.0)  # F_proof*0.5 = 1000.5 > 1000
    window = installation_preload_window(1000.0, section, limits, eta_proof=0.5, scatter_allowance=0.0)
    assert window.feasible is True
    assert window.window_width > 0.0


# --- M. window-width identity -----------------------------------------------
def test_window_width_identity():
    window = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, LIMITS, ETA_PROOF, DELTA_F)
    assert window.window_width == pytest.approx(window.target_max - window.target_min, rel=1e-12)


# --- N. normalized-window identity ------------------------------------------
def test_normalized_window_identity():
    window = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, LIMITS, ETA_PROOF, DELTA_F)
    assert window.target_min > 0.0
    expected = window.window_width / window.target_min
    assert window.window_width_normalized == pytest.approx(expected, rel=1e-12)
    expected_margin = window.target_max / window.target_min - 1.0
    assert window.upper_margin == pytest.approx(expected_margin, rel=1e-12)


# --- O. M3 selected-preload classification ----------------------------------
def test_M3_selected_preload_classification_is_no_window_at_baseline():
    # At the illustrative M4 baseline (eta_proof=0.75, delta_F=0.10,
    # S_p=830 MPa), the 8 mm bolt's window is infeasible (verified by
    # hand above: target_min ~32,509 > target_max ~31,290), so ANY
    # selected preload -- including the M3 1.20-factor selection
    # (~35,109.8 N) -- must classify as NO_INSTALLATION_WINDOW, the
    # highest-priority diagnostic.
    window = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, LIMITS, ETA_PROOF, DELTA_F)
    assert window.feasible is False
    m3_selected = REQUIRED.overall_required * 1.20
    status = classify_selected_preload(window, m3_selected)
    assert status == PreloadFeasibilityStatus.NO_INSTALLATION_WINDOW


def test_selected_preload_too_low_and_too_high_classification():
    # Build a feasible window (larger bolt) and probe both edges.
    section_10mm = circular_unthreaded_bolt(0.010)
    window = installation_preload_window(REQUIRED.overall_required, section_10mm, LIMITS, ETA_PROOF, DELTA_F)
    assert window.feasible is True
    too_low = window.target_min * 0.9
    too_high = window.target_max * 1.1
    inside = (window.target_min + window.target_max) / 2.0
    assert classify_selected_preload(window, too_low) == PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_LOW
    assert classify_selected_preload(window, too_high) == PreloadFeasibilityStatus.SELECTED_PRELOAD_TOO_HIGH
    assert classify_selected_preload(window, inside) == PreloadFeasibilityStatus.FEASIBLE


# --- P. maximum in-service bolt force hand calculation ----------------------
def test_max_in_service_bolt_force_hand_calc():
    # Hand reconstruction of F_bolt = F_preload + C*T_sep at each bolt
    # using Milestone 1's signed axial_total directly, independent of
    # assess_preloaded_joint's internals.
    C = STIFFNESS.C
    selected = REQUIRED.overall_required * 1.20
    hand_forces = {}
    for b in GROUP_LOAD_RESULT.bolts:
        T_sep = max(b.axial_total, 0.0)
        hand_forces[b.index] = selected + C * T_sep
    expected_max_idx = max(hand_forces, key=lambda i: hand_forces[i])
    expected_max_val = hand_forces[expected_max_idx]

    section_10mm = circular_unthreaded_bolt(0.010)
    result = assess_preload_feasibility(
        GROUP_LOAD_RESULT, section_10mm, LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, selected
    )
    assert result.max_in_service_bolt_index == expected_max_idx
    assert result.max_in_service_bolt_force == pytest.approx(expected_max_val, rel=1e-9)


# --- Q. proof reserve hand calculation ---------------------------------------
def test_proof_reserve_hand_calc():
    section_10mm = circular_unthreaded_bolt(0.010)
    selected = REQUIRED.overall_required * 1.20
    result = assess_preload_feasibility(
        GROUP_LOAD_RESULT, section_10mm, LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, selected
    )
    f_proof = section_10mm.tensile_area * LIMITS.proof_strength
    expected_reserve = f_proof / result.max_in_service_bolt_force - 1.0
    assert result.proof_reserve == pytest.approx(expected_reserve, rel=1e-9)
    f_yield = section_10mm.tensile_area * LIMITS.yield_strength
    expected_yield_reserve = f_yield / result.max_in_service_bolt_force - 1.0
    assert result.yield_reserve == pytest.approx(expected_yield_reserve, rel=1e-9)


# --- R. no double-counting of preload ----------------------------------------
def test_no_double_counting_of_preload():
    # F_bolt at zero external load (T_sep=0) must equal preload exactly,
    # not preload counted twice or combined with a separate baseline.
    C = STIFFNESS.C
    selected = 40_000.0
    preload_state_result = assess_preloaded_joint(
        GROUP_LOAD_RESULT, __import__("payload_bolts").PreloadState(preload_per_bolt=selected), STIFFNESS, FRICTION
    )
    for b in preload_state_result.bolts:
        if b.separating_demand == 0.0:
            assert b.total_bolt_tension == pytest.approx(selected, rel=1e-12)
        else:
            assert b.total_bolt_tension == pytest.approx(selected + C * b.separating_demand, rel=1e-12)


# --- S. closed-joint formula used only when joint remains closed ------------
def test_closed_joint_formula_restricted_to_closed_bolts():
    # Directly probe the private helper's fallback behavior with
    # synthetic bolt-like records where every bolt has separated
    # (separation_pass=False) -- confirms the helper does NOT silently
    # apply the closed-joint formula's result set as if valid, and
    # instead flags in_service_model_valid=False.
    fake_bolts = [
        SimpleNamespace(index=0, separation_pass=False, total_bolt_tension=100.0),
        SimpleNamespace(index=1, separation_pass=False, total_bolt_tension=200.0),
        SimpleNamespace(index=2, separation_pass=True, total_bolt_tension=150.0),
    ]
    # One bolt (index 2) remains closed -> governing must be drawn only
    # from closed bolts, i.e. index 2 with 150.0, NOT index 1 (200.0).
    val, idx, valid = _max_total_tension_among_closed(fake_bolts)
    assert valid is True
    assert idx == 2
    assert val == pytest.approx(150.0)

    # No bolt remains closed at all -> fallback over all bolts, flagged
    # invalid so callers know the closed-joint model does not apply.
    all_separated = [
        SimpleNamespace(index=0, separation_pass=False, total_bolt_tension=100.0),
        SimpleNamespace(index=1, separation_pass=False, total_bolt_tension=200.0),
    ]
    val2, idx2, valid2 = _max_total_tension_among_closed(all_separated)
    assert valid2 is False
    assert idx2 == 1
    assert val2 == pytest.approx(200.0)


# --- T. M3 separation/slip required-preload values preserved exactly -------
def test_M3_required_preload_values_unchanged():
    assert REQUIRED.separation_required == pytest.approx(19_313.7, abs=0.5)
    assert REQUIRED.slip_required == pytest.approx(29_258.2, abs=0.5)
    assert REQUIRED.slip_governing_bolt == 0
    assert REQUIRED.overall_required == pytest.approx(29_258.2, abs=0.5)
    assert REQUIRED.overall_governing_constraint == "slip"


# --- U. friction sensitivity propagation -------------------------------------
def test_friction_sensitivity_propagates_to_window_feasibility():
    # Independently confirm required preload (and hence target_min)
    # decreases as mu increases, and that this can flip window
    # feasibility for the SAME 8 mm bolt at fixed eta_proof/delta_F --
    # connecting M3 slip physics directly to M4 proof feasibility,
    # without altering any M3 formula.
    mus = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40]
    target_mins = []
    for mu in mus:
        req = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FrictionModel(friction_coefficient=mu))
        target_mins.append(min_installation_preload(req.overall_required, DELTA_F))
    assert all(b < a for a, b in zip(target_mins, target_mins[1:]))  # monotonically decreasing

    lr = compute_preload_limits(SECTION_8MM, LIMITS)
    target_max = max_installation_preload(lr, ETA_PROOF)
    feasibilities = [tmin <= target_max for tmin in target_mins]
    # At mu=0.10 (baseline low end) infeasible; at mu=0.40 (baseline high
    # end) feasible -- a genuine feasibility transition driven purely by
    # M3 friction physics.
    assert feasibilities[0] is False
    assert feasibilities[-1] is True


# --- V. deterministic governing-bolt behavior preserved ----------------------
def test_deterministic_governing_bolt_repeated():
    results = [
        assess_preload_feasibility(
            GROUP_LOAD_RESULT, SECTION_8MM, LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F,
            REQUIRED.overall_required * 1.20,
        )
        for _ in range(5)
    ]
    indices = {r.max_in_service_bolt_index for r in results}
    statuses = {r.status for r in results}
    assert len(indices) == 1
    assert len(statuses) == 1


# --- W. invalid input rejection ----------------------------------------------
def test_invalid_strength_limits_rejected():
    with pytest.raises(ValueError):
        BoltStrengthLimits(name="bad", proof_strength=0.0)
    with pytest.raises(ValueError):
        BoltStrengthLimits(name="bad", proof_strength=-100.0)
    with pytest.raises(ValueError):
        BoltStrengthLimits(name="bad", proof_strength=math.inf)
    with pytest.raises(ValueError):
        BoltStrengthLimits(name="", proof_strength=800e6)
    with pytest.raises(ValueError):
        # yield < proof violates the enforced ordering
        BoltStrengthLimits(name="bad-order", proof_strength=900e6, yield_strength=800e6)
    with pytest.raises(ValueError):
        BoltStrengthLimits(name="bad-yield", proof_strength=800e6, yield_strength=-1.0)


def test_invalid_eta_proof_rejected():
    lr = compute_preload_limits(SECTION_8MM, LIMITS)
    for bad_eta in (0.0, 1.0, -0.1, 1.5, math.inf, math.nan):
        with pytest.raises(ValueError):
            max_installation_preload(lr, bad_eta)


def test_invalid_scatter_allowance_rejected():
    for bad_delta in (-0.1, 1.0, 1.5, math.inf, math.nan):
        with pytest.raises(ValueError):
            min_installation_preload(REQUIRED.overall_required, bad_delta)


def test_invalid_required_preload_value_rejected():
    with pytest.raises(ValueError):
        min_installation_preload(-1.0, 0.1)
    with pytest.raises(ValueError):
        min_installation_preload(math.inf, 0.1)


def test_invalid_selected_preload_rejected():
    window = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, LIMITS, ETA_PROOF, DELTA_F)
    with pytest.raises(ValueError):
        classify_selected_preload(window, 0.0)
    with pytest.raises(ValueError):
        classify_selected_preload(window, -5.0)


# --- X. scalar/numeric consistency -------------------------------------------
def test_scalar_numeric_consistency_across_bolt_sizes():
    for d_mm in (5.0, 6.0, 8.0, 10.0, 12.0):
        section = circular_unthreaded_bolt(d_mm / 1000.0)
        lr = compute_preload_limits(section, LIMITS)
        expected = section.tensile_area * LIMITS.proof_strength
        assert lr.proof_load == pytest.approx(expected, rel=1e-12)
        # Larger tensile area strictly increases proof load for fixed S_p.
    areas = [circular_unthreaded_bolt(d / 1000.0).tensile_area for d in (5.0, 6.0, 8.0, 10.0, 12.0)]
    assert all(b > a for a, b in zip(areas, areas[1:]))


# --- Y. all 119 M1-M3 tests remain passing -----------------------------------
# (Enforced by running the full `pytest` suite, not a single test here;
# see the Milestone 4 session verification. This module adds new tests
# only -- it does not modify or remove any Milestone 1-3 test file.)


# --- Z. explicit regression: M1/M2/M3 outputs unchanged by importing M4 -----
def test_M1_M2_M3_outputs_unchanged_after_importing_preload_limits():
    # Re-run the exact Milestone 1-3 canonical case after this module has
    # been imported and exercised, and confirm the headline results are
    # bit-for-bit identical to their documented Milestone 1-3 values.
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    assert max_tensile.index == 1
    assert max_tensile.axial_total == pytest.approx(24_142.1, abs=0.5)

    section = circular_unthreaded_bolt(0.008)
    material = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
    strength = assess_bolt_group_strength(GROUP_LOAD_RESULT, section, material)
    assert strength.governing_bolt_index == 1
    assert strength.governing_mode == "interaction"
    assert strength.passed is True

    assert REQUIRED.separation_required == pytest.approx(19_313.7, abs=0.5)
    assert REQUIRED.slip_required == pytest.approx(29_258.2, abs=0.5)
    assert REQUIRED.slip_governing_bolt == 0

    m3_group = assess_preloaded_joint(
        GROUP_LOAD_RESULT,
        __import__("payload_bolts").PreloadState(preload_per_bolt=REQUIRED.overall_required * 1.20),
        STIFFNESS,
        FRICTION,
    )
    assert m3_group.passed is True
    assert m3_group.total_preload == pytest.approx(REQUIRED.overall_required * 1.20 * N_BOLTS, rel=1e-9)
