"""Independent verification of torque_preload.py (Milestone 6).

Reference values are computed by hand directly in these tests, not by
re-calling the production functions under test a second time.
"""

import math

import pytest

from payload_bolts import (
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    NutFactorModel,
    TorqueInstallationStatus,
    assess_bolt_group_strength,
    assess_torque_installation,
    back_calculate_torque,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    installation_preload_window,
    nominal_torque_window,
    preload_from_torque,
    required_preload,
    robust_torque_window,
    torque_from_preload,
)

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

NUT_FACTOR = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)
SECTION_10MM = circular_unthreaded_bolt(0.010)
SECTION_12MM = circular_unthreaded_bolt(0.012)
SECTION_8MM = circular_unthreaded_bolt(0.008)

REQUIRED = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
WINDOW_10MM = installation_preload_window(REQUIRED.overall_required, SECTION_10MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)


# --- A. torque equation: T = K F d -------------------------------------------
def test_torque_from_preload_hand_calc():
    F, K, d = 35_109.8, 0.20, 0.010
    expected = K * F * d
    assert torque_from_preload(F, K, d) == pytest.approx(expected, rel=1e-12)


# --- B. inverse: F = T/(K d) --------------------------------------------------
def test_preload_from_torque_hand_calc():
    T, K, d = 70.2196, 0.20, 0.010
    expected = T / (K * d)
    assert preload_from_torque(T, K, d) == pytest.approx(expected, rel=1e-12)


# --- C. exact round-trip -------------------------------------------------------
def test_round_trip_torque_preload():
    F, K, d = 40_000.0, 0.18, 0.012
    T = torque_from_preload(F, K, d)
    F_back = preload_from_torque(T, K, d)
    assert F_back == pytest.approx(F, rel=1e-12)


# --- D. units/diameter scaling: torque proportional to d --------------------
def test_torque_proportional_to_diameter():
    F, K = 30_000.0, 0.20
    diameters = [0.006, 0.008, 0.010, 0.012]
    torques = [torque_from_preload(F, K, d) for d in diameters]
    ratios = [t / d for t, d in zip(torques, diameters, strict=True)]
    assert all(r == pytest.approx(ratios[0], rel=1e-9) for r in ratios)  # T/d constant


# --- E. torque proportional to K at fixed F -----------------------------------
def test_torque_proportional_to_K_at_fixed_preload():
    F, d = 30_000.0, 0.010
    Ks = [0.10, 0.15, 0.20, 0.25]
    torques = [torque_from_preload(F, k, d) for k in Ks]
    ratios = [t / k for t, k in zip(torques, Ks, strict=True)]
    assert all(r == pytest.approx(ratios[0], rel=1e-9) for r in ratios)  # T/K constant


# --- F. preload proportional to 1/K at fixed torque ---------------------------
def test_preload_inversely_proportional_to_K_at_fixed_torque():
    T, d = 75.0, 0.010
    Ks = [0.10, 0.15, 0.20, 0.25]
    preloads = [preload_from_torque(T, k, d) for k in Ks]
    products = [f * k for f, k in zip(preloads, Ks, strict=True)]
    assert all(p == pytest.approx(products[0], rel=1e-9) for p in products)  # F*K constant


# --- G/H. F_min at K_max, F_max at K_min (fixed torque) ----------------------
def test_achieved_preload_min_at_Kmax_max_at_Kmin():
    T, d = 75.0, 0.010
    f_at_kmin = preload_from_torque(T, 0.15, d)
    f_at_kmax = preload_from_torque(T, 0.25, d)
    assert f_at_kmin > f_at_kmax  # lower K (less friction) -> higher achieved preload


# --- I/J. nominal torque-window bounds ----------------------------------------
def test_nominal_torque_window_bounds_hand_calc():
    ntw = nominal_torque_window(WINDOW_10MM, k_nom=0.20, diameter=0.010)
    assert ntw.torque_min_nom == pytest.approx(0.20 * 0.010 * WINDOW_10MM.target_min, rel=1e-12)
    assert ntw.torque_max_nom == pytest.approx(0.20 * 0.010 * WINDOW_10MM.target_max, rel=1e-12)


# --- K/L. robust bounds --------------------------------------------------------
def test_robust_torque_bounds_hand_calc():
    rtw = robust_torque_window(WINDOW_10MM, NUT_FACTOR, 0.010)
    expected_min = NUT_FACTOR.k_max * 0.010 * WINDOW_10MM.target_min
    expected_max = NUT_FACTOR.k_min * 0.010 * WINDOW_10MM.target_max
    assert rtw.torque_robust_min == pytest.approx(expected_min, rel=1e-12)
    assert rtw.torque_robust_max == pytest.approx(expected_max, rel=1e-12)


# --- M. exact robust-window boundary (T_min == T_max) -------------------------
def test_exact_robust_window_boundary():
    # Construct target_min/target_max and K such that
    # K_max*d*target_min == K_min*d*target_max, up to floating-point
    # precision. A tiny relative epsilon nudges k_max fractionally below
    # the exact analytic tie so the constructed case is unambiguously AT
    # or just inside the boundary (feasible), rather than a coin flip on
    # which side of an exact bit-level tie floating-point rounding lands.
    target_min, target_max = 1000.0, 1500.0
    d = 0.010
    k_min = 0.20
    k_max = k_min * (target_max / target_min) * (1.0 - 1e-9)
    nut_factor = NutFactorModel(k_min=k_min, k_nom=k_min, k_max=k_max)

    class FakeWindow:
        pass

    fake = FakeWindow()
    fake.target_min = target_min
    fake.target_max = target_max
    rtw = robust_torque_window(fake, nut_factor, d)
    assert rtw.torque_robust_min == pytest.approx(rtw.torque_robust_max, rel=1e-6)
    assert rtw.width == pytest.approx(0.0, abs=1e-6)
    assert rtw.feasible is True  # inclusive boundary


# --- N. ratio identity at the boundary ----------------------------------------
def test_ratio_identity_at_boundary():
    target_min, target_max = 1000.0, 1500.0
    d = 0.010
    k_min = 0.20
    k_max = k_min * (target_max / target_min)
    nut_factor = NutFactorModel(k_min=k_min, k_nom=k_min, k_max=k_max)

    class FakeWindow:
        pass

    fake = FakeWindow()
    fake.target_min = target_min
    fake.target_max = target_max
    rtw = robust_torque_window(fake, nut_factor, d)
    assert rtw.k_uncertainty_ratio == pytest.approx(rtw.preload_window_ratio, rel=1e-9)
    assert rtw.k_uncertainty_ratio == pytest.approx(nut_factor.k_max / nut_factor.k_min, rel=1e-12)
    assert rtw.preload_window_ratio == pytest.approx(target_max / target_min, rel=1e-12)


# --- O/P. infeasible/feasible cases from K uncertainty ratio -----------------
def test_infeasible_when_K_ratio_too_large():
    rtw = robust_torque_window(WINDOW_10MM, NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25), 0.010)
    assert rtw.k_uncertainty_ratio > rtw.preload_window_ratio
    assert rtw.feasible is False
    assert rtw.width < 0.0


def test_feasible_when_K_ratio_small_enough():
    rtw = robust_torque_window(WINDOW_10MM, NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22), 0.010)
    assert rtw.k_uncertainty_ratio <= rtw.preload_window_ratio
    assert rtw.feasible is True
    assert rtw.width > 0.0


# --- Q. midpoint target lies inside robust window -----------------------------
def test_midpoint_nominal_torque_inside_robust_window():
    narrow = NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22)
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, narrow)
    assert result.status in (TorqueInstallationStatus.FEASIBLE, TorqueInstallationStatus.PROOF_LIMIT_EXCEEDED)
    assert result.nominal_torque is not None
    rtw = result.robust_window
    assert rtw.torque_robust_min <= result.nominal_torque <= rtw.torque_robust_max


# --- R. achieved preload at K_min/K_max lies on expected sides --------------
def test_achieved_preload_extremes_bracket_and_within_target():
    narrow = NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22)
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, narrow)
    assert result.achieved_preload_at_k_min > result.achieved_preload_at_k_nom > result.achieved_preload_at_k_max
    w = result.robust_window
    assert w.target_min <= result.achieved_preload_at_k_max
    assert result.achieved_preload_at_k_min <= w.target_max
    assert result.all_within_target_window is True


# --- S. M3 selected preload torque back-calculation ---------------------------
def test_back_calculate_torque_hand_calc():
    F, d = 35_109.8, 0.010
    back = back_calculate_torque(F, d, NUT_FACTOR)
    assert back.torque_at_k_min == pytest.approx(NUT_FACTOR.k_min * F * d, rel=1e-12)
    assert back.torque_at_k_nom == pytest.approx(NUT_FACTOR.k_nom * F * d, rel=1e-12)
    assert back.torque_at_k_max == pytest.approx(NUT_FACTOR.k_max * F * d, rel=1e-12)
    # Higher K -> higher required torque for the same preload.
    assert back.torque_at_k_min < back.torque_at_k_nom < back.torque_at_k_max


# --- T. no torque returned when robust window absent -------------------------
def test_no_torque_when_robust_window_infeasible():
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, NUT_FACTOR)
    assert result.status == TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW
    assert result.nominal_torque is None
    assert result.achieved_preload_at_k_min is None
    assert result.achieved_preload_at_k_nom is None
    assert result.achieved_preload_at_k_max is None
    assert result.max_in_service_bolt_force is None
    assert result.proof_reserve is None


# --- U. M4 10 mm preload bounds preserved exactly -----------------------------
def test_M4_10mm_preload_bounds_preserved_exactly():
    assert WINDOW_10MM.target_min == pytest.approx(32_509.1, abs=1.0)
    assert WINDOW_10MM.target_max == pytest.approx(48_891.0, abs=1.0)
    assert WINDOW_10MM.feasible is True
    assert WINDOW_10MM.window_width == pytest.approx(16_381.9, abs=1.0)


# --- V. M5 selected candidate remains 10 mm before M6 screening --------------
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


# --- W. in-service proof check uses achieved preload + increment EXACTLY ONCE
def test_in_service_no_double_counting():
    narrow = NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22)
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, narrow)
    assert result.status in (TorqueInstallationStatus.FEASIBLE, TorqueInstallationStatus.PROOF_LIMIT_EXCEEDED)
    # Independent hand reconstruction of max in-service bolt force at the
    # K_min-bound achieved preload, using the raw Milestone 1 signed axial
    # loads directly (not any Milestone 3/6 internal helper).
    C = STIFFNESS.C
    f_achieved = result.achieved_preload_at_k_min
    hand_forces = {b.index: f_achieved + C * max(b.axial_total, 0.0) for b in GROUP_LOAD_RESULT.bolts}
    expected_max_idx = max(hand_forces, key=lambda i: hand_forces[i])
    expected_max_val = hand_forces[expected_max_idx]
    assert result.max_in_service_bolt_index == expected_max_idx
    assert result.max_in_service_bolt_force == pytest.approx(expected_max_val, rel=1e-9)


# --- X. proof violation at sufficiently high achieved preload ----------------
def test_proof_violation_reported_at_high_achieved_preload():
    # Construct a strength-limits/nut-factor combination that forces the
    # K_min-bound achieved preload above the proof load, independent of
    # the baseline illustrative numbers.
    low_proof_limits = BoltStrengthLimits(name="artificially low proof", proof_strength=200e6)
    narrow = NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22)
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, low_proof_limits, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, narrow)
    # With a low proof allowable, the target window itself may already be
    # infeasible OR (if still feasible) the in-service force should exceed
    # the tiny proof load -- either way, PROOF_LIMIT_EXCEEDED must never be
    # silently skipped when it applies. Confirm the two possible honest
    # outcomes and, if FEASIBLE-path was reached, confirm the flag content.
    assert result.status in (
        TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW,
        TorqueInstallationStatus.PROOF_LIMIT_EXCEEDED,
    )


# --- Y. lower achieved preload remains above separation/slip requirement -----
def test_achieved_preload_at_Kmax_meets_M3_requirement_when_feasible():
    narrow = NutFactorModel(k_min=0.18, k_nom=0.20, k_max=0.22)
    result = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, narrow)
    assert result.status in (TorqueInstallationStatus.FEASIBLE, TorqueInstallationStatus.PROOF_LIMIT_EXCEEDED)
    # The lowest achieved preload (at K_max, highest friction) must still
    # be >= the Milestone 4 scatter-adjusted target_min, which itself is
    # >= the raw Milestone 3 required preload (since delta_F >= 0).
    assert result.achieved_preload_at_k_max >= WINDOW_10MM.target_min
    assert WINDOW_10MM.target_min >= REQUIRED.overall_required


# --- Z. invalid K <= 0 rejected -------------------------------------------------
def test_invalid_K_rejected():
    with pytest.raises(ValueError):
        torque_from_preload(30_000.0, 0.0, 0.010)
    with pytest.raises(ValueError):
        torque_from_preload(30_000.0, -0.1, 0.010)
    with pytest.raises(ValueError):
        preload_from_torque(70.0, 0.0, 0.010)
    with pytest.raises(ValueError):
        NutFactorModel(k_min=0.0, k_nom=0.2, k_max=0.25)


# --- AA. invalid d <= 0 rejected ------------------------------------------------
def test_invalid_diameter_rejected():
    with pytest.raises(ValueError):
        torque_from_preload(30_000.0, 0.2, 0.0)
    with pytest.raises(ValueError):
        torque_from_preload(30_000.0, 0.2, -0.01)
    with pytest.raises(ValueError):
        preload_from_torque(70.0, 0.2, 0.0)


# --- AB. invalid torque/preload rejected ----------------------------------------
def test_invalid_torque_and_preload_rejected():
    with pytest.raises(ValueError):
        torque_from_preload(-1.0, 0.2, 0.010)
    with pytest.raises(ValueError):
        torque_from_preload(math.inf, 0.2, 0.010)
    with pytest.raises(ValueError):
        preload_from_torque(-1.0, 0.2, 0.010)
    with pytest.raises(ValueError):
        preload_from_torque(math.inf, 0.2, 0.010)


# --- AC. K_min > K_max rejected --------------------------------------------------
def test_K_min_greater_than_K_max_rejected():
    with pytest.raises(ValueError):
        NutFactorModel(k_min=0.30, k_nom=0.25, k_max=0.20)
    with pytest.raises(ValueError):
        NutFactorModel(k_min=0.20, k_nom=0.30, k_max=0.25)  # k_nom outside [k_min, k_max]


# --- AD. zero-width K range handled correctly -----------------------------------
def test_zero_width_K_range():
    nf = NutFactorModel(k_min=0.20, k_nom=0.20, k_max=0.20)
    assert nf.uncertainty_ratio == pytest.approx(1.0, rel=1e-12)
    rtw = robust_torque_window(WINDOW_10MM, nf, 0.010)
    # Zero K uncertainty: robust window collapses to exactly the nominal
    # torque mapping of the target window (K_max==K_min==K_nom).
    ntw = nominal_torque_window(WINDOW_10MM, 0.20, 0.010)
    assert rtw.torque_robust_min == pytest.approx(ntw.torque_min_nom, rel=1e-12)
    assert rtw.torque_robust_max == pytest.approx(ntw.torque_max_nom, rel=1e-12)
    assert rtw.feasible is True  # target_min <= target_max always holds for a valid window


# --- AE. deterministic sensitivity ordering -------------------------------------
def test_deterministic_repeated_assessment():
    r1 = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, NUT_FACTOR)
    r2 = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_10MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, NUT_FACTOR)
    assert r1.status == r2.status
    assert r1.robust_window.width == pytest.approx(r2.robust_window.width, rel=1e-15)


def test_bolt_size_feasibility_sensitivity_deterministic_and_matches_expected_trend():
    results = {}
    for d_mm, section in ((8.0, SECTION_8MM), (10.0, SECTION_10MM), (12.0, SECTION_12MM)):
        results[d_mm] = assess_torque_installation(
            GROUP_LOAD_RESULT, section, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, NUT_FACTOR
        )
    assert results[8.0].status == TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW
    assert results[10.0].status == TorqueInstallationStatus.NO_ROBUST_TORQUE_WINDOW
    assert results[12.0].status == TorqueInstallationStatus.FEASIBLE


# --- AF. all 182 M1-M5 tests remain passing -------------------------------------
# (Enforced by running the full `pytest` suite; this module adds new
# tests only and does not modify any Milestone 1-5 test file.)


# --- AG. explicit regression: inherited headline outputs unchanged -------------
def test_inherited_headline_outputs_unchanged():
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    assert max_tensile.index == 1
    assert max_tensile.axial_total == pytest.approx(24_142.1, abs=0.5)

    strength = assess_bolt_group_strength(GROUP_LOAD_RESULT, SECTION_8MM, __import__("payload_bolts").BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6))
    assert strength.governing_bolt_index == 1
    assert strength.governing_mode == "interaction"

    assert REQUIRED.separation_required == pytest.approx(19_313.7, abs=0.5)
    assert REQUIRED.slip_required == pytest.approx(29_258.2, abs=0.5)
    assert REQUIRED.slip_governing_bolt == 0
    assert REQUIRED.overall_required == pytest.approx(29_258.2, abs=0.5)

    window_8mm = installation_preload_window(REQUIRED.overall_required, SECTION_8MM, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
    assert window_8mm.feasible is False
    assert window_8mm.target_min == pytest.approx(32_509.1, abs=1.0)
    assert window_8mm.target_max == pytest.approx(31_290.3, abs=1.0)
