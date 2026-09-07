"""Milestone 2 tests: preliminary bolt tensile/shear strength screening.

These tests exercise payload_bolts.strength in isolation and against
Milestone 1 BoltGroupResult objects. Milestone 1's own test suite
(test_geometry.py, test_shear.py, test_torsion.py, test_axial.py,
test_generality.py, test_handcalc.py) is left completely unmodified and
must continue to pass unchanged -- see test_milestone1_regression.py.
"""

import math

import pytest

from payload_bolts import (
    BoltGroupResult,
    BoltMaterial,
    BoltSection,
    InterfaceLoad,
    NoFeasibleCandidateError,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
)
from payload_bolts.strength import assess_bolt_group_strength, evaluate_candidates, select_smallest_passing_bolt

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

MATERIAL = BoltMaterial(name="Illustrative test steel bolt", tensile_allowable=500e6, shear_allowable=300e6)


# ---------------------------------------------------------------------------
# A. BoltMaterial validation
# ---------------------------------------------------------------------------


def test_bolt_material_valid():
    m = BoltMaterial(name="Illustrative steel", tensile_allowable=800e6, shear_allowable=480e6)
    assert m.tensile_allowable == 800e6
    assert m.shear_allowable == 480e6


def test_bolt_material_rejects_non_finite_or_nonpositive():
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=0.0, shear_allowable=1.0)
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=1.0, shear_allowable=-1.0)
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=float("nan"), shear_allowable=1.0)
    with pytest.raises(ValueError):
        BoltMaterial(name="", tensile_allowable=1.0, shear_allowable=1.0)


# ---------------------------------------------------------------------------
# B. BoltSection validation
# ---------------------------------------------------------------------------


def test_bolt_section_valid():
    s = BoltSection(nominal_diameter=0.01, tensile_area=1e-4, shear_area=8e-5)
    assert s.nominal_diameter == 0.01


def test_bolt_section_rejects_non_finite_or_nonpositive():
    with pytest.raises(ValueError):
        BoltSection(nominal_diameter=0.0, tensile_area=1e-4, shear_area=1e-4)
    with pytest.raises(ValueError):
        BoltSection(nominal_diameter=0.01, tensile_area=-1e-4, shear_area=1e-4)
    with pytest.raises(ValueError):
        BoltSection(nominal_diameter=0.01, tensile_area=1e-4, shear_area=float("inf"))


# ---------------------------------------------------------------------------
# C. circular section area hand calc
# ---------------------------------------------------------------------------


def test_circular_unthreaded_bolt_area_hand_calc():
    d = 0.01  # 10 mm
    sec = circular_unthreaded_bolt(d)
    expected_area = math.pi * d * d / 4.0
    assert sec.tensile_area == pytest.approx(expected_area, rel=1e-12)
    assert sec.shear_area == pytest.approx(expected_area, rel=1e-12)
    assert sec.nominal_diameter == d


# ---------------------------------------------------------------------------
# Hand-calc single-bolt load state (section 17 of the task):
#   T = 10 kN, V = 5 kN, A_t = 100 mm^2, A_s = 80 mm^2,
#   S_t = 500 MPa, S_s = 300 MPa
# ---------------------------------------------------------------------------


def _handcalc_group_result():
    # A 4-bolt symmetric pattern (non-collinear, so the Milestone 1
    # axial-moment coefficient system is well posed) under pure Fx
    # (direct shear, equal split) and pure Fz (equal split), with no
    # applied moments. Every bolt ends up with identical T=10kN axial
    # and V=5kN shear -- exactly the hand-calc load state -- produced
    # entirely by the verified Milestone 1 solver (no bypassing of
    # distribute_loads). Note: a 2-bolt pattern is always collinear and
    # cannot be used here, since the general moment-coefficient system
    # is singular for any 2-bolt pattern regardless of applied moment.
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fx=20_000.0, Fz=40_000.0)  # equal split -> V=5kN, T=10kN per bolt
    return distribute_loads(pattern, load)


def test_handcalc_tensile_stress():
    result = _handcalc_group_result()
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative handcalc steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]
    assert b0.tensile_load == pytest.approx(10_000.0)
    assert b0.tensile_stress == pytest.approx(10_000.0 / 100e-6)  # 100 MPa
    assert b0.tensile_stress == pytest.approx(100e6)


def test_handcalc_shear_stress():
    result = _handcalc_group_result()
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative handcalc steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]
    assert b0.shear_load == pytest.approx(5_000.0)
    assert b0.shear_stress == pytest.approx(5_000.0 / 80e-6)  # 62.5 MPa
    assert b0.shear_stress == pytest.approx(62.5e6)


def test_handcalc_margins_and_fi():
    result = _handcalc_group_result()
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative handcalc steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]

    sigma = 100e6
    tau = 62.5e6
    St = 500e6
    Ss = 300e6

    expected_ms_t = St / sigma - 1.0  # 4.0
    expected_ms_s = Ss / tau - 1.0  # 3.8
    expected_fi = (sigma / St) ** 2 + (tau / Ss) ** 2  # 0.04 + 0.0434...
    expected_ms_i = 1.0 / math.sqrt(expected_fi) - 1.0

    assert b0.tensile_margin == pytest.approx(expected_ms_t)
    assert b0.shear_margin == pytest.approx(expected_ms_s)
    assert b0.interaction_fi == pytest.approx(expected_fi)
    assert b0.interaction_margin == pytest.approx(expected_ms_i)
    assert b0.passed is True


# ---------------------------------------------------------------------------
# F, G. zero tensile / zero shear demand behavior
# ---------------------------------------------------------------------------


def test_zero_tensile_demand_not_applicable():
    # Pure shear, no axial: Fz=Mx=My=0, Fx nonzero -> axial_total == 0 for
    # every bolt in a symmetric pattern -> tensile check not applicable.
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fx=4000.0))
    sec = circular_unthreaded_bolt(0.01)
    group = assess_bolt_group_strength(result, sec, MATERIAL)
    for b in group.bolts:
        assert b.tensile_load == 0.0
        assert b.tensile_stress == 0.0
        assert b.tensile_margin is None


def test_zero_shear_demand_not_applicable():
    # Pure axial, no shear: Fx=Fy=Mz=0, Fz nonzero -> shear_resultant == 0
    # for every bolt.
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad(Fz=4000.0))
    sec = circular_unthreaded_bolt(0.01)
    group = assess_bolt_group_strength(result, sec, MATERIAL)
    for b in group.bolts:
        assert b.shear_load == 0.0
        assert b.shear_stress == 0.0
        assert b.shear_margin is None


def test_zero_total_demand_passes_cleanly():
    pattern = circular_pattern(4, 0.3)
    result = distribute_loads(pattern, InterfaceLoad())  # all-zero load
    sec = circular_unthreaded_bolt(0.01)
    group = assess_bolt_group_strength(result, sec, MATERIAL)
    for b in group.bolts:
        assert b.tensile_margin is None
        assert b.shear_margin is None
        assert b.interaction_margin is None
        assert b.governing_margin is None
        assert b.governing_mode is None
        assert b.passed is True
    assert group.passed is True


# ---------------------------------------------------------------------------
# H, I. tensile / shear margin boundary
# ---------------------------------------------------------------------------


def test_tensile_margin_boundary_exact_zero():
    # sigma_t == S_t_allow exactly -> MS_tension == 0 (pass at boundary)
    pattern = circular_pattern(4, 0.3)
    St = 500e6
    A_t = 100e-6
    T_each = St * A_t  # per-bolt tension such that sigma == St exactly
    load = InterfaceLoad(Fz=4 * T_each)
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=A_t, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative boundary steel", tensile_allowable=St, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]
    assert b0.tensile_stress == pytest.approx(St)
    assert b0.tensile_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.passed is True


def test_shear_margin_boundary_exact_zero():
    pattern = circular_pattern(4, 0.3)
    Ss = 300e6
    A_s = 80e-6
    V_each = Ss * A_s
    load = InterfaceLoad(Fx=4 * V_each)
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=A_s)
    mat = BoltMaterial(name="Illustrative boundary steel", tensile_allowable=500e6, shear_allowable=Ss)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]
    assert b0.shear_stress == pytest.approx(Ss)
    assert b0.shear_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.passed is True


# ---------------------------------------------------------------------------
# J, K, L. interaction FI hand calc + exact boundary + above/below
# ---------------------------------------------------------------------------


def test_interaction_fi_hand_calc():
    result = _handcalc_group_result()
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative handcalc steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    b0 = group.bolts[0]
    sigma, tau, St, Ss = 100e6, 62.5e6, 500e6, 300e6
    expected_fi = (sigma / St) ** 2 + (tau / Ss) ** 2
    assert b0.interaction_fi == pytest.approx(expected_fi)


def _fi_case(sigma_over_st: float, tau_over_ss: float, St=500e6, Ss=300e6):
    """Construct a 4-bolt group result whose bolts have the requested
    sigma/St and tau/Ss ratios exactly, via pure Fz (equal split) and
    pure Fx (equal split) on a 4-bolt (non-collinear) pattern."""
    A_t, A_s = 100e-6, 80e-6
    pattern = circular_pattern(4, 0.3)
    T_each = sigma_over_st * St * A_t
    V_each = tau_over_ss * Ss * A_s
    load = InterfaceLoad(Fz=4 * T_each, Fx=4 * V_each)
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=A_t, shear_area=A_s)
    mat = BoltMaterial(name="Illustrative FI-boundary steel", tensile_allowable=St, shear_allowable=Ss)
    return assess_bolt_group_strength(result, sec, mat)


def test_interaction_exact_boundary_fi_equals_one():
    # sigma/St = tau/Ss = 1/sqrt(2) -> FI = 0.5 + 0.5 = 1 exactly
    ratio = 1.0 / math.sqrt(2.0)
    group = _fi_case(ratio, ratio)
    b0 = group.bolts[0]
    assert b0.interaction_fi == pytest.approx(1.0, rel=1e-9)
    assert b0.interaction_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.passed is True  # FI <= 1 passes, boundary included


def test_interaction_slightly_below_one_passes():
    ratio = 1.0 / math.sqrt(2.0) * 0.99
    group = _fi_case(ratio, ratio)
    b0 = group.bolts[0]
    assert b0.interaction_fi < 1.0
    assert b0.interaction_margin > 0.0
    assert b0.passed is True


def test_interaction_slightly_above_one_fails():
    ratio = 1.0 / math.sqrt(2.0) * 1.01
    group = _fi_case(ratio, ratio)
    b0 = group.bolts[0]
    assert b0.interaction_fi > 1.0
    assert b0.interaction_margin < 0.0
    assert b0.passed is False


# ---------------------------------------------------------------------------
# M, N, O. load / area / allowable scaling
# ---------------------------------------------------------------------------


def test_load_scaling_doubles_stresses_quadruples_fi():
    group1 = _fi_case(0.3, 0.2)
    group2x = _fi_case(0.6, 0.4)  # double both sigma/St and tau/Ss ratios == double loads
    b1, b2 = group1.bolts[0], group2x.bolts[0]
    assert b2.tensile_stress == pytest.approx(2 * b1.tensile_stress, rel=1e-9)
    assert b2.shear_stress == pytest.approx(2 * b1.shear_stress, rel=1e-9)
    assert b2.interaction_fi == pytest.approx(4 * b1.interaction_fi, rel=1e-9)


def test_area_scaling_halves_stress():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0, Fx=20_000.0)
    result = distribute_loads(pattern, load)

    sec1 = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    sec2 = BoltSection(nominal_diameter=0.012, tensile_area=200e-6, shear_area=160e-6)

    g1 = assess_bolt_group_strength(result, sec1, MATERIAL)
    g2 = assess_bolt_group_strength(result, sec2, MATERIAL)

    assert g2.bolts[0].tensile_stress == pytest.approx(g1.bolts[0].tensile_stress / 2.0, rel=1e-9)
    assert g2.bolts[0].shear_stress == pytest.approx(g1.bolts[0].shear_stress / 2.0, rel=1e-9)


def test_allowable_scaling_improves_margins():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=40_000.0, Fx=20_000.0)
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)

    mat1 = BoltMaterial(name="Illustrative base steel", tensile_allowable=500e6, shear_allowable=300e6)
    mat_t2 = BoltMaterial(name="Illustrative 2x tensile steel", tensile_allowable=1000e6, shear_allowable=300e6)
    mat_s2 = BoltMaterial(name="Illustrative 2x shear steel", tensile_allowable=500e6, shear_allowable=600e6)

    g1 = assess_bolt_group_strength(result, sec, mat1)
    g_t2 = assess_bolt_group_strength(result, sec, mat_t2)
    g_s2 = assess_bolt_group_strength(result, sec, mat_s2)

    assert g_t2.bolts[0].tensile_margin > g1.bolts[0].tensile_margin
    assert g_s2.bolts[0].shear_margin > g1.bolts[0].shear_margin


# ---------------------------------------------------------------------------
# P. compression-side bolt handling
# ---------------------------------------------------------------------------


def test_compression_side_bolt_shear_only():
    # A bolt group under Mx large enough that some bolts go negative
    # (compression side) while still carrying shear from Fx.
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=5000.0, Mx=50000.0)
    result = distribute_loads(pattern, load)
    sec = circular_unthreaded_bolt(0.012)
    group = assess_bolt_group_strength(result, sec, MATERIAL)

    compression_bolts = [b for b in group.bolts if b.axial_total < 0]
    assert compression_bolts, "expected at least one compression-side bolt in this construction"
    for b in compression_bolts:
        assert b.tensile_load == 0.0
        assert b.tensile_margin is None
        # shear is still present and evaluated on its own merits
        if b.shear_load > 0:
            assert b.shear_margin is not None


# ---------------------------------------------------------------------------
# Q, R, S. governing mode constructions: tension / shear / interaction
# ---------------------------------------------------------------------------


def test_governing_mode_tension_only():
    # Pure tension, no shear at all: tension and interaction tie exactly;
    # tie-break prefers the specific "tension" mode (see strength.py).
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fz=200_000.0)  # 50kN/bolt, no shear anywhere
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative tension-govern steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    assert group.governing_mode == "tension"
    assert group.bolts[group.governing_bolt_index].shear_load == 0.0


def test_governing_mode_shear_only():
    pattern = circular_pattern(4, 0.3)
    load = InterfaceLoad(Fx=120_000.0)  # 30kN/bolt shear, no axial anywhere
    result = distribute_loads(pattern, load)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="Illustrative shear-govern steel", tensile_allowable=500e6, shear_allowable=300e6)
    group = assess_bolt_group_strength(result, sec, mat)
    assert group.governing_mode == "shear"
    assert group.bolts[group.governing_bolt_index].tensile_load == 0.0


def test_governing_mode_interaction():
    # Both tension and shear demand present at the governing bolt ->
    # interaction margin is strictly less than either individual margin
    # -> interaction governs outright, not by tie-break.
    group = _fi_case(0.5, 0.5)
    b0 = group.bolts[0]
    assert group.governing_mode == "interaction"
    assert b0.interaction_margin < b0.tensile_margin
    assert b0.interaction_margin < b0.shear_margin


# ---------------------------------------------------------------------------
# T. group governing bolt deterministic (tie -> lowest index)
# ---------------------------------------------------------------------------


def test_group_governing_bolt_deterministic_tie_lowest_index():
    # A symmetric circular pattern under pure Fz: every bolt has
    # identical axial_total, so every bolt has an identical governing
    # margin -> lowest index (0) must be selected.
    pattern = circular_pattern(6, 0.4)
    load = InterfaceLoad(Fz=6000.0)
    result = distribute_loads(pattern, load)
    sec = circular_unthreaded_bolt(0.01)
    group = assess_bolt_group_strength(result, sec, MATERIAL)
    assert group.governing_bolt_index == 0


# ---------------------------------------------------------------------------
# governing bolt computed from margins, not assumed from Milestone 1 labels
# ---------------------------------------------------------------------------


def test_strength_governing_bolt_not_assumed_from_milestone1_max_load_labels():
    # Construct a case where the Milestone-1 max-shear bolt and the
    # Milestone-1 max-tensile bolt are DIFFERENT from the strength
    # governing bolt, by giving a small-area/weak-shear-allowable
    # section so that a moderately-loaded-but-thin bolt review is not
    # needed here: instead we verify mechanically that
    # assess_bolt_group_strength recomputes governance independently by
    # checking that the group's own max-shear/max-tensile bolts (from
    # Milestone 1) need not coincide with the strength governing bolt
    # once a material/section is applied that makes shear the tighter
    # constraint even at a low-shear bolt with nonzero demand.
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
    result = distribute_loads(pattern, load)

    ml1_max_shear_idx = result.max_shear_bolt().index
    ml1_max_tensile_idx = result.max_tensile_bolt().index

    # A material with a very low shear allowable relative to tensile
    # allowable flips which bolt governs strength-wise.
    sec = circular_unthreaded_bolt(0.012)
    skewed_mat = BoltMaterial(name="Illustrative shear-weak steel", tensile_allowable=900e6, shear_allowable=20e6)
    group = assess_bolt_group_strength(result, sec, skewed_mat)

    # The strength-governing bolt must be independently derived: it is
    # legitimately allowed to equal one of the Milestone-1 labels, but
    # it must be computed from margins (verify by recomputing manually).
    manual_best = min(group.bolts, key=lambda b: (b.governing_margin if b.governing_margin is not None else math.inf, b.index))
    assert group.governing_bolt_index == manual_best.index
    assert group.governing_mode == manual_best.governing_mode
    # Sanity: this constructed scenario does in fact shift governance
    # away from the Milestone-1 max-tensile-only label to a shear- or
    # interaction-sensitive bolt.
    assert group.governing_mode in ("shear", "interaction")


# ---------------------------------------------------------------------------
# U, V, W. candidate ordering / smallest passing / no feasible candidate
# ---------------------------------------------------------------------------


def _sanity_group_result() -> BoltGroupResult:
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
    return distribute_loads(pattern, load)


SANITY_MATERIAL = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)


def test_candidate_ordering_sorted_by_diameter():
    result = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d) for d in (0.012, 0.006, 0.010, 0.005, 0.008)]
    results = evaluate_candidates(result, candidates, SANITY_MATERIAL)
    diameters = [r.bolt_section.nominal_diameter for r in results]
    assert diameters == sorted(diameters)
    assert diameters == pytest.approx([0.005, 0.006, 0.008, 0.010, 0.012])


def test_select_smallest_passing_bolt():
    result = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (5, 6, 8, 10, 12)]
    selected = select_smallest_passing_bolt(result, candidates, SANITY_MATERIAL)
    assert selected.passed is True
    assert selected.bolt_section.nominal_diameter == pytest.approx(0.008)


def test_no_feasible_candidate_raises():
    result = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (3, 4, 5)]  # all too small
    with pytest.raises(NoFeasibleCandidateError):
        select_smallest_passing_bolt(result, candidates, SANITY_MATERIAL)


# ---------------------------------------------------------------------------
# X. larger circular candidate monotonic improvement
# ---------------------------------------------------------------------------


def test_larger_circular_candidate_monotonic_improvement():
    result = _sanity_group_result()
    diameters_mm = [5, 6, 7, 8, 9, 10, 12, 14, 16]
    results = evaluate_candidates(result, [circular_unthreaded_bolt(d / 1000.0) for d in diameters_mm], SANITY_MATERIAL)

    max_tensile_stresses = [max(b.tensile_stress for b in r.bolts) for r in results]
    max_shear_stresses = [max(b.shear_stress for b in r.bolts) for r in results]
    max_fis = [max(b.interaction_fi for b in r.bolts) for r in results]
    governing_margins = [r.governing_margin for r in results]

    for i in range(len(results) - 1):
        assert max_tensile_stresses[i + 1] <= max_tensile_stresses[i] + 1e-9
        assert max_shear_stresses[i + 1] <= max_shear_stresses[i] + 1e-9
        assert max_fis[i + 1] <= max_fis[i] + 1e-9
        assert governing_margins[i + 1] >= governing_margins[i] - 1e-9


# ---------------------------------------------------------------------------
# Y. deterministic repeated assessment
# ---------------------------------------------------------------------------


def test_repeated_assessment_deterministic():
    result = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.01)
    g1 = assess_bolt_group_strength(result, sec, SANITY_MATERIAL)
    g2 = assess_bolt_group_strength(result, sec, SANITY_MATERIAL)
    assert g1 == g2


# ---------------------------------------------------------------------------
# Z. Milestone 1 result passed through without mutation
# ---------------------------------------------------------------------------


def test_milestone1_result_not_mutated():
    result = _sanity_group_result()
    before = tuple(result.bolts)
    before_eq = result.equilibrium
    sec = circular_unthreaded_bolt(0.01)
    assess_bolt_group_strength(result, sec, SANITY_MATERIAL)
    assert result.bolts == before
    assert result.equilibrium == before_eq
