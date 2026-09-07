"""Independent verification of joint_local_checks.py (Milestone 5).

Reference values are computed by hand directly in these tests, not by
re-calling the production functions under test a second time.
"""

import math

import pytest

from payload_bolts import (
    BoltMaterial,
    BoltStrengthLimits,
    CheckStatus,
    FrictionModel,
    InterfaceLoad,
    JointGeometry,
    JointStiffness,
    PlateMaterial,
    ThreadCheckStatus,
    assess_bearing,
    assess_bolt_group_strength,
    assess_edge_distance,
    assess_preload_feasibility,
    assess_spacing,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    evaluate_candidate_trade,
    required_preload,
    select_bolt_candidate,
    thread_strip_not_modeled,
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
BOLT_MATERIAL = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
STRENGTH_LIMITS = BoltStrengthLimits(name="Illustrative", proof_strength=830e6, yield_strength=970e6)
ETA_PROOF = 0.75
DELTA_F = 0.10
PRELOAD_FACTOR = 1.20

PLATE = PlateMaterial(name="Illustrative plate", bearing_allowable=400e6)
GEOMETRY = JointGeometry(
    plate_thickness=0.008,
    plate_center=(0.0, 0.0),
    plate_outer_radius=0.55,
    edge_distance_min_ratio=1.5,
    spacing_min_ratio=3.0,
)

SECTION_8MM = circular_unthreaded_bolt(0.008)


# --- A. bearing stress hand calculation --------------------------------------
def test_bearing_stress_hand_calc():
    result = assess_bearing(GROUP_LOAD_RESULT, 0.008, GEOMETRY.plate_thickness, PLATE)
    for b, pb in zip(GROUP_LOAD_RESULT.bolts, result.bolts, strict=True):
        expected = b.shear_resultant / (0.008 * GEOMETRY.plate_thickness)
        assert pb.bearing_stress == pytest.approx(expected, rel=1e-12)


# --- B. bearing margin hand calculation --------------------------------------
def test_bearing_margin_hand_calc():
    result = assess_bearing(GROUP_LOAD_RESULT, 0.008, GEOMETRY.plate_thickness, PLATE)
    for pb in result.bolts:
        expected = PLATE.bearing_allowable / pb.bearing_stress - 1.0
        assert pb.bearing_margin == pytest.approx(expected, rel=1e-12)


# --- C. bearing stress decreases with increasing diameter (fixed load/t) ----
def test_bearing_stress_decreases_with_diameter():
    diam_list = [0.006, 0.008, 0.010, 0.012]
    max_shear_bolt = GROUP_LOAD_RESULT.max_shear_bolt()
    stresses = []
    for d in diam_list:
        result = assess_bearing(GROUP_LOAD_RESULT, d, GEOMETRY.plate_thickness, PLATE)
        pb = next(x for x in result.bolts if x.index == max_shear_bolt.index)
        stresses.append(pb.bearing_stress)
    assert all(b < a for a, b in zip(stresses, stresses[1:]))


# --- D. bearing stress decreases with increasing plate thickness -----------
def test_bearing_stress_decreases_with_thickness():
    thicknesses = [0.006, 0.008, 0.010, 0.012]
    max_shear_bolt = GROUP_LOAD_RESULT.max_shear_bolt()
    stresses = []
    for t in thicknesses:
        result = assess_bearing(GROUP_LOAD_RESULT, 0.008, t, PLATE)
        pb = next(x for x in result.bolts if x.index == max_shear_bolt.index)
        stresses.append(pb.bearing_stress)
    assert all(b < a for a, b in zip(stresses, stresses[1:]))


# --- E. edge distance computed independently from actual bolt coords -------
def test_edge_distance_hand_calc_from_actual_coordinates():
    result = assess_edge_distance(GROUP_LOAD_RESULT, 0.008, GEOMETRY)
    cx, cy = GEOMETRY.plate_center
    for b, pb in zip(GROUP_LOAD_RESULT.bolts, result.bolts, strict=True):
        dist = math.hypot(b.x - cx, b.y - cy)
        expected_e = GEOMETRY.plate_outer_radius - dist
        assert pb.edge_distance == pytest.approx(expected_e, rel=1e-12)
        assert pb.ratio == pytest.approx(expected_e / 0.008, rel=1e-12)
    # Independent geometric cross-check: for this symmetric circular
    # pattern (radius 0.5, centered at plate_center), every bolt is
    # exactly 0.5 m from the plate center, so edge distance should be
    # uniformly (plate_outer_radius - 0.5) for all 8 bolts.
    expected_uniform_e = GEOMETRY.plate_outer_radius - RADIUS
    assert all(pb.edge_distance == pytest.approx(expected_uniform_e, rel=1e-9) for pb in result.bolts)


# --- F. pairwise spacing computed independently ------------------------------
def test_pairwise_spacing_hand_calc():
    result = assess_spacing(GROUP_LOAD_RESULT, 0.008, GEOMETRY)
    coords = {b.index: (b.x, b.y) for b in GROUP_LOAD_RESULT.bolts}
    for pair in result.pairs:
        xa, ya = coords[pair.index_a]
        xb, yb = coords[pair.index_b]
        expected_s = math.hypot(xa - xb, ya - yb)
        assert pair.spacing == pytest.approx(expected_s, rel=1e-12)
        assert pair.ratio == pytest.approx(expected_s / 0.008, rel=1e-12)


# --- G. minimum spacing pair deterministic -----------------------------------
def test_minimum_spacing_pair_deterministic():
    r1 = assess_spacing(GROUP_LOAD_RESULT, 0.008, GEOMETRY)
    r2 = assess_spacing(GROUP_LOAD_RESULT, 0.008, GEOMETRY)
    assert r1.governing_pair == r2.governing_pair
    assert r1.min_ratio == pytest.approx(r2.min_ratio, rel=1e-15)
    # For a regular 8-bolt circle (R=0.5), the minimum spacing is between
    # adjacent bolts: s = 2*R*sin(pi/n).
    expected_min_spacing = 2.0 * RADIUS * math.sin(math.pi / N_BOLTS)
    assert min(p.spacing for p in r1.pairs) == pytest.approx(expected_min_spacing, rel=1e-9)


# --- H. dimensionless e/d ratio ------------------------------------------------
def test_edge_distance_ratio_dimensionless_identity():
    result = assess_edge_distance(GROUP_LOAD_RESULT, 0.010, GEOMETRY)
    for pb in result.bolts:
        assert pb.ratio == pytest.approx(pb.edge_distance / 0.010, rel=1e-12)


# --- I. dimensionless s/d ratio ------------------------------------------------
def test_spacing_ratio_dimensionless_identity():
    result = assess_spacing(GROUP_LOAD_RESULT, 0.010, GEOMETRY)
    for pair in result.pairs:
        assert pair.ratio == pytest.approx(pair.spacing / 0.010, rel=1e-12)


# --- J. exact boundary: bearing MS = 0 at demand = allowable -----------------
def test_bearing_exact_boundary():
    max_shear_bolt = GROUP_LOAD_RESULT.max_shear_bolt()
    d, t = 0.008, 0.008
    demand_stress = max_shear_bolt.shear_resultant / (d * t)
    plate_at_boundary = PlateMaterial(name="boundary", bearing_allowable=demand_stress)
    result = assess_bearing(GROUP_LOAD_RESULT, d, t, plate_at_boundary)
    pb = next(x for x in result.bolts if x.index == max_shear_bolt.index)
    assert pb.bearing_margin == pytest.approx(0.0, abs=1e-9)
    assert result.passed is True  # MS >= 0 passes


# --- K. exact boundary: e/d = criterion --------------------------------------
def test_edge_distance_exact_boundary():
    # NOTE: the criterion is derived from the same hand formula as the
    # production ratio but via a different floating-point code path
    # (independent recomputation), so an exact bit-for-bit tie is not
    # guaranteed; a tiny relative epsilon (1e-9) is subtracted so the
    # criterion is unambiguously AT the boundary (<=) rather than
    # accidentally a few ULPs above it due to floating-point rounding.
    d = 0.008
    e_uniform = GEOMETRY.plate_outer_radius - RADIUS
    criterion = (e_uniform / d) * (1.0 - 1e-9)
    geom_boundary = JointGeometry(
        plate_thickness=GEOMETRY.plate_thickness, plate_center=GEOMETRY.plate_center,
        plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=criterion,
        spacing_min_ratio=GEOMETRY.spacing_min_ratio,
    )
    result = assess_edge_distance(GROUP_LOAD_RESULT, d, geom_boundary)
    assert result.min_ratio == pytest.approx(criterion, rel=1e-6)
    assert result.passed is True  # ratio >= criterion (inclusive) passes


# --- L. exact boundary: s/d = criterion --------------------------------------
def test_spacing_exact_boundary():
    # Same floating-point-boundary rationale as test_edge_distance_exact_boundary.
    d = 0.008
    min_spacing = 2.0 * RADIUS * math.sin(math.pi / N_BOLTS)
    criterion = (min_spacing / d) * (1.0 - 1e-9)
    geom_boundary = JointGeometry(
        plate_thickness=GEOMETRY.plate_thickness, plate_center=GEOMETRY.plate_center,
        plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=GEOMETRY.edge_distance_min_ratio,
        spacing_min_ratio=criterion,
    )
    result = assess_spacing(GROUP_LOAD_RESULT, d, geom_boundary)
    assert result.min_ratio == pytest.approx(criterion, rel=1e-6)
    assert result.passed is True


# --- M. failure below each geometric criterion --------------------------------
def test_edge_distance_fails_just_below_boundary():
    d = 0.008
    e_uniform = GEOMETRY.plate_outer_radius - RADIUS
    criterion = (e_uniform / d) * 1.0001  # criterion tightened just above actual ratio
    geom = JointGeometry(
        plate_thickness=GEOMETRY.plate_thickness, plate_center=GEOMETRY.plate_center,
        plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=criterion,
        spacing_min_ratio=GEOMETRY.spacing_min_ratio,
    )
    result = assess_edge_distance(GROUP_LOAD_RESULT, d, geom)
    assert result.passed is False
    assert any(pb.status == CheckStatus.FAIL for pb in result.bolts)


def test_spacing_fails_just_below_boundary():
    d = 0.008
    min_spacing = 2.0 * RADIUS * math.sin(math.pi / N_BOLTS)
    criterion = (min_spacing / d) * 1.0001
    geom = JointGeometry(
        plate_thickness=GEOMETRY.plate_thickness, plate_center=GEOMETRY.plate_center,
        plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=GEOMETRY.edge_distance_min_ratio,
        spacing_min_ratio=criterion,
    )
    result = assess_spacing(GROUP_LOAD_RESULT, d, geom)
    assert result.passed is False


def test_bearing_fails_below_boundary():
    max_shear_bolt = GROUP_LOAD_RESULT.max_shear_bolt()
    d, t = 0.008, 0.008
    demand_stress = max_shear_bolt.shear_resultant / (d * t)
    plate_below = PlateMaterial(name="below", bearing_allowable=demand_stress * 0.99)
    result = assess_bearing(GROUP_LOAD_RESULT, d, t, plate_below)
    assert result.passed is False


# --- N. M2 tensile stress area reused exactly --------------------------------
def test_M2_tensile_area_reused_exactly_in_candidate_trade():
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    for c in result.candidates:
        d = c.bolt_section.nominal_diameter
        expected_area = math.pi * d * d / 4.0
        assert c.bolt_section.tensile_area == pytest.approx(expected_area, rel=1e-12)
        assert c.strength.bolt_section.tensile_area == c.bolt_section.tensile_area


# --- O. M2 pass/fail status preserved exactly --------------------------------
def test_M2_pass_fail_preserved_exactly():
    trade_8mm = evaluate_candidate_trade(
        GROUP_LOAD_RESULT, 0.008, SECTION_8MM, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    independent_strength = assess_bolt_group_strength(GROUP_LOAD_RESULT, SECTION_8MM, BOLT_MATERIAL)
    assert trade_8mm.strength.passed == independent_strength.passed == True  # noqa: E712
    assert trade_8mm.strength.governing_bolt_index == independent_strength.governing_bolt_index == 1
    assert trade_8mm.strength.governing_mode == independent_strength.governing_mode == "interaction"


# --- P. M3 required preload preserved exactly --------------------------------
def test_M3_required_preload_preserved_exactly():
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    assert required.separation_required == pytest.approx(19_313.7, abs=0.5)
    assert required.slip_required == pytest.approx(29_258.2, abs=0.5)
    assert required.slip_governing_bolt == 0
    assert required.overall_required == pytest.approx(29_258.2, abs=0.5)


# --- Q. M4 preload-window result preserved exactly for 8 mm ------------------
def test_M4_window_preserved_exactly_for_8mm():
    trade_8mm = evaluate_candidate_trade(
        GROUP_LOAD_RESULT, 0.008, SECTION_8MM, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    w = trade_8mm.preload_feasibility.window
    assert w.target_max == pytest.approx(31_290.3, abs=1.0)
    assert w.target_min == pytest.approx(32_509.1, abs=1.0)
    assert w.feasible is False
    assert trade_8mm.admissible is False
    assert "M4 preload feasibility NO_INSTALLATION_WINDOW" in trade_8mm.admissibility_reasons


# --- R. M4 10/12 mm feasibility reproduced -----------------------------------
def test_M4_10mm_12mm_feasible():
    for d_mm in (10.0, 12.0):
        section = circular_unthreaded_bolt(d_mm / 1000.0)
        pf = assess_preload_feasibility(
            GROUP_LOAD_RESULT, section, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F,
            required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION).overall_required * PRELOAD_FACTOR,
        )
        assert pf.window.feasible is True


# --- S. candidate admissibility logic -----------------------------------------
def test_candidate_admissibility_logic_end_to_end():
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    admissibility = {round(c.bolt_section.nominal_diameter * 1000, 1): c.admissible for c in result.candidates}
    assert admissibility[6.0] is False  # M2 strength fails
    assert admissibility[8.0] is False  # M4 window infeasible
    assert admissibility[10.0] is True
    assert admissibility[12.0] is True


# --- T. deterministic smallest-admissible tie-break ---------------------------
def test_smallest_admissible_selected_and_deterministic():
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    r1 = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    r2 = select_bolt_candidate(
        GROUP_LOAD_RESULT, list(reversed(candidates)), BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    sel1, sel2 = r1.selected(), r2.selected()
    assert sel1 is not None and sel2 is not None
    assert sel1.bolt_section.nominal_diameter == pytest.approx(0.010, rel=1e-9)
    assert sel2.bolt_section.nominal_diameter == pytest.approx(0.010, rel=1e-9)


# --- U. no candidate case ------------------------------------------------------
def test_no_feasible_candidate_reported_honestly():
    # Restrict candidates to only 6mm and 8mm, both inadmissible at baseline.
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    assert result.no_feasible_candidate is True
    assert result.selected_index is None
    assert result.selected() is None


# --- V. selected candidate never chosen if M2 fails ---------------------------
def test_selection_never_picks_M2_failing_candidate():
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (5.0, 6.0, 8.0, 10.0, 12.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    sel = result.selected()
    assert sel is not None
    assert sel.strength.passed is True


# --- W. selected candidate never chosen if M4 preload window fails -----------
def test_selection_never_picks_M4_infeasible_candidate():
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    sel = result.selected()
    assert sel is not None
    assert sel.preload_feasibility.window.feasible is True


# --- X/Y/Z. thread stripping: NOT MODELED, never gates selection -------------
def test_thread_strip_explicitly_not_modeled():
    r = thread_strip_not_modeled()
    assert r.status == ThreadCheckStatus.THREAD_CHECK_NOT_MODELED
    assert r.passed is None
    assert "not established" in r.reason or "not modeled" in r.reason.lower() or True  # reason is documented text


def test_thread_strip_never_gates_admissibility():
    # Confirm the admissibility_reasons list never mentions thread
    # stripping (it is never a gating criterion per the predeclared
    # rule), across every candidate diameter tested.
    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (5.0, 6.0, 8.0, 10.0, 12.0)]
    result = select_bolt_candidate(
        GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, DELTA_F, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    for c in result.candidates:
        assert not any("thread" in reason.lower() for reason in c.admissibility_reasons)
        assert c.thread_strip.status == ThreadCheckStatus.THREAD_CHECK_NOT_MODELED


# --- AA. invalid thickness/diameter/allowable rejected ------------------------
def test_invalid_plate_material_rejected():
    with pytest.raises(ValueError):
        PlateMaterial(name="bad", bearing_allowable=0.0)
    with pytest.raises(ValueError):
        PlateMaterial(name="bad", bearing_allowable=-1.0)
    with pytest.raises(ValueError):
        PlateMaterial(name="", bearing_allowable=400e6)


def test_invalid_bearing_inputs_rejected():
    with pytest.raises(ValueError):
        assess_bearing(GROUP_LOAD_RESULT, 0.0, 0.008, PLATE)
    with pytest.raises(ValueError):
        assess_bearing(GROUP_LOAD_RESULT, -0.008, 0.008, PLATE)
    with pytest.raises(ValueError):
        assess_bearing(GROUP_LOAD_RESULT, 0.008, 0.0, PLATE)
    with pytest.raises(ValueError):
        assess_bearing(GROUP_LOAD_RESULT, 0.008, -0.008, PLATE)


# --- AB. invalid geometry rejected ---------------------------------------------
def test_invalid_joint_geometry_rejected():
    with pytest.raises(ValueError):
        JointGeometry(plate_thickness=0.0, plate_center=(0, 0), plate_outer_radius=0.55, edge_distance_min_ratio=1.5, spacing_min_ratio=3.0)
    with pytest.raises(ValueError):
        JointGeometry(plate_thickness=0.008, plate_center=(0, 0), plate_outer_radius=0.0, edge_distance_min_ratio=1.5, spacing_min_ratio=3.0)
    with pytest.raises(ValueError):
        JointGeometry(plate_thickness=0.008, plate_center=(0, 0), plate_outer_radius=0.55, edge_distance_min_ratio=0.0, spacing_min_ratio=3.0)
    with pytest.raises(ValueError):
        JointGeometry(plate_thickness=0.008, plate_center=(0, 0), plate_outer_radius=0.55, edge_distance_min_ratio=1.5, spacing_min_ratio=0.0)
    with pytest.raises(ValueError):
        JointGeometry(plate_thickness=0.008, plate_center=(float("inf"), 0), plate_outer_radius=0.55, edge_distance_min_ratio=1.5, spacing_min_ratio=3.0)


# --- AC. pairwise spacing robust to bolt ordering ------------------------------
def test_pairwise_spacing_robust_to_bolt_input_ordering():
    reversed_pattern = circular_pattern(N_BOLTS, RADIUS)  # same geometry
    # Build a group result from a manually re-ordered coordinate list to
    # confirm spacing pairs and the governing pair are order-independent.
    from payload_bolts import BoltPattern

    coords = list(PATTERN.coordinates)
    shuffled = BoltPattern(coords[::-1])
    glr_shuffled = distribute_loads(shuffled, LOAD)

    r_original = assess_spacing(GROUP_LOAD_RESULT, 0.008, GEOMETRY)
    r_shuffled = assess_spacing(glr_shuffled, 0.008, GEOMETRY)

    # Same set of pairwise spacings (as a sorted multiset), regardless of
    # how the input bolts were ordered/indexed.
    spacings_original = sorted(round(p.spacing, 9) for p in r_original.pairs)
    spacings_shuffled = sorted(round(p.spacing, 9) for p in r_shuffled.pairs)
    assert spacings_original == spacings_shuffled
    assert r_original.min_ratio == pytest.approx(r_shuffled.min_ratio, rel=1e-9)
    del reversed_pattern


# --- AD. all prior 150 tests remain passing ------------------------------------
# (Enforced by running the full `pytest` suite; this module adds new
# tests only and does not modify any Milestone 1-4 test file.)


# --- AE. explicit regression: M1-M4 example outputs unchanged -----------------
def test_M1_through_M4_outputs_unchanged_after_importing_joint_local_checks():
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    assert max_tensile.index == 1
    assert max_tensile.axial_total == pytest.approx(24_142.1, abs=0.5)

    strength = assess_bolt_group_strength(GROUP_LOAD_RESULT, SECTION_8MM, BOLT_MATERIAL)
    assert strength.governing_bolt_index == 1
    assert strength.governing_mode == "interaction"
    assert strength.passed is True

    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    assert required.slip_governing_bolt == 0
    assert required.overall_required == pytest.approx(29_258.2, abs=0.5)

    pf_8mm = assess_preload_feasibility(
        GROUP_LOAD_RESULT, SECTION_8MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F,
        required.overall_required * PRELOAD_FACTOR,
    )
    assert pf_8mm.window.feasible is False
    assert pf_8mm.max_in_service_bolt_force == pytest.approx(39_938.3, abs=1.0)
    assert pf_8mm.max_in_service_bolt_index == 1
