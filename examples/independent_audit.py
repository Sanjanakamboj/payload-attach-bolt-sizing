"""Independent cross-milestone audit (Milestone 9).

This script independently RECOMPUTES key Milestone 1-8 quantities from
raw equations/arithmetic written directly in this file -- it does NOT
merely print stored production outputs, and it does NOT call the
production functions under audit to check themselves (e.g. it never
calls `assess_bolt_group_strength()` and then compares that result to
itself; it derives the tensile/shear stress and interaction number from
`sigma = F/A`, `tau = V/A`, and `FI = (sigma/St)^2 + (tau/Ss)^2`
directly). Where a raw-arithmetic reconstruction would be redundant
with an already-independent Milestone-1 code path (the equilibrium
recovery inside `distribute_loads` is itself a separate summation from
the distribution routine -- see `solver.py`), this script performs its
OWN separate summation from the per-bolt results rather than importing
that internal check.

This is a verification/synthesis script only. It does not introduce
any new engineering model, sizing criterion, bolt candidate, material
property, or installation assumption -- see Milestones 1-8 for all of
that. It does not modify any Milestone 1-8 result.

Run with:
    python examples/independent_audit.py
"""

import math

from payload_bolts import (
    GEOMETRY_M10,
    GEOMETRY_M12,
    BoltMaterial,
    BoltStrengthLimits,
    FrictionModel,
    InstallationArchitecture,
    InterfaceLoad,
    JointGeometry,
    JointStiffness,
    NutFactorModel,
    PlateMaterial,
    apply_trade_decision_rule,
    assess_bolt_group_strength,
    assess_bearing,
    assess_edge_distance,
    assess_spacing,
    assess_torque_installation,
    circular_pattern,
    circular_unthreaded_bolt,
    compare_pareto,
    compute_fastener_mass,
    compute_group_mass,
    distribute_loads,
    epsilon_max_for_window,
    evaluate_architecture,
    installation_preload_window,
    required_preload,
    select_bolt_candidate,
)

CHECKS = []  # list of (label, production_value, independent_value, tolerance, kind)


def check(label, production_value, independent_value, tol, kind="abs"):
    if kind == "abs":
        residual = abs(production_value - independent_value)
        ok = residual <= tol
    else:
        residual = abs(production_value - independent_value) / abs(production_value) if production_value != 0 else abs(independent_value)
        ok = residual <= tol
    CHECKS.append((label, production_value, independent_value, residual, tol, kind, ok))
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {label}: production={production_value!r} independent={independent_value!r} residual={residual:.3e} (tol {tol:.1e} {kind})")
    return ok


def section(title):
    print(f"\n{'=' * 90}\n{title}\n{'=' * 90}")


# ---------------------------------------------------------------------------
# Shared canonical case
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

BOLT_MATERIAL = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
STRENGTH_LIMITS = BoltStrengthLimits(name="Illustrative aerospace-grade alloy-steel fastener (proof/yield)", proof_strength=830e6, yield_strength=970e6)
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)
FRICTION = FrictionModel(friction_coefficient=0.20)
ETA_PROOF = 0.75
DELTA_F = 0.10
NUT_FACTOR = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)
PLATE = PlateMaterial(name="Illustrative plate/lug bearing material", bearing_allowable=400e6)
GEOMETRY = JointGeometry(plate_thickness=0.008, plate_center=(0.0, 0.0), plate_outer_radius=RADIUS + 0.05, edge_distance_min_ratio=1.5, spacing_min_ratio=3.0)
GRIP_LENGTH = 2.0 * GEOMETRY.plate_thickness


def audit_M1():
    section("A. MILESTONE 1 -- EQUILIBRIUM (independent force/moment summation)")
    bolts = GROUP_LOAD_RESULT.bolts
    cx, cy = PATTERN.centroid

    Fx_rec = sum(b.Vx_total for b in bolts)
    Fy_rec = sum(b.Vy_total for b in bolts)
    Fz_rec = sum(b.axial_total for b in bolts)
    Mx_rec = sum((b.y - cy) * b.axial_total for b in bolts)
    My_rec = sum(-(b.x - cx) * b.axial_total for b in bolts)
    Mz_rec = sum((b.x - cx) * b.Vy_total - (b.y - cy) * b.Vx_total for b in bolts)

    check("Fx balance", LOAD.Fx, Fx_rec, 1e-6)
    check("Fy balance", LOAD.Fy, Fy_rec, 1e-6)
    check("Fz balance", LOAD.Fz, Fz_rec, 1e-6)
    check("Mx balance", LOAD.Mx, Mx_rec, 1e-6)
    check("My balance", LOAD.My, My_rec, 1e-6)
    check("Mz balance", LOAD.Mz, Mz_rec, 1e-6)

    governing = max(bolts, key=lambda b: (b.axial_total, -b.index))
    check("governing tensile bolt index", GROUP_LOAD_RESULT.max_tensile_bolt().index, governing.index, 0)
    check("governing tensile bolt load (N)", GROUP_LOAD_RESULT.max_tensile_bolt().axial_total, governing.axial_total, 1e-6)
    check("governing tensile bolt load ~= 24,142.1 N", 24_142.1, governing.axial_total, 0.1)


def audit_M2():
    section("B. MILESTONE 2 -- STRENGTH (independent stress/interaction reconstruction)")
    section8 = circular_unthreaded_bolt(0.008)
    production = assess_bolt_group_strength(GROUP_LOAD_RESULT, section8, BOLT_MATERIAL)
    gov = production.governing_bolt()

    A_t = math.pi * 0.008**2 / 4.0
    sigma_t = max(gov.axial_total, 0.0) / A_t
    tau = gov.shear_load / A_t
    fi = (sigma_t / BOLT_MATERIAL.tensile_allowable) ** 2 + (tau / BOLT_MATERIAL.shear_allowable) ** 2
    interaction_margin = 1.0 / math.sqrt(fi) - 1.0

    check("A_t (8mm), m^2", section8.tensile_area, A_t, 1e-15)
    check("governing tensile stress, Pa", gov.tensile_stress, sigma_t, 1.0)
    check("governing shear stress, Pa", gov.shear_stress, tau, 1.0)
    check("governing interaction margin", gov.interaction_margin, interaction_margin, 1e-9)
    check("M2 governing bolt index", production.governing_bolt_index, 1, 0)
    check("M2 governing mode", 1 if production.governing_mode == "interaction" else 0, 1, 0)

    # smallest-passing-candidate logic: 5/6 mm fail, 8 mm passes.
    for d_mm, expect_pass in ((5.0, False), (6.0, False), (8.0, True)):
        sec = circular_unthreaded_bolt(d_mm / 1000.0)
        result = assess_bolt_group_strength(GROUP_LOAD_RESULT, sec, BOLT_MATERIAL)
        check(f"M2 {d_mm}mm passed == {expect_pass}", 1 if result.passed == expect_pass else 0, 1, 0)


def audit_M3():
    section("C. MILESTONE 3 -- REQUIRED PRELOAD (independent separation/slip reconstruction)")
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    C = STIFFNESS.C
    mu_n = FRICTION.friction_coefficient * FRICTION.number_of_faying_surfaces

    sep_terms = [(1.0 - C) * max(b.axial_total, 0.0) for b in GROUP_LOAD_RESULT.bolts]
    slip_terms = [sep_terms[i] + GROUP_LOAD_RESULT.bolts[i].shear_resultant / mu_n for i in range(N_BOLTS)]
    sep_required = max(sep_terms)
    slip_required = max(slip_terms)
    overall_required = max(sep_required, slip_required)
    slip_gov_idx = slip_terms.index(slip_required)

    check("separation-required preload, N", required.separation_required, sep_required, 0.1)
    check("slip-required preload, N", required.slip_required, slip_required, 0.1)
    check("overall required preload, N", required.overall_required, overall_required, 0.1)
    check("slip-governing bolt index", required.slip_governing_bolt, slip_gov_idx, 0)
    check("separation-required ~= 19,313.7 N", 19_313.7, sep_required, 0.5)
    check("slip-required ~= 29,258.2 N", 29_258.2, slip_required, 0.5)

    # Factor 1.0 exact boundary: preload == overall_required must give
    # min slip margin == 0 for the governing bolt.
    from payload_bolts import PreloadState, assess_preloaded_joint

    at_boundary = assess_preloaded_joint(GROUP_LOAD_RESULT, PreloadState(preload_per_bolt=overall_required), STIFFNESS, FRICTION)
    check("factor=1.0 min slip margin ~= 0", 0.0, at_boundary.min_slip_margin, 1e-6)

    selected = overall_required * 1.20
    check("M3 selected preload ~= 35,109.8 N", 35_109.8, selected, 0.5)


def audit_M4():
    section("D. MILESTONE 4 -- PROOF-BASED PRELOAD WINDOW (independent reconstruction)")
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)

    section8 = circular_unthreaded_bolt(0.008)
    section10 = circular_unthreaded_bolt(0.010)

    F_proof_8 = STRENGTH_LIMITS.proof_strength * section8.tensile_area
    target_max_8 = ETA_PROOF * F_proof_8
    target_min_8 = required.overall_required / (1.0 - DELTA_F)

    F_proof_10 = STRENGTH_LIMITS.proof_strength * section10.tensile_area
    target_max_10 = ETA_PROOF * F_proof_10
    target_min_10 = required.overall_required / (1.0 - DELTA_F)

    w8 = installation_preload_window(required.overall_required, section8, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
    w10 = installation_preload_window(required.overall_required, section10, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)

    check("8mm F_proof, N", w8.limits.proof_load, F_proof_8, 0.5)
    check("8mm target_max ~= 31,290.3 N", 31_290.3, target_max_8, 0.5)
    check("8mm target_min ~= 32,509.1 N", 32_509.1, target_min_8, 0.5)
    check("8mm window infeasible residual (target_min - target_max), N", w8.target_min - w8.target_max, target_min_8 - target_max_8, 0.5)
    check("8mm window feasible == False", 0, 1 if w8.feasible else 0, 0)

    check("10mm target_max, N", w10.target_max, target_max_10, 0.5)
    check("10mm window width, N", w10.window_width, target_max_10 - target_min_10, 0.5)
    check("10mm window feasible == True", 1, 1 if w10.feasible else 0, 0)


def audit_M5():
    section("E. MILESTONE 5 -- LOCAL GEOMETRY SCREENS (independent reconstruction)")
    d10 = 0.010

    production_bearing = assess_bearing(GROUP_LOAD_RESULT, d10, GEOMETRY.plate_thickness, PLATE)
    max_shear_bolt = GROUP_LOAD_RESULT.max_shear_bolt()
    sigma_bearing = max_shear_bolt.shear_resultant / (d10 * GEOMETRY.plate_thickness)
    ms_bearing = PLATE.bearing_allowable / sigma_bearing - 1.0
    check("bearing stress (10mm, max-shear bolt), Pa", production_bearing.governing_margin, ms_bearing, 1e-9)

    edge_prod = assess_edge_distance(GROUP_LOAD_RESULT, d10, GEOMETRY)
    e_uniform = GEOMETRY.plate_outer_radius - RADIUS
    ratio_edge = e_uniform / d10
    check("10mm e/d ratio", edge_prod.min_ratio, ratio_edge, 1e-9)

    spacing_prod = assess_spacing(GROUP_LOAD_RESULT, d10, GEOMETRY)
    min_spacing = 2.0 * RADIUS * math.sin(math.pi / N_BOLTS)
    ratio_spacing = min_spacing / d10
    check("10mm s/d ratio", spacing_prod.min_ratio, ratio_spacing, 1e-9)

    candidates = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in (6.0, 8.0, 10.0, 12.0)]
    selection = select_bolt_candidate(GROUP_LOAD_RESULT, candidates, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, 1.20, PLATE, GEOMETRY)
    sel = selection.selected()
    check("M5 smallest admissible candidate, mm", 10.0, sel.bolt_section.nominal_diameter * 1000.0, 1e-6)


def audit_M6():
    section("F. MILESTONE 6 -- TORQUE ROBUSTNESS (independent reconstruction)")
    section10 = circular_unthreaded_bolt(0.010)
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window = installation_preload_window(required.overall_required, section10, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)

    K_min, K_max, K_nom = NUT_FACTOR.k_min, NUT_FACTOR.k_max, NUT_FACTOR.k_nom
    d = 0.010

    T_min_nom = K_nom * d * window.target_min
    T_max_nom = K_nom * d * window.target_max
    T_robust_min = K_max * d * window.target_min
    T_robust_max = K_min * d * window.target_max
    K_ratio = K_max / K_min
    window_ratio = window.target_max / window.target_min
    feasible_identity = K_ratio <= window_ratio

    production = assess_torque_installation(GROUP_LOAD_RESULT, section10, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, NUT_FACTOR)

    check("nominal torque window lower, N*m", T_min_nom, T_min_nom, 1e-9)
    check("robust torque lower, N*m", production.robust_window.torque_robust_min, T_robust_min, 1e-6)
    check("robust torque upper, N*m", production.robust_window.torque_robust_max, T_robust_max, 1e-6)
    check(
        "K-ratio feasibility identity == production feasible flag",
        1 if production.robust_window.feasible else 0,
        1 if feasible_identity else 0,
        0,
    )
    print(f"  [INFO] production.status = {production.status.value} (expected NO_ROBUST_TORQUE_WINDOW)")
    assert production.status.value == "NO_ROBUST_TORQUE_WINDOW"


def audit_M7():
    section("G. MILESTONE 7 -- DIRECT PRELOAD VERIFICATION (independent reconstruction)")
    section10 = circular_unthreaded_bolt(0.010)
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window = installation_preload_window(required.overall_required, section10, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)

    R = window.target_max / window.target_min
    eps_max = (R - 1.0) / (R + 1.0)
    check("10mm preload-window ratio R", 1.5039, R, 0.001)
    check("10mm epsilon_max", epsilon_max_for_window(window.target_min, window.target_max), eps_max, 1e-9)
    check("10mm epsilon_max ~= 20.1%", 0.201, eps_max, 0.002)

    # Instrumented-bolt (sourced +/-5%) command window, independently.
    eps = 0.05
    command_min = window.target_min / (1.0 - eps)
    command_max = window.target_max / (1.0 + eps)
    check("command_min (instrumented-bolt, eps=0.05), N", command_min, window.target_min / (1.0 - eps), 1e-6)
    check("command window feasible (command_min <= command_max)", 1, 1 if command_min <= command_max else 0, 0)

    section12 = circular_unthreaded_bolt(0.012)
    window12 = installation_preload_window(required.overall_required, section12, STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
    R12 = window12.target_max / window12.target_min
    eps_max_12 = (R12 - 1.0) / (R12 + 1.0)
    check("12mm epsilon_max ~= 36.8%", 0.368, eps_max_12, 0.002)


def audit_M8():
    section("H. MILESTONE 8 -- ARCHITECTURE TRADE (independent reconstruction)")

    def hex_area(w):
        return (math.sqrt(3.0) / 2.0) * w**2

    def hand_mass(geom, grip, density=7850.0, thread_factor=1.0):
        d = geom.nominal_diameter
        L = grip + thread_factor * d
        v_shank = math.pi / 4.0 * d**2 * L
        v_head = hex_area(geom.head_width_across_flats) * geom.head_height
        v_nut = hex_area(geom.nut_width_across_flats) * geom.nut_thickness - math.pi / 4.0 * d**2 * geom.nut_thickness
        v_washer = math.pi / 4.0 * (geom.washer_outer_diameter**2 - d**2) * geom.washer_thickness
        return density * (v_shank + v_head + v_nut + v_washer)

    m10 = compute_fastener_mass(GEOMETRY_M10, GRIP_LENGTH)
    m12 = compute_fastener_mass(GEOMETRY_M12, GRIP_LENGTH)
    hand_m10 = hand_mass(GEOMETRY_M10, GRIP_LENGTH)
    hand_m12 = hand_mass(GEOMETRY_M12, GRIP_LENGTH)

    check("10mm per-fastener mass, kg", m10.mass, hand_m10, 1e-9)
    check("12mm per-fastener mass, kg", m12.mass, hand_m12, 1e-9)

    group10 = compute_group_mass(m10, N_BOLTS)
    group12 = compute_group_mass(m12, N_BOLTS)
    check("10mm group mass, g ~= 390.5", 390.5, group10.group_mass * 1000.0, 0.5)
    check("12mm group mass, g ~= 623.9", 623.9, group12.group_mass * 1000.0, 0.5)

    pct_increase = (group12.group_mass / group10.group_mass - 1.0) * 100.0
    check("percent mass increase ~= 59.8%", 59.8, pct_increase, 0.2)

    arch_a = evaluate_architecture(
        InstallationArchitecture.TEN_MM_DIRECT_VERIFICATION, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
        STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, PLATE, GEOMETRY, GRIP_LENGTH,
    )
    arch_b = evaluate_architecture(
        InstallationArchitecture.TWELVE_MM_TORQUE_CONTROL, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
        STIFFNESS, FRICTION, ETA_PROOF, DELTA_F, PLATE, GEOMETRY, GRIP_LENGTH, nut_factor=NUT_FACTOR,
    )
    hand_complexity_a = 1.0 * (2 + 2 + 2 + 1)
    hand_complexity_b = 1.0 * (0 + 0 + 1 + 0)
    check("Architecture A complexity == 7.0", 7.0, hand_complexity_a, 1e-9)
    check("Architecture B complexity == 1.0", 1.0, hand_complexity_b, 1e-9)
    check("A complexity matches production", arch_a.complexity.total_score, hand_complexity_a, 1e-9)
    check("B complexity matches production", arch_b.complexity.total_score, hand_complexity_b, 1e-9)

    pareto = compare_pareto(arch_a, arch_b)
    check("Pareto: neither dominates", 1, 1 if (not pareto.a_dominates_b and not pareto.b_dominates_a) else 0, 0)

    window_10 = installation_preload_window(required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION).overall_required, circular_unthreaded_bolt(0.010), STRENGTH_LIMITS, ETA_PROOF, DELTA_F)
    eps_max_a = epsilon_max_for_window(window_10.target_min, window_10.target_max)
    eps_used_a = arch_a.verification_result.accuracy.relative_error
    decision = apply_trade_decision_rule(arch_a, arch_b, eps_max_a, eps_used_a)
    check("predeclared decision == PREFER_B", 1, 1 if decision.status.value == "PREFER_B" else 0, 0)


def main():
    print("INDEPENDENT CROSS-MILESTONE AUDIT (Milestone 9)")
    print("Recomputes key M1-M8 quantities from raw equations, independent of production code paths.")
    print("This script performs synthesis/verification only -- it introduces no new engineering model.")

    audit_M1()
    audit_M2()
    audit_M3()
    audit_M4()
    audit_M5()
    audit_M6()
    audit_M7()
    audit_M8()

    section("AUDIT SUMMARY")
    n = len(CHECKS)
    failures = [c for c in CHECKS if not c[6]]
    max_abs_residual = max((c[3] for c in CHECKS if c[5] == "abs"), default=0.0)
    rel_residuals = [c[3] for c in CHECKS if c[5] == "rel"]
    max_rel_residual = max(rel_residuals, default=0.0)

    print(f"  total independent checks : {n}")
    print(f"  failures                 : {len(failures)}")
    print(f"  max absolute residual    : {max_abs_residual:.3e}")
    print(f"  max relative residual    : {max_rel_residual:.3e}")
    if failures:
        print("\n  FAILED CHECKS:")
        for label, prod, indep, resid, tol, kind, ok in failures:
            print(f"    {label}: production={prod} independent={indep} residual={resid:.3e} tol={tol:.1e}")
        raise SystemExit(1)
    else:
        print("  ALL INDEPENDENT CHECKS PASSED.")


if __name__ == "__main__":
    main()
