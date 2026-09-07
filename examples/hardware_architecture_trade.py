"""Hardware installation-architecture trade closure: 10 mm + direct
preload verification vs. 12 mm + conventional torque-based installation
(Milestone 8).

This is a conceptual architecture trade, not a hardware qualification
or procurement decision. Complexity scores are normalized engineering
proxies, not dollar costs. The historical M5 10 mm selection is
preserved; Milestone 8 compares installation architectures rather than
silently rewriting prior selection logic.

Run with:
    python examples/hardware_architecture_trade.py
"""

from payload_bolts import (
    GEOMETRY_M10,
    GEOMETRY_M12,
    MAX_COMPLEXITY_THRESHOLD,
    MIN_TOLERANCE_RESERVE,
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
    circular_pattern,
    circular_unthreaded_bolt,
    compare_pareto,
    distribute_loads,
    epsilon_max_for_window,
    evaluate_architecture,
    installation_preload_window,
    required_preload,
)

# ---------------------------------------------------------------------------
# Same Milestone 1 group-load case as all earlier examples.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

# Milestone 2/3/4/6 illustrative baseline, unchanged.
BOLT_MATERIAL = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
STRENGTH_LIMITS = BoltStrengthLimits(name="Illustrative aerospace-grade alloy-steel fastener (proof/yield)", proof_strength=830e6, yield_strength=970e6)
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)
ETA_PROOF = 0.75
SCATTER_ALLOWANCE = 0.10
NUT_FACTOR_BASELINE = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)

# Milestone 5 local-joint screen geometry, unchanged.
PLATE = PlateMaterial(name="Illustrative plate/lug bearing material", bearing_allowable=400e6)
GEOMETRY = JointGeometry(
    plate_thickness=0.008, plate_center=(0.0, 0.0), plate_outer_radius=RADIUS + 0.05,
    edge_distance_min_ratio=1.5, spacing_min_ratio=3.0,
)

# Milestone 8 mass-model conceptual assumption: two clamped plates of
# the Milestone 5 illustrative plate thickness.
GRIP_LENGTH = 2.0 * GEOMETRY.plate_thickness


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_pct(x):
    return "n/a" if x is None else f"{x * 100.0:+.1f}%"


def main() -> None:
    print("HARDWARE INSTALLATION-ARCHITECTURE TRADE CLOSURE (Milestone 8)")
    print("Consumes the Milestone 1-7 8-bolt / R=0.5 m illustrative case unchanged.")
    print(
        "\n\"This is a conceptual architecture trade, not a hardware qualification or\n"
        "procurement decision.\"\n"
        "\"Complexity scores are normalized engineering proxies, not dollar costs.\"\n"
        "\"The historical M5 10 mm selection is preserved; Milestone 8 compares\n"
        "installation architectures rather than silently rewriting prior selection logic.\""
    )

    # -----------------------------------------------------------------
    # Inherited structural design
    # -----------------------------------------------------------------
    section_10mm = circular_unthreaded_bolt(0.010)
    m2 = assess_bolt_group_strength(GROUP_LOAD_RESULT, section_10mm, BOLT_MATERIAL)
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window_10mm = installation_preload_window(required.overall_required, section_10mm, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
    eps_max_10mm = epsilon_max_for_window(window_10mm.target_min, window_10mm.target_max)

    _rule("INHERITED STRUCTURAL DESIGN (Milestones 1-7, unchanged)")
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    print(f"  M1 max tensile bolt                = index {max_tensile.index}, {max_tensile.axial_total:,.1f} N")
    print(f"  M2 governing bolt/mode (10 mm)      = index {m2.governing_bolt_index} / {m2.governing_mode} / passed={m2.passed}")
    print(f"  M3 required preload (overall)       = {required.overall_required:,.1f} N/bolt")
    print(f"  M4/M5 10 mm preload window           = [{window_10mm.target_min:,.1f}, {window_10mm.target_max:,.1f}] N (epsilon_max={eps_max_10mm*100:.1f}%)")
    print("  M5 conceptual selection before installation-method analysis = 10 mm")
    print("  M6 finding: 10 mm baseline K=[0.15,0.25] -> NO_ROBUST_TORQUE_WINDOW; 12 mm -> robust")
    print("  M7 finding: 10 mm quantified direct methods (5-10% error) -> FEASIBLE (eps_max~=20.1%)")

    # -----------------------------------------------------------------
    # Mass model
    # -----------------------------------------------------------------
    _rule("MASS MODEL (see module docstring for source audit)")
    print("  material               = illustrative structural/carbon steel")
    print(f"  density                = 7,850 kg/m^3 (standard engineering value)")
    print(f"  grip length            = {GRIP_LENGTH*1000:.1f} mm (two clamped M5 plates)")
    print("  thread allowance       = 1.0 x nominal diameter (SAME rule for both candidates)")
    print(f"  fastener count (M1)     = {GROUP_LOAD_RESULT.pattern.n_bolts}")
    print("  mass-model components  = shank + hex head + hex nut (thread bore subtracted) + flat washer")
    print(f"  M10 geometry source     = {GEOMETRY_M10.source_basis}")
    print(f"  M12 geometry source     = {GEOMETRY_M12.source_basis}")
    print("  LIMITATION: reduced-order sourced-geometry proxy, not a CAD-accurate fastener model;")
    print("  no thread-root diameter reduction; single flat washer only (no lock washer/insert modeled).")

    # -----------------------------------------------------------------
    # Architectures
    # -----------------------------------------------------------------
    arch_a = evaluate_architecture(
        InstallationArchitecture.TEN_MM_DIRECT_VERIFICATION, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
        STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PLATE, GEOMETRY, GRIP_LENGTH,
    )
    arch_b = evaluate_architecture(
        InstallationArchitecture.TWELVE_MM_TORQUE_CONTROL, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
        STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PLATE, GEOMETRY, GRIP_LENGTH, nut_factor=NUT_FACTOR_BASELINE,
    )

    _rule("ARCHITECTURE A: 10 mm + direct preload verification")
    va = arch_a.verification_result
    print(f"  preload method          = {va.accuracy.method.value}  ({va.accuracy.source_basis})")
    print(f"  epsilon                 = {va.accuracy.relative_error:.2f}")
    print(f"  command window          = [{va.verified_window.command_min:,.1f}, {va.verified_window.command_max:,.1f}] N")
    print(f"  nominal target          = {va.nominal_target:,.1f} N")
    print(f"  achieved range          = [{va.achieved_min:,.1f}, {va.achieved_max:,.1f}] N")
    print(f"  in-service proof reserve = {_fmt_pct(arch_a.proof_reserve)}")
    print(f"  edge ratio / spacing ratio = {arch_a.edge_distance.min_ratio:.2f} / {arch_a.spacing.min_ratio:.2f}")
    print(f"  per-fastener mass        = {arch_a.mass.per_fastener.mass*1000:.2f} g")
    print(f"  total group mass         = {arch_a.mass.group_mass*1000:.1f} g")
    print(f"  complexity index         = {arch_a.complexity.total_score:.1f}  {arch_a.complexity}")
    print(f"  mandatory gates all pass? = {arch_a.admissible}")

    _rule("ARCHITECTURE B: 12 mm + conventional torque control")
    tb = arch_b.torque_result
    print(f"  K range                 = [{NUT_FACTOR_BASELINE.k_min:.2f}, {NUT_FACTOR_BASELINE.k_max:.2f}], K_nom={NUT_FACTOR_BASELINE.k_nom:.2f}")
    print(f"  robust torque window    = [{tb.robust_window.torque_robust_min:.2f}, {tb.robust_window.torque_robust_max:.2f}] N*m")
    print(f"  nominal torque           = {tb.nominal_torque:.2f} N*m")
    print(f"  achieved range           = [{tb.achieved_preload_at_k_max:,.1f}, {tb.achieved_preload_at_k_min:,.1f}] N")
    print(f"  in-service proof reserve = {_fmt_pct(arch_b.proof_reserve)}")
    print(f"  edge ratio / spacing ratio = {arch_b.edge_distance.min_ratio:.2f} / {arch_b.spacing.min_ratio:.2f}")
    print(f"  per-fastener mass        = {arch_b.mass.per_fastener.mass*1000:.2f} g")
    print(f"  total group mass         = {arch_b.mass.group_mass*1000:.1f} g")
    print(f"  complexity index         = {arch_b.complexity.total_score:.1f}  {arch_b.complexity}")
    print("  instrumentation requirement = none beyond standard calibrated torque wrench")
    print("  (NOTE: torque wrenches still require periodic calibration -- not zero process control)")
    print(f"  mandatory gates all pass? = {arch_b.admissible}")

    # -----------------------------------------------------------------
    # Comparison table
    # -----------------------------------------------------------------
    _rule("ARCHITECTURE COMPARISON")
    mass_delta = arch_b.mass.group_mass - arch_a.mass.group_mass
    mass_pct = (arch_b.mass.group_mass / arch_a.mass.group_mass - 1.0) * 100.0
    header = (
        f"{'':>28} {'A (10mm direct)':>18} {'B (12mm torque)':>18}"
    )
    print(header)
    print("-" * len(header))
    rows = [
        ("bolt diameter (mm)", f"{arch_a.nominal_diameter_mm:.1f}", f"{arch_b.nominal_diameter_mm:.1f}"),
        ("control robust?", str(arch_a.gates.installation_control_robust), str(arch_b.gates.installation_control_robust)),
        ("tolerance ratio", f"{arch_a.tolerance_ratio:.4f}", f"{arch_b.tolerance_ratio:.4f}"),
        ("in-service proof reserve", _fmt_pct(arch_a.proof_reserve), _fmt_pct(arch_b.proof_reserve)),
        ("bearing MS", f"{arch_a.bearing_governing_margin:+.2f}", f"{arch_b.bearing_governing_margin:+.2f}"),
        ("e/d", f"{arch_a.edge_distance.min_ratio:.2f}", f"{arch_b.edge_distance.min_ratio:.2f}"),
        ("s/d", f"{arch_a.spacing.min_ratio:.2f}", f"{arch_b.spacing.min_ratio:.2f}"),
        ("per-fastener mass (g)", f"{arch_a.mass.per_fastener.mass*1000:.2f}", f"{arch_b.mass.per_fastener.mass*1000:.2f}"),
        ("total group mass (g)", f"{arch_a.mass.group_mass*1000:.1f}", f"{arch_b.mass.group_mass*1000:.1f}"),
        ("mass delta vs A (g)", "--", f"{mass_delta*1000:+.1f} ({mass_pct:+.1f}%)"),
        ("complexity index", f"{arch_a.complexity.total_score:.1f}", f"{arch_b.complexity.total_score:.1f}"),
        ("mandatory gates pass?", str(arch_a.admissible), str(arch_b.admissible)),
    ]
    for label, va_, vb_ in rows:
        print(f"{label:>28} {va_:>18} {vb_:>18}")

    # -----------------------------------------------------------------
    # Pareto / selection
    # -----------------------------------------------------------------
    pareto = compare_pareto(arch_a, arch_b)
    eps_used_a = va.accuracy.relative_error
    decision = apply_trade_decision_rule(arch_a, arch_b, eps_max_10mm, eps_used_a)

    _rule("PARETO / SELECTION RESULT")
    print(f"  better mass            : {pareto.better_mass}")
    print(f"  better tolerance        : {pareto.better_tolerance}")
    print(f"  better proof reserve    : {pareto.better_proof_reserve}")
    print(f"  better packaging margin : {pareto.better_packaging}")
    print(f"  better complexity       : {pareto.better_complexity}")
    print(f"  A dominates B?          : {pareto.a_dominates_b}")
    print(f"  B dominates A?          : {pareto.b_dominates_a}")
    print(f"  NONDOMINATED (genuine trade)? : {pareto.nondominated}")
    print(f"\n  Predeclared decision rule: prefer A only if (1) both admissible, (2) A's mass")
    print(f"  is lower, (3) A's tolerance reserve (epsilon_max - epsilon_used) >= {MIN_TOLERANCE_RESERVE*100:.0f} "
          f"percentage points, (4) A's complexity <= {MAX_COMPLEXITY_THRESHOLD:.1f}.")
    print(f"\n  CONCEPTUAL SELECTION STATUS: {decision.status.value}")
    print(f"  REASON: {decision.reason}")

    # -----------------------------------------------------------------
    # Sensitivities
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: bolt/grip length (0.75x / 1.0x / 1.25x baseline)")
    from payload_bolts import compute_fastener_mass

    header = f"{'factor':>7} {'grip_mm':>8} {'A mass_g':>9} {'B mass_g':>9} {'delta_%':>8}"
    print(header)
    print("-" * len(header))
    for factor in (0.75, 1.0, 1.25):
        grip = GRIP_LENGTH * factor
        ma = compute_fastener_mass(GEOMETRY_M10, grip)
        mb = compute_fastener_mass(GEOMETRY_M12, grip)
        pct = (mb.mass / ma.mass - 1.0) * 100.0
        print(f"{factor:>7.2f} {grip*1000:>8.2f} {ma.mass*8*1000:>9.1f} {mb.mass*8*1000:>9.1f} {pct:>8.1f}")

    _rule("SENSITIVITY B: material density (reasonable illustrative range)")
    header = f"{'rho_kg/m3':>10} {'A mass_g':>9} {'B mass_g':>9}"
    print(header)
    print("-" * len(header))
    for rho in (7000.0, 7850.0, 8500.0):
        ma = compute_fastener_mass(GEOMETRY_M10, GRIP_LENGTH, density=rho)
        mb = compute_fastener_mass(GEOMETRY_M12, GRIP_LENGTH, density=rho)
        print(f"{rho:>10.0f} {ma.mass*8*1000:>9.1f} {mb.mass*8*1000:>9.1f}")

    _rule("SENSITIVITY C: direct-measurement epsilon (10 mm), reserve to epsilon_max")
    header = f"{'epsilon':>8} {'reserve_pp':>10} {'feasible?':>10}"
    print(header)
    print("-" * len(header))
    for eps in (0.02, 0.05, 0.10, 0.15):
        reserve = (eps_max_10mm - eps) * 100.0
        print(f"{eps:>8.2f} {reserve:>10.1f} {str(eps < eps_max_10mm):>10}")

    _rule("SENSITIVITY D: torque K-range width (12 mm)")
    header = f"{'family':>10} {'K_min':>6} {'K_max':>6} {'robust?':>8}"
    print(header)
    print("-" * len(header))
    for label, kmin, kmax in (("narrow", 0.18, 0.22), ("nominal", 0.15, 0.25), ("wide", 0.12, 0.28)):
        nf = NutFactorModel(k_min=kmin, k_nom=(kmin + kmax) / 2.0, k_max=kmax)
        arch_b_sens = evaluate_architecture(
            InstallationArchitecture.TWELVE_MM_TORQUE_CONTROL, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
            STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PLATE, GEOMETRY, GRIP_LENGTH, nut_factor=nf,
        )
        print(f"{label:>10} {kmin:>6.2f} {kmax:>6.2f} {str(arch_b_sens.gates.installation_control_robust):>8}")

    _rule("SENSITIVITY E: installation-complexity sub-score variation (A only; one factor at a time)")
    from payload_bolts import compute_complexity_index

    print(f"  baseline A complexity: hardware=2, measurement=2, calibration=2, process=1 -> total=7.0 (threshold {MAX_COMPLEXITY_THRESHOLD:.1f})")
    header = f"{'factor reduced by 1':>22} {'new total':>10} {'<=threshold?':>13} {'decision if all else fixed':>28}"
    print(header)
    print("-" * len(header))
    base_scores = {"hardware": 2, "measurement": 2, "calibration": 2, "process": 1}
    for name in base_scores:
        scores = dict(base_scores)
        scores[name] = max(0, scores[name] - 1)
        ci = compute_complexity_index(scores["hardware"], scores["measurement"], scores["calibration"], scores["process"])
        would_pass = ci.total_score <= MAX_COMPLEXITY_THRESHOLD
        arch_a_sens = evaluate_architecture(
            InstallationArchitecture.TEN_MM_DIRECT_VERIFICATION, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
            STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PLATE, GEOMETRY, GRIP_LENGTH, complexity=ci,
        )
        decision_sens = apply_trade_decision_rule(arch_a_sens, arch_b, eps_max_10mm, eps_used_a)
        print(f"{name:>22} {ci.total_score:>10.1f} {str(would_pass):>13} {decision_sens.status.value:>28}")
    print(
        "\n  GENUINE FINDING: reducing ANY ONE complexity sub-score by 1 point (e.g. a mature,\n"
        "  pre-qualified instrumented-bolt product line reducing calibration burden) flips the\n"
        "  complexity gate and the predeclared rule's recommendation from PREFER_B to PREFER_A --\n"
        "  the baseline result is close to the declared threshold, not a wide margin."
    )

    print("\n  Sub-sensitivity: measurement-error assumption crossover for the reserve criterion")
    header2 = f"{'epsilon':>8} {'reserve_pp':>10} {'reserve>=5pp?':>13}"
    print(header2)
    print("-" * len(header2))
    for eps in (0.10, 0.15, 0.1513, 0.16):
        reserve = (eps_max_10mm - eps) * 100.0
        print(f"{eps:>8.4f} {reserve:>10.2f} {str(reserve >= MIN_TOLERANCE_RESERVE * 100):>13}")
    print("  -> the reserve criterion alone would fail (independent of complexity) above epsilon~=15.1%.")

    _rule("SENSITIVITY F: structural carry-forward (delta_F, mu, eta_proof)")
    from payload_bolts import FrictionModel as _FrictionModel

    header = f"{'param':>12} {'value':>8} {'10mm eps_max':>13} {'12mm robust?':>13}"
    print(header)
    print("-" * len(header))
    for delta in (0.05, 0.10, 0.20):
        w = installation_preload_window(required.overall_required, section_10mm, STRENGTH_LIMITS, ETA_PROOF, delta)
        em = epsilon_max_for_window(w.target_min, w.target_max) if w.feasible else float("nan")
        print(f"{'delta_F':>12} {delta:>8.2f} {em*100:>12.1f}% {'n/a':>13}")
    for mu in (0.20, 0.30, 0.40):
        req_mu = required_preload(GROUP_LOAD_RESULT, STIFFNESS, _FrictionModel(friction_coefficient=mu))
        w = installation_preload_window(req_mu.overall_required, section_10mm, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        em = epsilon_max_for_window(w.target_min, w.target_max) if w.feasible else float("nan")
        arch_b_mu = evaluate_architecture(
            InstallationArchitecture.TWELVE_MM_TORQUE_CONTROL, GROUP_LOAD_RESULT, BOLT_MATERIAL, STRENGTH_LIMITS,
            STIFFNESS, _FrictionModel(friction_coefficient=mu), ETA_PROOF, SCATTER_ALLOWANCE, PLATE, GEOMETRY, GRIP_LENGTH,
            nut_factor=NUT_FACTOR_BASELINE,
        )
        print(f"{'mu':>12} {mu:>8.2f} {em*100:>12.1f}% {str(arch_b_mu.gates.installation_control_robust):>13}")
    for eta in (0.60, 0.70, 0.80):
        w = installation_preload_window(required.overall_required, section_10mm, STRENGTH_LIMITS, eta, SCATTER_ALLOWANCE)
        em = epsilon_max_for_window(w.target_min, w.target_max) if w.feasible else float("nan")
        print(f"{'eta_proof':>12} {eta:>8.2f} {em*100:>12.1f}% {'n/a':>13}")

    print(
        "\n  None of these sensitivities change the fundamental trade shape: A always wins mass\n"
        "  and packaging margin; B always wins tolerance/complexity at the illustrative baseline\n"
        "  weights. The conceptual recommendation follows the predeclared rule only and is not\n"
        "  re-tuned per sensitivity case."
    )


if __name__ == "__main__":
    main()
