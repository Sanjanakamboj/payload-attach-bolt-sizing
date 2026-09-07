"""Direct-preload verification methods and installation-control trade
for the Milestone 5 selected 10 mm candidate, compared against the
Milestone 6 torque-only result (Milestone 7).

Direct preload verification reduces dependence on torque-friction
uncertainty but does not eliminate calibration or measurement error.
Accuracy ranges are deterministic screening assumptions unless
explicitly tied to a checked source. This analysis is not an
installation work instruction or qualification procedure.

Run with:
    python examples/preload_verification_trade.py
"""

from payload_bolts import (
    BoltMaterial,
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    NutFactorModel,
    assess_bolt_group_strength,
    assess_torque_installation,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    epsilon_max_for_window,
    evaluate_installation_methods,
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
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)
ETA_PROOF = 0.75
SCATTER_ALLOWANCE = 0.10
NUT_FACTOR_BASELINE = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)

SELECTED_DIAMETER_MM = 10.0
SELECTED_SECTION = circular_unthreaded_bolt(SELECTED_DIAMETER_MM / 1000.0)
SECTION_12MM = circular_unthreaded_bolt(0.012)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_pct(x):
    return "n/a" if x is None else f"{x * 100.0:+.1f}%"


def _fmt(x, fmt="{:,.1f}"):
    return "n/a" if x is None else fmt.format(x)


def main() -> None:
    print("DIRECT-PRELOAD VERIFICATION METHODS AND INSTALLATION-CONTROL TRADE (Milestone 7)")
    print("Consumes the Milestone 1-6 8-bolt / R=0.5 m illustrative case unchanged.")
    print(
        "\n\"Direct preload verification reduces dependence on torque-friction uncertainty but\n"
        "does not eliminate calibration or measurement error.\"\n"
        "\"Accuracy ranges are deterministic screening assumptions unless explicitly tied to a\n"
        "checked source.\"\n"
        "\"This analysis is not an installation work instruction or qualification procedure.\""
    )

    # -----------------------------------------------------------------
    # Inherited design
    # -----------------------------------------------------------------
    m2 = assess_bolt_group_strength(GROUP_LOAD_RESULT, SELECTED_SECTION, BOLT_MATERIAL)
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
    torque_result = assess_torque_installation(GROUP_LOAD_RESULT, SELECTED_SECTION, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, NUT_FACTOR_BASELINE)

    _rule("INHERITED DESIGN (Milestones 1-6, unchanged)")
    print(f"  selected conceptual bolt (M5)          = {SELECTED_DIAMETER_MM:.1f} mm")
    print(f"  M2 strength: governing bolt/mode/pass   = index {m2.governing_bolt_index} / {m2.governing_mode} / {m2.passed}")
    print(f"  M3 required preload (overall)           = {required.overall_required:,.1f} N/bolt (governing: {required.overall_governing_constraint})")
    print(f"  M4/M5 preload window                     = [{window.target_min:,.1f}, {window.target_max:,.1f}] N")
    print(f"  M6 torque-only robust status             = {torque_result.status.value}")

    # -----------------------------------------------------------------
    # Preload-window tolerance
    # -----------------------------------------------------------------
    R = window.target_max / window.target_min
    eps_max = epsilon_max_for_window(window.target_min, window.target_max)
    _rule("PRELOAD-WINDOW TOLERANCE (10 mm)")
    print(f"  F_window,min = {window.target_min:,.1f} N")
    print(f"  F_window,max = {window.target_max:,.1f} N")
    print(f"  ratio R = F_max/F_min = {R:.4f}")
    print(f"  epsilon_max = (R-1)/(R+1) = {eps_max:.4f}  ({eps_max*100:.1f}%)")
    print("  -> the maximum SYMMETRIC deterministic preload-measurement/control error this")
    print("     window can tolerate at all, regardless of method")

    # -----------------------------------------------------------------
    # Verification methods table
    # -----------------------------------------------------------------
    trade = evaluate_installation_methods(GROUP_LOAD_RESULT, SELECTED_SECTION, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE)
    _rule("VERIFICATION METHODS (10 mm)")
    header = (
        f"{'method':>20} {'type':>8} {'eps':>6} {'src':>5} {'cmd_min':>10} {'cmd_max':>10} "
        f"{'width':>10} {'nominal':>10} {'ach_min':>10} {'ach_max':>10} {'reserve':>8} {'status':>24}"
    )
    print(header)
    print("-" * len(header))
    print(
        f"{'TORQUE_ONLY (M6)':>20} {'indirect':>8} {'n/a':>6} {'src':>5} "
        f"{_fmt(torque_result.robust_window.torque_robust_min):>10} {_fmt(torque_result.robust_window.torque_robust_max):>10} "
        f"{torque_result.robust_window.width:>10.2f} {_fmt(torque_result.nominal_torque):>10} "
        f"{'--':>10} {'--':>10} {_fmt_pct(None):>8} {torque_result.status.value:>24}"
    )
    for r in trade.results:
        src = "src" if not r.accuracy.is_illustrative else "illus"
        eps_str = "n/a" if r.accuracy.relative_error is None else f"{r.accuracy.relative_error:.2f}"
        cmd_min = _fmt(r.verified_window.command_min) if r.verified_window else "n/a"
        cmd_max = _fmt(r.verified_window.command_max) if r.verified_window else "n/a"
        width = f"{r.verified_window.width:,.1f}" if r.verified_window else "n/a"
        print(
            f"{r.accuracy.method.value:>20} {r.accuracy.measurement_type:>8} {eps_str:>6} {src:>5} "
            f"{cmd_min:>10} {cmd_max:>10} {width:>10} {_fmt(r.nominal_target):>10} "
            f"{_fmt(r.achieved_min):>10} {_fmt(r.achieved_max):>10} {_fmt_pct(r.proof_reserve):>8} {r.status.value:>24}"
        )

    print("\n  Major limitations:")
    for r in trade.results:
        print(f"    {r.accuracy.method.value:>20}: {r.accuracy.limitation}")

    # -----------------------------------------------------------------
    # 10 mm result
    # -----------------------------------------------------------------
    _rule("10 mm RESULT")
    print(f"  feasible methods: {[m.value for m in trade.feasible_methods]}")
    print(f"  any feasible?    : {trade.any_feasible}")

    # -----------------------------------------------------------------
    # 12 mm comparison
    # -----------------------------------------------------------------
    window_12mm = installation_preload_window(required.overall_required, SECTION_12MM, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
    eps_max_12 = epsilon_max_for_window(window_12mm.target_min, window_12mm.target_max)
    trade_12mm = evaluate_installation_methods(GROUP_LOAD_RESULT, SECTION_12MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE)
    torque_result_12mm = assess_torque_installation(GROUP_LOAD_RESULT, SECTION_12MM, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, NUT_FACTOR_BASELINE)

    _rule("12 mm COMPARISON")
    print(f"  preload window          = [{window_12mm.target_min:,.1f}, {window_12mm.target_max:,.1f}] N")
    print(f"  ratio R                 = {window_12mm.target_max/window_12mm.target_min:.4f}  (10 mm: {R:.4f})")
    print(f"  epsilon_max             = {eps_max_12:.4f}  ({eps_max_12*100:.1f}%)  (10 mm: {eps_max*100:.1f}%)")
    print(f"  feasible methods        : {[m.value for m in trade_12mm.feasible_methods]}")
    print(f"  M6 torque-only status   : {torque_result_12mm.status.value}  (10 mm: {torque_result.status.value})")

    # -----------------------------------------------------------------
    # Sensitivities
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: direct-preload measurement accuracy (epsilon), 10 mm")
    header = f"{'epsilon':>8} {'cmd_min':>11} {'cmd_max':>11} {'width':>11} {'feasible?':>10}"
    print(header)
    print("-" * len(header))
    from payload_bolts import command_window
    for eps in (0.02, 0.05, 0.10, 0.15, 0.20):
        vw = command_window(window.target_min, window.target_max, eps)
        print(f"{eps:>8.2f} {vw.command_min:>11,.1f} {vw.command_max:>11,.1f} {vw.width:>11,.1f} {str(vw.feasible):>10}")

    _rule("SENSITIVITY B: bolt size (10/12 mm)")
    header = f"{'d_mm':>5} {'epsilon_max':>12} {'feasible_methods':>18}"
    print(header)
    print("-" * len(header))
    for d_mm, section in ((10.0, SELECTED_SECTION), (12.0, SECTION_12MM)):
        w = installation_preload_window(required.overall_required, section, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        em = epsilon_max_for_window(w.target_min, w.target_max)
        t = evaluate_installation_methods(GROUP_LOAD_RESULT, section, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE)
        print(f"{d_mm:>5.1f} {em*100:>11.1f}% {len(t.feasible_methods):>18}")

    _rule("SENSITIVITY C: preload scatter allowance delta_F (5%/10%/20%), 10 mm")
    header = f"{'delta_F':>8} {'target_min':>11} {'epsilon_max':>12}"
    print(header)
    print("-" * len(header))
    for delta in (0.05, 0.10, 0.20):
        w = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, delta)
        em = epsilon_max_for_window(w.target_min, w.target_max)
        print(f"{delta:>8.2f} {w.target_min:>11,.1f} {em*100:>11.1f}%")

    _rule("SENSITIVITY D: friction coefficient mu (carried forward from M3), 10 mm")
    header = f"{'mu':>6} {'F_required':>11} {'window feasible?':>17} {'epsilon_max':>12}"
    print(header)
    print("-" * len(header))
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        req_mu = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FrictionModel(friction_coefficient=mu))
        w = installation_preload_window(req_mu.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        em_str = f"{epsilon_max_for_window(w.target_min, w.target_max)*100:.1f}%" if w.feasible else "n/a (window infeasible)"
        print(f"{mu:>6.2f} {req_mu.overall_required:>11,.1f} {str(w.feasible):>17} {em_str:>12}")

    _rule("SENSITIVITY E: proof-load installation fraction eta_proof (0.60/0.70/0.80), 10 mm")
    header = f"{'eta_proof':>10} {'target_max':>11} {'epsilon_max':>12}"
    print(header)
    print("-" * len(header))
    for eta in (0.60, 0.70, 0.80):
        w = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, eta, SCATTER_ALLOWANCE)
        em = epsilon_max_for_window(w.target_min, w.target_max)
        print(f"{eta:>10.2f} {w.target_max:>11,.1f} {em*100:>11.1f}%")

    _rule("SENSITIVITY F: preload-window ratio vs. torque K-uncertainty ratio (normalized comparison)")
    print(f"  10 mm preload-window ratio (F_max/F_min)      = {R:.4f}")
    print(f"  10 mm M6 torque K-uncertainty ratio (K_max/K_min) = {NUT_FACTOR_BASELINE.uncertainty_ratio:.4f}")
    print(f"  equivalent direct epsilon at K ratio (K_max/K_min - 1)/(K_max/K_min + 1) = {(NUT_FACTOR_BASELINE.uncertainty_ratio-1)/(NUT_FACTOR_BASELINE.uncertainty_ratio+1)*100:.1f}%")
    print(f"  10 mm epsilon_max (direct measurement)         = {eps_max*100:.1f}%")
    print(
        "  These are deterministic ratio comparisons, not probabilistic confidence bounds:\n"
        "  the M6 torque K-uncertainty ratio (1.667) exceeds the preload-window ratio (1.504),\n"
        "  which is why torque-only control fails; the sourced/illustrative direct-measurement\n"
        "  errors (5-10%) are all comfortably below epsilon_max (~20.1%), which is why direct\n"
        "  verification methods succeed where torque-only control does not."
    )

    # -----------------------------------------------------------------
    # Predeclared decision
    # -----------------------------------------------------------------
    _rule("PREDECLARED M7 DECISION")
    print(
        "  The M5-selected 10 mm candidate remains the preferred conceptual candidate if:\n"
        "  (1) its inherited direct preload window remains feasible; (2) at least one\n"
        "  realistically quantifiable direct-verification method has a robust preload-\n"
        "  control window; (3) maximum achieved preload remains below proof/yield\n"
        "  screening limits in service; (4) no prior mandatory M1-M5 criterion is violated."
    )
    if window.feasible and trade.any_feasible:
        print(
            "\n  RESULT: the 10 mm candidate's direct preload window is feasible, and at least\n"
            f"  one quantified direct-verification method ({[m.value for m in trade.feasible_methods]})\n"
            "  has a robust preload-control window with proof reserve intact in service. Direct\n"
            "  preload verification therefore makes the 10 mm candidate installation-robust,\n"
            "  where torque-only control (M6) did not."
        )
    else:
        print("\n  RESULT: the 10 mm candidate still lacks a robust installation method.")

    if eps_max_12 > eps_max:
        print(
            f"\n  Trade note (not a reselection): 12 mm's epsilon_max ({eps_max_12*100:.1f}%) is "
            f"materially larger than 10 mm's ({eps_max*100:.1f}%) -- 12 mm carries substantially\n"
            "  more installation tolerance/margin, but this project does not silently reselect\n"
            "  12 mm; that would require an explicit extension of the predeclared M5 hierarchy."
        )


if __name__ == "__main__":
    main()
