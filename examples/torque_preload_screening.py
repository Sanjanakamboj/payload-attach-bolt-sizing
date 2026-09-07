"""Torque-to-preload sizing, installation scatter, and installation-
robustness screening for the Milestone 5 selected 10 mm candidate
(Milestone 6).

The torque-preload relation is a reduced-order nut-factor model and is
not a production torque specification. Nut-factor variation is treated
deterministically here; this is not a statistical process-capability
analysis. Torque control does not directly measure achieved preload.

Run with:
    python examples/torque_preload_screening.py
"""

from payload_bolts import (
    BoltMaterial,
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
    required_preload,
    robust_torque_window,
)

# ---------------------------------------------------------------------------
# Same Milestone 1 group-load case as all earlier examples.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

# Milestone 2/3/4 illustrative baseline, unchanged.
BOLT_MATERIAL = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
STRENGTH_LIMITS = BoltStrengthLimits(name="Illustrative aerospace-grade alloy-steel fastener (proof/yield)", proof_strength=830e6, yield_strength=970e6)
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)
ETA_PROOF = 0.75
SCATTER_ALLOWANCE = 0.10
M3_PRELOAD_FACTOR = 1.20

# Milestone 5 selected conceptual candidate.
SELECTED_DIAMETER_MM = 10.0
SELECTED_SECTION = circular_unthreaded_bolt(SELECTED_DIAMETER_MM / 1000.0)

# ---------------------------------------------------------------------------
# Milestone 6 illustrative nut-factor baseline (see
# src/payload_bolts/torque_preload.py module docstring for the source
# audit behind these values).
# ---------------------------------------------------------------------------
NUT_FACTOR_BASELINE = NutFactorModel(k_min=0.15, k_nom=0.20, k_max=0.25)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_pct(x):
    return "n/a" if x is None else f"{x * 100.0:+.1f}%"


def _fmt(x, fmt="{:,.1f}"):
    return "n/a" if x is None else fmt.format(x)


def main() -> None:
    print("TORQUE-TO-PRELOAD SIZING AND INSTALLATION-ROBUSTNESS SCREENING (Milestone 6)")
    print("Consumes the Milestone 1-5 8-bolt / R=0.5 m illustrative case unchanged.")
    print(
        "\n\"The torque-preload relation is a reduced-order nut-factor model and is not a\n"
        "production torque specification.\"\n"
        "\"Nut-factor variation is treated deterministically here; this is not a\n"
        "statistical process-capability analysis.\"\n"
        "\"Torque control does not directly measure achieved preload.\""
    )

    # -----------------------------------------------------------------
    # Inherited design
    # -----------------------------------------------------------------
    m2 = assess_bolt_group_strength(GROUP_LOAD_RESULT, SELECTED_SECTION, BOLT_MATERIAL)
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)

    _rule("INHERITED DESIGN (Milestones 1-5, unchanged)")
    print(f"  selected conceptual bolt (M5)          = {SELECTED_DIAMETER_MM:.1f} mm")
    print(f"  M2 strength: governing bolt/mode/pass   = index {m2.governing_bolt_index} / {m2.governing_mode} / {m2.passed}")
    print(f"  M3 required preload (overall)           = {required.overall_required:,.1f} N/bolt (governing: {required.overall_governing_constraint})")
    print(f"  M4/M5 target preload window              = [{window.target_min:,.1f}, {window.target_max:,.1f}] N/bolt")
    print(f"  M4/M5 window width                        = {window.window_width:,.1f} N (feasible={window.feasible})")
    print(f"  proof/yield basis: S_p={STRENGTH_LIMITS.proof_strength/1e6:.0f} MPa, S_y={STRENGTH_LIMITS.yield_strength/1e6:.0f} MPa, F_proof={window.limits.proof_load:,.1f} N")

    # -----------------------------------------------------------------
    # Nut-factor model
    # -----------------------------------------------------------------
    _rule("NUT-FACTOR MODEL (illustrative baseline; see module docstring for source audit)")
    print("  equation: T = K * F * d   (inverse: F = T / (K * d))")
    print(f"  K_min = {NUT_FACTOR_BASELINE.k_min:.3f}")
    print(f"  K_nom = {NUT_FACTOR_BASELINE.k_nom:.3f}")
    print(f"  K_max = {NUT_FACTOR_BASELINE.k_max:.3f}")
    print(f"  K uncertainty ratio (K_max/K_min) = {NUT_FACTOR_BASELINE.uncertainty_ratio:.4f}")

    # -----------------------------------------------------------------
    # Nominal torque mapping
    # -----------------------------------------------------------------
    ntw = nominal_torque_window(window, NUT_FACTOR_BASELINE.k_nom, SELECTED_SECTION.nominal_diameter)
    _rule("NOMINAL TORQUE MAPPING (at K_nom only -- NOT yet robust to K uncertainty)")
    print(f"  T_min,nom = K_nom * d * F_target,min = {ntw.torque_min_nom:.3f} N*m")
    print(f"  T_max,nom = K_nom * d * F_target,max = {ntw.torque_max_nom:.3f} N*m")

    # -----------------------------------------------------------------
    # Robust torque window (central M6 result)
    # -----------------------------------------------------------------
    result = assess_torque_installation(
        GROUP_LOAD_RESULT, SELECTED_SECTION, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, SCATTER_ALLOWANCE, NUT_FACTOR_BASELINE,
    )
    rtw = result.robust_window
    _rule("ROBUST TORQUE WINDOW (central Milestone 6 result)")
    print(f"  T_robust,min = K_max*d*F_target,min = {rtw.torque_robust_min:.3f} N*m")
    print(f"  T_robust,max = K_min*d*F_target,max = {rtw.torque_robust_max:.3f} N*m")
    print(f"  width (T_robust,max - T_robust,min)  = {rtw.width:.3f} N*m")
    print(f"  STATUS                                = {result.status.value}")
    print(f"  preload-window ratio (F_max/F_min)    = {rtw.preload_window_ratio:.4f}")
    print(f"  K uncertainty ratio (K_max/K_min)     = {rtw.k_uncertainty_ratio:.4f}")
    print(
        "  feasibility identity: K_max/K_min <= F_target,max/F_target,min  ->  "
        f"{rtw.k_uncertainty_ratio:.4f} <= {rtw.preload_window_ratio:.4f}  ->  {rtw.feasible}"
    )
    print(f"  {result.message}")

    # -----------------------------------------------------------------
    # Nominal torque target
    # -----------------------------------------------------------------
    _rule("NOMINAL TORQUE TARGET")
    if result.nominal_torque is None:
        print("  NO ROBUST TORQUE TARGET EXISTS -- none is proposed under the baseline K range.")
        print("  (See sensitivity A below for a narrower K range that recovers feasibility.)")
    else:
        print(f"  selected midpoint torque T_nom          = {result.nominal_torque:.3f} N*m")
        print(f"  achieved preload at K_min (lowest friction, highest F) = {result.achieved_preload_at_k_min:,.1f} N")
        print(f"  achieved preload at K_nom                              = {result.achieved_preload_at_k_nom:,.1f} N")
        print(f"  achieved preload at K_max (highest friction, lowest F) = {result.achieved_preload_at_k_max:,.1f} N")
        print(f"  all three within target window?                       = {result.all_within_target_window}")
        print(f"  max in-service bolt force (at K_min bound)             = {result.max_in_service_bolt_force:,.1f} N (bolt {result.max_in_service_bolt_index})")
        print(f"  proof reserve at that bound                            = {_fmt_pct(result.proof_reserve)}")

    # -----------------------------------------------------------------
    # Historical M3 preload back-calculation
    # -----------------------------------------------------------------
    _rule("HISTORICAL M3 SELECTED-PRELOAD TORQUE BACK-CALCULATION (diagnostic only)")
    m3_selected = required.overall_required * M3_PRELOAD_FACTOR
    back = back_calculate_torque(m3_selected, SELECTED_SECTION.nominal_diameter, NUT_FACTOR_BASELINE)
    print(f"  M3 selected preload = {m3_selected:,.1f} N/bolt")
    print(f"  torque at K_min = {back.torque_at_k_min:.3f} N*m")
    print(f"  torque at K_nom = {back.torque_at_k_nom:.3f} N*m")
    print(f"  torque at K_max = {back.torque_at_k_max:.3f} N*m")
    print("  (diagnostic only -- NOT a production torque specification)")

    # -----------------------------------------------------------------
    # Sensitivities
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: nut-factor range width")
    header = f"{'family':>10} {'K_min':>6} {'K_max':>6} {'ratio':>7} {'robust?':>8} {'width_Nm':>9} {'T_nom_Nm':>9}"
    print(header)
    print("-" * len(header))
    for label, kmin, kmax in (("narrow", 0.18, 0.22), ("nominal", 0.15, 0.25), ("wide", 0.12, 0.28)):
        nf = NutFactorModel(k_min=kmin, k_nom=(kmin + kmax) / 2.0, k_max=kmax)
        r = assess_torque_installation(GROUP_LOAD_RESULT, SELECTED_SECTION, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, nf)
        t_nom_str = f"{r.nominal_torque:.2f}" if r.nominal_torque is not None else "n/a"
        print(f"{label:>10} {kmin:>6.2f} {kmax:>6.2f} {nf.uncertainty_ratio:>7.3f} {str(r.robust_window.feasible):>8} {r.robust_window.width:>9.2f} {t_nom_str:>9}")

    _rule("SENSITIVITY B: preload scatter/loss allowance delta_F")
    header = f"{'delta_F':>8} {'target_min':>11} {'robust?':>8} {'width_Nm':>9}"
    print(header)
    print("-" * len(header))
    for delta in (0.05, 0.10, 0.20):
        w = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, delta)
        rtw_d = robust_torque_window(w, NUT_FACTOR_BASELINE, SELECTED_SECTION.nominal_diameter)
        print(f"{delta:>8.2f} {w.target_min:>11,.1f} {str(rtw_d.feasible):>8} {rtw_d.width:>9.2f}")

    _rule("SENSITIVITY C: friction coefficient mu (carried forward from M3)")
    header = f"{'mu':>6} {'F_required':>11} {'robust?':>8} {'width_Nm':>9}"
    print(header)
    print("-" * len(header))
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        req_mu = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FrictionModel(friction_coefficient=mu))
        w = installation_preload_window(req_mu.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        rtw_mu = robust_torque_window(w, NUT_FACTOR_BASELINE, SELECTED_SECTION.nominal_diameter)
        print(f"{mu:>6.2f} {req_mu.overall_required:>11,.1f} {str(rtw_mu.feasible):>8} {rtw_mu.width:>9.2f}")

    _rule("SENSITIVITY D: proof-load installation fraction eta_proof (carried forward from M4)")
    header = f"{'eta_proof':>10} {'target_max':>11} {'robust?':>8} {'width_Nm':>9}"
    print(header)
    print("-" * len(header))
    for eta in (0.60, 0.70, 0.80):
        w = installation_preload_window(required.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, eta, SCATTER_ALLOWANCE)
        rtw_e = robust_torque_window(w, NUT_FACTOR_BASELINE, SELECTED_SECTION.nominal_diameter)
        print(f"{eta:>10.2f} {w.target_max:>11,.1f} {str(rtw_e.feasible):>8} {rtw_e.width:>9.2f}")

    _rule("SENSITIVITY E: bolt size (8/10/12 mm, established M2/M4/M5 properties)")
    header = f"{'d_mm':>5} {'direct window feasible?':>24} {'robust torque feasible?':>24} {'width_Nm':>9}"
    print(header)
    print("-" * len(header))
    for d_mm in (8.0, 10.0, 12.0):
        section = circular_unthreaded_bolt(d_mm / 1000.0)
        w = installation_preload_window(required.overall_required, section, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        rtw_d = robust_torque_window(w, NUT_FACTOR_BASELINE, section.nominal_diameter)
        print(f"{d_mm:>5.1f} {str(w.feasible):>24} {str(rtw_d.feasible):>24} {rtw_d.width:>9.2f}")

    _rule("SENSITIVITY F: nut-factor nominal value K_nom (fixed +/-0.025 half-width around K_nom)")
    header = f"{'K_nom':>7} {'K_min':>6} {'K_max':>6} {'T_min,nom_Nm':>13} {'T_max,nom_Nm':>13}"
    print(header)
    print("-" * len(header))
    for k_nom in (0.15, 0.18, 0.20, 0.22, 0.25):
        half = 0.025
        ntw_k = nominal_torque_window(window, k_nom, SELECTED_SECTION.nominal_diameter)
        print(f"{k_nom:>7.3f} {k_nom-half:>6.3f} {k_nom+half:>6.3f} {ntw_k.torque_min_nom:>13.2f} {ntw_k.torque_max_nom:>13.2f}")

    # -----------------------------------------------------------------
    # Predeclared decision
    # -----------------------------------------------------------------
    _rule("PREDECLARED M6 DECISION")
    print(
        "  The M5 selected 10 mm candidate remains installation-robust under torque\n"
        "  control only if: (1) the M5 direct preload window is feasible; (2) a robust\n"
        "  torque window exists for the baseline K range; (3) the midpoint torque keeps\n"
        "  achieved preload within bounds for every K in that range; (4) the maximum\n"
        "  achieved preload does not violate proof/yield in service."
    )
    if window.feasible and result.status == TorqueInstallationStatus.FEASIBLE:
        print("\n  RESULT: the 10 mm candidate REMAINS ACCEPTABLE under Milestone 6 torque uncertainty.")
    else:
        print(
            "\n  RESULT: the 10 mm candidate's DIRECT preload window (M4/M5) is feasible, but it\n"
            "  does NOT have a ROBUST TORQUE WINDOW at the illustrative baseline nut-factor\n"
            "  range K=[0.15, 0.25]. Torque-only control cannot be relied on by itself here.\n"
            "  Sensitivity A shows this is resolved by EITHER tighter friction control\n"
            "  (narrower K range, e.g. [0.18, 0.22]) OR direct preload measurement, OR\n"
            "  upsizing to 12 mm (robust even at the baseline K range) -- see README\n"
            "  'Final project synthesis' for the overall conclusion."
        )


if __name__ == "__main__":
    main()
