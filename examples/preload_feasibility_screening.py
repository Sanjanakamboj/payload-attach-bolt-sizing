"""Preload feasibility, proof/yield screening, and installation window
(Milestone 4).

Builds on the exact Milestone 1 8-bolt / R=0.5 m load case, the
Milestone 2 selected 8 mm illustrative bolt candidate, and the
Milestone 3 illustrative joint stiffness/friction/required-preload
results -- all reused unchanged.

This preload window is a deterministic screening construct, not a
torque specification or statistical installation guarantee.
Proof/yield properties are illustrative unless explicitly tied to a
sourced fastener specification.

Run with:
    python examples/preload_feasibility_screening.py
"""

from payload_bolts import (
    BoltMaterial,
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadState,
    assess_bolt_group_strength,
    assess_preload_feasibility,
    circular_pattern,
    circular_unthreaded_bolt,
    classify_selected_preload,
    distribute_loads,
    installation_preload_window,
    required_preload,
)

# ---------------------------------------------------------------------------
# Same Milestone 1 group-load case as the earlier examples.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
pattern = circular_pattern(N_BOLTS, RADIUS)

load = InterfaceLoad(
    Fx=20_000.0,
    Fy=-10_000.0,
    Fz=80_000.0,
    Mx=25_000.0,
    My=-15_000.0,
    Mz=12_000.0,
)

group_load_result = distribute_loads(pattern, load)

# Milestone 2 selected bolt (governing case reported for continuity).
SELECTED_DIAMETER_MM = 8.0
SELECTED_SECTION = circular_unthreaded_bolt(SELECTED_DIAMETER_MM / 1000.0)
M2_MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6
)
M2_STRENGTH = assess_bolt_group_strength(group_load_result, SELECTED_SECTION, M2_MATERIAL)

# Milestone 3 illustrative joint stiffness / friction (identical to
# examples/preloaded_joint_screening.py).
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)
REQUIRED = required_preload(group_load_result, STIFFNESS, FRICTION)
M3_PRELOAD_FACTOR = 1.20
M3_SELECTED_PRELOAD = REQUIRED.overall_required * M3_PRELOAD_FACTOR

# ---------------------------------------------------------------------------
# Milestone 4 illustrative proof/yield strength limits (see
# src/payload_bolts/preload_limits.py module docstring for the source
# audit behind these baseline values). DISTINCT from M2_MATERIAL above --
# these are proof/yield strengths, not a working-stress allowable.
# ---------------------------------------------------------------------------
STRENGTH_LIMITS = BoltStrengthLimits(
    name="Illustrative aerospace-grade alloy-steel fastener (proof/yield)",
    proof_strength=830e6,  # Pa, illustrative
    yield_strength=970e6,  # Pa, illustrative
)
ETA_PROOF = 0.75  # illustrative proof-load installation fraction
SCATTER_ALLOWANCE = 0.10  # illustrative deterministic preload scatter/loss allowance (delta_F)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_pct(x):
    return "n/a" if x is None else f"{x * 100.0:+.1f}%"


def main() -> None:
    print("PRELOAD FEASIBILITY, PROOF/YIELD, AND INSTALLATION WINDOW SCREENING (Milestone 4)")
    print("Consumes the Milestone 1-3 8-bolt / R=0.5 m illustrative case unchanged.")
    print(
        "\nThis preload window is a deterministic screening construct, not a torque\n"
        "specification or statistical installation guarantee.\n"
        "Proof/yield properties are illustrative unless explicitly tied to a sourced\n"
        "fastener specification."
    )

    # -----------------------------------------------------------------
    # Section A -- inherited design
    # -----------------------------------------------------------------
    _rule("SECTION A: INHERITED DESIGN (Milestones 1-3, unchanged)")
    print(f"  bolt size (Milestone 2 selected)      = {SELECTED_DIAMETER_MM:.1f} mm")
    print(f"  M2 governing bolt / mode               = index {M2_STRENGTH.governing_bolt_index} / {M2_STRENGTH.governing_mode}")
    print(f"  M3 separation-required preload         = {REQUIRED.separation_required:>12,.1f} N/bolt (bolt {REQUIRED.separation_governing_bolt})")
    print(f"  M3 slip-required preload               = {REQUIRED.slip_required:>12,.1f} N/bolt (bolt {REQUIRED.slip_governing_bolt})")
    print(f"  M3 overall required preload            = {REQUIRED.overall_required:>12,.1f} N/bolt (governing: {REQUIRED.overall_governing_constraint})")
    print(f"  M3 selected preload (factor {M3_PRELOAD_FACTOR:.2f})       = {M3_SELECTED_PRELOAD:>12,.1f} N/bolt")

    # -----------------------------------------------------------------
    # Section B -- proof/yield basis
    # -----------------------------------------------------------------
    window_8mm = installation_preload_window(
        REQUIRED.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE
    )
    _rule("SECTION B: PROOF/YIELD BASIS (illustrative, see module docstring for source audit)")
    print(f"  A_t (reused exactly from Milestone 2)  = {SELECTED_SECTION.tensile_area * 1e6:,.3f} mm^2")
    print(f"  S_p (proof strength)                   = {STRENGTH_LIMITS.proof_strength / 1e6:,.0f} MPa")
    print(f"  S_y (yield strength)                   = {STRENGTH_LIMITS.yield_strength / 1e6:,.0f} MPa")
    print(f"  F_proof = A_t * S_p                    = {window_8mm.limits.proof_load:>12,.1f} N")
    print(f"  F_yield = A_t * S_y                    = {window_8mm.limits.yield_load:>12,.1f} N")
    print(f"  eta_proof (installation fraction)      = {ETA_PROOF:.2f}")
    print(f"  F_target,max = eta_proof * F_proof      = {window_8mm.target_max:>12,.1f} N")

    # -----------------------------------------------------------------
    # Section C -- installation window
    # -----------------------------------------------------------------
    _rule("SECTION C: INSTALLATION WINDOW (8 mm)")
    print(f"  scatter/loss allowance delta_F         = {SCATTER_ALLOWANCE:.2f}")
    print(f"  F_target,min = F_required/(1-delta_F)   = {window_8mm.target_min:>12,.1f} N")
    print(f"  F_target,max                            = {window_8mm.target_max:>12,.1f} N")
    print(f"  window width  (target_max - target_min) = {window_8mm.window_width:>12,.1f} N")
    print(f"  normalized window width                 = {_fmt_pct(window_8mm.window_width_normalized)}")
    print(f"  FEASIBLE?                               = {window_8mm.feasible}")
    if not window_8mm.feasible:
        print(
            "  >>> NO FEASIBLE INSTALLATION WINDOW at 8 mm under these illustrative\n"
            "      assumptions: the scatter-adjusted minimum target preload exceeds the\n"
            "      proof-based installation ceiling. This is reported honestly, not forced."
        )

    # -----------------------------------------------------------------
    # Section D -- selected preload vs. window
    # -----------------------------------------------------------------
    result_8mm = assess_preload_feasibility(
        group_load_result, SELECTED_SECTION, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, SCATTER_ALLOWANCE, M3_SELECTED_PRELOAD,
    )
    _rule("SECTION D: M3 SELECTED PRELOAD vs. M4 WINDOW (8 mm)")
    print(f"  M3 selected preload                    = {M3_SELECTED_PRELOAD:>12,.1f} N/bolt")
    print(f"  window-only status                      = {result_8mm.window_status.value}")
    print(f"  GOVERNING STATUS                        = {result_8mm.status.value}")

    # -----------------------------------------------------------------
    # Section E -- in-service bolt screening
    # -----------------------------------------------------------------
    _rule("SECTION E: IN-SERVICE BOLT SCREENING (8 mm, at M3 selected preload)")
    print(f"  governing bolt (max total tension)     = index {result_8mm.max_in_service_bolt_index}")
    print(f"  in-service closed-joint model valid?   = {result_8mm.in_service_model_valid}")
    print(f"  maximum in-service bolt force F_bolt    = {result_8mm.max_in_service_bolt_force:>12,.1f} N")
    print(f"  proof reserve  (F_proof/F_bolt - 1)     = {_fmt_pct(result_8mm.proof_reserve)}")
    print(f"  yield reserve  (F_yield/F_bolt - 1)     = {_fmt_pct(result_8mm.yield_reserve)}")
    print(f"  in-service exceeds proof/yield?         = {result_8mm.in_service_exceeds_proof}")

    # -----------------------------------------------------------------
    # Section F -- sensitivities
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: preload scatter/loss allowance delta_F (8 mm)")
    header = f"{'delta_F':>8} {'target_min':>12} {'window width':>13} {'feasible':>9}"
    print(header)
    print("-" * len(header))
    for delta in (0.05, 0.10, 0.20):
        w = installation_preload_window(REQUIRED.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, delta)
        print(f"{delta:>8.2f} {w.target_min:>12,.1f} {w.window_width:>13,.1f} {str(w.feasible):>9}")

    _rule("SENSITIVITY B: proof strength S_p (8 mm, eta_proof/delta_F fixed)")
    header = f"{'S_p (MPa)':>10} {'F_proof':>11} {'target_max':>12} {'window width':>13} {'feasible':>9}"
    print(header)
    print("-" * len(header))
    for sp_mpa in (700.0, 830.0, 900.0, 1000.0):
        limits = BoltStrengthLimits(name="sens", proof_strength=sp_mpa * 1e6)
        w = installation_preload_window(REQUIRED.overall_required, SELECTED_SECTION, limits, ETA_PROOF, SCATTER_ALLOWANCE)
        print(f"{sp_mpa:>10.0f} {w.limits.proof_load:>11,.1f} {w.target_max:>12,.1f} {w.window_width:>13,.1f} {str(w.feasible):>9}")

    _rule("SENSITIVITY C: proof-load installation fraction eta_proof (8 mm)")
    header = f"{'eta_proof':>10} {'target_max':>12} {'window width':>13} {'feasible':>9}"
    print(header)
    print("-" * len(header))
    for eta in (0.60, 0.70, 0.80):
        w = installation_preload_window(REQUIRED.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, eta, SCATTER_ALLOWANCE)
        print(f"{eta:>10.2f} {w.target_max:>12,.1f} {w.window_width:>13,.1f} {str(w.feasible):>9}")

    _rule("SENSITIVITY D: friction coefficient mu, propagated through M3 slip physics (8 mm)")
    header = f"{'mu':>6} {'M3 F_required':>14} {'target_min':>12} {'target_max':>12} {'feasible':>9}"
    print(header)
    print("-" * len(header))
    limits_result = installation_preload_window(REQUIRED.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE).limits
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        req_mu = required_preload(group_load_result, STIFFNESS, FrictionModel(friction_coefficient=mu))
        w = installation_preload_window(req_mu.overall_required, SELECTED_SECTION, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        print(f"{mu:>6.2f} {req_mu.overall_required:>14,.1f} {w.target_min:>12,.1f} {w.target_max:>12,.1f} {str(w.feasible):>9}")
    del limits_result

    _rule("SENSITIVITY E: bolt size (Milestone 2 candidates, eta_proof/delta_F/mu fixed)")
    header = (
        f"{'d_mm':>6} {'A_t_mm2':>9} {'F_proof':>11} {'target_max':>12} {'target_min':>12} "
        f"{'window width':>13} {'M2 pass?':>9} {'feasible':>9}"
    )
    print(header)
    print("-" * len(header))
    for d_mm in (5.0, 6.0, 8.0, 10.0, 12.0):
        section = circular_unthreaded_bolt(d_mm / 1000.0)
        m2_result = assess_bolt_group_strength(group_load_result, section, M2_MATERIAL)
        w = installation_preload_window(REQUIRED.overall_required, section, STRENGTH_LIMITS, ETA_PROOF, SCATTER_ALLOWANCE)
        print(
            f"{d_mm:>6.1f} {section.tensile_area * 1e6:>9.2f} {w.limits.proof_load:>11,.1f} "
            f"{w.target_max:>12,.1f} {w.target_min:>12,.1f} {w.window_width:>13,.1f} "
            f"{str(m2_result.passed):>9} {str(w.feasible):>9}"
        )
    print(
        "\n  NOTE: the smallest Milestone 2 strength-passing candidate (8 mm) does NOT\n"
        "  have a feasible Milestone 4 preload-installation window under these\n"
        "  illustrative proof/scatter assumptions -- 10 mm and 12 mm do. This module\n"
        "  reports that honestly; it does NOT silently re-select a larger bolt. A\n"
        "  final bolt-size decision combining M2 strength and M4 preload feasibility\n"
        "  is left to a future milestone / an explicit engineering decision."
    )

    # -----------------------------------------------------------------
    # Interpretation
    # -----------------------------------------------------------------
    _rule("ENGINEERING INTERPRETATION")
    print(
        "  Milestone 3 established how much preload is analytically REQUIRED to\n"
        "  prevent separation/slip. Milestone 4 asks whether that requirement is\n"
        "  structurally FEASIBLE to install: the minimum installable target must\n"
        "  clear the M3 requirement even after a deterministic preload scatter/loss\n"
        "  allowance, and the maximum installable target is capped by a fraction of\n"
        "  the bolt's proof load. A feasible window exists only if the scatter-\n"
        "  adjusted minimum stays at or below the proof-based ceiling; this module\n"
        "  never forces or clips a negative window to appear feasible. Separately,\n"
        "  the maximum IN-SERVICE bolt tension (preload plus the bolt's C-fraction\n"
        "  share of external tensile load, valid only while that bolt's joint\n"
        "  remains closed) is screened against proof/yield load as an independent\n"
        "  check -- installation feasibility and in-service strength are two\n"
        "  distinct questions and are never combined into a single number. No\n"
        "  torque is inferred anywhere in this module: required/target preloads are\n"
        "  analytical force quantities only, not a torque specification. Real\n"
        "  preload/torque design additionally requires a torque-tension (nut-\n"
        "  factor) model, thread/under-head friction characterization, preload\n"
        "  relaxation and thermal effects, fatigue, and proof testing -- all\n"
        "  deferred to later milestones."
    )


if __name__ == "__main__":
    main()
