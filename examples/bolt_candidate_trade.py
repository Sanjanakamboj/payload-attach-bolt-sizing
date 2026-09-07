"""Bolt-size trade, local joint failure-mode screening, and preliminary
hardware selection (Milestone 5).

Resolves the engineering question exposed by Milestone 4: does the
Milestone 2 minimum strength-passing 8 mm bolt remain a defensible
conceptual choice once preload feasibility (M4) and local-joint
screening (bearing, edge distance, spacing -- new in M5) are added?

The selected bolt is the smallest candidate passing this reduced-order
screening set; it is not a flight-qualified or globally optimized
fastener selection. Local bearing, spacing, and thread checks are
conceptual screening models and do not replace detailed joint analysis.

Run with:
    python examples/bolt_candidate_trade.py
"""

from payload_bolts import (
    BoltMaterial,
    BoltStrengthLimits,
    FrictionModel,
    InterfaceLoad,
    JointGeometry,
    JointStiffness,
    PlateMaterial,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    evaluate_candidate_trade,
    required_preload,
    select_bolt_candidate,
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
PRELOAD_FACTOR = 1.20

# ---------------------------------------------------------------------------
# Milestone 5 illustrative local-joint assumptions (see
# src/payload_bolts/joint_local_checks.py module docstring for the
# source audit behind these baseline values).
# ---------------------------------------------------------------------------
PLATE = PlateMaterial(name="Illustrative plate/lug bearing material", bearing_allowable=400e6)  # Pa
GEOMETRY = JointGeometry(
    plate_thickness=0.008,  # m, illustrative conceptual flange thickness
    plate_center=(0.0, 0.0),  # concentric with the M1 bolt-circle center
    plate_outer_radius=RADIUS + 0.05,  # m, illustrative 50 mm edge margin
    edge_distance_min_ratio=1.5,  # (e/d)_min, source: mechanicalc.com Air-Force-Method bearing/shear-out transition
    spacing_min_ratio=3.0,  # (s/d)_min, source: AISC 360-22 general structural spacing convention
)

CANDIDATE_DIAMETERS_MM = [6.0, 8.0, 10.0, 12.0]
CANDIDATES = [(d / 1000.0, circular_unthreaded_bolt(d / 1000.0)) for d in CANDIDATE_DIAMETERS_MM]


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_margin(m):
    return "n/a" if m is None else f"{m:+.3f}"


def main() -> None:
    print("BOLT-SIZE TRADE AND LOCAL JOINT SCREENING STUDY (Milestone 5)")
    print("Consumes the Milestone 1-4 8-bolt / R=0.5 m illustrative case unchanged.")

    # -----------------------------------------------------------------
    # Inherited design state
    # -----------------------------------------------------------------
    _rule("INHERITED DESIGN STATE (Milestones 1-4, unchanged)")
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    max_shear = GROUP_LOAD_RESULT.max_shear_bolt()
    print(f"  M1 max tensile bolt   : index {max_tensile.index}, T = {max_tensile.axial_total:,.1f} N")
    print(f"  M1 max shear bolt     : index {max_shear.index}, V = {max_shear.shear_resultant:,.1f} N")
    section_8mm = circular_unthreaded_bolt(0.008)
    trade_8mm_preview = evaluate_candidate_trade(
        GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )
    print(f"  M2 selected/minimum strength-passing bolt = 8.0 mm (governing bolt {trade_8mm_preview.strength.governing_bolt_index} / {trade_8mm_preview.strength.governing_mode})")
    required = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    print(f"  M3 required preload (overall)             = {required.overall_required:,.1f} N/bolt (governing: {required.overall_governing_constraint}, bolt {required.overall_governing_bolt})")
    print(f"  M4 preload feasibility @ 8 mm              = {trade_8mm_preview.preload_feasibility.status.value} (window width {trade_8mm_preview.preload_feasibility.window.window_width:,.1f} N)")

    # -----------------------------------------------------------------
    # M5 local-joint assumptions
    # -----------------------------------------------------------------
    _rule("M5 LOCAL-JOINT ASSUMPTIONS (illustrative; see module docstring for source audit)")
    print(f"  plate thickness t                  = {GEOMETRY.plate_thickness * 1000:.1f} mm")
    print("  hole convention                    = idealized d_hole == candidate nominal diameter (no clearance)")
    print(f"  bearing allowable                  = {PLATE.bearing_allowable / 1e6:,.0f} MPa (illustrative)")
    print(f"  plate outer radius (illustrative)  = {GEOMETRY.plate_outer_radius * 1000:.1f} mm (bolt circle {RADIUS*1000:.1f} mm + 50 mm margin)")
    print(f"  edge-distance criterion (e/d)_min  = {GEOMETRY.edge_distance_min_ratio:.2f}  (mechanicalc.com Air-Force-Method bearing/shear-out transition)")
    print(f"  spacing criterion (s/d)_min        = {GEOMETRY.spacing_min_ratio:.2f}  (AISC 360-22 general structural convention, not aerospace-specific)")
    print("  thread stripping                   = NOT MODELED (see module docstring; never gates selection)")

    # -----------------------------------------------------------------
    # Predeclared selection rule
    # -----------------------------------------------------------------
    _rule("PREDECLARED SELECTION RULE (declared before evaluating candidates)")
    print(
        "  A candidate is ADMISSIBLE only if ALL of:\n"
        "    1. Milestone 2 strength passes;\n"
        "    2. the Milestone 4 installation-preload window is feasible AND the\n"
        "       Milestone 3 selected preload classifies as FEASIBLE against it;\n"
        "    3. bearing margin of safety >= 0;\n"
        "    4. the edge-distance screen passes;\n"
        "    5. the spacing screen passes.\n"
        "  (Thread stripping is NOT modeled and is never a gating criterion.)\n"
        "  Among ADMISSIBLE candidates, SELECT the smallest nominal diameter.\n"
        "  This is a conceptual minimum-size rule, not a claim of global optimality."
    )

    # -----------------------------------------------------------------
    # Candidate comparison
    # -----------------------------------------------------------------
    selection = select_bolt_candidate(
        GROUP_LOAD_RESULT, CANDIDATES, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION,
        ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, PLATE, GEOMETRY,
    )

    _rule("CANDIDATE COMPARISON")
    header = (
        f"{'d_mm':>5} {'A_t_mm2':>8} {'M2':>5} {'M2 gov':>12} {'M4 status':>22} {'M4 width_N':>11} "
        f"{'M4 Fmax_N':>10} {'bearMS':>8} {'e/d':>6} {'edgeOK':>7} {'s/d':>6} {'spcOK':>6} {'admiss':>7} {'select':>7}"
    )
    print(header)
    print("-" * len(header))
    for c in selection.candidates:
        d_mm = c.bolt_section.nominal_diameter * 1000.0
        at_mm2 = c.bolt_section.tensile_area * 1e6
        is_selected = "<<< SEL" if selection.selected_index is not None and selection.candidates[selection.selected_index] is c else ""
        print(
            f"{d_mm:>5.1f} {at_mm2:>8.2f} {str(c.strength.passed):>5} {c.strength.governing_mode!s:>12} "
            f"{c.preload_feasibility.status.value:>22} {c.preload_feasibility.window.window_width:>11,.0f} "
            f"{c.preload_feasibility.max_in_service_bolt_force:>10,.0f} {_fmt_margin(c.bearing.governing_margin):>8} "
            f"{c.edge_distance.min_ratio:>6.2f} {str(c.edge_distance.passed):>7} {c.spacing.min_ratio:>6.2f} "
            f"{str(c.spacing.passed):>6} {str(c.admissible):>7} {is_selected:>7}"
        )

    # -----------------------------------------------------------------
    # Result
    # -----------------------------------------------------------------
    _rule("RESULT")
    if selection.no_feasible_candidate:
        print("  NO FEASIBLE CANDIDATE among the evaluated diameters.")
    else:
        sel = selection.selected()
        d_mm = sel.bolt_section.nominal_diameter * 1000.0
        print(f"  selected conceptual candidate  : {d_mm:.1f} mm")
        print("  why smaller candidates fail:")
        for c in selection.candidates:
            d_c = c.bolt_section.nominal_diameter * 1000.0
            if d_c < d_mm:
                reasons = ", ".join(c.admissibility_reasons) if c.admissibility_reasons else "(none -- unexpected)"
                print(f"    {d_c:>5.1f} mm: {reasons}")
        print(f"  governing M2 mode/margin        : {sel.strength.governing_mode} / {_fmt_margin(sel.strength.governing_margin)}")
        print(f"  M4 window width                 : {sel.preload_feasibility.window.window_width:,.1f} N")
        print(f"  M4 proof reserve                : {_fmt_margin(sel.preload_feasibility.proof_reserve)}")
        print(f"  bearing governing margin        : {_fmt_margin(sel.bearing.governing_margin)}")
        print(f"  edge-distance min ratio         : {sel.edge_distance.min_ratio:.2f} (criterion {sel.edge_distance.criterion:.2f})")
        print(f"  spacing min ratio               : {sel.spacing.min_ratio:.2f} (criterion {sel.spacing.criterion:.2f}, pair {sel.spacing.governing_pair})")

    print(
        "\n  \"The selected bolt is the smallest candidate passing this reduced-order\n"
        "  screening set; it is not a flight-qualified or globally optimized fastener\n"
        "  selection.\"\n"
        "  \"Local bearing, spacing, and thread checks are conceptual screening models\n"
        "  and do not replace detailed joint analysis.\""
    )

    # -----------------------------------------------------------------
    # Sensitivity
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: plate thickness (0.75x / 1.0x / 1.25x baseline, 8 mm bearing only)")
    header = f"{'factor':>7} {'t_mm':>6} {'bearing MS':>11} {'admissible?':>12}"
    print(header)
    print("-" * len(header))
    for factor in (0.75, 1.0, 1.25):
        t = GEOMETRY.plate_thickness * factor
        geom = JointGeometry(plate_thickness=t, plate_center=GEOMETRY.plate_center, plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=GEOMETRY.edge_distance_min_ratio, spacing_min_ratio=GEOMETRY.spacing_min_ratio)
        trade = evaluate_candidate_trade(GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, PLATE, geom)
        print(f"{factor:>7.2f} {t*1000:>6.2f} {_fmt_margin(trade.bearing.governing_margin):>11} {str(trade.admissible):>12}")

    _rule("SENSITIVITY B: bearing allowable (-20% / nominal / +20%, 8 mm)")
    header = f"{'factor':>7} {'allow_MPa':>10} {'bearing MS':>11} {'admissible?':>12}"
    print(header)
    print("-" * len(header))
    for factor in (0.8, 1.0, 1.2):
        plate = PlateMaterial(name="sens", bearing_allowable=PLATE.bearing_allowable * factor)
        trade = evaluate_candidate_trade(GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, plate, GEOMETRY)
        print(f"{factor:>7.2f} {plate.bearing_allowable/1e6:>10.1f} {_fmt_margin(trade.bearing.governing_margin):>11} {str(trade.admissible):>12}")

    _rule("SENSITIVITY C: edge-distance criterion (e/d)_min (screened range, 8 mm)")
    header = f"{'(e/d)_min':>10} {'actual e/d':>11} {'edge pass?':>11}"
    print(header)
    print("-" * len(header))
    for crit in (1.5, 2.0, 2.5):
        geom = JointGeometry(plate_thickness=GEOMETRY.plate_thickness, plate_center=GEOMETRY.plate_center, plate_outer_radius=GEOMETRY.plate_outer_radius, edge_distance_min_ratio=crit, spacing_min_ratio=GEOMETRY.spacing_min_ratio)
        trade = evaluate_candidate_trade(GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, PLATE, geom)
        print(f"{crit:>10.2f} {trade.edge_distance.min_ratio:>11.2f} {str(trade.edge_distance.passed):>11}")

    _rule("SENSITIVITY D: thread engagement length -- NOT APPLICABLE (thread stripping not modeled)")

    _rule("SENSITIVITY E: preload scatter allowance delta_F (5% / 10% / 20%, 8 mm)")
    print("(NOTE: the M3 selected preload here stays FIXED at the baseline delta_F=0.10-derived")
    print(" value x factor 1.20 -- delta_F only moves the window's lower target, not the fixed")
    print(" selection -- so a feasible window does not by itself guarantee an admissible selection.)")
    header = f"{'delta_F':>8} {'M4 window width':>16} {'window status':>22} {'admissible?':>12}"
    print(header)
    print("-" * len(header))
    for delta in (0.05, 0.10, 0.20):
        trade = evaluate_candidate_trade(GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, FRICTION, ETA_PROOF, delta, PRELOAD_FACTOR, PLATE, GEOMETRY)
        print(f"{delta:>8.2f} {trade.preload_feasibility.window.window_width:>16,.1f} {trade.preload_feasibility.status.value:>22} {str(trade.admissible):>12}")

    _rule("SENSITIVITY F: friction coefficient mu (carried forward from M3, 8 mm)")
    print("(NOTE: unlike delta_F above, sweeping mu here re-derives BOTH the M3 required preload")
    print(" AND the selected preload (factor 1.20 applied fresh), so admissibility can flip to True.)")
    header = f"{'mu':>6} {'M4 window width':>16} {'window status':>22} {'admissible?':>12}"
    print(header)
    print("-" * len(header))
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        friction = FrictionModel(friction_coefficient=mu)
        trade = evaluate_candidate_trade(GROUP_LOAD_RESULT, 0.008, section_8mm, BOLT_MATERIAL, STRENGTH_LIMITS, STIFFNESS, friction, ETA_PROOF, SCATTER_ALLOWANCE, PRELOAD_FACTOR, PLATE, GEOMETRY)
        print(f"{mu:>6.2f} {trade.preload_feasibility.window.window_width:>16,.1f} {trade.preload_feasibility.status.value:>22} {str(trade.admissible):>12}")

    print(
        "\n  Across sensitivities A-C, bearing and edge-distance margins remain comfortably\n"
        "  positive throughout the tested ranges at this illustrative plate geometry --\n"
        "  they never become the governing reason the 8 mm bolt is rejected. Sensitivity E\n"
        "  shows a subtlety: loosening delta_F alone can make the WINDOW feasible (e.g. at\n"
        "  5%) without making the FIXED M3-selected preload admissible, because that\n"
        "  selection was never re-derived against the loosened window (SELECTED_PRELOAD_\n"
        "  TOO_HIGH). Sensitivity F shows friction mu re-derives both the requirement and\n"
        "  the selection together, and flips admissibility to True at mu>=0.30. The 8 mm\n"
        "  bolt's rejection in the baseline case is governed by M4 preload feasibility, not\n"
        "  by bearing or geometry."
    )


if __name__ == "__main__":
    main()
