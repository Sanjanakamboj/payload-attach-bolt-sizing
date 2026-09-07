"""Representative preloaded-joint screening study (Milestone 3).

Builds on the exact Milestone 1 8-bolt / R=0.5 m load case and the
Milestone 2 selected 8 mm illustrative bolt candidate (reported here for
continuity only -- the preload/joint mechanics in this script do not
depend on bolt diameter).

ILLUSTRATIVE ONLY: joint stiffness values, friction coefficient, and
selected preload are all labeled illustrative / preliminary. This
script does NOT convert preload to installation torque and does NOT
check preload against bolt proof/yield strength.

Run with:
    python examples/preloaded_joint_screening.py
"""

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadState,
    assess_bolt_group_strength,
    assess_preloaded_joint,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
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

# Milestone 2 selected bolt, reported for continuity only.
SELECTED_DIAMETER_MM = 8.0
_selected_section = circular_unthreaded_bolt(SELECTED_DIAMETER_MM / 1000.0)
_selected_material = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6
)
_selected_strength = assess_bolt_group_strength(group_load_result, _selected_section, _selected_material)

# ---------------------------------------------------------------------------
# Illustrative joint stiffness (preliminary equivalent stiffnesses).
# k_b/k_m chosen to give C ~ 0.2, a plausible order-of-magnitude
# structural-joint load-fraction value.
# ---------------------------------------------------------------------------
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2

# Illustrative interface friction coefficient.
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)

# ---------------------------------------------------------------------------
# Required preload FIRST -- computed before any preload is selected.
# ---------------------------------------------------------------------------
REQUIRED = required_preload(group_load_result, STIFFNESS, FRICTION)

# Selected preload chosen only after computing the requirement, with an
# explicit design reserve applied on top (illustrative factor).
PRELOAD_FACTOR = 1.2
SELECTED_PRELOAD_PER_BOLT = REQUIRED.overall_required * PRELOAD_FACTOR
_selected_preload_state = PreloadState(preload_per_bolt=SELECTED_PRELOAD_PER_BOLT, label="selected")
GROUP_RESULT = assess_preloaded_joint(group_load_result, _selected_preload_state, STIFFNESS, FRICTION)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_margin(m):
    return "n/a" if m is None else f"{m:+.3f}"


def main() -> None:
    print("PRELOADED JOINT SCREENING STUDY (illustrative stiffness/friction/preload, Milestone 3)")
    print("Consumes the Milestone 1 8-bolt / R=0.5 m sanity load case unchanged.")

    _rule("BOLT GROUP")
    print(f"  n_bolts                  = {pattern.n_bolts}")
    print(f"  radius                   = {RADIUS:.4f} m")
    print(f"  selected Milestone 2 bolt = {SELECTED_DIAMETER_MM:.1f} mm (continuity only)")
    print(f"  Milestone 2 governing bolt/mode = index {_selected_strength.governing_bolt_index} / {_selected_strength.governing_mode}")

    _rule("JOINT STIFFNESS (illustrative preliminary equivalent stiffnesses)")
    print(f"  k_b = {STIFFNESS.bolt_stiffness:.3e} N/m")
    print(f"  k_m = {STIFFNESS.member_stiffness:.3e} N/m")
    print(f"  C   = {STIFFNESS.C:.4f}")

    _rule("FRICTION (illustrative interface friction coefficient)")
    print(f"  mu               = {FRICTION.friction_coefficient:.3f}")
    print(f"  n_faying_surfaces = {FRICTION.number_of_faying_surfaces}")

    _rule("REQUIRED PRELOAD (computed before selecting a preload)")
    print(f"  separation requirement : {REQUIRED.separation_required:>12,.1f} N/bolt (governing bolt {REQUIRED.separation_governing_bolt})")
    print(f"  slip requirement       : {REQUIRED.slip_required:>12,.1f} N/bolt (governing bolt {REQUIRED.slip_governing_bolt})")
    print(f"  overall requirement    : {REQUIRED.overall_required:>12,.1f} N/bolt (governing: {REQUIRED.overall_governing_constraint}, bolt {REQUIRED.overall_governing_bolt})")

    _rule("SELECTED PRELOAD")
    reserve_pct = (SELECTED_PRELOAD_PER_BOLT / REQUIRED.overall_required - 1.0) * 100.0
    print(f"  preload_factor applied  : {PRELOAD_FACTOR:.2f}")
    print(f"  selected preload/bolt   : {SELECTED_PRELOAD_PER_BOLT:>12,.1f} N")
    print(f"  preload reserve         : {reserve_pct:+.1f} % above the analytical requirement")

    _rule("PER-BOLT TABLE")
    header = (
        f"{'idx':>3} {'T_ext':>10} {'T_sep':>10} {'dFb':>9} {'F_b_tot':>10} "
        f"{'F_clamp':>10} {'V':>9} {'V_fric':>9} {'MS_sep':>8} {'MS_slip':>8} {'PASS/FAIL':>10}"
    )
    print(header)
    print("-" * len(header))
    for b in GROUP_RESULT.bolts:
        verdict = "PASS" if (b.separation_pass and b.slip_pass) else "FAIL"
        print(
            f"{b.index:>3} {b.axial_total:>10.1f} {b.separating_demand:>10.1f} {b.additional_bolt_load:>9.1f} "
            f"{b.total_bolt_tension:>10.1f} {b.remaining_clamp_force:>10.1f} {b.shear_demand:>9.1f} "
            f"{b.friction_capacity:>9.1f} {_fmt_margin(b.separation_margin):>8} {_fmt_margin(b.slip_margin):>8} {verdict:>10}"
        )

    _rule("SUMMARY")
    print(f"  all locations closed?   : {GROUP_RESULT.all_locations_closed}")
    print(f"  no slip anywhere?       : {GROUP_RESULT.all_locations_no_slip}")
    print(f"  governing separation bolt : index {GROUP_RESULT.governing_separation_bolt_index}, MS = {_fmt_margin(GROUP_RESULT.min_separation_margin)}")
    print(f"  governing slip bolt        : index {GROUP_RESULT.governing_slip_bolt_index}, MS = {_fmt_margin(GROUP_RESULT.min_slip_margin)}")
    print(f"  OVERALL JOINT SCREEN       : {'PASS' if GROUP_RESULT.passed else 'FAIL'}")

    # -----------------------------------------------------------------
    # Sensitivity studies
    # -----------------------------------------------------------------
    _rule("SENSITIVITY A: preload factor (F_preload / F_required)")
    header = f"{'factor':>7} {'preload/bolt':>13} {'min MS_sep':>11} {'min MS_slip':>12} {'sep':>5} {'slip':>5} {'overall':>8}"
    print(header)
    print("-" * len(header))
    for factor in (0.5, 0.75, 1.0, 1.25, 1.5, 2.0):
        preload_val = REQUIRED.overall_required * factor
        g = assess_preloaded_joint(group_load_result, PreloadState(preload_per_bolt=preload_val), STIFFNESS, FRICTION)
        print(
            f"{factor:>7.2f} {preload_val:>13,.1f} {_fmt_margin(g.min_separation_margin):>11} "
            f"{_fmt_margin(g.min_slip_margin):>12} {'PASS' if g.all_locations_closed else 'FAIL':>5} "
            f"{'PASS' if g.all_locations_no_slip else 'FAIL':>5} {'PASS' if g.passed else 'FAIL':>8}"
        )

    _rule("SENSITIVITY B: friction coefficient (mu), stiffness/loads fixed")
    header = f"{'mu':>6} {'slip req/bolt':>14} {'gov bolt':>9} {'overall req/bolt':>17}"
    print(header)
    print("-" * len(header))
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        r = required_preload(group_load_result, STIFFNESS, FrictionModel(friction_coefficient=mu))
        print(f"{mu:>6.2f} {r.slip_required:>14,.1f} {r.slip_governing_bolt:>9} {r.overall_required:>17,.1f}")

    _rule("SENSITIVITY C: load fraction C (via member stiffness), mu/loads fixed")
    header = f"{'C':>6} {'sep req/bolt':>13} {'slip req/bolt':>14} {'overall req/bolt':>17}"
    print(header)
    print("-" * len(header))
    kb = STIFFNESS.bolt_stiffness
    for C in (0.10, 0.20, 0.30, 0.40):
        km = kb * (1.0 - C) / C
        stiffness_c = JointStiffness(bolt_stiffness=kb, member_stiffness=km)
        r = required_preload(group_load_result, stiffness_c, FRICTION)
        print(f"{C:>6.2f} {r.separation_required:>13,.1f} {r.slip_required:>14,.1f} {r.overall_required:>17,.1f}")

    _rule("ENGINEERING INTERPRETATION")
    print(
        "  Preload creates a compressive clamping reserve at each bolt location.\n"
        "  An external separating (tensile) load at a bolt is shared between an\n"
        "  increase in bolt tension (fraction C) and a loss of clamp force in the\n"
        "  clamped members (fraction 1-C). Joint separation occurs once the clamp\n"
        "  force is fully exhausted at a location. Friction capacity falls as\n"
        "  clamp force is unloaded, so local interface slip can occur before or\n"
        "  alongside separation depending on local shear demand and mu. Torsional\n"
        "  Mz is included here because it already contributes to each bolt's\n"
        "  Milestone 1 shear demand. Required preload above is an analytical\n"
        "  screening quantity only -- it is NOT an installation torque\n"
        "  specification. A real preload design also requires bolt proof/yield\n"
        "  limits, preload scatter, torque uncertainty, embedment loss, thermal\n"
        "  effects, and a more detailed contact/friction analysis, all deferred\n"
        "  to later milestones.\n"
        "  Lower C transfers less external load into the bolt and more clamp-\n"
        "  force loss into the members (greater separation/slip sensitivity for a\n"
        "  given preload). Higher C retains more clamp force for a given preload\n"
        "  but increases the bolt tension increment -- this is a trade, not a\n"
        "  universal preference for high C."
    )


if __name__ == "__main__":
    main()
