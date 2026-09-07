"""Representative bolt strength sizing study (Milestone 2).

Builds on the exact Milestone 1 8-bolt / R=0.5 m load case from
examples/payload_attach_sanity.py, then screens a small set of
illustrative idealized-circular-shank bolt candidates against an
illustrative bolt material.

ILLUSTRATIVE ONLY: candidate diameters use the idealized gross circular
shank area (A = pi*d^2/4) for BOTH tensile and shear area -- this is NOT
an actual ISO/SAE threaded tensile-stress-area calculation. The bolt
material properties are a labeled illustrative steel, not a sourced
fastener specification. Nothing in this example should be read as a
certification margin.

Run with:
    python examples/bolt_strength_sizing.py
"""

from payload_bolts import (
    BoltMaterial,
    InterfaceLoad,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    evaluate_candidates,
    select_smallest_passing_bolt,
)
from payload_bolts.strength import NoFeasibleCandidateError

# ---------------------------------------------------------------------------
# Same Milestone 1 group-load case as examples/payload_attach_sanity.py.
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

# ---------------------------------------------------------------------------
# Illustrative bolt material. Values chosen after inspecting the
# Milestone 1 governing loads (max tensile bolt ~24.1 kN, max shear bolt
# ~5.7 kN) so that the candidate table below contains both a failing
# and a passing candidate without distorting the physics.
# ---------------------------------------------------------------------------
MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt",
    tensile_allowable=800e6,  # Pa, illustrative
    shear_allowable=480e6,  # Pa, illustrative
)

# Idealized circular-shank candidate diameters (mm). A broader small-size
# range than the "8/10/12/14/16 mm" suggestion is used here specifically
# so the table contains at least one failing candidate -- 8 mm alone
# already passes comfortably against this material and the Milestone 1
# sanity loads.
CANDIDATE_DIAMETERS_MM = [5.0, 6.0, 8.0, 10.0, 12.0]
CANDIDATES = [circular_unthreaded_bolt(d / 1000.0) for d in CANDIDATE_DIAMETERS_MM]


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt_margin(m):
    return "n/a" if m is None else f"{m:+.3f}"


def main() -> None:
    print("BOLT STRENGTH SIZING STUDY (illustrative candidates/material, Milestone 2)")
    print("Consumes the Milestone 1 8-bolt / R=0.5 m sanity load case unchanged.")

    _rule("APPLIED INTERFACE LOADS")
    print(f"  Fx = {load.Fx:>12,.1f} N")
    print(f"  Fy = {load.Fy:>12,.1f} N")
    print(f"  Fz = {load.Fz:>12,.1f} N")
    print(f"  Mx = {load.Mx:>12,.1f} N*m")
    print(f"  My = {load.My:>12,.1f} N*m")
    print(f"  Mz = {load.Mz:>12,.1f} N*m")

    _rule("BOLT GROUP LOADS (Milestone 1, unchanged)")
    max_shear = group_load_result.max_shear_bolt()
    max_tensile = group_load_result.max_tensile_bolt()
    print(f"  max shear bolt   : index {max_shear.index}, V = {max_shear.shear_resultant:,.1f} N")
    print(f"  max tensile bolt : index {max_tensile.index}, T = {max_tensile.axial_total:,.1f} N")

    _rule("MATERIAL (illustrative)")
    print(f"  name              = {MATERIAL.name}")
    print(f"  tensile_allowable = {MATERIAL.tensile_allowable / 1e6:,.0f} MPa")
    print(f"  shear_allowable   = {MATERIAL.shear_allowable / 1e6:,.0f} MPa")

    _rule("CANDIDATE BOLTS (idealized gross circular shank area)")
    header = (
        f"{'d_mm':>6} {'A_t_mm2':>9} {'A_s_mm2':>9} {'min MS_t':>10} {'min MS_s':>10} "
        f"{'min MS_i':>10} {'gov_bolt':>8} {'gov_mode':>10} {'PASS/FAIL':>10}"
    )
    print(header)
    print("-" * len(header))

    candidate_results = evaluate_candidates(group_load_result, CANDIDATES, MATERIAL)
    for result in candidate_results:
        d_mm = result.bolt_section.nominal_diameter * 1000.0
        a_t_mm2 = result.bolt_section.tensile_area * 1e6
        a_s_mm2 = result.bolt_section.shear_area * 1e6
        verdict = "PASS" if result.passed else "FAIL"
        print(
            f"{d_mm:>6.1f} {a_t_mm2:>9.2f} {a_s_mm2:>9.2f} "
            f"{_fmt_margin(result.min_tension_margin()):>10} {_fmt_margin(result.min_shear_margin()):>10} "
            f"{_fmt_margin(result.min_interaction_margin()):>10} {result.governing_bolt_index:>8} "
            f"{str(result.governing_mode):>10} {verdict:>10}"
        )

    _rule("SELECTED")
    try:
        selected = select_smallest_passing_bolt(group_load_result, CANDIDATES, MATERIAL)
        d_mm = selected.bolt_section.nominal_diameter * 1000.0
        print(f"  smallest passing candidate : {d_mm:.1f} mm")
        print(f"  governing bolt              : index {selected.governing_bolt_index}")
        print(f"  governing mode              : {selected.governing_mode}")
        print(f"  governing margin (preliminary strength margin): {selected.governing_margin:+.3f}")
    except NoFeasibleCandidateError as exc:
        print(f"  NO FEASIBLE CANDIDATE: {exc}")


if __name__ == "__main__":
    main()
