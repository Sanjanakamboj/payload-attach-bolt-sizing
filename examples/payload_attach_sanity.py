"""Representative payload-attach bolt-group sanity case (Milestone 1).

ILLUSTRATIVE ONLY: pattern geometry and load magnitudes are order-of-
magnitude representative of a small-to-mid payload launch interface.
They are NOT tuned to size any particular bolt, and no strength or
margin-of-safety conclusion should be drawn from this example -- that
is out of scope for Milestone 1.

Run with:
    python examples/payload_attach_sanity.py
"""

import math

from payload_bolts import InterfaceLoad, circular_pattern, distribute_loads

# ---------------------------------------------------------------------------
# Pattern: 8 bolts evenly spaced on a 0.5 m radius bolt circle.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
pattern = circular_pattern(N_BOLTS, RADIUS)

# ---------------------------------------------------------------------------
# Applied interface loads (illustrative launch load-factor case).
# ---------------------------------------------------------------------------
load = InterfaceLoad(
    Fx=20_000.0,   # N
    Fy=-10_000.0,  # N
    Fz=80_000.0,   # N, tension
    Mx=25_000.0,   # N*m
    My=-15_000.0,  # N*m
    Mz=12_000.0,   # N*m
)

result = distribute_loads(pattern, load)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def main() -> None:
    print("PAYLOAD ATTACH BOLT-GROUP SANITY CASE (illustrative loads, Milestone 1)")

    _rule("PATTERN")
    cx, cy = pattern.centroid
    print(f"  n_bolts   = {pattern.n_bolts}")
    print(f"  radius    = {RADIUS:.4f} m")
    print(f"  centroid  = ({cx:.6f}, {cy:.6f}) m")
    print(f"  J         = {pattern.J:.6e} m^2")
    print(f"  Ix_group  = {pattern.Ix_group:.6e} m^2")
    print(f"  Iy_group  = {pattern.Iy_group:.6e} m^2")

    _rule("APPLIED LOADS")
    print(f"  Fx = {load.Fx:>12,.1f} N")
    print(f"  Fy = {load.Fy:>12,.1f} N")
    print(f"  Fz = {load.Fz:>12,.1f} N")
    print(f"  Mx = {load.Mx:>12,.1f} N*m")
    print(f"  My = {load.My:>12,.1f} N*m")
    print(f"  Mz = {load.Mz:>12,.1f} N*m")

    _rule("PER-BOLT TABLE")
    header = f"{'idx':>3} {'angle_deg':>10} {'x':>8} {'y':>8} {'Vx':>10} {'Vy':>10} {'V_res':>10} {'axial_total':>12}"
    print(header)
    print("-" * len(header))
    for b in result.bolts:
        print(
            f"{b.index:>3} {math.degrees(b.angle):>10.1f} {b.x:>8.4f} {b.y:>8.4f} "
            f"{b.Vx_total:>10.1f} {b.Vy_total:>10.1f} {b.shear_resultant:>10.1f} {b.axial_total:>12.1f}"
        )

    _rule("SUMMARY")
    max_shear = result.max_shear_bolt()
    max_tensile = result.max_tensile_bolt()
    max_abs_axial = result.max_abs_axial_bolt()
    print(f"  max shear bolt        : index {max_shear.index}, V = {max_shear.shear_resultant:,.1f} N")
    print(f"  max tensile bolt       : index {max_tensile.index}, T = {max_tensile.axial_total:,.1f} N")
    print(f"  max |axial| bolt       : index {max_abs_axial.index}, T = {max_abs_axial.axial_total:,.1f} N")

    _rule("EQUILIBRIUM")
    eq = result.equilibrium
    print(f"  Fx recovered = {eq.Fx_recovered:>14,.4f} N   (residual {eq.Fx_residual: .3e})")
    print(f"  Fy recovered = {eq.Fy_recovered:>14,.4f} N   (residual {eq.Fy_residual: .3e})")
    print(f"  Fz recovered = {eq.Fz_recovered:>14,.4f} N   (residual {eq.Fz_residual: .3e})")
    print(f"  Mx recovered = {eq.Mx_recovered:>14,.4f} N*m (residual {eq.Mx_residual: .3e})")
    print(f"  My recovered = {eq.My_recovered:>14,.4f} N*m (residual {eq.My_residual: .3e})")
    print(f"  Mz recovered = {eq.Mz_recovered:>14,.4f} N*m (residual {eq.Mz_residual: .3e})")
    print(f"  max |residual| = {eq.max_abs_residual:.3e}")


if __name__ == "__main__":
    main()
