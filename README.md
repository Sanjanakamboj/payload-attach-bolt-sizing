# payload-attach-bolt-sizing (STM-08)

Bolted-joint sizing for a payload-to-structure interface under representative
launch load factors.

**Milestone 1 computes elastic rigid-interface bolt-group load distribution
only. Bolt strength, preload, separation, slip, bearing, and final
margin-of-safety sizing are intentionally deferred.**

## Objective

Given a rigid payload interface and an idealized bolt group, how are launch
forces and moments distributed to the individual bolts, and can the resulting
bolt forces be verified exactly from static equilibrium?

## Current scope (Milestone 1)

- Validated bolt-pattern geometry (arbitrary coordinates, plus circular and
  rectangular pattern generators).
- A validated `InterfaceLoad` (Fx, Fy, Fz, Mx, My, Mz).
- A rigid, equal-bolt-stiffness bolt-group solver:
  - direct in-plane shear (equal split of Fx, Fy),
  - torsional shear from Mz (proportional to radius from the bolt-group
    centroid),
  - axial/tensile bolt load from Fz, Mx, My (linear distribution, solved
    generally from bolt-coordinate sums -- not a formula specialized to
    circular patterns).
- A per-bolt result structure and deterministic governing-bolt helpers
  (max shear, max tensile, max absolute axial).
- An independent equilibrium-verification helper that recovers group loads
  from the per-bolt results and reports residuals against the applied load.
- A representative (illustrative) payload-attach launch-load sanity case.
- An automated test suite covering geometry, shear, torsion, axial load,
  generality/validation, and one fully hand-derived closed-form case.

## Coordinate and sign convention

- The payload interface is modeled as a rigid plate lying in the **x-y**
  plane; **+z** is the interface normal.
- Each bolt is an idealized point fastener at `(x_i, y_i)`, in meters.
- **Fx, Fy** — in-plane shear resultants (N).
- **Fz** — axial/normal resultant (N). Positive Fz is **tension** (+z,
  pulling the payload away from the structure). Fz may be negative
  (compression) and is never silently clipped.
- **Mx, My** — overturning moments (N·m) about the x and y axes.
- **Mz** — in-plane torsional moment (N·m) about the interface normal,
  positive by the right-hand rule (counterclockwise viewed from +z).
- **All applied loads are referenced about the bolt-group centroid**, not
  necessarily the origin used to define bolt coordinates. Internally, bolt
  coordinates are translated to the centroid for all load-distribution math;
  original coordinates are retained unchanged for reporting. A worked test
  (`test_translated_pattern_same_relative_bolt_forces`) proves this makes the
  distributed bolt forces invariant to translating every bolt coordinate by a
  constant.
- Units are SI throughout: meters, newtons, newton-meters.

## Rigid-interface bolt-group model

This is an **elastic rigid-plate / equal-bolt-stiffness** bolt-group model:
the attach plate is treated as perfectly rigid, and every bolt is assumed to
have identical stiffness, so bolt loads are computed purely from geometry --
no stiffness-weighted load sharing, no contact/separation redistribution.

### Direct in-plane shear

```
Vx_direct,i = Fx / n
Vy_direct,i = Fy / n
```

### Torsional shear from Mz

For bolt `i` at centroid-relative coordinates `(x_i, y_i)`, with
`J = sum(x_i^2 + y_i^2)`:

```
Vx_torsion,i = -Mz * y_i / J
Vy_torsion,i =  Mz * x_i / J
```

Total shear: `Vx_i = Vx_direct,i + Vx_torsion,i`, `Vy_i = Vy_direct,i +
Vy_torsion,i`, `V_i = sqrt(Vx_i^2 + Vy_i^2)`.

### Axial/tensile load from Fz, Mx, My (overturning)

Bolt axial load is assumed to vary linearly with bolt coordinates (rigid
rotation of the attach flange): `T_i = c0 + cx*x_i + cy*y_i`. The
coefficients are solved from the general 3x3 system built from bolt-group
coordinate sums (not a formula specialized to any one pattern shape):

```
[ n,    Sx,   Sy  ] [c0]   [ Fz]
[ Sy,   Sxy,  Syy ] [cx] = [ Mx]
[-Sx,  -Sxx, -Sxy ] [cy]   [ My]
```

where `Sx = sum(x_i)`, `Sy = sum(y_i)`, `Sxx = sum(x_i^2)`,
`Syy = sum(y_i^2)`, `Sxy = sum(x_i*y_i)` (all centroid-relative, so `Sx = Sy
= 0` in practice, but the full system is solved as shown). A singular system
(degenerate bolt geometry) raises a clear error rather than guessing.

**Negative axial results are signed compression-side loads, not clipped to
zero.** Joint separation / contact redistribution is a later-milestone
concern.

## Equilibrium verification

`check_equilibrium` independently recomputes group-level loads from the
per-bolt result list (a separate code path from the distribution solve) and
reports both the recovered totals and their residual against the applied
`InterfaceLoad`:

```
Fx_recovered = sum(Vx_i)              Mx_recovered = sum(y_i * T_i)
Fy_recovered = sum(Vy_i)              My_recovered = sum(-x_i * T_i)
Fz_recovered = sum(T_i)               Mz_recovered = sum(x_i*Vy_i - y_i*Vx_i)
```

In the representative sanity case below, all residuals are at floating-point
precision (~1e-12 or exactly zero).

## Representative sanity case

`examples/payload_attach_sanity.py` — an 8-bolt circular pattern, 0.5 m bolt
circle, with illustrative launch-representative loads
(Fx=20 kN, Fy=-10 kN, Fz=80 kN tension, Mx=25 kN·m, My=-15 kN·m, Mz=12 kN·m).
Run it:

```bash
python examples/payload_attach_sanity.py
```

It reports pattern properties, applied loads, a per-bolt table, governing
bolts (max shear / max tensile / max |axial|), and the equilibrium
recovery/residual check. Loads are illustrative only and are not tuned to
size any particular bolt.

## Limitations (explicitly out of scope for Milestone 1)

- Rigid payload/interface plate assumption.
- Equal bolt stiffness (no stiffness-based load fraction).
- Linear elastic distribution only; no nonlinear contact.
- Bolt coordinates treated as ideal point fasteners.
- Moments referenced about the bolt-group centroid.
- No preload or torque.
- No contact/separation redistribution (negative axial values are signed,
  not physical compression-side bolt reactions after joint opening).
- No friction / shear load sharing through faying surfaces.
- No prying action.
- No bearing or tear-out / edge-distance checks.
- No pull-through.
- No fastener strength allowables or margin-of-safety calculation.
- No thread effects, no fatigue.
- No optimization or sensitivity/trade study (later milestone).
- No certification claim of any kind.

## Install and test

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest -q
python examples/payload_attach_sanity.py
```
