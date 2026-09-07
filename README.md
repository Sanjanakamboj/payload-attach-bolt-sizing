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

---

# Milestone 2 — preliminary bolt strength sizing

**Milestone 2 adds preliminary bolt tensile/shear strength screening
only. Preload, slip, separation, bearing, prying, thread failure, and
fatigue remain deferred.**

Milestone 2 consumes the Milestone 1 per-bolt loads (`axial_total`,
`shear_resultant`) exactly as produced by `distribute_loads` and layers
a strength assessment on top. **Bolt-group geometry, direct/torsional
shear, overturning axial distribution, and equilibrium verification are
completely unchanged from Milestone 1** — see `src/payload_bolts/strength.py`.

The engineering question: given the per-bolt axial and shear loads from
the rigid-interface bolt-group model, what bolt size/material is
required, which failure mode governs, and what margin exists against
tensile/shear strength?

## Bolt material representation

`BoltMaterial(name, tensile_allowable, shear_allowable)` — allowables in
Pa, both finite and > 0. Any illustrative material entry in this project
is explicitly labeled illustrative in its name and is not claimed to be
a sourced fastener specification.

## Bolt section / area convention

`BoltSection(nominal_diameter, tensile_area, shear_area)` — meters and
m². Tensile and shear areas are supplied **explicitly** by the caller;
Milestone 2 does not assume a real threaded fastener's tensile stress
area equals its gross shank area. `circular_unthreaded_bolt(diameter)`
is provided as an **idealized shank-area helper** (`A = pi*d^2/4` used
for both tensile and shear area) — labeled idealized, not an ISO/SAE
tensile-stress-area calculation.

## Signed axial-load policy (unchanged from Milestone 1, applied here)

Milestone 1's signed `axial_total` is never mutated. For the tensile
strength check only:

```
T_positive = max(axial_total, 0)
```

A bolt with `axial_total <= 0` has no tensile demand (tensile check not
applicable); Milestone 2 does not evaluate a compressive bolt failure
mode.

## Tensile stress

```
sigma_t = T_positive / A_t
```

## Shear stress

```
tau = V_resultant / A_s
```

`V_resultant` is the unmodified Milestone 1 `shear_resultant`.

## Margin definitions (preliminary bolt strength margins)

```
MS_tension     = S_t_allow / sigma_t - 1        (sigma_t > 0, else not applicable -> None)
MS_shear       = S_s_allow / tau - 1            (tau > 0, else not applicable -> None)
```

**Zero-demand convention**: when a stress is exactly zero, the
corresponding margin is `None` (not applicable), never a division by
zero and never `+inf`.

## Quadratic tension-shear interaction (illustrative)

An **illustrative quadratic tension-shear interaction**, not a named
fastener standard:

```
FI = (sigma_t / S_t_allow)^2 + (tau / S_s_allow)^2
```

Pass if `FI <= 1` (boundary included). Interaction margin:

```
MS_interaction = 1 / sqrt(FI) - 1          (FI > 0, else None)
```

## Governing mode and deterministic tie-break

Each bolt reports its tensile, shear, and interaction margins
**separately** — they are never collapsed into one undocumented number.
A bolt's *governing margin* is the smallest applicable (non-`None`)
margin among the three.

Because the quadratic form guarantees `MS_interaction <= MS_tension` and
`MS_interaction <= MS_shear` whenever both are applicable (with equality
exactly when the *other* demand is zero), a bolt loaded in pure tension
or pure shear ties exactly with "interaction". **This project's
tie-break convention prefers the more specific single-mode explanation
on a tie**: tension, then shear, then interaction — so a pure-tension
bolt is reported tension-governed and a pure-shear bolt shear-governed.
Interaction only governs outright (not via tie-break) when both tensile
and shear demand are simultaneously present, in which case its margin is
strictly the smallest of the three.

At the **group level**, the governing bolt is the one with the smallest
governing margin across all bolts (a bolt with no applicable margin at
all — zero total demand — is treated as having no constraint, i.e. it
cannot govern). Ties across bolts are broken by **lowest bolt index**.
The group's `passed` is true only if every bolt individually passes.

## Candidate bolt sizing

`evaluate_candidates(group_load_result, candidates, material)` assesses
a list of candidate `BoltSection`s against the same Milestone 1 group
load result, returned sorted by increasing nominal diameter.
`select_smallest_passing_bolt(...)` returns the first passing candidate
in that order and raises `NoFeasibleCandidateError` if none pass — it
never silently enlarges beyond the supplied candidate list. This
milestone uses an explicit idealized-diameter candidate list, not an
inferred standard fastener size table.

## Representative sizing result

`examples/bolt_strength_sizing.py` reuses the exact Milestone 1 8-bolt /
R=0.5 m sanity load case (max tensile bolt: index 1, T ≈ 24.1 kN; max
shear bolt: index 5, V ≈ 5.7 kN) against an illustrative
"Illustrative high-strength steel bolt" (tensile allowable 800 MPa,
shear allowable 480 MPa) and idealized circular-shank candidates at
5, 6, 8, 10, 12 mm nominal diameter:

- 5 mm and 6 mm candidates **fail** (interaction-governed at bolt 1).
- 8 mm, 10 mm, and 12 mm candidates **pass**.
- Smallest passing candidate: **8 mm**, governing bolt index 1,
  governing mode interaction, governing margin ≈ +0.66.

Run it:

```bash
python examples/bolt_strength_sizing.py
```

## Verification summary (Milestone 2)

- One fully hand-derived single-bolt load state (T=10 kN, V=5 kN,
  A_t=100 mm², A_s=80 mm², S_t=500 MPa, S_s=300 MPa) verified exactly
  for stress, both individual margins, and FI.
- Exact interaction boundary (`FI = 1`) verified to pass with zero
  margin; slightly above/below verified to fail/pass.
- Tensile and shear margin boundaries (`MS = 0` exactly) verified.
- Zero-tensile-demand, zero-shear-demand, and zero-total-demand cases
  verified not to divide by zero and to report `None`/clean pass.
- Load, area, and allowable scaling laws verified (doubling area halves
  stress; doubling allowable improves margin; doubling load quadruples
  FI under the quadratic form).
- Dedicated tension-governed, shear-governed, and interaction-governed
  constructions verified, plus a construction where the strength
  governing bolt differs from the Milestone 1 max-load labels,
  confirming governance is recomputed from margins, not assumed.
- Candidate ordering, smallest-passing selection, no-feasible-candidate
  handling, and monotonic improvement with increasing idealized
  diameter all verified.
- Deterministic repeated assessment and non-mutation of the Milestone 1
  `BoltGroupResult` verified.
- **All 47 Milestone 1 tests remain unchanged and passing.**

## Limitations

Milestone 1 limitations (rigid payload/interface plate, equal bolt
stiffness, linear elastic distribution, point fasteners, moments about
the bolt-group centroid) all still apply. In addition, for Milestone 2:

- Signed compression-side bolt loads from Milestone 1 are preserved;
  the tensile check ignores compressive bolt loading (no compressive
  bolt failure mode is evaluated).
- No preload or proof-load check.
- No torque.
- No friction / shear load transfer through faying (joint) surfaces.
- No joint separation / contact redistribution.
- No bearing or tear-out / edge-distance checks.
- No prying action.
- No pull-through.
- No thread stripping.
- No fatigue.
- Candidate areas are idealized (gross circular shank) unless a caller
  explicitly supplies real tensile/shear stress areas — never claimed
  to be sourced ISO/SAE fastener data.
- The quadratic tension-shear interaction is illustrative, not a named
  fastener standard.
- No certification claim of any kind.

## Install and test

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest -q
python examples/payload_attach_sanity.py
python examples/bolt_strength_sizing.py
```
