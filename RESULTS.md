# RESULTS.md — Engineering Results

A concise, results-first summary of this project's findings. For the
audit trail behind every number here, see [VERIFICATION.md](VERIFICATION.md).
For the full milestone-by-milestone derivation and source audit, see
[README.md](README.md).

This is a **generic, reduced-order conceptual engineering study**. It
is not a certified, qualified, or procurement-ready design.

## 1. Governing load result

For the representative 8-bolt / R=0.5 m circular pattern under the
illustrative interface load case (`Fx=20 kN, Fy=−10 kN, Fz=80 kN,
Mx=25 kN·m, My=−15 kN·m, Mz=12 kN·m`), **bolt 1 carries the governing
tensile load, 24,142.1 N**, and the group-level force/moment
equilibrium closes to machine precision (max residual ~3.6×10⁻¹²).

## 2. Strength-sizing result

Under illustrative tensile/shear allowables and a quadratic
tension-shear interaction criterion, **8 mm is the smallest passing
candidate under modeled criteria** (governing bolt 1, interaction
mode, margin +0.662). 5 mm and 6 mm fail.

## 3. Required preload result

The analytical preload required to prevent joint separation and
interface slip is **29,258.2 N/bolt**, governed by slip at bolt 0 (the
separation-only requirement is lower, 19,313.7 N/bolt). A 1.20 design
factor gives a selected preload of **35,109.8 N/bolt** — an analytical
screening quantity, not a torque specification.

## 4. The 8 mm proof-window conflict

Under illustrative proof strength, an illustrative installation
scatter allowance, and a proof-load installation-fraction ceiling, the
8 mm bolt's installation-preload window is **infeasible**:
scatter-adjusted minimum target 32,509.1 N exceeds the proof-based
maximum target 31,290.3 N by 1,218.8 N. **8 mm passes the original
strength screen but fails the proof-based preload-window screen** —
these are genuinely different failure modes, and the second one is not
visible from strength alone.

## 5. The 10 mm local-joint result

**10 mm is the smallest admissible conceptual candidate** once
strength, preload-window feasibility, local bearing stress, and
edge-distance/spacing geometry screens are all combined under a
predeclared admissibility rule. At 10 mm, bearing margin is +4.59,
edge-distance ratio 5.00 (vs. a 1.5 criterion), spacing ratio 38.27
(vs. a 3.0 criterion) — all comfortably clear. Thread stripping is
**explicitly not modeled** (no credible source-verified formula could
be established from available thread-geometry information) rather than
approximated.

## 6. Torque-control robustness result

Under an illustrative nut-factor range `K=[0.15, 0.25]`, the 10 mm
candidate has **no torque value that robustly guarantees the achieved
preload stays inside its target window for every friction condition in
that range** (`NO_ROBUST_TORQUE_WINDOW`; friction-uncertainty ratio
1.667 exceeds the preload window's own tolerance ratio 1.504). A
narrower, tighter-friction-control range (`K=[0.18, 0.22]`) restores
robustness; 12 mm is torque-robust at the original baseline range
outright.

## 7. Direct-verification result

The 10 mm preload window tolerates up to **`epsilon_max ≈ 20.1%`**
symmetric deterministic measurement/control error. Every quantified
direct preload-verification method examined (bolt elongation ~5%,
ultrasonic ~10%, load-sensing washer ~10%, instrumented/strain-gauged
bolt ~5% — the only figure read directly from a primary source) sits
comfortably inside that tolerance and is **feasible**, with in-service
proof reserves of +32.7% to +38.0%. 12 mm's tolerance is wider still
(`epsilon_max ≈ 36.8%`).

## 8. 10 mm vs. 12 mm architecture trade

| | 10 mm + instrumented-bolt verification | 12 mm + torque control |
|---|---|---|
| Installation-control robust? | Yes (deterministic sensitivity) | Yes (baseline K range) |
| In-service proof reserve | +38.0% | +39.9% |
| Total fastener-group mass | **390.5 g** | **623.9 g** (+59.8%) |
| Normalized complexity index | **7.0** | **1.0** |
| All 7 mandatory gates pass? | Yes | Yes |

**Neither architecture Pareto-dominates the other**: 10 mm wins mass
and local-joint packaging margin; 12 mm wins installation-tolerance
ratio, proof reserve, and normalized complexity.

## 9. Final conceptual recommendation

A predeclared decision rule — fixed before any architecture table was
computed — prefers the 10 mm architecture only if its mass is lower
**and** its measurement-error reserve to `epsilon_max` is ≥5
percentage points **and** its complexity index is ≤6.0. At the
illustrative baseline, 10 mm's complexity index (7.0) exceeds that
threshold by exactly one point, so the rule outputs **`PREFER_B` — the
12 mm + torque-control architecture.**

> The structural screening identifies 10 mm as the smallest admissible
> conceptual bolt. Once installation architecture is included, the
> baseline decision rule prefers the 12 mm torque-controlled
> architecture because it trades a 59.8% fastener-system mass increase
> for substantially lower installation complexity and robust torque
> control. **This preference is sensitivity-dependent, not a universal
> optimum**: reducing any single 10 mm installation-complexity
> sub-score by one point (e.g. a mature, pre-qualified instrumented-bolt
> product line needing less calibration overhead than assumed here)
> flips the recommendation to the 10 mm architecture.

This is a **deterministic sensitivity finding**, not a probabilistic
one. The historical Milestone 5 selection of 10 mm as the smallest
*structurally* admissible conceptual candidate is unchanged and is not
rewritten by this architecture-level preference.

## 10. Critical limitations

- All material, geometry, and installation-error figures are
  illustrative unless explicitly marked sourced (see
  [VERIFICATION.md §8](VERIFICATION.md#8-source-audit-accuracy-check-this-session)).
- No fatigue, prying, thermal preload, embedment/relaxation, nonlinear
  plate flexibility, or nonlinear contact is modeled anywhere.
- No torque-tension nut-factor detail (thread/under-head friction
  split, lubrication-specific coefficients), no torque-angle
  tightening, no instrument-calibration-drift modeling.
- No detailed instrumentation hardware design, wiring-harness mass, or
  electronics/data-system mass in the Milestone 8 complexity index.
- No real procurement cost anywhere in this project — the Milestone 8
  complexity index is an explicit, normalized, non-dollar proxy.
- Thread stripping is explicitly not modeled (Milestone 5).
- No proof testing, qualification, or certification of any kind. This
  is a conceptual reduced-order screening study only.
