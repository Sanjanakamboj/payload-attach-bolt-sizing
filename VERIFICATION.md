# VERIFICATION.md — Technical Audit

**Purpose:** answer "how do you know this is correct?" for every headline
number in this repository. This document is the audit trail behind
[RESULTS.md](RESULTS.md) and [README.md](README.md). It does not
introduce any new engineering model, criterion, candidate, or
assumption — see Milestones 1–8 (in README.md) for all of that. This
document is synthesis and verification only.

## 1. Test suite

```
pytest -q
```

**280 tests pass** across 15 test files (Milestones 1–8's own dedicated
test files plus this milestone's small helper test). Each milestone's
test file writes independent hand/reference formulas directly in the
test body — it never verifies a production function by calling that
same production function a second time. See each test file's own
docstring/comments for the specific independence strategy used.

| Milestone | Test file | Tests |
|---|---|---|
| M1 | `test_geometry.py` | 16 |
| M1 | `test_axial.py` | 10 |
| M1 | `test_shear.py` | 4 |
| M1 | `test_torsion.py` | 6 |
| M1 | `test_handcalc.py` | 3 |
| M1 | `test_generality.py` | 8 |
| M2 | `test_strength.py` | 32 |
| M3 | `test_preload.py` | 40 |
| M4 | `test_preload_limits.py` | 31 |
| M5 | `test_joint_local_checks.py` | 32 |
| M6 | `test_torque_preload.py` | 30 |
| M7 | `test_preload_verification.py` | 31 |
| M8 | `test_hardware_trade.py` | 33 |
| M9 | `test_independent_audit.py` | 4 |
| **Total** | | **280** |

## 2. Example-script regression

All eight prior milestone example scripts, plus this milestone's
`examples/independent_audit.py`, were re-run this session and their
headline outputs cross-checked against the values recorded in every
milestone's own README section and in this document's Section 3 below.
No discrepancy was found (see Section 6, "Documentation number audit").

```
python examples/payload_attach_sanity.py
python examples/bolt_strength_sizing.py
python examples/preloaded_joint_screening.py
python examples/preload_feasibility_screening.py
python examples/bolt_candidate_trade.py
python examples/torque_preload_screening.py
python examples/preload_verification_trade.py
python examples/hardware_architecture_trade.py
python examples/independent_audit.py
```

## 3. Independent-audit results (`examples/independent_audit.py`)

This script recomputes every headline cross-milestone quantity from
raw arithmetic written directly in the script — not by calling the
production function under audit and comparing it to itself. For
example, Milestone 2's tensile/shear stress and interaction margin are
recomputed from `sigma = F/A`, `tau = V/A`, `FI = (sigma/St)^2 +
(tau/Ss)^2` directly, not by calling `assess_bolt_group_strength()`
twice.

**Result: 59 independent checks, 0 failures, maximum absolute residual
4.0×10⁻² (a rounding artifact of a stated 1-decimal reference constant,
not a genuine discrepancy — see note below table).**

| Subsystem | Reference quantity | Independent route | Production value | Independent value | Residual | Tolerance | Status |
|---|---|---|---|---|---|---|---|
| M1 | Fx/Fy/Fz/Mx/My/Mz equilibrium | fresh summation of per-bolt `Vx_total`/`Vy_total`/`axial_total` vs. applied load | exact | exact | ≤3.6×10⁻¹² | 1×10⁻⁶ | PASS |
| M1 | Governing tensile bolt | `max(axial_total)` over fresh summation | bolt 1, 24,142.1 N | bolt 1, 24,142.14 N | 3.6×10⁻² | 0.1 N | PASS |
| M2 | 8 mm tensile stress | `sigma = max(T,0)/A_t` | 480,292,528 Pa | 480,292,528 Pa | 6×10⁻⁸ | 1 Pa | PASS |
| M2 | 8 mm interaction margin | `1/sqrt(FI) − 1` | 0.66208051 | 0.66208051 | 6.7×10⁻¹⁶ | 1×10⁻⁹ | PASS |
| M2 | Smallest passing candidate | strength pass/fail at 5/6/8 mm | 8 mm | 8 mm | 0 | — | PASS |
| M3 | Separation-required preload | `(1−C)·max(T,0)`, max over bolts | 19,313.7 N | 19,313.71 N | 8.5×10⁻³ | 0.5 N | PASS |
| M3 | Slip-required preload | `(1−C)T_sep + V/(μn)`, max over bolts | 29,258.2 N | 29,258.19 N | 5.5×10⁻³ | 0.5 N | PASS |
| M3 | Factor=1.0 exact boundary | min slip margin at `F=F_required` | 0.0 | 2.2×10⁻¹⁶ | 2.2×10⁻¹⁶ | 1×10⁻⁶ | PASS |
| M4 | 8 mm proof load | `S_p·A_t` | 41,720.35 N | 41,720.35 N | 0 | 0.5 N | PASS |
| M4 | 8 mm target_max | `eta_proof·F_proof` | 31,290.3 N | 31,290.26 N | 3.7×10⁻² | 0.5 N | PASS |
| M4 | 8 mm target_min | `F_required/(1−delta_F)` | 32,509.1 N | 32,509.11 N | 5.0×10⁻³ | 0.5 N | PASS |
| M4 | 8 mm window infeasibility residual | `target_min − target_max` | 1,218.84 N | 1,218.84 N | 0 | 0.5 N | PASS |
| M4 | 10 mm window width | `target_max − target_min` | 16,381.93 N | 16,381.93 N | 0 | 0.5 N | PASS |
| M5 | 10 mm bearing margin | `allowable/(V/(d·t)) − 1` | production `assess_bearing` | fresh formula | ≈0 | 1×10⁻⁹ | PASS |
| M5 | 10 mm e/d ratio | `(R_plate − R_bolt)/d` | 5.0000 | 5.0000 | 0 | 1×10⁻⁹ | PASS |
| M5 | 10 mm s/d ratio | `2R·sin(π/n)/d` | 38.2683 | 38.2683 | 7×10⁻¹⁵ | 1×10⁻⁹ | PASS |
| M5 | Smallest admissible candidate | full `select_bolt_candidate` re-run | 10 mm | 10 mm | 0 | — | PASS |
| M6 | Robust torque lower/upper | `K_max·d·F_min`, `K_min·d·F_max` | 81.27 / 73.34 N·m | 81.27 / 73.34 N·m | 0 | 1×10⁻⁶ | PASS |
| M6 | K-ratio feasibility identity | `K_max/K_min ≤ F_max/F_min` | infeasible | infeasible | 0 | — | PASS |
| M6 | Baseline 10 mm status | — | `NO_ROBUST_TORQUE_WINDOW` | `NO_ROBUST_TORQUE_WINDOW` | 0 | — | PASS |
| M7 | 10 mm preload-window ratio R | `F_max/F_min` | 1.5039 | 1.50392 | 1.8×10⁻⁵ | 1×10⁻³ | PASS |
| M7 | 10 mm epsilon_max | `(R−1)/(R+1)` | 0.201252 | 0.201252 | 0 | 1×10⁻⁹ | PASS |
| M7 | 12 mm epsilon_max | `(R12−1)/(R12+1)` | 0.368 (rounded) | 0.368217 | 2.2×10⁻⁴ | 2×10⁻³ | PASS |
| M8 | 10 mm/12 mm per-fastener mass | `rho·(V_shank+V_head+V_nut+V_washer)` (sourced ISO geometry) | 48.82 g / 78.00 g | 48.82 g / 78.00 g | 0 | 1×10⁻⁹ kg | PASS |
| M8 | Percent mass increase | `(m12/m10 − 1)·100` | 59.8% (rounded) | 59.7596% | 4.0×10⁻² | 0.2 | PASS |
| M8 | Complexity indices | hand sum of declared sub-scores × weight 1.0 | A=7.0, B=1.0 | A=7.0, B=1.0 | 0 | 1×10⁻⁹ | PASS |
| M8 | Pareto nondominance | 5-axis hand comparison | neither dominates | neither dominates | 0 | — | PASS |
| M8 | Predeclared decision | rule re-applied | `PREFER_B` | `PREFER_B` | 0 | — | PASS |

*Note on residuals like 3.6×10⁻² or 4.0×10⁻²: these arise where the
"production value" column above is a 1-decimal or 3-significant-figure
**rounded reference constant** quoted from README/RESULTS text (e.g.
"24,142.1 N", "59.8%"), compared against the full-precision computed
value (e.g. 24,142.14 N, 59.7596%). This is expected rounding, not a
numerical discrepancy — the row immediately using the full-precision
production object (e.g. "governing tensile bolt load (N)") shows exact
agreement to the solver's own floating-point precision.*

## 4. Repository-hygiene audit

Performed this session (see also Section 9 of the Milestone 9 session
report): no tracked `.venv`, `__pycache__`, `.pytest_cache`, egg-info,
build/dist artifacts, `.DS_Store`, editor artifacts, downloaded PDFs,
absolute machine paths, secrets/tokens, `TODO`/`FIXME` markers, debug
`print()` statements in package code, or temporary/commit-message
files. Confirmed via `git status --porcelain` (clean) and targeted
`grep` scans across `src/`, `tests/`, `examples/`.

## 5. Clean-environment reproducibility

A fresh virtual environment was created **outside** this repository,
the package installed via the exact documented command
(`pip install -e ".[dev]"`), and the full test suite plus every example
script re-run inside it. Results matched the development environment
exactly (same test count, same headline numbers). The temporary
environment was removed afterward and is not tracked. See the
Milestone 9 end-of-session report for the specific commands and
outcome.

## 6. Documentation number audit

Every headline number appearing in README.md and RESULTS.md was
cross-checked this session against fresh output from the example
scripts and the independent audit above:

`24,142.1 N`, `8 mm` (M2), `19,313.7 N`, `29,258.2 N`, `35,109.8 N`,
`32,509.1 N`, `31,290.3 N`, `10 mm` (M5), M6 `NO_ROBUST_TORQUE_WINDOW`,
`20.1%` epsilon_max, `36.8%` (12 mm epsilon_max), `390.5 g`, `623.9 g`,
`59.8%`, complexity `7.0` vs. `1.0`, `PREFER_B`.

No stale number was found; all match current script output within the
rounding precision documented in Section 3.

## 7. Known intentionally unmodeled items (unchanged from M1–M8)

Thread stripping (M5, `THREAD_CHECK_NOT_MODELED` — no credible
source-verified formula for the thread-geometry parameters available
to this project); torque-angle tightening / turn-of-nut accuracy (M7,
`METHOD_NOT_QUANTIFIED`); detailed instrumentation hardware design,
wiring, and electronics/data-system mass (M8); real procurement cost
(M8, no dollar figure anywhere); fatigue, prying, thermal preload,
embedment/relaxation, nonlinear plate flexibility, proof testing, and
certification/qualification (all milestones). Each is documented in
its own milestone's README section and module docstring; this document
does not repeat the full limitations lists — see README.md.

## 8. Source-audit accuracy check (this session)

Every external source citation in README.md and module docstrings was
reviewed this session for accurate characterization:

- Sources actually fetched and read directly (mechanicalc.com's
  "Bolted Joint Analysis," "Lug Analysis," and "Fastener Size Tables"
  pages; engineeringlibrary.org's NASA preloaded-joint methodology
  page) are labeled as directly read.
- Sources only available as a search-engine summary (a Shigley data
  point in M4, several M5/M6/M7/M8 corroborating figures, the NASA
  Systems Engineering Handbook trade-methodology description in M8)
  are consistently labeled "web-search-derived summary," "not
  independently primary-sourced," or equivalent — never presented as
  directly verified.
- Illustrative material/geometry properties remain labeled
  illustrative throughout; the AISC 360-22 spacing criterion remains
  explicitly flagged as a general structural-steel standard, not
  aerospace-specific.
- No cost claim appears anywhere without a sourced cost model (none
  exists in this project — Milestone 8 uses a normalized,
  non-dollar complexity index instead, by explicit design).
- No preload-verification method is presented as universally accurate;
  each carries its own stated error band and limitation.

No wording corrections were found necessary this session — the
existing milestone documentation already met this standard.
