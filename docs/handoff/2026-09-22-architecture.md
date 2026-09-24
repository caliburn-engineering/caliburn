# Handoff — architecture session, 2026-09-22

Working notes from the session that wrote the root `CONTEXT.md`, `docs/adr/0001`–`0006`
and `projects/ball-balancer/AGENTS.md`. The ADRs hold the **decisions**. This note
holds the **working state** those decisions left behind: findings, the plan, and open
items. Delete it once every workstream below has a spec.

## Next session: grill workstream 2 (physical vs controller model)

Workstream 1 is done: grilled (25 decisions, Q1–Q25), spec published as
caliburn#38, broken into 15 tracer-bullet tickets caliburn#39–#53, all
`ready-for-agent`. The grilling settled several things that touch workstream 2 —
read #38's out-of-scope section first: the "gain against a moved plant" case was
deferred here deliberately as this workstream's mismatch demonstration, ADR-0007
now records that a failed design never reaches the execution layer, and the
observer-Riccati fast-follow (#41's Q24 issue) reuses workstream 1's oracle
apparatus.

Old step-3 procedure, now for workstream 2: read the ADRs in the area, run
`/grilling`, then `/to-spec` and `/to-tickets`, and tick the workstream off here.

## Workstreams, in order

| # | Workstream | Scope | Status |
|---|---|---|---|
| 1 | **Numerical verification** | CARE residual + Kleinman refinement; oracle fixtures (SciPy, slycot/SLICOT, `pydrake`); opt-in live oracle tests (Drake, control-toolbox) behind a CMake option and ctest label; audit Bode, pole-zero, Nyquist, time response, controllability | ✅ Done — spec #38, tickets #39–#53 |
| 2 | **Physical vs controller model** (ADR-0001) | Three artifacts, mismatch indicator, resync; physical parameters drive the physical model *and* the scene | Next |
| 3 | **Portable controllers** (ADR-0002) | Design/execution split, generated gain-set header with provenance, several strategies with a reset before every controller switch | — |
| 4 | **Shared analysis toolkit** (ADR-0005) | Cut `analysis_lib → kinematics`, signal names on `LinearSystem`, channel grids, per-piece pull-down | — |
| 5 | **Golden-source merge-back** (ADR-0004) | Promote ball-balancer's implementations into `reference/`, resolve the `LqrResult` collision, a skill that makes agents check `reference/` first | — |

**Why verification goes first:** workstream 2's payoff is watching the controller
degrade under model mismatch. If the LQR itself is numerically off, you can't tell
mismatch from a bad gain. Workstream 2 unblocks the most (3 and 4 build on it), so it
comes straight after.

**How each workstream runs:** a fresh context window per workstream, running
`/grilling` → `/to-spec` → `/to-tickets`.

## LQR audit

Two implementations, both `caliburn::LqrResult` in the same namespace, with
different fields. They are never linked into the same target today.

### `reference/controllers/lqr.cpp` (golden source): the weaker one

- **`lqr()` builds P from Hamiltonian eigenvectors.** It uses `ComplexEigenSolver` on
  `H = [A, -BR⁻¹B'; -Q, -A']`, keeps the eigenvectors whose eigenvalues have negative
  real part, and sets `P = real(U2 · U1⁻¹)`. Eigenvectors are ill-conditioned when
  eigenvalues cluster, and a Hamiltonian's spectrum is symmetric about the imaginary
  axis, so clustering is common. This is the classic reason a hand-written C++ LQR
  disagrees with MATLAB.
- It uses an explicit `U1.inverse()` rather than a solve.
- It validates nothing: no symmetry or PSD/PD checks on Q and R, no stabilizability
  check, no residual check, no closed-loop stability check. The bare `real() < 0.0`
  test puts eigenvalues near zero on an arbitrary side. If more than n eigenvalues
  test as stable, it silently takes the first n.
- `dlqr()` iterates the DARE by value iteration, which converges only linearly
  (hence `max_iter = 5000`) and can stall on lightly damped systems. It throws on
  failure, whereas ball-balancer returns an error string. The two error contracts
  differ.

### `projects/ball-balancer/src/analysis/lqr.cpp`: the stronger one

- **The algebra is verified correct.** It writes `H = [A, BR⁻¹B'; Q, -A']`, which is
  the textbook Hamiltonian under the similarity `T = diag(I, -I)`, so the stable
  subspace is `[X₁; -X₂]`. The extraction `[W12; W22+I] P = [W11+I; W21]` (least
  squares via SVD) is the correct companion to that sign convention; I worked it
  through. The sign iteration `Z ← (Z + Z⁻¹)/2` is also correct.
- It validates Q (symmetric, PSD) and R (symmetric, PD via Cholesky), checks
  controllability, rejects a non-stabilizing P, and symmetrizes P.
- Gaps, in priority order:
  1. **No CARE residual check.** The closed-loop pole check catches the wrong
     invariant subspace, not an inaccurate right one. Add
     `‖A'P + PA − PBR⁻¹B'P + Q‖ / (‖A'P‖ + ‖PA‖ + ‖PBR⁻¹B'P‖ + ‖Q‖)`, expecting about
     1e-15. It certifies P without any oracle.
  2. **No balancing of H.** Determinant scaling is a single global scalar. The cascade
     plant mixes radians (order 1) with metres (order 10⁻²), which costs digits.
  3. **Determinant scaling can overflow or underflow** on a 14×14 matrix that is still
     workable. Norm scaling `c = sqrt(‖Z⁻¹‖ / ‖Z‖)` avoids that.
  4. **No refinement.** One or two Kleinman/Newton iterations from the stabilizing K
     converge quadratically to machine precision. This is the pragmatic route to
     SLICOT-grade accuracy without Fortran.
- **Unverified:** `checkControllability` probably ranks the Krylov matrix
  `[B AB … Aⁿ⁻¹B]`. At n = 7 with mixed scales that can have a condition number near
  10¹⁵ and give false answers. Read `src/analysis/system_properties.cpp` before
  concluding anything; a PBH/Hautus or staircase test is the sound alternative.

### Not yet audited

Bode (`frequency_response`), pole-zero (`pole_zero`, including the zero pencil and
`matchPoles`), Nyquist, time response (`time_response`), loop locus, and the golden
`pid`, `smc`, `kalman_filter`, `luenberger`. The user doubts these were derived
carefully. Proposed checks that need no oracle:

- Bode from the state space against the analytic transfer function of a known system.
- Step response against the matrix-exponential solution.
- Zeros against a system whose zeros are known by hand.
- Nyquist encirclements against what the criterion predicts from the number of
  unstable open-loop poles.

## Oracle and library notes

- **SLICOT** is the Fortran control library MATLAB's `care`/`lqr` use (`SB02OD`:
  generalized Schur on the Hamiltonian pencil, with balancing). **SVD is LAPACK, not
  SLICOT.** Check SLICOT's licence before depending on it.
- **Easy oracles:** `scipy.linalg.solve_continuous_are` (Schur-based), and
  python-control with `slycot` (SLICOT bindings).
- **`pydrake`** is pip-installable: a second independent oracle with no C++ build.
- **Drake from C++:** Bazel-native and large. Its supported binaries are Ubuntu LTS
  and macOS; the user is on Arch. Treat it as the hardest oracle to wire up.
- **ethz-adrl/control-toolbox:** Eigen-based and template-heavy; wants Boost and
  optionally CppAD, HPIPM/BLASFEO, IPOPT. It looked dormant. Verify activity and
  licence before investing.
- Neither Drake nor control-toolbox builds for Emscripten, and ball-balancer ships to
  the browser. Hence ADR-0003: they are test-only.
- **An idea to borrow for workstream 3:** in Drake, a `System` holds no state; state
  lives in a `Context` passed in. That shape fits the execution layer: stateless,
  testable, no heap.
- **Eigen gap:** `RealSchur` exists but has no reordering (LAPACK `dtrsen`). A Schur
  CARE solver needs reordering, or LAPACK linked into the design layer only (desktop,
  not Emscripten). Sign function + Kleinman + residual avoids this entirely.

## Open items for later workstreams

- **The word "plant" (workstream 2).** The workspace `CONTEXT.md` avoids it in favour
  of physical system / physical model / controller model, but ball-balancer's
  `CONTEXT.md` and code use it everywhere (`AppState::plant`). Settle the terminology
  in that grilling before renaming anything.
- **Scene rebuild (workstream 2).** `PlateView` builds `SimPlate` from
  `cascadeMechanism` once and does not rebuild it when a physical parameter changes.
  The seam that already exists is `handDesignToPlate` → `PlateView::setDesign`, with
  `samePlant` / `gainFitsCascade`, which today refuse a stale design rather than
  showing the mismatch.
- **Physical model derivation (workstream 2).** The cascade model is hand-derived.
  `reference/models/linearizer.cpp` could linearize the simulation numerically to
  check it.
- **Controller portability (workstream 3).** `legCommand` takes `TableKinematics&`.
  `retreatToHoldable` and `holdContactDown` tie constraint handling to the mechanism.
  `auto_balance` sits in `ball_dynamics`, which links `analysis_lib`, which links
  `kinematics`.
- **Docs migration.** New decisions go to `docs/adr/`, or inline in a project's
  `CONTEXT.md` when they attach to a term. `docs/plans`, `docs/specs` and
  `docs/research` stay as a dated archive.
