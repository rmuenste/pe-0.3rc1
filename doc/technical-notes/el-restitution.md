# Restitution in the Euler–Lagrange Contact Solver

`HardContactEulerLagrange` (the unresolved Euler–Lagrange solver behind FeatFloWer's
`q2p1_el_pipeflow`) used to be inelastic: the velocity solve stops the normal approach and the
material restitution `restitution_` was never read. This note describes the restitution switch
ported from `HardContactSemiImplicitTimesteppingSolvers` (HCSITS), what the Euler–Lagrange coupling
changes in the impact law, what review round 1 changed (section 6), the limits, and the tests.

> **On locations.** Symbols, not line numbers (see the note in
> [euler-lagrange-solver-conventions.md](euler-lagrange-solver-conventions.md)), except where a
> review finding is quoted.

## 1. Switch, keys, echo

| json key | type, default | solver call |
|---|---|---|
| `restitutionEnabled_` | bool, `false` | `setRestitution()` |
| `restitutionThreshold_` | real >= 0 (m/s), `0` | `setRestitutionThreshold()` |
| `restitutionScheme_` | `"post-pass"` (default) or `"in-solve"` | `setRestitutionPostPass( scheme == "post-pass" )` |
| `restitutionIterations_` | json integer in [1, INT_MAX], `4` | `setRestitutionIterations()` (minimum sweeps) |
| `restitutionMaxIterations_` | json integer in [1, INT_MAX], `24` | `setRestitutionMaxIterations()` (sweep budget) |
| `restitutionCheckInterval_` | json integer in [1, INT_MAX], `4` | `setRestitutionCheckInterval()` (sweeps between stop tests) |
| `restitutionAbsTolerance_` | real >= 0 (m/s), `1e-4` | `setRestitutionTolerance( abs, rel )` |
| `restitutionRelTolerance_` | real >= 0, `1e-3` | `setRestitutionTolerance( abs, rel )` |

The stop rule (min 4, max 24, check every 4, `1e-4 m/s + 1e-3` of the local impact scale) is
the review-round-2 proposal; regression tests use a strict setting (max 100, check every sweep,
`0 + 1e-9`). The values are collective configuration: every process must load the same deck (the
decisions are global maxima, but each process compares its rows with its own tolerance).
`restitutionThreshold_` is the single rest velocity `v_rest`: approaches slower than
`max( v_rest, 2 |dv_free| )` stay inelastic.

The first four defaults are the HCSITS defaults. Unlike every other key, the eight are resolved **per deck**: an
absent key means its default, never the value an earlier deck of the same process left in the
`SimulationConfig` singleton. They are read and validated together and assigned only if all are
valid (`SimulationConfig::setRestitutionParams()`); the sweep counts must be json
integers (1.9, 4.0, "4", 0, -3, 3000000000 are refused, not converted). The other keys keep the
overlay semantics they had (a later deck without a key keeps the earlier value); some callers
rely on that, e.g. the resume expectation the driver sets before the setup reads the deck.
`restitution_`, `staticFriction_`, `dynamicFriction_` stay the material values.

Which setups apply the switch (`pe/interface/support/optional_collision_params.h`):

| setup | switch |
|---|---|
| `setupELTerminalVelocity` (q2p1_el_pipeflow) | applied: `applyOptionalRestitutionParams()`, echo line |
| `setupELFrozenTrace` (parallel; materials take `restitution_` from the deck) | applied |
| `setupELFrozenTraceSerial` (materials hard-code `e = 0`), `runFrozenFieldTrace`, and every other setup that reads a deck (DKT, DNS drag, Archimedes, Kroupa, fluidization, span, drill, part bench, ATC, rotation, span complex, hash-grid test, lubrication lab) | an explicit `restitutionEnabled_ = true` is refused on every process: `rejectRestitutionRequest()` |

`applyOptionalRestitutionParams()` also refuses `restitutionEnabled_ = true` under a solver
without the switch (`HardContactAndFluid`, the DNS solver). With the keys absent the solver is bit
for bit the solver without the switch: no restitution array is sized, no extra sweep runs, no extra
MPI message is sent, nothing new is printed except the echo line.

The terminal-velocity setup prints one line on the root process after the walls are created
(`pe/interface/support/contact_params_echo.h`):

```
EL_CONTACT_PARAMS pair=particle-particle mu_static=0.3 mu_dynamic=0.3 e=0.9 | pair=particle-wall wall=zwalls mu_static=0.3 mu_dynamic=0.3 e=0.9 | restitution=on threshold=0.01 scheme=post-pass iterations=4 max_iterations=24 check_interval=4 abs_tolerance=0.0001 rel_tolerance=0.001
```

The pair values are the values a contact uses. `Material::getStaticFriction( m1, m2 )` is the SUM
of the two material values, so json `staticFriction_ 0.15` shows as `mu_static=0.3`. The pair
restitution of unlike materials is the square root of the inverse-stiffness-weighted mean of the
squared coefficients; for equal stiffnesses and values it is the material value. A contact also
uses `e = 0` when its normal speed at detection is below 1e-8 (section 5). There is one
`pair=particle-wall` segment per wall kind the setup created (`zwalls`, `boxwalls`, `cylinder`),
or `wall=none`. `restitution=off` means every contact is inelastic; `e=` is then the value a
bounce would use. `threshold=` is the user value; the solver adds its resting bound (section 3).

While the switch is on, the EL stepping (`stepELFrozenTrace()`, `src/interface/cfd_stepping.cpp`)
prints once per CFD step, on the root process (one line):

```
EL_RESTITUTION_STATS pe_step= 1233 cfd_step= 1234 scheme= post-pass candidates= 17 armed= 15 delivered= 14 separating= 15 demoted= 0 in_solve= 0 support= 140323 armed_visits= 120 impulse_gross= 3.1e-07 impulse_final= 3e-07 impulse_fallback= 0 substeps_converged= 9 substeps_capped= 0 substeps_skipped= 1 sweeps_mean_active= 8 sweeps_max= 12 residual= 8e-05 residual_rel= 4e-04 tangential_residual= 1e-05 support_closing_before= 2e-05 support_closing_after= 3e-05 impact_error= 9e-05 newton_shortfall= 0 approach_max= 0.12 resting_approach_max= 0.0098 violations_last_test= 0 support_scale_max= 0.11
```

- `pe_step` is the PE call counter (from 0), `cfd_step = pe_step + 1` the step number the CFD
  audits print; `scheme` is post-pass or in-solve.
- Counts are summed over the 10 sub-steps and over the processes: `candidates` (pre-step approach
  faster than the threshold), `armed` (candidates with compression impulse after the inelastic
  solve: the rows that may bounce), `delivered` (armed rows ending the pass with a positive
  booked restitution impulse: rebounds delivered), `separating` (rows armed at the classification
  that end the pass separating), `demoted` (armed rows whose compression impulse was withdrawn to
  zero, support rows for the rest of the pass), `in_solve` (in-solve Newton targets installed),
  `support` (all other rows), `armed_visits` (summed over sweeps); `impulse_gross` (separating
  increments booked as restitution), `impulse_final` (booked at the end of the passes) and
  `impulse_fallback` (support impulse of demoted rows), all unrelaxed. With the Poisson ceiling on
  every booking `impulse_final <= impulse_gross` and every booked row satisfies
  `lambda_R <= e lambda_C`.
- `substeps_converged / _capped / _skipped` are the stop reasons of the sub-steps;
  `sweeps_mean_active` is the mean sweep count over the sub-steps whose pass ran (skipped
  sub-steps excluded), `sweeps_max` the largest; `violations_last_test` the rows outside the
  tolerance at the last stop test, summed over processes and sub-steps (0 when every pass
  converged). All the same on every process.
- The velocity measures are maxima over sub-steps and processes, in m/s: `residual` (projected
  normal residual on the synchronised velocities at the last stop test) and `residual_rel` (the
  same relative to the row's local impact scale), `tangential_residual` (the tangential
  contact-point velocity change the Coulomb projection would still produce, through the contact
  response; part of the stop decision since round 3), `support_scale_max` (largest local impact
  scale of a support row), `support_closing_before`
  and `_after` (largest closing beyond the allowed approach `-dist/dt` of a support row after the
  inelastic solve and after the pass), `impact_error` (armed rows the cap does not limit) and
  `newton_shortfall` (armed rows the cap limits), `approach_max` (largest pre-step approach of a
  candidate) and `resting_approach_max` (largest approach left inelastic by the threshold, the
  number to calibrate `v_rest` against).

## 2. Impact law and scheme

- **Arming.** A contact with pair restitution `e > 0` is *armed* when its pre-step normal relative
  velocity `vn = n · (v1 - v2 + w1 × r1 - w2 × r2)` approaches faster than the threshold
  (`vn < -threshold`). The pre-step velocity is the velocity stored in the bodies at the start of
  the (sub)step, the Moreau–Jean choice (`restitutionPreStepNormalVelocity()`).
- **Capped post-pass** (default). The velocity solve stays inelastic. `RestitutionPass` then
  classifies the rows from the synchronised post-compression velocities and sweeps them
  (`restitutionPostPassSweep()`, each sweep followed by `synchronizeVelocities()`):
  - an **armed** row (a candidate with positive compression impulse) is driven towards the
    separation speed `e |vn|` (Newton target), its separating impulse held to `e` times its
    compression impulse (Poisson ceiling); the part of an increment that only stops an approach is
    always allowed; only Newton visits of armed rows book restitution impulse. An armed row whose
    compression impulse is withdrawn to zero is **demoted** to a support row (target `g0`) for the
    rest of the pass: with `lambda_C = 0` the ceiling allows no restitution, its later impulse is
    fallback support impulse;
  - every **support** row (all others, any `e`, also a candidate without compression) has a
    unilateral target relative to the post-compression state: `g0` (its normal velocity after the
    inelastic solve) if loaded, `min( g0, -dist/dt )` if free. An undisturbed support network is
    therefore an exact fixed point: a support row that no sweep has visited and whose relative
    velocity is still bitwise the post-compression one is skipped (no impulse, no friction change);
    once visited, a row stays affected for the rest of the pass (its impulse can differ from the
    compression state although its velocity returns to it), so a remote impact changes
    neither its velocities nor its wall loads, and the pass never repairs positions (that stays in
    the compression or split-impulse position solve); only closing introduced by a rebound is
    removed;
  - every visited row gets its Coulomb friction re-relaxed against the new total normal impulse,
    also when the normal increment is zero (a rebound can change the slip of a loaded row without
    changing its load); a row unloaded to zero normal impulse loses its tangential impulse;
  - the pass starts only if a row is armed on some process (else the step is bit for bit the
    inelastic step), runs at least `minSweeps` and then, every `checkInterval` sweeps, tests the
    projected normal AND tangential residual of every affected row on the synchronised
    velocities against `absTolerance + relTolerance * s`, at most `maxSweeps`. `s` is the row's
    local impact scale: the largest impact speed of an armed row on either of its mobile bodies,
    made equal on the owner and every shadow copy before the scales are assigned
    (`exchangeRestitutionBodyScales()`, two messages on the velocity-synchronisation path, only
    when the pass runs); fixed bodies (walls) carry no scale.
- **In-solve Newton target** (`"in-solve"`). The armed contact's target distance becomes
  `dist_ = e vn dt`, unless the contact has a positive gap that the free normal motion of the step
  does not close (`restitutionInSolveDistance()`); then the gap is kept. No cap: several
  simultaneous impacts of one body can gain energy (HCSITS test case 7).

For one contact the rebound normal velocity at the end of the impact step is

```
v_n+ = min( e |v_n-| , e |v_n,free| + (1 + e) p )
```

with `v_n,free` the normal velocity the step gives without the contact (section 3) and
`p = erp * depth / dt` the Baumgarte push-out (0 with split impulse). Under gravity alone
`|v_free| > |v_pre|` and the Newton target binds: `e |v_n-|`.

### Where the code lives (keeping the two solvers in sync)

| Part | HCSITS | HardContactEulerLagrange |
|---|---|---|
| `RestitutionStopRule`, `RestitutionPassStats`, `RestitutionPassWork`, `RestitutionPass` (classification, sweep, residuals, stop rule, diagnostics, refusal of mobile global bodies), `restitutionInSolveDistance()`, `restitutionPreStepNormalVelocity()`, `restitutionCountCandidate()` | `RestitutionPostPass.h` | the same |
| Arming threshold | `max( threshold, 2 |g| dt )`, contact caching | `max( threshold, 2 max |dv_free| )`, `armRestitution()` after the body caching |
| In-solve gap test velocity | free normal velocity after the body caching (`freeNormalVelocity()`) | the same |
| Refusal of a mobile global body | `RestitutionPass::refuseMobileGlobal()` in `simulationStep()` | the same, via `refuseUnsupportedRestitution()` |
| Shadow-correction send filter | exact zero test while the switch is on (approximate `== Vec3()` off) | exact always |

Both solvers declare `friend struct RestitutionPass;` and keep the members documented at the top of
`RestitutionPostPass.h`.

## 3. What the Euler–Lagrange coupling changes

**Implicit drag and the pre-impact velocity.** The driver's hydrodynamic increment
(`elSemiImplicitVelocity()`, drag implicit, gravity, buoyancy and pressure explicit) is folded into
the cached velocity before the contact solve, as HCSITS folds gravity. The Newton target uses the
pre-step velocity. The inelastic solve stops the post-hydro ("free") approach, so
`lambda_c = m_eff ( |v_n,free| + p )` and the Poisson ceiling is `e lambda_c`. If the drag
decelerates the approach during the impact step the ceiling binds (`e |v_free|` plus push-out
terms), if gravity accelerates it the Newton target binds. The difference is the hydro increment of
one sub-step: `O(dt/tau_p)` times the slip velocity for the drag, `O(dt F/m)` for the other forces
(`tau_p = m/B`; the Stokes estimate for case 11, `d = 0.87 mm`, `rho_p = 860` in gas, is about 2 s
against a sub-step of `2.5e-5 s`; it is not a measured dense-bed `m/B`).

**Sub-stepping (10 PE sub-steps per fluid step).** Arming, sweeps and threshold are evaluated per
PE sub-step. The hydro increment of the remaining sub-steps of that fluid step is computed from the
free-flight reference velocity (armed once per fluid step by the driver), the momentum-conserving
design of the drag sub-cycling: the drag reacts to the rebound from the next fluid step on.

**Resting threshold.** `Settings::gravity()` is zero in EL decks (the driver owns gravity), so the
HCSITS bound `2 |g| dt` is replaced per contact by `2 max( |dv_free,1|, |dv_free,2| )`, the larger
free-flight increment of the two bodies in this sub-step (`freeVelocityIncrement()`: the hydro
increment of a hydro body, `F dt / m` of a forced body, the owner's value for a shadow copy, 0 for a
fixed or sleeping body); under gravity alone `2 |g| dt`. This is a **current-step heuristic**: it
bounds the free relative normal increment of two spheres in this sub-step, but not an approach left
over from the previous sub-step (contact residual, overlap correction, a change of the drag at a
CFD update), and for non-spheres it omits the angular free increment at the contact point. A bed
exactly at rest cannot arm (pile test: switch on = switch off, bitwise). A settling bed under EL
drag does arm: in the disordered-bed test (section 7, case 7) 31-147 contacts arm per 400
sub-steps (micro-bounce), without creating kinetic energy beyond the hydro work, with a drift of
3-6e-5 m (0.3-0.6 % of r) against 2-5e-5 m with the switch off. Rule I would use instead (not
implemented, awaiting the owner): arm only if `vn < -max( threshold, 2 |dv_free,now|,
2 |dv_free,previous sub-step|, v_rest )` with `v_rest` a deck velocity below which pairs are
inelastic (granular practice: a fraction of `sqrt(g d)`), so that both a forcing change and the
residual approach of a settling bed stay inelastic. Review round 2 decided against a second key:
`restitutionThreshold_` is that `v_rest`. At the case-11 scale (0.87 mm, 10 sub-steps of 2.5e-5 s,
`pe-el-restitution-rest`) `v_rest = 0.01 m/s` gives a pile under drag carrying 20-90 % of the
weight zero candidates in a late 0.1 s window after 0.8 s of settling, while floor impacts at
0.05-0.2 m/s keep their rebound (ratio 0.900); during settling the largest approach left inelastic
is 0.0091-0.0100 m/s (`resting_approach_max`), so the bound is what keeps the settling bed
inelastic. With EL lubrication on, the lubrication fold enters `v_` before the arming (counted in
`dv_free` and `lambda_c`, not in the Newton target).

**`relaxationParam_` (alpha = 0.9).** In a multi-process run every `synchronizeVelocities()` folds
the corrections with alpha; a single process never does. The post-pass increments are folded like
the PGS impulses, so `p_` keeps one convention (applied impulse `alpha * p_`) and Newton's third
law holds across ranks. For one uncoupled contact whose Newton target binds, the normal velocity
after n sweeps obeys

```
g_n - T = (1 - alpha)^n (g_0 - T)
```

with `T = e |vn|` and `g_0` the velocity after the compression solve (in the two-rank test the
Baumgarte push-out `0.07 |v_rel|`): after 4 sweeps `-0.899917 |v_rel|`, the logged value. A binding
Poisson ceiling or neighbouring contacts change the recurrence; within a process Gauss–Seidel
updates are seen at once, from other processes only at the synchronisation, so results across
decompositions agree to the convergence tolerance, not bitwise (two-rank chain: 1e-14 here). The
pass now continues to the tolerance, which removes the deficit. `contactVirial_` and
`wallContactImpulse_` sum `p_` after the pass and contain the restitution impulse with the same
unrelaxed convention as the compression impulse (the known `1/alpha` defect of these sums is
unchanged).

**Shadow copies and process boundaries.** Each contact is armed and swept on the one process that
treats it. The pre-step velocity of a shadow copy is its owner's (`synchronize()`), its `v_` after
the first synchronisation the owner's post-hydro velocity. The skip test, the stop test and the
sweep count are global (`restitutionGlobalMax()`), so every process calls
`synchronizeVelocities()` the same number of times.

**Fixed and global bodies** (z-walls, box walls, the inner cylinder) have infinite mass and
`dv_free = 0`; their prescribed velocity enters `vn`; local–global contacts are treated on the
sphere owner. A **finite-mass (non-fixed) global body is refused** while the switch is on
(`RestitutionPass::refuseMobileGlobal()`, both solvers, std::runtime_error at the next step on
every process). The test reads the live body storage, so a body fixed and destroyed, a cleared and
repopulated world, or a body unfixed after it was added are all decided by the state at the step.

## 4. Follow-up (not fixed here): synchronisation of non-fixed global bodies

The REGISTRY of these bodies is fixed in round 3 (both solvers): `globalNonfixedBodies_` is rebuilt
from the live body storage at the start of every multi-process step and force synchronisation
(`refreshGlobalNonfixedBodies()`), `remove()` erases a global body whatever its current fixedness,
and `clear()` empties the set. What follows, the double application of the reduced corrections, is
still open.

Review round 1, P0-2 (line numbers of 1d030e7): the non-fixed-global branch of
`synchronizeVelocities()` (`HardContactEulerLagrange.h:4997`, now
`synchronizeVelocities()`, the `globalNonfixedBodies_` block with `MPI_Allreduce`) adds the reduced
corrections `dv_/dw_` to `v_/w_` in full and leaves `dv_/dw_` set, unlike the owner fold that
multiplies by `relaxationParam_` and clears them. Every further synchronisation reduces the same
correction again, and the final integration (`HardContactEulerLagrange.h:2795`, `v_ + dv_`) adds
the treating rank's local remainder, so the replicas of a mobile global body can end with different
velocities. The defect predates restitution (it already applies to the PGS iterations); the
post-pass would add synchronisations to it (`:2705`). Fix: give the reduced correction the same
applied weight as its contact partner and consume it exactly once; test a local sphere hitting a
mobile global sphere on two ranks (momentum, both replicas). Until then the combination is refused.

## 5. Limits

- A contact whose normal speed at detection is below `restitutionThreshold()` (1e-8, `Contact.cpp`)
  has `e = 0`: a pair born touching does not bounce in the step it is born; the post-pass keeps it
  inelastic (support contact). In a chain of touching spheres the support contacts therefore act
  inelastically (3-chain: `B` and `C` leave together at `(1+e) u / 3`).
- The post-pass cost grows with the number of sweeps (4-24 per sub-step with the default rule, each
  with one synchronisation; logged as `sweeps_mean_active` / `sweeps_max`). A sub-step that exhausts the
  budget is counted in `substeps_capped`; its residual is in the log line.
- After a bounce with Baumgarte an overlap left from the impact step can add push-out in the next
  step; the post-pass removes the push-out only in the impact step.
- Not covered by tests: non-spherical bodies under the EL solver, the inner-cylinder wall,
  lubrication with restitution, the in-solve scheme beyond single impacts and the positive gap,
  `runFrozenFieldTrace` (compiled only with MPI and CGAL, not built here), a periodic seam (left to
  the larger FeatFloWer MPI gate: the setup needs three ranks along a periodic axis).
- Observed, unchanged, not verified by a test: in a multi-process run the first
  `synchronizeVelocities()` folds the explicit force increment `F dt / m` of a body WITHOUT hydro
  state with `relaxationParam_` (such bodies receive 0.9 of their force); hydro bodies are not
  affected.

## 6. Review round 1: what changed, also for HCSITS

| Finding | Change | HCSITS |
|---|---|---|
| P0-1 support contacts | the sweep visits every contact (e = 0 included) with the inelastic target; the pass loop converges to a tolerance and skips steps without impacts | yes (shared). `pe-hcsits-restitution` output identical except the flat landing: before `v_z' = 0.7998, |omega'| = 0.0024`, after `0.8000, 0.0000` |
| P1-3 unloading | a contact unloaded to zero normal impulse loses its tangential impulse | yes (shared) |
| P0-2 mobile global body | refused | no (EL only) |
| P1-4 in-solve positive gap | gap kept unless the free motion closes it | yes (pre-step velocity as the closing test) |
| P1-5 / P2-10 config | per-deck defaults, validated together, integer check | - |
| P1-6 setups | applied in the two supporting EL setups, refused elsewhere | - |
| P1-7 test registration | the MPI test decides on the compiled solver (skip 77); deck tests need json | - |
| P2-8 / P2-9 | threshold claim qualified (section 3), recurrence stated and asserted | - |

HCSITS with restitution OFF is unchanged (the pass is not entered).

## 6b. Review round 2: what changed

Before evidence on 97563ed (job 164862), per executable: **runtime** = the executable compiled
against 97563ed, ran and failed; **compile only** = it needs API added in round 2 and did not
compile, so there is no behavioural before result for its checks.

| Finding | Change | Test | Before (97563ed) |
|---|---|---|---|
| 3 support rows (P1) | support targets relative to the post-compression state; undisturbed rows skipped (exact fixed point); pass only with an armed row; no positional repair in the pass | `pe-restitution-support` case 5 (squeezed sphere + pile + remote impact: bitwise unchanged, floor impulse identical) | runtime (both support executables) |
| 4 friction / stop on synchronised state (P1) | friction re-relaxed on every affected row, also at zero normal increment; stop test on the synchronised velocities, projected residual in m/s | `pe-restitution-support` case 6 (orthogonal impact, loaded sticking floor) | runtime |
| stop rule and budget | four keys, defaults min 4 / max 24 / every 4 / 1e-4 m/s + 1e-3; global decisions; per-deck resolution, echo | `pe-restitution-stoprule[-plain]`, `pe-el-restitution-config[-fluid]` | compile only |
| 9 statistics (P2) | candidates / armed / delivered, gross / final impulse, scheme, sub-steps converged / capped / skipped, residuals, `pe_step` + `cfd_step`; only armed rows book restitution | `pe-restitution-stoprule` case 3 (candidate without compression books nothing) | compile only |
| resting bed | `v_rest` = `restitutionThreshold_` only (the threshold existed before; the test fixes the chosen value, it shows no change of physics) | `pe-el-restitution-rest` | compile only |
| 1 refusal (P0) | from the live body storage | `pe-el-restitution` refusal cases b-d | runtime (cases b, c; case d registers the body mobile) |
| 2 / 8 HCSITS under MPI (P0 / P1) | shared refusal; exact-zero send filter with restitution on (off: unchanged) | `pe-hcsits-restitution-mpi` | runtime (momentum and refusal in one executable) |
| 5 inherited switch (P1) | `rejectRestitutionRequest()` applies the (disabled) deck settings | `pe-el-restitution-setup-inherit` | runtime (the `-fluid` variant passes before and after: no switch to inherit) |
| 6 HCSITS gap decision (P1) | in-solve gap decision after the body caching (free velocity) | `pe-restitution-support-plain` case 7 | runtime |
| 7 two-rank chain (P1) | contact midpoints on opposite sides of the cut, treating rank asserted, separation speed asserted | `pe-el-restitution-mpi` | compile only |
| 10 wording (P2) | tools: "minimum sweeps", fixed-count mode; applier diagnostic names the EL solver | - | - |

## 6c. Review round 3: what changed

Before evidence on 45c9268 (job 164883): every test below compiled against 45c9268 (fields added
in round 3 are read through detectors), so "runtime" means the executable ran and the named
assertion failed.

| Finding | Change | Test | Before (45c9268) |
|---|---|---|---|
| 1 global-body registry (P0, pre-existing) | `globalNonfixedBodies_` rebuilt from the live storage at every multi-process step and force synchronisation, `remove()` erases whatever the fixedness, `clear()` empties it; both EL and HCSITS (HardContactAndFluid and DEMSolver untouched) | `pe-global-registry-mpi[-plain]-{fixdestroy,clear,unfix}` (two ranks, restitution off) | runtime for `unfix` (both solvers abort with MPI_ERR_TRUNCATE: the ranks' collectives go out of step); `fixdestroy` and `clear` pass before and after (the stale entry did not change the trajectory here) |
| 2 tangential stop (P1) | tangential residual = tangential contact-point velocity change through the contact response, part of the per-row stop decision | `pe-restitution-pass-unit` case 4 | runtime: converged after 1 sweep with tangential residual 0.129 (tolerance 1e-3), wall contact left slipping at 0.45 m/s |
| 3 skip certificate (P1) | a row stays affected once visited (same predicate in sweep and residual) | `pe-restitution-pass-unit` cases 1, 2 | runtime: impulse kept (5e-2) at g0; friction not re-projected (-0.5) |
| 4 Poisson ceiling on every booking (P1) | armed rows with lambda_C = 0 demoted to support rows; fallback impulse tallied separately; `separating` count | `pe-restitution-pass-unit` case 3 | runtime: lambda_R 0.05 / 0.1 with lambda_C = 0 |
| 5 impact scale (P1) | owner <-> shadow maximum before the row scales; walls carry no scale | `pe-restitution-pass-unit` case 5; `pe-restitution-scale-mpi[-plain]` | runtime for the wall (unrelated row scale 0.9); the split chain's scale is read through a new field (no before value), its sweep count went 16 -> 12 (one-treater layout 8; the sweep counts are not compared: the split rows couple only at synchronisations) |
| 6 trace (P2) | trace variables read on the root and broadcast once; `violations_rank_max` label; violations summed in the stats | - | - |
| 7 records (P2) | this table; rest test with a two-sided rebound band, a drift bound and the speed sampled per sub-step; chain rebound assertion an inelastic chain cannot pass; MPI rule wording; HCSITS and EL setter documentation; preset log header with HEAD and dirty state | `pe-el-restitution-rest`, `pe-restitution-support[-plain]` | test tightenings, pass on 45c9268 too |

## 7. Tests

| Test (label) | What | Result (gcc 13.2, Release) |
|---|---|---|
| `pe-el-restitution` (unit, serial, rigidbody) | 0 defaults; 1 head-on pairs e = 0.9/0.5/0, mass ratios; 2 drop (apex e^2); 3 drop with EL drag; 4 oblique with friction; 5 resting pile (on = off bitwise); 6 chain and corner; 7 disordered bed under EL drag with forcing steps; 8 mobile global body refused | pass; case 7: impacting contacts 122 / 31 / 147 (Baumgarte) and 138 / 36 / 46 (split) in the three phases, energy created by contacts 0, drift <= 6.1e-5 m |
| `pe-restitution-support` / `-plain` (EL / HCSITS) | 3- and 5-chain into touching spheres (split, Baumgarte), drop on a resting pile, frictional unloading, positive gap (both schemes) | pass: most closing support contact -1.5e-10 (5-chain), largest closing pile contact 5.6e-9 at impact 1.18 m/s, x momentum conserved when the floor unloads, no impulse across the gap |
| `pe-el-restitution-config` / `-fluid` | json contract per deck, validation together, integer check, applier (refusal under HardContactAndFluid), echo lines | pass |
| `pe-el-restitution-setup[-fluid]-frozen/-dnsdrag` | serial frozen trace and DNS drag refuse `restitutionEnabled_ = true` before creating a body (EL and HardContactAndFluid) | pass |
| `pe-restitution-pass-unit` (unit, serial) | mock collision system: release at g0, slip re-projection, unload-reload ceiling, tangential stop, wall scale | pass |
| `pe-restitution-scale-mpi[-plain]` (integration, mpi, 2 ranks) | one-treater vs split chain, production rule: support scale e u in both, both converge, velocities within the tolerance | pass: sweeps 8 / 12 |
| `pe-global-registry-mpi[-plain]-*` (integration, mpi, 2 ranks) | registry through fix + destroy, clear + repopulate, registered fixed then unfixed (bitwise against references) | pass |
| `pe-el-restitution-mpi` (integration, mpi, 2 ranks) | pair and coupled chain across the cut against inside one domain; recurrence after 4 sweeps; support contact never closes; mobile global body refused on both ranks | pass: pair bitwise, -0.899917 = prediction, chains 1e-14 apart, 60 sweeps at the chains' impact step (strict rule) |
| `pe-kinematic-body`, `pe-surface-velocity` | detect the semi-implicit solver by type | pass |

Against 1d030e7 (before review round 1) the new cases fail: support contacts close at 0.45 u
(split) / 0.415 u (Baumgarte) in the chains and at 0.287 m/s in the pile, the x momentum changes
by up to 1.4e-3 when the floor unloads, the in-solve scheme bounces across the gap, the config
keeps the previous deck's values and accepts 1.9 / 4.0 iterations, the setups create their bodies
instead of refusing, the mobile global body is not refused, and the two-rank chain's support
contact closes at 0.2075 m/s. Against origin/master 6003b00 (no switch) `pe-el-restitution` fails
23 checks (all contacts inelastic); its resting-pile hash equals the new solver's with the switch
off and on.
