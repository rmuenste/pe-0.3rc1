# Restitution in the Euler–Lagrange Contact Solver

`HardContactEulerLagrange` (the unresolved Euler–Lagrange solver behind FeatFloWer's
`q2p1_el_pipeflow`) used to be inelastic: the velocity solve stops the normal approach and the
material restitution `restitution_` was never read. This note describes the restitution switch
ported from `HardContactSemiImplicitTimesteppingSolvers` (HCSITS), what the Euler–Lagrange coupling
changes in the impact law, the limits, and the tests.

> **On locations.** Symbols, not line numbers (see the note in
> [euler-lagrange-solver-conventions.md](euler-lagrange-solver-conventions.md)).

## 1. Switch, keys, echo

| json key | type, default | solver call |
|---|---|---|
| `restitutionEnabled_` | bool, `false` | `setRestitution()` |
| `restitutionThreshold_` | real >= 0 (m/s), `0` | `setRestitutionThreshold()` |
| `restitutionScheme_` | `"post-pass"` (default) or `"in-solve"` | `setRestitutionPostPass( scheme == "post-pass" )` |
| `restitutionIterations_` | int >= 1, `4` | `setRestitutionIterations()` |

The defaults are the HCSITS defaults. Invalid values throw in the `SimulationConfig` setters while
the deck is read. `restitution_`, `staticFriction_`, `dynamicFriction_` stay the material values.
`applyOptionalRestitutionParams()` (`pe/interface/support/optional_collision_params.h`) pushes the
four keys into the collision system. It is called by `setupELTerminalVelocity()` on every process.
It refuses `restitutionEnabled_ = true` under a solver without the switch (`HardContactAndFluid`).
With the keys absent, the solver is bit for bit the solver without the switch: no restitution array
is sized, no extra sweep runs, and no extra MPI message is sent.

The terminal-velocity setup prints one line on the root process after the walls are created
(`pe/interface/support/contact_params_echo.h`):

```
EL_CONTACT_PARAMS pair=particle-particle mu_static=0.3 mu_dynamic=0.3 e=0.9 | pair=particle-wall wall=zwalls mu_static=0.3 mu_dynamic=0.3 e=0.9 | restitution=on threshold=0 scheme=post-pass iterations=4
```

The pair values are the values a contact uses. `Material::getStaticFriction( m1, m2 )` is the SUM
of the two material values, so json `staticFriction_ 0.15` shows as `mu_static=0.3`. The pair
restitution is a stiffness-weighted mean: for equal stiffnesses and equal material values it is
the material value. There is one `pair=particle-wall` segment per wall kind the setup created
(`zwalls`, `boxwalls`, `cylinder`), or `wall=none` if it created none. `restitution=off` means
every contact is inelastic, and `e=` is then the value a bounce would use. `threshold=` is the user
value. The solver adds its resting bound (section 3).

## 2. Impact law and scheme

The scheme is the HCSITS scheme without changes:

- **Arming.** A contact with pair restitution `e > 0` is *armed* when its pre-step normal relative
  velocity `vn = n · (v1 - v2 + w1 × r1 - w2 × r2)` approaches faster than the threshold
  (`vn < -threshold`). The pre-step velocity is the velocity stored in the bodies at the start of
  the (sub)step, which is the Moreau–Jean choice. It is computed by
  `restitutionPreStepNormalVelocity()`.
- **Capped post-pass** (default, `restitutionScheme_ = "post-pass"`). The velocity solve stays
  inelastic. Then `restitutionIterations_` Gauss–Seidel sweeps (`restitutionPostPassSweep()`), each
  followed by `synchronizeVelocities()`, drive every armed contact towards the separation speed
  `e |vn|` (the Newton target). The separating part of its impulse is held to `e` times the
  compression impulse `lambda_c` that the inelastic solve applied (the Poisson ceiling). The part
  of an increment that only stops an approach is always allowed. Other contacts of the pair (with
  `e > 0`) are swept with the inelastic target. After a normal change, the Coulomb friction of the
  contact is re-relaxed against the new total normal impulse, so the friction bound is
  `mu (1 + e) lambda_c`.
- **In-solve Newton target** (`restitutionScheme_ = "in-solve"`). The armed contact's target
  distance becomes `dist_ = e vn dt`, so the relaxation itself aims at `-e vn`. There is no cap, and
  several simultaneous impacts of one body can gain energy (HCSITS test case 7). It is included
  because it costs one line.

For one contact this gives the following rebound normal velocity at the end of the impact step:

```
v_n+ = min( e |v_n-| , e |v_n,free| + (1 + e) p )
```

Here `v_n,free` is the normal velocity the step gives without the contact (section 3), and
`p = erp * depth / dt` is the Baumgarte push-out of the step's overlap. With split impulse
`p = 0` and the rebound is `e min( |v_n-|, |v_n,free| )`. Under gravity alone `|v_free| > |v_pre|`,
so the Newton target binds and the rebound is exactly `e |v_n-|`.

### Where the code lives (keeping the two solvers in sync)

| Part | HCSITS | HardContactEulerLagrange |
|---|---|---|
| Pass diagnostics `RestitutionPassStats` | `RestitutionPostPass.h` (moved there) | the same |
| Pre-step normal velocity | `restitutionPreStepNormalVelocity()` in the contact caching of `resolveContacts()` | the same function in `armRestitution()` |
| Arming threshold | `max( threshold, 2 |g| dt )` in the contact caching | `max( threshold, 2 max( |dv_free,1|, |dv_free,2| ) )` in `armRestitution()`, after the body caching |
| In-solve target `dist_ = e vn dt` | contact caching | `armRestitution()`, the same line |
| Post-pass sweep | `relaxRestitutionPostPass()` forwards to `restitutionPostPassSweep()` | the same forward |
| Sweep loop + `synchronizeVelocities()` | after the velocity solve, before split impulse | the same place |
| Setters and documentation | `setRestitution()` & co. | the same names. The EL documentation lists the EL specifics |

The sweep is one function template in `pe/core/collisionsystem/RestitutionPostPass.h`. It works on
the solver's contact and body arrays (both solvers declare it a friend), and both solvers use it,
so the impact law exists only once. Only the arming differs, for the reason given in section 3.
Moving the sweep did not change the HCSITS results: the outputs of `pe-hcsits-restitution`,
`pe-split-impulse-plain` and the other tests are identical before and after.

## 3. What the Euler–Lagrange coupling changes

**Implicit drag and the pre-impact velocity.** The driver's hydrodynamic increment
(`elSemiImplicitVelocity()`, with drag implicit and gravity, buoyancy and pressure explicit) is
folded into the cached velocity before the contact solve, as HCSITS folds gravity. The Newton
target uses the pre-step velocity, which is unaffected by the increment. The inelastic solve stops
the post-hydro ("free") approach, so `lambda_c = m_eff ( |v_n,free| + p )` and the Poisson ceiling
is `e lambda_c`. If the implicit drag decelerates the approach during the impact step,
`|v_free| < |v_pre|` and the ceiling binds: the rebound is `e |v_free|` (plus the push-out terms).
If gravity accelerates it, the Newton target binds. The two differ by `O(dt / tau_p)`. In case 11
(`d = 0.87 mm`, `rho_p = 860`, gas) `tau_p ≈ 2 s` against a PE sub-step of `2.5e-5 s`.

**Sub-stepping (10 PE sub-steps per fluid step).** Arming, sweeps and the threshold are evaluated
per PE sub-step, so the bounce happens in the sub-step of the impact. The hydro increment of the
remaining sub-steps of that fluid step is computed from the free-flight reference velocity
(`getEulerLagrangeRefVelocity()`, armed once per fluid step by the driver). That is the
momentum-conserving design of the drag sub-cycling and applies to inelastic contacts as well. The
drag therefore reacts to the rebound velocity from the next fluid step on.

**Resting threshold.** The EL solver does not apply gravity: `Settings::gravity()` is zero in the
EL decks, and the driver owns gravity. The HCSITS bound `2 |g| dt` would therefore be 0. It is
replaced, per contact, by twice the larger free-flight velocity increment
`|dv_free| = |v_ + dv_ - v_body|` of the two bodies (`freeVelocityIncrement()`), evaluated after
the body caching. This is the hydro increment of a hydro body, `F dt / m` of a body driven by
forces, the owner's value for a shadow copy, and 0 for a fixed or sleeping body. Under gravity
alone it equals `2 |g| dt`. A body at rest under gravity has a pre-step approach of at most about
that value, so a packed bed does not micro-bounce. The Baumgarte push-out at rest is
`0.7 g dt`, below the bound. Because the threshold needs the cached velocities, the EL solver arms
after its body caching, whereas HCSITS arms in the contact caching. With EL lubrication on
(`lubricationEnabled_`), the lubrication fold enters `v_` before the arming. It counts in `dv_free`
and in `lambda_c`, not in the Newton target.

**`relaxationParam_` (0.9).** In a multi-process run every `synchronizeVelocities()` folds the
corrections with `relaxationParam_`. A single process never does, because the function returns
early. The post-pass increments are folded like the PGS impulses, so `p_` keeps one convention:
the applied impulse is `relaxationParam_ * p_` in a multi-process run, and Newton's third law
holds across ranks. Each sweep re-targets on the folded velocity, so `n` sweeps deliver
`1 - 0.1^n` of the Newton bounce (0.9999 with 4, measured 0.899917 / 0.9 in the two-rank test). The
accumulated diagnostics `contactVirial_` and `wallContactImpulse_` sum `p_` after the pass. They
contain the restitution impulse (and the re-relaxed friction) with the same unrelaxed convention as
the compression impulse. The known defect is that these sums are `1 / relaxationParam_` times the
applied impulse in multi-process runs. It is unchanged, neither fixed nor made worse.

**Shadow copies and process boundaries.** Each contact is armed and swept on the one process that
treats it (the contact filtering of `resolveContacts()`). The pre-step velocity of a shadow copy is
its owner's, sent by `synchronize()`. Its `v_` after the first synchronisation is the owner's
post-hydro velocity, so `dv_free` agrees too. The corrections reach the owner as the PGS
corrections do. The sweep count is a solver setting, so every process calls
`synchronizeVelocities()` the same number of times. The switch must have the same value on every
process, which the setup guarantees by applying the deck everywhere. A pair across a cut bounces
exactly like the same pair inside one domain (two-rank test, difference 0).

**Fixed and global bodies** (z-walls, box walls, the inner cylinder). They have infinite mass and
`dv_free = 0`. Their pre-step velocity is the prescribed velocity, so a moving Couette wall
enters `vn` correctly. Local–global contacts are treated on the sphere owner. The wall's `e` and
`mu` come from the pair tables like any other pair (echo line).

## 4. Limits

- A contact whose normal velocity at detection is below `restitutionThreshold()` (1e-8, in
  `Contact.cpp`) gets `e = 0`. A pair that starts touching therefore bounces only from the next
  step on, and the post-pass skips it in the step it is born. This is the pe rule and holds in
  HCSITS as well. The three-sphere chain test exercises it.
- The sweep count is fixed and not iterated to convergence (the HCSITS documentation shows the
  effect of 2 against 4 sweeps on a flat box landing). Multi-process runs carry the relaxation
  deficit `0.1^n`.
- After a bounce with Baumgarte (`erp 0.7`, no split impulse), an overlap left over from the impact
  step can add push-out in the next step when `e` is small (`e < 0.7 * remaining overlap / dt`).
  The post-pass removes the push-out only in the impact step.
- The frozen-trace setup (`el_frozen_trace.h`) hard-codes `e = 0` for its materials and does not
  read the switch. Only the terminal-velocity setup (`q2p1_el_pipeflow`) applies and echoes it.
- Not covered by tests: non-spherical bodies under the EL solver, the inner-cylinder wall (a curved
  wall with a contact normal through the axis, which uses the same code path as the planes),
  lubrication together with restitution, and the in-solve scheme beyond a single impact.
- Observed while porting, unchanged, and not verified by a test: in a multi-process run the first
  `synchronizeVelocities()` folds the explicit force increment `F dt / m` of a body WITHOUT hydro
  state (it sits in `dv_`) with `relaxationParam_`, so such bodies receive 0.9 of their force.
  Hydro bodies fold their increment into `v_` beforehand and are not affected.

## 5. Tests

| Test (label) | What | Result (gcc 13.2, Release) |
|---|---|---|
| `pe-el-restitution` (unit, serial, rigidbody) | head-on pairs, e = 0.9/0.5 (Baumgarte and split), e = 0 (split), mass ratios 3 and 0.32 | `v_rel'/v_rel = -e`, momentum, and `E_rel'/E_rel = e^2`, all to 1e-12. e = 0 with the switch on is bitwise the switch off. The in-solve scheme gives the same single impact |
| | drop on the floor, gravity as a force, e = 0.9 | apex ratios 0.8099 / 0.8100 / 0.8098 (e^2 = 0.81), 61 rebounds, `E_max/E_0 = 1.000000`, at rest after 4 s |
| | drop with the EL drag (`tau_p` = 20 fluid steps, 10 sub-steps), Baumgarte and split | rebound = `min( e|v_pre|, e|v_free| + (1+e) p )` to 1e-9: 0.620262 = e |v_pre| (Baumgarte), 0.618054 = e |v_free| (split, ceiling binds) |
| | oblique impact, mu = 0.3 per pair | sliding: `p_t/p_n = 0.300000000000`, `p_n = (1+e) m |v_z|`. Sticking: contact point at rest, `p_t/p_n = 0.030` |
| | resting pile (pyramid in a box with a column on the apex), 5000 steps | max \|v\| 4.2e-6 (bound 2 g dt = 1.96e-3), drift 1.5e-7. Switch on, switch off and off-after-on give the same state hash (no contact of a resting pile is armed) |
| | three-sphere chain, sphere into a corner | `E_max/E_0 = 1` (no gain), momentum to rounding, corner rebound (0.45, 0, 0.45) = e u in both components |
| `pe-el-restitution-config` (unit, serial, interface) | json contract, validation, applier, exact echo lines | pass |
| `pe-el-restitution-mpi` (integration, mpi, 2 ranks) | EL setup from a deck, pair across the cut against the same pair inside one domain | bitwise equal, `v_rel'/v_rel = -0.899917` (Newton within the relaxation deficit) |
| `pe-kinematic-body`, `pe-surface-velocity` | now detect the semi-implicit solver by type. They used the restitution switch as its marker, which the EL solver now has too | pass |

Compiled against the solver without the switch (origin/master 6003b00), the `pe-el-restitution`
source fails 23 checks, because every contact stays inelastic. For example, the head-on rebound is
the Baumgarte artefact `-0.07 v_rel` instead of `-0.9 v_rel`, the first drop apex ratio is 0.21,
and the corner rebound is 0.035 instead of 0.45. Its resting-pile state hash (`b9b08ddda33abc12`)
equals the hash of the new solver with the switch off and on. The default path of the pile is
therefore bitwise the solver before the switch. The full presets are unchanged otherwise: every
test common to both trees prints the same output, apart from temporary paths, pids and the
interleaving of MPI output.
