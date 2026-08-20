# Lubrication as a switchable add-on component

Implements §3 of the D2.2 design spec
(`applications/q2p1_dns_drag/validation_cases/d22_lubrication/DESIGN_SPEC.md`):
turn PR #23's lubrication into a switchable C++ add-on usable by any collision
pipeline, and retire the isolated `HardContactLubricated` pipeline.

> **This PR stacks on `fix/lubrication-merge-resolution`.** That branch repairs
> the broken conflict resolution in merge 068c8c2 (three compile breaks and one
> runtime break); `master` does not compile without it. Merge the fix PR first,
> or review this one against that base — it is the parent of every commit here.

Branch: `feature/lubrication-addon`, based on `fix/lubrication-merge-resolution`
(`afc0564`).

---

## What changes, in one paragraph

Lubrication used to be a property of *one solver*. Selecting
`pe_CONSTRAINT_SOLVER=HardContactLubricated` at build time got you a
lubrication force loop, a contact type that could carry a lubrication tag,
detection branches switched on by a macro that solver self-defined, and AABB
inflation. Every other solver — including `HardContactAndFluid`, which is what
the DNS campaign actually ships — got none of it, while still honoring the
`lubricationEnabled_` json key enough to change its AABBs. After this PR
lubrication is a component: a closure, an application stage, and per-contact
state, all shared; any hard-contact pipeline opts in with one call; and the
json switch is the single control point for detection, inflation and force
together. The default is off, and off is provably free.

---

## Commits

Each builds standalone (verified, see below) and each is one concern.

| # | Commit | Concern |
|---|---|---|
| 1 | `dd6d962` | Extract the lubrication application stage into a shared component |
| 2 | `d06409c` | AABB padding tracks the EFFECTIVE lubrication cutoff |
| 3 | `ed7d74e` | Promote the detection gate from compile-time macro to runtime switch |
| 4 | `d4b61a6` | Hook the lubrication stage into `HardContactAndFluid` |
| 5 | `4a859ff` | Refuse loudly when `lubricationEnabled_` reaches a solver that cannot honor it |
| 6 | `c3a3c12` | Give every stage-capable pipeline real per-contact lubrication state |
| 7 | `9545c9f` | Retire the `HardContactLubricated` pipeline |
| 8 | `8eca829` | Add serial-mode coverage test for the lubrication add-on |
| 9 | `245b53b` | Pin AABB neutrality: lubrication off must not inflate bounding boxes |
| 10 | `a71c34d` | *(pre-existing)* Remove the unconditional angular-velocity reset |
| 11 | `fd3bafe` | **review** Fix out-of-bounds indexing on the lubrication-enabled path |
| 12 | `4273f98` | **review** AABB padding: compose the two bands, never override |
| 13 | `234c295` | **review** Inline the detection gate so the disabled path stays a single branch |
| 14 | `8ead8c9` | **review** Add a test for the refusal guard |
| 15 | `044b04a` | *(pre-existing)* Remove the hardcoded velocity limiters |
| 16 | `77f7a12` | **review** Docs: record the retired solver as history, fix dead build recipes |
| 17 | `64d0d91` | **review** Correct stale comments and tighten the test suite |

Commits 11-14, 16-17 answer the independent review (see "Review response").
Commits 10 and 15 are pre-existing defects unrelated to lubrication (see
"Pre-existing defects removed in passing").

Commit 6 is not in the spec's list. It exists because commits 4 and 5 were not
sufficient on their own — see "Findings" below. It is the commit a reviewer
should read most carefully.

### 1. Extract the stage — `pe/core/lubrication/LubricationStage.h`

`HardContactLubricated.h:2221-2472` (config snapshot → loop tagged contacts →
apply impulses) becomes a free function template:

```cpp
lubrication::applyLubricationStage( contacts, contactsMask_, v_, w_, dv_, dw_, dt );
```

Every hard-contact pipeline in pe caches `(v, w, dv, dw)` indexed by
`RigidBody::index_` and masks its contact list the same way, so the stage
composes with all of them. The `isEnabled()` guard stays with the **caller**, so
a disabled build pays one predictable branch and never reaches the extra
velocity synchronization (an MPI collective).

Two solver-held scalars moved with it: `minEpsLub_` and `alphaImpulseCap_` are
now mirrored in the `pe::lubrication::` store alongside the twelve switches
already there. Defaults (`1e-8`, `1.0`) match the former ctor initializers.

`HardContactLubricated` was pointed at the extracted stage *in this commit*,
deliberately, so its three solver-gated tests act as the fidelity check on the
extraction before the pipeline is retired. All three passed.

### 2 + 9. AABB padding = effective cutoff, and provably zero when off

Spec §3.4. `aabbPadding()` mirrors `lubricationCutoff()` exactly, mesh clamp
included. Dropping the old "padding is a strict superset of the force band"
property is safe: both partners carry the padding, so AABBs overlap at
`gap < 2·cutoff` while the force band is `gap < cutoff` — a 2× detection margin
remains.

`pe_lubrication_model_test` asserted the superseded intent
(*"mesh clamp NOT applied to padding"*) and is re-pointed at the approved rule.

**No silent inflation when lubrication is off — enforced, not argued.** This is a
hard requirement: bounding-box size feeds the HashGrids broad phase, so it
selects the grid hierarchy level and hence candidate-pair enumeration order,
which perturbs contact ordering in the sequential solver. Stray padding is a
silent trajectory change in every DNS case.

The regression is historical fact. Before the refactor,
`SphereBase::calcBoundingBox` added `getLubricationThreshold()`
**unconditionally** — a hard-coded `1e-2` in *absolute* units, under every
solver, whether or not lubrication existed in the build. On a 1.5 mm benchmark
sphere that is a **10 mm halo**: padding several times the body radius,
permanently on, invisible.

`pe_lubrication_aabb_neutrality_test` (commit 9) pins it. It runs in its own
process, so the parameter store is pristine, and links the campaign solver —
i.e. exactly the "`HardContactAndFluid` + shipped defaults + lubrication off"
configuration:

```
[defaults] isEnabled=false securityZone=false aabbPadding(R)=0
[defaults] sphere half-extent = 0.00150001 (R + contactThreshold = 0.00150001)
[defaults] plane  face        = -0.049999990000000001 (d + contactThreshold = -0.049999990000000001)
[enabled ] sphere half-extent = 0.0022500100000000002 (grew by 0.00075 = cutoffFactor*R)
```

Asserted with **bitwise** equality, not a tolerance: defaults neutral (switch
off, security zone off, contact generation off); `aabbPadding()` exactly zero
across radii `1e-6 … 1e3`; sphere half-extent exactly `radius + contactThreshold`
and plane face exactly `d + contactThreshold`; the json path neutral too
(pushing a *default* `SimulationConfig` through `applyOptionalLubricationParams`
— the call every setup makes — arms nothing and leaves boxes bitwise unmoved,
and pins both hysteresis half-widths at zero); an explicit check that the `1e-2`
halo specifically is gone; and non-vacuity — enabling *does* inflate, the mesh
clamp *does* bound it, disabling restores the neutral box bitwise.

**Verified by mutation:** reintroducing the old unconditional padding fails 11
of the checks; reverting restores a clean pass. The gate bites.

Two structural facts underpin the guarantee, both verified by grep over the
whole library: `aabbPadding()` has exactly **two** call sites
(`SphereBase.h:222`, `PlaneBase.cpp:140`) and no other body type pads at all;
and `setSecurityZone(true)` — the one branch that can return a nonzero padding
while the master switch is off — has exactly **one** call site, in
`CollisionSystem<C<...,ShortRangeRepulsion>>`'s constructor, which is never
instantiated under `HardContactAndFluid`.

### 3. Runtime detection gate

`#ifdef PE_LUBRICATION_CONTACTS` is deleted, along with both self-defines and
the commented-out propagation block in `Collisions.h`. Both branches
(`collideSphereSphere`, `collideSpherePlane`) now take an early-return fast path
gated on `lubrication::contactGenerationEnabled()`.

The fast path is the pre-add-on hard-contact test verbatim, and is
bitwise-identical to the previous disabled behavior: the old `#ifdef` arm
computed `hardWeight = computeHardWeight(dist, contactThreshold, delta)`, which
degenerates to exactly `dist < contactThreshold` when `delta == 0` — which
commit 5 enforces whenever lubrication is off.

### 4–5. Hook and refusal guard

The hook goes immediately after the first `synchronizeVelocities()` in
`resolveContacts`. Capability is an explicit opt-in marker:

```cpp
static constexpr bool hasLubricationStage = true;
```

`applyOptionalLubricationParams` throws `std::runtime_error` when
`lubricationEnabled_` is true and the active solver lacks it. Deliberately a
marker, not a structural probe: whether a solver *calls* the stage is not
observable from its type, and guessing is the exact failure mode the guard
exists to prevent.

### 7. Retirement

Four headers deleted plus registry entries in `Solvers.h`, `Types.h`,
`Configuration.h`, `ParallelTrait.h`, `RigidBodyTrait.h`, `CollisionSystem.h`
and the `ContactTrait` specialization. `Collisions.h` keeps the neutral
`HardContactEulerLagrange` default from the fix branch — retirement removes a
solver, it does not change which one is selected.

---

## Findings that changed the implementation

These are the things the spec could not have anticipated. Each is why some
commit looks different from the plan.

### A. The stage alone applies zero force — per-contact state was solver-private

After commits 4–5 the hook was in place, the switch was on, and the measured
resistance through `HardContactAndFluid` was **exactly zero**.

Only `ContactTrait<C<...,HardContactLubricated>>` actually *stored* the
lubrication flag and weight. Every other specialization — `HardContactAndFluid`
included — carried no-op stubs whose `getLubricationFlag()` returned `false`.
Detection called `setLubricationFlag()`, the value was discarded, the stage
skipped every contact. **The stubs compile.** Nothing warned.

Fixed in commit 6 by extracting the state the same way as the stage:
`pe/core/lubrication/ContactState.h`, with `lubrication::ContactState` for
stage-capable traits and `lubrication::NullContactState` for the deliberately
inert ones — so "no lubrication here" is a documented choice rather than
something indistinguishable from an oversight.

**Second half of the same commit:** the hook also had to exclude lubrication
pre-contacts from the hard-contact constraint solve. Those pairs carry a
*positive* surface gap and are not constraints. `HardContactLubricated` filtered
them explicitly; the hook had not carried that over, so they were being resolved
as if the bodies were touching.

### B. `HardContactAndFluid` discarded angular velocity — a debugging artifact, now removed

> **Read this even if you skip the rest. It is not a lubrication issue.**

`HardContactAndFluid::integratePositions` contained, with no guard and no
comment, placed after the translational update and before the rotation angle is
formed:

```cpp
w = Vec3(0,0,0);
```

`body->w_` was then overwritten with zero. **Under this solver no body could
rotate, ever.**

I initially recorded this as a long-standing modelling choice and left it alone.
That was wrong: the owner identifies it as a debugging artifact from a run where
angular velocity blew up. It is removed in commit `a71c34d`, which makes
`integratePositions` identical to the plain hard-contact implementation apart
from the separate (retained) application velocity limiter. It was the only
angular suppression in the file — the other `setAngularVel` call sites are MPI
sync paths restoring received values.

**Campaign implication.** `HardContactAndFluid` is the solver the DNS campaign
ships. Any result depending on particle rotation was produced with rotation
pinned at zero. In particular the draft-kiss-tumble benchmark: **a DKT run under
this solver could not tumble, because tumbling is rotation.** The recorded
*"tilt onset then stall at 4.2°, full tumble open"* is exactly what a
translation-only pipeline produces. This line is a candidate explanation that
has nothing to do with mesh resolution, timestep or contact parameters, and it
should be checked before other explanations are pursued.

Consequences inside this branch:

- lubrication sliding and twisting torques are now actually applied under the
  campaign solver, where before they were computed and thrown away;
- `pe_lubrication_bounce_stokes_test` — the torque assertions — now runs under
  `HardContactAndFluid` and passes, which is direct evidence rotation works
  rather than merely being un-zeroed;
- `pe_lubrication_serial_coverage_test` gains an explicit assertion that a
  freely spinning body keeps its spin (`[rotation] free spin w_y after one step
  = 3 (set to 3)`), so the artifact cannot creep back silently.

**Risk note.** If the original blow-up recurs, the principled tool is the
adaptive Baumgarte capping already present in this solver
(`setAdaptiveBaumgarteCapping` / `useAdaptiveBaumgarteCapping_`), or an explicit
angular limiter alongside the existing linear one — not a silent reset.

### C. Per-target solver selection is impossible in pe

The spec's "no skips under the neutral default" cannot be met with
`target_compile_definitions`, which was my own earlier recommendation on the fix
branch and is **wrong**. `pe_CONSTRAINT_SOLVER` feeds the global `Config`
typedef, and non-template library units are compiled against it — e.g.
`MPICommunication`, whose constructor takes `ProcessStorage<Config>&`. Mixing
solvers between library and consumer yields undefined references, not a working
binary. Verified by trying it.

The tests therefore link purpose-built library variants (`EXCLUDE_FROM_ALL`, so
they cost nothing unless testing is on):

| variant | solver | used by | why |
|---|---|---|---|
| `pe_static_lubstage_fluid` | `HardContactAndFluid` | legacy regression, bounce-Stokes, serial coverage, AABB neutrality | the campaign solver; carries buoyancy → reproduces the legacy reference trajectories |
| `pe_static_lubstage_plain` | `HardContactSemiImplicitTimesteppingSolvers` | two-sphere approach | the neutral pipeline; keeps the "works on any pipeline" claim tested |

The solver is forced via compile **options**, not definitions: CMake emits
`$(CXX_DEFINES)` before `$(CXX_FLAGS)`, so a user's
`-DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=..."` would otherwise silently
override a variant and give it the wrong pipeline. This was a real observed
failure, not a hypothetical.

**Cost:** two extra library compilations when `BUILD_TESTING=ON`. That is the
price of "never skips" given pe's architecture. Flagging it as the one place a
reviewer might reasonably prefer a different trade.

### D. `ShortRangeRepulsion` is a co-tenant of the detection branch

SRR reuses the same `addLubricationContact` channel for its Pan et al. security
zone, sets the legacy absolute threshold to its `rho`, and needs those
pre-contact pairs **whether or not lubrication is on**. Gating purely on
`isEnabled()` — as the spec says — would have silently disabled SRR's security
zone, since the fix branch defaults the switch to off.

The gate is therefore the union:

```cpp
inline bool contactGenerationEnabled() { return isEnabled() || getSecurityZone(); }
```

`setSecurityZone(true)` is declared by SRR's own constructor, never from json,
and `applyOptionalLubricationParams` deliberately does not touch it. `aabbPadding()`
honors the same flag. **This is a documented deviation from the spec's
`isEnabled()` wording** — see the deviations table.

Side effect worth noting: `ShortRangeRepulsion.h` defined
`PE_LUBRICATION_CONTACTS` *unconditionally* and is included unconditionally from
`CollisionSystem.h`, so the macro was in fact defined in **every** build
regardless of solver. The "compile-time gate" was never really a gate. Removing
it also removes a documented include-order fragility (`CollisionSystem.h`
carried a warning that these headers had to be included first or an earlier
`MaxContacts.h` include would lock in the wrong path through the include guard);
a runtime branch cannot have one.

---

## Review response

An independent review returned REVISIONS REQUIRED. Disposition of every finding:

| # | Finding | Disposition |
|---|---|---|
| 1 | Out-of-bounds indexing on the ON path (`HardContactAndFluid.h`) | **Fixed** `fd3bafe`. The dead "translation-locked" debug block indexed `body1_`/`body2_` with the unfiltered count while commit `c3a3c12` had resized them to the filtered one. Deleted rather than re-indexed — it was dead code with a live bug, surviving only because `-O2` eliminated it. Verified the other loops all take their bound from `p_.size()`; the plain host never had the block. |
| 2 | `securityZone` *replaced* the lubrication padding | **Fixed** `4273f98`. Now the **max** of the two bands. Under-inflation here is a missing interaction with no diagnostic, not a safe failure. Composition semantics pinned by five new assertions in the neutrality test. |
| 3 | Detection gate was two out-of-line calls per candidate pair | **Fixed** `234c295`. `isEnabled()`/`getSecurityZone()` are now extern flags with inline accessors; writes still go through the out-of-line setters. Verified by compiling `contactGenerationEnabled()` to assembly at `-O2`: **zero calls emitted**. |
| 4 | Refusal guard had no test | **Fixed** `8ead8c9`. Links the **shipped** `pe_static`, asserts the contract (`throws ⇔ enabled && !stage-capable`) so it stays meaningful in both CI configurations, and additionally pins that a refusal leaves the store *unarmed*. |
| 5 | Docs canonized the retired solver | **Fixed** `77f7a12`. Five documents; descriptions kept as provenance, every *instruction* corrected — including the `-Dpe_CONSTRAINT_SOLVER=...HardContactLubricated` recipe that can no longer configure, and a `lubricationEnabled_` default documented as `true` when it is `false`. |
| 6 | Stale comments | **Fixed** `64d0d91`. The `target_compile_definitions` claim (which contradicts Finding C three lines below it) and the nonexistent `pe_static_lubstage` target name; each test header now names the variant **and solver it actually links** — load-bearing, since bounce-Stokes and two-sphere deliberately use different ones. |
| 7 | `numContacts_` reported pre-contacts to FeatFloWer | **Fixed** `fd3bafe`. Reports `numContactsMaskedHard`; identical whenever lubrication is off. |
| 8 | Disclose the true disabled-path cost | **Done**, below. |
| 9 | `StageDiagnostics` computed and discarded, invariant Release-invisible | **Fixed** `64d0d91`, by a different route than suggested: asserted as its *physical* consequence (kinetic energy cannot increase) rather than by reading the diagnostics. There is no public contacts accessor, and the physical form cannot pass while the applied impulses disagree with the reported number. A second assertion checks the energy strictly *decreases*, so it cannot pass vacuously. |
| 10 | `VelocityBuffers` struct; `std::vector` narrows "any pipeline" | **Noted, not done** — see Known limitations. |
| 11 | Params store threading contract | **Fixed** `64d0d91`. Documented setup-time/single-threaded, and called out `registerSphereRadius()` from `SphereBase`'s constructor as the one write outside setup — a data race under concurrent body creation, which matters because FeatFloWer builds with OpenMP. |
| 12 | Example cleanups, CMake duplication | **Partly done** `64d0d91`: the dropped `setSlipLength`/`setMinEps` no-ops are documented (with the warning that `setEpsCritical` is *relative* and not the same quantity), two `-Wunused-parameter` warnings silenced. Routing `pe_static` through `pe_add_lubstage_library` **not done** — noted below. |

### Disabled-path cost, stated plainly (finding 8)

With lubrication off, versus the pre-add-on library:

- **one branch per timestep** per stage-capable solver (the `isEnabled()` guard
  around the stage call and its extra `synchronizeVelocities()`);
- **one inlined branch per candidate pair** in fine detection — after `234c295`
  a predictable test on a hot global, not a call. This is the only cost in a
  loop that scales with the broad phase, and it is where the retired macro cost
  literally nothing;
- **+16 bytes per contact** on stage-capable hosts, from `lubrication::ContactState`
  (a `bool` and a `real`) embedded in `ContactTrait`. Solvers using
  `NullContactState` are unchanged — it is an empty base.

Not free, then, but bounded and disclosed. The AABB path is *cheaper* than
before, because the unconditional `1e-2` padding is gone.

### Known limitations (finding 10, 12)

- The stage signature takes four adjacent `std::vector<Vec3>&`. That is the
  shape every current pe hard-contact pipeline already stores, but it does
  narrow "usable by any collision pipeline" to "any pipeline that happens to use
  `std::vector`". A `VelocityBuffers` view struct would decouple it. Deferred:
  it touches the signature this PR is trying to establish, and there is no
  second storage shape in-tree to justify it yet.
- `pe_static` is still defined separately from the `pe_add_lubstage_library()`
  helper, so the library definition exists twice. That duplication is what
  produced the compile-options-vs-definitions bug fixed during this work.
  Routing `pe_static` through the same helper would remove the class of bug;
  deferred as a build-system change with blast radius beyond lubrication.

---

## Pre-existing defects removed in passing

Two defects in `HardContactAndFluid` that **have nothing to do with
lubrication**. Both predate this branch (present at the campaign submodule pin
`5f90e7c`), both are owner-classified as stable debug scaffolding, and both
silently mutated physics. Separated out here so they can be judged on their own
terms — and, if wanted, reverted independently of the add-on work.

### `a71c34d` — angular velocity was reset to zero every step

`integratePositions` contained, unguarded and uncommented:

```cpp
w = Vec3(0,0,0);
```

`body->w_` was then overwritten with it. **Under this solver no body could
rotate, ever.**

**Campaign implication, and the reason this matters more than the rest of the
PR:** `HardContactAndFluid` is the solver the DNS campaign ships. Any result
depending on particle rotation was produced with rotation pinned at zero — in
particular the draft-kiss-tumble benchmark, where *tumbling is rotation*. The
recorded *"tilt onset then stall at 4.2°, full tumble open"* is exactly what a
translation-only pipeline produces. This is a candidate explanation that has
nothing to do with mesh resolution, timestep or contact parameters, and it
should be checked before others are pursued.

Removing it makes `integratePositions` identical to the plain hard-contact
implementation. Pinned by an assertion in the serial coverage test
(`[rotation] free spin w_y after one step = 3 (set to 3)`), and it is why the
torque test now runs under the campaign solver.

### `044b04a` — hardcoded dimensional velocity limiters

```cpp
// Velocity limiting. We know the fluid speeds in the current application.
if (body->v_.length() > 12.3) { body->v_.normalize(); body->v_ *= 4.1; }
```

pe has no unit system, so this silently caps translational velocity in any case
whose speeds exceed `12.3` in whatever units the deck uses, and rescales them to
an unrelated `4.1`. A unit-system landmine, not a stability mechanism. No
certified result is affected — campaign speeds never approached it in the units
used, so it never fired.

A **second** limiter of the same species, not named in the review, was found in
the velocity-caching block: `|v| > 32.0` renormalized to `12.3`, while its own
log line claimed *"clamped to 2.5"* — three mutually inconsistent constants in
one block. Removed too; scope expansion flagged explicitly here. The genuine
stuck-particle and high-velocity **diagnostics** are kept, with `32.0` now
purely a reporting threshold that changes no state.

**Risk note for both.** If a real instability surfaces, the principled tools are
the adaptive Baumgarte capping already in this solver, a properly dimensioned
CFL-based limit derived from the deck, or a loud error — not a silent reset or
rescale to a magic constant.

---

## Deviations from the spec

| # | Spec says | Implemented | Rationale |
|---|---|---|---|
| 1 | §3.3 gate detection on `lubrication::isEnabled()` | Gated on `isEnabled() \|\| getSecurityZone()` | SRR co-tenants the branch for its security zone and must keep it with lubrication off. Finding D. |
| 2 | §3.2 hook `HardContactAndFluid` | Hooked `HardContactAndFluid` **and** the plain hard-contact pipeline | §1 asks for "usable by any collision pipeline"; a claim with no test on a second pipeline is untested. Both variants now carry tests. |
| 3 | §3 (implied) stage extraction is sufficient | Also extracted per-contact state (`ContactState.h`) | Without it the stage runs and finds nothing. Finding A. |
| 4 | §3.6 tests run "under a stage-capable solver … no skips under the neutral default" | Achieved via two dedicated library variants, not per-target defines | Per-target selection cannot work in pe. Finding C. |
| 5 | §3.7 "one serial-mode + CFD-coupled test" | Serial-mode test only | The CFD-coupled half needs the FeatFloWer wire (§4), explicitly out of scope for this PR. Stated in the test header. |
| 6 | — | `aabbPadding()` returns the absolute threshold when a security zone is active | Same reason as #1; SRR's AABBs must not collapse. |

Nothing in the spec was silently adapted; every departure is above.

---

## Verification

### Build + test matrix

| configuration | build | ctest |
|---|---|---|
| Neutral default (`HardContactEulerLagrange`), Release | 0 errors | **12/12 passed, 0 failed, 0 skipped** |
| `-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid`, Release | 0 errors | **12/12 passed, 0 failed, 0 skipped** |
| **Debug + `-D_GLIBCXX_ASSERTIONS`** | 0 errors | 11/12 — all lubrication tests pass; one **pre-existing** unrelated failure, see below |

The Debug/assertions build is the lesson of review finding 1: an out-of-bounds
read hid behind `-O2` dead-code elimination and would never have shown up in the
Release matrix. It is now part of the bar.

**The one Debug failure is pre-existing and not lubrication-related.**
`pe-interface-serial-atc-resume-roundtrip` aborts on
`Material::getDensity: Assertion 'material < materials_.size()' failed` — an
invalid material ID in the ATC checkpoint/resume path. Verified by building the
**base branch** (`fix/lubrication-merge-resolution`) in the identical Debug
configuration: same test, same assertion, same abort. Worth its own look, since
it is the same species of latent defect as finding 1 — real, and invisible in
Release.

Note the torque test (`bounce-Stokes`) now runs under `HardContactAndFluid`: with
the angular-velocity artifact removed, the campaign solver expresses the full
lubrication wrench.

gcc 11.5.0, cmake 3.31.8, Release, non-MPI (the serial-PE arrangement).
All four inherited lubrication tests **pass** — none skips — in both
configurations. That permanently resolves the fix-branch test-skip issue.

Every commit was additionally checked to build standalone
(`cmake && make` at each of `dd6d962 … 9545c9f`): all 0.

### Runtime demonstration

Printed by `pe_lubrication_serial_coverage_test` itself, so it stays true rather
than being a one-off transcript:

```
[lubrication ENABLED, solver = HardContactAndFluid]
  aabbPadding(R) = 0.005 (cutoffFactor*R)
  pair  h/R=0.2    K_pipeline=0.030550333    K_closure=0.030550333    rel.diff=1.04e-14
  pair  h/R=0.1    K_pipeline=0.057005756    K_closure=0.057005756    rel.diff=2.09e-14
  pair  h/R=0.05   K_pipeline=0.10702876     K_closure=0.10702876     rel.diff=8.22e-15
  pair  h/R=0.02   K_pipeline=0.25225043     K_closure=0.25225043     rel.diff=6.00e-15
  pair  h/R=0.01   K_pipeline=0.49079337     K_closure=0.49079337     rel.diff=4.00e-15
  wall  h/R=0.2    K_pipeline=0.10060414     K_closure=0.10060414     rel.diff=4.66e-15
  wall  h/R=0.1    K_pipeline=0.19738278     K_closure=0.19738278     rel.diff=2.22e-16
  wall  h/R=0.05   K_pipeline=0.38841921     K_closure=0.38841921     rel.diff=0.00e+00
  wall  h/R=0.02   K_pipeline=0.957296       K_closure=0.957296       rel.diff=1.07e-13
  wall  h/R=0.01   K_pipeline=1.902358       K_closure=1.902358       rel.diff=1.09e-13

[lubrication DISABLED]
  aabbPadding(R)             = 0
  contactGenerationEnabled() = false
  pair velocity delta        = 0
  wall velocity delta        = 0
```

- **enabled = false ⇒** padding exactly `0`, detection fast path taken, velocity
  deltas exactly `0` (not "small" — bitwise zero), for both pair and wall.
- **enabled = true under `HardContactAndFluid` ⇒** nonzero force, matching the
  closure to `~1e-14` relative across the band, for both sphere–sphere and
  sphere–wall.

### Refusal guard

```
### EL (default, not stage-capable)
active solver stage-capable: no
enabled=false -> accepted (correct)
enabled=true  -> REFUSED: lubricationEnabled_ is true but the active
  pe_CONSTRAINT_SOLVER does not invoke the lubrication stage, so no lubrication
  force would be applied. Select a stage-capable solver (e.g.
  pe::response::HardContactAndFluid) or set lubricationEnabled_ to false.

### HardContactAndFluid (stage-capable)
active solver stage-capable: yes
enabled=false -> accepted (correct)
enabled=true  -> accepted
```

### Fidelity evidence

The strongest single result: **`pe_lubrication_legacy_regression_test` passes
against reference trajectories generated under `HardContactLubricated`,
bit-for-bit, through `HardContactAndFluid` + the extracted stage.** The
extraction and the hook reproduce the retired pipeline exactly.

### Not verified

- **Examples and tools.** Not built by default (`PE_BUILD_EXAMPLES=OFF`,
  `PE_BUILD_LIVE_VIEWER=OFF`) and not compile-checked. They are re-pointed
  mechanically at `HardContactAndFluid` + `lubrication::setEnabled(true)`.
- **MPI.** Everything here is `PE_USE_MPI=OFF`. The stage is placed exactly where
  the retired pipeline placed it, between the same two `synchronizeVelocities()`
  calls, so the MPI contract is unchanged by construction — but it is untested.
- **`pe_CONSTRAINT_SOLVER=ShortRangeRepulsion` as a selected solver** does not
  build. Confirmed **pre-existing** on the fix branch (`c_interface_queries.h`
  references `getNumberOfContacts`, which SRR's collision system lacks). Not a
  regression from this PR; SRR is exercised as a contact solver, not as the
  selected constraint solver.
- **Gate G0** (lubrication-off bitwise twins of DKT 20-step and e4_l3 against
  certified logs) is campaign work and belongs to the validation ladder, not to
  this PR. The disabled-path arguments above are the *design* case for why G0
  should pass; they are not a substitute for running it.

---

## Recommended follow-ups

1. **Run Gate G0 before anything downstream bumps the submodule.** The
   disabled path is now *enforced* bitwise by
   `pe_lubrication_aabb_neutrality_test` rather than merely argued, but note
   what that test does and does not say: it proves lubrication-off adds no
   padding **relative to `radius + contactThreshold`**. It does not say the
   AABBs match the campaign's *current* binary — they deliberately do not,
   because that binary carries the legacy unconditional `1e-2` halo. Shrinking
   it is the intended fix (spec §3.4) and is precisely the change G0's twins
   have to clear.
2. **`setMeshDx` is still a dead wire.** `lubricationMeshClampFactor_` now
   genuinely controls both the force cutoff *and* the AABB padding (commit 2),
   which is what makes the D2.1 "take over at gap ≲ 2h" rule expressible — but
   nothing calls `lubrication::setMeshDx()` outside a unit test. That is the
   FF-side wire (§4) and it is what unlocks the whole point of the clamp.
3. **Re-examine every rotation-dependent campaign result produced before commit
   `a71c34d`** (finding B), DKT first. Those runs had particle rotation pinned
   at zero by the removed artifact. This is the highest-value item in this list
   and it is independent of lubrication.
4. **CI**: the two library variants make a `BUILD_TESTING=ON` build noticeably
   heavier. If that is unwelcome, the alternative is to accept skips again, or
   to make the shipped default stage-capable. Worth an explicit decision rather
   than drift.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
