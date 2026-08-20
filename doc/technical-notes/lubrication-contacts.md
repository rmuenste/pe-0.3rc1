# Lubrication Contacts: Design and Initial Integration

Status
- Canonical implementation: the **runtime lubrication add-on** —
  `pe/core/lubrication/LubricationStage.h` (application stage),
  `pe/core/lubrication/LubricationModel.h` (Kroupa-2016 closure),
  `pe/core/lubrication/ContactState.h` (per-contact flag + blend weight),
  `pe/core/lubrication/Params.h` (runtime parameter store / master switch).
- **Retired:** the dedicated `pe::response::HardContactLubricated` collision system, and
  the `PE_LUBRICATION_CONTACTS` compile-time macro. Neither exists any more. Passages
  below that describe them are kept for provenance and marked as historical.
- Enabling lubrication is a runtime decision (`lubricationEnabled_` json key /
  `pe::lubrication::setEnabled(true)`), **not** a solver choice, and it defaults to OFF.
  It requires a stage-capable solver: `pe::response::HardContactAndFluid` or
  `pe::response::HardContactSemiImplicitTimesteppingSolvers`, both of which advertise
  `static constexpr bool hasLubricationStage = true;`. Enabling it under any other
  `pe_CONSTRAINT_SOLVER` throws in `pe::applyOptionalLubricationParams()`.
- Deprecated (to be removed next version): HardContactAndFluidWithLubrication, HardContactFluidLubrication
- Solver selection is still compile-time configuration in `pe/config/Collisions.h`
  (default: the neutral `pe::response::HardContactEulerLagrange`); this note describes
  the lubrication/contact stack, not a universal default.

## Overview
- Introduces a lubrication interaction regime between “no contact” and “hard contact”.
- Pairs are classified by surface gap `dist`:
  - Hard contact: `dist < contactThreshold`
  - Lubrication contact: `contactThreshold ≤ dist < contactThreshold + lubricationThreshold`
  - Otherwise: no contact
- Scope: sphere–sphere and sphere–plane only (initial).

## Detection Changes
- Sphere–sphere: when gap falls in the lubrication band, create a lubrication contact via `ContactVector::addLubricationContact(...)`.
- Sphere–plane: added the same lubrication branch.
- Lubrication contacts are flagged via `ContactTrait::setLubricationFlag()`.
- The detection branches are gated at **runtime** on
  `pe::lubrication::contactGenerationEnabled()` (`isEnabled() || getSecurityZone()`; the
  security zone belongs to `ShortRangeRepulsion`, which reuses the same pre-contact
  channel). There is no compile-time gate any more — `PE_LUBRICATION_CONTACTS` is gone.

## Contact State (lubrication mixin)
- `pe/core/lubrication/ContactState.h` supplies the per-contact lubrication flag and
  blend weight that a `ContactTrait` mixes in:
  - `setLubricationFlag()` / `getLubricationFlag()`,
  - `setLubricationWeight()` / `getLubricationWeight()` (clamped to `[0,1]`).
  Contact traits that do not carry the state provide inert stubs, so a solver that never
  runs the stage simply sees "no lubrication contacts".
- *Historical:* this state used to live in a `ContactTrait` specialization written
  specifically for the `HardContactLubricated` solver stack.

## Force Model (Production: Kroupa et al. 2016)
- Physics lives in `pe/core/lubrication/LubricationModel.h` (header-only, unit-tested in
  isolation by `tests/interface/pe_lubrication_model_test.cpp`); the application loop in
  `pe/core/lubrication/LubricationStage.h` only gathers kinematics, calls the
  model, integrates, and applies velocity corrections. A collision system invokes the
  stage after its first `synchronizeVelocities()`. (*Historically this loop was inlined
  in `pe/core/collisionsystem/HardContactLubricated.h`.*)
- Full pairwise resistance set (Langmuir 2016, 32, 8451−8460; Dance & Maxey 2003):
  - Normal squeeze force with `1/ε`, `ln ε`, and `ε ln ε` terms (pair eq 12 / wall eq 16)
  - Tangential sliding force `F_sl` (eq 13/17), sliding torque `M_sl` (eq 14/18), and
    twisting torque `M_tl` (eq 15/19; sign corrected to be dissipative)
  - `ε = h/a_ref` with `a_ref = 2·R1R2/(R1+R2)` (pair) or `R_sphere` (wall)
- Divergence treatment: Vinogradova slip factor `f*` for `h ≥ h_c` and force saturation
  (frozen separation) for `h < h_c`, with `h_c = ε_c·a_ref` (`lubricationEpsCritical_`,
  default 0.1 per the paper's calibration against Krieger/de Kruif data).
- Normal force acts on approach **and** separation (suction), switchable via
  `lubricationOnSeparation_`. Moving walls contribute their real velocity (shear cells).
- Time integration (default `"semi-implicit"`): the stiff normal mode is integrated
  exactly, `J_n = m_eff (1−exp(−K_n dt/m_eff)) |vrn| < m_eff|vrn|` — unconditionally
  stable at CFD-scale timesteps, no impulse cap needed. Tangential force/torques use the
  same exponential clamp per mode. `"explicit-capped"` retains the legacy cap.
- Legacy model preserved: `lubricationModel_="legacy"` + `lubricationCutoffFactor_=0`
  reproduces the pre-2026 solver bit-for-bit (normal-only `F = 6πμR_eff²(−vrn)/gap`,
  approach-only, impulse-capped) — guarded by
  `tests/interface/pe_lubrication_legacy_regression_test.cpp`.
- `μ` from `Settings::liquidViscosity()`; excludes lubrication contacts from the
  hard-contact constraint arrays (they don't enter the unilateral constraint solver).

### MPI Synchronization (Critical)
- **Velocity synchronization is required AFTER applying lubrication forces**
- Sequence in `resolveContacts()`:
  1. `synchronizeVelocities()` - sync before lubrication
  2. Apply lubrication force loop (modifies `dv_`, `dw_` arrays)
  3. `synchronizeVelocities()` - **sync after lubrication (REQUIRED for MPI correctness)**
  4. Hard contact solver iterations
- **Why this is critical**: Without the second sync, shadow copies on remote processes have stale velocity values, leading to:
  - Inconsistent forces in the hard contact solver
  - Incorrect collision response for bodies shared across process boundaries
  - Potential simulation divergence or unphysical results
- This synchronization overhead is necessary but minimal (one additional MPI exchange per time step)

## Enabling Lubrication In An Application

Two independent steps — a compile-time solver choice, then a runtime switch:

```bash
# 1. Build with a stage-capable constraint solver (the library default is the
#    neutral HardContactEulerLagrange, which does NOT run the stage).
cmake -S . -B build-lub -DCMAKE_BUILD_TYPE=Release -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON \
      -DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid"
cmake --build build-lub -j
```

```json
// 2. Turn lubrication on at runtime, in the simulation's json input:
{ "lubricationEnabled_": true }
```

Programmatic equivalent: `pe::lubrication::setEnabled(true)`. `pe_CONSTRAINT_SOLVER` can
equally be set in `pe/config/Collisions.h`; the other stage-capable solver is
`pe::response::HardContactSemiImplicitTimesteppingSolvers`. If step 2 is done without
step 1, `pe::applyOptionalLubricationParams()` throws at setup rather than running a
silent no-op.

> **Retired recipe.** Lubrication used to be selected by building against
> `-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactLubricated` (optionally with the
> `PE_LUBRICATION_CONTACTS` macro). That solver and that macro no longer exist, so such a
> configure line now fails outright. Use the two steps above.

## Runtime Parameters (JSON via SimulationConfig)
All model parameters are runtime input, parsed by `SimulationConfig::loadFromFile` and
pushed into the engine by `pe::applyOptionalLubricationParams()`
(`pe/interface/setup_optional_collision_params.h`), which every serial setup and
`setup_kroupa.h` call after `loadFromFile`. Other MPI setups still need the same
one-liner. Programmatic access: `pe::lubrication::` setters (`pe/core/lubrication/Params.h`).

| JSON key | Default | Meaning |
|---|---|---|
| `lubricationEnabled_` | `false` | Master switch (also gates lubrication-contact emission). Requires a stage-capable solver; otherwise `applyOptionalLubricationParams()` throws |
| `lubricationModel_` | `"kroupa2016"` | `"kroupa2016"` or `"legacy"` |
| `lubricationIntegration_` | `"semi-implicit"` | or `"explicit-capped"` |
| `lubricationTangential_` | `true` | Sliding force + sliding torque |
| `lubricationTwisting_` | `true` | Twisting torque |
| `lubricationSlipCorrection_` | `true` | Vinogradova `f*` |
| `lubricationOnSeparation_` | `true` | Normal suction on separation |
| `lubricationWallTerms_` | `true` | Wall resistance set for sphere–plane |
| `lubricationEpsCritical_` | `0.1` | `ε_c = h_c/a_ref` saturation / slip length |
| `lubricationCutoffFactor_` | `0.5` | `ε_cut = h_cut/a_ref`; `0` → legacy absolute threshold |
| `lubricationMeshClampFactor_` | `0` (off) | Clamp `h_cut ≤ c·Δx_CFD`; `Δx` set via `lubrication::setMeshDx()` by the CFD interface |
| `lubricationAabbInflation_` | `true` | Grow AABBs by the lubrication cutoff (switchable) |
| `minEpsLub_` | `1e-8` | Final numerical gap floor |
| `alphaImpulseCap_` | `1.0` | Cap factor (`explicit-capped` scheme only) |
| `contactHysteresisDelta_` / `lubricationHysteresisDelta_` | `1e-9` / `1e-3` | Blend half-widths |
| `fluidViscosity_` | — | `μ` (via `world->setViscosity`) |

### Outer cutoff semantics (CFD coupling)
`h_cut = min(ε_cut·a_ref, c·Δx_CFD)`. In **unresolved Euler–Lagrange** coupling the mesh
clamp stays off and `ε_cut` is set by model validity alone (0.5–1). In **marginally
resolved** coupling (~8–16 cells per diameter) the clamp binds so the model only adds
what the grid cannot resolve (`c ≈ 1–2`). Legacy absolute behavior: `ε_cut = 0`.

### AABB inflation switch
`SphereBase`/`PlaneBase::calcBoundingBox()` pad via `lubrication::aabbPadding(radius)`
(planes use the largest registered sphere radius; sphere-side padding alone already
covers the pair cutoff, so plane-box staleness is benign). Switching inflation off with
lubrication on emits a one-time warning: pairs may enter the band undetected.

## Legacy behavior recipe
```json
{ "lubricationModel_": "legacy", "lubricationCutoffFactor_": 0.0 }
```
reproduces pre-2026 trajectories exactly (regression-gated). The old no-op stubs
`setLubrication/setSlipLength/setMinEps` remain as deprecated no-ops.

## Regime Transition Blending
To prevent flickering between contact regimes when gaps oscillate near threshold boundaries, the detection step computes smooth blend weights directly from the current gap distance:

### Blended Weights
- Hard-contact weight `w_hard(dist)` is 1 for `dist ≤ contactThreshold − contactHysteresisDelta` and decreases smoothly to 0 at `dist = contactThreshold + contactHysteresisDelta`.
- Lubrication weight `w_lub(dist)` ramps from 0 to 1 across the same entry band, stays at 1 inside the lubrication window, then fades to 0 over `lubricationHysteresisDelta` around `contactThreshold + lubricationThreshold`.
- Weights are computed analytically each detection step—no per-pair history or bookkeeping.

### Blend Parameters
- `contactHysteresisDelta_`: Half-width of the hard-contact blend zone (default: 1e-9).
- `lubricationHysteresisDelta_`: Half-width of the lubrication cutoff blend zone (default: 1e-9).
- Both exposed via `setContactHysteresisDelta()` and `setLubricationHysteresisDelta()`.

### Benefits
- **No bookkeeping**: Eliminates maps and cleanup for per-pair regime state.
- **Smooth transitions**: Hard and lubrication responses fade continuously, preventing regime flapping.
- **MPI-friendly**: Purely local evaluation removes cross-rank synchronization concerns.

### Detection Integration
- Implemented in `pe/core/detection/fine/MaxContacts.h`:
  - `collideSphereSphere()`: Computes blend weights and emits weighted lubrication contacts.
  - `collideSpherePlane()`: Same blending logic for sphere-plane gaps.
- Accesses `CollisionSystem` via `theCollisionSystem()` singleton for blend parameter queries.

## Limitations (Current Implementation)
- Only sphere–sphere and sphere–plane pairs are supported.
- Unequal radii use the equal-sphere log coefficients with `ε = h/(2a)`; the `1/ε` pole
  is exact, the log terms are approximate (Jeffrey–Onishi β-dependent coefficients are a
  possible follow-up behind the same `computeWrench` interface).
- The hard-contact relaxation models are inelastic (no material restitution), so
  restitution-vs-Stokes validation (Gondret) is limited to the viscous/attenuation side;
  see `tests/interface/pe_lubrication_bounce_stokes_test.cpp` header.
- Other shapes (capsules, boxes, meshes) are not AABB-inflated for lubrication.
- MPI parity test variant for the lubrication stage is still to be added (needs an MPI
  test build with a stage-capable solver and `lubricationEnabled_` on; the two
  `synchronizeVelocities()` calls around the lubrication loop are unchanged, so the
  communication footprint is identical).

## Validation
- Level 0 (pure math, any build): `pe-lubrication-model` — coefficients vs hand values,
  slip-factor limits, dissipativity on random states, semi-implicit impulse bounds,
  cutoff/padding helpers.
- Level 1 (drives the stage through a real collision pipeline; the targets link the
  `pe_static_lubstage_fluid` / `pe_static_lubstage_plain` library variants, which are
  built with a stage-capable `pe_CONSTRAINT_SOLVER`, so they run and must pass in every
  build configuration — they no longer self-SKIP on the library default):
  `pe-lubrication-legacy-regression` (bit-for-bit legacy gate),
  `pe-lubrication-two-sphere-approach` (paper Fig. 7 pointwise: no-slip curve, slip
  departure, saturation plateau monotone in ε_c, switchable suction; **caution**: the
  paper's Fig. 7 caption says "approach of two spheres" but the curves use the
  **wall** resistance set, eq 16 — leading `ε⁻¹`, 4× the equal-sphere pair's `¼ε⁻¹` —
  because the figure compares against sphere-on-flat AFM data ("wall = particle j with
  zero velocity"). Plot the CSV's `wall …` curves via
  `plot_two_sphere_approach.py approach.csv 5000 wall` to reproduce the figure; plateaus
  land at 405/805/1605/3202 for ε_c = 0.02/0.01/0.005/0.0025),
  `pe-lubrication-bounce-stokes` (immersed wall impact: dissipativity, viscosity
  attenuation, dt robustness over 3 decades, wall tangential/twist sanity).
  Plain `-DBUILD_TESTING=ON` build; no solver override needed.
- Level 2 skeleton: `examples/shear_cell_kroupa` (boundary-driven shear cell, lubrication
  stress virial → `η_L(φ)` CSV) + `evaluate_viscosity.py` (Krieger–Dougherty /
  Maron–Pierce comparison). Full η(φ) campaign and CFD-coupled Level 3 are future work.

## MPI Parallel Execution
- **Fully supported**: Lubrication forces work correctly in MPI simulations
- **Synchronization overhead**: One additional `synchronizeVelocities()` call per time step (after lubrication, before hard contact solver)
- **Blending**: Weights are derived from the instantaneous gap on each process, so no regime state needs to be communicated.
- **Performance**: Minimal overhead for MPI communication (~1 additional MPI_Allreduce per time step)

## Regime Blending (Configuration)
- Goal: avoid regime flapping when the gap oscillates near the contact threshold.
- Configure entry/exit smoothness via `contactHysteresisDelta` (hard-contact blend) and `lubricationHysteresisDelta` (lubrication falloff).
- For coarse meshes or noisy distance signals, increase the deltas to widen the blend regions; reduce them for sharper transitions.

## MPI Considerations (High Priority)
- The lubrication feature targets FSI workflows that always run in MPI; design must be consistent across ranks.
- Ownership and determinism:
  - Generate/flag lubrication contacts only on the owning rank of the contact point (consistent with contact handling).
  - The blend weights depend solely on the instantaneous gap, so deterministic collision detection keeps ranks in sync without extra state.
- Velocity synchronization:
  - After applying lubrication corrections, perform a velocity synchronization so shadow copies see identical states before contact relaxation and position integration.
  - Practical ordering within `resolveContacts`:
    1) Cache contacts and body velocities
    2) synchronizeVelocities() (as usual)
    3) Apply lubrication `dv_/dw_` corrections
    4) synchronizeVelocities() again (ensure consistency of lubrication effects across ranks)
    5) Run hard-contact relaxation

## Next Steps
- Expose `eps_lub`, cap `α`, and blend widths (`contactHysteresisDelta`, `lubricationHysteresisDelta`) as configuration/settings.
- Extend to additional geometries once the core is validated.
- Add a minimal example validating slowdown in lubrication range, hysteresis behavior, and smooth transition to contact.
