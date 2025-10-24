# Lubrication Contacts: Design and Initial Integration

Status
- Canonical implementation: HardContactLubricated
- Deprecated (to be removed next version): HardContactAndFluidWithLubrication, HardContactFluidLubrication

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

## Contact Trait (HardContactLubricated)
- Added `ContactTrait` specialization to carry a lubrication flag for the `HardContactLubricated` solver stack:
  - `setLubricationFlag()` and `getLubricationFlag()`.

## Collision System Integration
- File: `pe/core/collisionsystem/HardContactLubricated.h`
- Excludes lubrication contacts from the hard-contact constraint arrays (they don't enter the unilateral constraint solver).
- After velocity synchronization inside `resolveContacts`, applies lubrication as external velocity corrections for flagged lubrication pairs:
  - Relative velocity at contact: `gdot = (v1 + w1 × r1) − (v2 + w2 × r2)`.
  - Normal component `vrn = n · gdot`; only resist approaching motion (`vrn < 0`).
  - Effective radius:
    - Sphere–sphere: `R_eff = (R1·R2)/(R1 + R2)`
    - Sphere–plane: `R_eff = R_sphere`
  - Normal lubrication magnitude (classical form): `F = 6π μ R_eff^2 (−vrn) / gap`
    - `μ` from `Settings::liquidViscosity()`
    - `gap = max(dist, eps_lub)` to regularize
  - Apply as velocity corrections to both bodies (linear and angular through torque `r × F`).

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

## Stability Safeguards
- Regularization: `eps_lub = 1e-8` (member `minEpsLub_`).
- Impulse capping: limit `|F| dt` by `α m_eff |vrn|` with `α = 1.0` (member `alphaImpulseCap_`).
  - `m_eff = 1/(m1^{-1} + m2^{-1})`.

## Parameters
- Uses existing thresholds from `pe/core/Thresholds.h`:
  - `contactThreshold`
  - `lubricationThreshold` (intended to be set from a simulation parameter, e.g., CFD cell size)
- Fluid properties from `pe/core/Settings.h`:
  - `Settings::liquidViscosity()` and `Settings::liquidDensity()` (buoyancy already present elsewhere).

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
- No tangential lubrication yet; normal component only.
- No partial submersion logic or deeper CFD coupling; keeps a simple analytical model.

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
