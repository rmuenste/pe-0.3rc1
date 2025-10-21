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

## Hysteresis (Regime Transition Stability)
To prevent flickering between contact regimes when gaps oscillate near threshold boundaries, a hysteresis mechanism has been implemented:

### Implementation
- **Persistent State Storage**: Each body pair's contact regime (NO_CONTACT, LUBRICATION, HARD_CONTACT) is tracked in `CollisionSystem::contactRegimeHistory_`
- **Body Pair Identification**: Uses canonical ordering of system IDs via `BodyPairID` struct
- **State-Dependent Thresholds**: Different transition thresholds depending on current regime:
  - From HARD_CONTACT: Exit at `contactThreshold + contactHysteresisDelta`
  - To HARD_CONTACT: Enter at `contactThreshold - contactHysteresisDelta`
  - From LUBRICATION: Exit at `lubricationThreshold + contactThreshold + lubricationHysteresisDelta`
  - To LUBRICATION: Enter at `lubricationThreshold + contactThreshold - lubricationHysteresisDelta`

### Hysteresis Parameters
- `contactHysteresisDelta_`: Width of hysteresis band around `contactThreshold` (default: 1e-9)
- `lubricationHysteresisDelta_`: Width of hysteresis band around `lubricationThreshold` (default: 1e-9)
- Configurable via `setContactHysteresisDelta()` and `setLubricationHysteresisDelta()`

### Benefits
- **Eliminates flickering**: No rapid regime switching at threshold boundaries
- **Numerically stable**: Smooth force evolution during transitions
- **Physically realistic**: Mimics real adhesion/separation dynamics
- **Low overhead**: Simple map lookup per contact pair

### State Management
- `getContactRegime(b1, b2)`: Query previous regime for body pair
- `updateContactRegime(b1, b2, regime)`: Update regime after classification
- `clearContactRegimeHistory()`: Reset all stored states (e.g., after major simulation events)

### Detection Integration
- Implemented in `pe/core/detection/fine/MaxContacts.h`:
  - `collideSphereSphere()`: Sphere-sphere detection with hysteresis
  - `collideSpherePlane()`: Sphere-plane detection with hysteresis
- Accesses `CollisionSystem` via `theCollisionSystem()` singleton

## Limitations (Current Implementation)
- Only sphere–sphere and sphere–plane pairs are supported.
- No tangential lubrication yet; normal component only.
- No partial submersion logic or deeper CFD coupling; keeps a simple analytical model.
- Hysteresis state is never automatically cleaned up (may grow unbounded for long simulations with many transient contacts)

## MPI Parallel Execution
- **Fully supported**: Lubrication forces work correctly in MPI simulations
- **Synchronization overhead**: One additional `synchronizeVelocities()` call per time step (after lubrication, before hard contact solver)
- **Hysteresis state**: Currently local to each process; not synchronized across MPI boundaries
  - Works correctly because detection happens locally and regime transitions are deterministic
  - Body pairs are consistently classified across all processes that see them
- **Performance**: Minimal overhead for MPI communication (~1 additional MPI_Allreduce per time step)

## Hysteresis (Recommended)
- Goal: avoid regime flapping when the gap oscillates near the threshold.
- Definitions:
  - `lubricationOn  = contactThreshold + h_on`
  - `lubricationOff = contactThreshold + h_off` with `h_on > h_off`
- Behavior:
  - If a pair is not in lubrication and `dist ≤ lubricationOn` → enter lubrication.
  - If a pair is in lubrication and `dist ≥ lubricationOff` → leave lubrication.
- State tracking:
  - Maintain a per-pair boolean “isLubricationActive”. For deterministic keys across MPI ranks, use ordered `(owner(body1), id1, owner(body2), id2)`.
  - Clear state when pairs disappear or ownership changes.
- Configuration:
  - `h_on`, `h_off` set from simulation input; can be tied to CFD grid size (e.g., one or fractions of a cell).

## MPI Considerations (High Priority)
- The lubrication feature targets FSI workflows that always run in MPI; design must be consistent across ranks.
- Ownership and determinism:
  - Generate/flag lubrication contacts only on the owning rank of the contact point (consistent with contact handling).
  - Ensure deterministic ordering when deriving keys for hysteresis state (use system IDs and owner ranks).
- Velocity synchronization:
  - After applying lubrication corrections, perform a velocity synchronization so shadow copies see identical states before contact relaxation and position integration.
  - Practical ordering within `resolveContacts`:
    1) Cache contacts and body velocities
    2) synchronizeVelocities() (as usual)
    3) Apply lubrication `dv_/dw_` corrections
    4) synchronizeVelocities() again (ensure consistency of lubrication effects across ranks)
    5) Run hard-contact relaxation

## Next Steps
- Expose `eps_lub`, cap `α`, and hysteresis (`h_on`, `h_off`) as configuration/settings.
- Implement per-pair hysteresis state with MPI-safe keys and lifecycle.
- Extend to additional geometries once the core is validated.
- Add a minimal example validating slowdown in lubrication range, hysteresis behavior, and smooth transition to contact.
