# Lubrication Contacts: Design and Initial Integration

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
- Excludes lubrication contacts from the hard-contact constraint arrays (they don’t enter the unilateral constraint solver).
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

## Limitations (Initial Cut)
- Only sphere–sphere and sphere–plane pairs are supported.
- No tangential lubrication yet; normal component only.
- No hysteresis on thresholds (can be added later to avoid regime flicker).
- No partial submersion logic or deeper CFD coupling; keeps a simple analytical model.

## Next Steps
- Expose `eps_lub` and cap `α` as configuration/settings.
- Add optional hysteresis (`lubricationOn`, `lubricationOff`).
- Extend to additional geometries once the core is validated.
- Add a minimal example validating slowdown in lubrication range and smooth transition to contact.

