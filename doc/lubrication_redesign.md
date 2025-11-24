# Lubrication Response Redesign

This document tracks the migration from the legacy lubrication-capable solver stacks
(`HardContactAndFluidWithLubrication`, `HardContactFluidLubrication`) to the new
`HardContactLubricated` implementation introduced in commit `f23c5712`.

## Overview

The HardContactLubricated stack provides a dedicated collision system specialization and
rigid-body trait wiring for lubrication-heavy setups. It is intended to replace the older,
hybrid `HardContactAndFluid*` solvers and to deliver cleaner configuration, improved
contact preprocessing, and easier debugging.

## Changes Implemented (Nov 2025)

### Default Solver Selection

- `pe/config/Collisions.h` now sets `pe_CONSTRAINT_SOLVER` to
  `pe::response::HardContactLubricated` so all builds pick up the new stack by default.

### Numerical Threshold Updates

- `Thresholds<double>::lubricationThreshold()` increased to `1e-2`
  (`pe/core/Thresholds.h`) so double-precision builds use the same near-contact cutoff as
  the float/long double specializations. This value is referenced throughout the
  HardContactLubricated pipeline (collision detection, solver tolerances, and diagnostics).

### Bounding Box Inflation

- `SphereBase::calcBoundingBox()` now grows each sphere’s AABB by both
  `contactThreshold` **and** `lubricationThreshold`. This ensures coarse detection wakes up
  before the gap shrinks below the lubrication cutoff, preventing the solver from missing
  pre-contact lubrication events.

### Fine Detection (Sphere/Plane)

- `MaxContacts::collideSpherePlane()` emits a contact whenever the gap is below
  `contactThreshold + lubricationThreshold`, mirroring the existing sphere-sphere logic.
  This mirrors the intended lubrication-trigger behavior and allows the
  HardContactLubricated solver to receive consistent pre-impact contact data for
  sphere/plane interactions.

### Example Coverage

- Added `examples/basic_lubrication/basic_lubrication.cpp`, a minimal sphere-vs-plane drop
  test built specifically for the HardContactLubricated stack. It sets lubrication-related
  world and collision-system parameters, exposes CLI hooks for step size, radius, and
  lubrication toggles, and writes optional VTK output for inspection.

## How Blending Works (Detection + Response)

HardContactLubricated enables `PE_LUBRICATION_CONTACTS`, so `MaxContacts` emits both hard and
lubrication contacts depending on gap distance. Two helper weights drive this:

- `computeHardWeight(dist, contactThreshold, contactBlend)`  
  - Weight = 1 below `contactThreshold - contactBlend`; 0 above `contactThreshold + contactBlend`;
    linear ramp in between. If `contactBlend` is zero, this is a sharp step at the threshold.
  - Used to decide whether to emit a hard contact (`addVertexFaceContact`).

- `computeLubricationWeight(dist, contactThreshold, lubricationThreshold, contactBlend, lubricationBlend)`  
  - Ramps up from 0→1 across `[contactThreshold - contactBlend, contactThreshold + contactBlend]`.
  - Stays at 1 until near the lubrication cutoff.
  - Ramps down 1→0 across `[(contactThreshold + lubricationThreshold - lubricationBlend), (contactThreshold + lubricationThreshold + lubricationBlend)]`.
  - Used to decide whether to emit a lubrication contact (`addLubricationContact`) and to store
    the lubrication weight on that contact.

### What Gets Emitted

- Sphere-sphere: `MaxContacts::collideSphereSphere` (PE_LUBRICATION_CONTACTS branch) emits:
  - Hard contact if `hardWeight > 0`.
  - Lubrication contact if `lubWeight > 0` (carrying the weight).
- Sphere-plane: same pattern in `collideSpherePlane`.
- In the overlap band near the contact threshold, both contacts are emitted, providing a smooth
  handoff instead of an abrupt switch.

### How It’s Applied in the Solver

- In `HardContactLubricated` (collision response loop):
  - Lubrication loop iterates contacts flagged as lubrication, applies the classical normal
    lubrication force (6πμR_eff^2 * vrn / gap) scaled by the stored lubrication weight, and
    caps impulses relative to approach momentum. If the stored weight is 0, no lubrication force
    is applied.
  - Hard contacts (from `addVertexFaceContact`) are then resolved by the standard relaxation
    solver.
- At `gap ≈ 0`, lubrication weight has ramped to 0, so only the hard contact is active. For
  larger gaps up to `contactThreshold + lubricationThreshold`, lubrication weight ramps on/off.

### Parameters That Shape the Blend

- `contactThreshold` — start of hard-contact activation.
- `lubricationThreshold` — distance beyond the contact threshold where lubrication fades out.
- `contactBlend` — half-width of the smoothing band around `contactThreshold` (used by both
  hard and lubrication ramps on entry).
- `lubricationBlend` — half-width of the smoothing band around `contactThreshold + lubricationThreshold`
  (lubrication ramp-down).

## Next Steps

- Extend the bounding-box inflation logic to other primitives (capsules, boxes, triangles)
  so lubrication contacts always have sufficient coarse-detection padding.
- Audit all fine-detection pairs to ensure the lubrication threshold is consistently
  honored (e.g., box-plane, mesh-plane cases).
- Flesh out `HardContactLubricated` solver documentation (currently marked TODO) with the
  algorithmic details, parameter meanings, and convergence guidelines.
- Gate lubrication-contact generation via `PE_LUBRICATION_CONTACTS` from `pe/config/Collisions.h`
  once we are ready to enable force application; a conditional define tied to
  `pe_CONSTRAINT_SOLVER == pe::response::HardContactLubricated` can be added there (kept
  disabled for now to allow diagnostic runs without applied forces).
