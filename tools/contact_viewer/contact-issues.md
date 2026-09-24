# Contact issues and observations

Technical problems and engine behaviour found with the contact viewer (`README.md` in this
folder covers usage). Observed with the repo's default `pe::response::HardContactEulerLagrange`
solver and `pe::detection::fine::MaxContacts`.

## Open problems

### Remaining gaps in contact generation

From a survey of `MaxContacts::collide()` and every routine it dispatches to:

- **Inner cylinder** is implemented only against sphere and ellipsoid. Against box, capsule,
  plane, triangle mesh, union and another inner cylinder the dispatch has no case and throws
  `std::runtime_error( "Unknown body type" )`. Inner cylinder vs cylinder depends on the order:
  inner cylinder first returns nothing (explicit empty case), cylinder first throws.
- **Primitive vs triangle mesh** (box, capsule, cylinder, ellipsoid) goes through GJK/EPA only:
  one contact, the mesh treated as convex. The DistanceMap is used only for plane-mesh and
  mesh-mesh; sphere-mesh uses a brute-force closest-triangle search (one contact, works for
  non-convex meshes).
- **Plane vs plane** generates nothing, by design (both fixed and infinite).

### Unverified observations

- Pair Lab preset "box on box: face-face, offset + yaw" gives five contacts, three with their
  points at z = 0.5 and two at z = 0.49. Not checked whether the mixed placement is intended.
- Contact point placement differs between routines: box-plane and capsule-plane put the point on
  the penetrating body; sphere-plane, ellipsoid-plane, cylinder-plane and the GJK/EPA path
  (`compatibleContactPoint()`) put it on the flat body's surface.

## Solver and engine behaviour to know

- **Gravity is not applied by the solver.** `HardContactEulerLagrange` ignores
  `World::setGravity()` on purpose: body forces belong to the outer CFD driver. Stack Lab adds
  `m g` to each dynamic body before every step and keeps the world gravity at 0, so a solver that
  does honour it does not apply it twice.
- **Friction is additive per pair.** `createMaterial()` fills the pair tables with the sum of the
  two materials' coefficients (`src/core/Materials.cpp`), so a single material with `csf = mu`
  gives a contact friction of 2 mu. Stack Lab gives each body mu / 2, so the GUI's mu is the real
  contact friction.
- **Dropped boxes bounce at restitution 0.** Landing at ~4.9 m/s with dt = 2e-3 penetrates ~1 cm
  in one step; the boxes bounce back at ~0.9 m/s and settle after ~1.8 s. Likely the position
  correction (error reduction 0.7 by default): lower it or halve dt and watch the kinetic-energy
  plot.

## Resolved

- **Fixed bodies inherited a stale velocity correction** (found as "the ramp scenario depends on
  what ran before it"). The hard-contact solvers keep per-body velocity corrections in `dv_` /
  `dw_`, indexed by the body's slot in the body storage and resized, not reset, every step.
  `initializeVelocityCorrections()` wrote them only for awake, non-fixed bodies, so a fixed
  body's slot kept the correction of whichever body used that index before. A fixed body never
  receives an impulse, so the stale value persisted and acted as a phantom velocity of the fixed
  body in every contact with it: boxes resting on a fixed ramp saw the ramp move away, got no
  reaction and fell through it (drift 2.4 after 1 s instead of 8e-7).
  - Triggered whenever a fixed non-plane body lands in a slot another body used before: after
    `World::clear()` (Stack Lab Reset; the ramp inherited the bottom tower box's correction), and
    within a single run when another body is destroyed (`BodyStorage::remove()` swaps the last
    body into the freed slot). Moving bodies in and out of local storage (MPI migration) can
    re-index bodies the same way; not tested.
  - Fresh worlds were unaffected (the arrays start zeroed), as was the ground plane (slot 0,
    which the previous world's plane also left untouched), which is why only the ramp showed it.
  - Fix: `initializeVelocityCorrections()` zeroes `dv`/`dw` first, in `HardContactEulerLagrange`,
    `HardContactAndFluid` and `HardContactSemiImplicitTimesteppingSolvers`.
    `tests/interface/pe_fixed_body_stale_correction_test.cpp` covers the fresh, after-clear and
    after-destroy cases (fails without the fix: drift 2.15 and 2.49).

- **Cylinder-plane** had no contact generation (`collideCylinderPlane()` was an empty stub;
  cylinders fell through the ground). Now up to four rim points per end cap;
  `tests/interface/pe_cylinder_plane_contact_test.cpp`.
- **Single-point cylinder contacts.** Box-cylinder, cylinder-cylinder and capsule-cylinder
  produced one GJK/EPA contact, so a cylinder could not stand on a box or on another cylinder.
  Now multi-point manifolds by feature clipping (`MaxContacts::addFeatureManifold()`);
  `tests/interface/pe_cylinder_manifold_test.cpp`. Effect in Stack Lab: drift of the top body
  7.6e-3 -> 3.5e-3 (upright cylinder stack) and 3.1e-2 -> 2.1e-3 (mixed-shape stack).
