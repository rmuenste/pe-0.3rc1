# Contact issues and observations

Technical problems and engine behaviour found with the contact viewer (`README.md` in this
folder covers usage). Observed with the repo's default `pe::response::HardContactEulerLagrange`
solver and `pe::detection::fine::MaxContacts`.

## Open problems

### Ramp scenario depends on what ran before it

Built fresh, the boxes on the 20° ramp stick at mu = 0.4 as they should (tan 20° ≈ 0.36):
kinetic energy ~1e-12, drift of the top box ~8e-7 after 1 s. After a Reset from a scenario that
ended with boxes resting on boxes, the rebuilt ramp boxes get no reaction from the fixed ramp and
fall through it, although `World::clear()` ran in between.

Reproduction (Stack Lab, 500 steps of dt = 2e-3 each, same process):

| Run before the ramp | Ramp E_kin after 1 s | Top-box drift |
|---|---|---|
| nothing / ramp / drop | 9.7e-13 | 8.0e-7 |
| box tower | 3.1e+1 | 2.4 |
| box pyramid | 2.2e+2 | 2.9 |
| brick wall | 3.6 | 0.95 |
| mixed-shape stack | 0.52 | 0.29 |

A step trace after the tower shows the first ramp box falling at exactly g from the first step
(v_z = -g dt, -2 g dt, ...) while the solver still reports contacts, i.e. none of them acts on
that box.

- Only the ramp is affected: it is the one scenario with a fixed body other than the ground
  plane (a fixed box). Every other scenario gives identical numbers whatever ran before.
- Not caused by the multi-point cylinder manifolds: the old and the new `MaxContacts.h` give
  identical ramp numbers in every order.
- Ruled out: randomness in the solver (none), warm starting (impulses `p_` are reset per step),
  stale per-body solver arrays (`v_`/`w_`/`dv_`/`dw_` are rewritten every step), static state in
  `collideBoxBox` (none), rotating the ramp after `setFixed( true )` (fixing it last changes
  nothing). `World::clear()` clears the coarse detector (`HashGrids::clear()`: grids,
  `nonGridBodies_`, `bodiesToAdd_`) and deletes the bodies.
- Still to check: the coarse detector's handling of fixed non-plane bodies after a clear, and
  state kept by the collision system outside `clear()` (attachables, joints, contact pool).

Workaround: restart the viewer before trusting a ramp run.

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

- **Cylinder-plane** had no contact generation (`collideCylinderPlane()` was an empty stub;
  cylinders fell through the ground). Now up to four rim points per end cap;
  `tests/interface/pe_cylinder_plane_contact_test.cpp`.
- **Single-point cylinder contacts.** Box-cylinder, cylinder-cylinder and capsule-cylinder
  produced one GJK/EPA contact, so a cylinder could not stand on a box or on another cylinder.
  Now multi-point manifolds by feature clipping (`MaxContacts::addFeatureManifold()`);
  `tests/interface/pe_cylinder_manifold_test.cpp`. Effect in Stack Lab: drift of the top body
  7.6e-3 -> 3.5e-3 (upright cylinder stack) and 3.1e-2 -> 2.1e-3 (mixed-shape stack).
