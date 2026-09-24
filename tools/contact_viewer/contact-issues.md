# Contact issues and observations

Technical problems and engine behaviour found with the contact viewer (`README.md` in this
folder covers usage). Observed with the repo's default `pe::response::HardContactEulerLagrange`
solver and `pe::detection::fine::MaxContacts`.

## Open problems

### Box-box: no preference for face axes

`MaxContacts::collideBoxBox` takes an edge-pair axis as soon as its depth beats the best face
axis by `accuracy`. ODE's `dBoxBox`, which it is ported from, requires a 5 % margin
(`fudge_factor = 1.05`) because edge axes are numerically fragile; the margin would give more
stable contact types between nearly aligned boxes. Not needed for correctness since the
near-parallel edge fix (see Resolved); not implemented.

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
- **Penetration becomes velocity (Baumgarte stabilisation).** The solver corrects a contact
  distance `d < 0` by demanding a separation velocity `erp * |d| / dt` (`dist_[j] *= erp_` in
  the contact caching of `HardContactEulerLagrange`), and that velocity stays in the bodies
  after the step: there is no split impulse / position projection that would remove the
  penetration without adding momentum. Consequences:
  - an initially penetrating state launches the bodies: Pair Lab "box on box: corner-face"
    (0.01 penetration) throws the top box up at exactly 7.000 m/s at dt = 1e-3 (3.5 m/s at
    2e-3, 2.0 m/s with error reduction 0.2, 0 with error reduction 0 or a touching start);
  - a smaller dt makes it worse, not better;
  - `setAdaptiveBaumgarteCapping()` does not help at these scales: its limit is
    `characteristic length / ( dt * aggressiveness )`, 20 m/s for a unit body at dt = 1e-3.
  Pair Lab therefore separates a penetrating posed state before the first step ("start from a
  touching state", on by default; untick it to see the raw response). Stack Lab builds its
  scenarios touching. An engine-side remedy would be a split-impulse (pseudo-velocity) position
  correction; not implemented.
- **Dropped boxes bounce at restitution 0.** Landing at ~4.9 m/s with dt = 2e-3 penetrates ~1 cm
  in one step; the boxes bounce back at ~0.9 m/s and settle after ~1.8 s. Very likely the same
  mechanism as above (the landing penetration is corrected with a velocity that stays in the
  body); lowering the error reduction reduces it.

## Resolved

- **Box-box: near-parallel edges replaced a real contact by a spurious "gap".** Observed in Pair
  Lab: a box driven edge first into another box's face passed 0.35 into it, then blew up. Before
  the pass-through `collide( A, B )` reported one edge-edge contact with a *positive* distance
  (+0.30 ... +0.22, shrinking at the approach speed), a horizontal normal ~24 degrees off the
  face normal and a contact point jumping between z = 0.5 and 0.0; the solver saw a gap and
  applied no reaction.
  - Cause: in the separating-axis test of `MaxContacts::collideBoxBox` (ported from ODE's
    `dBoxBox`) an edge-pair axis `edge_a x edge_b` was skipped only below machine epsilon. For
    edges parallel up to rounding (cross product 1e-16 ... 5e-16) the unnormalised separation is
    rounding noise and `sum /= length` an O(1) value of random sign; a positive one beat every
    (negative) face value and the routine emitted one edge-edge contact with `dist = maxDepth > 0`
    instead of the real contact. The unnormalised rejection test never caught the noise.
  - Evidence: 8.5 million near-contact configurations with near-parallel edges in general
    orientations gave 14,410 positive-distance contacts, all with the boxes overlapping (GJK
    distance 0, corners up to 0.46 deep). Clean edge-first impacts, edge-versus-face sweeps and
    resting aligned stacks were not affected; it needs near-parallel edges in general
    orientations (tilted or tumbling boxes).
  - Fix: edge pairs within 1e-6 of parallel are skipped (they add no separating directions beyond
    the face normals: exact for parallel edges, conservative by ~1e-6 times the box size for
    nearly parallel ones), and for the remaining axes the rejection also tests the normalised
    separation against `contactThreshold`, so no box-box contact has `dist > contactThreshold`.
  - `tests/interface/pe_box_box_parallel_edges_test.cpp`: 1.7 million random near-parallel
    configurations against GJK (no positive-distance contact, no contact for separated boxes, no
    missed overlap, depth at least that of the deepest corner) plus a clean edge-versus-face
    approach. Without the fix: 2,862 positive-distance contacts (worst +0.57) and 6,586 too
    shallow.
  - The blow-up after the pass-through came from the position correction described under "Solver
    and engine behaviour to know".

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
