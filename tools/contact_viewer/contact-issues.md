# Contact issues and observations

Technical problems and engine behaviour found with the contact viewer (`README.md` in this
folder covers usage). Observed with the repo's default `pe::response::HardContactEulerLagrange`
solver and `pe::detection::fine::MaxContacts`.

## Open problems

### Penetration correction adds momentum (split impulse)

The hard-contact solvers remove penetration with a velocity that stays in the bodies, so every
penetration, however it arises, is paid back as kinetic energy. The concrete behaviour and the
measurements are under "Penetration becomes velocity" in "Solver and engine behaviour to know";
this entry is the proposed remedy.

How it works today (`HardContactEulerLagrange`; `HardContactAndFluid` and
`HardContactSemiImplicitTimesteppingSolvers` have the same structure):

1. Contact caching: `dist_[j] = c->getDistance()`, scaled by `erp_` when negative (the Baumgarte
   term).
2. Relaxation (e.g. `relaxApproximateInelasticCoulombContactsByDecoupling()`): the normal
   constraint is on `gdot_n + dist_[i] / dt`, i.e. a penetrating contact requires the bodies to
   separate at `erp * |d| / dt`. The impulses `p_` go into the velocity corrections `dv_` / `dw_`.
3. Integration (`integratePositions()`): positions are advanced with `v_ + dv_` and the same
   velocity is stored back in the body (`body->v_ = v`). The separation velocity that corrected
   the penetration therefore survives the step.

Consequences: an initially penetrating state launches the bodies (7 m/s for 0.01 at dt = 1e-3),
a smaller dt makes it worse (the velocity scales with 1 / dt), bodies landing with a
one-step penetration bounce at restitution 0, and a deep penetration from any cause (a missed
contact, a hard mouse drag, a large dt) ends in a blow-up (~266 m/s for 0.38 deep).

Proposed remedy: split impulse (pseudo velocities), as used e.g. in Bullet:

- Solve the contacts twice per step: the velocity solve as now but with the Baumgarte term
  removed for penetrating contacts (`dist_ = min( dist, 0 )` becomes 0, only positive gaps are
  kept as allowed approach), and a second, position-only solve on separate pseudo-velocity
  corrections `dvp_` / `dwp_` whose target is `erp * |d| / dt` for the penetrating contacts only
  (no friction, non-negative normal impulses).
- Integrate positions with `v_ + dv_ + dvp_` (and the angular counterpart), but store only
  `v_ + dv_` as the new body velocity. The penetration is removed at the same rate as now, the
  pseudo velocity is discarded, and no momentum is added.
- Cost: a second relaxation loop over the penetrating contacts only (usually few); under MPI the
  pseudo corrections need the same synchronisation as `dv_` / `dw_` (`synchronizeVelocities()`).
- Keep it switchable (like `setAdaptiveBaumgarteCapping()`), default off at first, so existing
  CFD-coupled runs stay bit-identical until it is validated.
- Validation: the corner-face launch (top box must not rise), the drop at restitution 0 (no
  bounce), resting stacks (no change in drift), and the fixed-ramp and cylinder tests.

A cheaper stopgap: cap the correction velocity per contact to an absolute value (e.g. a
fraction of a body size per step, or a user-set maximum in m/s) instead of the current
`setAdaptiveBaumgarteCapping()` limit `characteristic length / ( dt * aggressiveness )`, which is
20 m/s for a unit body at dt = 1e-3 and does not bite. A cap limits the damage of a deep
penetration but still adds momentum and still makes small dt worse; the split impulse removes
the cause.

Neither is implemented. Pair Lab preset "ISSUE: box on box 0.05 deep (correction launch)" with
"start from a touching state" unticked shows the launch (35 m/s at dt = 1e-3).

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
- **Primitive vs triangle mesh without a DistanceMap** (box, capsule, cylinder, ellipsoid) goes
  through GJK/EPA: one contact, the mesh treated as convex; sphere-mesh uses a brute-force
  closest-triangle search (one contact, works for non-convex meshes). With a DistanceMap all five
  pairs use it (see Resolved).
- **DistanceMap manifolds are limited to six contacts per pair** (`emitDistanceMapContacts()`,
  deepest clusters first, up to five per cluster), and the mesh-mesh path samples only the query
  mesh's vertices, edge midpoints and face barycentres: a coarse query mesh on a fine one can
  miss shallow contacts between its samples.
- **Primitive-mesh DistanceMap sampling** (`collideTMeshWithDistanceMap()`): with a flat
  resting face all samples have nearly equal depth, so the "deepest" representative is chosen
  by sample index and can hop between frames (a centroid tie-break would avoid it). Samples
  deep inside the mesh get the nearest-surface normal, like every signed-distance method. The
  sphere and ellipsoid lattices are capped at 400 points (their deepest point is exact through
  the support-point sample; the cap only limits manifold points on very large bodies).
  DistanceMap pairs emit hard contacts only (no lubrication contacts).
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
  scenarios touching. The engine-side remedy (split impulse) is written up under "Open problems":
  "Penetration correction adds momentum".
- **Dropped boxes bounce at restitution 0.** Landing at ~4.9 m/s with dt = 2e-3 penetrates ~1 cm
  in one step; the boxes bounce back at ~0.9 m/s and settle after ~1.8 s. Very likely the same
  mechanism as above (the landing penetration is corrected with a velocity that stays in the
  body); lowering the error reduction reduces it.

## Resolved

- **Samples at a mesh's convex edges attributed to the nearer side face.** A primitive resting
  0.01 deep on the top of a box-like mesh has samples on the footprint boundary that are 0.01
  from the top face but 0 from a side face, and samples within a grid cell of the edge see a
  blend of both normals: the field reported sideways or tilted normals at too small a depth,
  and such a contact resists sliding across the edge (the solver reads the motion as an
  approach to the side face). The plane-mesh path had the same for the outline of a
  flat-bottomed mesh on a plane (depth 0). Remedy in `emitDistanceMapContacts()`: the samplers
  now carry the primitive's own surface normal with each sample (the plane path: the plane
  normal), and a candidate whose body normal is anti-parallel to a planar patch's mesh normal,
  i.e. a sample of the face pressing against that patch, takes the patch's normal and its depth
  below the patch's face plane, whether it sits inside the patch with a blended normal or in a
  neighbouring cluster with the side face's normal (then also required: below the face plane
  and adjacent). "Planar" means at least half of the patch's members have field normals within
  2 degrees of the deepest one's; a curved contact band (torus tube) spreads them over 8 to 10
  degrees and is left alone, as is a genuine contact with a second face, whose body normal is
  anti-parallel to that face, not to the patch. Mesh-mesh candidates carry no body normal and
  are unaffected. Tests: slab on a plane, capsule across the slab and a radius-3 cylinder cap on
  the slab now give every contact the face normal at depth 0.01 (outline included); a cylinder
  lying in a V-groove keeps 5 + 5 contacts with the two slopes' own normals.

- **Sample pitch of large primitives on a DistanceMap.** The box, capsule and cylinder samplers
  spread a capped number of samples over the whole body (25 per edge), so a 5 x 5 box over a
  torus with a 0.2 thick tube was sampled every 0.21 and the 0.087 wide contact band fell
  between the samples: no contact, the box sank until a sample entered the tube, was thrown
  back out, and so on. The samplers now cover only the part of the body inside the mesh's
  bounding box (transformed into the body frame, conservatively), at twice the grid spacing:
  faces, wall rings and cap rings outside that region are skipped, ring points outside it are
  dropped, and cap rings only at radii the region reaches. The overlap is never larger than the
  mesh, so the sample count is bounded by the mesh's resolution instead of the body's size, and
  the clustering link radius is 2.5 grid-based pitches. Test: the 5 x 5 box on the small torus
  (contacts on the tube crest at the right depth) and a cylinder of radius 3 standing on the
  2 x 2 slab (contacts spanning the slab). Pair Lab preset "large box on small torus (overlap
  sampling)"; --smoke checks it at five positions.

- **DistanceMap clustering with a fixed radius** (`emitDistanceMapContacts()`): a ball of a few
  grid cells around a seed fragmented a flat resting patch into many clusters (which is why the
  plane-mesh path had its clustering compiled out and emitted every penetrating sample: 52
  contacts for a resting torus, 200+ for deeper overlaps), while the primitive's extent as
  radius merged two separate patches with parallel normals into one (a box bridging the torus
  got three contacts for both ends). Now connected components: candidates within 2.5 sample
  pitches of each other with agreeing normals are linked transitively, so a patch of any size is
  one component and separate patches stay apart; components are emitted deepest first, each with
  its deepest point plus the members farthest along eight tangent directions (the outline), at
  most six per component and twelve per pair. The plane-mesh path uses it again (mesh, plane,
  plane normal, midpoint placement as before). Results: bridging box 4 + 4 contacts, slab on a
  plane 6 contacts spanning the face, box flat on a slab 5. Pair Lab presets "box bridging the
  torus (two patches)" and "torus on the ground (plane-mesh manifold)".

- **Primitive-mesh contacts ignored the DistanceMap.** Sphere-, box-, capsule-, cylinder- and
  ellipsoid-mesh contacts came from GJK/EPA (one contact, the mesh treated as convex; a sphere in
  the hole of a torus was "penetrating") or a brute-force closest triangle (sphere), even when the
  mesh had a DistanceMap; only plane-mesh and mesh-mesh used it. Now
  `MaxContacts::collideTMeshWithDistanceMap()` samples the primitive's surface at about twice the
  grid spacing (plus its support point against the mesh normal at its centre, so the deepest
  point of a smooth primitive is exact), looks the samples up in the signed distance field and
  builds the manifold with the clustering shared with the mesh-mesh path
  (`emitDistanceMapContacts()`, now with a caller-chosen cluster radius: the primitive's size, so
  one coherent contact patch is one cluster whose extremal points span the patch, and the deepest
  clusters are emitted first so the deepest contact is never dropped by the six-contact limit).
  Without a DistanceMap the previous paths are unchanged. The debug-only assert that restricted
  `createTriangleMesh( ..., vertices, faces, ... )` to convex meshes is gone (the non-convex AABB
  path and the DistanceMap handle them, as for meshes loaded from files).
  `tests/interface/pe_primitive_mesh_distancemap_test.cpp` (CGAL builds): each primitive on the
  tube of a torus against the analytic torus distance and normal, separated, in the hole (no
  contact), both dispatch orders, a translated and rotated torus, the GJK/EPA path without the
  DistanceMap, and on a slab mesh a box resting flat (six contacts spread over the whole face),
  a capsule whose ends lie outside the grid and a cylinder whose centre lies outside the grid.
- **DistanceMap nodes on the mesh surface had a zero normal** (`DistanceMap.cpp`, the
  `query == closest` case). For a mesh face lying on a grid plane, e.g. the top of a box-like
  mesh, that is a whole plane of zero normals, which the trilinear interpolation blends into
  short, useless normals in the adjacent cell layer (found when the box-on-slab test produced no
  contacts). Such nodes now take the closest triangle's face normal. Affects every DistanceMap
  path (mesh-mesh, plane-mesh, primitive-mesh).

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
