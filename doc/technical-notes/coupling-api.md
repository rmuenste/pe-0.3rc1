# The typed C++ coupling API (`pe::coupling`)

`pe::coupling` is the C++ interface through which a particle-resolved CFD code drives pe with
the fictitious boundary method (FBM). The CFD code asks which mesh points lie inside which rigid
body, imposes the rigid-body velocity there, integrates the hydrodynamic force and torque on
each body, hands them to pe and lets pe advance the bodies (contacts, gravity, buoyancy where
the solver owns them). The API covers exactly that loop, plus build information and
checkpoints. It was written for FEAT3 (C++), whose application-side wrapper
`FEAT::Control::RigidBody::PhysicsEngine` (`control/rigid_body/pe_world.cpp` in FEAT3) it
replaces, but nothing in it is FEAT3-specific.

Stage 1 (this state): serial-mode semantics, i.e. every CFD process holds the complete world
(pe built without MPI, or an MPI build on one process; on several processes see
[MPI builds](#mpi-builds)). Particle-resolved FBM only; there is no unresolved
Euler-Lagrange path.

## Files

| Where | What |
|---|---|
| `pe/interface/coupling/Types.h` | `BodyId`, `noBody`, `MaterialId`, `Vec3d`, `Quatd`, the enums `ShapeKind`, `Selection`, `QueryMethod`, `BodyForce`, `Setting`, the structs `BodyState`, `KinematicState` |
| `pe/interface/coupling/BuildInfo.h` | `BuildInfo`, `buildInfo()`, `isSupported( Setting )` |
| `pe/interface/coupling/Engine.h` | `class Engine` (includes the other headers) |
| `pe/interface/coupling/Export.h` | `PE_COUPLING_API`, the symbol-visibility attribute |
| `src/interface/coupling/Engine.cpp`, `BuildInfo.cpp` | Implementation, compiled into every pe library |
| `src/interface/coupling/SolverFeatures.h` | Internal: compile-time detection and dispatch of solver-dependent features |
| `src/interface/PointOwnership.h` | Internal: containment test, ownership rule and traversals, shared with `src/interface/object_queries.cpp` |
| `tests/interface/pe_coupling_*` | Tests, see [Tests](#tests) |

**The public headers include C++ standard library headers only**, declare nothing outside
`pe::coupling` and use no `using` at namespace scope. All pe types live behind a pimpl. The only
macros are the include guards and `PE_COUPLING_API` (`Export.h`), a documented exception: pe's
shared library is built with `-fvisibility=hidden`, so the API marks its own symbols
`visibility("default")` (GCC/Clang; empty elsewhere, where pe builds no shared library). It
depends on no pe header and not on `config.h`. A CFD code can therefore include `Engine.h` in
any translation unit, also one that includes `<mpi.h>`. That matters because pe's own headers,
in a build without MPI, define MPI stand-in types (`pe/core/MPI.h`) that clash with `<mpi.h>`;
with the old wrapper FEAT3 had to confine pe to a single translation unit for that reason. Link
against `pe::pe` (`pe_static` or `pe_shared`). The tests `pe-coupling-header-standalone-*`
compile each header with the bare compiler (no pe flags, no forced `config.h`, `-Werror`), alone
and after `<mpi.h>`; `pe-coupling-shared-smoke` links a coupling-only program against `libpe.so`.

## Minimal example

```cpp
#include <pe/interface/coupling/Engine.h>
namespace pc = pe::coupling;

const pc::BuildInfo info( pc::buildInfo() );
if( info.constraintSolver.find( "HardContactAndFluid" ) == std::string::npos )
   throw std::runtime_error( "pe must be built with HardContactAndFluid" );

pc::Engine engine;
engine.setGravity( { { 0.0, 0.0, -9.81 } } );
engine.setFluidDensity( 970.0 );
engine.setFluidViscosity( 0.373 );
engine.setSplitImpulse( true );
const pc::MaterialId particle( engine.createMaterial( "particle", 1120.0, 0.0, 0.25, 0.25 ) );
const pc::MaterialId wall( engine.createMaterial( "wall", 2500.0, 0.0, 0.25, 0.25 ) );
engine.createPlane( { { 0.0, 0.0, 1.0 } }, 0.0, wall );          // z = 0, fluid above
const pc::BodyId sphere( engine.createSphere( { { 0.0, 0.0, 0.1 } }, 0.0075, particle ) );

std::vector<pc::BodyState> bodies;
std::vector<pc::BodyId> owners( numDofs );
for( ... coupling steps ... ) {
   engine.states( pc::Selection::movable, bodies );               // rigid velocities to impose
   engine.refreshSpatialIndex();                                   // positions changed
   engine.classify( dofCoords, numDofs, owners.data(), pc::QueryMethod::spatialIndex );
   ... solve the flow, integrate force/torque per body, reduce them over all CFD processes ...
   for( const pc::BodyState& b : bodies )
      engine.addLoad( b.id, force[b.id], torque[b.id] );        // every body the flow acts on
   engine.applyLoads( 0.5, dt );                                   // under-relaxation 0.5
   engine.advance( dt, 1 );                                        // one pe step per CFD step
}
```

## Conventions

- **Units.** pe is unit-agnostic. All quantities are in one consistent unit system chosen by
  the caller (SI in FEAT3). pe never converts.
- **Vectors** are Cartesian `(x, y, z)` in the global frame, `std::array<double,3>`.
- **Quaternions** are `(r, i, j, k)`, real part first, unit length, rotating body-frame vectors
  into the global frame (pe's `Quat` order).
- **Planes.** `createPlane( n, d, ... )` creates the plane $\hat n \cdot x = d$. pe normalizes
  `n`; `d` is the offset along the *unit* normal. The normal points into the fluid; the
  half-space $\hat n \cdot x < d$ behind the plane is solid (pe's "inside", see
  [plane-conventions.md](plane-conventions.md)).
- **Identity.** A body is its pe system id (`BodyId = std::uint64_t`), assigned by pe at
  creation, stable for the life of the body, not reused in a process -- except after a native
  reload of the world by pe's binary checkpoint reader, which destroys all bodies and restores
  pe's id counters (recreate the Engine then). `noBody` (the maximum `uint64`) is never
  assigned. A world rebuilt in a fresh process by the same creation calls in the same order
  gets the same ids.
- **Precision.** The coupling API is supported and tested with pe built in double precision
  (pe's default, `pe/config/Precision.h`). Values are still checked after their conversion to `pe::real`, and
  `BuildInfo::realSize` / `realDigits` report the type, but no promise is made for float or
  long double builds.
- **Errors** are exceptions (`std::invalid_argument` for bad arguments, `std::runtime_error`
  for unsupported or unimplemented operations and bad checkpoints, `std::logic_error` for call
  order mistakes). The API writes nothing to stdout or stderr. It is not thread-safe. Each
  function validates and prepares everything that can throw (allocations, every value after
  its conversion to `pe::real`) before its first change, so what a throwing call leaves behind
  is, per operation:

  | Operation | World and Engine | Caller's buffers / stream |
  |---|---|---|
  | all not listed below | unchanged | unchanged (`bodyIds()`, `states()` replace the vector only on success) |
  | `refreshSpatialIndex()` | pe's guarantee: on an allocation failure the index can be incomplete (pe removes a body from its old cell before adding it to the new one) | - |
  | `createMaterial()` | API checks first; if one of pe's table allocations fails after pe registered the material, it stays (pe's guarantee) | - |
  | `createSphere()`, `createPlane()` | unchanged when the API's or pe's argument checks throw (both run before registration); if an allocation fails during pe's registration, pe's own guarantee applies (pe files the body with the coarse detector before adding it to the body storage) | - |
  | `owners()` | unchanged | `ids` may be cleared |
  | `classify()`, `containsPoints()` | unchanged | invalid arguments are found before the first result is written; an allocation failure later (spatial-index candidates per point, CGAL mesh predicates) can leave the results of earlier points written |
  | `advance()` | its checks are local and run before any collective and any step; a pe step that throws may leave bodies partially moved (pe gives no guarantee) | - |
  | `writeCheckpoint()` | never changed | text built in memory, written with one unformatted `write()`; on a stream failure the stream may hold a part |
  | `readCheckpoint()` | unchanged (everything validated before the first change; the commit cannot throw) | the stream may be consumed up to the error |
- **Inputs are checked against pe's preconditions** (`std::invalid_argument`), before pe sees
  them, so that in the supported double-precision build nothing that passes the API trips a pe
  assertion (each value is checked after its conversion to `pe::real`): finite vectors and
  material coefficients everywhere; quaternions unit within
  pe's own tolerance ($|r^2+i^2+j^2+k^2-1| <$ `Limits<real>::accuracy()`, 1e-8 for double, never
  normalized by the API, so restarts stay bitwise); plane normals whose squared length is a
  finite, normal (not subnormal) positive `real` (pe's normalization would otherwise overflow
  or underflow; a valid normal is handed to pe unchanged); spheres, and bodies moved by
  `setKinematicState()` or `readCheckpoint()`, that are representable at their position (the
  ends of the bounding box must be finite and strictly ordered around the centre in every
  coordinate, else the box overflows or collapses). The check is exact for spheres (pe's
  padded radius). For other bodies whose target orientation equals the current one it tests
  the current per-axis offsets of the box from the position at the target position: a body at
  its own pose (every pose of a checkpoint this build wrote, read back into the same world) is
  accepted; for a translated target the offsets come from world coordinates pe has already
  rounded, so a body whose half extent is about one unit in the last place of its coordinate
  can be refused at a target pe could represent (or accepted where its box collapses). For
  another orientation the target box is not known without moving the body, so an upper bound
  is tested (the distance from the position to the farthest corner of the current box): it
  never refuses a pose pe can represent (short of positions within a few body sizes of the
  largest `real`) -- so this build reads its own checkpoints -- but can accept a pose whose box
  collapses along one axis (a thin body turned so that its thin side faces a huge coordinate;
  pe asserts on it in Debug). Only the start and the target pose are checked: the override sets
  the position before the orientation (the FEAT3 wrapper's order, kept for bitwise
  equivalence), so pe also forms the box of the intermediate pose "target position, current
  orientation", which for a thin body can collapse although both checked poses are
  representable (pe asserts on it in Debug); error reduction in $[0, 1]$, contact relaxation positive, at least
  one solver iteration; accumulated loads that stay finite (`addLoad()` sums into a temporary
  and keeps the previous sum on overflow) and effective loads finite in `real`; query points
  finite and, for the spatial index, within the range the hash grid can index ($|x|$ up to
  $2^{d-2}$ times its smallest cell span, $d$ the bits of `size_t`: the grid converts
  $|x|$/span to `size_t`).

## Who owns gravity and buoyancy

The constraint solver is fixed when pe is built (`PE_CONSTRAINT_SOLVER`), and the solvers make
opposite decisions about body forces
([euler-lagrange-solver-conventions.md](euler-lagrange-solver-conventions.md)):

| Solver | Gravity | Buoyancy | The load handed to pe must ... |
|---|---|---|---|
| `HardContactAndFluid` (particle-resolved FBM) | `appliedBySolver` | `appliedBySolver`: $V(\rho_p-\rho_f)\,m^{-1}\,g\,\Delta t$ per step for spheres, capsules, cylinders, boxes, ellipsoids ($V$ the volume enclosed by the surface, $m$ the body's mass) | **not** contain the submerged weight or the hydrostatic pressure force |
| `HardContactSemiImplicitTimesteppingSolvers` | `appliedBySolver` | `appliedByDriver` | contain the buoyancy ($-\rho_f V g$) if it matters |
| `HardContactEulerLagrange` (pe's default) | `appliedByDriver` | `appliedByDriver` | contain the submerged weight $(m-\rho_f V)\,g$ with $m$ the body's mass and $V$ its displaced volume (`BodyState::mass`, `volume`); for a solid body this is $(\rho_p-\rho_f) V g$ |
| any solver without a declaration | `unknown` | `unknown` | (nobody classified it) |

For a solid body $m = \rho_p V$, so the increment is $(1-\rho_f/\rho_p)\,g\,\Delta t$; a hollow
sphere has the shell mass, so its increment is larger.

`BuildInfo::gravity` / `buoyancy` (enum `BodyForce`) report this for the linked library. They
come from the collision systems' explicit declarations `static constexpr bool appliesGravity` /
`appliesBuoyancy` (next to `hasLubricationStage`): behaviour cannot be detected from a type. A
solver that declares nothing is reported as `BodyForce::unknown`, not as "the driver applies
it", so a driver is never told to supply gravity for a solver nobody looked at; the refusal
message of `setGravity()` / `setFluidDensity()` says "the driver owns" only for a declared
`false`, and "does not declare" for an unknown solver. Note that
`HardContactAndFluid` applies *bare* gravity, without buoyancy, to every other shape (triangle
meshes, unions).

## Build information and settings

`buildInfo()` needs no engine and touches no pe singleton (callable before `MPI_Init`):
`mpi`, `constraintSolver` (the full `pe_CONSTRAINT_SOLVER` name), `gravity`, `buoyancy`,
`realSize` (`sizeof(pe::real)`), `realDigits` (its mantissa digits), `cgal`, `mobileInfinite`,
`sleepMode` (nonzero `PE_SLEEP_THRESHOLD`), `checkpointsSupported` (no sleep mode and
`realDigits` $\le 53$). A CFD code that needs a particular solver checks `constraintSolver`, as
FEAT3 does for `HardContactAndFluid`.

A world setting whose availability depends on the solver throws `std::runtime_error` naming
the setting and the solver when the solver lacks it; `isSupported( Setting )` asks beforehand:

| Setting / setter | Available when | Detection |
|---|---|---|
| `gravity` / `setGravity()` | gravity is `appliedBySolver` | declared `appliesGravity` |
| `fluidDensity` / `setFluidDensity()` | buoyancy is `appliedBySolver` | declared `appliesBuoyancy` |
| `fluidViscosity` / `setFluidViscosity()` | always (only lubrication models read it) | - |
| `errorReduction` / `setErrorReduction()` | `setErrorReductionParameter( real )` exists | `std::void_t` detector |
| `maxIterations` / `setMaxIterations()` | `setMaxIterations( size_t )` exists | detector |
| `contactRelaxation` / `setContactRelaxation()` | `setRelaxationParameter( real )` exists | detector |
| `splitImpulse` / `setSplitImpulse()`, `splitImpulse()` | `setSplitImpulse( bool )`, `getSplitImpulse()` exist | detector |

Under `HardContactEulerLagrange`, `setGravity()` and `setFluidDensity()` throw even for zero
values: a gravity set there would be silently ignored, which is exactly the trap of the table
above. `setFluidDensity()` also switches pe's liquid-solid mode on (`World::setLiquidSolid`), as
the FEAT3 wrapper did. All three solvers pe's tests build have the contact parameters and split
impulses.

**The contact relaxation acts only between processes.** pe applies `setRelaxationParameter()`
only to the velocity corrections exchanged between processes (`synchronizeVelocities()`, which
returns at once on one process), in all three solvers. On one process the value has no effect;
`pe-coupling-build-info*` pins that, `pe-coupling-mpi` shows its effect across a subdomain cut.

**Detection.** Every solver-dependent call goes through a dispatch *template* on the
collision-system or detector type (`SolverFeatures.h`, `PointOwnership.h`): only inside a
template is the discarded branch of `if constexpr` not instantiated, so a type without the
member really compiles (`pe-coupling-detection` checks that with stand-in types). This does not
make the whole library build under any solver: under `pe::response::FFDSolver`, for example,
`World.cpp` and the extern "C" interface do not build on master either.

## Engine lifetime

- `pc::Engine` is a handle on pe's process-wide world. It is not copyable or movable.
- **One Engine per process at a time.** The constructor throws `std::logic_error` when another
  Engine exists (a process-wide flag, the only global state of the API). After the first one is
  destroyed a new one may be created.
- The Engine does **not** own the bodies and does not reset the world: it works on whatever is
  in pe's world, including bodies created by native pe calls (`pe::createBox(...)`) or by an
  existing setup function (`pe/interface/setup/*.h`). Destroying the Engine leaves the world
  and its bodies alone. pe's materials are global as well.
- The Engine owns the coupling state: the loads added in the current coupling step, the
  previous load $F_{n-1}, T_{n-1}$ of every body that received one, and the triangle-mesh
  volume cache. **Replacing the handle forfeits the load history** (the next load of every body
  is applied unrelaxed) **and the cache.** What lives in the world survives: the marker "loads
  applied, step not advanced yet" is kept process-wide next to the one-Engine flag (set by
  `applyLoads()`, cleared by a successful `advance()` or `readCheckpoint()`), so a new handle
  still refuses a second `applyLoads()` and `writeCheckpoint()` until `advance()` has run (a
  successful `readCheckpoint()` is allowed: it replaces the velocities and clears the marker).
- **After a native reload of the world** (pe's binary checkpoint reader: it destroys the bodies
  and restores the id counters, so ids can be issued again, and an impulse applied before the
  reload is gone with the old bodies): create a new Engine (an old one would apply its id-keyed
  history and cache to other bodies) and call `acknowledgeWorldReload()`, which declares that
  the world stands at a coupling-step boundary: it clears the process-wide marker and that
  Engine's pending loads, load history and volume cache, and pe's body-level fluid-force
  history on every local body (bodies created by the reload start without it anyway; a body
  that survived, or was given a fluid force natively since, is reset to the same state, so the
  world matches the Engine's empty load history). It is never called implicitly;
  without it, a marker left by an interrupted step keeps refusing `applyLoads()` and
  `writeCheckpoint()` (advancing just to clear it would integrate a step without its load).
  Calling it while the world still holds an applied impulse would allow a second one.
- In MPI builds MPI must be initialized before the Engine is constructed.

## Creation

- `createMaterial( name, density, restitution, staticFriction, dynamicFriction )` creates a pe
  material (unique name). The parameters only pe's soft-contact solvers read are set to the
  values of the FEAT3 wrapper (Poisson 0.3, Young 2e6, stiffness 1e6, damping 1e3 / 2e3).
- `createSphere( centre, radius, material, userId = 0 )` creates a movable sphere and returns
  its system id. In an MPI build the centre must lie in the local subdomain (pe's check).
- `createPlane( normal, offset, material, userId = 0 )` creates a fixed plane. Once MPI is
  initialized (also on one process) pe creates planes only inside a global section, so
  `createPlane()` opens one itself (unless one is open). On more than one process a plane is a
  global body: every process must make the same `createPlane()` calls in the same order, so the
  plane gets the same id everywhere (`pe-coupling-mpi` checks that).

Other shapes are created with native pe calls; the Engine enumerates and couples them all the
same.

## Enumeration and state

- `Selection::movable`: local bodies that are finite and not fixed (the particles);
  `Selection::all`: all local bodies including fixed ones and planes;
  `Selection::shadowCopies`: shadow copies of remote bodies (MPI builds; empty otherwise).
- `count( selection )`, `bodyIds( selection, ids )`, `states( selection, states )` return them in
  pe's storage order. **That order changes when a body is removed** (pe moves the last body into
  the gap); identify bodies by id. The vector forms clear and refill the caller's vector and keep
  its capacity, so a vector reused every step is not reallocated.
- `BodyState`: id, user id, `ShapeKind` (sphere, capsule, ellipsoid, box, cylinder,
  triangleMesh, plane, other for unions and the inner types), position (centre of mass; for a
  plane its anchor point), linear and angular velocity, orientation, mass (as pe stores it),
  sphere radius (0 otherwise), **volume**, axis-aligned bounding box (infinite for planes),
  fixed, shadow.
- **Volume.** pe has no generic volume: `getVolume()` exists per shape for spheres, capsules,
  ellipsoids, boxes, cylinders and triangle meshes, and the API dispatches on the shape. A
  triangle mesh's `getVolume()` sums over all faces, so the Engine computes it once per body and
  caches it (a rigid body's volume does not change); `states()` costs O(1) per body after the
  first call, and `advance()` drops the entries of meshes that left the process (also on
  several processes, where shadow copies pass through). The volume is
  the volume enclosed by the surface, i.e. the displaced fluid volume (a hollow sphere reports
  its full sphere, the value `HardContactAndFluid` uses for buoyancy). Planes, unions and the
  inner types report 0 ("pe has none"); a union's volume would need its members.
- `state( id )`, `hasBody( id )`: lookup through the id map pe's `BodyStorage` keeps for local
  bodies and shadow copies, $O(\log n)$, always consistent with additions and removals (no
  cache to go stale). For an unknown id `hasBody()` returns `false` and `state()` throws
  `std::invalid_argument`.
- `setKinematicState( id, { position, orientation, linearVelocity, angularVelocity } )` overrides
  the kinematic state of a local finite body (for restarts), with the same pe calls in the same
  order as the FEAT3 wrapper. It refuses planes, non-finite values and quaternions outside pe's
  tolerance. Without `MOBILE_INFINITE` pe ignores velocities set on fixed bodies.

## Point classification

**Rule.** A point is owned by the body containing it; a point inside several bodies is owned by
the body with the **minimum system id**. The minimum does not depend on traversal order, so both
methods agree whenever they test the same bodies. Tested are all local bodies and all shadow
copies, with this shape rule:

| Shape | Tested |
|---|---|
| sphere, capsule, box, ellipsoid | always, fixed or not |
| cylinder, triangle mesh | only when **not** fixed (fixed ones are geometry of the CFD domain, not particles) |
| plane, union, inner cylinder, inner mesh | never |

The rule is the one of the extern "C" queries (`pointInsideParticles()`,
`pointInsideParticlesAccelerated()`, `pointOwners()`, `pointOwnersAccelerated()` in
`src/interface/object_queries.cpp`); both use the same code in `src/interface/PointOwnership.h`.

**Containment is pe's own predicate** (`containsPoint()` of each shape); the API is as correct
as that predicate. **Triangle meshes in a build without CGAL:** pe's predicate
(`trianglemeshtrait/Default.h`) is valid for convex meshes only -- it accepts every point
within $10^{-6}$ of the mesh-frame origin and tests the face planes -- so a concave movable
mesh is classified wrongly (a torus owns the points of its hole; points in a lobe in front of
another face's plane are missed). This is existing pe behaviour, shared with the extern "C"
queries. pe does not expose whether a mesh is convex (the flag is protected and user-stated,
and pe marks meshes it instantiates -- shadow copies, meshes restored from a binary checkpoint
-- as convex), so the API cannot refuse such meshes. Use convex movable meshes, or a CGAL build.

**Functions.** `owner( p, method )` returns the owner or `noBody`; `owners( p, ids, method )`
all owners ascending; `classify( xyz, n, ownerIds, method )` a batch of `n` points given
interleaved (`x0 y0 z0 x1 ...`) into a caller-owned array of `n` ids; `contains( id, p )` whether
body `id` contains `p` by the same rule (so a fixed cylinder or a plane never contains a point);
`containsPoints( id, xyz, n, flags )` the same for a batch of points against one body, with one
id lookup for the whole batch (for per-body classification of mesh entities), writing 1 or 0
per point into a caller-owned `unsigned char` array. Batch forms check all points before
writing the first result.

**Methods.** `QueryMethod::exhaustive` applies pe's containment predicate to every body, $O(N)$
per point.
`QueryMethod::spatialIndex` tests the candidates of pe's hierarchical hash grid
(`HashGrids::getBodiesNearPoint()`: the 27 cells around the point in every grid level, the
non-grid bodies, and the bodies added since the last contact search). Pitfalls:

- **Stale filing.** The grid files bodies during the contact search at the *start* of a pe step,
  before positions are integrated. After `advance()`, `setKinematicState()` or any native
  position change the filing is one step old, and a point query (27 cells) can miss interior
  points of a body that moved across a cell boundary. Call `refreshSpatialIndex()` after every
  change of positions and before spatial-index queries; it re-files the bodies (no contact
  detection, no physics). `advance()` does not refresh by itself, so the stepping arithmetic
  stays exactly that of plain pe. With the refresh the two methods return the same owners
  (tested with the grid active, after motion, owner identities and owner lists; a control
  without refresh misses a moved body).
- **Inactive grid.** The grid activates only once more than 32 bodies (`gridActivationThreshold`,
  per process) went through a contact search. Below that, `spatialIndex` tests every body:
  the same answers as the exhaustive method, but $O(N)$. `spatialIndexActive()` tells which case
  applies.
- **Pending bodies.** Bodies created since the last contact search are not in the grid yet;
  `getBodiesNearPoint()` returns them anyway (`bodiesToAdd_`), so they are classified correctly.

## Loads and stepping

One coupling step is: `addLoad()` for the bodies (any number of contributions per body, they
**add up**), `applyLoads( r, dt )`, `advance( dt, substeps )`.

- `addLoad( id, force, torque )` accumulates; the body must be local, movable (finite, not
  fixed), and the load finite. A contribution whose sum with the earlier ones would not be
  finite is refused and the earlier sum stays. `clearLoads()` discards the loads added so far
  (the previous loads stay); `hasPendingLoads()` tells whether some are waiting.
  `acknowledgeWorldReload()` is for a native reload of the world only (see "Engine lifetime").
- `applyLoads( r, dt )` with $r \in (0, 1]$ processes **only the bodies with a pending load**,
  in storage order. For such a body, with $F_n$ the sum added in this step:
  $F_\text{eff} = r F_n + (1-r) F_{n-1}$, the same for the torque. The first load a body gets
  from this Engine is applied unrelaxed ($F_\text{eff} = F_n$), whenever it comes; $F_n$
  becomes $F_{n-1}$ (kept by the Engine, part of the checkpoint). $F_\text{eff}$ is set as the
  body force and applied once as a velocity update through `RigidBody::applyFluidForces( dt, 1 )`
  ($v \mathrel{+}= \Delta t\,F_\text{eff}/m$, $\omega \mathrel{+}= \Delta t\,I^{-1}T_\text{eff}$,
  with the body's DOF masks); the force slot is reset afterwards. **A body without a pending
  load is not touched at all**: its force slot (a force set by a native call survives), its
  velocity and its load history stay as they are. To relax a body against its history with a
  zero load, add a zero load. **The history of a body is kept as long as the body exists**, also
  while it is fixed (a body fixed for a while and released is relaxed against its last load);
  it is dropped only when the body is destroyed. Without pending loads the call does nothing. A second
  `applyLoads()` with pending loads before the next `advance()` throws `std::logic_error`.
  A load added for a body that has been destroyed since throws before anything changes.
- `advance( dt, substeps = 1 )` runs `substeps` pe steps of `dt / substeps` (contact detection,
  the solver's velocity prediction with the body forces it owns, contact solve, integration).
  The relaxation factor and the number of substeps are always arguments. It throws
  `std::logic_error` if loads are pending (added, not applied or cleared) — forgetting
  `applyLoads()` is caught instead of silently carried into the next step.
- **No output.** `HardContactAndFluid` prints a diagnostic block per sphere and step when
  `SimulationConfig::getCfdRank()` is 1 (the serial setups' "representative" rank, also the
  default). The collision system now has a switch for exactly that block,
  `setPredictionDiagnostics( bool )` (default on, so existing drivers print as before);
  `advance()` switches it off for the step and restores it. The CFD rank is not touched, so
  force generators and triggers running inside the step see the caller's configuration.

**Equivalence with the FEAT3 wrapper.** For a sphere that receives a load every step and
`substeps = 1` the API performs the same operations in the same order as
`PhysicsEngine::apply_hydrodynamic_load()` and `advance()` of FEAT3's `pe_world.cpp`
(double-precision blend, `setForce`, `setTorque`, `applyFluidForces( dt, 1 )`,
`World::simulationStep( dt )`); `pe-coupling-feat-equivalence` replicates the wrapper call by
call and compares 1500 steps of a settling sphere bit for bit. One caveat for the owner: the
blend used to be compiled with FEAT3's flags and is now compiled with pe's. If FEAT3 compiles
with FMA contraction enabled (e.g. `-march=native` with GCC's default `-ffp-contract=fast` in GNU
mode) and pe does not, the last bits of the relaxed load can differ from the old FEAT3 binaries.

## Checkpoint

`writeCheckpoint( os )` writes the kinematic state and the load history of all local finite
bodies; `readCheckpoint( is )` restores them. Format version 3 (only version 3 is read), text,
classic locale, 17 significant digits (every double round-trips exactly). The fixed finite bodies are
included because with `MOBILE_INFINITE` (default on) a fixed body can move with a prescribed
velocity, and a restart must not put a moving wall back at its setup position:

```
pe-coupling-checkpoint 3
bodies <n>
body <id> <x y z> <r i j k> <vx vy vz> <wx wy wz> <hasPrevious 0|1> <Fx Fy Fz> <Tx Ty Tz>
     <hasPrevFluid 0|1> <fx fy fz> <tx ty tz>
...
fixed <m>
fixedbody <id> ... (the same fields)
...
end
```

(one line per record). `<hasPrevious> <F> <T>` is the Engine's load history (the raw load
$F_{n-1}, T_{n-1}$); `<hasPrevFluid> <f> <t>` is pe's own body-level fluid-force history
(`RigidBody::hasPrevFluidForce()`, `getPrevFluidForce()`, `getPrevFluidTorque()`): the
inputs `force_`, `torque_` of the last `applyFluidForces()` call, which it stores as given.
`Engine::applyLoads()` hands its already relaxed load in as those inputs, so through the
Engine they are the effective load $F_\text{eff}, T_\text{eff}$; a native caller with a
relaxation below 1 stores its unblended force instead. `applyLoads()` goes through
`applyFluidForces( dt, 1 )`, which adds $0 \cdot f$ when the flag is set; that zero can turn a
`-0` force component into `+0`, and with it a `-0` velocity component, so a rollback without
it is not bit for bit (the earlier format version 2 lacked it and is no longer read). `readCheckpoint()` restores it with the public
setter `RigidBody::setPrevFluidForce()`. **pe's own binary checkpoint (`BodyBinaryWriter` /
`BodyBinaryReader`) does not save this body-level state** (checked: neither touches
`prevFluidForce_`, `prevFluidTorque_` or `hasPrevFluidForce_`; a body it creates starts
without history, nor is it sent with a migrating body in MPI builds). That matters more for
the `extern "C"` serial stepping (`pe/interface/support/serial_stepping.h`), which calls
`applyFluidForces()` with a relaxation of 0.35: after a restart from a binary checkpoint its
first step is applied unrelaxed instead of relaxed against the previous force.

- **Scope.** The checkpoint restores worlds of finite, non-union bodies plus planes at rest,
  with empty force and torque slots. Both functions refuse, before writing or changing
  anything, a world with a plane that has any nonzero velocity component (a kinematic wall
  under `MOBILE_INFINITE`: a rebuilt world puts it back at its setup offset; compared exactly,
  pe's vector comparison has a 1e-8 tolerance), a union (its members accumulate their own
  incremental rotations, which restoring the union's orientation does not reproduce), any
  other body of `ShapeKind::other` (inner cylinders, inner meshes), and a finite body whose
  force or torque slot is not exactly zero (a native force waiting for the next step, which the
  checkpoint does not hold; `applyLoads()` and pe's step leave the slots empty). Fixed finite
  bodies carry their kinematic state and their load history. `readCheckpoint()` also refuses,
  before changing anything, a record the receiving build cannot restore exactly: in a build
  without `MOBILE_INFINITE`, where pe's velocity setters skip fixed bodies, a fixed body whose
  recorded velocity components are not bit for bit the ones the body keeps (a velocity written
  by a build with `MOBILE_INFINITE`, and also a `-0` where the body holds `+0`).
- **Between coupling steps only.** `writeCheckpoint()` refuses while loads are pending and
  after `applyLoads()` until `advance()` has run (also when the Engine handle was replaced in
  between): a checkpoint taken there would let a restored run apply a second load to
  velocities that already contain the first one.
- **Streams.** The text is built in memory in the classic locale and written with one
  unformatted `write()`, so a pending width or fill of the caller's stream adds nothing. The
  read is parsed in the classic locale, decimal, skipping white space, whatever the caller set
  (a stream left in `hex` would read a count of 10 as 16), with the pending width cleared (a
  `setw(3)` would cut the first token); the caller's locale, flags and width are restored. The
  private formatting buffer is checked before its text is copied out.
  `pe-coupling-checkpoint-errors` checks this with preconfigured streams.

- **Restart contract.** Rebuild the world with the same creation calls in the same order (so the
  bodies get the same ids), then `readCheckpoint()`. Bodies are matched by id. The read stops
  after `end`, so other data may follow in the same stream (FEAT3 appends its own coupling
  state). The stream's locale is left as it was.
- **Errors** (`std::runtime_error`, and nothing is changed): not a coupling checkpoint, a version
  other than 3, a movable or fixed body count different from the world's, an id that is not a
  local body of the right kind (movable / fixed finite) or appears twice, a quaternion outside
  pe's tolerance, a truncated or malformed text. `writeCheckpoint()` refuses while loads are
  pending and on non-finite values.
- **Build options.** With a nonzero `PE_SLEEP_THRESHOLD` pe puts slow bodies to sleep; a
  restore would wake them and lose the accumulated motion measure. A `pe::real` wider than
  double cannot round-trip through the 17-digit text. Both checkpoint functions refuse with an
  error in such builds (`BuildInfo::checkpointsSupported`); the default build supports them.
- **What is guaranteed -- two separate statements.**
  1. *Always, and tested:* `readCheckpoint()` restores bit for bit the positions, orientations,
     velocities and load history that `writeCheckpoint()` saved (`pe-coupling-checkpoint-write`
     reads a checkpoint back into the live world and compares every bit; every restart test
     restores into a fresh process).
  2. *Only conditionally:* the continuation is bit-for-bit identical to the uninterrupted run
     if pe processes the contacts in the same order and with the same geometry. That holds in
     the worlds of the restart tests -- `pe-coupling-checkpoint-restart` (three spheres, a
     plane, a fixed sphere moving with a prescribed velocity, grid inactive, all bodies created
     before the first step) and `pe-coupling-checkpoint-pile-restart` (64 spheres settling into
     a pile, grid active) -- and is known **not** to hold in general:
     - when the hash grid is active: it keeps the order of the bodies in each cell from its
       history (a body leaving a cell swaps the last one into its slot), while a rebuilt world
       files them in creation order (`pe-coupling-checkpoint-shake-restart`: 48 spheres
       exchanging cells, 7e-12 m after 400 steps; the same scenario with 27 spheres, grid
       inactive, restarts bitwise);
     - when bodies were created or destroyed after stepping began, also with the grid inactive:
       the coarse detector keeps pending and filed bodies in separate lists with swap-removal,
       so a rebuilt world that replays the creations and destructions before its first step
       traverses the same bodies in another order (`pe-coupling-checkpoint-history-restart`:
       A, B, C, one step, D created, A destroyed; 1e-17 m after 400 steps);
     - when a plane's stored geometry differs from what its creation call reproduces: a wall
       that moved and stopped, and -- measured by `pe-coupling-plane-geometry` -- a tilted
       plane at rest, whose stored normal pe re-derives from the plane's quaternion in the
       first step (normal (1e-5, 0, 1) becomes (0, -1e-5, 1): the tilt moves from x to y), so
       a rebuilt world starts with the creation normal while the uninterrupted one has the
       re-derived normal.
     The hard-contact solvers apply each contact impulse at once, so the contact order changes
     the round-off; in a chaotic multi-contact flow such differences can grow. The
     characterisation tests report whether their restart was bitwise and assert only
     finiteness and a loose bound (1e-6 in the test's units). Making restarts bitwise in
     general would need pe's contact order and plane geometry to be independent of history, a
     change of pe that was deliberately not made.
- **What the caller's setup must reproduce.** Everything the checkpoint does not save: planes
  and other infinite bodies, materials, world settings, pe's step counter, solver caches, and
  the order of the bodies inside pe's coarse detector (i.e. the same creation and destruction
  calls at the same points relative to the first step).

## MPI builds

Everything compiles with `PE_USE_MPI=ON`. **On one process the Engine is fully functional**
(`pe-coupling-mpi-single`). On more than one process of pe's communicator (stage 1: reading
and stepping; stage 2, branch `feature/coupling-api-mpi`: a domain-decomposed coupling, announced
by the macro `PE_COUPLING_DOMAIN_DECOMPOSITION` (defined as 1 in `Engine.h`) so that a client can
support pe versions with and without it):

- **Decomposition.** `decomposeSlabs( planes )` (collective, before the first body, also before
  planes) gives process k slab k of a chain of slabs between planes of any orientation, pe's
  `decomposeDomainPlaneSlabs()`; `readSlabPlanes( file )` reads planes in the format of pe's
  `loadSlabPlanes()` / pe_partpy `plane_single`. `ownsPoint( x )` tells whether this process owns
  a point (exactly one does; ties go to the lower rank): a body is created by the owner of its
  centre. `processRank()` / `processCount()` report pe's communicator.
- **Reading stays available**: enumeration (`Selection::shadowCopies`, `BodyState::shadow`
  report shadow copies), states, point classification (shadow copies are classified like local
  bodies, as in the extern "C" queries). `sphereContainsPoints()` evaluates pe's sphere predicate
  (`|x - X|^2 <= r^2`, the expression of `Sphere::containsPoint()`) for a sphere the process does
  not hold (for a coupling whose processes see all bodies).
- **`advance()` works and is collective.** Before every pe step it asks pe (collectively)
  whether a synchronization is required and synchronizes first. Creating bodies sets pe's
  sync-required flag, and `HardContactAndFluid` resolves contacts before it synchronizes, so
  without this the first step would run without a neighbouring rank's body (and abort on pe's
  assertion in a debug build). A state override on several processes also triggers it, and so
  does the first step of a newly created Engine (a replaced handle may have recorded an
  override that pe does not know about). Inside a pe exclusive section `advance()` throws
  `std::logic_error` locally, before its first collective (only one rank runs the section; a
  collective there would hang).
- **Loads.** `addLoad()` / `applyLoads()` work locally: each process adds and applies the loads
  of the bodies it holds. **The coupling history does not migrate**: pe's marshalling of a
  migrating body (`pe/core/Marshalling.h`) carries position, orientation, velocities and surface
  velocity, not the body-level fluid-force history (`RigidBody::hasPrevFluidForce()` and the
  stored force/torque), and the Engine's previous raw loads live per process. Both decide the
  result bit for bit (the blend of `applyLoads()` and, through `applyFluidForces( dt, 1 )`, the
  sign of a zero). `LoadHistory`, `loadHistory()` and `setLoadHistory()` expose both parts: the
  caller reads the history of every body after `applyLoads()` and restores it on the body's
  current owner before its next `applyLoads()`. Then the trajectory is bit for bit that of one
  process (`pe-coupling-mpi-dd-2/-4` against the single-process reference, including slab
  crossings and a restore; `pe-coupling-mpi-dd-broken-history`, which skips the restore, fails).
- **`setKinematicState()`** works on local bodies, also on bodies with shadow copies (the
  shadows follow at the next `advance()`), to a position in the local or a connected neighbouring
  subdomain: pe's synchronization migrates bodies to neighbours only and deletes a body no
  neighbour takes (an outflow). Refused ("not implemented yet"): a global body (planes
  included), a shadow copy, a position beyond the neighbours.
- **Checkpoints.** `prepareCheckpoint()` (collective) checks and formats the records of every
  process (all processes throw the message of the lowest-ranked process that found an error, or
  none does) and gathers them on the root (`MPISettings::root()`), which writes them with
  `writeCheckpoint()`: format version 3, records sorted by system id; any change of the world
  in between discards the prepared text. `readCheckpoint()` is collective: every process reads
  the same text, each body must be held by exactly one process, each record is restored by its
  holder, and a restored position must lie in the holder's or a neighbouring subdomain (the body
  then migrates at the next `advance()`). A global finite body is refused on several processes.
  On one process both functions are unchanged.

The names avoid assuming that every process holds every body (`Selection::movable` means
*local* movable bodies).

## Changes to pe itself

The coupling API needed a few small changes in pe; all existing tests pass with them.

- **System ids of global bodies without MPI (`pe/core/UniqueID.h`).** pe drew local and global
  ids from two counters that both start at 0; with MPI the global ones carry a different prefix,
  but without MPI (no prefix bits) the first global body (created in `pe_GLOBAL_SECTION`) and
  the first local body both got id 0, and `BodyStorage` asserted (debug) or overwrote its id
  lookup (release). `createGlobal()` now draws from the local counter when there are no prefix
  bits. MPI builds are unchanged (they always have prefix bits, also on one process). Nothing
  in pe decodes ids; pe's own checkpoint stores both counters and the ids, so it stays
  consistent. Tested by `pe-coupling-world` (mixed creation, lookup, removal).
- **Id counters on reading an older binary checkpoint (`src/core/BodyBinaryReader.cpp`).** A
  file written before the change above carries two independent counters (e.g. one global
  sphere: id 0, local counter 0, global counter 1); after restoring them, the next body would
  get id 0 again. Without prefix bits the reader now continues the shared counter after the
  larger of the two (every restored id is below the counter it was drawn from). Tested by
  `pe-coupling-world` with a freshly written file patched into the old counter layout. The
  repair covers the same-layout case only: a reader without prefix bits reading a file written
  without prefix bits. Reading pe binary checkpoints across builds (e.g. an old serial file
  read by a one-process MPI build, whose ids carry prefix bits) is outside the scope of this
  stage. (A file from an old build that held local and global bodies already carried
  duplicate ids; that cannot be repaired on reading.)
- **Ellipsoid bounding box after a change of orientation (`src/core/rigidbody/Ellipsoid.cpp`).**
  `setOrientation()` (both overloads) and the in-place `rotate()` overloads updated the
  quaternion and rotation matrix but not the bounding box (the code was copied from the
  sphere, whose box does not depend on the orientation), so a rotated ellipsoid reported, and
  was filed in the hash grid with, the box of its old orientation until the next step. They now
  recompute the box and signal the rotation as Box does. Box, capsule, cylinder, inner
  cylinder, union and plane already did. Triangle meshes and inner meshes call the recompute
  too, but it could still use stale vertices; see the next item. Tested by `pe-coupling-world`.
- **Accessors for the body-level fluid-force history (`pe/core/rigidbody/RigidBody.h`).**
  `hasPrevFluidForce()`, `getPrevFluidForce()`, `getPrevFluidTorque()` and
  `setPrevFluidForce( has, force, torque )` expose the state `applyFluidForces()` relaxes
  against, so the coupling checkpoint can save and restore it (see "Checkpoint"). Nothing else
  changed in `RigidBody`.
- **Quaternion equality and the triangle-mesh vertex cache (`pe/math/Quaternion.h`,
  `src/core/rigidbody/TriangleMeshBase.cpp`, `src/core/rigidbody/InnerMeshBase.cpp`).**
  `operator==` for quaternions compared component 2 twice and never component 3, so two
  quaternions differing only in `k` compared equal (e.g. +45 and -45 degrees about z). The
  world-space vertex cache of triangle meshes and inner meshes used that operator, and pe's
  tolerant vector equality, to decide whether it was current. A mesh turned about z at a fixed
  centre therefore kept its old vertices and its old bounding box, and both query methods
  classified it wrongly (the containment test rejects points outside the box first). The
  operator now compares all four components (still with pe's tolerance); the cache is current
  only if position and orientation are the same values, zeros with the same sign, as those it
  was filled with (a tolerant test also keeps it stale for changes below the tolerance; one
  helper, `src/core/rigidbody/VertexCachePose.h`, compares values and signs, not storage, so
  the padding of a `long double` plays no role). The two caches were the only users of
  quaternion `==`/`!=` in the library and the tests (found by compiling the serial and the MPI
  tree with the operators marked deprecated; a text search finds none in the examples either).
  Tested by `pe-coupling-world`.
- `appliesGravity` / `appliesBuoyancy` declarations in the three hard-contact collision
  systems; `setPredictionDiagnostics()` in `HardContactAndFluid` (default on).
- `povray::Writer::setTexture( ConstInnerMeshID, ... )` was declared and called but never
  defined, so no executable could link against pe's shared library; it is now defined.

## Relation to the extern "C" interface

| | extern "C" interface (`pe/interface/*.h`) | `pe::coupling` |
|---|---|---|
| Client | FeatFloWer (Fortran) | C++ CFD codes (FEAT3) |
| Body identity | indices into pe's storage, 64-bit ids as 8 `short`s | `std::uint64_t` system id |
| State | process-wide globals | the Engine object |
| Case setup | one `setupXxx`/`setupXxxSerial` per case | the CFD code builds the world through the API (or calls a setup, the Engine works on any world) |
| Relaxation, substeps | from `SimulationConfig` / hard-coded in the step functions | arguments of `applyLoads()` / `advance()` |
| Console output | per call in places | none |
| Point classification | `pointInsideParticles*()`, `pointOwners*()` | `owner()`, `owners()`, `classify()`, `contains()`, `containsPoints()` — same code (`src/interface/PointOwnership.h`) |

The extern "C" interface is unchanged. Both can be linked into one program; they share pe's
world, so mixing them on the same world is possible but the Engine's previous loads know only
about loads applied through `applyLoads()`.

## Deliberately not there (yet)

- Parallel coupling: load synchronization across processes, loads on shadow copies, migration
  of the load history, checkpoints of a decomposed world.
- Unresolved Euler-Lagrange coupling (hydro state, drag closures).
- Creation of shapes other than spheres and planes, body destruction, joints (use native pe).
- Case setup functions, configuration files, output writers (VTK, POV-Ray), logging.
- Lubrication switches (pe's runtime lubrication add-on is configured natively).
- Sleep-mode checkpoints.

## Tests

All in `tests/interface/`, each scenario in its own process, label `coupling`
(see [tests/interface/README.md](../../tests/interface/README.md)):

| CTest | What |
|---|---|
| `pe-coupling-header-standalone-*` | each header compiled alone by the bare compiler, and after `<mpi.h>` |
| `pe-coupling-header`, `pe-coupling-header-mpi-h` | a TU with only the header (and `<mpi.h>`) links, queries the build info and (serial) drives an Engine |
| `pe-coupling-shared-smoke` | the same program linked against `libpe.so` (configured with `PE_LIBRARY_TYPE=BOTH`) |
| `pe-coupling-detection` | the dispatch templates with stand-in types lacking every member; `unknown` vs declared false |
| `pe-coupling-build-info`, `-fluid`, `-plain` | build info and setting support under the three solvers, throwing setters, range checks, the effect of each contact parameter, one Engine per process |
| `pe-coupling-world` | creation, enumeration of native and API bodies of every shape, state, volume (and its cache), override, pe's quaternion tolerance, lookup after add/destroy, creation-time validation, distinct ids for global and local bodies, a native binary checkpoint in the old counter layout |
| `pe-coupling-classification` | analytic owners, spatial index vs exhaustive with an active grid, ownership rule, `containsPoints`, invalid points, after motion with and without refresh, legacy queries and flags agree |
| `pe-coupling-loads` | accumulation (overflow refused, earlier sum intact), relaxation arithmetic bit for bit, first-load rule for a body loaded late, untouched bodies, history of a temporarily fixed body (with and without another body, across a checkpoint), no-op and double application, no checkpoint between `applyLoads()` and `advance()` (also across a handle replacement; after a native reload until `acknowledgeWorldReload()`), native force/torque slots block checkpoints, clear, guards |
| `pe-coupling-fluid` | `HardContactAndFluid`: buoyant first step, settling on a plane, substeps, no stdout output while a trigger sees the caller's CFD rank |
| `pe-coupling-feat-equivalence` | bit-for-bit equality with the replicated FEAT3 wrapper |
| `pe-coupling-checkpoint-write`/`-restart`, `-pile-write`/`-pile-restart` | restart in a second process bit for bit (three spheres and a moving fixed sphere; a 64-sphere pile) |
| `pe-coupling-checkpoint-shake-write`/`-shake-restart` | characterisation: cell exchanges before multi-contact, restart only up to round-off |
| `pe-coupling-checkpoint-history-write`/`-history-restart` | characterisation: bodies created and destroyed after stepping began, grid inactive, restart only up to round-off |
| `pe-coupling-plane-geometry` | characterisation: the stored normal of a tilted plane at rest changes in its first step |
| `pe-coupling-checkpoint-errors` | error paths against a differing live state with a pending load (nothing may change), worlds out of scope (union, planes moving or rotating at 0.01 and at a sub-tolerance 1e-9), stream settings (hex, noskipws, pending width, fill, comma locale), build without checkpoint support |
| `pe-coupling-mpi`, `pe-coupling-mpi-single` | two ranks: synchronization before the first step (creation only, and after a handle replacement), relaxation across the cut, API planes with equal ids, shadow copies reported and classified, refusals, exclusive section; one rank: fully functional, a sphere resting on an API plane |

## Related notes

- [pe-interface-serial-mode.md](pe-interface-serial-mode.md): the extern "C" interface, serial
  mode and force synchronization by the CFD side.
- [euler-lagrange-solver-conventions.md](euler-lagrange-solver-conventions.md): the body-force
  semantics of the two fluid-coupled solvers.
- [hash-grids-algorithm.md](hash-grids-algorithm.md): the coarse detector behind
  `QueryMethod::spatialIndex`.
- [plane-conventions.md](plane-conventions.md), [rigid-body-documentation.md](rigid-body-documentation.md).
