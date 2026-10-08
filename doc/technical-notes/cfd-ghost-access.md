# Internal CFD ghost access

`World::getShadowBody()` has been removed. It assumed that every collision
system kept ghosts in a separate container and exposed unchecked storage
indexes through the public world API. This prevented `World.h` from compiling
with collision systems using other storage layouts.

The CFD interface now resolves ghosts through the internal adapter in
`src/interface/coupling_body_access.h`. This is implementation infrastructure,
not a replacement public World API. The initial backend uses collision systems
with mutable separate shadow storage, including HardContactAndFluid and
HardContactEulerLagrange. Shared-storage DEMSolverObsolete/FFDSolver support is
deferred; this change does not establish full CFD support for those solvers.

The adapter provides checked indexed access, system-ID lookup (`nullptr` when
absent), traversal in existing storage order, and the legacy remote-particle
index filter. All-ghost indexes and filtered particle indexes remain distinct.
The filter includes spheres, capsules, ellipsoids, non-fixed cylinders and
triangle meshes, preserving the existing interface convention. Other remote
queries with different geometry predicates retain those predicates.

Indexes and handles are valid only until synchronization, removal or other
storage mutation. Traversal callbacks may modify body state, but may not change
storage membership or synchronize. No container or storage iterator is exposed.

`setRemoteObjByIdx()` still updates ghost kinematics directly. Its ownership
semantics are intentionally deferred. Force synchronization, owned-body force
application, ghost force application and final body synchronization retain their
existing order. The adapter does not communicate or route updates to owners.

External callers of the removed World function must migrate their ghost access
to an appropriate integration layer; ordinary simulation code should use the
existing world body API. The Fortran/C coupling entry points and index ordering
are preserved. Internal indexed lookups now consistently throw
`std::out_of_range` instead of relying on storage assertions.

`pe-remote-query` covers separate-storage traversal, lookup, bounds, filtering,
legacy mapping output and the unchanged kinematic setter behavior without MPI.
It does not validate communication between ranks.
