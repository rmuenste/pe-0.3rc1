# Frozen-Field C++ Interface

`pe/interface/frozen_field_trace.h` exposes a collective C++ entry point for passive particle
tracing:

```cpp
pe::FrozenFieldTraceResult pe::runFrozenFieldTrace(
    const std::vector<std::array<double, 3>>& localSeeds,
    const pe::SurfaceMeshInput& surface,
    const pe::VelocityCallback& callback);
```

## Requirements

- Build PE with `MPI=ON`, `CGAL=ON`, and `USE_JSON=ON`.
- The caller initializes MPI before entering the function.
- The operation uses all ranks in `MPI_COMM_WORLD`.
- Every rank calls the function once and supplies the same surface mesh.
- `example.json` must be available in the process working directory.
- `processesX_ * processesY_ * processesZ_` must equal the communicator size.

The unit cube is decomposed uniformly using the configured process grid. Each seed is supplied by
the rank that owns it. `benchRadius_`, `stepsize_`, `substeps_`, and `timesteps_` control the tracer
spheres and loop.

## Surface Input

`SurfaceMeshInput` accepts complete OFF data held in a string or a path to an OBJ file readable on
every rank. Polygon faces are triangulated as a fan. The resulting surface must be closed,
consistently oriented with outward counter-clockwise winding, and have bounding box `[0,1]^3`.
Each rank hashes the canonical vertices and triangle indices to verify equivalent input.

PE creates the surface as one fixed global `TriangleMesh`, enables its `DistanceMap` using the
`domainBoundary_.distanceMap` resolution and tolerance settings, and inverts the map for domain
boundary queries. The boundary and tracer spheres remain in the PE world but have collision
generation disabled.

## Callback

The callback receives `3 * numVertices` contiguous position doubles and a same-sized writable
velocity array. It is called before the first step and after every main step, including when a rank
owns zero tracers. A nonzero return code, exception, or non-finite velocity aborts the collective
operation.

The callback velocity is applied directly. Gravity, contact response, tracer-tracer collisions,
and boundary collisions do not affect passive advection.

## Exit Results

Before each main step, PE predicts each center position using its current frozen velocity. If the
center crosses the supplied surface, the earliest segment-triangle intersection is recorded and the
tracer is removed before stepping. `TracerExit` reports the stable origin identity, exit rank,
one-based main timestep, interpolated time, canonical triangle index, cube face, intersection
position, and outgoing velocity.

Particles still present after `timesteps_` are returned as rank-local `TracerSurvivor` records.
Results are not gathered to rank zero.
