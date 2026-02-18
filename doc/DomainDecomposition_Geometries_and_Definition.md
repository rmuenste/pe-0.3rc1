# Domain Decomposition Geometries and Domain Definition in PE

## Purpose

This document captures, in implementation detail, which process/domain geometries are available in PE, how local and remote domains are defined, and what this implies for coupling PE with external domain-decomposed CFD solvers.

The focus is on:

1. Concrete geometry capabilities in code.
2. Exact APIs used to define domains and process connections.
3. Practical constraints when trying to match non-PE partitioning schemes.


## Core Model

PE represents process domains through the `ProcessGeometry` interface:

- File: `pe/core/domaindecomp/ProcessGeometry.h`
- Class: `ProcessGeometry`
- Core virtual API:
  - `intersectsWith(...)` for supported body types.
  - `containsPoint(...)`
  - `containsPointStrictly(...)`
  - `extractHalfSpaces(...)`

Both local domain and each connected remote process store a `ProcessGeometry` object:

- Local: `Domain::geometry_` (`pe/core/domaindecomp/Domain.h`)
- Remote: `Process::geometry_` (`pe/core/domaindecomp/Process.h`)


## How Domains Are Defined

### Local domain

Local domain geometry is set via:

- `defineLocalDomain(const Geometry&)`
- File: `pe/core/domaindecomp/DomainDecomposition.h`

This directly replaces the local `Domain` geometry object in the collision system.

### Remote process domains

Remote domains are created via `connect(...)` and stored as `Process` entries:

- File: `pe/core/domaindecomp/DomainDecomposition.h`
- Backend: `connect_backend(...)` in `src/core/domaindecomp/DomainDecomposition.cpp`

Available `connect` forms:

1. `connect(rank, a, b, c, d, offset)` (plane coefficients)
2. `connect(rank, normal, d, offset)`
3. `connect(rank, a, b, c, x, y, z, offset)` (plane through point)
4. `connect(rank, normal, gpos, offset)`
5. `connect(rank, geometry, offset)` (generic `ProcessGeometry`)

`connect_backend(...)` validates:

- MPI-enabled configuration.
- Rank in range and not self.
- Not already connected.


## Available Process Geometries

### 1) HalfSpace (primary production geometry)

- Header: `pe/core/domaindecomp/HalfSpace.h`
- Source: `src/core/domaindecomp/HalfSpace.cpp`

Defines a half-space by plane `n*x = d` plus optional overlap width `dx`:

- Constructors for `(a,b,c,d)`, `(normal,d)`, `(a,b,c,point)`, `(normal,point)`.
- Normal is normalized.
- `dx` is clamped to at least `contactThreshold`.

Implements:

- Body intersection logic for sphere/box/capsule/cylinder/ellipsoid/triangle mesh/union.
- Point containment:
  - `containsPoint`: boundary inclusive (`>= 0` test).
  - `containsPointStrictly`: boundary exclusive (`> 0` test).
- `extractHalfSpaces`: returns its own `(normal, d)` pair.

This is the most complete and stable geometry implementation for decomposition.

### 2) Intersection geometry (`intersect(...)`)

- File: `pe/core/domaindecomp/Intersection.h`

Template composition of 2..5 process geometries into an intersection:

- `containsPoint`: logical AND across component geometries.
- `containsPointStrictly`: logical AND.
- `intersectsWith`: logical AND behavior.
- `extractHalfSpaces`: concatenates half-space extractions from all components.

Used heavily for rectangular/polyhedral subdomains built from multiple `HalfSpace`s.

### 3) Merging geometry (`merge(...)`)

- File: `pe/core/domaindecomp/Merging.h`

Template union of 2..5 process geometries:

- `containsPoint`: logical OR.
- `containsPointStrictly`: logical OR.
- `intersectsWith`: logical OR behavior.
- `extractHalfSpaces`: concatenates child half-space descriptions.

Useful for non-convex unions, but less common in regular decomposition workflows.

### 4) TriMeshBoundary (present, not production-complete)

- Header: `pe/core/domaindecomp/TriMeshBoundary.h`
- Source: `src/core/domaindecomp/TriMeshBoundary.cpp`

Current implementation status:

- `containsPoint(...)` implemented via ray casting (with AABB precheck).
- `containsPointStrictly(...)` just calls `containsPoint(...)`.
- `extractHalfSpaces(...)` is empty.
- Several `intersectsWith(...)` methods return placeholder `true`.
- `print(...)` functions are empty.

Conclusion: usable for experimentation, not robust enough as a primary production process geometry for strict coupling constraints.

### 5) TriMeshDopBoundary (newer, more advanced, still partially conservative)

- Header: `pe/core/domaindecomp/TriMeshDopBoundary.h`
- Source: `src/core/domaindecomp/TriMeshDopBoundary.cpp`

Implements:

- OBJ loading.
- AABB computation.
- PCA/OBB-based 6-DOP extraction.
- `extractHalfSpaces(...)` from k-DOP planes.
- `containsPoint(...)` with AABB + DOP + ray casting.

Current caveats:

- Some `intersectsWith(...)` methods are still conservative/simplified (for example generic body returns `true`; others rely only on broad tests).
- Designed around 6-DOP currently (`k=6` path).
- Not included by default in `pe/core.h` (only `TriMeshBoundary` is included there).

Conclusion: stronger than `TriMeshBoundary`, but still not equivalent in maturity to the half-space based path for strict domain-exchange correctness.


## RectilinearGrid Decomposition Support

The built-in structured decomposition helper is:

- Header: `pe/core/domaindecomp/RectilinearGrid.h`
- Source: `src/core/domaindecomp/RectilinearGrid.cpp`

Key capabilities:

1. Nonuniform spacing:
   - `connect(origin, dxVec, dyVec, dzVec, bcLower, bcUpper)`
2. Uniform convenience form:
   - `connect(origin, lengths, numCells, bcLower, bcUpper)`
3. Boundary conditions per side:
   - `boundaryConditionOutflow`
   - `boundaryConditionOpen`
   - `boundaryConditionPeriodic`

Important behavior:

- Requires `px * py * pz == MPI world size`.
- All cell widths must be positive.
- Open boundaries extend the respective global side to `-inf` or `+inf`.
- Local domain is defined as exact intersection of 6 half-spaces.
- Remote neighbors are auto-connected for all 26 relative offsets (`dx,dy,dz in [-1,0,1] \ {0,0,0}`).
- Periodic edges/corners handled via `offset` vectors in `connect(...)`.


## Interface-Level Decomposition Functions

`pe/interface/decompose.h` provides high-level setup functions used in examples and setups:

1. `decomposeDomain(...)` (non-periodic)
2. `decomposePeriodic3D(...)` (periodic XYZ)
3. `decomposePeriodicXY3D(...)` (periodic X/Y only)
4. `decomposePeriodicX3D(...)` (periodic X only)
5. `decomposeDomain2DArchimedes(...)` (custom 2D using supplied half-space arrays)

Common pattern in these helpers:

- Define local domain with intersections of half-spaces.
- Explicitly connect every relevant neighbor (faces, edges, corners) with appropriate geometry.
- Apply periodicity through non-zero `offset` vectors.

These APIs are flexible in decomposition mode but still fundamentally half-space based.


## Domain Ownership Semantics (Important for Coupling)

Local ownership logic lives in `Domain`:

- `containsPoint(...)`: boundary inclusive.
- `containsPointStrictly(...)`: boundary exclusive.
- `ownsPoint(...)`:
  - strict inside => own.
  - if on boundary, lowest-rank process owning that point wins.

This rank tie-break rule is critical when matching PE with CFD ownership rules on partition interfaces.


## MPI Process-Setup Validation

`MPISystem::checkProcesses()` validates setup consistency:

- Symmetry of process connectivity graph.
- Compares half-space descriptions exchanged between neighbors:
  - local description of remote domain vs remote process exact domain description.

Important detail:

- The validation mechanism depends on `extractHalfSpaces(...)`.
- For geometry types with weak/no half-space extraction semantics, validation can be incomplete or less meaningful.


## Practical Constraints for PE + Decomposed CFD Coupling

Given current implementation state:

1. Most robust path:
   - HalfSpace + Intersection (and optionally Merging), with explicit `defineLocalDomain` and `connect`.
2. Structured but flexible:
   - `RectilinearGrid` supports nonuniform rectilinear partitions and mixed BCs.
3. General arbitrary partitions:
   - feasible only if represented as half-space systems (polyhedral approximation or exact polyhedra).
4. Mesh boundary geometries:
   - currently not at parity with half-space path for production-critical communication correctness.

This explains why PE process geometry can feel constrained when CFD decomposition is more general (unstructured/polyhedral/graph partitions): PE’s stable decomposition machinery is still dominated by half-space representations and Cartesian-like neighbor setup logic.


## Recommended Usage Pattern Today

For robust coupling with external CFD decompositions:

1. Convert each CFD partition cell to a convex half-space set (or union of small convex sets).
2. Build PE local domain via `defineLocalDomain(intersect(...))`.
3. Build explicit remote connections with `connect(rank, geometry, offset)`.
4. Ensure all possible migration neighbors are connected (faces, edges, corners, and farther if required by large bodies).
5. Run consistency checks (`checkProcesses()`) during setup/debug builds.

For periodic CFD boundaries:

- encode periodic wraparound via `offset` in `connect(...)`, matching CFD periodic transforms exactly.
