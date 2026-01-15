# GEMINI.md

This file provides guidance to Gemini (the AI agent) when working with code in this repository. It consolidates architectural knowledge, build instructions, and recent critical updates.

## Recent Architectural Updates & Fixes (Nov 2025 - Jan 2026)

### 1. Kinematic Body & Motor Handling (Critical)
A major fix was implemented for collisions involving "Kinematic Bodies" (bodies with fixed translation but prescribed rotational motion, like motors/rotors).
- **Problem:** The solver previously attempted to enforce static contact (`v_rel = 0`), which is mathematically impossible against a body with prescribed motion, leading to energy violation (excessive spin-up of the colliding body).
- **Solution:**
    - **Kinematic Detection:** The solver now detects bodies with `invMass=0` (translation-fixed) but `|angular_velocity| > 0`.
    - **Dynamic Friction Forcing:** For these contacts, the solver **skips the static friction attempt** and forces the **dynamic friction (sliding)** path.
    - **Baumgarte Disabled:** Baumgarte stabilization is now **disabled** for contacts where both bodies have zero translational DOF (`invMass=0`), as they cannot separate to resolve penetration.
- **Reference:** `@doc/collision_response_analysis-final.md`

### 2. Lubrication Model Changes
- **Canonical Implementation:** `HardContactLubricated` is the new standard.
- **Deprecation:** `HardContactAndFluidWithLubrication` and `HardContactFluidLubrication` are deprecated.
- **Runtime Controls:** `pe/core/lubrication/Params.*` exposes global controls for contact/lubrication blending, queried directly by `MaxContacts`.

### 3. DistanceMap Integration
- **Status:** Fully integrated into PE core as a fine collision detection algorithm.
- **Priority:** If a mesh has a DistanceMap, the system uses it and **does NOT fallback to GJK/EPA** (as these meshes are typically non-convex).
- **Multi-Contact:** Implements multi-contact manifold generation with clustering and representative selection (up to 6 contacts per pair).

## Build System

The PE (Physics Engine) has two build systems:

1. **CMake-based build (Recommended)**
   ```bash
   mkdir build && cd build
   cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC ..
   make
   ```

2. **Custom configure script**
   ```bash
   ./configure config-file.txt && make
   ```

### Common CMake Options
- `-DCMAKE_BUILD_TYPE=Release|Debug`: (default: Release)
- `-DLIBRARY_TYPE=STATIC|SHARED|BOTH`: (default: STATIC)
- `-DMPI=ON|OFF`: Build MPI support (default: OFF)
- `-DCGAL=ON|OFF`: Build CGAL support (required for DistanceMap examples) (default: OFF)
- `-DEXAMPLES=ON|OFF`: Build examples (default: OFF)

### Running Examples
Binaries are in `build/examples/*/`.
```bash
# Standard
./build/examples/boxstack/boxstack

# MPI
mpirun -np 4 ./build/examples/mpicube/mpicube

# CGAL/DistanceMap (requires -DCGAL=ON)
./build/examples/cgal_examples/mesh_simulation --mesh1 mesh1.obj --mesh2 mesh2.obj
```

## Architecture Overview

PE is a rigid body dynamics engine with a template-based trait system for configuration.

### Core Components
- `pe/core`: RigidBody management, collision detection/resolution.
- `pe/math`: Vectors, matrices, linear solvers.
- `pe/util`: Utilities.

### Rigid Body Class Hierarchy
The `RigidBody` class uses a mixin-style template inheritance to swap implementations at compile time:
```
RigidBody (Abstract Base)
└── inherits from: RigidBodyTrait<Config> (Config-dependent behavior)
    └── inherits from: RigidBodyBase (Common data: mass, pos, q, v, w)
```
- **Traits:** Located in `pe/core/rigidbody/rigidbodytrait/`.
- **Geometries:** Concrete types (`Sphere`, `Box`, `Capsule`, `TriangleMesh`) inherit from `RigidBody`.

### Collision Pipeline
1. **Broad Phase:** Hash grids or similar (configurable).
2. **Fine Phase:** `MaxContacts` class.
    - Checks for **DistanceMap** first.
    - Falls back to **GJK/EPA** for convex primitives if no DistanceMap is present.
3. **Resolution:** `HardContactAndFluid` (or similar solver).
    - Iterative Gauss-Seidel relaxation.
    - Handles Coulomb friction and lubrication forces.

## DistanceMap Implementation Details

DistanceMap provides O(1) signed distance queries for mesh-to-mesh collisions.

- **Header:** `pe/core/detection/fine/DistanceMap.h`
- **Coordinate Systems:**
    - DistanceMaps are stored in the **Local Frame** of the mesh.
    - Queries transform World Point -> Local Point -> Query SDF -> Transform Result -> World Frame.
    - **Critical:** Correct usage of `pointFromWFtoBF` and `vectorFromBFtoWF`.
- **Sampling:** Currently defaults to **all vertices** (sampling disabled) for maximum accuracy on small meshes.
- **Clustering:** Contacts are clustered by proximity (`2 * spacing`) and normal similarity (`dot > 0.9`) to generate stable manifolds.

## Extending the Codebase

### Adding New Geometries
1. **Update Enum:** Add type to `pe/core/GeomType.h`.
2. **Create Base:** `YourGeomBase` (properties).
3. **Create Trait:** `YourGeomTrait` (geometric algos like `support()`).
4. **Create Class:** `YourGeom` (inherits Trait, implements `RigidBody` virtuals).
5. **Register:** Add creation functions and integrate into collision dispatch.

### Creating a Collision System Specialization
1. Create response class in `pe/core/response/`.
2. Create config struct in `pe/core/configuration/`.
3. Specialize `CollisionSystem` template in `pe/core/collisionsystem/`.
4. Update `pe/config/Collisions.h` to use your new solver.

## Development Rules

- **Prefer existing modules:** Don't create new top-level folders without reason.
- **Follow naming:** Mimic existing class hierarchies.
- **Templates:** Use templates for performance-critical inner loops.
- **Git:** Check `git status` before committing.
