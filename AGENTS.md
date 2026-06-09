# AGENTS.md

This file provides guidance to Agents.

## Build System

For normal builds, build variants, target-specific builds, CGAL/DistanceMap builds, examples, and Ninja usage, see `doc/technical-notes/build-with-cmake.md`.

Legacy note: the repository still contains the old custom `./configure` flow and `ConfigFile`, but new work should prefer CMake.

## Smoke Tests

The main lightweight smoke test is `tests/interface/pe_interface_smoke_serial.cpp`, exposed as the `pe_interface_smoke_serial` target and `pe-interface-serial-*` CTest cases. See `tests/interface/README.md` for harness details and `doc/technical-notes/build-with-cmake.md` for configure/build/run commands.

## Dependencies

### Required
- Boost >= 1.68 (thread, system, filesystem, program_options)
- CMake >= 3.15

### Optional
- MPI (for parallel execution)
- CGAL (for computational geometry)
- OpenCL (for GPGPU support) (maybe outdated, no active usage since 2022)
- Irrlicht >= 1.7.9 (for visualization) (deprecated, project is not actively maintained)

## Architecture Overview

PE is a physics engine for rigid body dynamics simulation with the following main components:

### Core Components
- `pe/core`: Main simulation engine components
  - RigidBody management and collisions
  - Contact detection and resolution (including DistanceMap acceleration)
  - Domain decomposition for MPI parallelization
- `pe/math`: Mathematical primitives and algorithms
  - Vectors and matrices
  - Linear solvers
  - Constraints handling
- `pe/util`: Utility functions and containers

### Geometry Primitives
- Box, Sphere, Capsule, Cylinder, Plane
- Triangle meshes and unions of primitives

### Parallelization
- MPI-based domain decomposition for distributed memory parallelism
- Thread-based parallelism for shared memory systems
- Optional GPGPU support via OpenCL

### Visualization Modules
- `pe/vtk`, `pe/opendx`: VTK is the main visualization mode now (actively maintained)
- `pe/povray`: POV-Ray scene export (old, but it still works)
- `pe/irrlicht`: Irrlicht-based real-time visualization (deprecated, not actively maintained)

### External Integration
- `pe/interface`: C/C++ interface for integration with other software (e.g. the FeatFloWer CFD solver).
- PE can be driven from a CFD solver in two distinct ways with separate interface functions: normal parallel/domain-decomposition mode, and `PE_SERIAL_MODE` (each CFD rank runs its own full serial PE instance, used mainly for debugging/bring-up). For the `PE_SERIAL_MODE` concept, the `commf2c_*` Fortran/C entry-point convention, the representative-rank I/O pattern, force synchronization, and how to add a new interface setup function, see `doc/technical-notes/pe-interface-serial-mode.md`.

## Repo Structure

This repo is organized like a C++ physics engine with public API headers, implementation files, examples, docs, and generated build trees.

### Top Level

- `pe/`: Public headers and most of the engine's API surface.
- `src/`: Implementation `.cpp` files corresponding to the modules under `pe/`.
- `examples/`: Runnable sample simulations and demos.
- `tests/`: Test code, currently small and mostly interface/math focused. The main smoke-test entry point is `tests/interface/pe_interface_smoke_serial.cpp`.
- `doc/`: Documentation, especially technical notes about collision detection, solvers, DistanceMap, lubrication, MPI, and related engine topics.
- `tutorial/`: Tutorial header files for guided examples.
- `cmake/`: Custom CMake helper modules and config templates.
- `media/`: Small bundled assets used by examples or visualization.
- Root build/config files include `CMakeLists.txt`, `CMakePresets.json`, `configure` (legacy, deprecated), and `ConfigFile` (legacy, deprecated).

### Core Source Layout

`pe/` is the main public include tree:

- `pe/core/`: Main simulation engine: bodies, collision systems, contacts, detection, domain decomposition, responses, lubrication, joints, notifications, and IO.
- `pe/core/rigidbody/`: Shape/body classes, their base classes, customization header. The base classes for shapes/rigid bodies RigidBody.h, RigidBodyBase.h contain most of the functions we can call from objects that inherit from them.
- `pe/math/`: Vectors, matrices, constraints, expressions, solvers, sparse math, and type traits.
- `pe/util/`: General utilities: logging, timing, threading, singleton/policy helpers, and traits.
- `pe/config/`: Compile-time configuration headers.
- `pe/interface/`: C/C++ integration interface.
-  `pe/vtk/`, `pe/irrlicht/`, `pe/povray/`, `pe/opendx/`: Visualization/export integrations.
- `pe/support/`, `pe/system/`: Support and system-level helpers.

`src/` mirrors much of that structure with compiled implementation files:

- `src/core/`: Runtime implementation for the physics core.
- `src/core/detection/fine/`: Fine collision detection implementation, including DistanceMap.
- `src/core/lubrication/`: Lubrication runtime implementation.
- `src/math/`, `src/util/`, `src/interface/`: Implementations for math, utilities, and external interface.
- `src/irrlicht/`, `src/povray/`, `src/vtk/`, `src/opendx/`: Visualization/export implementations.

### Examples

`examples/` contains many standalone demos grouped by scenario:

- Basic rigid body demos: `boxstack`, `cradle`, `domino`, `well`, `chain`, `cylinder`, and similar folders.
- Geometry/collision demos: `detection`, `convex_hull`, `triangle_mesh`, `kdop_example`, and `trimeshdop_demo`.
- Lubrication demos: `basic_lubrication`, `lubrication_demo`, and `sphere_column_serial_hcl`.
- MPI/distributed examples: many `mpi*` folders such as `mpicube`, `mpigranular`, `mpiperiodic`, and `mpispheres`.
- CGAL/DistanceMap examples: `examples/cgal_examples/`, including mesh collision tests, mesh simulation, DistanceMap debug tools, and VTK output helpers.

### Docs

`doc/technical-notes/` is the best place for design-level explanations. Start with `doc/technical-notes/README.md` to choose the right note; the directory covers topics like simulation setup examples, DistanceMap, collision conventions, contact solvers, batch generation, domain decomposition, lubrication contacts, rigid bodies, and MPI synchronization.

Simulation setup note:
- For concrete serial and MPI setup patterns, see `doc/technical-notes/simulation-setup-examples.md`.
- In short, serial simulations usually get `WorldID` via `theWorld()`, create bodies/materials, then advance with `world->simulationStep(dt)` or `world->run(...)`. MPI simulations must initialize MPI, define the local domain, connect neighboring remote domains, create bodies on the owning process, synchronize, and then step/run.

## Development Notes

The PE library can be built as static (default) or shared library. The library name is "pe" and will be found in the `build/lib` directory.

When extending the codebase:
- Prefer adding to existing modules over creating new ones
- Follow the existing naming conventions and class hierarchies
- Use templates for performance-critical code
- Implement new geometries by extending the existing base classes

Specialized implementation guides:
- For creating a new `CollisionSystem` specialization, see `doc/technical-notes/new-collision-system-implementation-guide.md`.

DistanceMap note:
- DistanceMap is the preferred collision path for `TriangleMesh` use in PE, especially for non-convex or complex meshes. GJK/EPA remains available as a fallback for convex meshes or meshes without DistanceMap acceleration.
- For implementation details, usage, and tuning, see `doc/technical-notes/distance-map-implementation.md`.
