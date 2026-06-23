# Technical Notes Index

This directory contains focused engineering notes for PE internals. Use this index to find the right deep-dive quickly; some analysis and roadmap notes may describe investigations or proposed work rather than canonical current behavior.

## Build and Configuration

- [adding-json-parameters.md](adding-json-parameters.md): How to add a simulation parameter to `SimulationConfig` and load it from JSON at startup.
- [build-with-cmake.md](build-with-cmake.md): General CMake build setup, Unix Makefiles workflow, Ninja workflow, and target-specific build commands.
- [command-line-interface.md](command-line-interface.md): Shared command-line helper API in `pe::CommandLineInterface`.

## Architecture and Core Types

- [technical-overview.md](technical-overview.md): Broad architecture guide for PE's simulation workflow and main implementation areas.
- [simulation-setup-examples.md](simulation-setup-examples.md): Concrete serial and MPI simulation setup patterns, with references to repository examples.
- [frozen-field-cpp-interface.md](frozen-field-cpp-interface.md): Collective callback-driven tracer interface, global TriangleMesh/DistanceMap setup, and exit attribution.
- [rigid-body-documentation.md](rigid-body-documentation.md): `RigidBody` base class, common rigid-body API, and inheritance model.
- [triangle-mesh.md](triangle-mesh.md): `TriangleMesh` hierarchy, related types, implementation details, and ecosystem notes.
- [vec3-mat3-documentation.md](vec3-mat3-documentation.md): Usage and conventions for `Vec3` and `Mat3`.
- [plane-conventions.md](plane-conventions.md): PE's plane representation, non-standard conventions, and common pitfalls.

## Collision Detection

- [collision-detection-conventions.md](collision-detection-conventions.md): Contact normals, relative velocity, penetration depth, and other collision convention details.
- [hash-grids-algorithm.md](hash-grids-algorithm.md): Hierarchical hash grid coarse collision detection.
- [max-contacts.md](max-contacts.md): `MaxContacts` fine collision/contact generation responsibilities.
- [distance-map-implementation.md](distance-map-implementation.md): Current DistanceMap implementation for `TriangleMesh` collision, including preferred usage, API, contact generation, and tuning.
- [distance-map-roadmap.md](distance-map-roadmap.md): Planned DistanceMap improvements and future work.

## Collision Response and Solvers

- [contact-solvers-overview.md](contact-solvers-overview.md): Overview of compile-time selected response solver families and tradeoffs.
- [hard-contact-response-walkthrough.md](hard-contact-response-walkthrough.md): Sequential-impulse hard-contact response walkthrough.
- [ffd-solver.md](ffd-solver.md): Fast-Frictional Dynamics solver wiring and solver math notes.
- [relaxation-models.md](relaxation-models.md): Analysis of hard-contact relaxation models and material parameter effects.
- [new-collision-system-implementation-guide.md](new-collision-system-implementation-guide.md): Checklist for adding and wiring a new `CollisionSystem` specialization.

## Lubrication and Short-Range Forces

- [lubrication-contacts.md](lubrication-contacts.md): Lubrication contact design and integration, centered on `HardContactLubricated`.
- [lubrication-redesign.md](lubrication-redesign.md): Migration notes from deprecated lubrication solver stacks to `HardContactLubricated`.
- [short-range-rep-forces.md](short-range-rep-forces.md): Robust implementation plan for short-range repulsive forces in narrow-gap scenarios.
- [srr-implementation-lessons.md](srr-implementation-lessons.md): Lessons learned from implementing the short-range repulsion solver.

## Integration and Coupling

- [pe-interface-serial-mode.md](pe-interface-serial-mode.md): `PE_SERIAL_MODE` vs normal parallel mode, the `commf2c_*` Fortran/C entry-point convention, the representative-rank pattern, CFD force synchronization, and how to write a new interface setup function.

## Domain Decomposition and MPI

- [domain-decomposition-geometries-and-definition.md](domain-decomposition-geometries-and-definition.md): Process/domain geometries, local/remote domain definitions, and CFD coupling implications.
- [domain-geometry.md](domain-geometry.md): Analysis of surface-triangulated domain boundaries and their impact on PE.
- [mpi-data-communication-analysis.md](mpi-data-communication-analysis.md): Marshalling/unmarshalling architecture for MPI rigid-body data.
- [synchronize-analysis.md](synchronize-analysis.md): Focused analysis of `CollisionSystem::synchronize()`.
- [synchronize-general-analysis.md](synchronize-general-analysis.md): Broader synchronization architecture and domain decomposition patterns.

## Inventories

- [historical-analysis-inventory.md](historical-analysis-inventory.md): Tracks technical notes that may still be useful but need review before treating them as current-state documentation.
