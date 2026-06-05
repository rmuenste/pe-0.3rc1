# Simulation Setup Examples

This note expands the short simulation workflow from `AGENTS.md` with concrete examples from the repository. Use it when adding a new example, checking how a simulation is wired, or deciding where serial and MPI setup code belongs.

## Serial Simulation Pattern

`examples/simple_demo/simple_demo.cpp` is the compact reference for a serial setup:

1. Configure run parameters and optional command-line input near `examples/simple_demo/simple_demo.cpp:84`.
2. Get the simulation world with `WorldID world = theWorld()` and set global properties such as gravity near `examples/simple_demo/simple_demo.cpp:113`.
3. Add rigid bodies with material and physical properties. The demo creates planes, a sphere, and initial velocity near `examples/simple_demo/simple_demo.cpp:117`.
4. Optional visualization is enabled before the loop near `examples/simple_demo/simple_demo.cpp:126`.
5. Advance the simulation by calling `world->simulationStep(dt)` inside the time loop near `examples/simple_demo/simple_demo.cpp:135`.

Most serial examples follow the same shape, even when the bodies or visualization stack differ. `examples/boxstack/BoxStack.cpp` is another useful reference for a larger rigid-body scene with multiple object types.

Collision detection and response are usually not configured inside the example body. The normal selection point is compile-time configuration in `pe/config/Collisions.h`, which defines the coarse detector, fine detector, batch generator, and response solver macros. For solver-specific context, see `contact-solvers-overview.md`; for contact-generation conventions, see `collision-detection-conventions.md`.

## MPI Simulation Pattern

`examples/mpispheres/mpispheres.cpp` is the clearest reference for a parallel simulation that partitions a 3D domain:

1. Initialize MPI before using PE's MPI system. The example calls `MPI_Init` at `examples/mpispheres/mpispheres.cpp:85` and finalizes with `MPI_Finalize` at `examples/mpispheres/mpispheres.cpp:665`.
2. Create materials and get both the world and MPI system with `WorldID world = theWorld()` and `MPISystemID mpisystem = theMPISystem()` near `examples/mpispheres/mpispheres.cpp:186`.
3. Build the process topology. The example creates a Cartesian communicator with `MPI_Cart_create`, installs it into `mpisystem`, and reads each rank's coordinates near `examples/mpispheres/mpispheres.cpp:204`.
4. Define the local process domain with `defineLocalDomain(intersect(...))` near `examples/mpispheres/mpispheres.cpp:243`.
5. Register neighboring remote domains with `connect(rank, intersect(...))`. The face-neighbor connections begin near `examples/mpispheres/mpispheres.cpp:255`, and the file continues with edge and corner neighbors.
6. Validate the process/domain setup in debug builds with `mpisystem->checkProcesses()` near `examples/mpispheres/mpispheres.cpp:504`.
7. Create global boundary objects in a `pe_GLOBAL_SECTION` near `examples/mpispheres/mpispheres.cpp:515`.
8. Create local particles only on the owning process. The strong-scaling branch checks `world->ownsPoint(gpos)` before `createSphere` or `createGranularParticle` near `examples/mpispheres/mpispheres.cpp:555`; the weak-scaling branch verifies ownership before creating local particles near `examples/mpispheres/mpispheres.cpp:581`.
9. Call `world->synchronize()` after setup near `examples/mpispheres/mpispheres.cpp:599`, then run the simulation with `world->run(timesteps, stepsize)` near `examples/mpispheres/mpispheres.cpp:643`.

`examples/mpicube/MPICube.cpp` shows a different decomposition style: it splits the x-y plane into angular "cake slices" and connects only the two nearest neighboring process domains near `examples/mpicube/MPICube.cpp:319`. Use it as a reference when the process partition is geometric but not a regular Cartesian grid.

## Where to Look Next

- `pe/core/World.h`: world access, stepping, synchronization, and body ownership queries.
- `pe/core/MPISystem.h`: MPI communicator and rank/size access.
- `pe/core/domaindecomp/DomainDecomposition.h`: domain definition and neighbor connection helpers.
- `pe/config/Collisions.h`: compile-time collision and response algorithm selection.
- `domain-decomposition-geometries-and-definition.md`: deeper domain geometry background.
- `synchronize-general-analysis.md`: MPI synchronization architecture.
