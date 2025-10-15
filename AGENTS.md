# AGENTS.md

This file provides guidance to OpenAI Agents (e.g., ChatGPT in the Codex CLI) when working with code in this repository.

## Build System

The PE (Physics Engine) has two build systems:

1. Custom configure script (see ConfigFile for options)
   ```bash
   ./configure config-file.txt
   make
   ```

2. CMake-based build (recommended)
   ```bash
   mkdir build
   cd build
   cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC ..
   make
   ```

### Common CMake Options

- `-DCMAKE_BUILD_TYPE=Release|Debug`: Set build type (default: Release)
- `-DLIBRARY_TYPE=STATIC|SHARED|BOTH`: Build type of library (default: STATIC)
- `-DBLAS=ON|OFF`: Build support for BLAS (default: OFF)
- `-DMPI=ON|OFF`: Build support for MPI parallelization (default: OFF)
- `-DOPENCL=ON|OFF`: Build support for OpenCL (default: OFF)
- `-DIRRLICHT=ON|OFF`: Build support for Irrlicht visualization (default: OFF)
- `-DEXAMPLES=ON|OFF`: Build examples (default: OFF)
- `-DEIGEN=ON|OFF`: Build with Eigen library support (default: ON)
- `-DCGAL=ON|OFF`: Build with CGAL library support (default: OFF)
- `-DUSE_JSON=ON|OFF`: Build with JSON support (default: ON)

### Running Examples

After building with `-DEXAMPLES=ON`, the example binaries are located in `build/examples/*/`:

```bash
# Run a single-process example
./build/examples/boxstack/boxstack

# Run an MPI example with multiple processes
mpirun -np 4 ./build/examples/mpicube/mpicube

# Run CGAL-enabled examples (requires -DCGAL=ON)
./build/examples/cgal_examples/mesh_simulation --mesh1 mesh1.obj --mesh2 mesh2.obj
./build/examples/cgal_examples/mesh_collision_test reference.obj output test_sphere.obj
```

## Dependencies

### Required
- Boost >= 1.46.1 (thread, system, filesystem, program_options)
- CMake >= 3.15

### Optional
- Irrlicht >= 1.7.9 (for visualization)
- MPI (for parallel execution)
- OpenCL (for GPGPU support)
- CGAL (for computational geometry)

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

### Visualization Modules
- `pe/irrlicht`: Irrlicht-based real-time visualization
- `pe/povray`: POV-Ray scene export
- `pe/vtk`, `pe/opendx`: Additional visualization formats

### Parallelization
- MPI-based domain decomposition for distributed memory parallelism
- Thread-based parallelism for shared memory systems
- Optional GPGPU support via OpenCL

### External Integration
- `pe/interface`: C/C++ interface for integration with other software

## Library Structure

The library is structured as a header-only interface with implementation in source files:

- `pe/*.h`: Main include headers
- `pe/*/`: Component-specific headers
- `src/*/`: Implementation files

Most simulations need to:
1. Create a World object
2. Add bodies with physical properties
3. Configure collision detection
4. Set up response/solver methods
5. Run the time stepping loop

## Development Notes

The PE library can be built as static (default) or shared library. The library name is "pe" and will be found in the `build/lib` directory.

When extending the codebase:
- Prefer adding to existing modules over creating new ones
- Follow the existing naming conventions and class hierarchies
- Use templates for performance-critical code
- Implement new geometries by extending the existing base classes

## Creating a New Specialized CollisionSystem

PE uses a template-based design to specialize the collision system for different physical models. To create a new specialized CollisionSystem:

1. Create a response class in `pe/core/response/YourSpecialization.h`:
   ```cpp
   template< typename C, typename U1=NullType, typename U2=NullType >
   class YourSpecialization : private NonCopyable
   {
   public:
      pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
      pe_CONSTRAINT_MUST_BE_SAME_TYPE( U2, NullType );
   };
   ```

2. Create a configuration file in `pe/core/configuration/YourSpecialization.h`:
   ```cpp
   template< template<typename> class CD, typename FD, template<typename> class BG, template<typename,typename,typename> class CR >
   struct YourSpecializationConfig { /*...configuration details...*/ };
   
   typedef YourSpecializationConfig< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::YourSpecialization >  YSConfig;
   ```

3. Create a specialization of the CollisionSystem in `pe/core/collisionsystem/YourSpecialization.h`:
   ```cpp
   template< template<typename> class CD, typename FD, template<typename> class BG, template< template<typename> class, typename, template<typename> class, template<typename,typename,typename> class > class C >
   class CollisionSystem< C<CD,FD,BG,response::YourSpecialization> >
   { /*...implementation details...*/ };
   ```

4. Include the new specialization in `pe/core/CollisionSystem.h`:
   ```cpp
   #include <pe/core/collisionsystem/YourSpecialization.h>
   ```

5. To use the new specialization, modify `pe/config/Collisions.h`:
   ```cpp
   #define pe_CONSTRAINT_SOLVER  pe::response::YourSpecialization
   ```

6. Create a specialization of the RigidBodyTrait in `pe/core/rigidbody/rigidbodytrait/HardContactFluidLubrication.h`:
   ```cpp
   class RigidBodyTrait< C<CD,FD,BG,response::HardContactFluidLubrication> > : public MPIRigidBodyTrait
   { /*...implementation details...*/ };
   ```

## DistanceMap Integration

DistanceMap acceleration has been integrated into the PE core library as a fine collision detection algorithm alongside GJK and EPA. This provides 3D signed distance field-based collision detection for triangle meshes.

### Usage

```cpp
#include <pe/core/detection/fine/DistanceMap.h>

// Create triangle mesh with DistanceMap acceleration
TriangleMeshID mesh = createTriangleMesh(id, position, "mesh.obj", material, false, true);
mesh->enableDistanceMapAcceleration(resolution, tolerance);  // resolution=50, tolerance=5 (defaults)

// DistanceMap will be automatically used in collision detection
```

### File Structure

- **Header**: `pe/core/detection/fine/DistanceMap.h` - Co-located with GJK/EPA algorithms
- **Implementation**: `src/core/detection/fine/DistanceMap.cpp` - Included in PE library
- **Integration**: `pe/core/detection/fine/MaxContacts.h` - Collision detection pipeline

### CGAL Examples

The `examples/cgal_examples/` directory contains examples demonstrating CGAL and DistanceMap functionality:

- **`mesh_simulation.cpp`**: Complete physics simulation with two meshes, ground plane, and DistanceMap acceleration
- **`mesh_collision_test.cpp`**: Collision detection testing with DistanceMap validation
- **`cgal_box.cpp`**: Basic CGAL integration example

### Coordinate Transformations

DistanceMap collision detection properly handles coordinate transformations:
- Query vertices transformed from world coordinates to reference mesh local coordinates
- Distance queries performed in local coordinate system where DistanceMap is defined
- Results (normals, contact points) transformed back to world coordinates for collision response

### Current State

- ✅ DistanceMap fully integrated into PE core library
- ✅ Coordinate transformations implemented for proper mesh-to-mesh collision
- ✅ Sampling disabled for comprehensive vertex testing
- ✅ Examples updated with proper simulation loops and physics integration
- ✅ Build system configured for CGAL examples
