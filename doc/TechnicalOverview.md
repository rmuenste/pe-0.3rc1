# PE Physics Engine - Technical Overview

## Introduction

This technical overview provides a comprehensive guide to the PE (Physics Engine) architecture for developers familiar with rigid body physics simulation principles. It outlines the simulation workflow, identifies key components, and provides direct references to implementation files to help new users navigate the codebase effectively.

## Architecture Philosophy

PE employs a **template-based configuration system** that allows compile-time specialization of algorithms for different simulation scenarios. The core architecture is built around four fundamental components that can be mixed and matched:

1. **Coarse Collision Detection** - Broad-phase collision culling
2. **Fine Collision Detection** - Precise contact generation  
3. **Batch Generation** - Contact grouping for parallelization
4. **Constraint Solving** - Physics response calculation

## Configuration System

### Core Configuration Template

The entire physics engine is configured through a single template structure:

```cpp
// File: pe/core/Configuration.h:87-91
typedef Configuration< pe_COARSE_COLLISION_DETECTOR  // Broad-phase algorithm
                     , pe_FINE_COLLISION_DETECTOR    // Contact generation
                     , pe_BATCH_GENERATOR            // Parallelization strategy  
                     , pe_CONSTRAINT_SOLVER          // Physics solver
                     >::Config  Config;
```

### Algorithm Selection

Algorithms are selected via macros in `pe/config/Collisions.h`:

```cpp
// File: pe/config/Collisions.h:35
#define pe_COARSE_COLLISION_DETECTOR  pe::detection::coarse::HashGrids

// File: pe/config/Collisions.h:48  
#define pe_FINE_COLLISION_DETECTOR    pe::detection::fine::MaxContacts

// File: pe/config/Collisions.h:62
#define pe_BATCH_GENERATOR            pe::batches::UnionFind

// File: pe/config/Collisions.h:85
#define pe_CONSTRAINT_SOLVER          pe::response::HardContactAndFluid
```

## Simulation Workflow

### Main Simulation Loop

The simulation follows a standard rigid body physics pipeline with PE-specific optimizations:

```
1. Initialize World & Bodies
2. FOR each timestep:
   a. Apply External Forces
   b. Coarse Collision Detection  
   c. Fine Collision Detection
   d. Batch Generation
   e. Constraint Solving
   f. Time Integration
   g. Update Body States
3. Cleanup & Output
```

### Detailed Workflow Stages

#### Stage 1: World Management

**Purpose**: Central container managing all simulation entities

**Key Files**:
- `pe/core/World.h:61-100` - Main World class definition
- `pe/core/BodyManager.h` - Body lifecycle management
- `pe/core/rigidbody/BodyStorage.h` - Efficient body container

**Core Responsibilities**:
- Body creation and destruction
- Global simulation parameters (gravity, timestep)
- Domain decomposition coordination (MPI)
- Visualization interface

**Code Entry Points**:
```cpp
// Access the global simulation world
WorldID world = theWorld();

// Create rigid bodies
SphereID sphere = createSphere(id, x, y, z, radius, material);
BoxID box = createBox(id, x, y, z, lx, ly, lz, material);
```

#### Stage 2: Coarse Collision Detection (Broad Phase)

**Purpose**: Efficiently identify potentially colliding body pairs

**Available Algorithms**:
- **HashGrids**: `pe/core/detection/coarse/HashGrids.h`
- **SweepAndPrune**: `pe/core/detection/coarse/SweepAndPrune.h`
- **ExhaustiveSearch**: `pe/core/detection/coarse/ExhaustiveSearch.h`

**Core Implementation**:
- **Interface**: `pe/core/detection/coarse/Detectors.h`
- **Configuration**: Selected via `pe_COARSE_COLLISION_DETECTOR` macro

**Algorithm Details**:
```cpp
// HashGrids: Most commonly used, O(n) average case
// - Spatial partitioning using uniform grid
// - Handles dynamic body distribution well
// - Configurable grid resolution

// SweepAndPrune: O(n log n), good for coherent motion  
// - Sorts bodies along primary axis
// - Exploits temporal coherence
// - Efficient for clustered scenarios

// ExhaustiveSearch: O(n²), baseline for small systems
// - Tests all body pairs
// - Simple and robust
// - Useful for debugging and small simulations
```

#### Stage 3: Fine Collision Detection (Narrow Phase)

**Purpose**: Generate precise contact information for potentially colliding pairs

**Primary Implementation**:
- **MaxContacts**: `pe/core/detection/fine/MaxContacts.h:84-145`
- **GJK Algorithm**: `pe/core/detection/fine/GJK.h`
- **EPA Algorithm**: `pe/core/detection/fine/EPA.h`

**Contact Generation Process**:
```cpp
// File: pe/core/detection/fine/MaxContacts.h:99
template<typename CC> 
static void collide(BodyID b1, BodyID b2, CC& contacts);

// Specialized functions for each geometry pair:
collideSphereSphere()    // Analytical solution
collideSphereBox()       // Analytical with edge cases  
collideBoxBox()          // Complex analytical/GJK hybrid
collideMeshMesh()        // GJK/EPA for complex shapes
```

**Contact Data Structure**:
- **Contact Class**: `pe/core/contact/Contact.h:68`
- **Properties**: Position, normal, penetration depth, edge vectors
- **Memory Management**: Pool-based allocation for performance

#### Stage 4: Batch Generation

**Purpose**: Group contacts into independent sets for parallel processing

**Available Algorithms**:
- **UnionFind**: `pe/core/batches/UnionFind.h:50-83`
- **SingleBatch**: `pe/core/batches/SingleBatch.h:48-77`

**Parallelization Strategy**:
```cpp
// File: pe/core/batches/UnionFind.h:107-165
// Creates independent contact groups using graph connectivity
// - Each batch contains contacts that don't influence each other
// - Enables thread-level parallelism within each MPI process
// - O(n) time complexity for contact distribution
```

**Data Structures**:
- **BatchVector**: `pe/core/batches/BatchVector.h:47-100`
- **Contact Containers**: Template-based for flexibility

#### Stage 5: Constraint Solving (Physics Response)

**Purpose**: Compute forces and impulses to resolve contacts and maintain constraints

**Available Solvers**:
- **HardContactAndFluid**: `pe/core/response/HardContactAndFluid.h`
- **DEMSolver**: `pe/core/response/DEMSolver.h`
- **FrictionlessSolver**: `pe/core/response/FrictionlessSolver.h`
- **BoxFrictionSolver**: `pe/core/response/BoxFrictionSolver.h`

**Collision System Integration**:
- **Specializations**: `pe/core/collisionsystem/HardContactAndFluid.h`
- **Configuration**: `pe/core/configuration/HardContactAndFluid.h`

**Solver Characteristics**:
```cpp
// HardContactAndFluid: Production solver for most applications
// - Handles both collision and fluid coupling
// - Supports friction models
// - Iterative constraint resolution

// DEMSolver: Discrete Element Method
// - Specialized for granular materials
// - Force-based contact resolution
// - Efficient for large particle systems

// FrictionlessSolver: Simplified contact model
// - No friction computation
// - Fast but less realistic
// - Useful for performance testing
```

#### Stage 6: Time Integration

**Purpose**: Update body positions and velocities based on computed forces

**Integration Methods**:
- **Explicit Integration**: Simple forward Euler
- **Semi-implicit**: Velocity-Verlet style
- **Implicit Methods**: For stiff systems

**Implementation Details**:
- **Body Updates**: `pe/core/rigidbody/RigidBody.h`
- **Velocity Integration**: Linear and angular momentum
- **Position Updates**: Translation and rotation matrices

## Key Data Structures

### Rigid Bodies

**Base Class**: `pe/core/rigidbody/RigidBody.h`

**Geometry Types**:
- **Sphere**: `pe/core/rigidbody/Sphere.h`
- **Box**: `pe/core/rigidbody/Box.h`  
- **Capsule**: `pe/core/rigidbody/Capsule.h`
- **Cylinder**: `pe/core/rigidbody/Cylinder.h`
- **Plane**: `pe/core/rigidbody/Plane.h`
- **TriangleMesh**: `pe/core/rigidbody/TriangleMesh.h`
- **Union**: `pe/core/rigidbody/Union.h`

**Core Properties**:
```cpp
// Physical properties
Vec3 position_, linearVel_, angularVel_;
Quat orientation_;
real mass_, invMass_;
Mat3 inertiaTensor_, invInertiaTensor_;

// Simulation state  
bool fixed_, finite_;
MaterialID material_;
BodyID superBody_;  // For hierarchical bodies
```

### Contacts

**Contact Representation**: `pe/core/contact/Contact.h:55-100`

**Key Information**:
```cpp
GeomID body1_, body2_;          // Contacting geometries
Vec3 gpos_;                     // Global contact position  
Vec3 normal_;                   // Contact normal (body1 → body2)
Vec3 e1_, e2_;                  // Contact edge vectors
real dist_;                     // Penetration depth
```

**Contact Management**:
- **ContactVector**: `pe/core/contact/ContactVector.h`
- **Memory Pooling**: Efficient allocation/deallocation
- **Batch Organization**: Grouped for parallel processing

### Materials

**Material System**: `pe/core/Materials.h`

**Properties**:
```cpp
real density_;           // Mass per unit volume
real restitution_;       // Coefficient of restitution  
real staticFriction_;    // Static friction coefficient
real dynamicFriction_;   // Dynamic friction coefficient
```

**Predefined Materials**:
```cpp
// Common materials with realistic properties
extern const Material iron;
extern const Material steel;  
extern const Material aluminum;
extern const Material granite;
extern const Material oak;
```

## Parallelization Architecture

### Multi-Level Parallelism

PE employs a hierarchical parallelization strategy:

1. **MPI Level**: Domain decomposition across processes
2. **Thread Level**: Batch-based parallelism within processes  
3. **SIMD Level**: Vectorized operations where applicable

### Domain Decomposition (MPI)

**Implementation**: `pe/core/domaindecomp/`

**Key Files**:
- **Domain**: `pe/core/domaindecomp/Domain.h`
- **Process**: `pe/core/domaindecomp/Process.h`
- **Communication**: `pe/core/MPISystem.h`

**Strategy**:
```cpp
// Spatial partitioning of simulation domain
// - Each process owns a rectangular subdomain
// - Bodies migrate between processes as they move
// - Ghost layer communication for boundary interactions
// - Load balancing through dynamic domain adjustment
```

### Thread-Level Parallelism

**Batch Processing**: Contact batches are processed in parallel threads

**Synchronization**:
- **Minimal**: Only at batch boundaries
- **Lock-free**: Independent batches require no synchronization
- **Load Balancing**: Depends on batch size distribution

## Advanced Features

### Visualization Integration

**Irrlicht Support**: `pe/irrlicht/`
- Real-time 3D visualization
- Interactive camera controls
- Body property display

**POV-Ray Export**: `pe/povray/`
- High-quality ray-traced rendering
- Animation sequence generation
- Customizable materials and lighting

**VTK Output**: `pe/vtk/`
- Scientific visualization
- Data analysis integration
- Paraview compatibility

### GPU Acceleration

**OpenCL Support**: `pe/core/OpenCLColoredContacts.h`
- GPU-accelerated contact detection
- Parallel constraint solving
- Memory transfer optimization

### Force Generators

**Gravity**: `pe/core/Gravity.h`
- Uniform gravitational field
- Configurable direction and magnitude

**Springs**: `pe/core/Spring.h`
- Linear spring forces
- Damping support
- Connection between bodies

### Joints and Constraints

**Joint Types**:
- **BallJoint**: `pe/core/joint/BallJoint.h`
- **HingeJoint**: `pe/core/joint/HingeJoint.h`
- **SliderJoint**: `pe/core/joint/SliderJoint.h`
- **FixedJoint**: `pe/core/joint/FixedJoint.h`

## Performance Considerations

### Memory Management

**Object Pools**: Efficient allocation for frequently created objects
- **Body Pool**: `pe/core/rigidbody/BodyStorage.h`
- **Contact Pool**: `pe/core/contact/Contact.h:76`

**Template Optimizations**: Compile-time algorithm selection eliminates runtime overhead

### Algorithmic Complexity

**Coarse Detection**: O(n) average case with HashGrids
**Fine Detection**: O(k) where k = potentially colliding pairs
**Batch Generation**: O(n) linear in contact count
**Constraint Solving**: O(k·i) where i = iteration count

### Scalability

**MPI Scaling**: Tested up to hundreds of processes
**Thread Scaling**: Limited by batch count and load balance
**Memory Scaling**: Linear in body count with good cache locality

## Getting Started Guide

### Basic Simulation Setup

```cpp
#include <pe/engine.h>

int main() {
    // Initialize MPI (if enabled)
    pe::MPI::init();
    
    // Access the simulation world
    WorldID world = theWorld();
    
    // Configure simulation parameters
    world->setGravity(0, 0, -9.81);
    
    // Create rigid bodies
    SphereID sphere = createSphere(1, 0, 0, 10, 1.0, iron);
    PlaneID ground = createPlane(2, 0, 0, 1, 0, granite);
    
    // Simulation loop
    for (int step = 0; step < 1000; ++step) {
        world->simulationStep(0.01);  // 10ms timestep
    }
    
    // Cleanup
    pe::MPI::finalize();
    return 0;
}
```

### Key Headers to Include

```cpp
#include <pe/engine.h>              // Main engine header
#include <pe/core.h>                // Core simulation components  
#include <pe/materials.h>           // Material definitions
#include <pe/irrlicht.h>            // Visualization (if enabled)
```

### Configuration Customization

To modify the physics pipeline, edit `pe/config/Collisions.h`:

```cpp
// Switch to different algorithms
#define pe_COARSE_COLLISION_DETECTOR  pe::detection::coarse::SweepAndPrune
#define pe_BATCH_GENERATOR            pe::batches::SingleBatch  
#define pe_CONSTRAINT_SOLVER          pe::response::DEMSolver
```

### Build System Integration

Refer to `CLAUDE.md` in the project root for detailed build instructions:
- CMake-based build system (recommended)
- Custom configure script (legacy)
- Dependency management for optional features

## Debugging and Analysis

### Logging System

**Debug Output**: Comprehensive logging throughout the pipeline
- **Collision Detection**: Contact generation statistics
- **Batch Generation**: Batch composition and load balance
- **Constraint Solving**: Iteration counts and convergence

### Visualization Tools

**Real-time Debugging**: Irrlicht integration shows:
- Body positions and orientations
- Contact points and normals  
- Force vectors and constraint violations
- Performance statistics

### Performance Profiling

**Timing Sections**: `pe/util/Timing.h`
- Per-stage timing measurement
- Parallel efficiency analysis
- Memory usage tracking

## Extension Points

### Custom Geometry Types

To add new geometry primitives:
1. Inherit from `RigidBody` base class
2. Implement collision detection functions
3. Add to configuration template
4. Update fine detection algorithms

### Custom Solvers

To implement new constraint solvers:
1. Follow the solver interface pattern
2. Create configuration specialization  
3. Implement collision system integration
4. Add to configuration selection

### Custom Materials

Define new materials with specific properties:
```cpp
const Material myMaterial = Material::create("MyMaterial", 
                                           density, 
                                           restitution,
                                           staticFriction,
                                           dynamicFriction);
```

## Conclusion

The PE physics engine provides a flexible, high-performance framework for rigid body simulation with strong emphasis on:

- **Modularity**: Template-based algorithm selection
- **Performance**: Multi-level parallelization and optimization
- **Extensibility**: Clear interfaces for custom components
- **Robustness**: Extensive testing and validation

This technical overview provides the foundation for understanding and extending the PE codebase. For specific implementation details, refer to the individual component documentation and the referenced source files throughout this document.