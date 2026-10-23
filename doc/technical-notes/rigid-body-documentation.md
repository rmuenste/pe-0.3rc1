# RigidBody Class Documentation

## Overview

The `RigidBody` class is the abstract base class for all rigid bodies in the PE (Physics Engine) framework. It serves as the foundation for all geometric primitives and provides the common interface and functionality shared by all rigid body objects in the physics simulation.

## Class Hierarchy and Inheritance

The RigidBody class follows a sophisticated inheritance hierarchy that leverages templates and traits to provide flexible and extensible physics simulation capabilities:

```
RigidBody (abstract base class)
└── inherits from: RigidBodyTrait<Config>
    └── inherits from: RigidBodyBase
```

### Base Classes

1. **RigidBodyBase**: Contains the fundamental data members and basic functionality for all rigid bodies
2. **RigidBodyTrait<Config>**: A template-based trait class that allows customization based on the configuration type
3. **RigidBody**: The concrete abstract base class that all geometry primitives inherit from

## Trait-Based Feature Integration

The PE framework uses a sophisticated trait system to integrate different physics features and solver algorithms. This design allows for compile-time specialization of rigid body behavior based on the selected configuration.

### Configuration-Based Specialization

The `RigidBodyTrait<Config>` template parameter allows different collision response algorithms and physics models to be integrated:

- **Default**: Basic rigid body functionality
- **FFDSolver**: Fast domain decomposition solver traits
- **DEMSolver**: Discrete Element Method solver traits  
- **HardContactSemiImplicitTimesteppingSolvers**: robust baseline hard-contact dynamics without fluid coupling
- **HardContactAndFluid**: hard-contact dynamics with fluid-coupling support
- **HardContactLubricated**: canonical lubrication/contact implementation

### Trait Files Location
- Base trait: `pe/core/rigidbody/rigidbodytrait/Default.h`
- Specialized traits: `pe/core/rigidbody/rigidbodytrait/*.h`

## Core Data Members

The RigidBody class manages essential physical and geometric properties:

### Physical Properties
- `mass_`, `invMass_`: Mass and inverse mass
- `I_`, `Iinv_`: Moment of inertia tensor and its inverse (body frame)
- `v_`, `w_`: Linear and angular velocity
- `surfaceVel_`: Surface velocity in the body frame (conveyor belts, see below; default zero)
- `force_`, `torque_`: Accumulated forces and torques

### Geometric Properties
- `gpos_`: Global position (center of mass)
- `rpos_`: Relative position (within superordinate body)
- `q_`, `R_`: Orientation (quaternion and rotation matrix)
- `aabb_`: Axis-aligned bounding box

### Simulation State
- `finite_`: Whether the body is finite or infinite
- `visible_`: Visibility flag for visualization
- `fixed_`: Whether the body is fixed in space
- `remote_`, `global_`: MPI-related flags for parallel simulation

## Relationship to Concrete Geometries

All concrete geometry types inherit from RigidBody through their respective trait classes:

### Concrete Geometry Classes

1. **Sphere** (`pe/core/rigidbody/Sphere.h`)
   - Inherits from: `SphereTrait<Config>`
   - Geometry type: `sphereType`
   - Properties: radius, optional shell thickness (hollow sphere)
   - Hollow spheres: `createHollowSphere( uid, gpos, radius, shellThickness, material )` or
     `Sphere::setShellThickness( t )` on an existing sphere (0 = solid). Only the mass (wall
     volume times density) and the inertia, `(2/5) m (R^5 - r^5) / (R^3 - r^3)` with the
     thin-shell limit `(2/3) m R^2`, differ from the solid sphere; collision geometry,
     contact generation and bounding box are those of the outer sphere, so no detection or
     solver code is involved. The thickness is marshalled with the sphere (MPI shadow copies,
     checkpoint file format 0.3; format 0.2 files read back as solid spheres). Set it before
     adding the sphere to a union. Test: `tests/interface/pe_hollow_sphere_test.cpp`.

2. **Box** (`pe/core/rigidbody/Box.h`)
   - Inherits from: `BoxTrait<Config>`
   - Geometry type: `boxType`
   - Properties: length, width, height

3. **Cylinder** (`pe/core/rigidbody/Cylinder.h`)
   - Inherits from: `CylinderTrait<Config>`
   - Geometry type: `cylinderType`
   - Properties: radius, length

4. **Capsule** (`pe/core/rigidbody/Capsule.h`)
   - Inherits from: `CapsuleTrait<Config>`
   - Geometry type: `capsuleType`
   - Properties: radius, length

5. **Plane** (`pe/core/rigidbody/Plane.h`)
   - Inherits from: `PlaneTrait<Config>`
   - Geometry type: `planeType`
   - Properties: normal vector, displacement

6. **TriangleMesh** (`pe/core/rigidbody/TriangleMesh.h`)
   - Inherits from: `TriangleMeshTrait<Config>`
   - Geometry type: `triangleMeshType`
   - Properties: vertices, normals, triangle indices

### Inheritance Pattern
```
ConcreteGeometry (e.g., Sphere)
└── inherits from: GeometryTrait<Config> (e.g., SphereTrait<Config>)
    └── inherits from: GeometryBase (e.g., SphereBase)
        └── inherits from: GeomPrimitive
            └── inherits from: RigidBody
```

## Key Functionality

### Transformation Functions
- Coordinate transformations between body frame (BF) and world frame (WF)
- `pointFromBFtoWF()`, `pointFromWFtoBF()`: Point transformations
- `vectorFromBFtoWF()`, `vectorFromWFtoBF()`: Vector transformations
- `velFromBF()`, `velFromWF()`: Velocity calculations

### Force and Impulse Application
- `addForce()`, `addForceAtPos()`: Apply forces
- `addTorque()`: Apply torques
- `addImpulse()`, `addImpulseAtPos()`: Apply impulses
- `resetForce()`: Clear accumulated forces

### Geometric Queries
- `containsPoint()`, `containsRelPoint()`: Point containment tests
- `isSurfacePoint()`, `isSurfaceRelPoint()`: Surface point tests
- `support()`: Support function for collision detection

### MPI and Parallel Simulation
- `isRemote()`, `setRemote()`: Remote body management
- `isGlobal()`, `setGlobal()`: Global body handling

### Prescribed Motion
- `setTargetPose( gpos, q, dt )`: velocities that move the body onto a pose in one step
- `setRelSurfaceVel()`, `setSurfaceVel()`, `getRelSurfaceVel()`, `getSurfaceVel()`, `hasSurfaceVel()`:
  surface velocity in the body / world frame

## Kinematic (Moving Fixed) Bodies

pe has no separate kinematic body type (unlike Box3D's `b3_kinematicBody` or Bullet's kinematic
flag): **a fixed body with a prescribed velocity is the kinematic body.** `setFixed( true )` gives
the body infinite mass and inertia (`invMass_ = 0`, `Iinv_ = 0`); with mobile infinite bodies it
keeps its linear and angular velocity, and `setLinearVel()` / `setAngularVel()` (and the `Rel`
variants) work on it. The hard-contact collision systems (`HardContactSemiImplicitTimesteppingSolvers`,
`HardContactAndFluid`, `HardContactEulerLagrange`) then

- integrate its position and orientation with exactly that velocity (a global fixed body with
  `v_` alone, a local one with `v_ + dv_` where `dv_ = 0`), with neither gravity nor world damping
  (`World::setDamping()` does not touch a prescribed velocity);
- let its contacts see the velocity, so it pushes and drags other bodies without being pushed back.

The velocity persists until it is set again; to stop the body set it to zero. A shaker wall, a
piston or a moving platform is therefore a fixed body whose velocity is set before each step
(`examples/shaker`), and a bat or a stirrer that follows a trajectory uses `setTargetPose()`.

**Mobile infinite bodies are a build option** (`MOBILE_INFINITE`, CMake option
`PE_MOBILE_INFINITE`, default `ON` for every compiler; before 2026-10 the macro was defined for
GCC only, so the same code behaved differently under Clang/AppleClang/Intel):

| | `PE_MOBILE_INFINITE=ON` (default) | `OFF` (original pe) |
|---|---|---|
| `setFixed( true )` / `fix()` | keeps `v_`, `w_` | zeroes `v_`, `w_` |
| velocity setters on a fixed body | set the velocity | silently ignored |
| `isActive()` of a fixed body | `awake_` | `false` |
| `setTargetPose()` on a fixed body | moves it | no effect |
| surface velocity | works | works (independent of the macro) |

### setTargetPose()

`RigidBody::setTargetPose( const Vec3& gpos, const Quat& q, real dt )` sets
`v = (gpos - getPosition()) / dt` and `w = axis * angle / dt` from the shortest-arc rotation
`q * getQuaternion()^-1` (angle in `[0, pi]`, computed with `atan2`, accurate for small angles).
After the next `World::simulationStep( dt )` a fixed body sits on the target pose to rounding
(1e-16 in `tests/interface/pe_kinematic_body_test.cpp`, global and local bodies, a fixed Union,
rotations beyond pi), and its contacts see the velocity it got there with (Box3D's
`b3Body_SetTargetTransform()`). The position is the **center of mass** (`getPosition()`, for a
Union the union's center of mass), which the body rotates about; to prescribe another reference
point `p` given in the body frame pass `gpos = p_target - R(q) p`. On a non-fixed body the
function only sets the velocities (gravity, forces and contacts act on top). It wakes the body,
has no effect on bodies inside a Union (like the velocity setters) and throws
`std::invalid_argument` for `dt <= 0`.

### Surface velocity

`setRelSurfaceVel( v )` gives a body a velocity of its surface material relative to the body,
in the body frame, so it turns with the body (Box3D's `b3SurfaceMaterial::tangentVelocity`,
LIGGGHTS' `surface_vel` of a stationary mesh). `setSurfaceVel( v )` takes a world-frame vector and
stores it in the body frame at the current orientation. A conveyor belt is then a fixed box that
does not move at all (`examples/conveyor`), a treadmill a fixed plane. At a contact
(`ContactBase::calcSurfaceVel()`):

- each side's world-frame surface velocity is that of the primitive plus that of its enclosing
  top-level Union, so it can be set on a body inside a Union (one belt of a machine) or on the
  Union (all of it); the difference body 1 minus body 2 is projected onto the tangent plane and
  stored as `Contact::getSurfaceVel()`;
- it enters the relative contact velocity of the **friction only**. The normal
  (non-penetration) constraint never sees it, a frictionless contact ignores it, and the body
  itself is not moved by it. `Contact::getRelVel()` contains it, `getNormalRelVel()` does not.
- the choice between the static and the dynamic coefficient of friction (`Contact::Contact()`)
  uses the slip including it: a box riding a belt at belt speed gets the static coefficient, a
  box held on a running belt the dynamic one.

It works on fixed and non-fixed bodies, independently of `MOBILE_INFINITE`, and is honoured by
the three hard-contact systems (`static constexpr bool hasSurfaceVelocity` in the collision
system; any other solver: `World::simulationStep()` warns once, see
[contact-solvers-overview.md](contact-solvers-overview.md#kinematic-bodies-and-surface-velocities)).
An angular surface velocity is not provided: a fixed body of revolution (a roller, a turntable)
can simply rotate in place with `setAngularVel()`.

It is marshalled with every rigid body record (`marshal( Buffer&, const RigidBody& )`): MPI
shadow copies in the `synchronize()` of the five collision systems that use it (also of bodies
inside a Union) and checkpoints from file format 0.4 (0.2/0.3 files read back with zero surface
velocity; checked with a 0.3 file written by the previous version). The legacy
`MPIEncoder`/`MPIDecoder` path (`DEMSolverObsolete`, `FFDSolver`) does not carry it; those solvers
do not support surface velocities anyway. It is
not part of the per-step shadow-copy update notification (which would cost every particle 24 bytes
per step): a change on a body that already has shadow copies does not reach them. Set it on
global bodies (on every process, the usual case for belts and walls) or before the body is first
synchronized. The lubrication stage does not see it.

### Pairs of fixed bodies

Two fixed bodies never get a contact: the coarse detectors (`HashGrids::collide()`,
`SweepAndPrune`, `ExhaustiveSearch`) drop the pair before the fine detection, so a kinematic body
may move through walls, planes and other fixed bodies (tested: a moving fixed box passing through
a fixed box and a plane creates no contact, also in a Debug build). The hard-contact solvers keep
their `pe_INTERNAL_ASSERT` as a safety net and, in release builds, skip such a contact should one
ever reach them (its diagonal block would be zero and its impulse not a number).

### Sleeping

Sleep mode is compiled out by default (`pe::sleepThreshold = 0`, `pe/config/SleepMode.h`; enable
it with `-DPE_PREPROCESSOR_FLAGS=-DPE_SLEEP_THRESHOLD=<threshold>`). With it on:

- a body that drives the motion of others (`RigidBodyBase::drivesMotion()`: fixed with a non-zero
  velocity, or any body with a non-zero surface velocity) never falls asleep; sleeping would zero,
  i.e. freeze, the prescribed velocity (the Bullet pitfall);
- the hard-contact systems wake every body that touches such a body, or a contact with a non-zero
  surface velocity, before its velocity is cached (`RigidBodyBase::wakeByContact()`), and hold it
  awake for the step. The hold matters: friction accelerates a resting body gradually, and a body
  woken with its old (sub-threshold) motion fell asleep again at the end of the step and lost the
  velocity it had gained, so a platform never carried it;
- a sleeping body gets no velocity correction (`dv = dw = 0`, the stale-entry fix of fixed bodies).

Tests: `tests/interface/pe_kinematic_body_test.cpp` (fixed bodies keep velocities, fixed-fixed
pairs, `setTargetPose()`, sleep) and `tests/interface/pe_surface_velocity_test.cpp`, both run
against the three hard-contact pipelines (CTest `pe-kinematic-body*`, `pe-surface-velocity*`).

## Adding a New Concrete Geometry: Cone Example

To add a new geometry type (e.g., Cone) to the PE framework, follow these steps:

### Step 1: Update GeomType Enumeration
Add a new geometry type to `pe/core/GeomType.h`:
```cpp
enum GeomType {
   // ... existing types ...
   ellipsoidType     = 10,  // existing
   coneType         = 11   // new cone type
};
```

### Step 2: Create Base Class
Create `pe/core/rigidbody/ConeBase.h`:
```cpp
class ConeBase : public GeomPrimitive
{
protected:
   explicit ConeBase( GeomType type, id_t sid, id_t uid, const Vec3& gpos,
                      real radius, real height, MaterialID material, bool visible );
   
   real radius_;  // Base radius
   real height_;  // Height of cone
   
public:
   inline real getRadius() const { return radius_; }
   inline real getHeight() const { return height_; }
   
   // Geometric calculations specific to cone
   virtual real getVolume() const;
   virtual Vec3 getRelCenterOfMass() const;
   virtual Mat3 getBodyInertia() const;
};
```

### Step 3: Create Trait Class
Create `pe/core/rigidbody/ConeTrait.h`:
```cpp
template< typename C >
class ConeTrait : public ConeBase
{
protected:
   explicit ConeTrait( id_t sid, id_t uid, const Vec3& gpos,
                       real radius, real height, MaterialID material, bool visible );
   
public:
   // Cone-specific functionality
   virtual void calcBoundingBox();
   virtual bool containsRelPoint( const Vec3& rpos ) const;
   virtual bool isSurfaceRelPoint( const Vec3& rpos ) const;
   virtual Vec3 support( const Vec3& d ) const;
};
```

### Step 4: Create Main Cone Class
Create `pe/core/rigidbody/Cone.h`:
```cpp
class Cone : public ConeTrait<Config>
{
private:
   typedef ConeTrait<Config> Parent;
   
protected:
   explicit Cone( id_t sid, id_t uid, const Vec3& gpos,
                  real radius, real height, MaterialID material, bool visible );
   
public:
   // Transformation functions
   virtual void translate( const Vec3& dp );
   virtual void rotate( const Quat& dq );
   virtual void setPosition( const Vec3& gpos );
   virtual void setOrientation( const Quat& q );
   
   // Utility functions
   virtual bool containsRelPoint( const Vec3& rpos ) const;
   virtual bool containsPoint( const Vec3& gpos ) const;
   virtual bool isSurfaceRelPoint( const Vec3& rpos ) const;
   virtual bool isSurfacePoint( const Vec3& gpos ) const;
   
   // Output
   virtual void print( std::ostream& os, const char* tab ) const;
   
protected:
   virtual void update( const Vec3& dp );
   virtual void update( const Quat& dq );
};
```

### Step 5: Implement Creation Functions
Create cone creation functions in appropriate setup files:
```cpp
ConeID createCone( id_t uid, const Vec3& gpos, real radius, real height,
                   MaterialID material, bool visible = true );

ConeID createCone( id_t uid, real x, real y, real z, real radius, real height,
                   MaterialID material, bool visible = true );
```

### Step 6: Update Factory and Type System
1. Add cone creation to the geometry factory
2. Update type casting functions in `pe/core/rigidbody/BodyCast.h`
3. Add cone-specific collision detection algorithms
4. Update serialization/deserialization for MPI support

### Step 7: Integration with Collision System
1. Implement collision detection algorithms for cone-cone, cone-sphere, cone-box, etc.
2. Add cone support to broad-phase collision detection
3. Update contact generation algorithms

## Usage Notes

- All rigid bodies are automatically managed by the `BodyManager`
- Bodies can be part of `Union` compound geometries
- The trait system allows compile-time optimization for different physics models
- MPI parallelization is supported through remote/global body mechanisms
- Visualization integration is handled through the visibility flag

## Key Files

- `pe/core/rigidbody/RigidBody.h`: Main RigidBody class
- `pe/core/rigidbody/RigidBodyTrait.h`: Trait system header
- `pe/core/rigidbody/rigidbodytrait/*.h`: Specialized trait implementations
- `pe/core/GeomType.h`: Geometry type enumeration
- `pe/core/rigidbody/*.h`: Concrete geometry implementations

This architecture provides a flexible and extensible framework for rigid body physics simulation while maintaining high performance through template-based compile-time optimization.
