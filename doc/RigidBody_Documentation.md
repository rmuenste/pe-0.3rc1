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
- **HardContactSemiImplicitTimesteppingSolvers**: Semi-implicit timestepping for hard contacts
- **HardContactAndFluid**: Combined hard contact and fluid dynamics
- **HardContactFluidLubrication**: Lubrication effects in fluid-solid interactions

### Trait Files Location
- Base trait: `pe/core/rigidbody/rigidbodytrait/Default.h`
- Specialized traits: `pe/core/rigidbody/rigidbodytrait/*.h`

## Core Data Members

The RigidBody class manages essential physical and geometric properties:

### Physical Properties
- `mass_`, `invMass_`: Mass and inverse mass
- `I_`, `Iinv_`: Moment of inertia tensor and its inverse (body frame)
- `v_`, `w_`: Linear and angular velocity
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
   - Properties: radius

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