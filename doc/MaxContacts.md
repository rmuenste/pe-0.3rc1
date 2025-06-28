# MaxContacts Class Documentation

## Overview

The `MaxContacts` class is a fundamental component of the PE (Physics Engine) fine collision detection system. Located in `pe/core/detection/fine/MaxContacts.h`, this class serves as the primary contact generation module responsible for creating accurate contact points between colliding rigid bodies.

## Purpose and Design Philosophy

The MaxContacts class follows a "maximum accuracy" design philosophy, creating the maximal number of contact points necessary to handle collisions between rigid bodies with physical accuracy. This approach prioritizes simulation fidelity over computational efficiency, making it ideal for applications requiring precise collision response.

## Class Structure

```cpp
class MaxContacts : private NonCreatable
{
public:
   // Contact generation functions for all geometry pairs
   template< typename CC > static void collide(BodyID b1, BodyID b2, CC& contacts);
   // ... specialized collision functions for each geometry type pair
   
protected:
   // Utility functions for GJK/EPA collision detection
   template <typename Type1, typename Type2>
   static inline bool gjkEPAcollideHybrid(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint, real& penetrationDepth);
   // ... other utility functions
};
```

### Key Design Features

- **Non-instantiable**: Inherits from `NonCreatable`, making it a pure utility class with only static methods
- **Template-based**: Uses templates to handle different geometry types and contact containers
- **Comprehensive coverage**: Provides specialized collision detection for all supported geometry pairs

## Contact Generation Methods

The MaxContacts class provides collision detection between the following geometry types:

### Supported Geometry Types
- **Sphere**: Basic spherical primitives
- **Box**: Axis-aligned and oriented boxes
- **Capsule**: Cylinder with hemispherical caps
- **Cylinder**: Standard cylindrical shapes
- **InnerCylinder**: Hollow cylindrical shapes
- **Plane**: Infinite planes
- **TriangleMesh**: Complex polygonal meshes
- **Union**: Composite shapes

### Collision Function Naming Convention

The collision functions follow a consistent naming pattern:
```cpp
template<typename CC> 
static void collide[Geom1][Geom2](Geom1ID g1, Geom2ID g2, CC& contacts);
```

Examples:
- `collideSphereSphere()` - Sphere-sphere collision
- `collideSphereBox()` - Sphere-box collision
- `collideBoxBox()` - Box-box collision
- `collideCapsulePlane()` - Capsule-plane collision

## GJK/EPA Algorithm Integration

MaxContacts integrates the Gilbert-Johnson-Keerthi (GJK) and Expanding Polytope Algorithm (EPA) for collision detection:

### Hybrid GJK/EPA Approach

```cpp
template <typename Type1, typename Type2>
inline bool gjkEPAcollideHybrid(Type1 geom1, Type2 geom2, 
                                Vec3& normal, Vec3& contactPoint, 
                                real& penetrationDepth)
```

This hybrid approach:
1. **Distance Check**: Uses GJK to compute the distance between objects
2. **Threshold Evaluation**: Compares distance against `contactThreshold`
3. **Penetration Detection**: Uses EPA for objects in contact or penetrating
4. **Contact Generation**: Creates appropriate contact information

### Algorithm Flow

1. **Initial GJK**: Compute minimum distance between geometries
2. **Distance Classification**:
   - `> contactThreshold`: No contact generated
   - `< 0.01 * contactThreshold`: Use GJK+EPA for penetration analysis
   - Otherwise: Use GJK data for separated but close objects
3. **EPA Refinement**: For penetrating objects, EPA provides accurate contact normal and depth

## Contact Class Integration

### Contact Class Overview

The `Contact` class represents individual contact points between rigid bodies:

```cpp
class Contact : public ContactTrait<Config>
{
public:
   // Constructors for different contact types
   explicit Contact(GeomID g1, GeomID g2, const Vec3& gpos, 
                   const Vec3& normal, real dist);
   explicit Contact(GeomID g1, GeomID g2, const Vec3& gpos, 
                   const Vec3& normal, const Vec3& e1, 
                   const Vec3& e2, real dist);
   
   // Memory management
   static void* operator new(size_t size);
   static void operator delete(void* rawMemory);
};
```

### Contact Properties

Each contact contains:
- **Geometry References**: Pointers to the two colliding geometries
- **Contact Position**: Global position of the contact point
- **Contact Normal**: Unit vector pointing from geometry 1 to geometry 2
- **Penetration Distance**: Depth of penetration (negative for separation)
- **Contact Edges** (optional): Edge vectors for extended contact information

### Contact Classification

Contacts are classified based on relative velocity:
- **Separating Contact**: `v_rel > 0` (objects moving apart)
- **Resting Contact**: `v_rel = 0` (objects at rest)
- **Colliding Contact**: `v_rel < 0` (objects approaching)

### Memory Management

The Contact class uses a memory pool for efficient allocation:
```cpp
typedef MemoryPool<Contact, 10000> ContactPool;
```

This provides fast allocation/deallocation for the frequently created and destroyed contact objects.

## Usage in the Physics Engine

### Configuration

MaxContacts is configured as the default fine collision detector:

```cpp
// In pe/config/Collisions.h
#define pe_FINE_COLLISION_DETECTOR pe::detection::fine::MaxContacts
```

### Integration with Collision System

1. **Coarse Detection**: Broad-phase algorithms identify potentially colliding pairs
2. **Fine Detection**: MaxContacts generates precise contact information
3. **Contact Resolution**: Generated contacts are passed to the constraint solver
4. **Physics Response**: Contacts influence forces and motion updates

### Template Parameters

The collision functions accept a template parameter `CC` representing the contact container:
- Must support `push_back()` or similar insertion methods
- Typically `std::vector<Contact>` or similar container types
- Allows flexible contact storage strategies

## Performance Considerations

### Computational Complexity

- **Sphere-Sphere**: O(1) - Analytical solution
- **Primitive-Primitive**: O(1) to O(log n) - Analytical or iterative methods
- **Mesh Collisions**: O(n) to O(n²) - Depends on mesh complexity
- **GJK/EPA**: O(k) - Where k is iteration count (typically small)

### Optimization Features

- **Early Termination**: Distance thresholds prevent unnecessary computations
- **Analytical Methods**: Direct solutions for simple geometry pairs
- **Hybrid Algorithms**: Combines efficiency of GJK with accuracy of EPA
- **Memory Pooling**: Reduces allocation overhead for contact objects

## Algorithm Details

### Threshold Management

The system uses several thresholds for robust collision detection:
- **contactThreshold**: Minimum distance for contact generation
- **collisionThreshold**: Tolerance for velocity classification
- **Numerical Epsilon**: Prevents floating-point precision issues

### Robustness Features

- **Numerical Stability**: Careful handling of edge cases and degeneracies
- **Geometric Tolerance**: Appropriate thresholds for real-world precision
- **Fallback Methods**: Alternative algorithms for challenging cases

## Extensibility

### Adding New Geometry Types

To support a new geometry type:

1. **Create Geometry Class**: Implement the new primitive type
2. **Add Collision Functions**: Implement collision detection with existing types
3. **Update MaxContacts**: Add the new collision function templates
4. **GJK Support**: Ensure the geometry supports GJK distance queries

### Custom Contact Information

The template-based design allows custom contact containers and additional contact data without modifying the core algorithms.

## Relationship to Other Components

### Dependencies

- **GJK Algorithm**: Distance computation between convex shapes
- **EPA Algorithm**: Penetration depth and normal calculation
- **Geometry Primitives**: All supported rigid body types
- **Mathematical Utilities**: Vector operations, transformations
- **Contact System**: Contact class and container management

### Integration Points

- **Collision System**: Called by the fine collision detection pipeline
- **Response System**: Provides contacts to constraint solvers
- **Visualization**: Contact points can be rendered for debugging
- **Analysis Tools**: Contact statistics and debugging information

## Conclusion

The MaxContacts class represents a comprehensive and robust solution for fine collision detection in the PE physics engine. Its design prioritizes accuracy and completeness, making it suitable for applications requiring precise physical simulation. The integration with modern collision detection algorithms (GJK/EPA) and careful attention to numerical robustness ensures reliable operation across a wide range of simulation scenarios.