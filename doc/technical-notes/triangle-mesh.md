# TriangleMesh Class Hierarchy Documentation - PE Physics Engine

## Overview

This document provides a comprehensive analysis of the TriangleMesh class hierarchy in the PE physics engine, detailing the complete inheritance structure, related types, implementation details, and ecosystem components.

## Class Inheritance Hierarchy

### Complete Inheritance Chain

```
TriangleMesh (concrete class)
    ↑ inherits from
TriangleMeshTrait<Config> (template trait specialization)
    ↑ inherits from  
TriangleMeshBase (abstract base class)
    ↑ inherits from
GeomPrimitive (abstract geometric primitive base)
    ↑ inherits from
RigidBody (abstract rigid body base)
    ↑ inherits from
RigidBodyTrait<Config> (template trait specialization)
    ↑ inherits from
RigidBodyBase (core rigid body implementation)
    ↑ inherits from (multiple inheritance)
├── detection::coarse::BodyTrait<Config>    // Coarse collision detection
├── detection::fine::BodyTrait<Config>      // Fine collision detection  
├── batches::BodyTrait<Config>              // Batch generation
├── response::BodyTrait<Config>             // Collision response
├── BodyExtension                           // User extensions
└── NonCopyable                             // Utility base
```

### File Locations and Responsibilities

| Class | File Location | Primary Responsibility |
|-------|---------------|----------------------|
| **TriangleMesh** | `pe/core/rigidbody/TriangleMesh.h` | Final concrete triangle mesh implementation |
| **TriangleMeshTrait<Config>** | `pe/core/rigidbody/trianglemeshtrait/Default.h` | Collision response customization |
| **TriangleMeshBase** | `pe/core/rigidbody/TriangleMeshBase.h` | Abstract triangle mesh functionality |
| **GeomPrimitive** | `pe/core/rigidbody/GeomPrimitive.h` | Material properties and geometric interface |
| **RigidBody** | `pe/core/rigidbody/RigidBody.h` | Abstract rigid body interface |
| **RigidBodyTrait<Config>** | `pe/core/rigidbody/rigidbodytrait/Default.h` | Rigid body collision customization |
| **RigidBodyBase** | `pe/core/rigidbody/RigidBodyBase.h` | Core rigid body implementation |

## Core Type Definitions and Data Structures

### Primary Data Types (`pe/core/rigidbody/TriangleMeshTypes.h`)

```cpp
typedef std::vector<Vec3> Vertices;              // Triangle vertices
typedef std::vector<Vec3> Normals;               // Face and vertex normals
typedef std::vector<Vector3<size_t>> IndicesLists; // Face vertex indices
typedef std::vector<size_t> IndexList;           // General indexing
typedef std::vector<Vec2> TextureCoordinates;    // UV texture mapping
```

### Handle Types (`pe/core/Types.h`)

```cpp
typedef TriangleMesh* TriangleMeshID;             // Non-const handle
typedef const TriangleMesh* ConstTriangleMeshID;  // Const handle
typedef InnerMesh* InnerMeshID;                   // Specialized inner mesh
typedef const InnerMesh* ConstInnerMeshID;        // Const inner mesh
```

## Key Member Variables and Data Structures

### Core Geometry Data
```cpp
class TriangleMesh {
private:
    // Core geometric data
    Vertices vertices_;              // Inherited from TriangleMeshBase
    IndicesLists faceIndices_;       // Triangle face definitions
    Normals faceNormals_;           // Per-face normal vectors
    
    // Visualization data
    Normals vertexNormals_;         // Per-vertex normals for smooth rendering
    IndicesLists normalIndices_;    // Vertex normal assignments
    TextureCoordinates textureCoordinates_; // UV mapping coordinates
    IndicesLists textureIndices_;   // Texture coordinate assignments
    
    // Topology and optimization
    IndexList vertexEdge_;          // Vertex-to-edge mapping
    IndexList edgeEdge_;            // Edge pair relationships
    IndexList vertexVNeighbor_;     // Virtual neighbor vertices
    
    // Performance optimization
    size_t aabbBottomFrontLeftIndices_[3];  // AABB extreme vertices
    size_t aabbTopBackRightIndices_[3];     // AABB extreme vertices
    bool convex_;                   // Convexity flag for optimization
    size_t smallMeshLimit_;         // Algorithm switching threshold
    
    // Rendering cache
    bool renderSmooth_;             // Smooth vs flat shading
    std::string cachedPOVString_;   // POV-Ray cached output
#if HAVE_IRRLICHT
    ::irr::scene::SMeshBuffer* cachedIrrlichtBuffer_; // Irrlicht cache
#endif
};
```

## Factory Functions and Creation Mechanisms

### Primary Creation Functions

```cpp
// File-based creation
TriangleMeshID createTriangleMesh(
    id_t uid, const Vec3& gpos, const std::string file,
    MaterialID material, bool convex, bool visible=true,
    const Vec3& scale=Vec3(1.0, 1.0, 1.0), 
    bool clockwise=false, bool lefthanded=false
);

// Copy creation
TriangleMeshID createTriangleMesh(
    id_t uid, const Vec3& gpos, const TriangleMeshID source,
    MaterialID material, bool visible=true
);

// Direct vertex/face creation
TriangleMeshID createTriangleMesh(
    id_t uid, const Vec3& gpos, Vertices vertices,
    const IndicesLists& faces, MaterialID material,
    bool convex, bool visible=true
);

// MPI instantiation
TriangleMeshID instantiateTriangleMesh(
    id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
    const Quat& q, const Vertices& vertices,
    const IndicesLists& faces, MaterialID material,
    bool visible, bool fixed, bool reg=true
);
```

### Specialized Mesh Generators

```cpp
// Primitive generators
TriangleMeshID createRegularTetrahedron(
    id_t uid, const Vec3& gpos, real radius,
    MaterialID material, bool visible=true
);

TriangleMeshID createTriangulatedBox(
    id_t uid, const Vec3& gpos, const Vec3& lengths,
    MaterialID material, bool visible=true,
    const Vector3<size_t>& tessellation=Vector3<size_t>(1,1,1)
);

TriangleMeshID createRock(
    id_t uid, const Vec3& gpos, real radius, 
    MaterialID material, bool visible=true
);
```

## Mesh Loading Infrastructure

### Mesh Loader Classes

| Loader Class | File Location | Supported Format |
|--------------|---------------|------------------|
| **MeshLoader** | `pe/core/MeshLoader.h` | Abstract base interface |
| **OBJMeshLoader** | `pe/core/OBJMeshLoader.h` | Wavefront OBJ files |
| **STLMeshLoader** | `pe/core/STLMeshLoader.h` | STL files |

### Loading Capabilities

- **OBJ Format Support**: Vertices, faces, normals, texture coordinates
- **STL Format Support**: Basic triangle mesh data
- **Coordinate System Handling**: Left/right-handed coordinate conversion
- **Winding Order**: Clockwise/counter-clockwise triangle handling
- **Scaling**: Uniform and non-uniform scaling during load

## Virtual Function Interface

### Core Virtual Methods

```cpp
class TriangleMesh {
public:
    // Geometric transformations
    virtual void setVisible(bool visible) override;
    virtual void setPosition(real px, real py, real pz) override;
    virtual void setPosition(const Vec3& gpos) override;
    virtual void setOrientation(real r, real i, real j, real k) override;
    virtual void setOrientation(const Quat& q) override;
    
    // Translation operations
    virtual void translate(real dx, real dy, real dz) override;
    virtual void translate(const Vec3& dp) override;
    
    // Rotation operations
    virtual void rotate(real x, real y, real z, real angle) override;
    virtual void rotate(const Vec3& axis, real angle) override;
    virtual void rotate(real xangle, real yangle, real zangle) override;
    virtual void rotate(const Vec3& euler) override;
    virtual void rotate(const Quat& dq) override;
    virtual void rotateAroundOrigin(...) override;
    virtual void rotateAroundPoint(...) override;
    
    // Geometric queries
    virtual bool containsRelPoint(real px, real py, real pz) const override;
    virtual bool containsRelPoint(const Vec3& rpos) const override;
    virtual bool containsPoint(real px, real py, real pz) const override;
    virtual bool containsPoint(const Vec3& gpos) const override;
    virtual bool isSurfaceRelPoint(...) const override;
    virtual bool isSurfacePoint(...) const override;
    
    // Collision support
    virtual Vec3 support(const Vec3& d) const override;
    virtual Vec3 supportContactThreshold(const Vec3& d) const override;
    
    // Utility
    virtual void calcBoundingBox() override;
    virtual void print(std::ostream& os, const char* tab) const override;
};
```

## Template Specialization System

### Configuration Template

```cpp
template< typename C >  // Type of the configuration
class TriangleMeshTrait : public TriangleMeshBase
{
    // Collision response customization
    virtual void move(real dt) override;
};
```

### Configuration Types

The `Config` parameter is defined in `pe/core/Configuration.h`:

```cpp
typedef Configuration< pe_COARSE_COLLISION_DETECTOR,
                      pe_FINE_COLLISION_DETECTOR, 
                      pe_BATCH_GENERATOR,
                      pe_CONSTRAINT_SOLVER >::Config Config;
```

## Collision Detection Integration

### Support Function Implementation

```cpp
// GJK/EPA collision detection support
Vec3 TriangleMesh::support(const Vec3& d) const;
Vec3 TriangleMesh::support(const Vec3& d, size_t startIndex, size_t* pointIndex) const;
Vec3 TriangleMesh::supportContactThreshold(const Vec3& d) const;
```

### Collision System Integration

Triangle meshes are integrated into multiple collision system configurations:
- **HardContactAndFluid**: Standard rigid body dynamics
- **HardContactFluidLubrication**: With lubrication forces
- **HardContactSemiImplicitTimesteppingSolvers**: Semi-implicit time integration
- **DEMSolver**: Discrete element method
- **OpenCLSolver**: GPU-accelerated collision detection

### Fine Collision Detection

- **MaxContacts** class (`pe/core/detection/fine/MaxContacts.h`) provides specialized triangle mesh collision detection
- Friend class relationship allows optimized access to internal mesh data

## Visualization and Rendering Support

### Irrlicht Integration

```cpp
#if HAVE_IRRLICHT
class TriangleMesh {
private:
    ::irr::scene::SMeshBuffer* cachedIrrlichtBuffer_;
    
public:
    ::irr::scene::SMeshBuffer* getIrrlichtCacheBuffer() const;
    void updateIrrlichtCacheBuffer();
    void setRenderSmooth(bool smooth);
};
#endif
```

### POV-Ray Integration

```cpp
class TriangleMesh {
private:
    std::string cachedPOVString_;
    
public:
    void printPOVmesh2(std::ostream& os, const povray::Texture& texture,
                       const Vec3 euler, const Vec3 povPos) const;
    void setColoredTriangleTexture(povray::WriterID pov);
    void updatePOVcacheString();
};
```

### VTK and OpenDX Support

- **VTK Writer** (`pe/vtk/Writer.h`): Scientific visualization export
- **OpenDX Writer** (`pe/opendx/Writer.h`): Alternative visualization format
- **Vertex index mapping**: Direct correspondence with VTK indices

## Advanced Features and Optimizations

### Half-Edge Data Structure

```cpp
class TriangleMesh {
private:
    IndexList vertexEdge_;        // Vertex-to-edge mapping
    IndexList edgeEdge_;          // Edge pair relationships  
    IndexList vertexVNeighbor_;   // Virtual neighbor vertices
    
    void initHalfEdge();          // Initialize topology
};
```

**Benefits:**
- Efficient mesh traversal for collision detection
- Fast neighbor queries for optimization algorithms
- Support for complex geometric operations

### Performance Optimizations

```cpp
class TriangleMesh {
private:
    size_t smallMeshLimit_;       // Algorithm switching threshold (default: 512)
    bool convex_;                 // Convexity hint for optimization
    
    // AABB extreme vertex caching
    size_t aabbBottomFrontLeftIndices_[3];
    size_t aabbTopBackRightIndices_[3];
};
```

**Optimization Strategies:**
- **Small mesh limit**: Switch between brute force and sophisticated algorithms
- **Convexity flag**: Enable GJK/EPA optimizations for convex meshes
- **AABB caching**: Fast bounding box updates using cached extreme vertices
- **Rendering cache**: Avoid redundant visualization data generation

### Material and Physical Properties

```cpp
class TriangleMesh {
public:
    // Volume and mass calculations
    real getVolume() const;
    static real calcVolume(const Vertices& vertices, const IndicesLists& faceIndices);
    static real calcMass(const Vertices& vertices, const IndicesLists& faceIndices, real density);
    static real calcDensity(const Vertices& vertices, const IndicesLists& faceIndices, real mass);
    
    // Surface and containment queries
    bool hasClosedSurface() const;
    real getDepth(const Vec3& gpos) const;
    real getDistance(const Vec3& gpos) const;
    Vec3 getFaceNormal(std::size_t idx) const;
};
```

## Domain Decomposition Integration

### Boundary Support Classes

| Class | File Location | Purpose |
|-------|---------------|---------|
| **TriMeshBoundary** | `pe/core/domaindecomp/TriMeshBoundary.h` | MPI domain boundaries |
| **TriMeshDopBoundary** | `pe/core/domaindecomp/TriMeshDopBoundary.h` | k-DOP boundary approximation |

### Parallel Processing Features

- **MPI instantiation**: Support for distributed triangle mesh creation
- **Domain boundary checking**: Integration with domain decomposition
- **Ghost region handling**: Efficient cross-process mesh sharing

## Type System Integration

### Polymorphic Operations

The TriangleMesh class includes specialized template functions for polymorphic operations:

```cpp
// Counting operations
template<> size_t polymorphicCount<TriangleMesh>(RigidBody *const * first, RigidBody *const * last);
template<> size_t polymorphicCount<TriangleMesh>(GeomPrimitive *const * first, GeomPrimitive *const * last);

// Finding operations  
template<> RigidBody *const * polymorphicFind<TriangleMesh>(RigidBody *const * first, RigidBody *const * last);
template<> GeomPrimitive *const * polymorphicFind<TriangleMesh>(GeomPrimitive *const * first, GeomPrimitive *const * last);
```

### Type Identification

```cpp
// From pe/core/GeomType.h
enum GeomType {
    triangleMeshType = 6    // Unique identifier for triangle meshes
};
```

## Related Classes and Ecosystem

### Supporting Geometry Classes

| Class | Purpose | Relationship |
|-------|---------|--------------|
| **InnerMesh** | Interior mesh geometries | Specialized triangle mesh variant |
| **ConvexHull** | Convex hull generation | Utility for mesh processing |
| **GeomTools** | Geometric algorithms | Utility functions for meshes |

### Material and Texture System

- **MaterialID**: Integration with physics material properties
- **Texture coordinates**: UV mapping for rendering
- **Normal mapping**: Support for both face and vertex normals

## Implementation Considerations

### Memory Management

- **Caching systems**: Minimize redundant calculations for rendering
- **Data locality**: Optimized vertex and index storage
- **Reference counting**: Efficient memory management through handle system

### Thread Safety

- **Const correctness**: Careful const/non-const handle distinction
- **Immutable operations**: Safe concurrent read operations
- **Modification synchronization**: Write operations require proper locking

### Extensibility

- **Template trait system**: Easy collision response algorithm customization
- **Virtual interface**: Clean polymorphic behavior
- **Factory pattern**: Consistent object creation interface

## Conclusion

The TriangleMesh class hierarchy in the PE physics engine demonstrates a sophisticated design that balances:

1. **Performance**: Multiple optimization strategies and algorithm selection
2. **Flexibility**: Template-based customization and virtual interfaces  
3. **Integration**: Seamless interaction with collision detection, visualization, and parallel processing
4. **Robustness**: Comprehensive geometric operations and material support

The multi-level inheritance structure with trait-based customization allows the physics engine to adapt triangle mesh behavior for different collision response algorithms while maintaining a clean, polymorphic interface for user code.