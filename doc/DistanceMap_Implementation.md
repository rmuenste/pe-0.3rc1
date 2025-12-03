# DistanceMap-Based Mesh-to-Mesh Collision Detection

## Overview

The DistanceMap collision detection system provides efficient mesh-to-mesh collision detection using 3D signed distance fields (SDF). This approach accelerates collision queries between triangle meshes by precomputing a regular grid of signed distances, enabling fast interpolated queries during simulation.

### Key Features

- **Preprocessing-based**: Builds SDF grid once per mesh, enabling fast runtime queries
- **CGAL Integration**: Leverages CGAL's robust geometric algorithms for mesh processing
- **Coordinate Transformation**: Handles arbitrary mesh positions and orientations
- **Multi-Contact Manifolds**: Generates stable contact manifolds with clustering and extremal point selection
- **Non-Convex Mesh Support**: No fallback to GJK/EPA for meshes with DistanceMap (designed for non-convex geometry)
- **PE Integration**: Fully integrated into PE's collision detection pipeline

### Motivation

Traditional mesh-to-mesh collision detection using GJK/EPA can be computationally expensive for complex meshes. DistanceMap acceleration provides:

1. **O(1) Distance Queries**: Constant-time distance lookups via interpolation
2. **Smooth Gradients**: Continuous normals from SDF gradients
3. **Robust Inside/Outside**: CGAL-based spatial classification
4. **Precomputed Optimization**: Front-loads computation during mesh creation

## DistanceMap Generation Process

### Overview

The DistanceMap generation creates a 3D regular grid around a triangle mesh, computing signed distances at each grid point using CGAL's geometric algorithms.

### Implementation Location
- **Header**: `pe/core/detection/fine/DistanceMap.h`
- **Implementation**: `src/core/detection/fine/DistanceMap.cpp:47-150`

### Algorithm Steps

#### 1. Mesh Processing with CGAL

```cpp
// CGAL types used in DistanceMap::Impl
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Surface_mesh = CGAL::Surface_mesh<Point>;
using Tree = CGAL::AABB_tree<AABB_traits>;
using Point_inside = CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>;
```

The system uses CGAL's AABB tree for efficient distance queries and spatial classification.

#### 2. Grid Setup and Bounding Box Calculation

```cpp
// Compute expanded bounding box
CGAL::Bbox_3 bbox = tree.bbox();
pe::real h = std::max({dx, dy, dz}) / static_cast<pe::real>(resolution);

// Grid dimensions with tolerance padding
nx_ = static_cast<int>(std::ceil(expanded_dx / h)) + 1;
ny_ = static_cast<int>(std::ceil(expanded_dy / h)) + 1;  
nz_ = static_cast<int>(std::ceil(expanded_dz / h)) + 1;

// Origin shifted by tolerance
origin_.set(bbox.xmin() - tolerance * h, 
           bbox.ymin() - tolerance * h, 
           bbox.zmin() - tolerance * h);
```

**Parameters:**
- `spacing`: Target grid cell size
- `resolution`: Number of cells along largest dimension  
- `tolerance`: Number of padding cells around mesh

#### 3. Signed Distance Field Computation

For each grid point `(i,j,k)`:

```cpp
Point query(x, y, z);
pe::real unsigned_distance = std::sqrt(tree.squared_distance(query));

// Inside/outside classification using CGAL
CGAL::Bounded_side side = is_inside(query);
pe::real signed_distance = (side == CGAL::ON_UNBOUNDED_SIDE) ? 
                          unsigned_distance : -unsigned_distance;

sdf_[index] = signed_distance;
```

**Sign Convention:**
- **Positive**: Outside the mesh (free space)
- **Negative**: Inside the mesh (occupied space)
- **Zero**: On the mesh surface

#### 4. Gradient and Normal Computation

Surface normals are computed from SDF gradients using finite differences:

```cpp
// Gradient computation for normals (conceptual)
Vec3 gradient = computeGradient(i, j, k);
Vec3 normal = gradient.getNormalized();
normals_[index] = normal;
```

#### 5. Contact Point Calculation

Contact points are determined by projecting query points onto the nearest surface:

```cpp
Point closest = tree.closest_point(query);
contact_points_[index] = Vec3(closest.x(), closest.y(), closest.z());
```

### Memory Layout

The DistanceMap stores data in linear arrays with 3D indexing:

```cpp
int index = i + nx_ * j + nx_ * ny_ * k;

std::vector<pe::real> sdf_;           // Signed distances
std::vector<pe::real> alpha_;         // Alpha values for interpolation
std::vector<Vec3> normals_;           // Surface normals
std::vector<Vec3> contact_points_;    // Nearest surface points
```

## Collision Detection Pipeline

### Integration Point

DistanceMap collision detection is integrated into PE's fine collision detection system via `MaxContacts::collideTMeshTMesh()`.

**Location**: `pe/core/detection/fine/MaxContacts.h:3744-3775`

### Pipeline Overview

```
Broad Phase Detection → Fine Collision Detection → Contact Generation
                           ↓
                    DistanceMap Available?
                           ↓
                    [Yes] → DistanceMap Path (no fallback for non-convex meshes)
                           ↓
                    [No]  → GJK/EPA (for convex or meshes without DistanceMap)
```

**Important**: When a DistanceMap is present, the system does NOT fall back to GJK/EPA, as meshes using DistanceMap are typically non-convex and unsuitable for GJK/EPA.

### DistanceMap Detection Algorithm

**Location**: `pe/core/detection/fine/MaxContacts.h:3877-3975`

#### Step 1: DistanceMap Selection

```cpp
template< typename CC >
bool MaxContacts::collideWithDistanceMap( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
    const DistanceMap* distMap = nullptr;
    TriangleMeshID queryMesh = nullptr;
    TriangleMeshID referenceMesh = nullptr;
    
    if (mA->hasDistanceMap()) {
        distMap = mA->getDistanceMap();
        referenceMesh = mA;    // Has the DistanceMap
        queryMesh = mB;        // Queries against DistanceMap
    } else if (mB->hasDistanceMap()) {
        distMap = mB->getDistanceMap();
        referenceMesh = mB;
        queryMesh = mA;
    }
}
```

The algorithm prioritizes meshes with available DistanceMaps. Only one mesh needs a DistanceMap for collision detection to proceed.

#### Step 2: Coordinate System Transformations

**Critical Implementation Detail**: DistanceMaps are defined in the reference mesh's local coordinate system, but collision detection queries come from world coordinates.

```cpp
// Transform query vertex from world to reference mesh local coordinates
const Vec3& vertex = queryVertices[i];
Vec3 localVertex = referenceMesh->pointFromWFtoBF(vertex);

// Query DistanceMap in local coordinates
pe::real distance = distMap->interpolateDistance(localVertex[0], localVertex[1], localVertex[2]);
Vec3 localNormal = distMap->interpolateNormal(localVertex[0], localVertex[1], localVertex[2]);
Vec3 localContactPoint = distMap->interpolateContactPoint(localVertex[0], localVertex[1], localVertex[2]);
```

**PE Coordinate Transformation Methods**:
- `pointFromWFtoBF()`: World Frame → Body Frame (includes translation + rotation)
- `pointFromBFtoWF()`: Body Frame → World Frame (includes translation + rotation)  
- `vectorFromWFtoBF()`: World vectors → Body vectors (rotation only)
- `vectorFromBFtoWF()`: Body vectors → World vectors (rotation only)

#### Step 3: Distance Interpolation

The DistanceMap uses trilinear interpolation for smooth distance queries at arbitrary points:

```cpp
pe::real DistanceMap::interpolateDistance(pe::real x, pe::real y, pe::real z) const
{
    // Convert world coordinates to grid indices
    pe::real fi = (x - origin_[0]) / spacing_;
    pe::real fj = (y - origin_[1]) / spacing_;  
    pe::real fk = (z - origin_[2]) / spacing_;
    
    // Trilinear interpolation between 8 neighboring grid points
    return trilinearInterpolate(fi, fj, fk, sdf_);
}
```

#### Step 4: Contact Classification and Generation

```cpp
// Check for contact/penetration using contact threshold
if (distance < contactThreshold) {
    hasContact = true;
    
    if (distance < minDistance) {
        minDistance = distance;
        
        // Transform results back to world coordinates for collision response
        Vec3 worldNormal = referenceMesh->vectorFromBFtoWF(localNormal);
        Vec3 worldContactPoint = referenceMesh->pointFromBFtoWF(localContactPoint);
        
        // Store deepest contact information
        minPenetrationNormal = worldNormal;
        deepestContactPoint = worldContactPoint;
    }
}
```

#### Step 5: Contact Creation

```cpp
// Create contact if penetration was found
if (hasContact && minDistance < contactThreshold) {
    // Ensure proper normal direction (pointing from reference to query mesh)
    if (referenceMesh == mB) {
        minPenetrationNormal = -minPenetrationNormal;
    }
    
    contacts.addVertexFaceContact(queryMesh, referenceMesh, 
                                 deepestContactPoint, minPenetrationNormal, minDistance);
}
```

### Vertex Sampling Strategy

**Current Implementation**: Complete vertex enumeration (no sampling)

```cpp
// Use all vertices for comprehensive testing (sampling disabled)
size_t sampleStep = 1; // Process every vertex (no sampling)

for (size_t i = 0; i < queryVertices.size(); i += sampleStep) {
    // Process vertex[i]
}
```

**Performance Note**: Sampling was disabled to ensure comprehensive collision detection for small test meshes. For production use with large meshes, sampling can be re-enabled:

```cpp
// Re-enable sampling for performance if needed:
size_t sampleSize = 100UL; // Sample up to 100 points
size_t sampleStep = std::max(size_t(1), queryVertices.size() / sampleSize);
```

### GJK/EPA for Non-DistanceMap Meshes

When DistanceMap is NOT available, the system uses traditional GJK/EPA:

```cpp
void MaxContacts::collideTMeshTMesh( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
    // Priority-based collision detection
    // Try DistanceMap-based collision detection first (if available)
    if (mA->hasDistanceMap() || mB->hasDistanceMap()) {
        collideWithDistanceMap(mA, mB, contacts);
        // If a DistanceMap is present, do NOT fall back to GJK/EPA since meshes are non-convex
        return;
    }

    // GJK/EPA for convex meshes or meshes without DistanceMap
    Vec3 normal, contactPoint;
    pe::real penetrationDepth;

    if(gjkEPAcollideHybrid<TriangleMeshID, TriangleMeshID>(mA, mB, normal, contactPoint, penetrationDepth)) {
        contacts.addVertexFaceContact(mA, mB, contactPoint, normal, penetrationDepth);
    }
}
```

**Critical Design Decision**: The system explicitly does NOT fall back from DistanceMap to GJK/EPA when a DistanceMap is present, as meshes using DistanceMap are typically non-convex and unsuitable for GJK/EPA collision detection.

## Multi-Contact Manifold Generation

### Current Implementation

The DistanceMap collision detection system implements a **multi-contact manifold approach** that generates stable contact sets with intelligent clustering and representative selection.

**Location**: `pe/core/detection/fine/MaxContacts.h` (lines ~4038-4320)

### Algorithm Overview

The contact generation pipeline consists of four main phases:

1. **Candidate Collection**: Sample multiple geometric features (vertices, edges, faces)
2. **Contact Clustering**: Group nearby contacts with similar normals
3. **Representative Selection**: Choose deepest + extremal contacts per cluster
4. **Contact Limiting**: Cap total contacts at 6 per mesh pair for performance

### Phase 1: Comprehensive Candidate Sampling

The system samples three types of geometric features to ensure comprehensive collision detection:

```cpp
struct ContactCandidate {
    Vec3 worldPos;        // Query point in world coordinates
    Vec3 worldNormal;     // Contact normal in world coordinates
    real penetration;     // Penetration depth (positive = penetrating)
    Vec3 contactPoint;    // Surface contact point in world coordinates
};

std::vector<ContactCandidate> candidates;
```

#### Vertex Sampling
All query mesh vertices are tested against the reference mesh's DistanceMap:

```cpp
for (size_t i = 0; i < queryVertices.size(); ++i) {
    processQueryPoint(queryVertices[i]);
}
```

#### Edge Midpoint Sampling
Edge midpoints are sampled to detect edge-face contacts:

```cpp
for (const auto& face : queryFaceIndices) {
    for (size_t edgeIdx = 0; edgeIdx < face.size(); ++edgeIdx) {
        Vec3 edgeMidpoint = (queryVertices[face[edgeIdx]] +
                            queryVertices[face[(edgeIdx + 1) % face.size()]]) * 0.5;
        processQueryPoint(edgeMidpoint);
    }
}
```

#### Triangle Barycenter Sampling
Face barycenters are sampled to detect face-face contacts (critical for stability):

```cpp
for (const auto& face : queryFaceIndices) {
    Vec3 barycenter(0.0, 0.0, 0.0);
    for (size_t idx : face) {
        barycenter += queryVertices[idx];
    }
    barycenter /= static_cast<real>(face.size());
    processQueryPoint(barycenter);
}
```

#### Debug Validation (Debug Builds Only)

In debug builds, an expensive exact signed distance validation filters out false positives:

```cpp
pe_LOG_DEBUG_SECTION( log ) {
    // Validate against exact signed distance to avoid false negatives from coarse SDF interpolation
    const real validationSlack = std::max(distMap->getSpacing() * real(0.05), contactThreshold);
    real exactSignedDistance = referenceMesh->signedDistance(localPoint);
    if( exactSignedDistance > -validationSlack ) {
        return; // Skip spurious penetration reports
    }
}
```

**Note**: This validation only runs when debug logging is enabled, avoiding performance impact in production builds.

### Phase 2: Contact Clustering

Candidates are clustered by proximity and normal similarity to avoid redundant contacts:

```cpp
struct ContactCluster {
    std::vector<size_t> candidateIndices;
    Vec3 averageNormal;
    real maxPenetration;
};

const real clusteringRadius = 2.0 * distMap->getSpacing();
const real normalSimilarityThreshold = 0.9; // Dot product threshold
```

Clustering algorithm:
- Proximity-based: candidates within `clusteringRadius` are grouped
- Normal similarity: candidates with similar normals (dot > 0.9) are grouped
- Average normal computed per cluster for stability

### Phase 3: Representative Selection

Each cluster contributes up to 4 representative contacts:

1. **Deepest penetration contact**: Maximum penetration in the cluster
2. **Extremal contacts**: Up to 3 additional contacts in orthogonal tangent directions

```cpp
// Construct orthonormal basis in tangent plane
Vec3 tangent1, tangent2;
constructOrthonormalBasis(clusterNormal, tangent1, tangent2);

// Find extremal contacts projected onto tangent directions
for (each tangent direction) {
    find candidate with max projection onto tangent
    add to representatives if distinct from deepest
}
```

This ensures contacts are spatially distributed for proper torque generation.

### Phase 4: Contact Limiting

Total contacts are capped at 6 per mesh pair to balance stability with performance:

```cpp
const size_t maxContactsPerPair = 6;
```

### Benefits of Multi-Contact Manifolds

1. **Improved Stability**: Multiple contacts prevent rocking and provide stable support
2. **Better Force Distribution**: Contact forces distributed across contact area
3. **Proper Torque Generation**: Spatially separated contacts generate realistic torques
4. **Reduced Jittering**: Clustering provides temporal coherence
5. **Comprehensive Detection**: Vertex + edge + face sampling catches all contact types

### Performance Characteristics

- **Time Complexity**: O(v + e + f) where v=vertices, e=edges, f=faces
- **Memory**: ~32 bytes per candidate (typically 10-50 candidates per collision)
- **Contact Count**: Typically 1-6 contacts per mesh pair (capped at 6)
- **Clustering Cost**: O(n²) for small candidate sets (negligible in practice)

## Implementation Files and Integration

### Core Implementation Files

#### 1. DistanceMap Header
**File**: `pe/core/detection/fine/DistanceMap.h`

```cpp
class DistanceMap {
public:
    // Factory methods
    static std::unique_ptr<DistanceMap> createFromFile(
        const std::string& meshFile, pe::real spacing, int resolution, int tolerance);
    static std::unique_ptr<DistanceMap> createFromTriangleMesh(
        const TriangleMesh& mesh, pe::real spacing, int resolution, int tolerance);
    
    // Query interface
    pe::real interpolateDistance(pe::real x, pe::real y, pe::real z) const;
    Vec3 interpolateNormal(pe::real x, pe::real y, pe::real z) const;
    Vec3 interpolateContactPoint(pe::real x, pe::real y, pe::real z) const;
    
    // Grid properties
    int getNx() const { return pImpl_->nx_; }
    int getNy() const { return pImpl_->ny_; }
    int getNz() const { return pImpl_->nz_; }
    pe::real getSpacing() const { return pImpl_->spacing_; }
    const Vec3& getOrigin() const { return pImpl_->origin_; }
};
```

#### 2. DistanceMap Implementation  
**File**: `src/core/detection/fine/DistanceMap.cpp`

Key components:
- PIMPL pattern for CGAL dependency isolation
- CGAL mesh processing and AABB tree construction
- SDF grid computation with inside/outside classification
- Trilinear interpolation for distance queries

#### 3. MaxContacts Integration
**File**: `pe/core/detection/fine/MaxContacts.h`

**Triangle Mesh Collision Entry Point**: Lines 3744-3775
```cpp
template< typename CC >
void MaxContacts::collideTMeshTMesh( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
    // Try DistanceMap-based collision detection first
    if (mA->hasDistanceMap() || mB->hasDistanceMap()) {
        if (collideWithDistanceMap(mA, mB, contacts)) {
            return; // DistanceMap collision successful
        }
    }
    
    // GJK/EPA fallback
    // ... fallback implementation
}
```

**DistanceMap Collision Algorithm**: Lines 3877-3975
```cpp  
template< typename CC >
bool MaxContacts::collideWithDistanceMap( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
    // Complete implementation with coordinate transformations
    // Vertex enumeration and distance queries
    // Contact generation and world coordinate conversion
}
```

### TriangleMesh Integration

**File**: `pe/core/rigidbody/TriangleMesh.h`

DistanceMap integration in TriangleMesh class:

```cpp
class TriangleMesh : public TriangleMeshTrait<Config>
{
public:
    // DistanceMap acceleration
    void enableDistanceMapAcceleration(pe::real spacing, int resolution = 50, int tolerance = 5);
    bool hasDistanceMap() const;
    const DistanceMap* getDistanceMap() const;
    
private:
    std::unique_ptr<DistanceMap> distanceMap_;
};
```

### Build System Integration

**File**: `examples/cgal_examples/CMakeLists.txt`

```cmake
# CGAL examples with DistanceMap
add_executable(mesh_simulation mesh_simulation.cpp)
add_executable(mesh_collision_test mesh_collision_test.cpp VtkOutput.cpp)

# CGAL dependencies
find_package(GMP REQUIRED)
target_link_libraries(mesh_simulation PRIVATE ${GMP_LIBRARIES})
target_include_directories(mesh_simulation PRIVATE ${CGAL_INCLUDE_DIR})
```

**Conditional Compilation**: Code protected by `#ifdef PE_USE_CGAL`

### Usage Examples

#### 1. Basic Usage
**File**: `examples/cgal_examples/mesh_simulation.cpp`

```cpp
// Load triangle mesh with DistanceMap acceleration
TriangleMeshID mesh = createTriangleMesh(id, position, "mesh.obj", material, false, true);
mesh->enableDistanceMapAcceleration(0.1, 30, 3);  // spacing, resolution, tolerance

// DistanceMap automatically used in collision detection
world->simulationStep(timestep);
```

#### 2. Collision Testing
**File**: `examples/cgal_examples/mesh_collision_test.cpp`

```cpp
// Test DistanceMap collision detection
TriangleMeshID referenceMesh = createTriangleMesh(1, Vec3(0,0,0), "reference.obj", material1);
TriangleMeshID testMesh = createTriangleMesh(2, Vec3(0.5,0.5,0.5), "test.obj", material2);

referenceMesh->enableDistanceMapAcceleration(0.1, 50, 5);

// Direct collision testing
ContactVector contacts;
MaxContacts::collideTMeshTMesh(referenceMesh, testMesh, contacts);
```

### Configuration Parameters

#### DistanceMap Parameters

- **spacing**: Grid cell size (controls accuracy vs. memory)
- **resolution**: Cells along largest dimension (typically 30-100)  
- **tolerance**: Padding cells around mesh (typically 3-10)

#### Performance Tuning

```cpp
// High accuracy (large memory usage)
mesh->enableDistanceMapAcceleration(0.05, 100, 10);

// Balanced (recommended)
mesh->enableDistanceMapAcceleration(0.1, 50, 5);

// Fast (lower accuracy)  
mesh->enableDistanceMapAcceleration(0.2, 30, 3);
```

## Performance Characteristics

### Time Complexity

- **DistanceMap Creation**: O(n × log(m)) where n = grid points, m = mesh triangles
- **Single Distance Query**: O(1) via trilinear interpolation
- **Collision Detection**: O(v) where v = query mesh vertices  
- **Coordinate Transformation**: O(1) per vertex

### Memory Usage

Grid memory scales as:
```
Memory = nx × ny × nz × (sizeof(sdf_) + sizeof(alpha_) + sizeof(normals_) + sizeof(contact_points_))
       = nx × ny × nz × (8 + 8 + 24 + 24) bytes
       ≈ nx × ny × nz × 64 bytes (for double precision)
```

**Storage per voxel**:
- `sdf_`: 8 bytes (double)
- `alpha_`: 8 bytes (double, interpolation weights)
- `normals_`: 24 bytes (3 doubles)
- `contact_points_`: 24 bytes (3 doubles)

**Example**: 50³ grid = 125,000 voxels × 64 bytes ≈ 8MB per DistanceMap

### Performance Comparison

| Method | Preprocessing | Query Time | Memory | Accuracy |
|--------|--------------|-----------|---------|----------|
| GJK/EPA | None | O(k) iterations | Minimal | High |
| DistanceMap | O(n×log(m)) | O(1) | O(n) grid | High |
| Hybrid | O(n×log(m)) | O(1) + fallback | O(n) | High |

### Scalability Considerations

1. **Mesh Complexity**: DistanceMap performance independent of triangle count
2. **Grid Resolution**: Memory/accuracy trade-off via resolution parameter
3. **Multiple Meshes**: Each mesh can have independent DistanceMap
4. **Cache Efficiency**: Grid structure provides good spatial locality

## Conclusion

The DistanceMap mesh-to-mesh collision detection system provides an efficient acceleration structure for collision detection between triangle meshes. While the current single contact point approach trades some physical accuracy for computational efficiency, it offers robust and deterministic collision detection suitable for many physics simulation scenarios.

The system's integration with PE's collision detection pipeline provides seamless fallback to GJK/EPA methods, ensuring compatibility across all mesh configurations while providing acceleration when DistanceMaps are available.

Future enhancements could include multiple contact generation, contact manifold construction, and adaptive sampling strategies to improve physical accuracy while maintaining the performance benefits of the DistanceMap approach.