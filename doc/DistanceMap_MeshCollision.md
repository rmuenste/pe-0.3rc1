# DistanceMap-Based Mesh-to-Mesh Collision Detection

## Overview

The DistanceMap collision detection system provides efficient mesh-to-mesh collision detection using 3D signed distance fields (SDF). This approach accelerates collision queries between triangle meshes by precomputing a regular grid of signed distances, enabling fast interpolated queries during simulation.

### Key Features

- **Preprocessing-based**: Builds SDF grid once per mesh, enabling fast runtime queries
- **CGAL Integration**: Leverages CGAL's robust geometric algorithms for mesh processing
- **Coordinate Transformation**: Handles arbitrary mesh positions and orientations
- **Fallback System**: Seamlessly falls back to GJK/EPA when DistanceMap unavailable
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
                    [Yes] → DistanceMap Path
                           ↓
                    [No]  → GJK/EPA Fallback
```

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

### GJK/EPA Fallback

When DistanceMap is unavailable, the system falls back to traditional GJK/EPA:

```cpp
void MaxContacts::collideTMeshTMesh( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
    // Try DistanceMap-based collision detection first
    if (mA->hasDistanceMap() || mB->hasDistanceMap()) {
        if (collideWithDistanceMap(mA, mB, contacts)) {
            return; // DistanceMap collision successful
        }
    }

    // GJK/EPA fallback
    Vec3 normal, contactPoint;
    pe::real penetrationDepth;
    
    if(gjkEPAcollideHybrid<TriangleMeshID, TriangleMeshID>(mA, mB, normal, contactPoint, penetrationDepth)) {
        contacts.addVertexFaceContact(mA, mB, contactPoint, normal, penetrationDepth);
    }
}
```

## Contact Generation and Single Contact Point Approach

### Current Implementation

The DistanceMap collision detection system generates **exactly one contact point** per mesh pair - the deepest penetration contact.

**Location**: `pe/core/detection/fine/MaxContacts.h:3930-3946`

```cpp
// Find the deepest penetrating vertex
if (distance < minDistance) {
    minDistance = distance;
    
    // Store the deepest contact information (in world coordinates)
    minPenetrationNormal = worldNormal;
    deepestContactPoint = worldContactPoint;
}
```

### Single Contact Point Analysis

#### Advantages (Pros)

1. **Computational Efficiency**
   - **O(n) complexity**: Single pass through query vertices
   - **Minimal memory usage**: Stores only one contact per collision
   - **Fast contact resolution**: Constraint solver handles fewer contacts
   - **Deterministic results**: Always selects the same deepest point

2. **Numerical Stability**
   - **No contact clustering**: Avoids redundant nearby contacts
   - **Consistent contact normal**: Single well-defined contact direction
   - **Reduced solver complexity**: Fewer constraint equations to solve
   - **Robust convergence**: Simpler contact manifold topology

3. **Implementation Simplicity**
   - **Straightforward algorithm**: Easy to understand and debug  
   - **Minimal configuration**: No complex contact merging parameters
   - **Clear semantics**: Unambiguous contact semantics
   - **Predictable behavior**: Consistent across different mesh configurations

#### Disadvantages (Cons)

1. **Limited Contact Information**
   - **Missing contact manifolds**: Large contact areas represented by single point
   - **Incomplete force distribution**: Contact forces concentrated at single point
   - **Reduced stability**: May cause unrealistic contact behavior for large contacts
   - **Loss of contact details**: Complex contact geometry simplified away

2. **Physical Accuracy Limitations**
   - **Unrealistic contact mechanics**: Real contact involves contact patches, not points
   - **Moment calculations**: Missing torques from extended contact areas
   - **Pressure distribution**: Cannot represent distributed contact pressure
   - **Contact area effects**: Friction and normal forces should scale with contact area

3. **Simulation Quality Issues**
   - **Contact jittering**: Single contact point may "jump" between nearby vertices
   - **Insufficient constraint**: One contact may under-constrain relative motion
   - **Penetration artifacts**: Deep penetrations may not be fully resolved
   - **Stability problems**: Contact may be insufficient for stable stacking/resting

#### Alternative Approaches

1. **Multiple Contact Generation**

```cpp
// Collect all vertices within penetration threshold
std::vector<ContactInfo> contactCandidates;

for (size_t i = 0; i < queryVertices.size(); i += sampleStep) {
    if (distance < contactThreshold) {
        contactCandidates.push_back({vertex, distance, normal, contactPoint});
    }
}

// Generate multiple contacts (e.g., 4-point contact manifold)
generateContactManifold(contactCandidates, contacts);
```

**Pros**: More complete contact information, better force distribution
**Cons**: Increased computational cost, contact clustering issues

2. **Contact Manifold Construction**

```cpp
// Identify contact regions and construct manifold
ContactManifold manifold = identifyContactRegions(queryMesh, referenceMesh);
std::vector<ContactPoint> manifoldContacts = manifold.generateContacts();

for (const auto& contact : manifoldContacts) {
    contacts.addVertexFaceContact(queryMesh, referenceMesh, 
                                 contact.position, contact.normal, contact.depth);
}
```

**Pros**: Physically accurate contact representation, stable contact resolution
**Cons**: Complex implementation, higher computational overhead

3. **Adaptive Contact Sampling**

```cpp
// Adjust contact generation based on penetration depth and contact area
if (minDistance < -significantPenetrationThreshold) {
    // Generate multiple contacts for deep penetrations
    generateMultipleContacts(queryMesh, referenceMesh, contacts);
} else {
    // Single contact sufficient for shallow penetrations  
    generateSingleContact(queryMesh, referenceMesh, contacts);
}
```

**Pros**: Balances accuracy vs. performance, scales with contact complexity
**Cons**: Added complexity, parameter tuning required

4. **Contact Force Distribution**

```cpp
// Keep single geometric contact but distribute forces
Contact contact = generateSingleContact(queryMesh, referenceMesh);

// Scale contact force based on estimated contact area
pe::real contactArea = estimateContactArea(queryMesh, referenceMesh);
contact.setForceScale(contactArea);
```

**Pros**: Maintains simple contact structure, improves force accuracy
**Cons**: Contact area estimation complexity, approximate solution

### Recommended Improvements

1. **Short-term**: Implement contact area estimation to scale contact forces
2. **Medium-term**: Add adaptive contact generation based on penetration depth  
3. **Long-term**: Full contact manifold construction for high-accuracy simulations

### Performance vs. Accuracy Trade-offs

| Approach | Computation Cost | Memory Usage | Physical Accuracy | Implementation Complexity |
|----------|-----------------|--------------|-------------------|-------------------------|
| Single Contact | Low | Minimal | Limited | Simple |
| Multiple Contacts | Medium | Moderate | Good | Moderate |
| Contact Manifolds | High | High | Excellent | Complex |
| Adaptive Sampling | Variable | Variable | Good-Excellent | Complex |

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
Memory = nx × ny × nz × (4 × sizeof(pe::real) + 2 × sizeof(Vec3))
       ≈ nx × ny × nz × 40 bytes (for double precision)
```

**Example**: 50³ grid ≈ 5MB per DistanceMap

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