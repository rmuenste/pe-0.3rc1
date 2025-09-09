# CGAL Examples for PE Physics Engine

This directory contains examples demonstrating CGAL integration and DistanceMap acceleration features in the PE physics engine.

## Examples

### mesh_containspoint_test
Tests the optimized `containsPoint()` functionality with DistanceMap acceleration and optional CGAL comparison.

**Purpose**: 
- Validates the correctness of the DistanceMap-optimized containsPoint method
- Compares performance between DistanceMap (O(1)) and CGAL ray shooting (O(k*log n))
- Generates comprehensive test statistics
- Exports test points and results for visualization

**Usage**:
```bash
./mesh_containspoint_test <mesh_file.obj> [options]

DistanceMap Options:
  --spacing <value>       DistanceMap grid spacing (default: 0.05)
  --resolution <value>    DistanceMap resolution (default: 64) 
  --tolerance <value>     DistanceMap tolerance (default: 3)

Test Options:
  --points <count>        Number of test points (default: 1000)
  --no-cgal-compare       Disable CGAL comparison (DistanceMap-only testing)
  --verbose               Enable verbose output for disagreements

Export Options:
  --export-vtk [filename]     Export DistanceMap to VTK format (default: distance_map.vti)
  --export-points [filename]  Export test points with results to VTK (default: test_points.vtk)
```

**Example runs**:
```bash
# Basic test with cube mesh
./mesh_containspoint_test ../kdop_example/cube.obj

# High-precision test with visualization
./mesh_containspoint_test ../kdop_example/cube.obj --spacing 0.01 --resolution 128 --points 5000 \
  --export-vtk high_res_distmap --export-points cube_test_points

# Fast DistanceMap-only test (no CGAL comparison)
./mesh_containspoint_test ../kdop_example/dodecahedron.obj --no-cgal-compare --points 10000

# Full analysis with verbose disagreement reporting
./mesh_containspoint_test complex_mesh.obj --verbose --export-vtk --export-points
```

**Output Files**:
- `distance_map.vti`: DistanceMap visualization (when --export-vtk used)
- `test_points.vtk`: Point cloud with containment results (when --export-points used)
  - Fields: DistanceMapResult, CGALResult, Disagreement (when CGAL enabled)

**Expected output**:
- Loading and DistanceMap generation statistics
- Containment test results comparing both methods (when CGAL enabled)
- Performance comparison showing speed improvement
- Validation result (PASS/WARN/FAIL) with disagreement analysis

### mesh_collision_test
Tests DistanceMap-accelerated collision detection between triangle meshes.

### mesh_simulation
Complete physics simulation demonstrating mesh-mesh collision with DistanceMap acceleration.

### mesh_grid_test
Performs structured grid analysis with DistanceMap acceleration for scientific computing applications.

**Purpose**:
- Tests point containment on regular 3D grids (ideal for CFD, physics simulations)
- Provides extensible framework for multiple field calculations
- Supports mesh translation for coordinate transformation testing
- Exports VTK structured grids for visualization in ParaView/VisIt

**Key Features**:
- Structured grid generation with user-specified origin, spacing, and dimensions
- Extensible calculation framework (containment, signed distance, normals, contact points)
- Mesh positioning control for testing coordinate transformations
- Dual VTK export: test results grid + DistanceMap visualization
- Performance monitoring with per-calculation timing

**Usage**:
```bash
./mesh_grid_test <mesh_file.obj> [options]

Mesh Options:
  --mesh-position <x> <y> <z>   : Mesh position in world coordinates (default: 0 0 0)

Grid Options:
  --grid-origin <x> <y> <z>     : Grid origin coordinates (default: auto-center)
  --grid-spacing <dx> <dy> <dz> : Grid spacing per axis (default: 0.1 0.1 0.1)
  --grid-size <nx> <ny> <nz>    : Grid dimensions (default: 50 50 50)

DistanceMap Options:
  --dm-spacing <value>          : DistanceMap spacing (default: 0.05)
  --dm-resolution <value>       : DistanceMap resolution (default: 64)
  --dm-tolerance <value>        : DistanceMap tolerance (default: 3)

Calculation Options:
  --calc-distance               : Calculate signed distance values
  --calc-normal                 : Calculate surface normals
  --calc-contact                : Calculate contact points

Output Options:
  --output <filename>           : Output VTK filename (default: grid_results.vti)
  --export-distance-map [file]  : Export DistanceMap grid to VTK (default: distance_map.vti)
  --verbose                     : Enable verbose output
```

**Example runs**:
```bash
# Basic containment analysis
./mesh_grid_test chip1.obj

# High-resolution analysis with all fields
./mesh_grid_test sphere.obj \
  --grid-spacing 0.02 0.02 0.02 --grid-size 200 200 200 \
  --calc-distance --calc-normal --calc-contact \
  --output detailed_sphere_analysis

# Coordinate transformation testing
./mesh_grid_test mesh.obj \
  --mesh-position 5.0 0.0 2.0 \
  --grid-origin 0 0 0 \
  --grid-size 100 100 100 \
  --export-distance-map local_coords
```

**Output Files**:
- `grid_results.vti`: Test grid results in world coordinates
- `distance_map.vti`: DistanceMap in local mesh coordinates (when --export-distance-map used)

### debug_coordinate_transform
Systematic debugging tool for coordinate transformation validation.

**Purpose**:
- Validates coordinate transformations when meshes are translated
- Tests specific known points from ParaView analysis
- Provides detailed debugging output for transformation issues
- Confirms DistanceMap queries work correctly in translated coordinate systems

**Usage**:
```bash
./debug_coordinate_transform <mesh_file.obj>
```

**Test Methodology**:
1. **Phase 1**: Mesh at origin, test point at known location → Expected: INSIDE
2. **Phase 2**: Mesh at origin, test point translated +10Z → Expected: OUTSIDE  
3. **Phase 3**: Mesh translated +10Z, test point translated +10Z → Expected: INSIDE
4. **Phase 4**: Mesh translated +10Z, original test point → Expected: OUTSIDE

**Debug Output**:
- Mesh and test point positions
- DistanceMap containment results
- Local coordinate transformations
- Signed distance values
- Bounding box validation
- Point-in-bounds checking

**Example output**:
```
=== Test 1: Point vs Mesh at Origin (Expected: INSIDE) ===
Mesh position: (0, 0, 0)
Test point:    (-0.086035, -0.869566, 0.477526)
DistanceMap result: INSIDE
Local point:   (-0.086035, -0.869566, 0.477526)
Signed distance: -0.123456
Distance sign indicates: INSIDE
Point in DM bounds: YES
Point in AABB: YES
```

### cgal_box
Basic CGAL integration example.

## Requirements

- PE compiled with CGAL support (`-DCGAL=ON`)
- Examples enabled (`-DEXAMPLES=ON`) 
- Input mesh files in .obj or .off format

## Test Meshes

You can use the existing test meshes:
- `../kdop_example/cube.obj` - Simple cube mesh
- `../kdop_example/dodecahedron.obj` - More complex polyhedron
- `../trimeshdop_demo/cube.obj` - Alternative cube mesh

## Recommended Workflows

### 1. Basic DistanceMap Validation
```bash
# Step 1: Validate DistanceMap accuracy
./mesh_containspoint_test your_mesh.obj --verbose --export-vtk --export-points

# Step 2: Check results in ParaView
# - Load distance_map.vti to see DistanceMap structure
# - Load test_points.vtk to see containment results
# - Check for disagreements between methods
```

### 2. Coordinate Transformation Testing  
```bash
# Step 1: Debug coordinate transforms
./debug_coordinate_transform your_mesh.obj

# Step 2: Test specific translation scenarios
./mesh_grid_test your_mesh.obj \
  --mesh-position 5.0 2.0 -1.0 \
  --grid-origin 0 0 0 \
  --grid-size 100 100 100 \
  --export-distance-map local_view \
  --calc-distance

# Step 3: Verify in ParaView
# - local_view.vti shows mesh at origin (DistanceMap coordinates)
# - grid_results.vti shows mesh translated (world coordinates)
```

### 3. High-Performance Production Analysis
```bash
# Step 1: Fast DistanceMap-only testing (no CGAL overhead)
./mesh_containspoint_test your_mesh.obj --no-cgal-compare --points 100000

# Step 2: Structured grid analysis for scientific applications
./mesh_grid_test your_mesh.obj \
  --grid-spacing 0.01 0.01 0.01 \
  --grid-size 500 500 500 \
  --calc-distance --calc-normal \
  --output high_res_analysis
```

### 4. Parameter Optimization
```bash
# Test different DistanceMap parameters for your mesh
./mesh_containspoint_test your_mesh.obj --spacing 0.02 --resolution 128 --tolerance 5
./mesh_containspoint_test your_mesh.obj --spacing 0.05 --resolution 64 --tolerance 3
./mesh_containspoint_test your_mesh.obj --spacing 0.10 --resolution 32 --tolerance 2
```

## Performance Notes

### Containment Testing Performance
- **DistanceMap method**: ~0.001-0.01 ms per query (O(1))
- **CGAL ray shooting**: ~0.1-1.0 ms per query (O(k*log n))
- **Speed improvement**: 10x-100x faster with DistanceMap

### Grid Analysis Performance  
- **Small grids** (50³): ~1-10 ms total
- **Medium grids** (100³): ~10-100 ms total
- **Large grids** (500³): ~1-10 seconds total

Performance depends on:
- Mesh complexity (number of triangles)
- DistanceMap resolution and spacing
- Query point distribution
- Number of enabled calculations (distance, normals, contact points)

### Memory Usage
- **DistanceMap**: ~8 bytes × nx × ny × nz per field (SDF, alpha, normals, contacts)
- **Typical 64³ DistanceMap**: ~64MB total
- **Large 256³ DistanceMap**: ~4GB total

## Troubleshooting

### Common Issues

**"DistanceMap acceleration failed to initialize"**
- Check mesh file format (.obj or .off)
- Verify mesh is watertight and has proper normals
- Try increasing tolerance parameter
- Reduce resolution for very large meshes

**"Point outside DistanceMap bounds"**
- Check coordinate transformations in debug output
- Verify mesh position vs grid placement
- Increase DistanceMap tolerance for boundary queries

**"High disagreement rate between methods"**
- Usually indicates mesh geometry issues (non-watertight, bad normals)
- Try different DistanceMap parameters (spacing, resolution)
- Use `--verbose` flag to see specific disagreement locations

**Performance Issues**
- Reduce DistanceMap resolution for faster initialization
- Use `--no-cgal-compare` for DistanceMap-only testing
- Reduce grid size for structured grid analysis
- Enable only needed calculations (distance, normals, contacts)