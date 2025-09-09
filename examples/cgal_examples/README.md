# CGAL Examples for PE Physics Engine

This directory contains examples demonstrating CGAL integration and DistanceMap acceleration features in the PE physics engine.

## Examples

### mesh_containspoint_test
Tests the optimized `containsPoint()` functionality with DistanceMap acceleration.

**Purpose**: 
- Validates the correctness of the DistanceMap-optimized containsPoint method
- Compares performance between DistanceMap (O(1)) and CGAL ray shooting (O(k*log n))
- Generates comprehensive test statistics

**Usage**:
```bash
./mesh_containspoint_test <mesh_file.obj> [options]

Options:
  --spacing <value>     DistanceMap grid spacing (default: 0.05)
  --resolution <value>  DistanceMap resolution (default: 64) 
  --tolerance <value>   DistanceMap tolerance (default: 3)
  --points <count>      Number of test points (default: 1000)
  --verbose             Enable verbose output for disagreements
```

**Example runs**:
```bash
# Basic test with cube mesh
./mesh_containspoint_test ../kdop_example/cube.obj

# High-precision test with many points
./mesh_containspoint_test ../kdop_example/cube.obj --spacing 0.01 --resolution 128 --points 5000

# Verbose test to see disagreements
./mesh_containspoint_test ../kdop_example/dodecahedron.obj --verbose
```

**Expected output**:
- Loading and DistanceMap generation statistics
- Containment test results comparing both methods
- Performance comparison showing speed improvement
- Validation result (PASS/WARN/FAIL)

### mesh_collision_test
Tests DistanceMap-accelerated collision detection between triangle meshes.

### mesh_simulation
Complete physics simulation demonstrating mesh-mesh collision with DistanceMap acceleration.

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

## Performance Notes

The `mesh_containspoint_test` typically shows:
- **DistanceMap method**: ~0.001-0.01 ms per query (O(1))
- **CGAL ray shooting**: ~0.1-1.0 ms per query (O(k*log n))
- **Speed improvement**: 10x-100x faster with DistanceMap

Performance depends on:
- Mesh complexity (number of triangles)
- DistanceMap resolution and spacing
- Query point distribution