# Domain Decomposition Test Suite

This test application verifies the correctness of the domain decomposition functionality in the PE physics engine.

## Purpose

The test suite validates:

1. **Universal decomposeDomain Function**: Verifies that the single `decomposeDomain` function correctly handles all axis configurations:
   - Single-axis decompositions (e.g., 4x1x1, 1x4x1, 1x1x4)
   - Dual-axis decompositions (e.g., 2x2x1, 2x1x2, 1x2x2)  
   - Full 3D decompositions (e.g., 2x2x2, 3x3x3)

2. **Local Domain Geometry**: Tests that the local domain boundaries are correctly calculated using HalfSpace intersections

3. **Point Containment**: Verifies that points known to be inside/outside the local domain are correctly classified

4. **MPI Integration**: Ensures proper integration with MPI Cartesian topology

## Building

The test is built as part of the examples when MPI support is enabled:

```bash
mkdir build && cd build
cmake -DMPI=ON -DEXAMPLES=ON ..
make domain_decomposition_test
```

## Running

The test application automatically detects the number of available MPI processes and runs appropriate test configurations:

```bash
# Run with different process counts to test various configurations
mpirun -np 4 ./examples/domain_decomposition_test/domain_decomposition_test
mpirun -np 8 ./examples/domain_decomposition_test/domain_decomposition_test
```

## Test Configurations

The application tests the following configurations based on available processes:

| Processes | Configuration | Description |
|-----------|---------------|-------------|
| 4 | 4x1x1, 2x2x1, 1x4x1, 1x1x4 | Single and dual-axis tests |
| 8 | 2x2x2, 2x4x1, 4x2x1, 8x1x1 | Various decomposition patterns |

## Output

For each test configuration, the application outputs:
- Process grid dimensions (px × py × pz)
- Local domain boundaries for each MPI process
- Point containment test results
- Overall PASS/FAIL status

Example output:
```
================================================================================
  TESTING: X-axis only (4x1x1)
================================================================================

Test Configuration: X-axis only (4x1x1)
Process Grid: 4x1x1
Domain Bounds: [0, 0, 0]
Cell Sizes: [0.25, 1, 1]
Current Process: 0 / 4

Rank 0 (coords: 0,0,0) Domain Test:
Domain bounds: [0,0.25] x [0,1] x [0,1]
Point 0: (0.125,0.5,0.5) -> INSIDE OK
Point 1: (1e-10,1e-10,1e-10) -> INSIDE OK
...
Point 5: (-1e-06,0.5,0.5) -> OUTSIDE OK

Test Result: PASSED
```

## Verification Strategy

The test verifies domain decomposition correctness by:

1. **Geometric Validation**: For each process, calculates expected domain boundaries and verifies they match the decomposition result

2. **Point Classification**: Tests various points:
   - Domain center (should be inside)
   - Corner points (should be inside or on boundary)
   - Points just outside each boundary (should be outside)

3. **Cross-Process Consistency**: Ensures all processes report consistent results for their local domains

## Troubleshooting

Common issues and solutions:

- **"MPI not available"**: Ensure PE was built with MPI support (`-DMPI=ON`)
- **Process count mismatch**: Use process counts that match the test configurations (powers of 2 work well)
- **Compilation errors**: Ensure all required PE headers are available and MPI is properly configured

## Future Enhancements

Planned additions to the test suite:
- Neighbor connectivity verification
- Domain overlap/gap detection
- Performance benchmarking for large process counts
- Periodic boundary condition testing