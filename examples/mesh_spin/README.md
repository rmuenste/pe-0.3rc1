# Mesh Spin Example

This example demonstrates collision detection and response between two triangle meshes using DistanceMap acceleration. It is analogous to the `capsule_spin` example but uses triangle meshes instead of analytical capsule geometry.

## Description

The simulation creates two mesh objects:
- **Mesh 1**: Fully fixed (kinematic body) with prescribed rotation at 60 RPM about the z-axis
- **Mesh 2**: Translation-fixed (cannot translate but can rotate freely)

Both meshes are positioned such that Mesh 1's rotation will cause it to collide with Mesh 2, demonstrating:
- DistanceMap-accelerated collision detection for triangle meshes
- Kinematic body collision response (bodies with prescribed motion)
- Translation-fixed body behavior (rotational degrees of freedom only)
- Lever arm effects on angular momentum transfer

## Requirements

- PE built with CGAL support (`-DCGAL=ON`)
- Triangle mesh files (`cylinder.obj` or fallback to `cube.obj`)

## Building

```bash
cd build
cmake -DCGAL=ON -DEXAMPLES=ON ..
make mesh_spin
```

## Running

```bash
./build/examples/mesh_spin/mesh_spin
```

## Output

The example outputs:
- Position, orientation, and angular velocity of both meshes at each timestep
- Collision solver diagnostics (if debug output is enabled)
- VTK files for visualization in ParaView (in `./paraview_mesh/` directory)

## Visualization

View the simulation in ParaView:
```bash
paraview paraview_mesh/bodies_*.vtu
```

## Physics Setup

- **Timesteps**: 120
- **dt**: 0.001 s
- **RPM**: 60 (ω = 6.28 rad/s)
- **Gravity**: Disabled (0, 0, 0)
- **DistanceMap resolution**: 50
- **DistanceMap tolerance**: 5

## Expected Behavior

When Mesh 1 collides with Mesh 2:
- Mesh 1 maintains constant angular velocity (kinematic body with prescribed motion)
- Mesh 2 begins rotating due to contact forces
- The angular velocity ratio depends on lever arm geometry at the contact point
- Contact detection uses DistanceMap acceleration for efficient mesh-to-mesh collision
