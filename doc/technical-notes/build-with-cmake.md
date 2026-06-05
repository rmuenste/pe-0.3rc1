# Building with CMake

This note explains the normal CMake build flow for PE. The repository still contains the legacy custom `./configure` script and `ConfigFile`, but current development should use CMake.

## Prerequisites

- CMake 3.15 or newer
- C++ compiler with C++17 support
- Boost libraries: thread, system, filesystem, program_options, random
- Optional: Ninja, MPI, CGAL, Eigen, OpenCL, Irrlicht

## Recommended Default Build

Use an out-of-source build directory. The default development build should keep examples disabled unless you specifically need them.

```bash
mkdir -p build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DEXAMPLES=OFF ..
make -j 2
```

Equivalent source/build form:

```bash
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DEXAMPLES=OFF
cmake --build build -- -j 2
```

The static library is written to `build/lib/libpe.a`.

## Common Options

- `-DCMAKE_BUILD_TYPE=Release|Debug`: Build type, default `Release`.
- `-DLIBRARY_TYPE=STATIC|SHARED|BOTH`: Library variant, default `STATIC`.
- `-DEXAMPLES=ON|OFF`: Build examples, default `OFF`.
- `-DCGAL=ON|OFF`: Enable CGAL support, default `OFF`.
- `-DEIGEN=ON|OFF`: Enable Eigen support, default `ON`.
- `-DMPI=ON|OFF`: Enable MPI support, default `OFF`.
- `-DBLAS=ON|OFF`: Enable BLAS support, default `OFF`.
- `-DOPENCL=ON|OFF`: Enable OpenCL support, default `OFF`.
- `-DIRRLICHT=ON|OFF`: Enable Irrlicht visualization, default `OFF`.
- `-DUSE_JSON=ON|OFF`: Enable JSON support, default `ON`.

## Building Specific Targets

Build only the PE static library:

```bash
cmake --build build --target pe_static -- -j 2
```

Build one example after configuring with `-DEXAMPLES=ON`:

```bash
cmake --build build --target body_removal -- -j 2
```

Example executables are placed under `build/examples/<example_name>/`.

## Interface Smoke Tests

The primary lightweight smoke test is `tests/interface/pe_interface_smoke_serial.cpp`. It builds the `pe_interface_smoke_serial` executable and registers `pe-interface-serial-*` CTest cases. Each CTest case runs the binary in a fresh process because PE setup mutates process-global singletons.

Configure and run the serial interface smoke tests:

```bash
cmake -S . -B build-interface-tests -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DBUILD_TESTING=ON -DUSE_JSON=ON -DEIGEN=ON
cmake --build build-interface-tests --target pe_interface_smoke_serial -- -j 2
ctest --test-dir build-interface-tests -R pe-interface-serial --output-on-failure
```

For fixture layout, process isolation, CGAL-specific smoke coverage, and adding new cases, see `tests/interface/README.md`.

## Building Examples

Examples are useful for reproductions and manual checks, but they are not the default build. Some examples may lag the current configured solver/API, so prefer building the specific example you need instead of treating `-DEXAMPLES=ON` as a required smoke test.

```bash
cmake -S . -B build-examples -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DEXAMPLES=ON
cmake --build build-examples --target body_removal -- -j 2
```

## CGAL and DistanceMap Builds

CGAL is optional and must be enabled explicitly:

```bash
cmake -S . -B build-cgal -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DCGAL=ON -DEXAMPLES=OFF
cmake --build build-cgal -- -j 2
```

Build CGAL/DistanceMap examples by enabling both `CGAL` and `EXAMPLES`, then build the specific target:

```bash
cmake -S . -B build-cgal-examples -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DCGAL=ON -DEXAMPLES=ON
cmake --build build-cgal-examples --target mesh_simulation -- -j 2
```

Other useful CGAL example targets include `cgal_box`, `mesh_collision_test`, `mesh_distancemap_debug`, `mesh_containspoint_test`, `mesh_grid_test`, `plane_mesh_test`, `debug_coordinate_transform`, and `inverted_distancemap_simulation`.

## Ninja Generator

Ninja is an alternative CMake generator. It is usually faster than Unix Makefiles, but the project configuration is otherwise the same.

```bash
cmake -S . -B build-ninja -G Ninja -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DEXAMPLES=OFF
cmake --build build-ninja -- -j 2
```

Build a specific target with Ninja:

```bash
cmake --build build-ninja --target pe_static -- -j 2
```

You can also call `ninja` directly from the build directory:

```bash
cd build-ninja
ninja -j 2 pe_static
```

## Verification Notes

The default CMake library build was verified with:

```bash
cmake -S . -B build-cmake-verify -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLIBRARY_TYPE=STATIC -DEXAMPLES=OFF
cmake --build build-cmake-verify -- -j 2
```

That configuration completed successfully with warnings. A cached examples-enabled build can fail on stale examples; for example, `examples/box_domain/box_domain.cpp` currently calls `setSlipLength()` on the configured `CollisionSystem`, which does not provide that member.
