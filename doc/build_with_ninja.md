# Building with CMake and Ninja

This document explains how to configure and build the PE physics engine using the Ninja build system, which offers faster build times compared to Makefiles.

## Prerequisites

Ensure you have the following installed:
*   **CMake** (Version 3.15 or higher)
*   **Ninja** (build system)
*   **C++ Compiler** (GCC, Clang, or Intel, supporting C++14/17)
*   **Boost Libraries** (thread, system, filesystem, program_options, random)

## Configuration

To use Ninja, you must specify the `-G Ninja` generator during the CMake configuration step. It is recommended to perform an out-of-source build.

1.  Create a build directory (e.g., `build_ninja`):
    ```bash
    mkdir build_ninja
    cd build_ninja
    ```

2.  Run CMake with the Ninja generator. You can also specify the build type (Release/Debug) and enable examples if desired.

    **Standard Configuration (Release mode):**
    ```bash
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
    ```
    or
    ```bash
    cmake -S . -B build-ninja2 -DCMAKE_BUILD_TYPE=Release -DEXAMPLES=ON -G Ninja ..
    ```

    **Configuration with Examples Enabled:**
    To build examples (like `body_removal`), you must enable the `EXAMPLES` option:
    ```bash
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DEXAMPLES=ON ..
    ```

## Building with CGAL Support

The project includes optional support for the CGAL library, which is handled as an external project. Enabling this option will download, build, and install CGAL automatically.

### Configuration

To enable CGAL support, use the `-DCGAL=ON` flag during configuration:

```bash
mkdir build_ninja_cgal
cd build_ninja_cgal
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCGAL=ON ..
```

### Building the CGAL Target

When `CGAL` is enabled, a `cgal` target is available. This target triggers the download (git clone), configuration, build, and installation of the CGAL library into the build directory.

To build the `cgal` target:

```bash
cmake --build . --target cgal
```

Or using `ninja`:

```bash
ninja cgal
```

**Verification:**
After the build completes, you can verify that the CGAL headers have been installed correctly. They should be located at:
`build_ninja_cgal/extern/cgal/install/include/CGAL/`

Once the `cgal` target is built, other components of the project that depend on CGAL will be able to find the necessary headers and libraries.

### Building CGAL Examples

To build examples that depend on CGAL (located in `examples/cgal_examples`), you must ensure both `CGAL` and `EXAMPLES` are enabled during configuration:

```bash
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCGAL=ON -DEXAMPLES=ON ..
```

Then you can build all CGAL example targets:

```bash
ninja cgal_box mesh_collision_test mesh_simulation mesh_distancemap_debug mesh_containspoint_test mesh_grid_test plane_mesh_test debug_coordinate_transform inverted_distancemap_simulation
```

Or build a specific example target, for example `mesh_simulation`:

```bash
cmake --build . --target mesh_simulation
```

Or using `ninja`:

```bash
ninja mesh_simulation
```

**Note:** The first time you build a CGAL-dependent target, CMake will ensure the `cgal` external project is built and installed if it hasn't been already.

## Building the Library

To build the static library (`pe_static`), run the following command from your build directory:

```bash
cmake --build . --target pe_static
```

Alternatively, you can use the `ninja` command directly if it's in your path:
```bash
ninja pe_static
```

The compiled library will be located in `lib/libpe.a` (inside the build directory).

## Building Examples

To build a specific example, such as `body_removal`, ensure you have configured the project with `-DEXAMPLES=ON` (see Configuration section above).

Then, build the specific target:

```bash
cmake --build . --target body_removal
```

Or using `ninja` directly:
```bash
ninja body_removal
```

### Running the Example

After a successful build, the executable can be found in the `examples/<example_name>/` directory within your build folder.

For `body_removal`:
```bash
./examples/body_removal/body_removal
```
