# Building with CMake

This note explains the normal CMake build flow for PE. The repository still contains the legacy custom `./configure` script and `ConfigFile`, but current development should use CMake.

## Prerequisites

- CMake 3.15 or newer
- C++ compiler with C++17 support
- Boost libraries: thread, system, filesystem, program_options, random
- Optional: Ninja, MPI, CGAL, Eigen, nlohmann/json, OpenCL, Irrlicht. Eigen, json and CGAL are
  downloaded if they are not installed, see [Third-party libraries](#third-party-libraries).

## Build and Test Presets

The checked-in presets provide reproducible development and CI configurations.
They require CMake 3.21 or newer; the explicit CMake commands below remain
available when using the project's minimum supported CMake version.

The feature presets are deliberately independent:

- `serial`: Release build with MPI and CGAL disabled.
- `debug`: Debug counterpart of the serial build.
- `mpi`: Release build with MPI enabled and CGAL disabled.
- `cgal`: Release build with CGAL enabled and MPI disabled.

All four presets build a static library, enable the complete applicable CTest
suite, disable examples and interactive tools, and use separate build trees
under `build/presets/`.

```bash
cmake --preset serial
cmake --build --preset build-serial
ctest --preset test-serial

cmake --preset mpi
cmake --build --preset build-mpi
ctest --preset test-mpi
```

Replace `serial` with `debug` or `cgal` in the three preset names to use
those configurations. CTest labels such as `unit`, `integration`, `serial`,
`mpi`, `cgal`, `interface`, `checkpoint`, and `lubrication` support
focused runs, for example `ctest --preset test-mpi -L mpi`.

## Recommended Default Build

Use an out-of-source build directory. The default development build should keep examples disabled unless you specifically need them.

```bash
mkdir -p build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_BUILD_EXAMPLES=OFF ..
make -j 2
```

Equivalent source/build form:

```bash
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_BUILD_EXAMPLES=OFF
cmake --build build -- -j 2
```

The static library is written to `build/lib/libpe.a`.

## Common Options

- `-DCMAKE_BUILD_TYPE=Release|Debug`: Build type, default `Release`.
- `-DPE_LIBRARY_TYPE=STATIC|SHARED|BOTH`: Library variant, default `STATIC`.
- `-DPE_BUILD_EXAMPLES=ON|OFF`: Build examples, default `OFF`.
- `-DPE_USE_CGAL=ON|OFF`: Enable CGAL support, default `OFF`.
- `-DPE_USE_EIGEN=ON|OFF`: Enable Eigen support, default `ON`.
- `-DPE_USE_MPI=ON|OFF`: Enable MPI support, default `OFF`.
- `-DPE_USE_BLAS=ON|OFF`: Enable BLAS support, default `OFF`.
- `-DPE_USE_OPENCL=ON|OFF`: Enable OpenCL support, default `OFF`.
- `-DPE_USE_IRRLICHT=ON|OFF`: Enable Irrlicht visualization, default `OFF`.
- `-DPE_USE_JSON=ON|OFF`: Enable JSON support, default `ON`.
- `-DPE_PREFER_EXTERNAL_TPL=ON|OFF`, `-DPE_NO_EXTERNAL_DOWNLOAD=ON|OFF`,
  `-DPE_TPL_CACHE_DIRECTORY=<dir>`: where Eigen, json and CGAL come from, see
  [Third-party libraries](#third-party-libraries).
- `-DPE_MOBILE_INFINITE=ON|OFF`: Mobile infinite bodies, default `ON` for every compiler. Defines
  `MOBILE_INFINITE=1` (or `0`) for pe and all its users: a fixed body may carry a prescribed velocity
  (a kinematic body, see [rigid-body-documentation.md](rigid-body-documentation.md#kinematic-moving-fixed-bodies)).
  Before this option the macro was defined for GCC only, so Clang/Intel builds silently ignored the
  velocity of fixed bodies. With `OFF` (the original pe semantics) `examples/shaker` exits with a
  message and `conveyor --moving-belt` is skipped.
- Sleep mode is compiled out (`pe::sleepThreshold = 0` in `pe/config/SleepMode.h`); to enable it
  without editing the header pass `-DPE_PREPROCESSOR_FLAGS=-DPE_SLEEP_THRESHOLD=<threshold>`
  (a threshold on the recency-averaged `v^2 + w^2` of a body).

## Building Specific Targets

Build only the PE static library:

```bash
cmake --build build --target pe_static -- -j 2
```

Build one example after configuring with `-DPE_BUILD_EXAMPLES=ON`:

```bash
cmake --build build --target body_removal -- -j 2
```

Example executables are placed under `build/examples/<example_name>/`.

## Interface Smoke Tests

The primary lightweight smoke test is `tests/interface/pe_interface_smoke_serial.cpp`. It builds the `pe_interface_smoke_serial` executable and registers `pe-interface-serial-*` CTest cases. Each CTest case runs the binary in a fresh process because PE setup mutates process-global singletons.

Configure and run the serial interface smoke tests:

```bash
cmake -S . -B build-interface-tests -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DBUILD_TESTING=ON -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON
cmake --build build-interface-tests --target pe_interface_smoke_serial -- -j 2
ctest --test-dir build-interface-tests -R pe-interface-serial --output-on-failure
```

For fixture layout, process isolation, CGAL-specific smoke coverage, and adding new cases, see `tests/interface/README.md`.

## Building Examples

Examples are useful for reproductions and manual checks, but they are not the default build. (Until 2026-10 the serial example list in `CMakeLists.txt` was walked with a double stride, so `chain`, `domino`, `nano` and `shaker` were never built; they are now.) Some examples may lag the current configured solver/API, so prefer building the specific example you need instead of treating `-DPE_BUILD_EXAMPLES=ON` as a required smoke test.

```bash
cmake -S . -B build-examples -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_BUILD_EXAMPLES=ON
cmake --build build-examples --target body_removal -- -j 2
```

## CGAL and DistanceMap Builds

CGAL is optional and must be enabled explicitly:

```bash
cmake -S . -B build-cgal -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_USE_CGAL=ON -DPE_BUILD_EXAMPLES=OFF
cmake --build build-cgal -- -j 2
```

Build CGAL/DistanceMap examples by enabling both `PE_USE_CGAL` and `PE_BUILD_EXAMPLES`, then build the specific target:

```bash
cmake -S . -B build-cgal-examples -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_USE_CGAL=ON -DPE_BUILD_EXAMPLES=ON
cmake --build build-cgal-examples --target mesh_simulation -- -j 2
```

Other useful CGAL example targets include `cgal_box`, `mesh_collision_test`, `mesh_distancemap_debug`, `mesh_containspoint_test`, `mesh_grid_test`, `plane_mesh_test`, `debug_coordinate_transform`, and `inverted_distancemap_simulation`.

## Third-party libraries

Eigen, nlohmann/json and CGAL are provided by `pe_get_tpl()` in
[`cmake/PEGetTPL.cmake`](../../cmake/PEGetTPL.cmake), adapted from FEAT3's `feat_get_tpl()`.
An installed package is used first. Otherwise a pinned release archive is downloaded and its
SHA-256 is checked. Boost and MPI are never downloaded: they must be installed.

| Library | pe option | Installed package accepted | Pinned archive | Target |
|---|---|---|---|---|
| Eigen | `PE_USE_EIGEN` (ON) | `find_package(Eigen3 3.4.0 CONFIG)`, 3.4 or a newer version its version file accepts | `eigen-3.4.0.tar.bz2` (GitLab) | `Eigen3::Eigen` |
| nlohmann/json | `PE_USE_JSON` (ON) | `find_package(nlohmann_json 3.12.0 CONFIG)`, 3.x from 3.12 | `json.tar.xz` of v3.12.0 (GitHub release) | `nlohmann_json::nlohmann_json` |
| CGAL | `PE_USE_CGAL` (OFF) | `find_package(CGAL)`, any version | `CGAL-5.3.2-library.tar.xz` (GitHub release) | `CGAL::CGAL` if installed, else include path |

The downloaded copies are not configured with their own CMakeLists: pe creates the targets from
the source tree. This keeps Eigen's tests and the `JSON_*` options out of pe's build. Their
include directories are system include directories, so warnings in their headers are not shown.
Ubuntu 24.04 ships Eigen 3.4.0 (`libeigen3-dev`) and CGAL 5.6, but only json 3.11.3, so json is
downloaded there. pe accepts json only from 3.12 on, so that installed and downloaded builds use
the same version.

### Where a library comes from

For each library, in this order:

1. A target that already exists, for example one defined by a parent project, is used as it is.
2. `find_package()`, if `PE_PREFER_EXTERNAL_TPL` (default `ON`) or `PE_NO_EXTERNAL_DOWNLOAD` is
   on, or if `<Package>_DIR` is set (e.g. `-DEigen3_DIR=<prefix>/share/eigen3/cmake`). This
   also finds packages through `CMAKE_PREFIX_PATH`.
3. `FETCHCONTENT_SOURCE_DIR_<PACKAGE>` (upper case: `FETCHCONTENT_SOURCE_DIR_EIGEN3`,
   `FETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON`, `FETCHCONTENT_SOURCE_DIR_CGAL`), if it points to an
   extracted source tree.
4. The archive in the cache, downloaded first if needed.

Configure output shows which source was used, e.g.
`pe TPL Eigen3: found version 3.4.0 in /usr/share/cmake/eigen3` or
`pe TPL nlohmann_json: using the cached archive .../nlohmann_json-3.12.0-json.tar.xz`.
To make pe download Eigen even though Eigen is installed, pass `-DPE_PREFER_EXTERNAL_TPL=OFF`
or `-DCMAKE_DISABLE_FIND_PACKAGE_Eigen3=ON`.

### Archive cache

Downloaded archives are kept in `PE_TPL_CACHE_DIRECTORY` (default `thirdparty/cache` in the
source tree, ignored by git) as `<Package>-<version>-<file>`, for example
`Eigen3-3.4.0-eigen-3.4.0.tar.bz2`. Every build directory reuses them, so a second build
directory needs no network. A cached archive is used only if its hash matches the pin. With
`-DPE_TPL_CACHE_DIRECTORY=` (empty), each build directory keeps its own copy in
`_deps/pe-tpl-archives`. CI keeps `thirdparty/cache` with `actions/cache`.

### URLs, mirror and hashes

The Eigen and json pins have two URLs: upstream first, then a copy in
[tudo-math-ls3/pe-third-party](https://github.com/tudo-math-ls3/pe-third-party) (one release
per archive, named `<package>-<version>`; a file in an existing release is never replaced).
The CGAL pin has only the upstream URL, because the mirror has no CGAL release yet.
The URLs are tried in order:

- A failed download (unreachable host, HTTP error) moves on to the next URL.
- A file whose hash does not match the pin is deleted with a warning, and the next URL is tried.
  A file with a wrong hash is never used, from any URL or from the cache.
- If no URL delivers the pinned file, configuration stops with the list of failures.

Trying the next URL after a mismatch is safe because only a file with the pinned hash is ever
accepted. It also handles the case where an upstream host regenerates its archive. If the
mirror cannot help either, verify the new upstream archive and move the pin (below).

How the pinned hashes were verified:

| Archive | SHA-256 | Bytes | Independent check |
|---|---|---:|---|
| `eigen-3.4.0.tar.bz2` | `b4c198460eba6f28d34894e3a5710998818515104d6e74e5cc331ce31e46e626` | 2143091 | identical to `eigen3_3.4.0.orig.tar.bz2` of Debian's signed source package `eigen3_3.4.0-4.dsc` |
| `json.tar.xz` (v3.12.0) | `42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa` | 114576 | the SHA-256 in the upstream v3.12.0 release notes (an `.asc` signature is also published) |
| `CGAL-5.3.2-library.tar.xz` | `404e43f09fa590681976429e683278351056041e28f57e999eac4341a83811ad` | 5180180 | `sha256sum.txt` (and `sha512sum.txt`) of the CGAL v5.3.2 release, uploaded with the archive on 2022-06-06; the entry of `CGAL-5.3.2.tar.xz` in that file equals the SHA-256 in Spack's `cgal` package |

### Builds without network access

With `-DPE_NO_EXTERNAL_DOWNLOAD=ON`, pe never downloads anything. A library that is enabled
but not available is a configure error that names the pinned archive and the ways to provide
the library:

- install it and let `find_package()` find it (`<Package>_DIR` or `CMAKE_PREFIX_PATH`),
- copy the archive into `PE_TPL_CACHE_DIRECTORY` under the cache name,
- set `FETCHCONTENT_SOURCE_DIR_<PACKAGE>` to an extracted tree, for example
  `-DFETCHCONTENT_SOURCE_DIR_EIGEN3=/path/eigen-3.4.0`,
- or turn off the pe option that needs the library.

### pe as a sub-project

When pe is added with `add_subdirectory()` or FetchContent (FEAT3 does this, with
`PE_USE_EIGEN`, `PE_USE_JSON` and `PE_USE_CGAL` set to `OFF`), all of the above still works.
If the parent already defines `Eigen3::Eigen` or `nlohmann_json::nlohmann_json`, pe uses those
targets. The targets pe creates for downloaded libraries are global, so the parent can use them
too. With
CMake 3.24 or newer, a later `find_package(Eigen3)` or `find_package(nlohmann_json)` in the
parent is answered with pe's download (FetchContent `OVERRIDE_FIND_PACKAGE`). The FetchContent
names are the package names (`Eigen3`, `nlohmann_json`, `CGAL`), so if the parent declares one
of them first, the parent's declaration is used. All of pe's options and functions carry the
`PE_`/`pe_` prefix.

### Moving a pin

`tools/bump_tpl_pin.py` updates `VERSION`, `URLS` and `URL_HASH` of a `pe_get_tpl()` call in
`CMakeLists.txt`. It downloads the new archive, computes its hash, checks that the mirror copy is
identical, and stores the archive in the cache. Only Python 3 is needed:

```bash
python3 tools/bump_tpl_pin.py nlohmann_json 3.12.1 \
    https://github.com/nlohmann/json/releases/download/v3.12.1/json.tar.xz \
    --mirror https://github.com/tudo-math-ls3/pe-third-party/releases/download/nlohmann-json-3.12.1/json.tar.xz
```

If the mirror copy does not exist yet, the script reminds you to upload the archive as a new
release of `pe-third-party`. The hash it computes only describes what the server delivered.
Before committing, verify the archive against an independent source (a distribution's source
package, a published checksum or signature) and add it to the table above. `--dry-run` only
reports what would change.

## Ninja Generator

Ninja is an alternative CMake generator. It is usually faster than Unix Makefiles, but the project configuration is otherwise the same.

```bash
cmake -S . -B build-ninja -G Ninja -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_BUILD_EXAMPLES=OFF
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
cmake -S . -B build-cmake-verify -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DPE_BUILD_EXAMPLES=OFF
cmake --build build-cmake-verify -- -j 2
```

That configuration completed successfully with warnings.

Examples-enabled builds can still fail on examples that constrain the solver at compile time. `examples/mpiperiodic`, `examples/mpiperiodic_dev` and `examples/mpiplanedecomposition` assert that `pe_CONSTRAINT_SOLVER` is either `pe::response::HardContactSemiImplicitTimesteppingSolvers` or `pe::response::HardContactAndFluid`, because their parameters are tuned for those; build them with e.g. `-DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid"`. `examples/lubrication_demo` needs a stage-capable solver too, but refuses at runtime rather than at compile time, so it builds in any configuration.
