# PE Interface Smoke Tests

This directory contains lightweight standalone tests for PE setup functions that
are normally reached from FeatFloWer. The first runner is serial-only:

- Binary: `pe_interface_smoke_serial`
- Fixtures: `fixtures/serial`
- CTest prefix: `pe-interface-serial-*`

Each CTest case starts a separate process. The runner copies the serial fixtures
to a case-local run directory under the build tree, renames the selected config
to `example.json`, changes into that directory, and calls the real serial setup
function.

From a parent FeatFloWer build configured with `USE_PE=ON`,
`USE_PE_SERIAL_MODE=ON`, and `BUILD_TESTING=ON`:

```bash
cmake --build build-el-frozen-trace-gcc14 --target pe_interface_smoke_serial
ctest --test-dir build-el-frozen-trace-gcc14 -R pe-interface-serial --output-on-failure
```

From a standalone PE checkout, only PE's normal dependencies plus testing are
needed:

```bash
cmake -S . -B build-interface-tests \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DPE_USE_JSON=ON \
  -DPE_USE_EIGEN=ON

cmake --build build-interface-tests --target pe_interface_smoke_serial
ctest --test-dir build-interface-tests -R pe-interface-serial --output-on-failure
```

When PE is configured with `-DPE_USE_CGAL=ON`, the serial test set also includes
`pe-interface-serial-atc-cgal-distancemap` and
`pe-interface-serial-span-complex-distancemaps`. The ATC case enables the ATC
domain boundary DistanceMap and verifies that `DistanceMapWriter` writes the
configured VTI file. The SpanComplex case calls `setupSpanComplexSerial`, builds
DistanceMaps for the tool, chip, and workpiece OBJ fixtures, and verifies the
three generated maps. These CGAL cases are intentionally absent from CTest when
`PE_USE_CGAL=OFF`.

```bash
cmake -S . -B build-interface-tests-cgal \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DPE_USE_JSON=ON \
  -DPE_USE_EIGEN=ON \
  -DPE_USE_CGAL=ON

cmake --build build-interface-tests-cgal --target pe_interface_smoke_serial
ctest --test-dir build-interface-tests-cgal \
  -R pe-interface-serial-atc-cgal-distancemap \
  --output-on-failure
```

The generated VTI file is written in the case-local run directory, for example:

```text
build-interface-tests-cgal/tests/interface/serial-runs/atc-cgal-distancemap/atc_boundary_distancemap.vti
```

## MPI Coverage

MPI has an independent build and test path; CGAL is not required. The
`pe-checkpoint-metadata-mpi` test exercises the collective checkpoint write,
including the rank-invariant scratch filename and cross-rank body-count
reduction:

```bash
cmake --preset mpi
cmake --build --preset build-mpi
ctest --preset test-mpi -L mpi
```

The `pe-checkpoint-failure-*` cases run two ranks with a 60-second timeout. They cover
rank-local directory errors, MPI file-open errors, sidecar preservation and publication
failures, rank-local metadata/material/missing-file errors, and malformed body data.
PMPI fault injection covers write submission, completion, close, and local read errors.
Each case checks identical rank/path diagnostics on both ranks, scratch cleanup, and
subsequent successful checkpoint I/O.

```bash
ctest --preset test-mpi -R pe-checkpoint --output-on-failure
```

Checkpoint writes and restores are collective when MPI is initialized. Every rank must
call them in the same order. A recoverable local exception or MPI file I/O error is
reported to all ranks before the next collective. A write succeeds on all ranks only
after rank 0 publishes both files. Before `.peb` publication, failure restores the old
sidecar; after publication, failure leaves the new `.peb` without the old sidecar.
Cleanup is best effort if the filesystem itself prevents removal or rollback. Restore
failure does not roll back materials or bodies already restored on a rank.

## Combined MPI and CGAL Coverage

The frozen-field tracer genuinely requires both optional features. With MPI and
CGAL enabled, `pe_frozen_field_trace_mpi` registers two-rank OFF and OBJ cases.
They verify callback-driven passive advection, rank migration, exact unit-cube
exit attribution, and survivor reporting:

```bash
cmake -S . -B build-interface-mpi-cgal -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC -DBUILD_TESTING=ON \
  -DPE_BUILD_INTERFACE_TESTS=ON -DPE_USE_MPI=ON -DPE_USE_CGAL=ON \
  -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON
cmake --build build-interface-mpi-cgal
ctest --test-dir build-interface-mpi-cgal -L mpi --output-on-failure
```

## ATC Arc Containment (both PE modes)

`pe_atc_arc_containment.cpp` builds one program per build mode from the same source and runs
the ATC case (spheres inside the boundary surface of a 90 degree arc of a coiled tube, closed
by two spherical caps; inverted DistanceMap) on the fixture `fixtures/atc_arc`:

- with MPI and CGAL: `pe-atc-arc-containment-mpi-4`, the parallel `setupATC()` on four
  processes, one per slab of `single_plane.txt`,
- with CGAL and without MPI: `pe-atc-arc-containment-serial`, the serial-mode
  `setupATCSerial()`.

Both drive the engine like an Euler-Lagrange CFD driver (the submerged weight is added as a
force every step) for 600 steps: the 400 spheres fall onto the wall and pile up at the bottom
of the arc, in the parallel case across the plane between ranks 1 and 2. After every step the
sphere centres are compared with the exact surface and the owned spheres are counted over all
processes. The test fails if a centre leaves the domain, a sphere is lost or owned twice, or
a sphere reaches more than one radius through the wall.

```bash
cmake -S . -B build-atc-mpi-cgal -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPE_BUILD_INTERFACE_TESTS=ON -DPE_USE_MPI=ON -DPE_USE_CGAL=ON
cmake --build build-atc-mpi-cgal --target pe_atc_arc_containment
ctest --test-dir build-atc-mpi-cgal -R pe-atc-arc --output-on-failure
```

Drop `-DPE_USE_MPI=ON` for the serial-mode case. The program also runs by hand on other
inputs, with other gravity directions, a Stokes drag and a dump of the final positions; see
its file header. `fixtures/atc_arc/gen_atc_arc_inputs.py` writes the inputs (surface,
positions, slab planes, deck). The wall of this surface touches the bounding box of its
vertices along the bottom of the arc, exactly where the spheres settle, so the test also
guards the bounding box of meshes with an inverted DistanceMap
(`doc/technical-notes/distance-map-implementation.md`); the focused check of that is stage 12
of `pe_primitive_mesh_distancemap_test.cpp`.

With GCC 15 the pinned CGAL 5.3.2 needs `-DCMAKE_CXX_FLAGS=-Wno-template-body`.

## Lubrication Tests

- `pe-lubrication-model` (`pe_lubrication_model_test`): pure-math unit test of the
  Kroupa-2016 lubrication model (`pe/core/lubrication/LubricationModel.h`) and the
  cutoff/AABB-padding helpers. Runs in every build.
- `pe-lubrication-legacy-regression`, `pe-lubrication-two-sphere-approach`,
  `pe-lubrication-bounce-stokes`: World-based Level-1 validation of the
  shared lubrication stage (bit-for-bit legacy gate; paper Fig. 7 approach curve
  with slip/saturation; immersed wall impact with dt sweep and tangential/twist sanity).
  These need the lubricated constraint solver and self-report as SKIPPED (exit 77)
  under any other `pe_CONSTRAINT_SOLVER`. Lubricated build:

```bash
cmake -S . -B build-interface-tests-hcl -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC \
  -DBUILD_TESTING=ON -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON \
cmake --build build-interface-tests-hcl -j
ctest --test-dir build-interface-tests-hcl -R pe-lubrication --output-on-failure
```

Set `PE_LUB_CSV=1` when running the two validation binaries directly to dump plottable
curves (approach resistance, e(St) reference data).

## Coupling API Tests (`pe::coupling`)

The typed C++ coupling API (`pe/interface/coupling/`, see
`doc/technical-notes/coupling-api.md`) has its own tests, all labelled `coupling`. Each
scenario is its own process (pe's world is process-global). They need no fixtures and no
optional dependency; the solver-specific ones link the `pe_static_lubstage_fluid`
(HardContactAndFluid) or `pe_static_lubstage_plain` library variant.

| CTest | Binary | Build | Checks |
|---|---|---|---|
| `pe-coupling-header-standalone-{Export,Types,BuildInfo,Engine}` | the compiler itself | both | each public header compiles alone with the bare compiler (`-std=c++17 -Wall -Wextra -Wshadow -pedantic -Werror`, no pe flags, no forced `config.h`) |
| `pe-coupling-header-standalone-*-after-mpi-h` | the compiler itself | both, if an MPI is found | the same after `-include mpi.h` (pe built with or without MPI) |
| `pe-coupling-header`, `pe-coupling-header-mpi-h` | `pe_coupling_header_test`, `pe_coupling_header_mpi_h_test` | both | a TU with only the coupling header (and `<mpi.h>` around it) links against pe, queries the build information and, without MPI, drives an Engine |
| `pe-coupling-shared-smoke` | `pe_coupling_shared_smoke` | `PE_LIBRARY_TYPE=BOTH` | the same program linked against `libpe.so` (built with `-fvisibility=hidden`): the exported coupling API links and runs |
| `pe-coupling-detection` | `pe_coupling_detection_test` | serial | the solver-feature dispatch templates instantiated with stand-in types lacking every member; `BodyForce::unknown` vs a declared false |
| `pe-coupling-build-info`, `-fluid`, `-plain` | `pe_coupling_build_info*_test` | serial | `buildInfo()` and `isSupported()` under the configured solver, HardContactAndFluid and HardContactSemiImplicitTimesteppingSolvers; unsupported setters throw naming setting and solver; range checks; error reduction and iteration limit change a contact scenario, the contact relaxation does not on one process (pe behaviour); one Engine per process |
| `pe-coupling-world` | `pe_coupling_world_test` | serial | creation; enumeration of API and native bodies of every shape, fixed bodies, a plane; state against pe's accessors bit for bit; volume and its cache; state override with pe's quaternion tolerance; id lookup after add and destroy; creation-time validation (non-finite materials, unnormalizable plane normals, collapsing bounding boxes); distinct ids for native global and local bodies without MPI; a native binary checkpoint patched into the old two-counter layout does not make new bodies reuse ids; a rotated ellipsoid (2, 1, 1) reports its new bounding box right after `setKinematicState()`, `readCheckpoint()` and pe's other orientation setters, and both query methods own its rotated tips; pe's quaternion equality tells apart quaternions differing in one component; a non-convex mesh turned from +45 to -45 degrees about z reports the new box bit for bit and is classified there (both methods, also after a checkpoint round trip), and a sub-tolerance translation renews its vertices; a thin box at x = 2^40 is accepted at its own pose and reads its own checkpoints (per-axis position check) |
| `pe-coupling-classification` | `pe_coupling_classification_test` | serial | exhaustive vs analytic sphere containment; spatial index vs exhaustive with the hash grid asserted active; minimum-id rule and owner lists; fixed-body rule; `containsPoints()` vs `contains()`; invalid query points; after motion with refresh (a no-refresh control must miss a moved body); the legacy `pointInsideParticles*()` / `pointOwners*()` agree, flags included |
| `pe-coupling-loads` | `pe_coupling_loads_test` | serial | load accumulation, the relaxation arithmetic and the first-load rule bit for bit (also for a body first loaded in step 2), untouched bodies keep native forces and velocities, the history of a temporarily fixed body (with and without an unrelated body, across a checkpoint), overflowing accumulation refused with the earlier sum intact, no second `applyLoads()` and no checkpoint between `applyLoads()` and `advance()` (also after the Engine handle was replaced), after a native reload following `applyLoads()` both stay refused until `acknowledgeWorldReload()`, which also drops pending loads and history, a rollback by `readCheckpoint()` keeps a -0 velocity -0 under a -0 load as in the uninterrupted run (pe's body-level fluid-force history restored), native force/torque slots block checkpoints, no-op and double `applyLoads()`, `clearLoads()`, the guards |
| `pe-coupling-fluid` | `pe_coupling_fluid_test` | serial | HardContactAndFluid: first step from rest is $(1-\rho_f/\rho_p) g \Delta t$; a sphere settles onto a plane; `advance( dt, n )` equals n steps of `dt/n` bit for bit; `advance()` prints nothing on CFD rank 1 while a pe trigger inside the step still sees rank 1 |
| `pe-coupling-feat-equivalence` | `pe_coupling_feat_equivalence_test` | serial | the FEAT3 wrapper (`pe_world.cpp`) replicated call by call against the API, 1500 steps bit for bit, two configurations |
| `pe-coupling-checkpoint-write` / `-restart`, `-pile-write` / `-pile-restart` | `pe_coupling_checkpoint_test` | serial | N steps, checkpoint, M steps (a -0 force component around the checkpoint while pe's body-level fluid-force history holds a positive force); pe's body-level history is restored bit for bit and the state right after the first impulse after the restart equals the uninterrupted one; a second process rebuilds the world, restores, runs M; bit for bit (three spheres and a fixed sphere moving with a prescribed velocity; 64-sphere pile with the grid active) |
| `pe-coupling-checkpoint-shake-write` / `-shake-restart` | `pe_coupling_checkpoint_test` | serial | characterisation: bodies exchange grid cells before many simultaneous contacts; the restart is reproduced only up to contact-order round-off (deviations printed; finiteness and a loose 1e-6 bound asserted) |
| `pe-coupling-checkpoint-history-write` / `-history-restart` | `pe_coupling_checkpoint_test` | serial | characterisation: bodies created and destroyed after stepping began (grid inactive); the restart is reproduced only up to contact-order round-off |
| `pe-coupling-plane-geometry` | `pe_coupling_checkpoint_test` | serial | characterisation: pe changes the stored normal of a tilted plane at rest in its first step (reported) |
| `pe-coupling-checkpoint-errors` | `pe_coupling_checkpoint_test` | serial | wrong tag, versions 2 and 4, counts, a bad or non-finite body-level fluid-force record, unknown, duplicate and wrong-kind ids, non-unit quaternions, unrepresentable positions, bad history flag, truncation at the end, a fixed body with a velocity, or a `-0` velocity component, in a build without `MOBILE_INFINITE` (restored exactly with it); each against a differing live state with a pending load, nothing may change; a union and planes moving or rotating (also at a sub-tolerance 1e-9) are refused; preconfigured streams (hex, noskipws, pending input width, output width, fill, comma locale); a build without checkpoint support refuses |
| `pe-coupling-mpi` | `pe_coupling_mpi_test` (2 ranks) | MPI | `advance()` synchronizes before the first step (creation-triggered only; and after an Engine handle was replaced); contact relaxation acts across the cut; API planes with the same id on every rank; shadow copies enumerated and classified; loads, checkpoints and overrides of global (planes too), shadowed, shadow or migrating bodies throw "not implemented yet"; `advance()` in an exclusive section throws locally and both ranks continue |
| `pe-coupling-mpi-single` | `pe_coupling_mpi_test` (1 rank) | MPI | the MPI build is fully functional on one process, including an API plane with a sphere resting on it |

```bash
ctest --test-dir <build> -L coupling --output-on-failure
```

## Adding Tests

Add new interface coverage as a new CTest case, not as another setup call inside
an existing runner invocation. PE setup functions mutate process-global
singletons, so each scenario needs its own process.

1. Add or reuse fixture files under `fixtures/serial`.
2. Add a `pe_add_interface_serial_test(<ctest-name>, <case-name>)` entry in
   `tests/interface/CMakeLists.txt`. Wrap it in the relevant feature gate, such
   as `if(PE_USE_CGAL)`, when the case requires an optional dependency.
3. Update `pe_interface_smoke_serial.cpp` so `main()` maps `<case-name>` to the
   fixture config copied to `example.json`.
4. Implement a focused helper that calls exactly one setup function and checks
   the expected bodies, config flags, files, or roundtrip behavior.
5. Dispatch `<case-name>` to that helper in `main()`.

Future parallel interface coverage should live next to this runner as a separate
`pe_interface_smoke_parallel` target with fixtures under `fixtures/parallel` and
CTest names using the `pe-interface-parallel-*` prefix.
