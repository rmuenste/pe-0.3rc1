# FEAT3 &harr; PE Frozen-Field Tracer Coupling

How FEAT3's distributed finite-element velocity evaluation is coupled with PE's
parallel rigid-body/tracer engine, and — in detail — how to **compile the two
together** and **run** the coupled debug app. Companion PE notes:
[`frozen-field-cpp-interface.md`](frozen-field-cpp-interface.md) (the callback
interface PE exposes) and [`build-with-cmake.md`](build-with-cmake.md) (general PE
CMake usage).

This note lives in the PE repo but describes a **two-repo** build (FEAT3 with PE
nested inside it); read section 2 carefully.

Verified:
- 2026-07-07, WSL2 workstation, GCC 13, OpenMPI, 4 ranks (original bring-up).
- 2026-07-07, RHEL9 `lws` workstation (lws2024-09), GCC 13.2.0, OpenMPI 4.1.6,
  system CGAL 6.0.1, 4 ranks — baseline reproduced exactly. The RHEL9 run
  surfaced two build issues now folded into section 5.

## 1. Mission

PE advects passive tracer particles through a *frozen* (time-constant) velocity
field and calls back into FEAT3 to evaluate that field at arbitrary points on a
distributed mesh. The debug vehicle is the FEAT3 app
`area51/dbg_parti_velocity_eval.cpp` (target `dbg-parti-velocity-eval`).

Current milestone (done): full pipeline runs on 4 MPI ranks on a unit cube with
analytic field u = (0, 0, z); tracer exits are gathered to rank 0 and written as
structured JSON. Next milestone: scale up on a larger MPI system.

## 2. Repository topology — read carefully

Two independent git repositories, one nested in the other:

| Repo | Path | Remote | Branch / state |
|---|---|---|---|
| FEAT3 | *(the outer repo)* | `git@gitlab.tu-dortmund.de:feat3/feat3.git` | `rm_structured_exit_output` (local-only) |
| PE | `pe-0.3rc1/` | `git@github.com:rmuenste/pe-0.3rc1.git` | `master` @ `301afb9` (frozen-field interface, PR #20) |

- `pe-0.3rc1/` is **not** tracked by the FEAT3 repo (not a submodule — it shows up
  as an untracked directory). On a new machine, clone it into the FEAT3 root under
  exactly this name:
  `git clone git@github.com:rmuenste/pe-0.3rc1.git pe-0.3rc1`
  The FEAT3 top-level `CMakeLists.txt` does `add_subdirectory(pe-0.3rc1)`.
- Build directories (e.g. `build-mpi/`) are untracked; never commit them.

### Branch map (FEAT3)

- `master` — upstream FEAT3, untouched by this work.
- `messer_particle_coupling` — Maximilian Esser's WIP evaluator app.
- `rm_particle_coupling` — adds the actual PE coupling.
- `rm_structured_exit_output` — current branch, adds JSON exit output plus the
  `dbg-parti-velocity-eval` gmp/mpfr link fix (see section 5).

### Relevant PE history

PE gained the coupling interface in PR #20 (`feature/frozen-field-cpp-interface`,
merged as `301afb9`): `pe::runFrozenFieldTrace()` declared in
`pe/interface/frozen_field_trace.h`, implemented in
`src/interface/frozen_field_trace.cpp`, MPI test in
`tests/interface/pe_frozen_field_trace_mpi.cpp`.

## 3. What the app does (data flow)

1. `PartiDomainControl` partitions a 3D hexahedral mesh across ranks (2-level
   partitioning; SCOTCH is also available in-tree).
2. The analytic velocity u = (0, 0, z) is interpolated into a Lagrange-2 space.
   **Do not add a `sync_0()` after the interpolation** — the projected field is
   already type-1 consistent; `sync_0()` is additive and doubles shared DOFs on
   partition seams (bug fixed in `3f3b953a8`; only manifests in parallel).
3. A distributed point evaluator is built from `Global::InverseMapping` +
   `Global::ScalarRemoteLambda`: any rank can query any point; evaluation is
   routed to the rank owning the containing cell.
4. Rank 0 extracts the domain boundary, converts it to an **OFF triangle mesh via
   `Geometry::CGALWrapper`** (which repairs and *consistently orients* the
   otherwise-unoriented boundary triangle soup), and **broadcasts the OFF string
   to all ranks**. This step needs FEAT3 built with CGAL — see section 5, it is
   mandatory, not optional.
5. All cell midpoints are allgathered into one deterministic, byte-identical seed
   list on every rank.
6. `pe::runFrozenFieldTrace(seeds, OffMeshData, velocity_callback)` is called
   collectively on `MPI_COMM_WORLD`. PE decomposes the unit cube per the process
   grid in `example.json`, creates rank-owned tracer spheres, builds a global
   fixed `TriangleMesh` boundary with inverted `DistanceMap`, and steps
   `timesteps_` main steps, invoking the callback before the first and after every
   step. The callback wraps the FEAT3 evaluator from step 3.
7. Per-rank exits/survivors are packed into fixed-stride double records, gathered
   to rank 0, and written as JSON (`--exit-output`, default `domain_exits.json`).

## 4. Hard invariants (violating these aborts or corrupts the run)

- **Byte-identical collective input**: every rank must pass the *same* seed list
  and the *same* OFF string to `runFrozenFieldTrace` — PE hashes both and
  `MPI_Abort`s on mismatch. Hence the OFF is broadcast from rank 0 and the seed
  list is rebuilt deterministically from allgathered buffers.
- **The OFF must be closed AND consistently oriented.** PE validates every edge is
  shared by exactly two triangles wound in opposite directions, and that the
  signed volume is positive (outward CCW). A raw FEAT3 boundary soup is closed but
  *not* consistently oriented; only the CGAL path (step 4) fixes this. If FEAT3 is
  built without CGAL the app silently writes the raw soup and PE aborts with
  `surface must be closed and consistently oriented` — see section 5.
- **Rank count must equal the PE process grid**: `mpirun -np N` requires
  `N == processesX_ * processesY_ * processesZ_` in `example.json`. (There is **no**
  extra coordinator rank for this app — unlike the FeatFloWer EL apps.)
- **`example.json` is loaded from the current working directory** by PE. Run the
  app from a directory containing it.
- **The velocity callback is collective**: PE invokes it on every rank each step;
  the FEAT3 evaluator inside is itself collective over the comm. Do not add early
  returns / rank-dependent branches inside it.
- The app expects a `conformal:hypercube:3:3` mesh (3D hexahedra) and the
  mandatory flags `--mesh`, `--level`, `--surface-file`.

## 5. Build (FEAT3 + PE together), in detail

The coupled executable is a **FEAT3** CMake target (`dbg-parti-velocity-eval`)
that links the `feat` library and PE's `pe_static` library. PE is pulled in by the
FEAT3 top-level `add_subdirectory(pe-0.3rc1)`, so a single CMake configure/build of
the FEAT3 tree builds both.

### 5.1 Dependencies

| Dependency | Used by | Notes |
|---|---|---|
| C/C++ compiler | both | GCC 13.x verified. Do not switch compilers inside an existing CMake cache — reconfigure a fresh build dir. |
| MPI | both | OpenMPI verified (4.1.6). `FEAT_HAVE_MPI=ON`, `PE_USE_MPI=ON`. |
| CGAL | **both** | FEAT3 needs it to orient the boundary OFF (mandatory here); PE needs it for `TriangleMesh`. FEAT3 targets **CGAL 6.0.1**. |
| Boost | PE (+CGAL) | thread/system/filesystem/program_options/random. **Use one consistent version for headers and libs** (see 5.4). |
| GMP / MPFR | CGAL backend | CGAL's exact arithmetic; both FEAT3 and PE reference `__gmp*`/`__mpfr*`. |
| CMake ≥ 3.15, Ninja | build | Ninja generator verified. |

FEAT3 and PE may each resolve CGAL independently. If no system CGAL is found, each
falls back to downloading its own copy via FetchContent (needs network). Prefer a
single **system CGAL 6.0.1** for both (section 5.3).

### 5.2 Configure — the important flags

The minimal historical command (WSL2, where a compatible system CGAL was
auto-detected) was:

```bash
cmake -S . -B build-mpi -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
      -DFEAT_HAVE_MPI=ON -DPE_USE_MPI=ON -DPE_USE_CGAL=ON
```

**This is not sufficient on a machine without an auto-detected system CGAL.**
`-DPE_USE_CGAL=ON` only enables CGAL for *PE*. FEAT3's CGAL is controlled by the
separate `-DFEAT_HAVE_CGAL` option (default OFF). If it stays OFF, FEAT3 compiles
the boundary-OFF exporter's `#else` branch, which writes the **raw unoriented**
soup, and PE aborts at runtime (section 4). Always set `FEAT_HAVE_CGAL=ON`:

```bash
cmake -S . -B build-mpi -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
      -DFEAT_HAVE_MPI=ON -DPE_USE_MPI=ON \
      -DPE_USE_CGAL=ON -DFEAT_HAVE_CGAL=ON \
      -DCGAL_DIR=/path/to/cgal/lib64/cmake/CGAL \
      -DMPFR_LIBRARY=/path/to/libmpfr.so \
      -DGMP_LIBRARY=/path/to/libgmp.so
```

- `FEAT_HAVE_CGAL=ON` makes FEAT3 fetch/find **CGAL 6.0.1**. Its geometry kernel
  does `find_package(CGAL 5.5.5 REQUIRED)`, but 6.0.1 satisfies that minimum.
- `CGAL_DIR` should point at a CGAL `CGALConfig.cmake` directory so `find_package`
  uses the system install instead of downloading.
- `MPFR_LIBRARY` / `GMP_LIBRARY`: set these explicitly if the system lacks the
  unversioned `libmpfr.so` / `libgmp.so` dev symlinks. When FEAT3 enables the
  system CGAL, PE's `find_package(CGAL)` also resolves to it and runs
  `CGAL_SetupGMP.cmake`, which uses PE's `cmake/FindMPFR.cmake`; that finder needs
  `MPFR_LIBRARY` and will hard-fail (`Could NOT find MPFR (missing:
  MPFR_LIBRARY)`) if only the header is present. Pointing at a versioned
  `libmpfr.so.N` by full path is fine.

### 5.3 Example: RHEL9 `lws` cluster (environment modules)

```bash
source /etc/profile.d/modules.sh
module purge
module load gcc/13.2.0 openmpi/4.1.6 cgal/6.0.1     # do NOT load boost/1.88 — see 5.4

cd <feat3-root>
git clone git@github.com:rmuenste/pe-0.3rc1.git pe-0.3rc1   # if missing

cmake -S . -B build-mpi -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
      -DFEAT_HAVE_MPI=ON -DPE_USE_MPI=ON \
      -DPE_USE_CGAL=ON -DFEAT_HAVE_CGAL=ON \
      -DCGAL_DIR=/sfw/cgal/gcc13.2.0/lib64/cmake/CGAL \
      -DMPFR_LIBRARY=/usr/lib64/libmpfr.so.6 \
      -DGMP_LIBRARY=/usr/lib64/libgmp.so

cmake --build build-mpi --target dbg-parti-velocity-eval -j <N>
```

Modules must be loaded in **every** shell that configures, builds, or runs — an
agent/non-login shell is fresh each time and modules do not persist across
commands. Never pipe `module load` (e.g. `module load ... | grep`): `module` is a
shell function, the pipe runs it in a subshell, and the loaded environment is
lost.

### 5.4 Boost header/library consistency (a real trap)

CGAL and PE both use Boost. The compile-time headers and the link-time libraries
**must be the same Boost version**. On the `lws` cluster, loading `boost/1.88`
prepends `/sfw/boost/gcc13.2.0/include` (Boost 1.88) to `CPLUS_INCLUDE_PATH`, so PE
compiles against 1.88 headers, while CMake's `find_package(Boost)` resolves the
*libraries* to system Boost 1.75.0 (`/usr/lib64/libboost_*.so.1.75.0`). The result
is a link error:

```
undefined reference to boost::filesystem::detail::path_algorithms::append_v3(...)
```

(`append_v3` is a Boost ≥ 1.84 symbol absent from 1.75.) Fix: build with the
**system Boost only** (do not load the boost module). CGAL 6.0.1 is happy with
Boost ≥ 1.72. Because CMake does not track `CPLUS_INCLUDE_PATH`, switching the
Boost environment does **not** trigger a recompile on its own — force it with
`cmake --build build-mpi --target dbg-parti-velocity-eval --clean-first` after
changing the module set.

### 5.5 Build gotchas already solved (do not re-fight these)

- **gmp/mpfr link into the executable** (fixed in FEAT3 `area51/CMakeLists.txt`,
  commit "Link gmp/mpfr into dbg-parti-velocity-eval after pe_static"):
  `pe_static` (libpe.a) references `__gmp*`/`__mpfr*` from CGAL exact arithmetic in
  `TriangleMesh.cpp`. FEAT3 links gmp/mpfr only PRIVATEly into its own CGAL kernel
  lib, so they do not propagate, and `pe_static` is the last archive on the link
  line. The target now appends gmp/mpfr via a trailing `target_link_libraries`.
  Without it: `undefined reference to __gmpn_mul` etc.
- PE force-includes its generated `config.h` via directory-scoped flags that do
  not reach `area51/`; `area51/CMakeLists.txt` replicates
  `-include ${CMAKE_BINARY_DIR}/pe-0.3rc1/config.h` for this target.
- `kernel/geometry/CMakeLists.txt` links gmp/mpfr explicitly for CGAL's
  exact-arithmetic backend.
- PE's doc target is `pe-doc` (renamed; `doc` collides with FEAT3's).
- PE downloads Eigen/nlohmann-json/CGAL via FetchContent if not found. On an
  air-gapped cluster, pre-install them and point `CMAKE_PREFIX_PATH`/`CGAL_DIR` at
  them.
- PE compiles with a wall of warnings (auto_ptr, binary_function, C++20 `requires`,
  incomplete DistanceMap in inline paths). Pre-existing and benign — ignore them.

## 6. Run, in detail

### 6.1 Stage `example.json`

PE reads `example.json` **from the current working directory**. It is not in git
and not copied into the build tree automatically — stage one next to the binary.
For the 4-rank baseline (process grid 2&times;2&times;1):

```json
{
  "timesteps_": 10,
  "stepsize_": 0.2,
  "substeps_": 2,
  "processesX_": 2,
  "processesY_": 2,
  "processesZ_": 1,
  "vtk_": true,
  "useCheckpointer_": false,
  "benchRadius_": 0.01,
  "particleDensity_": 1.0,
  "domainBoundary_": { "distanceMap": { "resolution": 24, "tolerance": 3 } }
}
```

`vtk_: true` writes a `paraview/collector.pvd` animation; set it `false` for
performance runs.

### 6.2 Launch

```bash
source /etc/profile.d/modules.sh; module purge
module load gcc/13.2.0 openmpi/4.1.6 cgal/6.0.1
export OMPI_MCA_rmaps_base_oversubscribe=1          # if ranks > physical cores

cd build-mpi/area51                                 # must contain example.json
rm -f domain_exits.json temp_surface.off
mpirun --oversubscribe -np 4 ./dbg-parti-velocity-eval \
  --mesh ../../data/meshes/unit-cube-hexa.xml \
  --level 1 \
  --surface-file temp_surface.off \
  --exit-output domain_exits.json
```

- `-np` must equal `processesX_*processesY_*processesZ_` (here 4).
- On a workstation node (no batch scheduler) bare `mpirun` is fine; `--oversubscribe`
  (plus `OMPI_MCA_rmaps_base_oversubscribe=1`) lets you request more ranks than
  cores. On a real cluster, launch under the scheduler and log stdout to a file
  rather than streaming it.
- Larger runs: raise `--level`, grow the process grid in `example.json` in lockstep
  with `-np`, and consider the SCOTCH partitioner for large rank counts.

### 6.3 Expected result (regression baseline, level 1, 4 ranks)

- Boundary OFF: **26 vertices / 48 faces**, closed and consistently oriented;
  **8** gathered cell midpoints as seeds.
- All **8** tracers exit through face **ZMax**, two per rank:
  seeds at z=0.75 exit at step 2 (t &asymp; 0.311, analytic ln(1/0.75) &asymp; 0.288);
  seeds at z=0.25 exit at step 8 (t &asymp; 1.516, analytic ln(4) &asymp; 1.386);
  x/y unchanged (no lateral velocity). **0** survivors.
- `domain_exits.json`: `num_exits: 8`, `num_survivors: 0`, all `face: ZMax`.
- If exits show doubled velocities on some tracers, suspect a reintroduced
  `sync_0()` (section 3, step 2). If PE aborts with `surface must be closed and
  consistently oriented`, FEAT3 was built without CGAL (section 5.2).

Known cosmetic quirk: the JSON `velocity` of an exit is the velocity sampled at the
last step *before* crossing (e.g. ~0.9 instead of 1.0 at z=1). Fine for debugging;
account for it if consumed quantitatively.

## 7. Suggested next steps on a larger MPI system

1. Clone FEAT3, check out `rm_structured_exit_output`, clone PE into `pe-0.3rc1/`,
   build per section 5 (fresh build dir; never switch compilers inside an existing
   cache).
2. Smoke-test the 4-rank baseline (section 6) and diff `domain_exits.json` against
   the expected values before scaling anything.
3. Scale up: raise `--level`, grow the process grid in lockstep with `-np`, use
   SCOTCH for large rank counts, disable `vtk_`.
4. Open questions to probe at scale: seed lists are currently *global* on every
   rank (allgather of all midpoints) — memory/scaling hazard at high rank counts;
   the velocity callback does a collective inverse-mapping per PE step — measure
   its cost; PE's unit-cube domain decomposition vs FEAT3's mesh partitioning are
   independent — tracers usually cross ranks.
