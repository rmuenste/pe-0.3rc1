# PE Serial Mode and the CFD Coupling Interface

This note explains `PE_SERIAL_MODE`, how it differs from PE's "normal" parallel
(domain-decomposition) mode, and the conventions for writing interface setup
functions in `pe/interface/` that are called from an external CFD solver across
a Fortran/C boundary.

If you are adding a new coupled simulation case (a new `setupXxx` entry point),
read the [Writing a new interface setup function](#writing-a-new-interface-setup-function)
recipe at the end.

## Two modes, two sets of interface functions

PE can be driven from an external CFD solver in two different ways. They use
**separate** interface functions, selected at build time:

| | Normal (parallel) mode | `PE_SERIAL_MODE` |
|---|---|---|
| Build flag | `MPI=ON` (`HAVE_MPI=1`) | `MPI=OFF` (`HAVE_MPI=0`) + `PE_SERIAL_MODE` |
| Domain decomposition in PE | Yes — bodies are distributed across MPI processes | No — every PE instance holds **all** bodies |
| Setup functions | `setupXxx(MPI_Comm)` in `pe/interface/sim_setup.h` (e.g. [setup_atc.h](../../pe/interface/setup_atc.h)) | `setupXxxSerial(int cfd_rank)` in [sim_setup_serial.h](../../pe/interface/sim_setup_serial.h) |
| Body ownership | A body is owned by one process; remote copies are synchronized via PE's MPI layer | No ownership concept; each instance simulates everything |

### What `PE_SERIAL_MODE` is, and why it exists

In `PE_SERIAL_MODE` each CFD rank runs its **own independent, serial PE
instance**, built with `PE_USE_MPI=OFF`. There is no domain decomposition *inside* PE:
every PE instance creates and simulates the full set of rigid bodies.

This exists primarily as a **debugging and bring-up aid**. Chasing a
particle/rigid-body problem (a bad contact, a wrong force, a geometry/mesh
issue) inside a fully parallel, domain-decomposed run is hard: the bodies are
split across processes, migrate between domains, and bugs entangle with the
MPI synchronization logic. `PE_SERIAL_MODE` removes that entire axis of
complexity — the physics is reproduced in one process where it can be inspected
and stepped directly. Once a case behaves correctly in serial mode, it can be
moved to normal (parallel) mode with much higher confidence that any remaining
issues are in the decomposition/synchronization layer rather than in the body
setup or contact handling itself.

## The `commf2c_*` Fortran/C entry-point convention

The CFD solver calls into PE through `extern "C"` functions named
`commf2c_<case>_`. These live in [src/interface/c2f_interface.cpp](../../src/interface/c2f_interface.cpp)
and the file defines **each case twice**, guarded by build mode:

```cpp
#if HAVE_MPI                       // ── normal (parallel) mode ──────────────
extern "C" void commf2c_atc_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  // converts the Fortran communicator and calls the parallel setup:
  MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0);
  setupATC(CcommEx0);
}
#endif

#ifdef PE_SERIAL_MODE              // ── serial mode ─────────────────────────
extern "C" void commf2c_atc_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  pe::setupATCSerial(*remoteRank); // calls the serial setup
}
#endif
```

Key points when adding a case:

- **Add the symbol in both halves.** The parallel half (`#if HAVE_MPI`) takes
  `MPI_Fint*` communicators and calls the parallel `setupXxx(MPI_Comm)`; the
  serial half (`#ifdef PE_SERIAL_MODE`) takes plain `int*` and calls
  `setupXxxSerial(*remoteRank)`.
- **A case may be serial-only.** Several parallel-half entry points are
  deliberately *stubbed* — they print an error telling the user to rebuild with
  `PE_SERIAL_MODE` (e.g. `commf2c_rotation_`, `commf2c_drill_`,
  `commf2c_lubrication_lab_`). This is fine; not every case needs a parallel
  implementation. But the symbol must still exist in both halves so the CFD
  side links in either build.

## `remoteRank` vs `cfd_rank` vs `getCfdRank()` — same value, three names

The CFD process rank travels through three names; this is purely historical and
easy to misread:

1. At the Fortran/C boundary it arrives as `int *remoteRank` (a **1-based** MPI
   rank from the CFD communicator: 1, 2, 3, …, N).
2. The serial setup function receives it as its `int cfd_rank` parameter and
   stores it: `config.setCfdRank(cfd_rank)`.
3. Code inside the setup reads it back with `config.getCfdRank()`.

They are all the same integer. Treat `remoteRank`, `cfd_rank`, and
`getCfdRank()` as aliases for "this CFD rank".

## The representative-rank pattern

Because every CFD rank runs its own PE instance, any *side effect* — console
logging, VTK output, checkpoint writing — would otherwise happen N times (once
per rank). To do these exactly once, PE designates one rank as the
**representative**:

```cpp
config.setCfdRank(cfd_rank);
const bool isRepresentative = (config.getCfdRank() == 1);   // only CFD rank 1
```

Every I/O-producing or once-only operation is then guarded:

```cpp
// ✅ only the representative prints / writes / checkpoints
if (isRepresentative && config.getUseCheckpointer()) {
  activateCheckpointer(...);          // otherwise: "Checkpoint:checkpoint.0" ×N
}
if (isRepresentative && config.getVtk()) {
  activateWriter(...);
}
if (isRepresentative) {
  std::cout << " ... setup summary ... ";
}
```

This idiom is pervasive: `const bool isRepresentative = (config.getCfdRank() == 1);`
appears in every serial setup function in
[sim_setup_serial.h](../../pe/interface/sim_setup_serial.h), with dozens of
guarded statements. When you write a new setup function, follow it for **all**
output and persistence.

### Known wart: the hardcoded `== 1`

The representative test is a bare magic number (`getCfdRank() == 1`) duplicated
across ~10 setup functions. This is a bit smelly — the "which rank is
representative" policy is scattered rather than expressed in one place. A
`SimulationConfig::isRepresentative()` helper would centralize it and remove the
repeated literal. Until that refactor exists, keep using `== 1` so new code
stays consistent with the rest of the file, but be aware it is a candidate for
cleanup.

## Force synchronization (why serial-per-rank still gives correct forces)

The CFD solver itself is domain-decomposed. A single rigid body / particle can
overlap **several** CFD subdomains at once, and each of those subdomains
computes only the share of the hydrodynamic force it sees within its own part of
the flow field. No single CFD rank has the complete force on such a body.

Because each PE instance operates on the full body set, the partial forces from
all overlapping subdomains must be combined: the CFD layer performs an **MPI
summation (reduction) across ranks** so that, after synchronization, every PE
instance holds the *true* total force/torque for each body. PE then integrates
with that synchronized value.

This is why the per-rank-serial design is correct rather than redundant: PE does
the bookkeeping locally and identically on every rank, and the CFD layer's
collective reduction is what makes the independently-computed contributions add
up to the right answer. This design is well-suited to large bodies (roughly
< 20 particles) that genuinely span multiple CFD domains.

> Concrete example: in the FeatFloWer coupling this reduction is performed by the
> CFD-side collective `COMM_SUMMN`. That symbol lives in FeatFloWer, not in this
> repository — it is named here only to make the concept concrete for that
> integration.

## Handling variant-dependent collision-system parameters

The collision system is selected **at compile time** through `pe_CONSTRAINT_SOLVER`
in [pe/config/Collisions.h](../../pe/config/Collisions.h). Different solver
variants expose different parameter APIs: for example only `HardContactLubricated`
provides `setContactHysteresisDelta`, `setLubricationHysteresisDelta`,
`setAlphaImpulseCap`, and `setMinEpsLub`, while the default
`HardContactEulerLagrange` does not.

A setup function that wants to push these knobs must therefore compile against
*any* configured solver — applying them when the type supports them and silently
skipping them otherwise. The robust way to express this in PE (C++17) is a single
`if constexpr` guarded by a detection trait:

```cpp
// Detection trait: does this collision-system type expose the lubrication setters?
template <typename CS, typename = void>
struct HasLubricationParamSetters : std::false_type {};

template <typename CS>
struct HasLubricationParamSetters<CS, std::void_t<
    decltype(std::declval<CS&>().setContactHysteresisDelta(real{})),
    decltype(std::declval<CS&>().setMinEpsLub(real{}))>>  // …all required setters
    : std::true_type {};

template <typename CS>
inline void applyOptionalLubricationParams(CS& cs, const SimulationConfig& config) {
  if constexpr (HasLubricationParamSetters<CS>::value) {
    cs.setContactHysteresisDelta(config.getContactHysteresisDelta());
    // …apply the rest…
  } else {
    (void)cs; (void)config;  // solver has no such controls; nothing to do
  }
}
```

The live implementation is `applyOptionalLubricationParams` /
`HasLubricationParamSetters` in
[sim_setup_serial.h](../../pe/interface/sim_setup_serial.h).

### Why not a two-overload SFINAE pair

A tempting alternative is two overloads with the same parameter list — one with a
trailing `decltype(cs.setX(real{}), …, void())` return (viable only if the
setters exist) and a plain fallback:

```cpp
// ⚠️ FRAGILE — do not copy
template <typename CS>
auto applyOptionalLubricationParams(CS& cs, const SimulationConfig&)
    -> decltype(cs.setContactHysteresisDelta(real{}), /*…*/ void());   // (A)
template <typename CS>
void applyOptionalLubricationParams(CS&, const SimulationConfig&);     // (B)
```

This *looks* like it does the job, and it compiles **as long as only one overload
is ever viable** (e.g. while the configured solver lacks the setters). But (A)
and (B) have **identical parameter lists** and differ only in return type, which
does not participate in overload partial ordering. The moment a solver provides
all the setters — i.e. exactly the `HardContactLubricated` case the helper exists
to support — *both* overloads become viable and the call is **ambiguous**, so the
build breaks in the one configuration it was meant to keep working.

If you must stay C++14-compatible (no `if constexpr`), use **priority-tag
dispatch** instead of identical signatures: give the SFINAE implementation an
extra higher-priority tag argument (`int`) and the fallback a lower-priority one
(`long`), and have the public function call the implementation with `0`. The tag
breaks the tie deterministically in favor of the SFINAE overload.

## Writing a new interface setup function

To add a new serial coupled case `xyz`:

1. **Serial setup** — add `inline void setupXyzSerial(int cfd_rank)` in
   [sim_setup_serial.h](../../pe/interface/sim_setup_serial.h). Inside:
   - `pe::logging::Logger::setCustomRank(cfd_rank);`
   - load config, then `config.setCfdRank(cfd_rank);` and derive
     `isRepresentative`.
   - configure `theWorld()` / `theCollisionSystem()`, create materials and
     bodies (every instance creates **all** bodies).
   - guard every `std::cout`, VTK writer, and checkpointer with
     `isRepresentative`.
   - See `setupSpanComplexSerial` (added alongside the SpanComplex smoke test)
     as a clean, current worked example: it builds the tool/chip/workpiece
     meshes, enables DistanceMap acceleration, and prints a representative-only
     setup summary.
2. **Entry point** — add `commf2c_xyz_` in **both** guarded halves of
   [c2f_interface.cpp](../../src/interface/c2f_interface.cpp): a real parallel
   implementation (or an explicit "serial only" stub) under `#if HAVE_MPI`, and
   the `pe::setupXyzSerial(*remoteRank)` forwarder under `#ifdef PE_SERIAL_MODE`.
3. **Smoke test** (recommended) — add a case to
   [tests/interface/pe_interface_smoke_serial.cpp](../../tests/interface/pe_interface_smoke_serial.cpp)
   and register it in `tests/interface/CMakeLists.txt`. See
   [tests/interface/README.md](../../tests/interface/README.md).

## Related notes

- [simulation-setup-examples.md](simulation-setup-examples.md) — serial and MPI
  setup patterns for standalone (non-CFD-coupled) simulations.
- [domain-decomposition-geometries-and-definition.md](domain-decomposition-geometries-and-definition.md)
  — process/domain geometries and CFD coupling implications for the parallel mode.
- [build-with-cmake.md](build-with-cmake.md) — build flags, including MPI/CGAL.
