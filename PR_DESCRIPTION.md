# Checkpoint identity: make continuing runs pairable and resumable

Owner's statement of the problem:

> pe dumps have no time information in them, and it is hard to pair them with
> their FeatFloWer counterpart.

Branch: `feature/checkpoint-metadata`, based on `master` (`de855b6`, the merged
lubrication stack).

---

## What changes, in one paragraph

A pe checkpoint used to be a `.peb` file containing body state and nothing else —
no simulation time, no step index, no dt, no material table, no length — named
`checkpoint.<counter>` with a counter that restarts at zero every run. Two
checkpoints from different runs were indistinguishable, none could be paired with
the FeatFloWer `_dump` it belonged to, a job killed mid-write left a truncated
file that the reader half-read without complaint, and resuming into a process
whose material table did not happen to match the writer's read body densities out
of bounds. After this PR every checkpoint carries a human-readable sidecar with
its identity and its material table, is published atomically, is rejected if
truncated, reinstates its own materials before instantiating anything, and can be
made to hard-fail when the driver says it expected a different instant. Legacy
checkpoints still load, loudly.

---

## Commits

Each is one concern.

| # | Commit | Concern |
|---|---|---|
| 1 | `4b8614c` | `core`: expose the material table size and the marshalled body count |
| 2 | `6aa84d2` | `core`: reject a truncated rigid body parameter file instead of half-reading it |
| 3 | `b53f701` | `util`: checkpoint identity metadata as a human-readable sidecar |
| 4 | `1c40c65` | `util`: Checkpointer writes metadata, publishes atomically, restores materials |
| 5 | `ff8a059` | `config`: optional resume expectation keys |
| 6 | `07c40d0` | `interface`: guard the serial resume path and fix the ATC material ordering |
| 7 | `628fb6f` | `interface`: C entry points for driver-supplied checkpoint identity |
| 8 | `ab18849` | `tests`: checkpoint metadata contract, and route seeds through `writeCheckpoint()` |
| 9 | `76be00c` | `docs`: `FF-WIRING.md` — what FeatFloWer must do to pair its dumps with pe |

---

## The four defects, and what each fix is

### 1. No identity (the reported problem)

`Checkpointer::trigger()` named files `checkpoint.<counter_>` where `counter_` is
a per-process counter starting at zero (`src/util/Checkpointer.cpp`). The `.peb`
header (`src/core/BodyBinaryWriter.cpp:96-112`) holds a magic number, a format
version, five type sizes, the process count and the chunk offset table. Nothing
about *when*.

**Fix:** a sidecar `<name>.peinfo` next to `<name>.peb`
(`pe/util/CheckpointMetadata.h`, `src/util/CheckpointMetadata.cpp`) carrying
layout version, simulation time, step index, step size, body count, `.peb` size,
a free-form driver pairing tag and the full material table. Human-readable, one
directive per line:

```
# pe checkpoint metadata -- sidecar of the companion .peb file
metadataVersion 1
timeSource driver
simulationTime 1.20000000000000000e+01
timeStep 2400
stepSize 5.00000000000000010e-03
bodyCount 1204
pebBytes 483920
pairingTag ff:istep=2400:iout=3
materialCount 7
material 0 7.87399999999999967e+00 ... iron
```

**Why a sidecar and not the `.peb`.** `BodyBinaryReader` rejects any format
version it does not know exactly (`src/core/BodyBinaryReader.cpp:109-110`).
Extending the `.peb` would make every new checkpoint unreadable by existing
builds *and* would not help with the existing ones. A sidecar is additive in both
directions: older builds ignore it, this build reads a sidecar-less checkpoint as
"legacy".

Time and step come from the driver when it supplies them
(`setCheckpointIdentity()`, reachable from Fortran as
`set_pe_checkpoint_identity_`), otherwise from pe's own `TimeStep` counter. Which
one was used is recorded in `timeSource` rather than papered over — pe's counter
counts substeps and does not identify a driver step.

### 2. Non-atomic writes

`writeFileAsync()` opened the final path directly (`MPI_MODE_CREATE`, or a
truncating `ofstream`). A job killed mid-write left a short file *under the real
name*.

**Fix:** write to a pid-suffixed scratch name in the same directory, then
`rename(2)` (`checkpointTempPath()` / `commitCheckpointTempFile()`). Publication
order is: drop any stale sidecar → write `.peb` scratch → rename `.peb` → write
sidecar. Every intermediate state is either the previous checkpoint or a
sidecar-less one, and a sidecar-less checkpoint is reported loudly on read. A
mismatched (`.peb`, sidecar) pair is never published.

The write is synchronous now. It already effectively was — `trigger()` called
`flush()` on the line after `write()` — so nothing is lost, and direct
`Checkpointer::write()` callers no longer get a file that may still be in flight
when the call returns.

### 3. Truncated files were half-read

`BodyBinaryReader::readFile()` trusts the header's chunk offsets as read
positions and never checks the file is long enough. Neither the MPI nor the
`ifstream` path inspects the result of the reads. The reader's own comment said
what happens: assertions in debug, undefined behaviour in release.

**Fix, two layers:**

- `offsets_[p]` is the end of the last chunk, i.e. exactly the length a complete
  file must have. One comparison against the file size covers it, and works for
  checkpoints written by *any* pe version, sidecar or not.
- For checkpoints with a sidecar, the recorded `pebBytes` must match the file
  size exactly — which also catches a sidecar that belongs to a different
  checkpoint.

A sidecar missing a required directive is likewise treated as truncation: that is
what a half-written sidecar looks like.

### 4. The material defect (the ATC resume Debug assertion)

**Root cause, confirmed by reproduction.** Bodies serialise their material as a
bare index into the process-global material table
(`pe/core/Marshalling.h:298`, `buffer << obj.getMaterial()`). The table is *not*
in the checkpoint. `setupATCSerial()` called `readCheckpoint()` **before**
registering `"boundary"` and `"particleMaterial"`, so the restored bodies
referenced indices 5 and 6 against a table holding only the five built-ins
(`iron, copper, granite, oak, fir`). Instantiating a body calls
`Material::getDensity()` to compute its mass, which asserts
`material < materials_.size()`.

Reproduced on `de855b6`, Debug + `_GLIBCXX_ASSERTIONS`:

```
pe_interface_smoke_serial: pe/core/Materials.h:439: static pe::real
pe::Material::getDensity(pe::MaterialID): Assertion `( material <
materials_.size() ) || ASSERT_MESSAGE( "Invalid material ID" )' failed.
7/12 Test #7: pe-interface-serial-atc-resume-roundtrip ... Subprocess aborted
```

`setupFluidizationSRRSerial()` registers its materials *before* reading and is
correct today — which is why only the ATC case failed. `setupParticleBenchSerial()`
has the same defect in a different shape: its materials are created only on the
fresh-start branch, so any resume of that setup hits it.

Worth stating plainly: in **Release** this is not a crash, it is an
out-of-bounds `std::vector` read. The body silently gets whatever density is in
memory past the end of the table, and mass and inertia are computed from it. A
resumed run could differ from its continuation with no diagnostic at all.

**Fix, two layers:**

- *General.* The sidecar carries the material table, and `readCheckpoint()`
  reinstates it at identical indices before any body is instantiated. Materials
  already registered under the same name are reused after their properties are
  verified unchanged; missing ones are appended. This makes setup ordering
  non-load-bearing for every checkpoint written from now on.
- *Specific.* `setupATCSerial()` and `setupParticleBenchSerial()` now register
  their materials before the resume, on every path, matching the SRR pattern that
  was already correct. Legacy checkpoints have no table, so the ordering still has
  to be right for them; both places say so in a comment.

A material whose recorded properties differ from the live one is **refused**, not
reconciled. Mass and inertia are not serialised — they are recomputed from the
material density at instantiation — so a quietly different density is a quietly
different simulation. The error names the property and both values.

---

## The resume guard

`resumeFromConfiguredCheckpoint()` (`pe/interface/sim_setup_serial.h`) replaces
the three bare `readCheckpoint()` calls in the serial setups. It loads the
configured checkpoint and enforces the deck's expectation against it.

Four new optional deck keys, all opt-in:

| Key | Meaning |
|---|---|
| `resumeExpectedTime_` | Simulation time the checkpoint must carry |
| `resumeExpectedStep_` | Step index; negative = unchecked |
| `resumeExpectedTag_` | Pairing tag; empty = unchecked |
| `resumeTimeToleranceSteps_` | Time window in units of the checkpoint's own `stepSize`; default 0.5 |

With none of them present the behaviour is byte-identical to before.

The tolerance is expressed in steps, not seconds, and defaults to half a step —
the widest window that still identifies a unique step, which is exactly what a
pairing check must establish. Driver and pe both accumulate time by repeated
addition of the same dt, so they agree to within rounding and never exactly;
demanding equality would fail every real resume, and a fixed epsilon in seconds
would be a magic constant that silently means different things at different dt.

A set expectation that *cannot* be checked, because the checkpoint is legacy, is
an error rather than a silent pass. An expectation quietly dropped is worse than
no expectation.

---

## Verification matrix

Every configuration below runs the **complete** suite. Zero skips.

| Configuration | Build | Result |
|---|---|---|
| Neutral solver, Release | `-DCMAKE_BUILD_TYPE=Release` | 13/13 pass |
| Neutral solver, Debug + `_GLIBCXX_ASSERTIONS` | `-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-D_GLIBCXX_ASSERTIONS` | 13/13 pass |
| `HardContactAndFluid`, Release | `-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid` | 13/13 pass |
| `HardContactAndFluid`, Debug + `_GLIBCXX_ASSERTIONS` | both of the above | 13/13 pass |
| MPI, Release | `-DPE_USE_MPI=ON`, `pe_static` | compiles clean |

All with `-DPE_USE_JSON=ON -DPE_USE_EIGEN=ON -DPE_USE_CGAL=OFF`.
Baseline on `de855b6` for comparison: 12/12 Release, **11/12 Debug** —
`pe-interface-serial-atc-resume-roundtrip` aborted.

### The acceptance test

`pe-interface-serial-atc-resume-roundtrip`, Debug + `_GLIBCXX_ASSERTIONS`:
**aborted before → passes now.** It is a genuine regression test rather than a
test that merely stopped crashing, because the seed now writes its checkpoint
through the shipped `writeCheckpoint()` — the same path a real run uses, sidecar
and all — instead of a bare `BodyBinaryWriter`.

### New test: `pe-checkpoint-metadata`

Covers the five properties the sidecar exists to provide.

| Property | What is asserted |
|---|---|
| Roundtrip | Every field survives write→read; reals compare **exactly** (max_digits10); free-form tag and material name keep their spaces; an empty tag reads back as empty, not as a missing directive |
| Legacy acceptance | A missing sidecar is *reported*, not thrown on; a real `.peb` with its sidecar deleted still loads through `readCheckpoint()` and reports `present == false` |
| Truncation rejection | Half-written sidecar; sidecar promising more materials than it contains; sidecar declaring a future layout version; `.peb` truncated to half its length |
| Material restore | Full table recorded incl. custom materials; reinstating an already-matching table is a no-op and does not duplicate; a drifted density is refused |
| Mismatch guard | Eight cases asserted in **both directions** — no expectation, matching step, step off by one, exact time, time inside the window, time outside it, matching tag, different tag — plus: an expectation against a legacy checkpoint must be refused |

The guard test is deliberately two-directional so it cannot degenerate into
"throws always" or "throws never". It links the shipped `pe_static`, not a
solver variant: the checkpoint path is solver-independent and must hold in the
default configuration a user actually gets.

Atomicity is checked structurally — after a write, no `.tmp-` scratch file
remains in the checkpoint directory.

---

## Known limitations

1. **Legacy checkpoints get no guarantees.** No time, no step, no material
   table, no truncation detection beyond the header length check. They load with
   a warning that spells out exactly which guarantees are unavailable and why.
   This is the specified behaviour, but it means the material defect above is
   still reachable for a legacy checkpoint whose resume path registers materials
   late — which is why commit 6 fixes the ordering as well as the mechanism.

2. **The (`.peb`, sidecar) pair is not atomic as a unit.** Two files cannot be
   renamed atomically together. The publication order confines the failure window
   to "new `.peb`, no sidecar" — a loud legacy warning — never to a silently
   mispaired sidecar. Closing it fully would require a nonce in the `.peb`, i.e.
   a format break.

3. **MPI is compile-verified only.** The MPI paths in `storeCheckpoint()`
   (barrier + rank-0 rename) and in the reader's `MPI_File_get_size()` length
   check build clean but are not exercised by any test — the suite has no
   multi-rank checkpoint case, and did not before this PR either. The serial
   interface, which is what the DNS campaign runs, is fully covered.

4. **`bodyCount` is descriptive, not a gate.** `World::size()` is not a
   write/read invariant (non-global planes are not persisted at all; global
   bodies are written by rank 0 only), so comparing world sizes across a
   checkpoint raises false alarms — the first draft of this PR did exactly that
   and broke the SRR resume test on a 1204 vs 1207 difference. The count now
   records what the *writer* marshalled, and byte-exact `pebBytes` is the
   integrity gate instead. That is the stronger check anyway.

5. **Not serialised, and still not:** RNG state, the contact cache, and
   TriangleMesh distance maps. The ATC setup already discards restored meshes and
   rebuilds them for this reason. Out of scope here.

6. **The FeatFloWer side is not implemented.** By design — see `FF-WIRING.md`.
   Until it lands, checkpoints record `timeSource pe-timestep` (pe's substep
   counter) rather than a driver time, which is honest but not pairable.

---

## Follow-ups

- **Land the FF side** per `FF-WIRING.md`. The one ordering constraint worth
  repeating: `SolFromFile()` must restore `timens`/`istep_ns` *before* the
  expectation is built, or every resume fails against `SimPar@StartSimTime`.
- **A multi-rank checkpoint test.** Limitation 3 predates this PR but is now the
  largest uncovered surface in the checkpoint path.
- **Checkpoint naming.** `checkpoint.<counter>` still restarts at zero each run,
  so a resumed run overwrites its predecessor's checkpoints. The sidecar makes
  that detectable but does not prevent it; naming by step index would.
- **`resumeCheckpointFile_` defaults to `"../start.1"`**, a path that exists in
  no rundir. Harmless while `resume_` is false everywhere, but it is a trap.
- **Serialise the material table into a future `.peb` format version**, retiring
  the sidecar's table half, once a format break is acceptable for other reasons.
