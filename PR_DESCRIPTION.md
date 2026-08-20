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
| 10 | `9cecb08` | `docs`: PR description |
| 11 | `ce66b09` | `docs`: record that the examples build failure predates this branch |

Answering the independent review (see "Review response" below):

| # | Commit | Finding | Concern |
|---|---|---|---|
| 12 | `563a797` | 1 | `review`: make the checkpoint scratch filename rank-invariant |
| 13 | `4a67b14` | 2 | `review`: sum the marshalled body count across ranks for the sidecar |
| 14 | `1b47ce0` | 5, 9c, 9d, 9f | `review`: keep the previous sidecar until the new one is published |
| 15 | `e961823` | 8, 9b | `review`: tighten sidecar parsing and the zero-step-size case |
| 16 | `e2c2c57` | 3 | `review`: validate the `.peb` header before believing any of it |
| 17 | `a09b4b1` | 6, 7, 9a | `review`: refuse a resume expectation declared by both driver and deck |
| 18 | `1bbb114` | 4 | `tests`: exercise the reader's own truncation gate, and add two-rank coverage |
| 19 | `4a63188` | 10, 11, 12, 13 | `review`: rewrite FF-WIRING around the mechanism that actually works |

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

**Fix:** write to a scratch name in the same directory, then `rename(2)`
(`checkpointTempPath()` / `commitCheckpointTempFile()`). Publication order is:
move any stale sidecar aside → write `.peb` scratch → rename `.peb` → write the
new sidecar → drop the stale one. Every intermediate state is either the previous
checkpoint or a sidecar-less one, and a sidecar-less checkpoint is reported loudly
on read. A mismatched (`.peb`, sidecar) pair is never published.

The scratch name's disambiguating token is **passed in**, not sampled inside
`checkpointTempPath()`. That is load-bearing under MPI: `MPI_File_open` is
collective and requires an identical filename on every rank, so a per-rank pid
makes the call erroneous — see review finding 1, which is exactly the bug the first
version of this fix shipped.

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

- The header is validated in the order it is consumed: the fixed 13-byte prefix
  must exist before it is read; the offset table's extent, computed in 64-bit
  arithmetic from the file's own process count, must fit in the file before the
  buffer is resized to it; offsets must be non-decreasing and start at or after the
  global chunk before any unsigned subtraction uses them; and `offsets_[p]`, the end
  of the last chunk, must be within the file. This works for checkpoints written by
  *any* pe version, sidecar or not. (The ordering here is review finding 3 — the
  first version placed the single length check after the header had already been
  believed.)
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

## Review response

An independent review returned REVISIONS REQUIRED with five must-fix findings
(1, 2, 3+4, 10, 11) and eleven should-fix/nits. All are addressed. Two of the
must-fix findings were defects **this PR introduced**, both on the MPI path, and
both are worth naming plainly.

### Finding 1 — every rank used its own pid in the scratch filename (introduced here)

`checkpointTempPath()` sampled `::getpid()` itself, so the atomic-write change made
each rank pass a *different* filename to the collective `MPI_File_open`. Multi-rank
checkpoint writes went from working to hanging. The first version of the atomicity
fix traded one correctness problem for a worse one, and a serial-only suite passed
it without complaint.

The token is now a parameter rather than something the function samples — which puts
the requirement in front of every call site — and `collectiveCheckpointScratchToken()`
broadcasts rank 0's value. The sidecar deliberately keeps a *local* token: it is
written by one rank with plain POSIX calls, and calling the collective helper from
inside a rank-0-only branch would deadlock.

### Finding 2 — `bodyCount` was rank 0's own count (introduced here)

Documented as "body records the `.peb` contains", filled from one rank's tally.
Now reduced across ranks in the checkpoint layer; `BodyBinaryWriter`'s count stays
per-rank by design and its doc comment says so instead of leaving it to be inferred.

### Finding 3 — the length gate ran too late

The `.peb` length check was placed after the process count and offset table had
already been read, so a header-truncated file was parsed out of uninitialised buffer
memory; garbage offsets could pass the check and reach an unsigned subtraction that
wraps into an enormous allocation. Validation now follows the order the header is
consumed: fixed prefix, then offset-table extent in 64-bit arithmetic, then
monotonicity, each before the value it protects is used.

### Finding 4 — the truncation test was not testing the reader

The `.peb`-truncation fixture kept its sidecar, so the throw came from `pebBytes`
and the reader's own gate — the one that exists for legacy checkpoints, which have
no sidecar — was never reached. Now fixtured at three depths with no sidecar, one
per validation stage; each produces a distinct message.

### Findings 10–13 — FF-WIRING was recommending something impossible

The resume ordering it recommended cannot be achieved: `commf2c_*` runs inside each
application's own `General_init_ext`, which runs *before* `SolFromFile`. Deck keys
are now THE mechanism (they work, because `example.json` is parsed inside the same
`commf2c_*` call) and the C entry point is an appendix with the evidence. The tag
recommendation was also backwards — `iOut` is a cyclic slot whose counter resets on
restart, so a checkpoint tagged with it can point at a dump since overwritten by a
different instant.

**Found while verifying the review's claims, and not in the review:** `read_time_sol`
post-increments (`solution_io.f90:1364`), so after a restart `istep_ns` is the dump's
step **plus one**. An expectation built from the in-memory value would be off by one
and fail every resume. Documented, and it is a further argument for building the keys
in the staging script from `time.dmp` directly.

### Should-fix and nits

| # | Disposition |
|---|---|
| 5 | Fixed — stale sidecar is renamed aside and deleted only after the new one publishes, so a kill no longer destroys the previous one. The delete cannot simply move after the rename: a steady-state `.peb` can be byte-identical to its predecessor, so `pebBytes` cannot catch a leftover sidecar. |
| 6 | Fixed — declaring a resume expectation via both the driver call and the deck is refused, not resolved by precedence. Either winner would leave the losing call site reading as though it had taken effect. |
| 7 | Fixed — documented the `type(c_ptr), value` binding the skippable arguments require, in both the entry point's doc comment and FF-WIRING's appendix. |
| 8 | Fixed — `stepSize <= 0` with a time expectation now says the step size is the problem instead of reporting a time mismatch. Covered, including that a step expectation still works. |
| 9a | Fixed — negative step rejected in both entry points rather than wrapping to a huge unsigned value. |
| 9b | Fixed — repeated directives and trailing content both refused; `timeStep 1.5` no longer parses as 1. |
| 9c | Fixed — redundant `writer.wait()` dropped; `writeFile()` already waits. |
| 9d | Fixed — one-line rationale for `std::cerr` over the log section: the warning states that the run's guarantees do not hold and must survive logging being off. |
| 9e | **Documented, not fixed** — see limitation 6. |
| 9f | Fixed — `!counter_ == 0` spelled as `counter_ != 0`, with a comment on what it is for. |

---

## Verification matrix

Every configuration below runs the **complete** suite. Zero skips.

| Configuration | Build | Result |
|---|---|---|
| Neutral solver, Release | `-DCMAKE_BUILD_TYPE=Release` | 13/13 pass |
| Neutral solver, Debug + `_GLIBCXX_ASSERTIONS` | `-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-D_GLIBCXX_ASSERTIONS` | 13/13 pass |
| `HardContactAndFluid`, Release | `-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid` | 13/13 pass |
| `HardContactAndFluid`, Debug + `_GLIBCXX_ASSERTIONS` | both of the above | 13/13 pass |
| MPI, Release, **2 ranks** | `-DPE_USE_MPI=ON` + OpenMPI 4.1.6 | 12/14 pass; the 2 failures are pre-existing, see below |
| `PE_BUILD_EXAMPLES=ON`, Release | neutral solver | unchanged from `de855b6` (see below) |

All with `-DPE_USE_JSON=ON -DPE_USE_EIGEN=ON -DPE_USE_CGAL=OFF`.
Baseline on `de855b6` for comparison: 12/12 Release, **11/12 Debug** —
`pe-interface-serial-atc-resume-roundtrip` aborted.

### MPI: what was executed vs. what was only compiled

Corrected from the first version of this document, which claimed "MPI compiles
clean" on the strength of a configure step that had in fact **failed** — MPI is not
on the default `PATH` here and the `&&` chain never reached `make`. That claim was
wrong and is retracted.

What was actually done, using OpenMPI 4.1.6 from
`/sfw/openmpi/gcc13.2.x/4.1.6/ucx-threaded-noverbs`:

- **Executed, 2 ranks:** `pe-checkpoint-metadata-mpi`, the new two-rank test.
  Passes. Both MPI-path fixes were regression-verified by reverting them
  individually: with the scratch-token broadcast disabled the test **hangs** (the
  ranks never agree on a filename for the collective open); with the body-count
  reduction disabled it reports `bodyCount 3, expected 6`.
- **Executed, 1 rank:** the whole suite in the MPI build, 12/14.
- **Compiled only:** nothing in the checkpoint path. The MPI branches in
  `storeCheckpoint()`, `totalMarshalledBodyCount()`,
  `collectiveCheckpointScratchToken()` and the reader's `MPI_File_get_size()` gate
  are all executed by the two-rank test.
- **Not reachable at all:** pe's domain decomposition. `defineLocalDomain()` throws
  "Selected configuration is not MPI parallel" for *every* configuration, because
  `pe/core/ParallelTrait.h` contains no specialisation setting `value = 1` — the
  specialisation block is empty. Pre-existing and out of scope; the two-rank test
  works without it, since the checkpoint write is collective over `MPI_COMM_WORLD`
  and each rank contributes its own local bodies.

**The 2 MPI-config failures are pre-existing.** `pe-interface-serial-atc-resume-roundtrip`
and `pe-interface-serial-fluidization-srr-resume-roundtrip` abort with
"MPI_Exscan() was called before MPI_INIT" — the checkpoint write path is collective
even at size 1, and the smoke harness never initialises MPI. Verified by building
`de855b6` in the same MPI configuration in a detached worktree: **the same two tests
fail there**, 10/12. This PR's own tests do not have the problem — the new serial
test calls `MPI_Init` when `HAVE_MPI` — but the harness was not changed to match,
because it `fork`/`execv`s a seed child and initialising MPI before a fork needs more
care than a drive-by deserves. Follow-up below.

The examples build fails identically before and after this PR — three
pre-existing errors under a neutral solver, verified by building `de855b6` in a
detached worktree with the same flags: `examples/box_domain` calls
`setSlipLength()`, removed when the `HardContactLubricated` pipeline was retired,
and `examples/lubrication_demo` static-asserts that it needs
`HardContactAndFluid`. Neither is touched here. Noted so the failure is not
mistaken for a regression.

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

2. **The (`.peb`, sidecar) pair is still not atomic across a kill.** Two files
   cannot be renamed atomically together. The window is now confined to "new
   `.peb`, sidecar renamed aside but not yet republished", which reads as a legacy
   checkpoint — loud, never a wrong pairing — and the previous sidecar survives
   under a `.tmp-<pid>` name instead of being destroyed (review finding 5). Smaller
   than before, not gone. Closing it fully would require a nonce inside the `.peb`,
   i.e. a format break.

3. **Two MPI-config tests fail, both pre-existing.** See the MPI section of the
   verification matrix: `pe-interface-serial-{atc,fluidization-srr}-resume-roundtrip`
   abort with "MPI_Exscan() called before MPI_INIT", verified identical on
   `de855b6`. This PR's own tests initialise MPI and pass; the shared harness was
   not changed because of its `fork`/`execv` seed child.

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

6. **A throw from one rank inside `storeCheckpoint()` deadlocks the others**
   (review finding 9e). The barriers and the reduction are unconditional, so a rank
   that throws — a full disk, a failed rename — leaves its peers waiting. Fixing it
   properly means collective error propagation across the checkpoint path, which
   would be the first such mechanism in the library and is a larger change than this
   PR should make. Documented rather than half-done.

7. **The FeatFloWer side is not implemented.** By design — see `FF-WIRING.md`.
   Until it lands, checkpoints record `timeSource pe-timestep` (pe's substep
   counter) rather than a driver time, which is honest but not pairable.

8. **`setupDNSDragSerial` has no resume branch at all**, so none of this applies to
   the DNS drag application until one is added. This PR supplies the validation half
   of a mechanism whose loading half does not exist for that case.

---

## Follow-ups

- **Land the FF side** per `FF-WIRING.md`. The one ordering constraint worth
  repeating: build the expectation keys in the staging script from `time.dmp`
  directly, because the in-memory `istep_ns` is the dump's step **plus one** after
  `read_time_sol`'s post-increment.
- **Add a resume branch to `setupDNSDragSerial`** (limitation 8), following
  `setupFluidizationSRRSerial`: materials first, then
  `resumeFromConfiguredCheckpoint(config)`.
- **Initialise MPI in the serial smoke harness**, which would fix the two
  pre-existing MPI-config failures. Needs care around the harness's `fork`/`execv`
  seed child; the pattern is already in `pe_checkpoint_metadata_test.cpp`.
- **Fix the frozen-field MPI test's guard.** It reads `if(MPI AND CGAL)`, and
  nothing in this project ever sets a variable named `MPI`, so that test has never
  been built. Found while adding the two-rank test, which uses
  `PE_USE_MPI AND MPI_FOUND` instead. Left alone rather than enabled as a drive-by,
  since a test that has never run may well not pass.
- **`ParallelTrait` has no specialisation setting `value = 1`**, so
  `defineLocalDomain()` throws for every configuration and pe's domain decomposition
  is unreachable. Far outside this PR, but it is why the two-rank test does not use
  a decomposed domain, and it deserves an issue.
- **Collective error propagation in the checkpoint path** (limitation 6).
- **Checkpoint naming.** `checkpoint.<counter>` still restarts at zero each run,
  so a resumed run overwrites its predecessor's checkpoints. The sidecar makes
  that detectable but does not prevent it; naming by step index would.
- **`resumeCheckpointFile_` defaults to `"../start.1"`**, a path that exists in
  no rundir. Harmless while `resume_` is false everywhere, but it is a trap.
- **Serialise the material table into a future `.peb` format version**, retiring
  the sidecar's table half, once a format break is acceptable for other reasons.
