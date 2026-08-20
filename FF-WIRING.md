# FF-WIRING.md — what FeatFloWer must do to pair its dumps with pe checkpoints

Scope: the pe side of checkpoint identity is complete and on `feature/checkpoint-metadata`.
This note specifies the FeatFloWer changes that finish the loop. **Nothing here is implemented
in FeatFloWer; the FF side lands separately.**

The whole point is one property:

> A pe checkpoint and the FeatFloWer `_dump` it belongs to must be provably the same instant, or
> the run must refuse to start.

Today that pairing rests on the operator's memory, and §4 below shows it is not even a stable
thing to remember: FF's dump slots are cyclic and get silently reused across a restart.

Every file:line reference below was verified against the tree at the time of writing.

---

## 1. Write side — one call per time step

pe exposes, next to the existing `set_pe_timestep_`
(`src/interface/object_queries.cpp`):

```c
void set_pe_checkpoint_identity_(const double *simTime, const int *step, const char *tag);
```

`tag` is a **null-terminated** C string, or `NULL`. Call this once per time step, before pe can
write a checkpoint — next to the existing `set_pe_timestep_` call at the top of the time loop.
This one is straightforward: unlike the resume side, the write side has no ordering problem.

```fortran
use, intrinsic :: iso_c_binding

interface
  subroutine set_pe_checkpoint_identity(simTime, step, tag) &
      bind(C, name='set_pe_checkpoint_identity_')
    import :: c_double, c_int, c_char
    real(c_double), intent(in) :: simTime
    integer(c_int), intent(in) :: step
    character(kind=c_char), dimension(*), intent(in) :: tag
  end subroutine
end interface

character(len=64) :: cTag
write(cTag,'(A,I0)') 'ff:istep=', istep_ns
call set_pe_checkpoint_identity(timens, istep_ns, trim(cTag)//c_null_char)
```

Pass **the same `timens` and `istep_ns` that `write_sol_to_file` puts into `time.dmp`**
(`source/postprocessing/solution_io.f90:56` → `write_time_sol` → `write_sol_time`, which writes
time then step at `FullC0ntact/inshape3dcore/fortrancppinterface/io_func.hpp:294-295`). That is
the entire contract: identical numbers on both sides.

A negative step is refused rather than converted — it would wrap to an enormous unsigned value
and land in the sidecar looking like a plausible step index.

Without this call pe falls back to `TimeStep::step() * TimeStep::size()`, which counts pe
substeps and does **not** identify an FF step. The fallback is recorded honestly — the sidecar's
`timeSource` field reads `pe-timestep` instead of `driver` — but it cannot be paired.

### Use `istep_ns` as the tag, not `iOut`

`iOut` looks like the obvious pairing value — it is the number the operator later puts into
`SimPar@StartFile` — and it is the wrong one. It is a **cyclic slot**, not an identity
(`source/postprocessing/solution_io.f90:43-47`):

```fortran
if(.not.present(output_idx))then
  ifile = ifile+1
  iout=mod(ifile+imax_out-1,imax_out)+1
else
  iout = output_idx
end if
```

`imax_out` is `SimPar@BackUpNum` (`source/src_util/param_parser.f90:1022-1023`, reaching
`write_sol_to_file` via `source/postprocessing/post_utils.f90:158`). Two consequences:

- **Slots are reused.** With `BackUpNum = 10`, dump 3 and dump 13 are the same directory.
- **The counter resets on restart.** `ifile` is a module variable initialised to 0
  (`solution_io.f90:10`) and is never restored from a dump — it appears only at lines 10, 44, 45,
  146 and 147 of that file, and nothing reads it back. A restarted run therefore begins writing
  at slot 1 again and overwrites whatever the previous run left there.

So a pe checkpoint tagged with `iOut` can point at an FF dump that has since been overwritten by
a *different instant* under the same name — the exact silent mispairing this work exists to
prevent. `istep_ns` is monotone within a run and matches what `time.dmp` records, so tag with
that. Keep `iOut` in the tag only as extra human-readable context, never as the thing being
matched.

This overwrite hazard is also the strongest argument for the expectation keys in §2: they turn
"the dump under this name is not the one the checkpoint came from" from an invisible corruption
into a startup error.

---

## 2. Resume side — deck keys (the mechanism)

**Set the expectation in `example.json`.** This is the mechanism that works; the alternative
entry point in Appendix A does not fit the current application structure.

All keys optional; absent keys preserve today's behaviour exactly.

| Key | Type | Meaning |
|---|---|---|
| `resumeExpectedTime_` | real | Simulation time the checkpoint must carry |
| `resumeExpectedStep_` | integer | Step index the checkpoint must carry; negative = unchecked |
| `resumeExpectedTag_` | string | Pairing tag the checkpoint must carry; empty = unchecked |
| `resumeTimeToleranceSteps_` | real | Time window in units of the checkpoint's own `stepSize`; defaults to 0.5 |

These work because `example.json` is parsed *inside* the same `commf2c_*` call that performs the
resume (`SimulationConfig::loadFromFile`), so the values are in place before pe reads the
checkpoint. Nothing in FF writes or patches `example.json` at runtime — every reference to it is
a CMake configure-time copy — so the staging script is the sole author.

**Where to set them: the staging script.** A script that copies `_dump/processor_*/<N>/` into
place already knows `<N>`. It should read `time.dmp` from that directory — line 1 is the
simulation time, line 2 is the step index — and write those two numbers into `example.json`.
That is the least error-prone place, because it is the same step that chooses
`SimPar@StartFile`, and because the values go in **exactly as `time.dmp` records them**:

> The raw `time.dmp` step is what pe recorded. Do **not** use the in-memory `istep_ns` after a
> restart: `read_time_sol` post-increments it (`source/postprocessing/solution_io.f90:1364`,
> `istep = istep + 1`) so that it names the next step to compute. Building an expectation from
> the in-memory value would be off by one and fail every resume. Reading the file directly
> sidesteps this entirely — another reason the staging script is the right home.

Setting an expectation against a **legacy** pe checkpoint (written before this change, no
`.peinfo` sidecar) is itself a hard error: the expectation cannot be checked, and silently
dropping it would defeat the purpose.

---

## 3. What the operator gets

`checkpoints/checkpoint.7.peinfo` next to `checkpoints/checkpoint.7.peb`:

```
# pe checkpoint metadata -- sidecar of the companion .peb file
metadataVersion 1
timeSource driver
simulationTime 1.20000000000000000e+01
timeStep 2400
stepSize 5.00000000000000010e-03
bodyCount 1204
pebBytes 483920
pairingTag ff:istep=2400
materialCount 7
material 0 7.87400000000000000e+03 ... iron
...
```

`cat` on that file answers "which FF dump does this belong to?" — the question that motivated
this work.

---

## 4. Combined restart protocol, end to end

For the record, since no rundir in the FeatFloWer tree has ever exercised one: the D3.1 `*_t12`
runs are cold starts to a longer `MaxSimTime`, not restarts — their decks keep
`SimPar@StartingProc = 0` and their `example.json` keeps `resume_: false`.

**Producing run**
1. `example.json`: `useCheckpointer_: true`, `pointerspacing_: <N>`, `checkpoint_path_: checkpoints`.
2. Deck: `SimPar@BackUpFreq` set so FF dumps land on the same cadence as pe's checkpoints. They
   need not coincide exactly — the expectation check is what proves a given pair matches — but a
   cadence mismatch means most pairs are unusable.
3. FF calls `set_pe_checkpoint_identity_` each step (§1).
4. Mind `SimPar@BackUpNum`: it bounds how far back a restart can reach before the slot has been
   recycled (§1).

**Resuming run**
1. Deck: `SimPar@StartingProc = 1`, `SimPar@StartFile = <N>`, `SimPar@InitUmbrella = 0`,
   `SimPar@Umbrella = 0`. The umbrella keys are mandatory — re-smoothing desyncs the loaded field
   from the geometry (`docs/md_docs/parameter_reference.md`).
2. `example.json`: `resume_: true`, `resumeCheckpointFile_: checkpoint.<k>`, plus the expectation
   keys from §2, taken verbatim from the chosen dump's `time.dmp`.
3. pe validates; on mismatch the run stops with both values named rather than continuing from the
   wrong instant.

### `SimPar@StartFile` is a string whose meaning depends on the path

The deck parser reads it into a `CHARACTER(len=60)` (`source/src_util/param_parser.f90:932-934`,
`CSTART`). The integer constraint is imposed **downstream and only on some paths**:

| Path | Entry | Treatment of `StartFile` |
|---|---|---|
| Default dump | `SolFromFile` → `read_time_sol` → `read_sol_time` (`FullC0ntact/inshape3dcore/fortrancppinterface/io_func.hpp:309`) | `std::stoi(startName)` — **must be a bare integer**; `"_dump/01"` throws |
| Merged / repartition | `SolFromFileRepart` → `read_time_sol_single` (`source/postprocessing/solution_io.f90:1396`) | literal directory name under `_dump/` |
| `UseProvDump=Yes` | `SolFromFileProv` → `read_sol_from_file_prov_common` (`source/postprocessing/solution_io_provenance.f90:140`) | literal directory name under `_dump_prov/` |

`q2p1_dns_drag` has no provenance branch, so it always takes the `std::stoi` path — for that
application `StartFile` must be a bare integer. Several template decks carry the `"_dump/01"`
form, which is valid for the other two paths and fatal for this one.

---

## 5. The DNS drag application cannot resume pe yet

Stated plainly because it bounds what this PR buys: **`setupDNSDragSerial` has no resume branch
at all** — no `config.getResume()`, no `readCheckpoint()`. It unconditionally builds its scene
from scratch.

So for the DNS case this PR supplies the *validation* half of a mechanism whose *loading* half
does not exist. Before the protocol in §4 applies to `q2p1_dns_drag`, someone must add a resume
branch to `setupDNSDragSerial` following the pattern in `setupFluidizationSRRSerial`: register
materials first, then `resumeFromConfiguredCheckpoint(config)` instead of creating particles.
The setups that *do* have a resume branch today — ATC, Fluidization SRR, ParticleBench — get the
full protocol immediately.

---

## 6. Explicitly out of scope for FF

- FF needs no knowledge of the sidecar format. The two entry points and the four deck keys are
  the entire surface.
- FF must not write or parse `.peinfo` files.
- Choosing *which* checkpoint to resume from stays an operator/staging decision. pe's job is to
  refuse the wrong one, not to search for the right one.

---

## Appendix A — the C resume entry point, and why it is not the mechanism

pe also exposes:

```c
void set_pe_resume_expectation_(const double *simTime, const int *step, const char *tag);
```

It sets the same four expectation values at runtime. It is **not** usable from the FeatFloWer
applications as they are structured today, because it would have to be called after FF has
restored its time and step but before pe reads its checkpoint — and those two events happen in
the wrong order.

`commf2c_*` is invoked from inside each application's own `General_init_ext`, which runs **before**
`SolFromFile`:

| Application | `General_init_ext` called | `commf2c_*` inside it | `SolFromFile` |
|---|---|---|---|
| `applications/q2p1_dns_drag/app_init.f90` | `:21` | `:435` | `:41` |
| `applications/q2p1_dkt/app_init.f90` | `:20` | `:486`, `:499` | `:40` |
| `applications/q2p1_el_pipeflow/app_init.f90` | `:21` | `:466`, `:482` | `:41` |
| `applications/q2p1_bench_sedimentation/app_init.f90` | `:20` | `:486`, `:499` | `:40` |

(`General_init_ext` is not a shared routine — each application defines its own copy in its own
`app_init.f90`. The pattern is uniform across all of them.)

At the moment `commf2c_*` runs, `timens` still holds `SimPar@StartSimTime` and `istep_ns` is
zero. An expectation built there would be wrong on every resume. Using this entry point would
require restructuring app init so that the pe setup call moves after `SolFromFile` — a change to
every application, with its own consequences, and not something this PR asks for.

The entry point is kept for drivers that *do* control the ordering, and it is guarded: declaring
an expectation through both this call and the deck is refused rather than resolved by precedence,
because `loadFromFile` runs after the call and a silent winner would be the opposite of what the
call site reads like.

If you do use it, note that all three arguments are skippable only by passing a null pointer,
which `real(c_double), intent(in)` cannot express. Bind them by value as `type(c_ptr)`:

```fortran
interface
  subroutine set_pe_resume_expectation(simTime, step, tag) &
      bind(C, name='set_pe_resume_expectation_')
    import :: c_ptr
    type(c_ptr), value :: simTime, step, tag
  end subroutine
end interface

real(c_double), target  :: expTime
integer(c_int), target  :: expStep
call set_pe_resume_expectation(c_loc(expTime), c_loc(expStep), c_null_ptr)  ! tag unchecked
```

And remember the post-increment from §2: after `SolFromFile`, `istep_ns` is already the *next*
step, so an expectation built from it must subtract one.
