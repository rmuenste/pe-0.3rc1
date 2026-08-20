# FF-WIRING.md — what FeatFloWer must do to pair its dumps with pe checkpoints

Scope: the pe side of checkpoint identity is complete and on `feature/checkpoint-metadata`.
This note specifies the FeatFloWer changes that finish the loop. **Nothing here is implemented
in pe's sibling repository; the FF side lands separately.**

The whole point is one property:

> A pe checkpoint and the FeatFloWer `_dump` it belongs to must be provably the same instant, or
> the run must refuse to start.

Today that pairing is done by the operator's memory. FF's dump records its time and step
(`_dump/processor_<rank>/<iout>/time.dmp`, two lines: `timens`, then `istep_ns` —
`FullC0ntact/inshape3dcore/fortrancppinterface/io_func.hpp:294-295`), and pe now records the
same two numbers. All FF has to do is hand them across.

---

## 1. Write side — one call per time step

pe exposes, next to the existing `set_pe_timestep_`
(`src/interface/object_queries.cpp`):

```c
void set_pe_checkpoint_identity_(const double *simTime, const int *step, const char *tag);
```

`tag` is a **null-terminated** C string, or `NULL`. FF should call this once per time step,
before pe can write a checkpoint — i.e. next to the existing `set_pe_timestep_` call, at the
top of the time loop:

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
write(cTag,'(A,I0,A,I0)') 'ff:istep=', istep_ns, ':iout=', iOut
call set_pe_checkpoint_identity(timens, istep_ns, trim(cTag)//c_null_char)
```

Pass **the same `timens` and `istep_ns` that `write_sol_to_file` puts into `time.dmp`**. That is
the entire contract: identical numbers on both sides.

Without this call pe falls back to `TimeStep::step() * TimeStep::size()`, which counts pe
substeps and therefore does **not** identify an FF step. The fallback is recorded honestly —
the sidecar's `timeSource` field reads `pe-timestep` instead of `driver` — but it cannot be
paired.

Suggested tag content: whatever makes the pairing legible to a human reading the checkpoint
directory. The FF dump index `iOut` is the most useful single value, because that is the number
the operator later puts into `SimPar@StartFile`.

## 2. Resume side — one call before the setup entry point

pe exposes:

```c
void set_pe_resume_expectation_(const double *simTime, const int *step, const char *tag);
```

Call it **before** the `commf2c_*` setup call (that is where pe reads the checkpoint). A
mismatch is a hard error naming both values; a `NULL`/negative/empty argument leaves that field
unchecked.

Ordering in `app_init.f90` matters and is the one thing that can go wrong:

```
  call SolFromFile(CSTART, 1)              ! restores timens and istep_ns from time.dmp
  call set_pe_resume_expectation(timens, istep_ns, trim(cTag)//c_null_char)
  call commf2c_atc(...)                    ! pe reads its checkpoint here, and validates
```

`SolFromFile` must run first: before it, `timens` still holds `SimPar@StartSimTime` and
`istep_ns` is zero, so the expectation would be built from the wrong numbers and every resume
would fail. If restructuring the app init to that order is not practical, use the deck keys
instead (§3) — they are equivalent and are read from `example.json` at setup time.

Equivalently, and preferred when the rundir is staged by a script rather than by the solver:

## 3. Deck keys (`example.json`)

New, all optional; absent keys preserve today's behaviour exactly.

| Key | Type | Meaning |
|---|---|---|
| `resumeExpectedTime_` | real | Simulation time the checkpoint must carry |
| `resumeExpectedStep_` | integer | Step index the checkpoint must carry; negative = unchecked |
| `resumeExpectedTag_` | string | Pairing tag the checkpoint must carry; empty = unchecked |
| `resumeTimeToleranceSteps_` | real | Time window in units of the checkpoint's own `stepSize`; defaults to 0.5 |

A staging script that copies `_dump/processor_*/<N>/` into place already knows `<N>`; it can read
`time.dmp` (line 1 = time, line 2 = step) and write these three keys. That is the least
error-prone place to put the pairing, because it is the same step that chooses `SimPar@StartFile`.

Note that setting an expectation against a **legacy** pe checkpoint (one written before this
change, with no `.peinfo` sidecar) is itself a hard error: the expectation cannot be checked, and
silently dropping it would defeat the purpose.

## 4. What the operator gets

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
pairingTag ff:istep=2400:iout=3
materialCount 7
material 0 7.87400000000000000e+03 ... iron
...
```

`cat` on that file answers "which FF dump does this belong to?" — the question that motivated
this work.

## 5. Combined restart protocol, end to end

For the record, since no rundir in the FeatFloWer tree has ever exercised one (the D3.1 `*_t12`
runs are cold starts to a longer `MaxSimTime`, not restarts — their decks keep
`SimPar@StartingProc = 0` and their `example.json` keeps `resume_: false`):

**Producing run**
1. `example.json`: `useCheckpointer_: true`, `pointerspacing_: <N>`, `checkpoint_path_: checkpoints`.
2. Deck: `SimPar@BackUpFreq` set so FF dumps land on the same cadence as pe's checkpoints.
   They do not have to coincide exactly — the expectation check is what proves a given pair
   matches — but a cadence mismatch means most pairs are unusable.
3. FF calls `set_pe_checkpoint_identity_` each step (§1).

**Resuming run**
1. Deck: `SimPar@StartingProc = 1`, `SimPar@StartFile = <N>` (a bare integer — the C reader does
   `std::stoi`, so the `"_dump/01"` form found in several template decks throws),
   `SimPar@InitUmbrella = 0`, `SimPar@Umbrella = 0` (mandatory; see
   `docs/md_docs/parameter_reference.md:388-560` — re-smoothing desyncs the loaded field from
   the geometry).
2. `example.json`: `resume_: true`, `resumeCheckpointFile_: checkpoint.<k>`, plus the
   expectation keys from §3.
3. pe validates; on mismatch the run stops with both times named rather than continuing from
   the wrong instant.

## 6. Explicitly out of scope for FF

- The FF side needs no knowledge of the sidecar format. `set_pe_checkpoint_identity_` and
  `set_pe_resume_expectation_` are the entire surface.
- FF must not write or parse `.peinfo` files.
- Choosing *which* checkpoint to resume from stays an operator/staging decision. pe's job is to
  refuse the wrong one, not to search for the right one.
