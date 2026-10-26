# Deprecations

How pe retires input and behaviour that users may still rely on, and the list of everything
that is deprecated right now.

## Mechanism

Deprecated *input* (a built-in value that should come from the json file) and deprecated
*behaviour* (something a setup does that it should not) are reported at run time with
`pe::reportDeprecated()` from `pe/util/Deprecation.h`. A C++ `[[deprecated]]` attribute does
not fit these cases: the functions stay, only what they do without explicit input changes.

```cpp
if( !reportDeprecated( "archimedes.position", "set domainBoundary_.position", rank == 0 ) )
   abortSetup( "deprecated input, see the PE DEPRECATED message" );
```

- The message starts with `PE DEPRECATED [<id>]`, so `grep "PE DEPRECATED"` on a run's output
  lists everything a case still depends on.
- It goes to the pe log file of every process, and to `std::cerr` where `console` is true
  (pass `rank == 0` so it is printed once).
- As a warning it is printed once per ID and process.

Every deprecation goes through two stages:

1. **Warning.** The old input or behaviour still works; `reportDeprecated()` prints what is
   deprecated and what replaces it, and returns `true`.
2. **Removal.** The fallback is removed. Missing input stops the run with an error; deprecated
   behaviour is gone.

`"deprecationsAsErrors_": true` in the configuration file switches stage 2 on early:
`reportDeprecated()` prints the message as an error and returns `false`, and the caller stops
before it creates anything. Use it to check that a case no longer depends on deprecated input
before the fallback is removed.

## Adding a deprecation

1. Pick an ID `<area>.<item>` (e.g. `archimedes.position`). It stays stable until removal.
2. Add the replacement (usually a json key, see
   [adding-json-parameters.md](adding-json-parameters.md)); a `has...()` getter tells explicit
   input from the default.
3. Call `reportDeprecated()` where the old input or behaviour is used, before anything is
   created, and stop if it returns `false`.
4. Mention it in the documentation of the affected function and add a row below.

On removal, delete the fallback and the row, and turn the `reportDeprecated()` call into an
error (or remove it, for behaviour).

## Current deprecations

| ID | Where | Deprecated | Replacement | Since |
|---|---|---|---|---|
| `config.defaultFileName` | `SimulationConfig::loadConfigFile()` (`src/config/SimulationConfig.cpp`) | The configuration file is the default `example.json` | `--pe-config <file>`, `PE_CONFIG_FILE`, or `set_pe_config_file_()`, see [adding-json-parameters.md](adding-json-parameters.md) | 2026-10 |
| `archimedes.position` | `setupArchimedes()` (`pe/interface/setup_archimedes.h`) | Built-in position `[0.0274099, -2.56113, 0.116155]` of the channel mesh `archimedes.obj` | `"domainBoundary_": { "position": [x, y, z] }` | 2026-10 |
| `archimedes.resetAngularVelocityOnResume` | `setupArchimedes()` | After a resume, the angular velocities of all spheres are reset to zero, so the run does not continue the saved state | `"resetAngularVelocityOnResume_": false` keeps the restored angular velocities; the reset is removed in stage 2 | 2026-10 |
