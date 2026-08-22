# Lubrication Demo Examples

Serial test harnesses that exercise the lubrication pipeline in isolation, without MPI or CFD
coupling.  Two executables are provided:

* `lubrication_sphere_plane` – a single sphere approaching, dwelling near, and retreating from a
  fixed plane.
* `lubrication_sphere_sphere` – two spheres approaching one another, holding, and separating.

Each program exposes command line flags (via `--help`) for time step control, scripted approach /
retreat phases, hysteresis-blend widths, and logging cadence.  The demos set the runtime
lubrication blend parameters (`pe::lubrication::setContactHysteresisDelta` /
`setLubricationHysteresisDelta`) and emit detailed contact diagnostics with
`pe_LOG_DEBUG_SECTION`, making it easy
to inspect force magnitudes, blend weights, and gap trajectories in the log (ensure the build uses
`pe::logging::debug` or lower to enable debug logging).  The runs complete entirely in serial mode
and rely on a solver stack that invokes the shared lubrication stage.

The demos contain no solver-specific code, so they build under any `pe_CONSTRAINT_SOLVER`. They
refuse to run at startup, before enabling lubrication, unless the selected solver actually invokes
the stage -- `pe::response::HardContactAndFluid` or
`pe::response::HardContactSemiImplicitTimesteppingSolvers`, the two that declare
`hasLubricationStage`. Without that check they would run to completion under, say, the default
`HardContactEulerLagrange` and print a gap trace with no lubrication in it. Configure with e.g.

```bash
cmake -S . -B build-lub -DPE_BUILD_EXAMPLES=ON \
  -DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid"
```

Lubrication itself is a runtime switch: the demos call `pe::lubrication::setEnabled( true )` during
setup.
