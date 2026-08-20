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
and rely on the `HardContactAndFluid` solver stack, which invokes the shared lubrication stage.
Lubrication itself is a runtime switch: the demos call `pe::lubrication::setEnabled( true )` during
setup, so configure `pe_CONSTRAINT_SOLVER` as `pe::response::HardContactAndFluid` when building.
