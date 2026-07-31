# Euler–Lagrange Solver Conventions

Conventions and pitfalls specific to the `HardContactEulerLagrange` collision system,
which backs PE's coupling to an external Euler–Lagrange (unresolved CFD-DEM) driver
such as the FeatFloWer `q2p1_el_pipeflow` application.

Two points below have each cost real debugging time, so they are recorded here rather
than left to be rediscovered from the source.

> **On locations.** This note deliberately refers to *symbols and comments* rather than
> line numbers. An earlier version of this material cited fixed line numbers and they
> had drifted by several hundred lines within a couple of months.

## 1. Extend the E-L specialization, not `HardContactAndFluid`

Work that adds Euler–Lagrange behavior on the PE side belongs in the **existing E-L
specialization**:

- `pe/core/collisionsystem/HardContactEulerLagrange.h` — the full
  `CollisionSystem< C<CD,FD,BG,response::HardContactEulerLagrange> >` specialization,
  with its own velocity-prediction and correction path.
- `pe/core/response/HardContactEulerLagrange.h` — the matching response solver.

It is selected at compile time in `pe/config/Collisions.h`:

```cpp
#define pe_CONSTRAINT_SOLVER  pe::response::HardContactEulerLagrange
```

`HardContactAndFluid` is a *different, independent* specialization that happens to also
concern fluid coupling. It is **not** the E-L path, and E-L changes must not be made
there. Reading one while intending to modify the other is the specific mistake this
section exists to prevent — see §2 for why it is more than a cosmetic mix-up.

## 2. The two solvers make opposite gravity/buoyancy decisions

This is the trap. The two specializations disagree about *who* owns body forces, and the
disagreement is silent: both compile, both run, and picking the wrong mental model
produces a settling velocity that is wrong by the buoyancy term rather than an error.

### `HardContactEulerLagrange` — the driver owns gravity and buoyancy

Body caching uses only driver-supplied external forces. The source says so explicitly:

```cpp
// Cache body properties using only driver-owned external forces and torques. Euler-Lagrange
// body-force semantics such as gravity and buoyancy are handled by the outer driver, not by
// this PE collision system.
```

Its velocity prediction seeds `v_[j]` from `body->getLinearVel()` and applies only the
mapped external force via `getInvMass() * dt * …`. **No** buoyancy term is added inside PE.

### `HardContactAndFluid` — PE applies buoyancy itself

The same stage in `HardContactAndFluid` computes buoyancy internally from the material
density and `Settings::liquidDensity()`, and injects it into the predicted velocity:

```cpp
real buoyancy = vol * (rho - Settings::liquidDensity()) * body->getInvMass();
v_[j] = body->getLinearVel() + buoyancy * Settings::gravity() * dt;
```

### Consequence for an E-L driver

Under the active E-L solver the driver **must** supply the net submerged weight in the
force it maps to PE:

```
F_grav+buoy = (rho_p - rho_f) * V_i * g
```

Both terms, gravity *and* buoyancy. Omitting the Archimedes contribution `-rho_f*V_i*g`
on the assumption that "PE handles buoyancy" is correct for `HardContactAndFluid` and
wrong for `HardContactEulerLagrange`. No PE-side change is needed for gravity — keeping
all body-force semantics on the driver side is what the E-L solver intends.

Note this is orthogonal to the resolved-field pressure force `-V*grad(p)`, which a driver
may also supply. In a quiescent settling case `grad(p) ~ 0`, so the settling driver is the
`grav_buoy` term above.

## 3. The interface is compiled against one solver only

`pe/interface/*` is compiled once, against whatever `pe_CONSTRAINT_SOLVER` selects. Any
method reached from the interface that only one specialization provides will break every
other selection at compile time.

The E-L diagnostics (`getElLubricationVirial`, `getElContactVirial`, the wall-impulse
accumulators, `resetElLubricationVirial`) and the per-body hydro-state API
(`setEulerLagrangeHydroState`, `setEulerLagrangeRefVelocity`,
`hasEulerLagrangeHydroState`) exist **only** on `HardContactEulerLagrange`.

Call them through the guards in `pe/interface/el_optional_api.h` — `std::void_t` detectors
plus `if constexpr` dispatch — rather than directly, so the interface still builds for
other solvers. Fallbacks there are well defined: diagnostics report zero, the predicate
reports false, setters and resets are no-ops.

Two things make this easy to get wrong:

- Large parts of `src/interface/sim_setup.cpp` sit inside `#if HAVE_MPI`, which serial-mode
  builds never compile. A tree that builds clean in `PE_SERIAL_MODE` can still fail in a
  parallel build.
- The build uses `-Wfatal-errors`, so each translation unit reports only its **first**
  error. Fixing one missing method commonly just reveals the next; rebuild and re-scan
  until clean rather than assuming a single error is the whole story.

## Related notes

- [contact-solvers-overview.md](contact-solvers-overview.md) — the response solver families and how they are selected.
- [new-collision-system-implementation-guide.md](new-collision-system-implementation-guide.md) — checklist for wiring a new specialization.
- [lubrication-contacts.md](lubrication-contacts.md) — lubrication design, centered on `HardContactLubricated`.
