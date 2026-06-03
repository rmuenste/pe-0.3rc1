# Contact Solver Overview

PE selects its collision response solver at compile time through
`pe_CONSTRAINT_SOLVER` in `pe/config/Collisions.h`. There is no runtime solver
switch in the shared command-line interface. This note gives a short overview
of the main response families that are present in the codebase and the practical
tradeoffs between them.

## Hard-Contact Timestepping

Primary files:

- `pe/core/response/HardContactSemiImplicitTimesteppingSolvers.h`
- `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`

This is the classic hard-contact solver family in PE. It uses semi-implicit
timestepping and iterative relaxation over the global contact set. The code
describes the relaxation as a mixture of nonlinear Gauss-Seidel and Jacobi
updates, with configurable maximum iterations, relaxation parameter, error
reduction, and friction model.

Strengths:

- Good baseline choice for dense contact graphs, stacks, and multi-contact
  coupling.
- Several friction relaxation models are available, including decoupled,
  orthogonal-projection, and generalized maximum-dissipation variants.
- Penetration correction is explicit through ERP/Baumgarte-style controls.

Limitations:

- Results depend on iteration count and relaxation parameters.
- Runtime scales with contacts times iterations.
- It is a hard-contact solver, not a fluid-coupled solver.

If a "default-like" baseline solver is needed for hard-contact-only work, this
is the oldest and most robust family to look at. The actual solver is still the
one selected by the user in `pe/config/Collisions.h` before compilation.

## Hard-Contact With Lubrication

Primary files:

- `pe/core/response/HardContactLubricated.h`
- `pe/core/collisionsystem/HardContactLubricated.h`
- `pe/core/lubrication/Params.h`

`HardContactLubricated` extends the hard-contact timestepping path with
lubrication-aware contact handling. It keeps hard contacts in the iterative
constraint solve and applies lubrication corrections for contacts that are
tagged as lubrication contacts. Runtime parameters include lubrication
threshold, lubrication hysteresis, minimum lubrication gap regularization, and
an impulse cap.

Strengths:

- Canonical path for current lubrication work.
- Preserves the hard-contact solver structure while adding near-contact
  lubrication effects.
- Exposes runtime controls for contact/lubrication blending and regularization.

Limitations:

- More specialized than the baseline hard-contact solver.
- Requires the build and examples to be configured consistently for
  `HardContactLubricated`.
- The lubrication path is focused on supported pair types and current
  integration experiments, not a general fluid solver.

Deprecated or legacy lubrication stacks such as `HardContactAndFluid` should not
be used as the target for new work unless a specific historical comparison is
being made.

## Short-Range Repulsion

Primary files:

- `pe/core/response/ShortRangeRepulsion.h`
- `pe/core/collisionsystem/ShortRangeRepulsion.h`

`ShortRangeRepulsion` is a force-based response model rather than a hard-contact
constraint solver. It applies a soft repulsive force inside a security zone
before or near contact, following the Pan et al. formulation documented in the
header. The collision-system specialization supports subcycling and force
synchronization.

Strengths:

- Useful for soft pre-contact repulsion experiments and particle-wall or
  particle-particle separation control.
- Has a small parameter set: particle-particle stiffness, particle-wall
  stiffness, security-zone width, and optional normal damping.
- Avoids solving a global hard-contact complementarity problem.

Limitations:

- It is not a hard non-penetration constraint solver.
- Parameter choices strongly affect stiffness and timestep requirements.
- It should be considered a different physical model from the hard-contact
  solver families, not a drop-in equivalent.

## FFD Solver

Primary files:

- `pe/core/response/FFDSolver.h`
- `pe/core/collisionsystem/FFDSolver.h`

The FFD solver uses per-body quadratic programs. It solves normal contact
constraints and friction using Goldfarb-Idnani QP solves, with a half-step /
half-step integration structure around contact resolution.

Strengths:

- More direct per-body solve structure with fewer global iteration knobs.
- Explicit restitution handling based on accumulated constraint violation.
- Friction is handled in a principled QP framework with sampled friction cones.

Limitations:

- Per-body QP solves do not capture global contact coupling as directly as the
  hard-contact iterative solver.
- Cost grows with the number of constraints per body.
- Friction cone sampling can under-resolve friction unless enough samples are
  used.

## DEM Solvers

Primary files:

- `pe/core/response/DEMSolver.h`
- `pe/core/collisionsystem/DEMSolver.h`
- `pe/core/response/DEMSolverObsolete.h`
- `pe/core/collisionsystem/DEMSolverObsolete.h`

The DEM solvers are force-based discrete element methods. They compute contact
forces and then integrate the bodies, rather than solving hard-contact
constraints.

`DEMSolver` uses a linear spring-dashpot normal force and Haff-Werner
tangential damping capped by Coulomb friction. It has the newer MPI/contact
ownership logic with shadow copies and notification-based synchronization.

`DEMSolverObsolete` follows the older DEM pipeline. It still exists in the
tree, uses the older MPI encoder/decoder communication path, and supports both
linear spring-dashpot and Hertzian normal force models.

Practical distinction:

- Use `DEMSolver` as the modern DEM implementation when the newer
  synchronization and ownership logic matters.
- Treat `DEMSolverObsolete` as legacy code unless its Hertzian force model or
  historical behavior is specifically required.

## Complementarity And Friction Solver Classes

Primary files:

- `pe/core/response/FrictionlessSolver.h`
- `pe/core/response/BoxFrictionSolver.h`
- `pe/core/response/ConeFrictionSolver.h`
- `pe/core/response/PolyhedralFrictionSolver.h`
- `pe/core/response/OpenCLSolver.h`
- `pe/core/response/ContactSolver.h`

These classes are lower-level contact solver families built around
complementarity/friction formulations. They are parameterized by configured LCP
or friction subsolvers and are distinct from the newer hard-contact
timestepping collision-system specializations.

They are useful to understand when reading older configurations or
solver-specific contact traits, but most current hard-contact discussion should
start with `HardContactSemiImplicitTimesteppingSolvers` or
`HardContactLubricated`.

## Choosing A Solver Family

| Need | Solver family to inspect first |
|------|--------------------------------|
| Robust hard-contact-only rigid-body dynamics | `HardContactSemiImplicitTimesteppingSolvers` |
| Hard contact plus current lubrication experiments | `HardContactLubricated` |
| Soft near-contact repulsion without hard constraints | `ShortRangeRepulsion` |
| Per-body QP contact response with explicit restitution | `FFDSolver` |
| Force-based particle contact dynamics | `DEMSolver` |
| Legacy DEM behavior or Hertzian DEM comparison | `DEMSolverObsolete` |

The selected solver affects configuration traits, body/contact traits, collision
system specialization, communication behavior, and available runtime controls.
When changing `pe_CONSTRAINT_SOLVER`, check the corresponding
`pe/core/configuration/`, `pe/core/collisionsystem/`, and `pe/core/response/`
files together.
