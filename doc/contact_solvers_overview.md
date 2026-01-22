Here’s a focused comparison grounded in the code.

**Core Differences (Algorithm + Integration)**
- FFD: per‑body QP solve (normal then friction) using Goldfarb‑Idnani, with half‑step/half‑step integration around contact resolution. See `pe/core/collisionsystem/FFDSolver.h` and `pe/core/response/FFDSolver.h`.
- HardContact: semi‑implicit timestepping with iterative relaxation over all contacts (mixture of nonlinear Gauss‑Seidel/Jacobi) for a fixed number of iterations and selectable relaxation model. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
- FFD computes restitution from constraint violation statistics (`weightedCV/CV`) and updates velocities once per body; HardContact is inelastic timestepping with ERP/Baumgarte stabilization and no explicit restitution term in the flow. See `pe/core/response/FFDSolver.h` and `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
- FFD friction is a sampled cone (multiple tangential “bounds”) solved via a second QP in a friction basis; HardContact friction is handled by iterative models (decoupling, orthogonal projections, generalized max dissipation). See `pe/core/response/FFDSolver.h` and `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
- FFD has no batch generation or global contact iteration; HardContact is a global iterative solver with configurable max iterations and under‑relaxation. See `pe/core/collisionsystem/FFDSolver.h` and `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.

**HardContact is Sequential‑Impulses‑like**
- The code explicitly describes the solver as a mixture of nonlinear Gauss‑Seidel and Jacobi and applies under‑relaxation (`setRelaxationParameter`) and multiple iterations (`setMaxIterations`). See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.

**Advantages / Disadvantages**

FFD (QP per body)
- Advantages:
  - Deterministic per‑body solve without global iteration tuning; often fewer “solver knobs.” See `pe/core/response/FFDSolver.h`.
  - Explicit restitution handling based on accumulated constraint violation. See `pe/core/response/FFDSolver.h`.
  - Friction handled in a principled QP framework (cone sampling + basis). See `pe/core/response/FFDSolver.h`.
- Disadvantages:
  - Per‑body QP ignores cross‑body coupling beyond shared constraints; can be less globally consistent for large stacks or tightly coupled systems. See `pe/core/response/FFDSolver.h`.
  - QP cost grows with per‑body constraint count; friction cone sampling increases problem size. See `pe/core/response/FFDSolver.h`.
  - Sampling‑based friction can under‑resolve anisotropic friction unless `frictionSamples` is high. See `pe/core/response/FFDSolver.h`.

HardContact Semi‑Implicit Timestepping (iterative relaxation)
- Advantages:
  - Global iterative relaxation captures multi‑contact coupling and tends to handle large stacks and dense contact graphs more robustly. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
  - Multiple friction models (decoupling, orthogonal projections, generalized max dissipation) let you trade speed vs. accuracy. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
  - ERP and adaptive Baumgarte capping provide explicit penetration correction controls. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
- Disadvantages:
  - Requires tuning `maxIterations`, relaxation parameter, ERP/capping; convergence and results can be order‑dependent. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
  - More iteration‑heavy; runtime scales with iterations × contacts. See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.
  - Primarily inelastic; restitution effects need to be modeled indirectly (not explicit like FFD). See `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`.

If you want, I can add a short “when to use which” decision guide and a parameter‑tuning cheat sheet.

**DEMSolver vs DEMSolverObsolete**

DEMSolver overview
- Collision system does: detect contacts, per-contact DEM force response, synchronize forces, integrate bodies, synchronize bodies. It uses shadow copies and a notification-based MPI sync path. See `pe/core/collisionsystem/DEMSolver.h`.
- Response implements a linear spring-dashpot normal force plus Haff-Werner tangential damping capped by Coulomb friction. It includes explicit MPI ownership/acceptance logic for local/global/remote contacts. See `pe/core/response/DEMSolver.h`.

DEMSolverObsolete overview
- Collision system follows the older DEM pipeline: detect, resolve contacts, sync forces (firstCommunication), integrate, migrate bodies (secondCommunication). It uses the older MPI encoder/decoder flow. See `pe/core/collisionsystem/DEMSolverObsolete.h`.
- Response supports both linear spring-dashpot and Hertzian normal force models (if `dem::forceModel == dem::hertz` and the contact has an effective radius). Tangential force is Haff-Werner. Domain ownership is checked only by `domain_.ownsPoint(...)`. See `pe/core/response/DEMSolverObsolete.h`.

Core differences
- Contact ownership and MPI filtering: DEMSolver has explicit local/global/remote acceptance logic; DEMSolverObsolete only checks domain ownership. See `pe/core/response/DEMSolver.h` vs `pe/core/response/DEMSolverObsolete.h`.
- Force model: DEMSolverObsolete supports Hertzian normal force; DEMSolver uses linear spring-dashpot only. See `pe/core/response/DEMSolverObsolete.h` vs `pe/core/response/DEMSolver.h`.
- MPI/sync architecture: DEMSolver uses shadow copies and notification-based synchronization; DEMSolverObsolete uses first/second communication with body migration and manual cleanup. See `pe/core/collisionsystem/DEMSolver.h` vs `pe/core/collisionsystem/DEMSolverObsolete.h`.

Is the obsolete one really obsolete?
- The name and TODO comments indicate legacy status. It is still wired into traits/configs, but the newer DEMSolver has the modern MPI/contact ownership logic and is likely the intended default. See `pe/core/collisionsystem/DEMSolver.h` and `pe/core/response/DEMSolverObsolete.h`.
