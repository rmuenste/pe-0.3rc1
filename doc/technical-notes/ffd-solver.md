Here’s a focused analysis of the Fast‑Frictional Dynamics (FFD) solver as wired through the collision system specialization in `pe/core/collisionsystem/FFDSolver.h`, plus the actual solver math in `pe/core/response/FFDSolver.h`.

**Scope Note**
- `pe/core/collisionsystem/FFDSolver.h` is the collision system specialization that orchestrates the FFD time step and MPI exchange; the mathematical solve itself lives in `pe/core/response/FFDSolver.h`.

**Key Features**
- Specialized collision system for FFD that *ignores batch generation* (`BG` is unused) because FFD resolves per‑body constraints rather than batched contacts. See `pe/core/collisionsystem/FFDSolver.h`.
- Two‑half‑step integration with collision solve in between, matching Kaufman et al.’s FFD structure (explicitly referenced in `simulationStep`). See `pe/core/collisionsystem/FFDSolver.h`.
- Per‑body constraint accumulation (normal + friction) followed by per‑body QP solve. See `pe/core/response/FFDSolver.h`.
- MPI‑aware multi‑phase synchronization: forces, body migration, constraint exchange, and post‑step updates (4 communication phases). See `pe/core/collisionsystem/FFDSolver.h`.
- Detailed profiling timers for each phase (comm encode/MPI/decode, half steps, detection, response, cleanup). See `pe/core/collisionsystem/FFDSolver.h`.
- Friction handled by sampling a friction “cone” with `frictionSamples` directions and solving a secondary QP in the friction basis. See `pe/core/response/FFDSolver.h`.

**Collision System Methods and Flow**
- `simulationStep(real timestep)` (`pe/core/collisionsystem/FFDSolver.h`):
  - `timestep *= 0.5` → two half steps.
  - **Comm1:** synchronize forces for remote bodies (`firstCommunication`).
  - **Half step 1:** position half‑step + velocity half‑step (local, non‑fixed bodies).
  - **Comm2:** send bodies that crossed process boundaries after half‑step.
  - **Detection:** `detector_.findContacts(contacts_)`.
  - **Constraint setup:** `solver_.checkContacts(contacts_)`.
  - **Comm3:** exchange constraints across MPI (`thirdCommunication`).
  - **Resolve / Half step 2:** per body:
    - if constrained: `solver_.resolveContacts(*body)`.
    - else: `body->secondVelocityHalfStep(...)`.
    - always: `body->secondPositionHalfStep(...)`.
  - **Comm4:** final body updates to remote processes; cleanup of migrated bodies.
- `checkContacts` and `resolveContacts` live in `pe/core/response/FFDSolver.h` and are called from the collision system.

**Contact / Constraint Construction**
- `checkContact` (`pe/core/response/FFDSolver.h`):
  - Ignores contacts outside local domain (MPI).
  - Transforms contact position and normal into each body’s frame.
  - Computes normal twist `ntwist` induced by contact via inertia and mass.
  - Computes relative normal velocity and a constraint offset `d` when both bodies are dynamic.
- `collideBody` (`pe/core/response/FFDSolver.h`):
  - Builds normal constraint: `constraint = ntwist`, `offset = d - penetrationCorrection * distance / dt`.
  - Computes constraint violation `cv = ntwist * vel - d`.
  - Rejects separating contacts (`cv > contactTolerance`).
  - Accumulates restitution statistics (for later COR estimate).
  - If friction enabled:
    - Computes tangent direction from relative contact velocity.
    - Samples `frictionSamples` tangential directions by rotating around normal.
    - Adds each sampled friction bound (twist) to the body’s friction basis.

**Mathematical Formulation (from code + comments)**
The solver solves quadratic programs (QP) with inequality constraints using Goldfarb‑Idnani active‑set QP:

1) **Collision QP (normal constraints only)**  
   In `solveCollisionQP` (`pe/core/response/FFDSolver.h`), the QP is:

   - Variables: 6‑DoF twist response `x` (angular + linear).
   - Objective:
     ```
     minimize  a^T x + 1/2 x^T G x
     ```
     where `G` is the generalized mass matrix (body inertia + mass).
     `a = -M * v` (mass‑weighted current twist).
   - Constraints:
     ```
     A x - b >= 0
     ```
     `A` rows are normal constraint twists; `b` offsets include penetration correction.

   The solver computes a post‑collision response `r = x - v`.  
   If `responseRestriction` is set, the response magnitude is capped by
   `responseRestriction * maxStep` (estimated from constraint offsets and velocities).

2) **Friction QP (tangential constraints)**  
   In `solveFrictionQP` (`pe/core/response/FFDSolver.h`), with friction basis `U`:

   - Variables: coefficients in the friction basis.
   - Objective:
     ```
     minimize  a^T x + 1/2 x^T G x
     ```
     with:
     ```
     G = U^T M U
     a = U^T M phi
     ```
     where `phi` is the frictionless post‑collision twist.
   - Constraints:
     ```
     A x - b >= 0
     ```
     Each row derived from sampled friction bounds and scaled by 1/μ (coefficient of friction).
   - The friction twist is reconstructed: `delta = U * x`.

3) **Velocity Update**
   In `resolveContacts` (`pe/core/response/FFDSolver.h`):
   - Estimate coefficient of restitution:
     ```
     cor = weightedCV / CV
     ```
     clamped in [0, 1] by asserts.
   - Final twist:
     ```
     v_new = v + delta + (1 + cor) * r
     ```
     (converted back to world frame).

**Solver Method Details**
- **QP Solver:** Goldfarb‑Idnani active‑set method (`solve`, `solveInverseG`, `addConstraint`, `removeConstraint`).
- **Linear Algebra:** Cholesky decomposition + inversion of lower triangular matrix to obtain `G^{-1}`.
- **Constraint Handling:** Active set updated by checking violations (`s = A x - b`) and moving between primal/dual steps.

**MPI and Data Synchronization**
- Constraints, forces, and body state are exchanged explicitly:
  - `sendForce`, `recvForces` for force sync.
  - `sendConstraints`, `recvConstraints` for normal/friction constraint transfer.
  - Bodies and attachables are sent/updated as they migrate across domain partitions.
- Per‑phase timers allow profiling and reporting of MPI vs local compute.

If you want, I can also map this to the exact configuration parameters (like `contactTolerance`, `penetrationCorrection`, `frictionSamples`, `responseRestriction`) from `pe/system/FFDConfig.h` and describe their effect on the formulation and numerical behavior.
