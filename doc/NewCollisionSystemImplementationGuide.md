# New CollisionSystem Implementation Guide

This guide documents all components you must add (and wire) when introducing a new collision-system flavor in PE.

It uses `HardContactLubricated` as the concrete reference implementation.

## 1. Core Template Specializations You Need

When creating a new solver flavor `response::YourSolver`, the minimum pattern is:

1. `response` solver/tag type
2. `Configuration<CD,FD,BG,response::YourSolver>` specialization
3. `CollisionSystem< C<CD,FD,BG,response::YourSolver> >` specialization

In practice, many solver flavors also need:

4. `RigidBodyTrait< C<...response::YourSolver> >` specialization
5. `ContactTrait< C<...YourSolver> >` specialization
6. `ParallelTrait< C<...response::YourSolver> >` specialization

## 2. Reference Files (HardContactLubricated)

### 2.1 Solver/tag type
- `pe/core/response/HardContactLubricated.h`
- Class template: `HardContactLubricated<C,U1,U2>`

This type is used as the 4th template argument in `Configuration<...>`.

### 2.2 Configuration specialization
- `pe/core/configuration/HardContactLubricated.h`
- Specialization:
  - `Configuration<CD,FD,BG,response::HardContactLubricated>`

Notes:
- This file defines all config-level aliases (`BodyType`, `ContactType`, etc.).
- It can override batch generation strategy. In this case:
  - `typedef Configuration<CD,FD,batches::SingleBatch,response::HardContactLubricated> Config;`

### 2.3 CollisionSystem specialization (main implementation)
- `pe/core/collisionsystem/HardContactLubricated.h`
- Specialization:
  - `CollisionSystem< C<CD,FD,BG,response::HardContactLubricated> >`

This is the large implementation file (simulation step, contact filtering/caching, relaxation iterations, MPI sync, integration, etc.).

### 2.4 RigidBodyTrait specialization
- `pe/core/rigidbody/rigidbodytrait/HardContactLubricated.h`
- Specialization:
  - `RigidBodyTrait< C<CD,FD,BG,response::HardContactLubricated> >`

Why needed:
- Solver stores per-body state (`index_`, `oldForce_`, `oldTorque_`) that is read from the collision-system implementation.

### 2.5 ContactTrait specialization
- `pe/core/response/ContactTrait.h`
- Specialization:
  - `ContactTrait< C<CD,FD,BG,HardContactLubricated> >`

Why needed:
- Adds solver-specific contact data/API (here: lubrication flag + blend weight).
- `Contact` inherits `ContactTrait<Config>`, so any methods called by the solver must exist on this specialization.

### 2.6 ParallelTrait specialization
- `pe/core/ParallelTrait.h`
- Specialization:
  - `ParallelTrait< C<CD,FD,BG,response::HardContactLubricated> >`

Why needed:
- Compile-time MPI capability flag (`value = 1` for MPI-capable solver families).

## 3. Mandatory Wiring in Aggregator Headers

Even with all specializations implemented, missing includes in umbrella headers can make the solver appear "not found" or fall back incorrectly.

### 3.1 Response forward declaration
- `pe/core/response/Types.h`
- Add:
  - `template<typename,typename,typename> class YourSolver;`

### 3.2 Solver include registry
- `pe/core/response/Solvers.h`
- Add:
  - `#include <pe/core/response/YourSolver.h>`

### 3.3 Configuration include registry
- `pe/core/Configuration.h`
- Add:
  - `#include <pe/core/configuration/YourSolver.h>`

### 3.4 CollisionSystem include registry
- `pe/core/CollisionSystem.h`
- Add:
  - `#include <pe/core/collisionsystem/YourSolver.h>`

Important:
- Keep specialization includes before default implementation include.
- `CollisionSystem.h` explicitly warns about this ordering.

### 3.5 RigidBodyTrait include registry
- `pe/core/rigidbody/RigidBodyTrait.h`
- Add:
  - `#include <pe/core/rigidbody/rigidbodytrait/YourSolver.h>`

## 4. Configuration Macro Exposure

To activate your solver from the usual configuration path:

- Add solver option to docs/comments in `pe/config/Collisions.h`.
- Set:
  - `#define pe_CONSTRAINT_SOLVER pe::response::YourSolver`

If your solver needs additional compile-time behavior in detection/response, wire corresponding macros there as well.

## 5. When Additional Traits Are Required

You do **not** always need every trait specialization. Use this rule:

1. If collision-system code accesses solver-specific body fields, add `RigidBodyTrait` specialization.
2. If `Contact` needs solver-specific methods/data, add `ContactTrait` specialization.
3. If solver participates in MPI mode, add `ParallelTrait` specialization.

If you skip a needed specialization, failures are usually:
- compile errors on missing members (`body->index_`, `contact->set...`)
- wrong runtime behavior due to default trait silently lacking solver-specific state

## 6. HardContactLubricated-Specific Integration Gotcha

`HardContactLubricated` relies on lubrication contacts generated in fine detection.

- `pe/core/detection/fine/MaxContacts.h` gates lubrication paths with `#ifdef PE_LUBRICATION_CONTACTS`.
- `pe/core/collisionsystem/HardContactLubricated.h` defines:
  - `#define PE_LUBRICATION_CONTACTS 1`
- Lubrication contacts are created via:
  - `ContactVector::addLubricationContact(...)` in `pe/core/contact/ContactVector.h`
- That path calls:
  - `contact->setLubricationFlag()`
  - `contact->setLubricationWeight(weight)`
  so those methods must exist in `ContactTrait` for the active solver.

If your new solver introduces custom contact types/flags, ensure:
1. Fine detection creates them.
2. `ContactVector` exposes add-functions.
3. `ContactTrait` provides required methods/state.
4. Collision-system resolve path consumes them consistently.

## 7. Practical Step-by-Step Checklist for a New Solver

Use this checklist in order:

1. Add `pe/core/response/YourSolver.h` (solver/tag template).
2. Add forward declaration in `pe/core/response/Types.h`.
3. Add include in `pe/core/response/Solvers.h`.
4. Add `pe/core/configuration/YourSolver.h` specialization.
5. Add include in `pe/core/Configuration.h`.
6. Add `pe/core/collisionsystem/YourSolver.h` specialization.
7. Add include in `pe/core/CollisionSystem.h` before default include.
8. Add `pe/core/rigidbody/rigidbodytrait/YourSolver.h` if per-body fields are needed.
9. Add include in `pe/core/rigidbody/RigidBodyTrait.h`.
10. Add `ContactTrait` specialization in `pe/core/response/ContactTrait.h` if contact metadata/API is needed.
11. Add `ParallelTrait` specialization in `pe/core/ParallelTrait.h` if MPI-capable.
12. Expose solver in `pe/config/Collisions.h`.
13. Build and run at least one example using `pe_CONSTRAINT_SOLVER = pe::response::YourSolver`.

## 8. Recommended Validation Commands

From repo root:

```bash
rg -n "YourSolver" pe/core pe/config
rg -n "CollisionSystem< C<CD,FD,BG,response::YourSolver" pe/core/collisionsystem
rg -n "Configuration<CD,FD,BG,response::YourSolver" pe/core/configuration
rg -n "RigidBodyTrait< C<CD,FD,BG,response::YourSolver" pe/core/rigidbody/rigidbodytrait
rg -n "ContactTrait< C<CD,FD,BG,YourSolver" pe/core/response/ContactTrait.h
rg -n "ParallelTrait< C<CD,FD,BG,response::YourSolver" pe/core/ParallelTrait.h
```

If any required line is missing, integration is incomplete.
