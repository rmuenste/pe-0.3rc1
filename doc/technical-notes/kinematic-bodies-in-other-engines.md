# Kinematic Bodies, Moving Walls and Surface Velocity in Other Engines

This note compares how eleven rigid-body, DEM and non-smooth contact codes handle bodies of
infinite mass that move on a prescribed path (shaker floors, Couette shear walls, a swung bat)
or that drive contacts without moving (conveyor belts). Each row of the table below was read
from the library's source, and the last row shows pe on branch `feature/kinematic-fixed-bodies`.
The survey was the input for that branch's design (2026-10-22). Only source was read; none of
the other codes was built or run.

## Why this note exists

In pe a fixed body (`setFixed( true )`) has zero inverse mass and inertia. Whether it may also
carry a velocity used to depend on the macro `MOBILE_INFINITE`. `CMakeLists.txt` defined it only
for GNU compilers. Without it (Clang, AppleClang, Intel):

- `fix()` zeroed the velocity;
- `setLinearVel` and `setAngularVel` silently ignored fixed bodies;
- `isActive()` returned false for them.

So the conveyor belt, the shaker and the Couette walls of `setup/el_terminal_velocity.h` stood
still under Clang, and nothing reported it. The table tennis demo worked around it with a
dynamic bat of about a tonne whose pose and velocity were overwritten every step (now a fixed
bat driven by `setTargetPose`, see item 6 below). The contact
solver (`HardContactSemiImplicitTimesteppingSolvers.h`) was never the problem: it already uses a
fixed body's own velocity in the contact velocities, never applies a correction to it, and moves
it with that velocity.

## Versions surveyed

| Code | Kind | Source read |
|---|---|---|
| Bullet3 | game / robotics, impulse solver | `bullet3` 63c4d67e3 |
| Box3D | game, soft-step solver (Box2D v3 lineage) | `box3d` 5d83df8 |
| LIGGGHTS-PUBLIC | soft-sphere DEM | 3.8.0, `src/` |
| waLBerla (`mesa_pd`) | pe's successor framework, DEM + HCSITS | GitHub mirror `lssfau/walberla` dc16f31 |
| Project Chrono | multibody, NSC (complementarity) + SMC | `chrono` 4382b81 |
| Jolt Physics | game, impulse solver | `JoltPhysics` 01adafb |
| PhysX 5 | game / robotics, PGS and TGS | `PhysX` b97365c |
| Rapier | game / robotics, impulse solver (Rust) | `rapier` 475caa3 (differs from upstream; line numbers only valid there) |
| Yade | soft-sphere DEM | `yade` (gitlab trunk) dbc3e8e |
| LMGC90 | non-smooth contact dynamics (Moreau–Jean) | user release 2026.rc1 |
| MercuryDPM | soft-sphere DEM | `mercurydpm` (bitbucket) c801fe6 |

The checkouts are in `/data/warehouse17/rmuenste/code/liggghts-dem/` on the cluster.

## Comparison

| Code | Infinite-mass body that moves | How it is driven | Surface velocity without motion | Pairs with no dynamic body | Sleeping near a moving wall | Force on the wall |
|---|---|---|---|---|---|---|
| Bullet3 | runtime kinematic flag | pose; velocity derived from the pose change, once per step | per-contact callback (`m_contactMotion`), friction | filtered in broadphase and solver | pitfall: a kinematic body sleeps after 2 s and its velocity freezes | — |
| Box3D | kinematic body type | velocity, or `b3Body_SetTargetTransform` | `tangentVelocity` on the surface material, friction rows only | never created | a moving kinematic body keeps touching bodies awake | — |
| Jolt | kinematic motion type | `MoveKinematic(target pose)` → velocity | surface velocity in `ContactSettings` (linear and angular), friction rows only | need at least one dynamic body | a moving kinematic body wakes touching bodies; a static belt cannot | — |
| PhysX 5 | `eKINEMATIC` flag | `setKinematicTarget`; velocity set when the step starts, pose snapped to the target afterwards | contact-modification target velocity, normal **and** friction rows | per-scene filter mode, default "suppress" | an active kinematic body wakes every island it touches | — |
| Rapier | types Fixed, kinematic-by-position, kinematic-by-velocity | next pose → velocity; the user's pose is kept | hook sets `tangent_velocity`, friction rows only | excluded by default (`ActiveCollisionTypes`) | a kinematic body may sleep only at exactly zero velocity | — |
| Chrono (NSC) | none: a fixed body leaves the solver and its velocity is ignored | motor links impose the motion as constraints | none; `ChConveyor` is a heavy plate reset every step | inactive pairs dropped | pitfall: fixed bodies never wake anything | contact forces summed |
| waLBerla `mesa_pd` | `FIXED` flag, separate from infinite mass; integration skipped | the user moves the body by hand | none; a velocity on an infinite plane that never moves | opt-in filter (`ExcludeInfiniteInfinite`) | no sleeping | — |
| Yade | blocked degrees of freedom per axis (`blockedDOFs`) | engines set the velocity, `NewtonIntegrator` moves the body | conveyor example: velocity on facets the integrator skips (`mask`) | no filter (no geometry routine for such pairs) | no sleeping | yes; used by servo engines |
| LMGC90 | driven degree of freedom per component (infinite mass in that direction) | imposed velocity as a function of time; position integrated | not possible | skipped when both bodies are fully driven | — | reaction on driven directions = wall force |
| MercuryDPM | walls and fixed particles have zero inverse mass | prescribed position or velocity functions of time | no; a wall's velocity always moves it | fixed–fixed skipped | no sleeping | yes; force control as a velocity servo |
| LIGGGHTS | walls always have infinite mass | mesh movers (`fix move/mesh`) set velocity and position | `surface_vel` on a mesh that never moves | impossible by construction | — | `stress` and `servo` mesh modules |
| **pe** (this branch) | fixed body with a velocity, a runtime state per body; same on every compiler (`PE_MOBILE_INFINITE`, default ON) | `setLinearVel` / `setAngularVel`, integrated by the solver; or `setTargetPose( pos, q, dt )` → velocity, lands on the target to rounding (3e-16 per step, 2e-15 along a 100-step path); the velocity persists | `setSurfaceVel` / `setRelSurfaceVel` (linear, body frame), friction rows only, in the three hard-contact solvers; the static/dynamic coefficient is chosen from the slip including it; a warning elsewhere; no angular part | coarse detectors drop fixed–fixed pairs; the hard-contact solvers also skip them in release builds and assert in debug builds | sleeping is compiled out by default; when on, a driving body never sleeps and wakes bodies touching it | not accumulated |

None of the eleven libraries has a compile-time switch for any of this. pe's `PE_MOBILE_INFINITE=OFF`
exists only to reproduce the old non-GNU behaviour.

## Notes per code

**Bullet3**
- Static, kinematic and dynamic are collision flags per body (`btCollisionObject.h:130-133`).
- A kinematic body's velocity is a finite difference of its transform, computed once per
  `stepSimulation` in `saveKinematicState` (`btDiscreteDynamicsWorld.cpp:428`,
  `btRigidBody.cpp:105-118`). A velocity set by hand is overwritten.
- In the solver it is a solver body with zero inverse mass (`btSequentialImpulseConstraintSolver.cpp:771`).
- The conveyor hook is `m_contactMotion1/2` set in `gContactAddedCallback`, used as the friction
  target velocity (`:1089-1093`, `:1139-1142`). The solver comment calls it "a conveyor belt effect".
- Pitfall: kinematic bodies fall asleep and their velocity freezes unless
  `DISABLE_DEACTIVATION` is set (`btCollisionObject.cpp:63, 74`).

**Box3D**
- Kinematic bodies are integrated like dynamic ones, with gravity zeroed (`src/solver.c:97, 244-245`).
- Contact results are written back only to dynamic bodies (`src/contact_solver.c:676-686`).
- `b3Body_SetTargetTransform` (`src/body.c:1252-1322`) computes `v = (c2-c1)/dt` and ω from the
  shortest-arc quaternion difference.
- `b3SurfaceMaterial.tangentVelocity` (`include/box3d/types.h:425-427`) is projected onto the
  contact tangents and subtracted only in the friction rows (`src/contact_solver.c:642`).
  Samples: "Conveyor Belt" and "Conveyor Mesh" (`samples/sample_shapes.cpp:341-584`).
- Kinematic-dynamic contacts use the softer `contactSoftness`, not `staticSoftness`
  (`src/contact_solver.c:134-135`).

**LIGGGHTS**
- A wall's velocity is a per-node mesh property `"v"`. The contact reads it, interpolated at the
  contact point (`fix_wall_gran.cpp:992-996`).
- `surface_vel` makes it tangential on a mesh that never moves (`fix_mesh_surface.cpp:895-945`).
- `fix move/mesh` moves the nodes and adds its velocity, so movers superpose (`fix_move_mesh.cpp:229-233`).
- Combining `surface_vel` with `move/mesh` is an error (`fix_move_mesh.cpp:102-105`).
- The `stress` and `servo` mesh modules turn the wall force into a diagnostic or a PID-driven
  wall motion (`mesh_module_stress_servo.cpp:502-512`).

**waLBerla `mesa_pd`**
- pe's own port inside waLBerla was removed (`CHANGELOG.md:69`), so there is no upstream fix to borrow.
- `FIXED` means "skip integration" and is independent of the inverse mass
  (`src/mesa_pd/data/Flags.h:47-55`, `kernel/ExplicitEuler.h:96`). A FIXED body with finite mass
  is mis-weighted in HCSITS.
- Couette walls are FIXED half-spaces with a tangential velocity
  (`apps/benchmarks/FluidParticleCoupling/ForcesOnSphereNearPlane.cpp:321-322`). That velocity
  also enters the normal row.
- The Mixer showcase rotates its blades without giving them a velocity (`Mixer.cpp:440-449`), so
  contacts see stationary blades.

**Chrono**
- `SetFixed()` disables the body's solver variables (`ChBody.cpp:577-580`), and complementarity
  rows ignore its velocity.
- Prescribed motion is a fixed ground plus a motor link whose rheonomic row carries
  `Ct = -f'(t)` (`ChLinkMotorLinearPosition.cpp:46-50`).
- `ChConveyor` hides a heavy plate (mass 1e5) that is reset to the truss pose every step and
  driven by a lock with a ramp (`ChConveyor.cpp:266-289`). That is the same workaround pe's
  conveyor example used.
- Fixed bodies never wake neighbours (`ChSystem.cpp:535-549`).

**Jolt**
- `MoveKinematic` turns a target pose into velocities (`MotionProperties.inl:9-21`, quaternion
  forced to w ≥ 0).
- Contacts write results only to dynamic bodies (`ContactConstraintPart.h:94-103`).
- `ContactSettings::mRelativeLinearSurfaceVelocity` and `mRelativeAngularSurfaceVelocity`
  (`ContactListener.h:43-51`) become the bias of the friction rows
  (`ContactConstraintManager.cpp:162-173`). The normal row never sees them, and CCD ignores them.
  Sample: `Samples/Tests/General/ConveyorBeltTest.cpp`, a static belt.
- Per-contact inverse-mass scales (`mInvMassScale1/2`) can make a dynamic body kinematic in a
  single contact.

**PhysX 5**
- `setKinematicTarget` stores the target. The velocity is computed when the step starts
  (`ScKinematics.cpp:22-77`), and the pose is set exactly to the target after the solve (`:194`).
- `setLinearVelocity` on a kinematic body is refused.
- Contact modification `setTargetVelocity` acts on the normal and the friction rows
  (`DyContactPrepShared.h:286, 323`, `DyContactPrep.cpp:264-326`). It disables persistent friction
  anchors (`DyContactPrep.cpp:622`), and the pair order is not stable, so the user has to fix the sign.
- Pair filtering: `kineKineFilteringMode` and `staticKineFilteringMode` (`PxSceneDesc.h:631-640`).

**Rapier**
- Kinematic bodies come in two types: driven by position (the next pose gives the velocity,
  `rigid_body_components.rs:160-208`; the user's pose is kept at writeback) or driven by velocity.
- On `Fixed` and position-driven bodies, `set_linvel` is silently ignored (`rigid_body.rs:1022-1087`).
- Surface velocity is `SolverContact.tangent_velocity` set in `modify_solver_contacts`, in the
  friction rows only (`contact_with_coulomb_friction.rs:273-280`). The contact anchor also moves
  by `tangent_vel·dt` per substep (`:411`). Example: `examples3d/one_way_platforms3.rs`.
- Velocity caps and damping also apply to kinematic bodies (`worker.rs:649-679, 974`).

**Yade**
- `State::blockedDOFs` is a runtime bit mask per body (`core/State.hpp:20-26`).
  `NewtonIntegrator` ignores forces per blocked axis but always integrates `pos += vel·dt`
  (`NewtonIntegrator.cpp:41-73, 337`).
- Kinematic engines (`TranslationEngine`, `HarmonicMotionEngine`, `RotationEngine`, shear boxes)
  only set velocities, and reset them every step (`KinematicEngines.cpp:23-158`).
- Conveyor: facets with a velocity excluded from integration by `NewtonIntegrator(mask=…)`
  (`examples/conveyor/conveyor.py:33-48`).
- Contact forces are accumulated on fixed bodies too, and the triaxial and PID controllers read
  them (`TriaxialStressController.cpp:93-125`).

**LMGC90**
- A driven degree of freedom gets zero inverse mass in that component
  (`mod_RBDY3.f90:3496-3500`). Its imposed velocity overwrites the free velocity, so it enters
  the local free relative velocity through the ordinary Jacobian (`mod_SPPLx.f90:1564-1577`).
- Position is integrated with the theta method (`mod_RBDY3.f90:1670, 2460-2461`); a velocity
  without motion is not possible.
- The contact impulse on a driven direction (`Ireac`) is the force that keeps the motion and is
  written out.
- Driven angular components are in the body's principal frame.

**MercuryDPM**
- Every body has velocity and angular velocity; a wall always moves with it
  (`BaseInteractable.cc:549, 577`).
- Prescribed position or velocity are `std::function`s of time, evaluated at the half step
  (`BaseInteractable.cc:516-631`).
- The wall's velocity enters the normal and the tangential contact terms
  (`LinearViscoelasticInteraction.cc:86`).
- Shear without walls: `LeesEdwardsBoundary` (`LeesEdwardsBoundary.cc:168-174`).
- Fixed–fixed pairs are skipped (`DPMBase.cc:3118-3123`).

## What pe took over and where it differs

Adopted:
- **Runtime semantics with no compiler dependence.** A fixed body with a velocity is the
  kinematic body. This matches Box3D, Jolt, Rapier and Yade; PhysX and Rapier also keep a separate
  type for a static body with no velocity.
- **Velocity and target pose as the two ways to drive a body.** `setTargetPose` follows
  Box3D and Jolt: velocity from the pose difference and the shortest-arc rotation.
- **Surface velocity in the friction rows only.** This is Box3D, Jolt and Rapier; Bullet is
  close. PhysX also applies it to the normal row. That pe has no such option is deliberate:
  a conveyor should not push into or pull out of the belt.
- **Fixed–fixed pairs dropped before the contact solver,** as everywhere else.
- **Static or dynamic friction coefficient from the true slip.** `Contact::Contact()` selects the
  coefficient of the material pair from the tangential relative speed of the body velocities plus
  the relative surface velocity (body 1 minus body 2, as stored in `Contact::getSurfaceVel()`), in
  both the vertex/face and the edge/edge constructor. A box riding a belt at belt speed gets the
  static coefficient, a box held on a running belt the dynamic one
  (`tests/interface/pe_surface_velocity_test.cpp`, section 8). The sign of the surface velocity
  follows the contact's body order, so unlike PhysX the user never has to fix it.
- **Sleeping as in Box3D, Jolt, PhysX and Rapier.** A driving body never sleeps and wakes what it
  touches, and the woken body is held awake for the step (otherwise friction, which accelerates
  it gradually, never gets it above the sleep threshold before it falls asleep again). Sleep mode
  itself is compiled out by default (`pe::sleepThreshold = 0`; `-DPE_SLEEP_THRESHOLD=<value>`
  enables it). That avoids the Bullet pitfall (kinematic body frozen asleep) and the Chrono pitfall
  (fixed bodies cannot wake anything).

Checked in pe and found consistent with the other engines:
- **Bounces off a moving wall.** The Newton-restitution target uses the pre-step velocities of
  both bodies, the fixed one included (`HardContactSemiImplicitTimesteppingSolvers.h`,
  restitution set-up, `gdotPre` from `getLinearVel()`/`getAngularVel()`), as in Box3D, Chrono and
  PhysX. Only its normal component is used, so a surface velocity never enters it.
- **No persistent friction anchors in pe's friction.** The hard-contact solvers relax velocity-level
  impulses that start from zero every step (no warm start, no anchors), so PhysX's and Rapier's
  anchor handling for surface velocity is not needed.

Differences that are a choice, not a gap:
- **The velocity set by `setTargetPose` persists after the step.** Jolt does the same. PhysX
  zeroes it when no new target arrives, and Yade's engines reset it every step. Call
  `setTargetPose` every step for a path; set the velocity to zero to stop.
- **Position after a target pose is integrated, not snapped** (unlike PhysX and Rapier). It
  lands within rounding of the target.

## Gaps and follow-ups

1. **Friction coefficient on a moving surface: done.** `Contact::Contact()` chooses between the
   static and the dynamic coefficient from the slip including the surface velocity (after master's
   153551d fixed the tangential speed, which used to be zero). A box riding a belt at belt speed
   sticks, a box held on a running belt slides; tested in both contact constructors and on an
   inclined belt. A velocity-dependent transition between the coefficients remains open.
2. **Force on fixed bodies.** LMGC90, Yade, Chrono, LIGGGHTS and MercuryDPM accumulate it. It gives
   belt power and Couette wall shear stress, and it is the basis for walls servoed to a target force.
3. **Angular surface velocity,** as Jolt and LIGGGHTS `surface_ang_vel` have it: a turning surface
   on a body that does not rotate, for drums and rollers that are not round.
4. **Driving per direction,** as Yade and LMGC90 do it, e.g. a piston driven in x and free in z.
5. **Speed caps and damping.** Rapier applies its caps to kinematic bodies too. pe's three
   hard-contact collision systems exempt fixed bodies from world damping (`World::setDamping()`);
   the other solvers were not changed. Any future velocity cap should exempt them as well.
6. **The table tennis bats: done.** The server's and the machine-mode player's bat in
   `tools/table_tennis` are fixed unions with realistic densities driven by `setTargetPose` every
   step, instead of one-tonne dynamic unions whose pose and velocity were overwritten. Returns,
   spin and verdicts are unchanged; landings move by up to 2 mm (1 cm after a serve's own-side
   bounce), because the dynamic bat entered the contact with the solver's gravity increment
   g dt on top of the stroke velocity.
