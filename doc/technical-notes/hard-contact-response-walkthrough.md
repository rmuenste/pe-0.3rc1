# Hard-Contact Response Walkthrough

This note explains the sequential-impulse style hard-contact response used by
PE's semi-implicit timestepping solvers. The main example is the decoupled
inelastic Coulomb contact relaxation path, because it exposes the important
pieces of the algorithm: contact frames, effective masses, static friction,
dynamic friction, Baumgarte stabilization, and accumulated velocity
corrections.

The note also includes a modeling case study for prescribed-motion bodies. That
case was originally investigated in the `capsule_spin` setup and is useful
because it shows where the numerical algorithm, the rigid-body model, and the
chosen kinematic constraints interact in non-obvious ways.

## Implementation Scope

Relevant implementation files:

- `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`
- `pe/core/collisionsystem/HardContactAndFluid.h`
- `pe/core/response/HardContactSemiImplicitTimesteppingSolvers.h`
- `src/core/rigidbody/RigidBody.cpp`

The solver selected for a build is configured through `pe_CONSTRAINT_SOLVER` in
`pe/config/Collisions.h`. There is no universal runtime default. The
hard-contact-only baseline is
`pe::response::HardContactSemiImplicitTimesteppingSolvers`; the
`HardContactAndFluid` branch contains related historical experiments and
kinematic-body handling that are useful for understanding the case study below.

The function names to inspect first are:

- `resolveContacts()`
- `relaxInelasticCoulombContactsByDecoupling()`
- `relaxInelasticCoulombContactsByOrthogonalProjections()`
- `relaxInelasticGeneralizedMaximumDissipationContacts()`

## Contact Geometry And Notation

For each contact, the solver stores the two bodies, the contact point, the
normal direction, the friction coefficient, and the contact distance.

Let:

- `b1`, `b2`: the two rigid bodies.
- `x`: the contact point.
- `r1 = x - x1`, `r2 = x - x2`: moment arms from each body center to the
  contact point.
- `n`: contact normal, pointing from body 2 toward body 1.
- `t`, `o`: tangent directions spanning the contact plane.
- `R = [n | t | o]`: contact-frame matrix.
- `p`: contact impulse in world coordinates.
- `p_cf = R^T p`: contact impulse in normal/tangent/tangent coordinates.
- `mu`: Coulomb friction coefficient.

The relative contact velocity in world coordinates is:

```text
gdot = (v1 + dv1) - (v2 + dv2)
     + (w1 + dw1) x r1 - (w2 + dw2) x r2
```

The contact-frame relative velocity is:

```text
gdot_cf = R^T gdot = [gdot_n, gdot_t, gdot_o]^T
```

The accumulated velocity corrections `dv` and `dw` are the effect of contact
impulses during the current time step. They are stored separately from the base
body velocities while the iterative solver relaxes the contact set.

## Effective Mass

The effective mass maps an impulse at the contact point to the resulting change
in relative contact velocity. In world coordinates the inverse effective mass
operator has the form:

```text
K = (m1^-1 + m2^-1) I
  - [r1]x I1^-1 [r1]x
  - [r2]x I2^-1 [r2]x
```

where `[r]x` is the cross-product matrix for `r`. The solver transforms this
operator into the contact frame:

```text
K_cf = R^T K R
```

The cached inverses used by the relaxation routines include:

- full normal/tangent/tangent inverse: `K_cf^-1`
- normal-only inverse: `K_n^-1`
- tangent-plane inverse: `K_to^-1`

The translational part vanishes for translation-locked bodies because
`getInvMass()` returns zero. A fully fixed body also has zero inverse inertia,
so an impulse does not change its linear or angular velocity. A
translation-fixed body has zero inverse mass but may still have non-zero inverse
inertia, so contact impulses can still create angular velocity changes.

## Decoupled Inelastic Coulomb Relaxation

The decoupled Coulomb path is an iterative relaxation algorithm. Each contact is
processed repeatedly, and each update immediately modifies the accumulated
velocity corrections. This gives the method its sequential-impulse character:
later contacts in the same sweep see the updates produced by earlier contacts.

### 1. Remove The Previous Impulse

Before recomputing a contact's impulse, the solver removes the contribution of
the previous impulse estimate:

```text
dv1 -= m1^-1 p
dw1 -= I1^-1 (r1 x p)
dv2 += m2^-1 p
dw2 += I2^-1 (r2 x p)
```

This returns the local state to "all other contacts, but not this contact" so a
new impulse can be computed consistently.

### 2. Compute Relative Velocity

The solver recomputes `gdot` at the contact point from the base velocities plus
the current accumulated corrections. It then transforms the velocity into the
contact frame:

```text
gdot_cf = R^T gdot
```

If positional correction is active, the normal component also receives the
Baumgarte term:

```text
gdot_n <- gdot_n + dist / dt
```

For penetrating contacts, `dist` is negative, so the bias asks the solver to
create a separating velocity during this time step.

### 3. Separating Contact Check

If the biased normal velocity is non-negative, the contact is separating:

```text
if gdot_n >= 0:
    p = 0
```

No impulse is required because the contact is already moving apart according to
the solver's local criterion.

### 4. Static Friction Attempt

For a persisting contact, the solver first tries a static solution. It solves
the full 3x3 contact-frame system:

```text
p_cf = -K_cf^-1 gdot_cf
```

The resulting impulse is accepted as static if it lies inside the Coulomb cone:

```text
p_n >= 0
sqrt(p_t^2 + p_o^2) <= mu p_n
```

If this succeeds, the tangential relative velocity can be driven to zero within
the friction limit.

### 5. Dynamic Friction Fallback

If the static impulse leaves the friction cone, the contact is treated as
sliding.

The decoupled path first solves the normal component while accounting for the
effect of the current tangential impulse estimate:

```text
p_n = max(0, -K_n^-1 gdot_n_corrected)
```

It then solves the tangential part with the normal impulse fixed:

```text
p_to = -K_to^-1 gdot_to_corrected
```

Finally, the tangential impulse is projected back to the friction disk if it is
outside the Coulomb limit:

```text
if ||p_to|| > mu p_n:
    p_to <- (mu p_n / ||p_to||) p_to
```

This is approximate because the normal and tangential parts are solved in
separate stages. The benefit is a relatively simple sliding-contact update that
fits the sequential relaxation structure.

### 6. Apply The New Impulse

The accepted contact-frame impulse is transformed back to world coordinates:

```text
p = R p_cf
```

It is stored as the contact's current impulse estimate and applied to the
accumulated velocity corrections:

```text
dv1 += m1^-1 p
dw1 += I1^-1 (r1 x p)
dv2 -= m2^-1 p
dw2 -= I2^-1 (r2 x p)
```

The sign convention follows the stored contact normal and body ordering.

### 7. Iterate To Convergence

The solver sweeps over all contacts multiple times. Convergence is tracked by
the largest change in contact impulse during a sweep:

```text
delta_max = max_i ||p_new_i - p_old_i||
```

The configured maximum iteration count and relaxation parameter determine how
much work the solver does and how aggressively velocity corrections are applied.

## Baumgarte Stabilization

Baumgarte stabilization converts penetration depth into a velocity-level
correction:

```text
bias = erp * dist / dt
```

For a penetrating contact, `dist < 0`, so the bias pushes the normal velocity
toward separation. This is useful because the solver itself is velocity based:
without some position-error feedback, small penetrations can persist or drift.

The same mechanism can also inject artificial energy when the correction asks
for motion that the constrained bodies cannot realize. The amplification by
`1 / dt` is important. With `dt = 0.001`, a penetration of `0.001` length units
becomes a correction velocity on the order of `1.0` before ERP scaling.

The hard-contact solver contains optional adaptive Baumgarte capping hooks that
limit correction velocity. In `HardContactAndFluid`, historical kinematic-body
experiments also disabled Baumgarte for contacts where both bodies have zero
translational degrees of freedom. The physical reason is straightforward:
Baumgarte asks for translational separation, but two translation-locked bodies
cannot separate translationally. The correction impulse can then appear as
rotation instead of positional repair.

## Prescribed Motion Is Not Infinite Mass

A fixed or translation-fixed body in PE is represented by zero inverse mass, and
possibly zero inverse inertia. That is a numerical response model: impulses have
limited or no effect on selected degrees of freedom.

A prescribed-motion body is a stronger modeling statement. It says that a body
will keep a velocity because something outside the contact problem imposes it,
for example a motor-driven rotor. A contact impulse cannot slow that prescribed
motion down unless the model explicitly allows it.

This distinction matters for static friction. Static contact attempts to find an
impulse that makes the relative contact velocity zero:

```text
gdot_after = 0
```

For ordinary dynamic bodies, both sides can share the velocity correction. For a
prescribed-motion body, one side may not respond to the impulse at all. The
other body then receives the full correction needed to satisfy a contact goal
that may be incompatible with the intended physical model.

This is not primarily a convergence problem. The solver can converge perfectly
to the wrong physical target if the modeled constraints are inconsistent.

## Case Study: Rotating Fixed Capsule Against Translation-Fixed Capsule

The original investigation used a setup like:

```cpp
CapsuleID capsule = createCapsule( id++, 0.0, 0.0, 0.0, 0.2, 1.0, granite );
capsule->setFixed( true );
capsule->setAngularVel( 0.0, 0.0, omega );

CapsuleID capsule2 = createCapsule( id++, 0.7, 0.7, 0.0, 0.2, 1.0, granite );
capsule2->setTranslationFixed( true );
```

The first capsule behaves like a prescribed rotating body: inverse mass and
inverse inertia are zero, but the example imposes angular velocity. The second
capsule cannot translate, but it can rotate.

At first glance, a result where the second capsule reaches a larger angular
speed than the driving capsule looks like energy creation. The investigation
showed two separate effects.

### Baumgarte Contribution

With penetration present, Baumgarte generated a separating velocity request.
Because both bodies had zero translational freedom, that correction could not be
realized as translation. Disabling Baumgarte for the zero-translation-DOF case
reduced the observed spin-up substantially in the historical test.

This is a model-quality issue: a velocity-level penetration correction is not a
substitute for missing geometric separation degrees of freedom.

### Static Friction Goal

The static friction branch tries to make the contact point stick. With a
prescribed rotating body, sticking can be incompatible with the imposed motion.
The historical `HardContactAndFluid` experiment added kinematic-body detection
and forced such contacts into the dynamic friction path, treating them as
sliding rather than trying to enforce a static tangential constraint.

That model is more consistent with a motor-like body whose motion is imposed
externally.

### Lever-Arm Interpretation

The most important physical correction came from inspecting the contact
geometry. The contact point did not have equal moment arms on the two capsules:

```text
|r1| ~= 0.6402
|r2| ~= 0.3611
|r1| / |r2| ~= 1.773
```

For perfect rolling contact, tangential contact velocities would match:

```text
|omega1| |r1| = |omega2| |r2|
```

so:

```text
|omega2| = |omega1| |r1| / |r2|
```

With `|omega1| ~= 6.28`, perfect rolling would give
`|omega2| ~= 11.14`. The observed value after disabling the problematic
Baumgarte contribution was about `8.32`, which is below the perfect-rolling
value. That is consistent with sliding and frictional dissipation.

The lesson is that comparing angular speeds alone is not enough. Contact
geometry and moment arms determine the contact-point velocities. A larger
angular velocity on the smaller lever arm can be physically reasonable.

## Modeling Lessons

- Solver convergence does not prove physical correctness. It only proves that
  the numerical iteration reached its own target.
- Static contact constraints should be used carefully with prescribed-motion
  bodies.
- Baumgarte stabilization is a velocity-level repair mechanism; it can become
  unphysical when the requested separation cannot occur in the available
  degrees of freedom.
- For rotational contacts, compare tangential contact velocities, not angular
  velocities alone.
- Visualizing contact points and moment arms can be more informative than
  looking only at impulse magnitudes.

## Historical Notes

The original investigation was written around the `capsule_spin` example and
the `HardContactAndFluid` implementation branch. It also mentioned a
DistanceMap-based `mesh_spin` validation experiment. Some of those line numbers
and status details were intentionally removed from this article because they
were tied to a particular debugging snapshot.

The durable content is the algorithm walkthrough and the modeling analysis:

- how the decoupled inelastic Coulomb contact update works,
- why Baumgarte terms require care,
- why prescribed-motion bodies need a clear contact model,
- and why lever-arm geometry can make `|omega2| > |omega1|` physically
  plausible.

## References

- `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`
- `pe/core/collisionsystem/HardContactAndFluid.h`
- `pe/core/response/HardContactSemiImplicitTimesteppingSolvers.h`
- `src/core/rigidbody/RigidBody.cpp`
- `src/core/rigidbody/CapsuleBase.cpp`
- `examples/capsule_spin/capsule_spin.cpp`
