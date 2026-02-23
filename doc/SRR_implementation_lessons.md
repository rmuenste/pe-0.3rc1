# Lessons Learned: Short-Range Repulsion Solver Implementation

Implementation notes and debugging post-mortem for the `response::ShortRangeRepulsion`
collision system (Pan et al. 2002, Eq. 2.1), validated against a single-sphere
floor-impact test.

---

## 1. The `PE_LUBRICATION_CONTACTS` Include-Order Bug

### What happened

After wiring up the SRR solver and running the first test, the sphere fell straight
through the floor — the security-zone force never fired.  All the SRR code compiled
and linked cleanly, but at runtime `resolveContact` was never called.

### Root cause

The security-zone detection lives in `pe/core/detection/fine/MaxContacts.h`, which
contains two mutually exclusive code paths, selected at compile time:

```cpp
// pe/core/detection/fine/MaxContacts.h (simplified)
#ifdef PE_LUBRICATION_CONTACTS
   // generates LubricationContact for  0 < dist < lubricationThreshold
   ...
#else
   // only generates VertexFaceContact for dist < 1e-8 (hard contact)
   ...
#endif
```

`PE_LUBRICATION_CONTACTS` is defined inside both
`pe/core/collisionsystem/HardContactLubricated.h` and
`pe/core/collisionsystem/ShortRangeRepulsion.h`.  The problem was the include order
in `pe/core/CollisionSystem.h`:

```cpp
// BEFORE (buggy) — DEMSolverObsolete pulls MaxContacts.h FIRST,
// before the flag is defined; the include guard then prevents re-parsing.
#include <pe/core/collisionsystem/DEMSolverObsolete.h>   // ← MaxContacts.h parsed here
#include <pe/core/collisionsystem/DEMSolver.h>           //   without PE_LUBRICATION_CONTACTS
#include <pe/core/collisionsystem/FFDSolver.h>
#include <pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h>
#include <pe/core/collisionsystem/HardContactAndFluid.h>
#include <pe/core/collisionsystem/HardContactLubricated.h>  // ← too late: flag defined here
#include <pe/core/collisionsystem/ShortRangeRepulsion.h>    // ← too late
```

Because `DEMSolverObsolete.h` transitively includes `MaxContacts.h` before the flag
is defined, the include guard (`#ifndef _PE_CORE_DETECTION_FINE_MAXCONTACTS_H_`)
prevents the file from being re-parsed when the lubrication headers are finally
reached.  The non-lubrication path is locked in for the entire translation unit.

### Fix

Move the lubrication-defining headers to the top of the include list:

```cpp
// AFTER (correct) — lubrication headers first, define the flag
// before MaxContacts.h is parsed for the first time.
#include <pe/core/collisionsystem/HardContactLubricated.h>
#include <pe/core/collisionsystem/ShortRangeRepulsion.h>
#include <pe/core/collisionsystem/DEMSolverObsolete.h>
// ... rest unchanged
```

### General lesson

When two headers define mutually exclusive compile-time code paths via a preprocessor
flag and an include guard, the header that *defines* the flag must always be included
*before* the header that *consumes* it, regardless of what seems logical from a
dependency perspective.  Add a comment in `CollisionSystem.h` documenting this
constraint so it is not accidentally "fixed" in the future.

---

## 2. Sub-Cycling with Frozen Contacts

### What happened

After fixing the include order the SRR force fired, but the sphere still exploded
(velocity ≈ 477 m/s after one main step) instead of decelerating.

### Root cause

The `simulationStep` implementation detected contacts **once** before the sub-cycle
loop and then applied those same frozen contact objects for all N sub-steps:

```cpp
// BEFORE (buggy) — gap computed once, never updated
detector_.findContacts( contacts_ );              // ← outside loop

const real dt_sub = dt / static_cast<real>( nSubcycles_ );
for( size_t sub = 0; sub < nSubcycles_; ++sub ) {
    for( auto c = contacts_.begin(); c != contacts_.end(); ++c ) {
        solver_.resolveContact( *c );             // c->getDistance() is stale
    }
    body->move( dt_sub );
}
clearContacts();
```

`ContactBase::getDistance()` returns the stored `dist_` field set at detection time:

```cpp
inline real ContactBase::getDistance() const
{
   return dist_;   // set once at construction, never updated
}
```

During sub-cycling the sphere moves, but the force magnitude is computed from a gap
that no longer reflects the current position.  When the sphere reverses direction
inside the security zone the stale gap says "still approaching" while the sphere is
actually receding, and the force continues to push it outward, compounding with each
sub-step.

### Fix

Re-detect contacts at the start of every sub-step:

```cpp
// AFTER (correct) — fresh contact objects, current gap at each sub-step
const real dt_sub = dt / static_cast<real>( nSubcycles_ );
for( size_t sub = 0; sub < nSubcycles_; ++sub ) {
    clearContacts();                              // delete old contact objects
    detector_.findContacts( contacts_ );          // re-detect at current positions
    for( auto c = contacts_.begin(); c != contacts_.end(); ++c ) {
        solver_.resolveContact( *c );             // gap is now current
    }
    body->move( dt_sub );
}
clearContacts();                                  // clean up after last sub-step
```

Note: `clearContacts()` must be called (not just `contacts_.clear()`) because the
contact objects are heap-allocated and must be `delete`d to avoid memory leaks.

### General lesson

When sub-cycling any force that depends on a geometry quantity (gap, overlap, normal),
that quantity must be recomputed at each sub-step from the bodies' current positions.
Caching the detection result outside the sub-cycle loop is only safe for forces that
are independent of the current configuration (e.g., body-fixed gravity).

The cost of re-detection at each sub-step is real, but for soft-contact models the
alternative — a frozen gap — is unphysical and numerically unstable.

---

## 3. Velocity Damping Sign Error

### What happened

After fixing sub-cycling the sphere still exploded, but more slowly.  With `gamma =
0.5` the rebound velocity after the first bounce was +64 m/s instead of the expected
few cm/s.

### Root cause

The damping term in `resolveContact` had the wrong sign:

```cpp
// BEFORE (buggy)
// relVelN > 0 when bodies approach  (relVelN = -getNormalRelVel())
const real relVelN = -c->getNormalRelVel();
fmag -= gamma_ * relVelN;          // ← sign is inverted
if( fmag < real(0) ) fmag = real(0);
```

`getNormalRelVel()` returns `n · (v₁ - v₂)`.  For a sphere (b1) above a fixed floor
(b2) with normal pointing upward:
- approaching: `getNormalRelVel() < 0`  →  `relVelN > 0`
- receding:    `getNormalRelVel() > 0`  →  `relVelN < 0`

With `fmag -= gamma_ * relVelN`:
- **approaching** (relVelN > 0): force *decreases* — damping reduces the spring
  resistance, allowing deeper penetration.
- **receding** (relVelN < 0): `fmag -= negative` → force *increases* — the sphere
  is pushed outward harder when it is already moving outward.  This is the opposite
  of a dashpot: an *anti-dashpot* that amplifies oscillations exponentially.

The analytical result for the simplest case (linear approximation, small δ):

```
dv/dt ≈ F_spring/m + (γ/m)·v    (receding phase)
```

This is an unstable ODE whose solution grows as `exp(γ/m · t)`.  With the SI
parameters (m ≈ 1.52×10⁻⁴ kg, γ = 0.5) the characteristic time is 3×10⁻⁴ s, fast
enough to cause a factor of ≈ 22× velocity amplification within one main time step.

### Fix

Flip the sign to the standard spring-dashpot convention:

```cpp
// AFTER (correct) — standard spring-dashpot model
// approaching (relVelN > 0): add force → resist approach
// receding    (relVelN < 0): reduce force → resist separation
const real relVelN = -c->getNormalRelVel();
fmag += gamma_ * relVelN;          // ← correct sign
if( fmag < real(0) ) fmag = real(0);
```

- **approaching**: `fmag += gamma * |vz|` — extra upward force decelerates approach. ✓
- **receding**: `fmag -= gamma * |vz|` — less upward force, clamped to 0 (no adhesion). ✓

### General lesson

When adding a damping force to a repulsive spring:
1. Write out the sign of `getNormalRelVel()` for your geometry (approaching vs.
   receding) before touching any code.
2. Verify by desk-check: "during approach the force should *increase*; during
   recession it should *decrease*."
3. If the damped simulation is *less* stable than the undamped one, the sign is
   almost certainly wrong.

---

## 4. CGS → SI Unit Conversion

### What happened

The first working simulation (after the three bugs above were fixed) used parameters
copied directly from Pan et al. (2002) §4 without unit conversion.  The sphere was
100× too large and 430× too light, resulting in a qualitatively wrong simulation.

### Conversion table

| Quantity | Pan et al. (CGS) | SI | Factor |
|---|---|---|---|
| radius *a* | 0.3175 cm | 3.175×10⁻³ m | ×10⁻² |
| density ρₛ | 1.14 g/cm³ | 1140 kg/m³ | ×10³ |
| security zone ρ | 0.06858 cm | 6.858×10⁻⁴ m | ×10⁻² |
| stiffness ε | 5×10⁻⁷ dyne⁻¹ | 5×10⁻² N⁻¹ | ×10⁵ |

The stiffness conversion is the most surprising: 1 dyne⁻¹ = 10⁵ N⁻¹ (because
1 dyne = 10⁻⁵ N, so 1/dyne = 10⁵/N).  Getting this wrong by the factor of 10⁵ makes
the force either completely negligible or catastrophically stiff.

### General lesson

Always state units explicitly in variable declarations and comments.  Prefer to write
the conversion inline so it is auditable:

```cpp
// Pan et al. (2002) Eq. 2.1, SI conversion from CGS:
const real epsSRR( real(5e-7)       // dyne⁻¹ (paper value)
                   * real(1e5) );   // dyne⁻¹ → N⁻¹  (= 5e-2 N⁻¹)
```

---

## 5. Sub-Cycle Count and Timestep Sizing

### Analysis

The force peaks at full compression (gap = 0, δ = ρ):

```
F_max = ρ² / (ε · ρ²) = 1/ε = 1 / 5×10⁻² = 20 N
a_max = F_max / m = 20 / 1.52×10⁻⁴ ≈ 131 000 m/s²
```

The sub-step velocity increment must remain small relative to the entry velocity
(≈ 0.34 m/s at gap = ρ):

```
Δv_sub = a_max · dt_sub  ≪  v_entry
```

With N = 5000 sub-steps and dt = 10⁻³ s:

```
dt_sub = 10⁻³ / 5000 = 2×10⁻⁷ s
Δv_sub = 131 000 × 2×10⁻⁷ ≈ 0.026 m/s   (≈ 8% of v_entry)
```

This was found to be sufficient for stable integration.  N = 500 (dt_sub = 2×10⁻⁶ s)
produced visible energy non-conservation; N = 1000 was marginal; N = 5000 gave clean
settling curves.

### General lesson

Estimate `a_max = F_max / m` analytically before running any simulation.  Then choose
N such that `a_max · dt/N ≪ v_entry`.  A factor of 10–20 margin is typical; a factor
of 100 (as used here) is conservative but safe for a validation case.

---

## 6. Validation: Static Equilibrium Check

### Method

After settling, the sphere must satisfy `F_SRR = mg` at whatever gap it rests at.
This is a zero-free-parameter check that exercises the entire force computation path.

With γ = 0.5 N·s/m (near-critical damping), the sphere settled at gap/ρ ≈ 0.9913
within approximately 70 main time steps.  At that gap:

```
δ = ρ − gap = 6.858×10⁻⁴ − 6.799×10⁻⁴ ≈ 5.9×10⁻⁶ m
F_SRR = (δ/ρ)² / ε = (8.6×10⁻³)² / 5×10⁻² ≈ 1.48×10⁻³ N
mg    = 1.52×10⁻⁴ × 9.81             ≈ 1.49×10⁻³ N  ✓
```

The agreement to three significant figures confirms that:
- the contact detection threshold is correctly set to ρ,
- the force formula matches Pan et al. Eq. 2.1,
- the integration is conservative enough that equilibrium is maintained.

### General lesson

For any soft-contact model, always derive the expected equilibrium gap analytically
and confirm it in simulation before running multi-body or multi-step tests.  A static
equilibrium test catches sign errors, unit errors, and threshold mismatches without
any ambiguity.
