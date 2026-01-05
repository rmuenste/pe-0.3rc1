# Collision Response Issue: Fixed Body vs Translation-Fixed Body

## Problem Summary

When a kinematic body with prescribed rotational motion (Capsule 1) collides with a translation-fixed body (Capsule 2), the collision response generates excessive angular velocity in Capsule 2, violating energy conservation.

**Observed Behavior (before fixes):**
- Capsule 1: rotating at ω₁ = 6.28 rad/s (60 rpm), remains unchanged after collision
- Capsule 2: starts at rest, suddenly jumps to ω₂ = -11.73 rad/s after collision
- **Energy is created** in the system: |ω₂| > |ω₁| and Capsule 1 doesn't slow down

**After disabling Baumgarte stabilization:**
- Capsule 2: jumps to ω₂ = -8.32 rad/s (29% improvement)
- Still violates energy conservation: |ω₂| > |ω₁|

**Root Cause:** The collision solver treats Capsule 1 as a kinematic body with prescribed motion (motor-driven rotor) but attempts to enforce a static contact constraint (v_rel = 0), which is fundamentally incompatible with prescribed motion.

## Physical Scenario

### Initial Setup (from `examples/capsule_spin/capsule_spin.cpp`)

```cpp
// Capsule 1: Fully fixed, spinning at origin
CapsuleID capsule = createCapsule( id++, 0.0, 0.0, 0.0, 0.2, 1.0, granite );
capsule->setFixed( true );                    // invMass = 0, invInertia = 0
capsule->setAngularVel( 0.0, 0.0, omega );   // ω = 6.28 rad/s

// Capsule 2: Translation-fixed, can rotate
CapsuleID capsule2 = createCapsule( id++, 0.7, 0.7, 0.0, 0.2, 1.0, granite );
capsule2->setTranslationFixed( true );        // invMass = 0, invInertia ≠ 0
```

### Body Properties After Fixing

**Capsule 1 (`setFixed(true)`)** - from `src/core/rigidbody/RigidBody.cpp:450`:
```cpp
void RigidBody::fix() {
   fixed_ = true;
   invMass_ = real(0);     // Infinite mass
   Iinv_    = real(0);     // Infinite inertia
   v_ = real(0);
   w_ = real(0);           // Unless MOBILE_INFINITE is set
}
```

**Capsule 2 (`setTranslationFixed(true)`)** - from `src/core/rigidbody/RigidBody.cpp:201`:
```cpp
void RigidBody::setTranslationFixed( bool fixed ) {
   if( fixed ) {
      invMass_ = real(0);     // Infinite mass (prevents translation)
      v_ = real(0);           // Zero linear velocity
      // Keep Iinv_ and w_ unchanged for rotation
   }
}
```

### Capsule Moment of Inertia

From `src/core/rigidbody/CapsuleBase.cpp:147`:

For a capsule with radius `r` and cylindrical length `L`:

```cpp
const real density = calcDensity( r, L, mass );
const real sphereMass = (4/3) * π * r³ * density;
const real cylinderMass = π * r² * L * density;

// About axis along capsule (x-axis):
Ia = r² * (0.5 * cylinderMass + 0.4 * sphereMass);

// About perpendicular axes (y and z):
Ib = cylinderMass * (0.25*r² + (1/12)*L²)
     + sphereMass * (0.4*r² + 0.375*r*L + 0.25*L²);

I = diag(Ia, Ib, Ib);
```

For our capsules: `r = 0.2`, `L = 1.0`, `density ≈ 2.8` (granite)

## Collision Response Algorithm

### Location in Code

**File:** `pe/core/collisionsystem/HardContactAndFluid.h`

**Main function:** `resolveContacts()` starting at line 1608

### Step 1: Contact Caching (Lines 1752-1809)

For each contact, cache the contact properties:

```cpp
body1_[j]    = b1;
body2_[j]    = b2;
r1_[j]       = c->getPosition() - b1->getPosition();  // Moment arm from b1 center
r2_[j]       = c->getPosition() - b2->getPosition();  // Moment arm from b2 center
n_[j]        = c->getNormal();   // Contact normal (from b2 towards b1)
dist_[j]     = c->getDistance();
mu_[j]       = c->getFriction();
```

### Step 2: Effective Mass Calculation (Lines 1797-1807)

This is where the **problem occurs**:

```cpp
// Compute effective mass matrix in world frame
Mat3 diag = -(r1_[j] % b1->getInvInertia() % r1_[j] +
              r2_[j] % b2->getInvInertia() % r2_[j]);

// Add translational contributions
diag[0] += b1->getInvMass() + b2->getInvMass();
diag[4] += b1->getInvMass() + b2->getInvMass();
diag[8] += b1->getInvMass() + b2->getInvMass();

// Transform to contact frame
diag = trans(contactframe) * diag * contactframe;

// Compute inverse effective masses
diag_nto_inv_[j] = inv(diag);           // Full 3x3 inverse
diag_n_inv_[j]   = inv(diag[0]);        // Normal direction only
```

**Mathematical Formulation:**

The effective mass matrix relates impulse to velocity change at the contact point:

```
Δv_contact = M_eff^(-1) * p
```

Where:
```
M_eff = [m₁⁻¹ + m₂⁻¹]I - [r₁]×I₁⁻¹[r₁]× - [r₂]×I₂⁻¹[r₂]×
```

Components:
- `m₁⁻¹, m₂⁻¹`: Inverse masses (translational contribution)
- `I₁⁻¹, I₂⁻¹`: Inverse inertia tensors
- `[r]×`: Cross-product matrix of moment arm r
- `[r₁]×I₁⁻¹[r₁]×`: Rotational contribution from body 1
- `[r₂]×I₂⁻¹[r₂]×`: Rotational contribution from body 2

**For our case:**
- Body 1 (fixed): `m₁⁻¹ = 0`, `I₁⁻¹ = 0`
- Body 2 (translation-fixed): `m₂⁻¹ = 0`, `I₂⁻¹ ≠ 0`

Therefore:
```
M_eff = -[r₂]×I₂⁻¹[r₂]×
```

**Only the rotational inertia of body 2 contributes!**

In the normal direction (after transforming to contact frame):
```
M_eff,n = -n^T [r₂]×I₂⁻¹[r₂]× n
```

This can be **very small** depending on:
- Contact geometry (moment arm r₂)
- Capsule orientation (principal axes of I₂)
- Contact normal direction

### Step 3: Impulse Calculation (Lines 2158-2192)

For each contact in the iterative solver:

```cpp
// Calculate relative velocity at contact (without current impulse)
Vec3 gdot = (v₁ + dv₁) - (v₂ + dv₂) + (ω₁ + dω₁) × r₁ - (ω₂ + dω₂) × r₂;

// Transform to contact frame
Vec3 gdot_nto = trans(contactframe) * gdot;

// Add positional correction term (Baumgarte stabilization)
gdot_nto[0] += dist * (1/dt);

if( gdot_nto[0] >= 0 ) {
   // Separating contact
   p_[i] = Vec3();
} else {
   // Persisting contact - calculate impulse to make contact static
   Vec3 p_wf = n * (-diag_n_inv * gdot_nto[0]);

   p_[i] = p_wf;
}
```

**Formula:**
```
p_n = -M_eff,n^(-1) * (v_rel,n + bias)
```

Where:
- `p_n`: Normal impulse magnitude
- `M_eff,n^(-1)`: Inverse effective mass (= `diag_n_inv`)
- `v_rel,n`: Relative velocity in normal direction
- `bias = dist / dt`: **Baumgarte stabilization term**

**If `M_eff,n` is very small, then `M_eff,n^(-1)` is very large!**

This produces an **excessive impulse** for the same relative velocity.

#### Critical Issue: Baumgarte Bias Amplification

The bias term is computed from the contact distance and timestep:

**From lines 1788-1793:**
```cpp
dist_[j] = c->getDistance();
if( dist_[j] < 0 ) {
   // Penetration present - apply error reduction parameter
   maximumPenetration_ = std::max( maximumPenetration_, -dist_[j] );
   dist_[j] *= erp_;  // erp_ = 0.7 by default (line 434)
}
```

**From line 2167:**
```cpp
gdot_nto[0] += dist_[i] * dtinv;
```

**The amplification:**
```
bias = (0.7 * penetration_depth) / dt
```

For `dt = 0.001`:
```
bias = 700 * penetration_depth
```

Even a small penetration of 0.001 units creates a bias of 0.7 m/s!

**Combined with small effective mass:**
```
p_n = -M_eff,n^(-1) * (v_rel,n + 700 * penetration)
     = -HUGE * (v_rel,n + potentially_large_bias)
```

**The bias is NOT clamped!** This means:
1. Small penetration → large bias (700× amplification)
2. Large bias × huge inverse effective mass → **catastrophic impulse**
3. No upper limit on impulse magnitude

This is likely a **major contributor** to the energy-violating behavior!

## The Collision Response Algorithm: ApproximateInelasticCoulombContactByDecoupling

This section details the actual collision response algorithm used by default in the PE physics engine.

**Function:** `relaxApproximateInelasticCoulombContactsByDecoupling()`
**Location:** `pe/core/collisionsystem/HardContactAndFluid.h:2284-2415`
**Purpose:** Solve contact constraints with Coulomb friction using a decoupled approach

### Algorithm Overview

The method is an **iterative Gauss-Seidel relaxation** that processes each contact sequentially and updates velocities immediately. For each contact, it determines whether the contact is:
1. **Separating** (no impulse needed)
2. **Persisting with static friction** (no sliding)
3. **Persisting with dynamic friction** (sliding)

### Mathematical Formulation

#### Contact Frame

Contact frame basis vectors:
- **n**: Normal (points from body 2 toward body 1)
- **t**: First tangent vector (perpendicular to n)
- **o**: Second tangent vector (o = n × t)

Contact frame matrix:
```
R = [n | t | o]
```

#### Step 1: Remove Previous Impulse

Remove the velocity corrections from the current impulse estimate:

```cpp
dv₁ -= m₁⁻¹ · p
dω₁ -= I₁⁻¹ · (r₁ × p)
dv₂ += m₂⁻¹ · p
dω₂ += I₂⁻¹ · (r₂ × p)
```

#### Step 2: Calculate Relative Velocity

Relative velocity at contact point in world frame:
```
v_rel = (v₁ + dv₁) - (v₂ + dv₂) + (ω₁ + dω₁) × r₁ - (ω₂ + dω₂) × r₂
```

Transform to contact frame:
```
v_rel_nto = R^T · v_rel = [v_n, v_t, v_o]^T
```

Add Baumgarte stabilization term (if enabled):
```
v_n ← v_n + dist/dt
```

**Code (lines 2296-2314):**
```cpp
// Remove previous impulse
dv_[body1->index] -= body1->getInvMass() * p_[i];
dw_[body1->index] -= body1->getInvInertia() * (r1 % p_[i]);
dv_[body2->index] += body2->getInvMass() * p_[i];
dw_[body2->index] += body2->getInvInertia() * (r2 % p_[i]);

// Calculate relative velocity
Vec3 gdot = (v₁ + dv₁) - (v₂ + dv₂) + (ω₁ + dω₁) % r₁ - (ω₂ + dω₂) % r₂;

// Transform to contact frame
Mat3 contactframe(n, t, o);
Vec3 gdot_nto = trans(contactframe) * gdot;

// Add Baumgarte term
gdot_nto[0] += dist * dtinv;
```

#### Step 3: Separating Contact Check

If `v_n ≥ 0`, the contact is separating:
```
p = 0
```

**Code (lines 2319-2329):**
```cpp
if( gdot_nto[0] >= 0 ) {
   p_[i] = Vec3();
   // No impulse needed
}
```

#### Step 4: Static Contact Attempt

For persisting contacts (`v_n < 0`), first attempt a static solution by solving the full 3×3 system:

```
p_cf = -M_eff_nto⁻¹ · v_rel_nto
```

Where `M_eff_nto` is the effective mass matrix in contact frame (computed during contact caching).

**Friction cone check:**
```
|p_t| ≤ μ · p_n  and  p_n ≥ 0
```

If satisfied, contact is static (no sliding).

**Code (lines 2334-2344):**
```cpp
// Solve full 3x3 system
Vec3 p_cf = -(diag_nto_inv_[i] * gdot_nto);

// Check friction cone
real flimit = mu * p_cf[0];
real fsq = p_cf[1]*p_cf[1] + p_cf[2]*p_cf[2];

if( fsq > flimit*flimit || p_cf[0] < 0 ) {
   // Friction cone violated → dynamic contact
} else {
   // Contact is static
}
```

#### Step 5: Dynamic Contact (Decoupled Solution)

If friction cone is violated, the contact is sliding. Use a **decoupled approach**:

**5a. Solve Normal Component**

Account for tangential impulse from previous iteration:
```
p_tmp = (t · p_old) · t + (o · p_old) · o  (tangential part only)
```

Add its effect to normal velocity:
```
v_n' = v_n + n^T · [I₁⁻¹(r₁ × p_tmp) × r₁ + I₂⁻¹(r₂ × p_tmp) × r₂]
```

Solve for normal impulse:
```
p_n = -M_eff,n⁻¹ · v_n'
p_n = max(0, p_n)  (project to non-negative)
```

**Code (lines 2360-2371):**
```cpp
// Extract tangential component from previous iteration
Vec3 p_tmp = (trans(t) * p_[i]) * t + (trans(o) * p_[i]) * o;

// Add its rotational effect to normal velocity
real gdot_n = gdot_nto[0] + trans(n) *
   ((I₁⁻¹ * (r₁ % p_tmp)) % r₁ + (I₂⁻¹ * (r₂ % p_tmp)) % r₂);

// Solve normal impulse
p_cf[0] = -(diag_n_inv * gdot_n);

// Project to non-negative
p_cf[0] = max(0.0, p_cf[0]);
```

**5b. Solve Tangential Components**

Add effect of newly computed normal impulse:
```
p_tmp = n · p_n
v' = v_rel + I₁⁻¹(r₁ × p_tmp) × r₁ + I₂⁻¹(r₂ × p_tmp) × r₂
```

Project to tangential plane:
```
v_to = [t^T · v', o^T · v']^T
```

Solve for tangential impulse:
```
p_to = -M_eff,to⁻¹ · v_to
```

Apply friction limit (project to friction cone):
```
if |p_to| > μ · p_n:
   p_to ← (μ · p_n / |p_to|) · p_to
```

**Code (lines 2373-2390):**
```cpp
// Add normal impulse effect
p_tmp = n * p_cf[0];
Vec3 gdot2 = gdot + (I₁⁻¹ * (r₁ % p_tmp)) % r₁ +
                     (I₂⁻¹ * (r₂ % p_tmp)) % r₂;

// Project to tangential directions
Vec2 gdot_to;
gdot_to[0] = trans(t) * gdot2;
gdot_to[1] = trans(o) * gdot2;

// Solve tangential impulse
Vec2 ret = -(diag_to_inv * gdot_to);
p_cf[1] = ret[0];
p_cf[2] = ret[1];

// Project to friction cone
flimit = mu * p_cf[0];
fsq = p_cf[1]*p_cf[1] + p_cf[2]*p_cf[2];
if( fsq > flimit*flimit ) {
   real f = flimit / sqrt(fsq);
   p_cf[1] *= f;
   p_cf[2] *= f;
}
```

#### Step 6: Transform and Apply Impulse

Transform impulse back to world frame:
```
p_world = R · p_cf = n·p_n + t·p_t + o·p_o
```

Apply to both bodies:
```
dv₁ += m₁⁻¹ · p
dω₁ += I₁⁻¹ · (r₁ × p)
dv₂ -= m₂⁻¹ · p
dω₂ -= I₂⁻¹ · (r₂ × p)
```

**Code (lines 2395-2414):**
```cpp
// Transform to world frame
Vec3 p_wf = contactframe * p_cf;

// Store impulse
p_[i] = p_wf;

// Apply impulse
dv_[body1->index] += body1->getInvMass() * p_[i];
dw_[body1->index] += body1->getInvInertia() * (r1 % p_[i]);
dv_[body2->index] -= body2->getInvMass() * p_[i];
dw_[body2->index] -= body2->getInvInertia() * (r2 % p_[i]);
```

### Iteration and Convergence

The algorithm repeats for all contacts, then iterates multiple times (default: 100 iterations) until convergence. Convergence is measured by:
```
δ_max = max_i |p_new[i] - p_old[i]|
```

### Key Properties

**Advantages:**
- Handles Coulomb friction with cone constraint
- Gauss-Seidel gives immediate velocity updates (good convergence)
- Decoupling simplifies dynamic friction case

**Disadvantages:**
- Approximate: decoupling is not exact for sliding contacts
- Assumes contact constraint `v_rel = 0` is achievable
- **Problem:** Fails for kinematic bodies with prescribed motion (our case!)

### Why It Fails for Kinematic Bodies

The algorithm assumes:
```
v_rel = 0  (static contact goal)
```

Is achievable by adjusting velocities of **both** bodies. For a kinematic body:
```
Body 1: ω₁ = constant (prescribed, cannot change)
Body 2: Can only rotate (invMass = 0)
```

The solver computes impulse to achieve `v_rel = 0`, but:
- Impulse has **no effect** on body 1 (fully fixed)
- Body 2 must **fully compensate**, leading to |ω₂| > |ω₁|

**This is why energy is created in the capsule_spin example.**

## Velocity Integration (After Contact Resolution)

After the collision solver completes all iterations, the velocity corrections are integrated:

```cpp
// Apply impulse to bodies (happens in each iteration)
dv_[body1->index] += body1->getInvMass() * p;       // = 0 (fixed)
dω_[body1->index] += body1->getInvInertia() * (r₁ × p);  // = 0 (fixed)
dv_[body2->index] -= body2->getInvMass() * p;       // = 0 (translation-fixed)
dω_[body2->index] -= body2->getInvInertia() * (r₂ × p);  // ≠ 0 !!!
```

**Result:**
- Body 1: No change (completely fixed)
- Body 2: Large angular velocity change due to large impulse

**Angular velocity change:**
```
Δω₂ = I₂⁻¹ * (r₂ × p)
```

With large `p`, this produces large `Δω₂`.

## Energy Analysis

### Before Collision (timestep 104)

**Capsule 1:**
```
ω₁ = (0, 0, 6.28) rad/s
E₁,rot = (1/2) * ω₁^T * I₁ * ω₁
```

**Capsule 2:**
```
ω₂ = (0, 0, 0) rad/s
E₂,rot = 0
```

**Total energy:**
```
E_total = E₁,rot
```

### After Collision (timestep 105)

**Capsule 1:**
```
ω₁ = (0, 0, 6.28) rad/s  (UNCHANGED!)
E₁,rot = (1/2) * ω₁^T * I₁ * ω₁  (same as before)
```

**Capsule 2:**
```
ω₂ = (0, 0, -11.73) rad/s
E₂,rot = (1/2) * ω₂^T * I₂ * ω₂  (NEW ENERGY!)
```

**Total energy:**
```
E_total = E₁,rot + E₂,rot > E_before
```

**Energy is created! Violation of conservation!**

### Why This Happens: The Kinematic Body Problem

**Initial Hypothesis (Incorrect):**
We initially suspected the effective mass was too small, causing excessive impulses. However, diagnostic output showed:
- Effective mass (normal): **0.45-0.50 kg** (reasonable, not tiny!)
- Inverse effective mass: **2.0-2.2** (not huge)

**Actual Root Cause (Confirmed by Solver Output):**

The problem is **not** the magnitude of the effective mass, but the **goal** of the collision solver:

1. **Solver Goal:** Enforce static contact by making `v_rel = 0` at the contact point
2. **Capsule 1 Reality:** Kinematic body with prescribed motion - **MUST** rotate at ω₁ = 6.28 rad/s (like a motor)
3. **Impulse Calculation:** Solver computes impulse to achieve `v_rel = 0`
4. **Application to Capsule 1:** No effect (invMass = 0, invInertia = 0 → fully fixed)
5. **Application to Capsule 2:** Receives **entire velocity adjustment** needed to achieve `v_rel = 0`
6. **Result:** Capsule 2 spins faster than Capsule 1 to compensate for Capsule 1's continued rotation

**From actual solver output (Timestep 104):**
```
Contact 0 PERSISTING:
  Relative vel (normal): -2.50635 m/s (approaching)
  Impulse magnitude: 5.14
  Impulse (contact frame): <5.04, -1.01, 0>

Solver convergence:
  Iteration 0: delta_max = 5.04
  Iteration 10: delta_max = 8.3e-09  ← Converged perfectly!

Result (Timestep 105):
  ω₂ = -8.32 rad/s (|ω₂| > |ω₁| = 6.28)
  Contact SEPARATING: gdot = +0.016 (mission accomplished from solver's perspective)
```

**The solver works correctly** - it converges in ~10 iterations and achieves its goal. The problem is that **the goal itself is wrong** for kinematic bodies.

### The Fundamental Incompatibility

```
Static Contact Goal:    v_rel = 0
Kinematic Constraint:   ω₁ = constant (prescribed motion)
Translation Constraint: Both bodies cannot translate

Mathematically: UNSATISFIABLE
```

The solver tries to satisfy an impossible constraint by making Capsule 2 spin faster than the driving body.

## Proposed Solutions

### Option 0A: Disable Baumgarte for Zero-Translation-DOF Contacts (Most Physically Correct)

**Location:** After line 1792 in `HardContactAndFluid.h`

**The Key Insight:** When both bodies have zero translational degrees of freedom, Baumgarte stabilization is physically meaningless because penetration cannot be corrected through translation. Applying it creates artificial rotational energy.

```cpp
if( dist_[j] < 0 ) {
   maximumPenetration_ = std::max( maximumPenetration_, -dist_[j] );

   // Check if both bodies have zero translational freedom
   bool bothTranslationLocked = (b1->getInvMass() == real(0) &&
                                  b2->getInvMass() == real(0));

   if( bothTranslationLocked ) {
      // Penetration cannot be corrected - disable Baumgarte stabilization
      dist_[j] = real(0);
   } else {
      // Normal case - apply error reduction
      dist_[j] *= erp_;
   }
}
```

**Physical Justification:**
- Baumgarte creates translational separation velocity
- If no translational DOF exists, this velocity cannot be realized
- The impulse energy manifests as rotation instead
- This violates energy conservation and creates unphysical behavior

**Pros:**
- Addresses the root cause: Baumgarte is inappropriate for this case
- Physically motivated: don't apply correction that can't work
- Prevents artificial energy injection
- Simple and clear logic
- No arbitrary tuning parameters

**Cons:**
- Allows persistent penetration between such bodies
- May need to handle initial overlaps differently
- Doesn't help with the effective mass issue (but removes the multiplier)

### Option 0B: Clamp Baumgarte Bias Term

**Location:** After line 1792 in `HardContactAndFluid.h`

If you want to keep some Baumgarte correction but limit the damage:

```cpp
if( dist_[j] < 0 ) {
   maximumPenetration_ = std::max( maximumPenetration_, -dist_[j] );
   dist_[j] *= erp_;

   // Clamp penetration depth to prevent excessive bias
   const real maxPenetration = real(0.01);  // 1 cm max correction per step
   if( -dist_[j] > maxPenetration ) {
      dist_[j] = -maxPenetration * erp_;
   }
}
```

**Pros:**
- Limits damage from Baumgarte
- Still attempts some correction
- Independent of effective mass issue

**Cons:**
- Doesn't address the fundamental issue
- Still creates artificial energy (just less)
- Requires tuning

### Option 1: Minimum Effective Mass Threshold

**Location:** After line 1801 in `HardContactAndFluid.h`

```cpp
// After: diag = trans(contactframe) * diag * contactframe;

const real minEffectiveMass = real(1e-6);  // Tunable parameter
for( int k = 0; k < 3; ++k ) {
   if( std::abs(diag[k*4]) < minEffectiveMass ) {
      diag[k*4] = (diag[k*4] >= 0) ? minEffectiveMass : -minEffectiveMass;
   }
}
```

**Pros:**
- Simple implementation
- Prevents numerical blow-up
- Minimal code changes

**Cons:**
- Somewhat arbitrary threshold
- Doesn't address root cause

### Option 2: Special Case for Fixed vs Translation-Fixed

**Location:** After line 1801 in `HardContactAndFluid.h`

```cpp
// Detect both bodies have zero translational freedom
bool b1_translation_fixed = (b1->getInvMass() == real(0));
bool b2_translation_fixed = (b2->getInvMass() == real(0));
bool b1_fully_fixed = (b1->getInvInertia().sqrLength() == real(0));
bool b2_fully_fixed = (b2->getInvInertia().sqrLength() == real(0));

if( b1_translation_fixed && b2_translation_fixed &&
    (b1_fully_fixed || b2_fully_fixed) ) {
   // Apply damping/energy dissipation
   const real dampingFactor = real(0.1);  // Tunable
   diag *= (real(1) + dampingFactor);
}
```

**Pros:**
- Physically motivated (adds dissipation)
- Targets specific problematic case
- Can prevent energy creation

**Cons:**
- Adds special case logic
- Requires tuning damping factor
- May be too conservative

### Option 3: Cap Angular Velocity Change

**Location:** After line 2191 in `HardContactAndFluid.h`

```cpp
// After applying impulse, cap the angular velocity change
const real maxOmegaRatio = real(2.0);  // Max ratio of resulting omega to input omega

if( body2->getInvMass() == real(0) && body1->getInvInertia().sqrLength() == real(0) ) {
   // Body1 is fully fixed, body2 is translation-fixed
   Vec3 omega1_mag = body1->getAngularVel();
   real max_omega2 = maxOmegaRatio * omega1_mag.length();

   if( dω_[body2->index].length() > max_omega2 ) {
      dω_[body2->index] *= (max_omega2 / dω_[body2->index].length());
   }
}
```

**Pros:**
- Directly prevents excessive angular velocities
- Energy conserving (caps energy transfer)
- Easy to understand

**Cons:**
- Ad-hoc solution
- May affect legitimate high-speed collisions
- Doesn't fix underlying effective mass issue

### Option 4: Modify Effective Mass for Fixed Bodies

**Location:** Lines 1797-1807 in `HardContactAndFluid.h`

Use a "virtual mass" for the rotational contribution when dealing with fixed bodies:

```cpp
// When computing rotational contribution, add virtual mass term
Mat3 diag = -(r1_[j] % b1->getInvInertia() % r1_[j] +
              r2_[j] % b2->getInvInertia() % r2_[j]);

// Standard translational contribution
real invMassSum = b1->getInvMass() + b2->getInvMass();

// If both have zero translational mass, add virtual mass based on geometry
if( invMassSum == real(0) ) {
   // Estimate effective mass from moment arm and inertia
   real virtualInvMass = real(1) / (r1_[j].length() + r2_[j].length() + real(1e-6));
   invMassSum = virtualInvMass;
}

diag[0] += invMassSum;
diag[4] += invMassSum;
diag[8] += invMassSum;
```

**Pros:**
- Addresses root cause
- Scales with geometry
- Physically motivated

**Cons:**
- More complex
- Requires careful validation
- Changes behavior for all such contacts

## Configuration

Current collision response solver: `pe::response::HardContactAndFluid`

**File:** `pe/config/Collisions.h` line 86:
```cpp
#define pe_CONSTRAINT_SOLVER  pe::response::HardContactAndFluid
```

## Recommendations

### Summary of Fixes Applied

1. **✅ Option 0A: Disabled Baumgarte** (pe/core/collisionsystem/HardContactAndFluid.h:1796-1804)
   - **Result:** 29% reduction in excessive angular velocity (11.73 → 8.32 rad/s)
   - **Remaining issue:** Still violates energy conservation

### Root Cause: Kinematic Body Treatment

The fundamental problem is that **Capsule 1 is a kinematic body with prescribed motion** (like a motor-driven rotor), but the collision solver treats it like a regular dynamic body and tries to enforce a static contact constraint.

**Current solver behavior:**
- Goal: Make relative velocity `v_rel = 0` (static contact)
- Reality: Capsule 1 MUST maintain ω = 6.28 rad/s (prescribed motion)
- Result: Capsule 2 compensates by spinning faster than the driving body

**This is a fundamental modeling issue, not a numerical bug.**

### Options for Kinematic Bodies

#### Option A: Detect and Handle Kinematic Bodies Specially

Add detection for kinematic bodies (fully fixed with prescribed motion) and modify the collision response:

**Location:** `pe/core/collisionsystem/HardContactAndFluid.h` in relaxation functions

```cpp
// Detect kinematic body scenario
bool body1Kinematic = (b1->isFixed() && b1->getAngularVel().length() > 0);
bool body2Kinematic = (b2->isFixed() && b2->getAngularVel().length() > 0);

if( (body1Kinematic || body2Kinematic) && bothTranslationLocked ) {
   // Don't try to enforce static contact with kinematic bodies
   // Instead, use dissipative contact or allow sliding

   // Option 1: Apply damping instead of static contact
   // Option 2: Allow sliding contact with friction only
   // Option 3: Limit impulse based on kinematic body's motion
}
```

**Pros:**
- Addresses the actual problem
- Physically motivated
- Prevents energy creation

**Cons:**
- Requires defining what "kinematic" means (heuristic: fixed + non-zero velocity?)
- Needs careful design of alternative contact model
- May affect other scenarios

#### Option B: Use Restitution Model Instead of Static Contact

For contacts between kinematic and rotation-only bodies, use a coefficient of restitution approach instead of trying to make v_rel = 0:

```cpp
if( bothTranslationLocked && oneFullyFixed ) {
   // Use restitution model: v_rel_after = -e * v_rel_before
   const real e = 0.0;  // Perfectly inelastic
   // Compute impulse to achieve this instead of v_rel = 0
}
```

**Pros:**
- More physical for kinematic scenarios
- Naturally limits energy transfer
- Well-understood physics

**Cons:**
- Changes behavior for these contacts
- May not prevent sliding

#### Option C: Cap Impulse Based on Kinematic Motion

Limit the impulse magnitude based on the kinematic body's velocity:

```cpp
if( body1Kinematic && bothTranslationLocked ) {
   real maxOmega = body1->getAngularVel().length() * some_factor;
   // Cap the resulting omega2 to be <= maxOmega
}
```

**Pros:**
- Simple to implement
- Direct energy conservation

**Cons:**
- Ad-hoc approach
- Doesn't address fundamental modeling issue

### Testing Strategy

1. **Create unit test:** Fixed vs translation-fixed collision
2. **Monitor:** Energy conservation during collisions
3. **Track:** Maximum penetration depths and impulse magnitudes
4. **Verify:** Angular velocity ratios remain physically plausible (|ω₂| ≤ |ω₁| for elastic)
5. **Test:** Various contact geometries and orientations

### Key Insights

The problem has **two multiplicative factors**:

```
Excessive Impulse = HUGE_inverse_mass × LARGE_bias
                  = (small M_eff)^(-1) × (penetration / dt)
```

#### Why Baumgarte Stabilization Fails for Zero-Translation-DOF Contacts

**Baumgarte's assumption:** Bodies can separate translationally to correct penetration.

**Reality with both bodies translation-locked:**
```
Available translational DOF = 0
Baumgarte correction velocity = dist / dt (tries to separate)
Actual translational response = 0 (cannot separate)
Energy must go somewhere = manifests as ROTATION
```

**The physical contradiction:**
1. Penetration exists (bodies overlap)
2. Baumgarte creates impulse to separate them
3. Neither body can translate to separate
4. Impulse energy → artificial rotation in body 2
5. Energy conservation violated

**Example:** Motor-driven shaft penetrating bearing sleeve
- Real world: They stay penetrated (or bearing deforms)
- Baumgarte approach: Creates torque on sleeve → unphysical spin-up
- Correct approach: Accept the penetration, enforce non-penetrating velocity

**Therefore:** Disable Baumgarte when `invMass1 == 0 && invMass2 == 0`

#### Solution Strategy

**Phase 1 (Completed):**
- ✅ **Disabled Baumgarte (Option 0A):** Reduced excessive velocity by 29%

**Phase 2 (Needed):**
- 🔲 **Kinematic body detection and special handling** (Option A, B, or C above)
  - This is the fundamental fix needed to prevent energy violation
  - Requires design decision on how to handle kinematic-vs-rotation-only contacts

**Note:** Option 1 (minimum effective mass threshold) is NOT needed. The effective mass values are reasonable (~0.45-0.50 kg). The problem is the solver goal, not numerical instability.

## Summary

### What We Learned

1. **Initial Hypothesis (Wrong):** Small effective mass causing excessive impulses
   - **Reality:** Effective mass is reasonable (0.45-0.50 kg)
   - **Evidence:** Diagnostic output from solver

2. **Baumgarte Issue (Partially Correct):**
   - **Fix Applied:** Disabled Baumgarte for zero-translation-DOF contacts
   - **Result:** 29% reduction (11.73 → 8.32 rad/s)
   - **Contribution:** ~3.4 rad/s of the excessive velocity

3. **Root Cause (Confirmed):** Kinematic body with prescribed motion
   - **Problem:** Solver tries to enforce `v_rel = 0` (static contact)
   - **Conflict:** Capsule 1 MUST rotate at constant speed (kinematic)
   - **Result:** Capsule 2 over-compensates, spinning faster than the driver
   - **Evidence:** Solver converges perfectly but produces unphysical result

### The Fundamental Issue

```
Physics Model:  Kinematic body (motor-driven rotor) hitting rotation-only body (bearing)
Solver Treats:  Two dynamic bodies with achievable contact constraint
Reality:        Constraint is mathematically unsatisfiable
Consequence:    Energy-violating solution that satisfies the (wrong) constraint
```

### Next Steps

The collision solver needs **special handling for kinematic bodies** - bodies with prescribed motion that act as drivers/motors/actuators rather than passive dynamic objects. This requires a design decision on the appropriate contact model for kinematic-vs-constrained scenarios.

## References

- Example: `examples/capsule_spin/capsule_spin.cpp`
- Collision system: `pe/core/collisionsystem/HardContactAndFluid.h`
  - Contact caching: Lines 1752-1825
  - Baumgarte fix: Lines 1796-1804
  - Coulomb friction solver: Lines 2284-2415
- Rigid body fixing: `src/core/rigidbody/RigidBody.cpp`
- Capsule inertia: `src/core/rigidbody/CapsuleBase.cpp`
- Default relaxation model: `ApproximateInelasticCoulombContactByDecoupling` (line 439)
