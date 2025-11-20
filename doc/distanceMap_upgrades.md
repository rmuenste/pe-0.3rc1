# DistanceMap Implementation Status & Remaining Improvements

## Implementation Summary (Updated 2025-11-19)

### ✅ What's Implemented and Working Well

#### Core SDF Infrastructure
* **Coherent SDF pipeline** (AABB tree → signed distance via `Side_of_triangle_mesh` → trilinear interpolation)
* **Clear frame handling** (BF↔WF transforms) — critical for SDFs living in the ref mesh's local frame
* **Clean integration** (try SDF; else GJK/EPA) with a single collision entry point
* **Checkpointing support** — DistanceMap parameters saved/restored for simulation restart (rebuilds SDF from parameters)

#### Multi-Contact Manifold System ✅ IMPLEMENTED (Commit 8745056)
* **Comprehensive candidate gathering:**
  * ✅ Vertex sampling (all vertices)
  * ✅ Edge midpoint sampling (with deduplication)
  * ✅ Triangle barycenter sampling (face-face collision detection)
  * Eliminates face-face collision misses from vertex-only testing

* **Contact clustering & reduction:**
  * ✅ Proximity clustering (2.0 × DistanceMap spacing)
  * ✅ Normal similarity grouping (dot product > 0.9)
  * ✅ Prevents redundant nearby contacts

* **Representative selection:**
  * ✅ Deepest contact per cluster (max penetration)
  * ✅ Extremal contacts in tangent directions (up to 4 per cluster)
  * ✅ Proper tangent space calculations with orthonormal basis
  * ✅ Contact limit: 6 contacts per mesh pair for performance

* **Additional features:**
  * ✅ Comprehensive debug logging with cluster statistics
  * ✅ Contact area estimation for plane collisions
  * ✅ Configurable clustering for plane-mesh collisions (compile-time flag)

**Result:** Chair/table stability significantly improved; proper torque distribution; reduced friction jitter.

---

## Remaining High-Priority Improvements

### 1) Symmetric Testing (High Priority)

**Current Behavior:**
```cpp
// In collideWithDistanceMap (lines 3900-3910 in MaxContacts.h)
if (mA->hasDistanceMap()) {
    distMap = mA->getDistanceMap();
    referenceMesh = mA;
    queryMesh = mB;
} else if (mB->hasDistanceMap()) {
    distMap = mB->getDistanceMap();
    referenceMesh = mB;
    queryMesh = mA;
}
```

**Issue:** When **both** meshes have DistanceMaps, only one direction is tested (A→B or B→A, whichever comes first).

**Fix Needed:**
```cpp
// Proposed implementation
bool hasContactA = false, hasContactB = false;

if (mA->hasDistanceMap()) {
    hasContactA = collideWithDistanceMapUnidirectional(mA, mB, contacts);
}

if (mB->hasDistanceMap()) {
    hasContactB = collideWithDistanceMapUnidirectional(mB, mA, contacts);
}

// Deduplicate merged contacts by position (within 1-2 * spacing) and normal similarity
if (hasContactA && hasContactB) {
    deduplicateContacts(contacts, spacing);
}

return hasContactA || hasContactB;
```

**Benefits:**
* Dramatically reduces misses in grazing contacts
* Better detection for thin features and edge-edge contacts
* More robust collision detection for complex geometry

**Implementation Notes:**
* Refactor current `collideWithDistanceMap` into unidirectional version
* Add contact deduplication function with spatial hashing or naive O(n²) for small contact sets
* Use position proximity (1-2 × spacing) and normal similarity (dot > 0.95) for deduplication

---

### 2) Contact Persistence & Warm-Starting (Medium Priority)

**Current Behavior:** Contacts rebuilt from scratch each frame; solver starts with zero impulses.

**Issue:** Contact jittering, lost temporal coherence, slower constraint convergence.

**Fix Needed:**
* **Frame-to-frame contact matching:**
  * Store previous frame's contacts in body-frame coordinates
  * Match by nearest neighbor distance in BF (within tolerance)
  * Re-project matched contacts onto current φ=0 surface

* **Warm-start impulses:**
  * Store accumulated impulses (normal + friction) per contact
  * Transfer impulses to matched contacts in next frame
  * Apply temporal decay factor (0.95-0.99) to prevent stale impulse buildup

**Implementation Approach:**
```cpp
struct PersistentContact {
    Vec3 positionBF;           // In reference mesh body frame
    Vec3 normalBF;             // In reference mesh body frame
    real normalImpulse;        // Accumulated normal impulse
    Vec3 tangentialImpulse;    // Accumulated friction impulse
    int lifetimeFrames;        // Track contact age
};

// Per mesh-pair storage (in RigidBody or ContactCache)
std::vector<PersistentContact> previousContacts_;

// Match current contacts to previous, warm-start solver
void matchAndWarmStart(const std::vector<Contact>& current) {
    for (auto& contact : current) {
        auto* prev = findNearestMatch(contact, previousContacts_);
        if (prev && isValidMatch(contact, *prev)) {
            contact.setWarmStartImpulse(prev->normalImpulse * 0.95,
                                        prev->tangentialImpulse * 0.95);
        }
    }
}
```

**Benefits:**
* Reduced jittering and contact oscillation
* Faster constraint solver convergence (fewer iterations)
* More stable stacking behavior
* Smoother sliding motion

---

### 3) Continuous Collision Detection (CCD) Guardrail (Low-Medium Priority)

**Issue:** Fast-moving objects can tunnel through thin geometry even with SDF acceleration.

**Fix Options:**

**A) Conservative Advancement:**
```cpp
real computeSafeTOI(const DistanceMap* distMap,
                    const Vec3& startPos, const Vec3& velocity, real dt) {
    // Use SDF gradient magnitude as Lipschitz bound for conservative advancement
    real minDist = distMap->interpolateDistance(startPos);
    real gradMag = distMap->interpolateGradient(startPos).length();

    // Conservative safe distance based on gradient
    real safeDist = std::max(minDist - distMap->getSpacing(), 0.0);
    real maxSafeVelocity = safeDist * gradMag; // Lipschitz bound

    if (velocity.length() > maxSafeVelocity) {
        return safeDist / velocity.length(); // Sub-step needed
    }
    return dt; // Full timestep safe
}
```

**B) Adaptive Sub-Stepping:**
```cpp
// In time integration loop, before position update
for each body pair with DistanceMap {
    real minDist = computeMinimumDistance(bodyA, bodyB);
    real relativeSpeed = (bodyA.velocity - bodyB.velocity).length();

    if (minDist < threshold && relativeSpeed > speedThreshold) {
        int subSteps = std::ceil(relativeSpeed * dt / minDist);
        // Trigger sub-stepping for this pair
    }
}
```

**Benefits:**
* Prevents tunneling through thin geometry
* Maintains collision detection reliability at high velocities
* Leverages SDF's distance information for efficient bracketing

**Trade-offs:**
* Adds computational overhead for fast-moving objects
* Requires integration with time-stepping loop
* Most benefit for high-speed impact scenarios

---

## Medium-Priority Refinements

### 4) Normal/Contact Point Consistency

**Current Approach:** Mix of SDF gradient normals and CGAL AABB closest points.

**Potential Issues:** Near sharp features, gradient and closest point can disagree → tangential energy injection.

**Options:**

**A) Gradient-Based Projection (Recommended):**
```cpp
// Project query point onto surface along gradient
Vec3 localNormal = distMap->interpolateNormal(localPos).getNormalized();
real phi = distMap->interpolateDistance(localPos);
Vec3 projectedContact = localPos - phi * localNormal;

// Optional: refine with 1-2 Newton steps
for (int i = 0; i < 2; ++i) {
    real newPhi = distMap->interpolateDistance(projectedContact);
    Vec3 newNormal = distMap->interpolateNormal(projectedContact).getNormalized();
    projectedContact -= newPhi * newNormal;
}
```

**B) Cluster Normal Averaging:**
```cpp
// Already implemented: average normals per cluster to de-noise
Vec3 averageNormal = Vec3(0, 0, 0);
for (size_t idx : cluster.candidateIndices) {
    averageNormal += candidates[idx].worldNormal;
}
averageNormal = averageNormal.getNormalized();
```

**Status:** Current averaging approach is reasonable; gradient projection could further improve consistency near sharp features.

---

### 5) Memory Optimization Opportunities

**Current Storage per Voxel:**
* `sdf_`: 8 bytes (double)
* `alpha_`: 8 bytes (purpose unclear — document or remove)
* `normals_`: 24 bytes (3 doubles)
* `contact_points_`: 24 bytes (3 doubles)
* **Total:** 64 bytes/voxel (or 80 bytes if alpha is used)

**Note:** Documentation claims 40 bytes/voxel — should be corrected to 64 bytes.

**Optimization Options:**

1. **On-the-fly gradient computation:**
   * Drop `normals_` storage (saves 24 bytes/voxel)
   * Compute gradient from `sdf_` using centered differences on query
   * Trade: -37% memory for +small compute per query

2. **Drop precomputed contact points:**
   * If using gradient projection, `contact_points_` redundant (saves 24 bytes/voxel)
   * Compute on-the-fly: `contact = queryPos - phi * normal`

3. **Narrow-band SDF:**
   * Store only voxels with |φ| ≤ band (e.g., 5 × spacing)
   * Use sparse storage (hash map or run-length encoding)
   * Can reduce memory by 80-95% for large grids with small geometry

4. **Half-precision far-field:**
   * Use `float16` for voxels with |φ| > threshold
   * Keep `double` precision near surface

**Recommendation:** Start with on-the-fly gradient computation (easy win); defer narrow-band until memory becomes bottleneck.

---

### 6) Robustness Improvements

**A) Inside/Outside Classification:**
* Current: `CGAL::Side_of_triangle_mesh` (brittle on non-watertight meshes)
* Improvement: Add tolerance band (dilate mesh by grid spacing) to stabilize signs
* Alternative: Generalized winding number for CAD/scanned geometry

**B) Gradient Quality:**
* Use centered differences for ∇φ
* Clamp near grid boundaries
* Add epsilon (1e-10) in normalization to prevent division by zero

**C) Sign Convention:**
* Already correct: positive outside, negative inside
* Make penetration calculation explicit: `penetration = -min(distance, 0.0)`

---

## Solver-Side Recommendations

(These apply to the constraint solver, not DistanceMap directly)

* **Friction pyramid**: 4-8 directions per contact for anisotropic friction
* **Small normal compliance** (ERP/CFM): suppress contact buzz from SDF gradient noise
* **Split impulses**: separate penetration correction from velocity solve to prevent energy injection
* **Rolling resistance & torsional friction**: tiny coefficients (0.001-0.01) dramatically improve tall stack stability

---

## Diagnostic Testing Checklist

Recommended tests to validate improvements:

* ✅ **Vertex-only miss test**: Face-face plate collision (FIXED with edge/bary sampling)
* ⚠️ **Symmetric detection**: Two meshes with DistanceMaps, grazing contact (TODO: implement symmetric testing)
* ⚠️ **Persistence test**: Rolling sphere, track contact lifetime and impulse warm-starting (TODO: implement persistence)
* ✅ **Multi-contact stability**: Chair on plane (4 legs), verify no rocking (FIXED with manifold)
* ⚠️ **CCD test**: Fast-moving sphere vs thin wall, verify no tunneling (TODO: implement CCD guardrail)
* 📊 **Resolution sweep**: spacing = 0.5%, 1%, 2% of bbox diagonal; measure penetration error vs. spacing
* 📊 **Performance benchmark**: GJK/EPA vs DistanceMap collision time for various mesh complexities

---

## Implementation Priority Ranking

### High Priority (Immediate)
1. **Symmetric testing** when both meshes have DistanceMaps
   * Effort: Medium (1-2 days)
   * Impact: High (better collision coverage, fewer misses)

### Medium Priority (Next Sprint)
2. **Contact persistence & warm-starting**
   * Effort: High (3-5 days, requires solver integration)
   * Impact: High (stability, convergence speed)

3. **Memory optimization** (on-the-fly gradients)
   * Effort: Low (1 day)
   * Impact: Medium (memory reduction, slight compute increase)

### Low Priority (Future Work)
4. **CCD guardrail** for high-velocity scenarios
   * Effort: Medium (2-3 days)
   * Impact: Medium (specific to fast-moving objects)

5. **Narrow-band SDF** for memory-constrained applications
   * Effort: High (4-7 days, significant refactor)
   * Impact: Medium (conditional on memory bottleneck)

---

## Conclusion

The DistanceMap system has matured significantly with the **multi-contact manifold implementation** (commit 8745056), addressing the primary stability issues from single-point contacts. The system now provides:

* ✅ Comprehensive collision detection (vertex + edge + face sampling)
* ✅ Stable multi-contact manifolds with intelligent clustering
* ✅ Proper coordinate transformation handling
* ✅ Checkpoint/restart support

**Key remaining work:**
* **Symmetric testing** for bidirectional DistanceMap collision (highest priority)
* **Contact persistence** for temporal coherence and warm-starting
* **CCD guardrails** for high-velocity robustness

The system is production-ready for most scenarios; remaining improvements target advanced use cases (very fast motion, very complex contact scenarios, memory-constrained deployments).
