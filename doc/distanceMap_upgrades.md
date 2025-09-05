# What’s solid already

* **Coherent SDF pipeline** (AABB tree → signed distance via `Side_of_triangle_mesh` → trilinear interpolation).
* **Clear frame handling** (BF↔WF transforms) — critical for SDFs living in the ref mesh’s local frame.
* **Clean integration** (try SDF; else GJK/EPA) with a single collision entry point.

---

# High-impact issues & immediate fixes

## 1) Single contact point: physically under-constrained

**Effect:** Chairs/tables rock or collapse; wrong torques; friction jitter.
**Fix now:** Build a *persistent contact manifold* per pair; keep 3–8 points total.

* **Candidate gathering** (cheap, fits your current loop):

  * While scanning vertices, collect all with `distance < 0` (or `< threshold`).
  * Augment with **edge midpoints** and **triangle barycenters** for the query mesh to catch face–face/edge–face penetrations that no vertex samples (this is crucial; see §3).
* **Clustering & reduction:**

  * Cluster by proximity (e.g., radius = 2–3 \* spacing) **and** normal similarity (dot > 0.9).
  * For each cluster/patch, pick up to 4 representatives: the deepest point + extremals in two tangent directions (a “quadrant” heuristic).
* **Persistence:**

  * Match contacts frame-to-frame by nearest neighbor in BF, re-project onto φ=0, **warm-start** impulses.

> Start simple: keep `max_contacts=6` per pair; you’ll get 90% of the benefit.

---

## 2) Vertex-only tests miss collisions

Your doc states “complete vertex enumeration (no sampling)”. That still **misses face–face** configurations where neither side has a vertex inside the other.

**Fix:** For the *query* mesh, add:

* **Edge midpoints** (or a 2-pt subdivision of long edges).
* **Triangle barycenters** (1 per face).
  Gate this with a cheap **broadphase** (BVH vs. ref AABB of negative band) to keep it fast.

---

## 3) Symmetry: query both directions when both SDFs exist

Right now you pick whichever mesh has a distance map. That’s fine if only one has it, but if **both have SDFs**, do **A→B and B→A** and union the manifolds (deduplicate by position/normal). This dramatically reduces misses in grazing contacts and thin features.

---

## 4) Normal/closest-point consistency

You mix **SDF gradient** for normals with **AABB closest point** for the contact point. These can disagree near sharp features → tangential energy injection/jitter.

**Better options:**

* **Project along gradient:** `x_c = x - φ(x) * n(x)` (one Newton step; optionally refine by 1–2 secant steps using interpolated φ).
* Or **use CGAL closest point** for both position **and** normal (derive normal from the face; blend if on edges/vertices).
* If you keep gradient normals, **average normals per cluster** to de-noise.

---

## 5) Gradient quality & bounds

* Use **centered differences** for ∇φ; clamp when sampling hits grid edges.
* Add a tiny **ε** in normalization.
* Consider **on-the-fly gradient** from φ instead of storing normals (saves memory and guarantees consistency).

---

## 6) Sign convention & thresholds

Doc says **positive outside**, **negative inside** — good. In code you check `distance < contactThreshold`. Make this explicit:

* Use `penetration = -min(distance, 0)`.
* Use a **band** around 0 for candidate collection (e.g., `distance < 2*spacing`) to build patches even when barely touching.

---

## 7) Memory math in the doc is off

You claim `≈ nx*ny*nz*40 bytes (double)`. With what you list:

* `sdf_`: 8 B
* `alpha_`: 8 B (but see §8)
* `normals_`: 3\*8 = 24 B
* `contact_points_`: 24 B
  **Total** ≈ **64 B** per voxel (or **80 B** if `alpha_` is used). Correct the doc and/or reduce storage (see below).

---

## 8) Storage & perf knobs

* If you **compute gradient on demand**, you can drop `normals_`.
* If you **project along gradient**, you can drop `contact_points_`.
* `alpha_` is listed but not used in the doc — either remove it or document how it’s used (e.g., cached barycentric weights).

Consider:

* **Narrow band SDF** (store only |φ|≤band) + **mip pyramid** for early-out.
* **Half precision** for φ in far field if memory is tight.

---

## 9) Contact orientation

You flip the normal when `referenceMesh == mB`. Make sure the final normal follows your engine convention (e.g., points from **A→B**). Add an assertion in debug builds to check that the relative positions satisfy `n·(x_B - x_A) ≥ 0`.

---

## 10) Robustness of inside/outside

`Side_of_triangle_mesh` can be brittle on **non-watertight** or **self-intersecting** meshes. For assets like scanned or CAD-repaired geometry, a **generalized winding number** or **ray stabbing with tie-breaking** is more stable. At minimum, dilate the mesh by a **tolerance band** equal to your grid spacing to stabilize signs.

---

## 11) CCD guardrail

Even a fast SDF narrow-phase can tunnel. Add a **conservative advancement** step using φ and ‖∇φ‖ to bracket TOI, or sub-step when the minimum distance crosses a small positive threshold.

---

# Concrete “do-next” (drop-in with your current code)

### A) Multi-contact manifold (within your current loop)

```cpp
struct Cand { Vec3 x_w; Vec3 n_w; pe::real phi; Vec3 x0_w; }; // x0_w: projected contact
std::vector<Cand> cands;

// 1) Collect candidates: vertices, plus edge midpoints & face barycenters (see §2)
for (auto p_w : query_points /* verts + mids + bary */) {
    Vec3 p_b = referenceMesh->pointFromWFtoBF(p_w);
    auto phi = distMap->interpolateDistance(p_b[0], p_b[1], p_b[2]);
    if (phi < band) {
        Vec3 n_b = distMap->interpolateNormal(p_b[0], p_b[1], p_b[2]).normalized();
        Vec3 n_w = referenceMesh->vectorFromBFtoWF(n_b);
        // project along gradient for consistent x0
        Vec3 x0_b = p_b - phi * n_b;
        Vec3 x0_w = referenceMesh->pointFromBFtoWF(x0_b);
        cands.push_back({p_w, n_w, phi, x0_w});
    }
}

// 2) Cluster by position (r = 2–3 * spacing) and normal (dot > 0.9)
// 3) For each cluster, pick up to 4 reps: deepest and extremals in two tangents
// 4) Add to 'contacts' with penetration depth = max(0, -phi), position = x0_w, normal = n_w
```

### B) Symmetric pass when both SDFs exist

Run the same routine swapping roles and **merge** contacts (dedupe by x within 1–2 \* spacing and normal similarity).

### C) Persistence

Keep previous step’s contacts and try to **re-match** by nearest neighbor in BF, then **re-project** and **warm-start** impulses.

---

# Solver-side recommendations (since you use semi-implicit hard contact)

* **Friction pyramid**: 4 directions (or 8 if iterations are cheap) per contact.
* **Small normal compliance** (ERP/CFM) to suppress buzz, especially with SDF normal noise.
* **Split impulses** for penetration correction to remove energy injection on restitution.
* **Rolling resistance & torsional friction** (tiny coefficients) help tall stacks and “chair legs” stability a lot.

---

# Testing you’ll want (fast, diagnostic)

* **Chair on plane** (4 legs): drift & angular jitter with 1, 2, 4, 6 contact limits.
* **Face–face** plate on plate (no vertex penetration): verify that edge/bary sampling finds contacts.
* **Grazing contact** (thin wedge): normal continuity while sliding.
* **Friction ramp**: stick→slip angle ≈ arctan(μ).
* **Resolution sweep**: spacing = 0.5%, 1%, 2% of bbox diag; penetration bias & jitter vs. spacing.

---

# Minor doc nits to fix

* Correct **memory estimate** (see §7).
* Clarify **penetration criterion** (negative φ vs. positive threshold band).
* Mention that **closest point** and **normal** are either both SDF-derived or both CGAL-derived — don’t mix without noting implications.
* State clearly whether you **also** test **edge midpoints / face centers** (currently the doc says no).

---

# Summary

Your SDF path is a solid base and already a big win over pure GJK/EPA for complex, non-convex meshes. The main limitation is the *single-point* model and vertex-only sampling. Implementing a **persistent multi-point manifold** (with edge/bary sampling) is the single biggest upgrade: it fixes chairs/tables, improves torque correctness, and stabilizes stacks — without blowing up cost if you cap contacts at \~6 and reuse warm-starts. Add symmetry when both shapes have SDFs, and make normals/closest points consistent. The rest (CCD guardrail, sign robustness, memory trims) are incremental and straightforward.

