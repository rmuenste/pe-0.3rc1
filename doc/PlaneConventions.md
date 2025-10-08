# PE Plane Conventions and Common Pitfalls

## Overview

The `Plane` class in PE uses mathematical conventions that differ from many standard references and other physics engines. This document clarifies these conventions and highlights common pitfalls to help developers correctly implement plane-related algorithms.

## ⚠️ Warning: Non-Standard Conventions

PE's plane conventions are **NOT standard** and require careful attention when implementing algorithms involving planes. Always consult this document when working with planes to avoid subtle bugs.

---

## Plane Equation Convention

### PE's Plane Equation

From `pe/core/rigidbody/Plane.h` (line 75):

```
ax + by + cz = d
```

where:
- `(a, b, c)` are the components of the normal vector **n**
- `d` is the displacement parameter
- The normal **n** is always **normalized** (unit length)
- The normal **n** points toward the **OUTSIDE** of the plane

### Standard Mathematical Convention (for comparison)

Most textbooks and physics engines use:

```
ax + by + cz + d = 0    or equivalently    n·p + d = 0
```

**Key Difference**: PE's equation can be rewritten as `n·p - d = 0`, which means PE's `d` has the **opposite sign** from the standard convention's `d`.

---

## Inside vs. Outside Semantics

### Definition (from Plane.h lines 67-72)

> "The plane geometry is an infinite rigid body dividing the global space in two half spaces. One of these half spaces is considered to be inside the plane. The normal vector points to the OUTSIDE of a plane."

### PE's Inside/Outside Test

A point `p` is:
- **Inside** the plane if: `n·p - d < 0` (or equivalently: `n·p < d`)
- **On** the plane surface if: `n·p - d = 0` (or equivalently: `n·p = d`)
- **Outside** the plane if: `n·p - d > 0` (or equivalently: `n·p > d`)

### Interpretation of Displacement Parameter `d`

From Plane.h (lines 79-82):

> "d is the distance/displacement from the origin of the global world frame to the plane. A positive value of d indicates that the origin of the global world frame is inside the plane, whereas a negative value of d indicates that the origin is outside the plane."

**Important**: This statement describes the **sign of the depth of the origin**, not the geometric distance from origin to plane.

---

## The getDepth() Function

### Definition (Plane.h line 339-342)

```cpp
inline real Plane::getDepth(const Vec3& gpos) const
{
   return d_ - (trans(gpos) * normal_);
}
```

This returns: `d - n·p`

### Interpretation

- **Positive value**: Point is inside the plane
- **Zero**: Point is on the plane surface
- **Negative value**: Point is outside the plane

**Note**: Despite PE's non-standard plane equation, the `getDepth()` function follows standard physics engine semantics for penetration depth.

---

## Common Pitfalls and Solutions

### ❌ Pitfall 1: Computing a Point on the Plane Surface

**WRONG** (appears to work for d=0, fails otherwise):
```cpp
Vec3 planeOrigin = -plane->getDepth(Vec3(0,0,0)) * plane->getNormal();
```

This computes `-(d - 0) * n = -d * n`, which is:
- ✅ Correct when `d = 0` (gives origin)
- ❌ **Wrong** when `d ≠ 0` (gives point on wrong side of plane!)

**CORRECT**:
```cpp
Vec3 planeOrigin = plane->getDisplacement() * plane->getNormal();
```

This computes `d * n`, which for PE's equation `n·p = d` with normalized `n`, gives the point on the plane closest to the origin.

### Example

For a plane with normal `(0, -1, 0)` and `d = -3`:

- Plane equation: `-y = -3`, so `y = 3`
- The plane surface is at `y = 3`

**Wrong formula**:
```cpp
planeOrigin = -(-3 - 0) * (0, -1, 0) = 3 * (0, -1, 0) = (0, -3, 0)  // WRONG!
```

**Correct formula**:
```cpp
planeOrigin = -3 * (0, -1, 0) = (0, 3, 0)  // ✓ Correct
```

### ❌ Pitfall 2: Sign Confusion with Standard References

When porting algorithms from papers or other engines that use the standard form `n·p + d = 0`:

**Remember**: Their `d` = `-1 * PE's d`

If a reference says "distance from origin to plane is `d`", in PE you need to use `-d`.

### ❌ Pitfall 3: Penetration Direction

When computing contact points between a plane and another object:

**WRONG** (conceptually backwards):
```cpp
// Sampling points on the plane and checking if they're inside the mesh
Vec3 planeSamplePoint = /* point on plane */;
real distance = mesh->distanceQuery(planeSamplePoint);
if (distance < 0) { /* collision */ }
```

**CORRECT**:
```cpp
// Testing mesh vertices against the plane
Vec3 meshVertex = /* vertex on mesh */;
real penetration = plane->getDepth(meshVertex);
if (penetration > 0) { /* collision */ }
```

The correct approach tests **mesh points against the plane**, not plane points against the mesh.

---

## Quick Reference Formulas

### For a plane with normal `n` (normalized) and displacement `d`:

| Operation | Formula | Notes |
|-----------|---------|-------|
| Point on plane (closest to origin) | `p = d * n` | ✅ Use this! |
| Depth of point `p` | `depth = d - n·p` | Positive = inside |
| Distance from point `p` to plane | `dist = |d - n·p|` | Always positive |
| Project point `p` onto plane | `p_proj = p - (d - n·p) * n` | Equivalent to `p - depth * n` |
| Is point inside? | `n·p < d` | Equivalent to `depth > 0` |

### Example: Creating Common Planes

```cpp
// Horizontal plane at z = 0 (ground), normal pointing up
PlaneID ground = createPlane(id, 0.0, 0.0, 1.0, 0.0, material);
// Equation: z = 0, inside is below (z < 0)

// Horizontal plane at y = 3, normal pointing down (-y direction)
PlaneID ceiling = createPlane(id, 0.0, -1.0, 0.0, -3.0, material);
// Equation: -y = -3 → y = 3, inside is above (y > 3)

// Vertical wall at x = 5, normal pointing left (-x direction)
PlaneID wall = createPlane(id, -1.0, 0.0, 0.0, -5.0, material);
// Equation: -x = -5 → x = 5, inside is to the right (x > 5)
```

---

## Historical Context: The Bug That Led to This Document

A bug was discovered in the DistanceMap plane collision detection algorithm (`MaxContacts.h`, function `collidePlaneTMeshWithDistanceMap`) where the plane reference point was computed as:

```cpp
Vec3 planeOrigin = -plane->getDepth(Vec3(0,0,0)) * planeNormal;  // BUG!
```

This worked correctly for planes passing through the origin (`d = 0`) but failed for all other planes. The bug was hidden during initial testing because only horizontal ground planes at `z = 0` were tested.

When testing with a plane at `y = 3` (normal `(0,-1,0)`, `d = -3`), the mesh would sink halfway through the plane before stopping. The root cause was that the sampling grid was being placed at `y = -3` instead of `y = 3`.

**Lesson**: Always test plane-related code with:
1. Planes at the origin (`d = 0`)
2. Planes away from the origin (`d ≠ 0`)
3. Multiple normal directions (not just vertical)

---

## Recommendations for Developers

1. **Always verify plane position**: After creating a plane, verify it's in the expected location by testing a few known points with `getDepth()`.

2. **Use helper functions**: When implementing plane algorithms, write helper functions with clear names:
   ```cpp
   Vec3 getPointOnPlaneSurface(PlaneID plane) {
       return plane->getDisplacement() * plane->getNormal();
   }
   ```

3. **Add assertions**: When debugging plane code, add assertions to check invariants:
   ```cpp
   Vec3 pointOnPlane = plane->getDisplacement() * plane->getNormal();
   assert(std::abs(plane->getDepth(pointOnPlane)) < 1e-6); // Should be ~0
   ```

4. **Document assumptions**: When writing plane-related code, explicitly document which convention you're using in comments.

5. **Test thoroughly**: Always test with multiple plane orientations and positions, not just ground planes at the origin.

---

## See Also

- `pe/core/rigidbody/Plane.h` - Complete Plane class API
- `doc/CollisionDetectionConventions.md` - General collision detection conventions in PE
- `doc/MaxContacts.md` - Fine collision detection implementation details

---

## Version History

- **2025-01-XX**: Initial document created after fixing DistanceMap plane collision bug
- Contributors: [Your names here]
