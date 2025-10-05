# PE Collision Detection Conventions

This document explains the fundamental conventions and calculations used in PE's collision detection system, particularly focusing on normal directions, relative velocity calculations, and penetration depth conventions.

## Overview

PE (Physics Engine) uses a consistent set of conventions for collision detection and contact resolution. Understanding these conventions is crucial for implementing new collision algorithms or debugging contact issues.

## Contact Normal Direction Convention

### Primary Rule: Body2 → Body1
PE follows the convention that **contact normals point from Body2 to Body1**.

```cpp
// Example from collideSphereSphere in MaxContacts.h:
Vec3 normal( s1->getPosition() - s2->getPosition() );  // Points from s2 to s1
contacts.addVertexFaceContact( s1, s2, contactPoint, normal, penetration );
//                            body1, body2
```

### Practical Implications:
- For plane-mesh collision: Normal points **from plane to mesh**
- For sphere-sphere collision: Normal points **from second sphere to first sphere**
- Contact normal should point in the direction that separates the objects

## Penetration Depth Convention

### Sign Convention: Negative = Collision
PE uses **negative penetration depths** to indicate colliding contacts:

```cpp
// Fallback plane-mesh collision (MaxContacts.h line 3283):
contacts.addVertexFaceContact( mesh, plane, contactPoint, plane->getNormal(), -penetrationDepth );
//                                                                              ^^^^ NEGATIVE
```

### Interpretation:
- **Negative penetration**: Objects are colliding/interpenetrating
- **Positive penetration**: Objects are separating
- **Zero penetration**: Objects are just touching

### Why Negative?
The contact solver uses penetration depth with time step to calculate penetration correction:
```cpp
gdot_nto[0] += dist_[i] * dtinv;  // Add penetration correction
```
- Negative `dist_[i]` creates negative correction → collision response
- Positive `dist_[i]` creates positive correction → separation (wrong for collision)

## Relative Normal Velocity (gdot_n)

### Definition
The relative normal velocity `gdot_n` represents how fast objects are approaching/separating in the contact normal direction.

### Calculation
```cpp
// From HardContactAndFluid.h contact resolution:
Vec3 gdot = ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] )
          + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i]
          - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i];

// Project to contact frame (normal direction)
Mat3 contactframe( n_[i], t_[i], o_[i] );
Vec3 gdot_nto( trans( contactframe ) * gdot );
gdot_nto[0] += dist_[i] * dtinv;  // Add penetration correction
```

### Components:
- `v_[body1] - v_[body2]`: Linear velocity difference
- `w_[body1] % r1 - w_[body2] % r2`: Angular velocity contribution at contact points
- `dist_[i] * dtinv`: Penetration correction term

### Sign Interpretation:
- **Negative gdot_n**: Objects approaching → Collision response needed
- **Positive gdot_n**: Objects separating → No collision response
- **~Zero gdot_n**: Objects at rest relative to each other

## Contact Resolution Process

### Contact States
PE classifies contacts based on relative velocity:
```cpp
// Contact state determination (from debug output):
if (gdot_n < 0) {
    // "colliding" - objects approaching
    // Apply collision response forces
} else {
    // "separating" - objects moving apart
    // No forces needed
}
```

### Force Application
The contact solver applies forces to achieve:
1. **Zero relative normal velocity** (`gdot_n ≈ 0`)
2. **Non-penetrating contact** (`dist ≥ 0` after integration)

## Common Issues and Debugging

### Issue: Objects Sinking Through Contacts
**Symptoms:**
- Positive `gdot_n` values for colliding objects
- Zero collision forces (`force=<0,0,0>`)
- Gradual penetration increase over time

**Diagnosis:**
```cpp
// Debug output shows:
Contact X (colliding): gdot_n=0.083, dist=0.000166, force=<0,0,0>  // WRONG
// Should be:
Contact X (colliding): gdot_n=-0.589, dist=-0.000166, force=<nonzero>  // CORRECT
```

**Common Causes:**
1. **Wrong penetration sign**: Using positive instead of negative penetration
2. **Incorrect normal direction**: Normal pointing wrong way
3. **Coordinate transformation errors**: World/local coordinate confusion

### Issue: Inconsistent Collision Detection
**Symptoms:**
- Different behavior between collision algorithms
- Contacts marked as "colliding" but behaving like "separating"

**Solution:**
- Verify penetration depth calculation matches fallback algorithm
- Check normal direction follows Body2→Body1 convention
- Ensure coordinate transformations preserve sign conventions

## Algorithm Implementation Guidelines

### For New Collision Algorithms:
1. **Study fallback implementation** for reference behavior
2. **Match penetration sign convention** (negative = collision)
3. **Follow normal direction convention** (Body2→Body1)
4. **Test relative velocity calculation** (should be negative for collision)

### Debugging Checklist:
- [ ] Penetration depths are negative for colliding contacts
- [ ] Normal directions point from Body2 to Body1
- [ ] Relative normal velocities are negative for approaching objects
- [ ] Contact forces are non-zero for colliding contacts
- [ ] Objects don't sink through each other over time

## Examples

### Correct DistanceMap Implementation:
```cpp
// Fixed implementation from MaxContacts.h:
Vec3 contactNormal = plane->getNormal();  // Points from plane to mesh
contacts.addVertexFaceContact(mesh, plane, contactPoint, contactNormal, -candidate.penetration);
//                           body1, body2                                  ^^^^ NEGATIVE
```

### Expected Debug Output:
```
Contact 0 (colliding): gdot_n=-0.589, dist=-0.000166, force=<0.062,-0.219,1.074>
```
- Negative `gdot_n`: Objects approaching ✓
- Negative `dist`: Penetration ✓
- Non-zero `force`: Collision response ✓

## References

- `pe/core/detection/fine/MaxContacts.h`: Contact detection algorithms
- `pe/core/collisionsystem/HardContactAndFluid.h`: Contact resolution
- Fallback plane-mesh collision (MaxContacts.h line 3283): Reference implementation
- DistanceMap plane collision (MaxContacts.h line 4429): Fixed implementation

---
*This documentation was created after resolving the DistanceMap collision detection issue where incorrect penetration depth signs caused gradual object sinking.*