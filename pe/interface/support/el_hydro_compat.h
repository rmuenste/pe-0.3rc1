#pragma once

//=================================================================================================
// Solver-portable access to the Euler-Lagrange hydro state.
//
// setEulerLagrangeHydroState / hasEulerLagrangeHydroState exist only on the RigidBody trait
// of the HardContactEulerLagrange solver (pe/core/rigidbody/rigidbodytrait/
// HardContactEulerLagrange.h). The interface layer must keep compiling when a different
// pe_CONSTRAINT_SOLVER is selected (e.g. HardContactAndFluid), so all call sites go
// through these if-constexpr wrappers instead of calling the members directly.
//=================================================================================================

#include <pe/math/Vector3.h>

#include <type_traits>

namespace pe {

template <typename Body, typename = void>
struct HasEulerLagrangeHydroState : std::false_type {};

template <typename Body>
struct HasEulerLagrangeHydroState<Body, std::void_t<
    decltype(std::declval<Body&>().hasEulerLagrangeHydroState())>>
    : std::true_type {};

// True if the body carries an armed Euler-Lagrange hydro state; always false when the
// active solver has no such state.
template <typename Body>
inline bool bodyHasELHydroState(Body* body) {
  if constexpr (HasEulerLagrangeHydroState<Body>::value) {
    return body->hasEulerLagrangeHydroState();
  } else {
    (void)body;
    return false;
  }
}

// Sets the Euler-Lagrange hydro state if the active solver supports it and reports
// whether it did. Callers decide whether an unsupported solver is an error (external
// CFD entry points) or a harmless no-op (per-step state clearing).
template <typename Body>
inline bool trySetELHydroState(Body* body, real dragB, const Vec3& carrierVelocity,
                               const Vec3& otherForce, bool active) {
  if constexpr (HasEulerLagrangeHydroState<Body>::value) {
    body->setEulerLagrangeHydroState(dragB, carrierVelocity, otherForce, active);
    return true;
  } else {
    (void)body; (void)dragB; (void)carrierVelocity; (void)otherForce; (void)active;
    return false;
  }
}

}  // namespace pe
