//=================================================================================================
/*!
 *  \file pe/interface/el_optional_api.h
 *  \brief Compile-time detection of the optional Euler-Lagrange / SRR collision-system API
 *
 *  The CFD coupling interface is compiled once against whatever constraint solver
 *  pe_CONSTRAINT_SOLVER selects (see pe/config/Collisions.h). Some entry points call
 *  methods that only one specialization provides:
 *
 *    - the Euler-Lagrange virial/impulse diagnostics and the per-body hydrodynamic
 *      state setters exist only on response::HardContactEulerLagrange;
 *    - the short-range-repulsion knobs (rho/epsP/epsW/gamma, sub-cycle count) are only
 *      meaningful for solvers that implement SRR.
 *
 *  Referring to them unconditionally makes the interface library fail to build for every
 *  other solver (e.g. response::HardContactAndFluid). The detectors below follow the same
 *  pattern already used for HasLubricationParamSetters in pe/interface/sim_setup_serial.h:
 *  a std::void_t detector plus a single `if constexpr` dispatch function. A single
 *  `if constexpr` function is used deliberately rather than a two-overload SFINAE pair,
 *  because two overloads with identical parameter lists are ambiguous whenever both
 *  candidates are viable -- i.e. exactly for the solver the guard is meant to support.
 *
 *  Fallback semantics for solvers without the EL API: all diagnostics report zero (a
 *  well-defined "no EL contribution measured" value that the Fortran side can consume),
 *  and the per-body hydro-state setters are no-ops.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_EL_OPTIONAL_API_H_
#define _PE_INTERFACE_EL_OPTIONAL_API_H_

#include <pe/core.h>
#include <pe/math/Vector3.h>
#include <type_traits>
#include <utility>

namespace pe {

//=================================================================================================
// Euler-Lagrange diagnostic queries on the collision system
//=================================================================================================

//! Detect whether the active collision system exposes the EL virial/impulse diagnostics.
template <typename CollisionSystemT, typename = void>
struct HasElDiagnosticQueries : std::false_type {};

template <typename CollisionSystemT>
struct HasElDiagnosticQueries<CollisionSystemT, std::void_t<
    decltype(std::declval<const CollisionSystemT&>().getElLubricationVirial(std::declval<real*>())),
    decltype(std::declval<const CollisionSystemT&>().getElLubricationPairs()),
    decltype(std::declval<const CollisionSystemT&>().getElLubricationImpulse(std::declval<real*>())),
    decltype(std::declval<const CollisionSystemT&>().getElContactVirial(std::declval<real*>())),
    decltype(std::declval<const CollisionSystemT&>().getElContactVirialPairs()),
    decltype(std::declval<const CollisionSystemT&>().getElWallLubImpulse(int{}, std::declval<real*>())),
    decltype(std::declval<const CollisionSystemT&>().getElWallContactImpulse(int{}, std::declval<real*>())),
    decltype(std::declval<const CollisionSystemT&>().getElWallLubPairs())>>
    : std::true_type {};

//*************************************************************************************************
/*!\brief Zero-fills \a n components of \a out; the no-EL-solver fallback value. */
inline void elZeroFill(real* out, std::size_t n) {
  if (out == nullptr) return;
  for (std::size_t i = 0; i < n; ++i) out[i] = real(0);
}
//*************************************************************************************************

//! Lubrication impulse virial (9 components, row-major); zero when unsupported.
template <typename CollisionSystemT>
inline void elLubricationVirial(const CollisionSystemT& cs, real* sigma) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    cs.getElLubricationVirial(sigma);
  } else {
    (void)cs;
    elZeroFill(sigma, 9);
  }
}

//! Count of lubrication pair evaluations; zero when unsupported.
template <typename CollisionSystemT>
inline std::size_t elLubricationPairs(const CollisionSystemT& cs) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    return static_cast<std::size_t>(cs.getElLubricationPairs());
  } else {
    (void)cs;
    return 0u;
  }
}

//! Net momentum folded by the lubrication sweep (3 components); zero when unsupported.
template <typename CollisionSystemT>
inline void elLubricationImpulse(const CollisionSystemT& cs, real* dp) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    cs.getElLubricationImpulse(dp);
  } else {
    (void)cs;
    elZeroFill(dp, 3);
  }
}

//! Contact impulse virial (9 components, row-major); zero when unsupported.
template <typename CollisionSystemT>
inline void elContactVirial(const CollisionSystemT& cs, real* sigma) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    cs.getElContactVirial(sigma);
  } else {
    (void)cs;
    elZeroFill(sigma, 9);
  }
}

//! Count of contact-virial pair evaluations; zero when unsupported.
template <typename CollisionSystemT>
inline std::size_t elContactVirialPairs(const CollisionSystemT& cs) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    return static_cast<std::size_t>(cs.getElContactVirialPairs());
  } else {
    (void)cs;
    return 0u;
  }
}

//! Per-wall lubrication impulse (3 components); zero when unsupported.
template <typename CollisionSystemT>
inline void elWallLubImpulse(const CollisionSystemT& cs, int wall, real* dp) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    cs.getElWallLubImpulse(wall, dp);
  } else {
    (void)cs; (void)wall;
    elZeroFill(dp, 3);
  }
}

//! Per-wall contact impulse (3 components); zero when unsupported.
template <typename CollisionSystemT>
inline void elWallContactImpulse(const CollisionSystemT& cs, int wall, real* dp) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    cs.getElWallContactImpulse(wall, dp);
  } else {
    (void)cs; (void)wall;
    elZeroFill(dp, 3);
  }
}

//! Count of wall lubrication pair evaluations; zero when unsupported.
template <typename CollisionSystemT>
inline std::size_t elWallLubPairs(const CollisionSystemT& cs) {
  if constexpr (HasElDiagnosticQueries<CollisionSystemT>::value) {
    return static_cast<std::size_t>(cs.getElWallLubPairs());
  } else {
    (void)cs;
    return 0u;
  }
}

//! Detect whether the active collision system exposes the EL accumulator resets.
template <typename CollisionSystemT, typename = void>
struct HasElAccumulatorResets : std::false_type {};

template <typename CollisionSystemT>
struct HasElAccumulatorResets<CollisionSystemT, std::void_t<
    decltype(std::declval<CollisionSystemT&>().resetElLubricationVirial())>>
    : std::true_type {};

//*************************************************************************************************
/*!\brief Resets the per-macro-step lubrication impulse-virial accumulator.
 *
 * No-op for solvers that keep no such accumulator; their virial queries already report zero.
 */
template <typename CollisionSystemT>
inline void elResetLubricationVirial(CollisionSystemT& cs) {
  if constexpr (HasElAccumulatorResets<CollisionSystemT>::value) {
    cs.resetElLubricationVirial();
  } else {
    (void)cs;
  }
}
//*************************************************************************************************

//=================================================================================================
// Euler-Lagrange hydrodynamic state on a rigid body
//=================================================================================================

//! Detect whether the active body trait exposes the EL hydrodynamic state setters.
template <typename BodyT, typename = void>
struct HasElHydroState : std::false_type {};

template <typename BodyT>
struct HasElHydroState<BodyT, std::void_t<
    decltype(std::declval<BodyT&>().setEulerLagrangeHydroState(
        real{}, std::declval<const Vec3&>(), std::declval<const Vec3&>(), bool{})),
    decltype(std::declval<BodyT&>().setEulerLagrangeRefVelocity(std::declval<const Vec3&>()))>>
    : std::true_type {};

//*************************************************************************************************
/*!\brief Stores the CFD-provided hydrodynamic state on \a body, if the solver supports it.
 *
 * No-op for solvers without the Euler-Lagrange body trait: those solvers carry no drag
 * closure, so there is no state to record.
 */
template <typename BodyT>
inline void elSetHydroState(BodyT body, real dragB, const Vec3& carrierVelocity,
                            const Vec3& otherForce, bool active) {
  using BodyType = typename std::remove_pointer<BodyT>::type;
  if constexpr (HasElHydroState<BodyType>::value) {
    body->setEulerLagrangeHydroState(dragB, carrierVelocity, otherForce, active);
  } else {
    (void)body; (void)dragB; (void)carrierVelocity; (void)otherForce; (void)active;
  }
}
//*************************************************************************************************

//! Detect whether the active body trait exposes the EL hydro-state predicate.
template <typename BodyT, typename = void>
struct HasElHydroStateQuery : std::false_type {};

template <typename BodyT>
struct HasElHydroStateQuery<BodyT, std::void_t<
    decltype(std::declval<const BodyT&>().hasEulerLagrangeHydroState())>>
    : std::true_type {};

//*************************************************************************************************
/*!\brief Reports whether \a body currently carries a CFD-provided Euler-Lagrange hydro state.
 *
 * Always false for solvers without the Euler-Lagrange body trait: such a body cannot hold an
 * EL hydro state at all, so callers correctly fall through to their non-EL branch.
 */
template <typename BodyT>
inline bool elHasHydroState(BodyT body) {
  using BodyType = typename std::remove_pointer<BodyT>::type;
  if constexpr (HasElHydroStateQuery<BodyType>::value) {
    return body->hasEulerLagrangeHydroState();
  } else {
    (void)body;
    return false;
  }
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Arms the free-flight reference velocity of the semi-implicit drag sub-cycling. */
template <typename BodyT>
inline void elSetRefVelocity(BodyT body, const Vec3& uref) {
  using BodyType = typename std::remove_pointer<BodyT>::type;
  if constexpr (HasElHydroState<BodyType>::value) {
    body->setEulerLagrangeRefVelocity(uref);
  } else {
    (void)body; (void)uref;
  }
}
//*************************************************************************************************

//=================================================================================================
// Short-range-repulsion (SRR) parameters
//=================================================================================================

//! Detect whether the active contact solver exposes the SRR parameter setters.
template <typename ContactSolverT, typename = void>
struct HasSRRParamSetters : std::false_type {};

template <typename ContactSolverT>
struct HasSRRParamSetters<ContactSolverT, std::void_t<
    decltype(std::declval<ContactSolverT&>().setRho(real{})),
    decltype(std::declval<ContactSolverT&>().setEpsP(real{})),
    decltype(std::declval<ContactSolverT&>().setEpsW(real{})),
    decltype(std::declval<ContactSolverT&>().setGamma(real{}))>>
    : std::true_type {};

//! Detect whether the active collision system exposes sub-cycle control.
template <typename CollisionSystemT, typename = void>
struct HasSubcycleControl : std::false_type {};

template <typename CollisionSystemT>
struct HasSubcycleControl<CollisionSystemT, std::void_t<
    decltype(std::declval<CollisionSystemT&>().setNumSubcycles(std::size_t{}))>>
    : std::true_type {};

//*************************************************************************************************
/*!\brief Applies the SRR security-zone parameters only if the active solver implements them.
 *
 * Solvers without SRR (e.g. response::HardContactAndFluid) simply ignore these knobs, which
 * matches the behaviour of the solvers that already declare them as no-op stubs.
 */
template <typename ContactSolverT>
inline void applyOptionalSRRParams(ContactSolverT& solver, real rho, real epsP, real epsW,
                                   real gamma) {
  if constexpr (HasSRRParamSetters<ContactSolverT>::value) {
    solver.setRho  ( rho   );
    solver.setEpsP ( epsP  );
    solver.setEpsW ( epsW  );
    solver.setGamma( gamma );
  } else {
    (void)solver; (void)rho; (void)epsP; (void)epsW; (void)gamma;
  }
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Sets the contact sub-cycle count only if the active collision system supports it. */
template <typename CollisionSystemT>
inline void applyOptionalSubcycles(CollisionSystemT& cs, std::size_t nSubcycles) {
  if constexpr (HasSubcycleControl<CollisionSystemT>::value) {
    cs.setNumSubcycles(nSubcycles);
  } else {
    (void)cs; (void)nSubcycles;
  }
}
//*************************************************************************************************

} // namespace pe

#endif
