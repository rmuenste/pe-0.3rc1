#pragma once

//=================================================================================================
// Optional collision-system parameter shims and the single config -> engine wiring point
// for lubrication parameters. Setup functions (serial and MPI) call
// applyOptionalLubricationParams(*theCollisionSystem(), SimulationConfig::getInstance())
// once after SimulationConfig::loadFromFile.
//=================================================================================================

#include <pe/config/SimulationConfig.h>
#include <pe/core/lubrication/Params.h>
#include <pe/core/lubrication/LubricationModel.h>

#include <iostream>
#include <cstddef>
#include <type_traits>

namespace pe {

template <typename CollisionSystemT>
inline auto setOptionalLubrication(CollisionSystemT *cs, bool value)
    -> decltype(cs->setLubrication(value), void()) {
  cs->setLubrication(value);
}

inline void setOptionalLubrication(...) {
  // Active collision system has no lubrication switch.
}

template <typename CollisionSystemT>
inline auto setOptionalSlipLength(CollisionSystemT *cs, real value)
    -> decltype(cs->setSlipLength(value), void()) {
  cs->setSlipLength(value);
}

inline void setOptionalSlipLength(...) {
  // Active collision system has no slip-length parameter.
}

template <typename CollisionSystemT>
inline auto setOptionalAdaptiveBaumgarteCapping(CollisionSystemT *cs, bool enabled, real factor)
    -> decltype(cs->setAdaptiveBaumgarteCapping(enabled, factor), void()) {
  cs->setAdaptiveBaumgarteCapping(enabled, factor);
}

inline void setOptionalAdaptiveBaumgarteCapping(...) {
  // Active collision system has no adaptive Baumgarte capping.
}

// Detect whether the active (compile-time selected) collision system exposes the
// short-range-repulsion contact-solver parameters (rho/epsP/epsW/gamma + subcycling).
// Only the SRR-capable solver stacks provide them.
template <typename CollisionSystemT, typename = void>
struct HasSRRContactSolverParams : std::false_type {};

template <typename CollisionSystemT>
struct HasSRRContactSolverParams<CollisionSystemT, std::void_t<
    decltype(std::declval<CollisionSystemT&>().getContactSolver().setRho(real{})),
    decltype(std::declval<CollisionSystemT&>().getContactSolver().setEpsP(real{})),
    decltype(std::declval<CollisionSystemT&>().getContactSolver().setEpsW(real{})),
    decltype(std::declval<CollisionSystemT&>().getContactSolver().setGamma(real{})),
    decltype(std::declval<CollisionSystemT&>().setNumSubcycles(std::size_t{}))>>
    : std::true_type {};

// Apply SRR contact-solver parameters only if the active collision system exposes them;
// keeps SRR-specific setup functions compiling under other solvers (e.g. the lubricated
// stack), where the call degrades to a loud no-op.
template <typename CollisionSystemT>
inline void applyOptionalSRRParams(CollisionSystemT& cs, real rho, real epsP, real epsW,
                                   real gamma, std::size_t nSubcycles) {
  if constexpr (HasSRRContactSolverParams<CollisionSystemT>::value) {
    cs.getContactSolver().setRho(rho);
    cs.getContactSolver().setEpsP(epsP);
    cs.getContactSolver().setEpsW(epsW);
    cs.getContactSolver().setGamma(gamma);
    cs.setNumSubcycles(nSubcycles);
  } else {
    (void)cs; (void)rho; (void)epsP; (void)epsW; (void)gamma; (void)nSubcycles;
    std::cerr << "[pe] WARNING: active pe_CONSTRAINT_SOLVER has no SRR contact-solver "
                 "parameters; applyOptionalSRRParams is a no-op.\n";
  }
}

// Detect whether the active (compile-time selected) collision system exposes the
// lubrication/contact-hysteresis setters. Only HardContactLubricated provides them.
template <typename CollisionSystemT, typename = void>
struct HasLubricationParamSetters : std::false_type {};

template <typename CollisionSystemT>
struct HasLubricationParamSetters<CollisionSystemT, std::void_t<
    decltype(std::declval<CollisionSystemT&>().setContactHysteresisDelta(real{})),
    decltype(std::declval<CollisionSystemT&>().setLubricationHysteresisDelta(real{})),
    decltype(std::declval<CollisionSystemT&>().setAlphaImpulseCap(real{})),
    decltype(std::declval<CollisionSystemT&>().setMinEpsLub(real{}))>>
    : std::true_type {};

// Apply all runtime lubrication parameters from SimulationConfig:
//  - per-solver knobs via the collision-system setters, only if the active solver
//    exposes them (e.g., HardContactLubricated) -- see the trait above. A single
//    `if constexpr` function is used deliberately: a two-overload SFINAE pair with
//    identical parameter lists is ambiguous whenever both candidates are viable
//    (i.e. exactly the lubricated solver this is meant to support).
//  - model switches/parameters into the pe::lubrication:: global store, always;
//    they are read only by the lubrication-aware detection branches, the AABB
//    padding helper, and the HardContactLubricated solver, so pushing them is
//    harmless for other solvers.
template <typename CollisionSystemT>
inline void applyOptionalLubricationParams(CollisionSystemT& cs, const SimulationConfig& config) {
  if constexpr (HasLubricationParamSetters<CollisionSystemT>::value) {
    cs.setContactHysteresisDelta(config.getContactHysteresisDelta());
    cs.setLubricationHysteresisDelta(config.getLubricationHysteresisDelta());
    cs.setAlphaImpulseCap(config.getAlphaImpulseCap());
    cs.setMinEpsLub(config.getMinEpsLub());
  } else {
    // Constraint solver does not expose lubrication/hysteresis controls; nothing to do.
    (void)cs;
  }

  lubrication::setEnabled(config.getLubricationEnabled());
  lubrication::setModel(config.getLubricationModel() == "legacy"
                            ? lubrication::modelLegacy
                            : lubrication::modelKroupa2016);
  lubrication::setScheme(config.getLubricationIntegration() == "explicit-capped"
                             ? lubrication::schemeExplicitCapped
                             : lubrication::schemeSemiImplicit);
  lubrication::setTangential(config.getLubricationTangential());
  lubrication::setTwisting(config.getLubricationTwisting());
  lubrication::setSlipCorrection(config.getLubricationSlipCorrection());
  lubrication::setResistSeparation(config.getLubricationOnSeparation());
  lubrication::setWallTerms(config.getLubricationWallTerms());
  lubrication::setEpsCritical(config.getLubricationEpsCritical());
  lubrication::setCutoffFactor(config.getLubricationCutoffFactor());
  lubrication::setMeshClampFactor(config.getLubricationMeshClampFactor());
  lubrication::setAabbInflation(config.getLubricationAabbInflation());

  // Expert mode: with inflation off, coarse detection may miss pairs entering the
  // lubrication band between updates. Legitimate when the cutoff is small, but loud.
  if (config.getLubricationEnabled() && !config.getLubricationAabbInflation()) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      std::cerr << "[pe] WARNING: lubrication is enabled but AABB inflation is switched "
                   "off (lubricationAabbInflation_ = false); pairs may enter the "
                   "lubrication band undetected between coarse-detection updates.\n";
    }
  }
}

}  // namespace pe
