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
#include <stdexcept>
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

// Detect whether the active (compile-time selected) collision system invokes the shared
// lubrication stage (pe/core/lubrication/LubricationStage.h).
//
// A stage-capable pipeline advertises itself with a static constexpr member
//   static constexpr bool hasLubricationStage = true;
// which is the single source of truth for "this solver honors lubricationEnabled_".
// It is deliberately an explicit opt-in rather than a structural probe: whether a
// solver CALLS the stage is not observable from its type, and silently guessing is
// exactly the failure mode this guard exists to prevent.
template <typename CollisionSystemT, typename = void>
struct HasLubricationStage : std::false_type {};

template <typename CollisionSystemT>
struct HasLubricationStage<CollisionSystemT,
                           std::void_t<decltype(CollisionSystemT::hasLubricationStage)>>
    : std::integral_constant<bool, CollisionSystemT::hasLubricationStage> {};

// Pushes all runtime lubrication parameters from SimulationConfig into the
// pe::lubrication:: store, which is where the detection branches, the AABB padding
// helper and the stage all read from.
//
// Refuses loudly (D2.2 §3.5) when lubricationEnabled_ is true but the active collision
// system does not invoke the stage: the json switch would otherwise change AABB padding
// and generate pre-contact pairs while applying no lubrication force whatsoever -- a
// silent no-op that looks like a working configuration.
template <typename CollisionSystemT>
inline void applyOptionalLubricationParams(CollisionSystemT& cs, const SimulationConfig& config) {
  (void)cs;
  const bool enabled = config.getLubricationEnabled();

  if (enabled && !HasLubricationStage<CollisionSystemT>::value) {
    throw std::runtime_error(
        "lubricationEnabled_ is true but the active pe_CONSTRAINT_SOLVER does not invoke "
        "the lubrication stage, so no lubrication force would be applied. Select a "
        "stage-capable solver (e.g. pe::response::HardContactAndFluid) or set "
        "lubricationEnabled_ to false.");
  }

  lubrication::setEnabled(enabled);
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
  lubrication::setMinGap(config.getMinEpsLub());
  lubrication::setAlphaImpulseCap(config.getAlphaImpulseCap());

  // The hysteresis half-widths are lubrication BLEND bands: they widen the hard-contact
  // weight from a step into a ramp. With lubrication off they must be exactly zero, or
  // the shared detection path would stop being bitwise-identical to a no-lubrication
  // build (Gate G0). Pushed only when the add-on is active.
  lubrication::setContactHysteresisDelta(enabled ? config.getContactHysteresisDelta()
                                                 : real(0));
  lubrication::setLubricationHysteresisDelta(
      enabled ? config.getLubricationHysteresisDelta() : real(0));

  // Expert mode: with inflation off, coarse detection may miss pairs entering the
  // lubrication band between updates. Legitimate when the cutoff is small, but loud.
  if (enabled && !config.getLubricationAabbInflation()) {
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
