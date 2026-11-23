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

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <cstddef>
#include <type_traits>
#include <utility>

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
                            : config.getLubricationModel() == "kroupaDeficit"
                                  ? lubrication::modelKroupaDeficit
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

// Detect whether the active (compile-time selected) collision system has the restitution
// switch of the hard-contact solvers (HardContactSemiImplicitTimesteppingSolvers and
// HardContactEulerLagrange; see pe/core/collisionsystem/RestitutionPostPass.h).
template <typename CollisionSystemT, typename = void>
struct HasRestitutionSwitch : std::false_type {};

template <typename CollisionSystemT>
struct HasRestitutionSwitch<CollisionSystemT, std::void_t<
    decltype(std::declval<CollisionSystemT&>().setRestitution(bool{})),
    decltype(std::declval<CollisionSystemT&>().setRestitutionThreshold(real{})),
    decltype(std::declval<CollisionSystemT&>().setRestitutionPostPass(bool{})),
    decltype(std::declval<CollisionSystemT&>().setRestitutionIterations(std::size_t{})),
    decltype(std::declval<const CollisionSystemT&>().getRestitution()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionThreshold()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionPostPass()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionIterations()),
    decltype(std::declval<CollisionSystemT&>().setRestitutionTolerance(real{}, real{})),
    decltype(std::declval<CollisionSystemT&>().setRestitutionMaxIterations(std::size_t{})),
    decltype(std::declval<CollisionSystemT&>().setRestitutionCheckInterval(std::size_t{})),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionAbsTolerance()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionRelTolerance()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionCheckInterval()),
    decltype(std::declval<const CollisionSystemT&>().getRestitutionMaxIterations())>>
    : std::true_type {};

// Pushes the restitution switch (json restitutionEnabled_, restitutionThreshold_,
// restitutionScheme_, restitutionIterations_, restitutionMaxIterations_, restitutionCheckInterval_,
// restitutionAbsTolerance_, restitutionRelTolerance_; collective: every process must load the same deck; validated by the SimulationConfig setters) into the
// collision system. With the keys absent the values are the solver defaults (off, 0, capped
// post-pass, 4 sweeps), so the call leaves an inelastic solver bit for bit unchanged. The material
// value restitution_ is not touched: it is the coefficient a bouncing contact uses.
//
// Refuses loudly when restitutionEnabled_ is true but the active solver has no switch (e.g.
// HardContactAndFluid): the deck would otherwise ask for bouncing contacts and get inelastic ones.
// Call it on every process (the switch must be the same everywhere).
template <typename CollisionSystemT>
inline void applyOptionalRestitutionParams(CollisionSystemT& cs, const SimulationConfig& config) {
  if constexpr (HasRestitutionSwitch<CollisionSystemT>::value) {
    cs.setRestitution(config.getRestitutionEnabled());
    cs.setRestitutionThreshold(config.getRestitutionThreshold());
    cs.setRestitutionPostPass(config.getRestitutionScheme() == "post-pass");
    cs.setRestitutionIterations(static_cast<std::size_t>(config.getRestitutionIterations()));
    cs.setRestitutionMaxIterations(static_cast<std::size_t>(config.getRestitutionMaxIterations()));
    cs.setRestitutionCheckInterval(static_cast<std::size_t>(config.getRestitutionCheckInterval()));
    cs.setRestitutionTolerance(config.getRestitutionAbsTolerance(), config.getRestitutionRelTolerance());
  } else {
    (void)cs;
    if (config.getRestitutionEnabled()) {
      throw std::runtime_error(
          "restitutionEnabled_ is true but the active pe_CONSTRAINT_SOLVER has no restitution "
          "switch, so every contact would stay inelastic. The EL setups need "
          "pe::response::HardContactEulerLagrange (their stepping requires the EL hydro state); "
          "otherwise set restitutionEnabled_ to false.");
    }
  }
}

// Setups that do not support the restitution switch call this right after loading the deck (on
// every process): an explicit restitutionEnabled_ = true is refused instead of being ignored, and
// otherwise the deck's (disabled) settings are applied to the solver, so that a setup in the same
// process cannot inherit a switch an earlier, supporting setup turned on (World::clear() keeps the
// solver switches). Supporting setups call applyOptionalRestitutionParams() instead. Support means
// the setup applies the switch and gives its contacts a restitution from the deck; today that is
// the two EL setups with deck-driven materials, setupELTerminalVelocity and the parallel
// setupELFrozenTrace (see doc/technical-notes/el-restitution.md).
template <typename CollisionSystemT>
inline void rejectRestitutionRequest(CollisionSystemT& cs, const SimulationConfig& config, const std::string& setup) {
  if (config.getRestitutionEnabled()) {
    throw std::runtime_error(
        "restitutionEnabled_ is true, but " + setup + " does not support the restitution switch "
        "of the contact solver (it is applied by setupELTerminalVelocity and the parallel "
        "setupELFrozenTrace only). Set restitutionEnabled_ to false or remove the key.");
  }
  applyOptionalRestitutionParams(cs, config);   // switch off whatever an earlier setup left on
}

// Sets the shadow-copy margin for pairwise lubrication in a domain-decomposed (MPI) setup.
// Shared by the Euler-Lagrange setups; call it AFTER the process layout has been validated.
//
// The margin widens the shadow-copy overlap test so cross-boundary pairs within the
// surface-gap cutoff are visible to the designated treating rank. The full-visibility
// margin is sphereRadius + cutoff: for a pair (A owned by r1, B owned by r2) with
// gap < cutoff, r1 sees B iff dist(B_center, r1_box) <= R_B + margin, and that distance
// can reach R_A + R_B + cutoff when A's center sits on r1's boundary.
//
// The margin is CLAMPED so the total shadow reach (radius + margin) stays below the
// thinnest decomposed subdomain extent: pe cannot register shadow copies beyond direct
// neighbors ("Registering distant processes is not yet implemented"). Under the
// designated-treater relay a clamped margin is momentum-safe -- a pair the treater cannot
// see is skipped for a substep, never applied one-sided; only extremal near-cutoff pairs
// are affected.
//
// \param minSubdomainExtent Smallest extent of any subdomain along a DECOMPOSED axis
//        (axes with a single process do not count). Pass
//        std::numeric_limits<real>::max() when no axis is decomposed.
// \param verbose Print the clamp notice (pass true on one rank only).
//
// No-op when lubrication is disabled (the margin keeps its default of zero).
inline void applyLubricationShadowCopyMargin(const SimulationConfig& config,
                                             real minSubdomainExtent, bool verbose) {
  if (!config.getLubricationEnabled()) {
    return;
  }

  const real radius = config.getBenchRadius();
  real margin = config.getLubricationCutoff() + radius;
  const real reachCap = real(0.99) * minSubdomainExtent - radius;
  if (margin > reachCap) {
    if (verbose) {
      std::cout << "EL lubrication: shadow margin clamped " << margin
                << " -> " << reachCap << " (subdomain extent " << minSubdomainExtent
                << "); near-cutoff cross-rank pairs may be skipped.\n";
    }
    margin = reachCap;
  }
  lubrication::setShadowCopyMargin(std::max(margin, real(0)));
}

}  // namespace pe
