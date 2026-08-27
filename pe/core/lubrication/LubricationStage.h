//=================================================================================================
/*!
 *  \file pe/core/lubrication/LubricationStage.h
 *  \brief Solver-agnostic application stage for pairwise lubrication forces
 *
 *  This header holds the *application* half of the lubrication add-on: the per-timestep
 *  loop that walks the tagged lubrication contacts, evaluates the closure in
 *  pe/core/lubrication/LubricationModel.h, and folds the resulting impulses into a
 *  hard-contact solver's velocity-correction arrays.
 *
 *  It is deliberately a free function template over the solver's contact container and
 *  velocity buffers rather than a member of any one collision system: every hard-contact
 *  pipeline in pe caches (v, w, dv, dw) indexed by RigidBody::index_ and masks its contact
 *  list the same way, so the stage composes with all of them. A pipeline opts in with a
 *  single call placed immediately after its first synchronizeVelocities():
 *
 *  \code
 *     if( lubrication::isEnabled() ) {
 *        lubrication::applyLubricationStage( contacts, contactsMask_, v_, w_, dv_, dw_, dt );
 *        synchronizeVelocities();   // shadow copies must see the lubrication corrections
 *     }
 *  \endcode
 *
 *  The guard is the caller's, not the stage's, so that a disabled build pays exactly one
 *  predictable branch and never touches the extra velocity synchronization (an MPI
 *  collective). The stage itself re-checks the switch for direct/unit-test callers.
 *
 *  Extracted verbatim (modulo parameter sourcing) from the HardContactLubricated
 *  collision system, which is retired in the same change. The semi-implicit integrator,
 *  the legacy regression path, the blend-weight scaling and the diagnostics are
 *  behavior-preserving; tests/interface/pe_lubrication_legacy_regression_test.cpp and
 *  pe_lubrication_two_sphere_approach_test.cpp are the fidelity check.
 *
 *  Model reference: Kroupa et al., Langmuir 2016, 32, 8451-8460. See
 *  doc/technical-notes/lubrication-production-design.md.
 */
//=================================================================================================

#ifndef _PE_CORE_LUBRICATION_LUBRICATIONSTAGE_H_
#define _PE_CORE_LUBRICATION_LUBRICATIONSTAGE_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Settings.h>
#include <pe/core/TimeStep.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/logging/DebugSection.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>


namespace pe {
namespace lubrication {

//=================================================================================================
//
//  LUBRICATION APPLICATION STAGE
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reads the current runtime lubrication configuration into a per-timestep snapshot.
 *
 * The per-contact loop reads only the returned struct, never the global store, so a
 * configuration change mid-step cannot make a timestep internally inconsistent.
 */
inline ModelConfig currentModelConfig()
{
   ModelConfig cfg;
   cfg.enabled          = isEnabled();
   cfg.tangential       = getTangential();
   cfg.twisting         = getTwisting();
   cfg.slipCorrection   = getSlipCorrection();
   cfg.resistSeparation = getResistSeparation();
   cfg.wallTerms        = getWallTerms();
   cfg.model            = getModel();
   cfg.scheme           = getScheme();
   cfg.epsCritical      = getEpsCritical();
   cfg.cutoffFactor     = getCutoffFactor();
   cfg.minGap           = getMinGap();
   cfg.alphaImpulseCap  = getAlphaImpulseCap();
   cfg.viscosity        = Settings::liquidViscosity();
   return cfg;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Summary diagnostics produced by one invocation of the stage.
 *
 * \a dissipation must be <= 0: the model is purely resistive, so a positive value is a
 * sign-convention bug. Returned (rather than only logged) so unit tests can assert on it.
 */
struct StageDiagnostics {
   real   totalForce       = real(0);  //!< Sum of applied force magnitudes
   real   dissipation      = real(0);  //!< Sum of J.g + L.w over the step; must be <= 0
   real   maxNormalImpulse = real(0);  //!< Largest normal impulse applied
   size_t numContacts      = 0;        //!< Lubrication contacts actually acted on
   size_t numSaturated     = 0;        //!< Contacts inside the h_c saturation freeze
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Applies lubrication velocity corrections for all tagged lubrication contacts.
 *
 * \param contacts     The solver's contact container for this timestep.
 * \param contactsMask Per-contact mask; false entries are not owned by this process.
 * \param v            Cached pre-correction linear velocities, indexed by RigidBody::index_.
 * \param w            Cached pre-correction angular velocities, same indexing.
 * \param dv           Linear velocity corrections, accumulated in place.
 * \param dw           Angular velocity corrections, accumulated in place.
 * \param dt           Timestep size.
 * \return Summary diagnostics for the invocation.
 *
 * Acts only on contacts carrying the lubrication flag; hard contacts are left untouched
 * for the caller's constraint solver. Supports sphere-sphere and sphere-plane pairs;
 * any other geometry is skipped.
 */
template< typename Contacts >   // Type of the solver's contact container
inline StageDiagnostics applyLubricationStage( const Contacts& contacts,
                                               const std::vector<bool>& contactsMask,
                                               const std::vector<Vec3>& v,
                                               const std::vector<Vec3>& w,
                                               std::vector<Vec3>& dv,
                                               std::vector<Vec3>& dw,
                                               real dt )
{
   StageDiagnostics diag;

   const ModelConfig lubCfg = currentModelConfig();
   if( !lubCfg.enabled )
      return diag;

   // A configured mesh clamp with no pushed dx silently degrades to the bare relative
   // cutoff (cutoffFactor*aRef) - the run then applies lubrication over a band the case
   // design did not choose. Warn once, loudly, instead of guessing (same policy as the
   // solver-capability refusal in applyOptionalLubricationParams).
   if( getMeshClampFactor() > real(0) && getMeshDx() <= real(0) ) {
      static bool warned = false;
      if( !warned ) {
         warned = true;
         std::cerr << "[pe] WARNING: lubricationMeshClampFactor_ > 0 but no CFD mesh "
                      "width was pushed (set_lubrication_mesh_dx never called); the "
                      "mesh clamp is DISARMED and the outer cutoff falls back to "
                      "cutoffFactor*aRef alone.\n";
      }
   }

   // (1 - exp(-x))/x clamp factor for explicit modes: ~1 for soft modes, prevents
   // overshoot for stiff ones (exact integration of a linear drag mode over dt).
   const auto expClamp = []( real x ) {
      return ( x > real(1e-8) ) ? ( real(1) - std::exp( -x ) ) / x : real(1);
   };

   const size_t numContacts = contacts.size();

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Applying lubrication forces:\n";
   }

   for( size_t i = 0; i < numContacts; ++i ) {
      if( !contactsMask[i] ) continue;
      // Deliberately deduced, not spelled as ContactID/BodyID: keeping these expressions
      // DEPENDENT on the template parameter defers name lookup of body->index_ to
      // instantiation. index_ lives on the hard-contact RigidBody trait, so a concrete
      // BodyID here would hard-error at parse time in any translation unit whose active
      // solver lacks it (e.g. ShortRangeRepulsion), merely because this header was
      // included. It also makes "carries index_" the de facto stage-capability contract.
      auto c = contacts[i];
      // Only act on lubrication contacts; keep hard contacts for the constraint solver
      if( !c->getLubricationFlag() ) continue;

      auto b1 = c->getBody1();
      auto b2 = c->getBody2();
      const size_t i1 = b1->index_;
      const size_t i2 = b2->index_;

      // Geometry offsets at contact
      const Vec3 r1 = c->getPosition() - b1->getPosition();
      const Vec3 r2 = c->getPosition() - b2->getPosition();
      const Vec3 n  = c->getNormal();
      const real h  = c->getDistance();
      const real blend = c->getLubricationWeight();

      if( blend <= real(0) ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "         Status: SKIPPED (blend factor <= 0)\n";
         }
         continue;
      }

      const real invm1 = b1->getInvMass();
      const real invm2 = b2->getInvMass();
      const real invm_sum = invm1 + invm2;
      if( invm_sum <= real(0) ) continue;   // both bodies immobile: impulses are no-ops
      const real m_eff = real(1) / invm_sum;

      // Pre-correction velocities and relative surface velocity at the contact
      const Vec3 v1_pre = v[i1] + dv[i1];
      const Vec3 w1_pre = w[i1] + dw[i1];
      const Vec3 v2_pre = v[i2] + dv[i2];
      const Vec3 w2_pre = w[i2] + dw[i2];
      const Vec3 gdot = v1_pre - v2_pre + w1_pre % r1 - w2_pre % r2;
      const real vrn = trans(n) * gdot; // normal component, < 0 on approach

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Lubrication contact " << i << " (bodies " << b1->getID()
             << "-" << b2->getID() << "):\n"
             << "         Gap distance    = " << h << "\n"
             << "         Normal vrn      = " << vrn << "\n";
      }

      // Geometry dispatch: legacy reduced radius R_eff and pair reference radius aRef
      // (aRef defines eps = h/aRef; equals the paper's R_p for equal spheres).
      real R_eff = real(0);
      real aRef  = real(0);
      bool wall  = false;
      if( b1->getType() == sphereType && b2->getType() == sphereType ) {
         SphereID s1 = static_body_cast<Sphere>( b1 );
         SphereID s2 = static_body_cast<Sphere>( b2 );
         const real r1s = s1->getRadius();
         const real r2s = s2->getRadius();
         R_eff = ( r1s * r2s ) / ( r1s + r2s );
         aRef  = real(2) * R_eff;
      }
      else if( b1->getType() == sphereType && b2->getType() == planeType ) {
         SphereID s1 = static_body_cast<Sphere>( b1 );
         R_eff = s1->getRadius();
         aRef  = R_eff;
         wall  = true;
      }
      else if( b2->getType() == sphereType && b1->getType() == planeType ) {
         SphereID s2 = static_body_cast<Sphere>( b2 );
         R_eff = s2->getRadius();
         aRef  = R_eff;
         wall  = true;
      }
      else {
         // For now only sphere-sphere and sphere-plane
         pe_LOG_DEBUG_SECTION( log ) {
            log << "         Status: SKIPPED (unsupported geometry)\n";
         }
         continue;
      }

      if( R_eff <= real(0) ) continue;

      //=========================================================================
      // LEGACY MODEL: leading-order normal term, approach-only, impulse-capped.
      // Kept operation-for-operation identical to the pre-Kroupa solver so that
      // lubricationModel_="legacy" reproduces historical trajectories bit-for-bit
      // (guarded by tests/interface/pe_lubrication_legacy_regression_test.cpp).
      //=========================================================================
      if( lubCfg.model == modelLegacy ) {
         if( vrn >= real(0) ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "         Status: SKIPPED (separating, vrn >= 0)\n";
            }
            continue;    // legacy path only resists approaching motion
         }

         const real gap = std::max( h, lubCfg.minGap );
         const real Fmag_uncapped =
            legacyForceMagnitude( gap, R_eff, lubCfg.viscosity, vrn );
         real Fmag_capped = Fmag_uncapped;

         // Impulse capping relative to approach momentum; cap scaled by the blend
         // weight so it ramps up near hard-contact entry.
         const real Jprop = Fmag_capped * dt;
         const real Jcap  = ( lubCfg.alphaImpulseCap * blend ) * m_eff * (-vrn);
         if( Jprop > Jcap && Jprop > real(0) ) {
            Fmag_capped *= ( Jcap / Jprop );
         }

         const real Fmag_weighted = Fmag_capped * blend;
         const Vec3 F = Fmag_weighted * n;
         diag.totalForce += Fmag_weighted;

         dv[i1] += invm1 * F * dt;
         dw[i1] += b1->getInvInertia() * ( r1 % ( F * dt ) );
         dv[i2] -= invm2 * F * dt;
         dw[i2] += b2->getInvInertia() * ( r2 % ( -F * dt ) );

         ++diag.numContacts;
         diag.dissipation += trans( F * dt ) * gdot;
         diag.maxNormalImpulse = std::max( diag.maxNormalImpulse, Fmag_weighted * dt );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "         Model           = legacy\n"
                << "         Applied force   = " << Fmag_weighted << "\n"
                << "         Status: APPLIED\n";
         }
         continue;
      }

      //=========================================================================
      // KROUPA 2016 MODEL: full pairwise resistance set (normal squeeze with log
      // corrections, sliding force, sliding + twisting torques; wall variants),
      // Vinogradova slip correction and h_c saturation.
      //=========================================================================
      PairKinematics kin;
      kin.n = n;  kin.h = h;
      kin.r1 = r1;  kin.r2 = r2;
      kin.v1 = v1_pre;  kin.v2 = v2_pre;
      kin.w1 = w1_pre;  kin.w2 = w2_pre;
      kin.aRef = aRef;  kin.wall = wall;
      kin.hCut = lubricationCutoff( aRef );   // activation gap (mesh clamp included);
                                              // consumed by modelKroupaDeficit only

      const PairWrench wr = computeWrench( kin, lubCfg );

      Vec3 J;        // translational impulse on body 1 (unweighted)
      Vec3 L1, L2;   // pure-torque impulses on bodies 1/2 (unweighted)

      if( lubCfg.scheme == schemeSemiImplicit ) {
         // Normal mode: exact exponential update of the relative normal velocity.
         // |Jn| < m_eff |vrn| by construction: unconditionally stable, no cap needed.
         if( vrn < real(0) || lubCfg.resistSeparation ) {
            const real Kn = normalResistance( h, aRef, wall, lubCfg, kin.hCut );
            J = -m_eff * ( real(1) - std::exp( -Kn * dt / m_eff ) ) * vrn * n;
         }

         // Tangential force: explicit, with the exponential clamp of its own mode
         // (soft O(log eps) resistance, clamp ~1 in practice).
         const real Ks = slidingResistance( h, aRef, wall, lubCfg, kin.hCut );
         J += wr.Ft * ( dt * expClamp( Ks * dt / m_eff ) );

         // Pure torques: twist component clamped against its own resistance, the
         // sliding-torque remainder guarded with the heuristic K_s*aRef^2 rotational
         // stiffness. Effective inertia about n from both bodies.
         const real invIn = trans(n) * ( ( b1->getInvInertia() + b2->getInvInertia() ) * n );
         real cTwist = real(1);
         real cSlide = real(1);
         if( invIn > real(0) ) {
            const real I_eff = real(1) / invIn;
            const real Kt = twistingResistance( h, aRef, wall, lubCfg, kin.hCut );
            cTwist = expClamp( Kt * dt / I_eff );
            cSlide = expClamp( Ks * aRef * aRef * dt / I_eff );
         }
         const Vec3 Mt1  = ( trans(wr.M1) * n ) * n;   // twist part of M1
         const Vec3 Msl1 = wr.M1 - Mt1;
         const Vec3 Mt2  = ( trans(wr.M2) * n ) * n;
         const Vec3 Msl2 = wr.M2 - Mt2;
         L1 = ( Msl1 * cSlide + Mt1 * cTwist ) * dt;
         L2 = ( Msl2 * cSlide + Mt2 * cTwist ) * dt;
      }
      else {
         // Explicit force x dt with the legacy impulse cap applied to the normal
         // component (lubricationIntegration_ = "explicit-capped").
         Vec3 FnCapped = wr.Fn;
         const real JnProp = wr.Fn.length() * dt;
         const real Jcap   = ( lubCfg.alphaImpulseCap * blend ) * m_eff * std::fabs( vrn );
         if( JnProp > Jcap && JnProp > real(0) ) {
            FnCapped *= ( Jcap / JnProp );
         }
         J  = ( FnCapped + wr.Ft ) * dt;
         L1 = wr.M1 * dt;
         L2 = wr.M2 * dt;
      }

      // Blend-weight scaling (single multiplication of all impulses), then apply as
      // velocity corrections. Lever-arm torques r x J complement the pure torques.
      const Vec3 Jw  = blend * J;
      const Vec3 L1w = blend * L1;
      const Vec3 L2w = blend * L2;

      dv[i1] += invm1 * Jw;
      dw[i1] += b1->getInvInertia() * ( r1 % Jw + L1w );
      dv[i2] -= invm2 * Jw;
      dw[i2] += b2->getInvInertia() * ( r2 % ( -Jw ) + L2w );

      // Diagnostics
      ++diag.numContacts;
      if( h < lubCfg.epsCritical * aRef ) ++diag.numSaturated;
      const real JnMag = std::fabs( trans(Jw) * n );
      diag.maxNormalImpulse = std::max( diag.maxNormalImpulse, JnMag );
      diag.dissipation += trans(Jw) * gdot + trans(L1w) * w1_pre + trans(L2w) * w2_pre;
      diag.totalForce  += Jw.length() / dt;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "         Model           = kroupa2016 ("
             << ( lubCfg.scheme == schemeSemiImplicit ? "semi-implicit" : "explicit-capped" ) << ")\n"
             << "         Impulse J       = " << Jw << "\n"
             << "         Torque L1       = " << L1w << "\n"
             << "         Status: APPLIED\n";
      }
   }

   // Log summary for plotting and model health: dissipation must be <= 0 (the model
   // is purely resistive); a positive value indicates a sign-convention bug.
   pe_LOG_DEBUG_SECTION( log ) {
      log << "[LUBRICATION_SUMMARY] Step: " << TimeStep::step()
          << ", Contacts: " << diag.numContacts
          << ", Saturated: " << diag.numSaturated
          << ", TotalForce: " << diag.totalForce
          << ", MaxNormalImpulse: " << diag.maxNormalImpulse
          << ", Dissipation: " << diag.dissipation << "\n";
      if( diag.dissipation > real(0) ) {
         log << "[LUBRICATION_WARNING] Positive lubrication dissipation detected ("
             << diag.dissipation << ") - sign-convention bug?\n";
      }
   }

   return diag;
}
//*************************************************************************************************

} // namespace lubrication
} // namespace pe

#endif
