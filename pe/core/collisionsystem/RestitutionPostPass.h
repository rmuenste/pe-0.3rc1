//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/RestitutionPostPass.h
 *  \brief Newton/Poisson restitution of the hard-contact solvers: the capped post-pass sweep,
 *         the pre-step normal velocity of a contact and the diagnostics of the pass.
 *
 *  Shared by the two hard-contact collision systems that bounce impacting contacts:
 *
 *   - HardContactSemiImplicitTimesteppingSolvers (HCSITS), where the scheme was introduced, and
 *   - HardContactEulerLagrange (the Euler-Lagrange solver, port of the HCSITS scheme).
 *
 *  Both solvers keep their own switches (setRestitution(), setRestitutionThreshold(),
 *  setRestitutionPostPass(), setRestitutionIterations()) and their own arming step (which contact
 *  asks for a bounce, see the setRestitution() documentation of each solver: HCSITS arms in the
 *  contact caching of resolveContacts() with the threshold max( vn, 2 |g| dt ); the EL solver arms
 *  after its body caching with the threshold max( vn, 2 |dv_free| ), because gravity is applied by
 *  the CFD driver there). Everything after the arming is this file, so the impact law exists once:
 *
 *   - restitutionPreStepNormalVelocity(): the pre-impact normal velocity of a contact (the
 *     velocities stored in the bodies at the start of the step);
 *   - restitutionPostPassSweep(): one Gauss-Seidel sweep of the capped post-pass, called
 *     setRestitutionIterations() times after the inelastic velocity solve, each sweep followed by
 *     the solver's synchronizeVelocities().
 *
 *  restitutionPostPassSweep() works on the contact and body arrays of the solver it is
 *  instantiated with (both solvers declare it a friend). It reads p_, body1_/body2_ (with their
 *  index_), r1_/r2_, n_/t_/o_, surfaceVel_, dist_, mu_, diag_n_inv_, diag_to_inv_, cor_,
 *  impactSpeed_, v_/w_, relaxationModel_, and writes p_, dv_/dw_, restitutionImpulse_,
 *  restitutionPassStats_. A solver that wants the pass needs these members with the meaning they
 *  have in HCSITS (p_ = accumulated contact impulse on body 1, n_ from body 2 towards body 1,
 *  dist_ * dtinv = the normal velocity a separated contact may still approach with).
 */
//=================================================================================================

#ifndef _PE_CORE_COLLISIONSYSTEM_RESTITUTIONPOSTPASS_H_
#define _PE_CORE_COLLISIONSYSTEM_RESTITUTIONPOSTPASS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <pe/math/Matrix2x2.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector2.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//*************************************************************************************************
/*!\brief Diagnostics of the restitution post-pass of the last resolveContacts() call.
 * \ingroup collision_system
 *
 * Filled only while setRestitution() and setRestitutionPostPass() are on; all zero otherwise.
 * Counts are summed over the sweeps of one time step and are local to the process.
 */
struct RestitutionPassStats
{
   size_t armed;             //!< Armed contact visits (contacts asked to bounce, summed over the sweeps).
   size_t capped;            //!< Visits where the Poisson cap e * lambda_c cut the increment.
   size_t clamped;           //!< Visits where the accumulated restitution impulse was clamped at zero.
   real   clampedImpulse;    //!< Impulse the clamp removed from the restitution bookkeeping (moved to the compression part).
   real   restitutionImpulse;//!< Total restitution impulse delivered by the pass.
   RestitutionPassStats() : armed( 0 ), capped( 0 ), clamped( 0 ), clampedImpulse( 0 ), restitutionImpulse( 0 ) {}
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Pre-step normal relative velocity of a contact.
 * \ingroup collision_system
 *
 * \param b1 The first body of the contact.
 * \param b2 The second body of the contact.
 * \param r1 Contact point relative to the centre of body 1.
 * \param r2 Contact point relative to the centre of body 2.
 * \param n The contact normal (from body 2 towards body 1, normalized).
 * \return The normal component of the relative contact velocity; negative = approaching.
 *
 * Uses the velocities stored in the bodies, i.e. those at the start of the step, before the
 * solver adds the step's external forces (gravity in HCSITS, the driver's hydrodynamic increment
 * in the EL solver) to its cached velocities. With the cached velocities a resting contact would
 * look like an impact of |g| dt.
 */
template< typename BodyID >
inline real restitutionPreStepNormalVelocity( BodyID b1, BodyID b2, const Vec3& r1, const Vec3& r2, const Vec3& n )
{
   const Vec3 gdotPre( b1->getLinearVel() - b2->getLinearVel()
                       + b1->getAngularVel() % r1 - b2->getAngularVel() % r2 );
   return trans( n ) * gdotPre;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief One Gauss-Seidel sweep of the capped restitution post-pass (setRestitutionPostPass()).
 * \ingroup collision_system
 *
 * \param s The collision system whose contacts are swept (HCSITS or the EL solver).
 * \param dtinv The inverse of the time step size.
 * \return The largest normal impulse increment of the sweep.
 *
 * Visits the contacts whose material pair has a coefficient of restitution \f$ e > 0 \f$ and
 * updates their normal impulse incrementally (the contact's own impulse stays applied, only the
 * increment moves the velocities). For an armed contact (impactSpeed_ > 0 and a positive
 * compression impulse \f$ \lambda_c = \lambda - \lambda_r \f$) the target is the separation
 * speed \f$ e\,|\dot g_n^-| \f$ and the increment is limited to
 * \f$ \Delta\lambda \le \Delta\lambda_{appr} + [\,e\,(\lambda_c + \Delta\lambda_{appr}) - \lambda_r\,]_+ \f$
 * with \f$ \Delta\lambda_{appr} = \min( [-m_{eff}\,\dot g_n]_+, [\Delta\lambda]_+ ) \f$ the part
 * that only stops an approach. Other contacts of the pair keep the inelastic target
 * (-dist_/dt). After a normal change the Coulomb friction of the contact is re-relaxed in the
 * decoupled manner of relaxApproximateInelasticCoulombContactsByDecoupling() against the new
 * total normal impulse, unless the relaxation model is frictionless.
 *
 * Moved here verbatim from HardContactSemiImplicitTimesteppingSolvers::relaxRestitutionPostPass()
 * (which, like the EL solver's relaxRestitutionPostPass(), now forwards to it).
 */
template< typename CS >
real restitutionPostPassSweep( CS& s, real dtinv )
{
   real delta_max( 0 );
   const size_t numContactsMasked( s.p_.size() );
   const bool withFriction( s.relaxationModel_ != CS::InelasticFrictionlessContact );

   for( size_t i = 0; i < numContactsMasked; ++i ) {
      if( s.cor_[i] <= real( 0 ) )
         continue;

      const size_t i1( s.body1_[i]->index_ ), i2( s.body2_[i]->index_ );

      // Relative contact velocity with every impulse applied, including this contact's own.
      Vec3 gdot( ( s.v_[i1] + s.dv_[i1] ) - ( s.v_[i2] + s.dv_[i2] ) + ( s.w_[i1] + s.dw_[i1] ) % s.r1_[i] - ( s.w_[i2] + s.dw_[i2] ) % s.r2_[i] );
      const real gdot_n( trans( s.n_[i] ) * gdot );

      const real lambda ( trans( s.n_[i] ) * s.p_[i] );          // total normal impulse so far
      const real lambdaC( lambda - s.restitutionImpulse_[i] );  // its compression part
      const bool armed( s.impactSpeed_[i] > real( 0 ) && lambdaC > real( 0 ) );

      const real target( armed ? s.impactSpeed_[i] : -s.dist_[i] * dtinv );
      real d( s.diag_n_inv_[i] * ( target - gdot_n ) );
      d = std::max( lambda + d, real( 0 ) ) - lambda;

      // The part of the increment that only stops an approach is always allowed; the separating
      // remainder is restitution and is held to e times the compression impulse.
      const real approach( std::min( std::max( -s.diag_n_inv_[i] * gdot_n, real( 0 ) ), std::max( d, real( 0 ) ) ) );
      if( armed ) {
         ++s.restitutionPassStats_.armed;
         const real allowance( s.cor_[i] * ( lambdaC + approach ) - s.restitutionImpulse_[i] );
         const real dCapped( std::min( d, approach + std::max( allowance, real( 0 ) ) ) );
         if( dCapped < d ) ++s.restitutionPassStats_.capped;
         d = dCapped;
      }
      // Clamped at zero: a negative increment (the contact already separates faster than its
      // target, pushed by a neighbour) reduces the compression part once the restitution part is
      // used up, which keeps the cap e * lambda_c at or below Box3D's bookkeeping. Only contacts
      // that can arm (impactSpeed_ > 0) need the bookkeeping; the others have no cap.
      if( s.impactSpeed_[i] > real( 0 ) ) {
         const real lambdaR( s.restitutionImpulse_[i] + d - approach );
         if( lambdaR < real( 0 ) ) {
            ++s.restitutionPassStats_.clamped;
            s.restitutionPassStats_.clampedImpulse -= lambdaR;
         }
         s.restitutionImpulse_[i] = std::max( lambdaR, real( 0 ) );
         s.restitutionPassStats_.restitutionImpulse += std::max( d - approach, real( 0 ) );
      }

      if( d == real( 0 ) )
         continue;

      delta_max = std::max( delta_max, std::abs( d ) );
      Vec3 dp( s.n_[i] * d );

      // Friction against the new total normal impulse: sticking increment, projected onto the
      // Coulomb disc (decoupled from the normal component like the dynamic branch of the
      // approximate decoupling model).
      const real lambdaNew( lambda + d );
      if( withFriction && s.mu_[i] > real( 0 ) && lambdaNew > real( 0 ) ) {
         const Vec3 gdot2( gdot + s.surfaceVel_[i] + ( s.body1_[i]->getInvMass() + s.body2_[i]->getInvMass() ) * dp
                           + ( s.body1_[i]->getInvInertia() * ( s.r1_[i] % dp ) ) % s.r1_[i]
                           + ( s.body2_[i]->getInvInertia() * ( s.r2_[i] % dp ) ) % s.r2_[i] );
         Vec2 gdot_to;
         gdot_to[0] = trans( s.t_[i] ) * gdot2;
         gdot_to[1] = trans( s.o_[i] ) * gdot2;
         Vec2 pt;
         pt[0] = trans( s.t_[i] ) * s.p_[i];
         pt[1] = trans( s.o_[i] ) * s.p_[i];
         Vec2 ptNew( pt - ( s.diag_to_inv_[i] * gdot_to ) );
         const real flimit( s.mu_[i] * lambdaNew );
         const real fsq( ptNew[0] * ptNew[0] + ptNew[1] * ptNew[1] );
         if( fsq > flimit * flimit ) {
            const real f( flimit / std::sqrt( fsq ) );
            ptNew[0] *= f;
            ptNew[1] *= f;
         }
         dp += s.t_[i] * ( ptNew[0] - pt[0] ) + s.o_[i] * ( ptNew[1] - pt[1] );
      }

      s.p_[i] += dp;
      s.dv_[i1] += s.body1_[i]->getInvMass() * dp;
      s.dw_[i1] += s.body1_[i]->getInvInertia() * ( s.r1_[i] % dp );
      s.dv_[i2] -= s.body2_[i]->getInvMass() * dp;
      s.dw_[i2] -= s.body2_[i]->getInvInertia() * ( s.r2_[i] % dp );
   }

   return delta_max;
}
//*************************************************************************************************

} // namespace pe

#endif
