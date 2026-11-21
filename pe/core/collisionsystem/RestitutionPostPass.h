//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/RestitutionPostPass.h
 *  \brief Newton/Poisson restitution of the hard-contact solvers: the capped post-pass, its stop
 *         rule and diagnostics, the pre-step normal velocity and the in-solve gap rule.
 *
 *  Shared by the two hard-contact collision systems that bounce impacting contacts:
 *
 *   - HardContactSemiImplicitTimesteppingSolvers (HCSITS), where the scheme was introduced, and
 *   - HardContactEulerLagrange (the Euler-Lagrange solver, port of the HCSITS scheme).
 *
 *  Both solvers keep their own switches (setRestitution() and the setters listed in
 *  RestitutionStopRule) and their own arming step (which contact asks for a bounce: HCSITS arms in
 *  the contact caching of resolveContacts() with the threshold max( vn, 2 |g| dt ); the EL solver
 *  arms after its body caching with max( vn, 2 |dv_free| ), because the CFD driver applies
 *  gravity there). Everything after the arming is this file, so the impact law exists once:
 *
 *   - restitutionPreStepNormalVelocity(): the pre-impact normal velocity of a contact;
 *   - restitutionInSolveDistance(): the in-solve Newton target with the positive-gap rule;
 *   - restitutionPostPass(): the capped post-pass of one step (classification of the rows, the
 *     Gauss-Seidel sweeps restitutionPostPassSweep(), the stop rule on the synchronised state,
 *     the diagnostics).
 *
 *  The pass works on the contact and body arrays of the solver it is instantiated with (both
 *  solvers declare the templates friends). It reads p_, body1_/body2_ (with their index_),
 *  r1_/r2_, n_/t_/o_, surfaceVel_, dist_, mu_, diag_n_inv_, diag_to_inv_, cor_, impactSpeed_,
 *  v_/w_, relaxationModel_, restitutionStopRule_, and writes p_, dv_/dw_, restitutionImpulse_,
 *  restitutionWork_, restitutionPassStats_, and calls synchronizeVelocities() and
 *  exchangeRestitutionBodyScales() (owner <-> shadow maximum of a per-body value, collective; a
 *  no-op on one process). Bodies must provide index_, getInvMass(), getInvInertia(), isFixed(),
 *  getID(), getSystemID(). A solver that wants
 *  the pass needs these members with the meaning they have in HCSITS (p_ = accumulated contact
 *  impulse on body 1, n_ from body 2 towards body 1, dist_ * dtinv = the normal velocity a
 *  separated contact may still approach with).
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
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPITrait.h>
#include <pe/core/MPITag.h>
#include <pe/core/Marshalling.h>
#include <pe/core/notifications/NotificationType.h>
#include <pe/core/notifications/RigidBodyVelocityCorrectionNotification.h>
#include <pe/math/Matrix2x2.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector2.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//*************************************************************************************************
/*!\brief Stop rule and budget of the restitution post-pass (both solvers, collective).
 * \ingroup collision_system
 *
 * The pass runs at least minSweeps and at most max( minSweeps, maxSweeps ) sweeps. From sweep
 * minSweeps on, and then every checkInterval sweeps (and at the last one), it evaluates two
 * residuals of every affected row on the SYNCHRONISED velocities, both in velocity units: the
 * projected normal residual (the admissible normal impulse increment after the unilateral bound
 * and the Poisson cap, divided by the effective normal mass) and the projected tangential
 * residual (the tangential contact-point velocity change the Coulomb projection would still
 * produce, through the full contact response of the two bodies). It stops when both are within
 * absTolerance + relTolerance * s for every row, s the row's local impact scale (the largest
 * impact speed of an armed row on either of its mobile bodies, the same on every process that
 * holds the body; 0 away from impacts; fixed bodies such as walls carry no scale). The values
 * must be the same on every process: the decisions are global (MPI_MAX of the violations), but
 * each process compares its rows with its own values.
 * Defaults (review round 2): min 4, max 24, check every 4, 1e-4 (velocity units, m/s in the EL
 * decks) + 1e-3. Strict settings (e.g. max 100, abs 0, rel 1e-9) are for regression tests.
 */
struct RestitutionStopRule
{
   size_t minSweeps;      //!< setRestitutionIterations(): minimum number of sweeps (>= 1).
   size_t maxSweeps;      //!< setRestitutionMaxIterations(): budget (>= 1).
   size_t checkInterval;  //!< setRestitutionCheckInterval(): sweeps between two stop tests (>= 1).
   real   absTolerance;   //!< setRestitutionTolerance(): absolute part, velocity units (>= 0).
   real   relTolerance;   //!< setRestitutionTolerance(): part relative to the local impact scale (>= 0).
   RestitutionStopRule() : minSweeps( 4 ), maxSweeps( 24 ), checkInterval( 4 ), absTolerance( 1e-4 ), relTolerance( 1e-3 ) {}
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Diagnostics of the restitution of the last resolveContacts() call.
 * \ingroup collision_system
 *
 * Reset at the start of every step while setRestitution() is on. Counts and impulse sums are
 * local to the process; the velocity measures (impactMax ... restingApproachMax, except the two
 * approach maxima) are maxima over all processes.
 */
struct RestitutionPassStats
{
   // arming (both schemes)
   size_t candidates;        //!< Contacts approaching faster than the threshold (impactSpeed_ > 0, or an in-solve target).
   size_t inSolveTargets;    //!< In-solve scheme: contacts that got the Newton target in the solve.
   real   approachMax;       //!< Largest pre-step approach speed -vn of a contact with e > 0 (local).
   real   restingApproachMax;//!< Largest pre-step approach speed of a contact with e > 0 left inelastic by the threshold (local).
   // post-pass rows
   size_t armedRows;         //!< Candidates with a positive compression impulse after the inelastic solve: the rows that may bounce.
   size_t delivered;         //!< Armed rows that end the pass with a positive booked restitution impulse (rebounds delivered).
   size_t separating;        //!< Rows armed at the classification that end the pass separating (normal velocity > 0).
   size_t demoted;           //!< Armed rows whose compression impulse was withdrawn to zero: support rows for the rest of the pass.
   real   fallbackImpulse;   //!< Support impulse (positive increments) of the demoted rows; never booked as restitution.
   size_t supportRows;       //!< All other rows.
   size_t armed;             //!< Armed row visits (summed over the sweeps).
   size_t capped;            //!< Visits where the Poisson cap e * lambda_c cut the increment.
   size_t clamped;           //!< Visits where the accumulated restitution impulse was clamped at zero.
   real   clampedImpulse;    //!< Impulse the clamp removed from the restitution bookkeeping.
   real   restitutionImpulse;//!< Gross: sum of the separating increments booked as restitution (Newton visits of armed rows, before withdrawals and relaxation).
   real   finalRestitutionImpulse; //!< Final: sum of the booked restitution impulses of the armed rows at the end of the pass.
   // stop rule
   size_t sweeps;            //!< Sweeps run (0: skipped).
   size_t violations;        //!< Rows outside the tolerance at the last stop test, summed over the processes.
   int    stop;              //!< 0: no pass (scheme off or in-solve), 1: skipped (no armed row anywhere), 2: converged, 3: budget exhausted.
   real   impactMax;         //!< Largest impact speed e |vn| of an armed row over all processes.
   real   residual;          //!< Projected normal residual at the last stop test (velocity), max over processes.
   real   residualRel;       //!< Same, relative to the row's local impact scale (rows with a scale only).
   real   tangentialResidual;//!< Projected tangential residual of the affected rows at the last stop test (contact-point velocity), max over processes.
   real   supportScaleMax;   //!< Largest local impact scale of a support row (stop rule), max over processes.
   real   supportClosingBefore; //!< Largest closing beyond the allowed approach, max( -dist/dt - gdot_n, 0 ), of a support row after the inelastic solve.
   real   supportClosingAfter;  //!< The same after the pass.
   real   impactError;       //!< Largest |e|vn| - gdot_n| of an armed row the cap does not limit, after the pass.
   real   newtonShortfall;   //!< Largest e|vn| - gdot_n of an armed row the cap limits, after the pass.
   RestitutionPassStats()
      : candidates( 0 ), inSolveTargets( 0 ), approachMax( 0 ), restingApproachMax( 0 ), armedRows( 0 ), delivered( 0 )
      , separating( 0 ), demoted( 0 ), fallbackImpulse( 0 )
      , supportRows( 0 ), armed( 0 ), capped( 0 ), clamped( 0 ), clampedImpulse( 0 ), restitutionImpulse( 0 )
      , finalRestitutionImpulse( 0 ), sweeps( 0 ), violations( 0 ), stop( 0 ), impactMax( 0 ), residual( 0 ), residualRel( 0 )
      , tangentialResidual( 0 ), supportScaleMax( 0 ), supportClosingBefore( 0 ), supportClosingAfter( 0 ), impactError( 0 )
      , newtonShortfall( 0 ) {}
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Per-row state of the post-pass, frozen after the inelastic solve (restitutionPostPass()).
 * \ingroup collision_system
 */
struct RestitutionPassWork
{
   //! freeRow / loadedRow: support rows; armedRow: may bounce; demotedRow: an armed row whose
   //! compression impulse was withdrawn to zero, a support row (target g0) for the rest of the pass.
   enum Kind : unsigned char { freeRow = 0, loadedRow = 1, armedRow = 2, demotedRow = 3 };
   std::vector<Vec3> gdot0;        //!< Relative contact velocity after the synchronised inelastic solve.
   std::vector<real> target;       //!< Inelastic target of the row: g0 (loaded, armed, demoted) or min( g0, -dist/dt ) (free).
   std::vector<real> scale;        //!< Local impact scale of the row (stop rule).
   std::vector<unsigned char> kind;
   std::vector<unsigned char> affected; //!< 1 once a sweep visited the row: it stays affected for the rest of the pass.
   std::vector<real> lambda;       //!< Normal impulse of the row as a scalar ledger: n . p_ at the classification, then
                                   //!< changed by exactly the normal increments of the pass (0 after a release). Every
                                   //!< state decision of the pass (Newton, ceiling, friction disc, release) reads it, not
                                   //!< n . p_, whose roundoff for an oblique normal can leave a spurious load.
   std::vector<real> bodyScale;    //!< Largest impact speed of an armed row on the body (by index_; mobile bodies only).
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Counts an arming decision into the stats (both solvers, both schemes).
 * \ingroup collision_system
 *
 * \param stats The pass statistics.
 * \param vn The pre-step normal velocity of a contact with e > 0.
 * \param threshold The contact's effective threshold.
 * \return True if the contact is a candidate (vn < -threshold).
 */
inline bool restitutionCountCandidate( RestitutionPassStats& stats, real vn, real threshold )
{
   const bool candidate( vn < -threshold );
   if( candidate ) { ++stats.candidates; stats.approachMax = std::max( stats.approachMax, -vn ); }
   else if( vn < real( 0 ) ) stats.restingApproachMax = std::max( stats.restingApproachMax, -vn );
   return candidate;
}
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
/*!\brief Target distance of an armed contact for the in-solve Newton target.
 * \ingroup collision_system
 *
 * \param dist The contact's target distance as the compression solve has it (dist_: the gap of a
 *        separated contact, erp times the overlap of a penetrating one, 0 with split impulse).
 * \param e The pair's coefficient of restitution.
 * \param vn The pre-step normal velocity (< 0, the contact is armed).
 * \param vnClose The normal velocity that decides whether the step closes a positive gap: the
 *        free relative velocity of the step (both solvers evaluate it after their body caching).
 * \param dt The time step size.
 * \return e vn dt (the relaxation then aims at -e vn), or dist unchanged when the contact has a
 *         positive gap that the free motion of this step does not close (fine detection accepts
 *         gaps below contactThreshold, and such a contact must not bounce before it touches).
 */
inline bool restitutionInSolveKeepsGap( real dist, real vnClose, real dt )
{
   return dist > real( 0 ) && dist + vnClose * dt > real( 0 );
}

inline real restitutionInSolveDistance( real dist, real e, real vn, real vnClose, real dt )
{
   if( restitutionInSolveKeepsGap( dist, vnClose, dt ) )
      return dist;
   return e * vn * dt;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sum of a count over all processes.
 * \ingroup collision_system
 */
inline void restitutionGlobalSum( size_t& x )
{
#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      unsigned long long v( x );
      MPI_Allreduce( MPI_IN_PLACE, &v, 1, MPI_UNSIGNED_LONG_LONG, MPI_SUM, MPISettings::comm() );
      x = static_cast<size_t>( v );
   }
#else
   (void)x;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Element-wise largest values of \a x[0..n) over all processes.
 * \ingroup collision_system
 */
inline void restitutionGlobalMax( real* x, int n )
{
#if HAVE_MPI
   if( MPISettings::size() > 1 )
      MPI_Allreduce( MPI_IN_PLACE, x, n, MPITrait<real>::getType(), MPI_MAX, MPISettings::comm() );
#else
   (void)x; (void)n;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Debug traces of the post-pass (environment, read on the root process and broadcast).
 * \ingroup collision_system
 *
 *  - PE_RESTITUTION_TRACE="first:count": the passes first .. first + count - 1 (passes that run,
 *    counted from 0, the same on every process) print one line per sweep on the root process (one
 *    extra reduction per sweep): EL_RESTITUTION_TRACE pass= .. sweep= .. residual= ..
 *    residual_rel= .. tangential= .. violations_rank_max= .. (the largest number of rows outside
 *    the tolerance on one process).
 *  - PE_RESTITUTION_PAIRS="vmin" (velocity units): every step, every process prints its rows with
 *    e > 0 whose pre-step approach is faster than vmin (EL_RESTITUTION_PAIR: user and system ids,
 *    target distance, pre-step normal velocity, impact speed, normal impulse after the inelastic
 *    solve), and after the pass every armed row (EL_RESTITUTION_EVENT: the same with the final
 *    normal impulse, the booked restitution impulse and the final normal velocity). The step
 *    counter counts the calls of RestitutionPass::run().
 *
 * The root process reads the variables at the first call of RestitutionPass::run() and
 * broadcasts them on the PE communicator, so a variable set on the root process only is enough
 * and every process takes the same collective branches. A debug aid, not a diagnostic stream.
 */
struct RestitutionTrace
{
   long   first, count;  //!< PE_RESTITUTION_TRACE range (first < 0: off).
   real   pairSpeed;     //!< PE_RESTITUTION_PAIRS (< 0: off).
   size_t step;          //!< Calls of RestitutionPass::run().
   bool   parsed;
   RestitutionTrace() : first( -1 ), count( 0 ), pairSpeed( -1 ), step( 0 ), parsed( false ) {}

   static RestitutionTrace& instance() { static RestitutionTrace t; return t; }

   //! Collective at the first call (every process calls it at the start of every run()).
   void parse()
   {
      if( parsed ) return;
      parsed = true;
      double v[3] = { -1, 0, -1 };
      if( MPISettings::rank() == MPISettings::root() ) {
         const char* env( std::getenv( "PE_RESTITUTION_TRACE" ) );
         long a( 0 ), b( 0 );
         if( env && std::sscanf( env, "%ld:%ld", &a, &b ) == 2 && a >= 0 && b > 0 ) { v[0] = double( a ); v[1] = double( b ); }
         const char* pairs( std::getenv( "PE_RESTITUTION_PAIRS" ) );
         double vmin( 0 );
         if( pairs && std::sscanf( pairs, "%lf", &vmin ) == 1 && vmin >= 0 ) v[2] = vmin;
      }
#if HAVE_MPI
      if( MPISettings::size() > 1 )
         MPI_Bcast( v, 3, MPI_DOUBLE, MPISettings::root(), MPISettings::comm() );
#endif
      first = static_cast<long>( v[0] ); count = static_cast<long>( v[1] ); pairSpeed = real( v[2] );
   }

   bool active( size_t pass ) const
   {
      return first >= 0 && static_cast<long>( pass ) >= first && static_cast<long>( pass ) < first + count;
   }
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The capped restitution post-pass, shared by HCSITS and the EL solver.
 * \ingroup collision_system
 *
 * A collection of static member templates; the solvers declare the struct a friend (one line,
 * <tt>friend struct RestitutionPass;</tt>) and call RestitutionPass::run( *this, dtinv ) after
 * their synchronised inelastic velocity solve.
 */
struct RestitutionPass
{
//*************************************************************************************************
//! Relative contact velocity of row i from the current v_ + dv_, w_ + dw_ (no surface velocity).
template< typename CS >
   static inline Vec3 restitutionRowVelocity( const CS& s, size_t i )
{
   const size_t i1( s.body1_[i]->index_ ), i2( s.body2_[i]->index_ );
   return ( s.v_[i1] + s.dv_[i1] ) - ( s.v_[i2] + s.dv_[i2] ) + ( s.w_[i1] + s.dw_[i1] ) % s.r1_[i] - ( s.w_[i2] + s.dw_[i2] ) % s.r2_[i];
}

//! Admissible normal increment of row i at relative velocity gdot (what a visit would apply),
//! with the unilateral bound and, for an armed row, the Poisson cap; also says whether the row is
//! being driven to its Newton target, whether the cap cut the increment, and whether the
//! unilateral projection withdraws the whole normal load (release: d = -lambda; the sweep then
//! removes the stored impulse vector exactly and demotes an armed row on that visit).
template< typename CS >
   static inline real restitutionRowIncrement( const CS& s, size_t i, real gdot_n, bool& newton, bool& cut, real& approach, bool& release )
{
   const RestitutionPassWork& wk( s.restitutionWork_ );
   const real lambda ( wk.lambda[i] );
   const real lambdaC( lambda - s.restitutionImpulse_[i] );
   newton = wk.kind[i] == RestitutionPassWork::armedRow && lambdaC > real( 0 );
   const real target( newton ? s.impactSpeed_[i] : wk.target[i] );
   real d( s.diag_n_inv_[i] * ( target - gdot_n ) );
   release = lambda > real( 0 ) && !( lambda + d > real( 0 ) );
   d = release ? -lambda : std::max( lambda + d, real( 0 ) ) - lambda;
   approach = std::min( std::max( -s.diag_n_inv_[i] * gdot_n, real( 0 ) ), std::max( d, real( 0 ) ) );
   cut = false;
   if( newton && !release ) {
      const real allowance( s.cor_[i] * ( lambdaC + approach ) - s.restitutionImpulse_[i] );
      const real dCapped( std::min( d, approach + std::max( allowance, real( 0 ) ) ) );
      cut = dCapped < d;
      d = dCapped;
   }
   return d;
}

//! Tangential (Coulomb, decoupled) increment of row i for the normal increment d at relative
//! velocity gdot: the sticking increment projected onto the disc mu ( lambda + d ), or the removal
//! of the whole tangential impulse when the row unloads.
template< typename CS >
   static inline Vec3 restitutionRowFriction( const CS& s, size_t i, const Vec3& gdot, real d, bool withFriction )
{
   const real lambdaNew( s.restitutionWork_.lambda[i] + d );
   if( !( lambdaNew > real( 0 ) ) )
      return -( s.t_[i] * ( trans( s.t_[i] ) * s.p_[i] ) + s.o_[i] * ( trans( s.o_[i] ) * s.p_[i] ) );
   if( !withFriction || !( s.mu_[i] > real( 0 ) ) )
      return Vec3();
   const Vec3 dp( s.n_[i] * d );
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
   return s.t_[i] * ( ptNew[0] - pt[0] ) + s.o_[i] * ( ptNew[1] - pt[1] );
}

//! A support row that no sweep has visited yet and whose relative velocity is still the
//! post-compression one is undisturbed: the pass leaves it alone (exact fixed point: no impulse,
//! no friction change, no positional repair). Once visited, a row stays affected for the rest of
//! the pass (its impulse may differ from the compression state although its velocity returns to
//! it). The same predicate decides the sweep and the residuals.
template< typename CS >
   static inline bool restitutionRowUndisturbed( const CS& s, size_t i, const Vec3& gdot )
{
   const RestitutionPassWork& wk( s.restitutionWork_ );
   return wk.kind[i] != RestitutionPassWork::armedRow && wk.kind[i] != RestitutionPassWork::demotedRow && !wk.affected[i]
      && gdot[0] == wk.gdot0[i][0] && gdot[1] == wk.gdot0[i][1] && gdot[2] == wk.gdot0[i][2];
}

//! Tangential contact-point velocity change of row i produced by the impulse increment dp
//! (through the full response of both bodies, translation and rotation).
template< typename CS >
   static inline real restitutionTangentialSpeed( const CS& s, size_t i, const Vec3& dp )
{
   const Vec3 dg( ( s.body1_[i]->getInvMass() + s.body2_[i]->getInvMass() ) * dp
                  + ( s.body1_[i]->getInvInertia() * ( s.r1_[i] % dp ) ) % s.r1_[i]
                  + ( s.body2_[i]->getInvInertia() * ( s.r2_[i] % dp ) ) % s.r2_[i] );
   const real a( trans( s.t_[i] ) * dg ), b( trans( s.o_[i] ) * dg );
   return std::sqrt( a * a + b * b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief One Gauss-Seidel sweep of the capped restitution post-pass.
 * \ingroup collision_system
 *
 * \param s The collision system whose contacts are swept (HCSITS or the EL solver).
 * \param dtinv The inverse of the time step size.
 * \return void
 *
 * Rows (classified by restitutionPostPass() after the synchronised inelastic solve):
 *
 *  - armed (a candidate with positive compression impulse): driven to the separation speed
 *    \f$ e\,|\dot g_n^-| \f$ with the separating part of its impulse capped at e times its
 *    compression impulse (Poisson ceiling; the part that only stops an approach is always
 *    allowed); only these rows book restitution impulse;
 *  - support (all other rows, any e): unilateral target RELATIVE TO THE POST-COMPRESSION STATE,
 *    g0 for a loaded row and min( g0, -dist/dt ) for a free one, so that an undisturbed support
 *    network is an exact fixed point and only closing introduced by a rebound is removed; the
 *    positional repair stays in the compression (or split-impulse position) solve. A support row
 *    that no sweep has visited and whose relative velocity is still bitwise the post-compression
 *    one is skipped entirely; once visited it stays affected;
 *  - demoted (an armed row whose compression impulse was withdrawn to zero): a support row with
 *    target g0 for the rest of the pass; its impulse is fallback support impulse, never
 *    restitution (the Poisson ceiling e * lambda_c = 0 allows none).
 *
 * Every visited row also gets its Coulomb friction re-relaxed against the new total normal
 * impulse (decoupled, as relaxApproximateInelasticCoulombContactsByDecoupling()), also when the
 * normal increment is zero (a rebound can change the slip of a loaded row without changing its
 * load), unless the relaxation model is frictionless; a row whose normal impulse drops to zero
 * loses its tangential impulse with it.
 */
template< typename CS >
   static void restitutionPostPassSweep( CS& s, real dtinv )
{
   (void)dtinv;
   const size_t numContactsMasked( s.p_.size() );
   const bool withFriction( s.relaxationModel_ != CS::InelasticFrictionlessContact );
   RestitutionPassWork& wk( s.restitutionWork_ );
   RestitutionPassStats& stats( s.restitutionPassStats_ );

   for( size_t i = 0; i < numContactsMasked; ++i ) {
      const size_t i1( s.body1_[i]->index_ ), i2( s.body2_[i]->index_ );
      const Vec3 gdot( restitutionRowVelocity( s, i ) );
      if( restitutionRowUndisturbed( s, i, gdot ) )
         continue;
      wk.affected[i] = 1;
      const real gdot_n( trans( s.n_[i] ) * gdot );

      bool newton( false ), cut( false ), release( false );
      real approach( 0 );
      const real d( restitutionRowIncrement( s, i, gdot_n, newton, cut, approach, release ) );
      if( newton ) { ++stats.armed; if( cut ) ++stats.capped; }

      // Full withdrawal, decided by the projection on this visit: the stored impulse vector is
      // removed exactly (p_ = 0, both bodies lose exactly the old impulse), the booking is cleared
      // and an armed row is demoted now (also on the last sweep), so roundoff of n . p_ can never
      // leave it armed with a spurious compression allowance.
      if( release ) {
         if( wk.kind[i] == RestitutionPassWork::armedRow ) {
            wk.kind[i] = RestitutionPassWork::demotedRow;
            ++stats.demoted;
         }
         if( s.restitutionImpulse_[i] > real( 0 ) ) { ++stats.clamped; stats.clampedImpulse += s.restitutionImpulse_[i]; }
         s.restitutionImpulse_[i] = real( 0 );
         wk.lambda[i] = real( 0 );
         const Vec3 dp( -s.p_[i] );
         s.p_[i] = Vec3();
         s.dv_[i1] += s.body1_[i]->getInvMass() * dp;
         s.dw_[i1] += s.body1_[i]->getInvInertia() * ( s.r1_[i] % dp );
         s.dv_[i2] -= s.body2_[i]->getInvMass() * dp;
         s.dw_[i2] -= s.body2_[i]->getInvInertia() * ( s.r2_[i] % dp );
         continue;
      }

      // Restitution bookkeeping. A Newton visit of an armed row books the separating remainder of
      // its increment as restitution (the cap keeps lambda_R <= e lambda_C); a negative increment
      // reduces it, clamped at zero. An armed row without compression impulse left (lambda_C <= 0:
      // its load was withdrawn) is demoted to a support row: no restitution can be booked without
      // the ceiling, its further impulse is fallback support impulse.
      if( wk.kind[i] == RestitutionPassWork::armedRow ) {
         if( newton ) {
            const real lambdaR( s.restitutionImpulse_[i] + d - approach );
            if( lambdaR < real( 0 ) ) { ++stats.clamped; stats.clampedImpulse -= lambdaR; }
            s.restitutionImpulse_[i] = std::max( lambdaR, real( 0 ) );
            stats.restitutionImpulse += std::max( d - approach, real( 0 ) );
         }
         else {
            wk.kind[i] = RestitutionPassWork::demotedRow;
            ++stats.demoted;
            if( s.restitutionImpulse_[i] > real( 0 ) ) { ++stats.clamped; stats.clampedImpulse += s.restitutionImpulse_[i]; }
            s.restitutionImpulse_[i] = real( 0 );
         }
      }
      if( wk.kind[i] == RestitutionPassWork::demotedRow )
         stats.fallbackImpulse += std::max( d, real( 0 ) );

      wk.lambda[i] += d;
      const Vec3 dp( s.n_[i] * d + restitutionRowFriction( s, i, gdot, d, withFriction ) );
      if( dp[0] == real( 0 ) && dp[1] == real( 0 ) && dp[2] == real( 0 ) )
         continue;
      s.p_[i] += dp;
      s.dv_[i1] += s.body1_[i]->getInvMass() * dp;
      s.dw_[i1] += s.body1_[i]->getInvInertia() * ( s.r1_[i] % dp );
      s.dv_[i2] -= s.body2_[i]->getInvMass() * dp;
      s.dw_[i2] -= s.body2_[i]->getInvInertia() * ( s.r2_[i] % dp );
   }
}
//*************************************************************************************************


//*************************************************************************************************
//! Residuals of the synchronised state: m[0] projected normal residual (velocity), m[1] its
//! maximum relative to the local impact scale, m[2] projected tangential residual (tangential
//! contact-point velocity change through the contact response), and the number of rows whose
//! normal OR tangential residual is outside absTolerance + relTolerance * scale in m[3] (local).
template< typename CS >
   static inline void restitutionResiduals( const CS& s, real* m )
{
   const bool withFriction( s.relaxationModel_ != CS::InelasticFrictionlessContact );
   const RestitutionPassWork& wk( s.restitutionWork_ );
   m[0] = m[1] = m[2] = m[3] = 0;
   for( size_t i = 0; i < s.p_.size(); ++i ) {
      const Vec3 gdot( restitutionRowVelocity( s, i ) );
      if( restitutionRowUndisturbed( s, i, gdot ) ) continue;
      bool newton( false ), cut( false ), release( false );
      real approach( 0 );
      const real d( restitutionRowIncrement( s, i, trans( s.n_[i] ) * gdot, newton, cut, approach, release ) );
      const real rn( std::abs( d ) / s.diag_n_inv_[i] );
      const Vec3 dpt( restitutionRowFriction( s, i, gdot, d, withFriction ) );
      const real rt( restitutionTangentialSpeed( s, i, dpt ) );
      m[0] = std::max( m[0], rn );
      m[2] = std::max( m[2], rt );
      if( wk.scale[i] > real( 0 ) ) m[1] = std::max( m[1], rn / wk.scale[i] );
      const real tol( s.restitutionStopRule_.absTolerance + s.restitutionStopRule_.relTolerance * wk.scale[i] );
      if( rn > tol || rt > tol ) m[3] += real( 1 );
   }
}

//! Largest closing beyond the allowed approach -dist/dt of a support row (velocity).
template< typename CS >
   static inline real restitutionSupportClosing( const CS& s, real dtinv )
{
   real c( 0 );
   for( size_t i = 0; i < s.p_.size(); ++i ) {
      if( s.restitutionWork_.kind[i] == RestitutionPassWork::armedRow || s.restitutionWork_.kind[i] == RestitutionPassWork::demotedRow ) continue;
      c = std::max( c, -s.dist_[i] * dtinv - trans( s.n_[i] ) * restitutionRowVelocity( s, i ) );
   }
   return c;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The capped restitution post-pass of one time step (setRestitutionPostPass()).
 * \ingroup collision_system
 *
 * \param s The collision system (HCSITS or the EL solver), after its synchronised inelastic solve.
 * \param dtinv The inverse of the time step size.
 * \return void
 *
 *  1. Classification, from the synchronised post-compression velocities: armed rows (candidates,
 *     impactSpeed_ > 0, with a positive compression impulse), loaded and free support rows; the
 *     post-compression relative velocity and target of every row.
 *  2. If no row is armed on any process the pass ends: no sweep, the step is bit for bit the
 *     inelastic step (stats.stop = 1). Else the per-body impact scales are made equal on the
 *     owner and every shadow copy (exchangeRestitutionBodyScales(), maximum) and the row scales
 *     assigned.
 *  3. Sweeps (restitutionPostPassSweep() + synchronizeVelocities()), with the stop test of
 *     RestitutionStopRule on the synchronised state (stats.stop = 2 converged, 3 budget).
 *  4. Diagnostics: support closing before and after, impact error, Newton shortfall, residuals,
 *     delivered rebounds, gross and final restitution impulse.
 *
 * Collective: every process calls it in the same step and performs the same reductions (one at
 * the classification, one per stop test, two at the end), the scale exchange (only when the pass
 * runs) and synchronisations.
 */
template< typename CS >
   static void restitutionPostPass( CS& s, real dtinv )
{
   static size_t passCounter( 0 );   // debug trace only: counts the passes that run, the same on every process

   RestitutionPassStats& stats( s.restitutionPassStats_ );
   RestitutionPassWork& wk( s.restitutionWork_ );
   const RestitutionStopRule& rule( s.restitutionStopRule_ );
   const size_t n( s.p_.size() );

   // 1. classification
   wk.gdot0.resize( n ); wk.target.resize( n ); wk.scale.assign( n, real( 0 ) ); wk.kind.resize( n ); wk.affected.assign( n, 0 ); wk.lambda.resize( n );
   wk.bodyScale.assign( s.v_.size(), real( 0 ) );
   real impactMax( 0 ), closingBefore( 0 );
   for( size_t i = 0; i < n; ++i ) {
      wk.gdot0[i] = restitutionRowVelocity( s, i );
      const real g0( trans( s.n_[i] ) * wk.gdot0[i] );
      const real lambda0( trans( s.n_[i] ) * s.p_[i] );
      wk.lambda[i] = lambda0;
      s.restitutionImpulse_[i] = real( 0 );
      if( s.impactSpeed_[i] > real( 0 ) && lambda0 > real( 0 ) ) {
         wk.kind[i] = RestitutionPassWork::armedRow;
         wk.target[i] = g0;   // fallback once its compression part is used up
         ++stats.armedRows;
         impactMax = std::max( impactMax, s.impactSpeed_[i] );
         // fixed bodies (walls) carry no scale: an impact on a wall does not loosen the tolerance
         // of unrelated rows on the same wall
         const size_t i1( s.body1_[i]->index_ ), i2( s.body2_[i]->index_ );
         if( !s.body1_[i]->isFixed() ) wk.bodyScale[i1] = std::max( wk.bodyScale[i1], s.impactSpeed_[i] );
         if( !s.body2_[i]->isFixed() ) wk.bodyScale[i2] = std::max( wk.bodyScale[i2], s.impactSpeed_[i] );
      }
      else {
         wk.kind[i] = lambda0 > real( 0 ) ? RestitutionPassWork::loadedRow : RestitutionPassWork::freeRow;
         wk.target[i] = lambda0 > real( 0 ) ? g0 : std::min( g0, -s.dist_[i] * dtinv );
         ++stats.supportRows;
         closingBefore = std::max( closingBefore, -s.dist_[i] * dtinv - g0 );
      }
   }
   {
      real g[2] = { impactMax, closingBefore };
      restitutionGlobalMax( g, 2 );
      impactMax = g[0]; closingBefore = g[1];
   }
   stats.supportClosingBefore = closingBefore;
   if( !( impactMax > real( 0 ) ) ) { stats.stop = 1; stats.supportClosingAfter = closingBefore; return; }
   stats.impactMax = impactMax;
   const size_t pass( passCounter++ );

   // 2. the same body scale on the owner and every shadow copy, then the row scales
   s.exchangeRestitutionBodyScales( wk.bodyScale );
   real supportScale( 0 );
   for( size_t i = 0; i < n; ++i ) {
      const real own( wk.kind[i] == RestitutionPassWork::armedRow ? s.impactSpeed_[i] : real( 0 ) );
      wk.scale[i] = std::max( own, std::max( wk.bodyScale[s.body1_[i]->index_], wk.bodyScale[s.body2_[i]->index_] ) );
      if( wk.kind[i] != RestitutionPassWork::armedRow ) supportScale = std::max( supportScale, wk.scale[i] );
   }

   // 3. sweeps with the stop rule on the synchronised state
   const bool traced( RestitutionTrace::instance().active( pass ) );
   const size_t minSweeps( std::max( rule.minSweeps, size_t( 1 ) ) );
   const size_t maxSweeps( std::max( minSweeps, rule.maxSweeps ) );
   const size_t check( std::max( rule.checkInterval, size_t( 1 ) ) );
   stats.stop = 3;
   for( size_t it = 1; it <= maxSweeps; ++it ) {
      restitutionPostPassSweep( s, dtinv );
      s.synchronizeVelocities();
      ++stats.sweeps;
      const bool test( it >= minSweeps && ( ( it - minSweeps ) % check == 0 || it == maxSweeps ) );
      if( !test && !traced ) continue;
      real m[4];
      restitutionResiduals( s, m );
      const real localViolations( m[3] );
      restitutionGlobalMax( m, 4 );
      if( traced && MPISettings::rank() == MPISettings::root() )
         std::printf( "EL_RESTITUTION_TRACE pass= %zu sweep= %zu residual= %.6e residual_rel= %.6e tangential= %.6e violations_rank_max= %.0f\n",
                      pass, it, static_cast<double>( m[0] ), static_cast<double>( m[1] ), static_cast<double>( m[2] ), static_cast<double>( m[3] ) );
      if( !test ) continue;
      stats.residual = m[0]; stats.residualRel = m[1]; stats.tangentialResidual = m[2];
      stats.violations = static_cast<size_t>( localViolations );   // local here, summed at the end
      if( !( m[3] > real( 0 ) ) ) { stats.stop = 2; break; }
   }

   // 4. what the pass leaves
   real impactError( 0 ), shortfall( 0 );
   for( size_t i = 0; i < n; ++i ) {
      if( wk.kind[i] != RestitutionPassWork::armedRow && wk.kind[i] != RestitutionPassWork::demotedRow ) continue;
      const real gdot_n( trans( s.n_[i] ) * restitutionRowVelocity( s, i ) );
      if( gdot_n > real( 0 ) ) ++stats.separating;
      if( wk.kind[i] != RestitutionPassWork::armedRow ) continue;
      if( s.restitutionImpulse_[i] > real( 0 ) ) { ++stats.delivered; stats.finalRestitutionImpulse += s.restitutionImpulse_[i]; }
      bool newton( false ), cut( false );
      real approach( 0 );
      bool release( false );
      restitutionRowIncrement( s, i, gdot_n, newton, cut, approach, release );
      if( cut ) shortfall = std::max( shortfall, s.impactSpeed_[i] - gdot_n );
      else if( newton ) impactError = std::max( impactError, std::abs( s.impactSpeed_[i] - gdot_n ) );
   }
   real g[4] = { impactError, shortfall, restitutionSupportClosing( s, dtinv ), supportScale };
   restitutionGlobalMax( g, 4 );
   stats.impactError = g[0]; stats.newtonShortfall = g[1]; stats.supportClosingAfter = g[2]; stats.supportScaleMax = g[3];
   restitutionGlobalSum( stats.violations );
}
//*************************************************************************************************

   //! Pair trace (PE_RESTITUTION_PAIRS, RestitutionTrace): rows with e > 0 approaching faster than
   //! vmin before the pass (after = false), or the rows armed at the classification after it
   //! (after = true). Local, every process.
   template< typename CS >
   static void tracePairs( const CS& s, size_t step, real vmin, bool after )
   {
      const RestitutionPassWork& wk( s.restitutionWork_ );
      for( size_t i = 0; i < s.p_.size(); ++i ) {
         if( !( s.cor_[i] > real( 0 ) ) ) continue;
         const bool wasArmed( i < wk.kind.size() && ( wk.kind[i] == RestitutionPassWork::armedRow || wk.kind[i] == RestitutionPassWork::demotedRow ) );
         const real vn( restitutionPreStepNormalVelocity( s.body1_[i], s.body2_[i], s.r1_[i], s.r2_[i], s.n_[i] ) );
         if( after ? !wasArmed : !( vn < -vmin ) ) continue;
         const real lambda( trans( s.n_[i] ) * s.p_[i] );
         const real gdot_n( trans( s.n_[i] ) * restitutionRowVelocity( s, i ) );
         const char* tag( after ? ( wk.kind[i] == RestitutionPassWork::demotedRow ? "demoted" : "armed" )
                                : ( s.impactSpeed_[i] > real( 0 ) ? "candidate" : "below" ) );
         std::printf( "%s step= %zu rank= %d id1= %llu id2= %llu sid1= %llu sid2= %llu dist= %.6e vn_pre= %.6e impact= %.6e lambda= %.6e lambda_r= %.6e gdot_n= %.6e %s\n",
                      after ? "EL_RESTITUTION_EVENT" : "EL_RESTITUTION_PAIR", step, static_cast<int>( MPISettings::rank() ),
                      static_cast<unsigned long long>( s.body1_[i]->getID() ), static_cast<unsigned long long>( s.body2_[i]->getID() ),
                      static_cast<unsigned long long>( s.body1_[i]->getSystemID() ), static_cast<unsigned long long>( s.body2_[i]->getSystemID() ),
                      static_cast<double>( s.dist_[i] ), static_cast<double>( vn ), static_cast<double>( s.impactSpeed_[i] ),
                      static_cast<double>( lambda ), static_cast<double>( s.restitutionImpulse_[i] ), static_cast<double>( gdot_n ), tag );
      }
      std::fflush( stdout );
   }

   //! Entry point of both solvers, after their synchronised inelastic velocity solve (switch on,
   //! post-pass scheme). Collective: every process calls it in every step.
   template< typename CS >
   static void run( CS& s, real dtinv )
   {
      RestitutionTrace& tr( RestitutionTrace::instance() );
      tr.parse();
      const size_t step( tr.step++ );
      const bool pairs( tr.pairSpeed >= real( 0 ) );
      if( pairs ) tracePairs( s, step, tr.pairSpeed, false );
      restitutionPostPass( s, dtinv );
      if( pairs ) tracePairs( s, step, tr.pairSpeed, true );
   }


   //! Owner <-> shadow maximum of a per-body value \a scale (indexed by index_: local bodies first,
   //! then the shadow copies), so that every process holding a body sees the same value. Two
   //! messages on the velocity-synchronisation path (shadow copies -> owner, owner -> shadow
   //! copies; RigidBodyVelocityCorrectionNotification carries the value in dv[0]), only positive
   //! values are sent. Collective; a no-op on one process. Called by the post-pass only when it
   //! runs (switch on, an armed row on some process).
   template< typename CS >
   static void exchangeBodyScales( CS& s, std::vector<real>& scale )
   {
#if HAVE_MPI
      if( MPISettings::size() <= 1 ) return;
      typedef typename CS::BodyIterator    BI;
      typedef typename CS::ProcessIterator PI;
      // 1. shadow copies -> owner (maximum)
      size_t i( s.bodystorage_.size() );
      for( BI body = s.bodystorageShadowCopies_.begin(); body != s.bodystorageShadowCopies_.end(); ++body, ++i ) {
         if( !( scale[i] > real( 0 ) ) ) continue;
         typename CS::ProcessID process( body->getOwnerHandle() );
         marshal( process->getSendBuffer(), notificationType<RigidBodyVelocityCorrectionNotification>() );
         marshal( process->getSendBuffer(), RigidBodyVelocityCorrectionNotification( *(*body), Vec3( scale[i], 0, 0 ), Vec3() ) );
      }
      s.communicate( mpitagHCTSSynchronizeVelocityCorrections );
      for( PI process = s.processstorage_.begin(); process != s.processstorage_.end(); ++process ) {
         auto& buffer( process->getRecvBuffer() );
         NotificationType type;
         while( !buffer.isEmpty() ) {
            unmarshal( buffer, type );
            if( type != rigidBodyVelocityCorrectionNotification ) throw std::runtime_error( "Restitution scale exchange: invalid notification type." );
            RigidBodyVelocityCorrectionNotification::Parameters objparam;
            unmarshal( buffer, objparam );
            typename CS::BodyID b( *s.bodystorage_.find( objparam.sid_ ) );
            scale[b->index_] = std::max( scale[b->index_], objparam.dv_[0] );
         }
      }
      // 2. owner -> shadow copies (the maximum)
      i = 0;
      for( BI body = s.bodystorage_.begin(); body != s.bodystorage_.end(); ++body, ++i ) {
         if( body->isGlobal() || !( scale[i] > real( 0 ) ) ) continue;
         for( PI it = body->beginProcesses(); it != body->endProcesses(); ++it ) {
            marshal( it->getSendBuffer(), notificationType<RigidBodyVelocityCorrectionNotification>() );
            marshal( it->getSendBuffer(), RigidBodyVelocityCorrectionNotification( *(*body), Vec3( scale[i], 0, 0 ), Vec3() ) );
         }
      }
      s.communicate( mpitagHCTSSynchronizeVelocities );
      for( PI process = s.processstorage_.begin(); process != s.processstorage_.end(); ++process ) {
         auto& buffer( process->getRecvBuffer() );
         NotificationType type;
         while( !buffer.isEmpty() ) {
            unmarshal( buffer, type );
            if( type != rigidBodyVelocityCorrectionNotification ) throw std::runtime_error( "Restitution scale exchange: invalid notification type." );
            RigidBodyVelocityCorrectionNotification::Parameters objparam;
            unmarshal( buffer, objparam );
            typename CS::BodyID b( *s.bodystorageShadowCopies_.find( objparam.sid_ ) );
            scale[b->index_] = objparam.dv_[0];
         }
      }
#else
      (void)s; (void)scale;
#endif
   }

   //! Refusal of a finite-mass (non-fixed) global body while the switch is on (both solvers):
   //! their synchronisation reduces the corrections of such bodies without consuming them, so the
   //! extra synchronisations of the pass would apply them again (doc/technical-notes/el-restitution.md,
   //! follow-up). Read from the live body storage (globalNonfixedBodies_ is rebuilt from it at
   //! the start of every multi-process step). Global bodies exist on every process, so
   //! every process throws in the same step, before any communication of that step.
   template< typename CS >
   static void refuseMobileGlobal( const CS& s, const char* solver )
   {
      for( typename CS::BS::ConstIterator b = s.bodystorage_.begin(); b != s.bodystorage_.end(); ++b ) {
         if( !b->isGlobal() || b->isFixed() ) continue;
         char msg[640];
         std::snprintf( msg, sizeof( msg ), "%s: restitution (setRestitution(), json restitutionEnabled_) is not supported "
                        "together with a finite-mass global body (system id %llu, user id %llu): the synchronisation of non-fixed "
                        "global bodies would apply the post-pass corrections more than once. Fix the body (setFixed( true )) "
                        "or switch restitution off.", solver, static_cast<unsigned long long>( b->getSystemID() ),
                        static_cast<unsigned long long>( b->getID() ) );
         throw std::runtime_error( msg );
      }
   }
};
//*************************************************************************************************

} // namespace pe

#endif
