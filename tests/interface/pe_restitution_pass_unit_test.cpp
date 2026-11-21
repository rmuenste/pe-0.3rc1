//=================================================================================================
/*!
 *  \file tests/interface/pe_restitution_pass_unit_test.cpp
 *  \brief Unit tests of the restitution post-pass (RestitutionPass of RestitutionPostPass.h) on a
 *         mock collision system: rows, bodies and velocities set by hand, the sweeps and the
 *         whole pass called directly, the "other contacts" between sweeps emulated by setting dv_.
 *
 *  Serial (synchronizeVelocities() is a no-op as on one process). Bodies: spheres of mass 1,
 *  radius 0.01 (inverse inertia 25000), and fixed walls (inverse mass and inertia 0).
 *
 *    1. a free support row loaded by a disturbance whose velocity then returns bitwise to its
 *       post-compression value must release the impulse (review round 3, finding 3);
 *    2. a sliding row whose friction was re-relaxed at a disturbed velocity and whose velocity
 *       then returns to the original slip: the tangential projection is not stationary, the row
 *       must be visited (finding 3);
 *    3. unload - reload of an armed row with a Baumgarte (positive) post-compression velocity:
 *       lambda_R <= e lambda_C after every sweep; the fallback impulse after the full withdrawal
 *       is support impulse, not restitution (finding 4);
 *    4. a sticking wall contact that a capped floor rebound sets slipping: the pass must not
 *       report convergence while the tangential residual is outside the tolerance (finding 2);
 *    5. an impact on a fixed wall does not propagate an impact scale to an unrelated row on the
 *       same wall (finding 5);
 *    6. review round 4, finding 2: an armed row with an OBLIQUE normal (1,1,0)/sqrt(2) fully
 *       unloaded by the projection, then closed again: no restitution may be booked (n . p_ left a
 *       roundoff load of 2e-16 that kept the row armed with a fresh compression allowance);
 *    7. an armed row fully unloaded on the LAST sweep of a pass is counted as demoted and its
 *       stored impulse is exactly zero;
 *    8. review round 5, finding 1: one visit of a SLIDING row (mu 0.3) whose load increases
 *       (lambda 1, d 0.5): the tangential impulse ends on the disc 0.45 (b047e41: 0.60, the radius
 *       used lambda + 2d), and the residual evaluator agrees (zero tangential residual after it);
 *    9. the same with a partial withdrawal (lambda 1, d -0.6): friction limited to 0.12 (b047e41:
 *       discarded);
 *   10. randomized consistency (fixed seed): 300 independent sphere pairs with oblique normals,
 *       friction, armed and support rows, random disturbances over 8 sweeps (loads, withdrawals,
 *       releases); after every sweep |n . p - lambda| within rounding, tangential impulse within
 *       mu lambda, lambda_R <= e lambda_C, p_ = 0 after a release, the pair's momentum unchanged
 *       by the sweep.
 *
 *  The test compiles against the round-2 pass (45c9268); fields added later are set through
 *  detectors, so the before results are runtime results.
 */
//=================================================================================================

#include <pe/core/collisionsystem/RestitutionPostPass.h>
#include <pe/math/Matrix2x2.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector3.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <type_traits>
#include <vector>

using namespace pe;

static int failures = 0;
static void expect( bool ok, const std::string& what )
{
   if( !ok ) { std::printf( "FAIL: %s\n", what.c_str() ); ++failures; }
}

struct MockBody
{
   size_t index_;
   real invMass;
   Mat3 Iinv;
   bool fixed;
   Vec3 v, w;
   unsigned long long id;
   real getInvMass() const { return invMass; }
   const Mat3& getInvInertia() const { return Iinv; }
   bool isFixed() const { return fixed; }
   Vec3 getLinearVel() const { return v; }
   Vec3 getAngularVel() const { return w; }
   unsigned long long getID() const { return id; }
   unsigned long long getSystemID() const { return id; }
};

static const real kR( 0.01 ), kIinv( 25000 );

struct MockCS
{
   enum Model { InelasticFrictionlessContact = 0, ApproximateInelasticCoulombContactByDecoupling = 1 };
   int relaxationModel_ = ApproximateInelasticCoulombContactByDecoupling;
   std::vector<Vec3> p_, r1_, r2_, n_, t_, o_, surfaceVel_;
   std::vector<MockBody*> body1_, body2_;
   std::vector<real> dist_, mu_, diag_n_inv_, cor_, impactSpeed_, restitutionImpulse_;
   std::vector<Mat2> diag_to_inv_;
   std::vector<Vec3> v_, w_, dv_, dw_;
   RestitutionStopRule restitutionStopRule_;
   RestitutionPassWork restitutionWork_;
   RestitutionPassStats restitutionPassStats_;
   std::vector<MockBody> bodies;
   std::function<void( int )> hook;   //!< called after every sweep (the "other contacts")
   int syncs = 0;
   void synchronizeVelocities() { ++syncs; if( hook ) hook( syncs ); }
   void exchangeRestitutionBodyScales( std::vector<real>& ) {}

   MockCS() { bodies.reserve( 8 ); }
   size_t addSphere( const Vec3& v ) { return addBody( real(1), Mat3( kIinv, kIinv, kIinv ), false, v ); }
   size_t addWall() { return addBody( real(0), Mat3( real(0) ), true, Vec3() ); }
   size_t addBody( real im, const Mat3& Iinv, bool fixed, const Vec3& v )
   {
      MockBody b;
      b.index_ = bodies.size(); b.invMass = im; b.Iinv = Iinv; b.fixed = fixed; b.v = v; b.w = Vec3(); b.id = bodies.size() + 1;
      bodies.push_back( b );
      v_.push_back( v ); w_.push_back( Vec3() ); dv_.push_back( Vec3() ); dw_.push_back( Vec3() );
      return b.index_;
   }
   //! Row between sphere b1 (contact point at -R n from its centre) and body b2 (a wall).
   void addRow( size_t b1, size_t b2, const Vec3& n, const Vec3& t, real lambda, const Vec3& pt, real mu, real e, real impact )
   {
      body1_.push_back( &bodies[b1] ); body2_.push_back( &bodies[b2] );
      r1_.push_back( -kR * n ); r2_.push_back( Vec3() );
      n_.push_back( n ); t_.push_back( t ); o_.push_back( n % t ); surfaceVel_.push_back( Vec3() );
      p_.push_back( n * lambda + pt );
      dist_.push_back( real(0) ); mu_.push_back( mu ); cor_.push_back( e ); impactSpeed_.push_back( impact );
      restitutionImpulse_.push_back( real(0) );
      // sphere against a wall, contact point on the line of the normal: normal mass 1, tangential 1 / 3.5
      diag_n_inv_.push_back( real(1) );
      diag_to_inv_.push_back( Mat2( real(1) / real(3.5), real(0), real(0), real(1) / real(3.5) ) );
   }
   //! Row between two spheres b1 and b2 (contact points at -R n and +R n from the centres):
   //! normal mass 1/2, tangential response 2 (1 + R^2 Iinv) = 7.
   void addPairRow( size_t b1, size_t b2, const Vec3& n, const Vec3& t, real lambda, const Vec3& pt, real mu, real e, real impact )
   {
      addRow( b1, b2, n, t, lambda, pt, mu, e, impact );
      r2_.back() = kR * n;
      diag_n_inv_.back() = real(0.5);
      diag_to_inv_.back() = Mat2( real(1) / real(7), real(0), real(0), real(1) / real(7) );
   }
   Vec3 vel( size_t b ) const { return v_[b] + dv_[b]; }
};

// Fields added in review round 3 (absent in 45c9268): set when they exist.
template< typename W, typename = void > struct HasAffected : std::false_type {};
template< typename W > struct HasAffected< W, std::void_t< decltype( std::declval<W&>().affected ) > > : std::true_type {};
template< typename W, typename = void > struct HasLedger : std::false_type {};
template< typename W > struct HasLedger< W, std::void_t< decltype( std::declval<W&>().lambda ) > > : std::true_type {};
template< typename S, typename = void > struct HasFallback : std::false_type {};
template< typename S > struct HasFallback< S, std::void_t< decltype( std::declval<S&>().fallbackImpulse ) > > : std::true_type {};

// dependent helpers: the member access is only instantiated where the member exists
template< typename W > void initAffected( W& wk, size_t n ) { if constexpr ( HasAffected<W>::value ) wk.affected.assign( n, 0 ); else { (void)wk; (void)n; } }
template< typename W > void initLedger( W& wk, const MockCS& s )
{
   if constexpr ( HasLedger<W>::value ) { wk.lambda.resize( s.p_.size() ); for( size_t i = 0; i < s.p_.size(); ++i ) wk.lambda[i] = trans( s.n_[i] ) * s.p_[i]; }
   else { (void)wk; (void)s; }
}
template< typename S > bool fallbackTallied( const S& st )
{
   if constexpr ( HasFallback<S>::value ) return st.fallbackImpulse > real(0) && st.demoted == 1;
   else { (void)st; return true; }
}

//! Classification by hand (the state restitutionPostPass() builds before its first sweep).
static void classifyByHand( MockCS& s, real dtinv )
{
   RestitutionPassWork& wk( s.restitutionWork_ );
   const size_t n( s.p_.size() );
   wk.gdot0.resize( n ); wk.target.resize( n ); wk.scale.assign( n, real(0) ); wk.kind.resize( n );
   wk.bodyScale.assign( s.v_.size(), real(0) );
   initAffected( wk, n );
   initLedger( wk, s );
   for( size_t i = 0; i < n; ++i ) {
      wk.gdot0[i] = RestitutionPass::restitutionRowVelocity( s, i );
      const real g0( trans( s.n_[i] ) * wk.gdot0[i] ), lambda0( trans( s.n_[i] ) * s.p_[i] );
      if( s.impactSpeed_[i] > real(0) && lambda0 > real(0) ) { wk.kind[i] = RestitutionPassWork::armedRow; wk.target[i] = g0; }
      else { wk.kind[i] = lambda0 > real(0) ? RestitutionPassWork::loadedRow : RestitutionPassWork::freeRow;
             wk.target[i] = lambda0 > real(0) ? g0 : std::min( g0, -s.dist_[i] * dtinv ); }
   }
}

static real tangential( const MockCS& s, size_t i ) { return trans( s.t_[i] ) * s.p_[i]; }
static real normal( const MockCS& s, size_t i ) { return trans( s.n_[i] ) * s.p_[i]; }

//=================================================================================================
// 1. free row loaded, velocity back to g0: release
//=================================================================================================
static void caseRelease()
{
   MockCS s;
   s.relaxationModel_ = MockCS::InelasticFrictionlessContact;
   const size_t a( s.addSphere( Vec3( 0, 0, real(0.1) ) ) ), f( s.addWall() );
   s.addRow( a, f, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(0), Vec3(), real(0), real(0), real(0) );
   classifyByHand( s, real(1e4) );
   s.dv_[a] = Vec3( 0, 0, real(-0.15) );                 // a rebound elsewhere closes the row
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   const real loaded( normal( s, 0 ) );
   s.dv_[a] = Vec3();                                     // another contact restores the post-compression velocity
   const bool back( RestitutionPass::restitutionRowVelocity( s, 0 )[2] == s.restitutionWork_.gdot0[0][2] );
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   std::printf( "1. release: loaded %.6e by the disturbance, velocity back to g0 bitwise %d, impulse after the next sweep %.6e\n",
                static_cast<double>( loaded ), back ? 1 : 0, static_cast<double>( normal( s, 0 ) ) );
   expect( loaded > real(0) && back, "1. the disturbance loads the row and the velocity returns bitwise" );
   expect( normal( s, 0 ) == real(0), "1. a row back at g0 with a changed impulse is visited and releases its impulse" );
}

//=================================================================================================
// 2. sliding row, friction re-relaxed at a disturbed velocity, slip restored
//=================================================================================================
static void caseSlip()
{
   MockCS s;
   const size_t a( s.addSphere( Vec3( real(0.2), 0, 0 ) ) ), f( s.addWall() );
   // loaded sliding floor contact: lambda 1, friction at the disc boundary mu lambda = 0.5 against the slip
   s.addRow( a, f, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(1), Vec3( real(-0.5), 0, 0 ), real(0.5), real(0), real(0) );
   classifyByHand( s, real(1e4) );
   s.dv_[a] = Vec3( real(-0.2), 0, real(-0.3) );          // disturbance: closing, slip stopped
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   const real lambda1( normal( s, 0 ) ), pt1( tangential( s, 0 ) );
   s.dv_[a] = Vec3(); s.dw_[a] = Vec3();                  // velocity back to the original slip
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   const real pt2( tangential( s, 0 ) );
   std::printf( "2. slip: after the disturbance lambda %.6f friction %.6f; slip restored, friction after the next sweep %.6f (limit %.6f)\n",
                static_cast<double>( lambda1 ), static_cast<double>( pt1 ), static_cast<double>( pt2 ), static_cast<double>( 0.5 * lambda1 ) );
   expect( lambda1 > real(1) && std::fabs( pt1 + real(0.5) ) < real(1e-12), "2. the disturbance loads the row while it sticks" );
   expect( std::fabs( pt2 - pt1 ) > real(1e-6), "2. the restored slip is projected again (friction moves towards the new disc)" );
}

//=================================================================================================
// 3. unload - reload of an armed row: ceiling on every visit, fallback impulse is support
//=================================================================================================
static void caseUnloadReload()
{
   MockCS s;
   s.relaxationModel_ = MockCS::InelasticFrictionlessContact;
   const real e( 0.9 );
   const size_t a( s.addSphere( Vec3( 0, 0, real(0.05) ) ) ), f( s.addWall() );   // Baumgarte: g0 = +0.05
   s.addRow( a, f, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(1), Vec3(), real(0), e, e * real(1) );
   classifyByHand( s, real(1e4) );
   bool ok( true );
   const real velocities[4] = { real(-1), real(5), real(0), real(0) };   // -1: leave the sweep's result
   for( int k = 0; k < 4; ++k ) {
      if( velocities[k] >= real(0) ) s.dv_[a] = Vec3( 0, 0, velocities[k] - s.v_[a][2] );
      RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
      const real lambda( normal( s, 0 ) ), lr( s.restitutionImpulse_[0] ), lc( lambda - lr );
      const bool inv( lr <= e * lc + real(1e-12) );
      std::printf( "3. sweep %d: lambda %.6f lambda_R %.6f lambda_C %.6f, lambda_R <= e lambda_C %d\n", k + 1,
                   static_cast<double>( lambda ), static_cast<double>( lr ), static_cast<double>( lc ), inv ? 1 : 0 );
      ok = ok && inv;
   }
   expect( ok, "3. lambda_R <= e lambda_C after every sweep through unload and reload" );
   expect( fallbackTallied( s.restitutionPassStats_ ),
           "3. the reload after the full withdrawal is tallied as fallback support impulse (one demotion; not checked without the field)" );
}

//=================================================================================================
// 4. tangential residual in the stop decision
//=================================================================================================
static void caseTangentialStop()
{
   MockCS s;
   const real e( 0.9 );
   const size_t a( s.addSphere( Vec3() ) ), wall( s.addWall() ), floor( s.addWall() );
   // row 0: wall contact, normal +x, loaded, sticking (mu 10), visited first
   s.addRow( a, wall, Vec3( 1, 0, 0 ), Vec3( 0, 0, 1 ), real(1), Vec3(), real(10), real(0), real(0) );
   // row 1: armed floor impact (pre-step approach 1 m/s, e 0.9), compression 0.5: the cap binds
   s.addRow( a, floor, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(0.5), Vec3(), real(0), e, e * real(1) );
   s.restitutionStopRule_.minSweeps = 1; s.restitutionStopRule_.maxSweeps = 24; s.restitutionStopRule_.checkInterval = 1;
   s.restitutionStopRule_.absTolerance = real(1e-4); s.restitutionStopRule_.relTolerance = real(1e-3);
   RestitutionPass::restitutionPostPass( s, real(1e4) );
   const RestitutionPassStats& st( s.restitutionPassStats_ );
   const real tol( real(1e-4) + real(1e-3) * e );
   // the wall contact point's tangential (z) velocity at the end
   const Vec3 gw( RestitutionPass::restitutionRowVelocity( s, 0 ) );
   std::printf( "4. tangential stop: stop %d after %zu sweeps, normal residual %.3e, tangential residual %.3e (tolerance %.3e), "
                "wall contact slip at the end %.3e m/s\n", st.stop, st.sweeps, static_cast<double>( st.residual ),
                static_cast<double>( st.tangentialResidual ), static_cast<double>( tol ), static_cast<double>( gw[2] ) );
   expect( !( st.stop == 2 && st.tangentialResidual > tol ), "4. no convergence reported with the tangential residual outside the tolerance" );
   expect( st.stop == 2 && std::fabs( gw[2] ) <= tol, "4. the pass continues until the sticking wall contact holds" );
}

//=================================================================================================
// 5. walls carry no impact scale
//=================================================================================================
static void caseWallScale()
{
   MockCS s;
   s.relaxationModel_ = MockCS::InelasticFrictionlessContact;
   const real e( 0.9 );
   const size_t a( s.addSphere( Vec3() ) ), b( s.addSphere( Vec3() ) ), floor( s.addWall() );
   s.addRow( a, floor, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(0.5), Vec3(), real(0), e, e * real(1) );   // impact of a on the floor
   s.addRow( b, floor, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(1), Vec3(), real(0), real(0), real(0) );  // b resting on the floor, far away
   s.restitutionStopRule_.minSweeps = 1; s.restitutionStopRule_.maxSweeps = 4; s.restitutionStopRule_.checkInterval = 1;
   RestitutionPass::restitutionPostPass( s, real(1e4) );
   std::printf( "5. wall scale: scale of the impact row %.3f, of the unrelated resting row on the same floor %.3f\n",
                static_cast<double>( s.restitutionWork_.scale[0] ), static_cast<double>( s.restitutionWork_.scale[1] ) );
   expect( s.restitutionWork_.scale[0] == e && s.restitutionWork_.scale[1] == real(0),
           "5. the floor does not carry the impact scale to an unrelated row" );
}

//=================================================================================================
// 6. oblique full unload, then a closing reload: no restitution
//=================================================================================================
static void caseObliqueUnload()
{
   MockCS s;
   s.relaxationModel_ = MockCS::InelasticFrictionlessContact;
   const real e( 0.9 ), h( real(1) / std::sqrt( real(2) ) );
   const Vec3 n( h, h, 0 ), t( h, -h, 0 );
   const size_t a( s.addSphere( Vec3() ) ), f( s.addWall() );
   s.addRow( a, f, n, t, real(1), Vec3(), real(0), e, e * real(1) );
   classifyByHand( s, real(1e4) );
   s.dv_[a] = n * real(5);                                  // a neighbour pulls the row apart: full withdrawal
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   const real after1( trans( s.n_[0] ) * s.p_[0] );
   s.dv_[a] = n * real(-1);                                 // then closes it again at 1 m/s
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   std::printf( "6. oblique unload: n . p after the withdrawal %.3e, after the closing reload lambda %.6f, booked restitution %.6e, gross %.6e\n",
                static_cast<double>( after1 ), static_cast<double>( trans( s.n_[0] ) * s.p_[0] ),
                static_cast<double>( s.restitutionImpulse_[0] ), static_cast<double>( s.restitutionPassStats_.restitutionImpulse ) );
   expect( s.restitutionImpulse_[0] == real(0) && s.restitutionPassStats_.restitutionImpulse == real(0),
           "6. a fully withdrawn oblique row books no restitution when it is closed again" );
}

//=================================================================================================
// 7. full unload on the last sweep: demoted, impulse exactly zero
//=================================================================================================
static void caseLastSweepUnload()
{
   MockCS s;
   s.relaxationModel_ = MockCS::InelasticFrictionlessContact;
   const real e( 0.9 );
   const size_t a( s.addSphere( Vec3( 0, 0, real(5) ) ) ), f( s.addWall() );   // separating at 5 m/s after compression
   s.addRow( a, f, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(1), Vec3(), real(0), e, e * real(1) );
   s.restitutionStopRule_.minSweeps = 1; s.restitutionStopRule_.maxSweeps = 1; s.restitutionStopRule_.checkInterval = 1;
   RestitutionPass::restitutionPostPass( s, real(1e4) );
   std::printf( "7. last-sweep unload: sweeps %zu, demoted %zu, stored impulse (%.3e %.3e %.3e), booked %.3e\n", s.restitutionPassStats_.sweeps,
                s.restitutionPassStats_.demoted, static_cast<double>( s.p_[0][0] ), static_cast<double>( s.p_[0][1] ),
                static_cast<double>( s.p_[0][2] ), static_cast<double>( s.restitutionImpulse_[0] ) );
   expect( s.restitutionPassStats_.sweeps == 1 && s.restitutionPassStats_.demoted == 1, "7. a full unload on the last sweep is counted as demoted" );
   expect( s.p_[0][0] == real(0) && s.p_[0][1] == real(0) && s.p_[0][2] == real(0) && s.restitutionImpulse_[0] == real(0),
           "7. the released row's stored impulse is exactly zero" );
}

//=================================================================================================
// 8./9. one visit of a sliding row: the Coulomb radius uses the post-visit load once
//=================================================================================================
template< typename W > real ledger( const W& wk, const MockCS& s, size_t i )
{
   if constexpr ( HasLedger<W>::value ) return wk.lambda[i];
   else return trans( s.n_[i] ) * s.p_[i];
}

static void caseSlidingVisit( real dn, real expectFriction, const char* tag )
{
   MockCS s;
   const real mu( 0.3 );
   const size_t a( s.addSphere( Vec3( real(2), 0, 0 ) ) ), f( s.addWall() );   // slip 2 m/s along t = x
   s.addRow( a, f, Vec3( 0, 0, 1 ), Vec3( 1, 0, 0 ), real(1), Vec3( -mu, 0, 0 ), mu, real(0), real(0) );   // sliding, on the disc
   classifyByHand( s, real(1e4) );
   s.dv_[a] = Vec3( 0, 0, -dn );   // normal velocity -dn: the support target g0 = 0 gives d = dn (normal mass 1)
   RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
   const real lambda( trans( s.n_[0] ) * s.p_[0] ), pt( trans( s.t_[0] ) * s.p_[0] );
   real m[4];
   RestitutionPass::restitutionResiduals( s, m );
   std::printf( "%s: lambda after the visit %.6f (ledger %.6f), tangential impulse %.6f (disc mu lambda = %.6f), residual after the visit: normal %.3e tangential %.3e\n",
                tag, static_cast<double>( lambda ), static_cast<double>( ledger( s.restitutionWork_, s, 0 ) ), static_cast<double>( pt ),
                static_cast<double>( mu * lambda ), static_cast<double>( m[0] ), static_cast<double>( m[2] ) );
   expect( std::fabs( std::fabs( pt ) - expectFriction ) <= real(1e-12), std::string( tag ) + ": the tangential impulse ends on the disc of the post-visit load" );
   expect( m[2] <= real(1e-12), std::string( tag ) + ": the residual evaluator agrees with the sweep (zero tangential residual after the visit)" );
}

//=================================================================================================
// 10. randomized consistency
//=================================================================================================
static void caseRandomized()
{
   MockCS s;
   s.bodies.reserve( 700 );
   const int P( 300 );
   unsigned long long seed( 20261122ULL );
   const auto rnd = [&seed]() { seed = seed * 6364136223846793005ULL + 1442695040888963407ULL; return real( ( seed >> 11 ) * ( 1.0 / 9007199254740992.0 ) ); };
   const auto unit = [&]() { Vec3 v; do { v = Vec3( 2 * rnd() - 1, 2 * rnd() - 1, 2 * rnd() - 1 ); } while( v.length() < real(0.1) || v.length() > real(1) ); return v / v.length(); };
   for( int k = 0; k < P; ++k ) { s.addSphere( Vec3() ); s.addSphere( Vec3() ); }
   for( int k = 0; k < P; ++k ) {
      const Vec3 n( unit() );
      Vec3 t( n % unit() ); t /= t.length();
      const real mu( real(0.5) * rnd() ), lambda( rnd() < real(0.2) ? real(0) : real(2) * rnd() );
      const real e( rnd() < real(0.5) ? real(0.9) : real(0) ), impact( e > 0 && lambda > 0 && rnd() < real(0.7) ? e * real(2) * rnd() : real(0) );
      const real ang( real(6.283185307179586) * rnd() ), rad( mu * lambda * rnd() );
      const Vec3 o( n % t );
      s.addPairRow( 2 * k, 2 * k + 1, n, t, lambda, t * ( rad * std::cos( ang ) ) + o * ( rad * std::sin( ang ) ), mu, e, impact );
   }
   classifyByHand( s, real(1e4) );
   int bad( 0 ); size_t releases( 0 ), withdrawals( 0 ), loads( 0 );
   real worstLedger( 0 ), worstDisc( 0 ), worstCeiling( 0 ), worstMomentum( 0 );
   for( int sweep = 0; sweep < 8; ++sweep ) {
      for( size_t b = 0; b < s.dv_.size(); ++b ) s.dv_[b] += unit() * ( real(1.5) * rnd() );
      std::vector<Vec3> dv0( s.dv_ ); std::vector<real> lam0( P );
      for( int k = 0; k < P; ++k ) lam0[k] = ledger( s.restitutionWork_, s, k );
      RestitutionPass::restitutionPostPassSweep( s, real(1e4) );
      for( int k = 0; k < P; ++k ) {
         const real lambda( ledger( s.restitutionWork_, s, k ) ), np( trans( s.n_[k] ) * s.p_[k] );
         const real pt( std::sqrt( std::pow( trans( s.t_[k] ) * s.p_[k], 2 ) + std::pow( trans( s.o_[k] ) * s.p_[k], 2 ) ) );
         const real lr( s.restitutionImpulse_[k] );
         if( lam0[k] > 0 && lambda == 0 ) ++releases; else if( lambda < lam0[k] ) ++withdrawals; else if( lambda > lam0[k] ) ++loads;
         worstLedger = std::max( worstLedger, std::fabs( np - lambda ) / ( real(1) + std::fabs( lambda ) ) );
         worstDisc = std::max( worstDisc, pt - s.mu_[k] * lambda );
         worstCeiling = std::max( worstCeiling, lr - s.cor_[k] * ( lambda - lr ) );
         const Vec3 dmom( ( s.dv_[2 * k] - dv0[2 * k] ) + ( s.dv_[2 * k + 1] - dv0[2 * k + 1] ) );   // both masses 1
         worstMomentum = std::max( worstMomentum, dmom.length() );
         if( lambda == real(0) && lam0[k] > 0 && !( s.p_[k][0] == 0 && s.p_[k][1] == 0 && s.p_[k][2] == 0 ) ) ++bad;
      }
   }
   std::printf( "10. randomized (300 pairs, 8 sweeps): releases %zu, partial withdrawals %zu, loads %zu; worst |n.p - lambda|/(1+lambda) %.3e, "
                "worst |p_t| - mu lambda %.3e, worst lambda_R - e lambda_C %.3e, worst momentum change %.3e, released rows with p != 0: %d\n",
                releases, withdrawals, loads, static_cast<double>( worstLedger ), static_cast<double>( worstDisc ), static_cast<double>( worstCeiling ),
                static_cast<double>( worstMomentum ), bad );
   expect( releases > 0 && withdrawals > 0 && loads > 0, "10. the random visits exercise releases, partial withdrawals and loads" );
   expect( worstLedger <= real(1e-14), "10. |n . p - lambda| within rounding after every sweep" );
   expect( worstDisc <= real(1e-13), "10. tangential impulse within mu lambda after every sweep" );
   expect( worstCeiling <= real(1e-13), "10. lambda_R <= e lambda_C after every sweep" );
   expect( worstMomentum <= real(1e-13), "10. the sweep conserves the momentum of every pair" );
   expect( bad == 0, "10. a released row has p_ = 0 exactly" );
}

int main()
{
   caseRelease();
   caseSlip();
   caseUnloadReload();
   caseTangentialStop();
   caseWallScale();
   caseObliqueUnload();
   caseLastSweepUnload();
   caseSlidingVisit( real(0.5), real(0.45), "8. sliding row, load increase (lambda 1, d 0.5)" );
   caseSlidingVisit( real(-0.6), real(0.12), "9. sliding row, partial withdrawal (lambda 1, d -0.6)" );
   caseRandomized();
   if( failures == 0 ) std::printf( "pe_restitution_pass_unit_test: all checks passed\n" );
   else std::printf( "pe_restitution_pass_unit_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
