//=================================================================================================
/*!
 *  \file tests/interface/pe_restitution_stoprule_test.cpp
 *  \brief Stop rule and diagnostics of the restitution post-pass (RestitutionStopRule,
 *         RestitutionPassStats), against both solvers with the switch (pe-restitution-stoprule:
 *         HardContactEulerLagrange, -plain: HCSITS).
 *
 *  A sphere into a touching row of four (5-chain, split impulse, e = 0.9, no forces), checked on
 *  the impact step:
 *    1. default stop rule (min 4, max 24, check every 4, 1e-4 + 1e-3 of the local impact scale):
 *       converged (stop = 2) within the budget, residual within the tolerance, one candidate, one
 *       armed row, one rebound delivered, final restitution impulse > 0 and <= the gross one;
 *    2. budget 2 (min 1): stop = 3 after exactly 2 sweeps, residual above the tolerance, the support
 *       closing after the pass equal to the closing speed left on the support contacts, support
 *       closing before the pass 0;
 *    3. a candidate without compression (a positive gap the step does not close): no armed row,
 *       pass skipped (stop = 1), no restitution impulse booked (the round-2 bookkeeping edge).
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>

using namespace pe;

typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

template< typename T, typename = void > struct HasStopRule : std::false_type {};
template< typename T > struct HasStopRule< T, std::void_t< decltype( std::declval<T&>().setRestitutionCheckInterval( size_t(1) ) ) > > : std::true_type {};

static int failures = 0;
static void expect( bool ok, const std::string& what )
{
   if( !ok ) { std::printf( "FAIL: %s\n", what.c_str() ); ++failures; }
}

static const real kR( 0.01 );

template< typename Handle > RestitutionPassStats chain( Handle& cs, size_t minSweeps, size_t maxSweeps, real& closing )
{
   cs->setSplitImpulse( true );
   cs->setRestitution( true );
   cs->setRestitutionPostPass( true );
   cs->setRestitutionThreshold( real(0) );
   cs->setRestitutionIterations( minSweeps );
   cs->setRestitutionMaxIterations( maxSweeps );
   cs->setRestitutionCheckInterval( 4 );
   cs->setRestitutionTolerance( real(1e-4), real(1e-3) );
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const real u( 0.5 ), dt( 1e-4 ), gap( u * dt * real(5.9) );
   const MaterialID m( createMaterial( real(1000), real(0.9), real(0), real(0), real(0.25), real(300), real(1e5), real(10), real(10) ) );
   std::vector<SphereID> s;
   s.push_back( createSphere( 1, Vec3( -real(2) * kR - gap, 0, 0 ), kR, m ) );
   for( int k = 1; k < 5; ++k ) s.push_back( createSphere( 1 + k, Vec3( real(2) * ( k - 1 ) * kR, 0, 0 ), kR, m ) );
   s[0]->setLinearVel( Vec3( u, 0, 0 ) );
   RestitutionPassStats ps;
   for( int i = 0; i < 40; ++i ) {
      const real v0( s[0]->getLinearVel()[0] );
      world->simulationStep( dt );
      if( s[0]->getLinearVel()[0] != v0 ) { ps = cs->getRestitutionPassStats(); break; }
   }
   closing = 0;
   for( int k = 1; k + 1 < 5; ++k ) closing = std::max( closing, s[k]->getLinearVel()[0] - s[k+1]->getLinearVel()[0] );
   return ps;
}

template< typename Handle > void run( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasStopRule<CS>::value ) {
      real closing( 0 );
      const RestitutionPassStats a( chain( cs, 4, 24, closing ) );
      const real tol( real(1e-4) + real(1e-3) * a.impactMax );
      std::printf( "5-chain, default rule: stop %d, sweeps %zu, residual %.3e (rel %.3e, tol %.3e), tangential %.3e, candidates %zu, armed %zu, "
                   "delivered %zu, impulse gross %.6e final %.6e, closing before %.3e after %.3e, impact error %.3e, shortfall %.3e\n",
                   a.stop, a.sweeps, static_cast<double>( a.residual ), static_cast<double>( a.residualRel ), static_cast<double>( tol ),
                   static_cast<double>( a.tangentialResidual ), a.candidates, a.armedRows, a.delivered,
                   static_cast<double>( a.restitutionImpulse ), static_cast<double>( a.finalRestitutionImpulse ),
                   static_cast<double>( a.supportClosingBefore ), static_cast<double>( a.supportClosingAfter ),
                   static_cast<double>( a.impactError ), static_cast<double>( a.newtonShortfall ) );
      expect( a.stop == 2 && a.sweeps <= 24 && a.sweeps >= 4, "default rule: converged within the budget" );
      expect( a.residual <= tol, "default rule: residual within abs + rel * impact scale" );
      expect( a.candidates == 1 && a.armedRows == 1 && a.delivered == 1, "default rule: one candidate, one armed row, one rebound delivered" );
      expect( a.finalRestitutionImpulse > real(0) && a.finalRestitutionImpulse <= a.restitutionImpulse * ( real(1) + real(1e-12) ),
              "default rule: final restitution impulse positive and not above the gross one" );
      expect( a.supportClosingBefore <= real(1e-12), "default rule: no support closing after the compression solve" );

      const RestitutionPassStats b( chain( cs, 1, 2, closing ) );
      std::printf( "5-chain, budget 2: stop %d, sweeps %zu, residual %.3e, closing after %.3e (measured %.3e)\n",
                   b.stop, b.sweeps, static_cast<double>( b.residual ), static_cast<double>( b.supportClosingAfter ), static_cast<double>( closing ) );
      expect( b.stop == 3 && b.sweeps == 2, "budget 2: budget exhausted after 2 sweeps" );
      expect( b.residual > real(1e-4) + real(1e-3) * b.impactMax, "budget 2: residual above the tolerance" );
      expect( std::fabs( b.supportClosingAfter - closing ) <= real(1e-12), "budget 2: support closing after = closing left on the support contacts" );

      // 3. candidate without compression
      cs->setSplitImpulse( false );
      WorldID world = theWorld();
      world->clear();
      const MaterialID m( createMaterial( real(1000), real(0.9), real(0), real(0), real(0.25), real(300), real(1e5), real(10), real(10) ) );
      const real w( 0.5e-5 ), gap( 5e-9 );
      SphereID p = createSphere( 1, Vec3( -kR - real(0.5) * gap, 0, 0 ), kR, m );
      SphereID q = createSphere( 2, Vec3(  kR + real(0.5) * gap, 0, 0 ), kR, m );
      p->setLinearVel( Vec3( w, 0, 0 ) ); q->setLinearVel( Vec3( -w, 0, 0 ) );
      world->simulationStep( real(2.5e-5) );
      const RestitutionPassStats c( cs->getRestitutionPassStats() );
      std::printf( "candidate without compression: candidates %zu, armed %zu, stop %d, impulse %.3e\n",
                   c.candidates, c.armedRows, c.stop, static_cast<double>( c.restitutionImpulse ) );
      expect( c.candidates == 1 && c.armedRows == 0 && c.stop == 1 && c.restitutionImpulse == real(0),
              "a candidate without compression is not armed, the pass is skipped, nothing is booked" );
      cs->setRestitution( false );
   }
   else {
      (void)cs;
      std::printf( "pe_restitution_stoprule_test: no configurable stop rule, skipped\n" );
   }
}

int main()
{
   if( !HasStopRule<CollisionSystemType>::value ) return 77;
   CollisionSystemID cs = theCollisionSystem();
   run( cs );
   if( failures == 0 ) std::printf( "pe_restitution_stoprule_test: all checks passed\n" );
   else std::printf( "pe_restitution_stoprule_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
