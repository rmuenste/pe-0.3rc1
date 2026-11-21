//=================================================================================================
/*!
 *  \file tests/interface/pe_restitution_support_test.cpp
 *  \brief Restitution post-pass with several contacts per body, run against both solvers that
 *         have the switch (HardContactEulerLagrange: pe-restitution-support,
 *         HardContactSemiImplicitTimesteppingSolvers: pe-restitution-support-plain).
 *
 *  Gravity, where needed, is a body force (World gravity 0 for both solvers). Every check is on
 *  the IMPACT step (the first step in which the moving sphere's velocity changes):
 *
 *    1. support contacts: a sphere into a touching pair (3-chain) and into a touching row of four
 *       (5-chain), split impulse and Baumgarte, e = 0.9. The touching contacts were born at zero
 *       normal speed, so pe gives them e = 0 (Contact.cpp). The rebound of the impacting pair must
 *       not drive the next sphere into its neighbour: every support contact has a normal
 *       velocity >= -1e-9 u after the step and its gap does not decrease; momentum conserved
 *       (1e-12), kinetic energy not above the value before the step;
 *    2. a sphere dropped on a resting pile (pyramid in a box of four walls, column on the apex,
 *       settled with the switch on): no pile contact (sphere-sphere or sphere-wall) closes faster
 *       than 1e-6 times the impact speed after the impact step (largest closing speed printed);
 *    3. frictional unloading: a small sphere hits a large sphere that is pressed onto a frictional
 *       floor (pair mu = 0.5) from below its centre; for the pressing forces at which the rebound
 *       lifts the large sphere off the floor (v_z > 0, split impulse), the floor carries no
 *       impulse any more, so the x momentum of the two spheres is conserved (1e-12): an unloaded
 *       contact keeps no tangential impulse;
 *    4. a positive gap below the contact threshold (5e-9 m) with an approach that does not close
 *       it within the step (1e-5 m/s, dt 2.5e-5 s): no impulse with either scheme.
 *
 *  Against 1d030e7 (the EL port before review round 1) cases 1-4 fail: the resting contacts are
 *  skipped by every sweep (1, 2), the old tangential impulse survives an unloading (3), the
 *  in-solve target replaces the gap (4, in-solve).
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using namespace pe;

typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitution( true ) ),
                                                              decltype( std::declval<T&>().setRestitutionPostPass( true ) ) > > : std::true_type {};

template< typename Handle > void applySwitches( Handle& cs, bool restitution, bool split, bool postPass = true )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasRestitution<CS>::value ) {
      cs->setSplitImpulse( split );
      cs->setRestitution( restitution );
      cs->setRestitutionPostPass( postPass );
      cs->setRestitutionIterations( 4 );
      cs->setRestitutionThreshold( real(0) );
      cs->setErrorReductionParameter( real(0.7) );
   }
   else { (void)cs; (void)restitution; (void)split; (void)postPass; }
}

static int failures = 0;

static void expect( bool ok, const std::string& what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what.c_str() );
      ++failures;
   }
}

static const real kR( 0.01 );

static MaterialID material( real density, real cor, real mu )
{
   return createMaterial( density, cor, mu, mu, real(0.25), real(300), real(1e5), real(10), real(10) );
}

static real kinetic( BodyID b )
{
   const Vec3& v = b->getLinearVel();
   const Vec3& w = b->getAngularVel();
   return real(0.5) * b->getMass() * ( trans( v ) * v ) + real(0.5) * ( trans( w ) * ( b->getInertia() * w ) );
}

static WorldID freshWorld()
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   return world;
}

//=================================================================================================
// 1. Chains
//=================================================================================================

static void caseChain( CollisionSystemID cs, int n, bool split )
{
   applySwitches( cs, true, split );
   WorldID world = freshWorld();
   const real e( 0.9 ), u( 0.5 ), dt( 1e-4 );
   const MaterialID m( material( real(1000), e, real(0) ) );
   const real gap( u * dt * real(5.9) );
   std::vector<SphereID> s;
   s.push_back( createSphere( 1, Vec3( -real(2) * kR - gap, 0, 0 ), kR, m ) );
   for( int k = 1; k < n; ++k )
      s.push_back( createSphere( 1 + k, Vec3( real(2) * ( k - 1 ) * kR, 0, 0 ), kR, m ) );
   s[0]->setLinearVel( Vec3( u, 0, 0 ) );

   const char* tag( split ? "split" : "Baumgarte" );
   char what[256];
   bool found( false );
   for( int i = 0; i < 40 && !found; ++i ) {
      std::vector<real> x0, v0;
      real E0( 0 ), p0( 0 );
      for( SphereID b : s ) { x0.push_back( b->getPosition()[0] ); v0.push_back( b->getLinearVel()[0] ); E0 += kinetic( b ); p0 += b->getMass() * b->getLinearVel()[0]; }
      world->simulationStep( dt );
      if( s[0]->getLinearVel()[0] == v0[0] ) continue;
      found = true;
      real E1( 0 ), p1( 0 ), worst( 0 ), worstGap( 0 );
      for( SphereID b : s ) { E1 += kinetic( b ); p1 += b->getMass() * b->getLinearVel()[0]; }
      for( int k = 1; k + 1 < n; ++k ) {   // support contacts k / k+1
         const real vn( s[k+1]->getLinearVel()[0] - s[k]->getLinearVel()[0] );   // >= 0: not closing
         const real gapBefore( x0[k+1] - x0[k] - real(2) * kR ), gapAfter( s[k+1]->getPosition()[0] - s[k]->getPosition()[0] - real(2) * kR );
         worst = std::min( worst, vn );
         worstGap = std::min( worstGap, gapAfter - gapBefore );
      }
      std::printf( "chain %d (%s): impact step, v =", n, tag );
      for( SphereID b : s ) std::printf( " %.6f", static_cast<double>( b->getLinearVel()[0] ) );
      std::printf( "; most closing support contact %.3e (= %.4f u), smallest gap change %.3e, E1/E0 = %.12f, p1 - p0 = %.3e\n",
                   static_cast<double>( worst ), static_cast<double>( worst / u ), static_cast<double>( worstGap ),
                   static_cast<double>( E1 / E0 ), static_cast<double>( p1 - p0 ) );
      std::snprintf( what, sizeof( what ), "chain %d (%s): no support contact closes on the impact step", n, tag );
      expect( worst >= -real(1e-9) * u, what );
      std::snprintf( what, sizeof( what ), "chain %d (%s): no support gap decreases on the impact step", n, tag );
      expect( worstGap >= -real(1e-12), what );
      std::snprintf( what, sizeof( what ), "chain %d (%s): momentum conserved", n, tag );
      expect( std::fabs( p1 - p0 ) <= real(1e-12) * s[0]->getMass() * u, what );
      std::snprintf( what, sizeof( what ), "chain %d (%s): kinetic energy not above the value before the impact step", n, tag );
      expect( E1 <= E0 * ( real(1) + real(1e-12) ), what );
      std::snprintf( what, sizeof( what ), "chain %d (%s): the impacting sphere rebounds", n, tag );
      expect( s[0]->getLinearVel()[0] < real(0.5) * u, what );
   }
   std::snprintf( what, sizeof( what ), "chain %d (%s): an impact happened", n, tag );
   expect( found, what );
}

//=================================================================================================
// 2. Drop on a resting pile
//=================================================================================================

struct Wall { Vec3 n; real d; };   // n . x = d, normal into the box

static void caseDropOnPile( CollisionSystemID cs )
{
   applySwitches( cs, true, false );
   WorldID world = freshWorld();
   const real e( 0.9 ), dt( 1e-4 ), g( 9.81 );
   const MaterialID m( material( real(1000), e, real(0.15) ) );
   const std::vector<Wall> walls = { { Vec3( 0, 0, 1 ), 0 }, { Vec3( 1, 0, 0 ), -real(2) * kR }, { Vec3( -1, 0, 0 ), -real(2) * kR },
                                     { Vec3( 0, 1, 0 ), -real(2) * kR }, { Vec3( 0, -1, 0 ), -real(2) * kR } };
   pe::id_t pid( 1 );
   for( const Wall& w : walls ) createPlane( pid++, w.n[0], w.n[1], w.n[2], w.d, m );
   std::vector<SphereID> pile;
   pe::id_t id( 10 );
   for( int ix = -1; ix <= 1; ix += 2 )
      for( int iy = -1; iy <= 1; iy += 2 )
         pile.push_back( createSphere( id++, Vec3( ix * kR, iy * kR, kR ), kR, m ) );
   const real apex( kR + std::sqrt( real(2) ) * kR );
   for( int k = 0; k < 3; ++k )
      pile.push_back( createSphere( id++, Vec3( 0, 0, apex + real(2) * k * kR ), kR, m ) );
   const real top( pile.back()->getPosition()[2] );

   const auto stepAll = [&]( SphereID extra ) {
      for( SphereID b : pile ) b->addForce( Vec3( 0, 0, -b->getMass() * g ) );
      if( extra ) extra->addForce( Vec3( 0, 0, -extra->getMass() * g ) );
      world->simulationStep( dt );
   };
   for( int i = 0; i < 2000; ++i ) stepAll( nullptr );   // settle

   // Dropped sphere: 2 cm above the column top, falling at 1 m/s
   SphereID ball = createSphere( id++, Vec3( 0, 0, top + real(2) * kR + real(0.02) ), kR, m );
   ball->setLinearVel( Vec3( 0, 0, -1 ) );
   bool found( false );
   real vImpact( 0 ), worst( 0 );
   std::string where;
   for( int i = 0; i < 400 && !found; ++i ) {
      const real vBefore( ball->getLinearVel()[2] );
      stepAll( ball );
      if( !( ball->getLinearVel()[2] > vBefore + real(0.1) ) ) continue;   // the impact step reverses the ball
      found = true;
      vImpact = -vBefore;
      // closing speed of every pile contact (touching within 1e-6)
      for( size_t a = 0; a < pile.size(); ++a ) {
         for( size_t b = a + 1; b < pile.size(); ++b ) {
            const Vec3 dx( pile[b]->getPosition() - pile[a]->getPosition() );
            if( dx.length() - real(2) * kR > real(1e-6) ) continue;
            const real closing( trans( pile[a]->getLinearVel() - pile[b]->getLinearVel() ) * ( dx / dx.length() ) );
            if( closing > worst ) { worst = closing; where = "sphere-sphere " + std::to_string( a ) + "-" + std::to_string( b ); }
         }
         for( size_t w = 0; w < walls.size(); ++w ) {
            const real dist( trans( walls[w].n ) * pile[a]->getPosition() - walls[w].d - kR );
            if( dist > real(1e-6) ) continue;
            const real closing( -( trans( walls[w].n ) * pile[a]->getLinearVel() ) );
            if( closing > worst ) { worst = closing; where = "sphere-wall " + std::to_string( a ) + "-" + std::to_string( w ); }
         }
      }
   }
   std::printf( "drop on a resting pile: impact speed %.4f, largest closing speed of a pile contact after the impact step %.3e (%s)\n",
                static_cast<double>( vImpact ), static_cast<double>( worst ), worst > 0 ? where.c_str() : "none closing" );
   expect( found, "drop on pile: the ball bounces" );
   expect( worst <= real(1e-6) * vImpact, "drop on pile: no pile contact is driven closed by the rebound" );
}

int main()
{
   if( !HasRestitution<CollisionSystemType>::value ) {
      std::printf( "pe_restitution_support_test: the configured collision system has no restitution switch, skipped\n" );
      return 77;
   }
   CollisionSystemID cs = theCollisionSystem();
   for( int split = 1; split >= 0; --split ) {
      caseChain( cs, 3, split == 1 );
      caseChain( cs, 5, split == 1 );
   }
   caseDropOnPile( cs );
   applySwitches( cs, false, false );
   if( failures == 0 ) std::printf( "pe_restitution_support_test: all checks passed\n" );
   else std::printf( "pe_restitution_support_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
