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
 *       it within the step (1e-5 m/s, dt 2.5e-5 s): no impulse with either scheme;
 *    5. a remote impact leaves a disconnected preloaded support network (a sphere squeezed between
 *       the floor and a fixed lid with opposing push-out targets, a resting pile) bit for bit
 *       unchanged, also the floor impulse (EL);
 *    6. an orthogonal impact on a loaded, sticking floor contact: the floor friction still acts
 *       in the pass (the floor contact point does not slip after the impact step);
 *    7. in-solve with a positive gap and a body force: a braking force that keeps the gap open gives
 *       no impulse, an accelerating force that closes it gives the Newton target (the decision uses
 *       the free motion after the body caching, also in HCSITS).
 *
 *  Against 1d030e7 (the EL port before review round 1) cases 1-4 fail: the resting contacts are
 *  skipped by every sweep (1, 2), the old tangential impulse survives an unloading (3), the
 *  in-solve target replaces the gap (4, in-solve).
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/support/el_optional_api.h>

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

template< typename T, typename = void > struct HasStopRule : std::false_type {};
template< typename T > struct HasStopRule< T, std::void_t< decltype( std::declval<T&>().setRestitutionCheckInterval( size_t(1) ) ) > > : std::true_type {};

//! The regression setting of the stop rule: up to 100 sweeps, tested after every sweep, relative
//! tolerance 1e-9 (the build before the stop rule became configurable always ran that way).
template< typename Handle > void strictStopRule( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasStopRule<CS>::value ) {
      cs->setRestitutionMaxIterations( 100 );
      cs->setRestitutionCheckInterval( 1 );
      cs->setRestitutionTolerance( real(0), real(1e-9) );
   }
   else (void)cs;
}

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
      strictStopRule( cs );
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

//=================================================================================================
// 3. Frictional unloading
//=================================================================================================

static void caseUnloading( CollisionSystemID cs )
{
   const real e( 0.9 ), u( 1 ), dt( 1e-4 ), rB( 0.03 ), rA( kR ), angle( 20.0 * M_PI / 180.0 );
   int lifted( 0 );
   for( real gEff : { real(40), real(80), real(120), real(160), real(200), real(260), real(340) } ) {
      applySwitches( cs, true, true );
      WorldID world = freshWorld();
      const MaterialID m( material( real(1000), e, real(0.25) ) );   // pair mu = 0.5
      createPlane( 1, 0, 0, 1, 0, m );
      SphereID B = createSphere( 2, Vec3( 0, 0, rB ), rB, m );
      const Vec3 dir( -std::cos( angle ), 0, -std::sin( angle ) );   // from B's centre to A's at contact
      const Vec3 contactCentre( B->getPosition() + ( rA + rB ) * dir );
      SphereID A = createSphere( 3, contactCentre - Vec3( u * dt * real(5.9), 0, 0 ), rA, m );
      const Vec3 FB( 0, 0, -B->getMass() * gEff );
      for( int i = 0; i < 50; ++i ) { B->addForce( FB ); world->simulationStep( dt ); }   // B rests on the floor (A not yet moving)
      A->setLinearVel( Vec3( u, 0, 0 ) );
      bool found( false );
      for( int i = 0; i < 40 && !found; ++i ) {
         const real px0( A->getMass() * A->getLinearVel()[0] + B->getMass() * B->getLinearVel()[0] );
         const real vA0( A->getLinearVel()[0] );
         B->addForce( FB );
         world->simulationStep( dt );
         if( A->getLinearVel()[0] == vA0 ) continue;
         found = true;
         const real px1( A->getMass() * A->getLinearVel()[0] + B->getMass() * B->getLinearVel()[0] );
         const real vzB( B->getLinearVel()[2] );
         std::printf( "unloading, pressing %5.0f m/s^2: v_z(B) after %+.4e, x momentum change %+.3e (A's %.3e)\n",
                      static_cast<double>( gEff ), static_cast<double>( vzB ), static_cast<double>( px1 - px0 ),
                      static_cast<double>( A->getMass() * u ) );
         if( vzB > real(1e-9) ) {
            ++lifted;
            expect( std::fabs( px1 - px0 ) <= real(1e-12) * A->getMass() * u,
                    "unloading: a floor contact unloaded by the rebound keeps no tangential impulse (x momentum conserved)" );
         }
      }
      expect( found, "unloading: an impact happened" );
   }
   expect( lifted > 0, "unloading: at least one pressing force at which the rebound lifts the large sphere" );
}

//=================================================================================================
// 4. Positive gap below the contact threshold
//=================================================================================================

static void casePositiveGap( CollisionSystemID cs )
{
   const real dt( 2.5e-5 ), w( 0.5e-5 ), gap( 5e-9 );
   for( int postPass = 0; postPass < 2; ++postPass ) {
      applySwitches( cs, true, false, postPass == 1 );
      WorldID world = freshWorld();
      const MaterialID m( material( real(1000), real(0.9), real(0) ) );
      SphereID a = createSphere( 1, Vec3( -kR - real(0.5) * gap, 0, 0 ), kR, m );
      SphereID b = createSphere( 2, Vec3(  kR + real(0.5) * gap, 0, 0 ), kR, m );
      a->setLinearVel( Vec3(  w, 0, 0 ) );
      b->setLinearVel( Vec3( -w, 0, 0 ) );
      world->simulationStep( dt );
      const real gapAfter( b->getPosition()[0] - a->getPosition()[0] - real(2) * kR );
      std::printf( "positive gap 5e-9 m, %s: v after %.6e %.6e (before %.1e %.1e), gap after %.3e\n",
                   postPass ? "post-pass" : "in-solve ", static_cast<double>( a->getLinearVel()[0] ), static_cast<double>( b->getLinearVel()[0] ),
                   static_cast<double>( w ), static_cast<double>( -w ), static_cast<double>( gapAfter ) );
      expect( a->getLinearVel()[0] == w && b->getLinearVel()[0] == -w,
              std::string( "positive gap (" ) + ( postPass ? "post-pass" : "in-solve" ) + "): a gap the step does not close gives no impulse" );
   }
}

//=================================================================================================
// 5. A remote impact leaves a disconnected preloaded support network untouched
//=================================================================================================

struct SupportState { std::vector<Vec3> x, v, w; real wall[2][3]; };

//! A sphere squeezed between the floor plane and a fixed box above it (overlapping both by 1e-5 m:
//! opposing push-out targets the compression solve cannot both meet) and a resting pile on the
//! floor (global plane), gravity as a force, Baumgarte; with or without a pair of small spheres
//! impacting head-on 0.5 m away (floating, no force). Returns the state of the support network and
//! the accumulated floor impulse (EL solver; zero for HCSITS).
static SupportState runSupportNetwork( CollisionSystemID cs, bool remoteImpact )
{
   applySwitches( cs, true, false );
   WorldID world = freshWorld();
   elResetLubricationVirial( *cs );
   const real g( 9.81 ), dt( 2.5e-5 ), overlap( 1e-5 );
   const MaterialID m( material( real(1000), real(0.9), real(0.15) ) );
   pe_GLOBAL_SECTION { createPlane( 1, 0, 0, 1, 0, m ); }
   const real xs( 0.2 ), boxH( 0.01 );
   BoxID lid = createBox( 2, Vec3( xs, 0, real(2) * kR - overlap + real(0.5) * boxH ), Vec3( real(0.03), real(0.03), boxH ), m );
   lid->setFixed( true );
   std::vector<SphereID> net;
   net.push_back( createSphere( 10, Vec3( xs, 0, kR - real(0.5) * overlap ), kR, m ) );   // squeezed
   for( int ix = 0; ix < 2; ++ix ) for( int iy = 0; iy < 2; ++iy )
      net.push_back( createSphere( 11 + 2 * ix + iy, Vec3( ( ix - real(0.5) ) * real(2) * kR, ( iy - real(0.5) ) * real(2) * kR, kR ), kR, m ) );
   net.push_back( createSphere( 15, Vec3( 0, 0, kR + std::sqrt( real(2) ) * kR ), kR, m ) );
   SphereID a( nullptr ), b( nullptr );
   if( remoteImpact ) {
      const real u( 0.5 ), rr( real(0.25) * kR ), gap( 2 * u * dt * real(5.9) );
      a = createSphere( 20, Vec3( real(0.5) - rr - real(0.5) * gap, 0, real(0.05) ), rr, m );
      b = createSphere( 21, Vec3( real(0.5) + rr + real(0.5) * gap, 0, real(0.05) ), rr, m );
      a->setLinearVel( Vec3( u, 0, 0 ) ); b->setLinearVel( Vec3( -u, 0, 0 ) );
   }
   for( int i = 0; i < 20; ++i ) {
      for( SphereID sp : net ) sp->addForce( Vec3( 0, 0, -sp->getMass() * g ) );
      world->simulationStep( dt );
   }
   SupportState st;
   for( SphereID sp : net ) { st.x.push_back( sp->getPosition() ); st.v.push_back( sp->getLinearVel() ); st.w.push_back( sp->getAngularVel() ); }
   elWallContactImpulse( *cs, 0, st.wall[0] );
   elWallContactImpulse( *cs, 1, st.wall[1] );
   if( remoteImpact ) {
      std::printf( "remote pair after the impact: v = %.6f %.6f\n", static_cast<double>( a->getLinearVel()[0] ), static_cast<double>( b->getLinearVel()[0] ) );
      expect( a->getLinearVel()[0] < -real(0.4) && b->getLinearVel()[0] > real(0.4), "remote impact: the remote pair bounces (the pass runs)" );
   }
   return st;
}

static void caseRemoteImpact( CollisionSystemID cs )
{
   const SupportState quiet( runSupportNetwork( cs, false ) );
   const SupportState hit( runSupportNetwork( cs, true ) );
   bool same( true );
   real worst( 0 );
   for( size_t k = 0; k < quiet.v.size(); ++k )
      for( int c = 0; c < 3; ++c ) {
         same = same && quiet.x[k][c] == hit.x[k][c] && quiet.v[k][c] == hit.v[k][c] && quiet.w[k][c] == hit.w[k][c];
         worst = std::max( worst, std::fabs( quiet.v[k][c] - hit.v[k][c] ) );
      }
   bool wallSame( true );
   for( int w = 0; w < 2; ++w ) for( int c = 0; c < 3; ++c ) wallSame = wallSame && quiet.wall[w][c] == hit.wall[w][c];
   std::printf( "remote impact: support network %s (largest velocity difference %.3e), floor impulse %.12e / %.12e (without / with the impact)\n",
                same ? "bitwise unchanged" : "CHANGED", static_cast<double>( worst ),
                static_cast<double>( quiet.wall[0][2] ), static_cast<double>( hit.wall[0][2] ) );
   expect( same, "remote impact: the disconnected support network is bit for bit the network without the impact" );
   expect( wallSame, "remote impact: the floor impulse is bit for bit the one without the impact" );
}

//=================================================================================================
// 6. Orthogonal impact on a loaded, sticking floor contact
//=================================================================================================

//! A small sphere hits a large one, pressed onto a frictional floor (pair mu = 0.5), horizontally
//! at the height of its centre: the floor row keeps its normal load (d = 0 in the pass), but the
//! rebound changes its slip. The floor friction must still act in the pass: with ample friction
//! the floor contact point of the large sphere does not slip after the impact step.
static void caseOrthogonalImpact( CollisionSystemID cs )
{
   applySwitches( cs, true, true );
   WorldID world = freshWorld();
   const real u( 1 ), dt( 1e-4 ), rB( 0.03 ), rA( kR ), gEff( 2000 );
   const MaterialID m( material( real(1000), real(0.9), real(0.25) ) );
   createPlane( 1, 0, 0, 1, 0, m );
   SphereID B = createSphere( 2, Vec3( 0, 0, rB ), rB, m );
   SphereID A = createSphere( 3, Vec3( -( rA + rB ) - u * dt * real(5.9), 0, rB ), rA, m );
   const Vec3 FB( 0, 0, -B->getMass() * gEff );
   for( int i = 0; i < 20; ++i ) { B->addForce( FB ); world->simulationStep( dt ); }   // B loaded on the floor
   A->setLinearVel( Vec3( u, 0, 0 ) );
   bool found( false );
   for( int i = 0; i < 40 && !found; ++i ) {
      const real vA0( A->getLinearVel()[0] );
      B->addForce( FB );
      world->simulationStep( dt );
      if( A->getLinearVel()[0] == vA0 ) continue;
      found = true;
      const real slip( B->getLinearVel()[0] - B->getAngularVel()[1] * rB );   // floor contact point, along x
      std::printf( "orthogonal impact on a loaded floor contact: A %.6f, B v_x %.6e omega_y %.6e, floor slip %.3e\n",
                   static_cast<double>( A->getLinearVel()[0] ), static_cast<double>( B->getLinearVel()[0] ),
                   static_cast<double>( B->getAngularVel()[1] ), static_cast<double>( slip ) );
      expect( A->getLinearVel()[0] < -real(0.5) * u, "orthogonal impact: A rebounds" );
      expect( std::fabs( slip ) <= real(1e-6) * u, "orthogonal impact: the loaded floor contact still sticks after the pass" );
   }
   expect( found, "orthogonal impact: an impact happened" );
}

//=================================================================================================
// 7. In-solve: the gap decision uses the free motion of the step (forces included)
//=================================================================================================

static void casePositiveGapForces( CollisionSystemID cs )
{
   // gap 9e-9 m (below the contact threshold 1e-8), dt 2.5e-5 s. The approach speeds stay above the
   // EL resting bound 2 |dv_free| so that both solvers arm the contact.
   const real dt( 2.5e-5 ), gap( 9e-9 );
   for( int accelerate = 0; accelerate < 2; ++accelerate ) {
      applySwitches( cs, true, false, false );   // in-solve
      WorldID world = freshWorld();
      const MaterialID m( material( real(1000), real(0.9), real(0) ) );
      SphereID a = createSphere( 1, Vec3( -kR - real(0.5) * gap, 0, 0 ), kR, m );
      SphereID b = createSphere( 2, Vec3(  kR + real(0.5) * gap, 0, 0 ), kR, m );
      // braking: pre-step 2 x 2e-4 m/s closes the gap in the step (1e-8 m), a force of -1.5e-4 per
      // body leaves 2 x 5e-5 m/s (2.5e-9 m, the gap stays open); accelerating: pre-step 2 x 1.5e-4
      // (7.5e-9 m, open), a force of +1e-4 per body gives 2 x 2.5e-4 (1.25e-8 m, closes)
      const real w( accelerate ? real(1.5e-4) : real(2e-4) ), dv( accelerate ? real(1e-4) : -real(1.5e-4) );
      a->setLinearVel( Vec3(  w, 0, 0 ) );
      b->setLinearVel( Vec3( -w, 0, 0 ) );
      a->addForce( Vec3(  a->getMass() * dv / dt, 0, 0 ) );
      b->addForce( Vec3( -b->getMass() * dv / dt, 0, 0 ) );
      world->simulationStep( dt );
      const real rel( b->getLinearVel()[0] - a->getLinearVel()[0] );   // > 0: separating
      std::printf( "in-solve, positive gap, %s force: v after %.9e %.9e, relative %.6e\n", accelerate ? "accelerating" : "braking     ",
                   static_cast<double>( a->getLinearVel()[0] ), static_cast<double>( b->getLinearVel()[0] ), static_cast<double>( rel ) );
      if( accelerate )
         expect( rel > real(0), "in-solve: a gap the forced motion closes in the step bounces (Newton target)" );
      else
         expect( std::fabs( a->getLinearVel()[0] - ( w + dv ) ) < real(1e-15) && std::fabs( b->getLinearVel()[0] + ( w + dv ) ) < real(1e-15),
                 "in-solve: a gap the braked motion leaves open gives no impulse (free velocity kept)" );
   }
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
   caseUnloading( cs );
   casePositiveGap( cs );
   caseRemoteImpact( cs );
   caseOrthogonalImpact( cs );
   casePositiveGapForces( cs );
   applySwitches( cs, false, false );
   if( failures == 0 ) std::printf( "pe_restitution_support_test: all checks passed\n" );
   else std::printf( "pe_restitution_support_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
