//=================================================================================================
/*!
 *  \file tests/interface/pe_hcsits_restitution_test.cpp
 *  \brief Newton restitution of HardContactSemiImplicitTimesteppingSolvers (setRestitution()).
 *
 *  With the switch off every contact is inelastic: a dropped sphere stops dead. With it on, an
 *  impacting contact gets the Newton impact law gdot_n^+ = -e gdot_n^- as its normal target, with
 *  e the pairwise coefficient of restitution of the two materials; the tangential (Coulomb) impulse
 *  is bounded by mu times that larger normal impulse, so spin and tangential velocity exchange at
 *  the bounce like in a rigid-body impact. Resting contacts are excluded by an impact-speed
 *  threshold (max( user value, 2 |g| dt )).
 *
 *  Asserted (gravity through World::setGravity(), which this solver honours):
 *    1. ITTF drop test: a 40 mm sphere released with its bottom 0.305 m above a plane, e = 0.9,
 *       dt = 1e-4, split impulse off and on -> apex of the first rebound at e^2 * 0.305 = 0.247 m,
 *       within 3 %;
 *    2. the same drop with the switch off (split impulse on) -> the sphere rises less than 5 mm
 *       after the contact (with the Baumgarte term the push-out is an artificial ~0.1 m rebound,
 *       printed, not asserted);
 *    3. a sphere resting on the plane with the switch on, 1 s at dt = 1e-3 -> |v_z| < 1e-3 and
 *       the bottom stays within 1e-4 of the plane (no micro-bounces);
 *    4. a sliding backspin impact: v = (1, 0, -3), omega = (0, -400, 0), mu = 0.3, e = 0.9 -> the
 *       contact slides throughout (relative tangential velocity 9 m/s, the impulse removes 3.5 x
 *       1.71 = 6), so v_x' = v_x - mu (1 + e) |v_z| = -0.71 and v_z' = 2.7, each within 10 %;
 *       the horizontal velocity reverses.
 *
 *  Serial world setup, no MPI. Under a configured solver without the switch the test prints a
 *  note and returns 77 (CTest: skipped).
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>
#include <utility>

using namespace pe;

template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitution( true ) ) > > : std::true_type {};
typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

template< typename Handle > void applySwitches( Handle& cs, bool restitution, bool split )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasRestitution<CS>::value ) {
      cs->setRestitution( restitution );
      cs->setSplitImpulse( split );
   }
   else { (void)cs; (void)restitution; (void)split; }
}

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static const real kRadius( 0.02 );
static const real kG( 9.81 );

static MaterialID material()
{
   // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
   return createMaterial( real(80), real(0.9), real(0.15), real(0.15), real(0.25), real(300), real(1e5), real(10), real(10) );
}

struct Scene {
   SphereID ball;
   MaterialID mat;
};

static Scene build( real cor, real mu, const Vec3& pos, const Vec3& vel, const Vec3& avel )
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, -kG );
   Scene s;
   s.mat = material();
   Material::setRestitution( s.mat, s.mat, cor );
   Material::setStaticFriction( s.mat, s.mat, mu );
   Material::setDynamicFriction( s.mat, s.mat, mu );
   createPlane( 1, 0, 0, 1, 0, s.mat );
   s.ball = createSphere( 2, pos, kRadius, s.mat );
   s.ball->setLinearVel( vel );
   s.ball->setAngularVel( avel );
   return s;
}

//! Runs until the first upward velocity after a downward one, then until the apex; returns the
//! bottom height at the apex (or -1 if there was no bounce within maxSteps).
static real firstReboundApex( SphereID ball, real dt, int maxSteps )
{
   bool bounced = false;
   real apex( -1 );
   for( int i = 0; i < maxSteps; ++i ) {
      const real vzBefore( ball->getLinearVel()[2] );
      theWorld()->simulationStep( dt );
      const real vz( ball->getLinearVel()[2] );
      if( !bounced && vzBefore < 0 && vz >= 0 ) bounced = true;
      if( bounced ) {
         apex = std::max( apex, ball->getPosition()[2] - kRadius );
         if( vz < 0 ) return apex;   // past the apex
      }
   }
   return apex;
}

int main()
{
   if( !HasRestitution<CollisionSystemType>::value ) {
      std::printf( "pe_hcsits_restitution_test: the configured collision system has no restitution switch, skipped\n" );
      return 77;
   }
   CollisionSystemID cs = theCollisionSystem();
   char what[256];

   // 1. ITTF drop test, restitution on, split impulse off and on.
   const real h0( 0.305 ), e( 0.9 );
   for( int split = 0; split < 2; ++split ) {
      applySwitches( cs, true, split == 1 );
      Scene s = build( e, real(0.3), Vec3( 0, 0, h0 + kRadius ), Vec3(), Vec3() );
      const real apex( firstReboundApex( s.ball, real(1e-4), 20000 ) );
      const real expected( e * e * h0 );
      std::printf( "drop test, split %s: rebound apex %.4f m (expected %.4f)\n", split ? "on " : "off",
                   static_cast<double>( apex ), static_cast<double>( expected ) );
      std::snprintf( what, sizeof( what ), "restitution on (split %s): rebound apex within 3%% of e^2 h0", split ? "on" : "off" );
      expect( std::fabs( apex - expected ) < real(0.03) * expected, what );
   }

   // 2. Restitution off: no rebound. Split impulse on: with the Baumgarte term instead, the
   //    push-out of the one-step penetration (erp * |v_z| dt / dt = 0.7 |v_z|) is an artificial
   //    rebound of about 0.1 m (printed for information, not asserted; see the solver comment).
   {
      applySwitches( cs, false, false );
      Scene s0 = build( e, real(0.3), Vec3( 0, 0, h0 + kRadius ), Vec3(), Vec3() );
      const real apexBaumgarte( firstReboundApex( s0.ball, real(1e-4), 20000 ) );
      applySwitches( cs, false, true );
      Scene s = build( e, real(0.3), Vec3( 0, 0, h0 + kRadius ), Vec3(), Vec3() );
      const real apex( firstReboundApex( s.ball, real(1e-4), 20000 ) );
      std::printf( "drop test, restitution off: rebound apex %.4f m (split impulse), %.4f m (Baumgarte artifact)\n",
                   static_cast<double>( apex ), static_cast<double>( apexBaumgarte ) );
      expect( apex < real(0.005), "restitution off (split impulse): the sphere does not rebound (apex < 5 mm)" );
   }

   // 3. Rest: no micro-bounces.
   {
      applySwitches( cs, true, false );
      Scene s = build( e, real(0.3), Vec3( 0, 0, kRadius ), Vec3(), Vec3() );
      real maxVz( 0 ), maxDev( 0 );
      for( int i = 0; i < 1000; ++i ) {
         theWorld()->simulationStep( real(1e-3) );
         maxVz  = std::max( maxVz,  std::fabs( s.ball->getLinearVel()[2] ) );
         maxDev = std::max( maxDev, std::fabs( s.ball->getPosition()[2] - kRadius ) );
      }
      std::printf( "rest: max |v_z| %.3e, max bottom deviation %.3e\n", static_cast<double>( maxVz ), static_cast<double>( maxDev ) );
      expect( maxVz < real(1e-3) && maxDev < real(1e-4), "rest: a resting sphere stays at rest with restitution on" );
   }

   // 4. Sliding backspin impact.
   {
      applySwitches( cs, true, true );
      const real mu( 0.3 ), vx( 1 ), vz( -3 ), w( -400 );
      Scene s = build( e, mu, Vec3( 0, 0, kRadius + real(0.002) ), Vec3( vx, 0, vz ), Vec3( 0, w, 0 ) );
      bool bounced = false;
      for( int i = 0; i < 200 && !bounced; ++i ) {
         theWorld()->simulationStep( real(1e-4) );
         bounced = s.ball->getLinearVel()[2] > 0;
      }
      const Vec3 v( s.ball->getLinearVel() );
      const Vec3 av( s.ball->getAngularVel() );
      // At contact the fall has added a little to |v_z| (2 mm plus gravity over the steps).
      const real vzImpact( -std::sqrt( vz * vz + 2 * kG * real(0.002) ) );
      const real vxExpected( vx - mu * ( 1 + e ) * std::fabs( vzImpact ) );
      const real vzExpected( -e * vzImpact );
      std::printf( "backspin impact: v' = (%.3f, %.3f, %.3f) expected (%.3f, 0, %.3f), omega_y' = %.1f\n",
                   static_cast<double>( v[0] ), static_cast<double>( v[1] ), static_cast<double>( v[2] ),
                   static_cast<double>( vxExpected ), static_cast<double>( vzExpected ), static_cast<double>( av[1] ) );
      expect( bounced, "backspin impact: the sphere bounced" );
      expect( std::fabs( v[0] - vxExpected ) < real(0.1) * std::fabs( vxExpected ), "backspin impact: v_x' = v_x - mu (1+e) |v_z| within 10%" );
      expect( std::fabs( v[2] - vzExpected ) < real(0.1) * vzExpected, "backspin impact: v_z' = e |v_z| within 10%" );
      expect( v[0] < 0, "backspin impact: the horizontal velocity reverses" );
   }

   applySwitches( cs, false, false );
   if( failures == 0 ) std::printf( "pe_hcsits_restitution_test: all checks passed\n" );
   else std::printf( "pe_hcsits_restitution_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
