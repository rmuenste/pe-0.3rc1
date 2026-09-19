//=================================================================================================
/*!
 *  \file tests/interface/pe_ellipsoid_settle_test.cpp
 *  \brief Dynamics smoke test: a prolate spheroid dropped on a plane settles broadside.
 *
 *  Exercises the full pipeline (HashGrids broad phase on the corrected ellipsoid bounding box,
 *  MaxContacts ellipsoid-plane / ellipsoid-ellipsoid contact generation, hard-contact response).
 *  A prolate spheroid (rho = 2, A = 0.5, B = C = 0.25) tilted 40 degrees is dropped under gravity
 *  onto a horizontal plane and run to rest. Asserted:
 *    (a) it never falls through the plane (lowest surface point stays above -1% of B);
 *    (b) it comes to rest broadside with its center at height B within 1 percent;
 *    (c) no NaN anywhere in the state.
 *  Then a second spheroid is dropped crosswise onto the resting one, offset along the first
 *  one's axis so that it deterministically rolls off (a smooth convex body on top of another is
 *  an unstable equilibrium, so a "stack" is not a testable end state). Asserted: the two
 *  centers never come closer than 2B (the ellipsoid-ellipsoid contact holds), the upper one is
 *  deflected sideways by the impact and ends at rest on the plane at height B, the lower one
 *  stays put, nothing goes through the plane, no NaN.
 *
 *  A rolling ellipse conserves energy under a single frictional hard contact (no rolling
 *  resistance), so the world damping is used to bring the bodies to rest; the rest heights are
 *  set by the contacts, not by the damping.
 *
 *  Linked against pe_static_lubstage_plain: the library built with
 *  pe_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers (the plain
 *  hard-contact pipeline, which applies gravity itself; the shipped HardContactEulerLagrange
 *  default leaves body forces to the CFD driver and would let nothing fall).
 */
//=================================================================================================

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool finite3( const Vec3& v )
{
   return std::isfinite( v[0] ) && std::isfinite( v[1] ) && std::isfinite( v[2] );
}

static bool stateFinite( BodyID b )
{
   const Quat& q( b->getQuaternion() );
   return finite3( b->getPosition() ) && finite3( b->getLinearVel() ) && finite3( b->getAngularVel() )
       && std::isfinite( q[0] ) && std::isfinite( q[1] ) && std::isfinite( q[2] ) && std::isfinite( q[3] );
}

static void printState( const char* tag, EllipsoidID e )
{
   const Vec3 p( e->getPosition() ), v( e->getLinearVel() ), w( e->getAngularVel() );
   const Rot3& R( e->getRotation() );
   const Vec3 axis( R[0], R[3], R[6] );
   const real low( e->support( Vec3( 0, 0, -1 ) )[2] );
   std::printf( "%s: pos=(%.6f,%.6f,%.6f) |v|=%.3e |w|=%.3e a-axis=(%.4f,%.4f,%.4f) lowest z=%.6f\n",
                tag, p[0], p[1], p[2], v.length(), w.length(), axis[0], axis[1], axis[2], low );
}

int main()
{
   WorldID world = theWorld();
   world->setGravity( 0, 0, real(-9.81) );
   world->setDamping( real(0.05) );   // velocity retention per second (damping^dt per step)

   MaterialID mat = createMaterial( "settle_test", real(2), real(0.1), real(0.3), real(0.2),
                                    real(0.2), real(200), real(1e5), real(10), real(10) );

   PlaneID plane = createPlane( 0, Vec3( 0, 0, 1 ), real(0), mat );
   plane->setFixed( true );

   const real A = real(0.5), B = real(0.25), C = real(0.25);
   const real dt = real(1e-3);

   EllipsoidID e1 = createEllipsoid( 1, Vec3( 0, 0, real(0.6) ), A, B, C, mat );
   e1->rotate( Vec3( 0, 1, 0 ), real(40) * M_PI / real(180) );   // 40 degrees tilt about y
   expect( std::fabs( e1->getMass() - real(2) * ( real(4)/real(3) ) * M_PI * A * B * C ) < real(1e-12), "mass rho*V" );

   printState( "e1 start", e1 );

   bool finiteAll = true;
   real minLowest = real(1e300);
   const int steps1 = 6000;
   for( int i=0; i<steps1; ++i ) {
      world->simulationStep( dt );
      if( !stateFinite( e1 ) ) { finiteAll = false; break; }
      minLowest = std::min( minLowest, e1->support( Vec3( 0, 0, -1 ) )[2] );
      if( ( i + 1 ) % 1000 == 0 ) {
         char tag[32];
         std::snprintf( tag, sizeof( tag ), "e1 step %5d", i + 1 );
         printState( tag, e1 );
      }
   }
   printState( "e1 final", e1 );
   std::printf( "e1: minimum lowest surface point over the run = %.6e\n", minLowest );

   expect( finiteAll, "(c) no NaN in the single-spheroid run" );
   expect( minLowest > -real(0.01) * B, "(a) spheroid never falls through the plane (lowest point > -1% B)" );
   expect( std::fabs( e1->getPosition()[2] - B ) <= real(0.01) * B, "(b) spheroid rests broadside: center height = B within 1%" );
   expect( e1->getLinearVel().length() < real(1e-3) && e1->getAngularVel().length() < real(1e-3), "(b) spheroid is at rest" );
   {
      const Rot3& R( e1->getRotation() );
      expect( std::fabs( R[6] ) < real(0.02), "(b) a-axis is horizontal (|axis.z| < 0.02)" );
   }

   // --- second spheroid dropped crosswise onto the first, offset along its axis --------------
   const Vec3 p1( e1->getPosition() );
   const Rot3& R1( e1->getRotation() );
   const Vec3 axis1( R1[0], R1[3], R1[6] );
   const Vec3 drop( p1 + axis1 * real(0.15) + Vec3( 0, 0, real(1.0) - p1[2] ) );
   EllipsoidID e2 = createEllipsoid( 2, drop, A, B, C, mat );
   e2->rotate( Vec3( 0, 0, 1 ), real(0.5) * M_PI );   // a-axis along y: crosswise to e1
   printState( "e2 start", e2 );

   bool finiteAll2 = true;
   real minLowest2 = real(1e300);
   real minSeparation = real(1e300);
   const int steps2 = 6000;
   for( int i=0; i<steps2; ++i ) {
      world->simulationStep( dt );
      if( !stateFinite( e1 ) || !stateFinite( e2 ) ) { finiteAll2 = false; break; }
      minLowest2 = std::min( minLowest2, std::min( e1->support( Vec3( 0, 0, -1 ) )[2], e2->support( Vec3( 0, 0, -1 ) )[2] ) );
      minSeparation = std::min( minSeparation, ( e2->getPosition() - e1->getPosition() ).length() );
      if( ( i + 1 ) % 1000 == 0 ) {
         char tag[32];
         std::snprintf( tag, sizeof( tag ), "e2 step %5d", i + 1 );
         printState( tag, e2 );
      }
   }
   printState( "e1 final(2)", e1 );
   printState( "e2 final", e2 );
   std::printf( "impact: minimum lowest surface point = %.6e, minimum center separation = %.6f (2B = %.6f)\n",
                minLowest2, minSeparation, real(2) * B );

   expect( finiteAll2, "(c) no NaN in the two-spheroid run" );
   expect( minLowest2 > -real(0.01) * B, "impact: nothing falls through the plane" );
   expect( minSeparation >= real(2) * B * ( real(1) - real(0.02) ), "impact: centers never closer than 2B (ellipsoid-ellipsoid contact holds)" );
   expect( ( e1->getPosition() - p1 ).length() < real(0.05), "impact: lower spheroid stays put" );
   expect( std::fabs( e1->getPosition()[2] - B ) <= real(0.01) * B, "impact: lower spheroid still at height B" );
   const Vec3 lateral( e2->getPosition() - drop );
   expect( std::sqrt( lateral[0]*lateral[0] + lateral[1]*lateral[1] ) > real(0.1), "impact: upper spheroid was deflected sideways by the lower one" );
   expect( std::fabs( e2->getPosition()[2] - B ) <= real(0.01) * B, "impact: upper spheroid ends at rest on the plane at height B" );
   expect( e2->getLinearVel().length() < real(1e-3) && e2->getAngularVel().length() < real(1e-3), "impact: upper spheroid is at rest" );

   if( failures == 0 ) {
      std::printf( "pe_ellipsoid_settle_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_ellipsoid_settle_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
