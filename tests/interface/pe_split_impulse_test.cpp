//=================================================================================================
/*!
 *  \file tests/interface/pe_split_impulse_test.cpp
 *  \brief Split-impulse position correction of HardContactEulerLagrange (setSplitImpulse()).
 *
 *  With the Baumgarte term a penetrating contact demands a separation velocity erp * depth / dt
 *  that stays in the bodies: an overlapping initial state launches them, a body landing with a
 *  one-step penetration bounces at restitution 0, and a smaller dt makes both worse. With split
 *  impulse the velocity solve only stops the approach and a second, normal-only solve on pseudo
 *  velocities moves the positions apart by erp * depth per step without adding momentum.
 *
 *  Gravity is applied as a force per step (the Euler-Lagrange solver leaves body forces to the
 *  driver). Asserted, each case with the switch off and on:
 *    1. a unit box 0.05 deep in the ground plane, at rest, dt = 1e-3: off -> after one step the box
 *       moves up at more than 10 m/s (0.7 * 0.05 / 1e-3 = 35); on -> |v_z| < 0.05 after one step
 *       and after 200 steps, and the overlap has shrunk to below 1e-4 after 30 steps (0.3 of it
 *       remains per step) while the box rests on the plane;
 *    2. a unit box dropped from 0.5 above the plane with restitution 0, dt = 2e-3: off -> it
 *       bounces back up by more than 0.02; on -> it rises less than 2e-3 after the first contact
 *       and comes to rest;
 *    3. a box 0.05 deep in the plane carrying a second box that exactly touches it: on -> after
 *       50 steps the lower box is out of the plane (bottom within 1e-4 of z = 0), the upper one
 *       still touches it (gap within 1e-4) and both are at rest (|v| < 0.05): the pseudo motion
 *       propagates through the touching contact;
 *    4. a 6-box tower built touching, 1 s with the switch on: the top box drifts less than 0.01
 *       and the kinetic energy stays below 1e-2 (the correction does not destabilise rest).
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static std::vector<BodyID> dynamicBodies;

static MaterialID material()
{
   return createMaterial( real(1), real(0), real(0.2), real(0.2), real(0.25), real(300), real(1e5), real(10), real(10) );
}

static void step( real dt, int n )
{
   for( int i = 0; i < n; ++i ) {
      for( BodyID b : dynamicBodies )
         b->addForce( b->getMass() * Vec3( 0, 0, real(-9.81) ) );
      theWorld()->simulationStep( dt );
   }
}

static real bottom( BoxID b )
{
   return b->getPosition()[2] - real(0.5) * b->getLengths()[2];
}

static real kineticEnergy()
{
   real e( 0 );
   for( BodyID b : dynamicBodies ) {
      const Vec3& v = b->getLinearVel();
      const Vec3& w = b->getAngularVel();
      e += real(0.5) * b->getMass() * ( trans( v ) * v ) + real(0.5) * ( trans( w ) * ( b->getInertia() * w ) );
   }
   return e;
}

int main()
{
   WorldID world = theWorld();
   world->setGravity( 0, 0, 0 );   // applied as a force in step()
   CollisionSystemID cs = theCollisionSystem();
   char what[200];

   // 1. Box 0.05 deep in the plane.
   for( int split = 0; split < 2; ++split ) {
      cs->setSplitImpulse( split == 1 );
      world->clear();
      dynamicBodies.clear();
      const MaterialID m( material() );
      createPlane( 1, 0, 0, 1, 0, m );
      BoxID box = createBox( 2, Vec3( 0, 0, 0.45 ), Vec3( 1, 1, 1 ), m );
      dynamicBodies.push_back( box );

      step( real(1e-3), 1 );
      const real vz1( box->getLinearVel()[2] );
      std::printf( "box 0.05 deep, split %s: v_z after one step %+.4f\n", split ? "on " : "off", static_cast<double>( vz1 ) );
      if( split == 0 ) {
         expect( vz1 > real(10), "Baumgarte: the overlapping box is launched (v_z > 10 m/s after one step)" );
      }
      else {
         expect( std::fabs( vz1 ) < real(0.05), "split impulse: the overlapping box is not launched (|v_z| < 0.05 after one step)" );
         step( real(1e-3), 29 );
         const real pen30( -bottom( box ) );
         std::printf( "   overlap after 30 steps %.3e\n", static_cast<double>( pen30 ) );
         expect( pen30 < real(1e-4), "split impulse: the overlap is removed (below 1e-4 after 30 steps)" );
         step( real(1e-3), 170 );
         std::printf( "   after 200 steps: bottom %+.3e, v_z %+.3e\n", static_cast<double>( bottom( box ) ), static_cast<double>( box->getLinearVel()[2] ) );
         expect( std::fabs( bottom( box ) ) < real(1e-4) && std::fabs( box->getLinearVel()[2] ) < real(0.05),
                 "split impulse: the box rests on the plane after 200 steps" );
      }
   }

   // 2. Drop at restitution 0.
   for( int split = 0; split < 2; ++split ) {
      cs->setSplitImpulse( split == 1 );
      world->clear();
      dynamicBodies.clear();
      const MaterialID m( material() );
      createPlane( 1, 0, 0, 1, 0, m );
      BoxID box = createBox( 2, Vec3( 0, 0, 1.0 ), Vec3( 1, 1, 1 ), m );   // bottom 0.5 above the plane
      dynamicBodies.push_back( box );

      bool touched = false;
      real rebound( 0 ), lowest( 1 );
      for( int i = 0; i < 600; ++i ) {
         step( real(2e-3), 1 );
         const real z( bottom( box ) );
         if( z < real(1e-3) ) touched = true;
         if( touched ) {
            lowest  = std::min( lowest, z );
            rebound = std::max( rebound, z );
         }
      }
      std::printf( "drop, split %s: deepest %+.4f, highest after first contact %+.4f, final v_z %+.3e\n", split ? "on " : "off",
                   static_cast<double>( lowest ), static_cast<double>( rebound ), static_cast<double>( box->getLinearVel()[2] ) );
      expect( touched, "drop: the box reaches the plane" );
      if( split == 0 ) {
         expect( rebound > real(0.02), "Baumgarte: the box bounces at restitution 0 (rises more than 0.02)" );
      }
      else {
         expect( rebound < real(2e-3), "split impulse: no bounce at restitution 0 (rises less than 2e-3)" );
         expect( std::fabs( box->getLinearVel()[2] ) < real(0.05) && std::fabs( bottom( box ) ) < real(1e-3), "split impulse: the dropped box comes to rest on the plane" );
      }
   }

   // 3. Pseudo motion propagates through a touching contact.
   {
      cs->setSplitImpulse( true );
      world->clear();
      dynamicBodies.clear();
      const MaterialID m( material() );
      createPlane( 1, 0, 0, 1, 0, m );
      BoxID lower = createBox( 2, Vec3( 0, 0, 0.45 ), Vec3( 1, 1, 1 ), m );
      BoxID upper = createBox( 3, Vec3( 0, 0, 1.45 ), Vec3( 1, 1, 1 ), m );   // exactly on the lower box
      dynamicBodies.push_back( lower );
      dynamicBodies.push_back( upper );
      step( real(1e-3), 50 );
      const real gap( bottom( upper ) - ( lower->getPosition()[2] + real(0.5) ) );
      std::printf( "stack lifted out of the plane: lower bottom %+.3e, gap to upper %+.3e, |v| %.3e / %.3e\n",
                   static_cast<double>( bottom( lower ) ), static_cast<double>( gap ),
                   static_cast<double>( lower->getLinearVel().length() ), static_cast<double>( upper->getLinearVel().length() ) );
      expect( std::fabs( bottom( lower ) ) < real(1e-4), "propagation: the lower box is lifted out of the plane" );
      expect( std::fabs( gap ) < real(1e-4), "propagation: the upper box moved with it and still touches" );
      expect( lower->getLinearVel().length() < real(0.05) && upper->getLinearVel().length() < real(0.05), "propagation: both boxes at rest" );
   }

   // 4. Resting tower with the switch on.
   {
      cs->setSplitImpulse( true );
      world->clear();
      dynamicBodies.clear();
      const MaterialID m( material() );
      createPlane( 1, 0, 0, 1, 0, m );
      for( int i = 0; i < 6; ++i )
         dynamicBodies.push_back( createBox( 2 + i, Vec3( 0, 0, real(0.5) + i ), Vec3( 1, 1, 1 ), m ) );
      const Vec3 top0( dynamicBodies.back()->getPosition() );
      step( real(2e-3), 500 );
      const real drift( ( dynamicBodies.back()->getPosition() - top0 ).length() );
      std::printf( "tower, split on: top drift %.3e, E_kin %.3e\n", static_cast<double>( drift ), static_cast<double>( kineticEnergy() ) );
      expect( drift < real(0.01) && kineticEnergy() < real(1e-2), "tower: rests with the split impulse (drift < 0.01, E_kin < 1e-2)" );
      std::snprintf( what, sizeof what, "tower: no NaN" );
      expect( std::isfinite( drift ), what );
   }

   cs->setSplitImpulse( false );
   if( failures == 0 ) {
      std::printf( "pe_split_impulse_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_split_impulse_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
