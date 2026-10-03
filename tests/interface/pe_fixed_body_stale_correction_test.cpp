//=================================================================================================
/*!
 *  \file tests/interface/pe_fixed_body_stale_correction_test.cpp
 *  \brief A fixed body must not inherit the velocity correction of a previous body in its slot.
 *
 *  The hard-contact solvers keep the per-body velocity corrections in dv_/dw_, indexed by the
 *  body's position in the body storage and resized (not reset) every step.
 *  initializeVelocityCorrections() used to write them only for awake, non-fixed bodies, so the
 *  slot of a fixed body kept the correction of whichever body used that index before; since a
 *  fixed body never receives an impulse, the stale value acted as a phantom velocity of the fixed
 *  body in every contact with it for the rest of the run. Boxes resting on a fixed ramp then
 *  saw the ramp move away and fell through it.
 *
 *  Scenario: a fixed plank tilted 20 degrees (a fixed box, not a plane) carrying three unit boxes
 *  with pair friction 0.4 > tan 20 degrees, so they must stick. Gravity is applied as a force per
 *  step (the Euler-Lagrange solver leaves body forces to the driver). Asserted, after 1 s:
 *    1. fresh world: the boxes stay put (drift < 1e-5, kinetic energy < 1e-9);
 *    2. after World::clear() of a world that ended with a 6-box tower still settling (the ramp
 *       reuses the slot of the tower's bottom box): the same, and identical to case 1;
 *    3. one world, no clear: the ramp is the last body and an unrelated tumbling box is destroyed
 *       after 0.2 s, so BodyStorage::remove() swaps the ramp into the destroyed box's slot: the
 *       same.
 *  Before the fix cases 2 and 3 gave drifts of 2.4 and 2.5.
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>

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

static const real dt = real(2e-3);
static std::vector<BodyID> dynamicBodies;

static MaterialID material()
{
   // Pair friction is the sum of both materials' coefficients: 0.2 + 0.2 = 0.4.
   return createMaterial( real(1), real(0), real(0.2), real(0.2), real(0.25), real(300), real(1e5), real(10), real(10) );
}

static void step( int n )
{
   for( int i = 0; i < n; ++i ) {
      for( BodyID b : dynamicBodies )
         b->addForce( b->getMass() * Vec3( 0, 0, real(-9.81) ) );
      theWorld()->simulationStep( dt );
   }
}

static real kineticEnergy( const std::vector<BodyID>& bodies )
{
   real e( 0 );
   for( BodyID b : bodies )
      e += real(0.5) * b->getMass() * ( trans( b->getLinearVel() ) * b->getLinearVel() );
   return e;
}

struct RampResult { real drift; real ekin; };

//! Fixed plank + three boxes. With \a lone, an extra tumbling box is created first and destroyed
//! after 100 steps, and the plank is created last.
static RampResult runRamp( bool lone )
{
   theWorld()->clear();
   dynamicBodies.clear();
   const MaterialID m( material() );
   pe::id_t id( 0 );
   createPlane( ++id, 0, 0, 1, 0, m );

   BoxID loneBox( nullptr );
   if( lone ) {
      loneBox = createBox( ++id, Vec3( 30, 0, 3 ), Vec3( 1, 1, 1 ), m );
      loneBox->setAngularVel( 3, -2, 5 );
      loneBox->setLinearVel( 0, 0, 2 );
   }

   const real a( real(20) * real(3.14159265358979323846) / real(180) ), thick( real(0.2) );
   const Quat q( real(0), a, real(0) );
   const Rot3 R( q.toRotationMatrix() );
   const Vec3 center( 0, 0, real(4) * std::sin( a ) + thick + real(0.05) );

   std::vector<BodyID> onRamp;
   for( int i = 0; i < 3; ++i ) {
      BoxID b = createBox( ++id, center + R * Vec3( real(-2.5) + real(1.2) * i, 0, real(0.5) * thick + real(0.5) ),
                           Vec3( 1, 1, 1 ), m );
      b->setOrientation( q );
      onRamp.push_back( b );
   }
   BoxID ramp = createBox( ++id, center, Vec3( 8, 3, thick ), m );   // last body in the storage
   ramp->setOrientation( q );
   ramp->setFixed( true );

   dynamicBodies = onRamp;
   if( lone ) dynamicBodies.push_back( loneBox );
   const Vec3 start( onRamp[0]->getPosition() );

   if( lone ) {
      step( 100 );
      dynamicBodies.pop_back();
      for( World::Iterator it = theWorld()->begin(); it != theWorld()->end(); ++it )
         if( *it == loneBox ) { theWorld()->destroy( it ); break; }
      step( 400 );
   }
   else {
      step( 500 );
   }
   return RampResult{ ( onRamp[0]->getPosition() - start ).length(), kineticEnergy( onRamp ) };
}

int main()
{
   theWorld()->setGravity( 0, 0, 0 );   // gravity is applied as a force in step()

   // 1. Fresh world.
   const RampResult fresh( runRamp( false ) );
   std::printf( "fresh:            drift %.3e  E_kin %.3e\n", fresh.drift, fresh.ekin );
   expect( fresh.drift < real(1e-5) && fresh.ekin < real(1e-9), "fresh world: boxes stick on the fixed ramp" );

   // 2. After World::clear() of a settling 6-box tower.
   theWorld()->clear();
   dynamicBodies.clear();
   {
      const MaterialID m( material() );
      pe::id_t id( 0 );
      createPlane( ++id, 0, 0, 1, 0, m );
      for( int i = 0; i < 6; ++i )
         dynamicBodies.push_back( createBox( ++id, Vec3( 0, 0, real(0.5) + i ), Vec3( 1, 1, 1 ), m ) );
      step( 500 );
   }
   const RampResult afterClear( runRamp( false ) );
   std::printf( "after clear:      drift %.3e  E_kin %.3e\n", afterClear.drift, afterClear.ekin );
   expect( afterClear.drift < real(1e-5) && afterClear.ekin < real(1e-9), "after World::clear(): boxes stick on the fixed ramp" );
   expect( std::fabs( afterClear.drift - fresh.drift ) <= real(1e-12), "after World::clear(): identical to a fresh world" );

   // 3. One world: destroying another body moves the fixed ramp into its slot.
   const RampResult afterDestroy( runRamp( true ) );
   std::printf( "after destroy:    drift %.3e  E_kin %.3e\n", afterDestroy.drift, afterDestroy.ekin );
   expect( afterDestroy.drift < real(1e-5) && afterDestroy.ekin < real(1e-9), "after destroying another body: boxes stick on the fixed ramp" );

   if( failures == 0 ) {
      std::printf( "pe_fixed_body_stale_correction_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_fixed_body_stale_correction_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
