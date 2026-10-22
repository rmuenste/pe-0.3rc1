//=================================================================================================
/*!
 *  \file tests/interface/pe_kinematic_body_test.cpp
 *  \brief Kinematic (moving fixed) bodies: prescribed velocities, fixed-fixed pairs,
 *         RigidBody::setTargetPose() and sleep mode.
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  A fixed body (setFixed( true ): infinite mass and inertia) with a prescribed velocity is pe's
 *  kinematic body, provided MOBILE_INFINITE is set (CMake option PE_MOBILE_INFINITE, default ON
 *  for every compiler). Asserted:
 *    1. a fixed body keeps the velocity it had when it was fixed, takes new ones from the
 *       velocity setters, is active, moves with exactly that velocity, and is not damped by the
 *       world damping; a Union fixed through one of its bodies keeps its velocity, too. Without
 *       MOBILE_INFINITE the documented opposite: velocities zeroed, setters ignored, inactive;
 *    2. two fixed bodies never get a contact: a moving fixed box passing through a fixed box and
 *       a fixed plane creates no contact (the scene's only contact is a sphere resting on the
 *       plane), does not assert (run the Debug build) and moves exactly with its velocity;
 *    3. setTargetPose() lands a fixed body on the target pose after one step to 1e-12, local and
 *       global, for a rotation by 2.5 rad and by 4 rad (shortest arc: 2 pi - 4 the other way),
 *       along a 100-step trajectory with a sphere pushed around by the body, and for a fixed
 *       Union (its center of mass and the world position of a contained box); on a free cube it
 *       sets the velocities and wakes the body, and it rejects dt <= 0;
 *    4. a box at rest on a fixed platform (asleep if sleep mode is compiled in, i.e.
 *       pe::sleepThreshold > 0) is woken and carried when the platform starts to move, also at a
 *       speed below the sleep threshold, and when the platform gets a surface velocity instead;
 *       the moving platform itself never falls asleep.
 *
 *  Serial world setup, no MPI. Gravity: through World::setGravity() for the semi-implicit solver,
 *  as a body force for the other two hard-contact systems (no buoyancy, no driver forces).
 *  CTest runs this source against all three hard-contact collision systems; under any other
 *  configured solver the test prints a note and returns 77 (CTest: skipped).
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

using namespace pe;


//*************************************************************************************************
// Solver detection
//*************************************************************************************************

// All three hard-contact collision systems have setSplitImpulse(); only the semi-implicit one
// has setRestitution() and honours World::setGravity() without buoyancy.
template< typename T, typename = void > struct HasSplitImpulse : std::false_type {};
template< typename T > struct HasSplitImpulse< T, std::void_t< decltype( std::declval<T&>().setSplitImpulse( true ) ) > > : std::true_type {};
template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitution( true ) ) > > : std::true_type {};
typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

static const bool worldGravity( HasRestitution<CollisionSystemType>::value );


//*************************************************************************************************
// Helpers
//*************************************************************************************************

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static const real kG( 9.81 );
static MaterialID gmat;
static std::vector<BodyID> gravityBodies;   // bodies that get gravity as a force (other solvers)

static WorldID resetWorld()
{
   WorldID world = theWorld();
   world->clear();
   world->setDamping( 1 );
   world->setGravity( 0, 0, worldGravity ? -kG : real(0) );
   gravityBodies.clear();
   return world;
}

static void addGravity( BodyID b )
{
   if( !worldGravity ) gravityBodies.push_back( b );
}

static void step( real dt, int n = 1 )
{
   for( int i = 0; i < n; ++i ) {
      for( BodyID b : gravityBodies )
         b->addForce( b->getMass() * Vec3( 0, 0, -kG ) );
      theWorld()->simulationStep( dt );
   }
}

//! Largest deviation between the two rotations (applied to the three unit vectors).
static real rotationError( const Quat& a, const Quat& b )
{
   real err( 0 );
   for( size_t k = 0; k < 3; ++k ) {
      Vec3 e; e[k] = 1;
      err = std::max( err, ( a.rotate( e ) - b.rotate( e ) ).length() );
   }
   return err;
}

static bool finite( const Vec3& v )
{
   return std::isfinite( v[0] ) && std::isfinite( v[1] ) && std::isfinite( v[2] );
}


//*************************************************************************************************
// 1. Fixed bodies keep a prescribed velocity
//*************************************************************************************************

static void testFixedVelocity()
{
   WorldID world = resetWorld();
   const real dt( 0.01 );

   BoxID b = createBox( 1, Vec3( 0, 0, 5 ), Vec3( 1, 1, 1 ), gmat );
   b->setLinearVel( 1, 2, 3 );
   b->setAngularVel( real(0.1), real(0.2), real(0.3) );
   b->setFixed( true );
   expect( b->isFixed() && b->getInvMass() == real(0), "setFixed( true ): infinite mass" );

#if MOBILE_INFINITE
   expect( b->getLinearVel() == Vec3( 1, 2, 3 ) && b->getAngularVel() == Vec3( real(0.1), real(0.2), real(0.3) ), "a fixed body keeps the velocity it had" );
   expect( b->isActive(), "a fixed body with a velocity is active" );
   b->setLinearVel( -1, 0, real(0.5) );
   b->setAngularVel( 0, 0, 0 );
   expect( b->getLinearVel() == Vec3( -1, 0, real(0.5) ), "setLinearVel() works on a fixed body" );
   step( dt, 10 );
   expect( ( b->getPosition() - Vec3( real(-0.1), 0, real(5.05) ) ).length() < real(1e-12), "the fixed body moves with exactly its velocity" );
   world->setDamping( real(0.5) );
   step( dt );
   expect( b->getLinearVel() == Vec3( -1, 0, real(0.5) ), "world damping does not touch a prescribed velocity" );
   world->setDamping( 1 );

   // A Union fixed through one of its bodies keeps its velocity (Union::handleFixation()).
   UnionID u = createUnion( 2 );
   SphereID s1 = createSphere( 3, Vec3( 10, 0, 5 ), real(0.5), gmat );
   SphereID s2 = createSphere( 4, Vec3( 11, 0, 5 ), real(0.5), gmat );
   u->add( s1 );
   u->add( s2 );
   u->setLinearVel( real(0.5), 0, 0 );
   s1->setFixed( true );
   expect( u->isFixed() && u->getLinearVel() == Vec3( real(0.5), 0, 0 ), "a union fixed through a contained body keeps its velocity" );
#else
   expect( b->getLinearVel() == Vec3( 0, 0, 0 ) && b->getAngularVel() == Vec3( 0, 0, 0 ), "without MOBILE_INFINITE fixing zeroes the velocity" );
   b->setLinearVel( -1, 0, real(0.5) );
   expect( b->getLinearVel() == Vec3( 0, 0, 0 ), "without MOBILE_INFINITE the setters ignore fixed bodies" );
   expect( !b->isActive(), "without MOBILE_INFINITE a fixed body is inactive" );
   step( dt, 10 );
   expect( b->getPosition() == Vec3( 0, 0, 5 ), "without MOBILE_INFINITE the fixed body stays" );
#endif
}


//*************************************************************************************************
// 2. No contacts between fixed bodies
//*************************************************************************************************

static void testFixedFixed()
{
   resetWorld();
   const real dt( 0.01 );

   createPlane( 10, 0, 0, 1, 0, gmat );                                         // fixed by nature
   BoxID wall = createBox( 11, Vec3( 0, 0, real(0.1) ), Vec3( real(0.6), real(0.6), real(0.6) ), gmat );
   wall->setFixed( true );                                                     // in the plane
   BoxID mover = createBox( 12, Vec3( -1, 0, 0 ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
   mover->setFixed( true );                                                    // in the plane
   SphereID ball = createSphere( 13, Vec3( 3, 0, real(0.1) ), real(0.1), gmat );  // resting on the plane
   addGravity( ball );

   const Vec3 start( mover->getPosition() ), vel( 1, 0, 0 );
#if MOBILE_INFINITE
   mover->setLinearVel( vel );
   mover->setAngularVel( 0, 0, 3 );
#endif

   size_t minContacts( 1000 ), maxContacts( 0 );
   bool ok( true );
   for( int i = 1; i <= 200; ++i ) {
#if !MOBILE_INFINITE
      mover->setPosition( start + vel * ( i * dt ) );   // the fixed body cannot carry a velocity
#endif
      step( dt );
      const size_t n( theCollisionSystem()->getNumberOfContacts() );
      minContacts = std::min( minContacts, n );
      maxContacts = std::max( maxContacts, n );
      ok = ok && finite( mover->getPosition() ) && finite( ball->getPosition() ) && finite( wall->getPosition() );
   }
   std::printf( "fixed-fixed: %zu..%zu contacts per step, mover at x = %.12f\n", minContacts, maxContacts,
                static_cast<double>( mover->getPosition()[0] ) );
   expect( ok, "fixed-fixed: all positions finite" );
   expect( minContacts == 1 && maxContacts == 1, "fixed-fixed: only the sphere-plane contact, none between the fixed bodies" );
   expect( std::fabs( mover->getPosition()[0] - ( start[0] + 2 ) ) < real(1e-12), "fixed-fixed: the mover passed through with exactly its velocity" );
   expect( wall->getPosition() == Vec3( 0, 0, real(0.1) ) && std::fabs( ball->getPosition()[2] - real(0.1) ) < real(1e-3), "fixed-fixed: the others stay" );
}


//*************************************************************************************************
// 3. setTargetPose()
//*************************************************************************************************

static void checkTargetPose( BodyID b, const Vec3& target, const Quat& qTarget, real dt, const char* what )
{
   b->setTargetPose( target, qTarget, dt );
   step( dt );
   const real ePos( ( b->getPosition() - target ).length() );
   const real eRot( rotationError( b->getQuaternion(), qTarget ) );
   char msg[256];
   std::snprintf( msg, sizeof( msg ), "setTargetPose (%s): lands on the target, position %.1e, rotation %.1e", what,
                  static_cast<double>( ePos ), static_cast<double>( eRot ) );
   std::printf( "%s\n", msg );
   expect( ePos < real(1e-12) && eRot < real(1e-12), msg );
}

static void testTargetPose()
{
   const real dt( 1e-3 );
   const Vec3 axis( Vec3( 1, 2, 3 ).getNormalized() );

#if MOBILE_INFINITE
   // Local and global fixed boxes, rotation by 2.5 rad and by 4 rad (> pi: shortest arc).
   for( int global = 0; global < 2; ++global ) {
      resetWorld();
      BoxID b( 0 );
      if( global ) {
         pe_GLOBAL_SECTION {
            b = createBox( 20, Vec3( real(0.3), real(-0.2), 1 ), Vec3( 1, real(0.5), real(0.2) ), gmat );
            b->setFixed( true );
         }
      }
      else {
         b = createBox( 20, Vec3( real(0.3), real(-0.2), 1 ), Vec3( 1, real(0.5), real(0.2) ), gmat );
         b->setFixed( true );
      }
      b->setOrientation( Quat( Vec3( 0, 1, 0 ), real(0.3) ) );
      expect( b->isGlobal() == ( global == 1 ), "setTargetPose: body is global / local as intended" );

      const Quat q0( b->getQuaternion() );
      checkTargetPose( b, b->getPosition() + Vec3( real(0.1), real(-0.2), real(0.05) ), Quat( axis, real(2.5) ) * q0, dt,
                       global ? "global, 2.5 rad" : "local, 2.5 rad" );
      const Quat q1( b->getQuaternion() );
      b->setTargetPose( b->getPosition(), Quat( axis, real(4) ) * q1, dt );
      expect( std::fabs( b->getAngularVel().length() - ( real(2) * M_PI - real(4) ) / dt ) < real(1e-9) / dt,
              "setTargetPose: 4 rad turn takes the shortest arc (2 pi - 4)" );
      checkTargetPose( b, b->getPosition(), Quat( axis, real(4) ) * q1, dt, global ? "global, 4 rad" : "local, 4 rad" );
   }

   // A trajectory with a sphere in the way: contacts do not perturb the kinematic body.
   {
      resetWorld();
      BoxID b = createBox( 21, Vec3( 1, 0, real(0.5) ), Vec3( real(0.4), real(0.4), real(0.4) ), gmat );
      b->setFixed( true );
      createPlane( 22, 0, 0, 1, 0, gmat );
      SphereID s = createSphere( 23, Vec3( real(0.95), real(0.35), real(0.1) ), real(0.1), gmat );
      addGravity( s );
      real maxPos( 0 ), maxRot( 0 );
      for( int i = 1; i <= 100; ++i ) {
         const real t( i * dt * 10 );
         const Vec3 target( std::cos( t ), std::sin( t ), real(0.15) );
         const Quat qt( Vec3( 0, 0, 1 ), real(3) * t );
         b->setTargetPose( target, qt, dt * 10 );
         step( dt * 10 );
         maxPos = std::max( maxPos, ( b->getPosition() - target ).length() );
         maxRot = std::max( maxRot, rotationError( b->getQuaternion(), qt ) );
      }
      std::printf( "setTargetPose trajectory: max position error %.1e, rotation error %.1e, sphere moved to (%.3f, %.3f)\n",
                   static_cast<double>( maxPos ), static_cast<double>( maxRot ),
                   static_cast<double>( s->getPosition()[0] ), static_cast<double>( s->getPosition()[1] ) );
      expect( maxPos < real(1e-12) && maxRot < real(1e-12), "setTargetPose: 100-step trajectory followed to 1e-12 despite contacts" );
      expect( ( s->getPosition() - Vec3( real(0.95), real(0.35), real(0.1) ) ).length() > real(0.05), "setTargetPose: the kinematic box pushed the sphere" );
   }

   // A fixed Union: the position is its center of mass; the contained bodies follow.
   {
      resetWorld();
      UnionID u = createUnion( 30 );
      BoxID b1 = createBox( 31, Vec3( 0, 0, 1 ), Vec3( 1, 1, 1 ), gmat );
      BoxID b2 = createBox( 32, Vec3( 2, 0, 1 ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
      u->add( b1 );
      u->add( b2 );
      u->setFixed( true );
      const Vec3 u0( u->getPosition() ), rel( b2->getPosition() - u0 );
      const Vec3 target( u0 + Vec3( 0, real(0.3), real(0.1) ) );
      const Quat qt( Quat( axis, real(1.2) ) * u->getQuaternion() );
      const Vec3 relT( Quat( axis, real(1.2) ).rotate( rel ) );
      checkTargetPose( u, target, qt, dt, "fixed union" );
      expect( ( b2->getPosition() - ( target + relT ) ).length() < real(1e-12), "setTargetPose: the contained box follows the union" );
   }
#endif

   // A free cube: the function only sets velocities (no gravity, no contacts: lands, too).
   {
      WorldID world = resetWorld();
      world->setGravity( 0, 0, 0 );
      gravityBodies.clear();
      BoxID b = createBox( 40, Vec3( 0, 0, 3 ), Vec3( 1, 1, 1 ), gmat );
      const Vec3 target( real(0.01), 0, real(3.02) );
      const Quat qt( axis, real(0.4) );
      b->setTargetPose( target, qt, dt );
      expect( ( b->getLinearVel() - Vec3( 10, 0, 20 ) ).length() < real(1e-9) && ( b->getAngularVel() - axis * ( real(0.4) / dt ) ).length() < real(1e-9),
              "setTargetPose on a free body: v = dx/dt, w = axis angle/dt" );
      expect( b->isAwake(), "setTargetPose wakes the body" );
      step( dt );
      expect( ( b->getPosition() - target ).length() < real(1e-12) && rotationError( b->getQuaternion(), qt ) < real(1e-12),
              "setTargetPose: a free cube without forces lands on the target, too" );
      bool threw( false );
      try { b->setTargetPose( target, qt, 0 ); } catch( const std::invalid_argument& ) { threw = true; }
      expect( threw, "setTargetPose rejects dt <= 0" );
   }
}


//*************************************************************************************************
// 4. Sleep mode: bodies on a moving platform are woken and carried
//*************************************************************************************************

//! Rests a box on a fixed platform, then drives the platform; returns the box velocity.
static real carriedVelocity( bool surface, real speed, real& platformShift, bool& slept, bool& platformAwake )
{
   resetWorld();
   const real dt( 1e-3 );
   BoxID platform = createBox( 50, Vec3( 0, 0, real(-0.5) ), Vec3( 40, 4, 1 ), gmat );
   platform->setFixed( true );
   BoxID box = createBox( 51, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
   addGravity( box );

   step( dt, 500 );            // 0.5 s at rest: asleep if sleep mode is compiled in
   slept = !box->isAwake();

   if( surface )
      platform->setRelSurfaceVel( speed, 0, 0 );
   else
      platform->setLinearVel( speed, 0, 0 );
   step( dt, 1000 );           // 1 s: mu g = 3.9 m/s^2 reaches 0.5 m/s in 0.13 s
   platformShift = platform->getPosition()[0];
   platformAwake = platform->isAwake();
   expect( box->isAwake(), "sleep: the carried box is awake" );
   return box->getLinearVel()[0];
}

static void testSleep()
{
   std::printf( "sleep mode: pe::sleepThreshold = %g%s\n", static_cast<double>( sleepThreshold ),
                sleepThreshold > 0 ? "" : " (compiled out: the asleep checks are skipped)" );
   const bool checkAsleep( sleepThreshold > 0 && worldGravity );   // a gravity force wakes the box
   real shift( 0 ), v( 0 );
   bool slept( false ), platformAwake( false );
   char msg[256];

#if MOBILE_INFINITE
   const real speeds[2] = { real(0.5), real(1e-3) };   // the second is below any sensible threshold
   for( real speed : speeds ) {
      v = carriedVelocity( false, speed, shift, slept, platformAwake );
      std::printf( "sleep: platform at %g m/s, box slept %d, carried at %.6g m/s, platform moved %.6g m, awake %d\n",
                   static_cast<double>( speed ), slept, static_cast<double>( v ), static_cast<double>( shift ), platformAwake );
      if( checkAsleep ) expect( slept, "sleep: the box at rest fell asleep before the platform moved" );
      std::snprintf( msg, sizeof( msg ), "sleep: a box on a platform moving at %g m/s is woken and carried", static_cast<double>( speed ) );
      expect( std::fabs( v - speed ) < real(1e-3) * speed + real(1e-6), msg );
      expect( std::fabs( shift - speed ) < real(1e-9) && platformAwake, "sleep: the moving platform never falls asleep" );
   }
#endif

   v = carriedVelocity( true, real(0.5), shift, slept, platformAwake );
   std::printf( "sleep: platform with surface velocity 0.5 m/s, box slept %d, carried at %.6g m/s, platform moved %.3g m\n",
                slept, static_cast<double>( v ), static_cast<double>( shift ) );
   if( checkAsleep ) expect( slept, "sleep: the box at rest fell asleep before the belt started" );
   expect( std::fabs( v - real(0.5) ) < real(5e-4), "sleep: a box on a platform with a surface velocity is woken and carried" );
   expect( shift == real(0) && platformAwake, "sleep: the belt platform stays in place and awake" );
}


//*************************************************************************************************
int main()
{
   if( !HasSplitImpulse<CollisionSystemType>::value ) {
      std::printf( "pe_kinematic_body_test: the configured collision system is not a hard-contact system, skipped\n" );
      return 77;
   }
   std::printf( "MOBILE_INFINITE = %d, gravity through %s\n", MOBILE_INFINITE, worldGravity ? "World::setGravity()" : "body forces" );

   // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
   gmat = createMaterial( real(1000), real(0), real(0.2), real(0.2), real(0.25), real(300), real(1e5), real(10), real(10) );
   Material::setStaticFriction( gmat, gmat, real(0.4) );
   Material::setDynamicFriction( gmat, gmat, real(0.4) );

   testFixedVelocity();
   testFixedFixed();
   testTargetPose();
   testSleep();

   if( failures == 0 ) { std::printf( "pe_kinematic_body_test: all checks passed\n" ); return EXIT_SUCCESS; }
   std::printf( "pe_kinematic_body_test: %d check(s) failed\n", failures );
   return EXIT_FAILURE;
}
//*************************************************************************************************
