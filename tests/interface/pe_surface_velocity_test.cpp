//=================================================================================================
/*!
 *  \file tests/interface/pe_surface_velocity_test.cpp
 *  \brief Surface velocities (RigidBody::setRelSurfaceVel()): conveyor belts that do not move.
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  A surface velocity is a velocity of a body's surface material relative to the body, given in
 *  the body frame. At a contact it enters the relative velocity of the friction only, projected
 *  onto the tangent plane (Contact::getSurfaceVel()); the normal row and the body itself are not
 *  affected. Asserted:
 *    1. the contact carries the relative tangential surface velocity: body 1 minus body 2, the
 *       normal component of the belt's velocity removed; world/body frame setters agree;
 *    2. a box at rest on a fixed belt box with surface velocity (-2, 0, 0) slides at first
 *       (v = -mu g t within 10 % after 0.05 s) and then moves with the belt (within 1e-3) while
 *       the belt neither moves nor gets a velocity;
 *    3. the surface velocity turns with the belt: a belt rotated by 90 degrees about z with the
 *       body-frame velocity (1, 0, 0) carries the box along +y;
 *    4. a plane with a surface velocity spins up a sphere until its contact point moves with the
 *       belt (within 1e-3);
 *    5. a belt that is a box inside a fixed Union, and a Union-level surface velocity, carry a
 *       box; a belt pressing on a box from above (the box is body 1 or 2 either way) carries it;
 *    6. no effect on the normal direction: without friction a box on a running belt stays at rest
 *       and matches a run without surface velocity to 1e-12; with friction the vertical velocity
 *       of the accelerated box stays below 1e-6 and its height matches the reference to 1e-9;
 *    7. checkpoint round trip (file format 0.4): the surface velocity of a body, of a Union and
 *       of a body inside a Union is restored;
 *    8. the static or dynamic coefficient of friction (mu_s = 0.5, mu_d = 0.1) is selected from
 *       the true slip, body velocities plus surface velocity, in both Contact constructors
 *       (vertex/face and edge/edge): a box riding a running belt at belt speed gets mu_s, a box
 *       held still on it gets mu_d (and the cases without surface velocity as before). On a belt
 *       inclined with slope 0.3 (between the two) and running uphill, a box riding at belt speed
 *       is carried up (with the plain body velocities it gets mu_d and slides back down), and a
 *       box put down at rest slides down against the belt.
 *
 *  Serial world setup, no MPI. Gravity: through World::setGravity() for the semi-implicit solver,
 *  as a body force for the other collision systems. CTest runs this source against all three
 *  hard-contact collision systems; under a configured solver that does not honour surface
 *  velocities (no hasSurfaceVelocity marker) the test returns 77 (CTest: skipped).
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/util/Checkpointer.h>
#include <pe/util/CheckpointMetadata.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using namespace pe;


//*************************************************************************************************
// Solver detection
//*************************************************************************************************

template< typename T, typename = void > struct HasSurfaceVelocity : std::false_type {};
template< typename T > struct HasSurfaceVelocity< T, std::void_t< decltype( T::hasSurfaceVelocity ) > > : std::integral_constant<bool, T::hasSurfaceVelocity> {};
template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitution( true ) ) > > : std::true_type {};
typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

// Only the semi-implicit solver honours World::setGravity() without buoyancy.
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

static const real kG( 9.81 ), kMu( 0.4 );
static MaterialID gmat, slick;
static std::vector<BodyID> gravityBodies;

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

static BoxID createBelt( pe::id_t uid, const Vec3& center, MaterialID m )
{
   BoxID belt = createBox( uid, center, Vec3( 40, 4, 1 ), m );   // top face at center z + 0.5
   belt->setFixed( true );
   return belt;
}


//*************************************************************************************************
// 1. The contact carries the relative tangential surface velocity
//*************************************************************************************************

static void testContactSurfaceVel()
{
   resetWorld();
   BoxID belt = createBelt( 1, Vec3( 0, 0, real(-0.5) ), gmat );
   belt->setRelSurfaceVel( 1, 0, real(0.5) );   // with a normal part, which the contact drops
   SphereID s = createSphere( 2, Vec3( 0, 0, real(0.099) ), real(0.1), gmat );

   expect( belt->hasSurfaceVel() && !s->hasSurfaceVel() && RigidBody::isSurfaceVelUsed(), "hasSurfaceVel() / isSurfaceVelUsed()" );

   ContactVector<Contact,PtrDelete> contacts;
   detection::fine::MaxContacts::collide( s, belt, contacts );
   expect( contacts.size() == 1, "contact: one sphere-belt contact" );
   if( contacts.size() == 1 ) {
      const ContactID c( contacts[0] );
      const Vec3 vs( c->getSurfaceVel() );
      const real sign( c->getBody1() == belt ? real(1) : real(-1) );   // body 1 minus body 2
      std::printf( "contact: normal %s, surface velocity (%g, %g, %g), body 1 is the %s\n",
                   c->getNormal()[2] > 0 ? "+z" : "-z", static_cast<double>( vs[0] ), static_cast<double>( vs[1] ),
                   static_cast<double>( vs[2] ), sign > 0 ? "belt" : "sphere" );
      expect( ( vs - sign * Vec3( 1, 0, 0 ) ).length() < real(1e-14), "contact: surface velocity = belt part, sign body 1 minus body 2" );
      expect( std::fabs( trans( c->getNormal() ) * vs ) < real(1e-14), "contact: no normal component" );
      expect( std::fabs( c->getNormalRelVel() ) < real(1e-14) && ( c->getRelVel() - vs ).length() < real(1e-14),
              "contact: getRelVel() contains the surface velocity, getNormalRelVel() does not" );
   }
   s->clearContacts();

   // World- and body-frame setters agree on a rotated belt.
   belt->setOrientation( Quat( Vec3( 0, 0, 1 ), real(0.5) * M_PI ) );
   belt->setSurfaceVel( 0, 1, 0 );
   expect( ( belt->getRelSurfaceVel() - Vec3( 1, 0, 0 ) ).length() < real(1e-14) && ( belt->getSurfaceVel() - Vec3( 0, 1, 0 ) ).length() < real(1e-14),
           "setSurfaceVel(): stored in the body frame" );
}


//*************************************************************************************************
// 2.-5. Belts carry what lies on them
//*************************************************************************************************

static void testBoxBelt()
{
   resetWorld();
   const real dt( 1e-3 );
   BoxID belt = createBelt( 10, Vec3( 0, 0, real(-0.5) ), gmat );
   belt->setRelSurfaceVel( -2, 0, 0 );
   BoxID box = createBox( 11, Vec3( 5, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
   addGravity( box );

   step( dt, 50 );
   const real vEarly( box->getLinearVel()[0] ), expected( -kMu * kG * real(0.05) );
   step( dt, 1450 );
   const Vec3 v( box->getLinearVel() );
   std::printf( "box belt: v_x %.4f after 0.05 s (sliding, expected %.4f), v = (%.6f, %.2e, %.2e) after 1.5 s\n",
                static_cast<double>( vEarly ), static_cast<double>( expected ), static_cast<double>( v[0] ),
                static_cast<double>( v[1] ), static_cast<double>( v[2] ) );
   expect( std::fabs( vEarly - expected ) < real(0.1) * std::fabs( expected ), "box belt: friction accelerates the box at mu g" );
   expect( std::fabs( v[0] + 2 ) < real(1e-3) && std::fabs( v[1] ) < real(1e-6), "box belt: the box moves with the belt" );
   expect( belt->getPosition() == Vec3( 0, 0, real(-0.5) ) && belt->getLinearVel() == Vec3( 0, 0, 0 ), "box belt: the belt stays where it is" );
}

static void testRotatedBelt()
{
   resetWorld();
   const real dt( 1e-3 );
   BoxID belt = createBelt( 20, Vec3( 0, 0, real(-0.5) ), gmat );
   belt->setOrientation( Quat( Vec3( 0, 0, 1 ), real(0.5) * M_PI ) );
   belt->setRelSurfaceVel( 1, 0, 0 );   // body frame x = world y
   BoxID box = createBox( 21, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
   addGravity( box );
   step( dt, 1000 );
   const Vec3 v( box->getLinearVel() );
   std::printf( "rotated belt: v = (%.2e, %.6f, %.2e)\n", static_cast<double>( v[0] ), static_cast<double>( v[1] ), static_cast<double>( v[2] ) );
   expect( std::fabs( v[1] - 1 ) < real(1e-3) && std::fabs( v[0] ) < real(1e-6), "rotated belt: the surface velocity turns with the body" );
}

static void testPlaneBelt()
{
   resetWorld();
   const real dt( 1e-3 ), R( 0.1 );
   PlaneID plane = createPlane( 30, 0, 0, 1, 0, gmat );
   plane->setSurfaceVel( real(1.5), 0, 0 );
   SphereID s = createSphere( 31, Vec3( 0, 0, R ), R, gmat );
   addGravity( s );
   step( dt, 1000 );
   const Vec3 vc( s->getLinearVel() + s->getAngularVel() % Vec3( 0, 0, -R ) );   // contact point
   std::printf( "plane belt: sphere v_x %.4f, w_y %.3f, contact point v_x %.6f\n", static_cast<double>( s->getLinearVel()[0] ),
                static_cast<double>( s->getAngularVel()[1] ), static_cast<double>( vc[0] ) );
   expect( std::fabs( vc[0] - real(1.5) ) < real(1e-3) && std::fabs( vc[1] ) < real(1e-6), "plane belt: the contact point of the sphere moves with the belt" );
   expect( s->getLinearVel()[0] > real(0.1), "plane belt: the sphere is carried along" );
}

static void testUnionBelts()
{
   const real dt( 1e-3 );
   for( int level = 0; level < 2; ++level ) {
      resetWorld();
      UnionID u = createUnion( 40 );
      BoxID a = createBox( 41, Vec3( -10, 0, real(-0.5) ), Vec3( 18, 4, 1 ), gmat );
      BoxID b = createBox( 42, Vec3(  10, 0, real(-0.5) ), Vec3( 18, 4, 1 ), gmat );
      u->add( a );
      u->add( b );
      u->setFixed( true );
      if( level == 0 ) b->setRelSurfaceVel( 1, 0, 0 );   // the belt is one box of the union
      else             u->setRelSurfaceVel( 1, 0, 0 );   // the whole union is a belt
      BoxID box = createBox( 43, Vec3( 5, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
      addGravity( box );
      step( dt, 1000 );
      char msg[128];
      std::snprintf( msg, sizeof( msg ), "union belt (%s): the box moves with the belt", level == 0 ? "contained box" : "union level" );
      std::printf( "union belt (%s): v_x %.6f\n", level == 0 ? "contained box" : "union level", static_cast<double>( box->getLinearVel()[0] ) );
      expect( std::fabs( box->getLinearVel()[0] - 1 ) < real(1e-3), msg );
   }
}

static void testCeilingBelt()
{
   WorldID world = resetWorld();
   world->setGravity( 0, 0, 0 );
   const real dt( 1e-3 );
   BoxID belt = createBelt( 50, Vec3( 0, 0, real(1.0) ), gmat );   // bottom face at z = 0.5
   belt->setRelSurfaceVel( 1, 0, 0 );
   BoxID box = createBox( 51, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), gmat );
   for( int i = 0; i < 1000; ++i ) {
      box->addForce( box->getMass() * Vec3( 0, 0, kG ) );   // pressed against the belt from below
      world->simulationStep( dt );
   }
   std::printf( "ceiling belt: v = (%.6f, %.2e, %.2e)\n", static_cast<double>( box->getLinearVel()[0] ),
                static_cast<double>( box->getLinearVel()[1] ), static_cast<double>( box->getLinearVel()[2] ) );
   expect( std::fabs( box->getLinearVel()[0] - 1 ) < real(1e-3), "ceiling belt: a belt above the box carries it in its direction" );
}


//*************************************************************************************************
// 6. No effect on the normal direction
//*************************************************************************************************

struct Trace { Vec3 pos, vel, avel; real maxVz; };

static Trace runResting( MaterialID m, real beltSpeed, int steps )
{
   resetWorld();
   const real dt( 1e-3 );
   BoxID belt = createBelt( 60, Vec3( 0, 0, real(-0.5) ), m );
   if( beltSpeed != real(0) ) belt->setRelSurfaceVel( beltSpeed, 0, 0 );
   BoxID box = createBox( 61, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), m );
   addGravity( box );
   Trace t;
   t.maxVz = 0;
   for( int i = 0; i < steps; ++i ) {
      step( dt );
      t.maxVz = std::max( t.maxVz, std::fabs( box->getLinearVel()[2] ) );
   }
   t.pos = box->getPosition();
   t.vel = box->getLinearVel();
   t.avel = box->getAngularVel();
   return t;
}

static void testNormalUnaffected()
{
   // Frictionless: the belt must not matter at all.
   const Trace ref( runResting( slick, 0, 500 ) ), run( runResting( slick, 3, 500 ) );
   std::printf( "frictionless: |dpos| %.1e, |dv| %.1e, |v| %.1e\n", static_cast<double>( ( run.pos - ref.pos ).length() ),
                static_cast<double>( ( run.vel - ref.vel ).length() ), static_cast<double>( run.vel.length() ) );
   expect( ( run.pos - ref.pos ).length() < real(1e-12) && ( run.vel - ref.vel ).length() < real(1e-12) && run.vel.length() < real(1e-9),
           "frictionless: a running belt leaves a resting box exactly at rest" );

   // With friction: the box is accelerated horizontally, its vertical motion does not change.
   const Trace fref( runResting( gmat, 0, 300 ) ), frun( runResting( gmat, 3, 300 ) );
   std::printf( "friction: v_x %.4f, max |v_z| %.1e (reference %.1e), |dz| %.1e, |w| %.1e\n", static_cast<double>( frun.vel[0] ),
                static_cast<double>( frun.maxVz ), static_cast<double>( fref.maxVz ), static_cast<double>( std::fabs( frun.pos[2] - fref.pos[2] ) ),
                static_cast<double>( frun.avel.length() ) );
   expect( frun.vel[0] > real(1), "friction: the belt accelerates the box" );
   expect( frun.maxVz < real(1e-6) && std::fabs( frun.pos[2] - fref.pos[2] ) < real(1e-9), "friction: no vertical motion from the surface velocity" );
}


//*************************************************************************************************
// 7. Checkpoint round trip
//*************************************************************************************************

static void testCheckpoint()
{
   WorldID world = resetWorld();
   BoxID belt = createBelt( 70, Vec3( 0, 0, real(-0.5) ), gmat );
   belt->setRelSurfaceVel( real(-4.5), 0, 0 );
   UnionID u = createUnion( 71 );
   BoxID a = createBox( 72, Vec3( 0, 5, 0 ), Vec3( 1, 1, 1 ), gmat );
   BoxID b = createBox( 73, Vec3( 2, 5, 0 ), Vec3( 1, 1, 1 ), gmat );
   u->add( a );
   u->add( b );
   u->setRelSurfaceVel( 0, 1, 0 );
   b->setRelSurfaceVel( 0, 0, real(0.25) );
   createSphere( 74, Vec3( 0, -5, 1 ), real(0.1), gmat );

   const std::string dir = "./_ckpt_surface_velocity_test", name = "surface.1";
   setCheckpointIdentity( real(1), uint64_t(1), std::string( "sv" ) );
   writeCheckpoint( dir, name );
   for( World::Iterator it = world->begin(); it != world->end(); ) it = world->destroy( it );
   readCheckpoint( dir, name );

   int found( 0 );
   for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
      if( it->getID() == 70 ) {
         ++found;
         expect( it->getRelSurfaceVel() == Vec3( real(-4.5), 0, 0 ) && it->isFixed(), "checkpoint: surface velocity of a body" );
      }
      else if( it->getID() == 74 ) {
         ++found;
         expect( !it->hasSurfaceVel(), "checkpoint: no surface velocity stays none" );
      }
      else if( it->getType() == unionType ) {
         ++found;
         UnionID ru = static_body_cast<Union>( *it );
         expect( ru->getRelSurfaceVel() == Vec3( 0, 1, 0 ), "checkpoint: surface velocity of a union" );
         for( Union::Iterator sb = ru->begin(); sb != ru->end(); ++sb ) {
            if( sb->getID() == 73 ) { ++found; expect( sb->getRelSurfaceVel() == Vec3( 0, 0, real(0.25) ), "checkpoint: surface velocity of a body in a union" ); }
            if( sb->getID() == 72 ) { ++found; expect( !sb->hasSurfaceVel(), "checkpoint: union body without surface velocity" ); }
         }
      }
   }
   expect( found == 5, "checkpoint: all bodies restored" );
}


//*************************************************************************************************
// 8. Static or dynamic coefficient of friction from the true slip
//*************************************************************************************************

static const real kMuStatic( 0.5 ), kMuDynamic( 0.1 );

static void testFrictionSelection( MaterialID m )
{
   const real u( 0.5 );
   char msg[160];

   // Both constructors, direct: box resting on a belt box, contact normal +z from belt (2) to box (1).
   const struct { real beltSpeed, boxSpeed, mu; const char* what; } cases[4] = {
      { u, u, kMuStatic,  "riding a running belt at belt speed" },
      { u, 0, kMuDynamic, "held still on a running belt" },
      { 0, 0, kMuStatic,  "at rest on a belt at rest" },
      { 0, u, kMuDynamic, "sliding on a belt at rest" } };
   for( const auto& cs : cases ) {
      resetWorld();
      BoxID belt = createBelt( 90, Vec3( 0, 0, real(-0.5) ), m );
      belt->setRelSurfaceVel( cs.beltSpeed, 0, 0 );
      BoxID box = createBox( 91, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), m );
      box->setLinearVel( cs.boxSpeed, 0, 0 );
      ContactVector<Contact,PtrDelete> contacts;
      contacts.addVertexFaceContact( box, belt, Vec3( real(0.25), real(0.25), 0 ), Vec3( 0, 0, 1 ), real(-1e-6) );
      contacts.addEdgeEdgeContact( box, belt, Vec3( real(0.25), 0, 0 ), Vec3( 0, 0, 1 ), Vec3( 0, 1, 0 ), Vec3( 1, 0, 0 ), real(-1e-6) );
      std::snprintf( msg, sizeof( msg ), "friction selection: a box %s gets mu = %g (vertex/face, edge/edge)", cs.what, static_cast<double>( cs.mu ) );
      expect( contacts.size() == 2 && contacts[0]->getFriction() == cs.mu && contacts[1]->getFriction() == cs.mu, msg );
      box->clearContacts();
   }

   // Dynamic: horizontal belt with gravity tilted by slope 0.3 along -x; "uphill" is +x.
   const real slope( 0.3 ), dt( 1e-3 );
   const Vec3 g( Vec3( -slope, 0, -1 ).getNormalized() * kG );
   for( int riding = 0; riding < 2; ++riding ) {
      WorldID world = resetWorld();
      world->setGravity( 0, 0, 0 );
      BoxID belt = createBelt( 92, Vec3( 0, 0, real(-0.5) ), m );
      belt->setRelSurfaceVel( u, 0, 0 );
      BoxID box = createBox( 93, Vec3( 0, 0, real(0.25) ), Vec3( real(0.5), real(0.5), real(0.5) ), m );
      box->setLinearVel( riding ? u : real(0), 0, 0 );
      for( int i = 0; i < 300; ++i ) {
         box->addForce( box->getMass() * g );
         world->simulationStep( dt );
      }
      const real v( box->getLinearVel()[0] );
      // Sliding: a = g (slope - mu_d) / sqrt(1 + slope^2) down the incline.
      const real aSlide( kG * ( slope - kMuDynamic ) / std::sqrt( real(1) + slope * slope ) );
      std::printf( "inclined belt, box %s: v_x %.6f after 0.3 s (riding: %g, sliding: about %.4f)\n",
                   riding ? "riding at belt speed" : "put down at rest", static_cast<double>( v ),
                   static_cast<double>( u ), static_cast<double>( -aSlide * real(0.3) ) );
      if( riding )
         expect( std::fabs( v - u ) < real(1e-3), "inclined belt: a box riding at belt speed sticks (mu_s) and is carried up" );
      else
         expect( v < real(-0.5) * aSlide * real(0.3), "inclined belt: a box put down at rest slides down against the belt (mu_d)" );
   }
}


//*************************************************************************************************
int main()
{
   if( !HasSurfaceVelocity<CollisionSystemType>::value ) {
      std::printf( "pe_surface_velocity_test: the configured collision system does not honour surface velocities, skipped\n" );
      return 77;
   }
   std::printf( "gravity through %s\n", worldGravity ? "World::setGravity()" : "body forces" );

   // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
   gmat   = createMaterial( real(1000), real(0), real(0.2), real(0.2), real(0.25), real(300), real(1e5), real(10), real(10) );
   slick = createMaterial( real(1000), real(0), real(0),   real(0),   real(0.25), real(300), real(1e5), real(10), real(10) );
   Material::setStaticFriction( gmat, gmat, kMu );
   Material::setDynamicFriction( gmat, gmat, kMu );
   Material::setStaticFriction( slick, slick, 0 );
   Material::setDynamicFriction( slick, slick, 0 );

   testContactSurfaceVel();
   testBoxBelt();
   testRotatedBelt();
   testPlaneBelt();
   testUnionBelts();
   testCeilingBelt();
   testNormalUnaffected();
   testCheckpoint();

   {
      // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
      MaterialID dual = createMaterial( real(1000), real(0), kMuStatic, kMuDynamic, real(0.25), real(300), real(1e5), real(10), real(10) );
      Material::setStaticFriction( dual, dual, kMuStatic );
      Material::setDynamicFriction( dual, dual, kMuDynamic );
      testFrictionSelection( dual );
   }

   if( failures == 0 ) { std::printf( "pe_surface_velocity_test: all checks passed\n" ); return EXIT_SUCCESS; }
   std::printf( "pe_surface_velocity_test: %d check(s) failed\n", failures );
   return EXIT_FAILURE;
}
//*************************************************************************************************
