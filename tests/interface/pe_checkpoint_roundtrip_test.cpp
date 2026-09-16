//=================================================================================================
/*!
 *  \file tests/interface/pe_checkpoint_roundtrip_test.cpp
 *  \brief Checkpoint write/read round trip for the DNS resume path (sphere + ellipsoid).
 *
 *  Segmented FeatFloWer runs restart the fluid from a dump and, until now, re-created the
 *  particle cloud at rest from the xyz file (datasheet rows d52_v25f_l4_protocol,
 *  d62_chain_orientation_fix). The cure is pe's own checkpoint, written by the driver at the
 *  dump instant (pe_write_checkpoint_) and loaded by the DNS/DKT serial setups on resume.
 *  Ellipsoids were marshallable but never written or read by the checkpointer; this test pins:
 *    1. a moving sphere and a rotated, spinning ellipsoid survive write -> read with position,
 *       orientation quaternion, linear and angular velocity intact to round-off;
 *    2. the ellipsoid comes back as an ellipsoid with its semi-axes;
 *    3. the sidecar carries the driver identity set through setCheckpointIdentity()
 *       (time, step, pairing tag) and the body count;
 *    4. material indices resolve (two materials registered, bodies on the second).
 *
 *  Serial world, no MPI. Writes into ./_ckpt_roundtrip_test/ under the working directory.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/util/Checkpointer.h>
#include <pe/util/CheckpointMetadata.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool close( real x, real y, real tol = real(1e-13) )
{
   const real scale = std::max( real(1), std::max( std::fabs(x), std::fabs(y) ) );
   return std::fabs( x - y ) <= tol * scale;
}

static bool closeVec( const Vec3& u, const Vec3& v, real tol = real(1e-13) )
{
   return close( u[0], v[0], tol ) && close( u[1], v[1], tol ) && close( u[2], v[2], tol );
}

int main()
{
   WorldID world = theWorld();
   const std::string dir = "./_ckpt_roundtrip_test";
   const std::string name = "ffdump.7";

   // Two materials so that a body on the second one exercises the table index round trip.
   MaterialID matGround = createMaterial( "rt_ground", real(1.0), real(0.0), real(0.1), real(0.05),
                                          real(0.2), real(80), real(100), real(10), real(11) );
   MaterialID matBody = createMaterial( "rt_body", real(10.0), real(0.0), real(0.1), real(0.05),
                                        real(0.2), real(80), real(100), real(10), real(11) );
   (void)matGround;

   // --- scene: a translating+spinning sphere and a rotated, spinning ellipsoid ----------------
   const Vec3 sPos( 0.3, -0.2, 0.1 ), sVel( 0.12, -0.21, 0.05 ), sOmega( 1.0, 2.0, 3.0 );
   SphereID sph = createSphere( 11, sPos, real(0.5), matBody, true );
   sph->setLinearVel( sVel );
   sph->setAngularVel( sOmega );

   const real a = real(0.5), b = real(0.25), c = real(0.25);
   const Vec3 ePos( 4.0, 3.0, 4.0 ), eOmega( 0.0, 0.1, 0.0 );
   EllipsoidID ell = createEllipsoid( 12, ePos, a, b, c, matBody, true );
   ell->rotate( Vec3( 0.0, 1.0, 0.0 ), real(0.7) );   // tilt the a-axis in the x-z plane
   ell->setAngularVel( eOmega );
   ell->setLinearDofMask( Vec3( 0.0, 0.0, 0.0 ) );      // setup state, re-applied on resume
   const Quat eQuat = ell->getQuaternion();
   const Quat sQuat = sph->getQuaternion();
   const Vec3 eRadii = ell->getRadius();

   // --- driver identity, then write ---------------------------------------------------------
   setCheckpointIdentity( real(12.5), uint64_t(2500), std::string( "ff:istep=2500" ) );
   writeCheckpoint( dir, name );

   // --- clear the world so the read is a genuine restore -------------------------------------
   for( World::Iterator it = world->begin(); it != world->end(); ) {
      it = world->destroy( it );
   }
   expect( world->size() == 0, "world empty before read" );

   // --- read ----------------------------------------------------------------------------------
   const CheckpointMetadata meta = readCheckpoint( dir, name );
   expect( meta.present, "sidecar present" );
   expect( meta.bodyCount == 2, "sidecar body count 2" );
   expect( close( meta.simulationTime, real(12.5) ), "sidecar simulation time from driver" );
   expect( meta.timeStep == 2500, "sidecar step from driver" );
   expect( meta.pairingTag == "ff:istep=2500", "sidecar pairing tag" );
   expect( meta.timeSource == checkpointTimeFromDriver, "time source = driver" );

   expect( world->size() == 2, "two bodies restored" );

   int seenSphere = 0, seenEllipsoid = 0;
   for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
      BodyID body = *it;
      if( body->getType() == sphereType ) {
         ++seenSphere;
         expect( body->getID() == 11, "sphere uid" );
         expect( closeVec( body->getPosition(), sPos ), "sphere position" );
         expect( closeVec( body->getLinearVel(), sVel ), "sphere linear velocity" );
         expect( closeVec( body->getAngularVel(), sOmega ), "sphere angular velocity" );
         const Quat q = body->getQuaternion();
         expect( close( q[0], sQuat[0] ) && close( q[1], sQuat[1] ) &&
                 close( q[2], sQuat[2] ) && close( q[3], sQuat[3] ), "sphere quaternion" );
         expect( close( body->getMass(), real(10.0) * ( real(4) / real(3) ) * M_PI * real(0.125) ),
                 "sphere mass -> material index resolved to rt_body (rho=10)" );
      }
      else if( body->getType() == ellipsoidType ) {
         ++seenEllipsoid;
         {
            const Vec3 p = body->getPosition();
            const Vec3 r = static_body_cast<Ellipsoid>( body )->getRadius();
            const Quat q = body->getQuaternion();
            std::printf( "restored ellipsoid: uid=%lu pos=(%g,%g,%g) radii=(%g,%g,%g) q=(%g,%g,%g,%g) w=(%g,%g,%g) mass=%g\n",
                         (unsigned long)body->getID(), p[0], p[1], p[2], r[0], r[1], r[2],
                         q[0], q[1], q[2], q[3], body->getAngularVel()[0], body->getAngularVel()[1],
                         body->getAngularVel()[2], body->getMass() );
            std::printf( "expected  ellipsoid: uid=12 pos=(%g,%g,%g) radii=(%g,%g,%g) q=(%g,%g,%g,%g)\n",
                         ePos[0], ePos[1], ePos[2], eRadii[0], eRadii[1], eRadii[2],
                         eQuat[0], eQuat[1], eQuat[2], eQuat[3] );
         }
         expect( body->getID() == 12, "ellipsoid uid" );
         expect( closeVec( body->getPosition(), ePos ), "ellipsoid position" );
         expect( closeVec( body->getAngularVel(), eOmega ), "ellipsoid angular velocity" );
         const Quat q = body->getQuaternion();
         expect( close( q[0], eQuat[0] ) && close( q[1], eQuat[1] ) &&
                 close( q[2], eQuat[2] ) && close( q[3], eQuat[3] ), "ellipsoid quaternion (orientation)" );
         EllipsoidID e = static_body_cast<Ellipsoid>( body );
         expect( closeVec( e->getRadius(), eRadii ), "ellipsoid semi-axes" );
         // the tilted a-axis must point where it pointed before the round trip
         const Vec3 axisBefore = eQuat.rotate( Vec3( 1, 0, 0 ) );
         const Vec3 axisAfter = q.rotate( Vec3( 1, 0, 0 ) );
         expect( closeVec( axisBefore, axisAfter ), "ellipsoid a-axis direction" );
         expect( close( e->getMass(), real(10.0) * ( real(4) / real(3) ) * M_PI * a * b * c ),
                 "ellipsoid mass -> material index resolved" );
      }
      else {
         expect( false, "unexpected body type restored" );
      }
   }
   expect( seenSphere == 1, "exactly one sphere restored" );
   expect( seenEllipsoid == 1, "exactly one ellipsoid restored (ellipsoid checkpoint support)" );

   if( failures == 0 ) {
      std::printf( "pe_checkpoint_roundtrip_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_checkpoint_roundtrip_test: %d check(s) failed\n", failures );
   return EXIT_FAILURE;
}
