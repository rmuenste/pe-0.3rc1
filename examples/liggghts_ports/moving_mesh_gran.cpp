//=================================================================================================
/*!
 *  \file moving_mesh_gran.cpp
 *  \brief pe port of LIGGGHTS Tutorials_public/movingMeshGran (in.movingMeshGran).
 *
 *  LIGGGHTS: box 2 x 1 x 1 m (plane walls), 1275 spheres r = 0.04 inserted into the lower half
 *  (717 / 206 / 188 / 164 at t = 0, 0.1, 0.2, 0.3 s) with v = (0,0,-0.8); density 2500,
 *  restitution 0.9, friction 0.05, Hooke, dt 5e-5, 70000 steps (3.5 s). The bucket mesh
 *  (bucket.stl, 28 facets) becomes a wall at t = 1.5 s, translates with (-0.5,0,-0.3) for
 *  0.75 s, then rotates about the y axis through the origin with a period of 2 s for 1.25 s.
 *  pe: bucket.obj (LIGGGHTS mesh dump at step 0, world coordinates) as a kinematic fixed
 *  TriangleMesh created at t = 1.5 s and driven analytically (position, orientation, linear and
 *  angular velocity set every step); dt 5e-4.
 */
//=================================================================================================
#include <pe/system/WarningDisable.h>
#include "liggghts_common.h"
#include <pe/core/rigidbody/TriangleMesh.h>
using namespace pe;
using namespace lp;

int main( int argc, char* argv[] )
{
   Args args = parseArgs( argc, argv, "usage: lp_moving_mesh_gran [--dt 5e-4] [--tend 3.5] [--friction 0.025] [--out dir] [--no-vtk]  (needs bucket.obj)" );
   const real dt      ( args.dt   > 0 ? args.dt   : 5.0e-4 );
   const real tEnd    ( args.tEnd > 0 ? args.tEnd : 3.5 );
   const real friction( args.friction >= 0 ? args.friction : 0.025 );
   const real density ( 2500.0 ), radius( 0.04 );
   const unsigned int steps( stepsFor( tEnd, dt ) ), outSteps( stepsFor( 0.01, dt ) ), thermoSteps( stepsFor( 0.05, dt ) );
   const unsigned int insCounts[4] = { 717, 206, 188, 164 };   // LIGGGHTS insert/pack results
   const real tBucket( 1.5 ), tRotate( 2.25 );
   const Vec3 vLin( -0.5, 0.0, -0.3 );
   const real omega( 2.0*M_PI / 2.0 );                           // period 2 s about +y through the origin
   const Vec3 bucketCentroid( 0.367959984, 0.2, -0.000950674547 ); // from stl2obj_pe.py

   setSeed( 32452843 );
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 1.0 );
   theCollisionSystem()->setErrorReductionParameter( args.erp );
   if( args.vtk ) vtk::activateWriter( args.out, outSteps, 0, steps, false, true );

   MaterialID granular = createMaterial( "granular", density, 0.9, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );
   MaterialID wall     = createMaterial( "wall"    , density, 0.9, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );

   unsigned int id( 0 );
   createPlane( ++id,  1.0, 0.0, 0.0, -1.0, wall, false );
   createPlane( ++id, -1.0, 0.0, 0.0, -1.0, wall, false );
   createPlane( ++id,  0.0, 1.0, 0.0, -0.5, wall, false );
   createPlane( ++id,  0.0,-1.0, 0.0, -0.5, wall, false );
   createPlane( ++id,  0.0, 0.0, 1.0, -0.5, wall, false );
   createPlane( ++id,  0.0, 0.0,-1.0, -0.5, wall, false );

   struct RadiusFn { real r; real operator()() const { return r; } };
   RadiusFn radiusFn = { radius };
   BlockRegion region( Vec3( -0.9, -0.4, -0.5 ), Vec3( 0.9, 0.4, 0.0 ) );

   std::cout << "\n--MOVING_MESH_GRAN (pe port)-----------------------------------------------------\n"
             << " dt = " << dt << " s, " << steps << " steps, friction per material " << friction << ", erp " << args.erp << "\n"
             << "--------------------------------------------------------------------------------\n";

   TriangleMeshID bucket( 0 );
   Vec3 p0, p1;
   Thermo th; Thermo::header();
   timing::WcTimer timer; timer.start();
   unsigned int inserted( 0 );
   for( unsigned int step=0; step<steps; ++step ) {
      const real t( step*dt );
      // insertion
      for( int k=0; k<4; ++k )
         if( step == stepsFor( 0.1*k, dt ) ) {
            const unsigned int n( insertPack( world, id, insCounts[k], radiusFn, region, true, 100, Vec3( 0.0, 0.0, -0.8 ), granular ) );
            inserted += n;
            std::cout << " insertion at t=" << t << ": " << n << " particles (LIGGGHTS " << insCounts[k] << ")\n";
         }
      // bucket appears
      if( step == stepsFor( tBucket, dt ) ) {
         bucket = createTriangleMesh( ++id, bucketCentroid, "bucket.obj", wall, false, true );
         bucket->setFixed( true );
         p0 = bucket->getPosition();
         std::cout << " bucket created at t=" << t << ": " << bucket->size() << " triangles, AABB " << bucket->getAABB()
                   << "\n   (expected x[0.200,0.500] y[0,0.400] z[-0.148,0.100])\n";
      }
      // prescribed bucket motion for the coming step (state at time t+dt)
      if( bucket ) {
         const real tn( t + dt );
         if( tn <= tRotate ) {
            bucket->setPosition( p0 + vLin * ( tn - tBucket ) );
            bucket->setLinearVel( vLin );
            bucket->setAngularVel( 0.0, 0.0, 0.0 );
            p1 = p0 + vLin * ( tRotate - tBucket );
         }
         else {
            const real theta( omega * ( tn - tRotate ) );
            const Vec3 pos( p1[0]*std::cos(theta) + p1[2]*std::sin(theta), p1[1], -p1[0]*std::sin(theta) + p1[2]*std::cos(theta) );
            const Vec3 w( 0.0, omega, 0.0 );
            bucket->setPosition( pos );
            bucket->setOrientation( Quat( Vec3( 0.0, 1.0, 0.0 ), theta ) );
            bucket->setLinearVel( w % pos );
            bucket->setAngularVel( w );
         }
      }
      world->simulationStep( dt );
      if( (step+1) % thermoSteps == 0 || step+1 == steps ) {
         th.measure( world ); th.print( step+1, (step+1)*dt );
         if( bucket && ( (step+1) % stepsFor( 0.25, dt ) == 0 ) )
            std::cout << "          bucket centre " << bucket->getPosition() << "\n";
      }
   }
   timer.end();
   th.measure( world );
   // particles inside the bucket's AABB at the end (what the bucket carries)
   unsigned int carried( 0 );
   if( bucket ) {
      const auto& box = bucket->getAABB();
      for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
         const Vec3& p( s->getPosition() );
         if( p[0]>box[0] && p[1]>box[1] && p[2]>box[2] && p[0]<box[3] && p[1]<box[4] && p[2]<box[5] ) ++carried;
      }
   }
   std::cout << "--------------------------------------------------------------------------------\n"
             << " inserted " << inserted << " (LIGGGHTS 1275), ke " << th.ke << " J, z range [" << th.zmin << ", " << th.zmax << "]"
             << ", particles inside bucket AABB " << carried << "\n"
             << " wall-clock " << timer.total() << " s\n";
   return 0;
}
