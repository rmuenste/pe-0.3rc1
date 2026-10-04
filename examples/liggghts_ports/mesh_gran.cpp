//=================================================================================================
/*!
 *  \file mesh_gran.cpp
 *  \brief pe port of LIGGGHTS Tutorials_public/meshGran (in.meshGran).
 *
 *  LIGGGHTS: closed STL trough (meshes/mesh.stl, 92 facets, scaled 0.001 and rotated -90 deg
 *  about x), 20 spheres r = 0.005 inserted every 0.02 s through insertion_face.stl (z = -0.02,
 *  extruded 0.02) with v = (0,0,-1) until 500 particles; density 2500, restitution 0.7, friction
 *  0.05, Hooke, dt 5e-5, 40000 steps (2 s). The particles slide down the trough and leave it at
 *  the bottom (no floor, shrink-wrapped domain).
 *  pe: trough.obj is the LIGGGHTS mesh dump at step 0 (world coordinates, converted with
 *  runs/stl2obj_pe.py), loaded as a fixed TriangleMesh at the centroid pe computes for it;
 *  sphere-mesh contacts use pe's closest-triangle search (no DistanceMap needed); dt 1e-4.
 */
//=================================================================================================
#include <pe/system/WarningDisable.h>
#include "liggghts_common.h"
#include <pe/core/rigidbody/TriangleMesh.h>
using namespace pe;
using namespace lp;

int main( int argc, char* argv[] )
{
   Args args = parseArgs( argc, argv, "usage: lp_mesh_gran [--dt 1e-4] [--tend 2.0] [--friction 0.025] [--out dir] [--no-vtk]  (needs trough.obj, meshes/insertion_face.stl)" );
   const real dt      ( args.dt   > 0 ? args.dt   : 1.0e-4 );
   const real tEnd    ( args.tEnd > 0 ? args.tEnd : 2.0 );
   const real friction( args.friction >= 0 ? args.friction : 0.025 );
   const real density ( 2500.0 ), radius( 0.005 );
   const Vec3 vel( 0.0, 0.0, -1.0 );
   const unsigned int steps( stepsFor( tEnd, dt ) ), outSteps( stepsFor( 0.015, dt ) ), thermoSteps( stepsFor( 0.01, dt ) ),
                      insSteps( stepsFor( 0.02, dt ) ), insUntil( stepsFor( 0.5, dt ) );

   setSeed( 32452843 );
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 1.0 );
   theCollisionSystem()->setErrorReductionParameter( args.erp );
   if( args.vtk ) vtk::activateWriter( args.out, outSteps, 0, steps, false, true );

   MaterialID granular = createMaterial( "granular", density, 0.7, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );
   MaterialID wall     = createMaterial( "wall"    , density, 0.7, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );

   unsigned int id( 0 );
   // centroid printed by stl2obj_pe.py for trough.obj
   TriangleMeshID trough = createTriangleMesh( ++id, Vec3( 0.140426368, -0.0468224109, -0.345524782 ), "trough.obj", wall, false, true );
   trough->setFixed( true );
   std::cout << " trough: " << trough->size() << " triangles, mass " << trough->getMass() << " kg, AABB " << trough->getAABB()
             << "\n   (expected x[-0.008,0.508] y[-0.058,0.008] z[-1.209,0.008])\n";

   StreamFace face( readAsciiSTL( "meshes/insertion_face.stl" ), vel );
   std::vector<SphereID> unreleased;
   struct RadiusFn { real r; real operator()() const { return r; } };
   RadiusFn radiusFn = { radius };

   std::cout << "\n--MESH_GRAN (pe port)------------------------------------------------------------\n"
             << " dt = " << dt << " s, " << steps << " steps, 20 particles every 0.02 s until 0.5 s, friction per material " << friction << ", erp " << args.erp << "\n"
             << " insertion face normal " << face.normal << ", kinematic velocity " << face.kinVelocity << "\n"
             << "--------------------------------------------------------------------------------\n";

   Thermo th; Thermo::header();
   timing::WcTimer timer; timer.start();
   unsigned int inserted( 0 );
   for( unsigned int step=0; step<steps; ++step ) {
      if( step % insSteps == 0 && step < insUntil )
         inserted += insertStream( world, id, 20, radiusFn, face, 0.02, 100, granular, true, unreleased );
      world->simulationStep( dt );
      releaseStream( unreleased, face, world );
      if( (step+1) % thermoSteps == 0 || step+1 == steps ) { th.measure( world ); th.print( step+1, (step+1)*dt ); }
   }
   timer.end();
   th.measure( world );
   // how many particles are still inside the trough's x-range / have left at the bottom
   unsigned int inTrough( 0 ), below( 0 );
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
      if( s->getPosition()[2] < -1.209 ) ++below; else ++inTrough;
   }
   std::cout << "--------------------------------------------------------------------------------\n"
             << " inserted " << inserted << ", in trough " << inTrough << ", below trough outlet " << below
             << ", ke " << th.ke << " J, z range [" << th.zmin() << ", " << th.zmax() << "]\n"
             << " wall-clock " << timer.total() << " s\n";
   return 0;
}
