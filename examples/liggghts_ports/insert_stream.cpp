//=================================================================================================
/*!
 *  \file insert_stream.cpp
 *  \brief pe port of LIGGGHTS Tutorials_public/insert_stream (in.insert_stream).
 *
 *  LIGGGHTS: 5000-particle stream (only 1200 are inserted within the 1 s run) through the
 *  polygonal face meshes/face.stl (scale 0.005, z = 2.215), extruded 0.6 m against the insertion
 *  velocity (0,-0.5,-2); 300 particles every 0.3 s; radii 0.015 / 0.025 with mass fractions
 *  0.3 / 0.7; density 2500; restitution 0.9; friction 0.05; no walls (shrink-wrapped domain, the
 *  particles fall freely). Hertz model, dt 1e-5, 100000 steps.
 *  pe: same insertion including the LIGGGHTS pre-release kinematics (particles glide with the
 *  normal velocity component, without gravity or contacts, until they cross the face), free fall,
 *  hard contacts, dt 2.5e-4 (4000 steps).
 */
//=================================================================================================
#include <pe/system/WarningDisable.h>
#include "liggghts_common.h"
using namespace pe;
using namespace lp;

int main( int argc, char* argv[] )
{
   Args args = parseArgs( argc, argv, "usage: insert_stream [--dt 2.5e-4] [--tend 1.0] [--friction 0.025] [--out dir] [--no-vtk] [--erp 0.5]" );
   const real dt      ( args.dt   > 0 ? args.dt   : 2.5e-4 );
   const real tEnd    ( args.tEnd > 0 ? args.tEnd : 1.0 );
   const real friction( args.friction >= 0 ? args.friction : 0.025 );   // pair value 0.05
   const real density ( 2500.0 );
   const real radii[2] = { 0.015, 0.025 };
   const real numFrac0( numberFraction0( 0.3, 0.7, radii[0], radii[1], density ) );
   const Vec3 vel( 0.0, -0.5, -2.0 );
   const real extrude( 0.6 );
   const real tInsert( 0.3 );              // LIGGGHTS inserted at steps 1, 30001, 60001, 90001
   const unsigned int nPerInsert( 300 );   // particlerate 1000 /s * 0.3 s
   const unsigned int nTotal( 5000 );

   const unsigned int steps( stepsFor( tEnd, dt ) ), outSteps( stepsFor( 0.008, dt ) ),
                      thermoSteps( stepsFor( 0.01, dt ) ), insSteps( stepsFor( tInsert, dt ) );

   setSeed( 32452867 );
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 1.0 );
   theCollisionSystem()->setErrorReductionParameter( args.erp );
   if( args.vtk ) vtk::activateWriter( args.out, outSteps, 0, steps, false, true );

   MaterialID granular = createMaterial( "granular", density, 0.9, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );

   StreamFace face( readAsciiSTL( "meshes/face.stl", 0.005 ), vel );
   real area( 0 ); for( size_t i=0; i<face.tris.size(); ++i ) area += face.tris[i].area;
   const bool kinematic( true );     // LIGGGHTS-style pre-release kinematics (see header)
   std::vector<SphereID> unreleased;

   std::cout << "\n--INSERT_STREAM (pe port)--------------------------------------------------------\n"
             << " dt = " << dt << " s, " << steps << " steps, insertion of " << nPerInsert << " every " << tInsert << " s, erp " << args.erp << "\n"
             << " face: " << face.tris.size() << " triangles, area " << area << " m^2, normal " << face.normal
             << ", extruded " << extrude << " m, kinematic pre-release velocity " << face.kinVelocity << "\n"
             << " number fraction r=" << radii[0] << " : " << numFrac0 << ", friction per material " << friction << "\n"
             << "--------------------------------------------------------------------------------\n";

   unsigned int id( 0 ), inserted( 0 );
   struct RadiusFn { real f, r0, r1; real operator()() const { return rand<real>(0.0,1.0) < f ? r0 : r1; } };
   RadiusFn radiusFn = { numFrac0, radii[0], radii[1] };

   Thermo th; Thermo::header();
   timing::WcTimer timer; timer.start();
   for( unsigned int step=0; step<steps; ++step ) {
      if( step % insSteps == 0 && inserted < nTotal ) {
         const unsigned int n( insertStream( world, id, std::min( nPerInsert, nTotal-inserted ), radiusFn, face, extrude, 100, granular, kinematic, unreleased ) );
         inserted += n;
         std::cout << " insertion at step " << step << ": " << n << " particles (total " << inserted << ")\n";
      }
      world->simulationStep( dt );
      releaseStream( unreleased, face, world );
      if( (step+1) % thermoSteps == 0 || step+1 == steps ) {
         th.measure( world ); th.print( step+1, (step+1)*dt );
         if( !unreleased.empty() ) std::cout << "          (" << unreleased.size() << " particles still above the face)\n";
      }
   }
   timer.end();
   th.measure( world );
   std::cout << "--------------------------------------------------------------------------------\n"
             << " particles " << th.n << ", ke " << th.ke << " J, z range [" << th.zmin << ", " << th.zmax << "]\n"
             << " wall-clock " << timer.total() << " s\n";
   return 0;
}
