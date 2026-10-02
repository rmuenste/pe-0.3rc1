//=================================================================================================
/*!
 *  \file cylinder_pack.cpp
 *  \brief pe port of the LIGGGHTS cylinder-container tutorials
 *         contactModels/in.newModels, cohesion/in.noCohesion, hysteresis/in.noHysteresis.
 *
 *  All three share the container: cylinder r = 0.05 around the z axis, floor z = 0, lid z = 0.15.
 *   --case newModels    : 1800 spheres r = 0.0025 packed once into cylinder r 0.045, z [0,0.15],
 *                         v = (0,0,-0.5); restitution 0.95, friction 0.05; 0.5 s.
 *   --case noCohesion   : 250 spheres r = 0.0015 every 0.03 s (4 times) into cylinder centre
 *   --case noHysteresis   (0.01,0.01) r 0.025, z [0.05,0.0603], v = (0,0,-0.2); restitution 0.9,
 *                         friction 0.05; 0.5 s. LIGGGHTS uses Hertz (noCohesion) or Hooke
 *                         (noHysteresis) springs; in pe both are the same hard-contact problem.
 *  The LIGGGHTS variants with SJKR cohesion or hooke/hysteresis plasticity have no pe equivalent.
 */
//=================================================================================================
#include <pe/system/WarningDisable.h>
#include "liggghts_common.h"
using namespace pe;
using namespace lp;

int main( int argc, char* argv[] )
{
   Args args = parseArgs( argc, argv, "usage: cylinder_pack --case newModels|noCohesion|noHysteresis [--dt 1e-4] [--tend 0.5] [--friction 0.025] [--out dir] [--no-vtk] [--erp 0.5]" );
   const std::string cs( args.caseName.empty() ? "noCohesion" : args.caseName );
   const bool newModels( cs == "newModels" );
   if( !newModels && cs != "noCohesion" && cs != "noHysteresis" ) { std::cerr << "unknown case " << cs << "\n"; return 1; }

   const real dt      ( args.dt   > 0 ? args.dt   : 1.0e-4 );
   const real tEnd    ( args.tEnd > 0 ? args.tEnd : 0.5 );
   const real friction( args.friction >= 0 ? args.friction : 0.025 );
   const real density ( 2500.0 );
   const real cor     ( newModels ? 0.95 : 0.9 );
   const real radius  ( newModels ? 0.0025 : 0.0015 );
   const unsigned int steps( stepsFor( tEnd, dt ) ), outSteps( stepsFor( 0.008, dt ) ), thermoSteps( stepsFor( 0.01, dt ) );

   setSeed( 32452843 );
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 1.0 );
   theCollisionSystem()->setErrorReductionParameter( args.erp );
   if( args.vtk ) vtk::activateWriter( args.out, outSteps, 0, steps, false, true );

   MaterialID granular = createMaterial( "granular", density, cor, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );
   MaterialID wall     = createMaterial( "wall"    , density, cor, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );

   unsigned int id( 0 );
   // container: pe cylinders are created along x; rotate 90 deg about y to align with z
   InnerCylinderID cyl = createInnerCylinder( ++id, Vec3( 0.0, 0.0, 0.075 ), 0.05, 0.15, wall );
   cyl->rotate( 0.0, M_PI/2.0, 0.0 );
   cyl->setFixed( true );
   createPlane( ++id, 0.0, 0.0,  1.0,  0.0 , wall, false );   // floor z = 0
   createPlane( ++id, 0.0, 0.0, -1.0, -0.15, wall, false );   // lid   z = 0.15

   struct RadiusFn { real r; real operator()() const { return r; } };
   RadiusFn radiusFn = { radius };

   std::cout << "\n--CYLINDER_PACK case " << cs << " (pe port)-----------------------------------------\n"
             << " dt = " << dt << " s, " << steps << " steps, r = " << radius << ", cor " << cor
             << ", friction per material " << friction << "\n"
             << "--------------------------------------------------------------------------------\n";

   Thermo th; Thermo::header();
   timing::WcTimer timer; timer.start();
   unsigned int inserted( 0 );
   for( unsigned int step=0; step<steps; ++step ) {
      if( newModels ) {
         if( step == 0 ) {
            CylinderZRegion reg( 0.0, 0.0, 0.045, 0.0, 0.15 );
            inserted += insertPack( world, id, 1800, radiusFn, reg, true, 100, Vec3( 0.0, 0.0, -0.5 ), granular );
            std::cout << " insertion at step 0: " << inserted << " particles\n";
         }
      }
      else {
         const unsigned int insSteps( stepsFor( 0.03, dt ) );
         if( step % insSteps == 0 && step < stepsFor( 0.1, dt ) ) {   // LIGGGHTS: steps 1, 3001, 6001, 9001
            CylinderZRegion reg( 0.01, 0.01, 0.025, 0.05, 0.0603 );
            const unsigned int n( insertPack( world, id, 250, radiusFn, reg, true, 100, Vec3( 0.0, 0.0, -0.2 ), granular ) );
            inserted += n;
            std::cout << " insertion at step " << step << ": " << n << " particles (total " << inserted << ")\n";
         }
      }
      world->simulationStep( dt );
      if( (step+1) % thermoSteps == 0 || step+1 == steps ) { th.measure( world ); th.print( step+1, (step+1)*dt ); }
   }
   timer.end();
   th.measure( world );
   std::cout << "--------------------------------------------------------------------------------\n"
             << " particles " << th.n << ", ke " << th.ke << " J, rke " << th.rke << " J, z range [" << th.zmin << ", " << th.zmax << "]\n"
             << " wall-clock " << timer.total() << " s\n";
   return 0;
}
