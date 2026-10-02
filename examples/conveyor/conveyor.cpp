//=================================================================================================
/*!
 *  \file conveyor.cpp
 *  \brief Conveyor belt example: pe re-implementation of the LIGGGHTS-PUBLIC tutorial
 *         examples/LIGGGHTS/Tutorials_public/conveyor (in.conveyor).
 *
 *  Mapping of the LIGGGHTS setup onto pe:
 *
 *   LIGGGHTS (in.conveyor)                         pe (this file)
 *   -------------------------------------------    ----------------------------------------------
 *   region reg block -0.5 0.5 -0.2 0.2 -0.2 0.35   five fixed Planes (floor + 4 side walls)
 *   fix bx mesh/surface meshes/box.stl             (box.stl is exactly this open box, 10 facets)
 *   fix cv mesh/surface meshes/conveyor.stl        fixed Box, top face at z = 0, x in [-0.1,0.5]
 *       surface_vel -4.5 0 0                       belt->setLinearVel(-4.5,0,0), position reset
 *                                                  every step so the belt stays in place
 *   fix inface mesh/surface insertion_face.stl     insertion volume x[0.3,0.45] y[-0.17,0.17]
 *       + insert/stream extrude_length 0.1           z[0.2,0.3] (face extruded 0.1 against the
 *                                                  insertion velocity), centres inside (all_in no)
 *   particletemplate/sphere r=0.015 / 0.025        two sphere radii, MASS fractions 0.3 / 0.7
 *   particledistribution/discrete 0.3 0.7          (converted to number fractions ~ f_i/m_i)
 *   insert/stream mass 30 massrate 30 vel 0 0 -1   3 kg inserted every 0.1 s until 30 kg,
 *                                                  initial velocity (0,0,-1), overlap check
 *   density 2500, Young 5e6, Poisson 0.45,         material "granular": density 2500,
 *   restitution 0.3, friction 0.5                  cor 0.3, static/dynamic friction 0.5
 *   pair_style gran model hertz tangential history hard-contact solver
 *                                                  (HardContactSemiImplicitTimesteppingSolvers)
 *   timestep 1e-5, run 140000 (1.4 s)              dt = 5e-4 (hard contacts allow a larger step),
 *                                                  2800 steps = 1.4 s
 *   dump custom/vtk every 400 steps (0.004 s)      vtk::Writer every 8 steps (0.004 s)
 *
 *  Build (see md_docs/PE_CONVEYOR.md in the LIGGGHTS repo):
 *     configure pe with -DPE_BUILD_EXAMPLES=ON and
 *     -DPE_PREPROCESSOR_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers"
 *     then build the target "conveyor".
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <cmath>
#include <cstdlib>
#include <string>
#include <iomanip>
#include <iostream>
#include <vector>

#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/util/timing/WcTimer.h>

using namespace pe;


//*************************************************************************************************
/*!\brief A sphere that is about to be inserted (used for the overlap check). */
struct Candidate {
   Vec3 pos;
   real radius;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns true if a sphere (pos, radius) overlaps any existing or pending sphere. */
bool overlaps( const Vec3& pos, real radius, const std::vector<Candidate>& pending, WorldID world )
{
   for( std::vector<Candidate>::const_iterator c=pending.begin(); c!=pending.end(); ++c ) {
      const real d( ( c->pos - pos ).length() );
      if( d < c->radius + radius ) return true;
   }
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
      const real d( ( s->getPosition() - pos ).length() );
      if( d < s->getRadius() + radius ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
int main( int argc, char* argv[] )
{
   //---------------------------------------------------------------------------------------------
   // Parameters (SI units, mirroring in.conveyor)
   //---------------------------------------------------------------------------------------------
   const real dt          ( 5.0e-4 );   // time step size [s]
   const real tEnd        ( 1.4    );   // simulated time [s]  (LIGGGHTS: 140000 x 1e-5)
   const real tOutput     ( 0.004  );   // VTK output interval [s] (LIGGGHTS: every 400 steps)
   const real tThermo     ( 0.01   );   // screen output interval [s] (LIGGGHTS: thermo 1000)

   const real massTotal   ( 30.0   );   // total particle mass to insert [kg]
   const real massRate    ( 30.0   );   // insertion mass rate [kg/s]
   const real insertVel   ( 1.0    );   // insertion speed along -z [m/s]
   const real extrude     ( 0.1    );   // extrusion length of the insertion face [m]
   const real tInsert     ( extrude / insertVel );   // 0.1 s between two insertions
   const real massPerIns  ( massRate * tInsert );    // 3 kg per insertion

   const real density     ( 2500.0 );
   const real radii[2]    = { 0.015, 0.025 };
   const real massFrac[2] = { 0.3  , 0.7   };        // LIGGGHTS particledistribution/discrete
                                                     // weights are MASS fractions
   const unsigned int maxAttempt( 100 );             // insert/stream maxattempt 100

   // convert the mass fractions into number fractions: n_i ~ f_i / m_i
   const real mass0( density * real(4.0/3.0) * M_PI * radii[0]*radii[0]*radii[0] );
   const real mass1( density * real(4.0/3.0) * M_PI * radii[1]*radii[1]*radii[1] );
   const real numFrac0( ( massFrac[0]/mass0 ) / ( massFrac[0]/mass0 + massFrac[1]/mass1 ) );

   const Vec3 beltVel     ( -4.5, 0.0, 0.0 );        // conveyor surface velocity
   const Vec3 gravity     ( 0.0, 0.0, -9.81 );

   const unsigned int steps      ( static_cast<unsigned int>( std::floor( tEnd    / dt + 0.5 ) ) );
   const unsigned int outSteps   ( static_cast<unsigned int>( std::floor( tOutput / dt + 0.5 ) ) );
   const unsigned int thermoSteps( static_cast<unsigned int>( std::floor( tThermo / dt + 0.5 ) ) );
   const unsigned int insSteps   ( static_cast<unsigned int>( std::floor( tInsert / dt + 0.5 ) ) );

   // Friction coefficient assigned to EACH material. pe adds the coefficients of the two
   // materials of a contact pair (Materials.cpp), so 0.5 per material gives an effective
   // pair coefficient of 1.0; 0.25 reproduces the LIGGGHTS pair value of 0.5.
   real friction( 0.5 );
   real erp( 0.5 );       // error reduction parameter; acts like a restitution of ~erp on impacts (RUNBOOK 6.20)
   std::string outDir( "./paraview" );
   bool vtk( true );
   for( int i=1; i<argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--no-vtk" ) vtk = false;
      else if( arg == "--friction" && i+1 < argc ) friction = std::atof( argv[++i] );
      else if( arg == "--out" && i+1 < argc ) outDir = argv[++i];
      else if( arg == "--erp" && i+1 < argc ) erp = std::atof( argv[++i] );
      else if( arg == "--help" || arg == "-h" ) {
         std::cout << "usage: conveyor [--friction <mu per material, default 0.5>] [--erp <0.5>] [--out <dir>] [--no-vtk]\n";
         return 0;
      }
   }

   setSeed( 32452867 );   // same seed as the LIGGGHTS insert/stream command

   //---------------------------------------------------------------------------------------------
   // World, solver settings, output
   //---------------------------------------------------------------------------------------------
   WorldID world = theWorld();
   world->setGravity( gravity );
   world->setDamping( 1.0 );          // no artificial velocity damping

   // Hard-contact relaxation solver settings (defaults: 100 iterations, erp 0.7, relax 0.9)
   theCollisionSystem()->setMaxIterations( 100 );
   theCollisionSystem()->setErrorReductionParameter( erp );

   if( vtk )
      // writeEmptyFiles=true: the collector.pvd is written at activation time and only lists
      // body types that exist at that moment, so force all types to be listed/written.
      vtk::activateWriter( outDir, outSteps, 0, steps, false, true );

   //---------------------------------------------------------------------------------------------
   // Materials
   //---------------------------------------------------------------------------------------------
   //                                   name       density  cor  csf  cdf  poisson young  k    dampN dampT
   MaterialID granular = createMaterial( "granular", density, 0.3, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );
   MaterialID wall     = createMaterial( "wall"    , density, 0.3, friction, friction, 0.45, 5.0e6, 1e6, 1e5, 2e5 );

   //---------------------------------------------------------------------------------------------
   // Geometry
   //---------------------------------------------------------------------------------------------
   unsigned int id( 0 );

   // Container (box.stl): x in [-0.5,0.5], y in [-0.2,0.2], floor at z = -0.2, open top.
   // pe plane convention: n.x = d, the normal points into the free half space.
   createPlane( ++id,  0.0,  0.0,  1.0, -0.2, wall, false );   // floor  z = -0.2
   createPlane( ++id,  1.0,  0.0,  0.0, -0.5, wall, false );   // wall   x = -0.5
   createPlane( ++id, -1.0,  0.0,  0.0, -0.5, wall, false );   // wall   x =  0.5
   createPlane( ++id,  0.0,  1.0,  0.0, -0.2, wall, false );   // wall   y = -0.2
   createPlane( ++id,  0.0, -1.0,  0.0, -0.2, wall, false );   // wall   y =  0.2

   // Conveyor belt (conveyor.stl): rectangle z = 0, x in [-0.1,0.5], y in [-0.2,0.2].
   // Modelled as a thin fixed box whose top face is the belt surface. It is shrunk by 1 mm
   // in x and y so that it does not touch the container walls (no fixed-fixed contacts).
   const real  beltThickness( 0.02 );
   const Vec3  beltCenter( 0.2, 0.0, -0.5*beltThickness );
   const Vec3  beltLengths( 0.6 - 0.001, 0.4 - 0.002, beltThickness );
   BoxID belt = createBox( ++id, beltCenter, beltLengths, wall );
   belt->setFixed( true );
   belt->setLinearVel( beltVel );    // surface velocity (MOBILE_INFINITE allows this on fixed bodies)

   // Insertion volume: insertion_face.stl (z = 0.2, x in [0.3,0.45], y in [-0.17,0.17])
   // extruded by extrude_length AGAINST the insertion velocity (LIGGGHTS insert/stream), i.e.
   // upwards to z in [0.2,0.3]; the particles then stream down through the face. Like the
   // LIGGGHTS default "all_in no", only the particle centres have to lie inside the volume.
   const real insXmin( 0.30 ), insXmax( 0.45 );
   const real insYmin(-0.17 ), insYmax( 0.17 );
   const real insZmin( 0.2 ), insZmax( 0.2 + extrude );

   //---------------------------------------------------------------------------------------------
   // Banner
   //---------------------------------------------------------------------------------------------
   std::cout << "\n--CONVEYOR (pe port of the LIGGGHTS tutorial)------------------------------------\n"
             << " time step            = " << dt << " s\n"
             << " simulated time       = " << tEnd << " s  (" << steps << " steps)\n"
             << " insertion            = " << massPerIns << " kg every " << tInsert
             << " s until " << massTotal << " kg\n"
             << " number fraction r=" << radii[0] << " = " << numFrac0 << "\n"
             << " belt velocity        = " << beltVel << " m/s\n"
             << " friction per material= " << friction << "  (pair value " << 2*friction << "), erp " << erp << "\n"
             << " vtk output           = " << ( vtk ? "every " : "disabled" )
             << ( vtk ? std::to_string( outSteps ) + " steps -> ./paraview" : "" ) << "\n"
             << "--------------------------------------------------------------------------------\n";

   //---------------------------------------------------------------------------------------------
   // Time loop
   //---------------------------------------------------------------------------------------------
   real massInserted( 0.0 );
   real massCarry   ( 0.0 );   // mass that could not be placed in a previous insertion
   unsigned int nInsertions( 0 ), nParticles( 0 );

   timing::WcTimer timer;
   timer.start();

   std::cout << std::setw(8) << "step" << std::setw(10) << "time" << std::setw(8) << "atoms"
             << std::setw(14) << "mass_ins" << std::setw(14) << "ke" << std::setw(14) << "vmax" << "\n";

   for( unsigned int step=0; step<steps; ++step )
   {
      //--- particle insertion (like fix insert/stream) ----------------------------------------
      if( step % insSteps == 0 && massInserted < massTotal )
      {
         real target( std::min( massPerIns + massCarry, massTotal - massInserted ) );
         std::vector<Candidate> pending;
         real placed( 0.0 );
         unsigned int failures( 0 );

         while( placed < target && failures < 50 )
         {
            // Sample the radius once per particle (as LIGGGHTS does), then try up to
            // maxAttempt random positions for it. This keeps the size distribution honest.
            const real r( rand<real>( 0.0, 1.0 ) < numFrac0 ? radii[0] : radii[1] );
            const real m( density * real(4.0/3.0) * M_PI * r*r*r );
            if( placed + m > target + 0.5*m ) break;   // next particle would overshoot the mass

            bool found( false );
            Vec3 pos;
            for( unsigned int attempt=0; attempt<maxAttempt && !found; ++attempt ) {
               pos = Vec3( rand<real>( insXmin, insXmax ),
                           rand<real>( insYmin, insYmax ),
                           rand<real>( insZmin, insZmax ) );
               found = !overlaps( pos, r, pending, world );
            }
            if( !found ) { ++failures; continue; }

            Candidate c; c.pos = pos; c.radius = r;
            pending.push_back( c );
            placed += m;
         }

         for( std::vector<Candidate>::const_iterator c=pending.begin(); c!=pending.end(); ++c ) {
            SphereID s = createSphere( ++id, c->pos, c->radius, granular );
            s->setLinearVel( 0.0, 0.0, -insertVel );
            ++nParticles;
         }
         massInserted += placed;
         massCarry     = target - placed;
         ++nInsertions;
         std::cout << " insertion " << nInsertions << ": " << pending.size() << " particles, "
                   << placed << " kg (total " << massInserted << " kg)\n";
      }

      //--- one hard-contact time step -----------------------------------------------------------
      world->simulationStep( dt );

      // The belt is a fixed body with a prescribed surface velocity. pe integrates the position
      // of fixed bodies with their velocity, so put the belt back where it belongs and keep the
      // surface velocity for the next contact solve.
      belt->setPosition( beltCenter );
      belt->setLinearVel( beltVel );

      //--- screen output (like thermo) ----------------------------------------------------------
      if( (step+1) % thermoSteps == 0 || step+1 == steps )
      {
         real ke( 0.0 ), vmax( 0.0 );
         for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
            const real v2( s->getLinearVel().sqrLength() );
            ke   += real(0.5) * s->getMass() * v2;
            vmax  = std::max( vmax, std::sqrt( v2 ) );
         }
         std::cout << std::setw(8) << step+1 << std::setw(10) << std::fixed << std::setprecision(4) << (step+1)*dt
                   << std::setw(8) << nParticles
                   << std::setw(14) << std::setprecision(4) << massInserted
                   << std::setw(14) << std::scientific << std::setprecision(5) << ke
                   << std::setw(14) << std::fixed << std::setprecision(4) << vmax << "\n" << std::flush;
      }
   }

   timer.end();

   //---------------------------------------------------------------------------------------------
   // Summary
   //---------------------------------------------------------------------------------------------
   real zmin( 1e30 ), zmax( -1e30 ), xmin( 1e30 ), xmax( -1e30 );
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
      const Vec3& p( s->getPosition() );
      xmin = std::min( xmin, p[0] ); xmax = std::max( xmax, p[0] );
      zmin = std::min( zmin, p[2] ); zmax = std::max( zmax, p[2] );
   }
   std::cout << "--------------------------------------------------------------------------------\n"
             << " particles           = " << nParticles << "\n"
             << " mass inserted       = " << massInserted << " kg in " << nInsertions << " insertions\n"
             << " particle x range    = [" << xmin << ", " << xmax << "]\n"
             << " particle z range    = [" << zmin << ", " << zmax << "]\n"
             << " wall-clock time     = " << timer.total() << " s for " << steps << " steps\n"
             << "--------------------------------------------------------------------------------\n";

   return 0;
}
//*************************************************************************************************
