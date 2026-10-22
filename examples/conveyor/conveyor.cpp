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
 *       surface_vel -4.5 0 0                       belt->setSurfaceVel(-4.5,0,0): a surface
 *                                                  velocity, used by the contact friction only;
 *                                                  the belt does not move (--moving-belt: the old
 *                                                  path, a fixed box moving with setLinearVel()
 *                                                  and put back with setPosition() every step)
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
 *     then build the target "conveyor". Under any other solver the example refuses to run
 *     (exit code 77, which CTest reports as skipped).
 *
 *  Parallel (MPI builds): mpiexec -n <p> conveyor ... decomposes the container into p slabs
 *  along the belt direction x (liggghts_common.h: decomposeSlabs()); the walls and the belt are
 *  global bodies, the insertion draws identical candidates on every process and creates each
 *  particle on its owner. The thermo output is reduced over all processes. The run ends with a
 *  self check (particle count, container bounds, finite energies) that sets the exit code.
 *
 *  The thermo column vx_belt is the mean x velocity of the particles in the bottom layer on the
 *  belt (centre above the belt, below 2 r_max); the summary averages it over the second half of
 *  the run, to compare the surface-velocity belt with the --moving-belt one.
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

#include "liggghts_common.h"   // examples/liggghts_ports: insertion, thermo and parallel helpers
#include <examples/common/SolverConstraint.h>

using namespace pe;
using lp::Candidate;
using lp::rout;


//*************************************************************************************************
/*!\brief Mean x velocity of the particles in the bottom layer on the belt, over all processes.
 *
 * \param world The simulation world.
 * \param xmin, xmax The belt extent along x.
 * \param zmax Upper bound of the centres of the bottom layer (the belt top is z = 0).
 * \param count On return the number of particles in the layer.
 * \return The mean x velocity (0 for an empty layer).
 */
static real beltLayerVelocity( WorldID world, real xmin, real xmax, real zmax, unsigned int& count )
{
   real sum[2] = { 0, 0 };
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
      if( s->isRemote() ) continue;
      const Vec3& p( s->getPosition() );
      if( p[0] < xmin || p[0] > xmax || p[2] < 0 || p[2] > zmax ) continue;
      sum[0] += s->getLinearVel()[0];
      sum[1] += 1;
   }
#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      real all[2];
      MPI_Allreduce( sum, all, 2, MPITrait<real>::getType(), MPI_SUM, MPISettings::comm() );
      sum[0] = all[0];
      sum[1] = all[1];
   }
#endif
   count = static_cast<unsigned int>( sum[1] + real(0.5) );
   return count > 0 ? sum[0] / sum[1] : real(0);
}
//*************************************************************************************************

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR,
                       response::HardContactSemiImplicitTimesteppingSolvers >::Config IntendedConfig;


//*************************************************************************************************
int main( int argc, char* argv[] )
{
   lp::MpiScope mpi( argc, argv );
   if( !pe::examples::requireIntendedSolver<IntendedConfig>( "conveyor",
          "HardContactSemiImplicitTimesteppingSolvers",
          "the LIGGGHTS ports are tuned for the semi-implicit hard-contact solver (gravity through "
          "World::setGravity(), erp 0.5); build with -Dpe_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers" ) )
      return lp::kSkipExitCode;

   //---------------------------------------------------------------------------------------------
   // Parameters (SI units, mirroring in.conveyor)
   //---------------------------------------------------------------------------------------------
   const real dt          ( 5.0e-4 );   // time step size [s]
   real       tEnd        ( 1.4    );   // simulated time [s]  (LIGGGHTS: 140000 x 1e-5); --tend
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


   // Friction coefficient assigned to EACH material. pe adds the coefficients of the two
   // materials of a contact pair (Materials.cpp), so 0.5 per material gives an effective
   // pair coefficient of 1.0; 0.25 reproduces the LIGGGHTS pair value of 0.5.
   real friction( 0.5 );
   real erp( 0.5 );       // error reduction parameter; acts like a restitution of ~erp on impacts (RUNBOOK 6.20)
   bool splitImpulse( false );   // --split-impulse: the erp correction moves positions without adding momentum
   bool movingBelt( false );     // --moving-belt: belt velocity as a moving fixed body (the old path)
   std::string outDir( "./paraview" );
   bool vtk( true );
   for( int i=1; i<argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--no-vtk" ) vtk = false;
      else if( arg == "--friction" && i+1 < argc ) friction = std::atof( argv[++i] );
      else if( arg == "--out" && i+1 < argc ) outDir = argv[++i];
      else if( arg == "--erp" && i+1 < argc ) erp = std::atof( argv[++i] );
      else if( arg == "--tend" && i+1 < argc ) tEnd = std::atof( argv[++i] );
      else if( arg == "--split-impulse" ) splitImpulse = true;
      else if( arg == "--moving-belt" ) movingBelt = true;
      else if( arg == "--help" || arg == "-h" ) {
         rout() << "usage: conveyor [--friction <mu per material, default 0.5>] [--erp <0.5>] [--tend <1.4>] [--split-impulse] [--moving-belt] [--out <dir>] [--no-vtk]\n"
                << "  --moving-belt: belt as a fixed box moving with the belt velocity and put back every step\n"
                << "                 (the path before surface velocities) instead of a belt surface velocity\n"
                << "  runs serially or under MPI (mpiexec -n <p>): slab decomposition along x\n";
         return 0;
      }
   }

#if !MOBILE_INFINITE
   if( movingBelt ) {
      // A fixed body cannot carry a velocity in this build: the belt would silently stand still.
      rout() << "conveyor: --moving-belt needs mobile infinite bodies (configure pe with -DPE_MOBILE_INFINITE=ON)\n";
      return lp::kSkipExitCode;
   }
#endif

   setSeed( 32452867 );   // same seed as the LIGGGHTS insert/stream command (and on every process)

   // step counts (after the command line: --tend changes tEnd)
   const unsigned int steps      ( static_cast<unsigned int>( std::floor( tEnd    / dt + 0.5 ) ) );
   const unsigned int outSteps   ( static_cast<unsigned int>( std::floor( tOutput / dt + 0.5 ) ) );
   const unsigned int thermoSteps( static_cast<unsigned int>( std::floor( tThermo / dt + 0.5 ) ) );
   const unsigned int insSteps   ( static_cast<unsigned int>( std::floor( tInsert / dt + 0.5 ) ) );

   //---------------------------------------------------------------------------------------------
   // World, solver settings, output
   //---------------------------------------------------------------------------------------------
   WorldID world = theWorld();
   world->setGravity( gravity );
   world->setDamping( 1.0 );          // no artificial velocity damping

   // Hard-contact relaxation solver settings (defaults: 100 iterations, erp 0.7, relax 0.9)
   theCollisionSystem()->setMaxIterations( 100 );
   theCollisionSystem()->setErrorReductionParameter( erp );
   theCollisionSystem()->setSplitImpulse( splitImpulse );

   if( vtk )
      // writeEmptyFiles=true: the collector.pvd is written at activation time and only lists
      // body types that exist at that moment, so force all types to be listed/written.
      vtk::activateWriter( outDir, outSteps, 0, steps, false, true );

   // Parallel: slabs along the belt direction over the container length (end slabs unbounded).
   lp::decomposeSlabs( 0, -0.5, 0.5 );

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
   pe_GLOBAL_SECTION
   {
   createPlane( ++id,  0.0,  0.0,  1.0, -0.2, wall, false );   // floor  z = -0.2
   createPlane( ++id,  1.0,  0.0,  0.0, -0.5, wall, false );   // wall   x = -0.5
   createPlane( ++id, -1.0,  0.0,  0.0, -0.5, wall, false );   // wall   x =  0.5
   createPlane( ++id,  0.0,  1.0,  0.0, -0.2, wall, false );   // wall   y = -0.2
   createPlane( ++id,  0.0, -1.0,  0.0, -0.2, wall, false );   // wall   y =  0.2
   }

   // Conveyor belt (conveyor.stl): rectangle z = 0, x in [-0.1,0.5], y in [-0.2,0.2].
   // Modelled as a thin fixed box whose top face is the belt surface. It is shrunk by 1 mm
   // in x and y so that it does not touch the container walls (no fixed-fixed contacts).
   const real  beltThickness( 0.02 );
   const Vec3  beltCenter( 0.2, 0.0, -0.5*beltThickness );
   const Vec3  beltLengths( 0.6 - 0.001, 0.4 - 0.002, beltThickness );
   BoxID belt( 0 );
   pe_GLOBAL_SECTION
   {
      belt = createBox( ++id, beltCenter, beltLengths, wall );
      belt->setFixed( true );
      if( movingBelt )
         belt->setLinearVel( beltVel );   // moving fixed body (needs MOBILE_INFINITE), reset every step
      else
         belt->setSurfaceVel( beltVel );  // surface velocity: friction only, the belt stays in place
   }

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
   rout() << "\n--CONVEYOR (pe port of the LIGGGHTS tutorial)------------------------------------\n"
             << " processes            = " << lp::numProcesses() << "\n"
             << " time step            = " << dt << " s\n"
             << " simulated time       = " << tEnd << " s  (" << steps << " steps)\n"
             << " insertion            = " << massPerIns << " kg every " << tInsert
             << " s until " << massTotal << " kg\n"
             << " number fraction r=" << radii[0] << " = " << numFrac0 << "\n"
             << " belt velocity        = " << beltVel << " m/s ("
             << ( movingBelt ? "moving fixed box, reset every step" : "surface velocity" ) << ")\n"
             << " friction per material= " << friction << "  (pair value " << 2*friction << "), erp " << erp
             << ( splitImpulse ? " (split impulse)" : "" ) << "\n"
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

   rout() << std::setw(8) << "step" << std::setw(10) << "time" << std::setw(8) << "atoms"
          << std::setw(14) << "mass_ins" << std::setw(14) << "ke" << std::setw(14) << "vmax"
          << std::setw(10) << "n_belt" << std::setw(12) << "vx_belt" << "\n";
   lp::Thermo th;
   const real beltLayerTop( 2.0 * radii[1] );   // centres of the bottom layer on the belt
   real vxBeltSum( 0 );                         // time average over the second half of the run
   unsigned int vxBeltSamples( 0 );

   for( unsigned int step=0; step<steps; ++step )
   {
      //--- particle insertion (like fix insert/stream) ----------------------------------------
      if( step % insSteps == 0 && massInserted < massTotal )
      {
         real target( std::min( massPerIns + massCarry, massTotal - massInserted ) );
         std::vector<Candidate> existing;   // spheres of all processes
         lp::gatherSpheres( world, existing );
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
               found = !lp::overlaps( pos, r, pending, existing );
            }
            if( !found ) { ++failures; continue; }

            Candidate c; c.pos = pos; c.r = r;
            pending.push_back( c );
            placed += m;
         }

         // Same candidates and ids on every process; each particle is created by its owner.
         for( std::vector<Candidate>::const_iterator c=pending.begin(); c!=pending.end(); ++c ) {
            ++id;
            ++nParticles;
            if( !world->ownsPoint( c->pos ) ) continue;
            SphereID s = createSphere( id, c->pos, c->r, granular );
            s->setLinearVel( 0.0, 0.0, -insertVel );
         }
         lp::synchronizeIfParallel( world );
         massInserted += placed;
         massCarry     = target - placed;
         ++nInsertions;
         rout() << " insertion " << nInsertions << ": " << pending.size() << " particles, "
                << placed << " kg (total " << massInserted << " kg)\n";
      }

      //--- one hard-contact time step -----------------------------------------------------------
      world->simulationStep( dt );

      // --moving-belt: the belt is a fixed body moving with the belt velocity. pe integrates the
      // position of fixed bodies with their velocity, so put the belt back where it belongs and
      // keep the velocity for the next contact solve. (A surface velocity needs neither.)
      if( movingBelt ) {
         belt->setPosition( beltCenter );
         belt->setLinearVel( beltVel );
      }

      //--- screen output (like thermo) ----------------------------------------------------------
      if( (step+1) % thermoSteps == 0 || step+1 == steps )
      {
         th.measure( world );   // reduced over all processes
         unsigned int nBelt( 0 );
         const real vxBelt( beltLayerVelocity( world, -0.1, 0.5, beltLayerTop, nBelt ) );
         if( (step+1)*dt > 0.5*tEnd && nBelt > 0 ) {
            vxBeltSum += vxBelt;
            ++vxBeltSamples;
         }
         rout() << std::setw(8) << step+1 << std::setw(10) << std::fixed << std::setprecision(4) << (step+1)*dt
                << std::setw(8) << th.n
                << std::setw(14) << std::setprecision(4) << massInserted
                << std::setw(14) << std::scientific << std::setprecision(5) << th.ke
                << std::setw(14) << std::fixed << std::setprecision(4) << th.vmax
                << std::setw(10) << nBelt << std::setw(12) << vxBelt << "\n" << std::flush;
      }
   }

   timer.end();

   //---------------------------------------------------------------------------------------------
   // Summary
   //---------------------------------------------------------------------------------------------
   th.measure( world );
   rout() << "--------------------------------------------------------------------------------\n"
          << " particles           = " << th.n << " (" << nParticles << " inserted)\n"
          << " mass inserted       = " << massInserted << " kg in " << nInsertions << " insertions\n"
          << " particle x range    = [" << th.lo[0] << ", " << th.hi[0] << "]\n"
          << " particle z range    = [" << th.zmin() << ", " << th.zmax() << "]\n"
          << " belt x / belt layer vx (mean over t > " << 0.5*tEnd << " s) = " << belt->getPosition()[0] << " / "
          << ( vxBeltSamples > 0 ? vxBeltSum / vxBeltSamples : real(0) ) << " m/s\n"
          << " wall-clock time     = " << timer.total() << " s for " << steps << " steps\n"
          << "--------------------------------------------------------------------------------\n";

   // Self check (identical verdict on every process: everything is reduced): every inserted
   // particle still exists, all centres are inside the container (within 2 r_max of the walls
   // and the floor; the top is open), and the energies are finite.
   const real tol( 2.0 * radii[1] );
   bool ok( true );
   if( th.n != nParticles )                   { rout() << " CHECK FAILED: " << th.n << " particles, " << nParticles << " inserted\n"; ok = false; }
   if( !( th.ke == th.ke ) || th.ke > 1e4 )   { rout() << " CHECK FAILED: kinetic energy not finite\n"; ok = false; }
   if( th.n > 0 && ( th.lo[0] < -0.5 - tol || th.hi[0] > 0.5 + tol || th.lo[1] < -0.2 - tol || th.hi[1] > 0.2 + tol
                     || th.zmin() < -0.2 - tol || th.zmax() > 2.0 ) ) {
      rout() << " CHECK FAILED: particles outside the container, x [" << th.lo[0] << ", " << th.hi[0] << "], y ["
             << th.lo[1] << ", " << th.hi[1] << "], z [" << th.zmin() << ", " << th.zmax() << "]\n"; ok = false;
   }
   rout() << ( ok ? " self check ok\n" : " SELF CHECK FAILED\n" );
   return ok ? 0 : 1;
}
//*************************************************************************************************
