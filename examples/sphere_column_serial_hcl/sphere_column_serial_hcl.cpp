//=================================================================================================
/*!
 *  \file sphere_column_serial_hcl.cpp
 *  \brief Serial stacked-sphere HardContactLubricated (HCL) thin-column test.
 *
 *  Mirrors the ShortRangeRepulsion sphere_column_serial example but uses the
 *  HardContactLubricated solver (Stokes lubrication + hard-contact complementarity).
 *  Intended for side-by-side comparison of SRR vs HCL settling behaviour,
 *  equilibrium position, and per-step cost.
 *
 *  Parameters are converted from Pan et al. (2002) CGS to SI:
 *
 *    Pan et al. CGS                          SI
 *    ──────────────────────────────────────────────────────────────────
 *    radius   a  = 0.3175 cm            → 3.175e-3 m
 *    SRR zone ρ  = 0.06858 cm           → 6.858e-4 m   (= 0.2168 a)
 *    density  ρs = 1.14 g/cm³           → 1140 kg/m³
 *
 *  HCL uses physical Stokes lubrication instead of SRR penalty forces.
 *  Fluid properties (water at ~20°C): μ = 1e-3 Pa·s, ρ_f = 1000 kg/m³.
 *  Time step is 10× smaller than SRR (1e-4 s vs 1e-3 s) since HCL has
 *  no sub-cycling; total simulation time remains 0.3 s.
 *
 *  Compile-time guard: this binary requires pe_CONSTRAINT_SOLVER =
 *  pe::response::HardContactLubricated in pe/config/Collisions.h.
 */
//=================================================================================================

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <pe/core.h>
#include <pe/util.h>
#include <pe/vtk.h>

#include <pe/support.h>

// NOTE: This example requires pe_CONSTRAINT_SOLVER = pe::response::HardContactLubricated
// in pe/config/Collisions.h.  If the wrong solver is configured, the HCL-specific API calls
// (setLubricationThreshold, setMinEpsLub, etc.) will produce compile errors.

using namespace pe;

bool g_vtk( true );
bool singleSphereTest( false );
unsigned int g_numRows( 5 );  // configurable via --num-rows

//=================================================================================================
// Single-sphere floor-impact test (HCL version)
//=================================================================================================
void doSingleSphereTestHCL()
{
   const unsigned int timesteps( 3000 );
   const real         dt( real(1e-4) );

   // --- Pan et al. (2002) parameters in SI ---
   const real radParticle( real(3.175e-3) );  // sphere radius          [m]
   const real rhoParticle( real(1140)     );  // particle density  [kg/m³]
   const real lubThreshold( real(6.858e-4) ); // lubrication threshold  [m]  (matches SRR ρ)

   // Fluid properties (water at ~20°C)
   const real liquidVisc  ( real(1e-3)  );    // dynamic viscosity [Pa·s]
   const real liquidDens  ( real(1000)  );    // density           [kg/m³]

   const int  visspacing = 100;

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setViscosity( liquidVisc );
   world->setLiquidDensity( liquidDens );
   world->setLiquidSolid( true );

   MaterialID mat = createMaterial( "test", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   createPlane( 9999, 0.0, 0.0, 1.0, 0.0, mat, true );  // floor at z = 0

   // Configure HardContactLubricated solver
   theCollisionSystem()->setLubricationThreshold( lubThreshold );
   theCollisionSystem()->setMinEpsLub( real(1e-8) );
   theCollisionSystem()->setAlphaImpulseCap( real(1.0) );
   theCollisionSystem()->setMaxIterations( 100 );
   theCollisionSystem()->setErrorReductionParameter( real(0.8) );

   // Drop sphere from gap = 10*lubThreshold above floor
   SphereID s = createSphere( 0,
                              Vec3( 0.0, 0.0, radParticle + real(10)*lubThreshold ),
                              radParticle, mat, true );

   std::cout << std::fixed << std::setprecision(8);
   std::cout << "\n--- Single-sphere HCL floor-impact test (Pan et al. 2002 geometry) ---\n"
             << "  radius         = " << radParticle << " m  (0.3175 cm)\n"
             << "  rho_s          = " << rhoParticle << " kg/m3\n"
             << "  lubThreshold   = " << lubThreshold << " m  (matches SRR rho)\n"
             << "  minEpsLub      = 1e-8\n"
             << "  alphaImpulseCap= 1.0\n"
             << "  viscosity      = " << liquidVisc  << " Pa.s\n"
             << "  liquidDensity  = " << liquidDens  << " kg/m3\n"
             << "  dt = 1e-4 s,  timesteps = 3000  =>  T_total = 0.3 s\n"
             << "  Initial gap = 10*threshold = " << real(10)*lubThreshold << " m\n\n"
             << std::setw(6)  << "step"
             << std::setw(14) << "z [m]"
             << std::setw(14) << "gap [m]"
             << std::setw(10) << "gap/thr"
             << std::setw(14) << "vz [m/s]"
             << std::setw(12) << "t_step[ms]"
             << "\n"
             << std::string(70, '-') << "\n";

   if( g_vtk ) {
      vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false );
   }

   double last_step_ms = 0.0;

   for( unsigned int timestep = 0; timestep <= timesteps; ++timestep ) {
      const real z   = s->getPosition()[2];
      const real vz  = s->getLinearVel()[2];
      const real gap = z - radParticle;

      // Print every step for first 600 steps; then every 50
      if( timestep <= 600 || timestep % 50 == 0 ) {
         std::cout << std::setw(6)  << timestep
                   << std::setw(14) << z
                   << std::setw(14) << gap
                   << std::setw(10) << std::setprecision(4) << gap / lubThreshold
                   << std::setprecision(8)
                   << std::setw(14) << vz
                   << std::fixed << std::setprecision(2) << std::setw(12) << last_step_ms
                   << "\n";
      }

      auto t0 = std::chrono::steady_clock::now();
      world->simulationStep( dt );
      last_step_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0 ).count();
   }
}

//=================================================================================================
// Thin-column stacked-sphere test (HCL version) — Pan et al. (2002) geometry
//=================================================================================================
void doColumnTestHCL()
{
   // Pan et al. (2002) thin-column geometry (converted to SI):
   //   x ∈ [0, 0.2030 m]   (20.30 cm)
   //   y ∈ [0, 6.858e-3 m]  (0.686 cm — barely wider than one sphere diameter)
   //   floor at z = 0 (open top)
   //
   // Spheres are stacked in a simple square lattice with effective pitch
   //   d_eff = d + ρ = 0.635 cm + 0.06858 cm = 0.70358 cm
   // ensuring surface-to-surface separation ≥ ρ at t=0.
   //
   // N_row = floor(W / d_eff) = floor(20.30 / 0.70358) = 28 per row (single layer in y).
   // Number of rows (vertical layers) is configurable via --num-rows.
   // Full Pan et al. setup: 43 rows × 28 = 1204 spheres.

   const unsigned int timesteps( 3000 );
   const real         dt( real(1e-4) );
   const int          visspacing( 100 );

   // --- Pan et al. (2002) parameters in SI ---
   const real radParticle ( real(3.175e-3) );  // sphere radius          [m]  (0.3175 cm)
   const real dParticle   ( real(2) * radParticle );  // diameter          [m]
   const real rhoParticle ( real(1140)     );  // particle density  [kg/m³]
   const real lubThreshold( real(6.858e-4) );  // lubrication threshold  [m]  (= ρ = 0.06858 cm)

   // Effective pitch: center-to-center distance that gives surface gap = ρ
   const real dEff( dParticle + lubThreshold );  // 0.70358 cm in SI: 7.0358e-3 m

   // Fluid properties (water at ~20°C)
   const real liquidVisc  ( real(1e-3)  );     // dynamic viscosity [Pa·s]
   const real liquidDens  ( real(1000)  );     // density           [kg/m³]

   // Column box dimensions (SI)
   const real boxX( real(0.2030)   );   // 20.30 cm
   const real boxY( real(6.858e-3) );   // 0.686 cm  (= 2r + 0.508mm clearance)

   // Stacking layout
   const unsigned int nPerRow = static_cast<unsigned int>( std::floor( boxX / dEff ) );  // 28
   const unsigned int nRows   = g_numRows;
   const unsigned int nTotal  = nPerRow * nRows;

   // Center the row in x: leftover space is split equally on both sides
   // Row occupies (nPerRow - 1) * dEff + d = width of sphere centres + one diameter
   const real rowWidth = real(nPerRow - 1) * dEff + dParticle;
   const real xOffset  = ( boxX - rowWidth ) / real(2) + radParticle;  // x-centre of first sphere

   // y-centre: single layer, centred in the narrow channel
   const real y0 = boxY / real(2);

   // z-layout: first row centre at r + ρ/2 (gap to floor = ρ/2), then spaced by dEff
   const real zBase = radParticle + lubThreshold / real(2);

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setViscosity( liquidVisc );
   world->setLiquidDensity( liquidDens );
   world->setLiquidSolid( true );

   MaterialID mat = createMaterial( "test", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11 );

   // Bounding box: floor + 4 side walls (open top).
   createPlane( 9000,  0.0,  0.0,  1.0,  0.0,   mat, true );  // floor:      z = 0
   createPlane( 9001,  1.0,  0.0,  0.0,  0.0,   mat, true );  // left wall:  x = 0
   createPlane( 9002, -1.0,  0.0,  0.0, -boxX,  mat, true );  // right wall: x = boxX
   createPlane( 9003,  0.0,  1.0,  0.0,  0.0,   mat, true );  // back wall:  y = 0
   createPlane( 9004,  0.0, -1.0,  0.0, -boxY,  mat, true );  // front wall: y = boxY

   // Configure HardContactLubricated solver
   theCollisionSystem()->setLubricationThreshold( lubThreshold );
   theCollisionSystem()->setMinEpsLub( real(1e-8) );
   theCollisionSystem()->setAlphaImpulseCap( real(1.0) );
   theCollisionSystem()->setMaxIterations( 100 );
   theCollisionSystem()->setErrorReductionParameter( real(0.8) );

   // Create sphere stack: simple square lattice, single layer in y
   std::vector<SphereID> spheres;
   spheres.reserve( nTotal );
   int id = 0;
   for( unsigned int row = 0; row < nRows; ++row ) {
      const real zc = zBase + real(row) * dEff;
      for( unsigned int col = 0; col < nPerRow; ++col ) {
         const real xc = xOffset + real(col) * dEff;
         spheres.push_back( createSphere( id++, Vec3(xc, y0, zc), radParticle, mat, true ) );
      }
   }

   // Initial bed height = top of highest sphere
   const real bedHeight0 = zBase + real(nRows - 1) * dEff + radParticle;
   const real gapY = y0 - radParticle;

   std::cout << std::fixed << std::setprecision(6)
             << "\n--- Stacked-sphere HCL column test (Pan et al. 2002 geometry) ---\n"
             << "  box x          = " << boxX*real(1e2) << " cm  y = " << boxY*real(1e3) << " mm\n"
             << "  radius         = " << radParticle*real(1e3) << " mm\n"
             << "  d_eff          = " << dEff*real(1e3) << " mm  (d + rho)\n"
             << "  y-gap          = " << gapY*real(1e3)
             <<    " mm  (lubThreshold = " << lubThreshold*real(1e3) << " mm)"
             <<    "  -> ALWAYS inside y-wall lubrication zone\n"
             << "  spheres/row    = " << nPerRow << "\n"
             << "  rows           = " << nRows << "\n"
             << "  total spheres  = " << nTotal << "\n"
             << "  bed height(t=0)= " << bedHeight0*real(1e2) << " cm  ("
             <<    bedHeight0/real(2.54e-2) << " in)\n"
             << "  lubThreshold   = " << lubThreshold << " m\n"
             << "  viscosity      = " << liquidVisc  << " Pa.s\n"
             << "  liquidDensity  = " << liquidDens  << " kg/m3\n"
             << "  dt = 1e-4 s,  timesteps = " << timesteps << "\n"
             << "\n"
             << std::setw(6)  << "step"
             << std::setw(14) << "bedH [cm]"
             << std::setw(14) << "bedH [in]"
             << std::setw(14) << "minGap/thr"
             << std::setw(14) << "maxVz [m/s]"
             << std::setw(12) << "t_step[ms]"
             << "\n"
             << std::string(76, '-') << "\n";

   if( g_vtk ) {
      vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false );
   }

   double last_step_ms = 0.0;

   for( unsigned int timestep = 0; timestep <= timesteps; ++timestep ) {

      if( timestep <= 200 || timestep % 50 == 0 ) {
         // Compute bed statistics across all spheres
         real maxZ  = -std::numeric_limits<real>::max();
         real minGap = std::numeric_limits<real>::max();
         real maxVzMag = real(0);

         for( size_t i = 0; i < spheres.size(); ++i ) {
            const Vec3 pos = spheres[i]->getPosition();
            const Vec3 vel = spheres[i]->getLinearVel();
            const real top = pos[2] + radParticle;
            if( top > maxZ ) maxZ = top;
            const real gapFloor = pos[2] - radParticle;
            if( gapFloor < minGap ) minGap = gapFloor;
            const real vzAbs = std::abs( vel[2] );
            if( vzAbs > maxVzMag ) maxVzMag = vzAbs;
         }

         std::cout << std::setw(6)  << timestep
                   << std::setw(14) << std::setprecision(4) << maxZ * real(1e2)
                   << std::setw(14) << maxZ / real(2.54e-2)
                   << std::setw(14) << minGap / lubThreshold
                   << std::setw(14) << std::setprecision(6) << maxVzMag
                   << std::fixed << std::setprecision(2) << std::setw(12) << last_step_ms
                   << "\n";
      }

      auto t0 = std::chrono::steady_clock::now();
      world->simulationStep( dt );
      last_step_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0 ).count();
   }
}


int main( int argc, char* argv[] )
{
   setSeed( 12345 );

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "file",          value<std::string>()->default_value(""), "obj mesh file to be loaded" )
     ( "single-sphere", "run single-sphere floor-impact test instead of column test" )
     ( "num-rows",      value<unsigned int>()->default_value(5), "number of sphere rows in column test (full Pan et al. = 43)" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      g_vtk = false;
   if( vm.count( "single-sphere" ) > 0 )
      singleSphereTest = true;
   g_numRows = vm["num-rows"].as<unsigned int>();

   if( singleSphereTest ) {
      doSingleSphereTestHCL();
   }
   else {
      doColumnTestHCL();
   }

   return 0;
}
