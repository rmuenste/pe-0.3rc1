//=================================================================================================
/*!
 *  \file sphere_column_serial.cpp
 *  \brief Serial single-sphere SRR floor-impact test.
 *
 *  Validates the Short-Range Repulsion solver (Pan et al. 2002, Eq. 2.1) by dropping a single
 *  sphere and tracking its deceleration inside the security zone.
 *
 *  Parameters are converted from Pan et al. (2002) CGS to SI:
 *
 *    Pan et al. CGS                          SI
 *    ──────────────────────────────────────────────────────────────────
 *    radius   a  = 0.3175 cm            → 3.175e-3 m
 *    SRR zone ρ  = 0.06858 cm           → 6.858e-4 m   (= 0.2168 a)
 *    density  ρs = 1.14 g/cm³           → 1140 kg/m³
 *    stiffness ε = 5e-7 dyne⁻¹          → 5e-2 N⁻¹
 *                 (1 dyne⁻¹ = 1e5 N⁻¹)
 *
 *  With dt = 1e-3 s and F_max/m ≈ 131,000 m/s², sub-cycling at N=5000
 *  gives dt_sub = 2e-7 s, limiting Δv per sub-step to ~0.026 m/s << v_entry.
 *  Optional velocity damping (γ) is set for near-critical settling.
 *  Compile-time guard: this binary requires pe_CONSTRAINT_SOLVER =
 *  pe::response::ShortRangeRepulsion.
 */
//=================================================================================================

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <string>
#include <vector>

#include <pe/core.h>
#include <pe/util.h>
#include <pe/vtk.h>

#include <pe/support.h>

using namespace pe;

// Visualization variables
bool g_vtk( true );
bool singleSphereTest( false );
std::string g_vtk_output_dir( "./paraview" );
int g_vtk_spacing( 10 );
unsigned int g_numRows( 5 );

void doSingleSphereTest() {

   const unsigned int timesteps( 300 );
   const real         dt( real(1e-3) );

   // --- Pan et al. (2002) parameters in SI ---
   const real radParticle( real(3.175e-3) );  // sphere radius          [m]  (0.3175 cm)
   const real rhoParticle( real(1140)     );  // particle density  [kg/m³]  (1.14 g/cm³)
   const real rhoSRR     ( real(6.858e-4) );  // security zone width    [m]  (0.06858 cm)
   const real epsSRR     ( real(5e-2)     );  // stiffness         [N⁻¹]   (5e-7 dyne⁻¹ → 5e-2 N⁻¹)
   const real gammaSRR   ( real(0.5)      );  // velocity damping  [N·s/m]  (~critical near equilibrium)
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );

   MaterialID mat = createMaterial( "test", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   createPlane( 9999, 0.0, 0.0, 1.0, 0.0, mat, true );  // floor at z = 0

   // Configure ShortRangeRepulsion solver
   // Note: rhoSRR == default (6.858e-4), so lubrication threshold set in constructor stays in sync.
   theCollisionSystem()->getContactSolver().setRho  ( rhoSRR  );
   theCollisionSystem()->getContactSolver().setEpsP ( epsSRR  );
   theCollisionSystem()->getContactSolver().setEpsW ( epsSRR  );
   theCollisionSystem()->getContactSolver().setGamma( gammaSRR );

   // Sub-cycling: F_max/m ≈ 131,000 m/s²; N=5000 → dt_sub=2e-7 s → Δv_sub ≈ 0.026 m/s
   theCollisionSystem()->setNumSubcycles( 5000 );

   // Drop sphere from gap = 10ρ above floor
   SphereID s = createSphere( 0,
                              Vec3( 0.0, 0.0, radParticle + real(10)*rhoSRR ),
                              radParticle, mat, true );

   std::cout << std::fixed << std::setprecision(8);
   std::cout << "\n--- Single-sphere SRR floor-impact test (Pan et al. 2002, SI params) ---\n"
             << "  radius    = " << radParticle << " m  (0.3175 cm)\n"
             << "  rho_s     = " << rhoParticle << " kg/m3  (1.14 g/cm3)\n"
             << "  rho (SRR) = " << rhoSRR      << " m  (0.06858 cm)\n"
             << "  eps       = " << epsSRR       << " N^-1  (5e-7 dyne^-1 x 1e5)\n"
             << "  gamma     = " << gammaSRR     << " N.s/m\n"
             << "  nSubcycles= 5000,  dt = 1e-3 s  =>  dt_sub = 2e-7 s\n"
             << "  Initial gap = 10*rho = " << real(10)*rhoSRR << " m\n\n"
             << std::setw(6)  << "step"
             << std::setw(14) << "z [m]"
             << std::setw(14) << "gap [m]"
             << std::setw(10) << "gap/rho"
             << std::setw(14) << "vz [m/s]"
             << std::setw(12) << "t_step[ms]"
             << "\n"
             << std::string(70, '-') << "\n";

   // Setup of the VTK visualization
   if( g_vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( g_vtk_output_dir.c_str(), g_vtk_spacing, 0, timesteps, false);
   }

   double last_step_ms = 0.0;

   for( unsigned int timestep = 0; timestep <= timesteps; ++timestep ) {
      const real z   = s->getPosition()[2];
      const real vz  = s->getLinearVel()[2];
      const real gap = z - radParticle;

      // Print every step for first 60 steps; then every 5
      if( timestep <= 60 || timestep % 5 == 0 ) {
         std::cout << std::setw(6)  << timestep
                   << std::setw(14) << z
                   << std::setw(14) << gap
                   << std::setw(10) << std::setprecision(4) << gap / rhoSRR
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

void doColumnTest()
{
   // Pan et al. (2002) thin-column geometry (orig.txt, converted to SI):
   //   x ∈ [0, 0.2030 m]   (20.30 cm)
   //   y ∈ [0, 6.858e-3 m]  (0.686 cm — barely wider than one sphere diameter)
   //   floor at z = 0 (open top)

   const unsigned int timesteps( 300 );
   const real         dt( real(1e-3) );
   // --- Pan et al. (2002) parameters in SI ---
   const real radParticle( real(3.175e-3) );  // sphere radius          [m]  (0.3175 cm)
   const real dParticle  ( real(2) * radParticle );
   const real rhoParticle( real(1140)     );  // particle density  [kg/m³]  (1.14 g/cm³)
   const real rhoSRR     ( real(6.858e-4) );  // security zone width    [m]  (0.06858 cm)
   const real epsSRR     ( real(5e-2)     );  // stiffness         [N⁻¹]   (5e-7 dyne⁻¹)
   const real gammaSRR   ( real(0.5)      );  // velocity damping  [N·s/m]
   const real dEff       ( dParticle + rhoSRR );  // centre pitch with surface gap = rho

   // Column box dimensions (SI)
   const real boxX( real(0.2030)   );   // 20.30 cm
   const real boxY( real(6.858e-3) );   // 0.686 cm  (= 2r + 0.508mm clearance)

   // Stacked-bed layout: one layer in y, several rows in z, spheres spread across x.
   const unsigned int nPerRow = static_cast<unsigned int>( std::floor( boxX / dEff ) );
   const unsigned int nRows   = g_numRows;
   const unsigned int nTotal  = nPerRow * nRows;
   const real rowWidth = real(nPerRow - 1) * dEff + dParticle;
   const real xOffset  = ( boxX - rowWidth ) / real(2) + radParticle;
   const real y0       = boxY / real(2);
   const real zBase    = radParticle + rhoSRR / real(2);

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );

   MaterialID mat = createMaterial( "test", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11 );

   // Bounding box: floor + 4 side walls (open top).
   // Plane equation: n·x = d;  bodies live on the positive side (n·x > d).
   createPlane( 9000,  0.0,  0.0,  1.0,  0.0,   mat, true );  // floor:      z = 0
   createPlane( 9001,  1.0,  0.0,  0.0,  0.0,   mat, true );  // left wall:  x = 0
   createPlane( 9002, -1.0,  0.0,  0.0, -boxX,  mat, true );  // right wall: x = boxX
   createPlane( 9003,  0.0,  1.0,  0.0,  0.0,   mat, true );  // back wall:  y = 0
   createPlane( 9004,  0.0, -1.0,  0.0, -boxY,  mat, true );  // front wall: y = boxY

   // Configure ShortRangeRepulsion solver
   theCollisionSystem()->getContactSolver().setRho  ( rhoSRR  );
   theCollisionSystem()->getContactSolver().setEpsP ( epsSRR  );
   theCollisionSystem()->getContactSolver().setEpsW ( epsSRR  );
   theCollisionSystem()->getContactSolver().setGamma( gammaSRR );
   theCollisionSystem()->setNumSubcycles( 5000 );

   std::vector<SphereID> spheres;
   spheres.reserve( nTotal );
   int id = 0;
   for( unsigned int row = 0; row < nRows; ++row ) {
      const real zc = zBase + real(row) * dEff;
      for( unsigned int col = 0; col < nPerRow; ++col ) {
         const real xc = xOffset + real(col) * dEff;
         spheres.push_back( createSphere( id++, Vec3( xc, y0, zc ), radParticle, mat, true ) );
      }
   }

   const real bedHeight0 = zBase + real(nRows - 1) * dEff + radParticle;
   const real gapY = y0 - radParticle;

   std::cout << std::fixed << std::setprecision(6)
             << "\n--- Stacked-sphere SRR column test (Pan et al. 2002 geometry) ---\n"
             << "  box x          = " << boxX*real(1e2) << " cm  y = " << boxY*real(1e3) << " mm\n"
             << "  radius         = " << radParticle*real(1e3) << " mm\n"
             << "  d_eff          = " << dEff*real(1e3) << " mm  (d + rho)\n"
             << "  y-gap          = " << gapY*real(1e3)
             <<    " mm  (rhoSRR = " << rhoSRR*real(1e3) << " mm)"
             <<    "  → ALWAYS inside y-wall SRR zone\n"
             << "  spheres/row    = " << nPerRow << "\n"
             << "  rows           = " << nRows << "\n"
             << "  total spheres  = " << nTotal << "\n"
             << "  bed height(t=0)= " << bedHeight0*real(1e2) << " cm  ("
             <<    bedHeight0/real(2.54e-2) << " in)\n"
             << "  rho (SRR)      = " << rhoSRR  << " m\n"
             << "  eps            = " << epsSRR  << " N^-1\n"
             << "  gamma          = " << gammaSRR << " N.s/m\n"
             << "  nSubcycles     = 5000,  dt = 1e-3 s  =>  dt_sub = 2e-7 s\n"
             << "\n"
             << std::setw(6)  << "step"
             << std::setw(14) << "bedH [cm]"
             << std::setw(14) << "bedH [in]"
             << std::setw(14) << "minGap/rho"
             << std::setw(14) << "maxVz [m/s]"
             << std::setw(12) << "t_step[ms]"
             << "\n"
             << std::string(60, '-') << "\n";

   if( g_vtk ) {
      vtk::activateWriter( g_vtk_output_dir.c_str(), g_vtk_spacing, 0, timesteps, false );
   }

   double last_step_ms = 0.0;

   for( unsigned int timestep = 0; timestep <= timesteps; ++timestep ) {

      if( timestep <= 60 || timestep % 5 == 0 ) {
         real maxZ     = -std::numeric_limits<real>::max();
         real minGap   = std::numeric_limits<real>::max();
         real maxVzMag = real(0);

         for( unsigned int row = 0; row < nRows; ++row ) {
            for( unsigned int col = 0; col < nPerRow; ++col ) {
               const SphereID& sphere = spheres[row * nPerRow + col];
               const Vec3 pos = sphere->getPosition();
               const Vec3 vel = sphere->getLinearVel();
               const real top = pos[2] + radParticle;

               if( top > maxZ ) maxZ = top;

               const real gapFloor = pos[2] - radParticle;
               if( gapFloor < minGap ) minGap = gapFloor;

               if( row > 0 ) {
                  const SphereID& below = spheres[(row - 1) * nPerRow + col];
                  const real centerGap = pos[2] - below->getPosition()[2];
                  const real surfGap = centerGap - dParticle;
                  if( surfGap < minGap ) minGap = surfGap;
               }

               const real vzAbs = std::abs( vel[2] );
               if( vzAbs > maxVzMag ) maxVzMag = vzAbs;
            }
         }

         std::cout << std::setw(6)  << timestep
                   << std::setw(14) << std::setprecision(4) << maxZ * real(1e2)
                   << std::setw(14) << maxZ / real(2.54e-2)
                   << std::setw(14) << minGap / rhoSRR
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


   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "file",          value<std::string>()->default_value(""), "obj mesh file to be loaded" )
     ( "vtk-dir",       value<std::string>()->default_value("./paraview"), "output directory for ParaView VTK files" )
     ( "vtk-spacing",   value<int>()->default_value(10), "write VTK output every N timesteps" )
     ( "num-rows",      value<unsigned int>()->default_value(5), "number of sphere rows in column test" )
     ( "single-sphere", "run single-sphere floor-impact test instead of column test" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      g_vtk = false;
   g_vtk_output_dir = vm["vtk-dir"].as<std::string>();
   g_vtk_spacing = vm["vtk-spacing"].as<int>();
   if( g_vtk_spacing <= 0 ) {
      throw std::invalid_argument( "vtk-spacing must be positive" );
   }
   g_numRows = vm["num-rows"].as<unsigned int>();
   if( g_numRows == 0 ) {
      throw std::invalid_argument( "num-rows must be positive" );
   }
   if( vm.count( "single-sphere" ) > 0 )
      singleSphereTest = true;

   if (singleSphereTest) {
     doSingleSphereTest();
   }
   else {
     doColumnTest();
   }

   return 0;
}
