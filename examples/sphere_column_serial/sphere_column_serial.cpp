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
 */
//=================================================================================================

#include <iostream>
#include <stdexcept>

#include <pe/core.h>
#include <pe/util.h>
#include <pe/vtk.h>

using namespace pe;

int main( int /*argc*/, char* /*argv*/[] )
{
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
             << "\n"
             << std::string(58, '-') << "\n";

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
                   << "\n";
      }

      world->simulationStep( dt );
   }

   return 0;
}
