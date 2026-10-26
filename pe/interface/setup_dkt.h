//=================================================================================================
/*!
 *  \file pe/interface/setup_dkt.h
 *  \brief PE setup for the draft-kiss-tumble benchmark (parallel PE mode)
 *
 *  Copyright (C) 2023 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_SETUP_DKT_H_
#define _PE_INTERFACE_SETUP_DKT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <string>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/setup_config.h>




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief PE setup for the draft-kiss-tumble benchmark (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_dkt_(). Two equal spheres are released one above the
 * other, slightly offset in x, in a 2 x 2 x 8 column. The column has a ground plane at
 * z = 0 and four side walls at x = 0, x = 2, y = 0 and y = 2; it is open at the top.
 *
 * Read from the configuration file (SimulationConfig::getConfigFile()): gravity, fluid density
 * and viscosity, particle density and radius (benchRadius_), the contact material of the
 * spheres (restitution_, staticFriction_, dynamicFriction_), the process layout
 * (processesX_/Y_/Z_), the step size, and the VTK switch and spacing. The radius is checked
 * before anything is created: both spheres have to fit into the column without overlapping.
 *
 * Fixed in this file: the column size and the two sphere start positions.
 *
 * Sphere user IDs are numbered per process and are therefore not unique across processes.
 * Use the system ID (getSystemID()) wherever a globally unique ID is needed.
 *
 * Every error path aborts the run with a message.
 */
void setupDraftKissTumbBench(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real LX( 2.0 );                       // Column size, origin at (0,0,0)
  const real LY( 2.0 );
  const real LZ( 8.0 );

  const Vec3 spherePositions[] = { Vec3( 0.99, 1.0, 6.9 ),    // Leading (lower) sphere
                                   Vec3( 1.0,  1.0, 7.2 ) };  // Trailing (upper) sphere

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupDraftKissTumbBench: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  loadConfigFileOnAllRanks(ex0, "setupDraftKissTumbBench");

  world = theWorld();

  world->setGravity( config.getGravity() );
  world->setLiquidSolid(true);
  world->setLiquidDensity( config.getFluidDensity() );
  world->setViscosity( config.getFluidViscosity() );
  world->setDamping( 1.0 );

  TimeStep::stepsize( config.getStepsize() );

  //===============================================================================================
  // MPI system and validation
  //===============================================================================================
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  if( px <= 0 || py <= 0 || pz <= 0 ) {
    abortSetup("processesX_, processesY_ and processesZ_ must be positive.");
  }
  if( px*py*pz != mpisystem->getSize() ) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px*py*pz) + " (processesX_*Y_*Z_).");
  }

  const real radBench = config.getBenchRadius();
  if( radBench <= real(0) ) {
    abortSetup("benchRadius_ must be positive.");
  }

  // Both spheres have to lie inside the column and must not overlap at the start
  for( const Vec3& position : spherePositions ) {
    if( position[0] - radBench <= real(0) || position[0] + radBench >= LX ||
        position[1] - radBench <= real(0) || position[1] + radBench >= LY ||
        position[2] - radBench <= real(0) || position[2] + radBench >= LZ ) {
      abortSetup("benchRadius_ = " + std::to_string(radBench) + " is too large: the sphere at " +
                 "z = " + std::to_string(position[2]) + " does not fit into the column.");
    }
  }
  if( ( spherePositions[1] - spherePositions[0] ).length() <= real(2) * radBench ) {
    abortSetup("benchRadius_ = " + std::to_string(radBench) + " is too large: the two spheres, " +
               std::to_string(( spherePositions[1] - spherePositions[0] ).length()) +
               " apart, would overlap at the start.");
  }

  //===============================================================================================
  // 3D rectilinear, non-periodic domain decomposition of the column
  //===============================================================================================
  int dims   [] = { px, py, pz };
  int periods[] = { false, false, false };
  int reorder   = false;
  MPI_Comm cartcomm;

  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if( cartcomm == MPI_COMM_NULL ) {
    abortSetup("failed to create the cartesian communicator.");
  }
  mpisystem->setComm(cartcomm);

  // Cartesian coordinates of this process within the process grid
  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  // The decomposition covers exactly the column between the walls for any process layout.
  // (The subdomain size in x and y used to be fixed to 2.0, which matched the walls only
  // for processesX_ = processesY_ = 1; for that layout nothing has changed.)
  const real dx( LX / px );
  const real dy( LY / py );
  const real dz( LZ / pz );

  decomposeDomain(center, 0.0, 0.0, 0.0, dx, dy, dz, px, py, pz);

  pe_LOG_INFO_SECTION( log ) {
    log << "Subdomain (" << center[0] << "," << center[1] << "," << center[2] << ") of "
        << px << " x " << py << " x " << pz << ": x in [" << center[0] * dx << ", "
        << ( center[0] + 1 ) * dx << "], y in [" << center[1] * dy << ", "
        << ( center[1] + 1 ) * dy << "], z in [" << center[2] * dz << ", "
        << ( center[2] + 1 ) * dz << "]\n";
  }

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Ground plane, side walls and the two spheres
  //===============================================================================================
  // TODO: materials may be identified by their index elsewhere (e.g. checkpoints). Keep the
  //       creation order ("ground" first, then "Bench") until index-based use is ruled out.
  MaterialID gr = createMaterial("ground", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_GLOBAL_SECTION
  {
     createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );              // ground plane

     createPlane( 1778,+1.0, 0.0, 0.0, 0,   granite, false );     // wall at x = 0
     createPlane( 1779,-1.0, 0.0, 0.0,-LX,  granite, false );     // wall at x = LX
     createPlane( 1780, 0.0, 1.0, 0.0, 0,   granite, false );     // wall at y = 0
     createPlane( 1781, 0.0,-1.0, 0.0,-LY,  granite, false );     // wall at y = LY
  }

  // Contact material from restitution_/staticFriction_/dynamicFriction_ (defaults 0.0/0.1/0.05
  // are the former hard-coded values), as in setupDraftKissTumbSerial()
  MaterialID myMaterial = createMaterial("Bench", config.getParticleDensity(), config.getRestitution(),
                                         config.getStaticFriction(), config.getDynamicFriction(),
                                         0.2, 80, 100, 10, 11);

  // User IDs are numbered per process (see the function documentation)
  int idx = 0;
  for( const Vec3& position : spherePositions ) {
    if (world->ownsPoint( position )) {
      SphereID sphere = createSphere(idx, position, radBench, myMaterial, true);
      pe_LOG_INFO_SECTION( log ) {
        log << "Created sphere " << idx << " at " << position << ", mass " << sphere->getMass()
            << "\n";
      }
      ++idx;
    }
  }

  // Synchronization of the MPI processes
  world->synchronize();

  // Setup of the VTK visualization
  if( config.getVtk() ) {
    vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
  }

  //===============================================================================================
  // Setup summary
  //===============================================================================================
  unsigned long particlesLocal = static_cast<unsigned long>(idx);
  unsigned long particlesTotal ( 0 );
  MPI_Reduce( &particlesLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "DRAFT-KISS-TUMBLE SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Process grid (x, y, z)                  = " << px << " x " << py << " x " << pz << "\n"
      << " Subdomain size                          = " << Vec3(dx, dy, dz) << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Column size                             = " << Vec3(LX, LY, LZ) << "\n"
      << " Particle radius                         = " << radBench << "\n"
      << " Restitution / static / dynamic friction = " << config.getRestitution() << " / "
      << config.getStaticFriction() << " / " << config.getDynamicFriction() << "\n"
      << " Total number of particles               = " << particlesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;

    if( particlesTotal != 2 ) {
      std::cerr << "\nERROR in setupDraftKissTumbBench: expected 2 spheres, created "
                << particlesTotal << ".\n" << std::endl;
      MPI_Abort(cartcomm, 1);
    }
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

#endif
