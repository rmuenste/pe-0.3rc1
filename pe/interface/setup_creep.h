//=================================================================================================
/*!
 *  \file pe/interface/setup_creep.h
 *  \brief PE setup for the Creep Flow case (parallel PE mode)
 *
 *  Copyright (C) 2024 Raphael Münster
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

#ifndef _PE_INTERFACE_SETUP_CREEP_H_
#define _PE_INTERFACE_SETUP_CREEP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <exception>
#include <iostream>
#include <string>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/setup_optional_collision_params.h>


using namespace pe::povray;




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief PE setup for the Creep Flow case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_creep_(). A single heavy cylinder with a prescribed
 * initial linear and angular velocity in a gravity-free fluid. The domain is a fixed,
 * non-periodic box that is decomposed rectilinearly according to processesX_/Y_/Z_ of the
 * configuration.
 *
 * Read from the configuration file (SimulationConfig::getConfigFile()): the process layout, step size, number
 * of time steps, the VTK switch and spacing, and the lubrication settings (applied by
 * commf2c_creep_() after this setup). Fluid properties, gravity, the domain box and the
 * cylinder are fixed by the case.
 *
 * Every error path aborts the run with a message.
 */
void setupCreep(MPI_Comm ex0) {
  auto& config = SimulationConfig::getInstance();

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&]( const std::string& message ) {
    if( myRank == 0 ) {
      std::cerr << "\nERROR in setupCreep: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  try {
    SimulationConfig::loadConfigFile( myRank == 0 );
  }
  catch( const std::exception& e ) {
    abortSetup( e.what() );
  }

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Fluid properties
  const real simViscosity( 1.0 );
  const real simRho( 1.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  TimeStep::stepsize( config.getStepsize() );

  // Switch and slip length of the legacy lubricated solvers (setLubrication()); the json
  // lubrication settings are applied by commf2c_creep_() after this setup
  const bool useLubrication(false);
  const real slipLength( 0.01 );

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  // Checking the total number of MPI processes
  if( px <= 0 || py <= 0 || pz <= 0 ) {
    abortSetup("processesX_, processesY_ and processesZ_ must be positive.");
  }
  if( px*py*pz != mpisystem->getSize() ) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px*py*pz) + " (processesX_*processesY_*processesZ_).");
  }

  //===============================================================================================
  // Setup of the MPI processes: 3D rectilinear domain decomposition
  //===============================================================================================

  // Origin and size of the domain
  const real bx( -10.0 );
  const real by(  -5.0 );
  const real bz(   0.0 );
  const real LX( 45.0 );
  const real LY( 25.0 );
  const real LZ(  0.5 );

  // Size of a single subdomain
  const real dx( LX/px );
  const real dy( LY/py );
  const real dz( LZ/pz );

  // Non-periodic cartesian communicator with one process per subdomain
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

  pe_LOG_INFO_SECTION( log ) {
    log << "Process grid: " << dims[0] << " " << dims[1] << " " << dims[2]
        << ", coordinates of this process: " << center[0] << " " << center[1] << " "
        << center[2] << "\n";
  }

  decomposeDomain(center, bx, by, bz,
                  dx, dy, dz,
                  px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  // Setup of the VTK visualization
  if( config.getVtk() ) {
     vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
  }

  //===============================================================================================
  // Collision system parameters
  //===============================================================================================
  setOptionalLubrication(theCollisionSystem(), useLubrication);
  setOptionalSlipLength(theCollisionSystem(), slipLength);
  theCollisionSystem()->setMinEps(0.01);
  theCollisionSystem()->setMaxIterations(200);

  //===============================================================================================
  // Body creation: one heavy cylinder, created on the process that owns its position
  //===============================================================================================
  MaterialID heavy = createMaterial( "heavy", 10.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

  const Vec3 cylinderPos( 0.001, 0.0, 0.25 );
  if( world->ownsPoint(cylinderPos) ) {
    CylinderID cylinder = createCylinder( 1, cylinderPos, 1.0, 0.5001, heavy );
    cylinder->rotate( 0.0, 0.5 * M_PI, 0.0 );
    cylinder->setAngularVel( 0.0, 0.0, -50.0 );
    cylinder->setLinearVel( 100.0, 0.0, 0.0 );
    pe_LOG_INFO_SECTION( log ) {
      log << "Cylinder bounding box size in x = "
          << cylinder->getAABB()[3] - cylinder->getAABB()[0] << "\n";
    }
  }

  // Synchronization of the MPI processes
  world->synchronize();

  //===============================================================================================
  // Setup summary
  //===============================================================================================
  // Every body counts once: the bodies a process owns (not its shadow copies of neighbouring
  // bodies), plus the global bodies, which every process holds
  unsigned long bodiesLocal( 0 );
  unsigned long globalBodies( 0 );
  for( World::Iterator body = world->begin(); body != world->end(); ++body ) {
    if( body->isGlobal() )
      ++globalBodies;
    else if( !body->isRemote() )
      ++bodiesLocal;
  }
  unsigned long bodiesTotal( 0 );
  MPI_Reduce( &bodiesLocal, &bodiesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  bodiesTotal += globalBodies;

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "CREEP FLOW SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of objects                 = " << bodiesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Lubrication (json)                      = " << (config.getLubricationEnabled() ? "enabled" : "disabled") << "\n"
      << " Lubrication h_c (slip length)           = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain origin                           = " << Vec3(bx, by, bz) << "\n"
      << " Domain size                             = " << Vec3(LX, LY, LZ) << "\n"
      << " Domain volume                           = " << LX * LY * LZ << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

#endif
