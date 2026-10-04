#ifndef _PE_SETUP_CREEP_H_
#define _PE_SETUP_CREEP_H_

#include <cstdlib>
#include <iostream>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/setup_optional_collision_params.h>

using namespace pe::povray;

//=================================================================================================
// Setup for the Creep Flow case
//
// A single heavy cylinder with a prescribed initial linear and angular velocity in a
// gravity-free fluid. The domain is a fixed, non-periodic box that is decomposed
// rectilinearly according to processesX_/Y_/Z_ of the configuration.
//=================================================================================================
void setupCreep(MPI_Comm ex0) {
  auto& config = SimulationConfig::getInstance();

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Fluid properties
  const real simViscosity( 1.0 );
  const real simRho( 1.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication settings
  const bool useLubrication(false);
  const real slipLength( 0.01 );

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  // Checking the total number of MPI processes
  if( px*py*pz != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << px*py*pz << "\n\n" << std::endl;
     std::exit(EXIT_FAILURE);
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
     std::cout << "Error creating 3D communicator" << std::endl;
     MPI_Finalize();
     return;
  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
  }
  mpisystem->setComm(cartcomm);

  // Cartesian coordinates of this process within the process grid
  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "3D coordinates were created" << std::endl;
    std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
  }

  decomposeDomain(center, bx, by, bz,
                  dx, dy, dz,
                  px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
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
    std::cout << "Bounding box size = " << cylinder->getAABB()[3] - cylinder->getAABB()[0] << std::endl;
  }

  // Synchronization of the MPI processes
  world->synchronize();

  //===============================================================================================
  // Setup summary
  //===============================================================================================
  unsigned long bodiesLocal = static_cast<unsigned long>( theCollisionSystem()->getBodyStorage().size() );
  unsigned long bodiesTotal( 0 );
  MPI_Reduce( &bodiesLocal, &bodiesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "CREEP FLOW SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of objects                 = " << bodiesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Lubrication                             = " << (useLubrication ? "enabled" : "disabled") << "\n"
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

#endif
