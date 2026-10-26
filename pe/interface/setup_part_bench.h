//=================================================================================================
/*!
 *  \file pe/interface/setup_part_bench.h
 *  \brief PE setup for the single-particle sedimentation benchmark (parallel PE mode)
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

#ifndef _PE_INTERFACE_SETUP_PART_BENCH_H_
#define _PE_INTERFACE_SETUP_PART_BENCH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <exception>
#include <iostream>
#include <string>

#include <pe/config/SimulationConfig.h>
#include <pe/core/MPITrait.h>
#include <pe/interface/decompose.h>
#include <pe/interface/setup_optional_collision_params.h>




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief PE setup for the single-particle sedimentation benchmark (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_(). A single sphere settles onto a ground plane at
 * z = 0.
 *
 * QUARTER DOMAIN: the PE domain decomposed here is only one quarter of the full benchmark
 * domain. It starts at the origin and extends in +x and +y only:
 *
 *    x in [0, processesX_*0.05],  y in [0, processesY_*0.05],  z in [0, 0.16]
 *
 * The sphere is created at x = y = 0. That is the vertical center axis of the FULL domain,
 * so the sphere lies exactly on the corner edge of the PE quarter domain. This is intended
 * and not a positioning error; do not "fix" it by moving the sphere or the domain origin.
 *
 * Read from the configuration file (SimulationConfig::getConfigFile()): gravity, fluid density and viscosity, particle density and radius
 * (benchRadius_), the process layout (processesX_/Y_/Z_), the step size, and the VTK switch
 * and spacing. The lubrication switch is applied by the entry point after this setup.
 *
 * Fixed in this file: the quarter-domain size, the sphere start height and the slip length.
 *
 * Every error path aborts the run with a message.
 */
void setupParticleBench(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real dx( 0.05 );                      // Subdomain size in x (quarter domain)
  const real dy( 0.05 );                      // Subdomain size in y (quarter domain)
  const real LZ( 0.16 );                      // Domain height

  const Vec3 position( 0.0, 0.0, 0.1275 );    // On the center axis of the full domain
  const real slipLength( 0.75 );

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();

  world = theWorld();

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupParticleBench: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  try {
    SimulationConfig::loadConfigFile(myRank == 0);
  }
  catch (const std::exception& e) {
    abortSetup(e.what());
  }

  const real simViscosity( config.getFluidViscosity() );
  const real simRho( config.getFluidDensity() );
  const real rhoParticle( config.getParticleDensity() );
  const real radBench( config.getBenchRadius() );

  world->setGravity( config.getGravity() );
  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity( simViscosity );
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
  if( px * py * pz != mpisystem->getSize() ) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px * py * pz) + " (processesX_*Y_*Z_).");
  }

  if( radBench <= real(0) ) {
    abortSetup("benchRadius_ must be positive.");
  }

  //===============================================================================================
  // 3D rectilinear, non-periodic decomposition of the quarter domain
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

  const real dz( LZ / pz );

  // Quarter domain: origin at (0,0,0), extending in +x/+y (see the function documentation)
  decomposeDomain(center, 0.0, 0.0, 0.0, dx, dy, dz, px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Materials, ground plane and the benchmark sphere
  //===============================================================================================
  // TODO: materials may be identified by their index elsewhere (e.g. checkpoints). Keep the
  //       creation order ("ground" first, then "Bench") until index-based use is ruled out.
  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
     createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );
  }

  setOptionalSlipLength(theCollisionSystem(), slipLength);
  MaterialID myMaterial = createMaterial("Bench", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  theCollisionSystem()->setMinEps(5e-6 / radBench);

  SphereID spear(nullptr);
  if (world->ownsPoint( position )) {
    spear = createSphere(0, position, radBench, myMaterial, true);
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

  unsigned long particlesLocal = (spear != nullptr) ? 1UL : 0UL;
  unsigned long particlesTotal( 0 );
  unsigned long primitivesTotal( 0 );
  MPI_Reduce( &particlesLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesLocal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  primitivesTotal += globalBodies;

  real sphereVol(0);
  real sphereMass(0);
  if (spear != nullptr) {
    sphereMass = spear->getMass();
    sphereVol = real(4.0)/real(3.0) * M_PI * radBench * radBench * radBench;
  }

  real totalMass(0);
  real totalVol(0);
  MPI_Reduce( &sphereMass, &totalMass, 1, MPITrait<real>::getType(), MPI_SUM, 0, cartcomm );
  MPI_Reduce( &sphereVol, &totalVol, 1, MPITrait<real>::getType(), MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "PARTICLE BENCH SETUP"
      << "--------------------------------------------------------------\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Process grid (x, y, z)                  = " << px << " x " << py << " x " << pz << "\n"
      << " Subdomain size                          = " << Vec3(dx, dy, dz) << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Lubrication (json)                      = " << (config.getLubricationEnabled() ? "enabled" : "disabled") << "\n"
      << " Lubrication h_c                         = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " PE quarter domain size                  = " << Vec3(px * dx, py * dy, LZ) << "\n"
      << " Particle starting position              = " << position << "\n"
      << " Particle radius                         = " << radBench << "\n"
      << " Particle mass                           = " << totalMass << "\n"
      << " Particle volume                         = " << totalVol << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

#endif
