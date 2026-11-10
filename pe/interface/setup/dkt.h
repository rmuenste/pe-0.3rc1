//=================================================================================================
/*!\file pe/interface/setup/dkt.h
 *  \brief Draft-kiss-tumble benchmark and walled settling column (spheres from an xyz file).
 *
 *  Entry point: commf2c_dkt_(). Both halves build the same world: a ground plane at z = 0 and
 *  the spheres of xyzFilePath_; the parallel half additionally decomposes the CFD column it is
 *  handed into the processesX_/Y_/Z_ grid.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_DKT_H_
#define _PE_INTERFACE_SETUP_DKT_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/support/decompose.h>
#include <pe/interface/support/geometry_utils.h>

namespace pe {

//*************************************************************************************************
/*!\brief PE setup for the draft-kiss-tumble and walled-column cases (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 * \param xmin Global minimum of the CFD mesh in x (the column), passed by the CFD side.
 * \param xmax Global maximum of the CFD mesh in x.
 * \param ymin Global minimum of the CFD mesh in y.
 * \param ymax Global maximum of the CFD mesh in y.
 * \param zmin Global minimum of the CFD mesh in z; must be 0, see below.
 * \param zmax Global maximum of the CFD mesh in z.
 *
 * Reached from Fortran through commf2c_dkt_(). Builds the same world as
 * setupDraftKissTumbSerial(), so that the two PE modes solve one case: the ground plane at
 * z = 0 with the "ground" material, and the spheres listed in xyzFilePath_ (the DKT pair, the
 * D3.2 settling clouds) with the "Bench" material whose density, restitution and friction come
 * from the json config, as do gravity, the fluid properties and the step size. The materials are
 * registered in the serial order ("ground", then "Bench"). There are no side walls on the PE side
 * in either mode: the CFD mesh boundary confines the fluid, and the spheres never reach it.
 *
 * Parallel-mode specifics: the column [xmin,xmax] x [ymin,ymax] x [zmin,zmax] is decomposed into
 * the processesX_ x processesY_ x processesZ_ rectilinear grid of the json config, whose product
 * must equal the size of ex0; the CFD mesh partition must be the same Cartesian cut (see
 * doc/technical-notes/pe-interface-serial-mode.md). Each sphere is created once, on the rank whose
 * subdomain owns its start position, with its index in the xyz file as user id (the serial
 * numbering). Sphere system IDs are what the CFD side prints.
 *
 * Refused with a message and MPI_Abort: a process grid that does not match ex0, a degenerate box,
 * zmin != 0 (both halves put the ground plane at z = 0), resume_ = true (no pe checkpoint is
 * written in parallel PE mode), an xyz file without positions, a sphere start position outside
 * the box (it would never be created), and a created-sphere count that differs from the file.
 */
inline void setupDraftKissTumbBench(MPI_Comm ex0,
                                    real xmin, real xmax,
                                    real ymin, real ymax,
                                    real zmin, real zmax) {

  //===============================================================================================
  // Configuration, world and fluid properties (as in the serial half)
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();
  world = theWorld();

  loadSimulationConfig("example.json");

  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupDraftKissTumbBench: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  world->setGravity( config.getGravity() );
  world->setLiquidSolid(true);
  world->setLiquidDensity( config.getFluidDensity() );
  world->setViscosity( config.getFluidViscosity() );
  world->setDamping( 1.0 );

  // The PE stepsize must match the CFD deck TimeStep; take it from the config
  TimeStep::stepsize( config.getStepsize() );

  const real sphereRad = config.getBenchRadius();
  if( sphereRad <= real(0) ) {
    abortSetup("benchRadius_ must be positive.");
  }

  if( config.getResume() ) {
    abortSetup("resume_ = true, but no pe checkpoint is written in parallel PE mode "
               "(SimPar@PeCheckpointOnDump is refused there); run the segment from scratch "
               "or use the serial-mode build.");
  }

  //===============================================================================================
  // Process grid and column box
  //===============================================================================================
  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  if( px < 1 || py < 1 || pz < 1 ) {
    abortSetup("processesX_/Y_/Z_ must all be at least 1.");
  }
  if( px*py*pz != mpisystem->getSize() ) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px*py*pz) + " (processesX_*Y_*Z_).");
  }

  if( !(xmax > xmin) || !(ymax > ymin) || !(zmax > zmin) ) {
    abortSetup("degenerate CFD domain extents passed by the CFD side.");
  }
  // Both halves create the ground plane at z = 0 (setupDraftKissTumbSerial hard-codes it); a
  // column that starts elsewhere would put the floor inside or below the fluid.
  if( std::abs(zmin) > real(1e-12) * std::max(real(1), std::abs(zmax)) ) {
    abortSetup("the CFD domain starts at z = " + std::to_string(zmin) +
               ", but the ground plane of this case is at z = 0 in both PE modes.");
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

  const real dx( (xmax - xmin) / px );
  const real dy( (ymax - ymin) / py );
  const real dz( (zmax - zmin) / pz );

  decomposeDomain(center, xmin, ymin, zmin, dx, dy, dz, px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Materials, ground plane and spheres (values and order as in the serial half)
  //===============================================================================================
  // Materials are registered in this fixed order: checkpointed bodies carry a bare index into
  // the material table (see setupATCSerial).
  MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Contact material from config: restitution_/staticFriction_/dynamicFriction_
  // json keys (defaults 0.0/0.1/0.05 reproduce the former hard-coded values)
  MaterialID particleMaterial =
      createMaterial("Bench", config.getParticleDensity(), config.getRestitution(),
                     config.getStaticFriction(), config.getDynamicFriction(),
                     0.2, 80, 100, 10, 11);

  pe_GLOBAL_SECTION
  {
     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );   // floor contact plane at z = 0
  }

  // Sphere positions are read from the external xyz file, on every rank
  const std::string xyzPath = config.getXyzFilePath().string();
  const std::vector<Vec3> spherePositions = readVectorsFromFile(xyzPath);

  if( spherePositions.empty() ) {
    abortSetup("could not read any particle positions from '" + xyzPath + "'.");
  }

  int localCreated = 0;
  for( std::size_t i = 0; i < spherePositions.size(); ++i ) {
    const Vec3& position = spherePositions[i];
    if( position[0] < xmin || position[0] > xmax ||
        position[1] < ymin || position[1] > ymax ||
        position[2] < zmin || position[2] > zmax ) {
      std::ostringstream msg;
      msg << "sphere " << i << " of '" << xyzPath << "' starts at " << position
          << ", outside the CFD domain [" << xmin << "," << xmax << "] x [" << ymin << "," << ymax
          << "] x [" << zmin << "," << zmax << "]; no rank would own it.";
      abortSetup(msg.str());
    }
    if( world->ownsPoint( position ) ) {
      createSphere( static_cast<id_t>(i), position, sphereRad, particleMaterial );
      ++localCreated;
    }
  }

  // Synchronization of the MPI processes
  world->synchronize();

  // Setup of the VTK visualization
  if( config.getVtk() ) {
    vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
  }

  //===============================================================================================
  // Setup summary and consistency check
  //===============================================================================================
  unsigned long particlesLocal = static_cast<unsigned long>(localCreated);
  unsigned long bodiesLocal    = static_cast<unsigned long>( theCollisionSystem()->getBodyStorage().size() );
  unsigned long particlesTotal ( 0 );
  unsigned long bodiesTotal    ( 0 );
  MPI_Reduce( &particlesLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesLocal, &bodiesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "DKT SETUP (parallel PE mode)"
      << "----------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz
      << " (" << px << " x " << py << " x " << pz << ")\n"
      << " CFD domain                              = [" << xmin << "," << xmax << "] x ["
      << ymin << "," << ymax << "] x [" << zmin << "," << zmax << "]\n"
      << " Subdomain size                          = " << Vec3(dx, dy, dz) << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Fluid viscosity                         = " << config.getFluidViscosity() << "\n"
      << " Fluid density                           = " << config.getFluidDensity() << "\n"
      << " Gravity                                 = " << world->getGravity() << "\n"
      << " Particle radius                         = " << sphereRad << "\n"
      << " Particle density                        = " << config.getParticleDensity() << "\n"
      << " Position file                           = " << xyzPath << "\n";
    for( std::size_t i = 0; i < spherePositions.size(); ++i ) {
      std::cout << " Sphere " << i << " position                       = "
                << spherePositions[i] << "\n";
    }
    std::cout << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of bodies (incl. plane)    = " << bodiesTotal << "\n"
      << "--------------------------------------------------------------------------------\n"
      << std::endl;

    if( particlesTotal != spherePositions.size() ) {
      std::cerr << "\nERROR in setupDraftKissTumbBench: '" << xyzPath << "' lists "
                << spherePositions.size() << " spheres, but " << particlesTotal
                << " were created (every start position must be owned by exactly one subdomain).\n"
                << std::endl;
      MPI_Abort(cartcomm, 1);
    }
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

}  // namespace pe
#endif  // HAVE_MPI

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief Draft-Kiss-Tumble benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDraftKissTumbSerial(int cfd_rank) {
  // Set custom rank for PE logger BEFORE any logging occurs
  pe::logging::Logger::setCustomRank(cfd_rank);

  // Load configuration from JSON file
  SimulationConfig::loadFromFile("example.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();

  // Apply lubrication/contact parameters from configuration
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  // Gravity and fluid properties come from the configuration, never hardcoded
  world->setGravity(config.getGravity());

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());
  const real sphereRad(config.getBenchRadius());

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  // The PE stepsize must match the CFD deck TimeStep; take it from the config
  TimeStep::stepsize(config.getStepsize());

  // Materials are registered on BOTH paths and in this fixed order: checkpointed bodies carry a
  // bare index into the material table (see setupATCSerial), so a resumed run must build the
  // same table before it reads the checkpoint.
  MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Contact material from config: restitution_/staticFriction_/dynamicFriction_
  // json keys (defaults 0.0/0.1/0.05 reproduce the former hard-coded values)
  MaterialID particleMaterial =
      createMaterial("Bench", rhoParticle, config.getRestitution(),
                     config.getStaticFriction(), config.getDynamicFriction(),
                     0.2, 80, 100, 10, 11);

  const bool resume = config.getResume();
  const std::string xyzPath = config.getXyzFilePath().string();
  std::vector<Vec3> spherePositions;
  int idx = 0;
  int particlesCreated = 0;

  if (resume) {
    // Segmented-run resume (row d52_v25f_l4_protocol): the driver's dump restores only the
    // fluid, and re-creating the cloud from the xyz file re-inserts it at rest. The pe
    // checkpoint written by the driver at its dump instant (pe_write_checkpoint_) carries
    // positions, orientations, linear AND angular velocities of every body, including the
    // ground plane (global body), so nothing is created here. Loads on every rank: each CFD
    // rank runs its own serial pe instance.
    resumeFromConfiguredCheckpoint(config);
    for (auto it = world->begin(); it != world->end(); ++it) {
      if (it->getType() == sphereType) ++particlesCreated;
    }
    if (particlesCreated == 0) {
      throw std::runtime_error("setupDraftKissTumbSerial: resume checkpoint '" +
                               config.getResumeCheckpointFile() + "' holds no spheres");
    }
  } else {
    // Floor contact plane at z = 0
    createPlane(777, 0.0, 0.0, 1.0, 0, gr, true);

    // Sphere positions are read from the external xyz file
    spherePositions = readVectorsFromFile(xyzPath);

    if (spherePositions.empty()) {
      std::cerr << "ERROR: DKT setup could not read any particle positions from '"
                << xyzPath << "'" << std::endl;
      throw std::runtime_error("setupDraftKissTumbSerial: no particle positions read from '" +
                               xyzPath + "'");
    }

    for (const auto &spherePos : spherePositions) {
      createSphere(idx++, spherePos, sphereRad, particleMaterial);
      ++particlesCreated;
    }
  }

  if (isRepresentative && resume) {
    std::cout << "\n--" << "DKT SETUP (RESUMED from pe checkpoint)"
              << "----------------------------------------------\n"
              << " Checkpoint                              = " << config.getResumeCheckpointFile() << "\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;
  } else if (isRepresentative) {
    std::cout << "\n--" << "DKT SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Particle radius                         = " << sphereRad << "\n"
              << " Particle density                        = " << rhoParticle << "\n"
              << " Position file                           = " << xyzPath << "\n";
    for (std::size_t i = 0; i < spherePositions.size(); ++i) {
      std::cout << " Sphere " << i << " position                       = "
                << spherePositions[i] << "\n";
    }
    std::cout << " Total number of particles               = " << particlesCreated << "\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_DKT_H_
