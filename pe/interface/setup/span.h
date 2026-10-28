//=================================================================================================
/*!\file pe/interface/setup/span.h
 *  \brief Span (chip) FSI benchmark: one triangle mesh in a channel.
 *
 *  Entry point: commf2c_fsi_(). setupSpanSerial() is an alias of setupFSIBenchSerial().
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_SPAN_H_
#define _PE_INTERFACE_SETUP_SPAN_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <iostream>
#include <string>
#include <pe/config/SimulationConfig.h>
#include <pe/core/detection/fine/DistanceMap.h>
#include <pe/interface/support/decompose.h>

namespace pe {

using namespace pe::povray;

//*************************************************************************************************
/*!\brief PE setup for the span (chip) case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_fsi_() (the entry point keeps its historical name).
 * The case is a single free triangle-mesh chip ("chip1.obj") with DistanceMap acceleration
 * in a fixed, non-periodic box without boundary planes.
 *
 * Read from example.json: gravity, fluid density and viscosity, step size, the process layout
 * (processesX_/Y_/Z_), and the VTK switch and spacing.
 *
 * Fixed in this file: the domain size, the chip mesh file, its position and its density.
 *
 * Every error path aborts the run with a message.
 */
inline void setupSpan(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real LX(  6.0 );                      // Domain size, origin at (0,0,0)
  const real LY(  6.0 );
  const real LZ( 15.0 );

  const std::string chipFile( "chip1.obj" );
  const real chipDensity( 1000.0 );           // No config parameter available
  const Vec3 chipPos( 3.0, 3.0, 7.5 );        // No config parameter available

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();
  world = theWorld();

  loadSimulationConfig("example.json");

  const real simRho( config.getFluidDensity() );
  const real simViscosity( config.getFluidViscosity() );

  world->setGravity( config.getGravity() );
  world->setLiquidSolid(true);
  world->setLiquidDensity( simRho );
  world->setViscosity( simViscosity );
  world->setDamping( 1.0 );

  TimeStep::stepsize( config.getStepsize() );

  //===============================================================================================
  // MPI system and validation
  //===============================================================================================
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupSpan: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  if( px*py*pz != mpisystem->getSize() ) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px*py*pz) + " (processesX_*Y_*Z_).");
  }

  //===============================================================================================
  // 3D rectilinear, non-periodic domain decomposition
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

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
    std::cout << "3D coordinates were created" << std::endl;
    std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
  }

  const real dx( LX / px );
  const real dy( LY / py );
  const real dz( LZ / pz );

  decomposeDomain(center, 0.0, 0.0, 0.0, dx, dy, dz, px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Materials
  //===============================================================================================
  // TODO: "ground" and "tool" are not assigned to any body. They are kept on purpose: removing
  //       them shifts the material index of "chip", which matters wherever materials are
  //       identified by their index (e.g. checkpoints). Remove them once that is confirmed safe.
  MaterialID gr      = createMaterial("ground", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID toolMat = createMaterial("tool", chipDensity, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID chipMat = createMaterial("chip", chipDensity, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);
  (void)gr;
  (void)toolMat;

  //===============================================================================================
  // Body creation: the chip, created on the process that owns its position
  //===============================================================================================
  if( world->ownsPoint(chipPos) ) {
    TriangleMeshID chip = createTriangleMesh(1, chipPos, chipFile, chipMat, false, true);
    std::cout << "Chip is owned by domain: " << mpisystem->getRank() << " initially." << std::endl;
    std::cout << "Chip x:[" << chip->getAABB()[0] << "," << chip->getAABB()[3] << "]" << std::endl;

    // NOTE: the DistanceMap is built here, on the process that creates the chip, and nowhere
    //       else. How the map behaves when the chip migrates to another process (or is seen
    //       as a shadow copy) is a known open point that is handled separately. Keep this
    //       block as it is until that is resolved.
    chip->enableDistanceMapAcceleration(64, 3);  // resolution, tolerance
    if (!chip->hasDistanceMap()) {
      std::cerr << "WARNING: DistanceMap acceleration failed to initialize for chip" << std::endl;
    } else {
      std::cout << "DistanceMap acceleration enabled successfully for chip!" << std::endl;
      const DistanceMap* dm = chip->getDistanceMap();
      if (dm) {
        std::cout << "DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
        std::cout << "DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
        std::cout << "DistanceMap spacing: " << dm->getSpacing() << std::endl;
      }

      chip->calcBoundingBox();
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
  unsigned long meshesLocal( 0 );
  unsigned long bodiesLocal( 0 );
  for (unsigned int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++) {
    BodyID body = world->getBody(j);
    if (body->getType() == triangleMeshType) {
      ++meshesLocal;
    }
    ++bodiesLocal;
  }

  unsigned long meshesTotal( 0 );
  unsigned long bodiesTotal( 0 );
  MPI_Reduce( &meshesLocal, &meshesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesLocal, &bodiesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SPAN (CHIP) SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of triangle meshes         = " << meshesTotal << "\n"
      << " Total number of objects                 = " << bodiesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Chip Density                            = " << chipDensity << "\n"
      << " Chip mesh file                          = " << chipFile << "\n"
      << " Chip position                           = " << chipPos << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain size                             = " << Vec3(LX, LY, LZ) << "\n"
      << " Domain volume                           = " << LX * LY * LZ << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

}  // namespace pe
#endif  // HAVE_MPI

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief FSI benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupFSIBenchSerial(int cfd_rank) {
  // Set custom rank for PE logger BEFORE any logging occurs
  // This ensures each CFD domain gets a unique log file: pe<cfd_rank>.log
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

  //==============================================================================================
  // Simulation Input Parameters 
  //==============================================================================================
  // Set gravity from configuration
  world->setGravity( config.getGravity() );

  // Default fluid properties
  real slipLength(0.75);

  // Configuration from config singleton
  real simViscosity( config.getFluidViscosity() );
  real simRho( config.getFluidDensity() );

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  // CRITICAL: Enable automatic force reset after each simulation step
  // This prevents force accumulation when using substepping
  world->setAutoForceReset(true);

  //==============================================================================================
  // Visualization Configuration
  //==============================================================================================
  if (isRepresentative && config.getVtk()) {
      // Adjust VTK output spacing for substepping
      // With substepping, world->simulationStep() is called substeps times per main timestep
      // To maintain the same VTK output frequency, multiply spacing by substeps
      unsigned int effectiveVisspacing = config.getVisspacing() * config.getSubsteps();
      vtk::WriterID vtk = vtk::activateWriter( "./paraview", effectiveVisspacing, 0,
                                               config.getTimesteps() * config.getSubsteps(),
                                               false);
  }

  // Activate checkpointer if configured
  if (isRepresentative && config.getUseCheckpointer()) {
    activateCheckpointer(config.getCheckpointPath(),
                         effectiveCheckpointSpacing(config),
                         0, config.getTimesteps());
  }

  // Serial mode: no MPI, no domain decomposition
  // The entire simulation domain is owned by this process
  // Domain boundaries are handled by the CFD code
  // Create ground plane (global, owned by this domain)
  //
  //
  //
  // Setup of the ground plane
  int idx = 0;
  PlaneID plane = createPlane( idx++, 0.0, 0.0, 1.0, -0.0, granite );
  // +y
  createPlane( idx++, 0.0, 1.0, 0.0,  0.0, granite );
  // -y
  createPlane( idx++, 0.0,-1.0, 0.0,  -0.02, granite );

  //==============================================================================================
  // Bench Configuration
  //==============================================================================================
  real radBench = config.getBenchRadius();
  real rhoParticle( config.getParticleDensity() );
  Vec3 position = config.getBenchStartPosition();
  std::string fileName = std::string("span_cm.obj");

  // Set default timestep
  TimeStep::stepsize(config.getStepsize());
  MaterialID chipMat = createMaterial("chip"    , rhoParticle , 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);

  Vec3 chipPos = position;
  TriangleMeshID chip = createTriangleMesh(++idx, chipPos, fileName, chipMat, false, true);

#ifdef PE_USE_CGAL
  // Enable DistanceMap acceleration for the chip
  chip->enableDistanceMapAcceleration(64, 3);  // spacing, resolution, tolerance
  const bool distanceMapEnabled = chip->hasDistanceMap();
  const DistanceMap* dm = distanceMapEnabled ? chip->getDistanceMap() : nullptr;
  if (!distanceMapEnabled && isRepresentative) {
    std::cerr << "WARNING: DistanceMap acceleration failed to initialize for chip" << std::endl;
  }
#else
  const bool distanceMapEnabled = false;
  std::cout << "WARNING: DistanceMap acceleration failed to initialize for chip" << std::endl;
#endif

  if (isRepresentative) {
    std::cout << "\n--" << "SIMULATION SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Triangle mesh file                      = " << fileName << "\n"
              << " VTK output                              = " << (config.getVtk() ? "enabled" : "disabled") << "\n"
              << " Distance map enabled                    = " << (distanceMapEnabled ? "yes" : "no") << "\n";

#ifdef PE_USE_CGAL
    if (distanceMapEnabled && dm) {
      std::cout << " Distance map grid size                  = "
                << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << "\n"
                << " Distance map origin                     = ("
                << dm->getOrigin()[0] << ", "
                << dm->getOrigin()[1] << ", "
                << dm->getOrigin()[2] << ")\n"
                << " Distance map spacing                    = " << dm->getSpacing() << "\n";
    }
#endif

    std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

}

/**
 * @brief Span setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupSpanSerial(int cfd_rank) {
  setupFSIBenchSerial(cfd_rank);
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_SPAN_H_
