//=================================================================================================
/*!\file pe/interface/setup/dkt.h
 *  \brief Draft-kiss-tumble benchmark (two settling spheres).
 *
 *  Entry point: commf2c_dkt_().
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
#include <iostream>
#include <string>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/support/decompose.h>

namespace pe {

//*************************************************************************************************
/*!\brief PE setup for the draft-kiss-tumble benchmark (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_dkt_(). Two equal spheres are released one above the
 * other, slightly offset in x, in a 2 x 2 x 8 column. The column has a ground plane at
 * z = 0 and four side walls at x = 0, x = 2, y = 0 and y = 2; it is open at the top.
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density and radius
 * (benchRadius_), the process layout (processesX_/Y_/Z_), the step size, and the VTK switch
 * and spacing.
 *
 * Fixed in this file: the column size and the two sphere start positions.
 *
 * Sphere user IDs are numbered per process and are therefore not unique across processes.
 * Use the system ID (getSystemID()) wherever a globally unique ID is needed.
 *
 * Every error path aborts the run with a message.
 */
inline void setupDraftKissTumbBench(MPI_Comm ex0) {

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
  world = theWorld();

  loadSimulationConfig("example.json");

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

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupDraftKissTumbBench: " << message << "\n" << std::endl;
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

  const real radBench = config.getBenchRadius();
  if( radBench <= real(0) ) {
    abortSetup("benchRadius_ must be positive.");
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

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
    std::cout << "3D coordinates were created" << std::endl;
    std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
  }

  // The decomposition covers exactly the column between the walls for any process layout.
  // (The subdomain size in x and y used to be fixed to 2.0, which matched the walls only
  // for processesX_ = processesY_ = 1; for that layout nothing has changed.)
  const real dx( LX / px );
  const real dy( LY / py );
  const real dz( LZ / pz );

  decomposeDomain(center, 0.0, 0.0, 0.0, dx, dy, dz, px, py, pz);

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
     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );   // ground plane

     createPlane( 1778,+1.0, 0.0, 0.0, 0,   granite, false );     // wall at x = 0
     createPlane( 1779,-1.0, 0.0, 0.0,-LX,  granite, false );     // wall at x = LX
     createPlane( 1780, 0.0, 1.0, 0.0, 0,   granite, false );     // wall at y = 0
     createPlane( 1781, 0.0,-1.0, 0.0,-LY,  granite, false );     // wall at y = LY
  }

  MaterialID myMaterial = createMaterial("Bench", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // User IDs are numbered per process (see the function documentation)
  int idx = 0;
  for( const Vec3& position : spherePositions ) {
    if (world->ownsPoint( position )) {
      SphereID sphere = createSphere(idx, position, radBench, myMaterial, true);
      std::cout << "[Creating particle] at: " << position << " in domain: " << myRank << std::endl;
      std::cout << "[particle mass]: " << sphere->getMass()  << std::endl;
      std::cout << "[particle volume]: " << real(4.0)/real(3.0) * M_PI * radBench * radBench * radBench << std::endl;
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
  unsigned long bodiesLocal    = static_cast<unsigned long>( theCollisionSystem()->getBodyStorage().size() );
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  MPI_Reduce( &particlesLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesLocal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "DRAFT-KISS-TUMBLE SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Column size                             = " << Vec3(LX, LY, LZ) << "\n"
      << " Particle radius                         = " << radBench << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n" << std::endl;
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
