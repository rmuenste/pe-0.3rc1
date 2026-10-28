//=================================================================================================
/*!\file pe/interface/setup/rotation.h
 *  \brief Rotation case: rotating four-leg boundary mesh (serial mode only).
 *
 *  Entry point: commf2c_rotation_(); the parallel variant aborts with a message.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_ROTATION_H_
#define _PE_INTERFACE_SETUP_ROTATION_H_

// No parallel setup: the parallel entry point aborts with a message.

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief Rotation application setup for serial mode
 *
 * Uses the ATC setup with domain boundary DistanceMap for rotation simulations.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupRotationSerial(int cfd_rank) {
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

  // Set gravity from configuration
  world->setGravity( config.getGravity() );
  world->setDamping(1.0);

  // Configuration from config singleton
  real simViscosity( config.getFluidViscosity() );
  real simRho( config.getFluidDensity() );

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(config.getStepsize());

  // Activate checkpointer if configured
  if (isRepresentative && config.getUseCheckpointer()) {
    activateCheckpointer(config.getCheckpointPath(),
                         effectiveCheckpointSpacing(config),
                         0, config.getTimesteps());
  }

  //==============================================================================================
  // Domain Boundary Configuration
  //==============================================================================================
  int idx = 0;
  std::string boundaryFileName = std::string("four_leg_scaled.obj");

  Vec3 boundaryPos(0.05, 0.05, 0.015);  // Boundary mesh in reference position
  MaterialID boundaryMat = createMaterial("boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Create fixed main rotor
  TriangleMeshID boundaryMesh = createTriangleMesh(idx++, boundaryPos, boundaryFileName, boundaryMat, false, true);
  boundaryMesh->setFixed(true);
  boundaryMesh->setPosition(boundaryPos);
  boundaryMesh->setAngularVel(Vec3(0.0, 0.0, 2.0 * M_PI));

#ifdef PE_USE_CGAL
  // Enable DistanceMap acceleration for domain boundary
  int dmResolution = 128;
  int dmTolerance = 6;
  boundaryMesh->enableDistanceMapAcceleration(dmResolution, dmTolerance);

  const bool boundaryDistanceMapEnabled = boundaryMesh->hasDistanceMap();
  const DistanceMap* boundaryDM = boundaryDistanceMapEnabled ? boundaryMesh->getDistanceMap() : nullptr;

  if (!boundaryDistanceMapEnabled && isRepresentative) {
    std::cerr << "WARNING: DistanceMap acceleration failed to initialize for boundary" << std::endl;
  }

  if (boundaryDistanceMapEnabled && isRepresentative) {
    // Invert the DistanceMap for domain boundary representation
    // This transforms: inside mesh -> inside fluid domain (positive distance)
    //                  outside mesh -> outside fluid domain (negative distance)
    //                  normals -> point inward to keep particles in domain
    //boundaryMesh->getDistanceMap()->invertForDomainBoundary();

    std::cout << "\n--" << "DOMAIN BOUNDARY SETUP"
              << "--------------------------------------------------------------\n"
              << " Boundary mesh file                      = " << boundaryFileName << "\n"
              << " Distance map enabled                    = yes (INVERTED for domain boundary)\n"
              << " Distance map grid size                  = "
              << boundaryDM->getNx() << " x " << boundaryDM->getNy() << " x " << boundaryDM->getNz() << "\n"
              << " Distance map origin                     = ("
              << boundaryDM->getOrigin()[0] << ", "
              << boundaryDM->getOrigin()[1] << ", "
              << boundaryDM->getOrigin()[2] << ")\n"
              << " Distance map spacing                    = " << boundaryDM->getSpacing() << "\n"
              << " Distance map resolution                 = " << dmResolution << "\n"
              << " Distance map tolerance                  = " << dmTolerance << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
    boundaryMesh->print(std::cout, "  ");

    // Write inverted DistanceMap to VTK for visual inspection
    vtk::DistanceMapWriter::writeVTI("atc_boundary_distancemap.vti", *boundaryDM);
    std::cout << "Inverted DistanceMap written to: atc_boundary_distancemap.vti\n" << std::endl;
  }
#else
  const bool boundaryDistanceMapEnabled = false;
  if (isRepresentative) {
    std::cout << "WARNING: DistanceMap not available (PE_USE_CGAL not defined)" << std::endl;
  }
#endif

  if (isRepresentative) {
    std::cout << "\n--" << "ATC SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }

}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_ROTATION_H_
