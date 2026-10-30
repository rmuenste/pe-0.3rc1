//=================================================================================================
/*!\file pe/interface/setup/atc.h
 *  \brief ATC application test case: particles seeded along a centerline inside a boundary mesh.
 *
 *  Entry point: commf2c_atc_(). The parallel setupATC() aborts (not implemented).
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_ATC_H_
#define _PE_INTERFACE_SETUP_ATC_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <iostream>
#include <pe/core/MPI.h>

namespace pe {

//*************************************************************************************************
/*!\brief ATC (Application Test Case) setup for parallel PE mode -- NOT FUNCTIONAL YET.
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_atc_(). The parallel ATC setup is planned but not
 * implemented. Instead of returning silently with an empty world (which would let the CFD
 * run continue without any particles), this placeholder aborts the run with a clear message.
 *
 * The serial variant (setupATCSerial below, PE_SERIAL_MODE) is functional.
 *
 * When implementing, the usual steps are: load the configuration, set the world and fluid
 * properties, wire the communicator and decompose the domain, create the bodies,
 * synchronize, and activate the output writers.
 */
inline void setupATC(MPI_Comm ex0) {

  int rank = 0;
  if (ex0 != MPI_COMM_NULL) {
    MPI_Comm_rank(ex0, &rank);
  }

  if (rank == 0) {
    std::cerr << "\n"
              << "========================================================================\n"
              << "ERROR: setupATC() is not functional at the moment\n"
              << "========================================================================\n"
              << "The ATC setup for parallel PE mode is not implemented yet.\n"
              << "Use the PE serial mode (PE_SERIAL_MODE) for the ATC case for now.\n"
              << "========================================================================\n"
              << std::endl;
  }

  MPI_Abort(ex0 != MPI_COMM_NULL ? ex0 : MPI_COMM_WORLD, 1);
}
//*************************************************************************************************

}  // namespace pe
#endif  // HAVE_MPI

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief ATC (Application Test Case) setup for serial mode
 *
 * Generic setup for ATC application. This is a template setup that can be
 * customized for specific simulation needs.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupATCSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

  // Load configuration from JSON file
  SimulationConfig::loadFromFile("example.json");
  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);
  const bool resume = config.getResume();

  WorldID world = theWorld();

  // Apply lubrication/contact parameters from configuration
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  // Enable adaptive Baumgarte capping for contact stabilization (EL solver only)
  setOptionalAdaptiveBaumgarteCapping(cs.get(), true, real(50.0));

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

  // TODO: Add additional rigid bodies here
  // Example:
  // MaterialID myMaterial = createMaterial("ATCMaterial", density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT);
  // SphereID sphere = createSphere(idx++, position, radius, myMaterial, global);
  int idx = 0;

  //==============================================================================================
  // Material registration
  //==============================================================================================
  // Registered before the resume below, and in the same order on every path. Checkpointed bodies
  // carry a bare index into the process material table (Marshalling.h), so a table that is
  // shorter or differently ordered at read time resolves those indices against the wrong
  // material -- or past the end of the table. The checkpoint's own table is reinstated by
  // readCheckpoint() as well, but only for checkpoints that carry a metadata sidecar; keeping
  // this order correct is what makes legacy checkpoints resumable too.
  real rhoParticle( config.getParticleDensity() );
  MaterialID boundaryMat = createMaterial("boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID particleMaterial = createMaterial("particleMaterial", rhoParticle, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);

  int restoredBoundaryMeshes = 0;
  if (resume) {
    // Resume must load on every rank in PE serial mode, because each CFD rank runs
    // its own PE instance and later queries local particle storage.  TriangleMesh
    // distance-map checkpointing is not reliable yet, so discard restored meshes
    // and recreate the domain boundary exactly like the fresh-start path below.
    resumeFromConfiguredCheckpoint(config);

    WorldID restoredWorld = theWorld();
    for (auto it = restoredWorld->begin(); it != restoredWorld->end();) {
      if ((*it)->getType() == triangleMeshType) {
        it = restoredWorld->destroy(it);
        ++restoredBoundaryMeshes;
      } else {
        ++it;
      }
    }
  }

  //==============================================================================================
  // Domain Boundary Configuration
  //==============================================================================================
  const bool domainBoundaryEnabled = config.getDomainBoundaryEnabled();
  std::string boundaryFileName = config.getDomainBoundaryFilePath().string();
  Vec3 boundaryPos = config.getDomainBoundaryPosition();

  TriangleMeshID boundaryMesh(nullptr);
  if (domainBoundaryEnabled) {
    boundaryMesh = createTriangleMesh(idx++, boundaryPos, boundaryFileName, boundaryMat, false,
                                      config.getDomainBoundaryVisible());
    boundaryMesh->setFixed(config.getDomainBoundaryFixed());
    boundaryMesh->setPosition(boundaryPos);

#ifdef PE_USE_CGAL
    if (config.getDomainBoundaryDistanceMapEnabled()) {
      // Enable DistanceMap acceleration for domain boundary
      int dmResolution = config.getDomainBoundaryDistanceMapResolution();
      int dmTolerance = config.getDomainBoundaryDistanceMapTolerance();
      boundaryMesh->enableDistanceMapAcceleration(dmResolution, dmTolerance);

      const bool boundaryDistanceMapEnabled = boundaryMesh->hasDistanceMap();
      const DistanceMap* boundaryDM = boundaryDistanceMapEnabled ? boundaryMesh->getDistanceMap() : nullptr;

      if (!boundaryDistanceMapEnabled && isRepresentative) {
        std::cerr << "WARNING: DistanceMap acceleration failed to initialize for boundary" << std::endl;
      }

      if (boundaryDistanceMapEnabled && config.getDomainBoundaryInvertDistanceMap()) {
        boundaryMesh->getDistanceMap()->invertForDomainBoundary();
      }
      if (boundaryDistanceMapEnabled && isRepresentative) {
        // Invert the DistanceMap for domain boundary representation
        // This transforms: inside mesh -> inside fluid domain (positive distance)
        //                  outside mesh -> outside fluid domain (negative distance)
        //                  normals -> point inward to keep particles in domain
        std::cout << "\n--" << "DOMAIN BOUNDARY SETUP"
                  << "--------------------------------------------------------------\n"
                  << " Boundary mesh file                      = " << boundaryFileName << "\n"
                  << " Boundary mesh volume                    = " << boundaryMesh->getVolume() << "\n"
                  << " Distance map enabled                    = yes"
                  << (config.getDomainBoundaryInvertDistanceMap() ? " (INVERTED for domain boundary)" : "")
                  << "\n"
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

        if (config.getDomainBoundaryWriteDistanceMapVti()) {
          vtk::DistanceMapWriter::writeVTI(config.getDomainBoundaryDistanceMapVtiFile().string(),
                                           *boundaryDM);
          std::cout << "DistanceMap written to: "
                    << config.getDomainBoundaryDistanceMapVtiFile() << "\n" << std::endl;
        }

        // Sanity check: Test a point that should be outside the domain
        Vec3 testPointWorld(-0.4, -2.0, 0.19);  // Outside domain, within grid bounds
        Vec3 testPointLocal = boundaryMesh->pointFromWFtoBF(testPointWorld);
        real testDistance = boundaryDM->interpolateDistance(testPointLocal[0], testPointLocal[1], testPointLocal[2]);

        std::cout << "\n--" << "ESCAPE DETECTION SANITY CHECK"
                  << "--------------------------------------------------------------\n"
                  << " Test point (world): (" << testPointWorld[0] << ", " << testPointWorld[1] << ", " << testPointWorld[2] << ")\n"
                  << " Test point (local): (" << testPointLocal[0] << ", " << testPointLocal[1] << ", " << testPointLocal[2] << ")\n"
                  << " Signed distance: " << testDistance << "\n"
                  << " Expected: negative (outside domain)\n"
                  << " Result: " << (testDistance < 0.0 ? "PASS - point correctly identified as OUTSIDE" :
                                    (testDistance > 1e5 ? "OUT OF GRID BOUNDS" : "FAIL - point incorrectly identified as INSIDE")) << "\n"
                  << "--------------------------------------------------------------------------------\n" << std::endl;
      }
    }
#else
    if (config.getDomainBoundaryDistanceMapEnabled() && isRepresentative) {
      std::cout << "WARNING: DistanceMap not available (PE_USE_CGAL not defined)" << std::endl;
    }
#endif
  } else if (isRepresentative) {
    std::cout << "Domain boundary disabled by configuration" << std::endl;
  }
  //==============================================================================================
  // Particle Generation along Centerline
  //==============================================================================================
  std::vector<Vec3> edges = readVectorsFromFile("sorted_vertices_by_x_world.txt");

  // Store centerline in SimulationConfig singleton for stuck particle diagnostics
  config.setCenterlineVertices(edges);
  config.setTotalCenterlineLength(calculateTotalCenterlineLength(edges));

  if (isRepresentative) {
    pe_LOG_INFO_SECTION( log ) {
      log << "Centerline loaded: " << config.getCenterlineVertices().size() << " vertices, "
          << "total length = " << config.getTotalCenterlineLength() << "\n";
    }
  }

  real sphereRad =  config.getBenchRadius(); //  0.0183 Sphere radius for centerline particles

  // Generate sphere positions along the centerline.
  // num_rings=5 (one extra ring vs. before), margin_steps=20 (up from 4) to keep
  // a larger empty region at inlet/outlet while preserving ~6992 total particles:
  //   ~60 cross-sections × 118 particles/section ≈ 7080  (within ±100 of previous count)
  std::vector<Vec3> generatedSpherePositions =
      generatePointsAlongCenterline(edges, sphereRad, -1.0, 4, 20, 4);

  int particlesCreated = 0;
  const auto packingMethod = config.getPackingMethod();
  if (resume) {
    // Checkpoint resume owns particle initialization completely.  Do not create
    // any additional particles from the configured packing method.
    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      if ((*it)->getType() == sphereType) {
        ++particlesCreated;
      }
    }
  } else {
    if (packingMethod == SimulationConfig::PackingMethod::External) {
      std::vector<Vec3> spherePositions = readVectorsFromFile(config.getXyzFilePath().string());
      if (isRepresentative) {
        std::cout << " Loading external particle positions from: "
                  << config.getXyzFilePath() << "\n"
                  << " Positions read: " << spherePositions.size() << "\n";
      }
      for (auto spherePos: spherePositions) {
        createSphere(idx++, spherePos, sphereRad, particleMaterial);
        particlesCreated++;
      }
    } else if (packingMethod != SimulationConfig::PackingMethod::None) {
      // Create spheres at generated positions (serial mode: no domain ownership check needed)
      for (auto spherePos: generatedSpherePositions) {
        createSphere(idx++, spherePos, sphereRad, particleMaterial);
        particlesCreated++;
      }
    }
  }

  // Volume fraction computation
  real domainVol = 0.78;
  real partVol = 4. / 3. * M_PI * std::pow(sphereRad, 3);

  if (isRepresentative) {
    std::cout << "\n--" << "ATC SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Mode                                    = "
              << (resume ? std::string("resume from ") + config.getResumeCheckpointFile()
                         : std::string("fresh start")) << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Restored boundary meshes discarded      = " << restoredBoundaryMeshes << "\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << " Particle radius                         = " << sphereRad << "\n"
              << " Particle volume                         = " << partVol << "\n"
              << " Domain volume                           = " << domainVol << "\n"
              << " Volume fraction[%]                      = " << (particlesCreated * partVol) / domainVol * 100.0 << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }

}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_ATC_H_
