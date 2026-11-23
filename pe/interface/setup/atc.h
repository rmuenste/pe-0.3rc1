//=================================================================================================
/*!\file pe/interface/setup/atc.h
 *  \brief ATC application test case: particles seeded along a centerline inside a boundary mesh.
 *
 *  Entry point: commf2c_atc_().
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
#include <pe/interface/support/decompose.h>
#include <pe/interface/support/geometry_utils.h>
#include <pe/interface/support/optional_collision_params.h>
#include <pe/interface/support/serial_step_features.h>
#include <pe/vtk/DistanceMapWriter.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <pe/core/MPI.h>

namespace pe {

//*************************************************************************************************
/*!\brief ATC (Application Test Case) setup for parallel PE mode.
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_atc_(). Spheres inside a closed boundary surface (the
 * liquid segment in a turn of the Archimedes tube crystalliser); the surface is a fixed global
 * triangle mesh whose DistanceMap keeps the spheres inside. World, materials, boundary and
 * sphere numbering follow the serial half (setupATCSerial below), so both modes simulate the
 * same system.
 *
 * Decomposition: the domain is cut into a chain of slabs along the tube by the planes of
 * "single_plane.txt", one slab per process:
 *
 *    processesX_ = number of processes,  processesY_ = 1,  processesZ_ = 1
 *
 * The file has one plane per line ("px py pz nx ny nz") and is the file the pe_partpy
 * strategy plane_single partitioned the CFD mesh with: rank k of \a ex0 gets the region of
 * CFD subdomain k+1, see decomposeDomainPlaneSlabs(). It holds one plane per process, or one
 * less (the last line only closes the last subdomain). A single process needs no file.
 *
 * Input files, read from the working directory:
 *   - example.json                     configuration
 *   - single_plane.txt                 slab planes (more than one process)
 *   - domainBoundary_.file             boundary surface (OBJ, outward normals)
 *   - xyzFilePath_                     sphere positions (packingMethod_ "external")
 *   - sorted_vertices_by_x_world.txt   centerline (every other packing method but "none")
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density and radius
 * (benchRadius_), the process layout, the step size, the packing method, the domainBoundary_
 * block and the VTK switch and spacing. The boundary has to be fixed, and its position has to
 * be the centre of mass of the surface if the file holds world coordinates (a mesh read from a
 * file is moved so that its centre of mass sits at that position).
 *
 * The boundary has user ID 0 and sphere i of the position list has user ID i+1 (i without a
 * boundary), on whichever process creates it; this is the numbering of the serial half.
 *
 * Refused, with a message and MPI_Abort: a process layout that does not match the
 * communicator, a missing or malformed plane file or a wrong number of planes, resume_ (no
 * checkpoint is read here), a boundary that is not fixed, a boundary file that cannot be read,
 * a DistanceMap that is requested but cannot be built (also: a build without CGAL), an empty
 * position list for a packing method that needs one, a start position that no process owns or
 * that two processes own, and a sphere whose start position is not inside the boundary.
 */
inline void setupATC( MPI_Comm ex0 )
{
   //===================================================================================
   // Case constants
   //===================================================================================
   const std::string planesFile( "single_plane.txt" );
   const std::string centerlineFile( "sorted_vertices_by_x_world.txt" );
   const id_t boundaryId( 0 );   // User ID of the global boundary mesh (as in the serial half)

   //===================================================================================
   // Configuration, world and fluid properties (as in the serial half)
   //===================================================================================
   auto& config = SimulationConfig::getInstance();
   world = theWorld();

   loadSimulationConfig( "example.json" );
   rejectRestitutionRequest( *theCollisionSystem(), SimulationConfig::getInstance(), "setupATC" );

   mpisystem = theMPISystem();
   mpisystem->setComm( ex0 );

   const int myRank( mpisystem->getRank() );
   const int nSlabs( mpisystem->getSize() );

   // Prints the message once and aborts the whole run
   const auto abortSetup = [&]( const std::string& message ) {
      if( myRank == 0 ) {
         std::cerr << "\nERROR in setupATC: " << message << "\n" << std::endl;
      }
      MPI_Abort( ex0, 1 );
   };

   // Adaptive Baumgarte capping for contact stabilization (EL solver only)
   CollisionSystemID cs = theCollisionSystem();
   setOptionalAdaptiveBaumgarteCapping( cs.get(), true, real(50.0) );

   const real simViscosity( config.getFluidViscosity() );
   const real simRho( config.getFluidDensity() );
   const real rhoParticle( config.getParticleDensity() );
   const real sphereRad( config.getBenchRadius() );

   world->setGravity( config.getGravity() );
   world->setDamping( 1.0 );
   world->setLiquidSolid( true );
   world->setLiquidDensity( simRho );
   world->setViscosity( simViscosity );

   TimeStep::stepsize( config.getStepsize() );

   //===================================================================================
   // Validation (nothing is created before these checks have passed)
   //===================================================================================
   const int px( config.getProcessesX() );
   const int py( config.getProcessesY() );
   const int pz( config.getProcessesZ() );

   if( py != 1 || pz != 1 ) {
      abortSetup( "processesY_ and processesZ_ must be 1; the ATC domain is decomposed into "
                  "slabs along the tube (processesX_)." );
   }
   if( px != nSlabs ) {
      abortSetup( "invalid number of MPI processes: " + std::to_string( nSlabs ) + " != " +
                  std::to_string( px ) + " (processesX_)." );
   }
   if( sphereRad <= real(0) ) {
      abortSetup( "benchRadius_ must be positive." );
   }
   if( config.getResume() ) {
      abortSetup( "resume_ = true, but the parallel ATC setup reads no checkpoint; run the "
                  "segment from scratch or use the serial-mode build." );
   }

   const bool boundaryEnabled( config.getDomainBoundaryEnabled() );
   const bool boundaryDistanceMap( boundaryEnabled && config.getDomainBoundaryDistanceMapEnabled() );
   if( boundaryEnabled && !config.getDomainBoundaryFixed() ) {
      abortSetup( "domainBoundary_.fixed must be true: the boundary is a global body, and a "
                  "global body cannot move." );
   }
#ifndef PE_USE_CGAL
   if( boundaryDistanceMap ) {
      abortSetup( "domainBoundary_.distanceMap is enabled, but this pe build has no CGAL "
                  "support (PE_USE_CGAL); without the DistanceMap the boundary surface does "
                  "not contain the spheres." );
   }
#endif

   //===================================================================================
   // Domain decomposition: slabs along the tube, cut by the planes of the CFD partition
   //===================================================================================
   std::vector<HalfSpace> planes;
   if( nSlabs > 1 ) {
      std::string planeError;
      if( !loadSlabPlanes( planesFile, planes, planeError ) ) {
         abortSetup( planeError );
      }
      const int nPlanes( static_cast<int>( planes.size() ) );
      if( nPlanes != nSlabs-1 && nPlanes != nSlabs ) {
         abortSetup( planesFile + " holds " + std::to_string( nPlanes ) + " planes, but " +
                     std::to_string( nSlabs ) + " processes need " + std::to_string( nSlabs-1 ) +
                     " or " + std::to_string( nSlabs ) + " (one per CFD subdomain; the last one "
                     "only closes the last subdomain)." );
      }
   }

   decomposeDomainPlaneSlabs( myRank, planes, nSlabs );

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================
   // Materials (registered in the order of the serial half, see there)
   //===================================================================================
   MaterialID boundaryMat = createMaterial( "boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   MaterialID particleMaterial = createMaterial( "particleMaterial", rhoParticle, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   //===================================================================================
   // Domain boundary: fixed global triangle mesh with an inverted DistanceMap
   //===================================================================================
   const std::string boundaryFileName( config.getDomainBoundaryFilePath().string() );
   const Vec3 boundaryPos( config.getDomainBoundaryPosition() );

   TriangleMeshID boundaryMesh( nullptr );
   if( boundaryEnabled ) {
      std::string boundaryError;
      pe_GLOBAL_SECTION
      {
         try {
            boundaryMesh = createTriangleMesh( boundaryId, boundaryPos, boundaryFileName, boundaryMat,
                                               false, config.getDomainBoundaryVisible() );
            boundaryMesh->setFixed( true );
            boundaryMesh->setPosition( boundaryPos );

#ifdef PE_USE_CGAL
            if( boundaryDistanceMap ) {
               boundaryMesh->enableDistanceMapAcceleration(
                  config.getDomainBoundaryDistanceMapResolution(),
                  config.getDomainBoundaryDistanceMapTolerance() );
               if( !boundaryMesh->hasDistanceMap() ) {
                  boundaryError = "the DistanceMap of the boundary '" + boundaryFileName +
                                  "' could not be built.";
               }
               else if( config.getDomainBoundaryInvertDistanceMap() ) {
                  // inside the surface = inside the fluid domain (positive distance), the
                  // normals point into the domain
                  boundaryMesh->getDistanceMap()->invertForDomainBoundary();
               }
            }
#endif
         }
         catch( const std::exception& e ) {
            boundaryError = "the boundary '" + boundaryFileName + "' could not be created: " + e.what();
         }
      }
      if( !boundaryError.empty() ) {
         abortSetup( boundaryError );
      }

#ifdef PE_USE_CGAL
      if( myRank == 0 && boundaryMesh->hasDistanceMap() && config.getDomainBoundaryWriteDistanceMapVti() ) {
         vtk::DistanceMapWriter::writeVTI( config.getDomainBoundaryDistanceMapVtiFile().string(),
                                           *boundaryMesh->getDistanceMap() );
      }
#endif
   }

   //===================================================================================
   // Spheres: every process creates the start positions its slab owns
   //===================================================================================
   const SimulationConfig::PackingMethod packingMethod( config.getPackingMethod() );

   std::vector<Vec3> edges;   // non-const: the generator takes a mutable reference
   if( packingMethod != SimulationConfig::PackingMethod::External &&
       packingMethod != SimulationConfig::PackingMethod::None ) {
      edges = readVectorsFromFile( centerlineFile );
      if( edges.empty() ) {
         abortSetup( "read no centerline vertices from " + centerlineFile + "." );
      }
      // The centerline is kept for the stuck-particle diagnostics, as in the serial half
      config.setCenterlineVertices( edges );
      config.setTotalCenterlineLength( calculateTotalCenterlineLength( edges ) );
   }

   std::vector<Vec3> spherePositions;
   std::string packing( "none" );
   if( packingMethod == SimulationConfig::PackingMethod::External ) {
      const std::string xyzPath( config.getXyzFilePath().string() );
      spherePositions = readVectorsFromFile( xyzPath );
      packing = "external (" + xyzPath + ")";
      if( spherePositions.empty() ) {
         abortSetup( "could not read any particle positions from '" + xyzPath + "'." );
      }
   }
   else if( packingMethod != SimulationConfig::PackingMethod::None ) {
      // Rings along the centerline, with the parameters of the serial half
      spherePositions = generatePointsAlongCenterline( edges, sphereRad, -1.0, 4, 20, 4 );
      packing = "centerline rings (" + centerlineFile + ")";
   }

   const id_t firstSphereId( boundaryEnabled ? 1 : 0 );
   unsigned long createdLocal( 0 );
   unsigned long outsideLocal( 0 );   // start positions that are not inside the boundary
   for( std::size_t i = 0; i < spherePositions.size(); ++i ) {
      const Vec3& position( spherePositions[i] );
      if( !world->ownsPoint( position ) )
         continue;

#ifdef PE_USE_CGAL
      if( boundaryMesh != nullptr && boundaryMesh->hasDistanceMap() &&
          config.getDomainBoundaryInvertDistanceMap() ) {
         const Vec3 local( boundaryMesh->pointFromWFtoBF( position ) );
         const real distance( boundaryMesh->getDistanceMap()->interpolateDistance( local[0], local[1], local[2] ) );
         if( !( distance > real(0) ) || distance > real(1e5) ) {
            ++outsideLocal;
            continue;
         }
      }
#endif

      createSphere( firstSphereId + static_cast<id_t>( i ), position, sphereRad, particleMaterial );
      ++createdLocal;
   }

   // Synchronization of the MPI processes
   world->synchronize();

   // Setup of the VTK visualization
   if( config.getVtk() ) {
      vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true );
   }

   //===================================================================================
   // Setup summary and consistency check
   //===================================================================================
   std::vector<unsigned long> createdPerSlab( static_cast<std::size_t>( nSlabs ), 0 );
   MPI_Gather( &createdLocal, 1, MPI_UNSIGNED_LONG, &createdPerSlab[0], 1, MPI_UNSIGNED_LONG, 0, ex0 );
   unsigned long outsideTotal( 0 );
   MPI_Reduce( &outsideLocal, &outsideTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, ex0 );

   pe_EXCLUSIVE_SECTION( 0 )
   {
      unsigned long particlesTotal( 0 );
      std::ostringstream perSlab;
      for( int k = 0; k < nSlabs; ++k ) {
         particlesTotal += createdPerSlab[k];
         perSlab << ( k == 0 ? "" : " " ) << createdPerSlab[k];
      }

      const real partVol( real(4) / real(3) * M_PI * sphereRad * sphereRad * sphereRad );

      std::cout << "\n--" << "ATC SETUP (parallel PE mode)"
                << "----------------------------------------------------\n"
                << " Total number of MPI processes           = " << nSlabs << " (slabs along the tube)\n"
                << " Slab planes                             = "
                << ( nSlabs > 1 ? planesFile + ", " + std::to_string( planes.size() ) + " planes"
                                : std::string( "none (single process)" ) ) << "\n"
                << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
                << " Fluid viscosity                         = " << simViscosity << "\n"
                << " Fluid density                           = " << simRho << "\n"
                << " Gravity                                 = " << world->getGravity() << "\n"
                << " Packing                                 = " << packing << "\n"
                << " Total number of particles               = " << particlesTotal << "\n"
                << " Particles per slab                      = " << perSlab.str() << "\n"
                << " Particle radius                         = " << sphereRad << "\n"
                << " Particle density                        = " << rhoParticle << "\n";
      if( boundaryMesh != nullptr ) {
         const real domainVol( boundaryMesh->getVolume() );
         std::cout << " Boundary mesh file                      = " << boundaryFileName << "\n"
                   << " Boundary mesh volume                    = " << domainVol << "\n"
                   << " Volume fraction[%]                      = "
                   << ( particlesTotal * partVol ) / domainVol * 100.0 << "\n";
#ifdef PE_USE_CGAL
         if( boundaryMesh->hasDistanceMap() ) {
            const DistanceMap* dm( boundaryMesh->getDistanceMap() );
            std::cout << " Distance map grid size                  = "
                      << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << "\n"
                      << " Distance map spacing                    = " << dm->getSpacing()
                      << " (" << dm->getSpacing() / sphereRad << " particle radii)\n"
                      << " Distance map inverted for the boundary  = "
                      << ( config.getDomainBoundaryInvertDistanceMap() ? "yes" : "no" ) << "\n";
         }
         else {
            std::cout << " Distance map                            = disabled\n";
         }
#endif
      }
      else {
         std::cout << " Domain boundary                         = disabled\n";
      }
      std::cout << "--------------------------------------------------------------------------------\n"
                << std::endl;

      if( outsideTotal != 0 ) {
         std::cerr << "\nERROR in setupATC: " << outsideTotal << " of " << spherePositions.size()
                   << " start positions are not inside the boundary '" << boundaryFileName
                   << "'.\n" << std::endl;
         MPI_Abort( ex0, 1 );
      }
      if( particlesTotal != spherePositions.size() ) {
         std::cerr << "\nERROR in setupATC: the position list holds " << spherePositions.size()
                   << " spheres, but " << particlesTotal << " were created (every start "
                   << "position must be owned by exactly one slab).\n" << std::endl;
         MPI_Abort( ex0, 1 );
      }
   }

   MPI_Barrier( ex0 );
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
  rejectRestitutionRequest(*theCollisionSystem(), SimulationConfig::getInstance(), "setupATCSerial");
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
