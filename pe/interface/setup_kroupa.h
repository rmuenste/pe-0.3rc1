#ifndef _PE_SETUP_KROUPA_H_
#define _PE_SETUP_KROUPA_H_

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/interface/setup_optional_collision_params.h>
// Provides the seeded lattice generator elTerminalRandomSeeds() that is reused here.
// TODO: move the seeding helpers into a shared header so this include is not needed.
#include <pe/interface/setup_el_terminal_velocity.h>

using namespace pe::povray;

//*************************************************************************************************
/*!\brief PE setup for the Kroupa shear-cell case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_kroupa_(). A gravity-free suspension of equal spheres
 * in a cubic box that is periodic in x and y and bounded by two planes in z.
 *
 * Particles, depending on packingMethod_:
 *   - Grid: randomly chosen sites of a regular lattice until volumeFraction_ is reached. The
 *     choice is seeded with seed_, so a run is reproducible. The sphere radius is
 *     benchRadius_ reduced by a small safety gap.
 *   - External: positions from the xyz file, radius benchRadius_.
 * With resume_ the bodies come from the checkpoint file instead.
 *
 * Read from example.json: fluid density and viscosity, particle density and radius, the
 * process layout (processesX_/Y_/Z_), step size, packing method, volume fraction, seed, the
 * checkpoint settings, the lubrication settings, and the VTK switch and spacing.
 *
 * Lubrication is controlled by the json file only (lubricationEnabled_ and the related
 * parameters, applied through applyOptionalLubricationParams).
 *
 * Fixed in this file: the box size, zero gravity, the lattice safety gap and the solver
 * tolerances.
 *
 * Sphere user IDs are numbered per process and are therefore not unique across processes.
 * Use the system ID (getSystemID()) wherever a globally unique ID is needed.
 *
 * Every error path aborts the run with a message.
 */
void setupKroupa(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real LX( 0.1 );                      // Box size, origin at (0,0,0)
  const real LY( 0.1 );
  const real LZ( 0.1 );
  const real epsilon( 2e-4 );                // Lattice safety gap between sphere surfaces

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();
  world = theWorld();

  loadSimulationConfig("example.json");

  // Push runtime lubrication parameters (model switches, cutoff, hysteresis) into the engine
  applyOptionalLubricationParams(*theCollisionSystem(), config);

  const real simViscosity( config.getFluidViscosity() );
  const real simRho( config.getFluidDensity() );
  const real pRho( config.getParticleDensity() );

  world->setGravity( 0.0, 0.0, 0.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  TimeStep::stepsize( config.getStepsize() );

  //===============================================================================================
  // MPI system and validation (nothing is created before all checks have passed)
  //===============================================================================================
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupKroupa: " << message << "\n" << std::endl;
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
  if( px < 3 || py < 3 ) {
    abortSetup("the box is periodic in x and y, which requires processesX_ >= 3 and "
               "processesY_ >= 3 (distinct wrap neighbors), got " + std::to_string(px) +
               " and " + std::to_string(py) + ".");
  }

  const bool resume = config.getResume();
  const bool gridPacking     = (config.getPackingMethod() == SimulationConfig::PackingMethod::Grid);
  const bool externalPacking = (config.getPackingMethod() == SimulationConfig::PackingMethod::External);

  if( !resume && !gridPacking && !externalPacking ) {
    abortSetup("unsupported packingMethod_ (supported: Grid, External).");
  }
  if( resume && !config.getUseCheckpointer() ) {
    abortSetup("resume_ is set but the checkpointer is not enabled (useCheckpointer_).");
  }
  if( config.getBenchRadius() <= real(0) ) {
    abortSetup("benchRadius_ must be positive.");
  }

  // The grid packing keeps a safety gap between neighboring spheres
  const real sphereRadius = gridPacking ? config.getBenchRadius() - epsilon
                                        : config.getBenchRadius();
  if( sphereRadius <= real(0) ) {
    abortSetup("benchRadius_ is not larger than the lattice safety gap.");
  }

  //===============================================================================================
  // xy-periodic 3D rectilinear domain decomposition
  //===============================================================================================
  int dims   [] = { px, py, pz };
  int periods[] = { true, true, false };
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

  const real dx( LX/px );
  const real dy( LY/py );
  const real dz( LZ/pz );

  decomposePeriodicXY3D(center, 0.0, 0.0, 0.0,
                        dx, dy, dz,
                        LX, LY, LZ,
                        px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Materials, checkpointer and solver parameters
  //===============================================================================================
  // TODO: "ground" and "Bench" are not assigned to any body. They are kept on purpose: this
  //       case uses checkpoints, and removing them shifts the material index of
  //       "particleMaterial". Remove them once index-based use is ruled out.
  MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID myMaterial = createMaterial("Bench", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID particleMaterial = createMaterial( "particleMaterial", pRho, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  (void)gr;
  (void)myMaterial;

  CheckpointerID checkpointer;
  if (config.getUseCheckpointer()) {
    checkpointer = activateCheckpointer(config.getCheckpointPath(),
                                         config.getPointerspacing(),
                                         0, config.getTimesteps());
  }

  theCollisionSystem()->setMinEps(0.01);
  theCollisionSystem()->setMaxIterations(200);

  //===============================================================================================
  // Bodies: spheres from the packing (or the checkpoint) and the two z-walls
  //===============================================================================================
  unsigned long positionsTotal( 0 );

  if( resume ) {
    checkpointer->read( config.getResumeCheckpointFile() );
  }
  else {
    // The positions are determined on the root process and broadcast, so all processes
    // work on the same list.
    std::vector<Vec3> allPositions;
    if( myRank == 0 ) {
      if( externalPacking ) {
        allPositions = readVectorsFromFile(config.getXyzFilePath().string());
        if( allPositions.empty() ) {
          abortSetup("external packing read no positions from " +
                     config.getXyzFilePath().string() + ".");
        }
      }
      else {
        try {
          allPositions = elTerminalRandomSeeds(0.0, LX, 0.0, LY, 0.0, LZ,
                                               sphereRadius, epsilon,
                                               config.getVolumeFraction(),
                                               config.getSeed(),
                                               "box", Vec3(0.0, 0.0, 0.0), real(0), "z");
        } catch (const std::exception& ex) {
          abortSetup(std::string("grid packing failed: ") + ex.what() + ".");
        }
      }
    }

    positionsTotal = static_cast<unsigned long>(allPositions.size());
    MPI_Bcast(&positionsTotal, 1, MPI_UNSIGNED_LONG, 0, cartcomm);

    std::vector<double> flatPositions(3 * static_cast<std::size_t>(positionsTotal));
    if( myRank == 0 ) {
      for(std::size_t i(0); i < allPositions.size(); i++) {
        flatPositions[3 * i]     = allPositions[i][0];
        flatPositions[3 * i + 1] = allPositions[i][1];
        flatPositions[3 * i + 2] = allPositions[i][2];
      }
    }
    if( !flatPositions.empty() ) {
      MPI_Bcast(flatPositions.data(), static_cast<int>(flatPositions.size()), MPI_DOUBLE, 0, cartcomm);
    }

    // User IDs are numbered per process (see the function documentation)
    int idx = 0;
    for(std::size_t i(0); i < static_cast<std::size_t>(positionsTotal); i++) {
      const Vec3 position(flatPositions[3 * i], flatPositions[3 * i + 1], flatPositions[3 * i + 2]);
      if( world->ownsPoint(position) ) {
        createSphere(idx, position, sphereRadius, particleMaterial, true);
        ++idx;
      }
    }
  }

  pe_GLOBAL_SECTION
  {
     createPlane( 99999, 0.0, 0.0, 1.0, 0.0, particleMaterial, false ); // bottom border
     createPlane( 88888, 0.0, 0.0,-1.0, -LZ, particleMaterial, false ); // top border
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
  unsigned long spheresLocal( 0 );
  unsigned long bodiesLocal( 0 );
  for (unsigned int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++) {
    BodyID body = world->getBody(j);
    if (body->getType() == sphereType) {
      ++spheresLocal;
    }
    ++bodiesLocal;
  }

  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  MPI_Reduce( &spheresLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesLocal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  const real domainVol = LX * LY * LZ;
  const real partVol = 4./3. * M_PI * std::pow(sphereRadius, 3);
  const std::string packing = resume ? "checkpoint" : (externalPacking ? "External" : "Grid");

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "KROUPA SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Particle radius                         = " << sphereRadius << "\n"
      << " Particle volume                         = " << partVol << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Particle Density                        = " << pRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Lubrication (json)                      = " << (config.getLubricationEnabled() ? "enabled" : "disabled") << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain volume                           = " << domainVol << "\n"
      << " Resume                                  = " << (resume ? "resuming" : "not resuming") << "\n"
      << " Packing Method                          = " << packing << "\n"
      << " Volume fraction[%]                      = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
      << " Target VF[%]                            = " << config.getVolumeFraction() * 100.0 << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;

    if( !resume && particlesTotal != positionsTotal ) {
      std::cerr << "WARNING in setupKroupa: " << positionsTotal << " positions were requested but "
                << particlesTotal << " spheres were created; positions outside the box are dropped.\n";
    }
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

#endif
