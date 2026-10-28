//=================================================================================================
/*!\file pe/interface/setup/fluidization.h
 *  \brief Fluidization column (Pan et al. 2002), hard-contact and short-range-repulsion variants.
 *
 *  Entry points: commf2c_fluidization_() and, serial mode only, commf2c_fluidization_srr_().
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_FLUIDIZATION_H_
#define _PE_INTERFACE_SETUP_FLUIDIZATION_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/support/decompose.h>

namespace pe {

//*************************************************************************************************
/*!\brief PE setup for the fluidization column case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_fluidization_(). The case is a thin, fixed-size column
 * with a ground plane at z = 0. A fixed number of equal spheres is seeded on a regular lattice
 * that is filled x-first, then y, then z, starting above the ground plane.
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density, the process
 * layout (processesX_/Y_/Z_), the step size, the VTK switch and spacing, and the lattice
 * spacing factor (gap between sphere surfaces in units of the particle radius).
 *
 * Fixed in this file: the column bounds, the particle count and the particle diameter.
 *
 * Every error path aborts the run with a message. A setup that returned early would let the
 * CFD solver continue with an incomplete particle world.
 */
inline void setupFluidization(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real xMin =  0.0;
  const real xMax = 20.3;
  const real yMin =  0.0;
  const real yMax =  0.686;
  const real zMin =  0.0;
  const real zMax = 70.2;

  const int  targetParticles = 1204;
  const real radParticle     = real(0.5) * real(0.635);  // Diameter 0.635 cm

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();

  world = theWorld();

  loadSimulationConfig("example.json");

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  TimeStep::stepsize(config.getStepsize());

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
      std::cerr << "\nERROR in setupFluidization: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  if (px * py * pz != mpisystem->getSize()) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px * py * pz) + " (processesX_*Y_*Z_).");
  }

  // Seeding lattice: gap between sphere surfaces = spacingFactor * radius
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  const real spacing = spacingFactor * radParticle;
  const real pitch   = real(2.0) * radParticle + spacing;
  const real zStart  = std::max(real(4.0) * radParticle, zMin + radParticle);

  const real xSpan = (xMax - xMin) - real(2.0) * radParticle;
  const real ySpan = (yMax - yMin) - real(2.0) * radParticle;
  const real zSpan = (zMax - zStart) - radParticle;

  const int nxMax = static_cast<int>(std::floor(xSpan / pitch)) + 1;
  const int nyMax = static_cast<int>(std::floor(ySpan / pitch)) + 1;
  const int nzMax = static_cast<int>(std::floor(zSpan / pitch)) + 1;

  if (nxMax <= 0 || nyMax <= 0 || nzMax <= 0) {
    abortSetup("fluidization column too small for the requested particle diameter/spacing.");
  }

  const int maxCapacity = nxMax * nyMax * nzMax;
  if (maxCapacity < targetParticles) {
    abortSetup("fluidization grid capacity (" + std::to_string(maxCapacity) +
               ") is smaller than the requested particles (" +
               std::to_string(targetParticles) + ").");
  }

  //===============================================================================================
  // 3D rectilinear, non-periodic domain decomposition of the column
  //===============================================================================================
  int dims   [] = { px, py, pz };
  int periods[] = { false, false, false };
  int reorder   = false;
  MPI_Comm cartcomm;

  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if (cartcomm == MPI_COMM_NULL) {
    abortSetup("failed to create the cartesian communicator.");
  }
  mpisystem->setComm(cartcomm);

  // Cartesian coordinates of this process within the process grid
  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << Vec3(dims[0], dims[1], dims[2]) << std::endl;
    std::cout << "Rank:" << myRank << "->" << Vec3(center[0], center[1], center[2]) << std::endl;
  }

  const real dx = (xMax - xMin) / px;
  const real dy = (yMax - yMin) / py;
  const real dz = (zMax - zMin) / pz;

  decomposeDomain(center, xMin, yMin, zMin, dx, dy, dz, px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Bodies: ground plane and the particle lattice
  //===============================================================================================
  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_GLOBAL_SECTION
  {
     g_ground = createPlane(777, 0.0, 0.0, 1.0, 0.0, gr, true);
  }

  MaterialID myMaterial = createMaterial("FluidizationParticles", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  theCollisionSystem()->setMinEps(5e-6 / radParticle);

  // Every process walks the same global lattice and creates the spheres it owns, so the
  // user IDs (globalIndex) are consistent across processes.
  unsigned long localParticles = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;

  int globalIndex = 0;
  for (int iz = 0; iz < nzMax && globalIndex < targetParticles; ++iz) {
    const real z = zStart + static_cast<real>(iz) * pitch;
    for (int iy = 0; iy < nyMax && globalIndex < targetParticles; ++iy) {
      const real y = (yMin + radParticle) + static_cast<real>(iy) * pitch;
      for (int ix = 0; ix < nxMax && globalIndex < targetParticles; ++ix) {
        const real x = (xMin + radParticle) + static_cast<real>(ix) * pitch;
        const Vec3 position(x, y, z);
        if (world->ownsPoint(position)) {
          createSphere(globalIndex, position, radParticle, myMaterial, true);
          ++localParticles;
        }
        ++globalIndex;
        usedNx = std::max(usedNx, ix + 1);
        usedNy = std::max(usedNy, iy + 1);
        usedNz = std::max(usedNz, iz + 1);
      }
    }
  }

  // Synchronization of the MPI processes
  world->synchronize();

  // Setup of the VTK visualization
  if (config.getVtk()) {
    vtk::activateWriter("./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
  }

  //===============================================================================================
  // Setup summary
  //===============================================================================================
  unsigned long particlesTotal(0);
  unsigned long primitivesTotal(0);
  unsigned long localBodies = static_cast<unsigned long>(theCollisionSystem()->getBodyStorage().size());
  MPI_Reduce(&localParticles, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localBodies, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

  const real sphereVol = (real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle;
  const real localVol = static_cast<real>(localParticles) * sphereVol;
  const real localMass = localVol * rhoParticle;
  real totalMass(0.0);
  real totalVol(0.0);
  MPI_Reduce(&localMass, &totalMass, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localVol, &totalVol, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "FLUIDIZATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Column bounds [x,y,z]                   = "
      << "[" << xMin << "," << xMax << "] x "
      << "[" << yMin << "," << yMax << "] x "
      << "[" << zMin << "," << zMax << "]\n"
      << " Particle radius                         = " << radParticle << "\n"
      << " Particle diameter                       = " << (real(2.0) * radParticle) << "\n"
      << " Spacing factor                          = " << spacingFactor << "\n"
      << " Gap (between surfaces)                  = " << spacing << "\n"
      << " Center-to-center pitch                  = " << pitch << "\n"
      << " Grid start z                            = " << zStart << "\n"
      << " Grid extents used (nx,ny,nz)            = "
      << usedNx << ", " << usedNy << ", " << usedNz << "\n"
      << " Particle mass                           = " << totalMass << "\n"
      << " Particle volume                         = " << totalVol << "\n"
      << std::endl;
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
 * @brief Fluidization setup for serial mode
 *
 * Initialized as a copy of setupParticleBenchSerial so fluidization can diverge
 * from the sedimentation benchmark setup in a dedicated entry point.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupFluidizationSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);
  lubrication::setLubricationThreshold(real(0));

  world->setGravity(config.getGravity());

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());

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
                         config.getPointerspacing(),
                         0, config.getTimesteps());
  }

  const real xMin = 0.0;
  const real xMax = 20.3;
  const real yMin = 0.0;
  const real yMax = 0.686;
  //const real zMin = 2.0;
  const real zMax = 70.2;

  const int baseTargetParticles = 27;
  //const int targetParticles = static_cast<int>(real(1.000) * static_cast<real>(baseTargetParticles));
  const int targetParticles = 1.0; //static_cast<int>(real(1.000) * static_cast<real>(baseTargetParticles));
  const real radParticle = real(0.5) * real(0.635);  // Diameter 0.635 cm
  const real zMin = radParticle;
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  const real spacing = spacingFactor * radParticle;
  const real pitch = real(2.0) * radParticle + spacing;
  const real zStart = std::max(0.0, zMin + 0.125 * radParticle);

  const real xMid = (xMax - xMin) * 0.5;
  const real yMid = (yMax - yMin) * 0.5;

  // Keep one particle radius clearance to the x-boundaries on both sides.
  const real xGridMin = xMin +  1.5 * radParticle;
  const real xGridMax = xMax - radParticle;
  const real xSpan = xGridMax - xGridMin;
  const real ySpan = (yMax - yMin) - real(2.0) * radParticle;
  const real zSpan = (zMax - zStart) - radParticle;

  const int nxMax = static_cast<int>(std::floor(xSpan / pitch)) + 1;
  const int nyMax = static_cast<int>(std::floor(ySpan / pitch)) + 1;
  const int nzMax = static_cast<int>(std::floor(zSpan / pitch)) + 1;

  if (nxMax <= 0 || nyMax <= 0 || nzMax <= 0) {
    throw std::runtime_error("Fluidization setup failed: invalid grid dimensions for requested geometry.");
  }

  const int maxCapacity = nxMax * nyMax * nzMax;
  if (maxCapacity < targetParticles) {
    throw std::runtime_error("Fluidization setup failed: domain capacity smaller than requested particle count.");
  }

  int PlaneIDs = 10000;
  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
   
  createPlane(PlaneIDs++, 0.0, 0.0, 1.0, radParticle, gr, true);

  createPlane( PlaneIDs++, 0.0, 1.0, 0.0,  0.0, granite );
  // -y
  createPlane( PlaneIDs++, 0.0,-1.0, 0.0,  -0.686, granite );

  MaterialID myMaterial = createMaterial("FluidizationParticles", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  theCollisionSystem()->setMinEps(5e-6 / radParticle);

  int particlesCreated = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;
  SphereID theSphere;
  int globalIndex = 0;
  if (config.getPackingMethod() == SimulationConfig::PackingMethod::Grid) {
//          theSphere = createSphere(globalIndex, Vec3(xMid, yMid, zStart), radParticle, myMaterial, true);
//          ++particlesCreated;
//          ++globalIndex;
    for (int iz = 0; iz < nzMax && globalIndex < targetParticles; ++iz) {
      const real z = zStart + static_cast<real>(iz) * pitch;
      for (int iy = 0; iy < nyMax && globalIndex < targetParticles; ++iy) {
        const real y = (yMin + radParticle) + static_cast<real>(iy) * pitch;
        for (int ix = 0; ix < nxMax && globalIndex < targetParticles; ++ix) {
          const real x = xGridMin + static_cast<real>(ix) * pitch;
          createSphere(globalIndex, Vec3(x, y, z), radParticle, myMaterial, true);
          ++particlesCreated;
          ++globalIndex;
          usedNx = std::max(usedNx, ix + 1);
          usedNy = std::max(usedNy, iy + 1);
          usedNz = std::max(usedNz, iz + 1);
        }
      }
    }
  }

//  theSphere->setPosition(Vec3(10.4436, 0.321433, 2.11815));
//  theSphere->setLinearVel(Vec3(2.592734E-01, 1.774744E-01, 8.460528E-02));

  TimeStep::stepsize(config.getStepsize());

  if (isRepresentative) {
    const real totalVol = static_cast<real>(particlesCreated) *
      ((real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle);
    const real totalMass = totalVol * rhoParticle;

    std::cout << "\n--" << "Particle Fluidization SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Substepping                             = " << config.getSubsteps() << "\n"
              << " Auto force reset                        = enabled\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Column bounds [x,y,z]                   = "
              << "[" << xMin << "," << xMax << "] x "
              << "[" << yMin << "," << yMax << "] x "
              << "[" << zMin << "," << zMax << "]\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << " Particle radius                         = " << radParticle << "\n"
              << " Particle diameter                       = " << (real(2.0) * radParticle) << "\n"
              << " Spacing factor                          = " << spacingFactor << "\n"
              << " Gap (between surfaces)                  = " << spacing << "\n"
              << " Center-to-center pitch                  = " << pitch << "\n"
              << " Grid start z                            = " << zStart << "\n"
              << " Grid extents used (nx,ny,nz)            = "
              << usedNx << ", " << usedNy << ", " << usedNz << "\n"
              << " Total particle mass                     = " << totalMass << "\n"
              << " Total particle volume                   = " << totalVol << "\n"
              << " VTK output                              = " << (config.getVtk() ? "enabled" : "disabled") << "\n"
              << " Checkpointing                           = " << (config.getUseCheckpointer() ? "enabled" : "disabled") << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }
}

/**
 * @brief Fluidization setup for serial mode using Short-Range Repulsion (SRR)
 *
 * Pan et al. (2002) fluidization column with the ShortRangeRepulsion solver
 * instead of the lubrication-based hard-contact solver.  The SRR force law
 * (Eq. 2.1) prevents particle overlap via a soft quadratic penalty within a
 * security zone of width rho.
 *
 * Domain and particle geometry match setupFluidizationSerial (Pan et al. 2002,
 * CGS units): [0, 20.3] x [0, 0.686] x [2.0, 70.2] cm, diameter 0.635 cm.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupFluidizationSRRSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();

  world->setGravity(config.getGravity());

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  // CRITICAL: Enable automatic force reset after each simulation step
  // This prevents force accumulation when using substepping
  world->setAutoForceReset(true);

  //==============================================================================================
  // SRR solver configuration (Pan et al. 2002, Eq. 2.1, CGS parameters)
  //==============================================================================================
  // Paper parameters (§4):
  //   rho   = 0.06858 cm        (security zone width)
  //   eps_p = eps_w = 5e-7 dyne⁻¹
  //   gamma = velocity damping for near-critical settling
  //
  // Sub-cycling: F_max/m ~ 131,000 m/s²  →  N=5000 gives dt_sub = 2e-7 s
  //              for dt = 1e-3 s, Δv_sub ~ 0.026 m/s  (stable)
  const real rhoSRR   ( real(0.06858) );  // security zone width      [cm]
  //const real rhoSRR   ( real(0.0852) );  // security zone width      [cm]
  //const real rhoSRR   ( real(0.1) );  // security zone width      [cm]
  const real epsSRR   ( real(5e-7)    );  // stiffness                [dyne⁻¹]
  const real gammaSRR ( real(0.5)     );  // velocity damping         [dyne·s/cm]
  const size_t nSubcycles = 5000;

  applyOptionalSRRParams( cs->getContactSolver(), rhoSRR, epsSRR, epsSRR, gammaSRR );
  applyOptionalSubcycles( *cs, nSubcycles );

  // Inflate the lubrication threshold so that the fine collision detector
  // generates contacts at distances up to rho (the SRR security zone width).
  // Without this, contacts are only generated at gap < contactThreshold (1e-8)
  // and SRR never sees particles within the security zone.
  lubrication::setLubricationThreshold( real(0.07) );

  //==============================================================================================
  // Visualization Configuration
  //==============================================================================================
  if (isRepresentative && config.getVtk()) {
      unsigned int effectiveVisspacing = config.getVisspacing() * config.getSubsteps();
      vtk::WriterID vtk = vtk::activateWriter( "./paraview", effectiveVisspacing, 0,
                                               config.getTimesteps() * config.getSubsteps(),
                                               false);
  }

  // Activate checkpointer if configured
  if (isRepresentative && config.getUseCheckpointer()) {
    activateCheckpointer(config.getCheckpointPath(),
                         config.getPointerspacing(),
                         0, config.getTimesteps());
  }

  //==============================================================================================
  // Pan et al. (2002) column geometry (CGS units)
  //==============================================================================================
  const real xMin = 0.0;
  const real xMax = 20.3;
  const real yMin = 0.0;
  const real yMax = 0.686;
  const real zMin = 2.0;
  const real zMax = 70.2;

  const int baseTargetParticles = 1204;
  const int targetParticles = static_cast<int>(real(1.0) * static_cast<real>(baseTargetParticles));
  const real radParticle = real(0.5) * real(0.635);  // Diameter 0.635 cm
  const real minSpacingFactor = rhoSRR / radParticle;  // minimum to respect SRR security zone
  real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  if (spacingFactor < minSpacingFactor) {
    std::cerr << "\n*** WARNING: fluidizationSpacingFactor = " << spacingFactor
              << " gives inter-particle gap = " << (spacingFactor * radParticle)
              << " cm, which violates the SRR security zone rho = " << rhoSRR << " cm.\n"
              << "    Minimum safe spacing factor = " << minSpacingFactor
              << " (gap = rho = " << rhoSRR << " cm).\n"
              << "    Auto-correcting to " << minSpacingFactor << ".\n" << std::endl;
    spacingFactor = minSpacingFactor;
  }
  const real spacing = spacingFactor * radParticle;
  const real pitch = real(2.0) * radParticle + spacing;
  const real zStart = std::max(real(4.0) * radParticle, zMin + radParticle) + real(0.5);

  const real xGridMin = xMin + 1.5 * radParticle;
  const real xGridMax = xMax - radParticle;
  const real xSpan = xGridMax - xGridMin;
  const real ySpan = (yMax - yMin) - real(2.0) * radParticle;
  const real zSpan = (zMax - zStart) - radParticle;

  const int nxMax = static_cast<int>(std::floor(xSpan / pitch)) + 1;
  const int nyMax = static_cast<int>(std::floor(ySpan / pitch)) + 1;
  const int nzMax = static_cast<int>(std::floor(zSpan / pitch)) + 1;

  if (nxMax <= 0 || nyMax <= 0 || nzMax <= 0) {
    throw std::runtime_error("setupFluidizationSRRSerial: invalid grid dimensions for requested geometry.");
  }

  const int maxCapacity = nxMax * nyMax * nzMax;
  if (maxCapacity < targetParticles) {
    throw std::runtime_error("setupFluidizationSRRSerial: domain capacity smaller than requested particle count.");
  }

  //==============================================================================================
  // Materials (used by both fresh-start and resume paths).
  // Boundary planes: floor only (open top, open x-sides, no y-walls).
  // Y-walls removed: the column is quasi-2D (single particle row in y).
  // Y-confinement is enforced by locking the y-DOF on each particle instead.
  //==============================================================================================
  MaterialID gr         = createMaterial("ground",          rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID myMaterial = createMaterial("FluidizationSRR", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Place all particles at the y-midpoint of the column (quasi-2D, single row)
  const real yMid = real(0.5) * (yMin + yMax);

  int particlesCreated = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;

  const bool resume = config.getResume();
  if (resume) {
    //============================================================================================
    // Resume path: restore world state from a checkpoint.
    //
    // BodyBinaryReader::readFile clears the world before unmarshaling, then
    // restores spheres from the per-rank local buffer and planes (plus other
    // global bodies) from the globals section.  In serial PE mode the
    // pe_EXCLUSIVE_SECTION(0) block in the writer always executes, so the floor
    // plane is normally present in the checkpoint.
    //
    // Two pieces of state are NOT serialized by the Checkpointer and must be
    // re-applied here:
    //   1. linearDofMask_  — defaults to (1,1,1) on instantiation, so the
    //                        quasi-2D y-lock would otherwise be silently lost.
    //   2. The floor plane — defensive fallback in case a checkpoint format
    //                        change ever drops it from the globals section.
    //============================================================================================
    resumeFromConfiguredCheckpoint(config);

    int planeCount = 0;
    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      if ((*it)->getType() == planeType) { ++planeCount; }
    }
    if (planeCount < 3) {
      // Old checkpoints may only have the floor plane (or none).
      // Recreate all boundary planes; duplicates are harmless for
      // the collision detector (floor plane already present is fine).
      int PlaneIDs = 10000;
      createPlane(PlaneIDs++, 0.0, 0.0, 1.0, 2.0, gr, true);   // floor at z = 2.0
      createPlane(PlaneIDs++,  1.0, 0.0, 0.0,  xMin, gr, true); // left  wall at x = xMin
      createPlane(PlaneIDs++, -1.0, 0.0, 0.0, -xMax, gr, true); // right wall at x = xMax
    }

    // Re-apply quasi-2D y-DOF lock to all loaded spheres.
    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      if ((*it)->getType() == sphereType) {
        (*it)->setLinearDofMask( Vec3(1, 0, 1) );
        ++particlesCreated;
      }
    }
  } else {
    //============================================================================================
    // Fresh-start path: create the floor plane and pack particles on a regular grid.
    //============================================================================================
    int PlaneIDs = 10000;
    createPlane(PlaneIDs++, 0.0, 0.0, 1.0, 2.0, gr, true);   // floor at z = 2.0
    createPlane(PlaneIDs++,  1.0, 0.0, 0.0,  xMin, gr, true); // left  wall at x = xMin
    createPlane(PlaneIDs++, -1.0, 0.0, 0.0, -xMax, gr, true); // right wall at x = xMax

    int globalIndex = 0;
    if (config.getPackingMethod() == SimulationConfig::PackingMethod::Grid) {
      for (int iz = 0; iz < nzMax && globalIndex < targetParticles; ++iz) {
        const real z = zStart + static_cast<real>(iz) * pitch;
        for (int iy = 0; iy < nyMax && globalIndex < targetParticles; ++iy) {
          for (int ix = 0; ix < nxMax && globalIndex < targetParticles; ++ix) {
            const real x = xGridMin + static_cast<real>(ix) * pitch;
            SphereID sp = createSphere(globalIndex, Vec3(x, yMid, z), radParticle, myMaterial, true);
            // Lock y-axis: quasi-2D confinement without y-wall planes
            sp->setLinearDofMask( Vec3(1, 0, 1) );
            ++particlesCreated;
            ++globalIndex;
            usedNx = std::max(usedNx, ix + 1);
            usedNy = std::max(usedNy, iy + 1);
            usedNz = std::max(usedNz, iz + 1);
          }
        }
      }
    }
  }

  TimeStep::stepsize(config.getStepsize());

  if (isRepresentative) {
    const real sphereVol = (real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle;
    const real totalVol = static_cast<real>(particlesCreated) * sphereVol;
    const real totalMass = totalVol * rhoParticle;
    const real dt = config.getStepsize();
    const real dtSub = dt / static_cast<real>(nSubcycles);

    std::cout << "\n--" << "Fluidization SRR SETUP"
              << "--------------------------------------------------------------\n"
              << " Mode                                    = "
              << (resume ? std::string("resume from ") + config.getResumeCheckpointFile()
                         : std::string("fresh start")) << "\n"
              << " Simulation stepsize dt                  = " << dt << "\n"
              << " Substepping                             = " << config.getSubsteps() << "\n"
              << " Auto force reset                        = enabled\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Column bounds [x,y,z]                   = "
              << "[" << xMin << "," << xMax << "] x "
              << "[" << yMin << "," << yMax << "] x "
              << "[" << zMin << "," << zMax << "]\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << " Particle radius                         = " << radParticle << "\n"
              << " Particle diameter                       = " << (real(2.0) * radParticle) << "\n"
              << " Spacing factor                          = " << spacingFactor << "\n"
              << " Gap (between surfaces)                  = " << spacing << "\n"
              << " Center-to-center pitch                  = " << pitch << "\n"
              << " Grid start z                            = " << zStart << "\n"
              << " Grid extents used (nx,ny,nz)            = "
              << usedNx << ", " << usedNy << ", " << usedNz << "\n"
              << " Total particle mass                     = " << totalMass << "\n"
              << " Total particle volume                   = " << totalVol << "\n"
              << " --- SRR parameters (Pan et al. 2002) ---\n"
              << " rho (security zone)                     = " << rhoSRR << " cm\n"
              << " eps_p = eps_w                           = " << epsSRR << " dyne^-1\n"
              << " gamma (damping)                         = " << gammaSRR << " dyne.s/cm\n"
              << " nSubcycles                              = " << nSubcycles << "\n"
              << " dt_sub                                  = " << dtSub << " s\n"
              << " Lubrication threshold (contact range)    = " << lubrication::getLubricationThreshold() << " cm\n"
              << " VTK output                              = " << (config.getVtk() ? "enabled" : "disabled") << "\n"
              << " Checkpointing                           = " << (config.getUseCheckpointer() ? "enabled" : "disabled") << "\n"
              << " X-walls                                 = enabled (SRR repulsion at x=" << xMin << " and x=" << xMax << ")\n"
              << " Y-walls                                 = removed (quasi-2D)\n"
              << " Y-DOF constraint                        = locked (mask 1,0,1)\n"
              << " Particle y-placement                    = " << yMid << " cm (column midpoint)\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_FLUIDIZATION_H_
