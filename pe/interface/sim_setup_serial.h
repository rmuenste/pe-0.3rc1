#define PE_SERIAL_MODE
#ifdef PE_SERIAL_MODE

#include <pe/core.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/vtk/Writer.h>
#include <pe/vtk/DistanceMapWriter.h>
#include <pe/util/logging/Logger.h>
#include <pe/config/SimulationConfig.h>
#include <pe/util/Checkpointer.h>
#include <pe/interface/geometry_utils.h>
#include <pe/interface/sim_setup_serial_features.h>
#include <string>
#include <deque>
#include <map>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

namespace pe {

// Apply lubrication/contact hysteresis parameters only if the active collision system
// exposes the corresponding setters (e.g., HardContactLubricated). This avoids build
// failures for solvers that don't implement these knobs.
template <typename CollisionSystemT>
inline auto applyOptionalLubricationParams(CollisionSystemT& cs, const SimulationConfig& config)
    -> decltype(cs.setContactHysteresisDelta(real{}),
                cs.setLubricationHysteresisDelta(real{}),
                cs.setAlphaImpulseCap(real{}),
                cs.setMinEpsLub(real{}),
                void()) {
  cs.setContactHysteresisDelta(config.getContactHysteresisDelta());
  cs.setLubricationHysteresisDelta(config.getLubricationHysteresisDelta());
  cs.setAlphaImpulseCap(config.getAlphaImpulseCap());
  cs.setMinEpsLub(config.getMinEpsLub());
}

template <typename CollisionSystemT>
inline void applyOptionalLubricationParams(CollisionSystemT&, const SimulationConfig&) {
  // Constraint solver does not expose lubrication/hysteresis controls
}

inline unsigned int effectiveCheckpointSpacing(const SimulationConfig& config) {
  // Trigger::triggerAll() runs per substep; scale spacing to keep main-step cadence.
  return config.getPointerspacing() * config.getSubsteps();
}

/**
 * @brief Serial mode simulation setup
 *
 * In serial PE mode, each CFD domain runs its own independent serial PE instance.
 * No MPI domain decomposition or shadow copies are needed - all particles are local.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupParticleBenchSerial(int cfd_rank) {

  // Set custom rank for PE logger BEFORE any logging occurs
  // This ensures each CFD domain gets a unique log file: pe<cfd_rank>.log
  pe::logging::Logger::setCustomRank(cfd_rank);

  // Load configuration from JSON file
  SimulationConfig::loadFromFile("example.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);

  WorldID world = theWorld();
  // Apply lubrication/contact parameters from configuration
  CollisionSystemID cs = theCollisionSystem();
  const bool isRepresentative = (config.getCfdRank() == 1);
  const bool resume = config.getResume();
  applyOptionalLubricationParams(*cs, config);

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

  // Serial mode: no MPI, no domain decomposition
  // The entire simulation domain is owned by this process
  // Domain boundaries are handled by the CFD code

  int idx = 0;
  //==============================================================================================
  // Bench Configuration
  //==============================================================================================
  real radBench = config.getBenchRadius();
  real rhoParticle( config.getParticleDensity() );
  Vec3 position(-0.0, -0.0, 0.1275);

  int particlesCreated = 0;

  // Set default timestep
  TimeStep::stepsize(0.001);

  if (resume) {
    // Resume must load on every rank in PE serial mode, because each CFD rank runs
    // its own PE instance and later queries local particle storage.
    readCheckpoint(config.getCheckpointPath(), config.getResumeCheckpointFile());
    particlesCreated = static_cast<int>(theCollisionSystem()->getBodyStorage().size());
  } else {
    // Create static benchmark scene only for fresh starts.
    MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
    createPlane(777, 0.0, 0.0, 1.0, 0, gr, true);
    MaterialID myMaterial = createMaterial("Bench", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
    SphereID sphere(nullptr);
    sphere = createSphere(idx, position, radBench, myMaterial, true);
    ++idx;
    particlesCreated++;
  }

  // Activate periodic checkpoint writing on representative rank only.
  // Resume loading above is done on all ranks via a temporary reader.
  if (isRepresentative && config.getUseCheckpointer()) {
    activateCheckpointer(config.getCheckpointPath(),
                         effectiveCheckpointSpacing(config),
                         0, config.getTimesteps());
  }

  // Note: Particles will be added by FeatFloWer via the interface functions
  // This setup just initializes the PE world for this domain
  if (isRepresentative) {
    std::cout << "\n--" << "Particle Sedimentation SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << " Particle radius                         = " << radBench << "\n"
              << " Particle radius2                        = " << config.getBenchRadius() << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }
}

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
  const real zMin = 2.0;
  const real zMax = 70.2;

  const int baseTargetParticles = 1204;
  const int targetParticles = static_cast<int>(real(0.125) * static_cast<real>(baseTargetParticles));
  const real radParticle = real(0.5) * real(0.635);  // Diameter 0.635 cm
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  const real spacing = spacingFactor * radParticle;
  const real pitch = real(2.0) * radParticle + spacing;
  const real zStart = std::max(real(4.0) * radParticle, zMin + radParticle) + real(0.5);

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
  createPlane(PlaneIDs++, 0.0, 0.0, 1.0, 2.0, gr, true);

  createPlane( PlaneIDs++, 0.0, 1.0, 0.0,  0.0, granite );
  // -y
  createPlane( PlaneIDs++, 0.0,-1.0, 0.0,  -0.686, granite );

  MaterialID myMaterial = createMaterial("FluidizationParticles", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  theCollisionSystem()->setMinEps(5e-6 / radParticle);

  int particlesCreated = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;

  int globalIndex = 0;
  if (config.getPackingMethod() == SimulationConfig::PackingMethod::Grid) {
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
  const real epsSRR   ( real(5e-7)    );  // stiffness                [dyne⁻¹]
  const real gammaSRR ( real(0.5)     );  // velocity damping         [dyne·s/cm]
  const size_t nSubcycles = 5000;

  cs->getContactSolver().setRho  ( rhoSRR  );
  cs->getContactSolver().setEpsP ( epsSRR  );
  cs->getContactSolver().setEpsW ( epsSRR  );
  cs->getContactSolver().setGamma( gammaSRR );
  cs->setNumSubcycles( nSubcycles );

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
  const int targetParticles = static_cast<int>(real(0.125) * static_cast<real>(baseTargetParticles));
  const real radParticle = real(0.5) * real(0.635);  // Diameter 0.635 cm
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
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
  // Boundary planes: floor + y-walls (open top, open x-sides)
  //==============================================================================================
  int PlaneIDs = 10000;
  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  createPlane(PlaneIDs++, 0.0, 0.0, 1.0, 2.0, gr, true);   // floor at z = 2.0
  createPlane(PlaneIDs++, 0.0, 1.0, 0.0, 0.0, gr, true);   // +y wall at y = 0
  createPlane(PlaneIDs++, 0.0,-1.0, 0.0, -0.686, gr, true); // -y wall at y = 0.686

  //==============================================================================================
  // Particle placement (grid packing)
  //==============================================================================================
  MaterialID myMaterial = createMaterial("FluidizationSRR", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  int particlesCreated = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;

  int globalIndex = 0;
  if (config.getPackingMethod() == SimulationConfig::PackingMethod::Grid) {
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

  TimeStep::stepsize(config.getStepsize());

  if (isRepresentative) {
    const real sphereVol = (real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle;
    const real totalVol = static_cast<real>(particlesCreated) * sphereVol;
    const real totalMass = totalVol * rhoParticle;
    const real dt = config.getStepsize();
    const real dtSub = dt / static_cast<real>(nSubcycles);

    std::cout << "\n--" << "Fluidization SRR SETUP"
              << "--------------------------------------------------------------\n"
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
              << " VTK output                              = " << (config.getVtk() ? "enabled" : "disabled") << "\n"
              << " Checkpointing                           = " << (config.getUseCheckpointer() ? "enabled" : "disabled") << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }
}

/**
 * @brief Generic serial mode initialization
 *
 * This is a generic initialization function that can be used for various
 * simulation setups in serial mode.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupGeneralInitSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  WorldID world = theWorld();

  // Basic world setup
  world->setGravity(0.0, 0.0, -9.807);
  world->setDamping(1.0);

  // Default timestep
  TimeStep::stepsize(config.getStepsize());

  // Serial mode: minimal setup, particles managed by CFD code
}

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
 * @brief Archimedes benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupArchimedesSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  // Archimedes buoyancy test
  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  // Fluid properties
  real simViscosity(1.0e-3);
  real simRho(1000.0);

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(0.001);
}

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

  WorldID world = theWorld();

  // Apply lubrication/contact parameters from configuration
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  // Enable adaptive Baumgarte capping for contact stabilization
  theCollisionSystem()->setAdaptiveBaumgarteCapping(true, 50.0);

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
  // Domain Boundary Configuration
  //==============================================================================================
  std::string boundaryFileName = std::string("atc_boundary_param_zero.obj");
  Vec3 boundaryPos(0.0, 0.0, 0.0);  // Boundary mesh in reference position
  MaterialID boundaryMat = createMaterial("boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Create boundary mesh as fixed global object
  TriangleMeshID boundaryMesh = createTriangleMesh(idx++, boundaryPos, boundaryFileName, boundaryMat, false, true);
  boundaryMesh->setFixed(true);
  //boundaryMesh->setPosition(Vec3(0.000169, -2.256, 0.087286));
  boundaryMesh->setPosition(Vec3(0.000006, -2.25096, 0.08719));

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

  boundaryMesh->getDistanceMap()->invertForDomainBoundary();
  if (boundaryDistanceMapEnabled && isRepresentative) {
    // Invert the DistanceMap for domain boundary representation
    // This transforms: inside mesh -> inside fluid domain (positive distance)
    //                  outside mesh -> outside fluid domain (negative distance)
    //                  normals -> point inward to keep particles in domain
    //boundaryMesh->getDistanceMap()->invertForDomainBoundary();
    std::cout << "\n--" << "DOMAIN BOUNDARY SETUP"
              << "--------------------------------------------------------------\n"
              << " Boundary mesh file                      = " << boundaryFileName << "\n"
              << " Boundary mesh volume                    = " << boundaryMesh->getVolume() << "\n"
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

    // Write inverted DistanceMap to VTK for visual inspection
    vtk::DistanceMapWriter::writeVTI("atc_boundary_distancemap.vti", *boundaryDM);
    std::cout << "Inverted DistanceMap written to: atc_boundary_distancemap.vti\n" << std::endl;

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
#else
  const bool boundaryDistanceMapEnabled = false;
  if (isRepresentative) {
    std::cout << "WARNING: DistanceMap not available (PE_USE_CGAL not defined)" << std::endl;
  }
#endif
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
  real rhoParticle( config.getParticleDensity() );
  MaterialID particleMaterial = createMaterial("particleMaterial", rhoParticle, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);

  // Generate sphere positions along the centerline.
  // num_rings=5 (one extra ring vs. before), margin_steps=20 (up from 4) to keep
  // a larger empty region at inlet/outlet while preserving ~6992 total particles:
  //   ~60 cross-sections × 118 particles/section ≈ 7080  (within ±100 of previous count)
  std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges, sphereRad, -1.0, 4, 20, 4);

  int particlesCreated = 0;
  if (config.getPackingMethod() != SimulationConfig::PackingMethod::None) {
    // Create spheres at generated positions (serial mode: no domain ownership check needed)
    for (auto spherePos: spherePositions) {
      createSphere(idx++, spherePos, sphereRad, particleMaterial);
      particlesCreated++;
    }
  }

  // Volume fraction computation
  real domainVol = 0.78;
  real partVol = 4. / 3. * M_PI * std::pow(sphereRad, 3);

  if (isRepresentative) {
    std::cout << "\n--" << "ATC SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Total number of particles               = " << particlesCreated << "\n"
              << " Particle radius                         = " << sphereRad << "\n"
              << " Particle volume                         = " << partVol << "\n"
              << " Domain volume                           = " << domainVol << "\n"
              << " Volume fraction[%]                      = " << (particlesCreated * partVol) / domainVol * 100.0 << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }

}

/**
 * @brief Kroupa setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupKroupaSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  real simViscosity(1.0);
  real simRho(1.0);

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Creep flow setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupCreepSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, 0.0);  // Often negligible in creep flow
  world->setDamping(1.0);

  real simViscosity(1.0);
  real simRho(1.0);

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(0.001);
}

/**
 * @brief DNS drag setup for serial mode: periodic unit-cube sphere array
 *
 * Creates a monodisperse simple-cubic array of fixed spheres in a unit cube.
 * The number of spheres is derived from volume fraction and radius settings.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDNSDragSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto& config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);

  TimeStep::stepsize(config.getStepsize());

  const real radius = config.getBenchRadius();
  if (radius <= 0.0) {
    throw std::runtime_error("setupDNSDragSerial: benchRadius must be > 0");
  }

  const real domainLength = 0.1;
  const real domainVolume = domainLength * domainLength * domainLength;
  const real diameter = 2.0 * radius;
  const real wallSurfaceGap = diameter;
  const real particleSurfaceGap = diameter;
  const real wallCenterOffset = radius + wallSurfaceGap;
  const real minCenterSpacing = diameter + particleSurfaceGap;

  const real targetVF = std::max(real(0.0), config.getVolumeFraction());
  const real sphereVol = (4.0 / 3.0) * M_PI * std::pow(radius, 3);

  int targetCount = 1;
  if (targetVF > 0.0 && sphereVol > 0.0) {
    targetCount = std::max(1, static_cast<int>(std::round(targetVF * domainVolume / sphereVol)));
  }

  int nPerDim = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(targetCount))));
  nPerDim = std::max(1, nPerDim);

  const real availableSpan = domainLength - 2.0 * wallCenterOffset;
  int nPerDimMax = 0;
  if (availableSpan >= 0.0) {
    nPerDimMax = 1 + static_cast<int>(std::floor(availableSpan / minCenterSpacing));
  }
  if (nPerDimMax < 1) {
    throw std::runtime_error(
        "setupDNSDragSerial: spheres do not fit into [0,0.1]^3 with one-diameter wall clearance");
  }
  nPerDim = std::min(nPerDim, nPerDimMax);

  const real spacing = (nPerDim > 1) ? availableSpan / static_cast<real>(nPerDim - 1) : 0.0;
  MaterialID particleMaterial = createMaterial(
      "dns_drag_particle", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  int idx = 0;
  int particlesCreated = 0;
  for (int k = 0; k < nPerDim; ++k) {
    for (int j = 0; j < nPerDim; ++j) {
      for (int i = 0; i < nPerDim; ++i) {
        Vec3 pos(wallCenterOffset + static_cast<real>(i) * spacing,
                 wallCenterOffset + static_cast<real>(j) * spacing,
                 wallCenterOffset + static_cast<real>(k) * spacing);
        SphereID sphere = createSphere(++idx, pos, radius, particleMaterial, true);
        sphere->setFixed(true);
        sphere->setLinearVel(0.0, 0.0, 0.0);
        sphere->setAngularVel(0.0, 0.0, 0.0);
        ++particlesCreated;
      }
    }
  }

  const real actualVF = particlesCreated * sphereVol / domainVolume;

  if (isRepresentative) {
    std::cout << "\n--DNS DRAG SETUP---------------------------------------------\n"
              << " Fluid viscosity                         = " << config.getFluidViscosity() << "\n"
              << " Fluid density                           = " << config.getFluidDensity() << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Domain length                           = " << domainLength << "\n"
              << " Radius                                  = " << radius << "\n"
              << " Target volume fraction                  = " << targetVF << "\n"
              << " Wall surface gap                        = " << wallSurfaceGap << "\n"
              << " Particle surface gap                    = " << particleSurfaceGap << "\n"
              << " Spheres per dimension                   = " << nPerDim << "\n"
              << " Center spacing                          = " << spacing << "\n"
              << " Number of spheres                       = " << particlesCreated << "\n"
              << " Achieved volume fraction                = " << actualVF << "\n"
              << "-------------------------------------------------------------\n"
              << std::endl;
  }
}

/**
 * @brief Draft-Kiss-Tumble benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDraftKissTumbSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  real simViscosity(1.0);
  real simRho(1.0);

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(0.001);
}

/**
 * @brief 2x2x2 domain setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setup2x2x2Serial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Cylinder setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupCylSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Generic benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupBenchSerial(int cfd_rank) {
  setupGeneralInitSerial(cfd_rank);
}

/**
 * @brief Span setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupSpanSerial(int cfd_rank) {
  setupFSIBenchSerial(cfd_rank);
}

/**
 * @brief Lubrication Lab setup for serial mode
 *
 * Flexible setup for lubrication model experiments based on setupParticleBenchSerial.
 * Configuration loaded from JSON file.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupLubricationLabSerial(int cfd_rank) {
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

  // Set gravity from configuration
  world->setGravity( config.getGravity() );

  // Configuration from config singleton
  real simViscosity( config.getFluidViscosity() );
  real simRho( config.getFluidDensity() );

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  // Serial mode: no MPI, no domain decomposition
  // The entire simulation domain is owned by this process
  // Domain boundaries are handled by the CFD code

  // Create ground plane (global, owned by this domain)
  MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  createPlane(777, 0.0, 0.0, 1.0, 0, gr, true);

  int idx = 0;
  //==============================================================================================
  // Lubrication Lab Configuration
  //==============================================================================================
  real radBench = config.getBenchRadius();
  real rhoParticle( config.getParticleDensity() );
  Vec3 position(0.05, 0.05, 0.02);

  MaterialID myMaterial = createMaterial("Bench", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  SphereID sphere(nullptr);
  sphere = createSphere(idx, position, radBench, myMaterial, true);
  ++idx;

  // Set default timestep
  TimeStep::stepsize(config.getStepsize());

  // Activate checkpointer if configured
  if (isRepresentative && config.getUseCheckpointer()) {
    activateCheckpointer(config.getCheckpointPath(),
                         effectiveCheckpointSpacing(config),
                         0, config.getTimesteps());
  }

  if (isRepresentative) {
    std::cout << "\n--" << "LUBRICATION LAB SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Particle radius                         = " << radBench << "\n"
              << " Particle density                        = " << rhoParticle << "\n"
              << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  // Note: Particles will be added by FeatFloWer via the interface functions
  // This setup just initializes the PE world for this domain
}

/**
 * @brief Drill application setup for serial mode
 *
 * Setup for drill simulation with rotating drill mesh and workpiece.
 * Based on FSI benchmark setup with additional drill mesh.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDrillSerial(int cfd_rank) {
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
  MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  createPlane(777, 0.0, 0.0, 1.0, 0, gr, true);

  int idx = 0;
  //==============================================================================================
  // Workpiece Configuration
  //==============================================================================================
  real rhoParticle( config.getParticleDensity() );
  Vec3 position(-0.0, -0.0, 0.1275);
  std::string fileName = std::string("span2_scaled.obj");

  // Set default timestep
  TimeStep::stepsize(config.getStepsize());
  MaterialID chipMat = createMaterial("chip", rhoParticle, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);

  Vec3 chipPos = position;  // Keep hardcoded - no config function available
  TriangleMeshID chip = createTriangleMesh(++idx, chipPos, fileName, chipMat, false, true);

#ifdef PE_USE_CGAL
  // Enable DistanceMap acceleration for the chip (workpiece)
  chip->enableDistanceMapAcceleration(64, 3);  // spacing, resolution, tolerance
  const bool chipDistanceMapEnabled = chip->hasDistanceMap();
  const DistanceMap* chipDM = chipDistanceMapEnabled ? chip->getDistanceMap() : nullptr;
  if (!chipDistanceMapEnabled && isRepresentative) {
    std::cerr << "WARNING: DistanceMap acceleration failed to initialize for chip" << std::endl;
  }
#else
  const bool chipDistanceMapEnabled = false;
#endif

  //==============================================================================================
  // Drill Configuration
  //==============================================================================================
  std::string drillFileName = std::string("tool_scaled.obj");
  Vec3 drillPos(0.0, 0.0, 0.0);  // Position drill above workpiece
  MaterialID drillMat = createMaterial("drill", 7800.0, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);  // Steel density

  // Create drill as global rigid body
  TriangleMeshID drill = createTriangleMesh(++idx, drillPos, drillFileName, drillMat, false, true);
  drill->setFixed(true);

#ifdef PE_USE_CGAL
  // Enable DistanceMap acceleration for the drill
  drill->enableDistanceMapAcceleration(64, 3);
  const bool drillDistanceMapEnabled = drill->hasDistanceMap();
  const DistanceMap* drillDM = drillDistanceMapEnabled ? drill->getDistanceMap() : nullptr;
  if (!drillDistanceMapEnabled && isRepresentative) {
    std::cerr << "WARNING: DistanceMap acceleration failed to initialize for drill" << std::endl;
  }
#else
  const bool drillDistanceMapEnabled = false;
#endif

  if (isRepresentative) {
    std::cout << "\n--" << "DRILL SIMULATION SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << simViscosity << "\n"
              << " Fluid density                           = " << simRho << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Workpiece mesh file                     = " << fileName << "\n"
              << " Drill mesh file                         = " << drillFileName << "\n"
              << " VTK output                              = " << (config.getVtk() ? "enabled" : "disabled") << "\n"
              << " Workpiece distance map enabled          = " << (chipDistanceMapEnabled ? "yes" : "no") << "\n"
              << " Drill distance map enabled              = " << (drillDistanceMapEnabled ? "yes" : "no") << "\n";

#ifdef PE_USE_CGAL
    if (chipDistanceMapEnabled && chipDM) {
      std::cout << " Workpiece distance map grid size        = "
                << chipDM->getNx() << " x " << chipDM->getNy() << " x " << chipDM->getNz() << "\n"
                << " Workpiece distance map origin           = ("
                << chipDM->getOrigin()[0] << ", "
                << chipDM->getOrigin()[1] << ", "
                << chipDM->getOrigin()[2] << ")\n"
                << " Workpiece distance map spacing          = " << chipDM->getSpacing() << "\n";
    }
    if (drillDistanceMapEnabled && drillDM) {
      std::cout << " Drill distance map grid size            = "
                << drillDM->getNx() << " x " << drillDM->getNy() << " x " << drillDM->getNz() << "\n"
                << " Drill distance map origin               = ("
                << drillDM->getOrigin()[0] << ", "
                << drillDM->getOrigin()[1] << ", "
                << drillDM->getOrigin()[2] << ")\n"
                << " Drill distance map spacing              = " << drillDM->getSpacing() << "\n";
    }
#endif

    std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }
}

/**
 * @brief DCAV (Double Concentric Annular Viscometer?) setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDCAVSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::getInstance().setCfdRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, 0.0);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

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

/**
 * @brief Serial PE time stepping function with substepping support
 *
 * This function performs one time step of the PE simulation in serial mode with
 * configurable substepping. Substepping allows for more stable integration by
 * breaking each main timestep into multiple smaller substeps.
 *
 * Unlike the MPI version, this does not use MPI communications or domain decomposition.
 * Each CFD domain runs its own independent PE instance.
 *
 * Substepping configuration:
 * - Number of substeps is read from SimulationConfig::getSubsteps() (default: 1)
 * - VTK output is only triggered on the final substep to maintain output frequency
 * - Particle diagnostics are printed once per main timestep
 * - Checkpointer (if enabled) is triggered once per main timestep
 */
inline void stepSimulationSerial() {
  WorldID world = theWorld();
  auto &config = SimulationConfig::getInstance();
  const bool isRepresentative = (config.getCfdRank() == 1);

  static int timestep = 0;

  // Test log message to verify logging is working
  if (timestep == 0) {
    pe_LOG_INFO_SECTION( log ) {
      log << "PE Logging initialized for CFD rank " << config.getCfdRank() << "\n";
      log << "This log file should be named: pe" << config.getCfdRank() << ".log\n";
    }
  }

  // Substepping configuration
  int substeps = config.getSubsteps();
  real fullStepSize = config.getStepsize();
  real substepSize = fullStepSize / static_cast<real>(substeps);

  // CRITICAL: Apply external forces (fluid drag) with FULL timestep BEFORE substepping
  // The fluid forces are set by the CFD code before this function is called.
  // If we don't apply them here with the full timestep, they will only be applied
  // in the first substep (with reduced dt), then reset to zero, causing incorrect dynamics.
  // Use the PE library's built-in applyFluidForces() method which properly handles
  // force application and reset.
  SerialStepContext serialStepCtx = {
      world, config, isRepresentative, timestep, fullStepSize, substeps};
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;

    // DIAGNOSTIC: Track velocity changes from fluid force application
    Vec3 v_before = body->getLinearVel();
    Vec3 force = body->getForce();
    real forceMag = force.length();
    real mass = body->getMass();

    // Apply fluid forces using the library function with full main timestep
    // This applies forces to velocities and resets forces if forceReset is enabled
    body->applyFluidForces(fullStepSize);

    Vec3 v_after = body->getLinearVel();
    Vec3 deltaV = v_after - v_before;
    real deltaVMag = deltaV.length();
    FluidForceApplicationContext forceCtx = {
        body, v_before, force, forceMag, mass, v_after, deltaV, deltaVMag};
    serialStepFeatureSet().afterFluidForceApplication(serialStepCtx, forceCtx);
  }

  // Set global timestep to substep size for physics integration
  TimeStep::stepsize(substepSize);

  // Execute substeps (for collision handling and gravity)
  for (int istep = 0; istep < substeps; ++istep) {
    // Perform one substep
    // Note: world->simulationStep() calls Trigger::triggerAll() which triggers VTK output
    // VTK output frequency is controlled by adjusting visspacing in setup functions
    // (see setupDrillSerial() and setupFSIBenchSerial() for examples)
    world->simulationStep(substepSize);
  }

  // Restore original timestep size
  TimeStep::stepsize(fullStepSize);

  serialStepFeatureSet().afterMainStep(serialStepCtx);

  // Particle diagnostics output (once per main timestep only)
  if (isRepresentative) {
    unsigned int i(0);
    for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
      World::SizeType widx = static_cast<World::SizeType>(i);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if(body->getType() == sphereType ||
         body->getType() == capsuleType ||
         body->getType() == ellipsoidType ||
         body->getType() == cylinderType ||
         body->getType() == triangleMeshType) {

        const Vec3 vel = body->getLinearVel();
        const Vec3 ang = body->getAngularVel();
#ifdef PE_SERIAL_VERBOSE_PARTICLE_OUTPUT
        std::cout << "==Single Particle Data========================================================" << std::endl;

        std::cout << "Position: " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                     body->getPosition()[0] << " " <<
                                     body->getPosition()[1] << " " <<
                                     body->getPosition()[2] << std::endl;
        std::cout << "Velocity: " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                     body->getLinearVel()[0] << " " <<
                                     body->getLinearVel()[1] << " " <<
                                     body->getLinearVel()[2] << std::endl;
        std::cout << "Omega:    " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                     body->getAngularVel()[0] << " " <<
                                     body->getAngularVel()[1] << " " <<
                                     body->getAngularVel()[2] << std::endl;
       std::cout << "We are using " << substeps << " substeps per CFD step." << std::endl;
#endif
      }
    }
  }

  timestep++;
}

/**
 * @brief HashGrid test setup for serial mode
 *
 * Creates 1000 spheres in a 10×10×10 grid for HashGrid acceleration testing.
 * Minimal setup with no JSON dependency - all parameters hardcoded.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupHashGridTest(int cfd_rank) {
  // Logger setup
  pe::logging::Logger::setCustomRank(cfd_rank);

  // Load configuration from JSON file
  SimulationConfig::loadFromFile("pe_user_config.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  // Get World and CollisionSystem
  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();

  // Configure world from config file
  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);
  world->setAutoForceReset(true);
  
  //==============================================================================================
  // Visualization Configuration
  //==============================================================================================
  if (isRepresentative) {
      // Adjust VTK output spacing for substepping
      // With substepping, world->simulationStep() is called substeps times per main timestep
      // To maintain the same VTK output frequency, multiply spacing by substeps
      unsigned int effectiveVisspacing = config.getVisspacing() * config.getSubsteps();
      vtk::WriterID vtk = vtk::activateWriter( "./paraview", effectiveVisspacing, 0,
                                               config.getTimesteps() * config.getSubsteps(),
                                               true);
  }

  // Create Iron material (PE standard)
//  MaterialID iron = createMaterial("iron",
//    7.874,    // density kg/dm³
//    0.5,      // coefficient of restitution
//    0.1,      // static friction
//    0.1,      // dynamic friction
//    0.24,     // Poisson ratio
//    200.0,    // Young's modulus (GPa)
//    200.0,    // stiffness
//    0.0,      // normal damping
//    0.0);     // tangential damping

  // Grid parameters
  const real spacing = 0.1;              // center-to-center
  const int nx = 10, ny = 10, nz = 10;   // 1000 objects total
  const real startPos = 0.05;            // offset to center grid

  // Get packing method from config (default to Grid if not set)
  auto packingMethod = config.getPackingMethod();

  int idx = 0;
  int sphereCount = 0, boxCount = 0, capsuleCount = 0, meshCount = 0;

  if (packingMethod == SimulationConfig::PackingMethod::Grid) {
    // ==================================================================================
    // GRID PACKING: Regular sphere grid (original behavior)
    // ==================================================================================
    const real radius = 0.01;

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          // Create sphere (all local in serial mode)
          SphereID sphere = createSphere(idx, position, radius, iron, true);
          sphere->setLinearVel(0.0, 0.0, 0.0);
          sphere->setAngularVel(0.0, 0.0, 0.0);
          ++idx;
          ++sphereCount;
        }
      }
    }
  }
  else if (packingMethod == SimulationConfig::PackingMethod::MixedGrid) {
    // ==================================================================================
    // MIXED GRID PACKING: Mixed shapes with bounding sphere radius 0.02
    // ==================================================================================

    // All shapes fit within bounding sphere of radius 0.02
    const real boundingRadius = 0.02;

    // Sphere: radius = bounding radius
    const real sphereRadius = boundingRadius;  // 0.02

    // Box: cube fitting in bounding sphere
    // For a cube with side length a, bounding sphere radius = a*sqrt(3)/2
    // So: a = 2*r/sqrt(3)
    const real boxSideLength = 2.0 * boundingRadius / std::sqrt(3.0);  // ≈0.0231

    // Capsule: cylinder with hemispherical caps
    // Capsule radius = 0.01, cylindrical length = 0.02
    // Bounding sphere = sqrt(0.01² + 0.01²) ≈ 0.01414 < 0.02 ✓
    const real capsuleRadius = 0.01;
    const real capsuleLength = 0.02;  // Length of cylindrical part (excluding caps)

    int shapeType = 0;  // 0=sphere, 1=box, 2=capsule (cycle through)

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          if (shapeType == 0) {
            // Create sphere
            SphereID sphere = createSphere(idx, position, sphereRadius, iron, true);
            sphere->setLinearVel(0.0, 0.0, 0.0);
            sphere->setAngularVel(0.0, 0.0, 0.0);
            ++sphereCount;
          }
          else if (shapeType == 1) {
            // Create box (cube)
            Vec3 lengths(boxSideLength, boxSideLength, boxSideLength);
            BoxID box = createBox(idx, position, lengths, iron, true);
            box->setLinearVel(0.0, 0.0, 0.0);
            box->setAngularVel(0.0, 0.0, 0.0);
            ++boxCount;
          }
          else {
            // Create capsule (oriented along z-axis by default)
            CapsuleID capsule = createCapsule(idx, position, capsuleRadius, capsuleLength, iron, true);
            capsule->setLinearVel(0.0, 0.0, 0.0);
            capsule->setAngularVel(0.0, 0.0, 0.0);
            ++capsuleCount;
          }

          ++idx;
          shapeType = (shapeType + 1) % 3;  // Cycle: 0→1→2→0→...
        }
      }
    }
  }
  else if (packingMethod == SimulationConfig::PackingMethod::TriangleMeshGrid) {
    // ==================================================================================
    // TRIANGLE MESH GRID PACKING: Grid of cone meshes from cone.obj
    // ==================================================================================

    // Load cone.obj mesh - it should fit within bounding sphere of radius 0.02
    std::string coneFileName = std::string("cone.obj");

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          // Create triangle mesh cone at this position
          // global = true (not fixed, can move with fluid forces)
          TriangleMeshID cone = createTriangleMesh(idx, position, coneFileName, iron, false, true);
          cone->setLinearVel(0.0, 0.0, 0.0);
          cone->setAngularVel(0.0, 0.0, 0.0);
          ++idx;
          ++meshCount;
        }
      }
    }
  }

  // Set timestep from config
  TimeStep::stepsize(config.getStepsize());

  // Diagnostic output
  if (cfd_rank == 1) {
    std::cout << "\n--HASHGRID TEST SETUP\n";
    std::cout << "Configuration file: pe_user_config.json\n";
    std::cout << "Domain: [0, 1]^3\n";
    std::cout << "Grid: " << nx << "x" << ny << "x" << nz << " = " << idx << " objects\n";
    std::cout << "Spacing: " << spacing << "\n";
    std::cout << "Material: Iron (density=" << 7.874 << ")\n";
    std::cout << "Timestep: " << config.getStepsize() << " (" << config.getTimesteps() << " steps)\n";
    std::cout << "Fluid density: " << config.getFluidDensity() << "\n";
    std::cout << "Fluid viscosity: " << config.getFluidViscosity() << "\n";
    std::cout << "Gravity: " << config.getGravity() << "\n";

    if (packingMethod == SimulationConfig::PackingMethod::Grid) {
      std::cout << "Packing: Grid (spheres only)\n";
      std::cout << "  Spheres: " << sphereCount << " (radius=0.01)\n";
    }
    else if (packingMethod == SimulationConfig::PackingMethod::MixedGrid) {
      std::cout << "Packing: MixedGrid (bounding radius=0.02)\n";
      std::cout << "  Spheres: " << sphereCount << " (radius=0.02)\n";
      std::cout << "  Boxes: " << boxCount << " (side≈0.0231)\n";
      std::cout << "  Capsules: " << capsuleCount << " (r=0.01, l=0.02)\n";
    }
    else if (packingMethod == SimulationConfig::PackingMethod::TriangleMeshGrid) {
      std::cout << "Packing: TriangleMeshGrid (bounding radius=0.02)\n";
      std::cout << "  Triangle Meshes (cones): " << meshCount << " (from cone.obj)\n";
    }
    std::cout << std::endl;
  }
}

} // namespace pe

#endif  // PE_SERIAL_MODE
