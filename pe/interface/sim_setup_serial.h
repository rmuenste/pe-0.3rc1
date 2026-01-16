#define PE_SERIAL_MODE
#ifdef PE_SERIAL_MODE

#include <pe/core.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/vtk/Writer.h>
#include <pe/vtk/DistanceMapWriter.h>
#include <pe/util/logging/Logger.h>
#include <pe/config/SimulationConfig.h>
#include <string>

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

  // Create ground plane (global, owned by this domain)
  MaterialID gr = createMaterial("ground", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  createPlane(777, 0.0, 0.0, 1.0, 0, gr, true);

  int idx = 0;
  //==============================================================================================
  // Bench Configuration
  //==============================================================================================
  real radBench = config.getBenchRadius();
  real rhoParticle( config.getParticleDensity() );
  Vec3 position(-0.0, -0.0, 0.1275);

  MaterialID myMaterial = createMaterial("Bench", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  SphereID sphere(nullptr);
  sphere = createSphere(idx, position, radBench, myMaterial, true);
  ++idx;


  // Set default timestep
  TimeStep::stepsize(0.001);

  // Note: Particles will be added by FeatFloWer via the interface functions
  // This setup just initializes the PE world for this domain
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
  // Domain Boundary Configuration
  //==============================================================================================
  int idx = 0;
  std::string boundaryFileName = std::string("atc_boundary.obj");
  Vec3 boundaryPos(0.0, 0.0, 0.0);  // Boundary mesh in reference position
  MaterialID boundaryMat = createMaterial("boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Create boundary mesh as fixed global object
  TriangleMeshID boundaryMesh = createTriangleMesh(idx++, boundaryPos, boundaryFileName, boundaryMat, false, true);
  boundaryMesh->setFixed(true);
  boundaryMesh->setPosition(Vec3(0.000169, -2.256, 0.087286));

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

  // TODO: Add additional rigid bodies here
  // Example:
  // MaterialID myMaterial = createMaterial("ATCMaterial", density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT);
  // SphereID sphere = createSphere(idx++, position, radius, myMaterial, global);
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
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;

    // Apply fluid forces using the library function with full main timestep
    // This applies forces to velocities and resets forces if forceReset is enabled
    body->applyFluidForces(fullStepSize);
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
      }
    }
  }

  // Checkpointer trigger (if enabled) - once per main timestep
  // Note: Checkpointer is not auto-triggered via Trigger::triggerAll(),
  // so it must be called explicitly here after the substepping loop
  //
  // TODO: To enable checkpointing in serial mode:
  // 1. Create a global Checkpointer object similar to MPI mode (see sim_setup.cpp line 72-76)
  // 2. Initialize it with config parameters in a serial mode initialization function
  // 3. Add the following code here:
  //    if (config.getUseCheckpointer()) {
  //      checkpointer.trigger();
  //      checkpointer.flush();
  //    }

  timestep++;
}

} // namespace pe

#endif  // PE_SERIAL_MODE
