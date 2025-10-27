#ifdef PE_SERIAL_MODE

#include <pe/core.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/vtk/Writer.h>
#include <pe/util/logging/Logger.h>
#include <pe/config/SimulationConfig.h>
#include <string>

namespace pe {

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
  WorldID world = theWorld();

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

  WorldID world = theWorld();

  // Basic world setup
  world->setGravity(0.0, 0.0, -9.807);
  world->setDamping(1.0);

  // Default timestep
  TimeStep::stepsize(0.001);

  // Serial mode: minimal setup, particles managed by CFD code
}

/**
 * @brief FSI benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupFSIBenchSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

  WorldID world = theWorld();

  // FSI-specific settings
  world->setGravity(0.0, 0.0, 0.0);  // Often no gravity in FSI benchmarks
  world->setDamping(1.0);

  // Fluid properties for FSI
  real simViscosity(1.0e-3);  // Water-like
  real simRho(1000.0);        // Water density

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Archimedes benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupArchimedesSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

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
 * @brief Kroupa setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupKroupaSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

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
 * @brief DCAV (Double Concentric Annular Viscometer?) setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDCAVSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);

  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, 0.0);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Serial PE time stepping function
 *
 * This function performs one time step of the PE simulation in serial mode.
 * Unlike the MPI version, this does not use MPI communications or domain decomposition.
 * Each CFD domain runs its own independent PE instance.
 */
inline void stepSimulationSerial() {
  WorldID world = theWorld();
  
  static int timestep = 0;
  // Get the configured time step size
  real stepsize = TimeStep::size();
  
  // Perform one simulation step without MPI operations
  // In serial mode, all particles are local to this domain
  world->simulationStep(stepsize);
  
  unsigned int i(0);
  // No MPI barriers or reductions needed in serial mode
  // Force synchronization is handled by the CFD layer
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType || 
       body->getType() == capsuleType || 
       body->getType() == ellipsoidType || 
       body->getType() == cylinderType || 
       body->getType() == triangleMeshType) {

      Vec3 vel = body->getLinearVel();
      Vec3 ang = body->getAngularVel();

      std::cout << "==Single Particle Data========================================================" << std::endl;
      std::cout << "Position: " << body->getSystemID() << " " << body->getPosition()[2]  << " " << timestep * stepsize << std::endl;
      std::cout << "Velocity: " << body->getSystemID() << " " << body->getLinearVel()[2]  << " " << timestep * stepsize << std::endl;
      std::cout << "Angular: " << body->getSystemID() << " "<< body->getAngularVel()  << " " << timestep * stepsize << std::endl;
      
    }
  }
  timestep++;
}

} // namespace pe

#endif  // PE_SERIAL_MODE
