#ifdef PE_SERIAL_MODE

#include <pe/core.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/vtk/Writer.h>

namespace pe {

/**
 * @brief Serial mode simulation setup
 *
 * In serial PE mode, each CFD domain runs its own independent serial PE instance.
 * No MPI domain decomposition or shadow copies are needed - all particles are local.
 */
inline void setupParticleBenchSerial() {

  WorldID world = theWorld();

  // Set default gravity (can be overridden by loadSimulationConfig if available)
  world->setGravity(0.0, 0.0, -9.807);

  // Default fluid properties
  real simViscosity(1.0);
  real simRho(1.0);
  real slipLength(0.75);

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
 */
inline void setupGeneralInitSerial() {
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
 */
inline void setupFSIBenchSerial() {
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
 */
inline void setupArchimedesSerial() {
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
 */
inline void setupKroupaSerial() {
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
 */
inline void setupCreepSerial() {
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
 */
inline void setupDraftKissTumbSerial() {
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
 */
inline void setup2x2x2Serial() {
  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Cylinder setup for serial mode
 */
inline void setupCylSerial() {
  WorldID world = theWorld();

  world->setGravity(0.0, 0.0, -9.81);
  world->setDamping(1.0);

  TimeStep::stepsize(0.001);
}

/**
 * @brief Generic benchmark setup for serial mode
 */
inline void setupBenchSerial() {
  setupGeneralInitSerial();
}

/**
 * @brief Span setup for serial mode
 */
inline void setupSpanSerial() {
  setupFSIBenchSerial();
}

/**
 * @brief DCAV (Double Concentric Annular Viscometer?) setup for serial mode
 */
inline void setupDCAVSerial() {
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
  
  // Get the configured time step size
  real stepsize = TimeStep::size();
  
  // Perform one simulation step without MPI operations
  // In serial mode, all particles are local to this domain
  world->simulationStep(stepsize);
  
  // No MPI barriers or reductions needed in serial mode
  // Force synchronization is handled by the CFD layer
}

} // namespace pe

#endif  // PE_SERIAL_MODE
