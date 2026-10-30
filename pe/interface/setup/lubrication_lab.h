//=================================================================================================
/*!\file pe/interface/setup/lubrication_lab.h
 *  \brief Lubrication laboratory: one sphere over a plane (serial mode only).
 *
 *  Entry point: commf2c_lubrication_lab_(); the parallel variant aborts with a message.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_LUBRICATION_LAB_H_
#define _PE_INTERFACE_SETUP_LUBRICATION_LAB_H_

// No parallel setup: the parallel entry point aborts with a message.

#include <pe/interface/setup/serial_common.h>

namespace pe {

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

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_LUBRICATION_LAB_H_
