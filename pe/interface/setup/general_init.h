//=================================================================================================
/*!\file pe/interface/setup/general_init.h
 *  \brief Minimal PE bootstrap for CFD applications that run no particle simulation.
 *
 *  Entry point: commf2c_init_(). setupBenchSerial() is an alias kept for older callers.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_GENERAL_INIT_H_
#define _PE_INTERFACE_SETUP_GENERAL_INIT_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <iostream>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/World.h>
#include <pe/core/WorldID.h>

namespace pe {

//*************************************************************************************************
/*!\brief Minimal general-purpose PE bootstrap for CFD applications (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_init_(). It creates NO world content (no bodies,
 * no materials, no domain decomposition, no config file). It exists for two reasons, and
 * both must keep working:
 *
 * 1. Link anchor. The CFD library is built once with PE support, and its shared code calls
 *    the PE interface wrappers (particle queries, force synchronization, ...). Every CFD
 *    application links that shared code, including applications that never run a particle
 *    simulation. With static libraries the linker only extracts the PE interface objects
 *    if something references them at the right point of the link, so such applications
 *    would otherwise fail with undefined references to the wrappers. Calling this minimal
 *    setup from the application guarantees that the PE interface gets linked.
 *
 * 2. Communicator wiring. PE-side collectives, such as the MPI_Barrier in
 *    synchronizeForces(), run over MPISettings::comm(). The CFD master never takes part in
 *    them (e.g. GetForcesFC2 is guarded by IF myid /= 0). If the PE communicator were left
 *    at its default, those collectives would span a communicator that includes the master
 *    and deadlock in the first FBM force synchronization (observed in q2p1_fc_ext).
 *
 * The function is idempotent. It publishes the engine singletons through the interface
 * globals \a world and \a mpisystem (defined in src/interface/cfd_stepping.cpp), which the
 * stepping and query functions of the interface rely on.
 *
 * Keep this function minimal: anything case-specific belongs in a dedicated setup.
 */
inline void setupGeneralInit(MPI_Comm ex0) {

  if (ex0 == MPI_COMM_NULL) {
    std::cerr << "setupGeneralInit: invalid (null) CFD worker communicator; "
                 "cannot wire the PE communicator.\n";
    MPI_Abort(MPI_COMM_WORLD, 1);
  }

  world     = pe::theWorld();
  mpisystem = pe::theMPISystem();
  mpisystem->setComm(ex0);
}
//*************************************************************************************************

}  // namespace pe
#endif  // HAVE_MPI

#include <pe/interface/setup/serial_common.h>

namespace pe {

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
 * @brief Generic benchmark setup for serial mode
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupBenchSerial(int cfd_rank) {
  setupGeneralInitSerial(cfd_rank);
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_GENERAL_INIT_H_
