//=================================================================================================
/*!\file pe/interface/setup/dcav.h
 *  \brief DCAV and 2x2x2 domain bootstraps (serial mode only).
 *
 *  Entry point: commf2c_dcav_(); the parallel variant aborts with a message.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_DCAV_H_
#define _PE_INTERFACE_SETUP_DCAV_H_

// No parallel setup: the former setup2x2x2() decomposed nothing and created no particles.

#include <pe/interface/setup/serial_common.h>

namespace pe {

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

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_DCAV_H_
