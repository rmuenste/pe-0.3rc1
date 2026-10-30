//=================================================================================================
/*!\file pe/interface/setup/parallel_common.h
 *  \brief Prelude of every parallel-mode (HAVE_MPI) case setup.
 *
 *  Pulls in the engine, domain decomposition, output and configuration headers a parallel
 *  setupXxx(MPI_Comm) is written against, and declares the interface globals the setups publish
 *  the engine singletons into. The globals are defined once in src/interface/cfd_stepping.cpp,
 *  whose stepSimulation() relies on them.
 *
 *  Empty outside HAVE_MPI builds.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_PARALLEL_COMMON_H_
#define _PE_INTERFACE_SETUP_PARALLEL_COMMON_H_

#if HAVE_MPI

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include <pe/engine.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/Merging.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>
#include <pe/config/SimulationConfig.h>
#include <pe/util/Checkpointer.h>
#include <pe/interface/support/el_optional_api.h>
#include <pe/interface/support/el_hydro_compat.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

//=================================================================================================
//  Interface globals (defined in src/interface/cfd_stepping.cpp)
//=================================================================================================
extern pe::WorldID     world;      //!< The simulation world, set by every parallel setup.
extern pe::MPISystemID mpisystem;  //!< The MPI system, wired to the CFD worker communicator.
extern bool            g_vtk;      //!< VTK output switch read from the configuration.
extern pe::PlaneID     g_ground;   //!< Ground plane handle of the setups that create one.
extern int             called;     //!< Legacy call counter of the drill draft.

//=================================================================================================
//  Parallel stepping (src/interface/cfd_stepping.cpp)
//=================================================================================================
void stepSimulation();
void stepELFrozenTrace();

namespace pe {

//! Backwards-compatible wrapper around SimulationConfig::loadFromFile().
void loadSimulationConfig(const std::string &fileName);

}  // namespace pe

#endif  // HAVE_MPI

#endif  // _PE_INTERFACE_SETUP_PARALLEL_COMMON_H_
