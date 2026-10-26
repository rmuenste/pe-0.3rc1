//=================================================================================================
/*!
 *  \file pe/interface/setup_config.h
 *  \brief Loading of the configuration file in the parallel interface setups
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_SETUP_CONFIG_H_
#define _PE_INTERFACE_SETUP_CONFIG_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <exception>
#include <iostream>
#include <string>
#include <pe/config/SimulationConfig.h>
#include <pe/core/MPI.h>




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Loads the configuration file on every process of \a comm, or aborts the run.
 *
 * \param comm The communicator of the setup (all its processes have to call this function).
 * \param setupName Name of the calling setup, used in the error message.
 * \return void
 *
 * Every process loads SimulationConfig::getConfigFile() itself. Rank 0 prints the name of the
 * file and where the name came from. If any process fails (missing or unreadable file, invalid
 * json, a deprecation that is an error), the run is aborted with MPI_Abort(). The error is
 * printed by rank 0 if it failed itself, otherwise by every process that failed, together with
 * its rank: the processes may run in different working directories, so a failure on one process
 * says nothing about the others.
 */
inline void loadConfigFileOnAllRanks( MPI_Comm comm, const std::string& setupName )
{
   int rank( 0 );
   MPI_Comm_rank( comm, &rank );

   std::string error;
   try {
      pe::SimulationConfig::loadConfigFile( rank == 0 );
   }
   catch( const std::exception& e ) {
      error = e.what();
   }

   const int localFailed( error.empty() ? 0 : 1 );
   int anyFailed( 0 );
   MPI_Allreduce( &localFailed, &anyFailed, 1, MPI_INT, MPI_MAX, comm );
   if( !anyFailed )
      return;

   int rootFailed( rank == 0 ? localFailed : 0 );
   MPI_Bcast( &rootFailed, 1, MPI_INT, 0, comm );

   if( localFailed && ( rank == 0 || !rootFailed ) ) {
      std::cerr << "\nERROR in " << setupName << " (rank " << rank << "): " << error << "\n"
                << std::endl;
   }

   // Let every failing process print before the run is torn down
   MPI_Barrier( comm );
   MPI_Abort( comm, 1 );
}
//*************************************************************************************************

#endif
