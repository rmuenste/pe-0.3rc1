//=================================================================================================
/*!
 *  \file pe/interface/setup_atc.h
 *  \brief PE setup for the ATC (Application Test Case) case (parallel PE mode)
 *
 *  Copyright (C) 2025 Raphael Münster
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

#ifndef _PE_INTERFACE_SETUP_ATC_H_
#define _PE_INTERFACE_SETUP_ATC_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>

#include <pe/core/MPI.h>




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief ATC (Application Test Case) setup for parallel PE mode -- NOT FUNCTIONAL YET.
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_atc_(). The parallel ATC setup is planned but not
 * implemented. Instead of returning silently with an empty world (which would let the CFD
 * run continue without any particles), this placeholder aborts the run with a clear message.
 *
 * The serial variant (setupATCSerial in sim_setup_serial.h, PE_SERIAL_MODE) is functional.
 *
 * When implementing, the usual steps are: load the configuration, set the world and fluid
 * properties, wire the communicator and decompose the domain, create the bodies,
 * synchronize, and activate the output writers.
 */
void setupATC(MPI_Comm ex0) {

  int rank = 0;
  if (ex0 != MPI_COMM_NULL) {
    MPI_Comm_rank(ex0, &rank);
  }

  if (rank == 0) {
    std::cerr << "\n"
              << "========================================================================\n"
              << "ERROR: setupATC() is not functional at the moment\n"
              << "========================================================================\n"
              << "The ATC setup for parallel PE mode is not implemented yet.\n"
              << "Use the PE serial mode (PE_SERIAL_MODE) for the ATC case for now.\n"
              << "========================================================================\n"
              << std::endl;
  }

  MPI_Abort(ex0 != MPI_COMM_NULL ? ex0 : MPI_COMM_WORLD, 1);
}
//*************************************************************************************************

#endif
