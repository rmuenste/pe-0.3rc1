#ifndef _PE_SETUP_ATC_H_
#define _PE_SETUP_ATC_H_

#include <iostream>

#include <pe/core/MPI.h>

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
