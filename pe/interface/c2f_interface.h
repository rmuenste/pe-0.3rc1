#include <iostream>
#include <stdio.h>
#if HAVE_MPI
#include <mpi.h>

extern "C" void commf2c_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_fluidization_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
#endif
