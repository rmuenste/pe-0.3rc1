#include <pe/interface/c2f_interface.h>
#include <pe/interface/sim_setup.h>

#if HAVE_MPI
#include <pe/interface/object_queries.h>

#include <pe/interface/c_interface_queries.h>
#include <pe/interface/c_interface_particle_fbm.h>
#include <pe/interface/c_interface_particle_getset.h>



//=================================================================================================
/*
 *!\brief The function triggers the simulation setup and sets up the MPI communicators
 * \param Fcomm The MPI_COMM_WORLD communicator from the main fortran application
 * \param FcommEx0 A fortran MPI fortran MPI communicator that represents the subset of processes that should be used for the rigid body simulation
 */
extern "C" void commf2c_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
    MPI_Comm Ccomm;
    int status;
    MPI_Comm CcommEx0;
    Ccomm = MPI_Comm_f2c(*Fcomm); // Convert Fortran->C communicator

    int remRank = *remoteRank;
    int rank, size;
    MPI_Comm_rank (Ccomm, &rank);	/* get current process id */
    MPI_Comm_size (Ccomm, &size);	/* get number of processes */

    if (remRank != 0) {
      CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
      //printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
      setupParticleBench(CcommEx0);
    } 
}

extern "C" void commf2c_dcav_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  // force 4.0 multiplier take it out
  // force only in z considered
  // matching between fbmId and particleId
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setup2x2x2(CcommEx0);
  } 
}

extern "C" void commf2c_cyl_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  // force 4.0 multiplier take it out
  // force only in z considered
  // matching between fbmId and particleId
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupCyl(CcommEx0);
  } 
}

extern "C" void commf2c_init_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupSpan(CcommEx0);
  } 
}

extern "C" void commf2c_bench_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupBench(CcommEx0);
  } 
}

extern "C" void commf2c_kroupa_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> C) Configuration Kroupa with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupKroupa(CcommEx0);
  } 
}

extern "C" void commf2c_dkt_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  // force 4.0 multiplier take it out
  // force only in z considered
  // matching between fbmId and particleId
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupDraftKissTumbBench(CcommEx0);
  } 
}

#endif
