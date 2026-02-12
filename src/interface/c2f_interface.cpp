#include <pe/interface/c2f_interface.h>
#include <pe/interface/object_queries.h>

// Interface headers are always needed (both parallel and serial PE modes)
#include <pe/interface/c_interface_queries.h>
#include <pe/interface/c_interface_particle_fbm.h>
#include <pe/interface/c_interface_particle_getset.h>

#if HAVE_MPI
#include <pe/interface/sim_setup.h>

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

    if (rank == 1) {
      printf( "%d> C) Configuration Sed bench with %d processes.\n", remRank, size );
    }
    if (remRank != 0) {
      CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
      //printf( "%d> C) Hello world from process %d of %d\n", remRank, rank, size );
      setupParticleBench(CcommEx0);
    } 
}
//=================================================================================================

//=================================================================================================
extern "C" void commf2c_fluidization_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
    MPI_Comm Ccomm;
    MPI_Comm CcommEx0;
    Ccomm = MPI_Comm_f2c(*Fcomm);

    int remRank = *remoteRank;
    int rank, size;
    MPI_Comm_rank(Ccomm, &rank);
    MPI_Comm_size(Ccomm, &size);

    if (rank == 1) {
      printf("%d> C) Configuration Fluidization bench with %d processes.\n", remRank, size);
    }
    if (remRank != 0) {
      CcommEx0 = MPI_Comm_f2c(*FcommEx0);
      setupFluidization(CcommEx0);
    }
}
//=================================================================================================


//=================================================================================================
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
//=================================================================================================


//=================================================================================================
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
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_init_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> Initialized library pe for %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    //setupBench(CcommEx0);
  } 
}
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_fsi_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> C) Configuration FSI bench with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupSpan(CcommEx0);
  } 
}
//=================================================================================================


//=================================================================================================
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
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_creep_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> C) Configuration Creep with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupCreep(CcommEx0);
  } 
}
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_archimedes_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> C) Configuration Creep with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupArchimedes(CcommEx0);
  } 
}
//=================================================================================================


//=================================================================================================
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
//=================================================================================================


//=================================================================================================
// NOTE: setupLubricationLab() and setupDrill() parallel implementations are WIP
// Only serial mode (PE_SERIAL_MODE) is currently supported for these simulations
// These stubs prevent linker errors and provide clear runtime error messages
//=================================================================================================
extern "C" void commf2c_lubrication_lab_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  fprintf(stderr, "\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "ERROR: commf2c_lubrication_lab_() is not implemented in parallel PE mode\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "The Lubrication Lab simulation is currently only supported in PE serial mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "To use this simulation, rebuild with PE_SERIAL_MODE enabled:\n");
  fprintf(stderr, "  1. cd build\n");
  fprintf(stderr, "  2. cmake -DUSE_PE=ON ..\n");
  fprintf(stderr, "  3. cmake -DUSE_PE_SERIAL_MODE=ON ..\n");
  fprintf(stderr, "  4. make -j8\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "PE Serial Mode is optimized for large particles that span multiple domains.\n");
  fprintf(stderr, "========================================================================\n");
  exit(1);
}
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_drill_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  fprintf(stderr, "\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "ERROR: commf2c_drill_() is not implemented in parallel PE mode\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "The Drill simulation is currently only supported in PE serial mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "To use this simulation, rebuild with PE_SERIAL_MODE enabled:\n");
  fprintf(stderr, "  1. cd build\n");
  fprintf(stderr, "  2. cmake -DUSE_PE=ON ..\n");
  fprintf(stderr, "  3. cmake -DUSE_PE_SERIAL_MODE=ON ..\n");
  fprintf(stderr, "  4. make -j8\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "PE Serial Mode is optimized for large particles that span multiple domains.\n");
  fprintf(stderr, "========================================================================\n");
  exit(1);
}
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_atc_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  int remRank = *remoteRank;

  if(remRank != 0) {
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0);
    MPI_Comm_rank (CcommEx0, &rank);
    MPI_Comm_size (CcommEx0, &size);

    if (rank == 1) {
      printf( "%d> C) Configuration ATC with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    setupATC(CcommEx0);
  }
}
//=================================================================================================


//=================================================================================================
extern "C" void commf2c_rotation_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  fprintf(stderr, "\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "ERROR: commf2c_rotation_() is not implemented in parallel PE mode\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "The Rotation simulation is currently only supported in PE serial mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "To use this simulation, rebuild with PE_SERIAL_MODE enabled:\n");
  fprintf(stderr, "  1. cd build\n");
  fprintf(stderr, "  2. cmake -DUSE_PE=ON ..\n");
  fprintf(stderr, "  3. cmake -DUSE_PE_SERIAL_MODE=ON ..\n");
  fprintf(stderr, "  4. make -j8\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "PE Serial Mode is optimized for large particles that span multiple domains.\n");
  fprintf(stderr, "========================================================================\n");
  exit(1);
}
//=================================================================================================


#endif

#ifdef PE_SERIAL_MODE
#include <pe/interface/sim_setup_serial.h>

//=================================================================================================
// Serial PE Mode: Setup function implementations
//=================================================================================================
// In serial PE mode, each CFD domain runs an independent serial PE instance.
// These functions initialize the PE world for the current domain without MPI.
//=================================================================================================

extern "C" void commf2c_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Initialize PE world for this domain
  // Pass CFD rank for unique log filenames (pe<rank>.log)
  pe::setupParticleBenchSerial(*remoteRank);
}

extern "C" void commf2c_fluidization_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Fluidization setup
  pe::setupFluidizationSerial(*remoteRank);
}

extern "C" void commf2c_dcav_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: DCAV setup
  pe::setupDCAVSerial(*remoteRank);
}

extern "C" void commf2c_cyl_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Cylinder setup
  pe::setupCylSerial(*remoteRank);
}

extern "C" void commf2c_init_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Generic initialization
  pe::setupGeneralInitSerial(*remoteRank);
}

extern "C" void commf2c_fsi_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: FSI benchmark setup
  pe::setupFSIBenchSerial(*remoteRank);
}

extern "C" void commf2c_kroupa_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Kroupa setup
  pe::setupKroupaSerial(*remoteRank);
}

extern "C" void commf2c_creep_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Creep flow setup
  pe::setupCreepSerial(*remoteRank);
}

extern "C" void commf2c_archimedes_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Archimedes buoyancy setup
  pe::setupArchimedesSerial(*remoteRank);
}

extern "C" void commf2c_dkt_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Draft-Kiss-Tumble setup
  pe::setupDraftKissTumbSerial(*remoteRank);
}

extern "C" void commf2c_lubrication_lab_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Lubrication Lab setup
  pe::setupLubricationLabSerial(*remoteRank);
}

extern "C" void commf2c_drill_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Drill setup
  pe::setupDrillSerial(*remoteRank);
}

extern "C" void commf2c_atc_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: ATC setup
  pe::setupATCSerial(*remoteRank);
}

extern "C" void commf2c_rotation_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Rotation setup
  pe::setupRotationSerial(*remoteRank);
}

extern "C" void commf2c_hashgrid_test_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: HashGrid test setup
  pe::setupHashGridTest(*remoteRank);
}

#endif  // PE_SERIAL_MODE
