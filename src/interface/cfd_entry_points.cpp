//=================================================================================================
/*!\file src/interface/cfd_entry_points.cpp
 *  \brief The commf2c_* entry points FeatFloWer calls to set up a coupled PE simulation.
 *
 *  One block per case. Each block holds the parallel-mode implementation (HAVE_MPI: the
 *  Fortran communicators are converted and the setupXxx(MPI_Comm) of the case runs on the CFD
 *  worker ranks) and the serial-mode implementation (PE_SERIAL_MODE, no MPI in pe: every CFD
 *  rank runs setupXxxSerial(cfd_rank) on its own PE instance). The case setups live in
 *  pe/interface/setup/<case>.h; the prototypes are in pe/interface/cfd_entry_points.h.
 *
 *  The parallel-mode blocks apply the lubrication configuration centrally after the setup
 *  (D2.2): the serial setups apply it internally, but most parallel setups never did, so
 *  lubricationEnabled_ was a silent no-op in parallel mode. Applied post-setup (json is loaded
 *  by then); idempotent, and it refuses loudly on a solver without the lubrication stage.
 */
//=================================================================================================

#include <pe/interface/cfd_entry_points.h>
#include <pe/interface/support/optional_collision_params.h>
#include <pe/config/SimulationConfig.h>
#include <pe/core.h>

#include <cstdio>
#include <cstdlib>

#include <pe/interface/setup/part_bench.h>
#include <pe/interface/setup/fluidization.h>
#include <pe/interface/setup/dcav.h>
#include <pe/interface/setup/cyl.h>
#include <pe/interface/setup/general_init.h>
#include <pe/interface/setup/span.h>
#include <pe/interface/setup/kroupa.h>
#include <pe/interface/setup/creep.h>
#include <pe/interface/setup/dns_drag.h>
#include <pe/interface/setup/el_frozen_trace.h>
#include <pe/interface/setup/el_terminal_velocity.h>
#include <pe/interface/setup/archimedes.h>
#include <pe/interface/setup/dkt.h>
#include <pe/interface/setup/lubrication_lab.h>
#include <pe/interface/setup/drill.h>
#include <pe/interface/setup/atc.h>
#include <pe/interface/setup/rotation.h>
#include <pe/interface/setup/span_complex.h>
#include <pe/interface/setup/hashgrid_test.h>


//=================================================================================================
//  Single-particle sedimentation benchmark  (pe/interface/setup/part_bench.h)
//=================================================================================================
#if HAVE_MPI
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
      pe::setupParticleBench(CcommEx0);
      // Central lubrication-config application (D2.2): the serial setups apply this
      // internally, but most parallel setups never did, so lubricationEnabled_ was a
      // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
      // idempotent, and refuses loudly on a solver without the lubrication stage.
      pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
    } 
}
//=================================================================================================
#else
extern "C" void commf2c_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Initialize PE world for this domain
  // Pass CFD rank for unique log filenames (pe<rank>.log)
  pe::setupParticleBenchSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Fluidization column  (pe/interface/setup/fluidization.h)
//=================================================================================================
#if HAVE_MPI
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
      pe::setupFluidization(CcommEx0);
      // Central lubrication-config application (D2.2): the serial setups apply this
      // internally, but most parallel setups never did, so lubricationEnabled_ was a
      // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
      // idempotent, and refuses loudly on a solver without the lubrication stage.
      pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
    }
}
//=================================================================================================
#else
extern "C" void commf2c_fluidization_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Fluidization setup
  pe::setupFluidizationSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Fluidization column, short-range repulsion (serial mode only)  (pe/interface/setup/fluidization.h)
//=================================================================================================
#if !HAVE_MPI
extern "C" void commf2c_fluidization_srr_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Fluidization with Short-Range Repulsion solver
  pe::setupFluidizationSRRSerial(*remoteRank);
}
#endif  // !HAVE_MPI (serial mode only)

//=================================================================================================
//  DCAV  (pe/interface/setup/dcav.h)
//=================================================================================================
#if HAVE_MPI
//=================================================================================================
extern "C" void commf2c_dcav_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  // The parallel setup2x2x2() was removed: it performed no domain decomposition and
  // created no particles. Only the serial mode (setupDCAVSerial) is supported.
  fprintf(stderr, "\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "ERROR: commf2c_dcav_() is not implemented in parallel PE mode\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "The DCAV simulation is currently only supported in PE serial mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "To use this simulation, rebuild with PE_SERIAL_MODE enabled:\n");
  fprintf(stderr, "  1. cd build\n");
  fprintf(stderr, "  2. cmake -DUSE_PE=ON ..\n");
  fprintf(stderr, "  3. cmake -DUSE_PE_SERIAL_MODE=ON ..\n");
  fprintf(stderr, "  4. make -j8\n");
  fprintf(stderr, "========================================================================\n");
  exit(1);
}
//=================================================================================================
#else
extern "C" void commf2c_dcav_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: DCAV setup
  pe::setupDCAVSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Cylinder  (pe/interface/setup/cyl.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupCyl(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  } 
}
//=================================================================================================
#else
extern "C" void commf2c_cyl_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Cylinder setup
  pe::setupCylSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Minimal bootstrap  (pe/interface/setup/general_init.h)
//=================================================================================================
#if HAVE_MPI
//=================================================================================================
extern "C" void commf2c_init_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{   
  int remRank = *remoteRank;

  if(remRank != 0) { 
    int rank, size;

    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0); // Convert Fortran->C communicator
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", remRank);
      return;
    }
    MPI_Comm_rank (CcommEx0, &rank);	/* get current process id */
    MPI_Comm_size (CcommEx0, &size);	/* get number of processes */

    if (rank == 1) {
      printf( "%d> Initialized library pe for %d processes.\n", remRank, size );
    }
    // Minimal bootstrap: wires the PE communicator to the CFD worker comm
    // (Ex0) so passive PE use (queries, synchronizeForces) is safe; creates
    // no world content.
    pe::setupGeneralInit(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  }
}
//=================================================================================================
#else
extern "C" void commf2c_init_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Generic initialization
  pe::setupGeneralInitSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Span (chip) FSI benchmark  (pe/interface/setup/span.h)
//=================================================================================================
#if HAVE_MPI
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
      printf( "%d> C) Configuration span (chip) case with %d processes.\n", remRank, size );
    }
    if( CcommEx0 == MPI_COMM_NULL ) {
      printf( "%d> C)Error converting fortran communicator\n", rank);
       return;
    }
    pe::setupSpan(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  } 
}
//=================================================================================================
#else
extern "C" void commf2c_fsi_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: FSI benchmark setup
  pe::setupFSIBenchSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Kroupa lubrication column  (pe/interface/setup/kroupa.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupKroupa(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  } 
}
//=================================================================================================
#else
extern "C" void commf2c_kroupa_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Kroupa setup
  pe::setupKroupaSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Creep flow  (pe/interface/setup/creep.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupCreep(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  } 
}
//=================================================================================================
#else
extern "C" void commf2c_creep_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Creep flow setup
  pe::setupCreepSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  DNS drag  (pe/interface/setup/dns_drag.h)
//=================================================================================================
#if HAVE_MPI
//=================================================================================================
extern "C" void commf2c_dns_drag_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank)
{
  fprintf(stderr, "\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "ERROR: commf2c_dns_drag_() is not implemented in parallel PE mode\n");
  fprintf(stderr, "========================================================================\n");
  fprintf(stderr, "The DNS drag setup is currently only supported in PE serial mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "To use this simulation, rebuild with PE_SERIAL_MODE enabled:\n");
  fprintf(stderr, "  1. cd build\n");
  fprintf(stderr, "  2. cmake -DUSE_PE=ON ..\n");
  fprintf(stderr, "  3. cmake -DUSE_PE_SERIAL_MODE=ON ..\n");
  fprintf(stderr, "  4. make -j8\n");
  fprintf(stderr, "========================================================================\n");
  exit(1);
}
//=================================================================================================
#else
extern "C" void commf2c_dns_drag_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: DNS drag benchmark setup
  pe::setupDNSDragSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Frozen-field Euler-Lagrange trace  (pe/interface/setup/el_frozen_trace.h)
//=================================================================================================
#if HAVE_MPI
//=================================================================================================
extern "C" void commf2c_el_frozen_trace_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank,
                                         double *xmin, double *xmax,
                                         double *ymin, double *ymax,
                                         double *zmin, double *zmax)
{
  int remRank = *remoteRank;

  if (remRank != 0) {
    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0);
    int rank = 0;
    int size = 0;
    MPI_Comm_rank(CcommEx0, &rank);
    MPI_Comm_size(CcommEx0, &size);

    if (rank == 0) {
      printf("%d> C) Configuration frozen-field E-L trace with %d PE processes.\n", remRank, size);
    }
    if (CcommEx0 == MPI_COMM_NULL) {
      printf("%d> C) Error converting Fortran communicator for frozen-field E-L trace.\n", remRank);
      return;
    }

    pe::setupELFrozenTrace(CcommEx0, *xmin, *xmax, *ymin, *ymax, *zmin, *zmax);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  }
}
//=================================================================================================
#else
extern "C" void commf2c_el_frozen_trace_(int *Fcomm, int *FcommEx0, int *remoteRank,
                                         double *xmin, double *xmax,
                                         double *ymin, double *ymax,
                                         double *zmin, double *zmax) {
  // Serial PE mode: frozen-field general initialization only
  pe::setupELFrozenTraceSerial(*remoteRank, *xmin, *xmax, *ymin, *ymax, *zmin, *zmax);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Euler-Lagrange terminal velocity (parallel mode only)  (pe/interface/setup/el_terminal_velocity.h)
//=================================================================================================
#if HAVE_MPI
//=================================================================================================
extern "C" void commf2c_el_terminal_velocity_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank,
                                              double *xmin, double *xmax,
                                              double *ymin, double *ymax,
                                              double *zmin, double *zmax)
{
  int remRank = *remoteRank;

  if (remRank != 0) {
    MPI_Comm CcommEx0 = MPI_Comm_f2c(*FcommEx0);
    int rank = 0;
    int size = 0;
    MPI_Comm_rank(CcommEx0, &rank);
    MPI_Comm_size(CcommEx0, &size);

    if (rank == 0) {
      printf("%d> C) Configuration E-L terminal velocity with %d PE processes.\n", remRank, size);
    }
    if (CcommEx0 == MPI_COMM_NULL) {
      printf("%d> C) Error converting Fortran communicator for E-L terminal velocity.\n", remRank);
      return;
    }

    pe::setupELTerminalVelocity(CcommEx0, *xmin, *xmax, *ymin, *ymax, *zmin, *zmax);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  }
}
//=================================================================================================
#endif  // HAVE_MPI (no serial variant)

//=================================================================================================
//  Archimedes buoyancy  (pe/interface/setup/archimedes.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupArchimedes(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  } 
}
//=================================================================================================
#else
extern "C" void commf2c_archimedes_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Archimedes buoyancy setup
  pe::setupArchimedesSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Draft-kiss-tumble benchmark  (pe/interface/setup/dkt.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupDraftKissTumbBench(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  }
}
//=================================================================================================
#else
extern "C" void commf2c_dkt_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Draft-Kiss-Tumble setup
  pe::setupDraftKissTumbSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Lubrication lab  (pe/interface/setup/lubrication_lab.h)
//=================================================================================================
#if HAVE_MPI
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
#else
extern "C" void commf2c_lubrication_lab_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Lubrication Lab setup
  pe::setupLubricationLabSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Drill  (pe/interface/setup/drill.h)
//=================================================================================================
#if HAVE_MPI
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
#else
extern "C" void commf2c_drill_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Drill setup
  pe::setupDrillSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  ATC  (pe/interface/setup/atc.h)
//=================================================================================================
#if HAVE_MPI
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
    pe::setupATC(CcommEx0);
    // Central lubrication-config application (D2.2): the serial setups apply this
    // internally, but most parallel setups never did, so lubricationEnabled_ was a
    // silent no-op in parallel mode. Applied post-setup (json is loaded by then);
    // idempotent, and refuses loudly on a solver without the lubrication stage.
    pe::applyOptionalLubricationParams(*pe::theCollisionSystem(), pe::SimulationConfig::getInstance());
  }
}
//=================================================================================================
#else
extern "C" void commf2c_atc_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: ATC setup
  pe::setupATCSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Rotation  (pe/interface/setup/rotation.h)
//=================================================================================================
#if HAVE_MPI
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
#else
extern "C" void commf2c_rotation_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Rotation setup
  pe::setupRotationSerial(*remoteRank);
}
#endif  // HAVE_MPI

//=================================================================================================
//  Span complex (serial mode only)  (pe/interface/setup/span_complex.h)
//=================================================================================================
#if !HAVE_MPI
extern "C" void commf2c_span_complex_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: Span complex setup
  pe::setupSpanComplexSerial(*remoteRank);
}
#endif  // !HAVE_MPI (serial mode only)

//=================================================================================================
//  HashGrid test (serial mode only)  (pe/interface/setup/hashgrid_test.h)
//=================================================================================================
#if !HAVE_MPI
extern "C" void commf2c_hashgrid_test_(int *Fcomm, int *FcommEx0, int *remoteRank) {
  // Serial PE mode: HashGrid test setup
  pe::setupHashGridTest(*remoteRank);
}
#endif  // !HAVE_MPI (serial mode only)
