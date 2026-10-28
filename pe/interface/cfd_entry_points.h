//=================================================================================================
/*!\file pe/interface/cfd_entry_points.h
 *  \brief The C entry points through which FeatFloWer starts and steps a coupled PE simulation.
 *
 *  Every function here is bound from Fortran (applications/<app>/app_init.f90 calls one
 *  commf2c_<case>_, the FBM layer calls the step_* functions). The names and signatures are the
 *  ABI contract with the CFD side: a pe-internal reorganisation may move the implementations
 *  but must keep these symbols.
 *
 *  Two coupling modes, selected at build time:
 *   - parallel PE mode (HAVE_MPI): the Fortran communicators are converted to MPI_Comm and the
 *     setupXxx(MPI_Comm) of the case runs on the CFD worker ranks;
 *   - serial PE mode (PE_SERIAL_MODE, pe built without MPI): every CFD rank runs its own PE
 *     instance and setupXxxSerial(cfd_rank) creates all bodies on each of them. The
 *     communicator arguments are accepted and ignored.
 *
 *  Implementations: src/interface/cfd_entry_points.cpp (commf2c_*) and
 *  src/interface/cfd_stepping.cpp (step_*). Case setups: pe/interface/setup/<case>.h.
 *  Background: doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_CFD_ENTRY_POINTS_H_
#define _PE_INTERFACE_CFD_ENTRY_POINTS_H_

#if HAVE_MPI

#include <mpi.h>

//=================================================================================================
//  Parallel PE mode: setup entry points
//=================================================================================================
extern "C" void commf2c_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_fluidization_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_dcav_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_cyl_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_init_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_fsi_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_kroupa_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_creep_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_dns_drag_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_el_frozen_trace_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank,
                                         double *xmin, double *xmax,
                                         double *ymin, double *ymax,
                                         double *zmin, double *zmax);
extern "C" void commf2c_el_terminal_velocity_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank,
                                              double *xmin, double *xmax,
                                              double *ymin, double *ymax,
                                              double *zmin, double *zmax);
extern "C" void commf2c_archimedes_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_dkt_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_lubrication_lab_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_drill_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_atc_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);
extern "C" void commf2c_rotation_(MPI_Fint *Fcomm, MPI_Fint *FcommEx0, int *remoteRank);

#else

//=================================================================================================
//  Serial PE mode: setup entry points (communicator arguments unused)
//=================================================================================================
extern "C" void commf2c_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_fluidization_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_fluidization_srr_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_dcav_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_cyl_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_init_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_fsi_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_kroupa_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_creep_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_dns_drag_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_el_frozen_trace_(int *Fcomm, int *FcommEx0, int *remoteRank,
                                         double *xmin, double *xmax,
                                         double *ymin, double *ymax,
                                         double *zmin, double *zmax);
extern "C" void commf2c_archimedes_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_dkt_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_lubrication_lab_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_drill_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_atc_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_span_complex_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_rotation_(int *Fcomm, int *FcommEx0, int *remoteRank);
extern "C" void commf2c_hashgrid_test_(int *Fcomm, int *FcommEx0, int *remoteRank);

#endif  // HAVE_MPI

//=================================================================================================
//  Stepping entry points (both modes)
//=================================================================================================
//! Advances the PE simulation by one CFD time step.
extern "C" void step_simulation_();

//! Re-files every body into the HashGrid cell of its CURRENT AABB. Call after motion and before
//! any point classification (see the definition in src/interface/cfd_stepping.cpp).
extern "C" void refresh_spatial_index_();

//! Advances the frozen-field Euler-Lagrange tracer simulation by one CFD time step.
extern "C" void step_el_frozen_trace_();

#endif  // _PE_INTERFACE_CFD_ENTRY_POINTS_H_
