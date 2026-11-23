//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_mpi_test.cpp
 *  \brief pe::coupling stage-1 semantics in an MPI build: one process, and two processes
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
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *
 *  One process (mpiexec -n 1): the MPI build is fully functional: a plane created through the
 *  API (pe needs a global section once MPI is initialized) with a sphere coming to rest on it,
 *  loads, a checkpoint round trip, advance().
 *
 *  Two processes, subdomains split at x = 0:
 *    B  two spheres owned by different ranks touch across the cut and approach each other.
 *       The creation leaves pe's sync-required flag set; advance() must synchronize before the
 *       first pe step (HardContactAndFluid resolves contacts before it synchronizes), so the
 *       first step already sees the neighbour's shadow copy and stops the approach. Without
 *       that synchronization both spheres would keep their approach velocity in step 1. The
 *       velocities are set natively, so only pe's own sync-required flag triggers it.
 *       Run twice with contact relaxation 0.9 and 0.3: the response differs, which is where
 *       pe's contact relaxation acts (velocity corrections exchanged between processes).
 *    C  refusals before any change ("not implemented yet"): addLoad(), applyLoads(),
 *       writeCheckpoint(), readCheckpoint() on more than one process; setKinematicState() on a
 *       global body, on the owner of a shadowed body, on a shadow copy, and to a position
 *       outside the local subdomain (the body would migrate); an override inside the subdomain
 *       works; overriding an API plane (global) is "not implemented yet"; the API plane has the
 *       same id on every rank; advance() inside a pe exclusive section is refused locally
 *       (std::logic_error) and both ranks continue. Reading stays available: enumeration
 *       (shadow copies reported), state, classification of the shadow copy.
 *    H  an override recorded by one Engine survives the replacement of the handle: the new
 *       Engine synchronizes before its first step.
 *  The verdict is reduced over all ranks.
 */
//=================================================================================================

#include <mpi.h>
#include <pe/core.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/interface/coupling/Engine.h>

#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
using coupling_test::sameBits;
namespace pc = pe::coupling;

static const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };


//*************************************************************************************************
/*!\brief Sets the velocity of a body through the API (position and orientation unchanged).
 */
static void setVelocity( pc::Engine& engine, pc::BodyId id, const pc::Vec3d& v )
{
   const pc::BodyState s( engine.state( id ) );
   engine.setKinematicState( id, pc::KinematicState{ s.position, s.orientation, v, zero } );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief One process: everything works.
 */
static void singleProcess()
{
   pc::Engine engine;
   const pc::MaterialId mat( engine.createMaterial( "mpiSingleMat", 1500.0, 0.0, 0.1, 0.1 ) );

   // A plane through the API (pe requires a global section once MPI is initialized, also on
   // one process) and a sphere that falls onto it and comes to rest
   engine.setGravity( pc::Vec3d{ { 0.0, 0.0, -9.81 } } );
   const pc::BodyId floor( engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, mat ) );
   expect( engine.state( floor ).shape == pc::ShapeKind::plane, "1 process: API plane created" );
   const pc::BodyId ball( engine.createSphere( pc::Vec3d{ { 5.0, 0.0, 0.6 } }, 0.5, mat ) );
   for( int i=0; i<800; ++i ) engine.advance( 1.0e-3 );
   const pc::BodyState rest( engine.state( ball ) );
   std::printf( "1 process: sphere on the plane at z = %.6g, v_z = %.3g\n", rest.position[2],
                rest.linearVelocity[2] );
   expect( std::fabs( rest.position[2] - 0.5 ) < 1.0e-3 &&
           std::fabs( rest.linearVelocity[2] ) < 1e-6,
           "1 process: the sphere rests on the API plane" );
   engine.setGravity( zero );

   const pc::BodyId s( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 3.0 } }, 0.5, mat ) );
   engine.addLoad( s, pc::Vec3d{ { 1.0, 0.0, 0.0 } }, zero );
   engine.applyLoads( 0.5, 1.0e-3 );
   expect( engine.state( s ).linearVelocity[0] > 0.0, "1 process: load applied" );
   engine.advance( 1.0e-3 );
   if( pc::buildInfo().checkpointsSupported ) {
      std::stringstream ckpt;
      engine.writeCheckpoint( ckpt );
      const pc::BodyState before( engine.state( s ) );
      engine.readCheckpoint( ckpt );
      expect( sameBits( engine.state( s ).position, before.position ), "1 process: checkpoint" );
   }
   setVelocity( engine, s, pc::Vec3d{ { 0.0, 2.0, 0.0 } } );
   engine.advance( 1.0e-3 );
   expect( engine.state( s ).linearVelocity[1] == 2.0, "1 process: override and advance" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Two processes, stage B: the first step sees the neighbour. Returns the approach speed
 *        of the local sphere after the first step.
 */
static double firstStepAcrossCut( pc::Engine& engine, int rank, double relaxation )
{
   pe::theWorld()->clear();
   engine.advance( 1.0e-9 );   // consumes the new Engine's own first synchronization
   engine.setContactRelaxation( relaxation );
   const std::string name( "mpiCut" + std::to_string( int( relaxation * 10 ) ) );
   const pc::MaterialId mat( engine.createMaterial( name, 1500.0, 0.0, 0.1, 0.1 ) );
   const double x( rank == 0 ? -0.4999995 : 0.4999995 );   // touching across the cut
   const pc::BodyId mine( engine.createSphere( pc::Vec3d{ { x, 0.0, 0.0 } }, 0.5, mat ) );
   // The velocity is set natively, not through setKinematicState(): only pe's own
   // sync-required flag (set by the creation) may trigger the synchronization here
   ( *pe::theCollisionSystem()->getBodyStorage().find( mine ) )->setLinearVel(
      pe::Vec3( rank == 0 ? 1.0 : -1.0, 0.0, 0.0 ) );
   engine.advance( 1.0e-3 );
   const double v( engine.state( mine ).linearVelocity[0] );
   return rank == 0 ? v : -v;
}
//*************************************************************************************************


static void twoProcesses( int rank )
{
   pc::Engine engine;
   engine.setGravity( zero );
   if( rank == 0 ) {
      pe::defineLocalDomain( pe::HalfSpace( pe::Vec3( -1, 0, 0 ), 0 ) );
      pe::connect( 1, pe::HalfSpace( pe::Vec3( +1, 0, 0 ), 0 ) );
   }
   else {
      pe::defineLocalDomain( pe::HalfSpace( pe::Vec3( +1, 0, 0 ), 0 ) );
      pe::connect( 0, pe::HalfSpace( pe::Vec3( -1, 0, 0 ), 0 ) );
   }

   // B: synchronization before the first step; the relaxation acts across the cut
   const double approach9( firstStepAcrossCut( engine, rank, 0.9 ) );
   const double approach3( firstStepAcrossCut( engine, rank, 0.3 ) );
   std::printf( "[rank %d] approach speed after the first step: %.10g (relaxation 0.9), %.10g "
                "(0.3); 1 without the contact\n", rank, approach9, approach3 );
   expect( std::fabs( approach9 ) < 0.5,
           "B: the first step already resolves the contact across the cut" );
   expect( approach9 != approach3,
           "B: the contact relaxation changes the response across the cut" );

   // C: refusals and what stays available
   pe::theWorld()->clear();
   engine.setContactRelaxation( 0.9 );
   const pc::MaterialId mat( engine.createMaterial( "mpiCouplingMat", 1500.0, 0.0, 0.1, 0.1 ) );
   pc::BodyId straddling( pc::noBody ), farLocal( pc::noBody ), global( pc::noBody );
   if( rank == 0 ) {
      straddling = engine.createSphere( pc::Vec3d{ { -0.2, 0.0, 0.0 } }, 0.5, mat, 1 );
      farLocal   = engine.createSphere( pc::Vec3d{ { -3.0, 0.0, 0.0 } }, 0.5, mat, 2 );
   }
   else {
      farLocal   = engine.createSphere( pc::Vec3d{ { 3.0, 0.0, 0.0 } }, 0.5, mat, 3 );
   }
   pe_GLOBAL_SECTION {
      pe::SphereID g( pe::createSphere( 9, pe::Vec3( 0.0, 10.0, 0.0 ), 0.5,
                                        static_cast<pe::MaterialID>( mat ) ) );
      g->setFixed( true );
      global = g->getSystemID();
   }
   MPI_Bcast( &straddling, 1, MPI_UINT64_T, 0, MPI_COMM_WORLD );
   engine.advance( 1.0e-6 );   // synchronizes: rank 1 receives the shadow copy
   engine.refreshSpatialIndex();

   std::vector<pc::BodyId> movable, shadows;
   engine.bodyIds( pc::Selection::movable, movable );
   engine.bodyIds( pc::Selection::shadowCopies, shadows );

   const char* const ni( "not implemented yet" );
   const pc::BodyState far( engine.state( farLocal ) );
   expectThrows<std::runtime_error>( [&]() { engine.addLoad( farLocal, zero, zero ); },
                                     "C: addLoad on 2 processes", ni );
   expect( !engine.hasPendingLoads(), "C: the refused addLoad left nothing pending" );
   expectThrows<std::runtime_error>( [&]() { engine.applyLoads( 1.0, 1.0e-3 ); },
                                     "C: applyLoads on 2 processes", ni );
   std::stringstream ckpt;
   expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                     "C: writeCheckpoint on 2 processes", ni );
   std::stringstream fake( "pe-coupling-checkpoint 3\nbodies 0\nfixed 0\nend\n" );
   expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( fake ); },
                                     "C: readCheckpoint on 2 processes", ni );
   const pc::BodyState g( engine.state( global ) );
   expectThrows<std::runtime_error>( [&]() {
      engine.setKinematicState( global, pc::KinematicState{ g.position, g.orientation, zero,
                                                            zero } ); },
      "C: override of a global body", "global" );
   expectThrows<std::runtime_error>( [&]() {
      engine.setKinematicState( farLocal, pc::KinematicState{
         pc::Vec3d{ { -far.position[0], 0.0, 0.0 } }, far.orientation, zero, zero } ); },
      "C: override that would migrate the body", "migrate" );
   expect( sameBits( engine.state( farLocal ).position, far.position ),
           "C: the refused overrides changed nothing" );
   const pc::Vec3d inside{ { far.position[0] * 1.5, 0.0, 0.0 } };
   engine.setKinematicState( farLocal, pc::KinematicState{ inside, far.orientation, zero, zero } );
   expect( sameBits( engine.state( farLocal ).position, inside ), "C: an interior override works" );

   const pc::Vec3d insideShadowSide{ { 0.2, 0.0, 0.0 } };   // x > 0, inside the straddler
   if( rank == 0 ) {
      expect( movable.size() == 2 && shadows.empty(), "C rank 0: two local bodies, no shadows" );
      expect( !engine.state( straddling ).shadow, "C rank 0: the straddler is local" );
      const pc::BodyState st( engine.state( straddling ) );
      expectThrows<std::runtime_error>( [&]() {
         engine.setKinematicState( straddling, pc::KinematicState{ st.position, st.orientation,
                                                                   zero, zero } ); },
         "C rank 0: override of a body with shadow copies", "shadow copies" );
   }
   else {
      expect( movable.size() == 1 && movable[0] == farLocal, "C rank 1: one local movable body" );
      expect( shadows.size() == 1 && shadows[0] == straddling, "C rank 1: the shadow copy" );
      std::vector<pc::BodyState> states;
      engine.states( pc::Selection::shadowCopies, states );
      expect( states.size() == 1 && states[0].shadow && states[0].radius == 0.5,
              "C rank 1: shadow state" );
      expect( engine.hasBody( straddling ) && engine.state( straddling ).shadow,
              "C rank 1: state() reports the shadow" );
      for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive,
                                       pc::QueryMethod::spatialIndex } )
         expect( engine.owner( insideShadowSide, m ) == straddling, "C rank 1: shadow classified" );
      expect( engine.contains( straddling, insideShadowSide ), "C rank 1: contains(shadow)" );
      const pc::BodyState st( engine.state( straddling ) );
      expectThrows<std::runtime_error>( [&]() {
         engine.setKinematicState( straddling, pc::KinematicState{ st.position, st.orientation,
                                                                   zero, zero } ); },
         "C rank 1: override of a shadow copy", ni );
   }

   // A plane through the API on two processes (every rank makes the same call): a global
   // body with the same id everywhere; overriding it is "not implemented yet" (global), not
   // "infinite"
   const pc::BodyId plane( engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, -10.0, mat ) );
   std::uint64_t planeMin( plane ), planeMax( plane );
   MPI_Allreduce( MPI_IN_PLACE, &planeMin, 1, MPI_UINT64_T, MPI_MIN, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, &planeMax, 1, MPI_UINT64_T, MPI_MAX, MPI_COMM_WORLD );
   expect( planeMin == planeMax, "C: the API plane has the same id on every rank" );
   const pc::BodyState ps( engine.state( plane ) );
   expectThrows<std::runtime_error>( [&]() {
      engine.setKinematicState( plane, pc::KinematicState{ ps.position, ps.orientation, zero,
                                                           zero } ); },
      "C: override of a global plane", "global" );

   // advance() inside an exclusive section is refused locally, before any collective; both
   // ranks then continue together
   pe_EXCLUSIVE_SECTION( 0 ) {
      expectThrows<std::logic_error>( [&]() { engine.advance( 1.0e-3 ); },
                                      "C rank 0: advance() inside an exclusive section",
                                      "exclusive section" );
   }
   engine.advance( 1.0e-3 );
   std::vector<pc::BodyId> after;
   engine.bodyIds( pc::Selection::movable, after );
   expect( after.size() == movable.size(), "C: advance() keeps the local bodies" );
}


//*************************************************************************************************
/*!\brief Two processes: a state override recorded by one Engine is not lost when the handle is
 *        replaced before the next step.
 *
 * Rank 0 moves its (already synchronized, unshadowed) sphere through setKinematicState() so it
 * overlaps the cut and touches rank 1's sphere, both approaching; pe itself does not register
 * that a synchronization is needed. The Engine is then destroyed and a new one created; its
 * first step must still synchronize, so the contact is resolved in that step.
 */
static void handleReplacement( int rank )
{
   pe::theWorld()->clear();
   pc::BodyId mine( pc::noBody );
   {
      pc::Engine engine;
      const pc::MaterialId mat( engine.createMaterial( "mpiHandleMat", 1500.0, 0.0, 0.1, 0.1 ) );
      mine = engine.createSphere( pc::Vec3d{ { rank == 0 ? -3.0 : 0.5005, 0.0, 0.0 } }, 0.5, mat );
      engine.advance( 1.0e-9 );   // creation synchronized; no shadow copies yet
      expect( engine.count( pc::Selection::shadowCopies ) == 0, "handle: no shadow copies yet" );
      if( rank == 0 ) {
         const pc::BodyState s( engine.state( mine ) );
         engine.setKinematicState( mine, pc::KinematicState{ pc::Vec3d{ { -0.4985, 0.0, 0.0 } },
                                                             s.orientation,
                                                             pc::Vec3d{ { 1.0, 0.0, 0.0 } },
                                                             zero } );
      }
      else {
         ( *pe::theCollisionSystem()->getBodyStorage().find( mine ) )->setLinearVel(
            pe::Vec3( -1.0, 0.0, 0.0 ) );
      }
   }   // the handle is replaced before the next step
   pc::Engine engine;
   engine.advance( 1.0e-3 );
   const double v( engine.state( mine ).linearVelocity[0] );
   std::printf( "[rank %d] after the handle replacement: v_x = %.6g (+-1 without the contact)\n",
                rank, v );
   expect( std::fabs( v ) < 0.5, "handle: the new Engine synchronized before its first step" );
}
//*************************************************************************************************


int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int rank( 0 ), size( 1 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   expect( pc::buildInfo().mpi, "buildInfo().mpi" );

   if( size == 1 )
      singleProcess();
   else if( size == 2 ) {
      twoProcesses( rank );
      handleReplacement( rank );
   }
   else
      expect( false, "pe_coupling_mpi_test runs on 1 or 2 ranks" );

   int local( coupling_test::failures() ), total( 0 );
   MPI_Allreduce( &local, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 )
      std::printf( "pe_coupling_mpi_test (%d ranks): %s (%d failed checks over all ranks)\n",
                   size, total ? "FAILED" : "PASSED", total );
   MPI_Finalize();
   return total ? 1 : 0;
}
