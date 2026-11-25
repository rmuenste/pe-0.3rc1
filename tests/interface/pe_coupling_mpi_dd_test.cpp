//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_mpi_dd_test.cpp
 *  \brief The coupling API on a domain-decomposed world (several processes, pe built with MPI)
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  Usage: pe_coupling_mpi_dd_test <reference file> [--write-reference] [--broken <variant>]
 *
 *  On one process with --write-reference: a sphere driven by fluid loads (relaxation 0.35, some
 *  load components exactly -0 and +0) moves along x; its full state is written as hexadecimal
 *  bits per step. On 2 or 4 processes (equal slabs along x, decomposeSlabs()): the same sphere,
 *  created by the owner of its centre, crosses every slab boundary; each step the owner restores
 *  the body's coupling history (setLoadHistory(), from a record gathered from the previous
 *  owner), adds the load and applies it, then all processes advance. The states must equal the
 *  reference bit for bit; the body must be held by exactly one process in every step. Further:
 *  ownership of points (exactly one owner), the refusals (decomposition after creation, a wrong
 *  plane count, an override beyond the neighbours), loadHistory() / setLoadHistory(), the
 *  prepared checkpoint (written on the root, read collectively, continued bit for bit), and
 *  sphereContainsPoints() against containsPoints() of a sphere.
 *
 *  --broken skip-history: the owner does not restore the history (the test must fail).
 */
//=================================================================================================

#include <mpi.h>
#include <pe/interface/coupling/Engine.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
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
static const int numSteps( 120 );
static const double dt( 1.0e-3 );
static const double relaxation( 0.35 );
static bool brokenSkipHistory( false );

//*************************************************************************************************
/*!\brief The deterministic fluid load of a step (some components exactly -0 or +0).
 */
static void loadOf( int step, pc::Vec3d& force, pc::Vec3d& torque )
{
   const double t( step * dt );
   force  = pc::Vec3d{ { 0.02 * std::sin( 40.0 * t ), ( step % 3 == 0 ) ? -0.0 : 0.0,
                         -0.01 * std::cos( 25.0 * t ) } };
   torque = pc::Vec3d{ { ( step % 2 == 0 ) ? -0.0 : 1.0e-6 * t, 0.0, 2.0e-6 * std::sin( t ) } };
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The bits of a state as one text line.
 */
static std::string stateLine( int step, const pc::BodyState& s )
{
   std::ostringstream oss;
   oss << "step " << step;
   const auto put = [&]( double x ) {
      std::uint64_t b;
      std::memcpy( &b, &x, sizeof( b ) );
      char buf[24];
      std::snprintf( buf, sizeof( buf ), " %016llx", static_cast<unsigned long long>( b ) );
      oss << buf;
   };
   for( int k=0; k<3; ++k ) put( s.position[k] );
   for( int k=0; k<4; ++k ) put( s.orientation[k] );
   for( int k=0; k<3; ++k ) put( s.linearVelocity[k] );
   for( int k=0; k<3; ++k ) put( s.angularVelocity[k] );
   return oss.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equal slabs along x over [-2, 2], one per process.
 */
static std::vector<pc::SlabPlane> slabs( int n )
{
   std::vector<pc::SlabPlane> planes;
   for( int k=0; k+1<n; ++k ) {
      const double x( -2.0 + 4.0 * ( k + 1 ) / n );
      planes.push_back( pc::SlabPlane{ pc::Vec3d{ { x, 0.0, 0.0 } },
                                       pc::Vec3d{ { -1.0, 0.0, 0.0 } } } );
   }
   return planes;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The process that holds a body (exactly one is required), and its state there.
 */
static int holder( pc::Engine& engine, pc::BodyId id, pc::BodyState& state, const char* what )
{
   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::movable, ids );
   int held( 0 );
   for( pc::BodyId b : ids ) held += ( b == id ) ? 1 : 0;
   int total( 0 );
   MPI_Allreduce( &held, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   expect( total == 1, std::string( what ) + ": the body is held by exactly one process" );
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   int owner( held ? rank : -1 );
   MPI_Allreduce( MPI_IN_PLACE, &owner, 1, MPI_INT, MPI_MAX, MPI_COMM_WORLD );
   double buf[13] = {};
   if( held ) {
      const pc::BodyState s( engine.state( id ) );
      for( int k=0; k<3; ++k ) { buf[k] = s.position[k]; buf[7+k] = s.linearVelocity[k];
                                 buf[10+k] = s.angularVelocity[k]; }
      for( int k=0; k<4; ++k ) buf[3+k] = s.orientation[k];
   }
   MPI_Bcast( buf, 13, MPI_DOUBLE, owner < 0 ? 0 : owner, MPI_COMM_WORLD );
   for( int k=0; k<3; ++k ) { state.position[k] = buf[k]; state.linearVelocity[k] = buf[7+k];
                              state.angularVelocity[k] = buf[10+k]; }
   for( int k=0; k<4; ++k ) state.orientation[k] = buf[3+k];
   return owner;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The history of a body, gathered from its holder (the record every process keeps).
 */
static pc::LoadHistory gatherHistory( pc::Engine& engine, pc::BodyId id, int owner )
{
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   double buf[14] = {};
   if( rank == owner ) {
      const pc::LoadHistory h( engine.loadHistory( id ) );
      buf[0] = h.hasPrevious ? 1.0 : 0.0;
      buf[1] = h.hasFluid ? 1.0 : 0.0;
      for( int k=0; k<3; ++k ) {
         buf[2+k] = h.previousForce[k]; buf[5+k] = h.previousTorque[k];
         buf[8+k] = h.fluidForce[k];    buf[11+k] = h.fluidTorque[k];
      }
   }
   MPI_Bcast( buf, 14, MPI_DOUBLE, owner, MPI_COMM_WORLD );
   pc::LoadHistory h;
   h.hasPrevious = buf[0] != 0.0;
   h.hasFluid    = buf[1] != 0.0;
   for( int k=0; k<3; ++k ) {
      h.previousForce[k] = buf[2+k]; h.previousTorque[k] = buf[5+k];
      h.fluidForce[k]    = buf[8+k]; h.fluidTorque[k]    = buf[11+k];
   }
   return h;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief One coupling step: the holder restores the history, adds and applies the load; all
 *        advance. Returns the new record of the history.
 */
static pc::LoadHistory couplingStep( pc::Engine& engine, pc::BodyId id, int step,
                                     const pc::LoadHistory& record )
{
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   pc::BodyState s;
   const int owner( holder( engine, id, s, "coupling step" ) );
   if( rank == owner ) {
      if( !brokenSkipHistory )
         engine.setLoadHistory( id, record );
      pc::Vec3d f, t;
      loadOf( step, f, t );
      engine.addLoad( id, f, t );
      engine.applyLoads( relaxation, dt );
   }
   const pc::LoadHistory next( gatherHistory( engine, id, owner ) );
   engine.advance( dt );
   return next;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The sphere of the trajectory test, created by the owner of its centre.
 */
static pc::BodyId createTraveller( pc::Engine& engine, pc::MaterialId mat )
{
   const pc::Vec3d centre{ { -1.9, 0.1, 0.0 } };
   pc::BodyId id( pc::noBody );
   if( engine.ownsPoint( centre ) )
      id = engine.createSphere( centre, 0.05, mat, 7 );
   std::uint64_t mine( id == pc::noBody ? 0 : id ), all( 0 );
   int created( id == pc::noBody ? 0 : 1 ), total( 0 );
   MPI_Allreduce( &mine, &all, 1, MPI_UINT64_T, MPI_MAX, MPI_COMM_WORLD );
   MPI_Allreduce( &created, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   expect( total == 1, "creation: exactly one process owns the centre and creates the sphere" );
   id = all;
   // a fast start along x: the sphere crosses all slab boundaries within the run
   pc::BodyState s;
   const int owner( holder( engine, id, s, "creation" ) );
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   if( rank == owner )
      engine.setKinematicState( id, pc::KinematicState{ s.position, s.orientation,
                                                        pc::Vec3d{ { 33.0, 0.0, 0.0 } }, zero } );
   return id;
}
//*************************************************************************************************


int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int rank( 0 ), size( 1 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( argc < 2 ) {
      std::fprintf( stderr, "usage: %s <reference file> [--write-reference] [--broken <v>]\n",
                    argv[0] );
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }
   const std::string reference( argv[1] );
   bool writeReference( false );
   for( int i=2; i<argc; ++i ) {
      if( std::string( argv[i] ) == "--write-reference" ) writeReference = true;
      if( std::string( argv[i] ) == "--broken" && i+1 < argc &&
          std::string( argv[i+1] ) == "skip-history" ) brokenSkipHistory = true;
   }
   expect( pc::buildInfo().mpi, "buildInfo().mpi" );

   pc::Engine engine;
   engine.setGravity( zero );
   expect( engine.processCount() == size && engine.processRank() == rank, "rank and size" );

   // decomposition: plane count refused, then equal slabs
   if( size > 1 )
      expectThrows<std::invalid_argument>( [&]() { engine.decomposeSlabs( {} ); },
                                           "decomposition with a wrong plane count", "planes" );
   engine.decomposeSlabs( slabs( size ) );
   expectThrows<std::logic_error>( [&]() { engine.decomposeSlabs( slabs( size ) ); },
                                   "a second decomposition", "decomposed before" );

   // every point has exactly one owner
   {
      std::mt19937_64 rng( 99 );
      std::uniform_real_distribution<double> u( -3.0, 3.0 );
      int bad( 0 );
      for( int i=0; i<200; ++i ) {
         double x( u( rng ) );
         if( i % 10 == 0 ) x = -2.0 + 4.0 * ( i % 4 ) / std::max( size, 1 );   // on a plane
         const pc::Vec3d p{ { x, u( rng ), u( rng ) } };
         int own( engine.ownsPoint( p ) ? 1 : 0 ), total( 0 );
         MPI_Allreduce( &own, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
         bad += ( total != 1 ) ? 1 : 0;
      }
      expect( bad == 0, "ownsPoint(): every point has exactly one owner" );
   }

   const pc::MaterialId mat( engine.createMaterial( "ddMat", 1200.0, 0.0, 0.1, 0.1 ) );
   const pc::BodyId id( createTraveller( engine, mat ) );
   expectThrows<std::logic_error>( [&]() { engine.decomposeSlabs( slabs( size ) ); },
                                   "decomposition after body creation", "decomposed before" );

   // the trajectory: bit for bit the one of a single process
   std::vector<std::string> lines;
   std::vector<int> owners;
   pc::LoadHistory record;
   std::string checkpoint;
   pc::LoadHistory recordAtCheckpoint;
   const int checkpointStep( 40 );
   for( int step=1; step<=numSteps; ++step ) {
      record = couplingStep( engine, id, step, record );
      pc::BodyState s;
      owners.push_back( holder( engine, id, s, "step" ) );
      lines.push_back( stateLine( step, s ) );
      if( step == checkpointStep ) {
         engine.prepareCheckpoint();
         std::ostringstream os;
         if( rank == 0 )
            engine.writeCheckpoint( os );
         else
            expectThrows<std::logic_error>( [&]() { std::ostringstream o; engine.writeCheckpoint( o ); },
                                            "writeCheckpoint() on a non-root process", "root" );
         checkpoint = os.str();
         recordAtCheckpoint = record;
      }
   }
   int crossings( 0 );
   for( std::size_t i=1; i<owners.size(); ++i ) crossings += ( owners[i] != owners[i-1] ) ? 1 : 0;
   if( rank == 0 )
      std::printf( "pe_coupling_mpi_dd_test (%d ranks): the sphere changed its owner %d times\n",
                   size, crossings );
   expect( crossings == size - 1, "the sphere crossed every slab boundary" );

   if( writeReference ) {
      if( rank == 0 ) {
         std::ofstream out( reference.c_str() );
         for( const std::string& l : lines ) out << l << '\n';
      }
   }
   else {
      std::ifstream in( reference.c_str() );
      std::string l;
      int mismatch( -1 );
      for( std::size_t i=0; i<lines.size(); ++i ) {
         if( !std::getline( in, l ) || l != lines[i] ) { mismatch = int( i ) + 1; break; }
      }
      if( rank == 0 && mismatch > 0 )
         std::printf( "first differing step: %d\n", mismatch );
      expect( mismatch < 0, "the trajectory equals the single-process reference bit for bit" );
   }

   // the history: loadHistory() / setLoadHistory() round trip on the holder
   {
      pc::BodyState s;
      const int owner( holder( engine, id, s, "history" ) );
      if( rank == owner ) {
         pc::LoadHistory h;
         h.hasPrevious = true;
         h.previousForce = pc::Vec3d{ { 1.0, -0.0, 3.0 } };
         h.hasFluid = true;
         h.fluidTorque = pc::Vec3d{ { -0.0, 2.0, 0.5 } };
         const pc::LoadHistory before( engine.loadHistory( id ) );
         engine.setLoadHistory( id, h );
         const pc::LoadHistory back( engine.loadHistory( id ) );
         expect( back.hasPrevious && back.hasFluid && sameBits( back.previousForce, h.previousForce )
                 && sameBits( back.fluidTorque, h.fluidTorque ), "setLoadHistory() round trip" );
         pc::LoadHistory none;
         engine.setLoadHistory( id, none );
         expect( !engine.loadHistory( id ).hasPrevious && !engine.loadHistory( id ).hasFluid,
                 "setLoadHistory() without a history" );
         pc::LoadHistory bad( h );
         bad.fluidForce[0] = std::nan( "" );
         expectThrows<std::invalid_argument>( [&]() { engine.setLoadHistory( id, bad ); },
                                              "setLoadHistory() of a non-finite history", "finite" );
         engine.setLoadHistory( id, before );
      }
   }

   // the checkpoint: read collectively (the same text on all processes), continue bit for bit.
   // The sphere is now in the last slab; on 4 processes the checkpointed position lies two slabs
   // away, which is refused (pe migrates bodies to neighbours only).
   {
      int len( int( checkpoint.size() ) );
      MPI_Bcast( &len, 1, MPI_INT, 0, MPI_COMM_WORLD );
      checkpoint.resize( std::size_t( len ) );
      MPI_Bcast( &checkpoint[0], len, MPI_CHAR, 0, MPI_COMM_WORLD );
      const int ownerAtCheckpoint( owners[std::size_t( checkpointStep - 1 )] );
      pc::BodyState s;
      const int ownerNow( holder( engine, id, s, "before the restore" ) );
      if( std::abs( ownerNow - ownerAtCheckpoint ) > 1 ) {
         std::istringstream is( checkpoint );
         expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( is ); },
                                           "a checkpoint position beyond the neighbours",
                                           "neighbouring subdomains" );
         // move the body back step by step: one slab per synchronization
         for( int k=ownerNow-1; k>ownerAtCheckpoint; --k ) {
            pc::BodyState cur;
            const int o( holder( engine, id, cur, "moving back" ) );
            if( rank == o )
               engine.setKinematicState( id, pc::KinematicState{
                  pc::Vec3d{ { -2.0 + 4.0 * ( k + 0.5 ) / size, cur.position[1], cur.position[2] } },
                  cur.orientation, zero, zero } );
            engine.advance( 1.0e-12 );
         }
      }
      std::istringstream is( checkpoint );
      engine.readCheckpoint( is );
      record = recordAtCheckpoint;
      int mismatch( -1 );
      for( int step=checkpointStep+1; step<=numSteps; ++step ) {
         record = couplingStep( engine, id, step, record );
         pc::BodyState st;
         holder( engine, id, st, "after the restore" );
         if( mismatch < 0 && stateLine( step, st ) != lines[std::size_t( step - 1 )] )
            mismatch = step;
      }
      if( rank == 0 && mismatch > 0 )
         std::printf( "after the restore: first differing step %d\n", mismatch );
      expect( mismatch < 0, "the restored run continues bit for bit" );
   }

   // sphereContainsPoints(): pe's sphere predicate for a sphere the process need not hold
   {
      pc::BodyState s;
      const int owner( holder( engine, id, s, "predicate" ) );
      if( rank == owner ) {
         const double r( engine.state( id ).radius );
         std::mt19937_64 rng( 5 );
         std::uniform_real_distribution<double> u( -1.0, 1.0 );
         std::vector<double> xyz;
         for( int i=0; i<2000; ++i ) {
            pc::Vec3d d{ { u( rng ), u( rng ), u( rng ) } };
            const double n( std::sqrt( d[0]*d[0] + d[1]*d[1] + d[2]*d[2] ) );
            const double scale( r * ( i % 2 ? 1.0 : ( 1.0 + 1.0e-15 * u( rng ) ) ) / n );
            for( int k=0; k<3; ++k ) xyz.push_back( s.position[k] + d[k] * scale );
         }
         std::vector<unsigned char> a( 2000 ), b( 2000 );
         engine.containsPoints( id, xyz.data(), 2000, a.data() );
         pc::sphereContainsPoints( s.position, r, xyz.data(), 2000, b.data() );
         int in( 0 );
         for( unsigned char f : a ) in += f;
         expect( a == b && in > 0 && in < 2000,
                 "sphereContainsPoints() equals containsPoints() of the sphere (surface points)" );
      }
   }

   // a prepared checkpoint is invalidated by a change on the root: a second sphere in the root's
   // slab, prepareCheckpoint(), then setLoadHistory() of that sphere on the root
   if( size > 1 ) {
      const pc::Vec3d centre{ { -1.9, -0.5, 0.0 } };
      pc::BodyId extra( pc::noBody );
      if( engine.ownsPoint( centre ) )
         extra = engine.createSphere( centre, 0.05, mat, 8 );
      expect( ( rank == 0 ) == ( extra != pc::noBody ), "the second sphere is created on the root" );
      engine.prepareCheckpoint();
      if( rank == 0 ) {
         engine.setLoadHistory( extra, engine.loadHistory( extra ) );
         expectThrows<std::logic_error>( [&]() { std::ostringstream o; engine.writeCheckpoint( o ); },
                                         "writeCheckpoint() after setLoadHistory() on the root",
                                         "prepareCheckpoint" );
      }
   }

   int local( coupling_test::failures() ), total( 0 );
   MPI_Allreduce( &local, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 )
      std::printf( "pe_coupling_mpi_dd_test (%d ranks%s): %s (%d failed checks over all ranks)\n",
                   size, brokenSkipHistory ? ", broken: skip-history" : "",
                   total ? "FAILED" : "PASSED", total );
   MPI_Finalize();
   return total ? 1 : 0;
}
