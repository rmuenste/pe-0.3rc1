//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_mpi_dd_rules_test.cpp
 *  \brief The coupling API on a domain-decomposed world: ownership on inclined planes, the reach
 *         and displacement rules, and contacts across a slab plane after applied loads
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  Usage: pe_coupling_mpi_dd_rules_test incline
 *         pe_coupling_mpi_dd_rules_test parallel
 *         pe_coupling_mpi_dd_rules_test friction <reference file> [--write-reference]
 *
 *  incline (2 or 4 processes): dividing planes through the z axis, the first through (1,0,0)
 *  with the normal (-1,1,0). Every point on a plane has exactly one owner (the two slabs at a
 *  plane share exactly its points; the opposite half space is the exact complement), and a
 *  sphere centred on a plane is created by exactly one process (the lowest claiming rank). On
 *  4 processes a sphere next to the common axis reaches every slab: advance() refuses it on all
 *  processes (reach rule) and nothing moves.
 *
 *  parallel (4 processes): planes x = -1, -0.95, 1 (an interior slab of width 0.05) and the
 *  closing plane x = 3. createSphere() refuses a sphere whose reach is not smaller than 0.05; a
 *  small sphere whose displacement in one pe step can carry it two slabs on is refused by
 *  advance() on all processes (displacement rule), and stepped when slow; a checkpoint whose
 *  saved position lies beyond the closing plane (no process owns it) is refused.
 *
 *  friction (1 process with --write-reference, 2 processes): two touching spheres on both sides
 *  of the plane x = 0 (the contact point lies on it, so process 0 constructs the contact with a
 *  shadow copy of the sphere of process 1); static friction 0.9, dynamic friction 0.05. Each
 *  step the holder of the right sphere applies a load that pushes it against the other and
 *  along y. The velocities after each step must agree with the single-process reference to
 *  1e-4 of the largest velocity of the step: pe's contact construction selects the friction
 *  coefficient by the relative velocity, so it must see the velocity after the load, not the
 *  shadow copy's old one (with the old one it takes the static coefficient and the result
 *  differs by far more). The agreement is not bitwise: pe relaxes contacts between processes
 *  (setContactRelaxation()), measured 1e-6.
 */
//=================================================================================================

#include <mpi.h>
#include <pe/interface/coupling/Engine.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
namespace pc = pe::coupling;

static const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };

//*************************************************************************************************
/*!\brief The number of processes that claim a point.
 */
static int claims( pc::Engine& engine, const pc::Vec3d& p )
{
   int own( engine.ownsPoint( p ) ? 1 : 0 ), total( 0 );
   MPI_Allreduce( &own, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   return total;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creates a sphere on the lowest process that claims its centre; returns the id
 *        (everywhere) and the number of processes that created one.
 */
static pc::BodyId createOnOwner( pc::Engine& engine, const pc::Vec3d& c, double r,
                                 pc::MaterialId mat, int& created )
{
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   int claim( engine.ownsPoint( c ) ? rank : std::numeric_limits<int>::max() ), lowest( 0 );
   MPI_Allreduce( &claim, &lowest, 1, MPI_INT, MPI_MIN, MPI_COMM_WORLD );
   std::uint64_t id( 0 );
   int mine( 0 );
   if( rank == lowest ) {
      id = engine.createSphere( c, r, mat, 1 );
      mine = 1;
   }
   std::uint64_t all( 0 );
   MPI_Allreduce( &id, &all, 1, MPI_UINT64_T, MPI_MAX, MPI_COMM_WORLD );
   MPI_Allreduce( &mine, &created, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   return all;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The state of a body, gathered from its holder.
 */
static pc::BodyState stateOf( pc::Engine& engine, pc::BodyId id )
{
   int rank( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::movable, ids );
   int held( 0 );
   for( pc::BodyId b : ids ) held += ( b == id ) ? 1 : 0;
   int owner( held ? rank : -1 );
   MPI_Allreduce( MPI_IN_PLACE, &owner, 1, MPI_INT, MPI_MAX, MPI_COMM_WORLD );
   double buf[9] = {};
   if( held ) {
      const pc::BodyState s( engine.state( id ) );
      for( int k=0; k<3; ++k ) { buf[k] = s.position[k]; buf[3+k] = s.linearVelocity[k];
                                 buf[6+k] = s.angularVelocity[k]; }
   }
   MPI_Bcast( buf, 9, MPI_DOUBLE, owner < 0 ? 0 : owner, MPI_COMM_WORLD );
   pc::BodyState s{};
   for( int k=0; k<3; ++k ) { s.position[k] = buf[k]; s.linearVelocity[k] = buf[3+k];
                              s.angularVelocity[k] = buf[6+k]; }
   return s;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inclined planes: unique ownership on the planes, unique creation, the reach rule.
 */
static void incline( pc::Engine& engine, int size )
{
   // planes through the z axis; the first through (1,0,0) with the normal (-1,1,0) (a plane
   // through the axis shifted along x, as in the review case)
   std::vector<pc::SlabPlane> planes;
   planes.push_back( pc::SlabPlane{ pc::Vec3d{ { 1.0, 0.0, 0.0 } }, pc::Vec3d{ { -1.0, 1.0, 0.0 } } } );
   for( int k=1; k+1<size; ++k ) {
      const double a( 3.14159265358979 * ( 0.25 + 0.2 * k ) );
      planes.push_back( pc::SlabPlane{ pc::Vec3d{ { 1.0, 0.0, 0.0 } },
                                       pc::Vec3d{ { -std::sin( a ), std::cos( a ), 0.0 } } } );
   }
   engine.decomposeSlabs( planes );

   // points on the first plane: (1,0,0) + s (1,1,0) + t (0,0,1), on the half of it (s < 0)
   // that only slabs 0 and 1 share (planes through a common axis cut the space into wedges, and
   // on the other half of the first plane the last wedge overlaps the first)
   int bad( 0 ), checked( 0 );
   for( int i=-40; i<=-1; ++i ) {
      for( int j=0; j<3; ++j ) {
         const double s( 0.0173 * i ), t( 0.37 * j - 0.4 );
         const pc::Vec3d p{ { 1.0 + s, s, t } };
         bad += ( claims( engine, p ) != 1 ) ? 1 : 0;
         ++checked;
      }
   }
   expect( bad == 0, "every point on the inclined plane has exactly one owner (" +
                     std::to_string( bad ) + " of " + std::to_string( checked ) + " not)" );

   // a sphere centred on the plane is created by exactly one process
   const pc::MaterialId mat( engine.createMaterial( "rulesIncline", 1000.0, 0.0, 0.1, 0.1 ) );
   int created( 0 );
   const pc::Vec3d onPlane{ { 1.0 - 0.0173 * 17, -0.0173 * 17, 0.0 } };
   createOnOwner( engine, onPlane, 0.01, mat, created );
   expect( created == 1, "a sphere centred on the inclined plane is created exactly once" );
   engine.advance( 1.0e-3 );

   if( size >= 4 ) {
      // next to the common axis (all planes contain the point (1,0,0) and the z direction) a
      // sphere reaches every slab: the reach rule refuses the step on all processes
      const pc::BodyId near( createOnOwner( engine, pc::Vec3d{ { 1.01, 0.002, 0.3 } }, 0.05, mat,
                                            created ) );
      expect( created == 1, "the sphere next to the axis is created exactly once" );
      const pc::BodyState before( stateOf( engine, near ) );
      expectThrows<std::runtime_error>( [&]() { engine.advance( 1.0e-3 ); },
                                        "advance() with a sphere reaching a distant slab",
                                        "reaches into slab", "not a neighbour" );
      const pc::BodyState after( stateOf( engine, near ) );
      expect( before.position == after.position, "the refused step moved nothing" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Parallel planes with a thin interior slab: the creation rule and the displacement rule.
 */
static void parallel( pc::Engine& engine, int rank )
{
   std::vector<pc::SlabPlane> planes;
   for( double x : { -1.0, -0.95, 1.0, 3.0 } )
      planes.push_back( pc::SlabPlane{ pc::Vec3d{ { x, 0.0, 0.0 } }, pc::Vec3d{ { -1.0, 0.0, 0.0 } } } );
   engine.decomposeSlabs( planes );
   const pc::MaterialId mat( engine.createMaterial( "rulesParallel", 1000.0, 0.0, 0.1, 0.1 ) );

   // the creation rule: reach 0.05 + overlap is not smaller than the 0.05 slab (on the owner)
   const pc::Vec3d c{ { -1.5, 0.0, 0.0 } };
   int refused( 0 );
   if( engine.ownsPoint( c ) ) {
      try { engine.createSphere( c, 0.05, mat, 1 ); }
      catch( const std::invalid_argument& ex ) {
         refused = std::string( ex.what() ).find( "thinnest interior slab" ) != std::string::npos;
      }
   }
   MPI_Allreduce( MPI_IN_PLACE, &refused, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   expect( refused == 1, "createSphere() refuses a sphere as thick as the thinnest slab" );

   // the displacement rule: a small sphere in slab 0 that moves 0.4 in one step of 1e-3
   int created( 0 );
   const pc::BodyId id( createOnOwner( engine, pc::Vec3d{ { -1.2, 0.0, 0.0 } }, 0.01, mat,
                                       created ) );
   expect( created == 1, "the small sphere is created" );
   engine.advance( 1.0e-3 );   // at rest: allowed
   const pc::BodyState s0( stateOf( engine, id ) );
   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::movable, ids );
   for( pc::BodyId b : ids )
      if( b == id )
         engine.setKinematicState( id, pc::KinematicState{ s0.position,
                                   pc::Quatd{ { 1.0, 0.0, 0.0, 0.0 } }, pc::Vec3d{ { 400.0, 0.0, 0.0 } },
                                   zero } );
   expectThrows<std::runtime_error>( [&]() { engine.advance( 1.0e-3 ); },
                                     "advance() that could carry a body two slabs on",
                                     "can move into slab", "outflow" );
   const pc::BodyState s1( stateOf( engine, id ) );
   expect( s1.position == s0.position, "the refused step moved nothing" );
   // with ten substeps each pe step moves it 0.04: allowed
   engine.advance( 1.0e-3, 10 );
   const pc::BodyState s2( stateOf( engine, id ) );
   expect( s2.position[0] > s0.position[0] + 0.3, "the step with substeps moved the sphere" );

   // a checkpoint whose saved position lies beyond the closing plane: no process owns it
   engine.prepareCheckpoint();
   std::string text;
   if( rank == 0 ) {
      std::ostringstream os;
      engine.writeCheckpoint( os );
      text = os.str();
   }
   int len( int( text.size() ) );
   MPI_Bcast( &len, 1, MPI_INT, 0, MPI_COMM_WORLD );
   text.resize( std::size_t( len ) );
   MPI_Bcast( &text[0], len, MPI_CHAR, 0, MPI_COMM_WORLD );
   std::istringstream in( text );
   std::ostringstream edited;
   std::string line;
   while( std::getline( in, line ) ) {
      std::istringstream ls( line );
      std::string tag;
      pc::BodyId bid( 0 );
      if( ( ls >> tag >> bid ) && tag == "body" && bid == id ) {
         std::vector<std::string> tok;
         std::istringstream ts( line );
         std::string t;
         while( ts >> t ) tok.push_back( t );
         tok[2] = "3.5";
         line.clear();
         for( std::size_t i=0; i<tok.size(); ++i ) line += ( i ? " " : "" ) + tok[i];
      }
      edited << line << '\n';
   }
   std::istringstream beyond( edited.str() );
   expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( beyond ); },
                                     "a saved position beyond the closing plane", "owned by 0 processes" );
   const pc::BodyState s3( stateOf( engine, id ) );
   expect( s3.position == s2.position, "the refused restore changed nothing" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Two touching spheres on both sides of x = 0; returns one line per step.
 */
static std::vector<std::string> friction( pc::Engine& engine, int size )
{
   if( size > 1 ) {
      engine.decomposeSlabs( { pc::SlabPlane{ pc::Vec3d{ { 0.0, 0.0, 0.0 } },
                                              pc::Vec3d{ { -1.0, 0.0, 0.0 } } } } );
   }
   const pc::MaterialId mat( engine.createMaterial( "rulesFriction", 1000.0, 0.0, 0.9, 0.05 ) );
   int created( 0 );
   const pc::BodyId a( createOnOwner( engine, pc::Vec3d{ { -0.05, 0.0, 0.0 } }, 0.05, mat,
                                      created ) );
   const pc::BodyId b( createOnOwner( engine, pc::Vec3d{ { 0.05, 0.0, 0.0 } }, 0.05, mat,
                                      created ) );
   engine.advance( 1.0e-12 );   // shadow copies exist
   std::vector<std::string> lines;
   for( int step=1; step<=6; ++step ) {
      std::vector<pc::BodyId> ids;
      engine.bodyIds( pc::Selection::movable, ids );
      for( pc::BodyId x : ids )
         if( x == b )
            engine.addLoad( b, pc::Vec3d{ { -2.0, 3.0 * step, 0.0 } }, zero );
      engine.applyLoads( 1.0, 1.0e-3 );
      engine.advance( 1.0e-3 );
      std::ostringstream oss;
      oss.precision( 17 );
      oss << "step " << step;
      for( pc::BodyId x : { a, b } ) {
         const pc::BodyState s( stateOf( engine, x ) );
         for( int k=0; k<3; ++k ) oss << ' ' << s.linearVelocity[k];
         for( int k=0; k<3; ++k ) oss << ' ' << s.angularVelocity[k];
      }
      lines.push_back( oss.str() );
   }
   return lines;
}
//*************************************************************************************************


int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int rank( 0 ), size( 1 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   const std::string mode( argc > 1 ? argv[1] : "" );

   pc::Engine engine;
   engine.setGravity( zero );

   if( mode == "incline" ) {
      incline( engine, size );
   }
   else if( mode == "parallel" ) {
      parallel( engine, rank );
   }
   else if( mode == "friction" && argc > 2 ) {
      const std::string reference( argv[2] );
      const bool write( argc > 3 && std::string( argv[3] ) == "--write-reference" );
      const std::vector<std::string> lines( friction( engine, size ) );
      if( write ) {
         if( rank == 0 ) {
            std::ofstream out( reference.c_str() );
            for( const std::string& l : lines ) out << l << '\n';
         }
      }
      else {
         std::ifstream in( reference.c_str() );
         std::string l;
         double worst( 0.0 );
         std::size_t n( 0 );
         for( const std::string& mine : lines ) {
            if( !std::getline( in, l ) ) { worst = 1.0; break; }
            std::istringstream ra( l ), rb( mine );
            std::string w1, w2;
            int s1( 0 ), s2( 0 );
            ra >> w1 >> s1;
            rb >> w2 >> s2;
            std::vector<double> xa, xb;
            double x( 0 ), y( 0 );
            while( ra >> x && rb >> y ) { xa.push_back( x ); xb.push_back( y ); }
            double scale( 0.0 );
            for( double v : xa ) scale = std::max( scale, std::fabs( v ) );
            for( std::size_t i=0; i<xa.size(); ++i ) {
               worst = std::max( worst, std::fabs( xa[i] - xb[i] ) / scale );
               ++n;
            }
         }
         if( rank == 0 )
            std::printf( "pe_coupling_mpi_dd_rules_test friction (%d ranks): largest relative "
                         "difference to one process %.3e over %zu values\n", size, worst, n );
         expect( n == 6 * 12 && worst <= 1.0e-4,
                 "the velocities agree with the single-process reference (1e-4 of the largest)" );
      }
   }
   else {
      if( rank == 0 )
         std::fprintf( stderr, "usage: %s incline | parallel | friction <reference> "
                       "[--write-reference]\n", argv[0] );
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }

   int local( coupling_test::failures() ), total( 0 );
   MPI_Allreduce( &local, &total, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 )
      std::printf( "pe_coupling_mpi_dd_rules_test %s (%d ranks): %s (%d failed checks over all "
                   "ranks)\n", mode.c_str(), size, total ? "FAILED" : "PASSED", total );
   MPI_Finalize();
   return total ? 1 : 0;
}
