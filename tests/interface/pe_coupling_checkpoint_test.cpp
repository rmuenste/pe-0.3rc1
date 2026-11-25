//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_checkpoint_test.cpp
 *  \brief pe::coupling checkpoint: bitwise restart in a fresh process, and the error paths
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
 *  A restart happens in a new process that rebuilds the world with the same creation calls (so
 *  the bodies get the same system ids) and reads the checkpoint, and so does this test:
 *    --write <dir>  build the world (three spheres, a plane and a fixed sphere moving with a
 *                   prescribed velocity, under MOBILE_INFINITE; a restart that put it back at
 *                   its setup position would differ), run N
 *                   coupling steps (relaxed loads, contacts with the plane and between the
 *                   spheres), write <dir>/checkpoint.txt, run M more
 *                   steps, write the final states bit for bit to <dir>/reference.txt; then
 *                   check that reading a checkpoint back into the live world changes no bit;
 *    --read <dir>   (a second process, CTest fixture) build the identical world, read the
 *                   checkpoint, run M steps, compare with reference.txt bit for bit;
 *    --pile         with --write / --read: 64 spheres settling into a pile instead of three,
 *                   so the hash grid is active; restarts bit for bit;
 *    --shake        with --write / --read: 48 spheres with random velocities in a walled box,
 *                   bodies exchange hash-grid cells and many contacts are resolved at once.
 *                   CHARACTERISATION: the restart is NOT bit for bit. pe's hash grid keeps the
 *                   order of the bodies in each cell from its history (bodies leaving a cell
 *                   swap the last one into their slot), while a freshly built world files them
 *                   in creation order, so the contacts reach the solver in another order; the
 *                   run is reproduced up to round-off (about 1e-11 m after 400 steps here).
 *                   The same scenario with 27 spheres, grid inactive, restarts bit for bit.
 *                   The test reports which case it saw and the deviations, and asserts only
 *                   finiteness and a loose sanity bound (1e-6 m, 1e-6 m/s);
 *    --history      with --write / --read: CHARACTERISATION. Spheres A, B, C; the uninterrupted
 *                   run takes one step, creates D (landing on B and C) and destroys A; a rebuilt
 *                   world replays the creations and the destruction before its first step. pe's
 *                   coarse detector keeps pending and filed bodies in separate lists with
 *                   swap-removal, so it traverses the same bodies in another order, and the
 *                   restart is not bit for bit although the grid is inactive. Reported; only
 *                   finiteness and the loose bound are asserted;
 *    --plane-geometry  CHARACTERISATION: the stored normal of a tilted plane at rest (normal
 *                   (1e-5, 0, 1)) changes in its first step (pe re-derives it from the plane's
 *                   quaternion; the tilt moves from x to y), so a rebuilt world starts with
 *                   another plane geometry than the uninterrupted one. Reported only;
 *    --errors       the error paths: wrong tag, unsupported versions, body-count mismatches,
 *                   unknown, duplicate and wrong-kind ids, non-unit quaternions, an
 *                   unrepresentable position, a bad history flag, truncation (also at the very
 *                   end), a fixed body with a linear or angular velocity, or a -0 component,
 *                   in a build without MOBILE_INFINITE (with it: restored exactly); each
 *                   refused read is made against a live state that differs from the
 *                   checkpoint, with a pending load, and must leave kinematics, load history
 *                   and the pending load unchanged. Worlds outside the checkpoint's scope (a union,
 *                   a moving plane) are refused for write and read. Pending loads at write time
 *                   are refused. The caller's stream settings (hex, noskipws, width, fill, a
 *                   comma locale) change nothing, data after the checkpoint stays in the
 *                   stream. In a build without checkpoint support both functions must refuse.
 *  Linked against pe_static_lubstage_fluid (HardContactAndFluid). Serial, no MPI.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <locale>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::bits;
using coupling_test::expect;
using coupling_test::expectThrows;
namespace pc = pe::coupling;

static const double dt( 5.0e-4 );
static const int    stepsBefore( 400 );
static const int    stepsAfter( 400 );


//*************************************************************************************************
/*!\brief The scenarios of the restart test.
 */
enum class Scenario {
   basic,  //!< Three spheres, a plane and a fixed sphere moving with a prescribed velocity.
   pile,   //!< 64 spheres settling into a pile (hash grid active).
   shake,  //!< 48 spheres with random velocities in a walled box (cell exchanges, contacts).
   history //!< Bodies created and destroyed after stepping began (detector order differs).
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Builds the world: always the same calls in the same order.
 */
static void buildWorld( pc::Engine& engine, Scenario scenario, bool writing )
{
   engine.setGravity( pc::Vec3d{ { 0.0, 0.0, -9.81 } } );
   engine.setFluidDensity( 970.0 );
   engine.setFluidViscosity( 0.373 );
   engine.setSplitImpulse( true );
   const pc::MaterialId sphere( engine.createMaterial( "ckptSphere", 1120.0, 0.0, 0.25, 0.25 ) );
   const pc::MaterialId wall( engine.createMaterial( "ckptWall", 2500.0, 0.0, 0.25, 0.25 ) );
   engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, wall, 1 );

   if( scenario == Scenario::pile ) {
      for( int i=0; i<4; ++i )
         for( int j=0; j<4; ++j )
            for( int k=0; k<4; ++k )
               engine.createSphere( pc::Vec3d{ { 0.016*i + 0.001*k, 0.016*j - 0.0007*k,
                                                 0.01 + 0.0155*k } }, 0.0075, sphere, 2 );
      return;
   }

   if( scenario == Scenario::history ) {
      // A, B, C; in the uninterrupted run one step is taken before D is created and A is
      // destroyed. A rebuilt world replays the creations and the destruction before its first
      // step, so pe's coarse detector holds the same bodies in a different order.
      const double r( 0.0075 );
      const pc::BodyId a( engine.createSphere( pc::Vec3d{ { 0.1, 0.0, r } }, r, sphere, 2 ) );
      engine.createSphere( pc::Vec3d{ { 0.0, 0.0, r } }, r, sphere, 3 );
      engine.createSphere( pc::Vec3d{ { 2.0*r, 0.0, r } }, r, sphere, 4 );
      if( writing ) engine.advance( dt );
      engine.createSphere( pc::Vec3d{ { r, 0.0, r + std::sqrt( 3.0 ) * r + 1.0e-4 } }, r,
                           sphere, 5 );   // lands on B and C at once
      pe::destroy( *pe::theCollisionSystem()->getBodyStorage().find( a ) );
      return;
   }

   if( scenario == Scenario::shake ) {
      // Walls of a 0.1 x 0.1 box around the origin, normals into the box
      engine.createPlane( pc::Vec3d{ {  1.0, 0.0, 0.0 } }, -0.05, wall, 1 );
      engine.createPlane( pc::Vec3d{ { -1.0, 0.0, 0.0 } }, -0.05, wall, 1 );
      engine.createPlane( pc::Vec3d{ { 0.0,  1.0, 0.0 } }, -0.05, wall, 1 );
      engine.createPlane( pc::Vec3d{ { 0.0, -1.0, 0.0 } }, -0.05, wall, 1 );
      std::mt19937 rng( 17u );
      std::uniform_real_distribution<double> u( -1.0, 1.0 );
      for( int i=0; i<4; ++i )
         for( int j=0; j<4; ++j )
            for( int k=0; k<3; ++k ) {
               const pc::BodyId id( engine.createSphere(
                  pc::Vec3d{ { -0.036 + 0.024*i + 0.003*u( rng ), -0.036 + 0.024*j + 0.003*u( rng ),
                               0.008 + 0.016*k } }, 0.006, sphere, 2 ) );
               const pc::BodyState s( engine.state( id ) );
               engine.setKinematicState( id, pc::KinematicState{ s.position, s.orientation,
                  { { 0.4*u( rng ), 0.4*u( rng ), 0.2*u( rng ) } }, { { 0.0, 0.0, 0.0 } } } );
            }
      return;
   }

   engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.010 } }, 0.0075, sphere, 2 );
   engine.createSphere( pc::Vec3d{ { 0.004, 0.0, 0.030 } }, 0.0075, sphere, 3 );
   engine.createSphere( pc::Vec3d{ { -0.02, 0.01, 0.015 } }, 0.005, sphere, 4 );
   // A fixed sphere moving with a prescribed velocity (a kinematic body under MOBILE_INFINITE);
   // its position is part of the compared states, so a restart that put it back at its setup
   // position fails the comparison
   pe::SphereID pusher( pe::createSphere( 5, pe::Vec3( 0.03, 0.0, 0.0075 ), 0.004,
                                          static_cast<pe::MaterialID>( sphere ) ) );
   pusher->setFixed( true );
   pusher->setLinearVel( pe::Vec3( -0.05, 0.0, 0.0 ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief One coupling step with a state-dependent load on every movable body.
 */
static void couplingStep( pc::Engine& engine, int step,
                          const std::function<void()>& afterApply = std::function<void()>() )
{
   std::vector<pc::BodyState> states;
   engine.states( pc::Selection::movable, states );
   for( const pc::BodyState& s : states ) {
      const double c( 6.0 * pe::M_PI * 0.373 * s.radius );
      // Around the checkpoint the y force is +1e-12, then -0 twice: the first step after the
      // checkpoint relaxes -0 against -0 in the API (F_eff = -0), while pe's body-level
      // history holds a positive F_eff, which applyFluidForces( dt, 1 ) adds as 0 * F_prev
      double fy( -c * s.linearVelocity[1] );
      if( step == stepsBefore - 2 ) fy = 1.0e-12;
      if( step == stepsBefore - 1 || step == stepsBefore ) fy = -0.0;
      const pc::Vec3d f{ { -c * s.linearVelocity[0] + 1.0e-6 * ( step % 5 ),
                           fy, -c * s.linearVelocity[2] } };
      const pc::Vec3d t{ { -c * s.radius * s.radius * s.angularVelocity[0], 2.0e-9,
                           -c * s.radius * s.radius * s.angularVelocity[2] } };
      engine.addLoad( s.id, f, t );
   }
   engine.applyLoads( 0.7, dt );
   if( afterApply ) afterApply();
   engine.advance( dt );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The states of all finite bodies (movable and fixed).
 */
static std::vector<pc::BodyState> finiteStates( const pc::Engine& engine )
{
   std::vector<pc::BodyState> all, finite;
   engine.states( pc::Selection::all, all );
   for( const pc::BodyState& s : all )
      if( s.shape != pc::ShapeKind::plane ) finite.push_back( s );
   return finite;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The states as text: id and the bit patterns of all kinematic values, all finite bodies.
 */
static std::string fingerprint( const pc::Engine& engine )
{
   std::ostringstream oss;
   for( const pc::BodyState& s : finiteStates( engine ) ) {
      oss << s.id;
      for( double x : s.position ) oss << ' ' << std::hex << bits( x ) << std::dec;
      for( double x : s.orientation ) oss << ' ' << std::hex << bits( x ) << std::dec;
      for( double x : s.linearVelocity ) oss << ' ' << std::hex << bits( x ) << std::dec;
      for( double x : s.angularVelocity ) oss << ' ' << std::hex << bits( x ) << std::dec;
      oss << '\n';
   }
   return oss.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief pe's body-level fluid-force history of every finite body (flag, force and torque bits).
 */
static std::string fluidHistory( const pc::Engine& engine )
{
   std::ostringstream oss;
   for( const pc::BodyState& s : finiteStates( engine ) ) {
      const pe::ConstBodyID b( *pe::theCollisionSystem()->getBodyStorage().find( s.id ) );
      oss << s.id << ' ' << b->hasPrevFluidForce();
      for( int k=0; k<3; ++k ) oss << ' ' << std::hex << bits( b->getPrevFluidForce()[k] );
      for( int k=0; k<3; ++k ) oss << ' ' << bits( b->getPrevFluidTorque()[k] ) << std::dec;
      oss << '\n';
   }
   return oss.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Positions and velocities as text with 17 digits, for the non-bitwise comparison.
 */
static std::string positions( const pc::Engine& engine )
{
   std::ostringstream oss;
   oss.precision( 17 );
   for( const pc::BodyState& s : finiteStates( engine ) ) {
      oss << s.id;
      for( int k=0; k<3; ++k ) oss << ' ' << s.position[k];
      for( int k=0; k<3; ++k ) oss << ' ' << s.linearVelocity[k];
      oss << '\n';
   }
   return oss.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief In a build without checkpoint support (sleep mode, pe::real wider than double) both
 *        checkpoint functions must refuse; returns true if that case was handled.
 */
static bool refusedByBuild( pc::Engine& engine )
{
   if( pc::buildInfo().checkpointsSupported )
      return false;
   std::stringstream ckpt;
   expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                     "this build refuses to write checkpoints" );
   std::stringstream fake( "pe-coupling-checkpoint 3\nbodies 0\nfixed 0\nend\n" );
   expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( fake ); },
                                     "this build refuses to read checkpoints" );
   std::printf( "checkpoints are not supported by this build (sleep mode or wide real): "
                "refusal checked\n" );
   return true;
}
//*************************************************************************************************


static int runWrite( const std::string& dir, Scenario scenario )
{
   pc::Engine engine;
   buildWorld( engine, scenario, true );
   if( refusedByBuild( engine ) )
      return coupling_test::report( "pe_coupling_checkpoint_test --write" );
   const std::string setup( fingerprint( engine ) );
   for( int step=0; step<stepsBefore; ++step ) couplingStep( engine, step );
   {
      std::ofstream out( dir + "/checkpoint.txt" );
      engine.writeCheckpoint( out );
      expect( bool( out ), "checkpoint written" );
   }
   expect( fingerprint( engine ) != setup, "the bodies moved before the checkpoint" );
   const std::string fluid( fluidHistory( engine ) );
   std::ofstream( dir + "/fluid.txt" ) << fluid;
   expect( fluid.find( " 1 " ) != std::string::npos,
           "pe's body-level fluid-force history is set at the checkpoint" );

   // The state right after the first impulse following the checkpoint
   couplingStep( engine, stepsBefore, [&]() {
      std::ofstream( dir + "/impulse.txt" ) << fingerprint( engine );
   } );
   for( int step=stepsBefore+1; step<stepsBefore+stepsAfter; ++step )
      couplingStep( engine, step );
   expect( engine.spatialIndexActive() ==
           ( scenario == Scenario::pile || scenario == Scenario::shake ),
           "the hash grid is active exactly for the pile and the shaker" );
   const std::string reference( fingerprint( engine ) );
   std::ofstream( dir + "/reference.txt" ) << reference;
   std::ofstream( dir + "/positions.txt" ) << positions( engine );

   // Reading a checkpoint back into the live world changes no bit (done after the reference was
   // recorded, so an inexact read-back cannot contaminate the reference of the restart test)
   std::ostringstream first, second;
   engine.writeCheckpoint( first );
   std::istringstream in( first.str() );
   engine.readCheckpoint( in );
   expect( fingerprint( engine ) == reference, "read-back of a checkpoint is exact" );
   engine.writeCheckpoint( second );
   expect( first.str() == second.str(), "a read-back checkpoint is written identically" );

   std::vector<pc::BodyState> states;
   engine.states( pc::Selection::movable, states );
   double minZ( 1.0 );
   for( const pc::BodyState& s : states ) minZ = std::min( minZ, s.position[2] - s.radius );
   std::printf( "%zu movable bodies, lowest gap to the plane %.3g\n", states.size(), minZ );
   expect( minZ < 1.0e-4, "bodies are in contact with the plane" );
   if( scenario == Scenario::basic ) {
      // The fixed sphere moves with its prescribed velocity (-0.05 m/s from x = 0.03 for
      // 0.4 s) only with MOBILE_INFINITE; without it pe ignores that velocity and it stays put
      double pusherX( 0.0 );
      for( const pc::BodyState& b : finiteStates( engine ) )
         if( b.fixed ) pusherX = b.position[0];
      std::printf( "fixed pusher at x = %.6g (MOBILE_INFINITE %s)\n", pusherX,
                   pc::buildInfo().mobileInfinite ? "on" : "off" );
      if( pc::buildInfo().mobileInfinite )
         expect( std::fabs( pusherX - 0.01 ) < 1.0e-6, "the fixed sphere moved with its velocity" );
      else
         expect( pusherX == 0.03, "without MOBILE_INFINITE the fixed sphere does not move" );
   }
   return coupling_test::report( "pe_coupling_checkpoint_test --write" );
}


static int runRead( const std::string& dir, Scenario scenario )
{
   pc::Engine engine;
   buildWorld( engine, scenario, false );
   if( refusedByBuild( engine ) )
      return coupling_test::report( "pe_coupling_checkpoint_test --read" );
   std::ifstream in( dir + "/checkpoint.txt" );
   expect( bool( in ), "checkpoint file present" );
   engine.readCheckpoint( in );
   auto fileText = [&]( const std::string& name ) {
      std::ifstream file( dir + "/" + name );
      std::stringstream text;
      text << file.rdbuf();
      return text.str();
   };
   const std::string fluid( fileText( "fluid.txt" ) );
   expect( !fluid.empty() && fluidHistory( engine ) == fluid,
           "pe's body-level fluid-force history is restored bit for bit" );
   std::string impulse;
   couplingStep( engine, stepsBefore, [&]() { impulse = fingerprint( engine ); } );
   for( int step=stepsBefore+1; step<stepsBefore+stepsAfter; ++step )
      couplingStep( engine, step );
   if( scenario != Scenario::shake && scenario != Scenario::history )
      expect( impulse == fileText( "impulse.txt" ),
              "the state right after the first impulse equals the uninterrupted run bit for "
              "bit" );

   std::ifstream refFile( dir + "/reference.txt" );
   std::stringstream ref;
   ref << refFile.rdbuf();
   const std::string restarted( fingerprint( engine ) );
   expect( !ref.str().empty(), "reference present" );

   if( scenario != Scenario::shake && scenario != Scenario::history ) {
      expect( restarted == ref.str(), "restarted run equals the uninterrupted run bit for bit" );
      if( restarted != ref.str() )
         std::printf( "reference:\n%s\nrestarted:\n%s\n", ref.str().c_str(), restarted.c_str() );
      return coupling_test::report( "pe_coupling_checkpoint_test --read" );
   }

   // Characterisation (shake, history): report whether the restart is bitwise and how far off
   std::ifstream posFile( dir + "/positions.txt" );
   std::map<pc::BodyId,std::pair<pc::Vec3d,pc::Vec3d>> refState;
   pc::BodyId id( 0 );
   pc::Vec3d p, v;
   while( posFile >> id >> p[0] >> p[1] >> p[2] >> v[0] >> v[1] >> v[2] ) refState[id] = { p, v };
   double maxPos( 0.0 ), maxVel( 0.0 );
   bool finite( true );
   for( const pc::BodyState& s : finiteStates( engine ) ) {
      const auto it( refState.find( s.id ) );
      expect( it != refState.end(), "every body of the restarted run is in the reference" );
      if( it == refState.end() ) continue;
      for( int k=0; k<3; ++k ) {
         const double dp( std::fabs( s.position[k] - it->second.first[k] ) );
         const double dv( std::fabs( s.linearVelocity[k] - it->second.second[k] ) );
         if( !std::isfinite( dp ) || !std::isfinite( dv ) ) finite = false;
         else { maxPos = std::max( maxPos, dp ); maxVel = std::max( maxVel, dv ); }
      }
   }
   const bool bitwise( restarted == ref.str() );
   std::printf( "%s restart: %s, largest deviation %.3g m in position, %.3g m/s in velocity\n",
                scenario == Scenario::shake ? "shake" : "history",
                bitwise ? "bit for bit" : "NOT bit for bit", maxPos, maxVel );
   expect( refState.size() == finiteStates( engine ).size(), "same bodies" );
   expect( finite, "the restarted and the reference states are finite" );
   // A characterisation of behaviour that is not guaranteed (see the file comment): only a
   // loose sanity bound, in the test's units (m, m/s)
   expect( maxPos < 1.0e-6 && maxVel < 1.0e-6,
           "the restart stays close to the run (loose sanity bound 1e-6)" );
   return coupling_test::report( "pe_coupling_checkpoint_test --read (characterisation)" );
}


//*************************************************************************************************
/*!\brief A numpunct facet with a decimal comma, so the test needs no installed locale.
 */
struct CommaDecimal : std::numpunct<char>
{
   char        do_decimal_point() const override { return ','; }
   char        do_thousands_sep() const override { return '.'; }
   std::string do_grouping()      const override { return "\3"; }
};
//*************************************************************************************************


static std::string replaceFirst( std::string s, const std::string& from, const std::string& to )
{
   const std::size_t pos( s.find( from ) );
   if( pos != std::string::npos ) s.replace( pos, from.size(), to );
   return s;
}


//*************************************************************************************************
/*!\brief Replaces token \a index (0-based) of the first line starting with \a prefix.
 */
static std::string replaceToken( const std::string& text, const std::string& prefix,
                                 std::size_t index, const std::string& value )
{
   std::istringstream lines( text );
   std::string line, out;
   bool done( false );
   while( std::getline( lines, line ) ) {
      if( !done && line.rfind( prefix, 0 ) == 0 ) {
         std::istringstream tok( line );
         std::vector<std::string> t;
         std::string x;
         while( tok >> x ) t.push_back( x );
         t.at( index ) = value;
         line.clear();
         for( std::size_t i=0; i<t.size(); ++i ) line += ( i ? " " : "" ) + t[i];
         done = true;
      }
      out += line + "\n";
   }
   return out;
}
//*************************************************************************************************


static int runErrors()
{
   pc::Engine engine;
   buildWorld( engine, Scenario::basic, true );
   if( refusedByBuild( engine ) )
      return coupling_test::report( "pe_coupling_checkpoint_test --errors" );
   for( int step=0; step<20; ++step ) couplingStep( engine, step );

   // The checkpoint under test, then a live state that differs from it in kinematics and in
   // load history; the refused reads are checked against that live state
   std::ostringstream good;
   engine.writeCheckpoint( good );
   const std::string text( good.str() );
   for( int step=20; step<30; ++step ) couplingStep( engine, step );
   std::ostringstream liveText;
   engine.writeCheckpoint( liveText );
   const std::string live( liveText.str() );
   expect( live != text, "the live state differs from the checkpoint under test" );
   std::printf( "checkpoint:\n%s", text.c_str() );

   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::movable, ids );
   const std::string id0( std::to_string( ids[0] ) ), id1( std::to_string( ids[1] ) );
   std::string fixedId;
   for( const pc::BodyState& b : finiteStates( engine ) )
      if( b.fixed ) fixedId = std::to_string( b.id );
   expect( !fixedId.empty() && text.find( "fixedbody " + fixedId + " " ) != std::string::npos,
           "the fixed finite body is in the checkpoint" );

   // A refused read leaves kinematics, load history and the pending loads as they were
   const pc::Vec3d pendingForce{ { 1.0e-3, 0.0, 0.0 } }, zero{ { 0.0, 0.0, 0.0 } };
   auto refused = [&]( const std::string& bad, const std::string& what, const char* fragment ) {
      engine.addLoad( ids[0], pendingForce, zero );
      std::istringstream in( bad );
      expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( in ); }, what, fragment );
      expect( engine.hasPendingLoads(), what + ": the pending load is still there" );
      engine.clearLoads();
      std::ostringstream now;
      engine.writeCheckpoint( now );
      expect( now.str() == live, what + ": kinematics and load history unchanged" );
   };

   refused( replaceFirst( text, "pe-coupling-checkpoint", "something-else" ), "wrong tag",
            "not a pe coupling checkpoint" );
   refused( replaceFirst( text, "pe-coupling-checkpoint 3", "pe-coupling-checkpoint 2" ),
            "version 2 (without pe's body-level fluid-force history)",
            "unsupported checkpoint version 2" );
   refused( replaceFirst( text, "pe-coupling-checkpoint 3", "pe-coupling-checkpoint 4" ),
            "version 4", "unsupported checkpoint version 4" );
   refused( replaceFirst( text, "bodies 3", "bodies 2" ), "count 2 in the checkpoint",
            "body count mismatch" );
   refused( replaceFirst( text, "fixed 1", "fixed 0" ), "fixed count 0 in the checkpoint",
            "fixed finite bodies" );
   refused( replaceFirst( text, "fixedbody " + fixedId + " ", "fixedbody " + id0 + " " ),
            "a movable body in the fixed section (late in the stream)", "fixed finite" );
   refused( replaceFirst( text, "body " + id0 + " ", "body " + fixedId + " " ),
            "a fixed body in the movable section", "movable" );
   refused( replaceFirst( text, "body " + id0 + " ", "body 999999 " ), "unknown id", "999999" );
   refused( replaceFirst( text, "body " + id1 + " ", "body " + id0 + " " ), "duplicate id",
            "twice" );
   refused( text.substr( 0, text.size() / 2 ), "truncated", "truncated" );
   refused( replaceFirst( text, "end", "" ), "missing end (the very last token)", "truncated" );
   // tokens: tag id x y z r i j k ...
   refused( replaceToken( text, "body ", 5, "2" ), "non-unit quaternion", "unit quaternion" );
   refused( replaceToken( text, "fixedbody ", 5, "2" ),
            "non-unit quaternion of the fixed body (late in the stream)", "unit quaternion" );
   refused( replaceToken( text, "body ", 2, "1e100" ),
            "a position where the body cannot be represented", "represented" );
   refused( replaceToken( text, "fixedbody ", 15, "7" ), "a history flag other than 0 or 1",
            "malformed" );
   // tokens 22..28: pe's body-level fluid-force history (flag, force, torque)
   refused( replaceToken( text, "body ", 22, "2" ),
            "a body-level fluid-force flag other than 0 or 1", "malformed" );
   refused( replaceToken( text, "body ", 23, "inf" ), "a non-finite body-level fluid force",
            "malformed" );

   // A fixed body with a velocity (a record a MOBILE_INFINITE build writes): a build without
   // MOBILE_INFINITE cannot restore it and refuses it before changing anything; a build with
   // it restores the velocity exactly. Tokens 9..11 linear, 12..14 angular velocity.
   // A -0 where the body holds +0 is refused as well (the body would keep +0).
   const std::vector< std::pair<std::size_t, std::string> > patches{
      { 9, "0.25" }, { 13, "0.25" }, { 9, "-0" } };
   for( const auto& patch : patches ) {
      const std::size_t token( patch.first );
      const std::string moving( replaceToken( text, "fixedbody ", token, patch.second ) );
      const std::string what( "a fixed body with " + std::string( token == 9 ? "linear" :
                              "angular" ) + " velocity component " + patch.second );
      if( !pc::buildInfo().mobileInfinite ) {
         refused( moving, what + ", without MOBILE_INFINITE", "MOBILE_INFINITE" );
      }
      else {
         std::istringstream in( moving );
         engine.readCheckpoint( in );
         const pc::BodyState st( engine.state( std::stoull( fixedId ) ) );
         const double v( token == 9 ? st.linearVelocity[0] : st.angularVelocity[1] );
         const double expected( std::stod( patch.second ) );
         expect( v == expected && std::signbit( v ) == std::signbit( expected ),
                 what + ", with MOBILE_INFINITE: restored exactly" );
         std::istringstream back( live );
         engine.readCheckpoint( back );
      }
   }

   // A body more in the world than in the checkpoint
   {
      const pc::BodyId extra( engine.createSphere( pc::Vec3d{ { 0.1, 0.1, 0.1 } }, 0.004, 0 ) );
      std::istringstream in( text );
      expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( in ); },
                                        "extra body in the world", "body count mismatch" );
      pe::destroy( *pe::theCollisionSystem()->getBodyStorage().find( extra ) );
   }

   // Worlds the checkpoint cannot restore: a union, a moving plane
   {
      const pe::UnionID u( pe::createTetrasphere( 77, pe::Vec3( 0.5, 0.5, 0.5 ), 0.01,
                                                  pe::granite ) );
      std::ostringstream out;
      expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( out ); },
                                        "write with a union in the world", "union" );
      expect( out.str().empty(), "the refused write wrote nothing" );
      std::istringstream in( text );
      expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( in ); },
                                        "read with a union in the world", "union" );
      pe::destroy( u );
   }
   {
      pe::BodyID plane( nullptr );
      for( auto it=pe::theCollisionSystem()->getBodyStorage().begin();
           it!=pe::theCollisionSystem()->getBodyStorage().end(); ++it )
         if( it->getType() == pe::planeType ) plane = *it;
      // 1e-9 lies below pe's vector comparison tolerance (1e-8): the check must be exact
      for( const double speed : { 0.01, 1.0e-9 } ) {
         for( const bool angular : { false, true } ) {
            char speedText[32];
            std::snprintf( speedText, sizeof(speedText), "%g", speed );
            const std::string what( std::string( angular ? "rotating" : "moving" ) +
                                    " plane at " + speedText );
            if( angular ) plane->setAngularVel( pe::Vec3( 0.0, speed, 0.0 ) );
            else          plane->setLinearVel( pe::Vec3( 0.0, 0.0, speed ) );
            std::ostringstream out;
            if( pc::buildInfo().mobileInfinite ) {
               expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( out ); },
                                                 "write with a " + what, "moves" );
               std::istringstream in( text );
               expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( in ); },
                                                 "read with a " + what, "moves" );
            }
            else {
               // Without MOBILE_INFINITE pe ignores the velocity of the fixed plane
               engine.writeCheckpoint( out );
               expect( plane->getLinearVel()[2] == 0.0 && plane->getAngularVel()[1] == 0.0,
                       "without MOBILE_INFINITE a plane cannot move, and the checkpoint is "
                       "written" );
            }
            plane->setLinearVel( pe::Vec3( 0.0, 0.0, 0.0 ) );
            plane->setAngularVel( pe::Vec3( 0.0, 0.0, 0.0 ) );
         }
      }
   }

   // Pending loads at write time
   engine.addLoad( ids[0], pc::Vec3d{ { 1.0, 0.0, 0.0 } }, pc::Vec3d{ { 0.0, 0.0, 0.0 } } );
   std::ostringstream ignored;
   expectThrows<std::logic_error>( [&]() { engine.writeCheckpoint( ignored ); },
                                   "write with pending loads", "pending" );
   engine.clearLoads();

   // The caller's stream settings play no role: an input stream left in hex and noskipws, with
   // a comma-decimal locale, and an output stream with a pending width and fill
   {
      std::istringstream in( text + "TRAILER 42\n" );
      in.imbue( std::locale( std::locale::classic(), new CommaDecimal ) );
      in >> std::hex >> std::noskipws >> std::setw( 3 );   // a pending width would cut "pe-"
      const std::ios_base::fmtflags flags( in.flags() );
      engine.readCheckpoint( in );
      expect( in.flags() == flags, "the stream's format flags are restored" );
      expect( in.width() == 3, "the stream's pending width is restored" );
      in.width( 0 );
      expect( std::use_facet< std::numpunct<char> >( in.getloc() ).decimal_point() == ',',
              "the stream's locale is restored" );
      std::string trailer;
      in >> std::skipws >> trailer;
      expect( trailer == "TRAILER", "data after the checkpoint stays in the stream" );

      std::ostringstream restored;
      restored.imbue( std::locale( std::locale::classic(), new CommaDecimal ) );
      // a width larger than the whole text: a formatted insertion would pad it
      restored << std::setw( int( text.size() ) + 16 ) << std::setfill( '*' ) << std::hex;
      engine.writeCheckpoint( restored );
      expect( restored.str() == text, "the restored state is the checkpoint's, and writing to a "
                                      "stream with width, fill, hex and a comma locale gives the "
                                      "same text" );
   }
   return coupling_test::report( "pe_coupling_checkpoint_test --errors" );
}


//*************************************************************************************************
/*!\brief Characterisation: does pe change the stored geometry of a tilted plane at rest in
 *        its first step? (A rebuilt world has the creation geometry.)
 */
static int runPlaneGeometry()
{
   pc::Engine engine;
   engine.setGravity( pc::Vec3d{ { 0.0, 0.0, -9.81 } } );
   const pc::MaterialId wall( engine.createMaterial( "tiltWall", 2500.0, 0.0, 0.25, 0.25 ) );
   const pc::BodyId id( engine.createPlane( pc::Vec3d{ { 1.0e-5, 0.0, 1.0 } }, 0.0, wall ) );
   engine.createSphere( pc::Vec3d{ { 5.0, 5.0, 1.0 } }, 0.1, wall );
   const pe::PlaneID plane( static_cast<pe::PlaneID>(
      *pe::theCollisionSystem()->getBodyStorage().find( id ) ) );
   const pe::Vec3 n0( plane->getNormal() );
   const pe::real d0( plane->getDisplacement() );
   engine.advance( 1.0e-3 );
   const pe::Vec3 n1( plane->getNormal() );
   const pe::real d1( plane->getDisplacement() );
   const bool same( bits( n0[0] ) == bits( n1[0] ) && bits( n0[1] ) == bits( n1[1] ) &&
                    bits( n0[2] ) == bits( n1[2] ) && bits( d0 ) == bits( d1 ) );
   std::printf( "tilted plane at rest: normal (%.17g, %.17g, %.17g) d %.17g before the first "
                "step, (%.17g, %.17g, %.17g) d %.17g after: %s\n", n0[0], n0[1], n0[2], d0,
                n1[0], n1[1], n1[2], d1, same ? "unchanged" : "CHANGED" );
   expect( std::isfinite( n1[0] ) && std::isfinite( n1[1] ) && std::isfinite( n1[2] ),
           "the plane normal stays finite" );
   return coupling_test::report( "pe_coupling_checkpoint_test --plane-geometry" );
}
//*************************************************************************************************


int main( int argc, char** argv )
{
   const std::string mode( argc > 1 ? argv[1] : "" );
   const std::string variant( argc > 3 ? argv[3] : "" );
   const Scenario scenario( variant == "--pile"    ? Scenario::pile
                          : variant == "--shake"   ? Scenario::shake
                          : variant == "--history" ? Scenario::history : Scenario::basic );
   if( mode == "--write" && argc > 2 ) return runWrite( argv[2], scenario );
   if( mode == "--read" && argc > 2 ) return runRead( argv[2], scenario );
   if( mode == "--errors" ) return runErrors();
   if( mode == "--plane-geometry" ) return runPlaneGeometry();
   std::printf( "usage: %s --write <dir> [--pile|--shake] | --read <dir> [--pile|--shake] | "
                "--errors\n", argv[0] );
   return 2;
}
