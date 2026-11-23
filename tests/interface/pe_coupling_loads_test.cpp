//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_loads_test.cpp
 *  \brief pe::coupling load accumulation, under-relaxation, clearing and the step guards
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
 *  The velocity right after applyLoads() is checked bit for bit against a reference computed
 *  here: v += (dt / m) F_eff with F_eff = r F_n + (1 - r) F_(n-1), the first load of a body
 *  unrelaxed, and the same for the angular velocity with the inverse inertia. Over five
 *  coupling steps:
 *    - loads added in several parts accumulate (step 1: three contributions);
 *    - body b gets its FIRST load only in step 2: it is applied unrelaxed although a already
 *      has a history (applyLoads() touches only bodies with a pending load);
 *    - a body without a pending load is not touched at all: a force set on it through a native
 *      pe call and its velocity survive applyLoads(), and its history is not changed (its next
 *      load is relaxed against its last real one, step 4);
 *    - a zero load added explicitly relaxes a body against its history (step 3);
 *    - clearLoads() discards the pending loads, F_(n-1) survives (step 4);
 *    - relaxation 1 applies F_n unchanged (step 5).
 *  Plus: applyLoads() without pending loads is a no-op; a second applyLoads() with pending
 *  loads before the next advance() throws std::logic_error and changes nothing; advance() with
 *  pending loads throws; invalid relaxation / step size / substeps throw; a load on a fixed body
 *  or plane throws; a load added for a body destroyed before applyLoads() makes applyLoads()
 *  throw without changing any velocity.
 *  The history of a body that is fixed for a while is kept (with and without an unrelated
 *  body, and across a checkpoint written while it is fixed, which also restores pe's body-level
 *  fluid-force history of the fixed body, cleared natively before the read); an overflowing
 *  accumulation is refused and leaves the previous sum usable (cancelled exactly, then a known load applied); a
 *  checkpoint between applyLoads() and advance() is refused, also by a new Engine handle, which
 *  also refuses a second applyLoads() until advance(); after a native reload of the world
 *  following applyLoads() the refusals stay until acknowledgeWorldReload(), which also drops
 *  pending loads and history (also pe's body-level fluid-force history); a rollback by
 *  readCheckpoint() restores pe's body-level history, so a -0 force on a -0 velocity stays -0
 *  as in the uninterrupted run; a native force or torque waiting in a
 *  slot blocks both checkpoint functions. Checkpoint parts branch on
 *  BuildInfo::checkpointsSupported.
 *  No gravity, no contacts; linked against the configured pe_static.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>
#include <pe/util/Checkpointer.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
using coupling_test::sameBits;
namespace pc = pe::coupling;


//*************************************************************************************************
/*!\brief Reference model of one body: the expected velocities and the load history.
 */
struct Reference
{
   pc::BodyId id;
   pc::Vec3d  v;
   pc::Vec3d  w;
   bool       hasPrev;
   pc::Vec3d  prevF;
   pc::Vec3d  prevT;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creates a reference for a new body.
 */
static Reference makeReference( pc::BodyId id )
{
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };
   return Reference{ id, zero, zero, false, zero, zero };
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Predicts the velocities after applyLoads() the way RigidBody::applyFluidForces does,
 *        starting from the body's current velocities; call it right before applyLoads().
 *
 * The reference restarts from pe's velocities in every step because a simulation step may
 * change the angular velocity of a sphere by rounding (the gyroscopic term (I w) x w is not
 * exactly zero in floating point), which is not what this test is about.
 */
static void referenceApply( Reference& ref, const pc::Vec3d& F, const pc::Vec3d& T, double r,
                            double dt )
{
   const pe::ConstBodyID body( *pe::theCollisionSystem()->getBodyStorage().find( ref.id ) );
   const pe::Vec3& v0( body->getLinearVel() );
   const pe::Vec3& w0( body->getAngularVel() );
   ref.v = pc::Vec3d{ { v0[0], v0[1], v0[2] } };
   ref.w = pc::Vec3d{ { w0[0], w0[1], w0[2] } };
   pc::Vec3d effF( F ), effT( T );
   if( ref.hasPrev ) {
      for( int k=0; k<3; ++k ) {
         effF[k] = r * F[k] + ( 1.0 - r ) * ref.prevF[k];
         effT[k] = r * T[k] + ( 1.0 - r ) * ref.prevT[k];
      }
   }
   ref.prevF   = F;
   ref.prevT   = T;
   ref.hasPrev = true;

   const pe::Vec3 f( effF[0], effF[1], effF[2] ), t( effT[0], effT[1], effT[2] );
   const pe::Vec3 v( pe::Vec3( ref.v[0], ref.v[1], ref.v[2] ) +
                     body->getInvMass() * pe::real( dt ) * f );
   const pe::Vec3 w( pe::Vec3( ref.w[0], ref.w[1], ref.w[2] ) +
                     pe::real( dt ) * ( body->getInvInertia() * t ) );
   ref.v = pc::Vec3d{ { v[0], v[1], v[2] } };
   ref.w = pc::Vec3d{ { w[0], w[1], w[2] } };
}
//*************************************************************************************************


static void checkAgainstReference( const pc::Engine& engine, const Reference& ref,
                                   const std::string& label )
{
   const pc::BodyState s( engine.state( ref.id ) );
   expect( sameBits( s.linearVelocity, ref.v ), label + ": linear velocity bit for bit" );
   expect( sameBits( s.angularVelocity, ref.w ), label + ": angular velocity bit for bit" );
}


static pe::BodyID peBody( pc::BodyId id )
{
   return *pe::theCollisionSystem()->getBodyStorage().find( id );
}


//*************************************************************************************************
/*!\brief The history of a body that is fixed for a while survives, whatever else exists.
 *
 * Bodies a and b get a load; b is fixed natively; a gets loads for a few steps; b is unfixed and
 * gets a load: it must be relaxed against its old history. Run with and without an unrelated
 * third body (the history used to be pruned depending on the number of bodies), and across a
 * checkpoint written while b was fixed and read by a new Engine (which starts without history).
 */
static void checkFixedHistory( bool thirdBody, bool viaCheckpoint )
{
   const std::string label( std::string( thirdBody ? "with" : "without" ) + " a third body" +
                            ( viaCheckpoint ? ", across a checkpoint" : "" ) );
   pe::theWorld()->clear();
   const double r( 0.4 ), dt( 1.0e-3 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };
   const pc::Vec3d Fb{ { 0.0, 2.0, 0.0 } }, Fb2{ { 0.0, -1.0, 3.0 } };
   std::stringstream ckpt;
   Reference b( makeReference( pc::noBody ) );
   {
      pc::Engine engine;
      const pc::BodyId a( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, 0 ) );
      b = makeReference( engine.createSphere( pc::Vec3d{ { 5.0, 0.0, 0.0 } }, 0.5, 0 ) );
      if( thirdBody ) engine.createSphere( pc::Vec3d{ { -5.0, 0.0, 0.0 } }, 0.5, 0 );

      engine.addLoad( a, pc::Vec3d{ { 1.0, 0.0, 0.0 } }, zero );
      engine.addLoad( b.id, Fb, zero );
      referenceApply( b, Fb, zero, r, dt );
      engine.applyLoads( r, dt );
      engine.advance( dt );

      peBody( b.id )->setFixed( true );
      for( int i=0; i<3; ++i ) {
         engine.addLoad( a, pc::Vec3d{ { 0.5, 0.0, 0.0 } }, zero );
         engine.applyLoads( r, dt );
         engine.advance( dt );
      }
      if( viaCheckpoint ) {
         engine.writeCheckpoint( ckpt );
         expect( ckpt.str().find( "fixedbody " + std::to_string( b.id ) + " " ) !=
                 std::string::npos, "fixed history: the fixed body is in the checkpoint" );
      }
      else {
         peBody( b.id )->setFixed( false );
         engine.addLoad( b.id, Fb2, zero );
         referenceApply( b, Fb2, zero, r, dt );
         engine.applyLoads( r, dt );
         checkAgainstReference( engine, b, "fixed history (" + label + "): relaxed against the "
                                "load before it was fixed" );
         engine.advance( dt );   // complete the coupling step
      }
   }
   if( !viaCheckpoint ) return;

   pc::Engine engine;   // starts without history
   // pe's body-level fluid-force history of the fixed body: cleared natively before the read,
   // so only the restore can bring it back
   const pe::BodyID fixedBody( peBody( b.id ) );
   const bool savedHas( fixedBody->hasPrevFluidForce() );
   const pe::Vec3 savedForce( fixedBody->getPrevFluidForce() );
   const pe::Vec3 savedTorque( fixedBody->getPrevFluidTorque() );
   expect( savedHas && savedForce[1] != 0.0,
           "fixed history: pe's body-level history of the fixed body is set" );
   fixedBody->setPrevFluidForce( false, pe::Vec3( 0, 0, 0 ), pe::Vec3( 0, 0, 0 ) );
   engine.readCheckpoint( ckpt );
   expect( fixedBody->isFixed(), "fixed history: still fixed after the read" );
   auto sameVec = [&]( const pe::Vec3& x, const pe::Vec3& y ) {
      return sameBits( pc::Vec3d{ { x[0], x[1], x[2] } }, pc::Vec3d{ { y[0], y[1], y[2] } } );
   };
   expect( fixedBody->hasPrevFluidForce() == savedHas &&
           sameVec( fixedBody->getPrevFluidForce(), savedForce ) &&
           sameVec( fixedBody->getPrevFluidTorque(), savedTorque ),
           "fixed history (" + label + "): pe's body-level history of the fixed body is "
           "restored bit for bit" );
   peBody( b.id )->setFixed( false );
   engine.addLoad( b.id, Fb2, zero );
   referenceApply( b, Fb2, zero, r, dt );
   engine.applyLoads( r, dt );
   checkAgainstReference( engine, b, "fixed history (" + label + "): relaxed against the load "
                          "before it was fixed" );
   engine.advance( dt );   // complete the coupling step
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overflowing accumulation is refused and leaves the earlier sum usable; a checkpoint
 *        between applyLoads() and advance() is refused.
 *
 * The large value is derived from pe::real, so the case is representable in the tested type.
 */
static void checkAccumulationAndCheckpointGuard()
{
   pe::theWorld()->clear();
   pc::Engine engine;
   const double dt( 1.0e-3 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };
   Reference a( makeReference( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, 0 ) ) );
   const double large( 0.75 * double( std::numeric_limits<pe::real>::max() ) );
   const pc::Vec3d big{ { large, 0.0, 0.0 } }, minusBig{ { -large, 0.0, 0.0 } };
   engine.addLoad( a.id, big, zero );
   expectThrows<std::invalid_argument>( [&]() { engine.addLoad( a.id, big, zero ); },
                                        "a force sum that overflows", "would not be finite" );
   // The refused contribution left the first one in place: cancel it exactly, add a small
   // known load, and the applied impulse is that of the small load alone
   engine.addLoad( a.id, minusBig, zero );
   const pc::Vec3d F{ { 2.0, -0.5, 0.25 } };
   engine.addLoad( a.id, F, zero );
   referenceApply( a, F, zero, 1.0, dt );
   engine.applyLoads( 1.0, dt );
   checkAgainstReference( engine, a, "after a refused overflowing contribution the earlier sum "
                          "is intact" );
   engine.advance( dt );

   engine.addLoad( a.id, zero, pc::Vec3d{ { 0.0, -large, 0.0 } } );
   expectThrows<std::invalid_argument>( [&]() {
      engine.addLoad( a.id, zero, pc::Vec3d{ { 0.0, -large, 0.0 } } ); },
      "a torque sum that overflows", "would not be finite" );
   engine.clearLoads();
   expectThrows<std::invalid_argument>( [&]() {
      engine.addLoad( a.id, pc::Vec3d{ { std::nan( "" ), 0.0, 0.0 } }, zero ); },
      "a NaN contribution" );

   // Between applyLoads() and advance() a checkpoint would reopen the coupling step
   engine.addLoad( a.id, F, zero );
   engine.applyLoads( 1.0, dt );
   std::stringstream ckpt;
   if( pc::buildInfo().checkpointsSupported )
      expectThrows<std::logic_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                      "checkpoint after applyLoads, before advance",
                                      "after advance()" );
   else
      expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                        "this build refuses checkpoints" );
   expect( ckpt.str().empty(), "the refused checkpoint wrote nothing" );
   engine.advance( dt );
   if( pc::buildInfo().checkpointsSupported ) {
      engine.writeCheckpoint( ckpt );
      expect( !ckpt.str().empty(), "after advance() the checkpoint is written" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The "loads applied, step not advanced" state survives a replacement of the handle.
 */
static void checkAppliedStateAcrossHandles()
{
   pe::theWorld()->clear();
   const double dt( 1.0e-3 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } }, F{ { 1.0, 0.0, 0.0 } };
   pc::BodyId id( pc::noBody );
   {
      pc::Engine engine;
      id = engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, 0 );
      engine.addLoad( id, F, zero );
      engine.applyLoads( 1.0, dt );
   }   // the impulse stays in the world, the handle goes
   pc::Engine engine;
   const pc::BodyState before( engine.state( id ) );
   engine.addLoad( id, F, zero );
   expectThrows<std::logic_error>( [&]() { engine.applyLoads( 1.0, dt ); },
                                   "a new handle refuses a second applyLoads() before advance()",
                                   "already applied" );
   expect( sameBits( engine.state( id ).linearVelocity, before.linearVelocity ),
           "the refused second application changed nothing" );
   engine.clearLoads();
   std::stringstream ckpt;
   if( pc::buildInfo().checkpointsSupported )
      expectThrows<std::logic_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                      "a new handle refuses a checkpoint before advance()",
                                      "after advance()" );
   engine.advance( dt );
   engine.addLoad( id, F, zero );
   engine.applyLoads( 1.0, dt );   // allowed again after advance()
   engine.advance( dt );
   if( pc::buildInfo().checkpointsSupported )
      engine.writeCheckpoint( ckpt );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A native reload of the world after applyLoads(): the marker of the interrupted step
 *        stays until acknowledgeWorldReload() declares the restored world to stand at a
 *        coupling-step boundary.
 *
 * Native pe checkpoint between steps, applyLoads(), the Engine destroyed, the earlier native
 * checkpoint restored (pe's reader destroys the impulsed body), a new Engine: without the call
 * a load and a coupling checkpoint are refused; after it the restored velocity is the saved
 * one, the first load is applied unrelaxed (no history) and a checkpoint can be written. A
 * kept Engine with history and pending loads loses both by the call.
 */
static void checkNativeReload()
{
   pe::theWorld()->clear();
   const double dt( 1.0e-3 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } }, F{ { 1.0, 0.0, 0.0 } }, G{ { 0.0, 3.0, 0.0 } };
   const std::string dir( "./_coupling_native_reload" ), name( "between_steps" );
   pc::BodyId id( pc::noBody );
   pc::BodyState saved;
   {
      pc::Engine engine;
      const pc::MaterialId mat( engine.createMaterial( "reload", 2.0, 0.1, 0.1, 0.1 ) );
      id = engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, mat );
      engine.addLoad( id, F, zero );
      engine.applyLoads( 1.0, dt );
      engine.advance( dt );
      saved = engine.state( id );
      pe::writeCheckpoint( dir, name );   // between steps
      engine.addLoad( id, F, zero );
      engine.applyLoads( 0.5, dt );       // the step is interrupted here
   }
   pe::readCheckpoint( dir, name );       // destroys the impulsed body
   pc::Engine engine;
   expect( engine.hasBody( id ) && sameBits( engine.state( id ).linearVelocity,
                                             saved.linearVelocity ),
           "native reload: the saved velocity is back" );
   std::stringstream ckpt;
   if( pc::buildInfo().checkpointsSupported )
      expectThrows<std::logic_error>( [&]() { engine.writeCheckpoint( ckpt ); },
                                      "native reload: without the call writeCheckpoint() is "
                                      "refused", "after advance()" );
   engine.addLoad( id, G, zero );
   expectThrows<std::logic_error>( [&]() { engine.applyLoads( 1.0, dt ); },
                                   "native reload: without the call the load is refused",
                                   "already applied" );
   expect( engine.hasPendingLoads(), "native reload: the refusals kept the pending load" );

   engine.acknowledgeWorldReload();
   expect( !engine.hasPendingLoads(), "native reload: the call drops the pending loads" );
   expect( sameBits( engine.state( id ).linearVelocity, saved.linearVelocity ),
           "native reload: the call does not touch the world" );
   if( pc::buildInfo().checkpointsSupported ) {
      engine.writeCheckpoint( ckpt );   // allowed: the world is at a step boundary
      ckpt.str( "" );
   }
   engine.addLoad( id, G, zero );
   engine.applyLoads( 0.25, dt );   // first load of the new Engine: unrelaxed
   const double expected( saved.linearVelocity[1] + peBody( id )->getInvMass() * pe::real( dt ) *
                          3.0 );
   expect( engine.state( id ).linearVelocity[1] == expected &&
           engine.state( id ).linearVelocity[0] == saved.linearVelocity[0],
           "native reload: the first load after the call is applied unrelaxed" );
   engine.advance( dt );

   // A kept Engine: history and pending loads go, and pe's body-level fluid-force history
   engine.addLoad( id, G, zero );
   expect( peBody( id )->hasPrevFluidForce(), "kept Engine: pe's body-level history is set" );
   engine.acknowledgeWorldReload();
   expect( !engine.hasPendingLoads(), "kept Engine: the call drops the pending loads" );
   expect( !peBody( id )->hasPrevFluidForce(),
           "kept Engine: the call clears pe's body-level fluid-force history" );
   const pc::Vec3d v0( engine.state( id ).linearVelocity );
   engine.addLoad( id, F, zero );
   engine.applyLoads( 0.25, dt );   // without history: F, not 0.25 F + 0.75 G
   const pc::Vec3d v1( engine.state( id ).linearVelocity );
   expect( v1[1] == v0[1] && v1[0] == v0[0] + peBody( id )->getInvMass() * pe::real( dt ) * 1.0,
           "kept Engine: the call drops the load history" );
   engine.advance( dt );
   pe::theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A rollback restores pe's body-level fluid-force history: a -0 force on a -0 velocity
 *        stays -0 after the rollback, as in the uninterrupted run.
 *
 * applyLoads() hands the effective load to pe's applyFluidForces( dt, 1 ), which adds
 * 0 * F_prev when the body has a previous fluid force. With a positive F_prev that turns a -0
 * force component into +0, and with it a -0 velocity component. Sequence: one sphere with
 * v_x = -0 before its first load, coupling checkpoint, a trial load F_x = 1 applied, the
 * checkpoint read back, a load F_x = -0 applied; the uninterrupted branch applies F_x = -0
 * directly. The velocity bits right after that impulse must agree.
 */
static void checkRollbackSignOfZero()
{
   if( !pc::buildInfo().checkpointsSupported ) return;
   const double dt( 1.0e-3 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };
   const pc::Vec3d minusZeroX{ { -0.0, 0.0, 0.0 } }, trial{ { 1.0, 0.0, 0.0 } };
   pc::Vec3d branch[2];
   for( const bool rollback : { false, true } ) {
      pe::theWorld()->clear();
      pc::Engine engine;
      const pc::BodyId id( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, 0 ) );
      pc::BodyState st( engine.state( id ) );
      engine.setKinematicState( id, pc::KinematicState{ st.position, st.orientation,
                                                        minusZeroX, zero } );
      if( rollback ) {
         std::stringstream ckpt;
         engine.writeCheckpoint( ckpt );
         engine.addLoad( id, trial, zero );
         engine.applyLoads( 1.0, dt );   // the trial impulse, then rolled back
         expect( peBody( id )->hasPrevFluidForce(), "rollback: the trial set pe's history" );
         engine.readCheckpoint( ckpt );
         expect( !peBody( id )->hasPrevFluidForce(),
                 "rollback: pe's body-level history is restored (none before the first load)" );
      }
      engine.addLoad( id, minusZeroX, zero );
      engine.applyLoads( 1.0, dt );
      branch[rollback] = engine.state( id ).linearVelocity;
      engine.advance( dt );
   }
   std::printf( "sign of zero: uninterrupted v_x %g (signbit %d), after rollback %g (%d)\n",
                branch[0][0], int( std::signbit( branch[0][0] ) ), branch[1][0],
                int( std::signbit( branch[1][0] ) ) );
   expect( std::signbit( branch[0][0] ), "uninterrupted: v_x stays -0" );
   expect( sameBits( branch[1], branch[0] ),
           "rollback: the velocity right after the impulse equals the uninterrupted one bit for "
           "bit" );
   pe::theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A force or torque waiting in a body's slot (set natively) blocks both checkpoint
 *        functions: the checkpoint does not hold it. One sphere, no contacts.
 */
static void checkNativeForceSlots()
{
   if( !pc::buildInfo().checkpointsSupported ) return;
   pe::theWorld()->clear();
   pc::Engine engine;
   const pc::BodyId id( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, 0 ) );
   std::stringstream good;
   engine.writeCheckpoint( good );
   const std::string text( good.str() );

   for( const bool torque : { false, true } ) {
      pe::BodyID body( peBody( id ) );
      if( torque ) body->setTorque( pe::Vec3( 0.0, 0.0, 1e-300 ) );
      else         body->setForce( pe::Vec3( 1e-300, 0.0, 0.0 ) );   // below pe's tolerance
      const std::string what( torque ? "a pending native torque" : "a pending native force" );
      std::ostringstream out;
      expectThrows<std::runtime_error>( [&]() { engine.writeCheckpoint( out ); },
                                        "write with " + what, "waiting for the next step" );
      expect( out.str().empty(), what + ": nothing written" );
      std::istringstream in( text );
      const pc::BodyState before( engine.state( id ) );
      expectThrows<std::runtime_error>( [&]() { engine.readCheckpoint( in ); },
                                        "read with " + what, "waiting for the next step" );
      expect( sameBits( engine.state( id ).position, before.position ), what + ": unchanged" );
      body->resetForce();
   }
   std::istringstream in( text );
   engine.readCheckpoint( in );   // fine again with empty slots
}
//*************************************************************************************************


static void checkBasics()
{
   pc::Engine engine;
   const pc::MaterialId mat( engine.createMaterial( "loadsMat", 1200.0, 0.0, 0.1, 0.1 ) );
   const double dt( 2.0e-3 );
   const double r( 0.65 );
   const pc::Vec3d zero{ { 0.0, 0.0, 0.0 } };

   Reference a( makeReference( engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, 0.5, mat ) ) );
   Reference b( makeReference( engine.createSphere( pc::Vec3d{ { 5.0, 0.0, 0.0 } }, 0.25, mat ) ) );
   const pc::BodyId plane( engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, -50.0, mat ) );
   pe::SphereID fixedSphere( pe::createSphere( 0, pe::Vec3( 0, 5, 0 ), 0.5, pe::granite ) );
   fixedSphere->setFixed( true );

   // No pending loads: applyLoads() is a no-op, also twice in a row
   engine.applyLoads( r, dt );
   engine.applyLoads( r, dt );
   expect( sameBits( engine.state( a.id ).linearVelocity, zero ), "no-op applyLoads" );

   // Step 1: a gets three contributions (its first load, unrelaxed); b gets nothing and has a
   // force set natively, which applyLoads() must leave alone
   {
      const pc::Vec3d F1{ { 0.3, -1.1, 2.5 } }, F2{ { -0.05, 0.4, 0.125 } };
      const pc::Vec3d F3{ { 1e-3, 0, -7e-2 } };
      const pc::Vec3d T1{ { 0.01, 0.02, -0.03 } }, T2{ { -0.004, 0.0, 0.5 } };
      engine.addLoad( a.id, F1, T1 );
      engine.addLoad( a.id, F2, T2 );
      engine.addLoad( a.id, F3, zero );
      expect( engine.hasPendingLoads(), "pending after addLoad" );
      pc::Vec3d Fa, Ta;
      for( int k=0; k<3; ++k ) {
         Fa[k] = F1[k];  Fa[k] += F2[k];  Fa[k] += F3[k];
         Ta[k] = T1[k];  Ta[k] += T2[k];  Ta[k] += zero[k];
      }
      peBody( b.id )->setLinearVel( pe::Vec3( 0.0, 0.5, 0.0 ) );
      peBody( b.id )->setForce( pe::Vec3( 7.0, 8.0, 9.0 ) );

      expectThrows<std::logic_error>( [&]() { engine.advance( dt ); }, "advance with pending loads",
                                      "applyLoads" );
      referenceApply( a, Fa, Ta, r, dt );
      engine.applyLoads( r, dt );
      expect( !engine.hasPendingLoads(), "applyLoads consumes the pending loads" );
      checkAgainstReference( engine, a, "step 1 a (accumulated, unrelaxed)" );
      expect( peBody( b.id )->getForce() == pe::Vec3( 7.0, 8.0, 9.0 ),
              "step 1 b (no load): the natively set force survives" );
      expect( sameBits( engine.state( b.id ).linearVelocity, pc::Vec3d{ { 0.0, 0.5, 0.0 } } ),
              "step 1 b (no load): the velocity is untouched" );

      // A second application in the same coupling step is refused and changes nothing
      engine.addLoad( a.id, F1, T1 );
      const pc::BodyState before( engine.state( a.id ) );
      expectThrows<std::logic_error>( [&]() { engine.applyLoads( r, dt ); },
                                      "second applyLoads in one coupling step", "already applied" );
      expect( sameBits( engine.state( a.id ).linearVelocity, before.linearVelocity ) &&
              engine.hasPendingLoads(), "the refused second applyLoads changed nothing" );
      engine.clearLoads();
      engine.applyLoads( r, dt );   // nothing pending: still a no-op in the same step
   }
   peBody( b.id )->resetForce();
   peBody( b.id )->setLinearVel( pe::Vec3( 0.0, 0.0, 0.0 ) );
   engine.advance( dt );   // no gravity, no contact: linear velocities unchanged
   expect( sameBits( engine.state( a.id ).linearVelocity, a.v ), "step 1 a after advance" );

   // Step 2: a relaxed against step 1; b's FIRST load, unrelaxed although a has a history
   {
      const pc::Vec3d Fa{ { 0.7, 0.2, -0.4 } }, Ta{ { 0.0, -0.05, 0.02 } };
      const pc::Vec3d Fb{ { 1.5, 1.5, -1.5 } }, Tb{ { 0.3, 0.0, 0.0 } };
      engine.addLoad( a.id, Fa, Ta );
      engine.addLoad( b.id, Fb, Tb );
      referenceApply( a, Fa, Ta, r, dt );
      referenceApply( b, Fb, Tb, r, dt );
      engine.applyLoads( r, dt );
      checkAgainstReference( engine, a, "step 2 a (relaxed)" );
      checkAgainstReference( engine, b, "step 2 b (first load, unrelaxed)" );
      engine.advance( dt );
   }

   // Step 3: a gets an explicit zero load (relaxed against its history); b gets nothing and is
   // not touched
   {
      const pc::BodyState bBefore( engine.state( b.id ) );
      engine.addLoad( a.id, zero, zero );
      referenceApply( a, zero, zero, r, dt );
      engine.applyLoads( r, dt );
      checkAgainstReference( engine, a, "step 3 a (explicit zero load, relaxed)" );
      const pc::BodyState bAfter( engine.state( b.id ) );
      expect( sameBits( bAfter.linearVelocity, bBefore.linearVelocity ) &&
              sameBits( bAfter.angularVelocity, bBefore.angularVelocity ),
              "step 3 b (no load): untouched" );
      engine.advance( dt );
   }

   // Step 4: clearLoads() discards the pending loads; b's history is still its step-2 load
   {
      engine.addLoad( a.id, pc::Vec3d{ { 100.0, 100.0, 100.0 } }, pc::Vec3d{ { 9.0, 9.0, 9.0 } } );
      engine.clearLoads();
      expect( !engine.hasPendingLoads(), "clearLoads clears" );
      engine.advance( dt );   // allowed again
      const pc::Vec3d Fa{ { -0.25, 0.5, 1.0 } }, Fb{ { 0.0, -3.0, 0.5 } };
      engine.addLoad( a.id, Fa, zero );
      engine.addLoad( b.id, Fb, zero );
      referenceApply( a, Fa, zero, r, dt );
      referenceApply( b, Fb, zero, r, dt );
      engine.applyLoads( r, dt );
      checkAgainstReference( engine, a, "step 4 a (after clear)" );
      checkAgainstReference( engine, b, "step 4 b (relaxed against its step-2 load)" );
      engine.advance( dt );
   }

   // Step 5: relaxation 1 applies F_n unchanged
   {
      const pc::Vec3d Fa{ { 0.125, -0.375, 2.0 } };
      engine.addLoad( a.id, Fa, zero );
      referenceApply( a, Fa, zero, 1.0, dt );
      engine.applyLoads( 1.0, dt );
      checkAgainstReference( engine, a, "step 5 a (relaxation 1)" );
      engine.advance( dt );
   }

   // Guards
   engine.addLoad( a.id, zero, zero );
   expectThrows<std::invalid_argument>( [&]() { engine.applyLoads( 0.0, dt ); }, "relaxation 0" );
   expectThrows<std::invalid_argument>( [&]() { engine.applyLoads( 1.5, dt ); }, "relaxation 1.5" );
   expectThrows<std::invalid_argument>( [&]() { engine.applyLoads( r, 0.0 ); }, "dt 0" );
   engine.clearLoads();
   expectThrows<std::invalid_argument>( [&]() { engine.advance( -dt ); }, "advance dt < 0" );
   expectThrows<std::invalid_argument>( [&]() { engine.advance( dt, 0 ); }, "zero substeps" );
   expectThrows<std::invalid_argument>( [&]() { engine.addLoad( plane, zero, zero ); },
                                        "load on a plane", "fixed or infinite" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.addLoad( fixedSphere->getSystemID(), zero, zero ); }, "load on a fixed body" );
   expectThrows<std::invalid_argument>( [&]() { engine.addLoad( pc::noBody, zero, zero ); },
                                        "load on an unknown body" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.addLoad( a.id, pc::Vec3d{ { 1.0, std::nan( "" ), 0.0 } }, zero ); }, "NaN load" );
   expect( !engine.hasPendingLoads(), "rejected loads are not pending" );

   // A load for a body destroyed before applyLoads(): refused, nothing changes
   {
      const pc::BodyId d( engine.createSphere( pc::Vec3d{ { 0.0, -5.0, 0.0 } }, 0.3, mat ) );
      engine.addLoad( d, pc::Vec3d{ { 1.0, 0.0, 0.0 } }, zero );
      engine.addLoad( a.id, pc::Vec3d{ { 1.0, 0.0, 0.0 } }, zero );
      pe::destroy( peBody( d ) );
      const pc::BodyState before( engine.state( a.id ) );
      expectThrows<std::invalid_argument>( [&]() { engine.applyLoads( r, dt ); },
                                           "applyLoads with a destroyed body", "no longer" );
      expect( sameBits( engine.state( a.id ).linearVelocity, before.linearVelocity ),
              "a refused applyLoads changes no velocity" );
      engine.clearLoads();
   }

   pe::theWorld()->clear();
}


int main()
{
   checkBasics();
   for( const bool third : { false, true } )
      for( const bool ckpt : { false, true } )
         if( !ckpt || pc::buildInfo().checkpointsSupported )
            checkFixedHistory( third, ckpt );
   checkAccumulationAndCheckpointGuard();
   checkAppliedStateAcrossHandles();
   checkNativeReload();
   checkRollbackSignOfZero();
   checkNativeForceSlots();

   return coupling_test::report( "pe_coupling_loads_test" );
}
