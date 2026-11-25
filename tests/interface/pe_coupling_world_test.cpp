//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_world_test.cpp
 *  \brief pe::coupling creation, enumeration, state, state override and id lookup
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
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 *
 *  Checked against pe's own accessors:
 *    1. bodies created through the API (sphere, plane) and through native pe calls (box,
 *       capsule, cylinder, ellipsoid, triangle mesh, union, a fixed sphere) are all enumerated
 *       with the right shape kind, user id, fixed flag, position, orientation, velocities, mass,
 *       radius and bounding box; Selection::movable holds exactly the finite unfixed bodies,
 *       Selection::all every body, Selection::shadowCopies none (serial);
 *    2. the batch form reuses the caller's vector (no reallocation once it is large enough);
 *    3. setKinematicState() overrides position, orientation and velocities exactly and updates
 *       the bounding box; invalid overrides throw, with pe's own quaternion tolerance;
 *    3a. BodyState::volume is pe's volume of each shape, 0 for planes and unions;
 *    4. id lookup stays correct after a body is added and after one is destroyed (pe swaps the
 *       last body into the gap, so storage positions change while ids must not);
 *    5. creation-time validation: non-finite material coefficients, plane normals pe cannot
 *       normalize, spheres whose bounding box collapses at their position are refused before
 *       pe registers anything (run in Debug: no pe assertion may fire);
 *    6. bodies created natively inside pe_GLOBAL_SECTION and ordinary ones get distinct ids
 *       without MPI too, and lookup and removal work for the mixture;
 *    7. a pe binary checkpoint with the counter layout of builds before that change (patched
 *       into a freshly written file) does not make the next bodies reuse a restored id;
 *    8. an ellipsoid (2, 1, 1) turned 90 degrees about z by setKinematicState() reports the
 *       rotated box before any step and after readCheckpoint(); the spatial index (active grid,
 *       refreshed) and the exhaustive method own the rotated tips; pe's other in-place
 *       orientation changes (setOrientation( Quat ), rotate()) update the box as well;
 *    9. a thin box (2 x 2e-5 x 2e-5) at x = 2^40 is accepted at its own pose and reads its own
 *       checkpoint (the position check works per axis), also one written in another
 *       orientation;
 *   10. pe's quaternion equality tells quaternions apart that differ in any one component;
 *   11. a non-convex, asymmetric triangle mesh turned from +45 to -45 degrees about z at a fixed
 *       centre reports the box of the new pose (bit for bit, before any step and after
 *       readCheckpoint()), a point of the newly occupied region (in the mesh's kernel, where
 *       pe's containment test without CGAL is valid) is owned with both methods, and a
 *       translation by 1e-9 (below pe's tolerance) renews the vertices.
 *  Serial world, no MPI; linked against the configured pe_static.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>
#include <pe/util/Checkpointer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
using coupling_test::sameBits;
namespace pc = pe::coupling;


//*************************************************************************************************
/*!\brief Compares a state snapshot with pe's own view of the body, bit for bit.
 */
static void checkAgainstPe( const pc::BodyState& s, const std::string& label )
{
   pe::ConstBodyID body( nullptr );
   for( auto it=pe::theCollisionSystem()->getBodyStorage().begin();
        it!=pe::theCollisionSystem()->getBodyStorage().end(); ++it )
      if( it->getSystemID() == s.id ) body = *it;
   expect( body != nullptr, label + ": body found in pe's storage" );
   if( body == nullptr ) return;

   const pe::Vec3& p( body->getPosition() );
   const pe::Vec3& v( body->getLinearVel() );
   const pe::Vec3& w( body->getAngularVel() );
   const pe::Quat& q( body->getQuaternion() );
   expect( sameBits( s.position, pc::Vec3d{ { p[0], p[1], p[2] } } ), label + ": position" );
   expect( sameBits( s.linearVelocity, pc::Vec3d{ { v[0], v[1], v[2] } } ), label + ": velocity" );
   expect( sameBits( s.angularVelocity, pc::Vec3d{ { w[0], w[1], w[2] } } ), label + ": omega" );
   expect( sameBits( s.orientation, pc::Quatd{ { q[0], q[1], q[2], q[3] } } ), label + ": quat" );
   expect( coupling_test::bits( s.mass ) == coupling_test::bits( body->getMass() ),
           label + ": mass" );
   expect( s.userId == body->getID(), label + ": user id" );
   expect( s.fixed == body->isFixed(), label + ": fixed" );
   expect( !s.shadow, label + ": not a shadow copy" );
   const pe::RigidBody::AABB& box( body->getAABB() );
   expect( sameBits( s.aabbMin, pc::Vec3d{ { box[0], box[1], box[2] } } ), label + ": aabb min" );
   expect( sameBits( s.aabbMax, pc::Vec3d{ { box[3], box[4], box[5] } } ), label + ": aabb max" );
}
//*************************************************************************************************


static void checkEnumeration( pc::Engine& engine )
{
   using namespace pe;

   const pc::MaterialId mat( engine.createMaterial( "worldTestMat", 2500.0, 0.0, 0.3, 0.2 ) );
   const pc::BodyId plane( engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 2.0 } }, -1.0, mat, 1 ) );
   const pc::BodyId sphere( engine.createSphere( pc::Vec3d{ { 1.0, 2.0, 3.0 } }, 0.5, mat, 2 ) );

   // Native bodies
   const BoxID       box( createBox( 3, Vec3( 4, 0, 0 ), Vec3( 1, 2, 3 ), granite ) );
   const CapsuleID   cap( createCapsule( 4, Vec3( 8, 0, 0 ), 0.5, 2.0, granite ) );
   const CylinderID  cyl( createCylinder( 5, Vec3( 12, 0, 0 ), 0.5, 2.0, granite ) );
   const EllipsoidID ell( createEllipsoid( 6, Vec3( 16, 0, 0 ), 1.0, 0.5, 0.25, granite ) );
   const TriangleMeshID mesh( createTriangulatedBox( 7, Vec3( 20, 0, 0 ), Vec3( 1, 1, 1 ),
                                                     granite ) );
   const UnionID     un( createTetrasphere( 8, Vec3( 24, 0, 0 ), 0.5, granite ) );
   const SphereID    fixedSphere( createSphere( 9, Vec3( 28, 0, 0 ), 0.75, granite ) );
   fixedSphere->setFixed( true );

   // Some motion, so velocities and orientation are not trivial
   box->setLinearVel( 0.1, -0.2, 0.3 );
   box->setAngularVel( 0.4, 0.5, -0.6 );
   box->rotate( 0.3, -0.2, 0.1 );
   ell->rotate( 0.5, 0.0, 0.25 );

   const std::map<pc::BodyId,pc::ShapeKind> kinds = {
      { plane, pc::ShapeKind::plane }, { sphere, pc::ShapeKind::sphere },
      { box->getSystemID(), pc::ShapeKind::box }, { cap->getSystemID(), pc::ShapeKind::capsule },
      { cyl->getSystemID(), pc::ShapeKind::cylinder },
      { ell->getSystemID(), pc::ShapeKind::ellipsoid },
      { mesh->getSystemID(), pc::ShapeKind::triangleMesh },
      { un->getSystemID(), pc::ShapeKind::other },
      { fixedSphere->getSystemID(), pc::ShapeKind::sphere } };

   std::vector<pc::BodyState> all;
   engine.states( pc::Selection::all, all );
   expect( all.size() == kinds.size(), "Selection::all holds every body" );
   expect( engine.count( pc::Selection::all ) == kinds.size(), "count(all)" );
   for( const pc::BodyState& s : all ) {
      const std::string label( "body " + std::to_string( s.id ) );
      const auto k( kinds.find( s.id ) );
      expect( k != kinds.end() && k->second == s.shape, label + ": shape kind" );
      checkAgainstPe( s, label );
   }

   std::vector<pc::BodyId> movable;
   engine.bodyIds( pc::Selection::movable, movable );
   const std::vector<pc::BodyId> expectedMovable = {
      sphere, box->getSystemID(), cap->getSystemID(), cyl->getSystemID(), ell->getSystemID(),
      mesh->getSystemID(), un->getSystemID() };
   expect( movable == expectedMovable, "Selection::movable: finite unfixed bodies, storage order" );
   expect( engine.count( pc::Selection::movable ) == expectedMovable.size(), "count(movable)" );

   std::vector<pc::BodyId> shadows( 3, 42 );
   engine.bodyIds( pc::Selection::shadowCopies, shadows );
   expect( shadows.empty(), "no shadow copies in a serial world" );

   // Specific values
   const pc::BodyState ps( engine.state( plane ) );
   expect( ps.fixed && ps.radius == 0.0, "the plane is fixed, radius 0" );
   const pc::BodyState ss( engine.state( sphere ) );
   expect( ss.radius == 0.5 && !ss.fixed && ss.userId == 2, "sphere radius, fixed, user id" );
   expect( ss.aabbMin[2] < 2.5 && ss.aabbMax[2] > 3.5, "sphere bounding box encloses it" );
   expect( engine.state( fixedSphere->getSystemID() ).radius == 0.75, "fixed sphere radius" );
   expect( engine.state( box->getSystemID() ).radius == 0.0, "radius 0 for a box" );
   expect( std::fabs( ss.mass - 2500.0 * 4.0 / 3.0 * pe::M_PI * 0.125 ) < 1e-9 * ss.mass,
           "sphere mass" );

   // Volumes: pe's per-shape value where it has one, 0 otherwise
   const double pi( pe::M_PI );
   const auto vol = [&]( pc::BodyId id ) { return engine.state( id ).volume; };
   const auto near = []( double a, double b ) {
      return std::fabs( a - b ) <= 1e-12 * std::fabs( b );
   };
   expect( near( vol( sphere ), 4.0 / 3.0 * pi * 0.125 ), "volume: sphere" );
   expect( near( vol( box->getSystemID() ), 6.0 ), "volume: box" );
   expect( near( vol( cap->getSystemID() ), pi * 0.25 * 2.0 + 4.0 / 3.0 * pi * 0.125 ),
           "volume: capsule" );
   expect( near( vol( cyl->getSystemID() ), pi * 0.25 * 2.0 ), "volume: cylinder" );
   expect( near( vol( ell->getSystemID() ), 4.0 / 3.0 * pi * 1.0 * 0.5 * 0.25 ),
           "volume: ellipsoid" );
   expect( near( vol( mesh->getSystemID() ), 1.0 ), "volume: triangle mesh (unit box)" );
   expect( vol( plane ) == 0.0, "volume: plane reports 0" );
   expect( vol( un->getSystemID() ) == 0.0, "volume: union reports 0" );
   std::vector<pc::BodyState> again;
   engine.states( pc::Selection::all, again );
   for( const pc::BodyState& b : again )
      if( b.id == mesh->getSystemID() )
         expect( coupling_test::bits( b.volume ) ==
                 coupling_test::bits( double( mesh->getVolume() ) ),
                 "volume: the cached mesh volume equals pe's" );
   expect( near( vol( sphere ), double( static_cast<SphereID>( *pe::theCollisionSystem()
           ->getBodyStorage().find( sphere ) )->getVolume() ) ), "volume: pe's sphere value" );

   // Plane convention: normal normalized by pe, offset along it
   const PlaneID pePlane(
      static_cast<PlaneID>( *pe::theCollisionSystem()->getBodyStorage().find( plane ) ) );
   expect( pePlane->getNormal() == Vec3( 0, 0, 1 ) && pePlane->getDisplacement() == real( -1 ),
           "plane: unit normal (0,0,1), offset -1" );

   // Batch form: the vector keeps its storage once it is large enough
   std::vector<pc::BodyState> reused;
   reused.reserve( 16 );
   const pc::BodyState* data( reused.data() );
   for( int i=0; i<3; ++i ) {
      engine.states( pc::Selection::movable, reused );
      expect( reused.data() == data && reused.size() == expectedMovable.size(),
              "states() reuses the caller's vector" );
   }

   // Unknown ids
   expectThrows<std::invalid_argument>( [&]() { engine.state( pc::noBody ); }, "state(noBody)" );
   expect( !engine.hasBody( pc::noBody ), "hasBody(noBody)" );
   expectThrows<std::invalid_argument>( [&]() { engine.createSphere( pc::Vec3d{ { 0, 0, 0 } }, 1.0,
                                                                     pe::Material::count() ); },
                                        "createSphere with an invalid material" );
}


static void checkStateOverride( pc::Engine& engine )
{
   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::movable, ids );
   const pc::BodyId id( ids.at( 1 ) );   // the box

   pc::KinematicState k;
   k.position        = pc::Vec3d{ { -3.0, 1.25, 7.5 } };
   const double c( std::cos( 0.35 ) ), s( std::sin( 0.35 ) );
   k.orientation     = pc::Quatd{ { c, 0.0, s, 0.0 } };
   k.linearVelocity  = pc::Vec3d{ { 1.0e-3, -2.0, 0.125 } };
   k.angularVelocity = pc::Vec3d{ { -0.5, 0.25, 3.0 } };
   engine.setKinematicState( id, k );

   const pc::BodyState st( engine.state( id ) );
   expect( sameBits( st.position, k.position ), "override: position" );
   expect( sameBits( st.orientation, k.orientation ), "override: orientation" );
   expect( sameBits( st.linearVelocity, k.linearVelocity ), "override: velocity" );
   expect( sameBits( st.angularVelocity, k.angularVelocity ), "override: omega" );
   expect( st.aabbMin[0] < -3.0 && st.aabbMax[0] > -3.0 && st.aabbMin[2] < 7.5 &&
           st.aabbMax[2] > 7.5, "override: bounding box follows" );
   checkAgainstPe( st, "override" );

   pc::KinematicState bad( k );
   bad.orientation = pc::Quatd{ { 1.0, 1.0, 0.0, 0.0 } };
   expectThrows<std::invalid_argument>( [&]() { engine.setKinematicState( id, bad ); },
                                        "non-unit quaternion", "unit quaternion" );
   // pe's own tolerance: |norm^2 - 1| < 1e-8 (double); 5e-9 is accepted, 5e-8 refused
   bad.orientation = pc::Quatd{ { std::sqrt( 1.0 + 5.0e-9 ), 0.0, 0.0, 0.0 } };
   engine.setKinematicState( id, bad );
   expect( sameBits( engine.state( id ).orientation, bad.orientation ),
           "a quaternion within pe's accuracy is accepted unchanged (not normalized)" );
   bad.orientation = pc::Quatd{ { std::sqrt( 1.0 + 5.0e-8 ), 0.0, 0.0, 0.0 } };
   expectThrows<std::invalid_argument>( [&]() { engine.setKinematicState( id, bad ); },
                                        "quaternion just outside pe's accuracy",
                                        "unit quaternion" );
   engine.setKinematicState( id, k );
   bad = k;
   bad.position[1] = std::nan( "" );
   expectThrows<std::invalid_argument>( [&]() { engine.setKinematicState( id, bad ); },
                                        "non-finite state" );
   std::vector<pc::BodyId> all;
   engine.bodyIds( pc::Selection::all, all );
   expectThrows<std::invalid_argument>( [&]() { engine.setKinematicState( all.at( 0 ), k ); },
                                        "override of a plane", "infinite" );
   expect( sameBits( engine.state( id ).position, k.position ), "failed overrides change nothing" );
}


static void checkLookupAfterAddAndDestroy( pc::Engine& engine )
{
   using namespace pe;

   // Remember every body's position by id
   std::vector<pc::BodyState> before;
   engine.states( pc::Selection::all, before );

   // Add bodies natively and through the API
   const SphereID added( createSphere( 100, Vec3( -10, -10, 10 ), 0.3, granite ) );
   const pc::BodyId apiAdded( engine.createSphere( pc::Vec3d{ { -20.0, 0.0, 0.0 } }, 0.4, 0,
                                                   101 ) );
   expect( engine.hasBody( added->getSystemID() ), "natively added body is found" );
   expect( engine.state( added->getSystemID() ).userId == 100, "natively added body: right body" );
   expect( engine.state( apiAdded ).userId == 101, "API added body: right body" );

   // Destroy a body in the middle of the storage: pe moves the last body into its slot
   const pc::BodyId victim( before.at( 2 ).id );
   pe::BodyID victimBody( *theCollisionSystem()->getBodyStorage().find( victim ) );
   destroy( victimBody );
   expect( !engine.hasBody( victim ), "destroyed body is gone" );
   expectThrows<std::invalid_argument>( [&]() { engine.state( victim ); }, "state(destroyed)" );

   for( const pc::BodyState& b : before ) {
      if( b.id == victim ) continue;
      const pc::BodyState now( engine.state( b.id ) );
      expect( now.id == b.id && now.userId == b.userId && sameBits( now.position, b.position ),
              "lookup after destroy: body " + std::to_string( b.id ) );
   }
   const pc::BodyState moved( engine.state( apiAdded ) );
   expect( moved.userId == 101 && moved.position[0] == -20.0,
           "the swapped-in body is still found" );

   // Loads and stepping still work with the changed storage
   engine.addLoad( apiAdded, pc::Vec3d{ { 1.0, 0.0, 0.0 } }, pc::Vec3d{ { 0.0, 0.0, 0.0 } } );
   engine.applyLoads( 1.0, 1.0e-3 );
   expect( engine.state( apiAdded ).linearVelocity[0] > 0.0, "load applied after the removal" );
   engine.advance( 1.0e-3 );
}


//*************************************************************************************************
/*!\brief Creation-time validation: nothing invalid reaches pe (run in Debug, pe asserts would
 *        fire otherwise), and nothing is registered for a refused call.
 */
static void checkCreationValidation( pc::Engine& engine )
{
   const double nan( std::nan( "" ) ), inf( HUGE_VAL );
   const std::size_t materials( pe::Material::count() );
   for( const double bad : { nan, inf } ) {
      expectThrows<std::invalid_argument>( [&]() { engine.createMaterial( "badDensity", bad, 0.0,
                                                                          0.1, 0.1 ); },
                                           "material with a non-finite density", "not finite" );
      expectThrows<std::invalid_argument>( [&]() { engine.createMaterial( "badCor", 1.0, bad, 0.1,
                                                                          0.1 ); },
                                           "material with a non-finite restitution" );
      expectThrows<std::invalid_argument>( [&]() { engine.createMaterial( "badFriction", 1.0, 0.0,
                                                                          bad, 0.1 ); },
                                           "material with a non-finite friction" );
   }
   expect( pe::Material::count() == materials, "no refused material was registered" );

   const pc::MaterialId mat( engine.createMaterial( "validationMat", 1000.0, 0.0, 0.1, 0.1 ) );
   const std::size_t bodies( engine.count( pc::Selection::all ) );
   for( const pc::Vec3d& n : { pc::Vec3d{ { 1e300, 0.0, 0.0 } }, pc::Vec3d{ { 1e-200, 0.0, 0.0 } },
                               pc::Vec3d{ { 0.0, 0.0, 0.0 } }, pc::Vec3d{ { nan, 0.0, 1.0 } } } )
      expectThrows<std::invalid_argument>( [&]() { engine.createPlane( n, 0.0, mat ); },
                                           "plane with an unnormalizable normal" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, inf, mat ); }, "plane with inf offset" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.createSphere( pc::Vec3d{ { 1e100, 0.0, 0.0 } }, 1.0, mat ); },
      "a unit sphere at x = 1e100 (bounding box collapses)", "represented" );
   {
      const double huge( 0.6 * std::numeric_limits<double>::max() );   // centre + radius overflows
      expectThrows<std::invalid_argument>( [&]() {
         engine.createSphere( pc::Vec3d{ { huge, 0.0, 0.0 } }, huge, mat ); },
         "a sphere whose bounding box overflows to infinity", "represented" );
   }
   expectThrows<std::invalid_argument>( [&]() {
      engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, -1.0, mat ); }, "negative radius" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.0 } }, nan, mat ); }, "NaN radius" );
   expect( engine.count( pc::Selection::all ) == bodies, "no refused body was registered" );

   // A valid normal reaches pe unchanged (pe normalizes it)
   const pc::BodyId plane( engine.createPlane( pc::Vec3d{ { 0.0, 3.0, 4.0 } }, -100.0, mat ) );
   const pe::PlaneID pePlane(
      static_cast<pe::PlaneID>( *pe::theCollisionSystem()->getBodyStorage().find( plane ) ) );
   expect( std::fabs( pePlane->getNormal()[1] - 0.6 ) < 1e-15 &&
           std::fabs( pePlane->getNormal()[2] - 0.8 ) < 1e-15,
           "plane normal (0, 3, 4) -> (0, 0.6, 0.8)" );

   // An override to where the body cannot be represented is refused
   const pc::BodyId s( engine.createSphere( pc::Vec3d{ { 0.0, 40.0, 0.0 } }, 1.0, mat ) );
   const pc::BodyState st( engine.state( s ) );
   expectThrows<std::invalid_argument>( [&]() {
      engine.setKinematicState( s, pc::KinematicState{ pc::Vec3d{ { 0.0, 1e100, 0.0 } },
                                                       st.orientation, st.linearVelocity,
                                                       st.angularVelocity } ); },
      "override to x = 1e100", "represented" );
   expect( sameBits( engine.state( s ).position, st.position ),
           "the refused override changed nothing" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global and local bodies get distinct ids also without MPI (pe's UniqueID used two
 *        counters that both started at 0), and lookup and removal work for both.
 */
static void checkGlobalAndLocalIds( pc::Engine& engine )
{
   using namespace pe;
   pe::theWorld()->clear();
   SphereID global0( nullptr ), global1( nullptr );
   const SphereID local0( createSphere( 50, Vec3( 0, 0, 10 ), 0.5, granite ) );
   pe_GLOBAL_SECTION {
      global0 = createSphere( 51, Vec3( 3, 0, 10 ), 0.5, granite );
      global1 = createSphere( 52, Vec3( 6, 0, 10 ), 0.5, granite );
   }
   const SphereID local1( createSphere( 53, Vec3( 9, 0, 10 ), 0.5, granite ) );
   expect( global0->isGlobal() && global1->isGlobal() && !local0->isGlobal(), "global flags" );

   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::all, ids );
   std::vector<pc::BodyId> sorted( ids );
   std::sort( sorted.begin(), sorted.end() );
   expect( ids.size() == 4 && std::unique( sorted.begin(), sorted.end() ) == sorted.end(),
           "four distinct ids for two local and two global bodies" );
   for( const SphereID b : { local0, global0, global1, local1 } )
      expect( engine.state( b->getSystemID() ).userId == b->getID(), "lookup of mixed body " +
              std::to_string( b->getID() ) );

   const pc::BodyId gone( global0->getSystemID() );
   destroy( global0 );
   expect( !engine.hasBody( gone ), "destroyed global body is gone" );
   for( const SphereID b : { local0, global1, local1 } )
      expect( engine.state( b->getSystemID() ).userId == b->getID(),
              "lookup after removing a global body" );
   pe::theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads / writes an unsigned value at a byte offset in pe's buffer byte order
 *        (big-endian, as pe's checkpoint buffers store it).
 */
template< typename T >
static T peekValue( const std::string& bytes, std::size_t pos )
{
   T value( 0 );
   for( std::size_t i=0; i<sizeof(T); ++i )
      value = static_cast<T>( ( value << 8 ) | static_cast<unsigned char>( bytes[pos+i] ) );
   return value;
}

template< typename T >
static void pokeValue( std::string& bytes, std::size_t pos, T value )
{
   for( std::size_t i=0; i<sizeof(T); ++i )
      bytes[pos+sizeof(T)-1-i] = static_cast<char>( ( value >> ( 8*i ) ) & 0xff );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A pe binary checkpoint with the counter layout of builds before the shared-counter
 *        change: after reading it, new bodies must not reuse a restored id.
 *
 * Before the change, global and local ids came from two counters, so a file holding a global
 * body with id g could carry a local counter of g and a global counter of g + 1. The test
 * writes a checkpoint with one global sphere with the current build, checks the two counters
 * where pe's reader takes them from (the first value of the global and of the local chunk;
 * header offsets at bytes 13 and 17, everything big-endian),
 * patches them into that old layout, reads the file, creates a global and a local sphere and
 * checks that all ids are distinct and found. Without the reader's fix the next body gets g.
 */
static void checkOldCounterLayout()
{
   using namespace pe;
   theWorld()->clear();
   SphereID global( nullptr );
   pe_GLOBAL_SECTION {
      global = createSphere( 60, Vec3( 0, 0, 20 ), 0.5, granite );
   }
   const pe::id_t g( global->getSystemID() );
   const std::string dir( "./_coupling_old_counter_layout" ), name( "old" );
   pe::writeCheckpoint( dir, name );

   const std::string file( dir + "/" + name + ".peb" );
   std::string bytes;
   {
      std::ifstream in( file, std::ios::binary );
      bytes.assign( std::istreambuf_iterator<char>( in ), std::istreambuf_iterator<char>() );
   }
   expect( bytes.size() > 21 && bytes[0] == 'P' && bytes[1] == 'E', "old layout: a pe file" );
   const std::uint32_t offsetGlobal( peekValue<std::uint32_t>( bytes, 13 ) );
   const std::uint32_t offsetLocal( peekValue<std::uint32_t>( bytes, 17 ) );
   const bool layoutKnown( offsetLocal + 8 <= bytes.size() && offsetGlobal + 8 <= bytes.size() &&
                           peekValue<pe::id_t>( bytes, offsetLocal ) == g + 1 &&
                           peekValue<pe::id_t>( bytes, offsetGlobal ) == 0 );
   expect( layoutKnown, "old layout: counters found where the reader takes them (local g + 1, "
                        "global 0 with the shared counter)" );
   if( !layoutKnown ) return;
   pokeValue<pe::id_t>( bytes, offsetLocal, g );
   pokeValue<pe::id_t>( bytes, offsetGlobal, g + 1 );
   {
      std::ofstream out( file, std::ios::binary | std::ios::trunc );
      out.write( bytes.data(), static_cast<std::streamsize>( bytes.size() ) );
   }

   pe::readCheckpoint( dir, name );
   expect( theWorld()->size() == 1, "old layout: the global sphere is restored" );

   pc::Engine engine;   // a new Engine after the native reload
   SphereID global2( nullptr );
   pe_GLOBAL_SECTION {
      global2 = createSphere( 61, Vec3( 3, 0, 20 ), 0.5, granite );
   }
   const SphereID local( createSphere( 62, Vec3( 6, 0, 20 ), 0.5, granite ) );
   std::vector<pc::BodyId> ids;
   engine.bodyIds( pc::Selection::all, ids );
   std::sort( ids.begin(), ids.end() );
   std::printf( "old layout: restored id %llu, new global id %llu, new local id %llu\n",
                (unsigned long long)g, (unsigned long long)global2->getSystemID(),
                (unsigned long long)local->getSystemID() );
   expect( ids.size() == 3 && std::unique( ids.begin(), ids.end() ) == ids.end(),
           "old layout: three distinct ids after the read" );
   expect( engine.state( g ).userId == 60 &&
           engine.state( global2->getSystemID() ).userId == 61 &&
           engine.state( local->getSystemID() ).userId == 62,
           "old layout: lookups find each body" );
   theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A rotated ellipsoid reports, and is filed with, the bounding box of its new
 *        orientation right after setKinematicState() and readCheckpoint() (pe's ellipsoid
 *        orientation setters used to keep the old box).
 */
static void checkEllipsoidBox()
{
   using namespace pe;
   theWorld()->clear();
   pc::Engine engine;
   // Enough bodies for an active hash grid, far away
   for( int i=0; i<40; ++i )
      createSphere( 70 + i, Vec3( -50.0 - 2.0*i, 0, 0 ), 0.5, granite );
   const EllipsoidID ell( createEllipsoid( 69, Vec3( 30, 0, 0 ), 2.0, 1.0, 1.0, granite ) );
   const pc::BodyId id( ell->getSystemID() );
   engine.advance( 1.0e-6 );
   expect( engine.spatialIndexActive(), "ellipsoid: the hash grid is active" );

   const double c( std::cos( pe::M_PI / 4.0 ) ), sn( std::sin( pe::M_PI / 4.0 ) );
   const pc::Vec3d still{ { 0.0, 0.0, 0.0 } };
   const pc::KinematicState rotated{ pc::Vec3d{ { 30.0, 0.0, 0.0 } },
                                     pc::Quatd{ { c, 0.0, 0.0, sn } },   // 90 degrees about z
                                     still, still };
   const pc::KinematicState plain{ rotated.position, pc::Quatd{ { 1.0, 0.0, 0.0, 0.0 } },
                                   rotated.linearVelocity, rotated.angularVelocity };
   auto halfExtents = [&]() {
      const pc::BodyState st( engine.state( id ) );
      return pc::Vec3d{ { ( st.aabbMax[0] - st.aabbMin[0] ) / 2.0,
                          ( st.aabbMax[1] - st.aabbMin[1] ) / 2.0,
                          ( st.aabbMax[2] - st.aabbMin[2] ) / 2.0 } };
   };
   auto rotatedBox = [&]( const std::string& when ) {
      const pc::Vec3d h( halfExtents() );
      std::printf( "ellipsoid box %s: half extents (%.6g, %.6g, %.6g)\n", when.c_str(), h[0], h[1],
                   h[2] );
      expect( h[0] < 1.1 && h[1] > 1.9 && h[1] < 2.1, "ellipsoid: rotated box " + when );
   };
   const pc::Vec3d h0( halfExtents() );
   expect( h0[0] > 1.9 && h0[1] < 1.1, "ellipsoid: unrotated box (2, 1, 1)" );

   engine.setKinematicState( id, rotated );
   rotatedBox( "after setKinematicState(), before any step" );
   engine.refreshSpatialIndex();
   for( const pc::Vec3d& tip : { pc::Vec3d{ { 30.0, 1.9, 0.0 } },
                                 pc::Vec3d{ { 30.0, -1.9, 0.0 } } } )
      for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive,
                                       pc::QueryMethod::spatialIndex } )
         expect( engine.owner( tip, m ) == id,
                 "ellipsoid: the rotated tip is owned (both methods)" );
   expect( engine.owner( pc::Vec3d{ { 31.9, 0.0, 0.0 } }, pc::QueryMethod::spatialIndex ) ==
           pc::noBody, "ellipsoid: the old tip is fluid" );

   if( pc::buildInfo().checkpointsSupported ) {
      std::stringstream ckpt;
      engine.writeCheckpoint( ckpt );
      engine.setKinematicState( id, plain );
      expect( halfExtents()[0] > 1.9, "ellipsoid: unrotated again" );
      engine.readCheckpoint( ckpt );
      rotatedBox( "after readCheckpoint()" );
   }

   // pe's other in-place orientation changes of the ellipsoid (not used by the API), each from
   // the unrotated pose
   const Vec3 z( 0, 0, 1 );
   const std::vector< std::pair< std::string, std::function<void()> > > turns{
      { "setOrientation( Quat )", [&]() { ell->setOrientation( Quat( z, pe::M_PI / 2.0 ) ); } },
      { "rotate( axis, angle )",  [&]() { ell->rotate( z, pe::M_PI / 2.0 ); } },
      { "rotate( Quat )",         [&]() { ell->rotate( Quat( z, pe::M_PI / 2.0 ) ); } },
      { "rotate( x, y, z )",      [&]() { ell->rotate( 0.0, 0.0, pe::M_PI / 2.0 ); } } };
   for( const auto& turn : turns ) {
      engine.setKinematicState( id, plain );
      turn.second();
      rotatedBox( "after " + turn.first );
   }
   theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief pe's quaternion equality compares all four components (it used to compare component 2
 *        twice and never component 3).
 */
static void checkQuaternionEquality()
{
   using namespace pe;
   const Quat a( 0.5, 0.5, 0.5, 0.5 );
   expect( a == a && !( a != a ), "quaternion: equal to itself" );
   for( int k=0; k<4; ++k ) {
      real c[4] = { 0.5, 0.5, 0.5, 0.5 };
      c[k] = -0.5;
      const Quat b( c[0], c[1], c[2], c[3] );
      expect( !( a == b ) && a != b,
              "quaternion: differs in component " + std::to_string( k ) + " only" );
   }
   const Vec3 z( 0, 0, 1 );
   expect( Quat( z, pe::M_PI / 4.0 ) != Quat( z, -pe::M_PI / 4.0 ),
           "quaternion: +45 and -45 degrees about z differ" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A non-convex triangle mesh turned about z at a fixed centre reports the bounds of its
 *        new pose and is classified there (pe's vertex cache used to stay stale).
 *
 * The mesh: a right-triangle prism (legs 6 in x, 2 in y, height 2) whose top face has a dent
 * (an interior vertex lowered by 0.05), created natively with convex=false; asymmetric, so its
 * box at +45 and at -45 degrees about z differ. pe's containment test without CGAL ("behind
 * every face plane") is valid only for points in the kernel of the mesh; the test point is
 * checked to satisfy that predicate in the body frame and lies inside the prism by
 * construction, so the expected owner is right with or without CGAL. It is chosen near the
 * acute tip, where the -45 pose occupies space outside the +45 box.
 */
static void checkMeshRotation()
{
   using namespace pe;
   theWorld()->clear();
   pc::Engine engine;
   for( int i=0; i<40; ++i )
      createSphere( 90 + i, Vec3( -50.0 - 2.0*i, 0, 0 ), 0.5, granite );

   const real eps( 0.05 );
   const Vertices orig{ Vec3( 0, 0, -1 ), Vec3( 6, 0, -1 ), Vec3( 0, 2, -1 ),
                        Vec3( 0, 0,  1 ), Vec3( 6, 0,  1 ), Vec3( 0, 2,  1 ),
                        Vec3( 2, 2.0/3.0, 1 - eps ) };
   typedef Vector3<size_t> F;
   const IndicesLists faces{ F( 0, 2, 1 ),                              // bottom
                             F( 3, 4, 6 ), F( 4, 5, 6 ), F( 5, 3, 6 ),  // dented top
                             F( 0, 1, 4 ), F( 0, 4, 3 ),                // y = 0
                             F( 0, 3, 5 ), F( 0, 5, 2 ),                // x = 0
                             F( 1, 2, 5 ), F( 1, 5, 4 ) };              // hypotenuse
   const Vec3 centre( 30, 0, 0 );
   const TriangleMeshID mesh( createTriangleMesh( 89, centre, orig, faces, granite, false ) );
   const pc::BodyId id( mesh->getSystemID() );
   engine.advance( 1.0e-6 );
   expect( engine.spatialIndexActive(), "mesh: the hash grid is active" );

   const Vertices& bf( mesh->getBFVertices() );
   const Vec3 pBody( Vec3( 4.5, 0.2, 0 ) - orig[0] + bf[0] );   // inside the prism
   bool inKernel( true );
   for( const F& f : faces ) {
      const Vec3 normal( ( bf[f[1]] - bf[f[0]] ) % ( bf[f[2]] - bf[f[0]] ) );
      if( trans( pBody - bf[f[0]] ) * normal > real( 0 ) ) inKernel = false;
   }
   bool concave( false );   // some vertex in front of some face plane: not convex
   for( const F& f : faces ) {
      const Vec3 normal( ( bf[f[1]] - bf[f[0]] ) % ( bf[f[2]] - bf[f[0]] ) );
      for( const Vec3& v : bf )
         if( trans( v - bf[f[0]] ) * normal > real( 1e-12 ) ) concave = true;
   }
   expect( inKernel, "mesh: the test point satisfies pe's plane-based predicate" );
   expect( concave, "mesh: the mesh is not convex" );

   const pc::Vec3d still{ { 0.0, 0.0, 0.0 } };
   const pc::Vec3d c3{ { centre[0], centre[1], centre[2] } };
   auto pose = [&]( double degrees ) {
      const Quat q( Vec3( 0, 0, 1 ), degrees * pe::M_PI / 180.0 );
      return pc::KinematicState{ c3, pc::Quatd{ { q[0], q[1], q[2], q[3] } }, still, still };
   };
   // The box pe must report for the current pose, formed as pe forms it
   auto expectedBox = [&]( pc::Vec3d& lo, pc::Vec3d& hi ) {
      const pc::BodyState st( engine.state( id ) );
      const Quat q( st.orientation[0], st.orientation[1], st.orientation[2], st.orientation[3] );
      const Vec3 g( st.position[0], st.position[1], st.position[2] );
      Vec3 mn( g ), mx( g );
      for( const Vec3& v : bf ) {
         const Vec3 w( q.rotate( v ) + g );
         for( int k=0; k<3; ++k ) {
            mn[k] = std::min( mn[k], w[k] );
            mx[k] = std::max( mx[k], w[k] );
         }
      }
      for( int k=0; k<3; ++k ) {
         lo[k] = mn[k] - contactThreshold;
         hi[k] = mx[k] + contactThreshold;
      }
   };
   auto boxIsCurrent = [&]( const std::string& when ) {
      pc::Vec3d lo, hi;
      expectedBox( lo, hi );
      const pc::BodyState st( engine.state( id ) );
      expect( sameBits( st.aabbMin, lo ) && sameBits( st.aabbMax, hi ),
              "mesh: the reported box is that of the current pose " + when );
   };
   auto owned = [&]( const pc::Vec3d& point, const std::string& when ) {
      for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive,
                                       pc::QueryMethod::spatialIndex } )
         expect( engine.owner( point, m ) == id,
                 std::string( "mesh: the point is owned " ) +
                 ( m == pc::QueryMethod::exhaustive ? "(exhaustive) " : "(spatial index) " ) +
                 when );
   };

   engine.setKinematicState( id, pose( 45.0 ) );
   engine.refreshSpatialIndex();
   boxIsCurrent( "at +45" );
   const pc::BodyState at45( engine.state( id ) );
   const Vec3 w( Quat( Vec3( 0, 0, 1 ), -pe::M_PI / 4.0 ).rotate( pBody ) + centre );
   const pc::Vec3d point{ { w[0], w[1], w[2] } };
   bool outsideOld( false );
   for( int k=0; k<3; ++k )
      if( point[k] < at45.aabbMin[k] || point[k] > at45.aabbMax[k] ) outsideOld = true;
   expect( outsideOld, "mesh: the test point lies outside the +45 box" );

   engine.setKinematicState( id, pose( -45.0 ) );
   boxIsCurrent( "after setKinematicState() to -45, before any step" );
   engine.refreshSpatialIndex();
   owned( point, "at -45" );

   if( pc::buildInfo().checkpointsSupported ) {
      std::stringstream ckpt;
      engine.writeCheckpoint( ckpt );
      engine.setKinematicState( id, pose( 45.0 ) );
      engine.refreshSpatialIndex();
      engine.readCheckpoint( ckpt );
      boxIsCurrent( "after readCheckpoint() of -45" );
      engine.refreshSpatialIndex();
      owned( point, "after readCheckpoint() of -45" );
   }

   // A translation below pe's comparison tolerance (1e-8) also renews the vertices
   const pc::BodyState before( engine.state( id ) );
   pc::KinematicState nudged( pose( -45.0 ) );
   nudged.orientation = before.orientation;
   nudged.position[0] += 1.0e-9;
   engine.setKinematicState( id, nudged );
   boxIsCurrent( "after a translation by 1e-9" );
   expect( engine.state( id ).aabbMin[0] != before.aabbMin[0],
           "mesh: the box moved with a translation by 1e-9" );
   theWorld()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A thin, anisotropic box far from the origin: the position check works per axis, so the
 *        override to its own pose, and its own checkpoint, are accepted.
 */
static void checkAnisotropicBox()
{
   using namespace pe;
   theWorld()->clear();
   pc::Engine engine;
   const double x( std::ldexp( 1.0, 40 ) );   // 2^40: its spacing (2^-12) exceeds 1e-5
   const BoxID box( createBox( 80, Vec3( x, 0, 0 ), Vec3( 2.0, 2e-5, 2e-5 ), granite ) );
   const pc::BodyId id( box->getSystemID() );
   const pc::BodyState st( engine.state( id ) );
   const pc::KinematicState same{ st.position, st.orientation, st.linearVelocity,
                                  st.angularVelocity };
   engine.setKinematicState( id, same );   // the smallest half extent alone would refuse x
   expect( sameBits( engine.state( id ).position, st.position ), "thin box: own pose accepted" );
   if( pc::buildInfo().checkpointsSupported ) {
      std::stringstream ckpt;
      engine.writeCheckpoint( ckpt );
      engine.readCheckpoint( ckpt );
      expect( sameBits( engine.state( id ).position, st.position ),
              "thin box: this build reads its own checkpoint" );
   }
   // Another orientation (90 degrees about x: the thin sides swap, x keeps its extent) is
   // tested against an upper bound and accepted
   const double c( std::cos( pe::M_PI / 4.0 ) ), sn( std::sin( pe::M_PI / 4.0 ) );
   engine.setKinematicState( id, pc::KinematicState{ st.position, pc::Quatd{ { c, sn, 0.0, 0.0 } },
                                                     st.linearVelocity, st.angularVelocity } );
   expect( engine.state( id ).orientation[1] == sn, "thin box: rotated pose accepted" );
   if( pc::buildInfo().checkpointsSupported ) {
      // A checkpoint of the rotated pose, read while the box has its first orientation
      std::stringstream ckpt;
      engine.writeCheckpoint( ckpt );
      engine.setKinematicState( id, same );
      engine.readCheckpoint( ckpt );
      expect( engine.state( id ).orientation[1] == sn,
              "thin box: its checkpoint of another orientation is read" );
   }
   theWorld()->clear();
}
//*************************************************************************************************


int main()
{
   {
      pc::Engine engine;
      checkEnumeration( engine );
      checkStateOverride( engine );
      checkLookupAfterAddAndDestroy( engine );
      checkCreationValidation( engine );
      checkGlobalAndLocalIds( engine );
   }
   checkOldCounterLayout();
   checkEllipsoidBox();
   checkQuaternionEquality();
   checkMeshRotation();
   checkAnisotropicBox();
   return coupling_test::report( "pe_coupling_world_test" );
}
