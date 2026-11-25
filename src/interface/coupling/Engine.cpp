//=================================================================================================
/*!
 *  \file src/interface/coupling/Engine.cpp
 *  \brief Implementation of the typed C++ coupling API (pe::coupling::Engine)
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
 *  The load and stepping path reproduces the FEAT3 wrapper PhysicsEngine (control/rigid_body/
 *  pe_world.cpp) operation by operation: for each body that receives a load, the relaxed load
 *  is formed in double precision as r * F_n + (1 - r) * F_(n-1), set as the body force and
 *  torque and applied by RigidBody::applyFluidForces( dt, 1 ), and one coupling step is
 *  World::simulationStep( dt ). tests/interface/pe_coupling_feat_equivalence_test.cpp holds the
 *  two bit for bit together.
 *
 *  Exception safety: every member function validates and prepares everything that can throw
 *  (including all allocations and every value after its conversion to pe::real) before its
 *  first change of the world or of the coupling state, and then commits with operations that
 *  do not throw. The guarantee of each function is listed in pe/interface/coupling/Engine.h.
 */
//=================================================================================================


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <atomic>
#include <cmath>
#include <istream>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <pe/core.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/lubrication/Params.h>
#include <pe/interface/coupling/Engine.h>
#include <pe/system/SleepMode.h>
#include <pe/util/Limits.h>
#include <src/interface/PointOwnership.h>
#include <src/interface/coupling/SolverFeatures.h>
#include <fstream>
#include <map>
#if HAVE_MPI
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/interface/support/decompose.h>
#endif


namespace pe {

namespace coupling {

//=================================================================================================
//
//  LOCAL DEFINITIONS
//
//=================================================================================================

namespace {

//*************************************************************************************************
/*!\brief Whether an Engine exists in this process.
 *
 * The only process-wide state of the coupling API: it makes a second Engine fail loudly
 * instead of silently sharing pe's singleton world with the first one.
 */
std::atomic<bool> engineExists( false );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether loads were applied to pe's world and the step has not been advanced yet.
 *
 * Process-wide next to engineExists, because the applied impulse lives in the world, not in
 * the handle: a new Engine must still refuse a second applyLoads() and a checkpoint until
 * advance() has run. Set by applyLoads(), cleared by a successful advance() or readCheckpoint().
 */
std::atomic<bool> loadsAppliedThisStep( false );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The version of the checkpoint format written by Engine::writeCheckpoint().
 */
const int checkpointVersion( 3 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The tag in the first line of a checkpoint.
 */
const char* const checkpointMagic( "pe-coupling-checkpoint" );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The force and torque of one body.
 */
struct LoadRecord
{
   Vec3d force;   //!< Force.
   Vec3d torque;  //!< Torque.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Converts a coupling vector into a pe vector.
 */
inline Vec3 toPe( const Vec3d& v )
{
   return Vec3( real( v[0] ), real( v[1] ), real( v[2] ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Converts a pe vector into a coupling vector.
 */
inline Vec3d fromPe( const Vec3& v )
{
   return Vec3d{ { double( v[0] ), double( v[1] ), double( v[2] ) } };
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether all components are finite, as doubles and after conversion to pe::real.
 */
template< typename Array >
inline bool allFinite( const Array& a )
{
   for( std::size_t i=0; i<a.size(); ++i )
      if( !std::isfinite( a[i] ) || !std::isfinite( real( a[i] ) ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether a value is finite as a double and after conversion to pe::real.
 */
inline bool finiteReal( double x )
{
   return std::isfinite( x ) && std::isfinite( real( x ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether a pe vector is finite.
 */
inline bool finitePe( const Vec3& v )
{
   return std::isfinite( v[0] ) && std::isfinite( v[1] ) && std::isfinite( v[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether a pe vector is exactly zero (pe's operator== compares with a tolerance).
 */
inline bool exactlyZero( const Vec3& v )
{
   return v[0] == real( 0 ) && v[1] == real( 0 ) && v[2] == real( 0 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether two reals are the same value, a zero including its sign (bit for bit for
 *        finite values; compares values, not storage, so padding bytes play no role).
 */
inline bool sameBits( real a, real b )
{
   return a == b && std::signbit( a ) == std::signbit( b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Maps pe's geometry type onto the coupling shape kind.
 */
inline ShapeKind shapeKind( GeomType type )
{
   switch( type ) {
      case sphereType:       return ShapeKind::sphere;
      case capsuleType:      return ShapeKind::capsule;
      case ellipsoidType:    return ShapeKind::ellipsoid;
      case boxType:          return ShapeKind::box;
      case cylinderType:     return ShapeKind::cylinder;
      case triangleMeshType: return ShapeKind::triangleMesh;
      case planeType:        return ShapeKind::plane;
      default:               return ShapeKind::other;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The volume of a body where pe defines one for its shape, 0 otherwise.
 *
 * O(1) for every shape except the triangle mesh, whose getVolume() sums over all faces; the
 * Engine caches that one per body (Engine::Impl::volume()).
 */
inline double bodyVolume( ConstBodyID body )
{
   switch( body->getType() ) {
      case sphereType:       return double( static_cast<const Sphere*>( body )->getVolume() );
      case capsuleType:      return double( static_cast<const Capsule*>( body )->getVolume() );
      case ellipsoidType:    return double( static_cast<const Ellipsoid*>( body )->getVolume() );
      case boxType:          return double( static_cast<const Box*>( body )->getVolume() );
      case cylinderType:     return double( static_cast<const Cylinder*>( body )->getVolume() );
      case triangleMeshType: return double( static_cast<const TriangleMesh*>( body )->getVolume() );
      default:               return 0.0;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether a local body is a particle: finite and not fixed.
 */
inline bool isMovable( ConstBodyID body )
{
   return body->isFinite() && !body->isFixed();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether a local body is a fixed finite body (a wall or a kinematic body).
 */
inline bool isFixedFinite( ConstBodyID body )
{
   return body->isFinite() && body->isFixed();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether pe runs on more than one process of its communicator.
 */
inline bool parallelRun()
{
   return MPISettings::size() > 1;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The prefix of every error message of a member function.
 */
inline std::string where( const char* function )
{
   return std::string( "pe::coupling::Engine::" ) + function + ": ";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Formats a body id for an error message.
 */
inline std::string idString( BodyId id )
{
   std::ostringstream oss;
   oss.imbue( std::locale::classic() );
   oss << id;
   return oss.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws if the compiled solver does not support a setting.
 *
 * \param function The name of the calling member function.
 * \param setting The setting.
 * \param reason Why the solver lacks the setting.
 * \exception std::runtime_error The setting is not supported.
 */
inline void requireSetting( const char* function, Setting setting, const std::string& reason )
{
   if( !detail::settingSupported( setting ) ) {
      throw std::runtime_error( where( function ) + "setting '" + detail::settingName( setting ) +
                                "' is not supported by the compiled constraint solver " +
                                detail::constraintSolverName() + " (" + reason +
                                "); query pe::coupling::isSupported() beforehand" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws the "not implemented yet" error of the cross-process operations.
 */
[[noreturn]] inline void throwNotImplemented( const char* function, const std::string& what )
{
   throw std::runtime_error( where( function ) + what + " is not implemented yet (the coupling "
                             "API cannot yet carry this state across processes)" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws "not implemented yet" when pe runs on more than one process.
 */
inline void requireSingleProcess( const char* function, const char* what )
{
   if( parallelRun() )
      throwNotImplemented( function, std::string( what ) + " on more than one process" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws std::logic_error, locally, before a collective inside a pe exclusive section.
 *
 * Inside pe_EXCLUSIVE_SECTION only one process runs the code; a collective there would wait
 * for processes that skipped the section. pe's own entry points refuse that case, but only
 * after the Engine's own collectives, so the Engine refuses first.
 */
inline void requireNoExclusiveSection( const char* function )
{
   if( parallelRun() && ExclusiveSection::isActive() )
      throw std::logic_error( where( function ) + "collective call inside a pe exclusive section "
                              "on more than one process" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws std::invalid_argument for an invalid query point.
 *
 * \param function The name of the calling member function.
 * \param pos The point.
 * \param bound The largest coordinate magnitude the query can handle.
 */
inline void checkPoint( const char* function, const double* pos, double bound )
{
   for( int k=0; k<3; ++k ) {
      if( !finiteReal( pos[k] ) )
         throw std::invalid_argument( where( function ) + "a query point is not finite" );
      if( std::fabs( pos[k] ) > bound )
         throw std::invalid_argument( where( function ) + "a query point lies outside the range "
                                      "the hash grid can index" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The coordinate bound of a query method (infinite for the exhaustive traversal).
 */
inline double queryBound( QueryMethod method )
{
   if( method == QueryMethod::exhaustive )
      return std::numeric_limits<double>::infinity();
   return double( detail::pointQueryBound( theCollisionSystem()->getCoarseDetector() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws std::invalid_argument if a body of the given half extent is not representable
 *        at a position: in every coordinate, centre - h and centre + h must be finite and
 *        strictly ordered around the centre in pe::real, otherwise the bounding box overflows
 *        or degenerates (pe asserts on it).
 */
inline void checkRepresentable( const char* function, const Vec3d& position, real halfExtent )
{
   for( int k=0; k<3; ++k ) {
      const real c( position[k] );
      const real lo( c - halfExtent ), hi( c + halfExtent );
      if( !( std::isfinite( lo ) && std::isfinite( hi ) && lo < c && c < hi ) )
         throw std::invalid_argument( where( function ) + "the body cannot be represented at "
                                      "this position (its bounding box would overflow or "
                                      "collapse in pe::real)" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The half extent pe pads a sphere's bounding box with (SphereBase::calcBoundingBox()).
 */
inline real sphereBoxHalfExtent( real radius )
{
   return radius + contactThreshold + lubrication::aabbPadding( radius );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws std::invalid_argument if a body could not be represented at a target pose.
 *
 * \param function The name of the calling member function.
 * \param body The finite body (in its current pose).
 * \param state The target pose.
 *
 * What the check guarantees:
 * - A sphere: exact. Its box is the padded radius around the centre in every orientation, and
 *   the check forms the box ends as pe does.
 * - Any other body, target orientation bit-for-bit equal to the current one: the per-axis
 *   offsets of the current box from the current position (pos - min, max - pos) are tested at
 *   the target position. A body at its own pose -- in particular every pose of a checkpoint
 *   this build wrote, read back into the same world -- is accepted. For a translated target the
 *   offsets are recovered from world coordinates that pe has already rounded, so they can be
 *   off by about one unit in the last place of the current coordinate: a body whose half
 *   extent is of that order can be refused at a target where pe could represent it (or, in
 *   the same regime, accepted where its box collapses).
 * - Any other body, another orientation: the box of the target pose is not known without
 *   moving the body, so an upper bound is tested in every axis: the distance from the position
 *   to the farthest corner of the current box (the body lies in that sphere around its
 *   position in every orientation). An upper bound never refuses a pose whose real box is
 *   representable (short of a position within a few body sizes of the largest pe::real) -- so
 *   this build reads its own checkpoints -- but may accept a pose
 *   whose box collapses along one axis (a thin body turned so that its thin side faces a huge
 *   coordinate); pe then asserts in a Debug build.
 * - Only the start and the target pose are considered. The override sets the position before
 *   the orientation (the call order of the FEAT3 wrapper, kept for bitwise equivalence), so
 *   pe computes a box for the intermediate pose "target position, current orientation"; for a
 *   thin body that pose can collapse although start and target are both representable (pe
 *   asserts on it in Debug; a Release build continues and the target box is computed afresh).
 */
inline void checkRepresentablePose( const char* function, ConstBodyID body,
                                    const KinematicState& state )
{
   if( body->getType() == sphereType ) {
      checkRepresentable( function, state.position,
         sphereBoxHalfExtent( static_cast<const Sphere*>( body )->getRadius() ) );
      return;
   }

   const RigidBody::AABB& box( body->getAABB() );
   const Vec3& pos( body->getPosition() );
   const Quat& q( body->getQuaternion() );
   const bool sameOrientation( real( state.orientation[0] ) == q[0] &&
                               real( state.orientation[1] ) == q[1] &&
                               real( state.orientation[2] ) == q[2] &&
                               real( state.orientation[3] ) == q[3] );
   if( sameOrientation ) {
      for( int k=0; k<3; ++k ) {
         const real c( state.position[k] );
         const real lo( c - ( pos[k] - box[k] ) ), hi( c + ( box[k+3] - pos[k] ) );
         if( !( std::isfinite( lo ) && std::isfinite( hi ) && lo < c && c < hi ) )
            throw std::invalid_argument( where( function ) + "the body cannot be represented "
                                         "at this position (its bounding box would overflow "
                                         "or collapse in pe::real)" );
      }
      return;
   }

   real bound( 0 );
   for( int corner=0; corner<8; ++corner ) {
      real d2( 0 );
      for( int k=0; k<3; ++k ) {
         const real e( ( corner >> k ) & 1 ? box[k+3] - pos[k] : pos[k] - box[k] );
         d2 += e*e;
      }
      bound = std::max( bound, std::sqrt( d2 ) );
   }
   checkRepresentable( function, state.position, bound );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Keeps the solver's per-step diagnostic block off stdout for the scope of a step.
 *
 * HardContactAndFluid prints a block per sphere and step on the representative CFD rank
 * (SimulationConfig::getCfdRank() == 1, the default). The coupling API must not write to
 * stdout, so advance() switches only that block off (setPredictionDiagnostics) and restores
 * the switch afterwards, also on exceptions. The CFD rank itself stays as it is, so force
 * generators and triggers running inside the step see the caller's configuration.
 */
class DiagnosticsOffScope
{
public:
   explicit DiagnosticsOffScope( detail::ActiveCollisionSystem& cs )
      : cs_      ( cs )
      , previous_( false )
      , switched_( detail::trySetPredictionDiagnostics( cs, false, previous_ ) )
   {}

   ~DiagnosticsOffScope()
   {
      bool ignored( false );
      if( switched_ ) detail::trySetPredictionDiagnostics( cs_, previous_, ignored );
   }

   DiagnosticsOffScope( const DiagnosticsOffScope& ) = delete;
   DiagnosticsOffScope& operator=( const DiagnosticsOffScope& ) = delete;

private:
   detail::ActiveCollisionSystem& cs_;        //!< The collision system.
   bool                           previous_;  //!< The switch to restore.
   const bool                     switched_;  //!< Whether the solver has the switch.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Gives an input stream the checkpoint's formatting (classic locale, decimal, skipping
 *        white space) for the scope of a read and restores the caller's locale and flags.
 */
class StreamFormatScope
{
public:
   explicit StreamFormatScope( std::ios& stream )
      : stream_( stream )
      , locale_( stream.imbue( std::locale::classic() ) )
      , flags_ ( stream.flags( std::ios_base::dec | std::ios_base::skipws ) )
      , width_ ( stream.width( 0 ) )
   {}

   ~StreamFormatScope()
   {
      stream_.width( width_ );
      stream_.flags( flags_ );
      stream_.imbue( locale_ );
   }

   StreamFormatScope( const StreamFormatScope& ) = delete;
   StreamFormatScope& operator=( const StreamFormatScope& ) = delete;

private:
   std::ios&                     stream_;  //!< The stream.
   const std::locale             locale_;  //!< Its original locale.
   const std::ios_base::fmtflags flags_;   //!< Its original format flags.
   const std::streamsize         width_;   //!< Its original (pending) field width.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws if the build cannot write or read a bitwise checkpoint.
 *
 * Sleep mode: a restore could not restore the sleep state and the motion measure. A pe::real
 * wider than double cannot round-trip through the 17-digit text of a double.
 */
inline void requireCheckpointBuild( const char* function )
{
   if( sleepThreshold != real( 0 ) )
      throw std::runtime_error( where( function ) + "pe is compiled with sleep mode "
                                "(PE_SLEEP_THRESHOLD != 0); a checkpoint cannot restore the "
                                "sleep state and the motion measure of the bodies" );
   if( std::numeric_limits<real>::digits > std::numeric_limits<double>::digits )
      throw std::runtime_error( where( function ) + "pe::real is wider than double; the "
                                "checkpoint text cannot restore its values bit for bit" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Throws if the world holds a body the checkpoint cannot restore.
 *
 * Supported: finite non-union bodies, and planes at rest, with empty force and torque slots.
 * Refused: a plane with any nonzero velocity component (a kinematic wall under MOBILE_INFINITE;
 * a restart would put it back at its setup offset), a union (its members accumulate their own
 * orientations, which a restore of the union does not reproduce), every other body of
 * ShapeKind::other (inner cylinders, inner meshes), and a finite body with a nonzero force or
 * torque slot (a native force waiting for the next step, which the checkpoint does not hold).
 * All comparisons are exact.
 */
inline void requireCheckpointScope( const char* function )
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   for( auto it=cs.getBodyStorage().begin(); it!=cs.getBodyStorage().end(); ++it ) {
      const ConstBodyID body( *it );
      if( MPISettings::size() > 1 && body->isGlobal() && body->isFinite() )
         throw std::runtime_error( where( function ) + "body " + idString( body->getSystemID() ) +
                                   " is a global finite body, which a checkpoint on more than one "
                                   "process cannot restore (not implemented yet)" );
      if( shapeKind( body->getType() ) == ShapeKind::other )
         throw std::runtime_error( where( function ) + "body " + idString( body->getSystemID() ) +
                                   " is a union or another composite/inner type, which the "
                                   "checkpoint cannot restore" );
      if( body->getType() == planeType &&
          ( !exactlyZero( body->getLinearVel() ) || !exactlyZero( body->getAngularVel() ) ) )
         throw std::runtime_error( where( function ) + "plane " + idString( body->getSystemID() ) +
                                   " moves (a kinematic plane), which the checkpoint cannot "
                                   "restore" );
      if( body->isFinite() &&
          ( !exactlyZero( body->getForce() ) || !exactlyZero( body->getTorque() ) ) )
         throw std::runtime_error( where( function ) + "body " + idString( body->getSystemID() ) +
                                   " has a force or torque waiting for the next step, which the "
                                   "checkpoint does not hold" );
   }
}
//*************************************************************************************************

} // namespace




//=================================================================================================
//
//  IMPLEMENTATION STRUCTURE
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The coupling state of an Engine.
 */
struct Engine::Impl
{
   //**Utility functions***************************************************************************
   BodyID find         ( BodyId id, bool& shadow ) const;
   BodyID findLocal    ( const char* function, BodyId id ) const;
   void   collect      ( Selection selection, std::vector<BodyID>& bodies ) const;
   void   collectFixed ( std::vector<BodyID>& bodies ) const;
   void   overrideState( BodyID body, const KinematicState& state );
   double volume       ( ConstBodyID body ) const;
   void   fillState    ( ConstBodyID body, bool shadow, BodyState& s ) const;
   void   pruneHistory ();
   void   pruneVolumes () const;
   std::string checkpointRecord( const char* function, const char* tag, ConstBodyID body ) const;
   std::string reachError( ConstBodyID body, real h ) const;
   int         ownerSlab ( const Vec3& gpos ) const;
   bool        reachable ( const Vec3& gpos ) const;
   //**********************************************************************************************

   //**Member variables****************************************************************************
   WorldID                                  world_;            //!< pe's world (keeps it alive).
   std::unordered_map<BodyId,LoadRecord>    pending_;          //!< Loads of this coupling step.
   std::unordered_map<BodyId,LoadRecord>    previous_;         //!< F_(n-1), T_(n-1) per body,
                                                               //!< kept while the body exists.
   bool                                     overridden_;       //!< Synchronize before the next
                                                               //!< step (several processes).
   bool                                     decomposed_ = false; //!< decomposeSlabs() was called.
#if HAVE_MPI
   std::vector<HalfSpace>                   halfSpaces_;       //!< decomposeSlabs(): the dividing
                                                               //!< planes as pe's domain holds
                                                               //!< them (several processes only).
#endif
   real                                     minInteriorSlab_ = std::numeric_limits<real>::infinity();
                                                               //!< Thinnest interior slab if all
                                                               //!< planes are parallel, else inf.
   bool                                     prepared_ = false; //!< prepareCheckpoint() holds the
                                                               //!< current checkpoint (several
                                                               //!< processes).
   std::string                              preparedText_;     //!< The gathered checkpoint text
                                                               //!< (root process only).
   mutable std::unordered_map<BodyId,double> meshVolumes_;     //!< Cached triangle-mesh volumes.
   mutable std::vector<BodyID>              candidates_;       //!< Scratch: index candidates.
   mutable std::vector<BodyID>              bodies_;           //!< Scratch: selected bodies.
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds a body among the local bodies and the shadow copies.
 *
 * \param id The system id.
 * \param shadow Output: whether the body is a shadow copy.
 * \return The body, or nullptr if there is none with this id.
 *
 * A lookup in the id map pe's BodyStorage keeps (O(log n)), so it stays correct when bodies
 * are added or removed.
 */
BodyID Engine::Impl::find( BodyId id, bool& shadow ) const
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );

   const auto local( cs.getBodyStorage().find( id ) );
   if( local != cs.getBodyStorage().end() ) {
      shadow = false;
      return *local;
   }

   const BodyID remote( interface::detail::findShadowCopy( cs, id ) );
   shadow = ( remote != nullptr );
   return remote;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds a local body for a modifying operation.
 *
 * \param function The name of the calling member function.
 * \param id The system id.
 * \return The body.
 * \exception std::invalid_argument There is no body with this id.
 * \exception std::runtime_error The body is a shadow copy.
 */
BodyID Engine::Impl::findLocal( const char* function, BodyId id ) const
{
   bool shadow( false );
   const BodyID body( find( id, shadow ) );
   if( body == nullptr )
      throw std::invalid_argument( where( function ) + "no body with system id " + idString( id ) );
   if( shadow )
      throwNotImplemented( function, "modifying the shadow copy of body " + idString( id ) );
   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Collects the bodies of a selection in storage order.
 *
 * \param selection The selection.
 * \param bodies Output: cleared, then the selected bodies.
 * \return void
 */
void Engine::Impl::collect( Selection selection, std::vector<BodyID>& bodies ) const
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   bodies.clear();

   if( selection == Selection::shadowCopies ) {
      interface::detail::forEachShadowCopy( cs, [&]( BodyID body ) { bodies.push_back( body ); } );
      return;
   }

   for( auto it=cs.getBodyStorage().begin(); it!=cs.getBodyStorage().end(); ++it ) {
      if( selection == Selection::all || isMovable( *it ) )
         bodies.push_back( *it );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Collects the local fixed finite bodies in storage order.
 */
void Engine::Impl::collectFixed( std::vector<BodyID>& bodies ) const
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   bodies.clear();
   for( auto it=cs.getBodyStorage().begin(); it!=cs.getBodyStorage().end(); ++it )
      if( isFixedFinite( *it ) ) bodies.push_back( *it );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overrides the kinematic state of a body (no validation).
 *
 * \param body The body.
 * \param state The new state.
 * \return void
 *
 * Same calls in the same order as the FEAT3 wrapper's set_body_state().
 */
void Engine::Impl::overrideState( BodyID body, const KinematicState& state )
{
   body->setPosition( toPe( state.position ) );
   body->setOrientation( real( state.orientation[0] ), real( state.orientation[1] ),
                         real( state.orientation[2] ), real( state.orientation[3] ) );
   body->setLinearVel( toPe( state.linearVelocity ) );
   body->setAngularVel( toPe( state.angularVelocity ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The volume of a body; triangle-mesh volumes are computed once and cached by id.
 *
 * A rigid body's volume does not change, and system ids are never reused, so the cache never
 * goes stale (entries of destroyed meshes are dropped by pruneHistory()).
 */
double Engine::Impl::volume( ConstBodyID body ) const
{
   if( body->getType() != triangleMeshType )
      return bodyVolume( body );
   const auto it( meshVolumes_.find( body->getSystemID() ) );
   if( it != meshVolumes_.end() )
      return it->second;
   const double v( bodyVolume( body ) );
   meshVolumes_.emplace( body->getSystemID(), v );
   return v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Fills the state snapshot of a body (does not throw once the mesh volume is cached).
 */
void Engine::Impl::fillState( ConstBodyID body, bool shadow, BodyState& s ) const
{
   s.id              = body->getSystemID();
   s.userId          = static_cast<std::uint64_t>( body->getID() );
   s.shape           = shapeKind( body->getType() );
   s.position        = fromPe( body->getPosition() );
   s.linearVelocity  = fromPe( body->getLinearVel() );
   s.angularVelocity = fromPe( body->getAngularVel() );
   const Quat& q( body->getQuaternion() );
   s.orientation     = Quatd{ { double( q[0] ), double( q[1] ), double( q[2] ), double( q[3] ) } };
   s.mass            = double( body->getMass() );
   s.radius          = ( body->getType() == sphereType )
                     ? double( static_cast<const Sphere*>( body )->getRadius() ) : 0.0;
   s.volume          = volume( body );
   const RigidBody::AABB& box( body->getAABB() );
   s.aabbMin         = Vec3d{ { double( box[0] ), double( box[1] ), double( box[2] ) } };
   s.aabbMax         = Vec3d{ { double( box[3] ), double( box[4] ), double( box[5] ) } };
   s.fixed           = body->isFixed();
   s.shadow          = shadow;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Drops the load history of bodies that no longer exist locally.
 *
 * Erase only, so it does not throw. The history of a body that still exists is kept, also
 * while the body is fixed, independent of how many other bodies there are.
 */
void Engine::Impl::pruneHistory()
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   for( auto it=previous_.begin(); it!=previous_.end(); ) {
      if( cs.getBodyStorage().find( it->first ) == cs.getBodyStorage().end() )
         it = previous_.erase( it );
      else
         ++it;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Drops the cached volumes of meshes that exist neither locally nor as shadow copies.
 *
 * Erase only, so it does not throw. Called by advance(), also on several processes, so the
 * cache of a rank that sees meshes pass through as shadow copies stays bounded.
 */
void Engine::Impl::pruneVolumes() const
{
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   for( auto it=meshVolumes_.begin(); it!=meshVolumes_.end(); ) {
      if( cs.getBodyStorage().find( it->first ) == cs.getBodyStorage().end() &&
          interface::detail::findShadowCopy( cs, it->first ) == nullptr )
         it = meshVolumes_.erase( it );
      else
         ++it;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Validates a kinematic state against pe's preconditions.
 *
 * \param function The name of the calling member function.
 * \param state The state.
 * \exception std::invalid_argument A component is not finite or the quaternion is not unit.
 *
 * The quaternion test is pe's own (Quaternion( r, i, j, k ) asserts
 * |r^2 + i^2 + j^2 + k^2 - 1| < accuracy, evaluated in pe::real). The quaternion is never
 * normalized here: that would change its bits and break bitwise restarts.
 */
static void validateKinematicState( const char* function, const KinematicState& state )
{
   if( !allFinite( state.position ) || !allFinite( state.orientation ) ||
       !allFinite( state.linearVelocity ) || !allFinite( state.angularVelocity ) )
      throw std::invalid_argument( where( function ) + "the kinematic state is not finite" );

   const real r( state.orientation[0] ), i( state.orientation[1] ),
              j( state.orientation[2] ), k( state.orientation[3] );
   if( !( std::fabs( r*r + i*i + j*j + k*k - real(1) ) < Limits<real>::accuracy() ) )
      throw std::invalid_argument( where( function ) + "the orientation is not a unit quaternion "
                                   "(order r, i, j, k) within pe's accuracy" );
}
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creates the engine handle on pe's world.
 *
 * \exception std::logic_error Another Engine exists in this process.
 *
 * The world is created if it does not exist yet; its configuration and bodies are left as
 * they are. In MPI builds MPI must be initialized first. On more than one process the new
 * handle synchronizes before its first step: a previous handle may have overridden a state
 * that pe does not register as requiring a synchronization.
 */
//*************************************************************************************************
/*!\brief Formats the checkpoint record of a body: one line, including the newline.
 *
 * \param function The name of the calling member function (for messages).
 * \param tag "body" or "fixedbody".
 * \param body The body.
 * \return The line.
 * \exception std::runtime_error A non-finite state, load or fluid-force history.
 *
 * The same text on one process and on several: format version 3, classic locale, 17
 * significant digits (see Engine::writeCheckpoint()).
 */
std::string Engine::Impl::checkpointRecord( const char* function, const char* tag,
                                            ConstBodyID body ) const
{
   std::ostringstream out;
   out.imbue( std::locale::classic() );
   out.precision( 17 );
   BodyState s;
   fillState( body, false, s );
   if( !allFinite( s.position ) || !allFinite( s.orientation ) ||
       !allFinite( s.linearVelocity ) || !allFinite( s.angularVelocity ) )
      throw std::runtime_error( where( function ) + "body " + idString( s.id ) +
                                " has a non-finite state" );
   const auto previous( previous_.find( s.id ) );
   const bool hasPrevious( previous != previous_.end() );
   const LoadRecord prev( hasPrevious ? previous->second
                                      : LoadRecord{ Vec3d{ { 0.0, 0.0, 0.0 } },
                                                    Vec3d{ { 0.0, 0.0, 0.0 } } } );
   if( !allFinite( prev.force ) || !allFinite( prev.torque ) )
      throw std::runtime_error( where( function ) + "body " + idString( s.id ) +
                                " has a non-finite previous load" );
   out << tag << ' ' << s.id;
   for( int k=0; k<3; ++k ) out << ' ' << s.position[k];
   for( int k=0; k<4; ++k ) out << ' ' << s.orientation[k];
   for( int k=0; k<3; ++k ) out << ' ' << s.linearVelocity[k];
   for( int k=0; k<3; ++k ) out << ' ' << s.angularVelocity[k];
   // pe's own fluid-force history of the body (applyFluidForces() relaxes against it)
   const Vec3& fluidForce( body->getPrevFluidForce() );
   const Vec3& fluidTorque( body->getPrevFluidTorque() );
   if( !finitePe( fluidForce ) || !finitePe( fluidTorque ) )
      throw std::runtime_error( where( function ) + "body " + idString( s.id ) +
                                " has a non-finite previous fluid force" );
   out << ' ' << ( hasPrevious ? 1 : 0 );
   for( int k=0; k<3; ++k ) out << ' ' << prev.force[k];
   for( int k=0; k<3; ++k ) out << ' ' << prev.torque[k];
   out << ' ' << ( body->hasPrevFluidForce() ? 1 : 0 );
   for( int k=0; k<3; ++k ) out << ' ' << double( fluidForce[k] );
   for( int k=0; k<3; ++k ) out << ' ' << double( fluidTorque[k] );
   out << '\n';
   if( !out )
      throw std::runtime_error( where( function ) + "formatting the checkpoint failed" );
   return out.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks that a local body stays within the reach of pe's neighbour communication in the
 *        next pe step of size \a h (a decomposition by decomposeSlabs() on three or more
 *        processes).
 *
 * \param body A local finite body.
 * \param h The size of the next pe step.
 * \return An empty string, or the violated rule.
 *
 * pe keeps shadow copies on, and migrates bodies to, the neighbouring slabs only. With R the
 * reach of the body for pe's shadow-copy test (its bounding sphere radius plus the process
 * overlap contactThreshold plus the shadow-copy margin) the rules are: (i) the ball of radius R
 * around the centre touches no slab that is not the holder's or a neighbour of it (otherwise a
 * contact there is missed, or the next migration needs a distant process); (ii) the ball of
 * radius R + (|v| + |g| h) h, the reach after one pe step without contacts, does not either
 * (otherwise pe can delete the body as an outflow although a distant slab holds its centre).
 * A ball touches slab j if it lies within R of the positive side of plane j and of the negative
 * side of plane j-1 (a conservative test for inclined planes).
 */
std::string Engine::Impl::reachError( ConstBodyID body, real h ) const
{
   const int slab( MPISettings::rank() );
   const int nSlabs( MPISettings::size() );
#if HAVE_MPI
   const int nPlanes( static_cast<int>( halfSpaces_.size() ) );
   Vec3 centre( body->getPosition() );
   real radius( 0 );
   if( body->getType() == sphereType )
      radius = static_body_cast<const Sphere>( body )->getRadius();
   else {
      const RigidBody::AABB& box( body->getAABB() );
      centre = Vec3( real(0.5) * ( box[0] + box[3] ), real(0.5) * ( box[1] + box[4] ),
                     real(0.5) * ( box[2] + box[5] ) );
      radius = real(0.5) * std::sqrt( sq( box[3] - box[0] ) + sq( box[4] - box[1] ) +
                                      sq( box[5] - box[2] ) );
   }
   const real reach( radius + contactThreshold + lubrication::getShadowCopyMargin() );
   const real speed( body->getLinearVel().length() + Settings::gravity().length() * h );
   const real travel( reach + speed * h );
   const auto touches = [&]( int j, real r ) {
      const bool lower( j == 0 || trans( halfSpaces_[j-1].getNormal() ) * centre -
                                  halfSpaces_[j-1].getDisplacement() <= r );
      const bool upper( j >= nPlanes || trans( halfSpaces_[j].getNormal() ) * centre -
                                        halfSpaces_[j].getDisplacement() >= -r );
      return lower && upper;
   };
   for( int j=0; j<nSlabs; ++j ) {
      if( std::abs( j - slab ) < 2 )
         continue;
      std::ostringstream oss;
      if( touches( j, reach ) )
         oss << "body " << body->getSystemID() << " (held by process " << slab << ", slab "
             << slab << ") reaches into slab " << j << ", which is not a neighbour of slab "
             << slab << " (rule: its reach, radius plus pe's process overlap, " << reach
             << ", must stay within the holder's and the neighbouring slabs; pe keeps shadow "
             << "copies only there, so a contact in slab " << j << " would be missed and the "
             << "next migration would need a distant process)";
      else if( touches( j, travel ) )
         oss << "body " << body->getSystemID() << " (held by process " << slab << ", slab "
             << slab << ") can move into slab " << j << " in one pe step of " << h << " (rule: "
             << "its reach " << reach << " plus the displacement (|v| + |g| h) h = "
             << speed * h << " must stay within the holder's and the neighbouring slabs; pe "
             << "migrates bodies only to neighbours and would delete it as an outflow); use "
             << "more substeps or thicker slabs";
      if( !oss.str().empty() )
         return oss.str();
   }
   return std::string();
#else
   UNUSED( body ); UNUSED( h );
   return std::string();
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether pe's next synchronization hands a local body at \a gpos to its owner.
 *
 * With a slab decomposition (decomposeSlabs()): the owner of the point (ownerSlab(), pe's tie
 * rule; a point on the far face of a neighbour slab belongs to the slab beyond it) is this
 * process or a neighbour. Otherwise: the local subdomain or a connected neighbour subdomain
 * contains the point. pe migrates bodies to neighbours only and deletes a body that no
 * neighbour takes as an outflow. Always true on one process.
 */
bool Engine::Impl::reachable( const Vec3& gpos ) const
{
#if HAVE_MPI
   if( !parallelRun() )
      return true;
   if( !halfSpaces_.empty() ) {
      const int owner( ownerSlab( gpos ) );
      return owner >= 0 && std::abs( owner - MPISettings::rank() ) <= 1;
   }
   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   if( cs.getDomain().containsPoint( gpos ) )
      return true;
   for( auto p=cs.getProcessStorage().begin(); p!=cs.getProcessStorage().end(); ++p )
      if( p->containsPoint( gpos ) )
         return true;
   return false;
#else
   UNUSED( gpos );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The slab that owns a point under pe's ownership rule (decomposeSlabs(), several
 *        processes): the lowest slab whose closed region contains it (the tie of a point on a
 *        plane goes to the lower rank), evaluated with the very half spaces of pe's domain.
 *
 * \param gpos The point.
 * \return The slab (= rank), or -1 if no slab contains the point (beyond a closing plane).
 *
 * Every process computes the same result from the same planes; on a chain of slabs it is the
 * process whose Domain::ownsPoint() accepts the point.
 */
int Engine::Impl::ownerSlab( const Vec3& gpos ) const
{
#if HAVE_MPI
   const int nSlabs( MPISettings::size() );
   const int nPlanes( static_cast<int>( halfSpaces_.size() ) );
   for( int j=0; j<nSlabs; ++j ) {
      const bool lower( j == 0 || halfSpaces_[j-1].opposite().containsPoint( gpos ) );
      const bool upper( j >= nPlanes || halfSpaces_[j].containsPoint( gpos ) );
      if( lower && upper )
         return j;
   }
   return -1;
#else
   UNUSED( gpos );
   return 0;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Agrees on an error over pe's communicator (collective).
 *
 * \param error The local error message (empty if none).
 * \return The message of the lowest-ranked process that has one, on every process; empty if no
 *         process has one. Without MPI or on one process the local message.
 */
//! Whether the active collision system can rebuild its shadow copies (relocation on restart).
using HasClearShadowCopiesActive = detail::HasClearShadowCopies<detail::ActiveCollisionSystem>;

inline std::string agreeOnError( const std::string& error )
{
#if HAVE_MPI
   if( !parallelRun() )
      return error;
   int first( error.empty() ? std::numeric_limits<int>::max() : MPISettings::rank() );
   MPI_Allreduce( MPI_IN_PLACE, &first, 1, MPI_INT, MPI_MIN, MPISettings::comm() );
   if( first == std::numeric_limits<int>::max() )
      return std::string();
   int length( first == MPISettings::rank() ? static_cast<int>( error.size() ) : 0 );
   MPI_Bcast( &length, 1, MPI_INT, first, MPISettings::comm() );
   std::string message( first == MPISettings::rank() ? error : std::string( std::size_t( length ), ' ' ) );
   if( length > 0 )
      MPI_Bcast( &message[0], length, MPI_CHAR, first, MPISettings::comm() );
   return message;
#else
   return error;
#endif
}
//*************************************************************************************************




Engine::Engine()
   : impl_()
{
   bool expected( false );
   if( !engineExists.compare_exchange_strong( expected, true ) )
      throw std::logic_error( "pe::coupling::Engine: an Engine already exists in this process; "
                              "pe's world is a process-wide singleton, so only one Engine may "
                              "exist at a time" );

   try {
      impl_.reset( new Impl() );
      impl_->world_           = theWorld();
      impl_->overridden_      = parallelRun();
   }
   catch( ... ) {
      engineExists.store( false );
      throw;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Releases the engine handle; the world and its bodies stay.
 */
Engine::~Engine()
{
   impl_.reset();
   engineExists.store( false );
}
//*************************************************************************************************




//=================================================================================================
//
//  WORLD CONFIGURATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the gravitational acceleration.
 *
 * \param gravity The gravity vector.
 * \return void
 * \exception std::runtime_error The compiled solver does not declare that it applies gravity.
 * \exception std::invalid_argument The gravity is not finite (also after conversion to real).
 *
 * Available only when the solver applies gravity itself (BodyForce::appliedBySolver); with any
 * other solver the CFD side owns gravity, or nobody classified the solver.
 */
void Engine::setGravity( const Vec3d& gravity )
{
   requireSetting( "setGravity", Setting::gravity,
                   detail::bodyForceReason(
                      detail::gravitySemantics<detail::ActiveCollisionSystem>(), "gravity" ) );
   if( !allFinite( gravity ) )
      throw std::invalid_argument( where( "setGravity" ) + "the gravity is not finite" );
   impl_->world_->setGravity( toPe( gravity ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the gravitational acceleration stored in pe's world.
 *
 * \return The gravity vector (zero unless set).
 */
Vec3d Engine::gravity() const
{
   return fromPe( impl_->world_->getGravity() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the density of the surrounding fluid, used by the solver for buoyancy.
 *
 * \param density The fluid density.
 * \return void
 * \exception std::runtime_error The compiled solver does not declare that it applies buoyancy.
 * \exception std::invalid_argument The density is negative or not finite.
 *
 * Also switches pe's liquid-solid mode on (World::setLiquidSolid()), as the FEAT3 wrapper did.
 */
void Engine::setFluidDensity( double density )
{
   requireSetting( "setFluidDensity", Setting::fluidDensity,
                   detail::bodyForceReason(
                      detail::buoyancySemantics<detail::ActiveCollisionSystem>(), "buoyancy" ) );
   if( !finiteReal( density ) || density < 0.0 )
      throw std::invalid_argument( where( "setFluidDensity" ) + "invalid fluid density" );
   impl_->world_->setLiquidDensity( real( density ) );
   impl_->world_->setLiquidSolid( true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the dynamic viscosity of the surrounding fluid.
 *
 * \param viscosity The dynamic viscosity.
 * \return void
 * \exception std::invalid_argument The viscosity is negative or not finite.
 *
 * Only lubrication models read the viscosity (the lubrication add-on, the Euler-Lagrange
 * lubrication); the hard-contact step itself does not.
 */
void Engine::setFluidViscosity( double viscosity )
{
   if( !finiteReal( viscosity ) || viscosity < 0.0 )
      throw std::invalid_argument( where( "setFluidViscosity" ) + "invalid fluid viscosity" );
   impl_->world_->setViscosity( real( viscosity ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the error reduction parameter of the contact solver.
 *
 * \param erp The error reduction parameter in [0, 1] (pe default 0.7 for hard contacts).
 * \return void
 * \exception std::runtime_error The compiled solver has no such parameter.
 * \exception std::invalid_argument The value lies outside [0, 1] (checked in pe::real).
 */
void Engine::setErrorReduction( double erp )
{
   requireSetting( "setErrorReduction", Setting::errorReduction,
                   "no setErrorReductionParameter()" );
   const real value( erp );
   if( !( erp >= 0.0 && erp <= 1.0 && value >= real( 0 ) && value <= real( 1 ) ) )
      throw std::invalid_argument( where( "setErrorReduction" ) + "the error reduction "
                                   "parameter must lie in [0, 1]" );
   detail::trySetErrorReduction( *theCollisionSystem(), value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the maximum number of contact solver iterations.
 *
 * \param iterations The iteration limit, at least 1.
 * \return void
 * \exception std::runtime_error The compiled solver has no such parameter.
 * \exception std::invalid_argument Zero iterations.
 */
void Engine::setMaxIterations( std::size_t iterations )
{
   requireSetting( "setMaxIterations", Setting::maxIterations, "no setMaxIterations()" );
   if( iterations == 0 )
      throw std::invalid_argument( where( "setMaxIterations" ) + "the contact solver needs at "
                                   "least one iteration" );
   detail::trySetMaxIterations( *theCollisionSystem(), iterations );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the relaxation parameter of the contact solver.
 *
 * \param relaxation The relaxation parameter, positive and finite in pe::real (pe default 0.9;
 *        not the load under-relaxation of applyLoads()).
 * \return void
 * \exception std::runtime_error The compiled solver has no such parameter.
 * \exception std::invalid_argument The value is not positive or not finite (in pe::real).
 *
 * pe applies this parameter only to the velocity corrections exchanged between processes
 * (synchronizeVelocities()); on one process it has no effect.
 */
void Engine::setContactRelaxation( double relaxation )
{
   requireSetting( "setContactRelaxation", Setting::contactRelaxation,
                   "no setRelaxationParameter()" );
   const real value( relaxation );
   if( !finiteReal( relaxation ) || !( relaxation > 0.0 ) || !( value > real( 0 ) ) )
      throw std::invalid_argument( where( "setContactRelaxation" ) + "the contact relaxation "
                                   "must be positive and finite" );
   detail::trySetContactRelaxation( *theCollisionSystem(), value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Switches split impulses (penetration removal by pseudo velocities) on or off.
 *
 * \param enable \a true to enable split impulses.
 * \return void
 * \exception std::runtime_error The compiled solver has no split impulses.
 */
void Engine::setSplitImpulse( bool enable )
{
   requireSetting( "setSplitImpulse", Setting::splitImpulse, "no setSplitImpulse()" );
   detail::trySetSplitImpulse( *theCollisionSystem(), enable );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether split impulses are enabled.
 *
 * \return \a true if split impulses are enabled.
 * \exception std::runtime_error The compiled solver has no split impulses.
 */
bool Engine::splitImpulse() const
{
   requireSetting( "splitImpulse", Setting::splitImpulse, "no getSplitImpulse()" );
   bool enabled( false );
   detail::tryGetSplitImpulse( *theCollisionSystem(), enabled );
   return enabled;
}
//*************************************************************************************************



//=================================================================================================
//
//  DOMAIN DECOMPOSITION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Decomposes pe's domain into a chain of slabs, one per process (collective).
 *
 * \param planes The dividing planes in slab order: process k gets slab k, the positive side of
 *        plane k without the positive side of plane k-1 (see SlabPlane); n - 1 or n planes for
 *        n processes. Every process passes the same planes.
 * \return void
 * \exception std::invalid_argument A non-finite plane, a zero normal or a wrong number of planes.
 * \exception std::logic_error The domain was decomposed before, or bodies exist already.
 *
 * pe's decomposeDomainPlaneSlabs() (pe/interface/support/decompose.h): each process is
 * connected to the previous and the next slab only, so a body must never reach across a whole
 * slab. Must be called before the first body is created (also before planes). On one process
 * (or without MPI) the planes are validated and nothing else happens.
 */
void Engine::decomposeSlabs( const std::vector<SlabPlane>& planes )
{
   const char* const function( "decomposeSlabs" );
   requireNoExclusiveSection( function );
   const int n( processCount() );
   if( impl_->decomposed_ )
      throw std::logic_error( where( function ) + "the domain was decomposed before" );
   for( const SlabPlane& p : planes ) {
      if( !allFinite( p.point ) || !allFinite( p.normal ) )
         throw std::invalid_argument( where( function ) + "a plane is not finite" );
      const Vec3 normal( toPe( p.normal ) );
      if( !( normal.sqrLength() > real( 0 ) ) )
         throw std::invalid_argument( where( function ) + "a plane has a zero normal" );
   }
   if( n > 1 && planes.size() != std::size_t( n - 1 ) && planes.size() != std::size_t( n ) ) {
      std::ostringstream oss;
      oss << where( function ) << "a decomposition into " << n << " slabs needs " << ( n - 1 )
          << " or " << n << " planes, not " << planes.size();
      throw std::invalid_argument( oss.str() );
   }
   {
      // collective on several processes: every process refuses, or none
      int populated( theCollisionSystem()->getBodyStorage().size() > 0 ? 1 : 0 );
#if HAVE_MPI
      if( n > 1 )
         MPI_Allreduce( MPI_IN_PLACE, &populated, 1, MPI_INT, MPI_LOR, MPISettings::comm() );
#endif
      if( populated != 0 )
         throw std::logic_error( where( function ) + "bodies exist already (on at least one "
                                 "process); decompose the domain before creating bodies" );
   }
#if HAVE_MPI
   if( n > 1 ) {
      std::vector<HalfSpace> halfSpaces;
      for( const SlabPlane& p : planes )
         halfSpaces.push_back( HalfSpace( toPe( p.normal ), toPe( p.point ) ) );
      decomposeDomainPlaneSlabs( processRank(), halfSpaces, n );
      impl_->halfSpaces_ = halfSpaces;
      // all planes parallel (up to the orientation of the normal): the thinnest interior slab,
      // the distance of consecutive planes
      bool parallel( true );
      real thinnest( std::numeric_limits<real>::infinity() );
      for( std::size_t k=1; k<halfSpaces.size(); ++k ) {
         const Vec3& a( halfSpaces[k-1].getNormal() );
         const Vec3& b( halfSpaces[k].getNormal() );
         if( ( a % b ).length() > real( 1e-12 ) ) { parallel = false; break; }
         const real t( std::fabs( trans( b ) * toPe( planes[k-1].point ) -
                                  halfSpaces[k].getDisplacement() ) );
         thinnest = std::min( thinnest, t );
      }
      impl_->minInteriorSlab_ = parallel ? thinnest : std::numeric_limits<real>::infinity();
   }
#endif
   impl_->decomposed_ = true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the local subdomain owns a point (pe's ownership rule).
 *
 * \param point The point.
 * \return Whether a body centred at \a point would be created on (and owned by) this process.
 *         Exactly one process owns every point (ties go to the lower rank). Always true on one
 *         process.
 * \exception std::invalid_argument A non-finite point.
 */
bool Engine::ownsPoint( const Vec3d& point ) const
{
   if( !allFinite( point ) )
      throw std::invalid_argument( where( "ownsPoint" ) + "the point is not finite" );
   if( !parallelRun() )
      return true;
   return theCollisionSystem()->getDomain().ownsPoint( toPe( point ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The rank of this process in pe's communicator (0 without MPI).
 */
int Engine::processRank() const
{
   return MPISettings::rank();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The number of processes of pe's communicator (1 without MPI).
 */
int Engine::processCount() const
{
   return MPISettings::size();
}
//*************************************************************************************************




//=================================================================================================
//
//  CREATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creates a material in pe's global material table.
 *
 * \param name A unique material name.
 * \param density The material density.
 * \param restitution The coefficient of restitution.
 * \param staticFriction The static Coulomb friction coefficient.
 * \param dynamicFriction The dynamic Coulomb friction coefficient.
 * \return The material handle.
 * \exception std::invalid_argument A coefficient is not finite (checked here, before pe sees
 *            it), the name is in use or a coefficient is out of range (pe's checks).
 *
 * The parameters that only pe's soft-contact (DEM) solvers read are set to the values of the
 * FEAT3 wrapper: Poisson ratio 0.3, Young's modulus 2e6, stiffness 1e6, normal damping 1e3,
 * tangential damping 2e3. pe's materials are global and outlive the Engine.
 */
MaterialId Engine::createMaterial( const std::string& name, double density, double restitution,
                                   double staticFriction, double dynamicFriction )
{
   if( !finiteReal( density ) || !finiteReal( restitution ) || !finiteReal( staticFriction ) ||
       !finiteReal( dynamicFriction ) )
      throw std::invalid_argument( where( "createMaterial" ) + "a material coefficient is not "
                                   "finite" );
   const MaterialID material( pe::createMaterial( name, real( density ), real( restitution ),
                                                  real( staticFriction ), real( dynamicFriction ),
                                                  real( 0.3 ), real( 2.0e6 ), real( 1.0e6 ),
                                                  real( 1.0e3 ), real( 2.0e3 ) ) );
   return static_cast<MaterialId>( material );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creates a movable sphere.
 *
 * \param centre The initial centre.
 * \param radius The radius.
 * \param material The material.
 * \param userId The user id (pe's getID(); not required to be unique).
 * \return The system id of the new sphere.
 * \exception std::invalid_argument Invalid material, a non-finite or non-positive radius, a
 *            non-finite centre, a sphere too small to be represented at its centre in pe::real,
 *            or pe's checks (in MPI builds the centre must lie in the local subdomain).
 */
BodyId Engine::createSphere( const Vec3d& centre, double radius, MaterialId material,
                             std::uint64_t userId )
{
   if( material >= Material::count() )
      throw std::invalid_argument( where( "createSphere" ) + "invalid material handle" );
   if( !allFinite( centre ) || !finiteReal( radius ) || !( real( radius ) > real( 0 ) ) )
      throw std::invalid_argument( where( "createSphere" ) + "the sphere is not finite or has "
                                   "no positive radius" );
   checkRepresentable( "createSphere", centre, sphereBoxHalfExtent( real( radius ) ) );
   if( parallelRun() && impl_->decomposed_ ) {
      // pe keeps shadow copies on neighbouring slabs only: a sphere whose reach is not smaller
      // than the thinnest interior slab can span it and then needs a distant process
      const real reach( real( radius ) + contactThreshold + lubrication::getShadowCopyMargin() );
      if( !( reach < impl_->minInteriorSlab_ ) ) {
         std::ostringstream oss;
         oss << where( "createSphere" ) << "a sphere of radius " << radius << " has the reach "
             << reach << " (radius plus pe's process overlap), which is not smaller than the "
             << "thinnest interior slab of the decomposition (" << impl_->minInteriorSlab_
             << "); pe keeps shadow copies and migrates bodies only between neighbouring slabs, "
             << "so such a sphere could span a slab (missed contacts, a migration to a distant "
             << "process); use thicker slabs (fewer processes or other planes)";
         throw std::invalid_argument( oss.str() );
      }
   }
   impl_->prepared_ = false;
   const SphereID sphere( pe::createSphere( static_cast<id_t>( userId ), toPe( centre ),
                                            real( radius ), static_cast<MaterialID>( material ) ) );
   return sphere->getSystemID();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creates a fixed plane.
 *
 * \param normal The plane normal, pointing into the fluid (normalized by pe).
 * \param offset The offset \f$ d \f$ of the plane \f$ \hat n \cdot x = d \f$.
 * \param material The material.
 * \param userId The user id.
 * \return The system id of the new plane.
 * \exception std::invalid_argument Invalid material, a non-finite offset, or a normal whose
 *            squared length is not a finite normal (not subnormal) positive pe::real (pe's
 *            normalization would overflow or underflow).
 *
 * The half-space \f$ \hat n \cdot x < d \f$ behind the plane is solid. In an MPI run (also on
 * one process) a plane is a global body and must be created in a pe global section; the
 * function opens one itself unless one is open. On more than one process the call is
 * therefore collective in the sense that every process must make the same call (with the same
 * arguments, in the same order) so the plane gets the same id everywhere.
 */
BodyId Engine::createPlane( const Vec3d& normal, double offset, MaterialId material,
                            std::uint64_t userId )
{
   if( material >= Material::count() )
      throw std::invalid_argument( where( "createPlane" ) + "invalid material handle" );
   if( !allFinite( normal ) || !finiteReal( offset ) )
      throw std::invalid_argument( where( "createPlane" ) + "the plane is not finite" );
   const real nx( normal[0] ), ny( normal[1] ), nz( normal[2] );
   const real sq( nx*nx + ny*ny + nz*nz );
   if( !( sq >= std::numeric_limits<real>::min() && sq <= std::numeric_limits<real>::max() ) )
      throw std::invalid_argument( where( "createPlane" ) + "the plane normal cannot be "
                                   "normalized in pe::real (zero, too small or too large)" );

   PlaneID plane( nullptr );
   if( MPISettings::isParallel() && !GlobalSection::isActive() ) {
      pe_GLOBAL_SECTION {
         plane = pe::createPlane( static_cast<id_t>( userId ), nx, ny, nz, real( offset ),
                                  static_cast<MaterialID>( material ) );
      }
   }
   else {
      plane = pe::createPlane( static_cast<id_t>( userId ), nx, ny, nz, real( offset ),
                               static_cast<MaterialID>( material ) );
   }
   plane->setFixed( true );
   impl_->prepared_ = false;
   return plane->getSystemID();
}
//*************************************************************************************************




//=================================================================================================
//
//  ENUMERATION AND STATE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the number of bodies of a selection.
 *
 * \param selection The selection.
 * \return The number of bodies.
 */
std::size_t Engine::count( Selection selection ) const
{
   impl_->collect( selection, impl_->bodies_ );
   return impl_->bodies_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Lists the system ids of a selection, in storage order.
 *
 * \param selection The selection.
 * \param ids Output: replaced by the ids (its capacity is kept); unchanged on an exception.
 * \return void
 */
void Engine::bodyIds( Selection selection, std::vector<BodyId>& ids ) const
{
   impl_->collect( selection, impl_->bodies_ );
   ids.resize( impl_->bodies_.size() );   // strong guarantee; nothing below throws
   for( std::size_t i=0; i<impl_->bodies_.size(); ++i )
      ids[i] = impl_->bodies_[i]->getSystemID();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether a local body or shadow copy with this id exists.
 *
 * \param id The system id.
 * \return \a true if the body exists on this process.
 */
bool Engine::hasBody( BodyId id ) const
{
   bool shadow( false );
   return impl_->find( id, shadow ) != nullptr;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the state of a body.
 *
 * \param id The system id of a local body or shadow copy.
 * \return The state snapshot.
 * \exception std::invalid_argument There is no body with this id.
 */
BodyState Engine::state( BodyId id ) const
{
   bool shadow( false );
   const BodyID body( impl_->find( id, shadow ) );
   if( body == nullptr )
      throw std::invalid_argument( where( "state" ) + "no body with system id " + idString( id ) );
   BodyState s;
   impl_->fillState( body, shadow, s );
   return s;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the states of a selection, in storage order.
 *
 * \param selection The selection.
 * \param states Output: replaced by the states (its capacity is kept, so a vector reused from
 *        step to step is not reallocated); unchanged on an exception.
 * \return void
 *
 * Triangle-mesh volumes are computed once per body and cached, so the call costs O(1) per
 * body after the first.
 */
void Engine::states( Selection selection, std::vector<BodyState>& states ) const
{
   impl_->collect( selection, impl_->bodies_ );
   for( std::size_t i=0; i<impl_->bodies_.size(); ++i )   // fill the volume cache first
      impl_->volume( impl_->bodies_[i] );
   const bool shadow( selection == Selection::shadowCopies );
   states.resize( impl_->bodies_.size() );   // strong guarantee; nothing below throws
   for( std::size_t i=0; i<impl_->bodies_.size(); ++i )
      impl_->fillState( impl_->bodies_[i], shadow, states[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overrides the position, orientation and velocities of a body (e.g. for restarts).
 *
 * \param id The system id of a local finite body.
 * \param state The new kinematic state.
 * \return void
 * \exception std::invalid_argument Unknown id, an infinite body (plane), a non-finite state, a
 *            quaternion that is not unit within pe's accuracy, or a body too small to be
 *            represented at the new position.
 * \exception std::runtime_error On more than one process: a shadow copy, a global body (also a
 *            plane), or a position that neither the local nor a neighbouring subdomain
 *            contains -- not implemented yet. On any process: a global body inside a pe
 *            exclusive section (pe refuses to move it there).
 *
 * Without MOBILE_INFINITE pe ignores velocities set on fixed bodies. The hash grid sees the
 * new position after the next refreshSpatialIndex() or simulation step. On more than one
 * process the next advance() synchronizes before stepping: a body moved into a neighbouring
 * subdomain migrates there, and its shadow copies are updated, before the contact search.
 * Until then the body stays on this process and its shadow copies keep the old state. A
 * migrating body loses its coupling history (see LoadHistory).
 */
void Engine::setKinematicState( BodyId id, const KinematicState& state )
{
   const char* const function( "setKinematicState" );
   const BodyID body( impl_->findLocal( function, id ) );
   if( parallelRun() && body->isGlobal() )
      throwNotImplemented( function, "overriding the global body " + idString( id ) +
                           " on more than one process" );
   if( !body->isFinite() )
      throw std::invalid_argument( where( function ) + "body " + idString( id ) +
                                   " is infinite (a plane)" );
   if( body->isGlobal() && ExclusiveSection::isActive() )
      throw std::runtime_error( where( function ) + "the global body " + idString( id ) +
                                " cannot be moved inside a pe exclusive section" );
   validateKinematicState( function, state );
   checkRepresentablePose( function, body, state );

   if( parallelRun() && !impl_->reachable( toPe( state.position ) ) )
      throwNotImplemented( function, "moving body " + idString( id ) + " beyond the local and "
                           "the neighbouring subdomains (pe migrates bodies to neighbours only)" );

   impl_->overrideState( body, state );
   impl_->prepared_ = false;
   if( parallelRun() ) impl_->overridden_ = true;
}
//*************************************************************************************************




//=================================================================================================
//
//  POINT CLASSIFICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the owner of a point.
 *
 * \param point The point.
 * \param method The traversal.
 * \return The minimum system id of the bodies containing the point, or noBody.
 * \exception std::invalid_argument A non-finite coordinate, or (spatial index) a coordinate
 *            beyond what the hash grid can index.
 * \exception std::runtime_error QueryMethod::spatialIndex without a point query in the coarse
 *            detector.
 */
BodyId Engine::owner( const Vec3d& point, QueryMethod method ) const
{
   const double pos[3] = { point[0], point[1], point[2] };
   checkPoint( "owner", pos, queryBound( method ) );
   id_t ownerId( 0 );
   const bool found( method == QueryMethod::exhaustive
                   ? interface::detail::minimumOwnerExhaustive( pos, ownerId )
                   : interface::detail::minimumOwnerNearPoint( pos, impl_->candidates_, ownerId ) );
   return found ? static_cast<BodyId>( ownerId ) : noBody;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns every owner of a point.
 *
 * \param point The point.
 * \param ids Output: cleared, then the system ids of all bodies containing the point, ascending.
 * \param method The traversal.
 * \return void
 * \exception std::invalid_argument As owner() (then \a ids is unchanged).
 */
void Engine::owners( const Vec3d& point, std::vector<BodyId>& ids, QueryMethod method ) const
{
   static_assert( std::is_same<id_t,BodyId>::value, "pe::id_t must be a 64-bit unsigned integer" );

   const double pos[3] = { point[0], point[1], point[2] };
   checkPoint( "owners", pos, queryBound( method ) );
   if( method == QueryMethod::exhaustive )
      interface::detail::ownersExhaustive( pos, ids );
   else
      interface::detail::ownersNearPoint( pos, impl_->candidates_, ids );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Classifies a batch of points.
 *
 * \param xyz The points, interleaved (x0, y0, z0, x1, y1, z1, ...), 3 * numPoints values.
 * \param numPoints The number of points.
 * \param ownerIds Output: numPoints owners (minimum system id or noBody), caller-owned.
 * \param method The traversal.
 * \return void
 * \exception std::invalid_argument A null pointer with numPoints > 0, or an invalid point (as
 *            owner(); all points are checked before the first owner is written).
 */
void Engine::classify( const double* xyz, std::size_t numPoints, BodyId* ownerIds,
                       QueryMethod method ) const
{
   if( numPoints > 0 && ( xyz == nullptr || ownerIds == nullptr ) )
      throw std::invalid_argument( where( "classify" ) + "null point or owner array" );

   const double bound( queryBound( method ) );
   for( std::size_t i=0; i<numPoints; ++i )
      checkPoint( "classify", xyz + 3*i, bound );

   for( std::size_t i=0; i<numPoints; ++i ) {
      const double* pos( xyz + 3*i );
      id_t ownerId( 0 );
      const bool found( method == QueryMethod::exhaustive
                      ? interface::detail::minimumOwnerExhaustive( pos, ownerId )
                      : interface::detail::minimumOwnerNearPoint( pos, impl_->candidates_,
                                                                   ownerId ) );
      ownerIds[i] = found ? static_cast<BodyId>( ownerId ) : noBody;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether a body contains a point, by the rule of the classification.
 *
 * \param id The system id of a local body or shadow copy.
 * \param point The point.
 * \return \a true iff owners() of the point would list \a id; in particular \a false for
 *         planes, unions, fixed cylinders and fixed triangle meshes.
 * \exception std::invalid_argument There is no body with this id, or a non-finite point.
 */
bool Engine::contains( BodyId id, const Vec3d& point ) const
{
   bool shadow( false );
   const BodyID body( impl_->find( id, shadow ) );
   if( body == nullptr )
      throw std::invalid_argument( where( "contains" ) + "no body with system id " +
                                   idString( id ) );
   const double pos[3] = { point[0], point[1], point[2] };
   checkPoint( "contains", pos, std::numeric_limits<double>::infinity() );
   return interface::detail::bodyContainsPoint( body, pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests a batch of points against one body, with one id lookup for the whole batch.
 *
 * \param id The system id of a local body or shadow copy.
 * \param xyz The points, interleaved (x0, y0, z0, x1, ...), 3 * numPoints values.
 * \param numPoints The number of points.
 * \param flags Output: numPoints flags, 1 if the body contains the point (by the rule of
 *        contains()), 0 otherwise; caller-owned.
 * \return void
 * \exception std::invalid_argument There is no body with this id, a null pointer with
 *            numPoints > 0, or a non-finite point (all checked before the first flag is written).
 */
void Engine::containsPoints( BodyId id, const double* xyz, std::size_t numPoints,
                             unsigned char* flags ) const
{
   bool shadow( false );
   const BodyID body( impl_->find( id, shadow ) );
   if( body == nullptr )
      throw std::invalid_argument( where( "containsPoints" ) + "no body with system id " +
                                   idString( id ) );
   if( numPoints > 0 && ( xyz == nullptr || flags == nullptr ) )
      throw std::invalid_argument( where( "containsPoints" ) + "null point or flag array" );
   for( std::size_t i=0; i<numPoints; ++i )
      checkPoint( "containsPoints", xyz + 3*i, std::numeric_limits<double>::infinity() );

   for( std::size_t i=0; i<numPoints; ++i )
      flags[i] = interface::detail::bodyContainsPoint( body, xyz + 3*i ) ? 1u : 0u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Re-files every body of the hash grid into the cell of its current bounding box.
 *
 * \return void
 *
 * The hash grid files bodies in findContacts(), at the start of a simulation step, before the
 * positions are integrated; after advance() or setKinematicState() the filing is stale and a
 * QueryMethod::spatialIndex query can miss interior points of a body. Call this after every
 * change of positions and before spatial-index queries. It does no contact detection and no
 * physics. advance() does not call it, so the stepping arithmetic stays that of plain pe.
 */
void Engine::refreshSpatialIndex()
{
   detail::refreshSpatialIndex( theCollisionSystem()->getCoarseDetector() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether pe's hash grid is active.
 *
 * \return \a true if the grid is active.
 *
 * The grid activates once more than 32 bodies (gridActivationThreshold, per process) were
 * filed by a contact search; until then QueryMethod::spatialIndex tests every body (correct,
 * but O(number of bodies) per point).
 */
bool Engine::spatialIndexActive() const
{
   return detail::gridActive( theCollisionSystem()->getCoarseDetector() );
}
//*************************************************************************************************




//=================================================================================================
//
//  LOAD AND STEPPING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adds a hydrodynamic load to a body for the current coupling step.
 *
 * \param id The system id of a local movable body.
 * \param force The force (added to what was added before in this coupling step).
 * \param torque The torque about the centre of mass (likewise added).
 * \return void
 * \exception std::invalid_argument Unknown id, a fixed or infinite body, a non-finite load, or
 *            an accumulated load that would not be finite (the previous sum is kept).
 * \exception std::runtime_error The body is a shadow copy (on more than one process a load is
 *            added by the process that owns the body).
 *
 * The load must not contain the body forces the solver applies itself (see BuildInfo). To
 * relax a body against its history with a zero load in this step, add a zero load. On more
 * than one process each process adds the loads of its own bodies; the history of a body that
 * migrated stays behind (see LoadHistory, setLoadHistory()).
 */
void Engine::addLoad( BodyId id, const Vec3d& force, const Vec3d& torque )
{
   const BodyID body( impl_->findLocal( "addLoad", id ) );
   if( !isMovable( body ) )
      throw std::invalid_argument( where( "addLoad" ) + "body " + idString( id ) +
                                   " is fixed or infinite and takes no load" );
   if( !allFinite( force ) || !allFinite( torque ) )
      throw std::invalid_argument( where( "addLoad" ) + "the load of body " + idString( id ) +
                                   " is not finite" );

   impl_->prepared_ = false;   // a prepared checkpoint is no longer current (see prepareCheckpoint())
   const auto it( impl_->pending_.find( id ) );
   if( it == impl_->pending_.end() ) {
      // The first contribution is stored, not added to zero: 0 + (-0) would turn a negative
      // zero into a positive one and break the bit-for-bit agreement with a single load.
      impl_->pending_.emplace( id, LoadRecord{ force, torque } );
      return;
   }

   // Accumulate into a temporary: a sum that overflows is refused and the stored sum stays
   LoadRecord sum( it->second );
   for( std::size_t k=0; k<3; ++k ) {
      sum.force[k]  += force[k];
      sum.torque[k] += torque[k];
   }
   if( !allFinite( sum.force ) || !allFinite( sum.torque ) )
      throw std::invalid_argument( where( "addLoad" ) + "the accumulated load of body " +
                                   idString( id ) + " would not be finite" );
   it->second = sum;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declares that pe's world was replaced natively and now stands at a coupling-step
 *        boundary.
 *
 * \return void
 *
 * For use after a native reload of the world, e.g. by pe's binary checkpoint reader, which
 * destroys every body and restores pe's id counters: the restored world holds no impulse of
 * the current coupling step, and its ids may name other bodies than before. The call clears
 * the process-wide "loads applied, step not advanced yet" marker (so the next applyLoads() and
 * writeCheckpoint() are allowed), and this Engine's pending loads, load history (the next load
 * of every body is applied unrelaxed) and triangle-mesh volume cache. It is never called
 * implicitly; calling it while the world still holds an applied impulse would allow a second
 * one. The native-reload procedure: reload, create a new Engine, call this.
 *
 * pe's own fluid-force history of the bodies (RigidBody::hasPrevFluidForce(), which
 * applyFluidForces() relaxes against) is cleared on every local body as well, so the world
 * matches the Engine's empty load history: bodies created by pe's reader start without it
 * anyway (the reader does not save it), and a body that survived or was given a fluid force
 * natively since is reset to the same state. The kinematic state is not touched.
 */
void Engine::acknowledgeWorldReload()
{
   auto& storage( theCollisionSystem()->getBodyStorage() );
   for( auto it=storage.begin(); it!=storage.end(); ++it )
      it->setPrevFluidForce( false, Vec3(), Vec3() );
   impl_->pending_.clear();
   impl_->previous_.clear();
   impl_->meshVolumes_.clear();
   impl_->overridden_ = parallelRun();   // as after construction: synchronize before stepping
   loadsAppliedThisStep.store( false );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Discards the loads added in the current coupling step.
 *
 * \return void
 *
 * The previous loads (F_(n-1)) are kept.
 */
void Engine::clearLoads()
{
   impl_->pending_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether loads were added and not yet applied or cleared.
 *
 * \return \a true if loads are pending.
 */
bool Engine::hasPendingLoads() const
{
   return !impl_->pending_.empty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Applies the loads of the current coupling step, under-relaxed, as a velocity update.
 *
 * \param relaxation The under-relaxation factor r in (0, 1]; 1 applies F_n unchanged.
 * \param dt The coupling step size.
 * \return void
 * \exception std::invalid_argument Invalid relaxation or step size, a load was added for a
 *            body that no longer is a local movable body, or an effective load that is not
 *            finite in pe::real.
 * \exception std::logic_error Loads were already applied in this coupling step (before the
 *            next advance()).
 *
 * On more than one process the call is local: each process applies the loads added for its own
 * bodies, against the history the Engine of that process holds (restore it with
 * setLoadHistory() after a migration).
 *
 * Only the bodies with a pending load are processed, in storage order; every other body is
 * not touched (its force slot, velocity and load history stay as they are). For a processed
 * body, with F_n the sum of its loads added in this coupling step:
 * F_eff = r * F_n + (1 - r) * F_(n-1), and the same for the torque, where the first load a
 * body gets from this Engine is applied unrelaxed (F_eff = F_n). F_eff is set as the body
 * force and applied by RigidBody::applyFluidForces( dt, 1 ): v += dt F_eff / m,
 * w += dt I^-1 T_eff (with the body's DOF masks); the force slot is reset afterwards. F_n
 * becomes F_(n-1) (kept by the Engine as long as the body exists, also while it is fixed; part
 * of the checkpoint). The pending loads are cleared. Without pending loads the call does
 * nothing. Nothing is changed when an exception is thrown.
 */
void Engine::applyLoads( double relaxation, double dt )
{
   const char* const function( "applyLoads" );
   if( !std::isfinite( relaxation ) || relaxation <= 0.0 || relaxation > 1.0 )
      throw std::invalid_argument( where( function ) + "the relaxation must lie in (0, 1]" );
   if( !finiteReal( dt ) || dt <= 0.0 || !( real( dt ) > real( 0 ) ) )
      throw std::invalid_argument( where( function ) + "the step size must be positive" );
   if( impl_->pending_.empty() )
      return;
   if( loadsAppliedThisStep.load() )
      throw std::logic_error( where( function ) + "loads were already applied in this coupling "
                              "step (also by an earlier Engine handle); call advance() before "
                              "applying the next loads" );

   // ----- Prepare (everything that can throw) -----
   struct Planned
   {
      BodyID      body;
      LoadRecord  load;
      Vec3        force;      // F_eff in pe::real
      Vec3        torque;     // T_eff in pe::real
      LoadRecord* previous;   // nullptr: first load of this body
   };

   std::vector<BodyID>& bodies( impl_->bodies_ );
   impl_->collect( Selection::movable, bodies );

   std::vector<Planned> planned;
   planned.reserve( impl_->pending_.size() );
   std::unordered_map<BodyId,LoadRecord> fresh;   // nodes for the bodies' first loads

   for( std::size_t i=0; i<bodies.size(); ++i ) {
      const BodyId id( bodies[i]->getSystemID() );
      const auto pending( impl_->pending_.find( id ) );
      if( pending == impl_->pending_.end() )
         continue;

      const LoadRecord& load( pending->second );
      LoadRecord eff( load );
      LoadRecord* previousLoad( nullptr );
      const auto previous( impl_->previous_.find( id ) );
      if( previous != impl_->previous_.end() ) {
         // F_eff = r * F_n + (1 - r) * F_(n-1); same operations in the same order as the
         // FEAT3 wrapper
         const LoadRecord& prev( previous->second );
         for( int k=0; k<3; ++k ) {
            eff.force[k]  = relaxation * load.force[k]
                          + ( 1.0 - relaxation ) * prev.force[k];
            eff.torque[k] = relaxation * load.torque[k]
                          + ( 1.0 - relaxation ) * prev.torque[k];
         }
         previousLoad = &previous->second;
      }
      else {
         fresh.emplace( id, load );
      }

      const Planned p{ bodies[i], load, toPe( eff.force ), toPe( eff.torque ), previousLoad };
      if( !finitePe( p.force ) || !finitePe( p.torque ) )
         throw std::invalid_argument( where( function ) + "the effective load of body " +
                                      idString( id ) + " is not finite in pe::real" );
      planned.push_back( p );
   }

   if( planned.size() != impl_->pending_.size() ) {
      for( auto it=impl_->pending_.begin(); it!=impl_->pending_.end(); ++it ) {
         bool shadow( false );
         const BodyID body( impl_->find( it->first, shadow ) );
         if( body == nullptr || shadow || !isMovable( body ) )
            throw std::invalid_argument( where( function ) + "a load was added for body " +
                                         idString( it->first ) + ", which is no longer a local "
                                         "movable body" );
      }
   }

   // With enough buckets, merging the prepared nodes neither allocates nor rehashes
   impl_->previous_.reserve( impl_->previous_.size() + fresh.size() );

   // ----- Commit (no operation below throws) -----
   for( std::size_t i=0; i<planned.size(); ++i ) {
      const Planned& p( planned[i] );
      if( p.previous != nullptr )
         *p.previous = p.load;
      p.body->setForce( p.force );
      p.body->setTorque( p.torque );
      p.body->applyFluidForces( real( dt ), real( 1 ) );
   }
   impl_->previous_.merge( fresh );
   impl_->pending_.clear();
   loadsAppliedThisStep.store( true );
   impl_->prepared_ = false;
   // The shadow copies of these bodies still carry the velocities from before the impulses, and
   // pe's contact construction reads them (Contact::Contact(): the relative velocity selects
   // restitution and static or dynamic friction) before HardContactAndFluid synchronizes
   // velocities in the solve. advance() therefore synchronizes before the next pe step.
   if( parallelRun() ) impl_->overridden_ = true;

   // Forget the history of bodies that were destroyed (erase only)
   impl_->pruneHistory();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coupling history of a local finite body (see LoadHistory).
 *
 * \param id The system id of a local finite body.
 * \return The Engine's previous load of the body and pe's body-level fluid-force history.
 * \exception std::invalid_argument Unknown id or an infinite body.
 * \exception std::runtime_error A shadow copy.
 */
LoadHistory Engine::loadHistory( BodyId id ) const
{
   const BodyID body( impl_->findLocal( "loadHistory", id ) );
   if( !body->isFinite() )
      throw std::invalid_argument( where( "loadHistory" ) + "body " + idString( id ) +
                                   " is infinite (a plane)" );
   LoadHistory h;
   const auto previous( impl_->previous_.find( id ) );
   if( previous != impl_->previous_.end() ) {
      h.hasPrevious    = true;
      h.previousForce  = previous->second.force;
      h.previousTorque = previous->second.torque;
   }
   h.hasFluid    = body->hasPrevFluidForce();
   h.fluidForce  = fromPe( body->getPrevFluidForce() );
   h.fluidTorque = fromPe( body->getPrevFluidTorque() );
   return h;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Restores the coupling history of a local finite body (see LoadHistory).
 *
 * \param id The system id of a local finite body.
 * \param history The history, e.g. as loadHistory() returned it on the process that owned the
 *        body before it migrated.
 * \return void
 * \exception std::invalid_argument Unknown id, an infinite body or a non-finite history.
 * \exception std::runtime_error A shadow copy.
 *
 * Sets the Engine's previous load (or removes it, if \a history has none: the next load is then
 * applied unrelaxed) and pe's body-level fluid-force history exactly to the given values.
 * Nothing is changed when an exception is thrown.
 */
void Engine::setLoadHistory( BodyId id, const LoadHistory& history )
{
   const char* const function( "setLoadHistory" );
   const BodyID body( impl_->findLocal( function, id ) );
   if( !body->isFinite() )
      throw std::invalid_argument( where( function ) + "body " + idString( id ) +
                                   " is infinite (a plane)" );
   if( !allFinite( history.previousForce ) || !allFinite( history.previousTorque ) ||
       !allFinite( history.fluidForce ) || !allFinite( history.fluidTorque ) ||
       !finitePe( toPe( history.fluidForce ) ) || !finitePe( toPe( history.fluidTorque ) ) )
      throw std::invalid_argument( where( function ) + "the history of body " + idString( id ) +
                                   " is not finite" );
   if( history.hasPrevious ) {
      impl_->previous_.reserve( impl_->previous_.size() + 1 );
      impl_->previous_[id] = LoadRecord{ history.previousForce, history.previousTorque };
   }
   else {
      impl_->previous_.erase( id );
   }
   body->setPrevFluidForce( history.hasFluid, toPe( history.fluidForce ),
                            toPe( history.fluidTorque ) );
   impl_->prepared_ = false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Advances the world by one coupling step.
 *
 * \param dt The coupling step size.
 * \param substeps The number of pe steps of size dt / substeps (default 1).
 * \return void
 * \exception std::invalid_argument Invalid step size or zero substeps.
 * \exception std::logic_error Loads were added but neither applied nor cleared, or (on more
 *            than one process) the call is made inside a pe exclusive section; both are
 *            refused locally, before any collective.
 *
 * Each pe step does contact detection, the solver's velocity prediction (including the body
 * forces it owns), the contact solve and the position integration. The loads are not applied
 * here: applyLoads() applies them once per coupling step, before this call. On more than one
 * process the call is collective: before every pe step it asks pe (collectively) whether a
 * synchronization is required -- bodies were created, a state was overridden, or this is the
 * first step of a new Engine -- and synchronizes first, so shadow copies exist before the
 * contact search. If a pe step throws, the bodies may have moved partially (pe gives no
 * guarantee there).
 */
void Engine::advance( double dt, unsigned int substeps )
{
   requireNoExclusiveSection( "advance" );
   if( !finiteReal( dt ) || dt <= 0.0 )
      throw std::invalid_argument( where( "advance" ) + "the step size must be positive" );
   if( substeps == 0 )
      throw std::invalid_argument( where( "advance" ) + "the number of substeps must be positive" );
   const real h( real( dt ) / real( substeps ) );
   if( !( h > real( 0 ) ) )
      throw std::invalid_argument( where( "advance" ) + "the substep size is zero in pe::real" );
   {
      // collective on several processes: pending loads on one process make every process throw
      const std::string pending( impl_->pending_.empty() ? std::string() :
         where( "advance" ) + "loads were added but not applied (on process " +
         std::to_string( processRank() ) + "); call applyLoads() (or clearLoads()) before "
         "advance()" );
      const std::string agreed( parallelRun() ? agreeOnError( pending ) : pending );
      if( !agreed.empty() )
         throw std::logic_error( agreed );
   }

   detail::ActiveCollisionSystem& cs( *theCollisionSystem() );
   const DiagnosticsOffScope quiet( cs );
   for( unsigned int i=0; i<substeps; ++i ) {
      if( parallelRun() && impl_->decomposed_ && processCount() > 2 ) {
         // the reach and displacement rules of a slab decomposition, before anything moves
         // (collective; see Impl::reachError())
         std::string error;
         impl_->collect( Selection::movable, impl_->bodies_ );
         for( std::size_t b=0; b<impl_->bodies_.size() && error.empty(); ++b )
            error = impl_->reachError( impl_->bodies_[b], h );
         const std::string agreed( agreeOnError( error.empty() ? error :
                                                 where( "advance" ) + error ) );
         if( !agreed.empty() )
            throw std::runtime_error( agreed );
      }
      if( parallelRun() ) {
         bool needSync( detail::syncRequired( cs ) );   // collective
#if HAVE_MPI
         int overridden( impl_->overridden_ ? 1 : 0 );
         MPI_Allreduce( MPI_IN_PLACE, &overridden, 1, MPI_INT, MPI_LOR, MPISettings::comm() );
         needSync = needSync || ( overridden != 0 );
#endif
         if( needSync )
            World::synchronize();
         impl_->overridden_ = false;
      }
      impl_->world_->simulationStep( h );
   }
   loadsAppliedThisStep.store( false );
   impl_->prepared_ = false;
   impl_->pruneVolumes();
}
//*************************************************************************************************




//=================================================================================================
//
//  CHECKPOINT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes the kinematic state and the load history of all local finite bodies.
 *
 * \param os The stream (text). The finished checkpoint is written with one unformatted
 *        write(), so the stream's locale, format flags, width and fill play no role.
 * \return void
 * \exception std::logic_error Loads are pending, or loads were applied and advance() has not
 *            run yet (checkpoint between coupling steps).
 * \exception std::runtime_error A body the checkpoint cannot restore (a union or another
 *            composite/inner body, a moving plane), a non-finite state or load, a failed write,
 *            pe compiled with sleep mode or with a pe::real wider than double.
 * \exception std::logic_error On more than one process: called on a process other than the
 *            root, or without a current prepareCheckpoint().
 *
 * On more than one process the root process writes the text that prepareCheckpoint() gathered
 * (the records of all processes, sorted by system id); see prepareCheckpoint().
 *
 * Format version 3, plain text in the classic locale, 17 significant digits, so every double
 * is restored exactly. Every finite body carries its kinematic state, its previous load (the
 * Engine's raw load F_(n-1)) and pe's body-level fluid-force history (the flag and the
 * inputs force_, torque_ of the last RigidBody::applyFluidForces() call, which it stores as
 * given and relaxes against next time; with relaxation 1 they still decide the sign of a zero).
 * applyLoads() hands its already relaxed load in as those inputs, so through the Engine they
 * are the effective load; a native caller with a relaxation below 1 stores its unblended
 * force. The fixed ones too: a
 * body fixed for a while keeps its history; a fixed body moving with a prescribed velocity
 * under MOBILE_INFINITE keeps its motion. Each record is one line:

   \code
   pe-coupling-checkpoint 3
   bodies <n>
   body <id> <x y z> <r i j k> <vx vy vz> <wx wy wz> <hasPrevious> <Fx Fy Fz> <Tx Ty Tz>
        <hasPrevFluid> <fx fy fz> <tx ty tz>
   ...
   fixed <m>
   fixedbody <id> ... (the same fields)
   ...
   end
   \endcode

 * Only version 3 is read.
 */
void Engine::writeCheckpoint( std::ostream& os ) const
{
   const char* const function( "writeCheckpoint" );
   if( parallelRun() ) {
      if( !impl_->prepared_ )
         throw std::logic_error( where( function ) + "on more than one process, call "
                                 "prepareCheckpoint() on all processes first (and nothing that "
                                 "changes the world in between)" );
      if( processRank() != MPISettings::root() )
         throw std::logic_error( where( function ) + "on more than one process only the root "
                                 "process writes the checkpoint prepared by prepareCheckpoint()" );
      const std::string& text( impl_->preparedText_ );
      os.write( text.data(), static_cast<std::streamsize>( text.size() ) );
      if( !os )
         throw std::runtime_error( where( function ) + "writing the checkpoint failed" );
      return;
   }
   requireCheckpointBuild( function );
   if( !impl_->pending_.empty() )
      throw std::logic_error( where( function ) + "loads are pending; write the checkpoint "
                              "between coupling steps" );
   if( loadsAppliedThisStep.load() )
      throw std::logic_error( where( function ) + "loads were applied in this coupling step "
                              "(also by an earlier Engine handle); write the checkpoint after "
                              "advance()" );
   requireCheckpointScope( function );

   std::ostringstream out;
   out.imbue( std::locale::classic() );
   out.precision( 17 );
   out << checkpointMagic << ' ' << checkpointVersion << '\n';

   std::vector<BodyID>& bodies( impl_->bodies_ );
   impl_->collect( Selection::movable, bodies );
   out << "bodies " << bodies.size() << '\n';
   for( std::size_t i=0; i<bodies.size(); ++i )
      out << impl_->checkpointRecord( function, "body", bodies[i] );

   impl_->collectFixed( bodies );
   out << "fixed " << bodies.size() << '\n';
   for( std::size_t i=0; i<bodies.size(); ++i )
      out << impl_->checkpointRecord( function, "fixedbody", bodies[i] );
   out << "end\n";

   if( !out )
      throw std::runtime_error( where( function ) + "formatting the checkpoint failed" );
   const std::string text( out.str() );
   os.write( text.data(), static_cast<std::streamsize>( text.size() ) );
   if( !os )
      throw std::runtime_error( where( function ) + "writing the checkpoint failed" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prepares the checkpoint of a world spread over several processes (collective).
 *
 * \return void
 * \exception std::logic_error Loads are pending or were applied in this coupling step (on any
 *            process), or the call is made inside a pe exclusive section.
 * \exception std::runtime_error A world the checkpoint cannot restore, a non-finite record, or
 *            the reason of another process (see writeCheckpoint()).
 *
 * On one process (or without MPI) it does nothing: writeCheckpoint() builds the text itself. On
 * more than one process every process checks its own bodies and formats their records; the
 * processes agree on the outcome before anything is exchanged (all throw the message of the
 * lowest-ranked process that found an error, or none throws), then the
 * records are gathered on the root process (MPISettings::root()), which assembles the
 * checkpoint: the format of writeCheckpoint() (version 3), the records sorted by system id. The
 * root then writes it with writeCheckpoint(). The prepared text is a snapshot whose validity
 * ends with the next call that changes the world or the coupling state on ANY process. Only
 * the root's own changes are detected (creation, addLoad(), applyLoads(), advance(),
 * setKinematicState(), setLoadHistory(), readCheckpoint() discard it there; writeCheckpoint()
 * then throws); a change on another process is not, so a caller writes directly after
 * preparing, with no such call on any process in between (no collective validation).
 */
void Engine::prepareCheckpoint()
{
   const char* const function( "prepareCheckpoint" );
   requireNoExclusiveSection( function );
   impl_->prepared_ = false;
   impl_->preparedText_.clear();
   if( !parallelRun() )
      return;

#if HAVE_MPI
   std::string error, local;
   std::size_t numMovable( 0 ), numFixed( 0 );
   try {
      requireCheckpointBuild( function );
      if( !impl_->pending_.empty() )
         throw std::logic_error( where( function ) + "loads are pending; write the checkpoint "
                                 "between coupling steps" );
      if( loadsAppliedThisStep.load() )
         throw std::logic_error( where( function ) + "loads were applied in this coupling step; "
                                 "write the checkpoint after advance()" );
      requireCheckpointScope( function );
      std::vector<BodyID>& bodies( impl_->bodies_ );
      impl_->collect( Selection::movable, bodies );
      numMovable = bodies.size();
      for( std::size_t i=0; i<bodies.size(); ++i )
         local += "M " + impl_->checkpointRecord( function, "body", bodies[i] );
      impl_->collectFixed( bodies );
      numFixed = bodies.size();
      for( std::size_t i=0; i<bodies.size(); ++i )
         local += "F " + impl_->checkpointRecord( function, "fixedbody", bodies[i] );
   }
   catch( const std::exception& ex ) {
      error = ex.what();
   }
   const std::string agreed( agreeOnError( error ) );
   if( !agreed.empty() )
      throw std::runtime_error( agreed );
   UNUSED( numMovable );
   UNUSED( numFixed );

   // gather the records on the root
   const int root( MPISettings::root() );
   const int size( processCount() );
   int length( static_cast<int>( local.size() ) );
   std::vector<int> lengths( std::size_t( size ), 0 ), offsets( std::size_t( size ), 0 );
   MPI_Gather( &length, 1, MPI_INT, lengths.data(), 1, MPI_INT, root, MPISettings::comm() );
   std::string all;
   if( processRank() == root ) {
      int total( 0 );
      for( int r=0; r<size; ++r ) {
         offsets[std::size_t( r )] = total;
         total += lengths[std::size_t( r )];
      }
      all.resize( std::size_t( total ) );
   }
   MPI_Gatherv( local.data(), length, MPI_CHAR, processRank() == root ? &all[0] : nullptr,
                lengths.data(), offsets.data(), MPI_CHAR, root, MPISettings::comm() );

   if( processRank() == root ) {
      std::map<BodyId,std::string> movable, fixed;
      std::istringstream in( all );
      in.imbue( std::locale::classic() );
      std::string line;
      while( std::getline( in, line ) ) {
         std::istringstream ls( line.substr( 2 ) );
         ls.imbue( std::locale::classic() );
         std::string tag;
         BodyId id( 0 );
         ls >> tag >> id;
         auto& target( line[0] == 'M' ? movable : fixed );
         target[id] = line.substr( 2 ) + '\n';
      }
      std::ostringstream out;
      out.imbue( std::locale::classic() );
      out << checkpointMagic << ' ' << checkpointVersion << '\n';
      out << "bodies " << movable.size() << '\n';
      for( const auto& r : movable ) out << r.second;
      out << "fixed " << fixed.size() << '\n';
      for( const auto& r : fixed ) out << r.second;
      out << "end\n";
      impl_->preparedText_ = out.str();
   }
   impl_->prepared_ = true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Restores the kinematic states, the load history and pe's body-level fluid-force
 *        history written by writeCheckpoint().
 *
 * \param is The stream, positioned at the start of the checkpoint; it is read up to and
 *        including the "end" line, so further data may follow. It is parsed in the classic
 *        locale, decimal, skipping white space, whatever the caller set; the caller's locale and
 *        format flags are restored afterwards.
 * \return void
 * \exception std::runtime_error Not a coupling checkpoint, an unsupported version, a body count
 *            (movable or fixed) that differs from the world, a body id that is not a local
 *            body of the right kind in this world (or appears twice), a non-unit quaternion, a
 *            body that cannot be represented at its restored position, a truncated or malformed
 *            checkpoint, a world the checkpoint cannot restore (see writeCheckpoint()), a global
 *            body inside a pe exclusive section, pe compiled with sleep mode or a pe::real wider
 *            than double. On more than one process also: a record whose body is not held by
 *            exactly one process, a saved position that not exactly one process owns (beyond a
 *            closing plane, on an intersection of dividing planes), a non-sphere that has to
 *            move to another process, or a global finite body (not implemented yet); every
 *            process throws the message of the lowest-ranked process that found an error.
 *
 * On more than one process the call is collective: every process reads the same text, and each
 * record is restored on the process that owns its saved position under pe's ownership rule
 * (Domain::ownsPoint(): the process a running simulation would hold it on; a position on a
 * dividing plane belongs to the lower rank). A body that the rebuilt world holds elsewhere
 * (it migrated after its creation, possibly over several slabs) is moved there with its system
 * id, its state and both histories: the holder destroys it, the owner instantiates the sphere
 * with pe's migration parameters (user id, radius, shell, material, flags, surface velocity),
 * and all shadow copies are rebuilt (dropped and synchronized at once; the moved bodies get
 * theirs at the synchronization the next advance() performs before its first pe step).
 * Spheres only.
 *
 * Nothing in the world or in the Engine is changed when an exception is thrown (the stream may
 * have been read partially): the whole checkpoint is read and validated before the first body
 * is touched, and the commit cannot throw. The world must have been rebuilt exactly as when
 * the checkpoint was written (same creation calls in the same order, so the bodies get the
 * same system ids). Pending loads are cleared. When a restored run is bit-for-bit identical to
 * the uninterrupted one is described in doc/technical-notes/coupling-api.md.
 */
void Engine::readCheckpoint( std::istream& is )
{
   const char* const function( "readCheckpoint" );
   const bool parallel( parallelRun() );
   if( parallel )
      requireNoExclusiveSection( function );
   const StreamFormatScope formatScope( is );

   // On one process every error is thrown where it is found. On several, every process reads
   // the whole checkpoint, validates the records of its own bodies, and the processes agree on
   // the outcome (all throw, or all commit) before anything is changed.
   std::exception_ptr failure;
   std::size_t numMovable( 0 ), numFixed( 0 ), numMovableLocal( 0 ), numFixedLocal( 0 );

   const auto malformed = [&]() {
      return std::runtime_error( where( function ) + "truncated or malformed checkpoint" );
   };

   // every record, in checkpoint order (on several processes also the records of other processes)
   struct Record
   {
      BodyId         id;
      KinematicState state;
      bool           hasPrevious;
      LoadRecord     previous;
      bool           hasFluid;      //!< pe's body-level fluid-force history.
      Vec3d          fluidForce;
      Vec3d          fluidTorque;
      BodyID         body;          //!< The local body (nullptr: held by another process).
      int            owned;         //!< 1 if this process owns the saved position.
   };

   std::unordered_set<BodyId> seen;
   std::vector<Record> records;

   try {
   requireCheckpointBuild( function );
   requireCheckpointScope( function );
   std::string magic;
   int version( 0 );
   is >> magic >> version;
   if( !is || magic != checkpointMagic )
      throw std::runtime_error( where( function ) + "not a pe coupling checkpoint" );
   if( version != checkpointVersion ) {
      std::ostringstream oss;
      oss << where( function ) << "unsupported checkpoint version " << version
          << " (this pe reads version " << checkpointVersion << ")";
      throw std::runtime_error( oss.str() );
   }

   auto readRecord = [&]( const char* expectedTag, bool movable ) {
      std::string tag;
      Record r;
      r.id = 0;
      r.body = nullptr;
      r.owned = 1;
      is >> tag >> r.id;
      for( int k=0; k<3; ++k ) is >> r.state.position[k];
      for( int k=0; k<4; ++k ) is >> r.state.orientation[k];
      for( int k=0; k<3; ++k ) is >> r.state.linearVelocity[k];
      for( int k=0; k<3; ++k ) is >> r.state.angularVelocity[k];
      int hasPrevious( -1 );
      is >> hasPrevious;
      for( int k=0; k<3; ++k ) is >> r.previous.force[k];
      for( int k=0; k<3; ++k ) is >> r.previous.torque[k];
      int hasFluid( -1 );
      is >> hasFluid;
      for( int k=0; k<3; ++k ) is >> r.fluidForce[k];
      for( int k=0; k<3; ++k ) is >> r.fluidTorque[k];
      if( !is || tag != expectedTag || ( hasPrevious != 0 && hasPrevious != 1 ) ||
          ( hasFluid != 0 && hasFluid != 1 ) )
         throw malformed();
      r.hasPrevious = ( hasPrevious == 1 );
      r.hasFluid    = ( hasFluid == 1 );
      const BodyId id( r.id );

      bool shadow( false );
      const BodyID body( impl_->find( id, shadow ) );
      if( parallel && ( body == nullptr || shadow ) ) {
         // the body of another process (that it is held exactly once is checked collectively);
         // the record may still be restored here, if this process owns its saved position
         if( !seen.insert( id ).second )
            throw std::runtime_error( where( function ) + "body " + idString( id ) +
                                      " appears twice in the checkpoint" );
         try {
            validateKinematicState( function, r.state );
         }
         catch( const std::invalid_argument& ex ) {
            throw std::runtime_error( ex.what() );
         }
         if( !allFinite( r.previous.force ) || !allFinite( r.previous.torque ) ||
             !allFinite( r.fluidForce ) || !allFinite( r.fluidTorque ) )
            throw std::runtime_error( where( function ) + "body " + idString( id ) +
                                      " has a non-finite load history in the checkpoint" );
         records.push_back( r );
         return;
      }
      if( body == nullptr || shadow ||
          ( movable ? !isMovable( body ) : !isFixedFinite( body ) ) )
         throw std::runtime_error( where( function ) + "body " + idString( id ) + " of the "
                                   "checkpoint is not a local " +
                                   ( movable ? "movable" : "fixed finite" ) +
                                   " body of this world" );
      if( !seen.insert( id ).second )
         throw std::runtime_error( where( function ) + "body " + idString( id ) +
                                   " appears twice in the checkpoint" );
      if( body->isGlobal() && ExclusiveSection::isActive() )
         throw std::runtime_error( where( function ) + "the global body " + idString( id ) +
                                   " cannot be moved inside a pe exclusive section" );
      try {
         validateKinematicState( function, r.state );
         checkRepresentablePose( function, body, r.state );
      }
      catch( const std::invalid_argument& ex ) {
         throw std::runtime_error( ex.what() );
      }
      if( !allFinite( r.previous.force ) || !allFinite( r.previous.torque ) )
         throw std::runtime_error( where( function ) + "body " + idString( id ) +
                                   " has a non-finite previous load in the checkpoint" );
      if( !allFinite( r.fluidForce ) || !allFinite( r.fluidTorque ) )
         throw std::runtime_error( where( function ) + "body " + idString( id ) +
                                   " has a non-finite previous fluid force in the checkpoint" );
#if !( defined( MOBILE_INFINITE ) && MOBILE_INFINITE )
      // Without MOBILE_INFINITE pe's velocity setters skip fixed bodies, so the body keeps its
      // current velocity: a record (e.g. written by a build with MOBILE_INFINITE) whose
      // velocity is not bit for bit that one -- also only in the sign of a zero -- cannot be
      // restored
      if( !movable ) {
         const Vec3& v( body->getLinearVel() );
         const Vec3& w( body->getAngularVel() );
         for( int k=0; k<3; ++k )
            if( !sameBits( real( r.state.linearVelocity[k] ), v[k] ) ||
                !sameBits( real( r.state.angularVelocity[k] ), w[k] ) )
               throw std::runtime_error( where( function ) + "fixed body " + idString( id ) +
                                         " has a velocity in the checkpoint that this build "
                                         "(without MOBILE_INFINITE) cannot restore bit for bit" );
      }
#endif
      r.body = body;
      records.push_back( r );
   };

   auto readCount = [&]( const char* expectedTag, std::size_t expected, const char* what ) {
      std::string tag;
      std::size_t n( 0 );
      is >> tag >> n;
      if( !is || tag != expectedTag ) throw malformed();
      if( !parallel && n != expected ) {
         std::ostringstream oss;
         oss << where( function ) << "body count mismatch: the checkpoint has " << n << ' '
             << what << " bodies, the world has " << expected;
         throw std::runtime_error( oss.str() );
      }
      return n;
   };

   // ----- Read and validate everything -----
   impl_->collect( Selection::movable, impl_->bodies_ );
   numMovableLocal = impl_->bodies_.size();
   numMovable = readCount( "bodies", numMovableLocal, "movable" );
   impl_->collectFixed( impl_->bodies_ );
   numFixedLocal = impl_->bodies_.size();
   records.reserve( numMovable + numFixedLocal );
   for( std::size_t i=0; i<numMovable; ++i )
      readRecord( "body", true );

   numFixed = readCount( "fixed", numFixedLocal, "fixed finite" );
   for( std::size_t i=0; i<numFixed; ++i )
      readRecord( "fixedbody", false );

   std::string tag;
   is >> tag;
   if( !is || tag != "end" ) throw malformed();

   // the owner of every saved position under pe's own rule (the process a running simulation
   // migrates the body to; ties on a plane go to the lower rank)
   if( parallel ) {
      const Domain& domain( theCollisionSystem()->getDomain() );
      for( Record& r : records )
         r.owned = domain.ownsPoint( toPe( r.state.position ) ) ? 1 : 0;
   }
   }
   catch( ... ) {
      failure = std::current_exception();
   }

   if( !parallel ) {
      if( failure ) std::rethrow_exception( failure );
   }

   // Relocations: records whose saved position another process owns than the one that holds the
   // body now (a body that migrated after its creation, restored into a world rebuilt by the
   // creation calls). The holder sends the sphere's parameters, as pe's migration does.
   struct Relocation
   {
      BodyId        id;
      std::uint64_t uid;
      double        radius;
      double        shellThickness;
      double        surfaceVel[3];
      std::uint64_t material;
      int           visible;
      int           fixed;
      int           collisionEnabled;
      int           owner;
   };
   std::vector<int> holderOf( records.size(), MPISettings::rank() );
   std::vector<int> ownerOf ( records.size(), MPISettings::rank() );
   std::vector<Relocation> relocations;

   if( parallel ) {
#if HAVE_MPI
      std::string error;
      if( failure ) {
         try { std::rethrow_exception( failure ); }
         catch( const std::exception& ex ) { error = ex.what(); }
         catch( ... ) { error = where( function ) + "unknown error"; }
      }
      const std::string agreed( agreeOnError( error ) );
      if( !agreed.empty() )
         throw std::runtime_error( agreed );
      // every process read the same records; each body must be held by exactly one process and
      // its saved position owned by exactly one
      const int rank( MPISettings::rank() );
      const std::size_t n( records.size() );
      unsigned long counts[2] = { static_cast<unsigned long>( numMovableLocal ),
                                  static_cast<unsigned long>( numFixedLocal ) };
      MPI_Allreduce( MPI_IN_PLACE, counts, 2, MPI_UNSIGNED_LONG, MPI_SUM, MPISettings::comm() );
      std::vector<int> sums( 2*n ), ranks( 2*n );
      for( std::size_t r=0; r<n; ++r ) {
         sums[r]    = records[r].body != nullptr ? 1 : 0;
         sums[n+r]  = records[r].owned;
         ranks[r]   = records[r].body != nullptr ? rank : -1;
         ranks[n+r] = records[r].owned ? rank : -1;
      }
      if( n > 0 ) {
         MPI_Allreduce( MPI_IN_PLACE, sums.data(), static_cast<int>( 2*n ), MPI_INT, MPI_SUM,
                        MPISettings::comm() );
         MPI_Allreduce( MPI_IN_PLACE, ranks.data(), static_cast<int>( 2*n ), MPI_INT, MPI_MAX,
                        MPISettings::comm() );
      }
      std::ostringstream oss;
      if( counts[0] != numMovable || counts[1] != numFixed )
         oss << where( function ) << "body count mismatch: the checkpoint has " << numMovable
             << " movable and " << numFixed << " fixed finite bodies, the processes hold "
             << counts[0] << " and " << counts[1];
      for( std::size_t r=0; r<n && oss.str().empty(); ++r )
         if( sums[r] != 1 )
            oss << where( function ) << "body " << records[r].id << " of the checkpoint is held "
                << "by " << sums[r] << " processes (expected exactly one)";
      for( std::size_t r=0; r<n && oss.str().empty(); ++r )
         if( sums[n+r] != 1 )
            oss << where( function ) << "the saved position of body " << records[r].id
                << " is owned by " << sums[n+r] << " processes (expected exactly one: it lies "
                << "beyond a closing plane, or on an intersection of dividing planes)";
      if( !oss.str().empty() )
         throw std::runtime_error( oss.str() );
      for( std::size_t r=0; r<n; ++r ) {
         holderOf[r] = ranks[r];
         ownerOf[r]  = ranks[n+r];
      }

      // the holders describe the bodies that move to another process
      std::string local;
      std::vector<Relocation> mine;
      for( std::size_t r=0; r<n; ++r ) {
         if( holderOf[r] == ownerOf[r] || holderOf[r] != rank )
            continue;
         const BodyID body( records[r].body );
         if( body->getType() != sphereType ) {
            if( local.empty() )
               local = where( function ) + "body " + idString( records[r].id ) + " has to move "
                       "from process " + std::to_string( holderOf[r] ) + " to process " +
                       std::to_string( ownerOf[r] ) + ", which is implemented for spheres only";
            continue;
         }
         const ConstSphereID sphere( static_body_cast<const Sphere>( body ) );
         Relocation m;
         m.id = records[r].id;
         m.uid = static_cast<std::uint64_t>( body->getID() );
         m.radius = double( sphere->getRadius() );
         m.shellThickness = double( sphere->getShellThickness() );
         for( int k=0; k<3; ++k ) m.surfaceVel[k] = double( body->getRelSurfaceVel()[k] );
         m.material = static_cast<std::uint64_t>( sphere->getMaterial() );
         m.visible = body->isVisible() ? 1 : 0;
         m.fixed = body->isFixed() ? 1 : 0;
         m.collisionEnabled = body->isCollisionEnabled() ? 1 : 0;
         m.owner = ownerOf[r];
         mine.push_back( m );
      }
      if( local.empty() && !mine.empty() && !HasClearShadowCopiesActive::value )
         local = where( function ) + "moving a body to a distant process on restart needs a "
                 "collision system that can rebuild its shadow copies (clearShadowCopies())";
      const std::string agreedMove( agreeOnError( local ) );
      if( !agreedMove.empty() )
         throw std::runtime_error( agreedMove );

      // all descriptors to all processes (few bodies move)
      int count( static_cast<int>( mine.size() * sizeof( Relocation ) ) );
      std::vector<int> bytes( MPISettings::size() ), displs( MPISettings::size(), 0 );
      MPI_Allgather( &count, 1, MPI_INT, bytes.data(), 1, MPI_INT, MPISettings::comm() );
      int total( 0 );
      for( int p=0; p<MPISettings::size(); ++p ) { displs[p] = total; total += bytes[p]; }
      relocations.resize( std::size_t( total ) / sizeof( Relocation ) );
      MPI_Allgatherv( mine.data(), count, MPI_BYTE, relocations.data(), bytes.data(),
                      displs.data(), MPI_BYTE, MPISettings::comm() );
#endif
   }

   // ----- Commit (no operation below throws: pe's setters throw only for a global body inside
   // an exclusive section, which was refused above) -----
   impl_->pending_.clear();
   loadsAppliedThisStep.store( false );
   impl_->prepared_ = false;
   if( parallel ) impl_->overridden_ = true;

#if HAVE_MPI
   if( !relocations.empty() ) {
      // 1. drop every shadow copy (removal notifications, registrations cleared), destroy the
      //    moving bodies on their holders (no process is registered for them any more), and
      //    synchronize: the removals arrive, the remaining bodies get fresh shadow copies
      detail::tryClearShadowCopies( *theCollisionSystem() );
      std::unordered_set<BodyId> moving;
      for( const Relocation& m : relocations ) moving.insert( m.id );
      for( std::size_t r=0; r<records.size(); ++r ) {
         if( records[r].body != nullptr && moving.count( records[r].id ) ) {
            impl_->meshVolumes_.erase( records[r].id );
            destroy( records[r].body );
            records[r].body = nullptr;
         }
      }
      World::synchronize();
      // 2. the owners of the saved positions instantiate the spheres with their ids; the next
      //    advance() synchronizes before stepping and sends their shadow copies
      std::unordered_map<BodyId,const Relocation*> byId;
      for( const Relocation& m : relocations ) byId[m.id] = &m;
      for( std::size_t r=0; r<records.size(); ++r ) {
         const auto it( byId.find( records[r].id ) );
         if( it == byId.end() || it->second->owner != MPISettings::rank() )
            continue;
         const Relocation& m( *it->second );
         const KinematicState& st( records[r].state );
         const SphereID sphere( instantiateSphere(
            static_cast<id_t>( m.id ), static_cast<id_t>( m.uid ), toPe( st.position ), Vec3(),
            Quat( real( st.orientation[0] ), real( st.orientation[1] ), real( st.orientation[2] ),
                  real( st.orientation[3] ) ),
            real( m.radius ), static_cast<MaterialID>( m.material ), m.visible != 0,
            m.fixed != 0, true ) );
         sphere->setShellThickness( real( m.shellThickness ) );
         sphere->setCollisionEnabled( m.collisionEnabled != 0 );
         sphere->setRelSurfaceVel( Vec3( real( m.surfaceVel[0] ), real( m.surfaceVel[1] ),
                                         real( m.surfaceVel[2] ) ) );
         records[r].body = sphere;
      }
   }
#endif

   // 3. the records of the bodies this process now holds: state and both histories
   std::unordered_map<BodyId,LoadRecord> restored;
   for( std::size_t r=0; r<records.size(); ++r ) {
      const Record& e( records[r] );
      if( e.body == nullptr || ( parallel && ownerOf[r] != MPISettings::rank() ) )
         continue;
      if( e.hasPrevious ) restored.emplace( e.id, e.previous );
   }
   impl_->previous_.swap( restored );
   for( std::size_t r=0; r<records.size(); ++r ) {
      const Record& e( records[r] );
      if( e.body == nullptr || ( parallel && ownerOf[r] != MPISettings::rank() ) )
         continue;
      impl_->overrideState( e.body, e.state );
      e.body->setPrevFluidForce( e.hasFluid,
         Vec3( real( e.fluidForce[0] ), real( e.fluidForce[1] ), real( e.fluidForce[2] ) ),
         Vec3( real( e.fluidTorque[0] ), real( e.fluidTorque[1] ), real( e.fluidTorque[2] ) ) );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  FREE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reads the dividing planes of a slab decomposition from a text file.
 *
 * \param filename The plane file: one plane per line, "px py pz nx ny nz" (a point on the plane
 *        and its normal, taken as written); empty lines are skipped. This is the format of
 *        pe's loadSlabPlanes() (pe/interface/support/decompose.h) and of the pe_partpy strategy
 *        plane_single, so one file can partition the CFD mesh and decompose pe's domain.
 * \return The planes in file order (see Engine::decomposeSlabs()).
 * \exception std::runtime_error The file cannot be opened, a line does not hold six numbers, or
 *            a normal is zero or a number is not finite.
 */
std::vector<SlabPlane> readSlabPlanes( const std::string& filename )
{
   std::ifstream file( filename.c_str() );
   if( !file.is_open() )
      throw std::runtime_error( "pe::coupling::readSlabPlanes: could not open the plane file " +
                                filename );
   std::vector<SlabPlane> planes;
   std::string line;
   int lineNumber( 0 );
   while( std::getline( file, line ) ) {
      ++lineNumber;
      if( line.find_first_not_of( " \t\r" ) == std::string::npos )
         continue;
      std::istringstream iss( line );
      iss.imbue( std::locale::classic() );
      SlabPlane p;
      std::string rest;
      if( !( iss >> p.point[0] >> p.point[1] >> p.point[2] >> p.normal[0] >> p.normal[1]
                 >> p.normal[2] ) || ( iss >> rest ) )
         throw std::runtime_error( "pe::coupling::readSlabPlanes: line " +
                                   std::to_string( lineNumber ) + " of " + filename +
                                   " does not hold exactly six numbers (px py pz nx ny nz)" );
      if( !allFinite( p.point ) || !allFinite( p.normal ) ||
          !( p.normal[0]*p.normal[0] + p.normal[1]*p.normal[1] + p.normal[2]*p.normal[2] > 0.0 ) )
         throw std::runtime_error( "pe::coupling::readSlabPlanes: line " +
                                   std::to_string( lineNumber ) + " of " + filename +
                                   " holds a non-finite number or a zero normal" );
      planes.push_back( p );
   }
   return planes;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests points against a sphere with pe's sphere predicate, without a body.
 *
 * \param centre The centre.
 * \param radius The radius.
 * \param xyz The points, interleaved x0 y0 z0 x1 ...
 * \param numPoints The number of points.
 * \param flags Output: 1 where Sphere::containsPoint() of a sphere with this centre and radius
 *        accepts the point (|x - X|^2 <= r^2 in pe::real, the same expression), 0 elsewhere.
 * \return void
 * \exception std::invalid_argument Null pointers with numPoints > 0, or a non-finite argument.
 *
 * For callers that hold the states of bodies the local process does not hold (e.g. a coupling
 * whose processes see all bodies while pe distributes them).
 */
void sphereContainsPoints( const Vec3d& centre, double radius, const double* xyz,
                           std::size_t numPoints, unsigned char* flags )
{
   if( numPoints > 0 && ( xyz == nullptr || flags == nullptr ) )
      throw std::invalid_argument( "pe::coupling::sphereContainsPoints: null point or flag array" );
   if( !allFinite( centre ) || !std::isfinite( radius ) )
      throw std::invalid_argument( "pe::coupling::sphereContainsPoints: the sphere is not finite" );
   for( std::size_t i=0; i<3*numPoints; ++i )
      if( !std::isfinite( xyz[i] ) )
         throw std::invalid_argument( "pe::coupling::sphereContainsPoints: point " +
                                      std::to_string( i / 3 ) + " is not finite" );
   const Vec3 gpos_( toPe( centre ) );
   const real radius_( radius );
   for( std::size_t i=0; i<numPoints; ++i ) {
      const Vec3 gpos( real( xyz[3*i] ), real( xyz[3*i+1] ), real( xyz[3*i+2] ) );
      flags[i] = ( ( gpos - gpos_ ).sqrLength() <= ( radius_*radius_ ) ) ? 1u : 0u;
   }
}
//*************************************************************************************************

} // namespace coupling

} // namespace pe
