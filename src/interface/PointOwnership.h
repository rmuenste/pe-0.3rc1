//=================================================================================================
/*!
 *  \file src/interface/PointOwnership.h
 *  \brief Point containment and ownership rule shared by all CFD point classification queries
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
 *  Internal header (not installed, not part of any public API). It is the single definition of
 *  "which body owns a point" used by the extern "C" queries in src/interface/object_queries.cpp
 *  (pointInsideParticles(), pointInsideParticlesAccelerated(), pointOwners(),
 *  pointOwnersAccelerated()) and by pe::coupling::Engine (src/interface/coupling/Engine.cpp),
 *  so the paths cannot drift apart:
 *
 *  - bodyContainsPoint(): the shape dispatch. Spheres, capsules, boxes and ellipsoids are
 *    tested fixed or not; cylinders and triangle meshes only when not fixed (the FBM treats
 *    fixed ones as geometry, not as particles); every other type (planes, unions, inner
 *    cylinders, inner meshes) never contains a point.
 *  - the ownership rule: a point inside several bodies is owned by the body with the MINIMUM
 *    system id. The minimum does not depend on traversal order, so the exhaustive traversal and
 *    the hash-grid traversal agree by construction whenever they test the same bodies.
 *  - the two traversals: exhaustive (all local bodies, then all shadow copies) and near-point
 *    (the candidates of the coarse detector's getBodiesNearPoint()).
 */
//=================================================================================================

#ifndef _PE_SRC_INTERFACE_POINTOWNERSHIP_H_
#define _PE_SRC_INTERFACE_POINTOWNERSHIP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <pe/core.h>


namespace pe {

namespace interface {

namespace detail {

//=================================================================================================
//
//  DETECTION TRAITS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Detects whether a coarse detector offers the point query getBodiesNearPoint().
 */
template< typename CD, typename = void >
struct HasPointQuery : std::false_type
{};

template< typename CD >
struct HasPointQuery< CD, std::void_t< decltype( std::declval<const CD&>().getBodiesNearPoint(
   real(), real(), real(), std::declval< std::vector<BodyID>& >() ) ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!rief Detects whether a collision system keeps its shadow copies in a separate storage
 *        (getBodyShadowCopyStorage()).
 */
template< typename CS, typename = void >
struct HasShadowStorage : std::false_type
{};

template< typename CS >
struct HasShadowStorage< CS, std::void_t<
   decltype( std::declval<CS&>().getBodyShadowCopyStorage() ) > > : std::true_type
{};
//*************************************************************************************************




//=================================================================================================
//
//  CONTAINMENT AND OWNERSHIP RULE
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Does this body contain the point (in the sense of the FBM classification)?
 *
 * \param body The body to test.
 * \param pos The point.
 * \return \a true if the body takes part in the classification and contains the point.
 *
 * Fixed cylinders and fixed triangle meshes are skipped: the FBM treats them as geometry, not
 * as particles. Planes, unions and every other type never contain a point.
 */
inline bool bodyContainsPoint( ConstBodyID body, const double pos[3] )
{
   const GeomType t( body->getType() );
   if( t == sphereType )
      return static_cast<const Sphere*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   if( t == capsuleType )
      return static_cast<const Capsule*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   if( t == boxType )
      return static_cast<const Box*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   if( t == ellipsoidType )
      return static_cast<const Ellipsoid*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   if( t == cylinderType )
      return !body->isFixed() &&
             static_cast<const Cylinder*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   if( t == triangleMeshType )
      return !body->isFixed() &&
             static_cast<const TriangleMesh*>( body )->containsPoint( pos[0], pos[1], pos[2] );
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Keeps the smallest system id seen so far (the ownership rule for overlapping bodies).
 *
 * \param id The system id of a body containing the point.
 * \param found In: whether an owner was seen before; out: \a true.
 * \param ownerId In/out: the smallest system id seen so far.
 * \return void
 */
inline void ownerCandidate( id_t id, bool& found, id_t& ownerId )
{
   if( !found || id < ownerId ) {
      ownerId = id;
      found   = true;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sorts an owner list ascending and removes duplicates.
 *
 * \param owners The system ids of the bodies containing a point.
 * \return void
 */
inline void finishOwners( std::vector<id_t>& owners )
{
   std::sort( owners.begin(), owners.end() );
   owners.erase( std::unique( owners.begin(), owners.end() ), owners.end() );
}
//*************************************************************************************************




//=================================================================================================
//
//  TRAVERSALS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Visits every shadow copy of a collision system, in storage order (none if it keeps no
 *        separate shadow storage).
 *
 * A template on the collision-system type, so the discarded branch is never instantiated.
 *
 * \param cs The collision system.
 * \param visit Callable taking a BodyID.
 * \return void
 */
template< typename CS, typename Visitor >
inline void forEachShadowCopy( CS& cs, Visitor visit )
{
   if constexpr( HasShadowStorage<CS>::value ) {
      for( auto it=cs.getBodyShadowCopyStorage().begin();
           it!=cs.getBodyShadowCopyStorage().end(); ++it )
         visit( static_cast<BodyID>( *it ) );
   }
   else {
      (void)cs; (void)visit;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds a shadow copy by system id.
 *
 * \param cs The collision system.
 * \param id The system id.
 * \return The shadow copy, or nullptr.
 */
template< typename CS >
inline BodyID findShadowCopy( CS& cs, id_t id )
{
   if constexpr( HasShadowStorage<CS>::value ) {
      const auto it( cs.getBodyShadowCopyStorage().find( id ) );
      return ( it != cs.getBodyShadowCopyStorage().end() ) ? static_cast<BodyID>( *it ) : nullptr;
   }
   else {
      (void)cs; (void)id;
      return nullptr;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Visits every local body and then every shadow copy, in storage order.
 *
 * \param visit Callable taking a ConstBodyID.
 * \return void
 */
template< typename Visitor >
inline void forEachClassifiedBody( Visitor visit )
{
   CollisionSystem<Config>& cs( *theCollisionSystem() );
   for( auto it=cs.getBodyStorage().begin(); it!=cs.getBodyStorage().end(); ++it )
      visit( static_cast<ConstBodyID>( *it ) );
   forEachShadowCopy( cs, [&]( BodyID body ) { visit( static_cast<ConstBodyID>( body ) ); } );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Collects the candidates a coarse detector files near a point.
 *
 * \param cd The coarse detector (a template parameter, so a detector without a point query
 *        compiles and throws).
 * \param pos The point.
 * \param candidates Output: cleared, then filled with the candidate bodies (may repeat bodies).
 * \return void
 * \exception std::runtime_error The coarse detector has no point query.
 *
 * With pe's HashGrids the candidates are the bodies filed in the 27 cells around the point in
 * every grid of the hierarchy, all non-grid bodies (planes, or every body while the grid is
 * inactive) and all bodies added since the last findContacts() (bodiesToAdd_).
 */
template< typename CD >
inline void bodiesNearPoint( const CD& cd, const double pos[3], std::vector<BodyID>& candidates )
{
   candidates.clear();
   if constexpr( HasPointQuery<CD>::value ) {
      cd.getBodiesNearPoint( static_cast<real>( pos[0] ), static_cast<real>( pos[1] ),
                             static_cast<real>( pos[2] ), candidates );
   }
   else {
      (void)cd; (void)pos;
      throw std::runtime_error( "The configured coarse collision detector has no point query "
                                "(getBodiesNearPoint); spatial-index classification is "
                                "unavailable" );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The owner of a point by the exhaustive traversal.
 *
 * \param pos The point.
 * \param ownerId Output: the minimum system id of the bodies containing the point.
 * \return \a true if a body contains the point (then \a ownerId is set).
 */
inline bool minimumOwnerExhaustive( const double pos[3], id_t& ownerId )
{
   bool found( false );
   forEachClassifiedBody( [&]( ConstBodyID body ) {
      if( bodyContainsPoint( body, pos ) ) ownerCandidate( body->getSystemID(), found, ownerId );
   } );
   return found;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The owner of a point by the near-point (spatial index) traversal.
 *
 * \param pos The point.
 * \param candidates Scratch vector for the candidates (reused to avoid allocations).
 * \param ownerId Output: the minimum system id of the bodies containing the point.
 * \return \a true if a body contains the point (then \a ownerId is set).
 */
inline bool minimumOwnerNearPoint( const double pos[3], std::vector<BodyID>& candidates,
                                   id_t& ownerId )
{
   bodiesNearPoint( theCollisionSystem()->getCoarseDetector(), pos, candidates );
   bool found( false );
   for( std::vector<BodyID>::const_iterator it=candidates.begin(); it!=candidates.end(); ++it )
      if( bodyContainsPoint( *it, pos ) ) ownerCandidate( (*it)->getSystemID(), found, ownerId );
   return found;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Every owner of a point by the exhaustive traversal, ids ascending.
 *
 * \param pos The point.
 * \param owners Output: cleared, then the system ids of all bodies containing the point.
 * \return void
 */
inline void ownersExhaustive( const double pos[3], std::vector<id_t>& owners )
{
   owners.clear();
   forEachClassifiedBody( [&]( ConstBodyID body ) {
      if( bodyContainsPoint( body, pos ) ) owners.push_back( body->getSystemID() );
   } );
   finishOwners( owners );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Every owner of a point by the near-point (spatial index) traversal, ids ascending.
 *
 * \param pos The point.
 * \param candidates Scratch vector for the candidates (reused to avoid allocations).
 * \param owners Output: cleared, then the system ids of all bodies containing the point.
 * \return void
 */
inline void ownersNearPoint( const double pos[3], std::vector<BodyID>& candidates,
                             std::vector<id_t>& owners )
{
   bodiesNearPoint( theCollisionSystem()->getCoarseDetector(), pos, candidates );
   owners.clear();
   for( std::vector<BodyID>::const_iterator it=candidates.begin(); it!=candidates.end(); ++it )
      if( bodyContainsPoint( *it, pos ) ) owners.push_back( (*it)->getSystemID() );
   finishOwners( owners );
}
//*************************************************************************************************

} // namespace detail

} // namespace interface

} // namespace pe

#endif
