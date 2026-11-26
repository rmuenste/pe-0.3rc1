//=================================================================================================
/*!
 *  \file pe/interface/coupling/Engine.h
 *  \brief Typed C++ coupling API between a particle-resolved CFD code and pe
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
 *  This header includes C++ standard library headers only (see pe/interface/coupling/Types.h
 *  for the conventions: units, vectors, quaternion order, body identity). The design notes,
 *  pitfalls and the relation to the extern "C" interface are in
 *  doc/technical-notes/coupling-api.md.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_COUPLING_ENGINE_H_
#define _PE_INTERFACE_COUPLING_ENGINE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>
#include <pe/interface/coupling/BuildInfo.h>
#include <pe/interface/coupling/Export.h>
#include <pe/interface/coupling/Types.h>


//*************************************************************************************************
/*!rief Defined (as 1) by a pe whose coupling API runs a domain-decomposed world on several
 *        processes.
 * \ingroup coupling
 *
 * The functions this adds (Engine::decomposeSlabs(), Engine::ownsPoint(), Engine::processRank(),
 * Engine::processCount(), Engine::loadHistory(), Engine::setLoadHistory(),
 * Engine::prepareCheckpoint(), readSlabPlanes(), sphereContainsPoints()) are not present in older
 * pe versions with the coupling API; a client that supports both tests this macro.
 */
#define PE_COUPLING_DOMAIN_DECOMPOSITION 1
//*************************************************************************************************


namespace pe {

namespace coupling {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handle on pe's process-wide rigid-body world for fictitious-boundary-method coupling.
 * \ingroup coupling
 *
 * The Engine gives a CFD code typed access to pe's world: it configures the world, creates
 * materials, spheres and planes, enumerates bodies and their states, classifies points
 * (which body owns a mesh point), accumulates the hydrodynamic loads the CFD code computes,
 * applies them under-relaxed, advances the world and writes and reads checkpoints.
 *
 * \b One engine per process. pe's world is a process-wide singleton. The constructor throws
 * std::logic_error if another Engine exists in the process; after that Engine is destroyed a
 * new one may be created. The Engine does not own the bodies: it works on whatever is in the
 * world, including bodies created by native pe calls or by an existing pe setup function, and
 * destroying the Engine leaves the world and its bodies untouched. What the Engine owns is the
 * coupling state: the loads accumulated in the current coupling step, the previous load of
 * every body (for the under-relaxation) and a cache of triangle-mesh volumes. The constructor
 * does not change the configuration of the world (gravity, damping, ...).
 *
 * \b Replacing the handle forfeits the load history (the next load of every body is applied
 * unrelaxed) and the volume cache. What lives in the world survives: the "loads applied, step
 * not advanced yet" state is process-wide, so a new handle still refuses a second applyLoads()
 * and writeCheckpoint() until advance() has run (a successful readCheckpoint() is allowed and
 * clears that state, because it replaces the velocities).
 *
 * \b Native reload of the world. pe's binary checkpoint reader destroys all bodies and restores
 * pe's id counters, so ids can be issued again and the impulse of an interrupted coupling step
 * is gone with the old bodies. Procedure: reload, create a new Engine (an old one would keep
 * id-keyed history and caches of other bodies), then call acknowledgeWorldReload() to declare
 * that the world stands at a coupling-step boundary; it clears the process-wide marker, that
 * Engine's pending loads, history and caches, and pe's body-level fluid-force history of every
 * local body (bodies made by the reader start without it; pe's binary checkpoint does not save
 * it). The call is never made implicitly; without it the refusals above remain in force.
 *
 * \b Typical coupling step (serial mode: every CFD process holds the same complete world and
 * reduces the loads over all processes before handing them over):

   \code
   pe::coupling::Engine engine;
   ... configure, create bodies ...
   std::vector<pe::coupling::BodyState> bodies;
   for( step ... ) {
      engine.states( pe::coupling::Selection::movable, bodies );   // impose rigid velocities
      engine.refreshSpatialIndex();                                 // before spatialIndex queries
      engine.classify( xyz, n, owners, pe::coupling::QueryMethod::spatialIndex );
      ... solve the flow, integrate the loads ...
      for( each body b that the flow acts on )
         engine.addLoad( b.id, force, torque );
      engine.applyLoads( relaxation, dt );   // only the bodies that got a load
      engine.advance( dt, substeps );
   }
   \endcode

 * \b Planes. A plane is the set of points x with \f$ \hat n \cdot x = d \f$, where the normal
 * handed to createPlane() is normalized by pe and points into the fluid; the half-space
 * \f$ \hat n \cdot x < d \f$ behind the plane is solid. In an MPI run a plane is a global body:
 * createPlane() opens pe's global section itself, and on more than one process every process
 * must make the same createPlane() calls in the same order.
 *
 * \b Contact relaxation. setContactRelaxation() sets pe's relaxation of the velocity
 * corrections that are exchanged between processes; pe applies it only there, so on one
 * process it has no effect.
 *
 * \b Point ownership. A point is owned by the body containing it; a point inside several
 * bodies is owned by the body with the minimum system id. Tested are all local bodies and all
 * shadow copies, with this rule for the shapes: spheres, capsules, boxes and ellipsoids are
 * tested whether fixed or not; cylinders and triangle meshes are tested only when they are
 * not fixed (fixed ones are treated as geometry of the CFD domain, not as particles); planes,
 * unions and the inner cylinder and inner mesh types are never tested. The rule is shared
 * with the extern "C" queries (pointInsideParticles(), pointOwners(), ...). Containment is
 * pe's own predicate (containsPoint() of each shape). For triangle meshes in a build without
 * CGAL that predicate is valid for convex meshes only: it accepts every point within 1e-6 of
 * the mesh-frame origin and tests the face planes, so a concave movable mesh is classified
 * wrongly. pe does not expose whether a mesh is convex (and marks meshes it instantiates, e.g.
 * shadow copies and meshes restored from a binary checkpoint, as convex), so the API cannot
 * refuse such meshes; use convex movable meshes or a CGAL build.
 *
 * \b MPI builds. The engine works on the bodies of the local process. On one process it is
 * fully functional. On more than one process (pe's communicator; MPI must be initialized before
 * the Engine is constructed):
 * - decomposeSlabs() decomposes the domain into a chain of slabs, one per process (before any
 *   body is created; the two slabs at a plane share exactly its points, ties go to the lower
 *   rank); ownsPoint() tells which process creates a body (a body is created by the process
 *   that owns its centre; a caller that evaluates it on every process should still agree on
 *   one, e.g. the lowest claiming rank, before creating). pe keeps shadow copies on, and
 *   migrates bodies to, neighbouring slabs only, so: createSphere() refuses a sphere whose reach
 *   (radius plus pe's process overlap) is not smaller than the thinnest interior slab (a process
 *   on each side; parallel planes); and advance(), on three or more processes, refuses before
 *   every pe step, collectively, a local finite body (fixed ones included) whose reach touches a
 *   slab that is not a neighbour of its holder's (a contact there would be missed), or whose
 *   reach plus its displacement estimate d = (|v| + a h) h does (a the acceleration the solver
 *   applies; pe would delete it as an outflow), or that can reach a neighbouring slab j within
 *   d while its reach plus d touches a slab that is not a neighbour of j (the destination rule),
 *   or whose reach plus d crosses a closing plane (n planes for n processes: the region beyond
 *   it belongs to no process, and pe discards a contact whose point lies there; createSphere()
 *   refuses such a sphere too).
 *   d estimates a free body (contact impulses are not predicted). The decomposition and these
 *   rules are process-wide, like pe's domain: a replacement Engine handle keeps them; one
 *   decomposition per process lifetime (a native replacement of pe's MPI topology is not
 *   detected). After a pe step or a pre-step synchronization in which a body migrated into a
 *   process, advance() synchronizes before the next contact search, so that the new holder's
 *   further neighbours get its shadow copy in time (pe's migration sends copies only to the
 *   previous holder's neighbours). pe's migration does not carry a body's sleep state (it is
 *   awake on its new holder); with a pe built with sleep mode this is the caller's
 *   responsibility (the FEAT exchange refuses such a build);
 * - reading (enumeration, states, shadow copies via Selection::shadowCopies /
 *   BodyState::shadow, classification) works on the local bodies and shadow copies;
 *   sphereContainsPoints() evaluates pe's sphere predicate for a sphere the process does not
 *   hold;
 * - addLoad() and applyLoads() work locally: each process adds and applies the loads of its
 *   own bodies. A migrating body loses its coupling history (pe's migration does not carry it,
 *   see LoadHistory): read it with loadHistory() after applyLoads() and restore it on the new
 *   owner with setLoadHistory() before its next applyLoads(), and the continuation is bit for
 *   bit that of one process. applyLoads() changes velocities that shadow copies on other
 *   processes still hold the old values of, so the next advance() synchronizes before its first
 *   pe step (pe's contact construction reads them);
 * - advance() is collective (it synchronizes before a step when pe, applied loads or a state
 *   override need it; pending loads on any process make every process throw);
 * - setKinematicState() works on local bodies, also with shadow copies, to a position in the
 *   local or a neighbouring subdomain (pe migrates to neighbours only); a global body, a shadow
 *   copy or a farther position is refused ("not implemented yet");
 * - checkpoints: prepareCheckpoint() (collective) gathers the records on the root process,
 *   which writes them with writeCheckpoint() (version 3, records sorted by id). The prepared
 *   text is a snapshot: its validity ends with the next call that changes the world or the
 *   coupling state on ANY process; the root notices only its own changes (creation, addLoad(),
 *   applyLoads(), advance(), setKinematicState(), setLoadHistory(), readCheckpoint()), so write
 *   it directly after preparing, with no such call on any process in between.
 *   readCheckpoint() is collective (every process gets the same text) and restores each record
 *   on the process that owns its saved position (pe's rule), moving a sphere there with its id
 *   and histories if the rebuilt world holds it elsewhere; a global finite body is refused
 *   before anything changes.
 *
 * \b Checkpoints. Scope: finite non-union bodies plus planes at rest, with empty force and
 * torque slots, written between coupling steps. Both checkpoint functions refuse a world with
 * a moving plane, a union or any other body of ShapeKind::other, a finite body with a nonzero
 * force or torque slot, and builds with sleep mode or a pe::real wider than double;
 * readCheckpoint() also refuses a record this build cannot restore exactly (without
 * MOBILE_INFINITE: a fixed body whose recorded velocity is not bit for bit the one it keeps, also
 * in the sign of a zero). Two separate
 * statements (doc/technical-notes/coupling-api.md):
 * 1. Always, and tested: readCheckpoint() restores bit for bit the positions, orientations,
 *    velocities and load history that writeCheckpoint() saved, including pe's body-level
 *    fluid-force history (RigidBody::hasPrevFluidForce(), which applyLoads() goes through).
 * 2. Only conditionally: the continuation is bit-for-bit identical to the uninterrupted run if
 *    pe processes the contacts in the same order and with the same geometry. That holds in the
 *    worlds the restart tests use, and is known NOT to hold in general when the hash grid is
 *    active, when bodies were created or destroyed after stepping began, or when a plane's
 *    stored geometry differs from what its creation call reproduces. Everything the checkpoint
 *    does not save -- planes and other infinite bodies, materials, world settings, the order of
 *    the bodies inside pe's coarse detector -- must be reproduced by the caller's setup.
 *
 * \b Errors are reported by exceptions; the Engine writes nothing to stdout or stderr. It is
 * not thread-safe. What a throwing call leaves behind, per operation:
 * - Everything not listed below: nothing changed (strong guarantee) -- world, Engine state and
 *   the caller's output buffers.
 * - refreshSpatialIndex(): pe's guarantee; on an allocation failure the index can be
 *   incomplete (pe removes a body from its old cell before adding it to the new one).
 * - createMaterial(): pe's material table registers a material before extending its tables;
 *   if one of those allocations fails the material may stay registered (pe's guarantee). The
 *   API's own checks run before pe sees the arguments.
 * - createSphere(), createPlane(): the API's checks and pe's argument checks (e.g. a centre
 *   outside the local subdomain) throw before the body is registered. If an allocation fails
 *   during pe's registration, pe's own (weaker) guarantee applies: pe files the body with the
 *   coarse detector before adding it to the body storage.
 * - owners(): \a ids is cleared first, so on an exception it may be empty.
 * - classify(), containsPoints(): invalid arguments (null pointers, an invalid point) are found
 *   before the first result is written. An allocation failure later on (the spatial index
 *   collects candidates per point; CGAL mesh predicates may allocate) can leave the results of
 *   earlier points written.
 * - advance(): its checks run before the first step (on several processes collectively: the
 *   pending loads and the reach rules, so every process throws together); if a pe step throws,
 *   the bodies may have moved partially (pe gives no guarantee).
 * - writeCheckpoint(): the world and the Engine never change; the text is built in memory and
 *   written with one unformatted write(), so on a stream failure the stream may hold a part.
 * - readCheckpoint(): world and Engine unchanged (everything is validated before the first
 *   change, and the commit cannot throw); the stream may have been consumed up to the error.
 *   Exception: on several processes, when a sphere has to move to another process, the commit
 *   destroys, synchronizes and instantiates; an exception there is fatal (state unspecified).
 *
 * \b Precision. The API is supported and tested with pe built in double precision. Values are
 * still checked after their conversion to pe::real; BuildInfo reports the real type.
 */
class PE_COUPLING_API Engine
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Engine();
            Engine( const Engine& ) = delete;
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~Engine();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   Engine& operator=( const Engine& ) = delete;
   //@}
   //**********************************************************************************************

   //**World configuration functions***************************************************************
   /*!\name World configuration functions */
   //@{
   void  setGravity           ( const Vec3d& gravity );
   Vec3d gravity              () const;
   void  setFluidDensity      ( double density );
   void  setFluidViscosity    ( double viscosity );
   void  setErrorReduction    ( double erp );
   void  setMaxIterations     ( std::size_t iterations );
   void  setContactRelaxation ( double relaxation );  // pe applies it only between processes
                                                     // (velocity sync); no effect on one
   void  setSplitImpulse      ( bool enable );
   bool  splitImpulse         () const;
   //@}
   //**********************************************************************************************

   //**Domain decomposition functions**************************************************************
   /*!\name Domain decomposition functions */
   //@{
   void decomposeSlabs( const std::vector<SlabPlane>& planes );
   bool ownsPoint     ( const Vec3d& point ) const;
   int  processRank   () const;
   int  processCount  () const;
   //@}
   //**********************************************************************************************

   //**Creation functions**************************************************************************
   /*!\name Creation functions */
   //@{
   MaterialId createMaterial( const std::string& name, double density, double restitution,
                              double staticFriction, double dynamicFriction );
   BodyId     createSphere  ( const Vec3d& centre, double radius, MaterialId material,
                              std::uint64_t userId = 0 );
   BodyId     createPlane   ( const Vec3d& normal, double offset, MaterialId material,
                              std::uint64_t userId = 0 );
   //@}
   //**********************************************************************************************

   //**Enumeration and state functions*************************************************************
   /*!\name Enumeration and state functions */
   //@{
   std::size_t count            ( Selection selection ) const;
   void        bodyIds          ( Selection selection, std::vector<BodyId>& ids ) const;
   bool        hasBody          ( BodyId id ) const;
   BodyState   state            ( BodyId id ) const;
   void        states           ( Selection selection, std::vector<BodyState>& states ) const;
   void        setKinematicState( BodyId id, const KinematicState& state );
   //@}
   //**********************************************************************************************

   //**Point classification functions**************************************************************
   /*!\name Point classification functions */
   //@{
   BodyId owner              ( const Vec3d& point, QueryMethod method ) const;
   void   owners             ( const Vec3d& point, std::vector<BodyId>& ids,
                               QueryMethod method ) const;
   void   classify           ( const double* xyz, std::size_t numPoints, BodyId* ownerIds,
                               QueryMethod method ) const;
   bool   contains           ( BodyId id, const Vec3d& point ) const;
   void   containsPoints     ( BodyId id, const double* xyz, std::size_t numPoints,
                               unsigned char* flags ) const;
   void   refreshSpatialIndex();
   bool   spatialIndexActive () const;
   //@}
   //**********************************************************************************************

   //**Load and stepping functions*****************************************************************
   /*!\name Load and stepping functions */
   //@{
   void addLoad               ( BodyId id, const Vec3d& force, const Vec3d& torque );
   void clearLoads            ();
   bool hasPendingLoads       () const;
   void applyLoads            ( double relaxation, double dt );
   LoadHistory loadHistory    ( BodyId id ) const;
   void setLoadHistory        ( BodyId id, const LoadHistory& history );
   void advance               ( double dt, unsigned int substeps = 1 );
   void acknowledgeWorldReload();
   //@}
   //**********************************************************************************************

   //**Checkpoint functions************************************************************************
   /*!\name Checkpoint functions */
   //@{
   void prepareCheckpoint();
   void writeCheckpoint  ( std::ostream& os ) const;
   void readCheckpoint   ( std::istream& is );
   //@}
   //**********************************************************************************************

private:
   //**Type definitions****************************************************************************
   struct Impl;  //!< The coupling state and the pe-typed implementation.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::unique_ptr<Impl> impl_;  //!< The implementation (pimpl).
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

//=================================================================================================
//
//  FREE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Free functions of the coupling API */
//@{
PE_COUPLING_API std::vector<SlabPlane> readSlabPlanes( const std::string& filename );

PE_COUPLING_API void sphereContainsPoints( const Vec3d& centre, double radius, const double* xyz,
                                           std::size_t numPoints, unsigned char* flags );
//@}
//*************************************************************************************************

} // namespace coupling

} // namespace pe

#endif
