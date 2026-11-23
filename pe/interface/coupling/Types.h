//=================================================================================================
/*!
 *  \file pe/interface/coupling/Types.h
 *  \brief Value types of the typed C++ coupling API (pe::coupling)
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
 *  The coupling API headers (pe/interface/coupling/) include C++ standard library headers only.
 *  They define no macros except their include guards and PE_COUPLING_API (Export.h, symbol
 *  visibility, independent of every pe header) and nothing outside namespace pe::coupling, so
 *  a CFD code can include them in any translation unit, including one that also includes
 *  <mpi.h>. All pe types stay in the implementation (src/interface/coupling/).
 *
 *  Conventions used by every function of the API:
 *  - Units: pe is unit-agnostic. All quantities are expressed in one consistent unit system
 *    chosen by the caller (e.g. SI: m, kg, s, N, N m).
 *  - Vectors are Cartesian (x, y, z) in the global frame.
 *  - Quaternions are stored in the order (r, i, j, k), i.e. the real part first, and are
 *    unit quaternions that rotate body-frame vectors into the global frame.
 *  - Bodies are identified by pe's 64-bit system id (BodyId), which is assigned by pe when a
 *    body is created and never changes (see BodyId for when an id can be issued again).
 */
//=================================================================================================

#ifndef _PE_INTERFACE_COUPLING_TYPES_H_
#define _PE_INTERFACE_COUPLING_TYPES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>


namespace pe {

namespace coupling {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Identity of a rigid body: pe's 64-bit system id.
 * \ingroup coupling
 *
 * The system id is assigned by pe when a body is created (by this API or by any native pe
 * call) and is stable for the lifetime of the body. Ids are not reused within one process,
 * with one exception: pe's binary checkpoint reader (a native reload of the world) destroys
 * all bodies and restores pe's id counters, after which earlier ids can be issued again; an
 * Engine must be recreated after such a reload, and Engine::acknowledgeWorldReload() called. A world rebuilt in a fresh process by the same
 * sequence of creation calls gets the same ids, which is what makes the coupling checkpoints
 * restorable (see Engine::readCheckpoint()).
 */
typedef std::uint64_t  BodyId;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The "no body" value of BodyId.
 * \ingroup coupling
 *
 * Returned by the point classification for a point that lies in no body. pe never assigns this
 * value to a body (its id generator throws before reaching it).
 */
inline constexpr BodyId noBody = std::numeric_limits<BodyId>::max();
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle of a material, the index of the material in pe's global material table.
 * \ingroup coupling
 */
typedef std::size_t  MaterialId;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A Cartesian 3-vector (x, y, z) in the global frame.
 * \ingroup coupling
 */
typedef std::array<double,3>  Vec3d;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A unit quaternion in the order (r, i, j, k), real part first.
 * \ingroup coupling
 */
typedef std::array<double,4>  Quatd;
//*************************************************************************************************




//=================================================================================================
//
//  ENUMERATIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The geometric kind of a rigid body.
 * \ingroup coupling
 */
enum class ShapeKind {
   sphere,        //!< pe::Sphere (also hollow spheres).
   capsule,       //!< pe::Capsule.
   ellipsoid,     //!< pe::Ellipsoid.
   box,           //!< pe::Box.
   cylinder,      //!< pe::Cylinder.
   triangleMesh,  //!< pe::TriangleMesh.
   plane,         //!< pe::Plane (infinite).
   other          //!< Anything else: unions, inner cylinders, inner meshes.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Which bodies an enumeration (Engine::bodyIds(), Engine::states()) returns.
 * \ingroup coupling
 *
 * Bodies are returned in pe's storage order. That order is not stable when bodies are removed
 * (pe swaps the last body into the gap), so use the BodyId, not the position in the list, to
 * identify a body across steps.
 */
enum class Selection {
   movable,       //!< Local bodies that are finite and not fixed: the particles.
   all,           //!< All local bodies, including fixed bodies and planes.
   shadowCopies   //!< Shadow copies of bodies owned by another MPI process (empty without MPI).
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief How a point classification query finds the bodies to test.
 * \ingroup coupling
 *
 * Both methods apply the same containment test and the same ownership rule, so they return
 * the same owners as long as the spatial index is current (see Engine::refreshSpatialIndex()).
 */
enum class QueryMethod {
   exhaustive,    //!< Test every local body and every shadow copy: O(number of bodies) per point.
   spatialIndex   //!< Test only the candidates pe's hash grid files near the point.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Who applies a body force (gravity, buoyancy) with the compiled constraint solver.
 * \ingroup coupling
 */
enum class BodyForce {
   appliedBySolver,  //!< pe's solver applies it; the load handed to pe must not contain it.
   appliedByDriver,  //!< The solver does not apply it; the CFD driver must include it in the load.
   unknown           //!< The solver does not declare its semantics; nobody classified it.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A world setting whose availability depends on the compiled constraint solver.
 * \ingroup coupling
 *
 * Query availability with pe::coupling::isSupported(); a setter of an unsupported setting
 * throws std::runtime_error naming the setting and the solver.
 */
enum class Setting {
   gravity,            //!< Engine::setGravity(): the solver must apply gravity itself.
   fluidDensity,       //!< Engine::setFluidDensity(): the solver must apply buoyancy itself.
   fluidViscosity,     //!< Engine::setFluidViscosity(): always available.
   errorReduction,     //!< Engine::setErrorReduction(): contact error reduction parameter.
   maxIterations,      //!< Engine::setMaxIterations(): contact solver iteration limit.
   contactRelaxation,  //!< Engine::setContactRelaxation(): contact solver relaxation.
   splitImpulse        //!< Engine::setSplitImpulse() and Engine::splitImpulse().
};
//*************************************************************************************************




//=================================================================================================
//
//  STRUCTURES
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Snapshot of the state of one rigid body.
 * \ingroup coupling
 */
struct BodyState
{
   BodyId        id;               //!< pe system id.
   std::uint64_t userId;           //!< User id given at creation (pe's getID()).
   ShapeKind     shape;            //!< Geometric kind.
   Vec3d         position;         //!< Centre of mass (for a plane: its anchor point).
   Vec3d         linearVelocity;   //!< Linear velocity of the centre of mass.
   Vec3d         angularVelocity;  //!< Angular velocity in the global frame.
   Quatd         orientation;      //!< Orientation quaternion (r, i, j, k).
   double        mass;             //!< Mass as stored by pe.
   double        radius;           //!< Sphere radius; 0 for every other shape.
   double        volume;           //!< Volume enclosed by the surface (the displaced fluid
                                   //!< volume; a hollow sphere reports its full sphere) for
                                   //!< sphere, capsule, ellipsoid, box, cylinder and triangle
                                   //!< mesh; 0 where pe has none (plane, union, inner types).
   Vec3d         aabbMin;          //!< Lower corner of the axis-aligned bounding box.
   Vec3d         aabbMax;          //!< Upper corner of the axis-aligned bounding box.
   bool          fixed;            //!< Whether the body is fixed (infinite mass).
   bool          shadow;           //!< Whether this is a shadow copy of a remote body (MPI only).
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Kinematic state of a rigid body, as overridden by Engine::setKinematicState().
 * \ingroup coupling
 */
struct KinematicState
{
   Vec3d position;         //!< Centre of mass.
   Quatd orientation;      //!< Orientation quaternion (r, i, j, k), must be a unit quaternion.
   Vec3d linearVelocity;   //!< Linear velocity of the centre of mass.
   Vec3d angularVelocity;  //!< Angular velocity in the global frame.
};
//*************************************************************************************************

} // namespace coupling

} // namespace pe

#endif
