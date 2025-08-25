//=================================================================================================
/*!
 *  \file pe/core/rigidbody/trianglemeshtrait/Default.h
 *  \brief Header file for the default implementation of the TriangleMeshTrait class template.
 *
 *  Copyright (C) 2013-2014 Tobias Scharpff
 *  Copyright (C) 2025-*?   Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_CORE_RIGIDBODY_TRIANGLEMESHTRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_TRIANGLEMESHTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/TriangleMeshBase.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>

// Forward declaration for DistanceMap (avoid circular dependency)
namespace pe {
   class DistanceMap;
}

// CGAL includes - only when CGAL is enabled
#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Ray_3.h>
#include <CGAL/Random.h>
#include <vector>
#include <map>
#include <memory>
#include <random>

// Forward declaration in header, actual include would go in implementation files
// But since this is a template class, we need the include here
// #include "DistanceMap.h" // This will be included by implementation files that use these methods
#endif



namespace pe {

//=================================================================================================
//
//  CGAL TAG SYSTEM FOR OPTIONAL INTEGRATION
//
//=================================================================================================

namespace detail {

#ifdef PE_USE_CGAL
   struct HasCgalTag {};
#else
   struct NoCgalTag {};
#endif

//*************************************************************************************************
/*!\brief Tag-dispatched CGAL functions for triangle mesh operations.
 * \ingroup triangleMesh
 *
 * This struct provides a clean interface for CGAL-dependent operations through tag dispatch.
 * When CGAL is not available, the default implementation provides no-op behavior.
 */
template<typename Tag>
struct CgalFunctions
{
   // Default (no CGAL) implementation for convex hull
   static bool computeConvexHull(const Vertices& input_vertices, 
                                Vertices& hull_vertices, 
                                IndicesLists& hull_faces) {
      // No-op implementation when CGAL is not available
      hull_vertices.clear();
      hull_faces.clear();
      return false;
   }
   
   // Default (no CGAL) implementation for AABB tree/distance operations
   static real distanceToPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      return real(0);
   }
   
   static Vec3 closestPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      return point;
   }
   
   static std::pair<Vec3, size_t> closestPointAndPrimitive(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      return std::make_pair(point, 0);
   }
   
   static void enableDistanceAcceleration(size_t maxReferencePoints) {
      // No-op when CGAL is not available
   }
   
   static void invalidateAABBTree() {
      // No-op when CGAL is not available
   }
   
   // Default (no CGAL) implementation for point containment operations
   static bool containsPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      return false;  // Conservative: assume point is outside when CGAL not available
   }
   
   static real signedDistance(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      return real(0);  // Fallback: return unsigned distance
   }
};

#ifdef PE_USE_CGAL
//*************************************************************************************************
/*!\brief CGAL-enabled specialization for convex hull computation.
 * \ingroup triangleMesh
 */
template<>
struct CgalFunctions<HasCgalTag>
{
   typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
   typedef Kernel::Point_3 Point_3;
   typedef Kernel::Triangle_3 Triangle_3;
   typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
   
   // AABB tree typedefs
   typedef CGAL::AABB_triangle_primitive<Kernel, std::vector<Triangle_3>::const_iterator> Primitive;
   typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
   typedef CGAL::AABB_tree<Traits> AABBTree;
   
   static bool computeConvexHull(const Vertices& input_vertices, 
                                Vertices& hull_vertices, 
                                IndicesLists& hull_faces) {
      if (input_vertices.size() < 4) {
         // Need at least 4 points for a 3D convex hull
         return false;
      }
      
      try {
         // Convert PE vertices to CGAL points
         std::vector<Point_3> cgal_points;
         cgal_points.reserve(input_vertices.size());
         
         for (const auto& vertex : input_vertices) {
            cgal_points.emplace_back(vertex[0], vertex[1], vertex[2]);
         }
         
         // Compute convex hull using CGAL
         Polyhedron hull_polyhedron;
         CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), hull_polyhedron);
         
         if (hull_polyhedron.empty()) {
            return false;
         }
         
         // Convert CGAL polyhedron back to PE format
         hull_vertices.clear();
         hull_faces.clear();
         
         // Create vertex mapping
         std::map<typename Polyhedron::Vertex_const_handle, size_t> vertex_map;
         size_t vertex_index = 0;
         
         // Extract vertices
         for (auto v_it = hull_polyhedron.vertices_begin(); 
              v_it != hull_polyhedron.vertices_end(); ++v_it) {
            const Point_3& point = v_it->point();
            hull_vertices.emplace_back(CGAL::to_double(point.x()), 
                                      CGAL::to_double(point.y()), 
                                      CGAL::to_double(point.z()));
            vertex_map[v_it] = vertex_index++;
         }
         
         // Extract faces
         for (auto f_it = hull_polyhedron.facets_begin(); 
              f_it != hull_polyhedron.facets_end(); ++f_it) {
            auto he = f_it->facet_begin();
            Vector3<size_t> face_indices;
            
            // Get the three vertices of the triangle face
            face_indices[0] = vertex_map[he->vertex()];
            ++he;
            face_indices[1] = vertex_map[he->vertex()];
            ++he;
            face_indices[2] = vertex_map[he->vertex()];
            
            hull_faces.push_back(face_indices);
         }
         
         return true;
         
      } catch (const std::exception& e) {
         // CGAL operation failed
         hull_vertices.clear();
         hull_faces.clear();
         return false;
      }
   }
   
   // AABB tree and distance computation methods
   static void buildAABBTree(const Vertices& vertices, const IndicesLists& faces,
                            std::vector<Triangle_3>& triangles, 
                            std::unique_ptr<AABBTree>& aabbTree,
                            bool distanceAccelerationEnabled) {
      // Clear and rebuild triangle list
      triangles.clear();
      triangles.reserve(faces.size());
      
      // Convert PE triangle data to CGAL triangles
      for (const auto& face : faces) {
         if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
            const Vec3& v0 = vertices[face[0]];
            const Vec3& v1 = vertices[face[1]];
            const Vec3& v2 = vertices[face[2]];
            
            Point_3 p0(v0[0], v0[1], v0[2]);
            Point_3 p1(v1[0], v1[1], v1[2]);
            Point_3 p2(v2[0], v2[1], v2[2]);
            
            triangles.emplace_back(p0, p1, p2);
         }
      }
      
      // Create and build the AABB tree
      aabbTree = std::make_unique<AABBTree>(triangles.begin(), triangles.end());
      
      // Enable distance acceleration if requested
      if (distanceAccelerationEnabled) {
         aabbTree->accelerate_distance_queries();
      }
   }
   
   static real distanceToPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      // For stateless operation, build tree on-demand (not optimal but consistent with tag dispatch)
      std::vector<Triangle_3> triangles;
      std::unique_ptr<AABBTree> aabbTree;
      buildAABBTree(vertices, faces, triangles, aabbTree, false);
      
      if (!aabbTree || triangles.empty()) {
         return real(0);
      }
      
      Point_3 query(point[0], point[1], point[2]);
      return static_cast<real>(aabbTree->squared_distance(query));
   }
   
   static Vec3 closestPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      // For stateless operation, build tree on-demand (not optimal but consistent with tag dispatch)
      std::vector<Triangle_3> triangles;
      std::unique_ptr<AABBTree> aabbTree;
      buildAABBTree(vertices, faces, triangles, aabbTree, false);
      
      if (!aabbTree || triangles.empty()) {
         return point;
      }
      
      Point_3 query(point[0], point[1], point[2]);
      Point_3 closest = aabbTree->closest_point(query);
      return Vec3(CGAL::to_double(closest.x()), 
                  CGAL::to_double(closest.y()), 
                  CGAL::to_double(closest.z()));
   }
   
   static std::pair<Vec3, size_t> closestPointAndPrimitive(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      // For stateless operation, build tree on-demand (not optimal but consistent with tag dispatch)
      std::vector<Triangle_3> triangles;
      std::unique_ptr<AABBTree> aabbTree;
      buildAABBTree(vertices, faces, triangles, aabbTree, false);
      
      if (!aabbTree || triangles.empty()) {
         return std::make_pair(point, 0);
      }
      
      Point_3 query(point[0], point[1], point[2]);
      auto result = aabbTree->closest_point_and_primitive(query);
      Point_3 closest = result.first;
      
      // The primitive iterator points to our triangle in the triangles vector
      size_t primitive_id = std::distance(triangles.cbegin(), result.second);
      
      return std::make_pair(
         Vec3(CGAL::to_double(closest.x()), 
              CGAL::to_double(closest.y()), 
              CGAL::to_double(closest.z())),
         primitive_id
      );
   }
   
   static void enableDistanceAcceleration(size_t maxReferencePoints) {
      // This is handled in the stateful TriangleMeshTrait implementation
      // Tag dispatch version doesn't maintain state
   }
   
   static void invalidateAABBTree() {
      // This is handled in the stateful TriangleMeshTrait implementation
      // Tag dispatch version doesn't maintain state
   }
   
   // Point containment using Jordan Curve theorem with ray shooting
   static bool containsPoint(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      if (faces.empty()) {
         return false;
      }
      
      try {
         // Build AABB tree on-demand for ray shooting
         std::vector<Triangle_3> triangles;
         std::unique_ptr<AABBTree> aabbTree;
         buildAABBTree(vertices, faces, triangles, aabbTree, false);
         
         if (!aabbTree || triangles.empty()) {
            return false;
         }
         
         // Use ray shooting to count intersections (Jordan Curve theorem)
         Point_3 queryPoint(point[0], point[1], point[2]);
         
         // Use a single reliable ray direction (positive X-axis is usually good)
         // but offset it slightly to avoid edge cases
         typedef CGAL::Ray_3<Kernel> Ray_3;
         Ray_3 ray(queryPoint, typename Kernel::Vector_3(1.0, 0.001, 0.0001));
         
         // Use number_of_intersected_primitives for cleaner counting
         std::size_t numIntersections = aabbTree->number_of_intersected_primitives(ray);
         
         // Point is inside if odd number of intersections
         return (numIntersections % 2) == 1;
         
      } catch (const std::exception& e) {
         // CGAL operation failed, try a fallback approach
         try {
            // Fallback: simple bounding box check + distance-based heuristic
            Point_3 queryPoint(point[0], point[1], point[2]);
            std::vector<Triangle_3> triangles;
            std::unique_ptr<AABBTree> aabbTree;
            buildAABBTree(vertices, faces, triangles, aabbTree, false);
            
            if (!aabbTree) return false;
            
            // If the point is very close to the surface, use distance to determine
            real sqDist = static_cast<real>(aabbTree->squared_distance(queryPoint));
            
            // For points very close to surface, use a different approach
            if (sqDist < 1e-10) {
               // Point is on or very close to surface, check using multiple rays
               typedef CGAL::Ray_3<Kernel> Ray_3;
               std::vector<typename Kernel::Vector_3> directions = {
                  typename Kernel::Vector_3(1.0, 0.0, 0.0),
                  typename Kernel::Vector_3(0.0, 1.0, 0.0),
                  typename Kernel::Vector_3(-1.0, 0.0, 0.0)
               };
               
               int insideVotes = 0;
               for (const auto& dir : directions) {
                  Ray_3 testRay(queryPoint, dir);
                  std::size_t intersections = aabbTree->number_of_intersected_primitives(testRay);
                  if (intersections % 2 == 1) insideVotes++;
               }
               
               return insideVotes >= 2; // Majority vote
            }
            
            return false; // Conservative fallback
         } catch (...) {
            return false;
         }
      }
   }
   
   // Signed distance: negative inside, positive outside
   static real signedDistance(const Vertices& vertices, const IndicesLists& faces, const Vec3& point) {
      // First compute unsigned distance
      real unsignedDist = distanceToPoint(vertices, faces, point);
      
      // Then determine sign using point containment
      bool inside = containsPoint(vertices, faces, point);
      
      return inside ? -std::sqrt(unsignedDist) : std::sqrt(unsignedDist);
   }
};
#endif

} // namespace detail


//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Triangle mesh customization class for the collision response.
 * \ingroup triangleMesh
 *
 * The TriangleMeshBaseTrait class template is a customization class for the triangle mesh geometry. Its main
 * purpose is the customization of the triangle mesh class for the selected collision response
 * algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class TriangleMeshTrait : public TriangleMeshBase
{
protected:
   //**Type definitions****************************************************************************
   typedef TriangleMeshBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit TriangleMeshTrait( id_t sid, id_t uid, const Vec3& gpos,
                         const Vertices& vertices, const IndicesLists& faceIndices,
                         MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~TriangleMeshTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void move( real dt );
   //@}
   //**********************************************************************************************

   //**CGAL Integration functions******************************************************************
   /*!\name CGAL Integration functions */
   //@{
   bool computeConvexHull(Vertices& hull_vertices, IndicesLists& hull_faces) const;
   real distanceToPoint(const Vec3& point) const;
   Vec3 closestPoint(const Vec3& point) const;
   std::pair<Vec3, size_t> closestPointAndPrimitive(const Vec3& point) const;
   void enableDistanceAcceleration(size_t maxReferencePoints = 100000);
   void invalidateAABBTree();
   bool containsPoint(const Vec3& point) const;
   real signedDistance(const Vec3& point) const;
   
   // DistanceMap acceleration methods
   void enableDistanceMapAcceleration(pe::real spacing, int resolution = 50, int tolerance = 5);
   void disableDistanceMapAcceleration();
   bool hasDistanceMap() const;
   const DistanceMap* getDistanceMap() const;
   //@}
   //**********************************************************************************************

private:
   //**CGAL AABB Tree members***********************************************************************
   /*!\name CGAL AABB Tree members */
   //@{
#ifdef PE_USE_CGAL
   mutable std::unique_ptr<detail::CgalFunctions<detail::HasCgalTag>::AABBTree> aabbTree_;
   mutable std::vector<detail::CgalFunctions<detail::HasCgalTag>::Triangle_3> triangles_;
   mutable bool aabbTreeValid_;
   mutable bool distanceAccelerationEnabled_;
   mutable size_t maxReferencePoints_;
#endif
   //@}
   //**********************************************************************************************

   //**DistanceMap members**************************************************************************
   /*!\name DistanceMap members */
   //@{
#ifdef PE_USE_CGAL
   mutable std::unique_ptr<DistanceMap> distanceMap_;
   mutable bool distanceMapEnabled_;
#endif
   //@}
   //**********************************************************************************************

   //**CGAL AABB Tree helper methods***************************************************************
   /*!\name CGAL AABB Tree helper methods */
   //@{
   void buildAABBTree() const;
   void ensureAABBTreeValid() const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the TriangleMeshTrait constructor.
 *
 * \param sid Unique system-specific ID for the triangle mesh.
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param vertices The list of vertices that build the surface
 * \param faceIndices The list of indices which assign three elements of vertices to one triangle
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 */
template< typename C >  // Type of the configuration
TriangleMeshTrait<C>::TriangleMeshTrait( id_t sid, id_t uid, const Vec3& gpos,
                                 const Vertices& vertices, const IndicesLists& faceIndices,
                                 MaterialID material, bool visible )
   : Parent( sid, uid, gpos, vertices, faceIndices, material, visible )  // Initialization of the parent class
#ifdef PE_USE_CGAL
   , aabbTreeValid_(false)
   , distanceAccelerationEnabled_(false)
   , maxReferencePoints_(100000)
   , distanceMapEnabled_(false)
#endif
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the TriangleMeshTrait destructor.
 */
template< typename C >  // Type of the configuration
TriangleMeshTrait<C>::~TriangleMeshTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * Calculating one single time step of size \a dt for the triangle mesh. The global position, the
 * linear and angular velocity and the orientation of the triangle mesh are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::move( real dt )
{
   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );
   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the triangle mesh according to the acting forces (don't move a sleeping triangle mesh)
   if( awake_ ) {
      if( !fixed_ ) {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the triangle mesh,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the triangle mesh are full matrices of the form
         //                            ( Ixx Ixy Ixz )
         //                            ( Iyx Iyy Iyz )
         //                            ( Izx Izy Izz )
         const Vec3 wdot( R_ * ( Iinv_ * ( trans(R_) * torque_ ) ) );

         // Updating the linear velocity
         v_ += vdot * dt;

         // Updating the angular velocity
         w_ += wdot * dt;
      }

      // Calculating the translational displacement
      gpos_ += v_ * dt;

      // Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v_ *= drag;
         w_ *= drag;
      }

      // Setting the axis-aligned bounding box
      TriangleMeshBase::calcBoundingBox();

      // Invalidate AABB tree since the mesh has moved
      invalidateAABBTree();

      // Calculating the current motion of the triangle mesh
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the convex hull of the triangle mesh vertices using CGAL.
 *
 * \param hull_vertices Output parameter for the convex hull vertices.
 * \param hull_faces Output parameter for the convex hull face indices.
 * \return True if the convex hull was computed successfully, false otherwise.
 *
 * This function computes the convex hull of the current triangle mesh's vertices using CGAL
 * (if available). When CGAL is not available, the function returns false and clears the output
 * parameters. The implementation uses tag dispatch to select the appropriate behavior at
 * compile time based on whether PE_USE_CGAL is defined.
 */
template< typename C >  // Type of the configuration
bool TriangleMeshTrait<C>::computeConvexHull(Vertices& hull_vertices, IndicesLists& hull_faces) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // Get the vertices from the base class
   const Vertices& input_vertices = this->getBFVertices();
   
   // Use tag dispatch to call the appropriate implementation
   return detail::CgalFunctions<Tag>::computeConvexHull(input_vertices, hull_vertices, hull_faces);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Builds the CGAL AABB tree from the current triangle mesh.
 *
 * This method constructs the AABB tree from the current world-frame vertices of the triangle
 * mesh. The tree is built only when CGAL is available, otherwise this is a no-op.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::buildAABBTree() const
{
#ifdef PE_USE_CGAL
   // Ensure the vertex cache is up to date
   const_cast<TriangleMeshTrait<C>*>(this)->updateCache();
   
   // Get current world-frame vertices and face indices
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   
   // Clear and rebuild triangle list
   triangles_.clear();
   triangles_.reserve(faces.size());
   
   // Convert PE triangle data to CGAL triangles
   typedef detail::CgalFunctions<detail::HasCgalTag>::Point_3 Point_3;
   typedef detail::CgalFunctions<detail::HasCgalTag>::Triangle_3 Triangle_3;
   
   for (const auto& face : faces) {
      if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
         const Vec3& v0 = vertices[face[0]];
         const Vec3& v1 = vertices[face[1]];
         const Vec3& v2 = vertices[face[2]];
         
         Point_3 p0(v0[0], v0[1], v0[2]);
         Point_3 p1(v1[0], v1[1], v1[2]);
         Point_3 p2(v2[0], v2[1], v2[2]);
         
         triangles_.emplace_back(p0, p1, p2);
      }
   }
   
   // Create and build the AABB tree
   aabbTree_ = std::make_unique<detail::CgalFunctions<detail::HasCgalTag>::AABBTree>(
      triangles_.begin(), triangles_.end());
   
   // Enable distance acceleration if requested
   if (distanceAccelerationEnabled_) {
      aabbTree_->accelerate_distance_queries();
   }
   
   aabbTreeValid_ = true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Ensures the AABB tree is valid, rebuilding it if necessary.
 *
 * This method checks if the AABB tree is valid and rebuilds it if it has been invalidated
 * due to mesh movement or other changes.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::ensureAABBTreeValid() const
{
#ifdef PE_USE_CGAL
   if (!aabbTreeValid_) {
      buildAABBTree();
   }
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Invalidates the AABB tree, forcing it to be rebuilt on next access.
 *
 * This method should be called whenever the triangle mesh has moved or been modified
 * in a way that would affect the spatial structure.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::invalidateAABBTree()
{
#ifdef PE_USE_CGAL
   aabbTreeValid_ = false;
   aabbTree_.reset();  // Free memory immediately
   triangles_.clear();
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the squared distance from a point to the triangle mesh surface.
 *
 * \param point The query point in world coordinates.
 * \return The squared distance to the closest point on the mesh surface.
 *
 * This method uses CGAL's AABB tree for efficient distance computation. When CGAL is not
 * available, it returns 0.0.
 */
template< typename C >  // Type of the configuration
real TriangleMeshTrait<C>::distanceToPoint(const Vec3& point) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // For performance, use cached AABB tree when CGAL is available
#ifdef PE_USE_CGAL
   ensureAABBTreeValid();
   
   if (!aabbTree_) {
      // Fallback to tag dispatch if no cached tree
      const Vertices& vertices = this->getBFVertices();
      const IndicesLists& faces = this->getFaceIndices();
      return detail::CgalFunctions<Tag>::distanceToPoint(vertices, faces, point);
   }
   
   typedef detail::CgalFunctions<detail::HasCgalTag>::Point_3 Point_3;
   Point_3 query(point[0], point[1], point[2]);
   
   return static_cast<real>(aabbTree_->squared_distance(query));
#else
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   return detail::CgalFunctions<Tag>::distanceToPoint(vertices, faces, point);
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds the closest point on the triangle mesh surface to a query point.
 *
 * \param point The query point in world coordinates.
 * \return The closest point on the mesh surface in world coordinates.
 *
 * This method uses CGAL's AABB tree for efficient closest point computation. When CGAL is not
 * available, it returns the input point.
 */
template< typename C >  // Type of the configuration
Vec3 TriangleMeshTrait<C>::closestPoint(const Vec3& point) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // For performance, use cached AABB tree when CGAL is available
#ifdef PE_USE_CGAL
   ensureAABBTreeValid();
   
   if (!aabbTree_) {
      // Fallback to tag dispatch if no cached tree
      const Vertices& vertices = this->getBFVertices();
      const IndicesLists& faces = this->getFaceIndices();
      return detail::CgalFunctions<Tag>::closestPoint(vertices, faces, point);
   }
   
   typedef detail::CgalFunctions<detail::HasCgalTag>::Point_3 Point_3;
   Point_3 query(point[0], point[1], point[2]);
   
   Point_3 closest = aabbTree_->closest_point(query);
   return Vec3(CGAL::to_double(closest.x()), 
               CGAL::to_double(closest.y()), 
               CGAL::to_double(closest.z()));
#else
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   return detail::CgalFunctions<Tag>::closestPoint(vertices, faces, point);
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds the closest point and primitive ID on the triangle mesh surface.
 *
 * \param point The query point in world coordinates.
 * \return A pair containing the closest point and the face index.
 *
 * This method uses CGAL's AABB tree for efficient closest point and primitive computation.
 * When CGAL is not available, it returns the input point and face index 0.
 */
template< typename C >  // Type of the configuration
std::pair<Vec3, size_t> TriangleMeshTrait<C>::closestPointAndPrimitive(const Vec3& point) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // For performance, use cached AABB tree when CGAL is available
#ifdef PE_USE_CGAL
   ensureAABBTreeValid();
   
   if (!aabbTree_) {
      // Fallback to tag dispatch if no cached tree
      const Vertices& vertices = this->getBFVertices();
      const IndicesLists& faces = this->getFaceIndices();
      return detail::CgalFunctions<Tag>::closestPointAndPrimitive(vertices, faces, point);
   }
   
   typedef detail::CgalFunctions<detail::HasCgalTag>::Point_3 Point_3;
   Point_3 query(point[0], point[1], point[2]);
   
   auto result = aabbTree_->closest_point_and_primitive(query);
   Point_3 closest = result.first;
   
   // The primitive iterator points to our triangle in the triangles_ vector
   size_t primitive_id = std::distance(triangles_.cbegin(), result.second);
   
   return std::make_pair(
      Vec3(CGAL::to_double(closest.x()), 
           CGAL::to_double(closest.y()), 
           CGAL::to_double(closest.z())),
      primitive_id
   );
#else
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   return detail::CgalFunctions<Tag>::closestPointAndPrimitive(vertices, faces, point);
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Enables distance query acceleration for large triangle meshes.
 *
 * \param maxReferencePoints Maximum number of reference points for distance acceleration.
 *
 * This method enables CGAL's distance query acceleration, which can significantly improve
 * performance for large meshes at the cost of increased memory usage.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::enableDistanceAcceleration(size_t maxReferencePoints)
{
#ifdef PE_USE_CGAL
   distanceAccelerationEnabled_ = true;
   maxReferencePoints_ = maxReferencePoints;
   
   // If tree already exists, apply acceleration immediately
   if (aabbTree_) {
      aabbTree_->accelerate_distance_queries();
   }
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Determines if a point is inside the triangle mesh using ray shooting.
 *
 * \param point The query point in world coordinates.
 * \return True if the point is inside the mesh, false otherwise.
 *
 * This method uses the Jordan Curve theorem with ray shooting to determine point containment.
 * Multiple random rays are cast from the query point, and the number of intersections with
 * the mesh surface is counted. An odd number of intersections indicates the point is inside.
 * When CGAL is not available, this method returns false (conservative approach).
 */
template< typename C >  // Type of the configuration
bool TriangleMeshTrait<C>::containsPoint(const Vec3& point) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // Get the vertices and faces from the base class
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   
   // Use tag dispatch to call the appropriate implementation
   return detail::CgalFunctions<Tag>::containsPoint(vertices, faces, point);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the signed distance from a point to the triangle mesh surface.
 *
 * \param point The query point in world coordinates.
 * \return The signed distance: negative if inside, positive if outside.
 *
 * This method combines distance computation with point containment to provide signed distance.
 * Points inside the mesh have negative distance, points outside have positive distance.
 * When CGAL is not available, this method returns 0.0.
 */
template< typename C >  // Type of the configuration
real TriangleMeshTrait<C>::signedDistance(const Vec3& point) const
{
#ifdef PE_USE_CGAL
   using Tag = detail::HasCgalTag;
#else
   using Tag = detail::NoCgalTag;
#endif
   
   // Get the vertices and faces from the base class
   const Vertices& vertices = this->getBFVertices();
   const IndicesLists& faces = this->getFaceIndices();
   
   // Use tag dispatch to call the appropriate implementation
   return detail::CgalFunctions<Tag>::signedDistance(vertices, faces, point);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Enables DistanceMap acceleration for fast collision detection.
 *
 * \param spacing Grid spacing (uniform in all directions).
 * \param resolution Number of grid cells along the largest dimension.
 * \param tolerance Number of empty boundary cells around the mesh.
 *
 * This method creates and stores a DistanceMap for this triangle mesh, which can significantly
 * improve collision detection performance for mesh-mesh interactions at the cost of memory usage.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::enableDistanceMapAcceleration(pe::real spacing, int resolution, int tolerance)
{
#ifdef PE_USE_CGAL
   // Store the parameters for later use by derived class
   // The actual DistanceMap creation will happen in TriangleMesh::enableDistanceMapAcceleration()
   // which has access to both headers
   distanceMapEnabled_ = false;
   
   // Note: This is a base implementation. TriangleMesh will override this method
   // to provide the actual DistanceMap creation functionality.
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Disables DistanceMap acceleration.
 *
 * This method removes the stored DistanceMap to free memory, falling back to
 * standard collision detection methods.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::disableDistanceMapAcceleration()
{
#ifdef PE_USE_CGAL
   distanceMap_.reset();
   distanceMapEnabled_ = false;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if DistanceMap acceleration is available.
 *
 * \return True if a DistanceMap is available for this mesh, false otherwise.
 */
template< typename C >  // Type of the configuration
bool TriangleMeshTrait<C>::hasDistanceMap() const
{
#ifdef PE_USE_CGAL
   return distanceMapEnabled_ && distanceMap_;
#else
   return false;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Gets the DistanceMap instance.
 *
 * \return Pointer to the DistanceMap, or nullptr if not available.
 */
template< typename C >  // Type of the configuration
const DistanceMap* TriangleMeshTrait<C>::getDistanceMap() const
{
#ifdef PE_USE_CGAL
   return distanceMap_.get();
#else
   return nullptr;
#endif
}
//*************************************************************************************************

} // namespace pe

#endif
