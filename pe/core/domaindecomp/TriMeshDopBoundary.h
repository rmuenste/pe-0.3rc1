//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/TriMeshDopBoundary.h
 *  \brief Header file for the TriMeshDopBoundary class
 *
 *  Copyright (C) 2025 Raphael Münster
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

#ifndef _PE_CORE_DOMAINDECOMP_TRIMESHDOPBOUNDARY_H_
#define _PE_CORE_DOMAINDECOMP_TRIMESHDOPBOUNDARY_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/Types.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/core/detection/coarse/BoundingBox.h>
#include <iosfwd>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <vector>
#include <array>

namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!
 * \brief Implementation of a triangle mesh with k-DOP boundary for specifying subdomains.
 * 
 * This class represents a triangle mesh with k-DOP (Discrete Oriented Polytope) boundary
 * for process geometry. It performs point containment tests using the triangle mesh directly
 * and extracts k half-spaces that form a k-DOP boundary representation.
 * 
 * By default, a 6-DOP (Object Oriented Bounding Box) is used, but the class is designed
 * to support larger values of k in the future.
 * 
 * \ingroup domaindecomp
 */

class TriMeshDopBoundary : public ProcessGeometry
{
public:
   //**Type definitions****************************************************************************
   typedef detection::coarse::BoundingBox<real>  AABB;  //!< Type of the axis-aligned bounding box.
   //**********************************************************************************************
   
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit TriMeshDopBoundary(TriangleMeshID triangleMesh);
   
   // Constructor that loads a triangle mesh from an OBJ file
   explicit TriMeshDopBoundary(const std::string& objFile, bool clockwise = false, bool lefthanded = false);
   
   // Constructor with ID, position and file - similar to TriMeshBoundary
   TriMeshDopBoundary(id_t uid, const Vec3& gpos, const std::string file,
                      MaterialID material, bool convex, bool visible,
                      const Vec3& scale, bool clockwise, bool lefthanded);
   
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   bool intersectsWith(ConstBodyID b) const override;
   bool intersectsWith(ConstSphereID s) const override;
   bool intersectsWith(ConstBoxID b) const override;
   bool intersectsWith(ConstCapsuleID c) const override;
   bool intersectsWith(ConstCylinderID c) const override;
   bool intersectsWith(ConstUnionID u) const override;
   bool containsPoint(const Vec3& gpos) const override;
   bool containsPointStrictly(const Vec3& gpos) const override;
   void extractHalfSpaces(std::list<std::pair<Vec3, real>>& halfspaces) const override;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print(std::ostream& os) const override;
   void print(std::ostream& os, const char* tab) const override;
   //@}
   //**********************************************************************************************

   //**Initialization functions********************************************************************
   /*!\name Initialization functions */
   //@{
   static void initOBJ(const std::string& file,
                      Vertices& vertices, IndicesLists& faceIndices,
                      Normals& faceNormals,
                      Normals& vertexNormals, IndicesLists normalIndices,
                      TextureCoordinates& texturCoordinates, IndicesLists& texturIndices,
                      bool clockwise, bool lefthanded);
   //@}
   //**********************************************************************************************
   
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   const AABB& getAABB() const { return aabb_; }
   const Vec3& getCentroid() const { return gpos_; }
   //@}
   //**********************************************************************************************
   
private:
   void calcBoundingBox();                    // Calculate the axis-aligned bounding box
   void calcOBB();                            // Calculate the oriented bounding box (6-DOP)
   void computeKDOP(size_t k = 6);            // Compute the k-DOP representation
   
   bool intersectRayTriangle(const Vec3& rayOrigin,
                            const Vec3& rayDir,
                            const Vec3& V0,     
                            const Vec3& V1,     
                            const Vec3& V2) const;
                            
   // Helper method for computing principal component analysis for OBB
   void computePCA();

private:
   Vec3               gpos_;                   // Centroid of the triangle mesh
   TriangleMeshID     triangleMesh_;           // Pointer to the triangle mesh if provided externally
   Vertices           vertices_;               // Vertex positions
   IndicesLists       faceIndices_;            // Face indices
   Normals            faceNormals_;            // Face normals
   
   Normals            vertexNormals_;          // Vertex normals (for visualization)
   IndicesLists       normalIndices_;          // Normal indices
   
   TextureCoordinates textureCoordinates_;     // Texture coordinates (for visualization)
   IndicesLists       textureIndices_;         // Texture indices
   
   AABB               aabb_;                   // Axis-aligned bounding box
   
   // OBB-specific (6-DOP) data
   Mat3               orientation_;            // Orientation of the OBB (principal axes)
   Vec3               halfExtents_;            // Half-extents along principal axes
   
   // k-DOP data
   std::vector<Vec3>  dopDirections_;          // Directions for k-DOP
   std::vector<real>  dopDistances_;           // Distances for k-DOP
};
//*************************************************************************************************

} // namespace pe

#endif // _PE_CORE_DOMAINDECOMP_TRIMESHDOPBOUNDARY_H_