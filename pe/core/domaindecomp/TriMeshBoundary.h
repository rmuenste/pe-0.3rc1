//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/HalfSpace.h
 *  \brief Header file for the HalfSpace class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
 *                2025 Raphael Münster
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

#ifndef _PE_CORE_DOMAINDECOMP_TRIMESHBOUNDARY_H_
#define _PE_CORE_DOMAINDECOMP_TRIMESHBOUNDARY_H_

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
 * \brief Implementation of a closed triangle mesh for specifying subdomains.
 * 
 * The ClosedSurfaceTriangulation class represents a closed surface triangulation that can perform various intersection tests with spheres, boxes, capsules, and cylinders. It also provides functionality to 
determine the distance from a point to the center of these objects and check if a given point is inside the surface triangulation.
 * 
 * \ingroup domaindecomp
 */

class TriMeshBoundary : public ProcessGeometry
{
public:
   //**Type definitions****************************************************************************
   typedef detection::coarse::BoundingBox<real>  AABB;  //!< Type of the axis-aligned bounding box.
   //**********************************************************************************************
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   TriMeshBoundary(TriangleMeshID triangleMesh);
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************
   TriMeshBoundary( id_t uid, const Vec3& gpos, const std::string file,
                    MaterialID material, bool convex, bool visible,
                    const Vec3& scale,  bool clockwise, bool lefthanded );

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

    // Implement the pure virtual functions from ProcessGeometry
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
    void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const override;
    //@}
    //**********************************************************************************************

    //**Output functions****************************************************************************
    /*!\name Output functions */
    //@{
    void print( std::ostream& os                  ) const;
    void print( std::ostream& os, const char* tab ) const;
    //@}
    //**********************************************************************************************

   //**Initialization functions********************************************************************
   /*!\name Initialization functions */
   //@{
   /*static void initSTL( const char* const file,
                        Vertices& vertices, IndicesLists& faceIndices,
                        Normals& faceNormals );*/
   static void initOBJ( const std::string file,
                        Vertices& vertices, IndicesLists& faceIndices,
                        Normals& faceNormals,
                        Normals& vertexNormals, IndicesLists normalIndices,
                        TextureCoordinates& texturCoordinates, IndicesLists& texturIndices,
                        bool clockwise, bool lefthanded );
private:
   void calcBoundingBox();  // Calculation of the axis-aligned bounding box
   bool intersectRayTriangle(const Vec3 &rayOrigin,
                             const Vec3 &rayDir,
                             const Vec3 &V0,     
                             const Vec3 &V1,     
                             const Vec3 &V2    
   ) const;                        

private:
    Vec3               gpos_;
    TriangleMeshID     triangleMesh_;
    Vertices           vertices_;
    IndicesLists       faceIndices_;
    Normals            faceNormals_;        //!< Holds the normal of each face/triangle

    Normals            vertexNormals_;      //!< Holds the normal at the edge positions, only used for visualisation purposes
    IndicesLists       normalIndices_;      //!< List of indices which assign three elements of vertexNormals_ to one triangle

    TextureCoordinates textureCoordinates_; //!< Holds the texture coordinates at the edge positions, only used for visualisation purposes
    IndicesLists       textureIndices_;     //!< List of indices which assign three elements of textureCoordinates_ to one triangle

    IndexList          vertexEdge_;         //!< Mapping vertexIndex->edgeIndex.
                                            //!< Each edge the vertex belongs to is feasible.
                                            //!< The edge indices are taken implicitly taken form the face definition.
    IndexList          edgeEdge_;           //!< Maps each edge to its pair edge in the opposite direction.
                                            //!< The edge indices are taken implicitly taken form the face definition.
    IndexList          vertexVNeighbor_;    //!< Maps a virtual neighbor vertex to each vertex.
    AABB aabb_;                             //!< Axis-aligned bounding box for the rigid body.

   //**Triangle mesh setup functions***************************************************************
   /*! \cond PE_INTERNAL */
//   friend TriangleMeshID createTriMeshBoundary( id_t uid, const Vec3& gpos, const std::string file,
//                                             MaterialID material, bool convex, bool visible,
//                                             const Vec3& scale,  bool clockwise, bool lefthanded );
};
//*************************************************************************************************

} // namespace pe

#endif
