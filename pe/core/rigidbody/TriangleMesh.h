#ifndef USE_CGAL
#define USE_CGAL
#endif

//=================================================================================================
/*!
 *  \file pe/core/rigidbody/TriangleMesh.h
 *  \brief Header file for the TriangleMesh class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_RIGIDBODY_TRIANGLEMESH_H_
#define _PE_CORE_RIGIDBODY_TRIANGLEMESH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <vector>

#include <pe/util/Types.h>
#include <pe/core/rigidbody/TriangleMeshTrait.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>

#include <pe/povray/WriterID.h>
#include <pe/povray/Texture.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/math/Quaternion.h>

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
#include <irrlicht/irrlicht.h>
#endif
#ifdef USE_CGAL
   #include <CGAL/Simple_cartesian.h>
   #include <CGAL/AABB_tree.h>
   #include <CGAL/AABB_traits.h>
   #include <CGAL/config.h>
   #include <CGAL/Polyhedron_3.h>
   #include <CGAL/IO/OBJ_reader.h>
   #include <CGAL/Side_of_triangle_mesh.h>

   #include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
   #include <CGAL/AABB_halfedge_graph_segment_primitive.h>
   #include <CGAL/AABB_face_graph_triangle_primitive.h>
   #include <CGAL/Polyhedron_incremental_builder_3.h>
   #include <CGAL/Polygon_mesh_processing/intersection.h>

   #include <CGAL/Surface_mesh.h>
   #include <CGAL/boost/graph/copy_face_graph.h>

#endif

namespace pe {

   #ifdef USE_CGAL

      // Choose a geometry kernel
      typedef CGAL::Simple_cartesian<double> Kernel;

      // Make a short-hand for the geometry Kernel type
      typedef Kernel::FT FT;

      // Define short-hands for the other CGAL types that we
      // are going to use in the application
      typedef Kernel::Point_2 Point_2;
      typedef Kernel::Segment_2 Segment_2;

      typedef Kernel::Point_3 cgalPoint;
      typedef Kernel::Triangle_3 Triangle;
      typedef Kernel::Vector_3 Vec;

      typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

      typedef Kernel::Ray_3 cgalRay;

      typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>        Facet_Primitive;
      typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
      typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;

      typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
      typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
      typedef CGAL::AABB_tree<Traits> Tree;
      typedef Tree::Point_and_primitive_id Point_and_primitive_id;

      typedef Polyhedron::Vertex_iterator Vertex_iterator;
      typedef CGAL::Surface_mesh<Kernel::Point_3> Surface_mesh;

      typedef Polyhedron::Halfedge_handle Halfedge_handle;
      typedef Polyhedron::Vertex_handle Vertex_handle;
      typedef Polyhedron::Halfedge_around_vertex_circulator Halfedge_circulator;
      
      // std::vector<Tree*> trees;
      // std::vector<Polyhedron*> polyhedra;
   #endif

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup triangleMesh Triangle Mesh
 * \ingroup geometric_primitive
 *
 * The triangle mesh module combines all necessary functionality for the geometric primitive triangle
 * mesh. A detailed description of the triangle mesh primitive can be found with the class TriangleMesh.
 * This description also contains examples for the setup and destruction of a triangle mesh.
 * When a triangle mesh is created the center of mass is calculated. By default pe then aligns the
 * the global position with the center of mass and recalculates the vertices relative to the com. This
 * way a local coordinate system is created. After that user-defined transformations are applied.
 * 
 * VTK output:
 * In the vtk output the vertex indices match the vtk vertex indices. The face indices are the
 * vtk indices - 1. So vtk_cell_index = TriangMesh_index + 1
 */
/*!\brief Triangle mesh geometry.
 * \ingroup triangleMesh
 *
 * \section tringleMesh_general General
 * TODO
 *
 * \todo Review documentation.
 */
class TriangleMesh : public TriangleMeshTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef TriangleMeshTrait<Config>  Parent;  //!< The type of the parent class.
   struct LowToHighComp;
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class detection::fine::MaxContacts;
   /*! \endcond */
   //**********************************************************************************************


   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit TriangleMesh( id_t sid, id_t uid, const Vec3& gpos,
                          const Vertices& vertices, const IndicesLists& faceIndices,
                          const Normals& faceNormals,
                          const Normals& vertexNormals, const IndicesLists normalIndices,
                          const TextureCoordinates& texturCoordinates, const IndicesLists& texturIndices,
                          MaterialID material, bool visible, bool convex );
   explicit TriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                          const Vertices& vertices, const IndicesLists& faceIndices,
                          const Normals& faceNormals,
                          const Normals& vertexNormals, const IndicesLists normalIndices,
                          const TextureCoordinates& texturCoordinates, const IndicesLists& texturIndices,
                          MaterialID material, bool visible, bool fixed, bool convex );
   // explicit TriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
   //                        const Vertices& vertices, const IndicesLists& faceIndices,
   //                        const Normals& faceNormals,
   //                        const Normals& vertexNormals, const IndicesLists normalIndices,
   //                        const TextureCoordinates& texturCoordinates, const IndicesLists& texturIndices,
   //                        MaterialID material, bool visible, bool fixed, bool convex, Polyhedron, Tree tree);
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~TriangleMesh();
   //@}
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   struct Parameters : public GeomPrimitive::Parameters {
      Vertices     vertices_;
      IndicesLists faceIndices_;
      bool isConvex_;
   };
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t size()  const;
   inline size_t getSmallMeshLimit() const;
   bool convex() const;
   #ifdef USE_CGAL
      inline Polyhedron getPolyhedron() const;
      inline void setMeshwidth();
      inline double getMeshwidth() const;
   #endif
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible       ( bool visible );
   virtual void setPosition      ( real px, real py, real pz );
   virtual void setPosition      ( const Vec3& gpos );
   virtual void setOrientation   ( real r, real i, real j, real k );
   virtual void setOrientation   ( const Quat& q );
           void setRenderSmooth  ( bool smooth );
   inline void setSmallMeshLimit( size_t newLimit );
   //@}
   //**********************************************************************************************

   //**Translation functions***********************************************************************
   /*!\name Translation functions */
   //@{
   virtual void translate( real dx, real dy, real dz );
   virtual void translate( const Vec3& dp );
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions */
   //@{
   virtual void rotate( real x, real y, real z, real angle );
   virtual void rotate( const Vec3& axis, real angle );
   virtual void rotate( real xangle, real yangle, real zangle );
   virtual void rotate( const Vec3& euler );
   virtual void rotate( const Quat& dq );

   virtual void rotateAroundOrigin( real x, real y, real z, real angle );
   virtual void rotateAroundOrigin( const Vec3& axis, real angle );
   virtual void rotateAroundOrigin( real xangle, real yangle, real zangle );
   virtual void rotateAroundOrigin( const Vec3& euler );

   virtual void rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle );
   virtual void rotateAroundPoint( const Vec3& point, const Vec3& euler );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool containsRelPoint ( real px, real py, real pz ) const;
   virtual bool containsRelPoint ( const Vec3& rpos )          const;
   virtual bool containsPoint    ( real px, real py, real pz ) const;
   virtual bool containsPoint    ( const Vec3& gpos )          const;
   virtual bool isSurfaceRelPoint( real px, real py, real pz ) const;
   virtual bool isSurfaceRelPoint( const Vec3& rpos )          const;
   virtual bool isSurfacePoint   ( real px, real py, real pz ) const;
   virtual bool isSurfacePoint   ( const Vec3& gpos )          const;
   #ifdef USE_CGAL
   virtual bool containsPoint    ( cgalPoint ) const;
   virtual void initGeometry(std::string);
   virtual void initGeometry(const Vertices& vertices, const IndicesLists& faces);
   virtual void buildTreeStructures();
   virtual void moveVerticesToCOM(Vec3& center, Vec3 scale);
   virtual void moveVerticesToCOM(Vec3& center, Vec3 pos, Vec3 scale);
   #endif
          real getRelDepth   ( real px, real py, real pz ) const;
          real getRelDepth   ( const Vec3& rpos )          const;
   inline real getDepth      ( real px, real py, real pz ) const;
   inline real getDepth      ( const Vec3& gpos )          const;
   inline real getRelDistance( real px, real py, real pz ) const;
   inline real getRelDistance( const Vec3& rpos )          const;
   inline real getDistance   ( real px, real py, real pz ) const;
   inline real getDistance   ( const Vec3& gpos )          const;
   inline Vec3 getFaceNormal ( std::size_t idx)            const;

   virtual void calcBoundingBox();  // Calculation of the axis-aligned bounding box

   bool hasClosedSurface() const;
   void setColoredTriangleTexture(povray::WriterID pov);

   virtual inline Vec3 support                ( const Vec3& d ) const;
   virtual inline Vec3 supportContactThreshold( const Vec3& d ) const;
                  Vec3 support                ( const Vec3& d, size_t startIndex, size_t* pointIndex ) const;
           inline Vec3 supportContactThreshold( const Vec3& d, size_t startIndex, size_t* pointIndex ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os, const char* tab ) const;

   void printPOVmesh2(std::ostream& os, const povray::Texture& texture, const Vec3 euler, const Vec3 povPos) const;
#if HAVE_IRRLICHT
   inline ::irr::scene::SMeshBuffer* getIrrlichtCacheBuffer() const;
#endif
   //@}
   //**********************************************************************************************

private:
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
   void initHalfEdge();
   void updatePOVcacheString();
#if HAVE_IRRLICHT
   void updateIrrlichtCacheBuffer();
#endif
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void update( const Vec3& dp );  // Translational update of a subordinate triangle mesh
   virtual void update( const Quat& dq );  // Rotational update of a subordinate triangle mesh
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t aabbBottomFrontLeftIndices_[3]; //!< Indices of the 3 points giving the bottom front left edge of the AABB
   size_t aabbTopBackRightIndices_[3];    //!< Indices of the 3 points giving the top back right edge of the AABB

   Normals           faceNormals_;        //!< Holds the normal of each face/triangle

   Normals           vertexNormals_;      //!< Holds the normal at the edge positions, only used for visualisation purposes
   IndicesLists      normalIndices_;      //!< List of indices which assign three elements of vertexNormals_ to one triangle

   TextureCoordinates textureCoordinates_; //!< Holds the texture coordinates at the edge positions, only used for visualisation purposes
   IndicesLists      textureIndices_;     //!< List of indices which assign three elements of textureCoordinates_ to one triangle

   IndexList         vertexEdge_;         //!< Mapping vertexIndex->edgeIndex.
                                          //!< Each edge the vertex belongs to is feasible.
                                          //!< The edge indices are taken implicitly taken form the face definition.
   IndexList         edgeEdge_;           //!< Maps each edge to its pair edge in the opposite direction.
                                          //!< The edge indices are taken implicitly taken form the face definition.
   IndexList         vertexVNeighbor_;    //!< Maps a virtual neighbor vertex to each vertex.
                                          //!< This will help with not getting stack when hill climbing.

   bool        convex_;                   //!< \a true if the user states that the mesh convex, otherwise \a false
   size_t      smallMeshLimit_;           //!< For small triangle meshes brute force approaches are often times faster than cleaver solution. If the number of triangles in the mesh is smaller or equal this value the brute force algorithms are used.
   bool        renderSmooth_;             //!< Set \a true if the mesh should be rendered with vertex normals if given, only used for visualisation purposes
   std::string cachedPOVString_;          //!< Caching most parts of the PovRay output string.
   // Vertices&        vertices_;

   #ifdef USE_CGAL
      Tree tree_;
      Polyhedron polyhedron_;
      double meshwidth_;
   #endif

#if HAVE_IRRLICHT
   ::irr::scene::SMeshBuffer* cachedIrrlichtBuffer_; //!< Caching most parts of the irrlicht data structure
#endif
   //@}
   //**********************************************************************************************


private:
   //**Triangle mesh setup functions***************************************************************
   /*! \cond PE_INTERNAL */
   friend TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const std::string file,
                                             MaterialID material, bool convex, bool visible,
                                             const Vec3& scale,  bool clockwise, bool lefthanded );
   friend TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const std::string file,
                                             MaterialID material, bool convex, bool fixed, bool visible,
                                             const Vec3& scale,  bool clockwise, bool lefthanded );
   friend TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const TriangleMeshID source,
                                             MaterialID material, bool visible );
   friend TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, Vertices vertices,
                                             const IndicesLists& faces, MaterialID material,
                                             bool convex, bool visible );
   friend TriangleMeshID instantiateTriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                                  const Quat& q, const Vertices& vertices,
                                                  const IndicesLists& faces, MaterialID material,
                                                  bool visible, bool fixed, bool reg );
   friend TriangleMeshID instantiateTriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                                   const Quat& q, const Vertices& vertices,
                                                   const IndicesLists& faces, MaterialID material,
                                                   bool visible, bool fixed, bool reg, bool convex );
   friend TriangleMeshID createRegularTetrahedron( id_t uid, const Vec3& gpos, real radius,
                                                   MaterialID material,  bool visible );
   friend TriangleMeshID createTriangulatedBox( id_t uid, const Vec3& gpos, const Vec3& lengths,
                                                MaterialID material, bool visible,
                                                const Vector3<size_t>& teselation );
   friend TriangleMeshID createRock( id_t uid, const Vec3& gpos, real radius, MaterialID material,
                                     bool visible );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the number of triangles in the triangle mesh.
 *
 * \return The number of triangles.
 */
inline size_t TriangleMesh::size() const
{
   return faceIndices_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the small triangle mesh limit.
 *
 * \return void
 *
 * For small triangle meshes brute force approaches are often times faster than cleaver solution.
 * If the number of triangles in the mesh is smaller or equal this value the brute force algorithms
 * are used.
 *
 * \todo Review documentation.
 */
inline size_t TriangleMesh::getSmallMeshLimit() const
{
   return smallMeshLimit_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns if the Triangular Mesh is convex.
 *
 * \return void
 *
 * return boolean of convexity, true if Triangular mesh is declared as convex, else false
 * for inside check with CGAL
 */
inline bool TriangleMesh::convex() const
{
   return convex_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns if the Triangular Mesh is convex.
 *
 * \return void
 *
 * return boolean of conxity, true if Triangular mesh is declared as convex, else false
 * for inside check with CGAL
 */
#ifdef USE_CGAL
Polyhedron TriangleMesh::getPolyhedron() const
{
   return polyhedron_;
}



void TriangleMesh::setMeshwidth(){
   
   double max_length = std::numeric_limits<double>::min();

   for (auto eit = polyhedron_.edges_begin(); eit != polyhedron_.edges_end(); ++eit) {
       cgalPoint p1 = eit->vertex()->point();
       cgalPoint p2 = eit->opposite()->vertex()->point();
       double length = std::sqrt(CGAL::squared_distance(p1, p2));

       if (length > max_length) {
           max_length = length;
       }
   }

   this->meshwidth_ = max_length;

}


double TriangleMesh::getMeshwidth() const
{
   return meshwidth_;
}



#endif
//*************************************************************************************************


//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the small triangle mesh limit.
 *
 * \return void
 *
 * For small triangle meshes brute force approaches are often times faster than cleaver solution.
 * If the number of triangles in the mesh is smaller or equal this value the brute force algorithms
 * are used.
 *
 * \todo Review documentation.
 */
inline void TriangleMesh::setSmallMeshLimit(size_t newLimit)
{
   smallMeshLimit_ = newLimit;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the triangle mesh and a negative value, if the point
 * lies outside the triangle mesh. The returned depth is calculated relative to the closest side of the triangle mesh.
 */
inline real TriangleMesh::getDepth( real px, real py, real pz ) const
{
   return getRelDepth( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the triangle mesh and a negative value, if the point
 * lies outside the triangle mesh. The returned depth is calculated relative to the closest side of the triangle mesh.
 */
inline real TriangleMesh::getDepth( const Vec3& gpos ) const
{
   return getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the triangle mesh's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the triangle mesh and a negative value, if the
 * point lies inside the triangle mesh. The returned distance is calculated relative to the closest
 * side of the triangle mesh.
 */
inline real TriangleMesh::getRelDistance( real px, real py, real pz ) const
{
   return -getRelDepth( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the triangle mesh's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the triangle mesh and a negative value, if the
 * point lies inside the triangle mesh. The returned distance is calculated relative to the closest
 * side of the triangle mesh.
 */
inline real TriangleMesh::getRelDistance( const Vec3& rpos ) const
{
   return -getRelDepth( rpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return Distance of the global point.
 *
 * Returns a positive value, if the point lies outside the triangle mesh and a negative value, if the
 * point lies inside the triangle mesh. The returned distance is calculated relative to the closest
 * side of the triangle mesh.
 */
inline real TriangleMesh::getDistance( real px, real py, real pz ) const
{
   return -getRelDepth( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param idx The face index 
 * \return The face normal
 *
 * Returns a positive value, if the point lies outside the triangle mesh and a negative value, if the
 * point lies inside the triangle mesh. The returned distance is calculated relative to the closest
 * side of the triangle mesh.
 */
inline Vec3 TriangleMesh::getFaceNormal( std::size_t idx) const
{
   return faceNormals_[idx]; 
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Distance of the global point.
 *
 * Returns a positive value, if the point lies outside the triangle mesh and a negative value, if the
 * point lies inside the triangle mesh. The returned distance is calculated relative to the closest
 * side of the triangle mesh.
 */
inline real TriangleMesh::getDistance( const Vec3& gpos ) const
{
   return -getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 *
 * \todo Review documentation.
 */
inline Vec3 TriangleMesh::support( const Vec3& d ) const
{
   return support(d, 0, NULL);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 *
 * \todo Review documentation.
 */
inline Vec3 TriangleMesh::supportContactThreshold( const Vec3& d ) const
{
   return support(d, 0, NULL) + d*contactThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 * \todo Review documentation.
 */
inline Vec3 TriangleMesh::supportContactThreshold( const Vec3& d, size_t startIndex, size_t* pointIndex ) const
{
   return support(d, startIndex, pointIndex) + d*contactThreshold;
}
//*************************************************************************************************



// #ifdef USE_CGAL
// inline void TriangleMesh::initGeometry(std::string objPath) {

  
//   // only one output file
//   unsigned nOut = 0;
  
//   std::cout << "Name of mesh file: " << objPath << std::endl;

//   // Load a mesh from file in the CGAL off format
//   std::string::size_type dpos = objPath.rfind(".");
//   std::string offPath = objPath; 
//   offPath.replace(dpos+1,3,"off");
//   std::ifstream in(offPath);


//   if (!in)
//   {
//     std::cerr << "unable to open file: " << offPath << std::endl;
//     std::exit(EXIT_FAILURE);
//   }

//   Polyhedron *polyhedron_ = new Polyhedron();
//   CGAL::read_off(in, *polyhedron_);

//   in.close();

//   // polyhedra.push_back(polyhedron);

//   std::cout << "OFF file loaded successfully" << std::endl;


// } 


// inline void TriangleMesh::buildTreeStructures()
// {
//   std::cout << "Construct AABB tree..."<<std::endl;

//   Polyhedron polyhedron = TriangleMesh::getPolyhedron();

//    std::cout<<"is polyhedron empty "<<polyhedron.empty()<<std::endl;
// //   Tree *tree_ = new Tree(faces(TriangleMesh::getPolyhedron()*).first, faces(TriangleMesh::getPolyhedron()*).second, TriangleMesh::getPolyhedron()*);

//    Tree *tree_ = new Tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);

//   // Use the acceleration method for distances
//   tree_->accelerate_distance_queries();

//   std::cout << "done." << std::endl;
//   std::cout<<"is Tree empty "<< tree_->empty()<<std::endl;
// }
// #endif

//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================
#if HAVE_IRRLICHT
/*!\brief returns the cached buffer for the irrlicht visualisation
 *
 * \return pointer to the cached Buffer
 */
inline ::irr::scene::SMeshBuffer* TriangleMesh::getIrrlichtCacheBuffer() const
{
   return cachedIrrlichtBuffer_;
}
#endif
//*************************************************************************************************




//=================================================================================================
//
//  TRIANGLE MESH SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name triangle mesh setup functions */
//@{
TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const std::string file,
                                   MaterialID material, bool convex,
                                   bool visible=true, const Vec3& scale=Vec3(1.0, 1.0, 1.0),  bool clockwise=false, bool lefthanded=false );


TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const std::string file,
                                   MaterialID material, bool convex, bool fixed,
                                   bool visible=true, const Vec3& scale=Vec3(1.0, 1.0, 1.0),  bool clockwise=false, bool lefthanded=false );


TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const TriangleMeshID source,
                                   MaterialID material, bool visible=true);

TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, Vertices vertices,
                                   const IndicesLists& faces, MaterialID material,
                                   bool convex, bool visible=true );

TriangleMeshID instantiateTriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                        const Quat& q, const Vertices& vertices,
                                        const IndicesLists& faces, MaterialID material,
                                        bool visible, bool fixed, bool reg=true );

TriangleMeshID instantiateTriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                       const Quat& q, const Vertices& vertices,
                                       const IndicesLists& faces, MaterialID material,
                                       bool visible, bool fixed, bool reg=true, bool convex=false );
   

TriangleMeshID createRegularTetrahedron( id_t uid, const Vec3& gpos, real radius,
                                         MaterialID material, bool visible=true );

TriangleMeshID createTriangulatedBox( id_t uid, const Vec3& gpos, const Vec3& lengths,
                                      MaterialID material, bool visible=true,
                                      const Vector3<size_t>& teselation=Vector3<size_t>(1,1,1) );
TriangleMeshID createRock( id_t uid, const Vec3& gpos, real radius, MaterialID material,
                           bool visible=true );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Triangle mesh operators */
//@{
std::ostream& operator<<( std::ostream& os, const TriangleMesh& m );
std::ostream& operator<<( std::ostream& os, ConstTriangleMeshID m );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (TriangleMesh,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to rigid
 * bodies and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<TriangleMesh>( RigidBody *const * first,
                                     RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const TriangleMesh,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<const TriangleMesh>( RigidBody *const * first,
                                           RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (TriangleMesh,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<TriangleMesh>( const RigidBody *const * first,
                                     const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const TriangleMesh,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<const TriangleMesh>( const RigidBody *const * first,
                                           const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (TriangleMesh,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<TriangleMesh>( GeomPrimitive *const * first,
                                     GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const TriangleMesh,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<const TriangleMesh>( GeomPrimitive *const * first,
                                           GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (TriangleMesh,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<TriangleMesh>( const GeomPrimitive *const * first,
                                     const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to triangle meshes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of triangle meshes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const TriangleMesh,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to triangle meshes.
 */
template<>
inline size_t polymorphicCount<const TriangleMesh>( const GeomPrimitive *const * first,
                                           const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == triangleMeshType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC FIND FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (TriangleMesh,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a triangle mesh.
 */
template<>
inline RigidBody *const * polymorphicFind<TriangleMesh>( RigidBody *const * first,
                                                RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, RigidBody );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const TriangleMesh,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a triangle mesh.
 */
template<>
inline RigidBody *const * polymorphicFind<const TriangleMesh>( RigidBody *const * first,
                                                      RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, RigidBody );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (TriangleMesh,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a triangle mesh.
 */
template<>
inline const RigidBody *const * polymorphicFind<TriangleMesh>( const RigidBody *const * first,
                                                      const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, const RigidBody );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const TriangleMesh,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a triangle mesh.
 */
template<>
inline const RigidBody *const * polymorphicFind<const TriangleMesh>( const RigidBody *const * first,
                                                            const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, const RigidBody );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (TriangleMesh,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives until it finds the next pointer to a triangle mesh.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<TriangleMesh>( GeomPrimitive *const * first,
                                                    GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, GeomPrimitive );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const TriangleMesh,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a triangle mesh.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const TriangleMesh>( GeomPrimitive *const * first,
                                                          GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, GeomPrimitive );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (TriangleMesh,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a triangle mesh.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<TriangleMesh>( const GeomPrimitive *const * first,
                                                          const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( TriangleMesh, const GeomPrimitive );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a triangle mesh in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a triangle mesh.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const TriangleMesh,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a triangle mesh.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const TriangleMesh>( const GeomPrimitive *const * first,
                                                                const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const TriangleMesh, const GeomPrimitive );

   while( first != last && (*first)->getType() != triangleMeshType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


} // namespace pe

#endif
