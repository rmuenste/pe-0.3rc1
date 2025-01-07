#include <pe/core/domaindecomp/TriMeshBoundary.h>
#include <pe/core/OBJMeshLoader.h>
#include <pe/util/Logging.h>
#include <iostream>

namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Loads all triangle mesh data from a given OBJ-file and stores all date in the provided containers
 *
 * \param file Location of the OBJ-file that should be parsed.
 * \param vertices Container for the vertex data.
 * \param faceIndices Container for the list of indices which assign three elements of \a vertices to one triangle.
 * \param faceNormals Container for the face normal data.
 * \param vertexNormals Container for the vertex normal data.
 * \param normalIndices Container for the list of indices which assign three elements of \a vertexNormals to one triangle.
 * \param textureCoordinates Container for the texture coordinate data.
 * \param textureIndices Container for the list of indices which assign three elements of \a textureCoordinates to one triangle.
 * \param clockwise \a true if the triangles described in \a file are ordered clockwise, otherwise \a false, which is more common.
 * \param lefthanded \a true if the coordinates used in the OBJ-file are from a left-handed coordinate system, \a false if the coordinate system is right-handed (pe default)
 * \return void
 *
 * \todo Review documentation.
 */
void TriMeshBoundary::initOBJ(const std::string file,
                  Vertices& vertices, IndicesLists& faceIndices,
                  Normals& faceNormals,
                  Normals& vertexNormals, IndicesLists normalIndices,
                  TextureCoordinates& textureCoordinates, IndicesLists& textureIndices,
                  bool clockwise, bool lefthanded)
{
   OBJMeshLoader parser;

   parser.load(file, vertices, faceIndices, vertexNormals, normalIndices, textureCoordinates, textureIndices, clockwise, lefthanded);

   if(faceIndices.size()<4){
      std::stringstream error;
      error << "TriangleMesh::initOBJ: TriangleMashes with less than 4 (" << faceIndices.size() << ") faces not allowed.\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::invalid_argument(error.str());
   }

   //triangles.reserve(faceIndices_.size());
   faceNormals.reserve(faceIndices.size());

   for(size_t f = 0; f < faceIndices.size(); ++f) {
      Vec3& a = vertices[faceIndices[f][0]];
      Vec3& b = vertices[faceIndices[f][1]];
      Vec3& c = vertices[faceIndices[f][2]];

      //calculate face normal
      Vec3 ab = b-a;
      Vec3 ac = c-a;
      Vec3 n  = ab % ac;
      n.normalize();
      faceNormals.push_back(n);

      //check if face normal points in the same direction as the vertices normals
      bool inverted = false;
      if(normalIndices.size() > 0) {
         real cosVec1 = trans(n) * vertexNormals[normalIndices[f][0]];
         real cosVec2 = trans(n) * vertexNormals[normalIndices[f][1]];
         real cosVec3 = trans(n) * vertexNormals[normalIndices[f][2]];

         if(cosVec1 < 0 && cosVec2 < 0 && cosVec3 < 0 && !inverted) {
            std::stringstream error;
            error << "TriangleMesh::initOBJ: Vertex order seams to be inverted.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
            inverted = true;
         }
         else if (cosVec1 < 0 || cosVec2 < 0 || cosVec3 < 0) {
            std::stringstream error;
            error << "TriangleMesh::initOBJ: At least one vertex normal does not match face normal direction.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
         }
      }
   }
}

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================


TriMeshBoundary::TriMeshBoundary(TriangleMeshID triangleMesh) 
                                 : triangleMesh_(triangleMesh)  {}

//*************************************************************************************************


//*************************************************************************************************
 TriMeshBoundary::TriMeshBoundary( id_t uid, const Vec3& gpos, const std::string file,
                    MaterialID material, bool convex, bool visible,
                    const Vec3& scale,  bool clockwise, bool lefthanded )
{
   pe_INTERNAL_ASSERT( convex, "Only convex triangle meshes are allowed right now" );

   if(scale[0] <= real(0) || scale[1] <= real(0) || scale[2] <= real(0)){
      throw std::invalid_argument("Invalid scaling factor, only positive scaling allowed.");
   }

   // Checking for the input file formte
   if(file.find(".obj") == std::string::npos && file.find(".OBJ") == std::string::npos
            /*&& file.find(".stl") == std::string::npos && file.find(".STL") == std::string::npos*/) {
      throw std::invalid_argument("Unsupported triangle mesh format.");
   }

   //Variables to hold mesh information during inizialisierung
   Vertices         vertices;
   IndicesLists     faceIndices;
   Normals          faceNormals;
   Normals          vertexNormals;
   IndicesLists     normalIndices;
   TextureCoordinates texturCoordinates;
   IndicesLists     texturIndices;

   //reading the actual geometry file
   if((file.find(".obj") != std::string::npos) || (file.find(".OBJ") != std::string::npos)) {
      //Reading triangle mesh form OBJ-File
      TriMeshBoundary::initOBJ(file,
               vertices, faceIndices,
               faceNormals,
               vertexNormals, normalIndices,
               texturCoordinates, texturIndices,
               clockwise, lefthanded);
   }
   else {
      throw std::invalid_argument( "Invalid input file type for triangle mesh" );
   }

   //Calculate centre of mass and volume
   //http://stackoverflow.com/questions/2083771/a-method-to-calculate-the-centre-of-mass-from-a-stl-stereo-lithography-file
   real totalVolume ( 0.0 );
   real currentVolume ( 0.0 );
   Vec3 center (0.0, 0.0, 0.0);

   for (size_t i = 0; i < faceIndices.size(); ++i) {
      const Vec3& a = vertices[faceIndices[i][0]];
      const Vec3& b = vertices[faceIndices[i][1]];
      const Vec3& c = vertices[faceIndices[i][2]];

      //http://mathworld.wolfram.com/Tetrahedron.html
      currentVolume = (trans(a) * ( b % c ));
      totalVolume += currentVolume;
      center += (a + b +c) * currentVolume; //* 0.25
   }

   center /= totalVolume*4.0; //anstelle von *0.25
   static const real sixth = 1.0 / 6.0;
   totalVolume *= sixth;

   std::cout << file << std::endl;
   std::cout << "Total Volume = " << totalVolume << std::endl;
   std::cout << "X center = " << center[0] << std::endl; //xCenter/totalVolume
   std::cout << "Y center = " << center[1] << std::endl; //yCenter/totalVolume
   std::cout << "Z center = " << center[2] << std::endl; //zCenter/totalVolume
   std::cout << "#Verts   = " << vertices.size() << std::endl; //zCenter/totalVolume
   std::cout << "#Faces   = " << faceIndices.size() << std::endl; //zCenter/totalVolume

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created TriMeshBoundary " << "\n"
          << "   Number of triangles = " << faceIndices.size() << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const {

} 
//*************************************************************************************************

bool TriMeshBoundary::intersectRayTriangle(const Vec3 &rayOrigin,
                             const Vec3 &rayDir,
                             const Vec3 &V0,     
                             const Vec3 &V1,     
                             const Vec3 &V2) 
{
    // Step 1: Build edges E1 and E2
    Vec3 E1 = V1 - V0;
    Vec3 E2 = V2 - V0;

    // Step 2: Construct matrix M = [ -rayDir, E1, E2 ]
    // (Assuming we have a Mat3 constructor taking columns or similar)
//    Mat3 M;
//    M.setColumn(0, -rayDir);  // column 0 is -D
//    M.setColumn(1, E1);       // column 1 is E1
//    M.setColumn(2, E2);       // column 2 is E2
//
//    // Step 3: Compute determinant of M
//    float det = M.getDeterminant();
//    if (abs(det) < EPSILON) {
//        // Ray is parallel or degenerate => no intersection
//        return (false, 0, 0, 0);
//    }
//
//    // Step 4: Right-hand side vector C = (V0 - rayOrigin)
//    Vec3 C = V0 - rayOrigin;
//
//    // Step 5: Solve M * [t u v]^T = C
//    // Some approaches:
//    //   1) Use the inverse of M (if your Mat3 supports inverse).
//    //   2) Use Cramer's rule (determinants).
//    //   3) Use an adjoint-based method, etc.
//    // For simplicity, let's assume we have M.inverse() or a function solve(M, C):
//    Vec3 tuv = solveLinearSystem(M, C); // returns [ t, u, v ]
//
//    float t = tuv.x;
//    float u = tuv.y;
//    float v = tuv.z;
//
//    // Step 6: Check if intersection is valid
//    if (t >= 0 && u >= 0 && v >= 0 && (u + v) <= 1) {
//        // We have a valid intersection
//        return (true, t, u, v);
//    } else {
//        return (false, 0, 0, 0);
//    }    
return true;
}                       

//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstBodyID b) const {
    // Implement intersection test with another body
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstSphereID s) const {
    // Implement intersection test with a sphere
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstBoxID b) const {
    // Implement intersection test with a box
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstCapsuleID c) const {
    // Implement intersection test with a capsule
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstCylinderID c) const {
    // Implement intersection test with a cylinder
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectsWith(ConstUnionID u) const {
    // Implement intersection test with a union of shapes
    return true;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::containsPoint(const Vec3& gpos) const {
    // Check if the point is inside the surface triangulation
    std::cout << "Called routine containsPoint with input " << gpos << std::endl;
    return false;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::containsPointStrictly(const Vec3& gpos) const {
    // Check if the point is strictly inside the surface triangulation, meaning it's not on the boundary
    std::cout << "Called routine containsPointStrictly with input " << gpos << std::endl;
    return false;  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::print( std::ostream& os                  ) const {

}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::print( std::ostream& os, const char* tab ) const {

}

} // namespace pe
