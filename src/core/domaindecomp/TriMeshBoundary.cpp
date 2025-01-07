#include <pe/core/domaindecomp/TriMeshBoundary.h>
#include <pe/core/OBJMeshLoader.h>
#include <pe/core/Thresholds.h>
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

   //reading the actual geometry file
   if((file.find(".obj") != std::string::npos) || (file.find(".OBJ") != std::string::npos)) {
      //Reading triangle mesh form OBJ-File
      TriMeshBoundary::initOBJ(file,
               vertices_, 
               faceIndices_,
               faceNormals_,
               vertexNormals_, 
               normalIndices_,
               textureCoordinates_, 
               textureIndices_,
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

   for (size_t i = 0; i < faceIndices_.size(); ++i) {
      const Vec3& a = vertices_[faceIndices_[i][0]];
      const Vec3& b = vertices_[faceIndices_[i][1]];
      const Vec3& c = vertices_[faceIndices_[i][2]];

      //http://mathworld.wolfram.com/Tetrahedron.html
      currentVolume = (trans(a) * ( b % c ));
      totalVolume += currentVolume;
      center += (a + b +c) * currentVolume; //* 0.25
   }

   center /= totalVolume*4.0; //anstelle von *0.25
   gpos_ = center;
   static const real sixth = 1.0 / 6.0;
   totalVolume *= sixth;

   calcBoundingBox();

   std::cout << file << std::endl;
   std::cout << "Total Volume = " << totalVolume << std::endl;
   std::cout << "X center = " << center[0] << std::endl; //xCenter/totalVolume
   std::cout << "Y center = " << center[1] << std::endl; //yCenter/totalVolume
   std::cout << "Z center = " << center[2] << std::endl; //zCenter/totalVolume
   std::cout << "#Verts   = " << vertices_.size() << std::endl; //zCenter/totalVolume
   std::cout << "#Faces   = " << faceIndices_.size() << std::endl; //zCenter/totalVolume

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created TriMeshBoundary " << "\n"
          << "   Number of triangles = " << faceIndices_.size() << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const {

} 
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::intersectRayTriangle(const Vec3 &rayOrigin,
                             const Vec3 &rayDir,
                             const Vec3 &V0,     
                             const Vec3 &V1,     
                             const Vec3 &V2) const {

    // Step 1: Build edges E1 and E2
    Vec3 E1 = V1 - V0;
    Vec3 E2 = V2 - V0;

    // Step 2: Construct matrix M = [ -rayDir, E1, E2 ]
    // (Assuming we have a Mat3 constructor taking columns or similar)
    Mat3 M(-rayDir, E1, E2);

    // Step 3: Compute determinant of M
    real det = M.getDeterminant();
    if (std::abs(det) < parallelThreshold) {
        // Ray is parallel or degenerate => no intersection
        return false; // (false, 0, 0, 0);
    }

    // Step 4: Right-hand side vector C = (V0 - rayOrigin)
    Vec3 C = V0 - rayOrigin;

    // Step 5: Solve M * [t u v]^T = C
    // Some approaches:
    //   1) Use the inverse of M (if your Mat3 supports inverse).
    //   2) Use Cramer's rule (determinants).
    //   3) Use an adjoint-based method, etc.
    // For simplicity, let's assume we have M.inverse() or a function solve(M, C):
    //    Vec3 tuv = solveLinearSystem(M, C); // returns [ t, u, v ]
    Vec3 tuv = M.invert() * C;

    real t = tuv[0];
    real u = tuv[1];
    real v = tuv[2];

    // Step 6: Check if intersection is valid
    if (t >= 0 && u >= 0 && v >= 0 && (u + v) <= 1) {
        // We have a valid intersection
        return true; // (true, t, u, v);
    } else {
        return false; // (false, 0, 0, 0);
    }    
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
    // 0) Choose a ray direction. 
    //    Check  if the AABB contains the point
    if(!aabb_.contains(gpos))
      return false;

    // 1) Choose a ray direction. 
    //    Common to use e.g., +X axis: Vec3(1,0,0).
    //    In some robust code, you might randomize the direction
    //    to reduce the chance of special edge/vertex hits.
    Vec3 rayDir(1.0, 0.0, 0.0);

    // 2) Cast a ray from 'point' along 'rayDir' 
    //    and count the intersections with each triangle.
    int intersectionCount = 0;

    // Loop through all faces
    for(size_t f = 0; f < faceIndices_.size(); ++f) 
    {
        const Vec3 &A = vertices_[faceIndices_[f][0]];
        const Vec3 &B = vertices_[faceIndices_[f][1]];
        const Vec3 &C = vertices_[faceIndices_[f][2]];

        // Use your existing function to check intersection:
        bool hit = intersectRayTriangle(gpos, rayDir, A, B, C);

        // If there is an intersection, increment the counter.
        if(hit)
        {
            ++intersectionCount;
        }
    }

    // 3) Decide inside/outside by counting intersections:
    //    - If intersectionCount is odd => inside
    //    - If intersectionCount is even => outside
    bool isInside = (intersectionCount % 2 == 1);
    return isInside;

}
//*************************************************************************************************


//*************************************************************************************************
bool TriMeshBoundary::containsPointStrictly(const Vec3& gpos) const {
    // Check if the point is strictly inside the surface triangulation, meaning it's not on the boundary
    return containsPoint(gpos);  // Return a dummy value for now
}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::print( std::ostream& os                  ) const {

}
//*************************************************************************************************


//*************************************************************************************************
void TriMeshBoundary::calcBoundingBox() {
      aabb_[0] = gpos_[0];
      aabb_[1] = gpos_[1];
      aabb_[2] = gpos_[2]; //std::numeric_limits<real>::max();
      aabb_[3] = gpos_[0];
      aabb_[4] = gpos_[1];
      aabb_[5] = gpos_[2]; //std::numeric_limits<real>::min(); //is actually a positive value close to 0

      for(Vertices::const_iterator v=vertices_.begin(); v != vertices_.end(); ++v) {
         aabb_[0] = (*v)[0] < aabb_[0] ? (*v)[0] : aabb_[0];
         aabb_[1] = (*v)[1] < aabb_[1] ? (*v)[1] : aabb_[1];
         aabb_[2] = (*v)[2] < aabb_[2] ? (*v)[2] : aabb_[2];
         aabb_[3] = (*v)[0] > aabb_[3] ? (*v)[0] : aabb_[3];
         aabb_[4] = (*v)[1] > aabb_[4] ? (*v)[1] : aabb_[4];
         aabb_[5] = (*v)[2] > aabb_[5] ? (*v)[2] : aabb_[5];
      }

      //The bounding box is increased by pe::contactThreshold in all dimensions
      aabb_[0] -= contactThreshold;
      aabb_[1] -= contactThreshold;
      aabb_[2] -= contactThreshold;
      aabb_[3] += contactThreshold;
      aabb_[4] += contactThreshold;
      aabb_[5] += contactThreshold;

}
//*************************************************************************************************
void TriMeshBoundary::print( std::ostream& os, const char* tab ) const {
}

} // namespace pe
