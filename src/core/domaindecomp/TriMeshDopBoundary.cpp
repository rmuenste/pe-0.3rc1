#include <pe/core/domaindecomp/TriMeshDopBoundary.h>
#include <pe/core/OBJMeshLoader.h>
#include <pe/core/Thresholds.h>
#include <pe/util/Logging.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Ellipsoid.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#ifdef PE_USE_EIGEN
#include <Eigen/Dense>
#endif

namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor that takes an existing triangle mesh.
 *
 * \param triangleMesh The existing triangle mesh to use for the boundary.
 */
TriMeshDopBoundary::TriMeshDopBoundary(TriangleMeshID triangleMesh)
    : triangleMesh_(triangleMesh)
{
    // TODO: Extract mesh data from the provided triangleMesh
    calcBoundingBox();
    calcOBB();
    computeKDOP();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor that loads a triangle mesh from an OBJ file.
 *
 * \param objFile Path to the OBJ file to load.
 * \param clockwise \a true if the triangles described in the OBJ file are ordered clockwise, otherwise \a false.
 * \param lefthanded \a true if the coordinates used in the OBJ file are from a left-handed coordinate system, \a false if right-handed.
 */
TriMeshDopBoundary::TriMeshDopBoundary(const std::string& objFile, bool clockwise, bool lefthanded)
{
    // Checking for the input file format
    if(objFile.find(".obj") == std::string::npos && objFile.find(".OBJ") == std::string::npos) {
        throw std::invalid_argument("Unsupported triangle mesh format. Only OBJ files are supported.");
    }

    // Reading triangle mesh from OBJ-File
    initOBJ(objFile, 
            vertices_, 
            faceIndices_,
            faceNormals_,
            vertexNormals_, 
            normalIndices_,
            textureCoordinates_, 
            textureIndices_,
            clockwise, lefthanded);
    
    // Calculate center of mass
    real totalVolume(0.0);
    real currentVolume(0.0);
    Vec3 center(0.0, 0.0, 0.0);

    for (size_t i = 0; i < faceIndices_.size(); ++i) {
        const Vec3& a = vertices_[faceIndices_[i][0]];
        const Vec3& b = vertices_[faceIndices_[i][1]];
        const Vec3& c = vertices_[faceIndices_[i][2]];

        currentVolume = (trans(a) * (b % c));
        totalVolume += currentVolume;
        center += (a + b + c) * currentVolume;
    }

    center /= totalVolume * 4.0;
    gpos_ = center;
    
    calcBoundingBox();
    calcOBB();
    computeKDOP();
    
    // Logging the successful creation of the triangle mesh
    pe_LOG_DETAIL_SECTION(log) {
        log << "Created TriMeshDopBoundary from " << objFile << "\n"
            << "   Number of triangles = " << faceIndices_.size() << "\n";
    }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor that loads a triangle mesh from an OBJ file with additional parameters.
 *
 * \param uid Unique ID for the mesh.
 * \param gpos Global position of the mesh.
 * \param file Path to the OBJ file to load.
 * \param material Material ID for the mesh.
 * \param convex Whether the mesh is convex.
 * \param visible Whether the mesh is visible.
 * \param scale Scaling factors for the mesh.
 * \param clockwise \a true if the triangles described in the OBJ file are ordered clockwise, otherwise \a false.
 * \param lefthanded \a true if the coordinates used in the OBJ file are from a left-handed coordinate system, \a false if right-handed.
 */
TriMeshDopBoundary::TriMeshDopBoundary(id_t uid, const Vec3& gpos, const std::string file,
                                      MaterialID material, bool convex, bool visible,
                                      const Vec3& scale, bool clockwise, bool lefthanded)
{
    pe_INTERNAL_ASSERT(convex, "Only convex triangle meshes are allowed right now");

    if(scale[0] <= real(0) || scale[1] <= real(0) || scale[2] <= real(0)){
        throw std::invalid_argument("Invalid scaling factor, only positive scaling allowed.");
    }

    // Checking for the input file format
    if(file.find(".obj") == std::string::npos && file.find(".OBJ") == std::string::npos) {
        throw std::invalid_argument("Unsupported triangle mesh format. Only OBJ files are supported.");
    }

    // Reading triangle mesh from OBJ-File
    initOBJ(file, 
            vertices_, 
            faceIndices_,
            faceNormals_,
            vertexNormals_, 
            normalIndices_,
            textureCoordinates_, 
            textureIndices_,
            clockwise, lefthanded);
    
    // Apply scaling if needed
    if (scale != Vec3(1.0, 1.0, 1.0)) {
        for (auto& vertex : vertices_) {
            vertex[0] *= scale[0];
            vertex[1] *= scale[1];
            vertex[2] *= scale[2];
        }
    }
    
    // Calculate center of mass and volume
    real totalVolume(0.0);
    real currentVolume(0.0);
    Vec3 center(0.0, 0.0, 0.0);

    for (size_t i = 0; i < faceIndices_.size(); ++i) {
        const Vec3& a = vertices_[faceIndices_[i][0]];
        const Vec3& b = vertices_[faceIndices_[i][1]];
        const Vec3& c = vertices_[faceIndices_[i][2]];

        currentVolume = (trans(a) * (b % c));
        totalVolume += currentVolume;
        center += (a + b + c) * currentVolume;
    }

    center /= totalVolume * 4.0;
    gpos_ = center;
    
    // Apply translation to gpos if needed
    if (gpos != Vec3(0.0, 0.0, 0.0)) {
        Vec3 translation = gpos - center;
        for (auto& vertex : vertices_) {
            vertex += translation;
        }
        gpos_ = gpos;
    }
    
    calcBoundingBox();
    calcOBB();
    computeKDOP();
    
    // Logging the successful creation of the triangle mesh
    pe_LOG_DETAIL_SECTION(log) {
        log << "Created TriMeshDopBoundary " << uid << " from " << file << "\n"
            << "   Number of triangles = " << faceIndices_.size() << "\n"
            << "   Position = " << gpos_ << "\n";
    }
}
//*************************************************************************************************


//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts half spaces representing the k-DOP of the triangle mesh.
 *
 * \param halfspaces The list to store the extracted half spaces.
 * \return void
 *
 * This function extracts k half spaces that form the k-DOP (Discrete Oriented Polytope)
 * boundary representation of the triangle mesh. By default, a 6-DOP (Object Oriented
 * Bounding Box) is used.
 */
void TriMeshDopBoundary::extractHalfSpaces(std::list<std::pair<Vec3, real>>& halfspaces) const 
{
    halfspaces.clear();
    
    // Add all the k-DOP planes as half-spaces
    for (size_t i = 0; i < dopDirections_.size(); ++i) {
        halfspaces.push_back(std::make_pair(dopDirections_[i], dopDistances_[i]));
    }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a point is contained in the triangle mesh.
 *
 * \param gpos The global position to test.
 * \return \a true if the point is contained in the triangle mesh, \a false if not.
 */
bool TriMeshDopBoundary::containsPoint(const Vec3& gpos) const 
{
    // First, check against the AABB for quick rejection
    if (!aabb_.contains(gpos)) {
        return false;
    }

    // Then check against the k-DOP for another quick rejection
    for (size_t i = 0; i < dopDirections_.size(); ++i) {
        if (trans(dopDirections_[i]) * gpos > dopDistances_[i]) {
            return false;
        }
    }

    // Finally, perform the ray casting algorithm for precise containment test
    // Choose a direction for the ray
    Vec3 rayDir(1.0, 0.0, 0.0);

    // Count intersections with the triangle mesh
    int intersectionCount = 0;

    // Loop through all faces
    for (size_t f = 0; f < faceIndices_.size(); ++f) {
        const Vec3& A = vertices_[faceIndices_[f][0]];
        const Vec3& B = vertices_[faceIndices_[f][1]];
        const Vec3& C = vertices_[faceIndices_[f][2]];

        // Check intersection
        bool hit = intersectRayTriangle(gpos, rayDir, A, B, C);

        // If there is an intersection, increment the counter
        if (hit) {
            ++intersectionCount;
        }
    }

    // If the intersection count is odd, the point is inside
    return (intersectionCount % 2 == 1);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a point is strictly contained in the triangle mesh.
 *
 * \param gpos The global position to test.
 * \return \a true if the point is strictly contained in the triangle mesh, \a false if not.
 */
bool TriMeshDopBoundary::containsPointStrictly(const Vec3& gpos) const 
{
    // For simplicity, we use the same implementation as containsPoint
    // A more precise implementation would check if the point is on the boundary
    return containsPoint(gpos);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a ray intersects with a triangle.
 *
 * \param rayOrigin The origin of the ray.
 * \param rayDir The direction of the ray.
 * \param V0 The first vertex of the triangle.
 * \param V1 The second vertex of the triangle.
 * \param V2 The third vertex of the triangle.
 * \return \a true if the ray intersects with the triangle, \a false if not.
 */
bool TriMeshDopBoundary::intersectRayTriangle(const Vec3& rayOrigin,
                                           const Vec3& rayDir,
                                           const Vec3& V0,     
                                           const Vec3& V1,     
                                           const Vec3& V2) const 
{
    // Edge vectors
    Vec3 E1 = V1 - V0;
    Vec3 E2 = V2 - V0;

    // Set up matrix and solve system
    Mat3 M(-rayDir, E1, E2);
    
    // Compute determinant
    real det = M.getDeterminant();
    if (std::abs(det) < parallelThreshold) {
        // Ray is parallel or degenerate => no intersection
        return false;
    }
    
    // Solve the linear system using matrix inverse
    Vec3 C = V0 - rayOrigin;
    Vec3 tuv = M.invert() * C;
    
    real t = tuv[0];
    real u = tuv[1];
    real v = tuv[2];
    
    // Check if intersection is valid
    if (t >= 0 && u >= 0 && v >= 0 && (u + v) <= 1) {
        return true;
    } else {
        return false;
    }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the axis-aligned bounding box for the triangle mesh.
 *
 * \return void
 */
void TriMeshDopBoundary::calcBoundingBox() 
{
    if (vertices_.empty()) {
        return;
    }
    
    // Initialize with the centroid
    aabb_[0] = gpos_[0];
    aabb_[1] = gpos_[1];
    aabb_[2] = gpos_[2];
    aabb_[3] = gpos_[0];
    aabb_[4] = gpos_[1];
    aabb_[5] = gpos_[2];
    
    // Expand to include all vertices
    for (const auto& vertex : vertices_) {
        aabb_[0] = std::min(aabb_[0], vertex[0]);
        aabb_[1] = std::min(aabb_[1], vertex[1]);
        aabb_[2] = std::min(aabb_[2], vertex[2]);
        aabb_[3] = std::max(aabb_[3], vertex[0]);
        aabb_[4] = std::max(aabb_[4], vertex[1]);
        aabb_[5] = std::max(aabb_[5], vertex[2]);
    }
    
    // The bounding box is increased by pe::contactThreshold in all dimensions
    aabb_[0] -= contactThreshold;
    aabb_[1] -= contactThreshold;
    aabb_[2] -= contactThreshold;
    aabb_[3] += contactThreshold;
    aabb_[4] += contactThreshold;
    aabb_[5] += contactThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the principal component analysis for the OBB.
 *
 * \return void
 */
void TriMeshDopBoundary::computePCA() 
{
    #ifdef PE_USE_EIGEN
    if (vertices_.empty()) {
        return;
    }
    
    // Create a matrix of vertex positions
    Eigen::MatrixXd points(vertices_.size(), 3);
    for (size_t i = 0; i < vertices_.size(); ++i) {
        points(i, 0) = vertices_[i][0] - gpos_[0];
        points(i, 1) = vertices_[i][1] - gpos_[1];
        points(i, 2) = vertices_[i][2] - gpos_[2];
    }
    
    // Compute the covariance matrix
    Eigen::MatrixXd centered = points.rowwise() - points.colwise().mean();
    Eigen::MatrixXd cov = (centered.transpose() * centered) / double(points.rows() - 1);
    
    // Perform eigendecomposition
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    
    //Extract principal axes (the eigenvectors)
    //
    //In many libraries, the eigenvalues in the result are sorted from smallest to largest (e.g., index 0 is smallest, index 2 is largest if we have a 3×3).
    //We reverse the order so that index 2 becomes the largest variance direction. This is why we do
    //orientation_(i,0) = eig.eigenvectors()(0,2-i);
    //…
    //orientation_(i, 0) = eig.eigenvectors()(0, 2-i);…
    //This effectively picks the largest eigenvector first, second-largest next, and so on.
    for (int i = 0; i < 3; ++i) {
        orientation_(i, 0) = eig.eigenvectors()(0, 2-i);
        orientation_(i, 1) = eig.eigenvectors()(1, 2-i);
        orientation_(i, 2) = eig.eigenvectors()(2, 2-i);
    }
    
    // Ensure we have a right-handed coordinate system
    if (orientation_.getDeterminant() < 0) {
        orientation_(0,2) *= -1;
        orientation_(1,2) *= -1;
        orientation_(2,2) *= -1;
    }
    #else
    // Simple fallback implementation when Eigen is not available
    orientation_ = Mat3(1.0, 1.0, 1.0);
    
    // Compute a simplified covariance matrix
    real cxx = 0.0, cxy = 0.0, cxz = 0.0, cyy = 0.0, cyz = 0.0, czz = 0.0;
    
    for (const auto& v : vertices_) {
        real x = v[0] - gpos_[0];
        real y = v[1] - gpos_[1];
        real z = v[2] - gpos_[2];
        
        cxx += x * x;
        cxy += x * y;
        cxz += x * z;
        cyy += y * y;
        cyz += y * z;
        czz += z * z;
    }
    
    // Normalize
    real n = real(vertices_.size());
    cxx /= n; cxy /= n; cxz /= n;
    cyy /= n; cyz /= n; czz /= n;
    
    // Create the covariance matrix
    Mat3 cov;
    cov(0,0) = cxx; cov(0,1) = cxy; cov(0,2) = cxz;
    cov(1,0) = cxy; cov(1,1) = cyy; cov(1,2) = cyz;
    cov(2,0) = cxz; cov(2,1) = cyz; cov(2,2) = czz;
    
    // For a full PCA we would need to compute eigenvalues/eigenvectors
    // As a simple fallback, we'll just use the axes of the AABB
    pe_LOG_WARNING_SECTION(log) {
        log << "TriMeshDopBoundary::computePCA: Eigen library not available. "
            << "Using simplified orientation computation. For better results, "
            << "enable Eigen support by setting EIGEN=ON.\n";
    }
    
    // Ensure we have a right-handed coordinate system
    if (orientation_.getDeterminant() < 0) {
        orientation_(0,2) *= -1;
        orientation_(1,2) *= -1;
        orientation_(2,2) *= -1;
    }
    #endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the oriented bounding box (6-DOP) for the triangle mesh.
 *
 * \return void
 */
void TriMeshDopBoundary::calcOBB() 
{
    if (vertices_.empty()) {
        return;
    }
    
    // Compute the principal components
    computePCA();
    
    // Initialize half extents to minimum values
    halfExtents_ = Vec3(0, 0, 0);
    
    // Project vertices onto principal axes to find extents
    for (const auto& vertex : vertices_) {
        Vec3 localPos = orientation_.transpose() * (vertex - gpos_);
        
        halfExtents_[0] = std::max(halfExtents_[0], std::abs(localPos[0]));
        halfExtents_[1] = std::max(halfExtents_[1], std::abs(localPos[1]));
        halfExtents_[2] = std::max(halfExtents_[2], std::abs(localPos[2]));
    }
    
    // Add contact threshold to half extents
    halfExtents_[0] += contactThreshold;
    halfExtents_[1] += contactThreshold;
    halfExtents_[2] += contactThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the k-DOP representation for the triangle mesh.
 *
 * \param k The number of directions to use for the k-DOP.
 * \return void
 */
void TriMeshDopBoundary::computeKDOP(size_t k) 
{
    dopDirections_.clear();
    dopDistances_.clear();
    #ifdef PE_USE_EIGEN
    
    // For now, we support 6-DOP (OBB)
    if (k == 6) {
        // Add the 6 directions of the OBB
        dopDirections_.push_back(orientation_.col(0));
        dopDirections_.push_back(orientation_.col(1));
        dopDirections_.push_back(orientation_.col(2));

        dopDirections_.push_back(-orientation_.col(0));
        dopDirections_.push_back(-orientation_.col(1));
        dopDirections_.push_back(-orientation_.col(2));
        
        // Compute the distances
        for (size_t i = 0; i < 3; ++i) {
            real distance = trans(orientation_.col(i)) * gpos_ + halfExtents_[i];
            dopDistances_.push_back(distance);
        }
        
        for (size_t i = 0; i < 3; ++i) {
            real distance = trans(-orientation_.col(i)) * gpos_ + halfExtents_[i];
            dopDistances_.push_back(distance);
        }
    }
    else {
        // In the future, support for more directions can be added here
        pe_LOG_WARNING_SECTION(log) {
            log << "TriMeshDopBoundary::computeKDOP: Only 6-DOP is currently supported. "
                << "Using 6-DOP instead of requested " << k << "-DOP.\n";
        }
        computeKDOP(6);
    }
    #endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with generic bodies.
 *
 * \param b The body to test for intersection.
 * \return \a true if the body intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstBodyID b) const 
{
    // For simplicity, we use the OBB to test for intersections
    // A more precise implementation would test against the triangle mesh
    
    // TODO: Implement more precise intersection tests
    
    return true;  // Default conservative approach
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with spheres.
 *
 * \param s The sphere to test for intersection.
 * \return \a true if the sphere intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstSphereID s) const 
{
    // First, check against the AABB
    if (!aabb_.overlaps(s->getAABB())) {
        return false;
    }

    // Next, check against the k-DOP
    for (size_t i = 0; i < dopDirections_.size(); ++i) {
        real distance = trans(dopDirections_[i]) * s->getPosition() - dopDistances_[i];
        if (distance > s->getRadius()) {
            return false;
        }
    }

    // For simplicity, we consider the sphere to intersect if it passes the k-DOP test
    // A more precise implementation would check against the triangle mesh
    return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with boxes.
 *
 * \param b The box to test for intersection.
 * \return \a true if the box intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstBoxID b) const 
{
    // First, check against the AABB
    if (!aabb_.overlaps(b->getAABB())) {
        return false;
    }

    // For simplicity, we consider the box to intersect if it passes the AABB test
    // A more precise implementation would check against the k-DOP and triangle mesh
    return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with capsules.
 *
 * \param c The capsule to test for intersection.
 * \return \a true if the capsule intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstCapsuleID c) const 
{
    // First, check against the AABB
    if (!aabb_.overlaps(c->getAABB())) {
        return false;
    }

    // For simplicity, we consider the capsule to intersect if it passes the AABB test
    // A more precise implementation would check against the k-DOP and triangle mesh
    return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with cylinders.
 *
 * \param c The cylinder to test for intersection.
 * \return \a true if the cylinder intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstCylinderID c) const 
{
    // First, check against the AABB
    if (!aabb_.overlaps(c->getAABB())) {
        return false;
    }

    // For simplicity, we consider the cylinder to intersect if it passes the AABB test
    // A more precise implementation would check against the k-DOP and triangle mesh
    return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Implements intersection tests with unions.
 *
 * \param u The union to test for intersection.
 * \return \a true if the union intersects with the boundary, \a false if not.
 */
bool TriMeshDopBoundary::intersectsWith(ConstUnionID u) const 
{
    // First, check against the AABB
    if (!aabb_.overlaps(u->getAABB())) {
        return false;
    }

    // For simplicity, we consider the union to intersect if it passes the AABB test
    // A more precise implementation would check against the k-DOP and triangle mesh
    return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prints information about the triangle mesh boundary.
 *
 * \param os The output stream to print to.
 * \return void
 */
void TriMeshDopBoundary::print(std::ostream& os) const 
{
    os << "TriMeshDopBoundary(" 
       << "vertices: " << vertices_.size() 
       << ", faces: " << faceIndices_.size() 
       << ", centroid: " << gpos_ 
       << ")";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prints information about the triangle mesh boundary with tabulation.
 *
 * \param os The output stream to print to.
 * \param tab The tabulation string.
 * \return void
 */
void TriMeshDopBoundary::print(std::ostream& os, const char* tab) const 
{
    os << tab << "TriMeshDopBoundary:\n"
       << tab << "  Vertices: " << vertices_.size() << "\n"
       << tab << "  Faces: " << faceIndices_.size() << "\n"
       << tab << "  Centroid: " << gpos_ << "\n"
       << tab << "  AABB: [" << aabb_[0] << ", " << aabb_[1] << ", " << aabb_[2] << "] - ["
                             << aabb_[3] << ", " << aabb_[4] << ", " << aabb_[5] << "]\n"
       << tab << "  k-DOP: " << dopDirections_.size() << " directions\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Loads all triangle mesh data from a given OBJ-file.
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
 */
void TriMeshDopBoundary::initOBJ(const std::string& file,
                             Vertices& vertices, IndicesLists& faceIndices,
                             Normals& faceNormals,
                             Normals& vertexNormals, IndicesLists normalIndices,
                             TextureCoordinates& textureCoordinates, IndicesLists& textureIndices,
                             bool clockwise, bool lefthanded)
{
    OBJMeshLoader parser;

    parser.load(file, vertices, faceIndices, vertexNormals, normalIndices, textureCoordinates, textureIndices, clockwise, lefthanded);

    if (faceIndices.size() < 4) {
        std::stringstream error;
        error << "TriMeshDopBoundary::initOBJ: TriangleMeshes with less than 4 (" << faceIndices.size() << ") faces not allowed.\n";
        pe_LOG_INFO_SECTION(log) {
            log << error.str();
        }
        throw std::invalid_argument(error.str());
    }

    faceNormals.reserve(faceIndices.size());

    for (size_t f = 0; f < faceIndices.size(); ++f) {
        Vec3& a = vertices[faceIndices[f][0]];
        Vec3& b = vertices[faceIndices[f][1]];
        Vec3& c = vertices[faceIndices[f][2]];

        // calculate face normal
        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 n = ab % ac;
        n.normalize();
        faceNormals.push_back(n);

        // check if face normal points in the same direction as the vertices normals
        bool inverted = false;
        if (normalIndices.size() > 0) {
            real cosVec1 = trans(n) * vertexNormals[normalIndices[f][0]];
            real cosVec2 = trans(n) * vertexNormals[normalIndices[f][1]];
            real cosVec3 = trans(n) * vertexNormals[normalIndices[f][2]];

            if (cosVec1 < 0 && cosVec2 < 0 && cosVec3 < 0 && !inverted) {
                std::stringstream error;
                error << "TriMeshDopBoundary::initOBJ: Vertex order seems to be inverted.\n";
                pe_LOG_INFO_SECTION(log) {
                    log << error.str();
                }
                inverted = true;
            }
            else if (cosVec1 < 0 || cosVec2 < 0 || cosVec3 < 0) {
                std::stringstream error;
                error << "TriMeshDopBoundary::initOBJ: At least one vertex normal does not match face normal direction.\n";
                pe_LOG_INFO_SECTION(log) {
                    log << error.str();
                }
            }
        }
    }
}
//*************************************************************************************************

} // namespace pe