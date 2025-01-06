#include <pe/core/domaindecomp/TriMeshBoundary.h>

namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

TriMeshBoundary::TriMeshBoundary(const std::vector<Vec3>& vertices, 
                                 const std::vector<std::array<int, 3>>& triangles) 
                                 : vertices_(vertices), triangles_(triangles) {}

bool TriMeshBoundary::intersectsWith(ConstBodyID b) const {
    // Implement intersection test with another body
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::intersectsWith(ConstSphereID s) const {
    // Implement intersection test with a sphere
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::intersectsWith(ConstBoxID b) const {
    // Implement intersection test with a box
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::intersectsWith(ConstCapsuleID c) const {
    // Implement intersection test with a capsule
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::intersectsWith(ConstCylinderID c) const {
    // Implement intersection test with a cylinder
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::intersectsWith(ConstUnionID u) const {
    // Implement intersection test with a union of shapes
    return true;  // Return a dummy value for now
}

bool TriMeshBoundary::containsPoint(const Vec3& gpos) const {
    // Check if the point is inside the surface triangulation
    return false;  // Return a dummy value for now
}

bool TriMeshBoundary::containsPointStrictly(const Vec3& gpos) const {
    // Check if the point is strictly inside the surface triangulation, meaning it's not on the boundary
    return false;  // Return a dummy value for now
}

std::vector<Vec3> TriMeshBoundary::getVertices() const {
    // Get the vertices of the surface triangulation
    return vertices_;  // Return the stored vertices
}

std::vector<std::array<int, 3>> TriMeshBoundary::getTriangles() const {
    // Get the triangles that make up the surface triangulation
    return triangles_;  // Return the stored triangles
}

} // namespace pe
