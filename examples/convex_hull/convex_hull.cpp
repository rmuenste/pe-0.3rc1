//******************************************************************************************
//
// This example demonstrates the CGAL integration for computing convex hulls of triangle
// meshes in the PE physics engine. It loads a mesh from file, computes its convex hull,
// and outputs both the original mesh and the convex hull to OBJ files for visualization.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <iomanip>

// PE headers
#include <pe/engine.h>
#include <pe/support.h>

using namespace pe;

//*************************************************************************************************
/*!\brief Writes vertices and faces to an OBJ file.
 *
 * \param vertices The vertex data to write.
 * \param faces The face index data to write.
 * \param filename The output filename.
 * \return void
 *
 * This utility function writes triangle mesh data in OBJ format for visualization.
 */
void writeOBJ(const Vertices& vertices, const IndicesLists& faces, const std::string& filename) {
    std::ofstream obj_file(filename);
    if (!obj_file) {
        std::cerr << "Error: Cannot create file " << filename << std::endl;
        return;
    }
    
    // Write vertices
    for (const auto& vertex : vertices) {
        obj_file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }
    
    // Write faces (OBJ indices are 1-based)
    for (const auto& face : faces) {
        obj_file << "f " << (face[0] + 1) << " " << (face[1] + 1) << " " << (face[2] + 1) << std::endl;
    }
    
    obj_file.close();
    std::cout << "Mesh written to " << filename << std::endl;
}

//*************************************************************************************************
/*!\brief Creates a test triangle mesh with known geometry.
 *
 * \return TriangleMeshID for the created test mesh.
 *
 * This function creates a clearly non-convex "C"-shaped mesh for testing convex hull computation.
 */
TriangleMeshID createTestTriangleMesh() {
    // Create a clearly non-convex mesh: a "C" or horseshoe shape
    // This will have a concave region that should be "filled in" by the convex hull
    Vertices vertices = {
        // Outer boundary of the "C" shape
        Vec3(-2.0, -1.0, -0.5), // 0: outer left bottom
        Vec3(-2.0,  1.0, -0.5), // 1: outer left top
        Vec3( 1.0,  1.0, -0.5), // 2: outer right top
        Vec3( 1.0,  0.5, -0.5), // 3: inner right top
        Vec3(-1.0,  0.5, -0.5), // 4: inner left top
        Vec3(-1.0, -0.5, -0.5), // 5: inner left bottom
        Vec3( 1.0, -0.5, -0.5), // 6: inner right bottom
        Vec3( 1.0, -1.0, -0.5), // 7: outer right bottom
        
        // Same shape at different Z level (making it 3D)
        Vec3(-2.0, -1.0,  0.5), // 8: outer left bottom
        Vec3(-2.0,  1.0,  0.5), // 9: outer left top
        Vec3( 1.0,  1.0,  0.5), // 10: outer right top
        Vec3( 1.0,  0.5,  0.5), // 11: inner right top
        Vec3(-1.0,  0.5,  0.5), // 12: inner left top
        Vec3(-1.0, -0.5,  0.5), // 13: inner left bottom
        Vec3( 1.0, -0.5,  0.5), // 14: inner right bottom
        Vec3( 1.0, -1.0,  0.5), // 15: outer right bottom
        
        // Add some points inside the concave region that should be part of convex hull
        Vec3( 0.0,  0.0, -0.5), // 16: center of concave region (bottom)
        Vec3( 0.0,  0.0,  0.5), // 17: center of concave region (top)
        Vec3(-0.5,  0.0,  0.0), // 18: middle of concave region
    };
    
    // Create faces for the "C" shape (non-convex)
    IndicesLists faces = {
        // Bottom face (Z = -0.5) - "C" shape
        Vector3<size_t>(0, 1, 4), Vector3<size_t>(0, 4, 5),
        Vector3<size_t>(0, 5, 6), Vector3<size_t>(0, 6, 7),
        Vector3<size_t>(1, 2, 3), Vector3<size_t>(1, 3, 4),
        
        // Top face (Z = 0.5) - "C" shape  
        Vector3<size_t>(8, 12, 9), Vector3<size_t>(8, 13, 12),
        Vector3<size_t>(8, 14, 13), Vector3<size_t>(8, 15, 14),
        Vector3<size_t>(9, 11, 10), Vector3<size_t>(9, 12, 11),
        
        // Side faces connecting bottom and top
        Vector3<size_t>(0, 8, 9), Vector3<size_t>(0, 9, 1),   // left outer
        Vector3<size_t>(1, 9, 10), Vector3<size_t>(1, 10, 2), // top outer
        Vector3<size_t>(2, 10, 11), Vector3<size_t>(2, 11, 3), // right top
        Vector3<size_t>(3, 11, 12), Vector3<size_t>(3, 12, 4), // inner top
        Vector3<size_t>(4, 12, 13), Vector3<size_t>(4, 13, 5), // inner left
        Vector3<size_t>(5, 13, 14), Vector3<size_t>(5, 14, 6), // inner bottom
        Vector3<size_t>(6, 14, 15), Vector3<size_t>(6, 15, 7), // right bottom
        Vector3<size_t>(7, 15, 8), Vector3<size_t>(7, 8, 0),   // bottom outer
        
        // Add some internal faces to make it clearly non-convex
        Vector3<size_t>(16, 4, 5), Vector3<size_t>(16, 5, 6),
        Vector3<size_t>(17, 12, 11), Vector3<size_t>(17, 13, 12),
        Vector3<size_t>(18, 16, 17), Vector3<size_t>(18, 4, 16)
    };
    
    // Create the triangle mesh
    MaterialID testMaterial = createMaterial("test", 1000.0, 0.3, 0.2, 0.05, 0.1, 300.0, 1e6, 1e5, 2e5);
    return createTriangleMesh(1, Vec3(0, 0, 0), vertices, faces, testMaterial, false, true);
}

//*************************************************************************************************
/*!\brief Main function demonstrating CGAL convex hull integration.
 */
int main(int argc, char** argv) {
    std::cout << "PE Physics Engine - CGAL Convex Hull Integration Example\n";
    std::cout << "=========================================================\n\n";

#ifdef PE_USE_CGAL
    std::cout << "CGAL support is ENABLED\n\n";
#else
    std::cout << "CGAL support is DISABLED - this example will demonstrate fallback behavior\n\n";
#endif

    try {
        // Initialize PE world
        WorldID world = theWorld();
        
        TriangleMeshID mesh;
        
        if (argc > 1) {
            // Load mesh from file if provided
            std::string filename = argv[1];
            std::cout << "Loading mesh from file: " << filename << std::endl;
            
            MaterialID material = createMaterial("default", 1000.0, 0.3, 0.2, 0.05, 0.1, 300.0, 1e6, 1e5, 2e5);
            mesh = createTriangleMesh(1, Vec3(0, 0, 0), filename, material, false, true);
            
            if (!mesh) {
                std::cerr << "Error: Failed to load mesh from " << filename << std::endl;
                return 1;
            }
        } else {
            // Create test mesh
            std::cout << "Creating test mesh (non-convex C-shape)..." << std::endl;
            mesh = createTestTriangleMesh();
        }
        
        std::cout << "Original mesh:" << std::endl;
        std::cout << "  Vertices: " << mesh->getBFVertices().size() << std::endl;
        std::cout << "  Faces: " << mesh->getFaceIndices().size() << std::endl;
        
        // Write original mesh to file
        writeOBJ(mesh->getBFVertices(), mesh->getFaceIndices(), "original_mesh.obj");
        
        // Compute convex hull
        std::cout << "\nComputing convex hull..." << std::endl;
        
        Vertices hull_vertices;
        IndicesLists hull_faces;
        
        bool success = mesh->computeConvexHull(hull_vertices, hull_faces);
        
        if (success) {
            std::cout << "Convex hull computed successfully!" << std::endl;
            std::cout << "  Hull vertices: " << hull_vertices.size() << std::endl;
            std::cout << "  Hull faces: " << hull_faces.size() << std::endl;
            
            // Write convex hull to file
            writeOBJ(hull_vertices, hull_faces, "convex_hull.obj");
            
            std::cout << "\nConvex hull statistics:" << std::endl;
            std::cout << "  Reduction in vertices: " 
                      << mesh->getBFVertices().size() - hull_vertices.size() 
                      << " (" << (100.0 * (mesh->getBFVertices().size() - hull_vertices.size()) / mesh->getBFVertices().size())
                      << "%)" << std::endl;
            std::cout << "  Reduction in faces: " 
                      << mesh->getFaceIndices().size() - hull_faces.size() 
                      << " (" << (100.0 * (mesh->getFaceIndices().size() - hull_faces.size()) / mesh->getFaceIndices().size())
                      << "%)" << std::endl;
            
        } else {
#ifdef PE_USE_CGAL
            std::cout << "Convex hull computation failed!" << std::endl;
            std::cerr << "This could be due to insufficient points or CGAL computation errors." << std::endl;
#else
            std::cout << "Convex hull computation returned false (expected - CGAL not available)" << std::endl;
            std::cout << "To enable CGAL functionality, build with -DCGAL=ON" << std::endl;
#endif
        }
        
        // Test AABB tree and distance computation functionality
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "CGAL AABB Tree Distance Computation Demo" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
#ifdef PE_USE_CGAL
        std::cout << "\nTesting distance computation with CGAL AABB tree..." << std::endl;
        
        // Enable distance acceleration for better performance on larger meshes
        mesh->enableDistanceAcceleration();
        std::cout << "Distance acceleration enabled." << std::endl;
        
        // Test points for distance computation
        std::vector<Vec3> testPoints = {
            Vec3(0.0, 0.0, 0.0),    // Center point (should be inside or very close)
            Vec3(5.0, 0.0, 0.0),    // Far point on positive X axis
            Vec3(-5.0, 0.0, 0.0),   // Far point on negative X axis
            Vec3(0.0, 5.0, 0.0),    // Far point on positive Y axis
            Vec3(0.0, -5.0, 0.0),   // Far point on negative Y axis
            Vec3(0.0, 0.0, 5.0),    // Far point on positive Z axis
            Vec3(0.0, 0.0, -5.0),   // Far point on negative Z axis
            Vec3(2.0, 2.0, 2.0),    // Diagonal point
            Vec3(-0.5, 0.25, 0.0),  // Point inside the concave region (for C-shape)
        };
        
        std::cout << "\nDistance computation results:" << std::endl;
        std::cout << "Query Point          | Distance²  | Closest Point" << std::endl;
        std::cout << std::string(65, '-') << std::endl;
        
        for (size_t i = 0; i < testPoints.size(); ++i) {
            const Vec3& queryPoint = testPoints[i];
            
            // Compute squared distance
            real sqDistance = mesh->distanceToPoint(queryPoint);
            
            // Find closest point
            Vec3 closestPt = mesh->closestPoint(queryPoint);
            
            // Print results
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "(" << std::setw(5) << queryPoint[0] 
                      << "," << std::setw(5) << queryPoint[1]
                      << "," << std::setw(5) << queryPoint[2] << ") | "
                      << std::setw(8) << sqDistance << " | "
                      << "(" << std::setw(5) << closestPt[0]
                      << "," << std::setw(5) << closestPt[1] 
                      << "," << std::setw(5) << closestPt[2] << ")" << std::endl;
        }
        
        // Test closest point and primitive functionality
        std::cout << "\nTesting closest point with primitive ID..." << std::endl;
        Vec3 testPoint(1.5, 1.5, 0.0);
        auto result = mesh->closestPointAndPrimitive(testPoint);
        
        std::cout << "Query point: (" << testPoint[0] << ", " << testPoint[1] << ", " << testPoint[2] << ")" << std::endl;
        std::cout << "Closest point: (" << result.first[0] << ", " << result.first[1] << ", " << result.first[2] << ")" << std::endl;
        std::cout << "Primitive (face) ID: " << result.second << std::endl;
        
        std::cout << "\nAABB tree distance computation completed!" << std::endl;
        
#else
        std::cout << "\nCGAL support is DISABLED - distance computation functionality not available" << std::endl;
        std::cout << "To enable CGAL distance queries, build with -DCGAL=ON" << std::endl;
        
        // Test the fallback behavior
        std::cout << "\nTesting fallback behavior (should return default values):" << std::endl;
        Vec3 testPoint(1.0, 1.0, 1.0);
        real distance = mesh->distanceToPoint(testPoint);
        Vec3 closest = mesh->closestPoint(testPoint);
        auto result = mesh->closestPointAndPrimitive(testPoint);
        
        std::cout << "Distance to point: " << distance << " (expected: 0.0)" << std::endl;
        std::cout << "Closest point: (" << closest[0] << ", " << closest[1] << ", " << closest[2] << ") (expected: input point)" << std::endl;
        std::cout << "Primitive ID: " << result.second << " (expected: 0)" << std::endl;
#endif
        
        // Test Point Containment and Signed Distance functionality
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Point Containment & Signed Distance Demo" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        // Test points for containment (designed for the C-shaped mesh)
        std::vector<std::pair<Vec3, std::string>> containmentTestPoints = {
            {Vec3(0.0, 0.0, 0.0), "Center (concave region)"},
            {Vec3(-0.5, 0.0, 0.0), "Inside concave region"},
            {Vec3(-1.5, 0.0, 0.0), "Inside solid part"},
            {Vec3(5.0, 0.0, 0.0), "Far outside (positive X)"},
            {Vec3(-5.0, 0.0, 0.0), "Far outside (negative X)"},
            {Vec3(0.0, 3.0, 0.0), "Far outside (positive Y)"},
            {Vec3(-1.5, 0.8, 0.0), "Inside thick part"},
            {Vec3(0.5, 0.8, 0.0), "Inside upper arm"},
            {Vec3(0.5, -0.8, 0.0), "Inside lower arm"},
        };
        
        std::cout << "\nPoint containment and signed distance results:" << std::endl;
        std::cout << "Query Point          | Description          | Inside | Signed Dist" << std::endl;
        std::cout << std::string(75, '-') << std::endl;
        
        for (const auto& testCase : containmentTestPoints) {
            const Vec3& queryPoint = testCase.first;
            const std::string& description = testCase.second;
            
            // Test point containment
            bool inside = mesh->containsPoint(queryPoint);
            
            // Test signed distance
            real signedDist = mesh->signedDistance(queryPoint);
            
            // Print results
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "(" << std::setw(5) << queryPoint[0] 
                      << "," << std::setw(5) << queryPoint[1]
                      << "," << std::setw(5) << queryPoint[2] << ") | "
                      << std::setw(20) << description << " | "
                      << std::setw(6) << (inside ? "YES" : "NO") << " | "
                      << std::setw(8) << signedDist << std::endl;
        }
        
#ifdef PE_USE_CGAL
        std::cout << "\nPoint containment analysis:" << std::endl;
        std::cout << "- Negative signed distances indicate points inside the mesh" << std::endl;
        std::cout << "- Positive signed distances indicate points outside the mesh" << std::endl;
        std::cout << "- The C-shaped mesh should show complex inside/outside behavior" << std::endl;
        std::cout << "- Points in the concave region should be classified as outside" << std::endl;
#else
        std::cout << "\nCGAL support is DISABLED - point containment returns false (conservative)" << std::endl;
        std::cout << "Signed distance returns 0.0 as fallback when CGAL is not available" << std::endl;
        std::cout << "To enable full point containment functionality, build with -DCGAL=ON" << std::endl;
#endif
        
        std::cout << "\nExample completed!" << std::endl;
        std::cout << "Output files:" << std::endl;
        std::cout << "  - original_mesh.obj: Original input mesh" << std::endl;
        if (success) {
            std::cout << "  - convex_hull.obj: Computed convex hull" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}