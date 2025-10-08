//******************************************************************************************
//
// Debug test for coordinate transformations in TriangleMesh with DistanceMap
// Tests the specific case where mesh translation affects point containment queries
//
//******************************************************************************************

#include <iostream>
#include <string>
#include <iomanip>

// Local headers
#include <pe/core/detection/fine/DistanceMap.h>

// PE headers
#include <pe/core.h>
#include <pe/math.h>
#include <pe/core/rigidbody/TriangleMesh.h>

// Using pe namespace
using namespace pe;

void testPointContainment(const TriangleMeshID& mesh, const Vec3& testPoint, const std::string& description) {
    std::cout << "\n=== " << description << " ===" << std::endl;
    
    // Print mesh position
    std::cout << "Mesh position: (" << mesh->getPosition()[0] << ", " << mesh->getPosition()[1] << ", " << mesh->getPosition()[2] << ")" << std::endl;
    std::cout << "Test point:    (" << testPoint[0] << ", " << testPoint[1] << ", " << testPoint[2] << ")" << std::endl;
    
    // Test with DistanceMap if available
    if (mesh->hasDistanceMap()) {
        bool dmResult = mesh->containsPoint(testPoint);
        std::cout << "DistanceMap result: " << (dmResult ? "INSIDE" : "OUTSIDE") << std::endl;
        
        // Get additional DistanceMap info for debugging
        const DistanceMap* dm = mesh->getDistanceMap();
        if (dm) {
            // Transform point to local coordinates (same as mesh does internally)
            Vec3 localPoint = mesh->pointFromWFtoBF(testPoint);
            std::cout << "Local point:   (" << localPoint[0] << ", " << localPoint[1] << ", " << localPoint[2] << ")" << std::endl;
            
            // Query distance directly
            pe::real distance = dm->interpolateDistance(localPoint[0], localPoint[1], localPoint[2]);
            std::cout << "Signed distance: " << distance << std::endl;
            std::cout << "Distance sign indicates: " << (distance < 0 ? "INSIDE" : "OUTSIDE") << std::endl;
            
            // Check if point is within DistanceMap bounds
            Vec3 dmOrigin = dm->getOrigin();
            pe::real spacing = dm->getSpacing();
            int nx = dm->getNx(), ny = dm->getNy(), nz = dm->getNz();
            
            bool inBounds = (localPoint[0] >= dmOrigin[0] && localPoint[0] <= dmOrigin[0] + (nx-1)*spacing) &&
                           (localPoint[1] >= dmOrigin[1] && localPoint[1] <= dmOrigin[1] + (ny-1)*spacing) &&
                           (localPoint[2] >= dmOrigin[2] && localPoint[2] <= dmOrigin[2] + (nz-1)*spacing);
            
            std::cout << "Point in DM bounds: " << (inBounds ? "YES" : "NO") << std::endl;
            if (!inBounds) {
                std::cout << "DistanceMap bounds: [" << dmOrigin[0] << "," << dmOrigin[0] + (nx-1)*spacing << "] x "
                          << "[" << dmOrigin[1] << "," << dmOrigin[1] + (ny-1)*spacing << "] x "
                          << "[" << dmOrigin[2] << "," << dmOrigin[2] + (nz-1)*spacing << "]" << std::endl;
            }
        }
    } else {
        std::cout << "No DistanceMap available!" << std::endl;
    }
    
    // Print mesh AABB for reference
    const auto& bbox = mesh->getAABB();
    std::cout << "Mesh AABB: [" << bbox[3] << "," << bbox[0] << "] x ["
              << bbox[4] << "," << bbox[1] << "] x ["
              << bbox[5] << "," << bbox[2] << "]" << std::endl;
    
    // Check if point is in AABB
    bool inAABB = (testPoint[0] >= bbox[3] && testPoint[0] <= bbox[0]) &&
                  (testPoint[1] >= bbox[4] && testPoint[1] <= bbox[1]) &&
                  (testPoint[2] >= bbox[5] && testPoint[2] <= bbox[2]);
    std::cout << "Point in AABB: " << (inAABB ? "YES" : "NO") << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Debug: Coordinate Transformation Test ===" << std::endl;
    
#ifdef PE_USE_CGAL
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <mesh_file.obj>" << std::endl;
        return 1;
    }
    
    std::string meshFile = argv[1];
    
    // Fixed test point from ParaView analysis
    Vec3 testPoint(-0.086035, -0.869566, 0.477526);
    
    try {
        // Initialize PE world
        WorldID world = theWorld();
        MaterialID material = createMaterial("debug", 1000, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
        
        std::cout << "\n=== PHASE 1: Mesh at Origin ===" << std::endl;
        
        // Create mesh at origin
        TriangleMeshID mesh = createTriangleMesh(1, Vec3(0, 0, 0), meshFile, material, false, true);
        
        if (!mesh) {
            std::cerr << "ERROR: Failed to load mesh from " << meshFile << std::endl;
            return 1;
        }
        
        std::cout << "Mesh loaded successfully!" << std::endl;
        
        // Enable DistanceMap
        mesh->enableDistanceMapAcceleration(64, 3);
        
        if (!mesh->hasDistanceMap()) {
            std::cerr << "ERROR: DistanceMap failed to initialize" << std::endl;
            return 1;
        }
        
        std::cout << "DistanceMap enabled successfully!" << std::endl;
        
        // Test 1: Original point with mesh at origin - should be INSIDE
        testPointContainment(mesh, testPoint, "Test 1: Point vs Mesh at Origin (Expected: INSIDE)");
        
        std::cout << "\n=== PHASE 2: Translated Point ===" << std::endl;
        
        // Test 2: Translate point +10 in Z - should be OUTSIDE
        Vec3 translatedPoint = testPoint + Vec3(0, 0, 10);
        testPointContainment(mesh, translatedPoint, "Test 2: Point+10Z vs Mesh at Origin (Expected: OUTSIDE)");
        
        std::cout << "\n=== PHASE 3: Translated Mesh ===" << std::endl;
        
        // Test 3: Translate mesh +10 in Z to align with translated point - should be INSIDE again
        mesh->setPosition(Vec3(0, 0, 10));
        
        // Force update of cached data
        mesh->calcBoundingBox();
        
        testPointContainment(mesh, translatedPoint, "Test 3: Point+10Z vs Mesh+10Z (Expected: INSIDE)");
        
        std::cout << "\n=== PHASE 4: Cross-Validation ===" << std::endl;
        
        // Test 4: Original point vs translated mesh - should be OUTSIDE
        testPointContainment(mesh, testPoint, "Test 4: Original Point vs Mesh+10Z (Expected: OUTSIDE)");
        
        std::cout << "\n=== SUMMARY ===" << std::endl;
        std::cout << "If coordinate transformations work correctly:" << std::endl;
        std::cout << "  Test 1: INSIDE  (point in mesh at origin)" << std::endl;
        std::cout << "  Test 2: OUTSIDE (point moved away from mesh)" << std::endl;
        std::cout << "  Test 3: INSIDE  (point and mesh moved together)" << std::endl;
        std::cout << "  Test 4: OUTSIDE (original point, moved mesh)" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
    
#else
    std::cout << "ERROR: This debug test requires CGAL support. Please compile with -DCGAL=ON" << std::endl;
    return 1;
#endif
}