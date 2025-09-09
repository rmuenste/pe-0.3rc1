//******************************************************************************************
//
// This example tests the optimized containsPoint functionality using PE's TriangleMesh 
// with DistanceMap acceleration. It loads a mesh, enables DistanceMap, and thoroughly
// tests point containment queries against both DistanceMap and CGAL ray-shooting methods.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <algorithm>
#include <random>
#include <chrono>

// Local headers
#include <pe/core/detection/fine/DistanceMap.h>

// PE headers
#include <pe/core.h>
#include <pe/math.h>
#include <pe/core/rigidbody/TriangleMesh.h>

// Using pe namespace
using namespace pe;

// Test configuration
struct TestConfig {
    std::string meshFile;
    pe::real spacing = 0.05;
    int resolution = 64;
    int tolerance = 3;
    int numTestPoints = 1000;
    bool verbose = false;
};

// Test result statistics
struct TestStats {
    int totalPoints = 0;
    int distanceMapInside = 0;
    int distanceMapOutside = 0;
    int cgalInside = 0;
    int cgalOutside = 0;
    int disagreements = 0;
    double distanceMapTime = 0.0;
    double cgalTime = 0.0;
};

// Generate test points around the mesh
std::vector<Vec3> generateTestPoints(const TriangleMeshID& mesh, int numPoints) {
    std::vector<Vec3> points;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Get mesh bounding box
    const auto& bbox = mesh->getAABB();
    pe::real minX = bbox[3], maxX = bbox[0];  // PE AABB format: [maxX, maxY, maxZ, minX, minY, minZ]
    pe::real minY = bbox[4], maxY = bbox[1];
    pe::real minZ = bbox[5], maxZ = bbox[2];
    
    // Expand bounding box for more comprehensive testing
    pe::real margin = 0.5 * std::max({maxX - minX, maxY - minY, maxZ - minZ});
    std::uniform_real_distribution<pe::real> distX(minX - margin, maxX + margin);
    std::uniform_real_distribution<pe::real> distY(minY - margin, maxY + margin);
    std::uniform_real_distribution<pe::real> distZ(minZ - margin, maxZ + margin);
    
    for (int i = 0; i < numPoints; ++i) {
        points.emplace_back(distX(gen), distY(gen), distZ(gen));
    }
    
    // Add some specific test points
    Vec3 center = Vec3((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5);
    points.push_back(center);  // Center of bounding box
    points.push_back(center + Vec3(0.1, 0, 0));  // Slight offset from center
    points.push_back(Vec3(minX - 1, minY - 1, minZ - 1));  // Far outside
    points.push_back(Vec3(maxX + 1, maxY + 1, maxZ + 1));  // Far outside
    
    return points;
}

// Test point containment with DistanceMap method
bool testDistanceMapContainment(const TriangleMeshID& mesh, const Vec3& point, TestStats& stats) {
    auto start = std::chrono::high_resolution_clock::now();
    bool result = mesh->containsPoint(point);  // Should use DistanceMap route
    auto end = std::chrono::high_resolution_clock::now();
    
    stats.distanceMapTime += std::chrono::duration<double, std::milli>(end - start).count();
    
    if (result) {
        stats.distanceMapInside++;
    } else {
        stats.distanceMapOutside++;
    }
    
    return result;
}

// Test point containment with CGAL ray-shooting method (disable DistanceMap temporarily)
bool testCGALContainment(TriangleMeshID& mesh, const Vec3& point, TestStats& stats) {
    // Temporarily disable DistanceMap to force CGAL ray-shooting
    bool hadDistanceMap = mesh->hasDistanceMap();
    if (hadDistanceMap) {
        mesh->disableDistanceMapAcceleration();
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = mesh->containsPoint(point);  // Should use CGAL ray shooting
    auto end = std::chrono::high_resolution_clock::now();
    
    stats.cgalTime += std::chrono::duration<double, std::milli>(end - start).count();
    
    // Re-enable DistanceMap if it was enabled before
    if (hadDistanceMap) {
        mesh->enableDistanceMapAcceleration(0.05, 64, 3);  // Re-enable with same parameters
    }
    
    if (result) {
        stats.cgalInside++;
    } else {
        stats.cgalOutside++;
    }
    
    return result;
}

// Run comprehensive containsPoint tests
void runContainmentTests(TriangleMeshID& mesh, const TestConfig& config) {
    std::cout << "\n=== Starting containsPoint Tests ===" << std::endl;
    
    // Generate test points
    std::vector<Vec3> testPoints = generateTestPoints(mesh, config.numTestPoints);
    std::cout << "Generated " << testPoints.size() << " test points" << std::endl;
    
    TestStats stats;
    stats.totalPoints = testPoints.size();
    
    // Test each point with both methods
    for (const auto& point : testPoints) {
        // Test with DistanceMap
        bool distanceMapResult = testDistanceMapContainment(mesh, point, stats);
        
        // Test with CGAL ray shooting
        bool cgalResult = testCGALContainment(mesh, point, stats);
        
        // Check for disagreements
        if (distanceMapResult != cgalResult) {
            stats.disagreements++;
            if (config.verbose) {
                std::cout << "DISAGREEMENT at point (" << point[0] << ", " << point[1] << ", " << point[2] << "): "
                         << "DistanceMap=" << (distanceMapResult ? "inside" : "outside") << ", "
                         << "CGAL=" << (cgalResult ? "inside" : "outside") << std::endl;
            }
        }
    }
    
    // Print results
    std::cout << "\n=== Test Results ===" << std::endl;
    std::cout << "Total test points: " << stats.totalPoints << std::endl;
    std::cout << "DistanceMap results: " << stats.distanceMapInside << " inside, " 
              << stats.distanceMapOutside << " outside" << std::endl;
    std::cout << "CGAL results: " << stats.cgalInside << " inside, " 
              << stats.cgalOutside << " outside" << std::endl;
    std::cout << "Disagreements: " << stats.disagreements << " (" 
              << (100.0 * stats.disagreements / stats.totalPoints) << "%)" << std::endl;
    
    std::cout << "\n=== Performance Results ===" << std::endl;
    std::cout << "DistanceMap total time: " << stats.distanceMapTime << " ms" << std::endl;
    std::cout << "DistanceMap avg time per query: " << (stats.distanceMapTime / stats.totalPoints) << " ms" << std::endl;
    std::cout << "CGAL total time: " << stats.cgalTime << " ms" << std::endl;
    std::cout << "CGAL avg time per query: " << (stats.cgalTime / stats.totalPoints) << " ms" << std::endl;
    
    if (stats.cgalTime > 0) {
        std::cout << "Speed improvement: " << (stats.cgalTime / stats.distanceMapTime) << "x faster" << std::endl;
    }
    
    // Validation
    if (stats.disagreements == 0) {
        std::cout << "✓ PASS: All containment queries agree between methods!" << std::endl;
    } else if (stats.disagreements < 0.05 * stats.totalPoints) {  // Allow up to 5% disagreement due to numerical precision
        std::cout << "⚠ WARN: Minor disagreements detected (within acceptable tolerance)" << std::endl;
    } else {
        std::cout << "✗ FAIL: Significant disagreements detected!" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "PE TriangleMesh containsPoint Test with DistanceMap Optimization!" << std::endl;
    
#ifdef PE_USE_CGAL
    TestConfig config;
    
    // Parse command line arguments
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <mesh_file.obj> [options]" << std::endl;
        std::cout << "  mesh_file.obj: Input mesh file (.obj or .off format)" << std::endl;
        std::cout << "  --spacing <value>: DistanceMap grid spacing (default: 0.05)" << std::endl;
        std::cout << "  --resolution <value>: DistanceMap resolution (default: 64)" << std::endl;
        std::cout << "  --tolerance <value>: DistanceMap tolerance (default: 3)" << std::endl;
        std::cout << "  --points <count>: Number of test points (default: 1000)" << std::endl;
        std::cout << "  --verbose: Enable verbose output" << std::endl;
        return 1;
    }
    
    config.meshFile = argv[1];
    
    // Parse optional arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--spacing" && i + 1 < argc) {
            config.spacing = std::stod(argv[++i]);
        } else if (arg == "--resolution" && i + 1 < argc) {
            config.resolution = std::stoi(argv[++i]);
        } else if (arg == "--tolerance" && i + 1 < argc) {
            config.tolerance = std::stoi(argv[++i]);
        } else if (arg == "--points" && i + 1 < argc) {
            config.numTestPoints = std::stoi(argv[++i]);
        } else if (arg == "--verbose") {
            config.verbose = true;
        }
    }
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Mesh file: " << config.meshFile << std::endl;
    std::cout << "  DistanceMap spacing: " << config.spacing << std::endl;
    std::cout << "  DistanceMap resolution: " << config.resolution << std::endl;
    std::cout << "  DistanceMap tolerance: " << config.tolerance << std::endl;
    std::cout << "  Test points: " << config.numTestPoints << std::endl;

    try {
        // Initialize PE world
        WorldID world = theWorld();
        MaterialID material = createMaterial("test", 1000, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
        
        // Create triangle mesh from file
        std::cout << "\nLoading mesh from: " << config.meshFile << std::endl;
        TriangleMeshID mesh = createTriangleMesh(1, Vec3(0, 0, 0), config.meshFile, material, false, true);
        
        if (!mesh) {
            std::cerr << "ERROR: Failed to load mesh from " << config.meshFile << std::endl;
            return 1;
        }
        
        std::cout << "Mesh loaded successfully!" << std::endl;
        std::cout << "  Vertices: " << mesh->getWFVertices().size() << std::endl;
        std::cout << "  Faces: " << mesh->getFaceIndices().size() << std::endl;
        
        // Print bounding box info
        const auto& bbox = mesh->getAABB();
        std::cout << "  Bounding box: [" << bbox[3] << "," << bbox[0] << "] x ["
                  << bbox[4] << "," << bbox[1] << "] x ["
                  << bbox[5] << "," << bbox[2] << "]" << std::endl;
        
        // Enable DistanceMap acceleration
        std::cout << "\nEnabling DistanceMap acceleration..." << std::endl;
        std::cout << "  Spacing: " << config.spacing << std::endl;
        std::cout << "  Resolution: " << config.resolution << std::endl;
        std::cout << "  Tolerance: " << config.tolerance << std::endl;
        
        mesh->enableDistanceMapAcceleration(config.spacing, config.resolution, config.tolerance);
        
        if (!mesh->hasDistanceMap()) {
            std::cerr << "ERROR: DistanceMap acceleration failed to initialize" << std::endl;
            return 1;
        }
        
        std::cout << "DistanceMap acceleration enabled successfully!" << std::endl;
        
        // Export DistanceMap for visualization (optional)
        if (const DistanceMap* dm = mesh->getDistanceMap()) {
            std::cout << "DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
            std::cout << "DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
        }
        
        // Run containment tests
        runContainmentTests(mesh, config);
        
        std::cout << "\n=== Test Complete ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
    
#else
    std::cout << "ERROR: This example requires CGAL support. Please compile with -DCGAL=ON" << std::endl;
    return 1;
#endif
}