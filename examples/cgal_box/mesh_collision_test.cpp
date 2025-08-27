//******************************************************************************************
//
// This example tests DistanceMap-accelerated collision detection using PE's TriangleMesh interface.
// It creates two TriangleMesh objects, enables DistanceMap on one, and tests collision detection.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <algorithm>
#include <cctype>

// Local headers
#include "DistanceMap.h"
#include "VtkOutput.h"

// PE headers
#include <pe/core.h>
#include <pe/math.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/detection/fine/MaxContacts.h>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/contact/Contact.h>

// Using pe namespace
using namespace pe;

int main(int argc, char* argv[]) {
    std::cout << "PE TriangleMesh DistanceMap Collision Test!" << std::endl;
    
#ifdef PE_USE_CGAL
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <reference_mesh.obj> <output_prefix> <test_sphere.obj>" << std::endl;
        std::cout << "  reference_mesh.obj: Primary mesh for DistanceMap generation (positioned at origin)" << std::endl;
        std::cout << "  output_prefix: Prefix for output files" << std::endl; 
        std::cout << "  test_sphere.obj: Sphere mesh to test against DistanceMap (positioned at test location)" << std::endl;
        std::cout << std::endl;
        std::cout << "Expected test scenario:" << std::endl;
        std::cout << "  - Reference mesh centered at (0,0,0)" << std::endl;
        std::cout << "  - Test sphere (radius ~1.0) positioned at (0.0, 3.1, 4.7)" << std::endl;
        std::cout << "  - Sphere should be inside bounding box but NOT colliding" << std::endl;
        std::cout << "  - All distance queries should return POSITIVE values" << std::endl;
        return 1;
    }

    std::string referenceMeshFile = argv[1];
    std::string outputPrefix = argv[2];
    std::string testSphereFile = argv[3];

    std::cout << "=== PE Environment Setup ===" << std::endl;
    
    // Initialize PE world
    WorldID world = theWorld();
    world->setGravity(0.0, 0.0, 0.0);  // No gravity for controlled testing
    std::cout << "PE World initialized with zero gravity" << std::endl;

    // Create materials
    MaterialID material1 = createMaterial("reference_mesh_mat", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
    MaterialID material2 = createMaterial("test_sphere_mat", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
    std::cout << "Materials created" << std::endl;

    std::cout << std::endl << "=== TriangleMesh Creation ===" << std::endl;
    
    try {
        // Mesh 1: Reference mesh at origin (0,0,0) - COM centered
        std::cout << "Loading reference mesh from: " << referenceMeshFile << std::endl;
        TriangleMeshID referenceMesh = createTriangleMesh(1, Vec3(0,0,0), referenceMeshFile, material1, false, true);
        std::cout << "Reference mesh created with " << referenceMesh->getBFVertices().size() << " vertices" << std::endl;
        std::cout << "Reference mesh positioned at: " << referenceMesh->getPosition() << std::endl;
        
        // Mesh 2: Test sphere at specific position - inside bounding box, no collision expected
        std::cout << "Loading test sphere from: " << testSphereFile << std::endl;
        TriangleMeshID testSphere = createTriangleMesh(2, Vec3(0.0, 3.1, 4.7), testSphereFile, material2, false, true);
        std::cout << "Test sphere created with " << testSphere->getBFVertices().size() << " vertices" << std::endl;
        std::cout << "Test sphere positioned at: " << testSphere->getPosition() << std::endl;

        std::cout << std::endl << "=== DistanceMap Acceleration ===" << std::endl;
        
        // Enable DistanceMap on reference mesh
        std::cout << "Enabling DistanceMap acceleration on reference mesh..." << std::endl;
        referenceMesh->enableDistanceMapAcceleration(0.1, 50, 5);  // spacing=0.1, resolution=50, tolerance=5
        
        // Verify DistanceMap creation
        std::cout << "Reference mesh has DistanceMap: " << (referenceMesh->hasDistanceMap() ? "YES" : "NO") << std::endl;
        std::cout << "Test sphere has DistanceMap: " << (testSphere->hasDistanceMap() ? "YES" : "NO") << std::endl;
        
        if (!referenceMesh->hasDistanceMap()) {
            std::cerr << "ERROR: Failed to create DistanceMap for reference mesh!" << std::endl;
            return 1;
        }

        std::cout << std::endl << "=== Direct Collision Detection Test ===" << std::endl;
        
        // Test MaxContacts::collideTMeshTMesh() directly
        typedef ContactVector<Contact, PtrDelete> Contacts;
        Contacts contacts;

        std::cout << "Testing direct collision detection via MaxContacts::collideTMeshTMesh()..." << std::endl;
        detection::fine::MaxContacts::collideTMeshTMesh(referenceMesh, testSphere, contacts);

        std::cout << "Number of contacts detected: " << contacts.size() << std::endl;

        // Log contact information
        if (contacts.size() > 0) {
            std::cout << "Contact details:" << std::endl;
            for (auto it = contacts.begin(); it != contacts.end(); ++it) {
                const auto& contact = *it;
                std::cout << "  Contact Point: " << contact->getPosition() << std::endl;
                std::cout << "  Contact Normal: " << contact->getNormal() << std::endl;
                std::cout << "  Penetration Depth: " << contact->getDistance() << std::endl;
                std::cout << "  ---" << std::endl;
            }
        } else {
            std::cout << "No contacts detected - as expected for this test scenario." << std::endl;
        }

        std::cout << std::endl << "=== DistanceMap Query Validation ===" << std::endl;
        
        int positiveCount = 0;
        int negativeCount = 0;
        // Direct DistanceMap queries for sphere vertices
        const DistanceMap* distMap = referenceMesh->getDistanceMap();
        if (distMap) {
            const auto& sphereVertices = testSphere->getWFVertices();
            
            std::cout << "Testing DistanceMap queries on sphere vertices:" << std::endl;
            std::cout << "Total sphere vertices: " << sphereVertices.size() << std::endl;
            
            real minDistance = std::numeric_limits<real>::max();
            real maxDistance = std::numeric_limits<real>::lowest();
            
            // Sample first 10 vertices for detailed output
            size_t sampleCount = std::min(size_t(10), sphereVertices.size());
            std::cout << "Detailed results for first " << sampleCount << " vertices:" << std::endl;
            
            for (size_t i = 0; i < sampleCount; ++i) {
                const Vec3& vertex = sphereVertices[i];
                real distance = distMap->interpolateDistance(vertex[0], vertex[1], vertex[2]);
                Vec3 normal = distMap->interpolateNormal(vertex[0], vertex[1], vertex[2]);
                
                std::cout << "  Vertex[" << i << "]: " << vertex 
                          << " -> Distance: " << distance 
                          << ", Normal: " << normal << std::endl;
                
                if (distance > 0) positiveCount++;
                else negativeCount++;
                
                minDistance = std::min(minDistance, distance);
                maxDistance = std::max(maxDistance, distance);
            }
            
            // Process all vertices for statistics
            for (size_t i = sampleCount; i < sphereVertices.size(); ++i) {
                const Vec3& vertex = sphereVertices[i];
                real distance = distMap->interpolateDistance(vertex[0], vertex[1], vertex[2]);
                
                if (distance > 0) {
                  positiveCount++;
                } 
                else {
                  negativeCount++;
                std::cout << "  Vertex[" << i << "]: " << vertex 
                          << " -> Distance: " << distance  << std::endl;
                } 
                
                minDistance = std::min(minDistance, distance);
                maxDistance = std::max(maxDistance, distance);
            }
            
            std::cout << "Summary statistics:" << std::endl;
            std::cout << "  Positive distances: " << positiveCount << " (" << (100.0 * positiveCount / sphereVertices.size()) << "%)" << std::endl;
            std::cout << "  Negative distances: " << negativeCount << " (" << (100.0 * negativeCount / sphereVertices.size()) << "%)" << std::endl;
            std::cout << "  Distance range: [" << minDistance << ", " << maxDistance << "]" << std::endl;
            
        } else {
            std::cerr << "ERROR: DistanceMap not available for queries!" << std::endl;
            return 1;
        }

        std::cout << std::endl << "=== Test Results Validation ===" << std::endl;
        
        // Expected behavior for sphere at (0.0, 3.1, 4.7):
        // - Should be inside reference mesh's bounding box
        // - Should NOT collide with reference mesh surface  
        // - All distance queries should return POSITIVE values
        // - No contacts should be detected

        bool noContactsDetected = (contacts.size() == 0);
        bool allDistancesPositive = (negativeCount == 0 && positiveCount > 0);
        bool testPassed = noContactsDetected && allDistancesPositive;
        
        std::cout << "Test validation:" << std::endl;
        std::cout << "  No contacts detected: " << (noContactsDetected ? "PASS" : "FAIL") << std::endl;
        std::cout << "  All distances positive: " << (allDistancesPositive ? "PASS" : "FAIL") << std::endl;
        std::cout << "  Overall test result: " << (testPassed ? "PASSED" : "FAILED") << std::endl;

        if (!testPassed) {
            std::cerr << "TEST FAILED: Unexpected collision behavior detected!" << std::endl;
            if (!noContactsDetected) {
                std::cerr << "  - Contacts were detected when none were expected" << std::endl;
            }
            if (!allDistancesPositive) {
                std::cerr << "  - Some distances were negative (indicating penetration)" << std::endl;
            }
            return 1;
        }

        std::cout << std::endl << "=== VTK Output Generation ===" << std::endl;
        
        std::cout << "Exporting visualization files with prefix: " << outputPrefix << std::endl;
        
        // For now, just log that VTK export would happen here
        // TODO: Implement VTK export for PE TriangleMesh objects and collision results
        std::cout << "VTK export placeholder - would export:" << std::endl;
        std::cout << "  - " << outputPrefix << "_reference_mesh.vtk (reference mesh with DistanceMap)" << std::endl;
        std::cout << "  - " << outputPrefix << "_test_sphere.vtk (test sphere with distance values)" << std::endl;
        std::cout << "  - " << outputPrefix << "_contacts.vtk (contact points if any)" << std::endl;

        std::cout << std::endl << "=== Test Completed Successfully ===" << std::endl;
        
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Exception occurred during test: " << e.what() << std::endl;
        return 1;
    }

#else
    std::cout << "This example requires CGAL support. Please rebuild with CGAL=ON" << std::endl;
#endif

    return 0;
}