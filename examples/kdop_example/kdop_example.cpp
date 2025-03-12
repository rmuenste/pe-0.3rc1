//=================================================================================================
/*!
 *  \file kdop_example.cpp
 *  \brief Example demonstrating the TriMeshDopBoundary class and k-DOP extraction
 *
 *  This example creates a TriMeshDopBoundary instance and prints out the planes of the k-DOP.
 */
//=================================================================================================

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/engine.h>
#include <pe/support.h>
#include <pe/core/domaindecomp/TriMeshDopBoundary.h>
#include <iostream>
#include <iomanip>

using namespace pe;

//*************************************************************************************************
/*!\brief Print the half-spaces of a k-DOP boundary.
 *
 * \param halfspaces List of half-spaces defining the k-DOP.
 * \return void
 */
void printHalfSpaces(const std::list<std::pair<Vec3, real>>& halfspaces)
{
    int index = 0;
    
    // Print a header for the half-spaces
    std::cout << "K-DOP contains " << halfspaces.size() << " half-spaces:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Index | Normal Vector        | Distance" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    // Print each half-space
    for (const auto& halfspace : halfspaces) {
        const Vec3& normal = halfspace.first;
        real distance = halfspace.second;
        
        std::cout << std::setw(5) << index++ << " | "
                  << std::setw(6) << std::fixed << std::setprecision(3) << normal[0] << ", "
                  << std::setw(6) << std::fixed << std::setprecision(3) << normal[1] << ", "
                  << std::setw(6) << std::fixed << std::setprecision(3) << normal[2] << " | "
                  << std::setw(8) << std::fixed << std::setprecision(4) << distance
                  << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
}

//*************************************************************************************************
/*!\brief Print information about a triangle mesh.
 *
 * \param objFile Name of the OBJ file.
 * \param mesh TriMeshDopBoundary instance.
 * \return void
 */
void printMeshInfo(const std::string& objFile, const TriMeshDopBoundary& mesh)
{
    std::cout << "\nMesh from file: " << objFile << std::endl;
    std::cout << "Centroid: " << mesh.getCentroid() << std::endl;
    
    // Get AABB info
    const auto& aabb = mesh.getAABB();
    std::cout << "AABB: [" << aabb[0] << ", " << aabb[1] << ", " << aabb[2] << "] - ["
              << aabb[3] << ", " << aabb[4] << ", " << aabb[5] << "]" << std::endl;
              
    // Extract and print half-spaces
    std::list<std::pair<Vec3, real>> halfspaces;
    mesh.extractHalfSpaces(halfspaces);
    printHalfSpaces(halfspaces);
    
    std::cout << std::endl;
}

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the kdop_example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success status.
 *
 * This example demonstrates the use of the TriMeshDopBoundary class by loading OBJ files,
 * creating k-DOP boundaries, and printing out the planes of the k-DOP.
 */
int main(int argc, char** argv)
{
    // Set up material for mesh creation
    MaterialID material = createMaterial("material", 1.0, 0.8, 0.1, 0.05, 0.3, 200, 1e3, 1e2, 2e2);
    
    std::cout << "==================================================" << std::endl;
    std::cout << " TriMeshDopBoundary Example - K-DOP Visualization " << std::endl;
    std::cout << "==================================================" << std::endl;

    // Create a TriMeshDopBoundary from a cube OBJ file
    std::string cubeFile = "sub.obj";
    try {
        // Method 1: Direct construction from OBJ file
        TriMeshDopBoundary cubeMesh(cubeFile, false, false);
        printMeshInfo(cubeFile, cubeMesh);
        
        // Method 2: Construction with parameters (transformed cube)
        pe::id_t uid = 1;
        Vec3 position(1.0, 0.0, 0.0);  // Shifted cube
        Vec3 scale(2.0, 1.0, 1.0);     // Scaled cube
        bool visible = true;
        bool convex = true;
        
        TriMeshDopBoundary transformedCubeMesh(
            uid, position, cubeFile, material, convex, visible, scale, false, false
        );
        printMeshInfo(cubeFile + " (transformed)", transformedCubeMesh);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with cube mesh: " << e.what() << std::endl;
    }
    
    // Create a TriMeshDopBoundary from a more complex OBJ file
//    std::string dodecahedronFile = "dodecahedron.obj";
//    try {
//        TriMeshDopBoundary dodecahedronMesh(dodecahedronFile, false, false);
//        printMeshInfo(dodecahedronFile, dodecahedronMesh);
//    }
//    catch (const std::exception& e) {
//        std::cerr << "Error with dodecahedron mesh: " << e.what() << std::endl;
//    }
    
    return 0;
}