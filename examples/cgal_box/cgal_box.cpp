//******************************************************************************************
//
// This example demonstrates SDF-based collision detection using CGAL.
// It creates a DistanceMap from one mesh and tests collision with another mesh.
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

// CGAL headers
#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/OFF.h>
#endif

using namespace pe;

// CGAL types for this example
#ifdef PE_USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
#endif

    

int main(int argc, char* argv[]) {
    std::cout << "CGAL SDF Collision Detection Example using DistanceMap!" << std::endl;
    
#ifdef PE_USE_CGAL
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <mesh1.off> <output_prefix> <mesh2.off>" << std::endl;
        std::cout << "  mesh1.off: Primary mesh for SDF generation" << std::endl;
        std::cout << "  output_prefix: Prefix for output files" << std::endl; 
        std::cout << "  mesh2.off: Secondary mesh to test against SDF" << std::endl;
        return 1;
    }

    std::string meshFile = argv[1];
    std::string output_prefix = argv[2];
    std::string chipFile = argv[3];

    // Create DistanceMap from primary mesh using appropriate loading method
    const double grid_spacing = 0.1;  // Adjust as needed
    const int resolution = 50;        // Grid resolution
    std::unique_ptr<DistanceMap> distance_map;
    
    // Detect file format and choose loading path
    std::string meshExt = meshFile.substr(meshFile.find_last_of('.'));
    std::transform(meshExt.begin(), meshExt.end(), meshExt.begin(), ::tolower);
    
    if (meshExt == ".obj") {
        std::cout << "OBJ file detected - using PE TriangleMesh loader (with COM centering)" << std::endl;
        
        // Use PE path: robust OBJ loading with automatic COM centering
        try {
            // Create dummy material (not used for SDF computation)
           MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
            auto material = pe::createMaterial("mesh_material", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
            
            // Load mesh using PE - automatically centers vertices around COM
            auto pe_mesh = pe::createTriangleMesh(1, pe::Vec3(0,0,0), meshFile, material, false, true);
            
            std::cout << "PE mesh loaded successfully" << std::endl;
            
            // Convert to DistanceMap
            distance_map = DistanceMap::create(*pe_mesh, grid_spacing, resolution);
        }
        catch (const std::exception& e) {
            std::cerr << "Error loading OBJ file with PE: " << e.what() << std::endl;
            return 1;
        }
    }
    else if (meshExt == ".off") {
        std::cout << "OFF file detected - using direct CGAL loader" << std::endl;
        
        // Use direct CGAL path for OFF files
        Surface_mesh primaryMesh;
        std::ifstream input1(meshFile);
        if (!input1 || !CGAL::IO::read_OFF(input1, primaryMesh)) {
            std::cerr << "Error: Cannot read OFF file " << meshFile << std::endl;
            return 1;
        }
        
        std::cout << "Loaded primary mesh with " << num_vertices(primaryMesh)
                  << " vertices and " << num_faces(primaryMesh) << " faces." << std::endl;
        
        // Create DistanceMap from CGAL mesh
        distance_map = DistanceMap::create(primaryMesh, grid_spacing, resolution);
    }
    else {
        std::cerr << "Error: Unsupported file format '" << meshExt << "'. Supported: .obj, .off" << std::endl;
        return 1;
    }
    
    if (!distance_map) {
        std::cerr << "Error: Failed to create DistanceMap" << std::endl;
        return 1;
    }

    // Export SDF grid to VTI file for visualization
    std::cout << "Exporting SDF grid to VTI..." << std::endl;
    
    // Convert contact points from double to float for VTI export
    const auto& contact_points_double = distance_map->getContactPointData();
    std::vector<std::array<float, 3>> contact_points_float(contact_points_double.size());
    for (size_t i = 0; i < contact_points_double.size(); ++i) {
        contact_points_float[i] = {
            static_cast<float>(contact_points_double[i][0]),
            static_cast<float>(contact_points_double[i][1]),
            static_cast<float>(contact_points_double[i][2])
        };
    }
    
    // Create dummy face index for VTI export (not used in DistanceMap)
    std::vector<int> face_index(distance_map->getSdfData().size(), 0);
    
    auto origin = distance_map->getOrigin();
    write_vti("sdf.vti",
              distance_map->getSdfData(),
              distance_map->getAlphaData(), 
              distance_map->getNormalData(),
              contact_points_float,
              face_index,
              distance_map->getNx(), distance_map->getNy(), distance_map->getNz(),
              distance_map->getSpacing(), distance_map->getSpacing(), distance_map->getSpacing(),
              origin[0], origin[1], origin[2]);

    // Load secondary mesh for testing using appropriate loading method  
    Surface_mesh secondaryMesh;
    std::string chipExt = chipFile.substr(chipFile.find_last_of('.'));
    std::transform(chipExt.begin(), chipExt.end(), chipExt.begin(), ::tolower);
    
    if (chipExt == ".obj") {
        std::cout << "Loading secondary OBJ mesh with PE..." << std::endl;
        
        try {
            // Create dummy material for secondary mesh
            auto material2 = pe::createMaterial("chip_material", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
            
            // Load secondary mesh using PE
            auto pe_chip_mesh = pe::createTriangleMesh(2, pe::Vec3(0,0,0), chipFile, material2, false, true);
            
            // Convert PE mesh to CGAL mesh for vertex iteration
            const auto& pe_vertices = pe_chip_mesh->getBFVertices();
            const auto& pe_faces = pe_chip_mesh->getFaceIndices();
            
            // Add vertices to CGAL mesh
            std::vector<Surface_mesh::Vertex_index> vertex_map(pe_vertices.size());
            for (size_t i = 0; i < pe_vertices.size(); ++i) {
                const auto& v = pe_vertices[i];
                Point p(v[0], v[1], v[2]);
                vertex_map[i] = secondaryMesh.add_vertex(p);
            }
            
            // Add faces to CGAL mesh
            for (size_t i = 0; i < pe_faces.size(); ++i) {
                const auto& face = pe_faces[i];
                if (face.size() == 3) {
                    secondaryMesh.add_face(vertex_map[face[0]], 
                                         vertex_map[face[1]], 
                                         vertex_map[face[2]]);
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error loading secondary OBJ file: " << e.what() << std::endl;
            return 1;
        }
    }
    else if (chipExt == ".off") {
        std::cout << "Loading secondary OFF mesh with CGAL..." << std::endl;
        
        std::ifstream input2(chipFile);
        if (!input2 || !CGAL::IO::read_OFF(input2, secondaryMesh)) {
            std::cerr << "Error: Cannot read OFF file " << chipFile << std::endl;
            return 1;
        }
    }
    else {
        std::cerr << "Error: Unsupported secondary file format '" << chipExt << "'. Supported: .obj, .off" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded secondary mesh with " << num_vertices(secondaryMesh)
              << " vertices and " << num_faces(secondaryMesh) << " faces." << std::endl;

    // Test secondary mesh vertices against SDF
    std::cout << "Testing secondary mesh against SDF..." << std::endl;
    
    std::vector<double> chipAlpha;
    std::vector<double> chipSDF;
    std::vector<std::array<double, 3>> chipNormal;
    std::vector<std::array<double, 3>> chipContactPoint;
    
    chipAlpha.reserve(num_vertices(secondaryMesh));
    chipSDF.reserve(num_vertices(secondaryMesh));
    chipNormal.reserve(num_vertices(secondaryMesh));
    chipContactPoint.reserve(num_vertices(secondaryMesh));

    int collision_count = 0;
    for (auto v : secondaryMesh.vertices()) {
        const Point& p = secondaryMesh.point(v);
        double x = CGAL::to_double(p.x());
        double y = CGAL::to_double(p.y());
        double z = CGAL::to_double(p.z());
        
        // Use DistanceMap interpolation methods
        double alpha = distance_map->interpolateAlpha(x, y, z);
        double sdf = distance_map->interpolateDistance(x, y, z);
        auto normal = distance_map->interpolateNormal(x, y, z);
        auto contact_point = distance_map->interpolateContactPoint(x, y, z);
        
        chipAlpha.push_back(alpha);
        chipSDF.push_back(sdf);
        chipNormal.push_back(normal);
        chipContactPoint.push_back(contact_point);
        
        // Count potential collisions (negative SDF means inside primary mesh)
        if (sdf < 0) {
            collision_count++;
        }
    }

    std::cout << "Secondary mesh collision analysis:" << std::endl;
    std::cout << "  Total vertices: " << num_vertices(secondaryMesh) << std::endl;
    std::cout << "  Vertices inside primary mesh: " << collision_count << std::endl;
    std::cout << "  Collision percentage: " << (100.0 * collision_count / num_vertices(secondaryMesh)) << "%" << std::endl;

    // Export results to VTK
    std::cout << "Exporting results to VTK..." << std::endl;
    write_vtk(secondaryMesh, chipAlpha, chipSDF, chipNormal, chipContactPoint, "collision_test.vtk");

    std::cout << "Done! Files generated:" << std::endl;
    std::cout << "  sdf.vti - SDF grid visualization" << std::endl;
    std::cout << "  collision_test.vtk - Secondary mesh with SDF data" << std::endl;

#else
    std::cout << "This example requires CGAL support. Please rebuild with CGAL=ON" << std::endl;
#endif

    return 0;
}






