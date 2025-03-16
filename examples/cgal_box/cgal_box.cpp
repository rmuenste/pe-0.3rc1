//******************************************************************************************
//
// This example demonstrates how to use CGAL's oriented_bounding_box functionality 
// to create optimal oriented bounding boxes (OBB) for a set of points.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>

// PE headers
#include <pe/core.h>
#include <pe/math.h>

// CGAL headers
#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/OFF.h>
#endif

// Define kernel
#ifdef PE_USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector_3; //typedef for Vector type
typedef CGAL::Surface_mesh<Point> Surface_mesh;
#endif

using namespace pe;
// Function to export the original mesh to OBJ file
void write_mesh_to_obj(const Surface_mesh& mesh, const std::string& filename) {
    std::ofstream obj_file(filename);
    if (!obj_file) {
        std::cerr << "Error: Cannot create file " << filename << std::endl;
        return;
    }
    
    // Create vertex map
    std::unordered_map<Surface_mesh::Vertex_index, int> vertex_to_id;
    int vertex_id = 1; // OBJ indices start at 1
    
    // Write vertices
    for (auto v : mesh.vertices()) {
        const Point& p = mesh.point(v);
        obj_file << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        vertex_to_id[v] = vertex_id++;
    }
    
    // Write faces
    for (auto f : mesh.faces()) {
        obj_file << "f";
        for (auto v : mesh.vertices_around_face(mesh.halfedge(f))) {
            obj_file << " " << vertex_to_id[v];
        }
        obj_file << std::endl;
    }
    
    obj_file.close();
    std::cout << "Original mesh written to " << filename << std::endl;
}

// Function to write OBB to OBJ file
void write_obb_to_obj(const std::array<Point, 8>& obb_points, const std::string& filename) {
    std::ofstream obj_file(filename);
    if (!obj_file) {
        std::cerr << "Error: Cannot create file " << filename << std::endl;
        return;
    }
    
    // Write vertices - OBJ indices start at 1
    for (int i = 0; i < 8; ++i) {
        obj_file << "v " << obb_points[i].x() << " " 
                         << obb_points[i].y() << " " 
                         << obb_points[i].z() << std::endl;
    }
    
    // Write faces (triangulating the box)
    // Bottom face (triangulated as 0-1-2 and 0-2-3)
    obj_file << "f 1 2 3" << std::endl;
    obj_file << "f 1 3 4" << std::endl;
    
    // Top face (triangulated as 4-5-6 and 4-6-7)
    obj_file << "f 5 6 7" << std::endl;
    obj_file << "f 5 7 8" << std::endl;
    
    // Side faces
    // Face 0-1-5-4 (triangulated)
    obj_file << "f 1 2 6" << std::endl;
    obj_file << "f 1 6 5" << std::endl;
    
    // Face 1-2-6-5 (triangulated)
    obj_file << "f 2 3 7" << std::endl;
    obj_file << "f 2 7 6" << std::endl;
    
    // Face 2-3-7-6 (triangulated)
    obj_file << "f 3 4 8" << std::endl;
    obj_file << "f 3 8 7" << std::endl;
    
    // Face 3-0-4-7 (triangulated)
    obj_file << "f 4 1 5" << std::endl;
    obj_file << "f 4 5 8" << std::endl;
    
    obj_file.close();
    std::cout << "OBB written to " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "CGAL Oriented Bounding Box Example" << std::endl;
    
#ifdef PE_USE_CGAL
    std::string meshFile;
    Surface_mesh fromFile;
    if (argc >= 2) {
      meshFile = argv[1];

      std::string output_prefix = (argc > 2) ? argv[2] : "output";
      std::ifstream input(meshFile);

      if (!input || !CGAL::IO::read_OFF(input, fromFile)) {
        std::cerr << "Error: Cannot read file " << meshFile << "\n";
        return 1;
      }

      std::vector<Point> pointsFromMesh;
      for(auto v : fromFile.vertices()) {
        pointsFromMesh.push_back(fromFile.point(v));
      }

      // Calculate the oriented bounding box
      std::array<Point, 8> obb_mesh;
      CGAL::oriented_bounding_box(fromFile, obb_mesh, 
                               CGAL::parameters::use_convex_hull(true));

      // Output the OBB points
      std::cout << "Oriented Bounding Box vertices:" << std::endl;
      for (int i = 0; i < 8; ++i) {
          std::cout << "  " << i << ": (" 
                    << obb_mesh[i].x() << ", " 
                    << obb_mesh[i].y() << ", " 
                    << obb_mesh[i].z() << ")" << std::endl;                               
      }

      // Calculate and print the OBB dimensions
      // We need to find the three edge vectors from point 0
      Vector_3 v01(obb_mesh[0], obb_mesh[1]);
      Vector_3 v03(obb_mesh[0], obb_mesh[3]);
      Vector_3 v04(obb_mesh[0], obb_mesh[4]);
      
      double length = std::sqrt(v01.squared_length());
      double width = std::sqrt(v03.squared_length());
      double height = std::sqrt(v04.squared_length());
      
      std::cout << "OBB Dimensions: " << std::endl;
      std::cout << "  Length: " << length << std::endl;
      std::cout << "  Width: " << width << std::endl;
      std::cout << "  Height: " << height << std::endl;
      
      // Calculate volume
      double volume = length * width * height;
      std::cout << "  Volume: " << volume << std::endl;
      
      // Write the OBB to an OBJ file
      std::string obb_filename = output_prefix + "_obb.obj";
      write_obb_to_obj(obb_mesh, obb_filename);
      
      // Write the original mesh to an OBJ file for comparison
      std::string mesh_filename = output_prefix + "_mesh.obj";
      write_mesh_to_obj(fromFile, mesh_filename);

    }

    // Create a set of points (e.g., a simple shape)
    std::vector<Point> points;
    
    // Add a set of points that form a rotated shape
    const double rotation_angle = 30.0 * (M_PI / 180.0); // 30 degrees in radians
    const double cos_theta = std::cos(rotation_angle);
    const double sin_theta = std::sin(rotation_angle);
    
    // Create a box shape with rotation
    for (int x = -2; x <= 2; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                // Apply rotation around Z-axis
                double rx = x * cos_theta - y * sin_theta;
                double ry = x * sin_theta + y * cos_theta;
                
                points.push_back(Point(rx, ry, z));
            }
        }
    }
    
    std::cout << "Created " << points.size() << " points." << std::endl;
    
    // Create a surface mesh from the points
    Surface_mesh sm;
    
    // First, we need to triangulate the points to create a mesh
    std::vector<std::vector<std::size_t>> polygons;
    
    // Use convex hull to create a simple mesh
    // (In a real application, you might have a proper mesh already)
    std::vector<Point> box;
    std::array<Point, 8> obb_points;
    
    // Calculate the oriented bounding box
    CGAL::oriented_bounding_box(points, obb_points, 
                               CGAL::parameters::use_convex_hull(true));
    
    std::cout << "Oriented bounding box corners:" << std::endl;
    for (const auto& p : obb_points) {
        std::cout << "  (" << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;
    }
    
    // Extract OBB dimensions and orientation
    // Calculate the edges of the OBB
    Vector_3 edge1(obb_points[1].x() - obb_points[0].x(),
                obb_points[1].y() - obb_points[0].y(),
                obb_points[1].z() - obb_points[0].z());
                
    Vector_3 edge2(obb_points[3].x() - obb_points[0].x(),
                obb_points[3].y() - obb_points[0].y(),
                obb_points[3].z() - obb_points[0].z());
                
    Vector_3 edge3(obb_points[4].x() - obb_points[0].x(),
                obb_points[4].y() - obb_points[0].y(),
                obb_points[4].z() - obb_points[0].z());
    
    // Calculate dimensions
    double length = std::sqrt(edge1.squared_length());
    double width = std::sqrt(edge2.squared_length());
    double height = std::sqrt(edge3.squared_length());
    
    std::cout << "OBB dimensions:" << std::endl;
    std::cout << "  Length: " << length << std::endl;
    std::cout << "  Width: " << width << std::endl;
    std::cout << "  Height: " << height << std::endl;
    
    // Calculate center of the OBB
    double center_x = 0, center_y = 0, center_z = 0;
    for (const auto& p : obb_points) {
        center_x += p.x();
        center_y += p.y();
        center_z += p.z();
    }
    center_x /= obb_points.size();
    center_y /= obb_points.size();
    center_z /= obb_points.size();
    
    std::cout << "OBB center: (" << center_x << ", " << center_y << ", " << center_z << ")" << std::endl;
    
    // Normalize the edge vectors to get the orientation
    edge1 = edge1 / length;
    edge2 = edge2 / width;
    edge3 = edge3 / height;
    
    std::cout << "OBB orientation vectors:" << std::endl;
    std::cout << "  X: (" << edge1.x() << ", " << edge1.y() << ", " << edge1.z() << ")" << std::endl;
    std::cout << "  Y: (" << edge2.x() << ", " << edge2.y() << ", " << edge2.z() << ")" << std::endl;
    std::cout << "  Z: (" << edge3.x() << ", " << edge3.y() << ", " << edge3.z() << ")" << std::endl;
    
    // Convert to PE's data structures if needed
    // Here you might create a PE box with the center and orientation
    
    // Write OBB points to a file for visualization
    std::ofstream outfile("obb_result.obj");
    if (outfile.is_open()) {
        // Write vertices
        for (const auto& p : obb_points) {
            outfile << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        }
        
        // Write faces (for a box)
        outfile << "f 1 2 3 4" << std::endl;  // bottom face
        outfile << "f 5 6 7 8" << std::endl;  // top face
        outfile << "f 1 2 6 5" << std::endl;  // side
        outfile << "f 2 3 7 6" << std::endl;  // side
        outfile << "f 3 4 8 7" << std::endl;  // side
        outfile << "f 4 1 5 8" << std::endl;  // side
        
        outfile.close();
        std::cout << "Wrote OBB to obb_result.obj" << std::endl;
    }
    
    // Write original points to a file for visualization
    std::ofstream pointfile("points.obj");
    if (pointfile.is_open()) {
        for (const auto& p : points) {
            pointfile << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        }
        pointfile.close();
        std::cout << "Wrote original points to points.obj" << std::endl;
    }
    
#else
    std::cout << "This example requires CGAL support. Please rebuild with CGAL=ON" << std::endl;
#endif

    return 0;
}