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


    }

    
#else
    std::cout << "This example requires CGAL support. Please rebuild with CGAL=ON" << std::endl;
#endif

    return 0;
}