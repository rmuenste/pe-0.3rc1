#include <iostream>
#include <fstream>
#include <vector>
#include <typeinfo>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/squared_distance_3.h> 
#include <CGAL/number_utils.h>
#include <cstddef>


namespace cgal3d {

#ifndef BOOST_NO_INT64_T
  typedef uint64_t my_t;
#else
  typedef uint32_t my_t;
#endif


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<int, K> Vb;
typedef CGAL::Delaunay_triangulation_cell_base_3<K>         Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb>        Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>              Delaunay;
typedef Delaunay::Point_3                                   Point;


// Function to write Delaunay triangulation to VTK file
void writeDelaunayToVTK(const std::vector<std::vector<double>>& points,
                        const std::vector<std::vector<int>>& edges,
                        const std::string& filename);

int computeDelaunayTriangulation(std::vector<std::vector<double>> &points, Delaunay &dt, std::vector<Point> &cgalPoints);

//=================================================================================================
// outputDataToFile
//=================================================================================================
// Function to positions, distances and wall collisions to an exchangefile
void outputDataToFile(const std::vector<double>& all_points_x,
                      const std::vector<double>& all_points_y,
                      const std::vector<double>& all_points_z,
                      const std::vector<my_t>& allSystemIDs,
                      const std::vector<int>& globalWallContacts,
                      const std::vector<double>& globalWallDistances,
                      std::vector<std::vector<double>> &points,
                      real L,
                      real phi,
                      real radius,
                      std::string fileName,
                      std::string vtkFileName
                      ) {
    
    // Open the file
    std::ofstream outputFile(fileName);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return;
    }

    Delaunay dt;
    std::vector<Point> cgalPoints;
    computeDelaunayTriangulation(points, dt, cgalPoints); 

    outputFile << "CELL_SIZE 1" << std::endl;
    outputFile << L << std::endl;
    outputFile << "RADIUS 1" << std::endl;
    outputFile << radius << std::endl;
    outputFile << "VOLUME_FRACTION 1" << std::endl;
    outputFile << phi << std::endl;

    outputFile << "POINTS " << all_points_x.size() << std::endl;
    // Output all_points_x, all_points_y, all_points_z
    for (size_t i = 0; i < all_points_x.size(); ++i) {
        outputFile << all_points_x[i] << " " << all_points_y[i] << " " << all_points_z[i] << std::endl;
    }

    outputFile << "WALLINFO " << globalWallContacts.size() << std::endl;
    // Output globalWallContacts and globalWallDistances
    for (size_t i = 0; i < globalWallContacts.size(); ++i) {
        outputFile << globalWallContacts[i] << " " << globalWallDistances[i] << std::endl;
    }

    // Extract connectivity of tetrahedra
    std::vector<std::vector<int>> tetrahedraConnectivity;
    for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
        std::vector<int> tetrahedronVertices;
        for (int i = 0; i < 4; ++i) { // 4 vertices for each tetrahedron
            tetrahedronVertices.push_back(cell->vertex(i)->info());
        }
        tetrahedraConnectivity.push_back(tetrahedronVertices);
    }

    // Extract connectivity of facets (triangles)
    std::vector<std::vector<int>> facetsConnectivity;
    for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
        for (int i = 0; i < 4; ++i) {
            Delaunay::Cell_handle neighbor = cell->neighbor(i);
            if (dt.is_infinite(neighbor)) {
                // Infinite neighbor represents a facet
                std::vector<int> facetVertices;
                for (int j = 0; j < 3; ++j) {
                    facetVertices.push_back(cell->vertex((i + j) % 4)->info());
                }
                facetsConnectivity.push_back(facetVertices);
            }
        }
    }

    // Extract connectivity of edges
    std::vector<std::vector<int>> edgesConnectivity;
    int count(0);
    for (auto edge = dt.finite_edges_begin(); edge != dt.finite_edges_end(); ++edge) {
        std::vector<int> edgeVertices;
        edgeVertices.push_back(edge->first->vertex((edge->second) % 4)->info());
        edgeVertices.push_back(edge->first->vertex((edge->third) % 4)->info());
        edgesConnectivity.push_back(edgeVertices);
        count++;
    }

    // Print tetrahedra connectivity
    outputFile << "TETRAS " << tetrahedraConnectivity.size() << "\n";
    for (const auto& tetrahedron : tetrahedraConnectivity) {
        for (int i : tetrahedron) {
            outputFile << i << " ";
        }
        outputFile << "\n";
    }

    // Print facets connectivity
    outputFile << "FACETS " << facetsConnectivity.size() << "\n";
    for (const auto& facet : facetsConnectivity) {
        for (int i : facet) {
            outputFile << i << " ";
        }
        outputFile << "\n";
    }

    // Print edges connectivity
    outputFile << "EDGES " << edgesConnectivity.size() << "\n";
    for (const auto& edge : edgesConnectivity) {
        double sq_dist = CGAL::squared_distance(cgalPoints[edge[0]], cgalPoints[edge[1]]);
        outputFile << edge[0] << " " << edge[1] << " " << CGAL::sqrt(sq_dist) - 2. * radius;
        outputFile << "\n";
    }
    
    // Close the file
    outputFile.close();

    writeDelaunayToVTK(points, edgesConnectivity, vtkFileName);

}
//=================================================================================================


std::vector<std::vector<double>> extractPointsFromFile(const std::string& filename) {
    std::vector<std::vector<double>> points;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return points; // Return an empty vector in case of error
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("POINTS") != std::string::npos) {
            std::istringstream iss(line);   
            std::string keyword;
            int numPoints;
            iss >> keyword >> numPoints;
            for (int i = 0; i < numPoints; ++i) {
                double x, y, z;
                std::getline(file, line);
                std::istringstream ps(line);   
                if (ps >> x >> y >> z) {
                    points.push_back({x, y, z});
                } else {
                    std::cerr << "Error: Invalid data format after POINTS keyword." << std::endl;
                    points.clear(); // Clear the vector in case of error
                    break;
                }
            }
        }
    }
    
    file.close();
    return points;
}

// Function to read points from a file
std::vector<std::vector<double>> readPointsFromFile(const std::string& filename) {
    std::vector<std::vector<double>> points;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }

    double x, y, z;
    while (file >> x >> y >> z) {
        points.push_back({x, y, z});
    }

    file.close();
    return points;
}

// Function to write Delaunay triangulation to VTK file
void writeDelaunayToVTK(const std::vector<std::vector<double>>& points,
                        const std::vector<std::vector<int>>& edges,
                        const std::string& filename) {

    std::ofstream vtk_file(filename);

    if (!vtk_file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    vtk_file << "# vtk DataFile Version 3.0\n";
    vtk_file << "Delaunay Triangulation\n";
    vtk_file << "ASCII\n";
    vtk_file << "DATASET POLYDATA\n";
    vtk_file << "POINTS " << points.size() << " float\n";

    for (const auto& point : points) {
        vtk_file << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    vtk_file << "LINES " << edges.size() << " " << edges.size() * 3 << "\n";

    for (const auto& edge : edges) {
        vtk_file << "2 " << edge[0] << " " << edge[1] << "\n";
    }

    vtk_file.close();
}

int computeDelaunayTriangulation(std::vector<std::vector<double>> &points, Delaunay &dt, std::vector<Point> &cgalPoints) {

//  Sample Points
//==================================================================================
//    // Extract points
//    std::vector<std::vector<double>> points;// = readPointsFromFile("input.dat");
//    for (auto vertex = dt.finite_vertices_begin(); vertex != dt.finite_vertices_end(); ++vertex) {
//        points.push_back({vertex->point().x(), vertex->point().y(), vertex->point().z()});
//        cgalPoints.push_back(vertex->point());
//    }
//==================================================================================

    // Read points from file
    //std::vector<std::vector<double>> points = extractPointsFromFile("input.dat");

    // Insert points into the triangulation with vertex info
    for (const auto& point : points) {
        Delaunay::Vertex_handle v = dt.insert(Delaunay::Point(point[0], point[1], point[2]));
        v->info() = dt.number_of_vertices() - 1; // Assigning info to the last inserted vertex
        cgalPoints.push_back(Delaunay::Point(point[0], point[1], point[2]));
    }

    return 0;
}

}

