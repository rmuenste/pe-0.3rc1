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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<int, K> Vb;
typedef CGAL::Delaunay_triangulation_cell_base_3<K>         Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb>        Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>              Delaunay;
typedef Delaunay::Point_3                                   Point;

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

int computeDelaunayTriangulation() {

    Delaunay dt;

    double radius = 0.01;

    std::vector<Point> cgalPoints;

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
    std::vector<std::vector<double>> points = readPointsFromFile("input.dat");

    // Insert points into the triangulation with vertex info
    for (const auto& point : points) {
        Delaunay::Vertex_handle v = dt.insert(Delaunay::Point(point[0], point[1], point[2]));
        v->info() = dt.number_of_vertices() - 1; // Assigning info to the last inserted vertex
        cgalPoints.push_back(Delaunay::Point(point[0], point[1], point[2]));
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

    std::string fileName("same_file.dat");
    std::ofstream outputFile(fileName);

    if(!outputFile.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return EXIT_FAILURE;
    }

    // Print points
    std::cout << "POINTS " << points.size() << std::endl;
    outputFile << "POINTS " << points.size() << "\n";
    for (const auto& point : points) {
        std::cout << point[0] << " " << point[1] << " " << point[2] << std::endl;
        outputFile << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    // Print tetrahedra connectivity
    std::cout << "TETRAS " << tetrahedraConnectivity.size() << std::endl;
    outputFile << "TETRAS " << tetrahedraConnectivity.size() << "\n";
    for (const auto& tetrahedron : tetrahedraConnectivity) {
        for (int i : tetrahedron) {
            std::cout << i << " ";
            outputFile << i << " ";
        }
        std::cout << std::endl;
        outputFile << "\n";
    }

    // Print facets connectivity
    std::cout << "FACETS " << facetsConnectivity.size() << std::endl;
    outputFile << "FACETS " << facetsConnectivity.size() << "\n";
    for (const auto& facet : facetsConnectivity) {
        for (int i : facet) {
            std::cout << i << " ";
            outputFile << i << " ";
        }
        std::cout << std::endl;
        outputFile << "\n";
    }

    // Print edges connectivity
    std::cout << "EDGES " << edgesConnectivity.size() << std::endl;
    outputFile << "EDGES " << edgesConnectivity.size() << "\n";
    for (const auto& edge : edgesConnectivity) {
        double sq_dist = CGAL::squared_distance(cgalPoints[edge[0]], cgalPoints[edge[1]]);
        std::cout << edge[0] << " " << edge[1] << " " << CGAL::sqrt(sq_dist) - 2. * radius;
        outputFile << edge[0] << " " << edge[1] << " " << CGAL::sqrt(sq_dist) - 2. * radius;
        std::cout << std::endl;
        outputFile << "\n";
    }
    outputFile.close();

    writeDelaunayToVTK(points, edgesConnectivity, "delaunay.vtk");
    return 0;
}