#include "VtkOutput.h"
#include <iostream>
#include <fstream>
#include <iomanip>

#ifdef PE_USE_CGAL
#include <CGAL/boost/graph/helpers.h>
#endif

void write_vti(const std::string& filename,
               const std::vector<double>& sdf,
               const std::vector<int>& alpha,
               const std::vector<std::array<double, 3>>& normals,
               const std::vector<std::array<float, 3>>& contact_points,
               const std::vector<int>& face_index,
               int nx, int ny, int nz,
               double dx, double dy, double dz,
               double x0, double y0, double z0)
{
    std::ofstream out(filename);
    out << std::fixed << std::setprecision(6);

    out << R"(<?xml version="1.0"?>)" << "\n";
    out << R"(<VTKFile type="ImageData" version="0.1" byte_order="LittleEndian">)" << "\n";
    out << "  <ImageData Origin=\"" << x0 << " " << y0 << " " << z0 << "\" "
        << "Spacing=\"" << dx << " " << dy << " " << dz << "\" "
        << "WholeExtent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";

    out << "    <Piece Extent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";

    // Point data (SDF + alpha)
    out << "      <PointData Scalars=\"sdf\">\n";

    out << "        <DataArray type=\"Float32\" Name=\"sdf\" format=\"ascii\">\n";
    for (double v : sdf) out << v << " ";
    out << "\n        </DataArray>\n";

    out << "        <DataArray type=\"Int32\" Name=\"alpha\" format=\"ascii\">\n";
    for (int a : alpha) out << a << " ";
    out << "\n        </DataArray>\n";

    out << "        <DataArray type=\"Float32\" Name=\"normal\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int z = 0; z < nz; ++z){
        for (int y = 0; y < ny; ++y){
            for (int x = 0; x < nx; ++x) {
                int idx = x + y * nx + z * nx * ny;
                const auto& n = normals[idx];
                out << n[0] << " " << n[1] << " " << n[2] << " \n";
            }
        }   
    }
    out << "\n        </DataArray>\n";

    out << "        <DataArray type=\"Float32\" Name=\"contact_point\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int z = 0; z < nz; ++z){
        for (int y = 0; y < ny; ++y){
            for (int x = 0; x < nx; ++x) {
                int idx = x + y * nx + z * nx * ny;
                const auto& c = contact_points[idx];
                out << c[0] << " " << c[1] << " " << c[2] << " \n";
            }
        }   
    }
    out << "\n        </DataArray>\n";

    out << "        <DataArray type=\"Int32\" Name=\"face\" format=\"ascii\">\n";
    for (int f : face_index) out << f << " ";
    out << "\n        </DataArray>\n";

    out << "      </PointData>\n";
    out << "      <CellData/>\n";  // no cell data
    out << "    </Piece>\n";
    out << "  </ImageData>\n";
    out << "</VTKFile>\n";
}

#ifdef PE_USE_CGAL
void write_vtk(const Surface_mesh& mesh,
               const std::vector<double>& alpha,
               const std::vector<double>& SDF,
               const std::vector<std::array<double,3>>& normals,
               const std::vector<std::array<double,3>>& contactpoints,
               const std::string& filename) {

    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Cannot open output file");
    }

    // VTK header
    out << "# vtk DataFile Version 3.0\n";
    out << "Mesh exported from CGAL\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n";

    // Write points
    out << "POINTS " << mesh.number_of_vertices() << " double\n";
    for (auto v : mesh.vertices()) {
        const auto& p = mesh.point(v);
        out << CGAL::to_double(p.x()) << " "
            << CGAL::to_double(p.y()) << " "
            << CGAL::to_double(p.z()) << "\n";
    }
    
    // Count connectivity size
    size_t num_faces = mesh.number_of_faces();
    size_t connectivity_size = 0;
    for (auto f : mesh.faces()) {
        auto vrange = CGAL::vertices_around_face(mesh.halfedge(f), mesh);
        connectivity_size += (1 + std::distance(vrange.begin(), vrange.end()));
    }

    // Write faces
    out << "POLYGONS " << num_faces << " " << connectivity_size << "\n";
    for (auto f : mesh.faces()) {
        auto vrange = CGAL::vertices_around_face(mesh.halfedge(f), mesh);
        size_t n = std::distance(vrange.begin(), vrange.end());
        out << n;
        for (auto v : vrange) {
            out << " " << static_cast<int>(v); // VTK wants int indices
        }
        out << "\n";
    }

    // Write scalar values
    out << "POINT_DATA " << mesh.number_of_vertices() << "\n";
    out << "SCALARS alpha double 1\n";
    out << "LOOKUP_TABLE default\n";
    for (double val : alpha) {
        out << val << "\n";
    }

    out << "SCALARS SDF double 1\n";
    out << "LOOKUP_TABLE default\n";
    for (double val : SDF) {
        out << val << "\n";
    }

    out << "SCALARS normals double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (const std::array<double,3>& val : normals) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }

    out << "SCALARS contact_point double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (const std::array<double,3>& val : contactpoints) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }
}
#endif