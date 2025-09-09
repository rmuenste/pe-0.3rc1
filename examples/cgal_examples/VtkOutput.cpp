#include "VtkOutput.h"
#include <iostream>
#include <fstream>
#include <iomanip>

// PE includes
#include <pe/math.h>

#ifdef PE_USE_CGAL
#include <CGAL/boost/graph/helpers.h>
#endif

void write_vti(const std::string& filename,
               const std::vector<pe::real>& sdf,
               const std::vector<int>& alpha,
               const std::vector<pe::Vec3>& normals,
               const std::vector<pe::Vec3>& contact_points,
               const std::vector<int>& face_index,
               int nx, int ny, int nz,
               pe::real dx, pe::real dy, pe::real dz,
               pe::real x0, pe::real y0, pe::real z0)
{
    std::ofstream out(filename);
    out << std::fixed << std::setprecision(6);

    out << R"(<?xml version="1.0"?>)" << "\n";
    out << R"(<VTKFile type="ImageData" version="0.1" byte_order="LittleEndian">)" << "\n";
    out << "  <ImageData Origin=\"" << x0 << " " << y0 << " " << z0 << "\" "
        << "Spacing=\"" << dx << " " << dy << " " << dz << "\" "
        << "WholeExtent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";

    out << "    <Piece Extent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";

    out << "      <PointData Scalars=\"sdf\">\n";

    out << "        <DataArray type=\"Float32\" Name=\"sdf\" format=\"ascii\">\n";
    for (pe::real v : sdf) out << v << " ";
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
    out << "      <CellData/>\n";
    out << "    </Piece>\n";
    out << "  </ImageData>\n";
    out << "</VTKFile>\n";
}

void write_vtk_points(const std::string& filename,
                      const std::vector<pe::Vec3>& points,
                      const std::vector<bool>& distanceMapResults,
                      const std::vector<bool>& cgalResults,
                      bool hasCgalResults)
{
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Cannot open output file for point cloud");
    }

    out << std::fixed << std::setprecision(6);
    
    out << "# vtk DataFile Version 3.0\n";
    out << "Point containment test results\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n";

    // Write points
    out << "POINTS " << points.size() << " double\n";
    for (const auto& point : points) {
        out << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    // Write point data
    out << "POINT_DATA " << points.size() << "\n";
    
    // DistanceMap results
    out << "SCALARS DistanceMapResult int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (bool result : distanceMapResults) {
        out << (result ? 1 : 0) << "\n";
    }

    // CGAL results (if available)
    if (hasCgalResults) {
        out << "SCALARS CGALResult int 1\n";
        out << "LOOKUP_TABLE default\n";
        for (bool result : cgalResults) {
            out << (result ? 1 : 0) << "\n";
        }

        // Disagreement field
        out << "SCALARS Disagreement int 1\n";
        out << "LOOKUP_TABLE default\n";
        for (size_t i = 0; i < points.size(); ++i) {
            out << (distanceMapResults[i] != cgalResults[i] ? 1 : 0) << "\n";
        }
    }
}

#ifdef PE_USE_CGAL
void write_vtk(const Surface_mesh& mesh,
               const std::vector<pe::real>& alpha,
               const std::vector<pe::real>& SDF,
               const std::vector<pe::Vec3>& normals,
               const std::vector<pe::Vec3>& contactpoints,
               const std::string& filename) {

    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Cannot open output file");
    }

    out << "# vtk DataFile Version 3.0\n";
    out << "Mesh exported from CGAL\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n";

    out << "POINTS " << mesh.number_of_vertices() << " double\n";
    for (auto v : mesh.vertices()) {
        const auto& p = mesh.point(v);
        out << CGAL::to_double(p.x()) << " "
            << CGAL::to_double(p.y()) << " "
            << CGAL::to_double(p.z()) << "\n";
    }
    
    size_t num_faces = mesh.number_of_faces();
    size_t connectivity_size = 0;
    for (auto f : mesh.faces()) {
        auto vrange = CGAL::vertices_around_face(mesh.halfedge(f), mesh);
        connectivity_size += (1 + std::distance(vrange.begin(), vrange.end()));
    }

    out << "POLYGONS " << num_faces << " " << connectivity_size << "\n";
    for (auto f : mesh.faces()) {
        auto vrange = CGAL::vertices_around_face(mesh.halfedge(f), mesh);
        size_t n = std::distance(vrange.begin(), vrange.end());
        out << n;
        for (auto v : vrange) {
            out << " " << static_cast<int>(v);
        }
        out << "\n";
    }

    out << "POINT_DATA " << mesh.number_of_vertices() << "\n";
    out << "SCALARS alpha double 1\n";
    out << "LOOKUP_TABLE default\n";
    for (pe::real val : alpha) {
        out << val << "\n";
    }

    out << "SCALARS SDF double 1\n";
    out << "LOOKUP_TABLE default\n";
    for (pe::real val : SDF) {
        out << val << "\n";
    }

    out << "SCALARS normals double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (const auto& val : normals) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }

    out << "SCALARS contact_point double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (const auto& val : contactpoints) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }
}
#endif
