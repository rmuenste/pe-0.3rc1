#ifndef VTKOUTPUT_H
#define VTKOUTPUT_H

#include <vector>
#include <array>
#include <string>

#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
#endif

/**
 * @brief Write a structured VTI (VTK Image Data) file for 3D grid data
 * @param filename Output filename
 * @param sdf Signed distance field values
 * @param alpha Inside/outside classification (1=inside, 0=outside)
 * @param normals Surface normal vectors at each grid point
 * @param contact_points Closest surface contact points
 * @param face_index Face indices for contact points
 * @param nx, ny, nz Grid dimensions
 * @param dx, dy, dz Grid spacing
 * @param x0, y0, z0 Grid origin coordinates
 */
void write_vti(const std::string& filename,
               const std::vector<double>& sdf,
               const std::vector<int>& alpha,
               const std::vector<std::array<double, 3>>& normals,
               const std::vector<std::array<float, 3>>& contact_points,
               const std::vector<int>& face_index,
               int nx, int ny, int nz,
               double dx, double dy, double dz,
               double x0, double y0, double z0);

#ifdef PE_USE_CGAL
/**
 * @brief Write a VTK polygonal data file for mesh with scalar data
 * @param mesh The surface mesh to export
 * @param alpha Inside/outside values at mesh vertices
 * @param SDF Signed distance field values at mesh vertices
 * @param normals Surface normal vectors at mesh vertices
 * @param contactpoints Closest surface contact points at mesh vertices
 * @param filename Output filename
 */
void write_vtk(const Surface_mesh& mesh,
               const std::vector<double>& alpha,
               const std::vector<double>& SDF,
               const std::vector<std::array<double,3>>& normals,
               const std::vector<std::array<double,3>>& contactpoints,
               const std::string& filename);
#endif

#endif // VTKOUTPUT_H