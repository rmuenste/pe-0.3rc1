#ifndef VTKOUTPUT_H
#define VTKOUTPUT_H

#include <vector>
#include <array>
#include <string>

// PE includes
#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>

#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
#endif

/**
 * @brief Write a structured VTI (VTK Image Data) file for 3D grid data
 */
void write_vti(const std::string& filename,
               const std::vector<pe::real>& sdf,
               const std::vector<int>& alpha,
               const std::vector<pe::Vec3>& normals,
               const std::vector<pe::Vec3>& contact_points,
               const std::vector<int>& face_index,
               int nx, int ny, int nz,
               pe::real dx, pe::real dy, pe::real dz,
               pe::real x0, pe::real y0, pe::real z0);

#ifdef PE_USE_CGAL
/**
 * @brief Write a VTK polygonal data file for mesh with scalar data
 */
void write_vtk(const Surface_mesh& mesh,
               const std::vector<pe::real>& alpha,
               const std::vector<pe::real>& SDF,
               const std::vector<pe::Vec3>& normals,
               const std::vector<pe::Vec3>& contactpoints,
               const std::string& filename);
#endif

#endif // VTKOUTPUT_H
