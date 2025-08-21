#ifndef DISTANCEMAP_H
#define DISTANCEMAP_H

#include <vector>
#include <array>
#include <memory>

#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Side_of_triangle_mesh.h>

// Forward declaration for PE TriangleMesh
namespace pe {
    class TriangleMesh;
}

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;
typedef CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> Point_inside;
typedef boost::graph_traits<Surface_mesh>::face_descriptor face_descriptor;
#endif

/**
 * @brief A 3D signed distance field (SDF) grid for collision detection
 * 
 * This class creates a regular 3D grid around a triangle mesh and computes
 * signed distances at each grid point using CGAL. It supports trilinear 
 * interpolation for smooth distance and normal queries at arbitrary points.
 */
class DistanceMap {
public:
    /**
     * @brief Create a distance map from a CGAL triangle mesh
     * @param mesh The input CGAL Surface_mesh
     * @param spacing Grid spacing (uniform in all directions)
     * @param resolution Number of grid cells along the largest dimension (default: 50)
     * @param tolerance Number of empty boundary cells around the mesh (default: 5)
     * @return Unique pointer to the created DistanceMap, or nullptr if CGAL not available
     */
#ifdef PE_USE_CGAL
    static std::unique_ptr<DistanceMap> create(const Surface_mesh& mesh, 
                                             double spacing,
                                             int resolution = 50,
                                             int tolerance = 5);

    /**
     * @brief Create a distance map from a PE TriangleMesh (supports OBJ files with COM centering)
     * @param pe_mesh The input PE TriangleMesh with vertices already centered around COM
     * @param spacing Grid spacing (uniform in all directions)
     * @param resolution Number of grid cells along the largest dimension (default: 50)
     * @param tolerance Number of empty boundary cells around the mesh (default: 5)
     * @return Unique pointer to the created DistanceMap, or nullptr if CGAL not available
     */
    static std::unique_ptr<DistanceMap> create(const pe::TriangleMesh& pe_mesh,
                                             double spacing,
                                             int resolution = 50,
                                             int tolerance = 5);
#endif

    /**
     * @brief Get the signed distance at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate  
     * @param z Z coordinate
     * @return Interpolated signed distance (negative inside, positive outside)
     */
    double interpolateDistance(double x, double y, double z) const;

    /**
     * @brief Get the surface normal at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated unit normal vector pointing outward from surface
     */
    std::array<double, 3> interpolateNormal(double x, double y, double z) const;

    /**
     * @brief Get the closest surface contact point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated contact point on the mesh surface
     */
    std::array<double, 3> interpolateContactPoint(double x, double y, double z) const;

    /**
     * @brief Get the inside/outside classification at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated alpha value (1.0 = inside, 0.0 = outside, fractional at boundary)
     */
    double interpolateAlpha(double x, double y, double z) const;

    // Grid access methods
    int getNx() const { return nx_; }
    int getNy() const { return ny_; }
    int getNz() const { return nz_; }
    double getSpacing() const { return spacing_; }
    std::array<double, 3> getOrigin() const { return origin_; }

    // Direct grid data access (for visualization/debugging)
    const std::vector<double>& getSdfData() const { return sdf_; }
    const std::vector<int>& getAlphaData() const { return alpha_; }
    const std::vector<std::array<double, 3>>& getNormalData() const { return normals_; }
    const std::vector<std::array<double, 3>>& getContactPointData() const { return contact_points_; }

private:
    // Private constructor - use create() method
    DistanceMap(int nx, int ny, int nz, double spacing, const std::array<double, 3>& origin);

    // Trilinear interpolation helper for scalar values
    template<typename T>
    double trilinearInterpolateScalar(const std::vector<T>& data, 
                                    double x, double y, double z) const;

    // Trilinear interpolation helper for 3D vector values  
    template<typename T>
    std::array<double, 3> trilinearInterpolateVector(const std::vector<std::array<T, 3>>& data,
                                                   double x, double y, double z) const;

    // Grid dimensions
    int nx_, ny_, nz_;
    double spacing_;
    std::array<double, 3> origin_;

    // Grid data
    std::vector<double> sdf_;                               // Signed distance values
    std::vector<int> alpha_;                                // Inside/outside classification  
    std::vector<std::array<double, 3>> normals_;           // Surface normal vectors
    std::vector<std::array<double, 3>> contact_points_;    // Closest surface points
};

#endif // DISTANCEMAP_H