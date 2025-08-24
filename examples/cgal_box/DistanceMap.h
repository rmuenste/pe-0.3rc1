#ifndef DISTANCEMAP_H
#define DISTANCEMAP_H

#include <memory>
#include <string>
#include <vector>

// PE type includes
#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>
#include <pe/core/rigidbody/TriangleMesh.h>

namespace CGAL {
    template<typename K> class Surface_mesh;
}

namespace pe {

/**
 * @brief A 3D signed distance field (SDF) grid for collision detection
 *
 * This class creates a regular 3D grid around a triangle mesh and computes
 * signed distances at each grid point. It supports trilinear interpolation
 * for smooth distance and normal queries at arbitrary points.
 *
 * The underlying implementation may use CGAL if available, but the public
 * interface is independent of it.
 */
class DistanceMap {
public:
    /**
     * @brief Destructor
     */
    ~DistanceMap();

    /**
     * @brief Create a distance map from a mesh file (.obj or .off)
     * @param meshFile Path to the input mesh file.
     * @param spacing Grid spacing (uniform in all directions).
     * @param resolution Number of grid cells along the largest dimension (default: 50).
     * @param tolerance Number of empty boundary cells around the mesh (default: 5).
     * @return Unique pointer to the created DistanceMap, or nullptr on failure.
     */
    static std::unique_ptr<DistanceMap> createFromFile(
        const std::string& meshFile,
        pe::real spacing,
        int resolution = 50,
        int tolerance = 5
    );

    /**
     * @brief Create a distance map from a PE TriangleMesh
     * @param mesh Reference to the PE TriangleMesh
     * @param spacing Grid spacing (uniform in all directions).
     * @param resolution Number of grid cells along the largest dimension (default: 50).
     * @param tolerance Number of empty boundary cells around the mesh (default: 5).
     * @return Unique pointer to the created DistanceMap, or nullptr on failure.
     */
    static std::unique_ptr<DistanceMap> create(
        const pe::TriangleMeshID& mesh,
        pe::real spacing,
        int resolution = 50,
        int tolerance = 5
    );

#ifdef PE_USE_CGAL

    template<typename K>
    using SurfaceMesh = CGAL::Surface_mesh<K>;
    using CGALKernel = CGAL::Exact_predicates_inexact_constructions_kernel;

    /**
     * @brief Create a distance map from a CGAL Surface_mesh
     * @param mesh Reference to the CGAL Surface_mesh
     * @param spacing Grid spacing (uniform in all directions).
     * @param resolution Number of grid cells along the largest dimension (default: 50).
     * @param tolerance Number of empty boundary cells around the mesh (default: 5).
     * @return Unique pointer to the created DistanceMap, or nullptr on failure.
     */
    template<typename Point>
    static std::unique_ptr<DistanceMap> create(
        const SurfaceMesh<Point>& mesh,
        pe::real spacing,
        int resolution = 50,
        int tolerance = 5
    );
#endif

    /**
     * @brief Get the signed distance at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated signed distance (negative inside, positive outside)
     */
    pe::real interpolateDistance(pe::real x, pe::real y, pe::real z) const;

    /**
     * @brief Get the surface normal at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated unit normal vector pointing outward from surface
     */
    pe::Vec3 interpolateNormal(pe::real x, pe::real y, pe::real z) const;

    /**
     * @brief Get the closest surface contact point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated contact point on the mesh surface
     */
    pe::Vec3 interpolateContactPoint(pe::real x, pe::real y, pe::real z) const;

    /**
     * @brief Get the inside/outside classification at a point using trilinear interpolation
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Interpolated alpha value (1.0 = inside, 0.0 = outside, fractional at boundary)
     */
    pe::real interpolateAlpha(pe::real x, pe::real y, pe::real z) const;

    // Grid access methods
    int getNx() const;
    int getNy() const;
    int getNz() const;
    pe::real getSpacing() const;
    pe::Vec3 getOrigin() const;

    // Direct grid data access (for visualization/debugging)
    const std::vector<pe::real>& getSdfData() const;
    const std::vector<int>& getAlphaData() const;
    const std::vector<pe::Vec3>& getNormalData() const;
    const std::vector<pe::Vec3>& getContactPointData() const;

private:
    // Private constructor - use createFromFile() factory method
    DistanceMap();

    class Impl; // Forward-declare the implementation class
    std::unique_ptr<Impl> _pimpl; // Pointer to implementation
};

} // namespace pe

#endif // DISTANCEMAP_H
