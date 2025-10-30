#include <pe/core/detection/fine/DistanceMap.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <fstream>

// PE includes
#include <pe/core.h>
#include <pe/math.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/util/logging/DebugSection.h>

// Conditionally include CGAL headers
#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/IO/OFF.h>
#endif

namespace pe {

//=================================================================================================
// PIMPL Implementation
//=================================================================================================

class DistanceMap::Impl {
public:
#ifdef PE_USE_CGAL
    // CGAL-specific types
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using CGALKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point = Kernel::Point_3;
    using Surface_mesh = CGAL::Surface_mesh<Point>;
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using AABB_traits = CGAL::AABB_traits<Kernel, Primitive>;
    using Tree = CGAL::AABB_tree<AABB_traits>;
    using Point_inside = CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>;
    using face_descriptor = boost::graph_traits<Surface_mesh>::face_descriptor;

    // Constructor: Computes the SDF grid from a CGAL mesh
    Impl(const Surface_mesh& mesh, int resolution, int tolerance) {
        // Build AABB tree for distance queries
        Tree tree(faces(mesh).first, faces(mesh).second, mesh);
        tree.accelerate_distance_queries();
        Point_inside is_inside(tree);

        // Compute mesh bounding box
        CGAL::Bbox_3 bbox = tree.bbox();
        
        // Calculate grid spacing based on largest dimension
        pe::real dx = bbox.xmax() - bbox.xmin();
        pe::real dy = bbox.ymax() - bbox.ymin(); 
        pe::real dz = bbox.zmax() - bbox.zmin();
        
        pe::real h = std::max({dx, dy, dz}) / static_cast<pe::real>(resolution);
        
        // Expand bounding box by tolerance
        pe::real expanded_dx = dx + 2 * tolerance * h;
        pe::real expanded_dy = dy + 2 * tolerance * h;
        pe::real expanded_dz = dz + 2 * tolerance * h;
        
        // Calculate grid dimensions
        nx_ = static_cast<int>(std::ceil(expanded_dx / h)) + 1;
        ny_ = static_cast<int>(std::ceil(expanded_dy / h)) + 1;
        nz_ = static_cast<int>(std::ceil(expanded_dz / h)) + 1;
        spacing_ = h;
        
        // Grid origin (shifted by tolerance)
        origin_.set(bbox.xmin() - tolerance * h, bbox.ymin() - tolerance * h, bbox.zmin() - tolerance * h);
        
        pe_LOG_DEBUG_SECTION( log ) {
            log << "Creating SDF grid: " << nx_ << " x " << ny_ << " x " << nz_
                << " (spacing: " << spacing_ << ")\n";
        }
        
        // Resize data vectors
        size_t total_size = nx_ * ny_ * nz_;
        sdf_.resize(total_size);
        alpha_.resize(total_size);
        normals_.resize(total_size);
        contact_points_.resize(total_size);
        
        int inside_count = 0;
        
        // Fill SDF grid
        for (int k = 0; k < nz_; ++k) {
            for (int j = 0; j < ny_; ++j) {
                for (int i = 0; i < nx_; ++i) {
                    pe::real x = origin_[0] + i * spacing_;
                    pe::real y = origin_[1] + j * spacing_;
                    pe::real z = origin_[2] + k * spacing_;
                    
                    int index = i + nx_ * j + nx_ * ny_ * k;
                    
                    Point query(x, y, z);
                    
                    Tree::Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
                    Point closest = pp.first;
                    
                    contact_points_[index].set(CGAL::to_double(closest[0]), CGAL::to_double(closest[1]), CGAL::to_double(closest[2]));
                    
                    pe::real distance = std::sqrt(CGAL::squared_distance(query, closest));
                    
                    if (is_inside(query) == CGAL::ON_BOUNDED_SIDE) {
                        ++inside_count;
                        alpha_[index] = 1;
                        sdf_[index] = -distance;
                    } else {
                        alpha_[index] = 0;
                        sdf_[index] = distance;
                    }
                    
                    // Calculate vector from closest surface point to query point
                    pe::real lx = CGAL::to_double(query[0] - closest[0]);
                    pe::real ly = CGAL::to_double(query[1] - closest[1]);
                    pe::real lz = CGAL::to_double(query[2] - closest[2]);
                    
                    pe::real ln = std::sqrt(lx*lx + ly*ly + lz*lz);
                    if (ln > 1e-8) {
                        // Ensure consistent outward-pointing normals from mesh surface
                        if (is_inside(query) == CGAL::ON_BOUNDED_SIDE) {
                            // Inside: query-closest points inward, so flip to outward
                            normals_[index].set(-lx / ln, -ly / ln, -lz / ln);
                        } else {
                            // Outside: query-closest points outward (correct)
                            normals_[index].set( lx / ln, ly / ln, lz / ln);
                        }
                    } else {
                        normals_[index].set(0, 0, 0);
                    }
                }
            }
        }
        
        pe_LOG_DEBUG_SECTION( log ) {
            log << "SDF computation complete. Inside points: " << inside_count
                << " (" << double(inside_count) / double(nx_*ny_*nz_) * 100 << "%)\n";
        }
    }

    // Trilinear interpolation for scalar values
    template<typename T>
    pe::real trilinearInterpolateScalar(const std::vector<T>& data, pe::real x, pe::real y, pe::real z) const {
        double gi = (x - origin_[0]) / spacing_;
        double gj = (y - origin_[1]) / spacing_;
        double gk = (z - origin_[2]) / spacing_;
        
        int i = static_cast<int>(std::floor(gi));
        int j = static_cast<int>(std::floor(gj));
        int k = static_cast<int>(std::floor(gk));
        
        if (i < 0 || i >= nx_-1 || j < 0 || j >= ny_-1 || k < 0 || k >= nz_-1) {
            return 1e6;
        }
        
        double xd = gi - i;
        double yd = gj - j;
        double zd = gk - k;
        
        int index = i + nx_ * j + nx_ * ny_ * k;
        
        double c000 = static_cast<double>(data[index]);
        double c100 = static_cast<double>(data[index + 1]);
        double c010 = static_cast<double>(data[index + nx_]);
        double c110 = static_cast<double>(data[index + 1 + nx_]);
        double c001 = static_cast<double>(data[index + nx_ * ny_]);
        double c101 = static_cast<double>(data[index + 1 + nx_ * ny_]);
        double c011 = static_cast<double>(data[index + nx_ + nx_ * ny_]);
        double c111 = static_cast<double>(data[index + 1 + nx_ + nx_ * ny_]);
        
        return c000 * (1-xd) * (1-yd) * (1-zd) +
               c100 * xd * (1-yd) * (1-zd) +
               c010 * (1-xd) * yd * (1-zd) +
               c110 * xd * yd * (1-zd) +
               c001 * (1-xd) * (1-yd) * zd +
               c101 * xd * (1-yd) * zd +
               c011 * (1-xd) * yd * zd +
               c111 * xd * yd * zd;
    }

    // Trilinear interpolation for vector values
    pe::Vec3 trilinearInterpolateVector(const std::vector<pe::Vec3>& data, pe::real x, pe::real y, pe::real z) const {
        double gi = (x - origin_[0]) / spacing_;
        double gj = (y - origin_[1]) / spacing_;
        double gk = (z - origin_[2]) / spacing_;
        
        int i = static_cast<int>(std::floor(gi));
        int j = static_cast<int>(std::floor(gj));
        int k = static_cast<int>(std::floor(gk));
        
        if (i < 0 || i >= nx_-1 || j < 0 || j >= ny_-1 || k < 0 || k >= nz_-1) {
            return pe::Vec3(0.0, 0.0, 0.0);
        }
        
        double xd = gi - i;
        double yd = gj - j;
        double zd = gk - k;
        
        int index = i + nx_ * j + nx_ * ny_ * k;
        
        pe::Vec3 result(0.0, 0.0, 0.0);
        
        for (int comp = 0; comp < 3; ++comp) {
            double c000 = static_cast<double>(data[index][comp]);
            double c100 = static_cast<double>(data[index + 1][comp]);
            double c010 = static_cast<double>(data[index + nx_][comp]);
            double c110 = static_cast<double>(data[index + 1 + nx_][comp]);
            double c001 = static_cast<double>(data[index + nx_ * ny_][comp]);
            double c101 = static_cast<double>(data[index + 1 + nx_ * ny_][comp]);
            double c011 = static_cast<double>(data[index + nx_ + nx_ * ny_][comp]);
            double c111 = static_cast<double>(data[index + 1 + nx_ + nx_ * ny_][comp]);
            
            result[comp] = c000 * (1-xd) * (1-yd) * (1-zd) +
                           c100 * xd * (1-yd) * (1-zd) +
                           c010 * (1-xd) * yd * (1-zd) +
                           c110 * xd * yd * (1-zd) +
                           c001 * (1-xd) * (1-yd) * zd +
                           c101 * xd * (1-yd) * zd +
                           c011 * (1-xd) * yd * zd +
                           c111 * xd * yd * zd;
        }
        
        return result;
    }

    // Constructor: Creates DistanceMap from serialized data (for shadow copies)
    Impl(const std::vector<pe::real>& sdfData,
         const std::vector<int>& alphaData,
         const std::vector<pe::Vec3>& normalData,
         const std::vector<pe::Vec3>& contactPointData,
         int nx, int ny, int nz,
         pe::real spacing,
         const pe::Vec3& origin)
        : nx_(nx), ny_(ny), nz_(nz), spacing_(spacing), origin_(origin),
          sdf_(sdfData), alpha_(alphaData), normals_(normalData), contact_points_(contactPointData)
    {
        // Basic validation
        size_t expectedSize = static_cast<size_t>(nx) * ny * nz;
        if (sdf_.size() != expectedSize || alpha_.size() != expectedSize ||
            normals_.size() != expectedSize || contact_points_.size() != expectedSize) {
            std::cerr << "Warning: DistanceMap data size mismatch during reconstruction" << std::endl;
            sdf_.clear(); // Mark as invalid
        }
    }

    // Public interface of the Impl
    pe::real interpolateDistance(pe::real x, pe::real y, pe::real z) const { return trilinearInterpolateScalar(sdf_, x, y, z); }
    pe::Vec3 interpolateNormal(pe::real x, pe::real y, pe::real z) const { return trilinearInterpolateVector(normals_, x, y, z); }
    pe::Vec3 interpolateContactPoint(pe::real x, pe::real y, pe::real z) const { return trilinearInterpolateVector(contact_points_, x, y, z); }
    pe::real interpolateAlpha(pe::real x, pe::real y, pe::real z) const { return trilinearInterpolateScalar(alpha_, x, y, z); }

    bool isValid() const { return !sdf_.empty(); }

    // Grid accessors
    int getNx() const { return nx_; }
    int getNy() const { return ny_; }
    int getNz() const { return nz_; }
    pe::real getSpacing() const { return spacing_; }
    pe::Vec3 getOrigin() const { return origin_; }
    const std::vector<pe::real>& getSdfData() const { return sdf_; }
    const std::vector<int>& getAlphaData() const { return alpha_; }
    const std::vector<pe::Vec3>& getNormalData() const { return normals_; }
    const std::vector<pe::Vec3>& getContactPointData() const { return contact_points_; }

private:
    int nx_, ny_, nz_;
    pe::real spacing_;
    pe::Vec3 origin_;
    std::vector<pe::real> sdf_;
    std::vector<int> alpha_;
    std::vector<pe::Vec3> normals_;
    std::vector<pe::Vec3> contact_points_;

#else // No CGAL
public:
    // Skeleton implementation
    Impl(pe::real spacing, int resolution, int tolerance) {}
    pe::real interpolateDistance(pe::real, pe::real, pe::real) const { std::cerr << "Warning: DistanceMap used without CGAL support." << std::endl; return 1e6; }
    pe::Vec3 interpolateNormal(pe::real, pe::real, pe::real) const { std::cerr << "Warning: DistanceMap used without CGAL support." << std::endl; return pe::Vec3(0,0,0); }
    pe::Vec3 interpolateContactPoint(pe::real, pe::real, pe::real) const { std::cerr << "Warning: DistanceMap used without CGAL support." << std::endl; return pe::Vec3(0,0,0); }
    pe::real interpolateAlpha(pe::real, pe::real, pe::real) const { std::cerr << "Warning: DistanceMap used without CGAL support." << std::endl; return 0.0; }
    bool isValid() const { return false; }
    int getNx() const { return 0; }
    int getNy() const { return 0; }
    int getNz() const { return 0; }
    pe::real getSpacing() const { return 0; }
    pe::Vec3 getOrigin() const { return pe::Vec3(0,0,0); }
    const std::vector<pe::real>& getSdfData() const { static std::vector<pe::real> empty; return empty; }
    const std::vector<int>& getAlphaData() const { static std::vector<int> empty; return empty; }
    const std::vector<pe::Vec3>& getNormalData() const { static std::vector<pe::Vec3> empty; return empty; }
    const std::vector<pe::Vec3>& getContactPointData() const { static std::vector<pe::Vec3> empty; return empty; }
#endif
};

//=================================================================================================
// DistanceMap Method Implementations
//=================================================================================================

// Private constructor, destructor
DistanceMap::DistanceMap() : _pimpl(nullptr) {}
DistanceMap::~DistanceMap() = default;

// Factory function
std::unique_ptr<DistanceMap> DistanceMap::createFromFile(
    const std::string& meshFile,
    int resolution,
    int tolerance)
{
#ifdef PE_USE_CGAL
    std::string meshExt = meshFile.substr(meshFile.find_last_of('.'));
    std::transform(meshExt.begin(), meshExt.end(), meshExt.begin(), ::tolower);

    Impl::Surface_mesh cgal_mesh;

    if (meshExt == ".obj") {
        try {
            auto material = pe::createMaterial("mesh_material", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
            auto pe_mesh = pe::createTriangleMesh(1, pe::Vec3(0,0,0), meshFile, material, false, true);
            
            const auto& pe_vertices = pe_mesh->getBFVertices();
            const auto& pe_face_indices = pe_mesh->getFaceIndices();

            std::vector<Impl::Surface_mesh::Vertex_index> vertex_map(pe_vertices.size());
            for (size_t i = 0; i < pe_vertices.size(); ++i) {
                const auto& v = pe_vertices[i];
                vertex_map[i] = cgal_mesh.add_vertex(Impl::Point(v[0], v[1], v[2]));
            }

            for (const auto& face : pe_face_indices) {
                if (face.size() == 3) {
                    cgal_mesh.add_face(vertex_map[face[0]], vertex_map[face[1]], vertex_map[face[2]]);
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error loading OBJ file with PE: " << e.what() << std::endl;
            return nullptr;
        }
    } else if (meshExt == ".off") {
        std::ifstream input(meshFile);
        if (!input || !CGAL::IO::read_OFF(input, cgal_mesh)) {
            std::cerr << "Error: Cannot read OFF file " << meshFile << std::endl;
            return nullptr;
        }
    } else {
        std::cerr << "Error: Unsupported file format '" << meshExt << "'. Supported: .obj, .off" << std::endl;
        return nullptr;
    }

    if (!cgal_mesh.is_valid()) {
        std::cerr << "Error: Loaded CGAL mesh is not valid!" << std::endl;
        return nullptr;
    }

    auto map = std::unique_ptr<DistanceMap>(new DistanceMap());
    map->_pimpl = std::make_unique<Impl>(cgal_mesh, resolution, tolerance);
    
    if (!map->_pimpl->isValid()) {
        return nullptr;
    }
    return map;
#else
    std::cerr << "Warning: DistanceMap::createFromFile requires CGAL support. Please rebuild with PE_USE_CGAL=ON" << std::endl;
    return nullptr;
#endif
}

// Create method for PE TriangleMesh
std::unique_ptr<DistanceMap> DistanceMap::create(
    const pe::TriangleMeshID& mesh,
    int resolution,
    int tolerance)
{
#ifdef PE_USE_CGAL
    try {
        // Convert PE TriangleMesh to CGAL Surface_mesh
        Impl::Surface_mesh cgal_mesh;
        
        const auto& pe_vertices = mesh->getBFVertices();
        const auto& pe_face_indices = mesh->getFaceIndices();

        std::vector<Impl::Surface_mesh::Vertex_index> vertex_map(pe_vertices.size());
        for (size_t i = 0; i < pe_vertices.size(); ++i) {
            const auto& v = pe_vertices[i];
            vertex_map[i] = cgal_mesh.add_vertex(Impl::Point(v[0], v[1], v[2]));
        }

        for (const auto& face : pe_face_indices) {
            if (face.size() == 3) {
                cgal_mesh.add_face(vertex_map[face[0]], vertex_map[face[1]], vertex_map[face[2]]);
            }
        }
        
        auto map = std::unique_ptr<DistanceMap>(new DistanceMap());
        map->_pimpl = std::make_unique<Impl>(cgal_mesh, resolution, tolerance);
        
        if (!map->_pimpl->isValid()) {
            return nullptr;
        }
        return map;
    }
    catch (const std::exception& e) {
        std::cerr << "Error creating DistanceMap from PE TriangleMesh: " << e.what() << std::endl;
        return nullptr;
    }
#else
    std::cerr << "Warning: DistanceMap::create requires CGAL support. Please rebuild with PE_USE_CGAL=ON" << std::endl;
    return nullptr;
#endif
}

// Create from serialized data method (for shadow copies)
#ifdef PE_USE_CGAL
std::unique_ptr<DistanceMap> DistanceMap::createFromData(
    const std::vector<pe::real>& sdfData,
    const std::vector<int>& alphaData,
    const std::vector<pe::Vec3>& normalData,
    const std::vector<pe::Vec3>& contactPointData,
    int nx, int ny, int nz,
    pe::real spacing,
    const pe::Vec3& origin)
{
    try {
        auto map = std::unique_ptr<DistanceMap>(new DistanceMap());
        map->_pimpl = std::make_unique<Impl>(sdfData, alphaData, normalData, contactPointData,
                                           nx, ny, nz, spacing, origin);

        if (!map->_pimpl->isValid()) {
            return nullptr;
        }
        return map;
    }
    catch (const std::exception& e) {
        std::cerr << "Error creating DistanceMap from serialized data: " << e.what() << std::endl;
        return nullptr;
    }
}
#endif

// Template implementation for CGAL Surface_mesh
#ifdef PE_USE_CGAL
template<typename Point>
std::unique_ptr<DistanceMap> DistanceMap::create(
    const SurfaceMesh<Point>& mesh,
    int resolution,
    int tolerance)
{
    try {
        auto map = std::unique_ptr<DistanceMap>(new DistanceMap());
        map->_pimpl = std::make_unique<Impl>(mesh, resolution, tolerance);
        
        if (!map->_pimpl->isValid()) {
            return nullptr;
        }
        return map;
    }
    catch (const std::exception& e) {
        std::cerr << "Error creating DistanceMap from CGAL Surface_mesh: " << e.what() << std::endl;
        return nullptr;
    }
}
//DistanceMap::Impl::
// Explicit template instantiation for the kernel type we use
template std::unique_ptr<DistanceMap> DistanceMap::create<DistanceMap::Impl::CGALKernel::Point_3>(
    const SurfaceMesh<DistanceMap::Impl::CGALKernel::Point_3>& mesh,
    int resolution,
    int tolerance);
#endif

// Forwarding methods
pe::real DistanceMap::interpolateDistance(pe::real x, pe::real y, pe::real z) const { return _pimpl->interpolateDistance(x, y, z); }
pe::Vec3 DistanceMap::interpolateNormal(pe::real x, pe::real y, pe::real z) const { return _pimpl->interpolateNormal(x, y, z); }
pe::Vec3 DistanceMap::interpolateContactPoint(pe::real x, pe::real y, pe::real z) const { return _pimpl->interpolateContactPoint(x, y, z); }
pe::real DistanceMap::interpolateAlpha(pe::real x, pe::real y, pe::real z) const { return _pimpl->interpolateAlpha(x, y, z); }
int DistanceMap::getNx() const { return _pimpl->getNx(); }
int DistanceMap::getNy() const { return _pimpl->getNy(); }
int DistanceMap::getNz() const { return _pimpl->getNz(); }
pe::real DistanceMap::getSpacing() const { return _pimpl->getSpacing(); }
pe::Vec3 DistanceMap::getOrigin() const { return _pimpl->getOrigin(); }
const std::vector<pe::real>& DistanceMap::getSdfData() const { return _pimpl->getSdfData(); }
const std::vector<int>& DistanceMap::getAlphaData() const { return _pimpl->getAlphaData(); }
const std::vector<pe::Vec3>& DistanceMap::getNormalData() const { return _pimpl->getNormalData(); }
const std::vector<pe::Vec3>& DistanceMap::getContactPointData() const { return _pimpl->getContactPointData(); }

} // namespace pe
