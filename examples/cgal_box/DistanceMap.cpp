#include "DistanceMap.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <map>

// PE includes for TriangleMesh integration
#ifdef PE_USE_CGAL
#include <pe/core.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#endif

DistanceMap::DistanceMap(int nx, int ny, int nz, double spacing, const std::array<double, 3>& origin)
    : nx_(nx), ny_(ny), nz_(nz), spacing_(spacing), origin_(origin)
{
    size_t total_size = nx_ * ny_ * nz_;
    sdf_.resize(total_size);
    alpha_.resize(total_size);
    normals_.resize(total_size);
    contact_points_.resize(total_size);
}

#ifdef PE_USE_CGAL
std::unique_ptr<DistanceMap> DistanceMap::create(const Surface_mesh& mesh, 
                                               double spacing,
                                               int resolution,
                                               int tolerance)
{
    // Build AABB tree for distance queries
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.accelerate_distance_queries();
    Point_inside is_inside(tree);

    // Compute mesh bounding box
    CGAL::Bbox_3 bbox = tree.bbox();
    
    std::cout << "Mesh bounding box: [" 
              << bbox.xmin() << ", " << bbox.xmax() << "] x ["
              << bbox.ymin() << ", " << bbox.ymax() << "] x ["
              << bbox.zmin() << ", " << bbox.zmax() << "]" << std::endl;

    // Calculate grid spacing based on largest dimension
    double dx = bbox.xmax() - bbox.xmin();
    double dy = bbox.ymax() - bbox.ymin(); 
    double dz = bbox.zmax() - bbox.zmin();
    
    double h = std::max({dx, dy, dz}) / static_cast<double>(resolution);
    
    // Expand bounding box by tolerance
    double expanded_dx = dx + 2 * tolerance * h;
    double expanded_dy = dy + 2 * tolerance * h;
    double expanded_dz = dz + 2 * tolerance * h;
    
    // Calculate grid dimensions
    int nx = static_cast<int>(std::ceil(expanded_dx / h)) + 1;
    int ny = static_cast<int>(std::ceil(expanded_dy / h)) + 1;
    int nz = static_cast<int>(std::ceil(expanded_dz / h)) + 1;
    
    // Grid origin (shifted by tolerance)
    std::array<double, 3> origin = {
        bbox.xmin() - tolerance * h,
        bbox.ymin() - tolerance * h, 
        bbox.zmin() - tolerance * h
    };
    
    std::cout << "Creating SDF grid: " << nx << " x " << ny << " x " << nz 
              << " (spacing: " << h << ")" << std::endl;
    
    // Create DistanceMap instance
    auto distance_map = std::unique_ptr<DistanceMap>(new DistanceMap(nx, ny, nz, h, origin));
    
    // Create face index mapping for contact point face tracking
    std::map<face_descriptor, std::size_t> face_index_map;
    std::size_t face_id = 0;
    for (face_descriptor f : faces(mesh)) {
        face_index_map[f] = face_id++;
    }
    
    int inside_count = 0;
    
    // Fill SDF grid
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                double x = origin[0] + i * h;
                double y = origin[1] + j * h;
                double z = origin[2] + k * h;
                
                int index = i + nx * j + nx * ny * k;
                
                Point query(x, y, z);
                
                // Find closest point on mesh surface
                Tree::Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
                Point closest = pp.first;
                
                // Store contact point
                distance_map->contact_points_[index] = {
                    CGAL::to_double(closest[0]),
                    CGAL::to_double(closest[1]), 
                    CGAL::to_double(closest[2])
                };
                
                // Calculate unsigned distance
                double distance = std::sqrt(CGAL::squared_distance(query, closest));
                
                // Determine inside/outside and sign distance
                if (is_inside(query) == CGAL::ON_BOUNDED_SIDE) {
                    ++inside_count;
                    distance_map->alpha_[index] = 1;      // Inside
                    distance_map->sdf_[index] = -distance;  // Negative distance inside
                } else {
                    distance_map->alpha_[index] = 0;      // Outside  
                    distance_map->sdf_[index] = distance;   // Positive distance outside
                }
                
                // Calculate surface normal (points from surface to query point)
                double lx = CGAL::to_double(query[0] - closest[0]);
                double ly = CGAL::to_double(query[1] - closest[1]);
                double lz = CGAL::to_double(query[2] - closest[2]);
                
                double ln = std::sqrt(lx*lx + ly*ly + lz*lz);
                if (ln > 1e-8) {
                    distance_map->normals_[index] = { lx / ln, ly / ln, lz / ln };
                } else {
                    distance_map->normals_[index] = { 0, 0, 0 };
                }
            }
        }
    }
    
    std::cout << "SDF computation complete. Inside points: " << inside_count 
              << " (" << double(inside_count) / double(nx*ny*nz) * 100 << "%)" << std::endl;
    
    return distance_map;
}

std::unique_ptr<DistanceMap> DistanceMap::create(const pe::TriangleMesh& pe_mesh,
                                               double spacing,
                                               int resolution,
                                               int tolerance)
{
    std::cout << "Creating DistanceMap from PE TriangleMesh (vertices already COM-centered)..." << std::endl;
    
    // Get vertices and face indices from PE TriangleMesh
    const auto& pe_vertices = pe_mesh.getBFVertices();      // Body frame vertices (COM-centered)
    const auto& pe_face_indices = pe_mesh.getFaceIndices();
    
    std::cout << "PE mesh: " << pe_vertices.size() << " vertices, " << pe_face_indices.size() << " faces" << std::endl;
    
    // Convert PE mesh to CGAL Surface_mesh
    Surface_mesh cgal_mesh;
    
    // Add vertices to CGAL mesh
    std::vector<Surface_mesh::Vertex_index> vertex_map(pe_vertices.size());
    for (size_t i = 0; i < pe_vertices.size(); ++i) {
        const auto& v = pe_vertices[i];
        Point p(v[0], v[1], v[2]);  // Convert Vec3 to CGAL Point_3
        vertex_map[i] = cgal_mesh.add_vertex(p);
    }
    
    // Add faces to CGAL mesh
    for (size_t i = 0; i < pe_face_indices.size(); ++i) {
        const auto& face = pe_face_indices[i];
        if (face.size() == 3) {  // Ensure it's a triangle
            cgal_mesh.add_face(vertex_map[face[0]], 
                              vertex_map[face[1]], 
                              vertex_map[face[2]]);
        } else {
            std::cerr << "Warning: Non-triangular face detected at index " << i 
                      << " with " << face.size() << " vertices. Skipping." << std::endl;
        }
    }
    
    std::cout << "Converted to CGAL mesh: " << cgal_mesh.number_of_vertices() 
              << " vertices, " << cgal_mesh.number_of_faces() << " faces" << std::endl;
    
    // Verify mesh is valid
    if (!cgal_mesh.is_valid()) {
        std::cerr << "Error: Converted CGAL mesh is not valid!" << std::endl;
        return nullptr;
    }
    
    // Use existing CGAL-based create function
    return create(cgal_mesh, spacing, resolution, tolerance);
}


#endif

double DistanceMap::interpolateDistance(double x, double y, double z) const {
    return trilinearInterpolateScalar(sdf_, x, y, z);
}

std::array<double, 3> DistanceMap::interpolateNormal(double x, double y, double z) const {
    return trilinearInterpolateVector(normals_, x, y, z);
}

std::array<double, 3> DistanceMap::interpolateContactPoint(double x, double y, double z) const {
    return trilinearInterpolateVector(contact_points_, x, y, z);
}

double DistanceMap::interpolateAlpha(double x, double y, double z) const {
    return trilinearInterpolateScalar(alpha_, x, y, z);
}

template<typename T>
double DistanceMap::trilinearInterpolateScalar(const std::vector<T>& data, 
                                             double x, double y, double z) const {
    // Convert to grid coordinates
    double gi = (x - origin_[0]) / spacing_;
    double gj = (y - origin_[1]) / spacing_;
    double gk = (z - origin_[2]) / spacing_;
    
    // Get integer grid indices
    int i = static_cast<int>(std::floor(gi));
    int j = static_cast<int>(std::floor(gj));
    int k = static_cast<int>(std::floor(gk));
    
    // Check bounds and clamp to valid range
    if (i < 0 || i >= nx_-1 || j < 0 || j >= ny_-1 || k < 0 || k >= nz_-1) {
        // Return a large positive distance for points outside grid
        return 1e6;
    }
    
    // Fractional parts for interpolation
    double xd = gi - i;
    double yd = gj - j; 
    double zd = gk - k;
    
    // Get the 8 corner values
    int index = i + nx_ * j + nx_ * ny_ * k;
    
    double c000 = static_cast<double>(data[index]);
    double c100 = static_cast<double>(data[index + 1]);
    double c010 = static_cast<double>(data[index + nx_]);
    double c110 = static_cast<double>(data[index + 1 + nx_]);
    double c001 = static_cast<double>(data[index + nx_ * ny_]);
    double c101 = static_cast<double>(data[index + 1 + nx_ * ny_]);
    double c011 = static_cast<double>(data[index + nx_ + nx_ * ny_]);
    double c111 = static_cast<double>(data[index + 1 + nx_ + nx_ * ny_]);
    
    // Trilinear interpolation
    double result = c000 * (1-xd) * (1-yd) * (1-zd) +
                   c100 * xd * (1-yd) * (1-zd) +
                   c010 * (1-xd) * yd * (1-zd) +
                   c110 * xd * yd * (1-zd) +
                   c001 * (1-xd) * (1-yd) * zd +
                   c101 * xd * (1-yd) * zd +
                   c011 * (1-xd) * yd * zd +
                   c111 * xd * yd * zd;
                   
    return result;
}

template<typename T>
std::array<double, 3> DistanceMap::trilinearInterpolateVector(const std::vector<std::array<T, 3>>& data,
                                                            double x, double y, double z) const {
    // Convert to grid coordinates
    double gi = (x - origin_[0]) / spacing_;
    double gj = (y - origin_[1]) / spacing_;
    double gk = (z - origin_[2]) / spacing_;
    
    // Get integer grid indices
    int i = static_cast<int>(std::floor(gi));
    int j = static_cast<int>(std::floor(gj));
    int k = static_cast<int>(std::floor(gk));
    
    // Check bounds and return zero vector for points outside grid
    if (i < 0 || i >= nx_-1 || j < 0 || j >= ny_-1 || k < 0 || k >= nz_-1) {
        return {0.0, 0.0, 0.0};
    }
    
    // Fractional parts for interpolation
    double xd = gi - i;
    double yd = gj - j; 
    double zd = gk - k;
    
    // Get the 8 corner values
    int index = i + nx_ * j + nx_ * ny_ * k;
    
    std::array<double, 3> result = {0.0, 0.0, 0.0};
    
    // Interpolate each component separately
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

// Explicit template instantiations
template double DistanceMap::trilinearInterpolateScalar<double>(const std::vector<double>&, double, double, double) const;
template double DistanceMap::trilinearInterpolateScalar<int>(const std::vector<int>&, double, double, double) const;
template std::array<double, 3> DistanceMap::trilinearInterpolateVector<double>(const std::vector<std::array<double, 3>>&, double, double, double) const;
template std::array<double, 3> DistanceMap::trilinearInterpolateVector<float>(const std::vector<std::array<float, 3>>&, double, double, double) const;