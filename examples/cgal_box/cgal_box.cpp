//******************************************************************************************
//
// This example demonstrates how to use CGAL's oriented_bounding_box functionality 
// to create optimal oriented bounding boxes (OBB) for a set of points.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>

// PE headers
#include <pe/core.h>
#include <pe/math.h>

// CGAL headers
#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/bounding_box.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/OFF.h>
#endif

// Define kernel
#ifdef PE_USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector_3; //typedef for Vector type
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;
typedef CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> Point_inside;
typedef boost::graph_traits<Surface_mesh>::face_descriptor face_descriptor;
#endif

using namespace pe;




void write_vti(const std::string& filename,
               const std::vector<double>& sdf,
               const std::vector<int>& alpha,
               const std::vector<std::array<double, 3>> normals,
               std::vector<std::array<float, 3>> contact_points,
               const std::vector<int>& face_index,
               int nx, int ny, int nz,
               double dx, double dy, double dz,
               double x0, double y0, double z0);

template <typename T>
void trilinear_interpolation(int i, int j, int k,
                            int nx, int ny, int nz,
                            double x0, double y0, double z0,
                            const double h,
                            const std::vector<T>& value,
                            double x, double y, double z,
                            double& a);


template <typename T>

void trilinear_interpolation(int i, int j, int k,
                            int nx, int ny, int nz,
                            double x0, double y0, double z0,
                            const double h,
                            const std::vector<std::array<T,3>>& value,
                            double x, double y, double z,
                            std::array<double, 3>& a);


#ifdef PE_USE_CGAL
void write_vtk(const Surface_mesh& mesh,
    const std::vector<double>& alpha,
    const std::vector<double>& SDF,
    const std::vector<std::array<double,3>>& normals,
    const std::vector<std::array<double,3>>& contactpoints,
    const std::string& filename);
#endif
    


int main(int argc, char* argv[]) {
    // std::cout << "CGAL Oriented Bounding Box Example!" << std::endl;
    std::cout << "CGAL SDF Collision Detection Example!" << std::endl;
    
#ifdef PE_USE_CGAL
        std::string meshFile;
        std::string chipFile;

        Surface_mesh fromFile;
        Surface_mesh secondaryMesh;
        if (argc >= 2) {
        meshFile = argv[1];
        chipFile = argv[3];

        std::string output_prefix = (argc > 2) ? argv[2] : "output";
        std::ifstream input(meshFile);

        if (!input || !CGAL::IO::read_OFF(input, fromFile)) {
            std::cerr << "Error: Cannot read file " << meshFile << "\n";
            return 1;
        }

        std::vector<Point> pointsFromMesh;
        for(auto v : fromFile.vertices()) {
            pointsFromMesh.push_back(fromFile.point(v));
        }

        Tree tree(faces(fromFile).first, faces(fromFile).second, fromFile);
        tree.accelerate_distance_queries();
        Point_inside is_inside(tree);

        // Compute bounding box
        CGAL::Bbox_3 bbox = tree.bbox();


        std::cout << "Loaded mesh with " << num_vertices(fromFile)
                << " vertices and " << num_faces(fromFile) << " faces." << std::endl;
        
            // Define background grid resolution
        const int n = 50;
        double h;
        double hx; double hy; double hz;
        double tol;
        tol = 5; // number of empty cells on bounding box boundary
        int inside_count = 0;

        double dx = (bbox.xmax() - bbox.xmin());
        double dy = (bbox.ymax() - bbox.ymin());
        double dz = (bbox.zmax() - bbox.zmin());

        h = std::max({dx, dy, dz}) / static_cast<double>(n);
        hx = h; hy = h; hz = h;
        //hx = dx / static_cast<double>(n);
        //hy = dy / static_cast<double>(n);
        //hz = dz / static_cast<double>(n);
        dx += 2*tol*hx; dy += 2*tol*hy; dz += 2*tol*hz;

        int nx = static_cast<int>(std::ceil(dx / hx)) + 1;
        int ny = static_cast<int>(std::ceil(dy / hy)) + 1;
        int nz = static_cast<int>(std::ceil(dz / hz)) + 1;

        std::vector<std::array<double, 3>> normals(nx * ny * nz);
        std::vector<std::array<float, 3>> contact_points(nx * ny * nz);
        std::vector<face_descriptor> face_list(nx * ny * nz);

        std::vector<int> alpha(nx * ny * nz, 0);
        std::vector<double> sdf(nx * ny * nz, 0.0f);
        std::vector<Point> points;
        points.reserve(nx * ny * nz);

        // for face index reconstruction
        std::map<face_descriptor, std::size_t> face_index_map;
        std::size_t id = 0;
        std::vector<int> face_index(nx * ny * nz, 0);
        
        std::array<double, 3> origin = {bbox.xmin() - tol*hx, bbox.ymin() - tol*hy, bbox.zmin() - tol*hz};

        for (face_descriptor f : faces(fromFile)) {
            face_index_map[f] = id++;
        }

        for (int k = 0; k < nz; ++k) {
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    double x = origin[0] + i * hx;
                    double y = origin[1] + j * hy;
                    double z = origin[2] + k * hz;

                    points.emplace_back(x, y, z);
                    int index = i + nx*j + nx*ny*k;

                    
                    Point query(x,y,z);
                    Tree::Point_and_primitive_id pp = tree.closest_point_and_primitive(query);

                    contact_points[index] = {
                        static_cast<float>(pp.first[0]),
                        static_cast<float>(pp.first[1]),
                        static_cast<float>(pp.first[2])
                    };

                    face_index[index] = face_index_map[pp.second];

                    Point closest(pp.first[0], pp.first[1], pp.first[2]);
                    sdf[index] = std::sqrt(CGAL::squared_distance(query, closest));


                    if (is_inside(query) == CGAL::ON_BOUNDED_SIDE) {
                        ++inside_count;
                        alpha[index] = 1;
                        sdf[index] = -static_cast<float>(sdf[index]);;

                    }

                    float lx = contact_points[index][0]-query[0];
                    float ly = contact_points[index][1]-query[1];
                    float lz = contact_points[index][2]-query[2];
                    
                    float ln = std::sqrt(lx*lx + ly*ly + lz*lz);
                    if (ln > 1e-8f) {
                        normals[index] = { lx / ln, ly / ln, lz / ln };
                    }
                    else{
                        normals[index] = {0, 0, 0};
                    }

                }
            }
        }

        std::cout << "Total inside points: " << inside_count <<", "<<double(inside_count)/double(nx*ny*nz) *100 <<"\% of mesh nodes "<< std::endl;
        std::cout << "Write vtu" << std::endl;


        write_vti("sdf.vti",
                sdf, alpha, normals,
                contact_points, face_index,
                nx, ny, nz,
                hx, hy, hz,
                origin[0], origin[1], origin[2]);


        std::ifstream input2(chipFile);
        if (!input2 || !CGAL::IO::read_OFF(input2, secondaryMesh)) {
            std::cerr << "Error: Cannot read file " << chipFile << "\n";
            return 1;
        }

        std::vector<Point> pointsFromChip;
        for(auto v : secondaryMesh.vertices()) {
            pointsFromChip.push_back(secondaryMesh.point(v));
        }

        Tree chiptree(faces(secondaryMesh).first, faces(secondaryMesh).second, secondaryMesh);
        chiptree.accelerate_distance_queries();


        std::cout << "Loaded secondaryMesh with " << num_vertices(secondaryMesh)
            << " vertices secondaryMesh " << num_faces(secondaryMesh) << " faces." << std::endl;
        
        int entry = 0;
        std::vector<double> chipAlpha(num_vertices(secondaryMesh));
        std::vector<double> chipSDF(num_vertices(secondaryMesh));
        std::vector<std::array<double, 3>> chipNormal(num_vertices(secondaryMesh));
        std::vector<std::array<double, 3>> chipContactPoint(num_vertices(secondaryMesh));

        double default_sdf = *std::max_element(sdf.begin(), sdf.end());

        for(auto v : secondaryMesh.vertices()) {
            
            const Point& p = secondaryMesh.point(v);

            int i = static_cast<int>(std::floor((p.x()-origin[0])/hx));
            int j = static_cast<int>(std::floor((p.y()-origin[1])/hy));
            int k = static_cast<int>(std::floor((p.z()-origin[2])/hz));
            
            // default values in case of point out of bounding box
            if ( (i>=nx || j>=ny || k >=nz) || (i<0 || j<0 || k<0) ){
                chipAlpha[entry] = -1;
                chipSDF[entry] = default_sdf;
                chipNormal[entry] = {0,0,0};
                chipContactPoint[entry] = {origin[0], origin[1], origin[2]};
            }
            else{   
                trilinear_interpolation(i, j, k, nx, ny, nz,
                                    origin[0] + i * hx, origin[1] + j * hy, origin[2] + k * hz,
                                    hx, alpha,
                                    p.x(), p.y(), p.z(),
                                    chipAlpha[entry]);

                trilinear_interpolation(i, j, k, nx, ny, nz,
                                    origin[0] + i * hx, origin[1] + j * hy, origin[2] + k * hz,
                                    hx, sdf,
                                    p.x(), p.y(), p.z(),
                                    chipSDF[entry]);
                
                trilinear_interpolation(i, j, k, nx, ny, nz,
                                    origin[0] + i * hx, origin[1] + j * hy, origin[2] + k * hz,
                                    hx, normals,
                                    p.x(), p.y(), p.z(),
                                    chipNormal[entry]);

                trilinear_interpolation(i, j, k, nx, ny, nz,
                                    origin[0] + i * hx, origin[1] + j * hy, origin[2] + k * hz,
                                    hx, contact_points,
                                    p.x(), p.y(), p.z(),
                                    chipContactPoint[entry]);

            }
            ++entry;
        }

        write_vtk(secondaryMesh,
            chipAlpha,
            chipSDF,
            chipNormal,
            chipContactPoint,
             "./out.vtk");

    }




#else
    std::cout << "This example requires CGAL support. Please rebuild with CGAL=ON" << std::endl;
#endif

    return 0;
}





void write_vti(const std::string& filename,
               const std::vector<double>& sdf,
               const std::vector<int>& alpha,
               const std::vector<std::array<double, 3>> normals,
               std::vector<std::array<float, 3>> contact_points,
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
    for (float v : sdf) out << v << " ";
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

    // // Write scalar values
    // out << "POINT_DATA " << mesh.number_of_vertices() << "\n";
    out << "SCALARS SDF double 1\n";
    out << "LOOKUP_TABLE default\n";
    for (double val : SDF) {
        out << val << "\n";
    }

    out << "SCALARS normals double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (std::array<double,3> val : normals) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }

    
    out << "SCALARS contact_point double 3\n";
    out << "LOOKUP_TABLE default\n";
    for (std::array<double,3> val : contactpoints) {
        out << val[0]<<" "<<val[1]<<" "<<val[2] << "\n";
    }
}
#endif


template <typename T>
void trilinear_interpolation(int i, int j, int k,
                            int nx, int ny, int nz,
                            double x0, double y0, double z0,
                            const double h,
                            const std::vector<T>& value,
                            double x, double y, double z,
                            double& a)
{
  int index = i + nx*j + nx*ny*k;

  double xd = (x-x0) / h;
  double yd = (y-y0) / h;
  double zd = (z-z0) / h;

  a = value[index           ]*(1-xd)*(1-yd)*(1-zd)
    + value[index+1         ]*   xd *(1-yd)*(1-zd)
    + value[index+nx        ]*(1-xd)*   yd *(1-zd)
    + value[index+1+nx      ]*   xd *   yd *(1-zd)
    + value[index+nx*ny     ]*(1-xd)*(1-yd)*   zd
    + value[index+1+nx*ny   ]*   xd *(1-yd)*   zd
    + value[index+nx+nx*ny  ]*(1-xd)*   yd *   zd
    + value[index+1+nx+nx*ny]*   xd *   yd *   zd;

}

template <typename T>
void trilinear_interpolation(int i, int j, int k,
                            int nx, int ny, int nz,
                            double x0, double y0, double z0,
                            const double h,
                            const std::vector<std::array<T,3>>& value,
                            double x, double y, double z,
                            std::array<double, 3>& a)
{
  int index = i + nx*j + nx*ny*k;

  double xd = (x-x0) / h;
  double yd = (y-y0) / h;
  double zd = (z-z0) / h;

  a[0] = value[index           ][0]*(1-xd)*(1-yd)*(1-zd)
       + value[index+1         ][0]*   xd *(1-yd)*(1-zd)
       + value[index+nx        ][0]*(1-xd)*   yd *(1-zd)
       + value[index+1+nx      ][0]*   xd *   yd *(1-zd)
       + value[index+nx*ny     ][0]*(1-xd)*(1-yd)*   zd
       + value[index+1+nx*ny   ][0]*   xd *(1-yd)*   zd
       + value[index+nx+nx*ny  ][0]*(1-xd)*   yd *   zd
       + value[index+1+nx+nx*ny][0]*   xd *   yd *   zd;

  a[1] = value[index           ][1]*(1-xd)*(1-yd)*(1-zd)
       + value[index+1         ][1]*   xd *(1-yd)*(1-zd)
       + value[index+nx        ][1]*(1-xd)*   yd *(1-zd)
       + value[index+1+nx      ][1]*   xd *   yd *(1-zd)
       + value[index+nx*ny     ][1]*(1-xd)*(1-yd)*   zd
       + value[index+1+nx*ny   ][1]*   xd *(1-yd)*   zd
       + value[index+nx+nx*ny  ][1]*(1-xd)*   yd *   zd
       + value[index+1+nx+nx*ny][1]*   xd *   yd *   zd;

  a[2] = value[index           ][2]*(1-xd)*(1-yd)*(1-zd)
       + value[index+1         ][2]*   xd *(1-yd)*(1-zd)
       + value[index+nx        ][2]*(1-xd)*   yd *(1-zd)
       + value[index+1+nx      ][2]*   xd *   yd *(1-zd)
       + value[index+nx*ny     ][2]*(1-xd)*(1-yd)*   zd
       + value[index+1+nx*ny   ][2]*   xd *(1-yd)*   zd
       + value[index+nx+nx*ny  ][2]*(1-xd)*   yd *   zd
       + value[index+1+nx+nx*ny][2]*   xd *   yd *   zd;

}