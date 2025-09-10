//******************************************************************************************
//
// This example performs containment and distance field analysis on a structured 3D grid
// using PE's TriangleMesh with DistanceMap acceleration. It provides an extensible
// framework for various calculations on regular grid points.
//
//******************************************************************************************

#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <functional>
#include <fstream>
#include <cassert>
#include <chrono>
#include <iomanip>

// Local headers
#include <pe/core/detection/fine/DistanceMap.h>
#include <pe/vtk/UtilityWriters.h>

// PE headers
#include <pe/core.h>
#include <pe/math.h>
#include <pe/core/rigidbody/TriangleMesh.h>

// Using pe namespace
using namespace pe;

// Configuration structure
struct GridConfig {
    // Mesh and DistanceMap settings
    std::string meshFile;
    pe::Vec3 meshPosition = pe::Vec3(0, 0, 0);  // Mesh position in world coordinates
    pe::real dmSpacing = 0.05;
    int dmResolution = 64;
    int dmTolerance = 3;
    
    // Grid specification
    pe::Vec3 gridOrigin = pe::Vec3(0, 0, 0);
    pe::Vec3 gridSpacing = pe::Vec3(0.1, 0.1, 0.1);
    std::array<int, 3> gridSize = {50, 50, 50};
    bool autoOrigin = true;  // Auto-center grid on mesh
    
    // Output settings
    std::string outputFile = "grid_results";
    
    // Calculation flags
    bool calcContainment = true;   // Always enabled
    bool calcDistance = false;
    bool calcNormal = false;
    bool calcContact = false;
    
    // Export flags
    bool exportDistanceMap = false;  // Export the DistanceMap grid itself
    std::string distanceMapFile = "distance_map";
    
    bool verbose = false;
};

// Calculation framework
struct GridCalculation {
    std::string name;
    std::function<std::vector<pe::real>(const DistanceMap*, const pe::Vec3&)> calculate;
    int components;  // 1 for scalar, 3 for vector
    std::string description;
};

// Generate structured 3D grid points
std::vector<pe::Vec3> generateStructuredGrid(const GridConfig& config) {
    std::vector<pe::Vec3> gridPoints;
    const auto& origin = config.gridOrigin;
    const auto& spacing = config.gridSpacing;
    const auto& size = config.gridSize;
    
    int totalPoints = size[0] * size[1] * size[2];
    gridPoints.reserve(totalPoints);
    
    if (config.verbose) {
        std::cout << "Generating structured grid:" << std::endl;
        std::cout << "  Origin: (" << origin[0] << ", " << origin[1] << ", " << origin[2] << ")" << std::endl;
        std::cout << "  Spacing: (" << spacing[0] << ", " << spacing[1] << ", " << spacing[2] << ")" << std::endl;
        std::cout << "  Dimensions: " << size[0] << " x " << size[1] << " x " << size[2] << std::endl;
        std::cout << "  Total points: " << totalPoints << std::endl;
    }
    
    for (int k = 0; k < size[2]; ++k) {
        for (int j = 0; j < size[1]; ++j) {
            for (int i = 0; i < size[0]; ++i) {
                pe::Vec3 point(
                    origin[0] + i * spacing[0],
                    origin[1] + j * spacing[1],
                    origin[2] + k * spacing[2]
                );
                gridPoints.push_back(point);
            }
        }
    }
    
    return gridPoints;
}

// Setup calculation functions
std::vector<GridCalculation> setupCalculations(const GridConfig& config, const TriangleMeshID& mesh) {
    std::vector<GridCalculation> calculations;
    
    // Containment (always included)
    calculations.push_back({
        "Containment",
        [mesh](const DistanceMap* dm, const pe::Vec3& point) -> std::vector<pe::real> {
            // Use mesh->containsPoint() for proper coordinate transformation
            bool inside = mesh->containsPoint(point);
            return {inside ? 1.0 : 0.0};
        },
        1,
        "Point containment (1=inside, 0=outside)"
    });
    
    if (config.calcDistance) {
        calculations.push_back({
            "SignedDistance",
            [mesh](const DistanceMap* dm, const pe::Vec3& point) -> std::vector<pe::real> {
                // Transform to local coordinates before DistanceMap query
                pe::Vec3 localPoint = mesh->pointFromWFtoBF(point);
                return {dm->interpolateDistance(localPoint[0], localPoint[1], localPoint[2])};
            },
            1,
            "Signed distance to surface (negative=inside, positive=outside)"
        });
    }
    
    if (config.calcNormal) {
        calculations.push_back({
            "Normal",
            [mesh](const DistanceMap* dm, const pe::Vec3& point) -> std::vector<pe::real> {
                // Transform to local coordinates before DistanceMap query
                pe::Vec3 localPoint = mesh->pointFromWFtoBF(point);
                pe::Vec3 normal = dm->interpolateNormal(localPoint[0], localPoint[1], localPoint[2]);
                // Transform normal back to world coordinates
                pe::Vec3 worldNormal = mesh->vectorFromBFtoWF(normal);
                return {worldNormal[0], worldNormal[1], worldNormal[2]};
            },
            3,
            "Surface normal vectors"
        });
    }
    
    if (config.calcContact) {
        calculations.push_back({
            "ContactPoint",
            [mesh](const DistanceMap* dm, const pe::Vec3& point) -> std::vector<pe::real> {
                // Transform to local coordinates before DistanceMap query
                pe::Vec3 localPoint = mesh->pointFromWFtoBF(point);
                pe::Vec3 contact = dm->interpolateContactPoint(localPoint[0], localPoint[1], localPoint[2]);
                // Transform contact point back to world coordinates
                pe::Vec3 worldContact = mesh->pointFromBFtoWF(contact);
                return {worldContact[0], worldContact[1], worldContact[2]};
            },
            3,
            "Closest contact points on surface"
        });
    }
    
    return calculations;
}

// Calculate grid origin automatically from mesh bounding box
pe::Vec3 calculateAutoOrigin(const TriangleMeshID& mesh, const GridConfig& config) {
    const auto& bbox = mesh->getAABB();
    pe::real minX = bbox[3], maxX = bbox[0];  // PE AABB format: [maxX, maxY, maxZ, minX, minY, minZ]
    pe::real minY = bbox[4], maxY = bbox[1];
    pe::real minZ = bbox[5], maxZ = bbox[2];
    
    // Calculate grid bounds
    pe::real gridWidth = (config.gridSize[0] - 1) * config.gridSpacing[0];
    pe::real gridHeight = (config.gridSize[1] - 1) * config.gridSpacing[1];
    pe::real gridDepth = (config.gridSize[2] - 1) * config.gridSpacing[2];
    
    // Center grid on mesh with small margin
    pe::real margin = 0.1;
    pe::real centerX = (minX + maxX) * 0.5;
    pe::real centerY = (minY + maxY) * 0.5;
    pe::real centerZ = (minZ + maxZ) * 0.5;
    
    return pe::Vec3(
        centerX - gridWidth * 0.5,
        centerY - gridHeight * 0.5,
        centerZ - gridDepth * 0.5
    );
}

// Perform grid calculations
void performGridCalculations(const TriangleMeshID& mesh, const GridConfig& config) {
    std::cout << "\n=== Starting Grid Analysis ===" << std::endl;
    
    // Generate grid points
    GridConfig gridConfig = config;
    if (config.autoOrigin) {
        gridConfig.gridOrigin = calculateAutoOrigin(mesh, config);
        if (config.verbose) {
            std::cout << "Auto-calculated grid origin: (" 
                     << gridConfig.gridOrigin[0] << ", " 
                     << gridConfig.gridOrigin[1] << ", " 
                     << gridConfig.gridOrigin[2] << ")" << std::endl;
        }
    }
    
    std::vector<pe::Vec3> gridPoints = generateStructuredGrid(gridConfig);
    
    // Setup calculations
    std::vector<GridCalculation> calculations = setupCalculations(config, mesh);
    
    std::cout << "Enabled calculations:" << std::endl;
    for (const auto& calc : calculations) {
        std::cout << "  - " << calc.name << ": " << calc.description << std::endl;
    }
    
    // Get DistanceMap
    const DistanceMap* dm = mesh->getDistanceMap();
    if (!dm) {
        throw std::runtime_error("DistanceMap not available on mesh");
    }
    
    // Perform calculations
    std::vector<std::vector<std::vector<pe::real>>> results(calculations.size());
    std::vector<double> timings(calculations.size());
    
    for (size_t calcIdx = 0; calcIdx < calculations.size(); ++calcIdx) {
        const auto& calc = calculations[calcIdx];
        results[calcIdx].reserve(gridPoints.size());
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (const auto& point : gridPoints) {
            std::vector<pe::real> result = calc.calculate(dm, point);
            results[calcIdx].push_back(result);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        timings[calcIdx] = std::chrono::duration<double, std::milli>(end - start).count();
        
        std::cout << "Completed " << calc.name << " calculation in " 
                 << std::fixed << std::setprecision(2) << timings[calcIdx] << " ms" << std::endl;
    }
    
    // Export results to VTK
    std::string vtiFile = config.outputFile + ".vti";
    std::cout << "\nExporting results to " << vtiFile << "..." << std::endl;
    
    // Prepare data for VTI export (flattened)
    std::vector<pe::real> containmentData;
    std::vector<pe::real> distanceData;
    std::vector<pe::Vec3> normalData;
    std::vector<pe::Vec3> contactData;
    
    containmentData.reserve(gridPoints.size());
    if (config.calcDistance) distanceData.reserve(gridPoints.size());
    if (config.calcNormal) normalData.reserve(gridPoints.size());
    if (config.calcContact) contactData.reserve(gridPoints.size());
    
    for (size_t i = 0; i < gridPoints.size(); ++i) {
        // Containment (always first)
        containmentData.push_back(results[0][i][0]);
        
        // Other calculations
        size_t calcIdx = 1;
        if (config.calcDistance && calcIdx < results.size()) {
            distanceData.push_back(results[calcIdx][i][0]);
            calcIdx++;
        }
        if (config.calcNormal && calcIdx < results.size()) {
            const auto& normal = results[calcIdx][i];
            normalData.emplace_back(normal[0], normal[1], normal[2]);
            calcIdx++;
        }
        if (config.calcContact && calcIdx < results.size()) {
            const auto& contact = results[calcIdx][i];
            contactData.emplace_back(contact[0], contact[1], contact[2]);
            calcIdx++;
        }
    }
    
    // Fill missing data with defaults if needed
    if (!config.calcDistance) {
        distanceData.resize(gridPoints.size(), 0.0);
    }
    if (!config.calcNormal) {
        normalData.resize(gridPoints.size(), pe::Vec3(0, 0, 1));
    }
    if (!config.calcContact) {
        contactData.resize(gridPoints.size(), pe::Vec3(0, 0, 0));
    }
    
    // Write VTI file
    pe::Vec3 origin(gridConfig.gridOrigin[0], gridConfig.gridOrigin[1], gridConfig.gridOrigin[2]);
    pe::Vec3 spacing(config.gridSpacing[0], config.gridSpacing[1], config.gridSpacing[2]);
    
    // Prepare scalar fields
    pe::vtk::DistanceMapWriter::ScalarFieldMap scalarFields;
    if (config.calcDistance) {
        scalarFields["distance"] = distanceData;
    } else {
        scalarFields["containment"] = containmentData;
    }
    
    // Convert containment to alpha field
    std::vector<pe::real> alphaData(containmentData.begin(), containmentData.end());
    scalarFields["alpha"] = alphaData;
    
    // Prepare vector fields  
    pe::vtk::DistanceMapWriter::VectorFieldMap vectorFields;
    if (config.calcNormal) {
        vectorFields["normal"] = normalData;
    }
    if (config.calcContact) {
        vectorFields["contact_point"] = contactData;
    }
    
    pe::vtk::DistanceMapWriter::writeCustomGrid(vtiFile, origin, spacing,
                                               config.gridSize[0], config.gridSize[1], config.gridSize[2],
                                               scalarFields, vectorFields);
    
    std::cout << "VTK export completed successfully!" << std::endl;
    
    // Performance summary
    std::cout << "\n=== Performance Summary ===" << std::endl;
    std::cout << "Grid points processed: " << gridPoints.size() << std::endl;
    double totalTime = 0;
    for (size_t i = 0; i < calculations.size(); ++i) {
        totalTime += timings[i];
        std::cout << calculations[i].name << ": " << timings[i] << " ms ("
                 << (timings[i] / gridPoints.size()) << " ms/point)" << std::endl;
    }
    std::cout << "Total calculation time: " << totalTime << " ms" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "PE TriangleMesh Grid Analysis with DistanceMap!" << std::endl;
    
#ifdef PE_USE_CGAL
    GridConfig config;
    
    // Parse command line arguments
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <mesh_file.obj> [options]" << std::endl;
        std::cout << "  mesh_file.obj: Input mesh file (.obj or .off format)" << std::endl;
        std::cout << "Mesh Options:" << std::endl;
        std::cout << "  --mesh-position <x> <y> <z>   : Mesh position in world coordinates (default: 0 0 0)" << std::endl;
        std::cout << "Grid Options:" << std::endl;
        std::cout << "  --grid-origin <x> <y> <z>     : Grid origin coordinates (default: auto-center)" << std::endl;
        std::cout << "  --grid-spacing <dx> <dy> <dz> : Grid spacing per axis (default: 0.1 0.1 0.1)" << std::endl;
        std::cout << "  --grid-size <nx> <ny> <nz>    : Grid dimensions (default: 50 50 50)" << std::endl;
        std::cout << "DistanceMap Options:" << std::endl;
        std::cout << "  --dm-spacing <value>          : DistanceMap spacing (default: 0.05)" << std::endl;
        std::cout << "  --dm-resolution <value>       : DistanceMap resolution (default: 64)" << std::endl;
        std::cout << "  --dm-tolerance <value>        : DistanceMap tolerance (default: 3)" << std::endl;
        std::cout << "Calculation Options:" << std::endl;
        std::cout << "  --calc-distance               : Calculate signed distance values" << std::endl;
        std::cout << "  --calc-normal                 : Calculate surface normals" << std::endl;
        std::cout << "  --calc-contact                : Calculate contact points" << std::endl;
        std::cout << "Output Options:" << std::endl;
        std::cout << "  --output <filename>           : Output VTK filename (default: grid_results.vti)" << std::endl;
        std::cout << "  --export-distance-map [file]  : Export DistanceMap grid to VTK (default: distance_map.vti)" << std::endl;
        std::cout << "  --verbose                     : Enable verbose output" << std::endl;
        return 1;
    }
    
    config.meshFile = argv[1];
    
    // Parse optional arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--mesh-position" && i + 3 < argc) {
            config.meshPosition[0] = std::stod(argv[++i]);
            config.meshPosition[1] = std::stod(argv[++i]);
            config.meshPosition[2] = std::stod(argv[++i]);
        } else if (arg == "--grid-origin" && i + 3 < argc) {
            config.gridOrigin[0] = std::stod(argv[++i]);
            config.gridOrigin[1] = std::stod(argv[++i]);
            config.gridOrigin[2] = std::stod(argv[++i]);
            config.autoOrigin = false;
        } else if (arg == "--grid-spacing" && i + 3 < argc) {
            config.gridSpacing[0] = std::stod(argv[++i]);
            config.gridSpacing[1] = std::stod(argv[++i]);
            config.gridSpacing[2] = std::stod(argv[++i]);
        } else if (arg == "--grid-size" && i + 3 < argc) {
            config.gridSize[0] = std::stoi(argv[++i]);
            config.gridSize[1] = std::stoi(argv[++i]);
            config.gridSize[2] = std::stoi(argv[++i]);
        } else if (arg == "--dm-spacing" && i + 1 < argc) {
            config.dmSpacing = std::stod(argv[++i]);
        } else if (arg == "--dm-resolution" && i + 1 < argc) {
            config.dmResolution = std::stoi(argv[++i]);
        } else if (arg == "--dm-tolerance" && i + 1 < argc) {
            config.dmTolerance = std::stoi(argv[++i]);
        } else if (arg == "--calc-distance") {
            config.calcDistance = true;
        } else if (arg == "--calc-normal") {
            config.calcNormal = true;
        } else if (arg == "--calc-contact") {
            config.calcContact = true;
        } else if (arg == "--output" && i + 1 < argc) {
            config.outputFile = argv[++i];
        } else if (arg == "--export-distance-map") {
            config.exportDistanceMap = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                config.distanceMapFile = argv[++i];
            }
        } else if (arg == "--verbose") {
            config.verbose = true;
        }
    }
    
    std::cout << "\nConfiguration:" << std::endl;
    std::cout << "  Mesh file: " << config.meshFile << std::endl;
    std::cout << "  Mesh position: (" << config.meshPosition[0] << ", " << config.meshPosition[1] << ", " << config.meshPosition[2] << ")" << std::endl;
    std::cout << "  DistanceMap spacing: " << config.dmSpacing << std::endl;
    std::cout << "  DistanceMap resolution: " << config.dmResolution << std::endl;
    std::cout << "  DistanceMap tolerance: " << config.dmTolerance << std::endl;
    if (config.autoOrigin) {
        std::cout << "  Grid origin: auto-center on mesh" << std::endl;
    } else {
        std::cout << "  Grid origin: (" << config.gridOrigin[0] << ", " << config.gridOrigin[1] << ", " << config.gridOrigin[2] << ")" << std::endl;
    }
    std::cout << "  Grid spacing: (" << config.gridSpacing[0] << ", " << config.gridSpacing[1] << ", " << config.gridSpacing[2] << ")" << std::endl;
    std::cout << "  Grid size: " << config.gridSize[0] << " x " << config.gridSize[1] << " x " << config.gridSize[2] << std::endl;
    std::cout << "  Output file: " << config.outputFile << ".vti" << std::endl;
    if (config.exportDistanceMap) {
        std::cout << "  DistanceMap export: " << config.distanceMapFile << ".vti" << std::endl;
    }
    std::cout << "  Calculations: containment";
    if (config.calcDistance) std::cout << ", distance";
    if (config.calcNormal) std::cout << ", normals";
    if (config.calcContact) std::cout << ", contact-points";
    std::cout << std::endl;

    try {
        // Initialize PE world
        WorldID world = theWorld();
        MaterialID material = createMaterial("test", 1000, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
        
        // Create triangle mesh from file
        std::cout << "\nLoading mesh from: " << config.meshFile << std::endl;
        TriangleMeshID mesh = createTriangleMesh(1, config.meshPosition, config.meshFile, material, false, true);
        
        if (!mesh) {
            std::cerr << "ERROR: Failed to load mesh from " << config.meshFile << std::endl;
            return 1;
        }
        
        std::cout << "Mesh loaded successfully!" << std::endl;
        std::cout << "  Vertices: " << mesh->getWFVertices().size() << std::endl;
        std::cout << "  Faces: " << mesh->getFaceIndices().size() << std::endl;
        
        // Print bounding box info
        const auto& bbox = mesh->getAABB();
        std::cout << "  Bounding box: [" << bbox[3] << "," << bbox[0] << "] x ["
                  << bbox[4] << "," << bbox[1] << "] x ["
                  << bbox[5] << "," << bbox[2] << "]" << std::endl;
        
        // Enable DistanceMap acceleration
        std::cout << "\nEnabling DistanceMap acceleration..." << std::endl;
        mesh->enableDistanceMapAcceleration(config.dmSpacing, config.dmResolution, config.dmTolerance);
        
        if (!mesh->hasDistanceMap()) {
            std::cerr << "ERROR: DistanceMap acceleration failed to initialize" << std::endl;
            return 1;
        }
        
        std::cout << "DistanceMap acceleration enabled successfully!" << std::endl;
        
        if (const DistanceMap* dm = mesh->getDistanceMap()) {
            std::cout << "DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
            std::cout << "DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
            
            // Export DistanceMap if requested
            if (config.exportDistanceMap) {
                std::string dmVtiFile = config.distanceMapFile + ".vti";
                std::cout << "Exporting DistanceMap to " << dmVtiFile << "..." << std::endl;
                
                pe::vtk::DistanceMapWriter::writeVTI(dmVtiFile, *dm);
                
                std::cout << "DistanceMap export completed successfully!" << std::endl;
                std::cout << "Note: DistanceMap is in LOCAL mesh coordinates" << std::endl;
            }
        }
        
        // Perform grid calculations
        performGridCalculations(mesh, config);
        
        std::cout << "\n=== Analysis Complete ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
    
#else
    std::cout << "ERROR: This example requires CGAL support. Please compile with -DCGAL=ON" << std::endl;
    return 1;
#endif
}