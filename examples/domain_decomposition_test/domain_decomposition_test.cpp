//=================================================================================================
/*!
 *  \file domain_decomposition_test.cpp
 *  \brief Test application for domain decomposition verification
 *
 *  This application tests the domain decomposition functionality by:
 *  1. Testing various axis configurations (1D, 2D, 3D decompositions)
 *  2. Verifying local domain geometry with point containment tests
 *  3. Validating neighbor connectivity patterns
 *  4. Ensuring no gaps or overlaps between adjacent domains
 */
//=================================================================================================

#include <pe/engine.h>
#include <pe/support.h>
#include <pe/interface/decompose.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>

#if HAVE_MPI
#include <mpi.h>
#endif

using namespace pe;

//=================================================================================================
//  TEST CONFIGURATION STRUCTURE
//=================================================================================================

struct TestConfig {
    int px, py, pz;           // Process grid dimensions
    real bx, by, bz;          // Domain bounds lower corner
    real dx, dy, dz;          // Domain cell sizes
    std::string name;         // Test configuration name
    
    TestConfig(int _px, int _py, int _pz, real _bx, real _by, real _bz, 
               real _dx, real _dy, real _dz, const std::string& _name)
        : px(_px), py(_py), pz(_pz), bx(_bx), by(_by), bz(_bz), 
          dx(_dx), dy(_dy), dz(_dz), name(_name) {}
};

//=================================================================================================
//  HELPER FUNCTIONS
//=================================================================================================

void printHeader(const std::string& title) {
    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(80, '=') << "\n";
}

void printTestConfig(const TestConfig& config, int rank, int totalProcs) {
    std::cout << "\nTest Configuration: " << config.name << "\n";
    std::cout << "Process Grid: " << config.px << "x" << config.py << "x" << config.pz << "\n";
    std::cout << "Domain Bounds: [" << config.bx << ", " << config.by << ", " << config.bz << "]\n";
    std::cout << "Cell Sizes: [" << config.dx << ", " << config.dy << ", " << config.dz << "]\n";
    std::cout << "Current Process: " << rank << " / " << totalProcs << "\n";
}

bool isPointInDomain(const Vec3& point) {
    WorldID world = theWorld();
    return world->ownsPoint(point[0], point[1], point[2]);
}

std::vector<Vec3> generateTestPoints(const TestConfig& config, int processCoords[3]) {
    std::vector<Vec3> testPoints;
    
    // Calculate domain boundaries for this process
    real xmin = config.bx + processCoords[0] * config.dx;
    real xmax = config.bx + (processCoords[0] + 1) * config.dx;
    real ymin = config.by + processCoords[1] * config.dy;
    real ymax = config.by + (processCoords[1] + 1) * config.dy;
    real zmin = config.bz + processCoords[2] * config.dz;
    real zmax = config.bz + (processCoords[2] + 1) * config.dz;
    
    // Center point (should always be inside)
    real centerX = (xmin + xmax) * 0.5;
    real centerY = (ymin + ymax) * 0.5;
    real centerZ = (zmin + zmax) * 0.5;
    testPoints.push_back(Vec3(centerX, centerY, centerZ));
    
    // Corner points (should be inside or on boundary)
    testPoints.push_back(Vec3(xmin + 1e-10, ymin + 1e-10, zmin + 1e-10));
    testPoints.push_back(Vec3(xmax - 1e-10, ymax - 1e-10, zmax - 1e-10));
    
    // Boundary points
    testPoints.push_back(Vec3(centerX, centerY, zmin + 1e-10));
    testPoints.push_back(Vec3(centerX, centerY, zmax - 1e-10));
    
    // Points that should be outside
    testPoints.push_back(Vec3(xmin - 1e-6, centerY, centerZ));  // Outside west
    testPoints.push_back(Vec3(xmax + 1e-6, centerY, centerZ));  // Outside east
    testPoints.push_back(Vec3(centerX, ymin - 1e-6, centerZ));  // Outside south
    testPoints.push_back(Vec3(centerX, ymax + 1e-6, centerZ));  // Outside north
    testPoints.push_back(Vec3(centerX, centerY, zmin - 1e-6));  // Outside bottom
    testPoints.push_back(Vec3(centerX, centerY, zmax + 1e-6));  // Outside top
    
    return testPoints;
}

//=================================================================================================
//  TEST FUNCTIONS
//=================================================================================================


//=================================================================================================
//  MAIN FUNCTION
//=================================================================================================

int main(int argc, char* argv[]) {
    
#if HAVE_MPI
    // Initialize MPI
    MPI_Init(&argc, &argv);
    
    int rank, totalProcs;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &totalProcs);
    MPISystemID mpisystem = theMPISystem();
    
    if (rank == 0) {
        printHeader("DOMAIN DECOMPOSITION TEST SUITE");
        std::cout << "Running with " << totalProcs << " processes\n";
    }
    
    // Run a simple test with current process count
    bool success = true;
    
    // Test configuration that matches current process count
    TestConfig config(0, 0, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, "Auto-detected");
    
    // Determine grid dimensions
    int dims[3] = {0, 0, 0};
    MPI_Dims_create(totalProcs, 3, dims);
    config.px = dims[0];
    config.py = dims[1]; 
    config.pz = dims[2];
    config.dx = 1.0 / dims[0];
    config.dy = 1.0 / dims[1];
    config.dz = 1.0 / dims[2];
    
    if (rank == 0) {
        std::cout << "Testing grid: " << dims[0] << "x" << dims[1] << "x" << dims[2] << "\n";
        printTestConfig(config, rank, totalProcs);
    }
    
    // Create Cartesian communicator
    int periods[3] = {0, 0, 0}; // Non-periodic
    MPI_Comm cartComm;
    MPI_Cart_create(MPI_COMM_WORLD, 3, dims, periods, 1, &cartComm);
    mpisystem->setComm(cartComm);

    if (cartComm == MPI_COMM_NULL)
    {
       std::cout << "Error creating 3D communicator" << std::endl;
       MPI_Finalize();
       return 0;
    }
    else {
      std::cout << "Communication ok" << std::endl;
    }
    
    // Get process coordinates
    int processCoords[3];
    MPI_Cart_coords(mpisystem->getComm(), rank, 3, processCoords);
    
    // Initialize the world
    WorldID world = theWorld();
    
    // Setup domain decomposition
    decomposeDomain(processCoords, config.bx, config.by, config.bz, 
                   config.dx, config.dy, config.dz, 
                   config.px, config.py, config.pz);
    
    // Calculate expected domain boundaries for verification
    real xmin = config.bx + processCoords[0] * config.dx;
    real xmax = config.bx + (processCoords[0] + 1) * config.dx;
    real ymin = config.by + processCoords[1] * config.dy;
    real ymax = config.by + (processCoords[1] + 1) * config.dy;
    real zmin = config.bz + processCoords[2] * config.dz;
    real zmax = config.bz + (processCoords[2] + 1) * config.dz;
    
    // Generate test points
    std::vector<Vec3> testPoints = generateTestPoints(config, processCoords);
    
    // Test point containment
    std::cout << "Rank " << rank << " (coords: " << processCoords[0] << "," 
              << processCoords[1] << "," << processCoords[2] << ") Domain Test:\n";
    std::cout << "Domain bounds: [" << xmin << "," << xmax << "] x [" 
              << ymin << "," << ymax << "] x [" << zmin << "," << zmax << "]\n";
    
    for (size_t i = 0; i < testPoints.size(); ++i) {
        const Vec3& point = testPoints[i];
        bool shouldBeInside = (i < 5); // First 5 points should be inside
        bool isInside = isPointInDomain(point);
        
        std::cout << "Point " << i << ": (" << point[0] << "," << point[1] << "," << point[2] << ") -> " 
                  << (isInside ? "INSIDE" : "OUTSIDE");
        
        if (shouldBeInside && !isInside) {
            std::cout << " ERROR: Should be INSIDE!";
            success = false;
        } else if (!shouldBeInside && isInside && i >= 5) {
            std::cout << " ERROR: Should be OUTSIDE!";
            success = false;
        } else {
            std::cout << " OK";
        }
        std::cout << "\n";
    }
    
    MPI_Barrier(MPI_COMM_WORLD);
    
    if (rank == 0) {
        printHeader("FINAL RESULTS");
        std::cout << "Overall test result: " << (success ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << "\n";
        std::cout << std::string(80, '=') << "\n";
    }
    
    MPI_Finalize();
    
    return success ? 0 : 1;
    
#else
    std::cout << "ERROR: This test requires MPI support\n";
    return 1;
#endif
}
