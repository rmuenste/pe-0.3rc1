//=================================================================================================
/*!
 *  \file simple_test.cpp
 *  \brief Simplified domain decomposition test
 */
//=================================================================================================

#include <pe/engine.h>
#include <pe/support.h>
#include <pe/interface/decompose.h>
#include <iostream>

#if HAVE_MPI
#include <mpi.h>
#endif

using namespace pe;

int main(int argc, char* argv[]) {
    
#if HAVE_MPI
    // Initialize MPI
    MPI_Init(&argc, &argv);
    
    int rank, totalProcs;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &totalProcs);
    
    std::cout << "===============================================" << std::endl;
    std::cout << "  DOMAIN DECOMPOSITION SIMPLE TEST" << std::endl;
    std::cout << "===============================================" << std::endl;
    
    if (rank == 0) {
        std::cout << "Running with " << totalProcs << " processes" << std::endl;
    }
    
    // Determine grid dimensions
    int dims[3] = {0, 0, 0};
    MPI_Dims_create(totalProcs, 3, dims);
    
    if (rank == 0) {
        std::cout << "Process grid: " << dims[0] << "x" << dims[1] << "x" << dims[2] << std::endl;
    }
    
    // Create Cartesian communicator
    int periods[3] = {0, 0, 0}; // Non-periodic
    MPI_Comm cartComm;
    MPI_Cart_create(MPI_COMM_WORLD, 3, dims, periods, 1, &cartComm);
    
    // Get process coordinates
    int processCoords[3];
    MPI_Cart_coords(cartComm, rank, 3, processCoords);
    
    // Initialize the world
    WorldID world = theWorld();
    
    // Domain parameters
    real bx = 0.0, by = 0.0, bz = 0.0;
    real dx = 1.0 / dims[0];
    real dy = 1.0 / dims[1]; 
    real dz = 1.0 / dims[2];
    
    std::cout << "Rank " << rank << " coordinates: (" 
              << processCoords[0] << "," << processCoords[1] << "," << processCoords[2] << ")" << std::endl;
    
    // Setup domain decomposition
    decomposeDomain(processCoords, bx, by, bz, dx, dy, dz, dims[0], dims[1], dims[2]);
    
    // Calculate expected domain boundaries
    real xmin = bx + processCoords[0] * dx;
    real xmax = bx + (processCoords[0] + 1) * dx;
    real ymin = by + processCoords[1] * dy;
    real ymax = by + (processCoords[1] + 1) * dy;
    real zmin = bz + processCoords[2] * dz;
    real zmax = bz + (processCoords[2] + 1) * dz;
    
    std::cout << "Rank " << rank << " expected domain: [" 
              << xmin << "," << xmax << "] x [" 
              << ymin << "," << ymax << "] x [" 
              << zmin << "," << zmax << "]" << std::endl;
    
    // Test center point (should be owned by this process)
    real centerX = (xmin + xmax) * 0.5;
    real centerY = (ymin + ymax) * 0.5;
    real centerZ = (zmin + zmax) * 0.5;
    
    bool ownsCenter = world->ownsPoint(centerX, centerY, centerZ);
    
    std::cout << "Rank " << rank << " center point (" 
              << centerX << "," << centerY << "," << centerZ << ") -> "
              << (ownsCenter ? "OWNED" : "NOT OWNED") << std::endl;
    
    MPI_Barrier(MPI_COMM_WORLD);
    
    if (rank == 0) {
        std::cout << "===============================================" << std::endl;
        std::cout << "  Test completed" << std::endl;
        std::cout << "===============================================" << std::endl;
    }
    
    MPI_Finalize();
    
    return 0;
    
#else
    std::cout << "ERROR: This test requires MPI support" << std::endl;
    return 1;
#endif
}