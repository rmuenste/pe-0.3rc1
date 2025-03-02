//=================================================================================================
/*!
 *  \file trimeshdop_demo.cpp
 *  \brief Example demonstrating TriMeshDopBoundary for domain decomposition
 *
 *  This example demonstrates the use of TriMeshDopBoundary for domain decomposition
 *  by loading OBJ files for process boundaries and simulating particles within these domains.
 */
//=================================================================================================

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <iostream>
#include <pe/vtk.h>
#include <pe/core/domaindecomp/TriMeshDopBoundary.h>

using namespace pe;
using namespace pe::timing;
using namespace pe::povray;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the trimeshdop_demo example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * This example demonstrates the use of TriMeshDopBoundary for domain decomposition by
 * creating a 2x2x1 domain with TriMeshDopBoundary definitions from OBJ files.
 */
int main(int argc, char** argv)
{
   /////////////////////////////////////////////////////
   // MPI Initialization
   MPI_Init(&argc, &argv);

   /////////////////////////////////////////////////////
   // Simulation parameters
   const size_t timesteps(1000);  // Total number of time steps
   const real   stepsize(0.005);   // Size of a single time step
   const bool   vtk(true);         // Enable VTK visualization
   const size_t visspacing(10);    // Visualization output every 10 steps

   // Setup of the domain dimensions
   const real xmin(-1.0), xmax(1.0);
   const real ymin(-1.0), ymax(1.0);
   const real zmin(-0.5), zmax(0.5);
   
   /////////////////////////////////////////////////////
   // Initial setups
   
   // Creates the material with the following properties
   MaterialID material = createMaterial("material", 1.0, 0.8, 0.1, 0.05, 0.3, 200, 1e6, 1e5, 2e5);

   // Set up world and MPI system
   WorldID     world     = theWorld();
   world->setGravity(0.0, 0.0, -9.81); // Standard gravity
   world->setDamping(0.02);
   MPISystemID mpisystem = theMPISystem();

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 2x2x1 Domain Decomposition
   
   // Create Cartesian communicator
   int dims[]    = {2, 2, 1};
   int periods[] = {false, false, false};
   MPI_Comm cartcomm;

   MPI_Cart_create(MPI_COMM_WORLD, 3, dims, periods, false, &cartcomm);
   if (cartcomm == MPI_COMM_NULL) {
      MPI_Finalize();
      return 0;
   }

   mpisystem->setComm(cartcomm);
   
   // Get the coordinates of this process in the cartesian grid
   int coords[3];
   MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, coords);
   
   pe_LOG_INFO_SECTION(log) {
      log << "Process " << mpisystem->getRank() 
          << " at coordinates (" << coords[0] << ", " << coords[1] << ", " << coords[2] << ")\n";
   }
   
   // Define domain decomposition using TriMeshDopBoundary
   
   // For demonstration, we're using a simple cube OBJ file
   // In a real application, you would use more complex geometries
   std::string objFile = "cube.obj";
   
   // Create the local domain boundary based on the process coordinates
   // Note: For simplicity, we're using cubes for all processes, but in a real
   // application each process could have a different geometry
   
   real scaleX = (xmax - xmin) / dims[0];
   real scaleY = (ymax - ymin) / dims[1];
   real scaleZ = (zmax - zmin) / dims[2];
   
   real posX = xmin + (coords[0] + 0.5) * scaleX;
   real posY = ymin + (coords[1] + 0.5) * scaleY;
   real posZ = zmin + (coords[2] + 0.5) * scaleZ;
   
   // Set the material properties for the boundary
   MaterialID boundaryMaterial = createMaterial("boundaryMaterial", 10.0, 0.2, 0.5, 0.3, 0.2, 100, 1e7, 1e6, 1e6);

   // Create our local domain using a TriMeshDopBoundary
   TriMeshDopBoundary localDomain(
      objFile,  // Load from OBJ file
      false,    // Not clockwise
      false     // Not left-handed
   );
   
   // Define the local domain for this process
   defineLocalDomain(localDomain);
   
   // Connect to neighboring processes
   int neighbor, neighborCoords[3];
   
   // Check neighboring processes in all directions
   for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
         for (int dz = -1; dz <= 1; ++dz) {
            // Skip ourselves
            if (dx == 0 && dy == 0 && dz == 0) {
               continue;
            }
            
            // Calculate neighbor coordinates
            neighborCoords[0] = coords[0] + dx;
            neighborCoords[1] = coords[1] + dy;
            neighborCoords[2] = coords[2] + dz;
            
            // Skip if outside the grid
            if (neighborCoords[0] < 0 || neighborCoords[0] >= dims[0] ||
                neighborCoords[1] < 0 || neighborCoords[1] >= dims[1] ||
                neighborCoords[2] < 0 || neighborCoords[2] >= dims[2]) {
               continue;
            }
            
            // Get the rank of the neighbor
            MPI_Cart_rank(cartcomm, neighborCoords, &neighbor);
            
            // Create a TriMeshDopBoundary for the neighbor
            real neighborPosX = xmin + (neighborCoords[0] + 0.5) * scaleX;
            real neighborPosY = ymin + (neighborCoords[1] + 0.5) * scaleY;
            real neighborPosZ = zmin + (neighborCoords[2] + 0.5) * scaleZ;
            
            TriMeshDopBoundary neighborDomain(
               objFile,  // Load from OBJ file
               false,    // Not clockwise
               false     // Not left-handed
            );
            
            // Connect to the neighbor
            connect(neighbor, neighborDomain);
            
            pe_LOG_INFO_SECTION(log) {
               log << "Process " << mpisystem->getRank() 
                   << " connected to neighbor " << neighbor 
                   << " at coordinates (" << neighborCoords[0] << ", " 
                   << neighborCoords[1] << ", " << neighborCoords[2] << ")\n";
            }
         }
      }
   }
   
   // Setup of the VTK visualization
   if (vtk) {
      vtk::WriterID vtkw = vtk::activateWriter("./paraview", visspacing, 0, timesteps, false);
   }

   /////////////////////////////////////////////////////
   // Create spheres in the local domain
   int idx = 0;
   const real sphereRadius = 0.05;
   
   // Create spheres at random positions in the local domain
   for (int i = 0; i < 10; ++i) {
      // Generate a random position within the local domain bounds
      real x = posX + (rand<real>() - 0.5) * 0.8 * scaleX;
      real y = posY + (rand<real>() - 0.5) * 0.8 * scaleY;
      real z = posZ + (rand<real>() - 0.5) * 0.8 * scaleZ;
      
      Vec3 position(x, y, z);
      
      // Only create the sphere if this process owns the point
      if (world->ownsPoint(position)) {
         SphereID sphere = createSphere(idx, position, sphereRadius, material, true);
         idx++;
      }
   }

   // Configure collision detection
   theCollisionSystem()->setErrorReductionParameter(0.2);
   
   // Synchronize all processes
   world->synchronize();

   /////////////////////////////////////////////////////
   // Count total number of bodies
   unsigned long particlesTotal(0);
   unsigned int localBodies = static_cast<unsigned int>(theCollisionSystem()->getBodyStorage().size());
   MPI_Reduce(&localBodies, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

   pe_EXCLUSIVE_SECTION(0) {
      std::cout << "\n-- SIMULATION SETUP ----------------------------------\n"
                << " Total number of MPI processes = " << dims[0] * dims[1] * dims[2] << "\n"
                << " Total bodies                 = " << particlesTotal << "\n"
                << " Timesteps                    = " << timesteps << "\n"
                << " Timestep size                = " << stepsize << "\n"
                << "----------------------------------------------------\n" << std::endl;
   }

   /////////////////////////////////////////////////////
   // Simulation loop
   WcTimer simTime;
   simTime.start();

   for (unsigned int timestep = 0; timestep <= timesteps; ++timestep) {
      pe_EXCLUSIVE_SECTION(0) {
         std::cout << "\r Time step " << timestep << " of " << timesteps << "   " << std::flush;
      }
      world->simulationStep(stepsize);
   }

   simTime.end();

   /////////////////////////////////////////////////////
   // Simulation timing results
   double localTime(simTime.total());
   double globalMin(0.0);
   double globalMax(0.0);
   MPI_Reduce(&localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm);
   MPI_Reduce(&localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm);

   pe_EXCLUSIVE_SECTION(0) {
      std::cout << "\n-- SIMULATION RESULTS -------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "----------------------------------------------------\n" << std::endl;
   }

   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Finalize();
   return 0;
}
//*************************************************************************************************