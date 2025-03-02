//=================================================================================================
/*!
 *  \file rectigrid_demo.cpp
 *  \brief Example demonstrating RectilinearGrid domain decomposition
 *
 *  This example creates a 3x3x3 domain decomposition with a sphere in the middle.
 *  It uses the HardContactSemiImplicitTimesteppingSolver with HashGrid detection.
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

using namespace pe;
using namespace pe::timing;
using namespace pe::povray;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the rectigrid_demo example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * This example demonstrates the use of RectilinearGrid for domain decomposition with a 3x3x3 grid
 * and places a sphere in the middle of the domain to observe gravity effects.
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

   // Setup of the domain dimensions [0,1]x[0,1]x[0,1]
   const real xmin(0.0), xmax(1.0);
   const real ymin(0.0), ymax(1.0);
   const real zmin(0.0), zmax(1.0);
   
   // Domain decomposition - 3x3x3 grid
   const int px(3);
   const int py(3);
   const int pz(3);

   /////////////////////////////////////////////////////
   // Initial setups

   // Creates the material with the following properties:
   // - material density               : 1.0
   // - coefficient of restitution     : 0.8 
   // - coefficient of static friction : 0.1
   // - coefficient of dynamic friction: 0.05
   // - Poisson's ratio                : 0.3
   // - Young's modulus                : 200
   // - Contact stiffness              : 1e6
   // - dampingN                       : 1e5
   // - dampingT                       : 2e5
   MaterialID material = createMaterial("material", 1.0, 0.8, 0.1, 0.05, 0.3, 200, 1e6, 1e5, 2e5);

   // Set up world and MPI system
   WorldID     world     = theWorld();
   world->setGravity(0.0, 0.0, -9.81); // Standard gravity
   world->setDamping(0.02);
   MPISystemID mpisystem = theMPISystem();

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D RectilinearGrid Domain Decomposition
   
   // Setup for RectilinearGrid
   int dims[]    = {px, py, pz};
   int periods[] = {false, false, false};
   MPI_Comm cartcomm;

   // Create Cartesian communicator
   MPI_Cart_create(MPI_COMM_WORLD, 3, dims, periods, false, &cartcomm);
   if (cartcomm == MPI_COMM_NULL) {
      MPI_Finalize();
      return 0;
   }

   mpisystem->setComm(cartcomm);
   
   // Using RectilinearGrid for domain decomposition
   RectilinearGrid grid;
   //grid.createGrid(px, py, pz, false, false, false);
   //grid.setBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
   //grid.setupGrid();
   
   // Setup of the VTK visualization
   if (vtk) {
      vtk::WriterID vtkw = vtk::activateWriter("./paraview", visspacing, 0, timesteps, false);
   }

   /////////////////////////////////////////////////////
   // Create a sphere in the middle of the domain
   int idx = 0;
   const real sphereRadius = 0.05;
   const Vec3 position(0.5, 0.5, 0.5); // Center of the domain
   
   // Only create the sphere if this process owns the center point
   if (world->ownsPoint(position)) {
      SphereID sphere = createSphere(idx, position, sphereRadius, material, true);
      idx++;
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
                << " Total number of MPI processes = " << px * py * pz << "\n"
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