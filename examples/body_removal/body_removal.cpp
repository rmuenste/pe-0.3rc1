//=================================================================================================
/*!
 *  \file body_removal.cpp
 *  \brief Example demonstrating body removal during simulation
 *
 *  This example shows how to use the World::destroy function to remove bodies during simulation.
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
#include <iomanip>
#include <vector>

using namespace pe;
using namespace pe::timing;

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Prints information about the current simulation state.
 *
 * \param world The world handle.
 * \param step The current simulation step.
 * \return void
 */
void printSimulationState(WorldID world, size_t step)
{
   // Print current simulation step and body count
   std::cout << "Step " << std::setw(4) << step 
             << ": Bodies = " << std::setw(3) << world->size() << std::endl;

   // Print all sphere positions (first 3 only for brevity)
   size_t count = 0;
   for (auto it = world->begin<Sphere>(); it != world->end<Sphere>() && count < 3; ++it, ++count) {
      std::cout << "  Sphere " << std::setw(2) << it->getID() << ": " 
                << std::fixed << std::setprecision(2) << it->getPosition() << std::endl;
   }

   if (world->size() > 3) {
      std::cout << "  ..." << std::endl;
   }
}

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the body_removal example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * This example demonstrates how to remove bodies during a simulation using the World::destroy
 * function. Several spheres are created, then a few simulation steps are run, after which
 * a sphere is removed, followed by more simulation steps.
 */
int main(int argc, char** argv)
{
   /////////////////////////////////////////////////////
   // Simulation parameters
   
   const size_t num_spheres = 5;       // Number of spheres to create
   const size_t initial_steps = 10;    // Number of steps before removing a body
   const size_t final_steps = 10;      // Number of steps after removing a body
   const real stepsize = 0.01;         // Size of a single time step

   /////////////////////////////////////////////////////
   // Initial setup
   
   // Create material for the spheres
   MaterialID material = createMaterial("material", 1.0, 0.8, 0.1, 0.05, 0.3, 200, 1e3, 1e2, 2e2);
   
   // Setup the simulation world
   WorldID world = theWorld();
   world->setGravity(0.0, 0.0, -9.81);  // Standard gravity
   world->setDamping(0.02);             // Small damping factor
   
   // Create several spheres at different positions
   std::vector<SphereID> spheres;
   for (size_t i = 0; i < num_spheres; ++i) {
      real x = -2.0 + i;
      real y = 0.0;
      real z = 5.0;  // Start height - will fall due to gravity
      
      SphereID sphere = createSphere(i, x, y, z, 0.2, material);
      spheres.push_back(sphere);
   }
   
   std::cout << "\n-- SIMULATION START --\n" << std::endl;
   printSimulationState(world, 0);
   
   /////////////////////////////////////////////////////
   // Initial simulation steps (before removal)
   
   for (size_t step = 1; step <= initial_steps; ++step) {
      // Perform one simulation step
      world->simulationStep(stepsize);
      
      // Print current state every few steps
      if (step % 2 == 0) {
         printSimulationState(world, step);
      }
   }
   
   /////////////////////////////////////////////////////
   // Body removal
   
   // Identify which sphere to remove (middle one)
   size_t remove_idx = num_spheres / 2;
   
   std::cout << "\n-- REMOVING SPHERE " << remove_idx << " --\n" << std::endl;
   
   // Find the iterator for the sphere to remove
   World::Iterator it = world->begin();
   for (size_t i = 0; i < remove_idx; ++i) {
      ++it;
   }
   
   // Remove the sphere using World::destroy
   world->destroy(it);
   
   // Remove from our tracking vector as well
   spheres.erase(spheres.begin() + remove_idx);
   
   // Print state after removal
   printSimulationState(world, initial_steps + 1);
   
   /////////////////////////////////////////////////////
   // Final simulation steps (after removal)
   
   for (size_t step = initial_steps + 2; step <= initial_steps + final_steps + 1; ++step) {
      // Perform one simulation step
      world->simulationStep(stepsize);
      
      // Print current state every few steps
      if (step % 2 == 0) {
         printSimulationState(world, step);
      }
   }
   
   std::cout << "\n-- SIMULATION COMPLETE --\n" << std::endl;
   
   return 0;
}