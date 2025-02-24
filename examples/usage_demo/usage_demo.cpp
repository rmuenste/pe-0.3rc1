//=================================================================================================
/*!
 *  \file BoxStack.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstring>
#include <iostream>
#include <sstream>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/povray.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>

#include <random>
#include <algorithm>
#include <vector>
#include <pe/core/Types.h>

using namespace pe;
using namespace pe::povray;


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
using namespace pe::irrlicht;
#endif




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================
std::vector<Vec3> generateRandomPositionsBox(real LX, real LY, real LZ, 
                                             real diameter, 
                                             real volumeFraction, 
                                             real eps)
{
    std::vector<Vec3> positions;

    // Define the effective cell size
    real cellSize = diameter + eps;

    // Particle volume
    real partVol = (4.0 / 3.0) * M_PI * std::pow(0.5 * diameter, 3);

    // Box volume
    real domainVol = LX * LY * LZ;

    std::cout << "Trying to generate volume fraction: " 
              << volumeFraction * 100.0 << " % " << std::endl;

    // Number of cells in each dimension
    int Nx = static_cast<int>(LX / cellSize);
    int Ny = static_cast<int>(LY / cellSize);
    int Nz = static_cast<int>(LZ / cellSize);

    // Total number of cells
    int totalCells = Nx * Ny * Nz;

    // Maximum possible volume fraction with this regular grid
    real maxPhi = (totalCells * partVol) / domainVol;

    if (volumeFraction > maxPhi) {
        std::cout << "User defined volume fraction: " << volumeFraction 
                  << " is too high for the current configuration (max: " 
                  << maxPhi << ")" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Keep track of which cells are used
    std::vector<bool> cellVisited(totalCells, false);

    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate random indices for each dimension
    std::uniform_int_distribution<int> distX(0, Nx - 1);
    std::uniform_int_distribution<int> distY(0, Ny - 1);
    std::uniform_int_distribution<int> distZ(0, Nz - 1);

    // Keep generating until we either fill all cells or reach the desired volume fraction
    while (positions.size() < static_cast<size_t>(totalCells) &&
           (partVol * positions.size() / domainVol) < volumeFraction)
    {
        // Random cell indices
        int x = distX(gen);
        int y = distY(gen);
        int z = distZ(gen);

        // Compute 1D index for visited-check
        int cellIndex = x + y * Nx + z * Nx * Ny;

        // Check if this cell hasn't been visited yet
        if (!cellVisited[cellIndex]) {
            cellVisited[cellIndex] = true;

            // Center of the chosen cell in each dimension
            real posX = (x + 0.5) * cellSize;
            real posY = (y + 0.5) * cellSize;
            real posZ = (z + 0.5) * cellSize;

            positions.push_back(Vec3(posX, posY, posZ));
        }
    }

    // Final volume fraction reached
    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
    std::cout << "Final volume fraction: " << solidFraction << " % " << std::endl;
    std::cout << "Number of particle positions: " << positions.size() << std::endl;

    return positions;
}


//*************************************************************************************************
/*!\brief Returns a random angle between \f$ [-\frac{\pi}{20}..\frac{\pi}{20}] \f$.
 *
 * \return The random angle (radian measure).
 */
real angle()
{
   return rand<real>( -M_PI/real(20), M_PI/real(20) );
}
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the box stack example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Constants and variables
   const unsigned int timesteps ( 3000 );  // Total number of time steps
   const unsigned int visspacing(   30 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   const unsigned int H ( 4 );              // Height of the box stack
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "file", value<std::string>()->default_value(""), "obj mesh file to be loaded" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   std::string fileName(vm["file"].as<std::string>());

   if( fileName.empty() ) {
     std::cout << "Need to enter a file via the --file command line parameter. Setting a default value." << std::endl;
     fileName = std::string("span_aligned.obj");
   }

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.4 );
   //======================================================================================== 
   // Here is how to create some random positions on a grid up to a certain
   // volume fraction.
   //======================================================================================== 
   real epsilon              = 2e-4;
   real targetVolumeFraction = 0.30;
   real radius2              = 0.0010925 - epsilon;

   const real LX( 1.0 );
   const real LY( 0.1 );
   const real LZ( 0.1 );

   std::vector<Vec3> allPositions = generateRandomPositionsBox(LX, LY, LZ, 
                                                               2. * radius2, 
                                                               targetVolumeFraction, 
                                                               epsilon);

   // Setup of the ground plane
   PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, -0.0, granite );


   // Setup of the metal sphere
   SphereID s = createSphere( ++id, 0.0, 0.05, 1.4, 0.04, granite );
   s->setLinearVel( 0.0, 0.0,-1.0 );

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

//   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
//      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
//      world->simulationStep( 0.004 );
//      //std::cout << "[particle position]: " << s->getPosition() << std::endl;
//   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
