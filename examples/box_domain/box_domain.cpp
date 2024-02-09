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
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <random>
#include <algorithm>

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

real sign()
{
   real val = rand<real>( -M_PI/real(20), M_PI/real(20) );
   return (val > 0) ? 1.0 : -1.0; 
}


//=================================================================================================
// GenerateRandomPositions
//=================================================================================================
// Function to generate random positions within a cubic domain
std::vector<Vec3> generateRandomPositions(real L, real diameter, real volumeFraction, real eps) {

    WorldID world = theWorld();

    std::vector<Vec3> positions;

    real cellSize = diameter + eps;

    real partVol = 4./3. * M_PI * std::pow(0.5 * diameter, 3);
    real domainVol = L * L * L;

    std::cout << "Trying to generate volume fraction:  " << volumeFraction * 100.0 << std::endl;

    // Calculate the number of cells along one side of the cubic grid
    int gridSize = static_cast<int>(L / cellSize);

    // Calculate the total number of cells in the grid
    int totalCells = gridSize * gridSize * gridSize;

    // Calculate the maximum volume fraction possible for the current configuration
    real maxPhi = ((totalCells * partVol) / domainVol);

    if (volumeFraction > maxPhi) {
      std::cout << "User defined volume fraction: " << volumeFraction << " is too high for the current configuration" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    // Initialize a vector to keep track of visited cells
    std::vector<bool> cellVisited(totalCells, false);

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<real> dis(-cellSize / 2.0, cellSize / 2.0);

    // Generate random positions until the grid is full or the volume fraction is reached
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) / totalCells) < volumeFraction) {
    while (positions.size() < totalCells && (partVol * positions.size() / domainVol) < volumeFraction) {
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) * partVol) / domainVol < volumeFraction) {
        // Generate random cell indices
        int x = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int y = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int z = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);

        int cellIndex = x + y * gridSize + z * gridSize * gridSize;
        
        // Calculate the position of the cell center
        real posX = (x + 0.5) * cellSize;
        real posY = (y + 0.5) * cellSize;
        real posZ = (z + 0.5) * cellSize;

        Vec3 gpos(posX, posY, posZ);

        // Check if the cell has not been visited
        //if (!cellVisited[cellIndex] && world->ownsPoint( gpos ) ) {
        if (!cellVisited[cellIndex]) {
            // Mark the cell as visited
            cellVisited[cellIndex] = true;

            // Calculate the position of the cell center
            double posX = (x + 0.5) * cellSize;
            double posY = (y + 0.5) * cellSize;
            double posZ = (z + 0.5) * cellSize;

            // Create a Vec3 object for the position and add it to the positions vector
            positions.push_back(Vec3(posX, posY, posZ));
        }
    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
//    std::cout << MPISettings::rank() << ")local fraction:  " << (static_cast<real>(positions.size()) / totalCells) << " of " << volumeFraction << std::endl;
//    std::cout << MPISettings::rank() << ")local:  " << positions.size() << " of " << totalCells << std::endl;
    }

    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
    std::cout << MPISettings::rank() << ")Volume fraction:  " << solidFraction << std::endl;
    std::cout << MPISettings::rank() << ")local:  " << positions.size() << std::endl;
    return positions;
}



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
   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 1000 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.0005 );  // Size of a single time step

   // Visualization variables
   const size_t visspacing    (   10 );  // Number of time steps in-between two POV-Ray files
   bool povray  ( true );
   bool irrlicht( false );
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Fixed simulation parameters
   const real L(0.01);
   const real LX(0.04);
   const real LY(0.02);
   const real LZ(0.04);

//   // Parsing the command line arguments
//   CommandLineInterface& cli = CommandLineInterface::getInstance();
////   cli.getDescription().add_options()
////     ( "file", value<std::string>()->default_value(""), "obj mesh file to be loaded" );
//   cli.parse( argc, argv );
//   cli.evaluateOptions();
//   variables_map& vm = cli.getVariablesMap();
//   if( vm.count( "no-povray" ) > 0 )
//      povray = false;
//   if( vm.count( "no-irrlicht" ) > 0 )
//      irrlicht = false;
//   if( vm.count( "no-vtk" ) > 0 )
//      vtk = false;

//   std::string fileName(vm["file"].as<std::string>());
//
//   if( fileName.empty() ) {
//     std::cout << "Need to enter a file via the --file command line parameter. Setting a default value." << std::endl;
     std::string fileName = std::string("sphere.obj");
//   }

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );

   real simViscosity( 8.37e-5 );
   real simRho( 1.0 );
   world->setViscosity( simViscosity );
   world->setLiquidDensity( simRho );
 
   // Particle Bench Config 
   real slipLength( 0.01 );
   real minEps( 0.01 );
   world->setLiquidSolid(true);
   world->setDamping( 1.0 );
 
   // Lubrication switch
   bool useLubrication(true);
   theCollisionSystem()->setSlipLength(slipLength);
   theCollisionSystem()->setMinEps(minEps);
   theCollisionSystem()->setLubrication( useLubrication );


   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   //======================================================================================== 
   // The way we atm include lubrication by increasing contact threshold
   // has problems: the particles get distributed to more domain bc the threshold AABB
   // is much larger than the particle actually is.
   // We can even run into the "registering distant domain" error when the AABB of the 
   // particle is close in size to the size of a domain part!
   //======================================================================================== 
   int id = 1;
 
   //======================================================================================== 
   // Here is how to create some random positions on a grid up to a certain
   // volume fraction.
   //======================================================================================== 
   bool resume = false;
   real epsilon = 1e-4;
   real targetVolumeFraction = 0.35;
   real radius2 = 0.01 - epsilon;
   Vec3 gpos (LX * 0.5 , LY * 0.5, radius2 + epsilon);
   Vec3 vel(0.025, 0.0, 0.0);

   std::vector<Vec3> allPositions;
   int numPositions;
   
   //allPositions = generateRandomPositions(0.1, 2.0 * radius2, targetVolumeFraction, epsilon); 


   // Creates the material "myMaterial" with the following material properties:
   //  - material density               : 2.54
   //  - coefficient of restitution     : 0.8
   //  - coefficient of static friction : 0.1
   //  - coefficient of dynamic friction: 0.05
   //  - Poisson's ratio                : 0.2
   //  - Young's modulus                : 80
   //  - Contact stiffness              : 100
   //  - dampingN                       : 10
   //  - dampingT                       : 11
   //MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   int planeId = 99999;
   //======================================================================================== 
   // Here we add some planes
   BodyID botPlane = createPlane( 6666, 0.0, 0.0, 1.0, 0.0, elastic, false ); // bottom border
   BodyID topPlane = createPlane( 7777, 0.0, 0.0, -1.0, -LZ, elastic, false ); // top border
   BodyID rightPlane = createPlane( 8888, 1.0, 0.0, 0.0, 0.0, elastic, false ); // right border
   BodyID leftPlane = createPlane( 9999,-1.0, 0.0,  0.0, -LX, elastic, false ); // left border
   BodyID frontPlane = createPlane( 5555, 0.0, 1.0, 0.0, 0.0, elastic, false ); // front border
   BodyID backPlane = createPlane( 4444, 0.0, 1.0,  0.0, -LY, elastic, false ); // back border
   std::cout << "topPlaneID: "  << topPlane->getSystemID() << std::endl;
   std::cout << "botPlaneID: "  << botPlane->getSystemID() << std::endl;

   BodyID s,s1,s2,s3,s4;
   s = createSphere( id++, gpos, radius2, elastic );
   gpos[2] += 2. * radius2 + epsilon;
   s1 = createSphere( id++, gpos, radius2, elastic );
//   gpos[2] += 2. * radius2 + epsilon;
//   s2 = createSphere( id++, gpos, radius2, elastic );
//   gpos[2] += 2. * radius2 + epsilon;
//   s3 = createSphere( id++, gpos, radius2, elastic );
//   gpos[2] += 2. * radius2 + epsilon;
//   s4 = createSphere( id++, gpos, radius2, elastic );
 
   unsigned int particlesTotal = 1;
   real domainVol = LX * LY * LZ;
   real partVol = 4./3. * M_PI * std::pow(radius2, 3);
   real phi = (particlesTotal * partVol)/domainVol * 100.0;
 
   std::string resOut = (resume) ? " resuming " : " not resuming ";
   std::string useLub = (useLubrication) ? "enabled" : "disabled";
 
   pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\n--" << "SIMULATION SETUP"
       << "--------------------------------------------------------------\n"
       << " Total timesteps                         = " << timesteps << "\n"
       << " Timestep size                           = " << stepsize << "\n"
       << " Total particles                         = " << particlesTotal << "\n"
       << " particle volume                         = " << partVol << "\n"
       << " Fluid Viscosity                         = " << simViscosity << "\n"
       << " Fluid Density                           = " << simRho << "\n"
       << " Lubrication                             = " << useLub << "\n"
       << " Lubrication h_c                         = " << slipLength << "\n"
       << " Lubrication threshold                   = " << lubricationThreshold << "\n"
       << " Contact threshold                       = " << contactThreshold << "\n"
       << " Domain volume                           = " << LX * LY * LZ << "\n"
       << " Resume                                  = " << resOut  << "\n"
       << " Volume fraction[%]                      = " << phi << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep << " of " << timesteps << "   " << std::flush;
      world->simulationStep( 0.0005 );
      //std::cout << "[particle1 position]: " << s->getPosition() << std::endl;
      //std::cout << std::endl;
      //std::cout << "[particle " << s->getSystemID() << " velocity]: " << s->getLinearVel() << std::endl;
      //std::cout << "[particle2 position]: " << s1->getPosition() << std::endl;
      //std::cout << "[particle " << s1->getSystemID() << " velocity]: " << s1->getLinearVel() << std::endl;
      //std::cout << "[particle3 velocity]: " << s2->getLinearVel() << std::endl;
      //std::cout << "[particle4 velocity]: " << s3->getLinearVel() << std::endl;
      //std::cout << "[particle5 velocity]: " << s4->getLinearVel() << std::endl;
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
