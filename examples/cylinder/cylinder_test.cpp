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
   const unsigned int visspacing(   10 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   const unsigned int H ( 4 );              // Height of the box stack
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool povray  ( true );
   bool irrlicht( false );
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

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
   world->setGravity( 0.0, 0.0, -1.0 );
   world->setViscosity(3e-3);
   

   real radius = 0.05;
   real dist = 3. * radius + 1e-3;
   // Setup of the metal sphere

   int xmax = 2;
   int ymax = 2;
   int zmax = 8;
   real dx = 2 * radius + 0.1 * radius;
   real dy = 2 * radius + 0.1 * radius;
   real dz = 2 * radius + 0.1 * radius;
   Vec3 pos(-0.3, -0.3, radius);
   for (int z = 0; z < zmax; ++z) {
     for (int y = 0; y < ymax; ++y) {
       for (int x = 0; x < xmax; ++x) {
         real jitter = sign() * 0.01 * radius;

         createSphere( ++id, pos[0] + x * dx + jitter, pos[1] + y * dy + jitter, pos[2] + z * dz, 0.05, iron );
       }
     }
   }

//   SphereID s = createSphere( ++id, 0.0, 0.0, 0.054, 0.05, iron );
//   s->setLinearVel( 1.1, 0.0, 0.0 );
//
//   SphereID s1 = createSphere( ++id, 0.2, 0.0, 0.054, 0.05, iron );
//   s1->setLinearVel( 0.0, 0.0, 0.0 );

//   CylinderID cyl(0);
//   cyl = createCylinder( 10011, 0.0, 0.0, 0.0, 0.2, 0.8, iron );
//   cyl->setFixed(true);
//   cyl->rotate(0.0, M_PI/2.0, 0.0);

   InnerCylinderID cyl2(0);
   cyl2 = createInnerCylinder( 10012, 0.0, 0.0, 2.4, 1.0, 4.8, iron );
   cyl2->setFixed(true);
   cyl2->rotate(0.0, M_PI/2.0, 0.0);

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   //theCollisionSystem()->setErrorReductionParameter(0.35);
   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep << " of " << timesteps << "   " << std::flush;
      world->simulationStep( 0.0005 );
//      std::cout << std::endl;
//      std::cout << "[particle1 position]: " << s->getPosition() << std::endl;
//      std::cout << "[particle1 velocity]: " << s->getLinearVel() << std::endl;
//      std::cout << "[particle2 position]: " << s1->getPosition() << std::endl;
//      std::cout << "[particle2 velocity]: " << s1->getLinearVel() << std::endl;
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
