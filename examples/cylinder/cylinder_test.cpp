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
   const unsigned int timesteps ( 5000 );  // Total number of time steps
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
   world->setGravity( 0.0, 0.0, 1.0 );

   // Setup of the metal sphere
   SphereID s = createSphere( ++id, -0.3, 0.0, 0.2, 0.05, iron );
   s->setLinearVel( 1.0, 0.0, 0.0 );

   CylinderID cyl(0);
   cyl = createCylinder( 10011, 0.0, 0.0, 0.0, 0.2, 0.8, iron );
   cyl->setFixed(true);
   cyl->rotate(0.0, M_PI/2.0, 0.0);

   InnerCylinderID cyl2(0);
   cyl2 = createInnerCylinder( 10012, 0.0, 0.0, 0.0, 0.4, 0.8, iron );
   cyl2->setFixed(true);
   cyl2->rotate(0.0, M_PI/2.0, 0.0);

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   // Setup of the POV-Ray visualization
   if( povray ) {
      WriterID pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "colors.inc" );
      pov->include( "woods.inc" );
      pov->include( "metals.inc" );
      pov->setFilename( "./video/box%.pov" );
      pov->setBackground( 1.0, 1.0, 1.0 );
      pov->addLightSource( PointLight( Vec3( 0.0, -10.0, 30.0 ), Color( 0.95, 0.95, 0.95 ) ) );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( 0.0, -5.0, 0.0 );
      camera->setFocus   ( 0.0,   0.0, 0.0 );

      // Setting the ground plane texture
      Finish grassFinish(
         Ambient( 0.2 )
      );
      PlainTexture grassTexture(
         ImagePigment( gif, "grass.gif", planar, true ),
         grassFinish,
         Scale( 20.0 ),
         Rotation( M_PI/2.0, 0.0, 0.0 )
      );
      //pov->setTexture( plane, grassTexture );

      // Setting the textures of the boxes
      std::ostringstream oss;
      for( World::Bodies::CastIterator<Box> b=world->begin<Box>(); b!=world->end<Box>(); ++b )
      {
         oss.str( "" );
         oss << "T_Wood" << rand<unsigned int>( 1, 12 );
         pov->setTexture( *b, CustomTexture( oss.str() ) );
      }

      // Setting the sphere texture
      pov->setTexture( s, CustomTexture( "T_Chrome_1A" ) );
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
      world->simulationStep( 0.005 );
      std::cout << "[particle position]: " << s->getPosition() << std::endl;
      std::cout << "[particle velocity]: " << s->getLinearVel() << std::endl;
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
