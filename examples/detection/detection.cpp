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
#include <pe/core/detection/coarse/HashGrids.h>

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
   bool povray  ( true );
   bool irrlicht( false );
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );

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
      camera->setLocation( 8.0, -25.0, 2.0 );
      camera->setFocus   ( 0.0,   0.0, 7.5 );

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

      // Setting the sphere texture
      //pov->setTexture( s, CustomTexture( "T_Chrome_1A" ) );
      //
      //pov->setTexture( plane, grassTexture );
   }

   typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactAndFluid>::Config TargetConfig3;
   BodyStorage<TargetConfig3> bs;
   pe::detection::coarse::HashGrids<TargetConfig3> grid(bs);


   int id = 0;
   // Create a custom material for the benchmark
   MaterialID simple = createMaterial("simple", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
   Vec3 gpos = Vec3(0, 0, 0);
   real radius = 1.0;
   Vec3 boxDimensions(radius, radius, radius);
   BodyID particle;

   // Wall parameters
   int wallHeight = 5;  // Number of boxes in the vertical direction
   int wallWidth = 10;  // Number of box towers in the x-direction
   real boxSpacing = radius * 2.0; // Spacing between box centers in each direction
  
   for (int i = 0; i < wallWidth; ++i) {
       for (int j = 0; j < wallHeight; ++j) {
           Vec3 position(i * boxSpacing, radius , j * boxSpacing); // Calculate box position
           createBox(id++, position, boxDimensions, simple); // Create the box
       }
   }      


   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
      world->simulationStep( 0.004 );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
