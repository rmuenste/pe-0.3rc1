//=================================================================================================
/*!
 *  \file simple_granular.cpp
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
#include <pe/engine.h>
#include <pe/support.h>

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
   const unsigned int timesteps ( 20 );  // Total number of time steps
   const unsigned int visspacing(   10 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool povray  ( true );
   bool irrlicht( true );
   //bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   if( vm.count( "no-irrlicht" ) > 0 )
      irrlicht = false;
//   if( vm.count( "no-vtk" ) > 0 )
//      vtk = false;

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.4 );

   // Setup of the ground plane
   PlaneID plane = createPlane( 0, 0.0, 0.0, 1.0, 0.0, granite );

   MaterialID granular = createMaterial( "granular", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

//   // Setup of the VTK visualization
//   if( vtk ) {
//      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
//   }

   real radius = 1.5;
   Vec3 gpos(0, 0, 2. * radius);

   // Setup of the metal sphere
   SphereID s = createSphere( ++id, 0.0, -25.0, 7.5, 1.5, iron );
   s->setLinearVel( 0.0, 5.5, 0.1 );
   WriterID pov2;
   // Setup of the POV-Ray visualization
   if( povray ) {
      WriterID pov = activateWriter();
      pov2 = pov;
      pov->setSpacing( visspacing );
      pov->include( "colors.inc" );
      pov->include( "woods.inc" );
      pov->include( "metals.inc" );
      pov->setFilename( "./video/box%.pov" );
      pov->setBackground( 1.0, 1.0, 1.0 );
      pov->addLightSource( PointLight( Vec3( 0.0, -10.0, 30.0 ), Color( 0.95, 0.95, 0.95 ) ) );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( 20.0, -35.0, 30.0 );
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
      pov->setTexture( plane, grassTexture );

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

   BodyID particle = createGranularParticle( ++id, gpos, radius, granular );

   gpos[0] += 5. * radius;
   particle = createAgglomerate( ++id, gpos, radius, granular, 5, 0.8, true );

   gpos[0] += 4. * radius;
   particle = createTristar( ++id, gpos, radius, 2. * radius, granular, true );

   gpos[0] += 5. * radius;
   particle = createStellaOctangula( ++id, gpos, radius,  granular, pov2);

   gpos[0] += 2. * radius;
   particle = createBipyramid( ++id, gpos, radius,  granular, pov2);

   // Setup of the Irrlicht visualization
#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->setSpacing( visspacing );
      viewer->addPointLightSource( 0.0F, -10.0F, 30.0F, 1.0F, 1.0F, 1.0F );
      viewer->addFPSCamera  ( 8.0F, -23.0F, 12.0F, 0.0F, 0.0F, 7.5F );
   }
#else
   UNUSED( irrlicht );
#endif


   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep < timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
      world->simulationStep( 0.0004 );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
