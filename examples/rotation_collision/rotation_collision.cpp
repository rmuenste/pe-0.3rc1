//=================================================================================================
/*!
 *  \file rotation_collision.cpp
 *  \brief Example demonstrating translation-fixed vs fully-fixed rigid bodies
 *
 *  This example shows:
 *  - A rotating rod (fully fixed in position but can rotate)
 *  - A translation-fixed rod (can rotate but cannot translate)
 *  - Collision between them causing the translation-fixed rod to start rotating
 *
 *  Copyright (C) 2024
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
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

using namespace pe;
using namespace pe::povray;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the rotation collision example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Constants and variables
   const unsigned int timesteps ( 500 );  // Total number of time steps
   const unsigned int visspacing(   10 );  // Spacing between visualizations
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "no-vtk", "Disable VTK visualization" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );  // Standard gravity

   // Create materials
   MaterialID steel = createMaterial( "steel", 7800.0, 0.3, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   // Setup of the ground plane
   PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, -1.0, granite );

   // Create the rotating rod (fully fixed in position, but we'll give it angular velocity)
   // Position: (0, 0, 2), Size: 5.0 x 1.0 x 1.0 (long rod along x-axis)
   BoxID rotatingRod = createBox( id++, 0.0, 0.0, 2.0, 5.0, 1.0, 1.0, steel );
   rotatingRod->setFixed( true );  // Completely fixed (can't translate or rotate naturally)
   rotatingRod->setAngularVel( 0.0, 0.0, 1.0 );  // Rotate around z-axis

   // Create the translation-fixed rod (can rotate but cannot translate)
   // Position: (3.5, 0, 2), Size: 5.0 x 1.0 x 1.0 (long rod along x-axis)
   // Position it so the rotating rod will hit it
   BoxID translationFixedRod = createBox( id++, 3.5, 0.0, 2.0, 5.0, 1.0, 1.0, steel );
   translationFixedRod->setTranslationFixed( true );  // Can rotate but cannot translate
   

   std::cout << "\n" << pe_BROWN << "ROTATION COLLISION DEMO" << pe_OLDCOLOR << std::endl;
   std::cout << "=======================" << std::endl;
   std::cout << "Rotating rod (fixed): " << rotatingRod->getID() << " at position " << rotatingRod->getPosition() << std::endl;
   std::cout << "Translation-fixed rod: " << translationFixedRod->getID() << " at position " << translationFixedRod->getPosition() << std::endl;
   std::cout << std::endl;

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << std::setw(3) << timestep+1 << " of " << timesteps;
      
      // Print some key information every 50 steps
      if( timestep % 50 == 0 ) {
         std::cout << std::endl;
         std::cout << "  Rotating rod angular velocity: " << rotatingRod->getAngularVel() << std::endl;
         std::cout << "  Translation-fixed rod angular velocity: " << translationFixedRod->getAngularVel() << std::endl;
         std::cout << "  Translation-fixed rod position: " << translationFixedRod->getPosition() << std::endl;
      }
      
      std::cout << std::flush;
      
      world->simulationStep( 0.01 );  // 10ms time steps
   }

   std::cout << "\n--------------------------------------------------------------------------------" << std::endl;
   std::cout << "\nFinal state:" << std::endl;
   std::cout << "Translation-fixed rod angular velocity: " << translationFixedRod->getAngularVel() << std::endl;
   std::cout << "Translation-fixed rod position (should be unchanged): " << translationFixedRod->getPosition() << std::endl;
   std::cout << std::endl;

   return 0;
}
//*************************************************************************************************