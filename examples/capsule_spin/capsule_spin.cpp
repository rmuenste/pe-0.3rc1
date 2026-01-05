//=================================================================================================
/*!
 *  \file capsule_spin.cpp
 *  \brief Serial example rotating a capsule about the z-axis at 60 rpm
 *
 *  Copyright (C) 2025
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

#include <iostream>
#include <pe/core.h>
#include <pe/vtk.h>
#include <pe/util.h>

using namespace pe;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the capsule spin example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Constants and variables
   const unsigned int timesteps( 120 );
   const real dt( 0.001 );
   const real rpm( 60.0 );
   const real omega( ( rpm / 60.0 ) * 2.0 * M_PI );
         unsigned int id( 0 );
   const unsigned int visspacing(   10 );  // Spacing between two visualizations (POV-Ray & Irrlicht)

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );

   // Capsule centered at the origin
   CapsuleID capsule = createCapsule( id++, 0.0, 0.0, 0.0, 0.2, 1.0, iron );
   capsule->setFixed( true );
   capsule->setLinearVel( 0.0, 0.0, 0.0 );
   capsule->setAngularVel( 0.0, 0.0, omega );

   CapsuleID capsule2 = createCapsule( id++, 0.7, 0.7, 0.0, 0.2, 1.0, iron );
   capsule2->setTranslationFixed( true );

   // Setup of the VTK visualization
   vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);

   std::cout << "\n--" << pe_BROWN << "CAPSULE SPIN" << pe_OLDCOLOR
             << "-------------------------------------------------------------" << std::endl;
   std::cout << "Center: " << capsule->getPosition()
             << " | Angular velocity: " << capsule->getAngularVel() << std::endl;

   // Simulation loop
   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   \n";
      std::cout << "Center: " << capsule->getPosition()
                << " | Orientation: " << capsule->getQuaternion() << "\n"
                << " | Angular velocity: " << capsule->getAngularVel() << std::endl;
      std::cout << "Center2: " << capsule2->getPosition()
                << " | Orientation2: " << capsule2->getQuaternion() << "\n"
                << " | Angular velocity2: " << capsule2->getAngularVel() << std::endl;
      world->simulationStep( dt );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;

   return 0;
}
//*************************************************************************************************
