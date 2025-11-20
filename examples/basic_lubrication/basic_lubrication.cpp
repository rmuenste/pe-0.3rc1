//=================================================================================================
/*!
 *  \file basic_lubrication.cpp
 *  \brief Minimal sphere-plane setup for the HardContactLubricated solver stack
 *
 *  This example is intended to provide a lightweight regression/diagnostic case to exercise
 *  the HardContactLubricated solver configuration.  It drops a single sphere onto a ground plane
 *  (normal n = (0,0,1) through the origin) and reports the gap evolution so that lubrication
 *  forces and solver settings can be tuned and verified independently from complex meshes.
 *
 *  Copyright (C) 2025 PE Physics Engine
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

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>

using namespace pe;


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \brief Main function for the basic lubrication example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Defaults that can be overridden via CLI
   const unsigned int defaultSteps( 2500U );
   const real         defaultDt( 5.0e-4 );
   const real         defaultRadius( 0.01 );
   const real         defaultHeight( 0.05 );   // Height above plane, not including radius
   const unsigned int defaultVisSpacing( 20 );

   bool vtkOutput( true );

   // Parse CLI
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "timesteps", value<unsigned int>()->default_value( defaultSteps ), "Number of simulation steps to execute" )
      ( "dt", value<double>()->default_value( defaultDt ), "Time step size" )
      ( "radius", value<double>()->default_value( defaultRadius ), "Sphere radius" )
      ( "height", value<double>()->default_value( defaultHeight ), "Initial height of sphere center above plane" )
      ( "no-vtk", "Disable VTK output writer" )
      ( "no-lubrication", "Disable lubrication forces (diagnostic switch)" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   if( vm.count( "no-vtk" ) > 0 ) {
      vtkOutput = false;
   }

   const unsigned int timesteps = vm["timesteps"].as<unsigned int>();
   const real timestepSize      = static_cast<real>( vm["dt"].as<double>() );
   const real radius            = static_cast<real>( vm["radius"].as<double>() );
   const real startHeight       = static_cast<real>( vm["height"].as<double>() );
   const bool useLubrication    = ( vm.count( "no-lubrication" ) == 0 );

   if( radius <= real(0) ) {
      std::cerr << "Sphere radius must be positive." << std::endl;
      return EXIT_FAILURE;
   }

   std::cout << "\n--" << pe_BROWN << "BASIC LUBRICATION TEST" << pe_OLDCOLOR
             << "-------------------------------------------------------" << std::endl;
   std::cout << " timesteps    : " << timesteps << "\n"
             << " dt           : " << timestepSize << "\n"
             << " radius       : " << radius << "\n"
             << " start height : " << startHeight << "\n"
             << " lubrication  : " << ( useLubrication ? "enabled" : "disabled" ) << std::endl;
   std::cout << " NOTE: Configure pe_CONSTRAINT_SOLVER to pe::response::HardContactLubricated for this example."
             << std::endl;

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setViscosity( 8.37e-5 );      // Representative low-viscosity lubricant
   world->setLiquidDensity( 1.0 );      // Simplified density, can be overridden in configs
   world->setLiquidSolid( true );
   world->setDamping( 1.0 );

   // Configure collision system parameters relevant for the HardContactLubricated solver stack.
   theCollisionSystem()->setLubrication( useLubrication );
   theCollisionSystem()->setSlipLength( 0.01 );
   theCollisionSystem()->setMinEps( 0.005 );
   theCollisionSystem()->setErrorReductionParameter( 0.05 );

   unsigned int id( 0U );

   MaterialID planeMaterial = createMaterial( "plane_lubricated", 2.5, 0.0, 0.5, 0.3, 0.25, 150, 1000, 10, 11 );
   MaterialID sphereMaterial = createMaterial( "sphere_lubricated", 7.0, 0.2, 0.45, 0.35, 0.22, 300, 5.0e4, 20, 25 );

   // Ground plane with the requested normal (0,0,1) and offset 0 (plane at origin).
   PlaneID groundPlane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, planeMaterial );
   groundPlane->setFixed( true );
   std::cout << " Ground plane created with normal (0,0,1) and offset 0." << std::endl;

   const real sphereCenterZ = radius + startHeight;
   SphereID probeSphere = createSphere( id++, 0.0, 0.0, sphereCenterZ, radius, sphereMaterial );
   probeSphere->setLinearVel( 0.0, 0.0, -0.05 );
   probeSphere->setAngularVel( 0.0, 0.0, 0.0 );

   if( vtkOutput ) {
      const unsigned int spacing = std::max( 1U, defaultVisSpacing );
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview_basic_lubrication", spacing, 0, timesteps, false );
      (void)vtkw;
   }

   real minimumGap = std::numeric_limits<real>::max();
   real firstContactTime = real(-1);
   const unsigned int logFrequency = std::max( 1U, timesteps / 25U );

   for( unsigned int step = 0; step < timesteps; ++step ) {
      world->simulationStep( timestepSize );

      const Vec3 position = probeSphere->getPosition();
      const Vec3 velocity = probeSphere->getLinearVel();
      const real gap = position[2] - radius; // Plane lies at z = 0, gap is center height minus radius.
      minimumGap = std::min( minimumGap, gap );

      if( gap <= real(1.0e-5) && firstContactTime < real(0) ) {
         firstContactTime = static_cast<real>( step ) * timestepSize;
      }

      if( step % logFrequency == 0 ) {
         std::cout << " step " << step
                   << " | z = " << position[2]
                   << " | vz = " << velocity[2]
                   << " | gap = " << gap
                   << " | contacts = " << theCollisionSystem()->getNumberOfContacts()
                   << std::endl;
      }
   }

   std::cout << "\n Simulation complete.\n"
             << "  minimum center-plane gap : " << minimumGap << "\n"
             << "  first contact time       : "
             << ( firstContactTime >= real(0) ? firstContactTime : real(-1) )
             << std::endl;

   if( !useLubrication ) {
      std::cout << " Lubrication forces were disabled via --no-lubrication; "
                   "re-run without that switch to exercise the HardContactLubricated solver."
                << std::endl;
   }

   return EXIT_SUCCESS;
}
//*************************************************************************************************
