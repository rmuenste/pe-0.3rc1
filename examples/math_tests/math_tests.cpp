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
   const unsigned int timesteps ( 3000 );  // Total number of time steps
   const unsigned int visspacing(   30 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   const unsigned int H ( 4 );              // Height of the box stack
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool povray  ( false );
   bool irrlicht( false );
   bool vtk( false );

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
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.4 );

   HalfSpace hs(1, 0, 0, 0);
   // Setup of the ground plane
   PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, -0.0, granite );
   // +y
   createPlane( id++, 0.0, 1.0, 0.0, -0.1, granite );
   // -y
   createPlane( id++, 0.0,-1.0, 0.0,  -0.1, granite );

   real dx = 1, dy = 1, dz = 1;
  defineLocalDomain( intersect(
     intersect(
     HalfSpace( Vec3(+1,0,0), +(0) ),
     HalfSpace( Vec3(-1,0,0), -( 1 ) ) ),
     HalfSpace( Vec3(0,+1,0), +(-0.5) ),
     HalfSpace( Vec3(0,-1,0), -(0.5 ) ),
     HalfSpace( Vec3(0,0,+1), +(0) ),
     HalfSpace( Vec3(0,0,-1), -(0.025 ) ) ) );

   Vec3 pos = Vec3(1, 0.0, 0.0125);
   CylinderID cyl = createCylinder(id++, pos, 0.1, 0.025, granite);
   cyl->rotate(0.0, -0.5 * M_PI, 0.0);
   SphereID sph = createSphere(id++, pos, 0.1, granite);

   theCollisionSystem()->getDomain().getGeometry()->intersectsWith(cyl);
   theCollisionSystem()->getDomain().getGeometry()->intersectsWith(sph);

   return 0;
}
//*************************************************************************************************
