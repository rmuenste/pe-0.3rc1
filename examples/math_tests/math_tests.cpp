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


//*************************************************************************************************
Vec3 calculateLub(real eta_f, Vec3 v_r, Vec3 n, real epsilon, real radius) {

    const real pi = 3.14159265358979323846;
    const Vec3 coefficient = -6 * pi * radius * eta_f * v_r * n;

    if( std::isnan(epsilon) ) { 
      std::cout << "NaN epsilon given:  " << epsilon << " log epsilon: " << std::log(epsilon) << std::endl;
    }

    real term1 = 1.0 / 4.0 * std::pow(epsilon, -1);
    if( std::isnan(epsilon) ) { 
      std::cout << "NaN epsilon given:  " << epsilon << " log epsilon: " << std::log(epsilon) << std::endl;
    }
//    real term2 = 0.0;
    real term2 = -9.0 / 40.0 * std::log(epsilon);
    if( std::isnan(epsilon) ) { 
      std::cout << "NaN epsilon given:  " << epsilon << " log epsilon: " << std::log(epsilon) << std::endl;
    }
    real term3 = -3.0 / 112.0 * epsilon * std::log(epsilon);
//    real term3 = 0.0;

    if( (std::isnan(term1)) ||
        (std::isnan(term2)) ||
        (std::isnan(term3)) ) {
      std::cout << "NaN lubrication found with another particle, epsilon, term:  " << epsilon << " " << term1 << " " << term2 << " log epsilon: " << std::log(epsilon) << std::endl;
    }

    return coefficient * (term1 + term2 + term3);
}
//*************************************************************************************************
real calc_f_star(real h, real hc_) {
    //double   f_star = (h / hc_) * ((1. + (h / (6. * hc_)) * std::log(1. + (6. * hc_) / h)) - 1.0);
    double f_star = (h / (3. * hc_)) * ((1. + (h / (6. * hc_))) * std::log(1. + (6. * hc_ / h)) - 1.0);
    //     f_star   = (h / (3. * hc_)) * ((1. + (h / (6. * hc_))) * std::log(1. + (6. * hc_ / h)) - 1.);
    return f_star;
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

   real simViscosity( 8.37e-5 );
   real simRho( 1.0 );
   world->setViscosity( simViscosity );
   world->setLiquidDensity( simRho );

   // Particle Bench Config 
   world->setLiquidSolid(true);
   world->setDamping( 1.0 );

   // Lubrication switch
   bool useLubrication(true);

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
   //Lubrication particle 
   //force: <1.34127e-08,-1.01345e-23,-4.40328e-24> | 
   //vr: <-0.0205702,-1.00345e-09,-6.32506e-10> | 
   //normal velocity: 0.0205702 | 
   //normal <-1,3.35845e-08,2.37152e-08>
   //Distance: 0.01 | 
   //eps: 2.08333
   real e2      = 2e-4;
   real radius  = 0.005 - e2;
   Vec3 vr(-0.0205702,-1.00345e-09,-6.32506e-10);
   Vec3 n(-1,3.35845e-08,2.37152e-08);
   real dist(0.01);
   real eps(2.08333);
   real rad = radius;
//   Vec3 lubricationForce = theCollisionSystem()->calculateLubricationForce(simViscosity, vr, n, eps, rad);
//   std::cout << "Lubrication particle force: " << lubricationForce 
//                                               << " | vr: " 
//                                               << vr 
//                                               << " | Distance: " 
//                                               << dist 
//                                               << " | eps: " 
//                                               << eps 
//                                               << std::endl;
//   eps = 0.1;
//   lubricationForce = calculateLub(simViscosity, vr, n, eps, rad);
//   std::cout << "Lubrication particle force: " << lubricationForce 
//                                               << " | vr: " 
//                                               << vr 
//                                               << " | eps: " 
//                                               << eps 
//                                               << std::endl;
//
//   real slipLength( 0.001 );
//   real fc =  calc_f_star(eps, slipLength);
//   std::cout << "Corrected particle force: "   << lubricationForce  * fc
//                                               << " | correction fc: " 
//                                               << fc 
//                                               << " | eps: " 
//                                               << eps 
//                                               << " | hc: " 
//                                               << slipLength 
//                                               << std::endl;

   //Vec3 lubricationForce = calculateLubricationForce(visc, vr, normal, eps, rad);
   //Vec3 pos = Vec3(1, 0.0, 0.0125);
   //CylinderID cyl = createCylinder(id++, pos, 0.1, 0.01251, granite);
   //cyl->rotate(0.0, -0.5 * M_PI, 0.0);
   //SphereID sph = createSphere(id++, pos, 0.1, granite);

   //theCollisionSystem()->getDomain().getGeometry()->intersectsWith(cyl);
   //theCollisionSystem()->getDomain().getGeometry()->intersectsWith(sph);


   return 0;
}
//*************************************************************************************************
