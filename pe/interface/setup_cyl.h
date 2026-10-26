//=================================================================================================
/*!
 *  \file pe/interface/setup_cyl.h
 *  \brief Example PE setup for a cylindrical domain (parallel PE mode)
 *
 *  Copyright (C) 2023 Raphael Münster
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

#ifndef _PE_INTERFACE_SETUP_CYL_H_
#define _PE_INTERFACE_SETUP_CYL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iostream>
#include <string>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/setup_config.h>




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Example setup: one sphere in a cylindrical domain (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 * \return void
 *
 * Reached from Fortran through commf2c_cyl_(). This setup is the reference example for a
 * parallel interface setup function with a cylindrical domain. It shows the usual steps:
 *
 *  -# load the configuration file and set the world and fluid properties,
 *  -# validate the process layout before anything is created,
 *  -# create the cartesian communicator and decompose the domain,
 *  -# create the domain boundary in a global section and the particles on their owning process,
 *  -# synchronize, activate the output and print a setup summary on rank 0.
 *
 * \b Domain: the boundary is a fixed, global InnerCylinder of radius \c cylinderRadius and
 * height \c cylinderHeight. Its axis is the z-axis, and it reaches from z = 0 to
 * z = cylinderHeight. The inside of the InnerCylinder (mantle and both caps) is the simulated
 * domain. Keep this placement: the contact points of the sphere-InnerCylinder collision
 * (MaxContacts::collideSphereInnerCylinder()) are only correct for a cylinder whose axis passes
 * through the origin and whose bottom cap lies at the origin, i.e. for this position and the
 * rotation by -pi/2 about the y-axis (an InnerCylinder is created along the x-axis).
 *
 * \b Decomposition: the cylinder is cut along its height axis into processesZ_ slabs of equal
 * height; processesX_ and processesY_ must be 1.
 *
 * \b Particles: a single sphere in the center of the cylinder.
 *
 * Read from the configuration file (SimulationConfig::getConfigFile()): gravity, fluid density
 * and viscosity, particle density, the sphere radius (benchRadius_), the process layout, step
 * size, number of time steps, the VTK switch and spacing, and the lubrication settings (applied
 * by commf2c_cyl_() after this setup). The size of the cylinder is fixed by the case constants.
 *
 * Every error path aborts the run with a message.
 */
void setupCyl( MPI_Comm ex0 )
{
   // Case constants: size of the cylindrical domain and fixed user IDs
   const real cylinderRadius( 1.0 );
   const real cylinderHeight( 8.0 );
   const pe::id_t cylinderId( 1000000 );
   const pe::id_t sphereId( 1 );

   auto& config = SimulationConfig::getInstance();

   int myRank( 0 );
   MPI_Comm_rank( ex0, &myRank );

   // Prints the message once and aborts the whole run
   const auto abortSetup = [&]( const std::string& message ) {
      if( myRank == 0 )
         std::cerr << "\nERROR in setupCyl: " << message << "\n" << std::endl;
      MPI_Abort( ex0, 1 );
   };

   loadConfigFileOnAllRanks( ex0, "setupCyl" );

   // World and fluid properties
   world = theWorld();
   world->setGravity( config.getGravity() );
   world->setViscosity( config.getFluidViscosity() );
   world->setLiquidDensity( config.getFluidDensity() );
   world->setLiquidSolid( true );
   world->setDamping( 1.0 );

   TimeStep::stepsize( config.getStepsize() );

   // Validation of the process layout and the particle (nothing is created before this point)
   mpisystem = theMPISystem();
   mpisystem->setComm( ex0 );

   const int px( config.getProcessesX() );
   const int py( config.getProcessesY() );
   const int pz( config.getProcessesZ() );
   const real sphereRadius( config.getBenchRadius() );

   if( px != 1 || py != 1 || pz < 1 ) {
      abortSetup( "the cylinder is decomposed along its height axis only: processesX_ and "
                  "processesY_ must be 1 and processesZ_ at least 1, got " + std::to_string( px ) +
                  " x " + std::to_string( py ) + " x " + std::to_string( pz ) + "." );
   }
   if( pz != mpisystem->getSize() ) {
      abortSetup( "invalid number of MPI processes: " + std::to_string( mpisystem->getSize() ) +
                  " != " + std::to_string( pz ) + " (processesZ_)." );
   }
   if( sphereRadius <= real(0) || sphereRadius >= cylinderRadius ||
       real(2) * sphereRadius >= cylinderHeight ) {
      abortSetup( "benchRadius_ = " + std::to_string( sphereRadius ) + " does not fit into the "
                  "cylinder (radius " + std::to_string( cylinderRadius ) + ", height " +
                  std::to_string( cylinderHeight ) + ")." );
   }

   // Cartesian communicator with one process per slab
   int dims   [] = { px, py, pz };
   int periods[] = { false, false, false };
   int reorder   = false;
   MPI_Comm cartcomm;

   MPI_Cart_create( ex0, 3, dims, periods, reorder, &cartcomm );
   if( cartcomm == MPI_COMM_NULL )
      abortSetup( "failed to create the cartesian communicator." );
   mpisystem->setComm( cartcomm );

   int center[3];
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 3, center );

   // Slabs along the height axis; in x and y each slab covers the whole cylinder
   const real slabHeight( cylinderHeight / pz );
   decomposeDomain( center, -cylinderRadius, -cylinderRadius, real(0),
                    real(2) * cylinderRadius, real(2) * cylinderRadius, slabHeight,
                    px, py, pz );

   pe_LOG_INFO_SECTION( log ) {
      log << "Cylinder slab " << center[2] << " of " << pz << ": z in [" << center[2] * slabHeight
          << ", " << ( center[2] + 1 ) * slabHeight << "]\n";
   }

   theMPISystem()->checkProcesses();

   // Materials (name, density, restitution, static and dynamic friction, Poisson's ratio,
   // Young's modulus, stiffness, normal and tangential damping)
   MaterialID wallMaterial = createMaterial( "cylinderWall", 1.0,
                                             0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   MaterialID particleMaterial = createMaterial( "particleMaterial", config.getParticleDensity(),
                                                 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   // Domain boundary: global, so every process knows it
   pe_GLOBAL_SECTION
   {
      const Vec3 cylinderPos( 0.0, 0.0, 0.5 * cylinderHeight );
      InnerCylinderID cylinder = createInnerCylinder( cylinderId, cylinderPos, cylinderRadius,
                                                      cylinderHeight, wallMaterial );
      cylinder->rotate( 0.0, -0.5 * M_PI, 0.0 );   // axis from x to z, see the documentation
      cylinder->setFixed( true );
   }

   // Particles: created only on the process that owns the position
   const Vec3 spherePos( 0.0, 0.0, 0.5 * cylinderHeight );
   if( world->ownsPoint( spherePos ) )
      createSphere( sphereId, spherePos, sphereRadius, particleMaterial );

   world->synchronize();

   if( config.getVtk() )
      vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(),
                           false, true );

   // Setup summary: every sphere counts once, on its owner (not as a shadow copy)
   unsigned long spheresLocal( 0 );
   for( World::Iterator body = world->begin(); body != world->end(); ++body ) {
      if( body->getType() == sphereType && !body->isRemote() )
         ++spheresLocal;
   }
   unsigned long spheresTotal( 0 );
   MPI_Reduce( &spheresLocal, &spheresTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

   const real domainVolume( M_PI * cylinderRadius * cylinderRadius * cylinderHeight );
   const real sphereVolume( real(4) / real(3) * M_PI * std::pow( sphereRadius, 3 ) );
   const real volumeFraction( spheresTotal * sphereVolume / domainVolume * real(100) );
   const char* lubrication( config.getLubricationEnabled() ? "enabled" : "disabled" );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << "CYLINDER SETUP"
                << "----------------------------------------------------------------\n"
                << " Total number of MPI processes           = " << pz << "\n"
                << " Slabs along the cylinder axis           = " << pz << "\n"
                << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
                << " Cylinder radius / height                = " << cylinderRadius << " / "
                                                                   << cylinderHeight << "\n"
                << " Domain volume                           = " << domainVolume << "\n"
                << " Total number of spheres                 = " << spheresTotal << "\n"
                << " Sphere radius                           = " << sphereRadius << "\n"
                << " Fluid viscosity                         = " << config.getFluidViscosity()
                                                                   << "\n"
                << " Fluid density                           = " << config.getFluidDensity() << "\n"
                << " Particle density                        = " << config.getParticleDensity()
                                                                   << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
                << " Lubrication (json)                      = " << lubrication << "\n"
                << " Volume fraction[%]                      = " << volumeFraction << "\n"
                << std::endl;
      std::cout << std::string( 80, '-' ) << "\n" << std::endl;
   }

   MPI_Barrier( cartcomm );
}
//*************************************************************************************************

#endif
