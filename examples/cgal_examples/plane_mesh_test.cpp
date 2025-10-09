//=================================================================================================
/*!
 *  \file plane_mesh_test.cpp
 *  \brief Dedicated test for plane-mesh collision detection with DistanceMap
 *
 *  This example tests collision detection between a single triangle mesh and a plane.
 *  It focuses specifically on DistanceMap-based plane collision detection and debugging.
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

#include <cstring>
#include <iostream>
#include <sstream>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/povray.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/vtk/UtilityWriters.h>

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
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\\brief Main function for the plane-mesh collision test.
 *
 * \\param argc Number of command line arguments.
 * \\param argv Array of command line arguments.
 * \\return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
#ifdef PE_USE_CGAL
   // Constants and variables
   const unsigned int timesteps ( 2501 );    // Total number of time steps
   const unsigned int visspacing(   10 );    // Spacing between two visualizations
   const real timestep_size( 0.001 );        // Size of each simulation time step
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "mesh", value<std::string>()->default_value(""), "mesh file to be loaded" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   std::string meshFile(vm["mesh"].as<std::string>());

   if( meshFile.empty() ) {
     std::cout << "Usage: " << argv[0] << " --mesh <file.obj>" << std::endl;
     std::cout << "Using default mesh file for demonstration." << std::endl;
     meshFile = std::string("span2.obj");
   }

   std::cout << "\\n--" << pe_BROWN << "PLANE-MESH COLLISION TEST" << pe_OLDCOLOR
             << "-------------------------------------------------------" << std::endl;
   std::cout << "Mesh file: " << meshFile << std::endl;
   std::cout << "Timesteps: " << timesteps << std::endl;
   std::cout << "Time step size: " << timestep_size << std::endl;

   // Simulation world setup
   WorldID world = theWorld();
   // Version 1: 
   world->setGravity( 0.0, 0.0,-9.81 );  // Standard gravity in -z direction

   // Disable error reduction for cleaner contact dynamics
   theCollisionSystem()->setErrorReductionParameter(0.075);
   std::cout << "\\nWorld initialized with gravity: " << world->getGravity() << std::endl;

   // Create materials
   MaterialID planeMaterial = createMaterial("plane", 2.5, 0.0, 0.3, 0.05, 0.2, 80, 100, 10, 11);
   MaterialID meshMaterial = createMaterial("mesh_mat", 1.2, 0.0, 0.2, 0.1, 0.2, 80, 100, 10, 11);

   // Setup of the ground plane with normal (0,0,1) at location (0,0,0)
   PlaneID groundPlane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, planeMaterial );
   std::cout << "Ground plane created with normal (0,0,1) at z=0" << std::endl;

   // plane normal in y directions
   PlaneID yMinus = createPlane( id++, 0.0,-1.0, 0.0,-3.0, planeMaterial );
   PlaneID yPlus = createPlane( id++, 0.0, 1.0, 0.0,-3.0, planeMaterial );

   // plane normal in y directions
   PlaneID xMinus = createPlane( id++,-1.0, 0.0, 0.0,-3.0, planeMaterial );
   PlaneID xPlus = createPlane( id++, 1.0, 0.0, 0.0,-3.0, planeMaterial );

   // Load mesh object
   TriangleMeshID testMesh = nullptr;

   try {
      // Load mesh at elevated position (will fall onto plane)
      std::cout << "\\nLoading test mesh from: " << meshFile << std::endl;
      testMesh = createTriangleMesh(++id, Vec3(0.0, 0.0, 8.03), meshFile, meshMaterial, false, true);
      std::cout << "Test mesh created with " << testMesh->getBFVertices().size() << " vertices at position: " << testMesh->getPosition() << std::endl;

      // Enable DistanceMap acceleration on the mesh
      testMesh->enableDistanceMapAcceleration(64, 6);  // resolution=64, tolerance=6

      // Version 1: Orientation for gravity in -z direction
      testMesh->rotate(-M_PI/2.0, 0.0, 0.0);
      //testMesh->rotate(0.0, 0.0,-M_PI);
      bool distanceMapEnabled = testMesh->hasDistanceMap();
      std::cout << "DistanceMap acceleration enabled on test mesh: " << (distanceMapEnabled ? "SUCCESS" : "FAILED") << std::endl;

      if (distanceMapEnabled) {
         const DistanceMap* dm = testMesh->getDistanceMap();
         if (dm) {
            std::cout << "DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
            std::cout << "DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
            std::cout << "DistanceMap spacing: " << dm->getSpacing() << std::endl;

            // Export DistanceMap for visualization
            pe::vtk::DistanceMapWriter::writeVTI("plane_mesh_test_sdf.vti", *dm);
            std::cout << "DistanceMap exported to plane_mesh_test_sdf.vti" << std::endl;
         }
      }

   }
   catch (const std::exception& e) {
      std::cerr << "ERROR loading mesh file: " << e.what() << std::endl;
      std::cout << "Note: Make sure the mesh file exists in the current directory or provide a valid path." << std::endl;
      std::cout << "The simulation will continue with a fallback sphere for demonstration purposes." << std::endl;
   }

   // Setup of the VTK visualization
   if( vtk && testMesh ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview_plane_mesh_test", visspacing, 0, timesteps, false);
      std::cout << "\\nVTK visualization enabled - output will be written to ./paraview_plane_mesh_test" << std::endl;
   }

   // Add a fallback sphere if mesh loading failed
   if (!testMesh) {
      std::cout << "\\nAdding fallback sphere for demonstration..." << std::endl;
      SphereID testSphere = createSphere( ++id, 0.0, 0.0, 3.0, 0.5, meshMaterial );
      testSphere->setLinearVel( 0.0, 0.0, -1.0 );
      std::cout << "Fallback sphere created for demonstration" << std::endl;
   }

   // Simulation loop
   std::cout << "\\n--" << pe_BROWN << "STARTING PLANE-MESH COLLISION TEST" << pe_OLDCOLOR
             << "---------------------------------------" << std::endl;

   real contactTime = -1.0;
   Vec3 firstContactPosition;
   bool contactDetected = false;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
       std::cout << "Timestep " << timestep << " (t=" << timestep * timestep_size << "s):" << std::endl;

      // Advance physics simulation by one time step
      world->simulationStep( timestep_size );

      // Monitor for first contact
      if (testMesh && !contactDetected) {
         real currentZ = testMesh->getPosition()[2];
         // Check if mesh is very close to or below the plane
         if (currentZ <= 0.5) {  // Within 0.5 units of the plane
            contactDetected = true;
            contactTime = timestep * timestep_size;
            firstContactPosition = testMesh->getPosition();
            std::cout << std::endl;
            std::cout << "\\n*** FIRST CONTACT DETECTED ***" << std::endl;
            std::cout << "Contact time: " << contactTime << " seconds (timestep " << timestep << ")" << std::endl;
            std::cout << "Contact position: " << firstContactPosition << std::endl;
            std::cout << "Mesh velocity: " << testMesh->getLinearVel() << std::endl;
         }
      }

      // Print object state periodically
      if( timestep % 10 == 0 ) {
         std::cout << std::endl;
         std::cout << "Timestep " << timestep << " (t=" << timestep * timestep_size << "s):";

         if (testMesh) {
            std::cout << " Mesh pos: " << testMesh->getPosition()
                      << ", vel: " << testMesh->getLinearVel() << std::endl;
         }
      }
   }

   std::cout << "\\n\\n--" << pe_BROWN << "PLANE-MESH COLLISION TEST COMPLETED" << pe_OLDCOLOR
             << "------------------------------------" << std::endl;

   if (testMesh) {
      std::cout << "Final mesh state:" << std::endl;
      std::cout << "  Final position: " << testMesh->getPosition() << std::endl;
      std::cout << "  Final velocity: " << testMesh->getLinearVel() << std::endl;

      if (contactDetected) {
         std::cout << "\\nCollision Summary:" << std::endl;
         std::cout << "  First contact at t=" << contactTime << "s" << std::endl;
         std::cout << "  Contact position: " << firstContactPosition << std::endl;
         std::cout << "  DistanceMap collision detection: " << (testMesh->hasDistanceMap() ? "ENABLED" : "DISABLED") << std::endl;
      } else {
         std::cout << "\\nNo collision detected during simulation." << std::endl;
      }
   }

   std::cout << "Total simulation time: " << timesteps * timestep_size << " seconds" << std::endl;

   if (vtk) {
      std::cout << "VTK output files written to ./paraview_plane_mesh_test/ directory" << std::endl;
   }

#else
   std::cout << "This test requires CGAL support. Please rebuild PE with -DCGAL=ON" << std::endl;
   return 1;
#endif

   return 0;
}
//*************************************************************************************************