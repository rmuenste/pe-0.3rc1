//=================================================================================================
/*!
 *  \file mesh_simulation.cpp
 *  \brief CGAL-enabled mesh simulation example with DistanceMap collision detection
 *
 *  This example demonstrates a physics simulation using triangle meshes with CGAL support.
 *  It loads two mesh objects, adds a ground plane, and runs a complete physics simulation
 *  with DistanceMap-accelerated collision detection.
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
#include "VtkOutput.h"

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
/*!\brief Main function for the mesh simulation example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
#ifdef PE_USE_CGAL
   // Constants and variables
   const unsigned int timesteps ( 500 );    // Total number of time steps
   const unsigned int visspacing(   10 );   // Spacing between two visualizations
   const real timestep_size( 0.005 );       // Size of each simulation time step
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "mesh1", value<std::string>()->default_value(""), "first mesh file to be loaded" )
     ( "mesh2", value<std::string>()->default_value(""), "second mesh file to be loaded" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   std::string mesh1File(vm["mesh1"].as<std::string>());
   std::string mesh2File(vm["mesh2"].as<std::string>());

   if( mesh1File.empty() || mesh2File.empty() ) {
     std::cout << "Usage: " << argv[0] << " --mesh1 <file1.obj> --mesh2 <file2.obj>" << std::endl;
     std::cout << "Using default mesh files for demonstration." << std::endl;
     mesh1File = std::string("mesh1.obj");
     mesh2File = std::string("mesh2.obj");
   }

   std::cout << "\n--" << pe_BROWN << "CGAL MESH SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;
   std::cout << "Mesh 1: " << mesh1File << std::endl;
   std::cout << "Mesh 2: " << mesh2File << std::endl;
   std::cout << "Timesteps: " << timesteps << std::endl;
   std::cout << "Time step size: " << timestep_size << std::endl;

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );  // Standard gravity in -z direction

   //theCollisionSystem()->setErrorReductionParameter(0.0125);
   theCollisionSystem()->setErrorReductionParameter(0.00);
   std::cout << "\nWorld initialized with gravity: " << world->getGravity() << std::endl;

   // Create materials
   MaterialID groundMaterial = createMaterial("ground", 2.5, 0.0, 0.3, 0.05, 0.2, 80, 100, 10, 11);
   MaterialID mesh1Material = createMaterial("mesh1_mat", 1.2, 0.0, 0.2, 0.1, 0.2, 80, 100, 10, 11);
   MaterialID mesh2Material = createMaterial("mesh2_mat", 0.8, 0.0, 0.25, 0.08, 0.2, 80, 100, 10, 11);

   // Setup of the ground plane with normal (0,0,1) at location (0,0,0)
//   PlaneID groundPlane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, groundMaterial );
//   std::cout << "Ground plane created with normal (0,0,1) at z=0" << std::endl;

   // Load mesh objects
   TriangleMeshID mesh1 = nullptr;
   TriangleMeshID mesh2 = nullptr;

   try {
      // Load first mesh at elevated position
      std::cout << "\nLoading mesh 1 from: " << mesh1File << std::endl;
      mesh1 = createTriangleMesh(++id, Vec3(0.0, 0.0, 0.0), mesh1File, mesh1Material, false, true);
      mesh1->setFixed( true );
      std::cout << "Mesh 1 created with " << mesh1->getBFVertices().size() << " vertices at position: " << mesh1->getPosition() << std::endl;
      
      // Enable DistanceMap acceleration on mesh 1
      mesh1->enableDistanceMapAcceleration(0.1, 30, 3);  // spacing=0.1, resolution=30, tolerance=3
      std::cout << "DistanceMap acceleration enabled on mesh 1: " << (mesh1->hasDistanceMap() ? "SUCCESS" : "FAILED") << std::endl;

      auto origin = mesh1->getDistanceMap()->getOrigin();
      // Create dummy face index for VTI export (not used in DistanceMap)
      std::vector<int> face_index(mesh1->getDistanceMap()->getSdfData().size(), 0);
      write_vti("sdf.vti",
                mesh1->getDistanceMap()->getSdfData(),
                mesh1->getDistanceMap()->getAlphaData(), 
                mesh1->getDistanceMap()->getNormalData(),
                mesh1->getDistanceMap()->getContactPointData(),
                face_index,
                mesh1->getDistanceMap()->getNx(), mesh1->getDistanceMap()->getNy(), mesh1->getDistanceMap()->getNz(),
                mesh1->getDistanceMap()->getSpacing(), mesh1->getDistanceMap()->getSpacing(), mesh1->getDistanceMap()->getSpacing(),
                origin[0], origin[1], origin[2]);
      
      // Set initial linear velocity (slight downward and forward motion)
      //mesh1->setLinearVel( 0.5, 0.0, -0.2 );
      
      // Load second mesh at different elevated position
      std::cout << "\nLoading mesh 2 from: " << mesh2File << std::endl;
      //mesh2 = createTriangleMesh(++id, Vec3(0.0, 3.1, 4.7), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(0.0, -1.72496, 8.36196), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(0.0, 3.03749, 5.82685), mesh2File, mesh2Material, false, true);
      //  TriangleMeshID testSphere = createTriangleMesh(2, Vec3(-1.02965, 1.80596, 5.78679), testSphereFile, material2, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(-1.55176,2.2521, 5.82685), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(-0.95, 1.80596, 5.78679), mesh2File, mesh2Material, false, true);
      mesh2 = createTriangleMesh(++id, Vec3(1.13668,-2.79345, 6.24138), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(2.0, 0.0, 6.2), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(0.0, 0.0, 6.02), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(0.0, 0.0, 6.41), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3(-0.0, 0.0, 2.01), mesh2File, mesh2Material, false, true);
      //mesh2 = createTriangleMesh(++id, Vec3( 0.0, 0.1, 2.1), mesh2File, mesh2Material, false, true);
      std::cout << "Mesh 2 created with " << mesh2->getBFVertices().size() << " vertices at position: " << mesh2->getPosition() << std::endl;
      // (0, -1.72496 m , 8.36196 m)
      //3.03749 m, 5.82685 m
      // Vec3(-1.55176,2.2521, 5.82685);
      //  
      // Set initial linear velocity (slight sideways and downward motion)
      //mesh2->setLinearVel( -0.3, 0.2, -0.1 );
      
   }
   catch (const std::exception& e) {
      std::cerr << "ERROR loading mesh files: " << e.what() << std::endl;
      std::cout << "Note: Make sure the mesh files exist in the current directory or provide valid paths." << std::endl;
      std::cout << "The simulation will continue without mesh objects for demonstration purposes." << std::endl;
   }

   // Setup of the VTK visualization
   if( vtk && (mesh1 || mesh2) ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview_mesh_sim", visspacing, 0, timesteps, false);
      std::cout << "\nVTK visualization enabled - output will be written to ./paraview_mesh_sim" << std::endl;
   }

   // Add some additional objects for interaction if mesh loading failed
   if (!mesh1 && !mesh2) {
      std::cout << "\nAdding fallback simulation objects..." << std::endl;
      SphereID sphere1 = createSphere( ++id, 0.0, 0.0, 3.0, 0.5, mesh1Material );
      SphereID sphere2 = createSphere( ++id, 1.5, 0.5, 4.0, 0.3, mesh2Material );
      sphere1->setLinearVel( 0.2, 0.0, -0.1 );
      sphere2->setLinearVel( -0.1, 0.1, -0.2 );
      std::cout << "Fallback spheres created for demonstration" << std::endl;
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "STARTING PHYSICS SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps+1 << "   " << std::flush;
      
      // Advance physics simulation by one time step
      world->simulationStep( timestep_size );
      
      // Print object positions periodically
      if( timestep % 20 == 0 ) {
         std::cout << std::endl;
         std::cout << "Timestep " << timestep << " (t=" << timestep * timestep_size << "s):" << std::endl;
         
         if (mesh1) {
            std::cout << "  Mesh 1 position: " << mesh1->getPosition() 
                      << ", velocity: " << mesh1->getLinearVel() << std::endl;
         }
         if (mesh2) {
            std::cout << "  Mesh 2 position: " << mesh2->getPosition() 
                      << ", velocity: " << mesh2->getLinearVel() << std::endl;
         }
      }
   }

   std::cout << "\n\n--" << pe_BROWN << "SIMULATION COMPLETED" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;
   
   if (mesh1 || mesh2) {
      std::cout << "Final object states:" << std::endl;
      if (mesh1) {
         std::cout << "  Mesh 1 final position: " << mesh1->getPosition() << std::endl;
         std::cout << "  Mesh 1 final velocity: " << mesh1->getLinearVel() << std::endl;
      }
      if (mesh2) {
         std::cout << "  Mesh 2 final position: " << mesh2->getPosition() << std::endl; 
         std::cout << "  Mesh 2 final velocity: " << mesh2->getLinearVel() << std::endl;
      }
   }

   std::cout << "Total simulation time: " << timesteps * timestep_size << " seconds" << std::endl;
   
   if (vtk) {
      std::cout << "VTK output files written to ./paraview_mesh_sim/ directory" << std::endl;
   }

#else
   std::cout << "This example requires CGAL support. Please rebuild PE with -DCGAL=ON" << std::endl;
   return 1;
#endif

   return 0;
}
//*************************************************************************************************