//=================================================================================================
/*!
 *  \file mesh_spin.cpp
 *  \brief Example rotating a triangle mesh about the z-axis at 60 rpm using DistanceMap collision detection
 *
 *  This example is analogous to capsule_spin but uses triangle meshes with DistanceMap acceleration
 *  instead of capsules. It demonstrates collision detection and response for kinematic bodies
 *  (bodies with prescribed motion) colliding with translation-fixed bodies.
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
#include <pe/core/rigidbody/TriangleMesh.h>

using namespace pe;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mesh spin example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
#ifdef PE_USE_CGAL
   // Constants and variables
   const unsigned int timesteps( 3000 );
   const real dt( 0.0005 );
   const real rpm( 60.0 );
   const real omega( ( rpm / 60.0 ) * 2.0 * M_PI );
         unsigned int id( 0 );
   const unsigned int visspacing(   10 );  // Spacing between two visualizations (POV-Ray & Irrlicht)

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );

   // Create materials
   MaterialID meshMaterial = createMaterial("mesh_mat", 1.0, 0.0, 0.3, 0.1, 0.2, 80, 100, 10, 11);

   std::cout << "\n--" << pe_BROWN << "MESH SPIN (DistanceMap)" << pe_OLDCOLOR
             << "---------------------------------------------------" << std::endl;

   // Create first mesh (kinematic body - fixed with prescribed rotation)
   // Using a simple elongated box mesh similar to capsule shape
   std::cout << "Creating mesh 1 (kinematic rotor)..." << std::endl;
   TriangleMeshID mesh1 = nullptr;

   try {
      // Try to load from file if available, otherwise create procedurally
      mesh1 = createTriangleMesh( id++, Vec3(0.0, 0.0, 0.0), "four_leg_scaled.obj", meshMaterial, false, true );
   } catch (...) {
      std::cout << "Note: cylinder.obj not found, attempting to use cube.obj" << std::endl;
      try {
         mesh1 = createTriangleMesh( id++, Vec3(0.0, 0.0, 0.0), "../trimeshdop_demo/cube.obj", meshMaterial, false, true );
      } catch (...) {
         std::cerr << "ERROR: Could not load mesh file. Please provide cylinder.obj or ensure cube.obj is available." << std::endl;
         return 1;
      }
   }

   mesh1->setFixed( true );
   mesh1->setLinearVel( 0.0, 0.0, 0.0 );
   mesh1->setAngularVel( 0.0, 0.0, omega );

   // Enable DistanceMap acceleration
   mesh1->enableDistanceMapAcceleration(50, 5);
   std::cout << "Mesh 1 created with " << mesh1->getBFVertices().size() << " vertices" << std::endl;
   std::cout << "DistanceMap enabled: " << (mesh1->hasDistanceMap() ? "YES" : "NO") << std::endl;

   // Create second mesh (translation-fixed, can only rotate)
   std::cout << "\nCreating mesh 2 (translation-fixed)..." << std::endl;
   TriangleMeshID mesh2 = nullptr;

   try {
      mesh2 = createTriangleMesh( id++, Vec3(0.0275, 0.0125, 0.0), "four_leg_scaled.obj", meshMaterial, false, true );
   } catch (...) {
      try {
         mesh2 = createTriangleMesh( id++, Vec3(0.0275, 0.0125, 0.0), "../trimeshdop_demo/cube.obj", meshMaterial, false, true );
      } catch (...) {
         std::cerr << "ERROR: Could not load mesh file for mesh 2." << std::endl;
         return 1;
      }
   }

   mesh2->setTranslationFixed( true );

   // Enable DistanceMap acceleration
   mesh2->enableDistanceMapAcceleration(128, 5);
   std::cout << "Mesh 2 created with " << mesh2->getBFVertices().size() << " vertices" << std::endl;
   std::cout << "DistanceMap enabled: " << (mesh2->hasDistanceMap() ? "YES" : "NO") << std::endl;

   // Setup of the VTK visualization
   vtk::WriterID vtkw = vtk::activateWriter( "./paraview_mesh", visspacing, 0, timesteps, false);

   std::cout << "\nInitial state:" << std::endl;
   std::cout << "Mesh 1 - Center: " << mesh1->getPosition()
             << " | Angular velocity: " << mesh1->getAngularVel() << std::endl;
   std::cout << "Mesh 2 - Center: " << mesh2->getPosition()
             << " | Angular velocity: " << mesh2->getAngularVel() << std::endl;

   // Simulation loop
   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   \n";
      std::cout << "Mesh 1 - Center: " << mesh1->getPosition()
                << " | Orientation: " << mesh1->getQuaternion() << "\n"
                << " | Angular velocity: " << mesh1->getAngularVel() << std::endl;
      std::cout << "Mesh 2 - Center: " << mesh2->getPosition()
                << " | Orientation: " << mesh2->getQuaternion() << "\n"
                << " | Angular velocity: " << mesh2->getAngularVel() << std::endl;
      world->simulationStep( dt );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;

   return 0;

#else
   std::cerr << "ERROR: This example requires PE to be built with CGAL support (-DCGAL=ON)" << std::endl;
   return 1;
#endif
}
//=================================================================================================
