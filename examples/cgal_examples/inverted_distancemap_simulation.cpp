//=================================================================================================
/*!
 *  \file inverted_distancemap_simulation.cpp
 *  \brief Example demonstrating inverted DistanceMap for domain boundary containment
 *
 *  This example demonstrates the use of an inverted DistanceMap for representing domain
 *  boundaries. A triangle mesh "boundary.obj" is loaded as a containment vessel, and its
 *  DistanceMap is inverted so that:
 *  - Positive distances indicate the interior (valid domain)
 *  - Negative distances indicate the exterior
 *  - Normals point inward to keep particles inside the domain
 *
 *  A sphere is placed at the origin (0,0,0), which by design of boundary.obj is inside
 *  the domain. The simulation then runs, demonstrating particle containment.
 *
 *  Configuration is loaded from "example.json" using SimulationConfig::loadFromFile().
 *
 *  Copyright (C) 2026 PE Physics Engine
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
#include <iomanip>
#include <vector>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/vtk/DistanceMapWriter.h>
#include <pe/config/SimulationConfig.h>

using namespace pe;


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the inverted DistanceMap simulation example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * This example demonstrates:
 * 1. Loading simulation parameters from a JSON configuration file
 * 2. Creating a boundary mesh and computing an inverted DistanceMap
 * 3. Placing a sphere inside the boundary domain
 * 4. Running a physics simulation where the sphere is contained by the boundary
 */
int main( int argc, char* argv[] )
{
#ifdef PE_USE_CGAL
   //==============================================================================================
   // Configuration Loading
   //==============================================================================================

   // Load configuration from JSON file
   std::string configFile = "example.json";

   // Allow override via command line
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "config", value<std::string>()->default_value("example.json"), "JSON configuration file" )
      ( "boundary", value<std::string>()->default_value("boundary.obj"), "boundary mesh file" )
      ( "sphere-radius", value<real>()->default_value(0.05), "radius of the test sphere" )
      ( "dm-resolution", value<int>()->default_value(64), "DistanceMap grid resolution" )
      ( "dm-tolerance", value<int>()->default_value(6), "DistanceMap tolerance layers" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   configFile = vm["config"].as<std::string>();
   std::string boundaryFile = vm["boundary"].as<std::string>();
   real sphereRadius = vm["sphere-radius"].as<real>();
   int dmResolution = vm["dm-resolution"].as<int>();
   int dmTolerance = vm["dm-tolerance"].as<int>();

   // Load simulation configuration
   std::cout << "\n--" << pe_BROWN << "INVERTED DISTANCEMAP SIMULATION" << pe_OLDCOLOR
             << "--------------------------------------------" << std::endl;
   std::cout << "Loading configuration from: " << configFile << std::endl;

   try {
      SimulationConfig::loadFromFile(configFile);
   } catch (const std::exception& e) {
      std::cerr << "Warning: Could not load config file '" << configFile << "': " << e.what() << std::endl;
      std::cout << "Using default configuration values." << std::endl;
   }

   auto& config = SimulationConfig::getInstance();

   // Extract simulation parameters from config
   const size_t timesteps = config.getTimesteps();
   const real timestepSize = config.getStepsize();
   const unsigned int visspacing = config.getVisspacing();
   const Vec3 gravity = config.getGravity();

   std::cout << "\nSimulation parameters:" << std::endl;
   std::cout << "  Timesteps:     " << timesteps << std::endl;
   std::cout << "  Timestep size: " << timestepSize << std::endl;
   std::cout << "  Visspacing:    " << visspacing << std::endl;
   std::cout << "  Gravity:       " << gravity << std::endl;
   std::cout << "  Boundary file: " << boundaryFile << std::endl;
   std::cout << "  Sphere radius: " << sphereRadius << std::endl;

   //==============================================================================================
   // World Setup
   //==============================================================================================

   unsigned int id = 0;

   WorldID world = theWorld();
   world->setGravity( gravity );
   world->setDamping( 1.0 );

   std::cout << "\nWorld initialized with gravity: " << world->getGravity() << std::endl;

   // Create materials
   MaterialID boundaryMaterial = createMaterial("boundary_mat", 2.5, 0.0, 0.3, 0.05, 0.2, 80, 100, 10, 11);
   MaterialID sphereMaterial = createMaterial("sphere_mat", config.getParticleDensity(), 0.0, 0.2, 0.1, 0.2, 80, 100, 10, 11);

   //==============================================================================================
   // Boundary Mesh with Inverted DistanceMap
   //==============================================================================================

   std::cout << "\n--" << pe_BROWN << "DOMAIN BOUNDARY SETUP" << pe_OLDCOLOR
             << "------------------------------------------------------" << std::endl;

   TriangleMeshID boundaryMesh = nullptr;
   //Vec3 boundaryPos(-0.000168,-2.256, 0.087286);  // Boundary mesh in reference position
   //Vec3 boundaryPos( 0.021973,-2.23205, 0.091003);  // Boundary mesh in reference position
   //Vec3 boundaryPos(-0.000001,-2.23203, 0.087193);  // Boundary mesh in reference position
   Vec3 boundaryPos( 0.000006,-2.25096, 0.08719);  // Boundary mesh in reference position

   try {
      std::cout << "Loading boundary mesh from: " << boundaryFile << std::endl;

      // Create the boundary mesh at the origin (fixed, global)
      boundaryMesh = createTriangleMesh(id++, Vec3(0.0, 0.0, 0.0), boundaryFile, boundaryMaterial, false, true);
      boundaryMesh->setFixed(true);
      boundaryMesh->setPosition(boundaryPos);

      std::cout << "Boundary mesh created with " << boundaryMesh->getBFVertices().size()
                << " vertices at position: " << boundaryMesh->getPosition() << std::endl;

      // Enable DistanceMap acceleration
      std::cout << "\nComputing DistanceMap (resolution=" << dmResolution
                << ", tolerance=" << dmTolerance << ")..." << std::endl;

      boundaryMesh->enableDistanceMapAcceleration(dmResolution, dmTolerance);

      if (!boundaryMesh->hasDistanceMap()) {
         std::cerr << "ERROR: Failed to create DistanceMap for boundary mesh!" << std::endl;
         return 1;
      }

      const DistanceMap* dm = boundaryMesh->getDistanceMap();
      std::cout << "DistanceMap created successfully:" << std::endl;
      std::cout << "  Grid size: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
      std::cout << "  Origin:    (" << dm->getOrigin()[0] << ", "
                << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
      std::cout << "  Spacing:   " << dm->getSpacing() << std::endl;

      // Write the original (non-inverted) DistanceMap for comparison
      vtk::DistanceMapWriter::writeVTI("boundary_distancemap_original.vti", *dm);
      std::cout << "\nOriginal DistanceMap written to: boundary_distancemap_original.vti" << std::endl;

      // Invert the DistanceMap for domain boundary representation
      std::cout << "\nInverting DistanceMap for domain boundary representation..." << std::endl;
      std::cout << "  Before inversion: inside mesh = negative distance" << std::endl;
      std::cout << "  After inversion:  inside mesh = positive distance (valid domain)" << std::endl;

      boundaryMesh->getDistanceMap()->invertForDomainBoundary();

      std::cout << "DistanceMap inverted successfully!" << std::endl;

      // Write the inverted DistanceMap
      vtk::DistanceMapWriter::writeVTI("boundary_distancemap_inverted.vti", *boundaryMesh->getDistanceMap());
      std::cout << "Inverted DistanceMap written to: boundary_distancemap_inverted.vti" << std::endl;

   } catch (const std::exception& e) {
      std::cerr << "ERROR loading boundary mesh: " << e.what() << std::endl;
      std::cerr << "Make sure '" << boundaryFile << "' exists in the current directory." << std::endl;
      return 1;
   }

   //==============================================================================================
   // Sphere Creation (inside the boundary domain)
   //==============================================================================================

   std::cout << "\n--" << pe_BROWN << "SPHERE SETUP" << pe_OLDCOLOR
             << "-------------------------------------------------------------" << std::endl;

   // Create a sphere at the origin - by design of boundary.obj, this is inside the domain
   Vec3 spherePosition(1.5878, 0.47876, 0.0);
   spherePosition += boundaryPos;  // Adjust position according to boundary mesh offset
   SphereID sphere = createSphere(id++, spherePosition, sphereRadius, sphereMaterial);

   std::cout << "Sphere created:" << std::endl;
   std::cout << "  Position: " << sphere->getPosition() << std::endl;
   std::cout << "  Radius:   " << sphere->getRadius() << std::endl;
   std::cout << "  Mass:     " << sphere->getMass() << std::endl;

   // Give the sphere an initial velocity to test containment
   sphere->setLinearVel(Vec3(0.1, 0.05, 0.2));
   std::cout << "  Initial velocity: " << sphere->getLinearVel() << std::endl;

   //==============================================================================================
   // VTK Visualization Setup
   //==============================================================================================

   if (config.getVtk() && visspacing > 0) {
      vtk::WriterID vtkw = vtk::activateWriter("./paraview_inverted_dm", visspacing, 0, timesteps, false);
      std::cout << "\nVTK visualization enabled - output to ./paraview_inverted_dm/" << std::endl;
   }

   //==============================================================================================
   // Simulation Loop
   //==============================================================================================

   std::cout << "\n--" << pe_BROWN << "STARTING PHYSICS SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------" << std::endl;
   std::cout << "The sphere should remain contained within the boundary mesh." << std::endl;
   std::cout << "The inverted DistanceMap ensures collision response pushes the sphere inward.\n" << std::endl;

   for (size_t timestep = 0; timestep <= timesteps; ++timestep) {
      std::cout << "\r Timestep " << timestep << " / " << timesteps << "   " << std::flush;

      // Advance physics simulation
      world->simulationStep(timestepSize);

      // Print sphere state periodically
      if (timestep % 100 == 0 && timestep > 0) {
         std::cout << std::endl;
         std::cout << "  t=" << std::fixed << std::setprecision(4) << (timestep * timestepSize) << "s"
                   << "  pos=" << sphere->getPosition()
                   << "  vel=" << sphere->getLinearVel() << std::endl;
      }
   }

   //==============================================================================================
   // Final Results
   //==============================================================================================

   std::cout << "\n\n--" << pe_BROWN << "SIMULATION COMPLETED" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   std::cout << "\nFinal sphere state:" << std::endl;
   std::cout << "  Position: " << sphere->getPosition() << std::endl;
   std::cout << "  Velocity: " << sphere->getLinearVel() << std::endl;

   std::cout << "\nTotal simulation time: " << timesteps * timestepSize << " seconds" << std::endl;

   std::cout << "\nOutput files:" << std::endl;
   std::cout << "  - boundary_distancemap_original.vti  (before inversion)" << std::endl;
   std::cout << "  - boundary_distancemap_inverted.vti  (after inversion)" << std::endl;
   if (config.getVtk()) {
      std::cout << "  - ./paraview_inverted_dm/            (VTK time series)" << std::endl;
   }

   std::cout << "\nUse ParaView to visualize the DistanceMaps and compare the sign convention." << std::endl;

#else
   std::cout << "This example requires CGAL support. Please rebuild PE with -DCGAL=ON" << std::endl;
   return 1;
#endif

   return 0;
}
//*************************************************************************************************
