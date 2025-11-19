//=================================================================================================
/*!
 *  \file mesh_distancemap_debug.cpp
 *  \brief DistanceMap checkpoint debugging application
 *
 *  This application is designed to debug DistanceMap checkpoint/restart issues by comparing
 *  DistanceMaps restored from checkpoints with freshly created DistanceMaps from original mesh files.
 *
 *  Two modes:
 *  1. Normal mode: Run simulation and create checkpoints
 *  2. Debug mode: Load checkpoint, rebuild DistanceMap from original files, compare and exit
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
#include <iomanip>
#include <cmath>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/povray.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/BodyBinaryWriter.h>
#include <pe/core/BodyBinaryReader.h>
#include <pe/vtk/UtilityWriters.h>

using namespace pe;
using namespace pe::povray;


//*************************************************************************************************
// Helper Functions
//*************************************************************************************************

#ifdef PE_USE_CGAL

//=================================================================================================
/*!
 * \brief Compare grid metadata between two DistanceMaps
 *
 * \param dm1 First DistanceMap (typically from checkpoint)
 * \param dm2 Second DistanceMap (typically from fresh mesh file)
 * \param label1 Label for first DistanceMap
 * \param label2 Label for second DistanceMap
 * \return True if all metadata matches exactly, false otherwise
 */
bool compareDistanceMapMetadata(const DistanceMap* dm1, const DistanceMap* dm2,
                                 const std::string& label1, const std::string& label2)
{
   bool allMatch = true;

   std::cout << "\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "DISTANCEMAP METADATA COMPARISON" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

   // Compare grid dimensions
   std::cout << "\n" << pe_BLUE << "Grid Dimensions:" << pe_OLDCOLOR << std::endl;
   std::cout << "  " << std::setw(25) << std::left << label1 << ": nx=" << dm1->getNx()
             << " ny=" << dm1->getNy() << " nz=" << dm1->getNz() << std::endl;
   std::cout << "  " << std::setw(25) << std::left << label2 << ": nx=" << dm2->getNx()
             << " ny=" << dm2->getNy() << " nz=" << dm2->getNz() << std::endl;

   if (dm1->getNx() != dm2->getNx() || dm1->getNy() != dm2->getNy() || dm1->getNz() != dm2->getNz()) {
      std::cout << "  " << pe_RED << "[MISMATCH] Grid dimensions differ!" << pe_OLDCOLOR << std::endl;
      allMatch = false;
   } else {
      std::cout << "  " << pe_GREEN << "[MATCH] Grid dimensions identical" << pe_OLDCOLOR << std::endl;
   }

   // Compare spacing
   std::cout << "\n" << pe_BLUE << "Grid Spacing:" << pe_OLDCOLOR << std::endl;
   std::cout << "  " << std::setw(25) << std::left << label1 << ": " << std::setprecision(15) << dm1->getSpacing() << std::endl;
   std::cout << "  " << std::setw(25) << std::left << label2 << ": " << std::setprecision(15) << dm2->getSpacing() << std::endl;

   real spacingDiff = std::abs(dm1->getSpacing() - dm2->getSpacing());
   real spacingRelError = spacingDiff / dm1->getSpacing();

   std::cout << "  Absolute difference: " << std::scientific << spacingDiff << std::endl;
   std::cout << "  Relative error: " << std::fixed << std::setprecision(6) << (spacingRelError * 100.0) << "%" << std::endl;

   if (spacingRelError > 1e-10) {
      std::cout << "  " << pe_RED << "[MISMATCH] Spacing differs beyond tolerance (1e-10)" << pe_OLDCOLOR << std::endl;
      allMatch = false;
   } else {
      std::cout << "  " << pe_GREEN << "[MATCH] Spacing within tolerance" << pe_OLDCOLOR << std::endl;
   }

   // Compare origin
   std::cout << "\n" << pe_BLUE << "Grid Origin:" << pe_OLDCOLOR << std::endl;
   const Vec3& origin1 = dm1->getOrigin();
   const Vec3& origin2 = dm2->getOrigin();

   std::cout << "  " << std::setw(25) << std::left << label1 << ": (" << std::setprecision(15)
             << origin1[0] << ", " << origin1[1] << ", " << origin1[2] << ")" << std::endl;
   std::cout << "  " << std::setw(25) << std::left << label2 << ": (" << std::setprecision(15)
             << origin2[0] << ", " << origin2[1] << ", " << origin2[2] << ")" << std::endl;

   Vec3 originDiff = origin1 - origin2;
   real originDiffMag = originDiff.length();

   std::cout << "  Difference vector: (" << std::scientific << originDiff[0] << ", "
             << originDiff[1] << ", " << originDiff[2] << ")" << std::endl;
   std::cout << "  Magnitude: " << originDiffMag << std::endl;

   if (originDiffMag > 1e-10) {
      std::cout << "  " << pe_RED << "[MISMATCH] Origin differs beyond tolerance (1e-10)" << pe_OLDCOLOR << std::endl;
      allMatch = false;
   } else {
      std::cout << "  " << pe_GREEN << "[MATCH] Origin within tolerance" << pe_OLDCOLOR << std::endl;
   }

   // Summary
   std::cout << "\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
   if (allMatch) {
      std::cout << pe_GREEN << "RESULT: All metadata matches!" << pe_OLDCOLOR << std::endl;
   } else {
      std::cout << pe_RED << "RESULT: Metadata MISMATCHES detected!" << pe_OLDCOLOR << std::endl;
   }
   std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

   return allMatch;
}


//=================================================================================================
/*!
 * \brief Compare and report bounding boxes of two meshes
 *
 * \param mesh1 First triangle mesh
 * \param mesh2 Second triangle mesh
 * \param label1 Label for first mesh
 * \param label2 Label for second mesh
 */
void compareBoundingBoxes(TriangleMeshID mesh1, TriangleMeshID mesh2,
                         const std::string& label1, const std::string& label2)
{
   std::cout << "\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "BOUNDING BOX COMPARISON" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

   const auto& aabb1 = mesh1->getAABB();
   const auto& aabb2 = mesh2->getAABB();

   // Extract bounds
   real xmin1 = aabb1[0], ymin1 = aabb1[1], zmin1 = aabb1[2];
   real xmax1 = aabb1[3], ymax1 = aabb1[4], zmax1 = aabb1[5];

   real xmin2 = aabb2[0], ymin2 = aabb2[1], zmin2 = aabb2[2];
   real xmax2 = aabb2[3], ymax2 = aabb2[4], zmax2 = aabb2[5];

   // Compute dimensions
   real dx1 = xmax1 - xmin1;
   real dy1 = ymax1 - ymin1;
   real dz1 = zmax1 - zmin1;
   real maxDim1 = std::max({dx1, dy1, dz1});

   real dx2 = xmax2 - xmin2;
   real dy2 = ymax2 - ymin2;
   real dz2 = zmax2 - zmin2;
   real maxDim2 = std::max({dx2, dy2, dz2});

   // Report first mesh
   std::cout << "\n" << pe_BLUE << label1 << ":" << pe_OLDCOLOR << std::endl;
   std::cout << "  Min: (" << std::setprecision(15) << xmin1 << ", " << ymin1 << ", " << zmin1 << ")" << std::endl;
   std::cout << "  Max: (" << std::setprecision(15) << xmax1 << ", " << ymax1 << ", " << zmax1 << ")" << std::endl;
   std::cout << "  Dimensions: dx=" << dx1 << " dy=" << dy1 << " dz=" << dz1 << std::endl;
   std::cout << "  Largest dimension: " << maxDim1 << " (used for spacing calculation)" << std::endl;

   // Report second mesh
   std::cout << "\n" << pe_BLUE << label2 << ":" << pe_OLDCOLOR << std::endl;
   std::cout << "  Min: (" << std::setprecision(15) << xmin2 << ", " << ymin2 << ", " << zmin2 << ")" << std::endl;
   std::cout << "  Max: (" << std::setprecision(15) << xmax2 << ", " << ymax2 << ", " << zmax2 << ")" << std::endl;
   std::cout << "  Dimensions: dx=" << dx2 << " dy=" << dy2 << " dz=" << dz2 << std::endl;
   std::cout << "  Largest dimension: " << maxDim2 << " (used for spacing calculation)" << std::endl;

   // Compare
   std::cout << "\n" << pe_BLUE << "Differences:" << pe_OLDCOLOR << std::endl;
   std::cout << "  Min point delta: (" << std::scientific
             << (xmin1 - xmin2) << ", " << (ymin1 - ymin2) << ", " << (zmin1 - zmin2) << ")" << std::endl;
   std::cout << "  Max point delta: (" << std::scientific
             << (xmax1 - xmax2) << ", " << (ymax1 - ymax2) << ", " << (zmax1 - zmax2) << ")" << std::endl;
   std::cout << "  Dimension delta: (" << std::scientific
             << (dx1 - dx2) << ", " << (dy1 - dy2) << ", " << (dz1 - dz2) << ")" << std::endl;
   std::cout << "  Largest dimension delta: " << std::scientific << (maxDim1 - maxDim2) << std::endl;

   real maxDimRelError = std::abs(maxDim1 - maxDim2) / maxDim1;
   std::cout << "  Largest dimension relative error: " << std::fixed << std::setprecision(6)
             << (maxDimRelError * 100.0) << "%" << std::endl;

   if (maxDimRelError > 1e-10) {
      std::cout << "  " << pe_RED << "[WARNING] Bounding boxes differ!" << pe_OLDCOLOR << std::endl;
      std::cout << "  " << pe_RED << "This will cause different spacing calculations!" << pe_OLDCOLOR << std::endl;
   } else {
      std::cout << "  " << pe_GREEN << "[OK] Bounding boxes match within tolerance" << pe_OLDCOLOR << std::endl;
   }

   std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
}


//=================================================================================================
/*!
 * \brief Run DistanceMap comparison in debug mode
 *
 * \param checkpointMesh Mesh loaded from checkpoint (with restored DistanceMap)
 * \param freshMesh Mesh loaded from original file (with fresh DistanceMap)
 * \param meshLabel Label for this mesh (e.g., "Mesh 1")
 * \param resolution Resolution parameter used for DistanceMap
 * \param tolerance Tolerance parameter used for DistanceMap
 */
void runDistanceMapComparison(TriangleMeshID checkpointMesh, TriangleMeshID freshMesh,
                              const std::string& meshLabel, int resolution, int tolerance)
{
   std::cout << "\n\n" << pe_BROWN << "###############################################" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "### " << meshLabel << " COMPARISON" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "###############################################" << pe_OLDCOLOR << std::endl;

   std::cout << "\nDistanceMap Parameters:" << std::endl;
   std::cout << "  Resolution: " << resolution << std::endl;
   std::cout << "  Tolerance: " << tolerance << std::endl;

   // Compare bounding boxes
   compareBoundingBoxes(checkpointMesh, freshMesh,
                       "Checkpoint Mesh", "Fresh Mesh");

   // Get DistanceMaps
   const DistanceMap* checkpointDM = checkpointMesh->getDistanceMap();
   const DistanceMap* freshDM = freshMesh->getDistanceMap();

   if (!checkpointDM) {
      std::cerr << pe_RED << "ERROR: Checkpoint mesh has no DistanceMap!" << pe_OLDCOLOR << std::endl;
      return;
   }

   if (!freshDM) {
      std::cerr << pe_RED << "ERROR: Fresh mesh has no DistanceMap!" << pe_OLDCOLOR << std::endl;
      return;
   }

   // Compare metadata
   bool metadataMatch = compareDistanceMapMetadata(checkpointDM, freshDM,
                                                   "Checkpoint DistanceMap", "Fresh DistanceMap");

   // Export VTI files for visual comparison
   std::cout << "\n" << pe_BLUE << "Exporting VTI files for visualization:" << pe_OLDCOLOR << std::endl;

   std::string checkpointVTI = meshLabel + "_checkpoint_dm.vti";
   std::string freshVTI = meshLabel + "_fresh_dm.vti";

   try {
      pe::vtk::DistanceMapWriter::writeVTI(checkpointVTI, *checkpointDM);
      std::cout << "  Wrote: " << checkpointVTI << std::endl;

      pe::vtk::DistanceMapWriter::writeVTI(freshVTI, *freshDM);
      std::cout << "  Wrote: " << freshVTI << std::endl;

      std::cout << "\n" << pe_GREEN << "Open these files in ParaView to visually compare the DistanceMaps" << pe_OLDCOLOR << std::endl;
   } catch (const std::exception& e) {
      std::cerr << pe_RED << "ERROR exporting VTI files: " << e.what() << pe_OLDCOLOR << std::endl;
   }
}

#endif // PE_USE_CGAL


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!
 * \brief Main function for the DistanceMap debugging application.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the application.
 */
int main( int argc, char* argv[] )
{
#ifdef PE_USE_CGAL
   // Constants and variables
   const unsigned int timesteps ( 2501 );    // Total number of time steps
   const unsigned int visspacing(   10 );   // Spacing between two visualizations
   const real timestep_size( 0.001 );       // Size of each simulation time step
         unsigned int id( 0 );              // User-specific ID counter

   // Visualization variables
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "mesh1", value<std::string>()->default_value(""), "first mesh file to be loaded" )
     ( "mesh2", value<std::string>()->default_value(""), "second mesh file to be loaded" )
     ( "checkpoint-spacing", value<unsigned int>()->default_value(500), "write checkpoint every N timesteps (0 = disabled)" )
     ( "restart-from", value<std::string>()->default_value(""), "restart from checkpoint file (enables DEBUG MODE)" )
     ( "resolution", value<int>()->default_value(64), "DistanceMap resolution parameter" )
     ( "tolerance", value<int>()->default_value(6), "DistanceMap tolerance parameter" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   std::string mesh1File(vm["mesh1"].as<std::string>());
   std::string mesh2File(vm["mesh2"].as<std::string>());
   unsigned int checkpointSpacing = vm["checkpoint-spacing"].as<unsigned int>();
   std::string restartFile = vm["restart-from"].as<std::string>();
   int dmResolution = vm["resolution"].as<int>();
   int dmTolerance = vm["tolerance"].as<int>();

   bool debugMode = !restartFile.empty();

   // Validation
   if (mesh1File.empty() || mesh2File.empty()) {
      std::cerr << "ERROR: Both --mesh1 and --mesh2 must be specified!" << std::endl;
      std::cout << "\nUsage:" << std::endl;
      std::cout << "  Normal mode (create checkpoints):" << std::endl;
      std::cout << "    " << argv[0] << " --mesh1 <file1.obj> --mesh2 <file2.obj> --checkpoint-spacing 500" << std::endl;
      std::cout << "\n  Debug mode (compare DistanceMaps):" << std::endl;
      std::cout << "    " << argv[0] << " --restart-from <checkpoint.peb> --mesh1 <file1.obj> --mesh2 <file2.obj>" << std::endl;
      return 1;
   }

   std::cout << "\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "DISTANCEMAP CHECKPOINT DEBUG TOOL" << pe_OLDCOLOR << std::endl;
   std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

   if (debugMode) {
      std::cout << "\n" << pe_GREEN << "MODE: DEBUG (Compare DistanceMaps)" << pe_OLDCOLOR << std::endl;
      std::cout << "Checkpoint file: " << restartFile << std::endl;
   } else {
      std::cout << "\n" << pe_GREEN << "MODE: NORMAL (Run simulation)" << pe_OLDCOLOR << std::endl;
   }

   std::cout << "Mesh 1: " << mesh1File << std::endl;
   std::cout << "Mesh 2: " << mesh2File << std::endl;
   std::cout << "DistanceMap Resolution: " << dmResolution << std::endl;
   std::cout << "DistanceMap Tolerance: " << dmTolerance << std::endl;

   if (!debugMode && checkpointSpacing > 0) {
      std::cout << "Checkpointing enabled: every " << checkpointSpacing << " timesteps" << std::endl;
   }

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   theCollisionSystem()->setErrorReductionParameter(0.00);

   // Create materials
   MaterialID groundMaterial = createMaterial("ground", 2.5, 0.0, 0.3, 0.05, 0.2, 80, 100, 10, 11);
   MaterialID mesh1Material = createMaterial("mesh1_mat", 1.2, 0.0, 0.2, 0.1, 0.2, 80, 100, 10, 11);
   MaterialID mesh2Material = createMaterial("mesh2_mat", 0.8, 0.0, 0.25, 0.08, 0.2, 80, 100, 10, 11);

   // Initialize checkpoint writer
   BodyBinaryWriter checkpointWriter;

   // Load mesh objects
   TriangleMeshID mesh1Checkpoint = nullptr;
   TriangleMeshID mesh2Checkpoint = nullptr;
   TriangleMeshID mesh1Fresh = nullptr;
   TriangleMeshID mesh2Fresh = nullptr;
   PlaneID groundPlane = nullptr;

   if (debugMode) {
      // ===== DEBUG MODE =====
      std::cout << "\n" << pe_BLUE << "Loading checkpoint..." << pe_OLDCOLOR << std::endl;

      try {
         BodyBinaryReader checkpointReader;
         checkpointReader.readFile(restartFile.c_str());

         std::cout << "Checkpoint loaded successfully!" << std::endl;
         std::cout << "Total bodies in world: " << world->size() << std::endl;

         // Find meshes in loaded world
         for (World::Iterator it = world->begin(); it != world->end(); ++it) {
            if (it->getType() == triangleMeshType) {
               TriangleMeshID tmesh = static_cast<TriangleMeshID>(*it);
               if (!mesh1Checkpoint) {
                  mesh1Checkpoint = tmesh;
                  std::cout << "Found mesh 1 from checkpoint: " << mesh1Checkpoint->getBFVertices().size() << " vertices" << std::endl;
                  std::cout << "  DistanceMap enabled: " << (mesh1Checkpoint->hasDistanceMap() ? "YES" : "NO") << std::endl;
               } else if (!mesh2Checkpoint) {
                  mesh2Checkpoint = tmesh;
                  std::cout << "Found mesh 2 from checkpoint: " << mesh2Checkpoint->getBFVertices().size() << " vertices" << std::endl;
                  std::cout << "  DistanceMap enabled: " << (mesh2Checkpoint->hasDistanceMap() ? "YES" : "NO") << std::endl;
               }
            }
            else if (it->getType() == planeType) {
               groundPlane = static_cast<PlaneID>(*it);
            }
         }

         if (!mesh1Checkpoint || !mesh2Checkpoint) {
            std::cerr << pe_RED << "ERROR: Could not find both meshes in checkpoint!" << pe_OLDCOLOR << std::endl;
            return 1;
         }

      } catch (const std::exception& e) {
         std::cerr << pe_RED << "ERROR loading checkpoint: " << e.what() << pe_OLDCOLOR << std::endl;
         return 1;
      }

      // Now create fresh meshes from files in the same world
      std::cout << "\n" << pe_BLUE << "Creating fresh meshes from original files..." << pe_OLDCOLOR << std::endl;

      // Create ground plane
      groundPlane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, groundMaterial );
      std::cout << "Ground plane created" << std::endl;

      try {
         // Load mesh 1 fresh
         std::cout << "Loading fresh mesh 1: " << mesh1File << std::endl;
         mesh1Fresh = createTriangleMesh(1000, mesh1Checkpoint->getPosition(), mesh1File, mesh1Material, false, true);
         mesh1Fresh->setFixed(true);
         mesh1Fresh->setOrientation(mesh1Checkpoint->getQuaternion());
         mesh1Fresh->enableDistanceMapAcceleration(dmResolution, dmTolerance);
         std::cout << "Fresh mesh 1 created: " << mesh1Fresh->getBFVertices().size() << " vertices" << std::endl;
         std::cout << "  DistanceMap enabled: " << (mesh1Fresh->hasDistanceMap() ? "YES" : "NO") << std::endl;

         // Load mesh 2 fresh
         std::cout << "Loading fresh mesh 2: " << mesh2File << std::endl;
         mesh2Fresh = createTriangleMesh(2000, mesh2Checkpoint->getPosition(), mesh2File, mesh2Material, false, true);
         mesh2Fresh->setOrientation(mesh2Checkpoint->getQuaternion());
         mesh2Fresh->enableDistanceMapAcceleration(dmResolution, dmTolerance);
         std::cout << "Fresh mesh 2 created: " << mesh2Fresh->getBFVertices().size() << " vertices" << std::endl;
         std::cout << "  DistanceMap enabled: " << (mesh2Fresh->hasDistanceMap() ? "YES" : "NO") << std::endl;

      } catch (const std::exception& e) {
         std::cerr << pe_RED << "ERROR loading fresh mesh files: " << e.what() << pe_OLDCOLOR << std::endl;
         return 1;
      }

      // Now compare the DistanceMaps
      std::cout << "\n\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "STARTING DISTANCEMAP COMPARISON" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

      // Compare mesh 1
      if (mesh1Checkpoint->hasDistanceMap() && mesh1Fresh->hasDistanceMap()) {
         runDistanceMapComparison(mesh1Checkpoint, mesh1Fresh, "Mesh1", dmResolution, dmTolerance);
      } else {
         std::cerr << pe_RED << "ERROR: Mesh 1 missing DistanceMap in checkpoint or fresh version!" << pe_OLDCOLOR << std::endl;
      }

      // Compare mesh 2
      if (mesh2Checkpoint->hasDistanceMap() && mesh2Fresh->hasDistanceMap()) {
         runDistanceMapComparison(mesh2Checkpoint, mesh2Fresh, "Mesh2", dmResolution, dmTolerance);
      } else {
         std::cerr << pe_RED << "ERROR: Mesh 2 missing DistanceMap in checkpoint or fresh version!" << pe_OLDCOLOR << std::endl;
      }

      std::cout << "\n\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "DEBUG MODE COMPARISON COMPLETE" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
      std::cout << "\nCheck the generated .vti files in ParaView for visual comparison." << std::endl;

      // Remove checkpoint meshes and keep fresh meshes for simulation
      // This tests if the checkpoint-restored DistanceMaps are causing issues
      std::cout << "\n" << pe_BLUE << "Removing checkpoint meshes, keeping fresh meshes for simulation..." << pe_OLDCOLOR << std::endl;

      if (mesh1Checkpoint) {
         // Find the checkpoint mesh in the world and destroy it
         for (World::Iterator it = world->begin(); it != world->end(); ++it) {
            if (it->getID() == mesh1Checkpoint->getID()) {
               world->destroy(it);
               std::cout << "  Removed mesh 1 checkpoint (ID: " << mesh1Checkpoint->getID() << ")" << std::endl;
               mesh1Checkpoint = nullptr;
               break;
            }
         }
      }

      if (mesh2Checkpoint) {
         // Find the checkpoint mesh in the world and destroy it
         for (World::Iterator it = world->begin(); it != world->end(); ++it) {
            if (it->getID() == mesh2Checkpoint->getID()) {
               world->destroy(it);
               std::cout << "  Removed mesh 2 checkpoint (ID: " << mesh2Checkpoint->getID() << ")" << std::endl;
               mesh2Checkpoint = nullptr;
               break;
            }
         }
      }

      // Reassign fresh meshes to checkpoint variables for simulation loop
      mesh1Checkpoint = mesh1Fresh;
      mesh2Checkpoint = mesh2Fresh;

      std::cout << "Bodies remaining in world: " << world->size() << std::endl;
      std::cout << "\n" << pe_GREEN << "Now continuing with simulation using FRESH meshes..." << pe_OLDCOLOR << std::endl;

   } else {
      // ===== NORMAL MODE =====
      std::cout << "\n" << pe_BLUE << "Creating simulation..." << pe_OLDCOLOR << std::endl;

      // Create ground plane
      groundPlane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, groundMaterial );
      std::cout << "Ground plane created" << std::endl;

      try {
         // Load mesh 1
         std::cout << "Loading mesh 1: " << mesh1File << std::endl;
         mesh1Checkpoint = createTriangleMesh(++id, Vec3(0.0, 0.0, 0.0), mesh1File, mesh1Material, false, true);
         mesh1Checkpoint->setFixed(true);
         mesh1Checkpoint->enableDistanceMapAcceleration(dmResolution, dmTolerance);
         std::cout << "Mesh 1 created: " << mesh1Checkpoint->getBFVertices().size() << " vertices" << std::endl;
         std::cout << "  DistanceMap enabled: " << (mesh1Checkpoint->hasDistanceMap() ? "YES" : "NO") << std::endl;

         // Load mesh 2
         std::cout << "Loading mesh 2: " << mesh2File << std::endl;


         mesh2Checkpoint = createTriangleMesh(++id, Vec3(0.0, 0.0, 7.695888000000004e-02), mesh2File, mesh2Material, false, true);
         //mesh2Checkpoint = createTriangleMesh(++id, Vec3(0.0, 0.0, 0.1275), mesh2File, mesh2Material, false, true);
         mesh2Checkpoint->enableDistanceMapAcceleration(dmResolution, dmTolerance);
         std::cout << "Mesh 2 created: " << mesh2Checkpoint->getBFVertices().size() << " vertices" << std::endl;
         std::cout << "  DistanceMap enabled: " << (mesh2Checkpoint->hasDistanceMap() ? "YES" : "NO") << std::endl;

      } catch (const std::exception& e) {
         std::cerr << pe_RED << "ERROR loading mesh files: " << e.what() << pe_OLDCOLOR << std::endl;
         return 1;
      }
   }

   // Setup VTK visualization
   if (vtk) {
      std::string vtkPath = debugMode ? "./paraview_mesh_debug_restart" : "./paraview_mesh_debug";
      vtk::WriterID vtkw = vtk::activateWriter( vtkPath.c_str(), visspacing, 0, timesteps, false);
      std::cout << "\nVTK visualization enabled - output: " << vtkPath << std::endl;
   }

   // Simulation loop
   {
      std::cout << "\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
      if (debugMode) {
         std::cout << pe_BROWN << "STARTING SIMULATION (DEBUG MODE - from checkpoint)" << pe_OLDCOLOR << std::endl;
      } else {
         std::cout << pe_BROWN << "STARTING SIMULATION (NORMAL MODE)" << pe_OLDCOLOR << std::endl;
      }
      std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

      for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
         std::cout << "\r Time step " << timestep+1 << " of " << timesteps+1 << "   " << std::flush;

         // Advance physics simulation
         world->simulationStep( timestep_size );

         // Write checkpoint if enabled (disabled in debug mode)
         if (!debugMode && checkpointSpacing > 0 && timestep > 0 && timestep % checkpointSpacing == 0) {
            std::cout << std::endl;
            std::ostringstream checkpointName;
            checkpointName << "checkpoint_" << std::setfill('0') << std::setw(6) << timestep << ".peb";

            std::cout << "Writing checkpoint: " << checkpointName.str() << std::endl;
            try {
               checkpointWriter.writeFileAsync(checkpointName.str().c_str());
               checkpointWriter.wait();
               std::cout << "Checkpoint written successfully" << std::endl;
            } catch (const std::exception& e) {
               std::cerr << pe_RED << "ERROR writing checkpoint: " << e.what() << pe_OLDCOLOR << std::endl;
            }
         }

         // Print positions periodically
         if( timestep % 20 == 0 ) {
            std::cout << std::endl;
            std::cout << "Timestep " << timestep << " (t=" << timestep * timestep_size << "s):" << std::endl;
            if (mesh1Checkpoint) {
               std::cout << "  Mesh 1: pos=" << mesh1Checkpoint->getPosition()
                         << " vel=" << mesh1Checkpoint->getLinearVel() << std::endl;
            }
            if (mesh2Checkpoint) {
               std::cout << "  Mesh 2: pos=" << mesh2Checkpoint->getPosition()
                         << " vel=" << mesh2Checkpoint->getLinearVel() << std::endl;
            }
         }
      }

      std::cout << "\n\n" << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "SIMULATION COMPLETE" << pe_OLDCOLOR << std::endl;
      std::cout << pe_BROWN << "========================================" << pe_OLDCOLOR << std::endl;

      if (mesh1Checkpoint || mesh2Checkpoint) {
         std::cout << "Final states:" << std::endl;
         if (mesh1Checkpoint) {
            std::cout << "  Mesh 1: pos=" << mesh1Checkpoint->getPosition()
                      << " vel=" << mesh1Checkpoint->getLinearVel() << std::endl;
         }
         if (mesh2Checkpoint) {
            std::cout << "  Mesh 2: pos=" << mesh2Checkpoint->getPosition()
                      << " vel=" << mesh2Checkpoint->getLinearVel() << std::endl;
         }
      }
   }

#else
   std::cout << "This application requires CGAL support. Rebuild PE with -DCGAL=ON" << std::endl;
   return 1;
#endif

   return 0;
}
//*************************************************************************************************
