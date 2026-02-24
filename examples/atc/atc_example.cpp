//=================================================================================================
/*!
 *  \file atc_example.cpp
 *  \brief Standalone ATC (Annular Tube Configuration) example for the PE physics engine
 *
 *  This example demonstrates the ATC simulation workflow from pe/interface/sim_setup_serial.h
 *  in a standalone context (without the FeatFloWer CFD solver). It performs the following:
 *
 *  1. Loads simulation parameters from a JSON configuration file (example.json)
 *  2. Creates a ground plane as a simple domain boundary
 *  3. Generates particles along a centerline (read from sorted_vertices_by_x_world.txt)
 *  4. Runs the time-stepping loop using stepSimulationSerial()
 *
 *  This is a pure PE example — no MPI, no CFD coupling. The "serial mode" here means
 *  PE runs as a single serial instance, the same way each CFD rank would run PE when
 *  integrated with FeatFloWer in PE_SERIAL_MODE.
 *
 *  Usage:
 *    ./atc_example
 *
 *  Required files in the working directory:
 *    - example.json                       : JSON configuration file
 *    - sorted_vertices_by_x_world.txt     : Centerline vertex file
 *
 *  Copyright (C) 2024 Raphael Muenster
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

#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/config/SimulationConfig.h>
#include <pe/util/logging/Logger.h>
#include <pe/interface/geometry_utils.h>
#include <pe/util/timing/WcTimer.h>

using namespace pe;

/**
 * @brief Result structure for centerline position calculations
 */
struct CenterlinePosition {
  Vec3 closestPoint;         // Closest point on centerline to particle
  real radialDistance;       // Euclidean distance from particle to closest point
  real lengthAlongCurve;     // Cumulative length from start to closest point
  real percentageAlongCurve; // Percentage along centerline (0-100%)
  int segmentIndex;          // Which edge contains the closest point
};

/**
 * @brief Calculate total length of a piecewise linear centerline
 * @param centerline Vector of vertices defining the centerline
 * @return Total length of the centerline
 */
inline real calculateTotalCenterlineLength(const std::vector<Vec3>& centerline) {
  real totalLength = 0.0;
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    totalLength += (centerline[i + 1] - centerline[i]).length();
  }
  return totalLength;
}

/**
 * @brief Find closest point on centerline to a given particle position
 *
 * For each line segment in the centerline, this function:
 * 1. Projects the particle position onto the infinite line containing the segment
 * 2. Clamps the projection to the segment endpoints
 * 3. Computes the distance from particle to the clamped point
 * 4. Tracks the segment with minimum distance
 *
 * @param particlePos Position of the particle in world coordinates
 * @param centerline Vector of vertices defining the centerline
 * @param totalLength Precomputed total length of the centerline
 * @return CenterlinePosition structure with geometric diagnostics
 */
inline CenterlinePosition calculateCenterlinePosition(
    const Vec3& particlePos,
    const std::vector<Vec3>& centerline,
    real totalLength) {

  // Handle empty or degenerate centerline
  if (centerline.size() < 2) {
    return CenterlinePosition{Vec3(0,0,0), 0.0, 0.0, 0.0, -1};
  }

  real minDistance = std::numeric_limits<real>::max();
  Vec3 closestPoint;
  real lengthAlongCurve = 0.0;
  real cumulativeLength = 0.0;
  int closestSegmentIndex = -1;

  // Iterate over each line segment in the centerline
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    const Vec3& v0 = centerline[i];
    const Vec3& v1 = centerline[i + 1];
    Vec3 edgeVec = v1 - v0;
    real edgeLength = edgeVec.length();

    // Skip degenerate segments
    if (edgeLength < 1e-10) {
      continue;
    }

    // Project particle onto infinite line containing edge
    // t = (AP · AB) / |AB|^2 where A=v0, B=v1, P=particlePos
    Vec3 toParticle = particlePos - v0;
    real t = trans(toParticle) * edgeVec / (edgeLength * edgeLength);

    // Clamp t to [0, 1] to constrain point to line segment
    t = std::max(real(0), std::min(real(1), t));

    // Closest point on this segment
    Vec3 pointOnSegment = v0 + edgeVec * t;
    real distance = (particlePos - pointOnSegment).length();

    // Update if this is the closest segment so far
    if (distance < minDistance) {
      minDistance = distance;
      closestPoint = pointOnSegment;
      lengthAlongCurve = cumulativeLength + (t * edgeLength);
      closestSegmentIndex = static_cast<int>(i);
    }

    cumulativeLength += edgeLength;
  }

  // Calculate percentage along centerline
  real percentageAlongCurve = (totalLength > 0.0) ?
                               (lengthAlongCurve / totalLength) * 100.0 : 0.0;

  return CenterlinePosition{
    closestPoint,
    minDistance,
    lengthAlongCurve,
    percentageAlongCurve,
    closestSegmentIndex
  };
}

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the ATC example.
 *
 * This example mirrors the setupATCSerial() + stepSimulationSerial() workflow from
 * pe/interface/sim_setup_serial.h. It demonstrates how the PE physics engine is configured
 * and run for the Annular Tube Configuration scenario.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return 0 on success.
 */
int main( int argc, char* argv[] )
{
   //==============================================================================================
   // Phase 1: Configuration Loading
   //==============================================================================================
   // In the integrated FeatFloWer+PE scenario, the CFD rank is passed from the MPI layer.
   // Here we simulate being CFD rank 1 (the representative rank).
   const int cfd_rank = 1;

   pe::logging::Logger::setCustomRank(cfd_rank);

   // Load configuration from JSON file (same mechanism as setupATCSerial)
   SimulationConfig::loadFromFile("example.json");
   auto &config = SimulationConfig::getInstance();
   config.setCfdRank(cfd_rank);
   const bool isRepresentative = (config.getCfdRank() == 1);

   std::cout << "\n"
             << "================================================================================\n"
             << "  ATC Example - PE Standalone\n"
             << "================================================================================\n"
             << std::endl;

   //==============================================================================================
   // Phase 2: World Setup (mirrors setupATCSerial)
   //==============================================================================================
   WorldID world = theWorld();

   // Apply lubrication/contact parameters from configuration
   CollisionSystemID cs = theCollisionSystem();

   // Set gravity from configuration
   world->setGravity( config.getGravity() );
   world->setDamping(1.0);

   // Fluid properties (used for buoyancy in liquid-solid mode)
   real simViscosity( config.getFluidViscosity() );
   real simRho( config.getFluidDensity() );

   world->setLiquidSolid(true);
   world->setLiquidDensity(simRho);
   world->setViscosity(simViscosity);

   TimeStep::stepsize(config.getStepsize());

   unsigned int id = 0;

   //==============================================================================================
   // Phase 3: Visualization Setup
   //==============================================================================================
   if (isRepresentative && config.getVtk()) {
      unsigned int effectiveVisspacing = config.getVisspacing() * config.getSubsteps();
      vtk::WriterID vtk = vtk::activateWriter( "./paraview", effectiveVisspacing, 0,
                                                config.getTimesteps() * config.getSubsteps(),
                                                false);
   }

   //==============================================================================================
   // Phase 4: Domain Boundary
   //==============================================================================================
   // In the full ATC setup, a triangulated tube mesh with inverted DistanceMap is used.
   // Here we use a simple ground plane as the domain floor for the standalone example.
   std::string boundaryFileName = std::string("atc_boundary_param_zero.obj");
   Vec3 boundaryPos(0.0, 0.0, 0.0);  // Boundary mesh in reference position
   MaterialID boundaryMat = createMaterial("boundary", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
 
   // Create boundary mesh as fixed global object
   TriangleMeshID boundaryMesh = createTriangleMesh(id++, boundaryPos, boundaryFileName, boundaryMat, false, true);
   boundaryMesh->setFixed(true);
   //boundaryMesh->setPosition(Vec3(0.000169, -2.256, 0.087286));
   boundaryMesh->setPosition(Vec3(0.000006, -2.25096, 0.08719));
 
   // Enable DistanceMap acceleration for domain boundary
   int dmResolution = 128;
   int dmTolerance = 6;
   boundaryMesh->enableDistanceMapAcceleration(dmResolution, dmTolerance);
 
   const bool boundaryDistanceMapEnabled = boundaryMesh->hasDistanceMap();
   const DistanceMap* boundaryDM = boundaryDistanceMapEnabled ? boundaryMesh->getDistanceMap() : nullptr;
 
   if (!boundaryDistanceMapEnabled && isRepresentative) {
     std::cerr << "WARNING: DistanceMap acceleration failed to initialize for boundary" << std::endl;
   }
 
   boundaryMesh->getDistanceMap()->invertForDomainBoundary();
   if (boundaryDistanceMapEnabled && isRepresentative) {
     // Invert the DistanceMap for domain boundary representation
     // This transforms: inside mesh -> inside fluid domain (positive distance)
     //                  outside mesh -> outside fluid domain (negative distance)
     //                  normals -> point inward to keep particles in domain
     //boundaryMesh->getDistanceMap()->invertForDomainBoundary();
     std::cout << "\n--" << "DOMAIN BOUNDARY SETUP"
               << "--------------------------------------------------------------\n"
               << " Boundary mesh file                      = " << boundaryFileName << "\n"
               << " Boundary mesh volume                    = " << boundaryMesh->getVolume() << "\n"
               << " Distance map enabled                    = yes (INVERTED for domain boundary)\n"
               << " Distance map grid size                  = "
               << boundaryDM->getNx() << " x " << boundaryDM->getNy() << " x " << boundaryDM->getNz() << "\n"
               << " Distance map origin                     = ("
               << boundaryDM->getOrigin()[0] << ", "
               << boundaryDM->getOrigin()[1] << ", "
               << boundaryDM->getOrigin()[2] << ")\n"
               << " Distance map spacing                    = " << boundaryDM->getSpacing() << "\n"
               << " Distance map resolution                 = " << dmResolution << "\n"
               << " Distance map tolerance                  = " << dmTolerance << "\n"
               << "--------------------------------------------------------------------------------\n" << std::endl;
 
     // Sanity check: Test a point that should be outside the domain
     Vec3 testPointWorld(-0.4, -2.0, 0.19);  // Outside domain, within grid bounds
     Vec3 testPointLocal = boundaryMesh->pointFromWFtoBF(testPointWorld);
     real testDistance = boundaryDM->interpolateDistance(testPointLocal[0], testPointLocal[1], testPointLocal[2]);
 
     std::cout << "\n--" << "ESCAPE DETECTION SANITY CHECK"
               << "--------------------------------------------------------------\n"
               << " Test point (world): (" << testPointWorld[0] << ", " << testPointWorld[1] << ", " << testPointWorld[2] << ")\n"
               << " Test point (local): (" << testPointLocal[0] << ", " << testPointLocal[1] << ", " << testPointLocal[2] << ")\n"
               << " Signed distance: " << testDistance << "\n"
               << " Expected: negative (outside domain)\n"
               << " Result: " << (testDistance < 0.0 ? "PASS - point correctly identified as OUTSIDE" :
                                 (testDistance > 1e5 ? "OUT OF GRID BOUNDS" : "FAIL - point incorrectly identified as INSIDE")) << "\n"
               << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   //==============================================================================================
   // Phase 5: Particle Generation along Centerline
   //==============================================================================================
   std::vector<Vec3> edges = readVectorsFromFile("sorted_vertices_by_x_world.txt");

   if (edges.empty()) {
      std::cerr << "ERROR: Could not read centerline vertices from sorted_vertices_by_x_world.txt\n"
                << "       Make sure this file exists in the working directory." << std::endl;
      return 1;
   }

   // Store centerline in SimulationConfig singleton for stuck particle diagnostics
   config.setCenterlineVertices(edges);
   config.setTotalCenterlineLength(calculateTotalCenterlineLength(edges));

   std::cout << "  Centerline: " << edges.size() << " vertices, total length = "
             << config.getTotalCenterlineLength() << "\n" << std::endl;

   real sphereRad = config.getBenchRadius();
   real rhoParticle( config.getParticleDensity() );
   MaterialID particleMaterial = createMaterial("particleMaterial", rhoParticle,
                                                 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);

   // Generate sphere positions along the centerline
   std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges, sphereRad);

   int particlesCreated = 0;
   if (config.getPackingMethod() != SimulationConfig::PackingMethod::None) {
      for (const auto& spherePos : spherePositions) {
         SphereID s = createSphere(id++, spherePos, sphereRad, particleMaterial);
         particlesCreated++;
      }
   }

   // Volume fraction computation
   real domainVol = 0.78;
   real partVol = 4.0 / 3.0 * M_PI * std::pow(sphereRad, 3);

   std::cout << "\n--" << "ATC SETUP"
             << "--------------------------------------------------------------\n"
             << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
             << " Substeps per main step                  = " << config.getSubsteps() << "\n"
             << " Total timesteps                         = " << config.getTimesteps() << "\n"
             << " Fluid viscosity                         = " << simViscosity << "\n"
             << " Fluid density                           = " << simRho << "\n"
             << " Gravity                                 = " << world->getGravity() << "\n"
             << " Total number of particles               = " << particlesCreated << "\n"
             << " Particle radius                         = " << sphereRad << "\n"
             << " Particle density                        = " << rhoParticle << "\n"
             << " Particle volume                         = " << partVol << "\n"
             << " Domain volume (reference)               = " << domainVol << "\n"
             << " Volume fraction [%]                     = " << (particlesCreated * partVol) / domainVol * 100.0 << "\n"
             << "--------------------------------------------------------------------------------\n" << std::endl;

   //==============================================================================================
   // Phase 6: Time-Stepping Loop (mirrors stepSimulationSerial)
   //==============================================================================================
   std::cout << "\n--RIGID BODY SIMULATION"
             << "---------------------------------------------------------" << std::endl;

   // Substepping configuration
   int substeps = config.getSubsteps();
   real fullStepSize = config.getStepsize();
   real substepSize = fullStepSize / static_cast<real>(substeps);

   // Timing instrumentation
   timing::WcTimer timerTotal;
   timing::WcTimer timerForceApplication;
   timing::WcTimer timerSimulation;
   timing::WcTimer timerSphereOutput;
   timing::WcTimer timerDiagnostics;
   timing::WcTimer timerOverall;

   timerOverall.start();

   for (size_t timestep = 0; timestep < config.getTimesteps(); ++timestep) {

      timerTotal.start();

      //===========================================================================================
      // Step 6a: Apply external forces (fluid drag) with FULL timestep BEFORE substepping
      //===========================================================================================
      // In the integrated scenario, fluid forces are set by the CFD code before this point.
      // Here we have no CFD, so forces are zero (only gravity acts).
      timerForceApplication.start();
      for (auto it = theCollisionSystem()->getBodyStorage().begin();
           it != theCollisionSystem()->getBodyStorage().end(); ++it) {
         BodyID body = *it;
         // Apply fluid forces using the library function with full main timestep
         body->applyFluidForces(fullStepSize);
      }
      timerForceApplication.end();

      //===========================================================================================
      // Step 6b: Execute substeps (collision handling and gravity integration)
      //===========================================================================================
      TimeStep::stepsize(substepSize);

      timerSimulation.start();
      for (int istep = 0; istep < substeps; ++istep) {
         world->simulationStep(substepSize);
      }
      timerSimulation.end();

      // Restore original timestep size
      TimeStep::stepsize(fullStepSize);

      //===========================================================================================
      // Step 6c: Sphere position output (every 50th timestep)
      //===========================================================================================
      timerSphereOutput.start();
      if (timestep == 0 || timestep % 50 == 0) {
         mkdir("spheres", 0755);

         std::ostringstream filename;
         filename << "spheres/spheres_" << timestep << ".txt";

         std::ofstream outfile(filename.str().c_str());
         if (outfile.is_open()) {
            outfile << "# Timestep: " << timestep << "\n";
            outfile << "# Format: sphere_id x y z\n";

            for (auto it = theCollisionSystem()->getBodyStorage().begin();
                 it != theCollisionSystem()->getBodyStorage().end(); ++it) {
               BodyID body = *it;
               if (body->getType() == sphereType) {
                  Vec3 pos = body->getPosition();
                  outfile << body->getSystemID() << " "
                          << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
               }
            }
            outfile.close();
         }
      }
      timerSphereOutput.end();

      //===========================================================================================
      // Step 6d: Particle diagnostics output
      //===========================================================================================
      timerDiagnostics.start();

      // Count particles
      size_t numBodies = theCollisionSystem()->getBodyStorage().size();
      size_t numSpheres = 0;
      for (auto it = theCollisionSystem()->getBodyStorage().begin();
           it != theCollisionSystem()->getBodyStorage().end(); ++it) {
         if ((*it)->getType() == sphereType) numSpheres++;
      }
      timerDiagnostics.end();

      timerTotal.end();

      //===========================================================================================
      // Step 6e: Timing report
      //===========================================================================================
      double tTotal     = timerTotal.last();
      double tForce     = timerForceApplication.last();
      double tSim       = timerSimulation.last();
      double tSphereOut = timerSphereOutput.last();
      double tDiag      = timerDiagnostics.last();

      auto pct = [&](double t) -> double {
         return (tTotal > 0.0) ? (t / tTotal * 100.0) : 0.0;
      };

      std::cout << "\n"
                << "================================================================================\n"
                << "  PE stepSimulationSerial - Timing Report (Timestep " << timestep << ")\n"
                << "================================================================================\n"
                << "  Configuration:\n"
                << "    Step size (full):   " << fullStepSize << "\n"
                << "    Substeps:           " << substeps << "\n"
                << "    Substep size:       " << substepSize << "\n"
                << "    Bodies in storage:  " << numBodies << "\n"
                << "    Spheres:            " << numSpheres << "\n"
                << "  --------------------------------------------------------------------------------\n"
                << "  Phase                        Time [s]        Time [ms]     Share [%]\n"
                << "  --------------------------------------------------------------------------------\n"
                << std::fixed << std::setprecision(6)
                << "  Force application          " << std::setw(12) << tForce
                << "    " << std::setw(12) << std::setprecision(3) << (tForce * 1000.0)
                << "    " << std::setw(7) << std::setprecision(2) << pct(tForce) << "\n"
                << std::setprecision(6)
                << "  Simulation step (" << substeps << " sub)   " << std::setw(12) << tSim
                << "    " << std::setw(12) << std::setprecision(3) << (tSim * 1000.0)
                << "    " << std::setw(7) << std::setprecision(2) << pct(tSim) << "\n"
                << std::setprecision(6)
                << "  Sphere file output         " << std::setw(12) << tSphereOut
                << "    " << std::setw(12) << std::setprecision(3) << (tSphereOut * 1000.0)
                << "    " << std::setw(7) << std::setprecision(2) << pct(tSphereOut) << "\n"
                << std::setprecision(6)
                << "  Particle diagnostics       " << std::setw(12) << tDiag
                << "    " << std::setw(12) << std::setprecision(3) << (tDiag * 1000.0)
                << "    " << std::setw(7) << std::setprecision(2) << pct(tDiag) << "\n"
                << "  --------------------------------------------------------------------------------\n"
                << std::setprecision(6)
                << "  TOTAL                      " << std::setw(12) << tTotal
                << "    " << std::setw(12) << std::setprecision(3) << (tTotal * 1000.0)
                << "    " << std::setw(7) << std::setprecision(2) << 100.0 << "\n"
                << "================================================================================\n"
                << std::endl;
   }

   timerOverall.end();

   //==============================================================================================
   // Phase 7: Final Summary
   //==============================================================================================
   std::cout << "\n"
             << "================================================================================\n"
             << "  Simulation Complete\n"
             << "================================================================================\n"
             << "  Total timesteps:          " << config.getTimesteps() << "\n"
             << "  Total wall-clock time:    " << std::fixed << std::setprecision(3)
             << timerOverall.last() << " s\n"
             << "  Avg time per timestep:    " << std::setprecision(3)
             << (timerOverall.last() / config.getTimesteps() * 1000.0) << " ms\n"
             << "================================================================================\n"
             << std::endl;

   return 0;
}
//*************************************************************************************************
