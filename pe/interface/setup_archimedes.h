//=================================================================================================
/*!
 *  \file pe/interface/setup_archimedes.h
 *  \brief PE setup for the Archimedes screw case (parallel PE mode)
 *
 *  Copyright (C) 2024 Raphael Münster
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

#ifndef _PE_INTERFACE_SETUP_ARCHIMEDES_H_
#define _PE_INTERFACE_SETUP_ARCHIMEDES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/interface/setup_optional_collision_params.h>
#include <pe/util/Deprecation.h>


using namespace pe::povray;




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Loads dividing planes from a text file and converts them to half spaces.
 *
 * \param filename The plane file; one plane per line as "px py pz nx ny nz" (a point on the
 *                 plane and its normal).
 * \param axis The coordinate axis (0 = x, 1 = y) used to orient the normals consistently.
 * \param positive If true the normals are flipped to point in the +axis direction, otherwise
 *                 in the -axis direction.
 * \param halfSpaces The half spaces are appended to this list in file order.
 * \return false if the file could not be opened, true otherwise.
 *
 * Lines that do not contain six numbers are skipped with a warning.
 */
inline bool loadArchimedesHalfSpaces(const std::string &filename, int axis, bool positive,
                                     std::vector<HalfSpace> &halfSpaces)
{
   std::ifstream file(filename);
   if (!file.is_open())
   {
      return false;
   }

   std::string line;
   while (std::getline(file, line))
   {
      std::istringstream iss(line);
      double px, py, pz, nx, ny, nz;

      // Read the plane's point and normal from the line
      if (!(iss >> px >> py >> pz >> nx >> ny >> nz))
      {
         std::cerr << "Warning: skipping malformed line in " << filename << ": " << line << "\n";
         continue;
      }

      // Orient the normal consistently along the chosen axis
      Vec3 normal(nx, ny, nz);
      if ((positive && normal[axis] < 0.0) || (!positive && normal[axis] > 0.0))
      {
         normal = -normal;
      }
      const Vec3 point(px, py, pz);

      // Signed distance of the plane from the origin along the (oriented) unit normal
      halfSpaces.emplace_back(normal, trans(point) * normal / normal.length());
   }

   return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief PE setup for the Archimedes screw case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_archimedes_(). The domain is the helical channel of the
 * screw: its boundary is the fixed global triangle mesh "archimedes.obj", whose interior is the
 * simulated part. Spheres are seeded in rings along the centerline of the channel.
 *
 * Decomposition: the domain is not a box. It is cut into slabs by the planes of
 * "planes_div30x.txt", following the channel in x-direction. The supported and standard
 * layout is a decomposition in x only:
 *
 *    processesX_ = number of slabs,  processesY_ = 1,  processesZ_ = 1
 *
 * processesZ_ = 2 additionally splits every slab once across the channel, using the planes
 * of "planes_div30y.txt" (note: this second direction is driven by processesZ_, not by
 * processesY_). Both plane files need at least processesX_ planes, in slab order.
 *
 * Input files; a missing file aborts the run before anything is created. The configuration
 * file is named by SimulationConfig::getConfigFile() (--pe-config, PE_CONFIG_FILE, or the
 * deprecated default example.json); all other files are read from the working directory:
 *   - planes_div30x.txt   slab planes
 *   - planes_div30y.txt   cross-channel planes
 *   - vertices.txt        centerline of the channel for seeding (only if spheres are seeded)
 *   - archimedes.obj      channel boundary mesh
 *
 * Read from the configuration file: gravity, fluid density and viscosity, particle density and radius
 * (benchRadius_), the process layout, step size, packing method (None creates no spheres),
 * the checkpoint and resume settings, the lubrication settings, the VTK switch and spacing,
 * the position of the channel mesh (domainBoundary_.position), resetAngularVelocityOnResume_
 * and deprecationsAsErrors_.
 *
 * The channel volume in the setup summary (and with it the volume fraction) is the volume of
 * archimedes.obj, so the mesh has to be closed.
 *
 * Deprecated (see doc/technical-notes/deprecations.md):
 *   - config.defaultFileName: the configuration file is the default example.json.
 *   - archimedes.position: without domainBoundary_.position the built-in position of the
 *     channel mesh is used.
 *   - archimedes.resetAngularVelocityOnResume: on resume, the angular velocities of all
 *     spheres are reset to zero unless resetAngularVelocityOnResume_ is false.
 *
 * Sphere user IDs are numbered per process and are therefore not unique across processes.
 * Use the system ID (getSystemID()) wherever a globally unique ID is needed.
 *
 * Every error path aborts the run with a message.
 */
void setupArchimedes(MPI_Comm ex0)
{
   //===================================================================================
   // Case constants
   //===================================================================================
   const std::string planesXFile( "planes_div30x.txt" );
   const std::string planesYFile( "planes_div30y.txt" );
   const std::string centerlineFile( "vertices.txt" );
   const std::string meshFile( "archimedes.obj" );

   // Deprecated: used only if domainBoundary_.position is not set (archimedes.position)
   const Vec3 legacyMeshPos(0.0274099, -2.56113, 0.116155);
   const int  archimedesId( 1000000 );   // Fixed user ID of the global channel mesh

   //===================================================================================
   // Configuration, world and fluid properties
   //===================================================================================
   auto& config = SimulationConfig::getInstance();
   world = theWorld();

   int myRank = 0;
   MPI_Comm_rank(ex0, &myRank);

   // Prints the message once and aborts the whole run
   const auto abortSetup = [&](const std::string& message) {
      if (myRank == 0) {
         std::cerr << "\nERROR in setupArchimedes: " << message << "\n" << std::endl;
      }
      MPI_Abort(ex0, 1);
   };

   try {
      SimulationConfig::loadConfigFile(myRank == 0);
   }
   catch (const std::exception& e) {
      abortSetup(e.what());
   }

   const real simViscosity( config.getFluidViscosity() );
   const real simRho( config.getFluidDensity() );
   const real pRho( config.getParticleDensity() );
   const real sphereRad( config.getBenchRadius() );

   world->setGravity( config.getGravity() );
   world->setViscosity( simViscosity );
   world->setLiquidDensity( simRho );
   world->setLiquidSolid(true);
   world->setDamping(1.0);

   TimeStep::stepsize( config.getStepsize() );

   //===================================================================================
   // MPI system and validation (nothing is created before all checks have passed)
   //===================================================================================
   mpisystem = theMPISystem();
   mpisystem->setComm(ex0);

   const int px = config.getProcessesX();
   const int py = config.getProcessesY();
   const int pz = config.getProcessesZ();

   if (py != 1)
   {
      abortSetup("processesY_ must be 1; the Archimedes decomposition runs in x-direction "
                 "(processesX_), with an optional cross-channel split through processesZ_.");
   }
   if (pz != 1 && pz != 2)
   {
      abortSetup("processesZ_ must be 1 (standard, x-only decomposition) or 2 (one "
                 "cross-channel split per slab), got " + std::to_string(pz) + ".");
   }
   if (px * pz != mpisystem->getSize())
   {
      abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
                 " != " + std::to_string(px * pz) + " (processesX_*processesZ_).");
   }
   if (sphereRad <= real(0))
   {
      abortSetup("benchRadius_ must be positive.");
   }

   const bool resume      = config.getResume();
   const bool seedSpheres = !resume &&
                            (config.getPackingMethod() != SimulationConfig::PackingMethod::None);
   if (resume && !config.getUseCheckpointer())
   {
      abortSetup("resume_ is set but the checkpointer is not enabled (useCheckpointer_).");
   }

   // All input files are expected in the working directory
   const std::string cwd = boost::filesystem::current_path().string();
   const auto requireFile = [&](const std::string& file, const std::string& purpose) {
      if (!boost::filesystem::is_regular_file(file))
      {
         abortSetup("the " + purpose + " file " + file + " was not found in the working "
                    "directory " + cwd + ".");
      }
   };
   requireFile(planesXFile, "slab plane");
   requireFile(planesYFile, "cross-channel plane");
   requireFile(meshFile, "channel mesh");
   if (seedSpheres)
   {
      requireFile(centerlineFile, "centerline");
   }

   // Deprecated input and behaviour (nothing is created yet, so stopping here is clean)
   const auto deprecated = [&](const std::string& id, const std::string& message) {
      if (!reportDeprecated(id, message, myRank == 0))
      {
         abortSetup("deprecated input or behaviour (deprecationsAsErrors_ is set), see the "
                    "PE DEPRECATED message.");
      }
   };

   Vec3 meshPos(legacyMeshPos);
   if (config.hasDomainBoundaryPosition())
   {
      meshPos = config.getDomainBoundaryPosition();
   }
   else
   {
      deprecated("archimedes.position",
                 "the position of " + meshFile + " is built in; set it in the configuration file with "
                 "\"domainBoundary_\": { \"position\": [x, y, z] }. The built-in position is "
                 "[0.0274099, -2.56113, 0.116155].");
   }

   const bool resetAngularVelocity = resume && config.getResetAngularVelocityOnResume();
   if (resetAngularVelocity)
   {
      deprecated("archimedes.resetAngularVelocityOnResume",
                 "the angular velocities of all spheres are reset to zero after the resume, so "
                 "the run does not continue the saved state. Set "
                 "\"resetAngularVelocityOnResume_\": false in the configuration file to keep them.");
   }

   // Slab planes are oriented in -x, cross-channel planes in +y
   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;
   if (!loadArchimedesHalfSpaces(planesXFile, 0, false, halfSpaces))
   {
      abortSetup("could not read the plane file " + planesXFile + ".");
   }
   if (!loadArchimedesHalfSpaces(planesYFile, 1, true, halfSpacesY))
   {
      abortSetup("could not read the plane file " + planesYFile + ".");
   }
   if (halfSpaces.size() < static_cast<std::size_t>(px) ||
       halfSpacesY.size() < static_cast<std::size_t>(px))
   {
      abortSetup("the plane files need at least processesX_ = " + std::to_string(px) +
                 " planes each, found " + std::to_string(halfSpaces.size()) + " in " +
                 planesXFile + " and " + std::to_string(halfSpacesY.size()) + " in " +
                 planesYFile + ".");
   }

   //===================================================================================
   // Plane-based 2D domain decomposition (slabs in x, optional cross-channel split)
   //===================================================================================
   int dims[] = {px, pz};
   int periods[] = {false, false};
   int reorder = false;
   MPI_Comm cartcomm;

   MPI_Cart_create(ex0, 2, dims, periods, reorder, &cartcomm);
   if (cartcomm == MPI_COMM_NULL)
   {
      abortSetup("failed to create the cartesian communicator.");
   }
   mpisystem->setComm(cartcomm);

   // Cartesian coordinates of this process within the 2D process grid
   int center[3] = {0, 0, 0};
   MPI_Cart_coords(cartcomm, mpisystem->getRank(), 2, center);

   pe_LOG_INFO_SECTION(log)
   {
     log << "Center: " << center[0] << " " << center[1] << "\n";
   }

   // TODO: Open point. decomposeDomain2DArchimedes() always intersects the slab with the
   //       cross-channel half space of planes_div30y.txt, also for processesZ_ = 1. Each
   //       process then owns only one side of that plane, and the other side of the channel
   //       has no owning process. To decide: apply the y-plane only for processesZ_ = 2 (and
   //       require planes_div30y.txt only then), or make processesZ_ = 2 mandatory.
   decomposeDomain2DArchimedes(center, cartcomm, halfSpaces, halfSpacesY, px, py, pz);

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================
   // Materials, checkpointer and solver parameters
   //===================================================================================
   // TODO: "elastic" is not assigned to any body. It is kept on purpose: this case uses
   //       checkpoints, and removing it shifts the material index of "particleMaterial".
   //       Remove it once index-based use is ruled out.
   MaterialID elastic = createMaterial("elastic", 1.4, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
   MaterialID particleMaterial = createMaterial( "particleMaterial", pRho, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   (void)elastic;

   CheckpointerID checkpointer;
   if (config.getUseCheckpointer()) {
     checkpointer = activateCheckpointer(config.getCheckpointPath(),
                                          config.getPointerspacing(),
                                          0, config.getTimesteps());
   }

   theCollisionSystem()->setMinEps(0.01);
   theCollisionSystem()->setMaxIterations(200);

   //===================================================================================
   // Bodies: spheres along the channel centerline (or from the checkpoint) and the screw
   //===================================================================================
   if (resume)
   {
      checkpointer->read( config.getResumeCheckpointFile() );
   }
   else if (seedSpheres)
   {
      std::vector<Vec3> edges = readVectorsFromFile(centerlineFile);  // non-const: the generator takes a mutable reference
      if (edges.empty())
      {
         abortSetup("read no centerline vertices from " + centerlineFile + ".");
      }

      // User IDs are numbered per process (see the function documentation)
      int idx = 0;
      const std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges, sphereRad);
      for (const Vec3& spherePos : spherePositions) {
         if (world->ownsPoint(spherePos))
         {
            createSphere( idx++, spherePos, sphereRad, particleMaterial );
         }
      }
   }

   // Deprecated (archimedes.resetAngularVelocityOnResume): new spheres start without spin
   // anyway, so this only changes the angular velocities restored from the checkpoint.
   if (resetAngularVelocity)
   {
      for (unsigned int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++)
      {
         BodyID body = world->getBody(j);
         if (body->getType() == sphereType)
         {
           body->setAngularVel(Vec3(0,0,0));
         }
      }
   }

   real channelVolume(0);
   pe_GLOBAL_SECTION
   {
      MaterialID archi = createMaterial("archimedes", 1.0, 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
      // TODO: Open point. The mesh is created with convex = true (5th argument), but the
      //       helical channel is not convex. To check: do sphere-channel collisions go through
      //       this mesh (then convex = true is wrong), or are they handled elsewhere?
      TriangleMeshID archimedes = createTriangleMesh(archimedesId, Vec3(0, 0, 0.0), meshFile, archi, true, true, Vec3(1.0, 1.0, 1.0), false, false);
      archimedes->setPosition(meshPos);
      archimedes->setFixed(true);

      // The sign of the mesh volume depends on the orientation of its faces
      channelVolume = std::abs(archimedes->getVolume());
   }

   // Synchronization of the MPI processes
   world->synchronize();

   // Setup of the VTK visualization
   if (config.getVtk())
   {
     vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
   }

   //===================================================================================
   // Setup summary
   //===================================================================================
   // Every body counts once: the bodies a process owns (not its shadow copies of neighbouring
   // bodies), plus the global bodies, which every process holds
   unsigned long spheresLocal(0);
   unsigned long bodiesLocal(0);
   unsigned long globalBodies(0);
   for (World::Iterator body = world->begin(); body != world->end(); ++body)
   {
      if (body->isGlobal())
      {
         ++globalBodies;
      }
      else if (!body->isRemote())
      {
         ++bodiesLocal;
         if (body->getType() == sphereType)
         {
            ++spheresLocal;
         }
      }
   }

   unsigned long particlesTotal(0);
   unsigned long primitivesTotal(0);
   MPI_Reduce(&spheresLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
   MPI_Reduce(&bodiesLocal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
   primitivesTotal += globalBodies;

   const real partVol = 4. / 3. * M_PI * std::pow(sphereRad, 3);
   const std::string packing = resume ? "checkpoint" : (seedSpheres ? "centerline rings" : "none");

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "\n--" << "ARCHIMEDES SETUP"
                << "--------------------------------------------------------------\n"
                << " Total number of MPI processes           = " << px * pz << "\n"
                << " Slabs in x / cross-channel splits       = " << px << " / " << pz << "\n"
                << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
                << " Total number of particles               = " << particlesTotal << "\n"
                << " Particle radius                         = " << sphereRad << "\n"
                << " Particle volume                         = " << partVol << "\n"
                << " Total number of objects                 = " << primitivesTotal << "\n"
                << " Fluid Viscosity                         = " << simViscosity << "\n"
                << " Fluid Density                           = " << simRho << "\n"
                << " Particle Density                        = " << pRho << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
                << " Lubrication (json)                      = " << (config.getLubricationEnabled() ? "enabled" : "disabled") << "\n"
                << " Lubrication threshold                   = " << lubricationThreshold << "\n"
                << " Contact threshold                       = " << contactThreshold << "\n"
                << " Channel volume                          = " << channelVolume << "\n"
                << " Resume                                  = " << (resume ? "resuming" : "not resuming") << "\n"
                << " Packing                                 = " << packing << "\n"
                << " Volume fraction[%]                      = " << (particlesTotal * partVol) / channelVolume * 100.0 << "\n"
                << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n"
                << std::endl;
   }

   MPI_Barrier(cartcomm);
}
//*************************************************************************************************

#endif
