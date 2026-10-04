#ifndef _PE_SETUP_ARCHIMEDES_H_
#define _PE_SETUP_ARCHIMEDES_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/interface/setup_optional_collision_params.h>

using namespace pe::povray;

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
 * Lines that do not contain six numbers are skipped with a message.
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
         std::cerr << "Error: Malformed line in " << filename << ": " << line << "\n";
         continue;
      }

      // Orient the normal consistently along the chosen axis
      Vec3 normal(nx, ny, nz);
      if ((positive && normal[axis] < 0.0) || (!positive && normal[axis] > 0.0))
      {
         normal = -normal;
      }
      const Vec3 point(px, py, pz);

      // trans(-point) * normal:
      //  - < 0: The global origin is outside the half space
      //  - > 0: The global origin is inside the half space
      //  - = 0: The global origin is on the surface of the half space
      const bool originOutside = (trans(-point) * normal < 0.0);

      // Distance of the plane from the origin (point-normal formula), signed by the side
      // of the origin
      double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      halfSpaces.emplace_back(normal, dO);
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief PE setup for the Archimedes screw case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_archimedes_(). Spheres are seeded in rings along the
 * centerline of the screw channel; the screw itself is a fixed global triangle mesh.
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
 * Input files, read from the working directory:
 *   - example.json        configuration
 *   - planes_div30x.txt   slab planes
 *   - planes_div30y.txt   cross-channel planes
 *   - vertices.txt        centerline of the channel for seeding
 *   - archimedes.obj      screw mesh
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density and radius
 * (benchRadius_), the process layout, step size, packing method (None creates no spheres),
 * the checkpoint and resume settings, the lubrication settings, and the VTK switch and
 * spacing.
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

   const Vec3 archimedesPos(0.0274099, -2.56113, 0.116155);
   const int  archimedesId( 1000000 );   // Fixed user ID of the global screw mesh

   // TODO: origin of this value is undocumented; it is only used for the volume fraction
   //       in the setup summary.
   const real channelVolume( 0.604 );

   //===================================================================================
   // Configuration, world and fluid properties
   //===================================================================================
   auto& config = SimulationConfig::getInstance();
   world = theWorld();

   loadSimulationConfig("example.json");

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

   int myRank = 0;
   MPI_Comm_rank(ex0, &myRank);

   // Prints the message once and aborts the whole run
   const auto abortSetup = [&](const std::string& message) {
      if (myRank == 0) {
         std::cerr << "\nERROR in setupArchimedes: " << message << "\n" << std::endl;
      }
      MPI_Abort(ex0, 1);
   };

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

   // Slab planes are oriented in -x, cross-channel planes in +y
   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;
   if (!loadArchimedesHalfSpaces(planesXFile, 0, false, halfSpaces))
   {
      abortSetup("could not open the plane file " + planesXFile + ".");
   }
   if (!loadArchimedesHalfSpaces(planesYFile, 1, true, halfSpacesY))
   {
      abortSetup("could not open the plane file " + planesYFile + ".");
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
      const std::vector<Vec3> edges = readVectorsFromFile(centerlineFile);
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

   // All spheres start without spin. NOTE: this also resets the angular velocities that
   // were restored from a checkpoint on resume; kept as it is for now.
   for (unsigned int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      BodyID body = world->getBody(j);
      if (body->getType() == sphereType)
      {
        body->setAngularVel(Vec3(0,0,0));
      }
   }

   pe_GLOBAL_SECTION
   {
      MaterialID archi = createMaterial("archimedes", 1.0, 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
      TriangleMeshID archimedes = createTriangleMesh(archimedesId, Vec3(0, 0, 0.0), meshFile, archi, true, true, Vec3(1.0, 1.0, 1.0), false, false);
      archimedes->setPosition(archimedesPos);
      archimedes->setFixed(true);
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
   unsigned long spheresLocal(0);
   unsigned long bodiesLocal(0);
   for (unsigned int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      BodyID body = world->getBody(j);
      if (body->getType() == sphereType)
      {
         ++spheresLocal;
      }
      ++bodiesLocal;
   }

   unsigned long particlesTotal(0);
   unsigned long primitivesTotal(0);
   MPI_Reduce(&spheresLocal, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
   MPI_Reduce(&bodiesLocal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

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
