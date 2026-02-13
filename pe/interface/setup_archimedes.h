
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/config/SimulationConfig.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <pe/core/Types.h>
#include <pe/math/Quaternion.h>

using namespace pe::povray;

// Function to load planes from file and create HalfSpace instances
void loadPlanesAndCreateHalfSpaces(const std::string &filename, std::vector<HalfSpace> &halfSpaces)
{
   std::ifstream file(filename);
   std::string line;

   if (!file.is_open())
   {
      std::cerr << "Error: Could not open file " << filename << "\n";
      return;
   }
   int counter = 0;
   while (std::getline(file, line))
   {
      std::istringstream iss(line);
      double px, py, pz, nx, ny, nz;

      // Read the plane's point and normal from the line
      if (!(iss >> px >> py >> pz >> nx >> ny >> nz))
      {
         std::cerr << "Error: Malformed line: " << line << "\n";
         continue;
      }

      // Create a Vec3 for the normal vector
      Vec3 normal(nx, ny, nz);
      if (normal[0] > 0.0)
      {
         normal = -normal;
      }
      Vec3 point(px, py, pz);

      // std::cout << "Plane " << counter << " If " << trans(-point) * normal << " < 0 then the origin is outside." << std::endl;

      bool originOutside = (trans(-point) * normal < 0.0);
      //*  - > 0: The global origin is outside the half space\n
      //*  - < 0: The global origin is inside the half space\n
      //*  - = 0: The global origin is on the surface of the half space

      // Calculate the distance from the origin using the point-normal formula
      double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      // Create the HalfSpace instance
      halfSpaces.emplace_back(normal, dO);
      counter++;
   }

   file.close();
}
//*************************************************************************************************


//*************************************************************************************************
// Function to load planes from file and create HalfSpace instances
void loadPlanesYAndCreateHalfSpaces(const std::string &filename, std::vector<HalfSpace> &halfSpaces)
{
   std::ifstream file(filename);
   std::string line;

   if (!file.is_open())
   {
      std::cerr << "Error: Could not open file " << filename << "\n";
      return;
   }
   int counter = 0;
   while (std::getline(file, line))
   {
      std::istringstream iss(line);
      double px, py, pz, nx, ny, nz;

      // Read the plane's point and normal from the line
      if (!(iss >> px >> py >> pz >> nx >> ny >> nz))
      {
         std::cerr << "Error: Malformed line: " << line << "\n";
         continue;
      }

      // Create a Vec3 for the normal vector
      Vec3 normal(nx, ny, nz);
      if (normal[1] < 0.0)
      {
         normal = -normal;
      }
      Vec3 point(px, py, pz);

      bool originOutside = (trans(-point) * normal < 0.0);
      //*  - > 0: The global origin is outside the half space\n
      //*  - < 0: The global origin is inside the half space\n
      //*  - = 0: The global origin is on the surface of the half space

      // Calculate the distance from the origin using the point-normal formula
      double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      // Create the HalfSpace instance
      halfSpaces.emplace_back(normal, dO);
      counter++;
   }

   file.close();
}
//*************************************************************************************************


//*************************************************************************************************
// Function to load planes from file and create HalfSpace instances
void makePlanesAndCreateHalfSpaces(std::vector<HalfSpace> &halfSpaces)
{

   std::vector<Vec3> planePoints;
   std::vector<Vec3> planeNormals;

   planePoints.push_back(Vec3(2.1019248962402344, -1.9163930416107178, 0.02711978182196617));
   planePoints.push_back(Vec3(-1.047705888748169, -2.5512213706970215, 0.24490927159786224));
   planePoints.push_back(Vec3(-0.035260219126939774, -2.7580153942108154, 0.1991162747144699));
   planePoints.push_back(Vec3(1.0637520551681519, -2.544778823852539, 0.14949828386306763));

   planeNormals.push_back(Vec3(-0.7013657689094543, -0.7127944231033325, -0.0031759117264300585));
   planeNormals.push_back(Vec3(-0.9239068627357483, 0.3802013397216797, 0.042931802570819855));
   planeNormals.push_back(Vec3(-0.9990096688270569, 0.012757861986756325, 0.04262349754571915));
   planeNormals.push_back(Vec3(-0.9221128225326538, -0.38450124859809875, 0.043206337839365005));
   planeNormals.push_back(Vec3(-0.7013657689094543, -0.7127944231033325, -0.0031759117264300585));

   int counter = 0;
   for (auto idx(0); idx < planePoints.size(); ++idx)
   {

      // Create a Vec3 for the normal vector
      Vec3 normal = planeNormals[idx];
      if (normal[0] > 0.0)
      {
         normal = -normal;
      }
      Vec3 point = planePoints[idx];

      bool originOutside = (trans(-point) * normal < 0.0);
      //*  - > 0: The global origin is outside the half space\n
      //*  - < 0: The global origin is inside the half space\n
      //*  - = 0: The global origin is on the surface of the half space

      // Calculate the distance from the origin using the point-normal formula
      double dO = std::abs(trans(point) * normal) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      // Create the HalfSpace instance
      halfSpaces.emplace_back(normal, dO);
      counter++;
   }
}
//*************************************************************************************************



// Radius of each sphere
real sphereRad = 0.0182;   // d_p = 364 microns
//real sphereRad = 0.01;   // Radius of each sphere
//real sphereRad = 0.015;  // Radius of each sphere

// Setup for the Archimedes case
//===================================================================================
void setupArchimedes(MPI_Comm ex0)
{
   auto& config = SimulationConfig::getInstance();
   world = theWorld();

   loadSimulationConfig("example.json");

   Vec3 userGravity = config.getGravity();

//   Vec3 myGravity(0.0, -980.665, 0.0);
//   myGravity *= 0.5;
//
//   RotationMatrix<real> rotation( Vec3( 0.0, 0.0, 1.0 ), -M_PI/real(4.) );
//   Vec3 newGravity = rotation * myGravity; 
//
//   //myGravity *= 0.25;
//   //myGravity *= 0.15;
//   world->setGravity(newGravity);
   world->setGravity(userGravity);

   // Re 1.5 configuration
   real simViscosity( config.getFluidViscosity() );
   real simRho( config.getFluidDensity() );
   real pRho( config.getParticleDensity() );
   world->setViscosity( simViscosity );
   world->setLiquidDensity( simRho );

   // Particle Bench Config
   real slipLength(0.01);
   world->setLiquidSolid(true);
   world->setDamping(1.0);

   // Lubrication switch
   bool useLubrication(false);

   // Configuration of the MPI system
   mpisystem = theMPISystem();
   mpisystem->setComm(ex0);

   const real L(45.0);
   const real LY(25.0);
   const real LZ(0.5);
   const real dx(L / config.getProcessesX());
   const real dy(LY / config.getProcessesY());
   const real dz(LZ / config.getProcessesZ());

   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;

   loadPlanesAndCreateHalfSpaces("planes_div30x.txt", halfSpaces);
   loadPlanesYAndCreateHalfSpaces("planes_div30y.txt", halfSpacesY);

   int my_rank;
   MPI_Comm_rank(ex0, &my_rank);

   // Checking the total number of MPI processes
   if (config.getProcessesX() * config.getProcessesY() * config.getProcessesZ() != mpisystem->getSize())
   {
      std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << config.getProcessesX() * config.getProcessesY() * config.getProcessesZ() << "\n\n"
                << std::endl;
      std::exit(EXIT_FAILURE);
   }

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

   // Computing the Cartesian coordinates of the neighboring processes
   int dims[] = {config.getProcessesX(), config.getProcessesZ()};
   int periods[] = {false, false};
   int reorder = false;

   int rank;      // Rank of the neighboring process
   int center[3]; // Definition of the coordinates array 'center' (the cartesian topology)

   //===================================================================================
   MPI_Comm cartcomm; // The new MPI communicator with Cartesian topology

   /*
    * Here the actual cartesian communicator is created from MPI_COMM_WORLD and the parameters
    * of the cartesian grid setup
    * \param MPI_COMM_WORLD The default communicator
    * \param ndims Number of dimensions of the cartesian grid
    * \param dims Array of size ndims, dims[i] = number of processes in dimension i
    * \param wrap_around Array of size ndims with wrap_around[i] = wrapping on/off for dimension i
    */
   MPI_Cart_create(ex0, 2, dims, periods, false, &cartcomm);
   if (cartcomm == MPI_COMM_NULL)
   {
      std::cout << "Error creating 3D communicator" << std::endl;
      MPI_Finalize();
      return;
   }

   mpisystem->setComm(cartcomm);

   // Here the cartesian coordinates of the different processes are created
   /*
    * \param comm2D The cartesian communicator created by MPI_Cart_create
    * \param my_rank The rank with regard to MPI_COMM_WORLD
    * \param ndims Dimensions of the cartesian grid
    * \param coord An array of a size equivalent to the dimension of the cartesian grid
    *  coord[0] x coord[0] would correspond to the cartesian coordinates of the first process of a 2D cartesian grid
    */
   MPI_Cart_coords(cartcomm, mpisystem->getRank(), 2, center);

   int my_cart_rank;
   MPI_Cart_rank(cartcomm, center, &my_cart_rank);

   //===================================================================================
   int px = config.getProcessesX();
   int py = config.getProcessesY();
   int pz = config.getProcessesZ();

   pe_LOG_INFO_SECTION(log) 
   {
     log << "Center: " << center[0] << " " << center[1] << "\n";
   }

   // Perform 2D domain decomposition for Archimedes simulation
   decomposeDomain2DArchimedes(center, cartcomm, halfSpaces, halfSpacesY, px, py, pz);
   
   //===================================================================================

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================

   // Setup of the VTK visualization
   if (g_vtk)
   {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
   }
   
   // Checkpointer setup
   CheckpointerID checkpointer;
   if (config.getUseCheckpointer()) {
     checkpointer = activateCheckpointer(config.getCheckpointPath(),
                                          config.getPointerspacing(),
                                          0, config.getTimesteps());
   }

   // Create a custom material for the benchmark
   MaterialID elastic = createMaterial("elastic", 1.4, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
   MaterialID particleMaterial = createMaterial( "particleMaterial", pRho, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   //========================================================================================
   // The way we atm include lubrication by increasing contact threshold
   // has problems: the particles get distributed to more domain bc the threshold AABB
   // is much larger than the particle actually is.
   // We can even run into the "registering distant domain" error when the AABB of the
   // particle is close in size to the size of a domain part!
   //=================================================================================
   theCollisionSystem()->setLubrication(useLubrication);
   theCollisionSystem()->setSlipLength(slipLength);
   theCollisionSystem()->setMinEps(0.01);
   theCollisionSystem()->setMaxIterations(200);

   //=================================================================================
   // Here is how to create some random positions on a grid up to a certain
   // volume fraction.
   //=================================================================================
   bool resume               = config.getResume();
   real epsilon              = 2e-4;
   real targetVolumeFraction = config.getVolumeFraction();
   sphereRad                 = config.getBenchRadius();

   int idx = 0;
   real h = 0.0075;
   //=================================================================================

   //=================================================================================
   //                   We have a heavy throwing ball
   //=================================================================================
   std::string fileName = std::string("archimedes.obj");
   BodyID particle;
   //=================================================================================
  if (config.getPackingMethod() != SimulationConfig::PackingMethod::None) {

     if (!resume)
     {

       std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
       std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges, sphereRad);
       for (auto spherePos: spherePositions) {
         if (world->ownsPoint(spherePos))
         {
           createSphere( idx++, spherePos, sphereRad, particleMaterial );
         }
       }
     }
     else
     {
        if (checkpointer) checkpointer->read( config.getResumeCheckpointFile() );
     }
   }

   for (int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      World::SizeType widx = static_cast<World::SizeType>(j);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if (body->getType() == sphereType)
      {
        body->setAngularVel(Vec3(0,0,0));
      }
   }

   //=================================================================================

   Vec3 archimedesPos(0.0274099, -2.56113, 0.116155);

   TriangleMeshID archimedes;
   pe_GLOBAL_SECTION
   {
      MaterialID archi = createMaterial("archimedes", 1.0, 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
      archimedes = createTriangleMesh(++idx, Vec3(0, 0, 0.0), fileName, archi, true, true, Vec3(1.0, 1.0, 1.0), false, false);
      archimedes->setPosition(archimedesPos);
      archimedes->setFixed(true);
   }

   // Synchronization of the MPI processes
   world->synchronize();

   //=================================================================================
   // Calculating the total number of particles and primitives
   unsigned long particlesTotal(0);
   unsigned long primitivesTotal(0);
   unsigned long bla = idx;

   int numBodies(0);
   int numTotal(0);
   unsigned int j(0);

   real buoyancy = 0;
   //=================================================================================
//   for (; j < theCollisionSystem()->getBodyStorage().size(); j++)
//   {
//      World::SizeType widx = static_cast<World::SizeType>(j);
//      BodyID body = world->getBody(static_cast<unsigned int>(widx));
//      if (body->getType() == sphereType)
//      {
//         SphereID s = static_body_cast<Sphere>(body);
//         Vec3 pos = body->getPosition();
//         if(pos[0] < 0.029) {
//           world->remove(body);
//           std::cout << "Removing body: " << pos << std::endl;
//         }
//      }
//   }
//   
//   // Synchronization of the MPI processes
//   world->synchronize();
   //=================================================================================

   const real   deltaT( config.getStepsize() );  // Size of a single time step
   numBodies = 0;
   numTotal  = 0;
   j = 0;
   for (; j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      World::SizeType widx = static_cast<World::SizeType>(j);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if (body->getType() == sphereType)
      {
         SphereID s = static_body_cast<Sphere>(body);
         MaterialID mat = s->getMaterial();
         real rho = Material::getDensity( mat );
         real rad = s->getRadius();
         real vol = s->getVolume();
         buoyancy = vol * (rho - Settings::liquidDensity()) * body->getInvMass();
         numBodies++;
         numTotal++;
      }
      else
      {
         numTotal++;
      }
   }

   Vec3 effGrav = buoyancy * Settings::gravity() * deltaT;
   unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
   unsigned long bodiesTotal = static_cast<unsigned long>(numTotal);
   MPI_Reduce(&bodiesUpdate, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
   MPI_Reduce(&bodiesTotal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

   real domainVol = 0.604;
   real partVol = 4. / 3. * M_PI * std::pow(sphereRad, 3);

   std::string resOut = (resume) ? "resuming " : "not resuming ";
   std::string useLub = (useLubrication) ? "enabled" : "disabled";

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "\n--" << "SIMULATION SETUP"
                << "--------------------------------------------------------------\n"
                << " Total number of MPI processes           = " << px * py * pz << "\n"
                << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
                << " Total number of particles               = " << particlesTotal << "\n"
                << " particle radius                         = " << sphereRad << "\n"
                << " particle volume                         = " << partVol << "\n"
                << " Total number of objects                 = " << primitivesTotal << "\n"
                << " Fluid Viscosity                         = " << simViscosity << "\n"
                << " Fluid Density                           = " << simRho << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
                << " EFF Gravity                             = " << effGrav << "\n"
                << " Buoyancy                                = " << buoyancy << "\n"
                << " Lubrication                             = " << useLub << "\n"
                << " Lubrication h_c (slip length)           = " << slipLength << "\n"
                << " Lubrication threshold                   = " << lubricationThreshold << "\n"
                << " Contact threshold                       = " << contactThreshold << "\n"
                << " Domain cube side length                 = " << L << "\n"
                << " Domain volume                           = " << domainVol << "\n"
                << " Resume                                  = " << resOut << "\n"
                << " Volume fraction[%]                      = " << (particlesTotal * partVol) / domainVol * 100.0 << "\n"
                << " Total objects                           = " << primitivesTotal << "\n"
                << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n"
                << std::endl;
   }

   MPI_Barrier(cartcomm);
}
