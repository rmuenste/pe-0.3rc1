
#include <pe/interface/decompose.h>
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


//*************************************************************************************************
std::vector<Vec3> readVectorsFromFile(const std::string& fileName) {
    std::vector<Vec3> vectors;
    std::ifstream file(fileName);
    
    // Check if the file was successfully opened
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << fileName << std::endl;
        return vectors; // Return an empty vector in case of error
    }
    
    std::string line;
    
    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);  // Create a string stream from the line
        float x, y, z;
        
        // Parse the line for three float values
        if (ss >> x >> y >> z) {
            // Create a Vec3 object and add it to the vector
            vectors.emplace_back(x, y, z);
        }
    }
    
    file.close();  // Close the file
    return vectors;
}
//*************************************************************************************************


//real sphereRad = 0.0182;  d_p = 364 microns // Radius of each sphere
//real sphereRad = 0.0182;  d_p = 364 microns // Radius of each sphere
real sphereRad = 0.01;  // Radius of each sphere
//*************************************************************************************************
std::vector<Vec3> generatePointsAlongCenterline(std::vector<Vec3> &vecOfEdges) {

    // Step 1: Measure the total length of the curve
    double curve_length = 0.0;
    std::vector<double> edge_lengths;
    std::vector<Vec3> wayPoints;
    int num_rings = 4;

    // User-defined parameters
    real sphereRadius = sphereRad;  // Radius of each sphere
    real dt = 1. * sphereRad;           // Distance from the sphere surface to the circle center
    int num_steps = 3;      // Number of divisions along the curve
    std::vector<Vec3> sphere_positions;

    size_t num_edges = vecOfEdges.size() - 1;

    for (size_t i = 0; i < num_edges; ++i) {
        Vec3 v1 = vecOfEdges[i];
        Vec3 v2 = vecOfEdges[i + 1];
        double edge_length = (v2 - v1).length();
        edge_lengths.push_back(edge_length);
        curve_length += edge_length;
    }

    //std::cout << "Curve length: " << curve_length << std::endl;

    // Step 2: Set ds (step size)
    double ds = curve_length / real(num_steps);

    // Step 3: Compute cumulative lengths to help find the edge containing each step
    std::vector<double> cumulative_lengths;
    cumulative_lengths.push_back(0.0);  // Starting point

    for (size_t i = 0; i < edge_lengths.size(); ++i) {
        cumulative_lengths.push_back(cumulative_lengths.back() + edge_lengths[i]);
    }

    // Step 4: Traverse the curve in increments of ds
    for (double s = ds + 0.2 * ds; s <= curve_length-ds; s += ds) {
        // Find the edge that contains the current distance s
        size_t edge_index = 0;
        while (edge_index < num_edges && s > cumulative_lengths[edge_index + 1]) {
            ++edge_index;
        }

        if (edge_index >= num_edges) {
            break;  // Reached the end of the curve
        }

        // Compute the parameter t along the current edge
        double edge_start = cumulative_lengths[edge_index];
        double edge_end = cumulative_lengths[edge_index + 1];
        double t = (s - edge_start) / (edge_end - edge_start);

        // Calculate the point along the edge at distance s
        Vec3 v1 = vecOfEdges[edge_index];
        Vec3 v2 = vecOfEdges[edge_index + 1];
        Vec3 point_on_edge = v1 + (v2 - v1) * t;

        // Step 5: Take a user-defined action at the point
        wayPoints.push_back(point_on_edge);

        //-------------------- User Action Start --------------------
        // Generate a ring of spheres around the current edge at point_on_edge
        Vec3 edge_direction = v2 - v1;
        Vec3 someVector(1.0, 0., 0.);
        if ( std::abs( trans(edge_direction) * someVector ) > 0.999) 
          someVector = Vec3(0.0, 1.0, 0.0);

        // Compute orthogonal vectors u and v in the plane perpendicular to edge_direction
        Vec3 u = (edge_direction % someVector).getNormalized();
        Vec3 v = (edge_direction % u).getNormalized();

        for (int j(0); j < num_rings; ++j) {

          // Compute circle radius
          real circle_radius = sphereRadius + dt + j * (2. * sphereRadius + dt);

          real circumference = 2. * M_PI * circle_radius;
          
          //std::cout << "circumference = " << circumference << std::endl;
          // Compute maximum number of spheres without overlap
          int max_spheres = int(circumference / (2. * sphereRadius)) - 1;

          if (max_spheres < 1)
             max_spheres = 1;

          //std::cout << "max_spheres = " << max_spheres << std::endl;

          // Compute exact angle step
          real theta_step = 2. * M_PI / max_spheres;

          // Place spheres around the circle
          for(int i(0); i < max_spheres; ++i) {
             real theta = i * theta_step;
             Vec3 sphere_offset = (std::cos(theta) * u + std::sin(theta) * v) * circle_radius;
             sphere_positions.push_back(Vec3(point_on_edge + sphere_offset));
          }

        }


    }

    real minDist = std::numeric_limits<real>::max();
    for (size_t i = 1; i < wayPoints.size(); ++i) {
        real dist = (wayPoints[i-1] - wayPoints[i]).length();
        if (minDist > dist) minDist = dist;
        //std::cout << "Distance between [" << i-1 << ", " << i << "] = " << (wayPoints[i-1] - wayPoints[i]).length() << std::endl;
    }
    //std::cout << "Minimal distance: " << minDist  << " => minRadius = " << minDist * 0.5 << std::endl;

    return sphere_positions;

}


//===================================================================================
// Setup for the Archimedes case
//===================================================================================
void setupArchimedesZ(MPI_Comm ex0)
{

   world = theWorld();
   world->setGravity(0.0, 0.0, 0.0);

   // Re 1.5 configuration
   real simViscosity(0.01e0);
   real simRho(1.0);
   world->setViscosity(simViscosity);
   world->setLiquidDensity(simRho);

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
   const real dx(L / processesX);
   const real dy(LY / processesY);
   const real dz(LZ / processesZ);
   std::vector<HalfSpace> halfSpaces;

   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);

   int my_rank;
   MPI_Comm_rank(ex0, &my_rank);

   // Checking the total number of MPI processes
   if (processesX * processesY * processesZ != mpisystem->getSize())
   {
      std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX * processesY * processesZ << "\n\n"
                << std::endl;
      std::exit(EXIT_FAILURE);
   }

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

   // Computing the Cartesian coordinates of the neighboring processes
   int dims[] = {processesX, processesZ};
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

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "3D coordinates were created" << std::endl;
      std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
   }

   //===================================================================================

   int west[] = {center[0] - 1, center[1], center[2]};
   int east[] = {center[0] + 1, center[1], center[2]};
   int south[] = {center[0], center[1] - 1, center[2]};
   int north[] = {center[0], center[1] + 1, center[2]};
   int southwest[] = {center[0] - 1, center[1] - 1, center[2]};
   int southeast[] = {center[0] + 1, center[1] - 1, center[2]};
   int northwest[] = {center[0] - 1, center[1] + 1, center[2]};
   int northeast[] = {center[0] + 1, center[1] + 1, center[2]};

   if (west[0] < 0)
   {
      defineLocalDomain(
          halfSpaces[center[0]]);
   }
   else
   {
      // -x of halfSpaces[mpisystem->getRank()] and +x of halfSpaces[mpisystem->getRank()-1]
      HalfSpace hs = halfSpaces[center[0] - 1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
      defineLocalDomain(intersect(
          halfSpaces[center[0]],
          hs_flip));
   }

   //===================================================================================
   // Connecting the west neighbor
   if (west[0] > 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs = halfSpaces[west[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect(rank,
              intersect(
                  hs1,
                  hs_flip));
   }

   if (west[0] == 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];

      connect(rank,
              hs1);
   }

   // Connecting the east neighbor
   if (east[0] < processesX)
   {
      MPI_Cart_rank(cartcomm, east, &rank);

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect(rank,
              intersect(
                  hs1,
                  hs_flip));
   }

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================

   // Setup of the VTK visualization
   if (g_vtk)
   {
      vtk::WriterID vtk = vtk::activateWriter("./paraview", visspacing, 0, timesteps, false);
   }

   // Create a custom material for the benchmark
   MaterialID elastic = createMaterial("elastic", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
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
   bool resume = false;
   real epsilon = 2e-4;
   real targetVolumeFraction = 0.10;
   real radius2 = 0.05;

   int idx = 0;
   real h = 0.0075;
   // GetNonNewtViscosity
   // fbm force
   // fbm update
   //=================================================================================

   //=================================================================================
   //                   We have a heavy throwing ball
   //=================================================================================
   std::string fileName = std::string("archimedes.obj");

   BodyID particle;
   //=================================================================================
   if (!resume)
   {

     std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
     std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges);
     for (auto spherePos: spherePositions) {
       if (world->ownsPoint(spherePos))
       {
         createSphere( idx++, spherePos, sphereRad, elastic );
       }
     }

//      if (world->ownsPoint(pos1))
//      {
//         particle = createSphere(idx++, pos1, radius2, elastic);
//      }
//      if (world->ownsPoint(pos2))
//      {
//         particle = createSphere(idx++, pos2, radius2, elastic);
//      }
//      if (world->ownsPoint(pos3))
//      {
//         particle = createSphere(idx++, pos3, radius2, elastic);
//      }
//      if (world->ownsPoint(pos4))
//      {
//         particle = createSphere(idx++, pos4, radius2, elastic);
//      }
   }
   else
   {
      checkpointer.read( "../start.5" );
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
   for (; j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      World::SizeType widx = static_cast<World::SizeType>(j);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if (body->getType() == sphereType)
      {
         numBodies++;
         numTotal++;
      }
      else
      {
         numTotal++;
      }
   }

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
                << " particle volume                         = " << partVol << "\n"
                << " Total number of objects                 = " << primitivesTotal << "\n"
                << " Fluid Viscosity                         = " << simViscosity << "\n"
                << " Fluid Density                           = " << simRho << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
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


//===================================================================================
// Setup for the Archimedes case
//===================================================================================
void setupArchimedesXY(MPI_Comm ex0)
{

   world = theWorld();
   world->setGravity(0.0, 0.0, 0.0);

   // Re 1.5 configuration
   real simViscosity(0.01e0);
   real simRho(1.0);
   world->setViscosity(simViscosity);
   world->setLiquidDensity(simRho);

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
   const real dx(L / processesX);
   const real dy(LY / processesY);
   const real dz(LZ / processesZ);
   std::vector<HalfSpace> halfSpaces;

   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);
   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);

   int my_rank;
   MPI_Comm_rank(ex0, &my_rank);

   // Checking the total number of MPI processes
   if (processesX * processesY * processesZ != mpisystem->getSize())
   {
      std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX * processesY * processesZ << "\n\n"
                << std::endl;
      std::exit(EXIT_FAILURE);
   }

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

   // Computing the Cartesian coordinates of the neighboring processes
   int dims[] = {processesX, processesZ};
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

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "3D coordinates were created" << std::endl;
      std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
   }

   //===================================================================================

   int west[] = {center[0] - 1, center[1], center[2]};
   int east[] = {center[0] + 1, center[1], center[2]};
   int south[] = {center[0], center[1] - 1, center[2]};
   int north[] = {center[0], center[1] + 1, center[2]};
   int southwest[] = {center[0] - 1, center[1] - 1, center[2]};
   int southeast[] = {center[0] + 1, center[1] - 1, center[2]};
   int northwest[] = {center[0] - 1, center[1] + 1, center[2]};
   int northeast[] = {center[0] + 1, center[1] + 1, center[2]};

   if (west[0] < 0)
   {
      defineLocalDomain(
               halfSpaces[center[0]]
               );
   }
   else
   {
      // -x of halfSpaces[mpisystem->getRank()] and +x of halfSpaces[mpisystem->getRank()-1]
      HalfSpace hs = halfSpaces[center[0] - 1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
      defineLocalDomain(intersect(
          halfSpaces[center[0]],
          hs_flip));
   }

   //===================================================================================
   // Connecting the west neighbor
   if (west[0] > 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs = halfSpaces[west[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect(rank,
              intersect(
              hs1,
              hs_flip));
   }

   if (west[0] == 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];

      connect(rank,
              hs1);
   }

   // Connecting the east neighbor
   if (east[0] < processesX)
   {
      MPI_Cart_rank(cartcomm, east, &rank);

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect(rank,
              intersect(
              hs1,
              hs_flip));
   }

//   // Connecting the south neighbor
//   if( south[1] >= 0 ) {
//      MPI_Cart_rank( cartcomm, south, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }
//
//   // Connecting the south-west neighbor
//   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
//      MPI_Cart_rank( cartcomm, southwest, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }
// 
//   // Connecting the south-east neighbor
//   if( southeast[0] < px && southeast[1] >= 0 ) {
//      MPI_Cart_rank( cartcomm, southeast, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }
// 
//   // Connecting the north neighbor
//   if( north[1] < py ) {
//      MPI_Cart_rank( cartcomm, north, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }
//
//   // Connecting the north-west neighbor
//   if( northwest[0] >= 0 && northwest[1] < py ) {
//      MPI_Cart_rank( cartcomm, northwest, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }
// 
//   // Connecting the north-east neighbor
//   if( northeast[0] < px && northeast[1] < py ) {
//      MPI_Cart_rank( cartcomm, northeast, &rank );
//      connect( rank, intersect(
//         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
//   }

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================

   // Setup of the VTK visualization
   if (g_vtk)
   {
      vtk::WriterID vtk = vtk::activateWriter("./paraview", visspacing, 0, timesteps, false);
   }

   // Create a custom material for the benchmark
   MaterialID elastic = createMaterial("elastic", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
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
   bool resume = true;
   real epsilon = 2e-4;
   real targetVolumeFraction = 0.10;
   real radius2 = 0.05;

   int idx = 0;
   real h = 0.0075;
   // GetNonNewtViscosity
   // fbm force
   // fbm update
   //=================================================================================

   //=================================================================================
   //                   We have a heavy throwing ball
   //=================================================================================
   std::string fileName = std::string("archimedes.obj");

   BodyID particle;
   //=================================================================================
   if (!resume)
   {

     std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
     std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges);
     for (auto spherePos: spherePositions) {
       if (world->ownsPoint(spherePos))
       {
         createSphere( idx++, spherePos, sphereRad, elastic );
       }
     }

//      if (world->ownsPoint(pos1))
//      {
//         particle = createSphere(idx++, pos1, radius2, elastic);
//      }
//      if (world->ownsPoint(pos2))
//      {
//         particle = createSphere(idx++, pos2, radius2, elastic);
//      }
//      if (world->ownsPoint(pos3))
//      {
//         particle = createSphere(idx++, pos3, radius2, elastic);
//      }
//      if (world->ownsPoint(pos4))
//      {
//         particle = createSphere(idx++, pos4, radius2, elastic);
//      }
   }
   else
   {
      checkpointer.read( "../start.4" );
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
   for (; j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      World::SizeType widx = static_cast<World::SizeType>(j);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if (body->getType() == sphereType)
      {
         numBodies++;
         numTotal++;
      }
      else
      {
         numTotal++;
      }
   }

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
                << " particle volume                         = " << partVol << "\n"
                << " Total number of objects                 = " << primitivesTotal << "\n"
                << " Fluid Viscosity                         = " << simViscosity << "\n"
                << " Fluid Density                           = " << simRho << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
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



//=================================================================================================
// Setup for the Empty case
//=================================================================================================
void setupArchimedesEMPTY(MPI_Comm ex0) {
//void setupEmpty(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Re 1.5 configuration
  real simViscosity( 1.0e0 );
  real simRho( 1.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );

  // Particle Bench Config 
  real slipLength( 0.01 );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication switch
  bool useLubrication(false);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real L( 45.0 );
  const real LY( 25.0 );
  const real LZ( 0.5 );
  const real dx( L/processesX );
  const real dy( LY/processesY );
  const real dz( LZ/processesZ );

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( processesX*processesY*processesZ != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX*processesY*processesZ << "\n\n" << std::endl;
     std::exit(EXIT_FAILURE);
  }

  /////////////////////////////////////////////////////
  // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

  // Computing the Cartesian coordinates of the neighboring processes
  int dims   [] = { processesX, processesY, processesZ };
  //int periods[] = { true, true, false };
  int periods[] = { false, false, false };
  int reorder   = false;

  int rank;           // Rank of the neighboring process
  int center[3];      // Definition of the coordinates array 'center' (the cartesian topology)
  MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

  /*
   * Here the actual cartesian communicator is created from MPI_COMM_WORLD and the parameters
   * of the cartesian grid setup
   * \param MPI_COMM_WORLD The default communicator
   * \param ndims Number of dimensions of the cartesian grid
   * \param dims Array of size ndims, dims[i] = number of processes in dimension i 
   * \param wrap_around Array of size ndims with wrap_around[i] = wrapping on/off for dimension i 
   */
  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if( cartcomm == MPI_COMM_NULL ) {
     std::cout << "Error creating 3D communicator" << std::endl;
     MPI_Finalize();
     return;
  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
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
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  int my_cart_rank;
  MPI_Cart_rank(cartcomm, center, &my_cart_rank);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "3D coordinates were created" << std::endl;
    std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
  }

//===========================================================================================================

//  real bx = 0.0;
//  real by = 0.0;
//  real bz = 0.0;

  real bx = -10.0;
  real by = -5.0;
  real bz =  0.0;
//  const real L( 2.0 );
//  const real LY( 1.0 );
//  const real LZ( 0.05 );

  // Size of the domain
  const real lx( L );
  const real ly( LY);
  const real lz( LZ);

  decomposeDomain(center, bx, by, bz, 
                   dx, dy, dz, 
                   px, py, pz);

//===========================================================================================================


//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
  }


  // Create a custom material for the benchmark
  MaterialID myMaterial = createMaterial("Bench", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID elastic = createMaterial( "elastic", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  //======================================================================================== 
  // The way we atm include lubrication by increasing contact threshold
  // has problems: the particles get distributed to more domain bc the threshold AABB
  // is much larger than the particle actually is.
  // We can even run into the "registering distant domain" error when the AABB of the 
  // particle is close in size to the size of a domain part!
  //======================================================================================== 
  theCollisionSystem()->setLubrication(useLubrication);
  theCollisionSystem()->setSlipLength(slipLength);
  theCollisionSystem()->setMinEps(0.01);
  theCollisionSystem()->setMaxIterations(200);

  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  bool resume               = false;
  real epsilon              = 2e-4;
  real targetVolumeFraction = 0.35;
  real radius2              = 0.005 - epsilon;

  int idx = 0;
  real h = 0.0075;
  
  //=========================================================================================
  BodyID particle;
  Vec3 gpos(0.05 , 0.05, 0.1 - radius2 - epsilon);
  Vec3 gpos2(0.05 , 0.05, gpos[2]);

  //=========================================================================================  
  //                   We have a heavy throwing ball
  //=========================================================================================  
  MaterialID heavy = createMaterial( "heavy", 10.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  //=========================================================================================  

  // Synchronization of the MPI processes
  world->synchronize();

  //=========================================================================================  
// Calculating the total number of particles and primitives
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  unsigned long bla = idx;

  int numBodies (0);
  int numTotal (0);
  unsigned int j(0);
  for (; j < theCollisionSystem()->getBodyStorage().size(); j++) {
    World::SizeType widx = static_cast<World::SizeType>(j);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      numBodies++;
      numTotal++;
    } else {
      numTotal++;
    }
  }

  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  unsigned long bodiesTotal = static_cast<unsigned long>(numTotal);
  MPI_Reduce( &bodiesUpdate, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesTotal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  real domainVol = L * L * L;
  real partVol = 4./3. * M_PI * std::pow(radius2, 3);

  std::string resOut = (resume) ? "resuming " : "not resuming ";
  std::string useLub = (useLubrication) ? "enabled" : "disabled";

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n" 
      << " Total number of particles               = " << particlesTotal << "\n"
      << " particle volume                         = " << partVol << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n" 
      << " Lubrication                             = " << useLub << "\n"
      << " Lubrication h_c (slip length)           = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain cube side length                 = " << L << "\n"
      << " Domain volume                           = " << L * L * L << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Volume fraction[%]                      = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   MPI_Barrier(cartcomm);
}



// Setup for the Archimedes case
//===================================================================================
//void setupNormal(MPI_Comm ex0)
void setupArchimedes(MPI_Comm ex0)
{

   world = theWorld();
   world->setGravity(0.0, -980.665, 0.0);

   // Re 1.5 configuration
   real simViscosity(0.01e0);
   real simRho(1.0);
   world->setViscosity(simViscosity);
   world->setLiquidDensity(simRho);

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
   const real dx(L / processesX);
   const real dy(LY / processesY);
   const real dz(LZ / processesZ);

   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;

   loadPlanesAndCreateHalfSpaces("planes_div30x.txt", halfSpaces);
   loadPlanesYAndCreateHalfSpaces("planes_div30y.txt", halfSpacesY);

   int my_rank;
   MPI_Comm_rank(ex0, &my_rank);

   // Checking the total number of MPI processes
   if (processesX * processesY * processesZ != mpisystem->getSize())
   {
      std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX * processesY * processesZ << "\n\n"
                << std::endl;
      std::exit(EXIT_FAILURE);
   }

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

   // Computing the Cartesian coordinates of the neighboring processes
   int dims[] = {processesX, processesZ};
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

   int west[] = {center[0] - 1, center[1]};
   int east[] = {center[0] + 1, center[1]};

   int south[] = {center[0], center[1] - 1};
   int north[] = {center[0], center[1] + 1};

   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };

   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   std::vector<HalfSpace> mySpaces;

   if (west[0] < 0)
   {
      HalfSpace hs_y = halfSpacesY[center[0]];  
      if(center[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      mySpaces.push_back(halfSpaces[center[0]]);
      mySpaces.push_back(hs_y);

      defineLocalDomain(intersect(
               halfSpaces[center[0]],
               hs_y
      ));
   }
   else
   {
      // -x of halfSpaces[mpisystem->getRank()] and +x of halfSpaces[mpisystem->getRank()-1]
      HalfSpace hs = halfSpaces[center[0] - 1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
      
      HalfSpace hs_y = halfSpacesY[center[0]];  
      if(center[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      mySpaces.push_back(halfSpaces[center[0]]);
      mySpaces.push_back(hs_flip);
      mySpaces.push_back(hs_y);

      defineLocalDomain(intersect(
          halfSpaces[center[0]],
          hs_flip,
          hs_y
         ));
   }
   //===================================================================================

   pe_LOG_INFO_SECTION(log) 
   {
     log << "Center: " << center[0] << " " << center[1] << "\n";
   }

   //===================================================================================
   // Connecting the west neighbor
   if (west[0] > 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs = halfSpaces[west[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      HalfSpace hs_y = halfSpacesY[west[0]];  
      if(west[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[west[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,
              intersect(
              hs1,
              hs_flip,
              hs_y
              ));
   }

   if (west[0] == 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];

      HalfSpace hs_y = halfSpacesY[west[0]];  
      if(west[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[west[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,intersect(
              hs1,
              hs_y
              )
      );
   }

   //===================================================================================
   // Connecting the east neighbor
   if (east[0] < processesX)
   {
      MPI_Cart_rank(cartcomm, east, &rank);

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      HalfSpace hs_y = halfSpacesY[east[0]];  
      if(east[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[east[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,
              intersect(
              hs1,
              hs_flip,
              hs_y
              ));
   }

   //===================================================================================
   // Connecting the south neighbor
   if( south[1] >= 0 ) {

      MPI_Cart_rank( cartcomm, south, &rank );

      HalfSpace hs_y = halfSpacesY[center[0]];  

      if(mySpaces.size() == 2) {
         connect( rank, intersect(
            mySpaces[0],
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            mySpaces[0],
            mySpaces[1],
            hs_y));
      }
   }

   //===================================================================================
   // Connecting the north neighbor
   if( north[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, north, &rank );

      HalfSpace hs_y = halfSpacesY[center[0]];  
      HalfSpace hsy_temp = halfSpacesY[center[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      if(mySpaces.size() == 2) {
         connect( rank, intersect(
            mySpaces[0],
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            mySpaces[0],
            mySpaces[1],
            hs_y
         ));
      }
   }

   //===================================================================================
   // Connecting the south-west neighbor
   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southwest, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs_y = halfSpacesY[west[0]];  
      if (west[0] > 0)
      {
         HalfSpace hs = halfSpaces[west[0] - 1];
         HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
         connect( rank, intersect(
            hs1,
            hs_flip,
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            hs1,
            hs_y
         ));
      }

   }
 
   //===================================================================================
   // Connecting the south-east neighbor
   if( southeast[0] < processesX && southeast[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southeast, &rank );

      HalfSpace hs_y = halfSpacesY[east[0]];  

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect( rank, intersect(
         hs1,
         hs_flip,
         hs_y
      ));
   }

   //===================================================================================
   // Connecting the north-west neighbor
   if( northwest[0] >= 0 && northwest[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, northwest, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];

      HalfSpace hs_y = halfSpacesY[west[0]];  
      HalfSpace hsy_temp = halfSpacesY[west[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      if (west[0] > 0)
      {
         HalfSpace hs = halfSpaces[west[0] - 1];
         HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
         connect( rank, intersect(
            hs1,
            hs_flip,
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            hs1,
            hs_y
         ));
      }
   }
 
   //===================================================================================
   // Connecting the north-west neighbor
   if( northeast[0] < processesX && northeast[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, northeast, &rank );

      HalfSpace hs_y = halfSpacesY[east[0]];  
      HalfSpace hsy_temp = halfSpacesY[east[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect( rank, intersect(
         hs1,
         hs_flip,
         hs_y
      ) );
   }
   
   //===================================================================================

   // Checking the process setup
   theMPISystem()->checkProcesses();

   //===================================================================================

   // Setup of the VTK visualization
   if (g_vtk)
   {
      vtk::WriterID vtk = vtk::activateWriter("./paraview", visspacing, 0, timesteps, false);
   }

   // Create a custom material for the benchmark
   MaterialID elastic = createMaterial("elastic", 1.4, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
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
   bool resume = true;
   real epsilon = 2e-4;
   real targetVolumeFraction = 0.10;
   real radius2 = 0.05;

   int idx = 0;
   real h = 0.0075;
   // GetNonNewtViscosity
   // fbm force
   // fbm update
   //=================================================================================

   //=================================================================================
   //                   We have a heavy throwing ball
   //=================================================================================
   std::string fileName = std::string("archimedes.obj");

   BodyID particle;
   //=================================================================================
   if (!resume)
   {

     std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
     std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges);
     for (auto spherePos: spherePositions) {
       if (world->ownsPoint(spherePos))
       {
         createSphere( idx++, spherePos, sphereRad, elastic );
       }
     }
   }
   else
   {
      checkpointer.read( "../start.77" );
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

   const real   deltaT( 0.0005 );  // Size of a single time step
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

