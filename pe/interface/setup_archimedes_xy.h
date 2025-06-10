#pragma once

using namespace pe::povray;

//===================================================================================
// Setup for the Archimedes case - XY plane subdivision
//===================================================================================
void setupArchimedesXY(MPI_Comm ex0);


//===================================================================================
// Setup for the Archimedes case
//===================================================================================
void setupArchimedesXY(MPI_Comm ex0)
{

   auto& config = SimulationConfig::getInstance();
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
   const real dx(L / config.getProcessesX());
   const real dy(LY / config.getProcessesY());
   const real dz(LZ / config.getProcessesZ());
   std::vector<HalfSpace> halfSpaces;

   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);
   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);

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

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "3D coordinates were created" << std::endl;
      std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
   }

   //===================================================================================
   int px = config.getProcessesX();
   int py = config.getProcessesY();
   int pz = config.getProcessesZ();

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
   if (east[0] < config.getProcessesX())
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
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
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
