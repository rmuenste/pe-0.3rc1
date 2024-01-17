#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <pe/core/Types.h>

using namespace pe::povray;

//=================================================================================================
// GenerateRandomPositions
//=================================================================================================
// Function to generate random positions within a cubic domain
std::vector<Vec3> generateRandomPositions(real L, real diameter, real volumeFraction, real eps) {

    WorldID world = theWorld();

    std::vector<Vec3> positions;

    real cellSize = diameter + eps;

    real partVol = 4./3. * M_PI * std::pow(0.5 * diameter, 3);
    real domainVol = L * L * L;

    std::cout << "Trying to generate volume fraction:  " << volumeFraction * 100.0 << std::endl;

    // Calculate the number of cells along one side of the cubic grid
    int gridSize = static_cast<int>(L / cellSize);

    // Calculate the total number of cells in the grid
    int totalCells = gridSize * gridSize * gridSize;

    // Calculate the maximum volume fraction possible for the current configuration
    real maxPhi = ((totalCells * partVol) / domainVol);

    if (volumeFraction > maxPhi) {
      std::cout << "User defined volume fraction: " << volumeFraction << " is too high for the current configuration" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    // Initialize a vector to keep track of visited cells
    std::vector<bool> cellVisited(totalCells, false);

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<real> dis(-cellSize / 2.0, cellSize / 2.0);

    // Generate random positions until the grid is full or the volume fraction is reached
    while (positions.size() < totalCells && (partVol * positions.size() / domainVol) < volumeFraction) {
        // Generate random cell indices
        int x = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int y = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int z = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);

        int cellIndex = x + y * gridSize + z * gridSize * gridSize;
        
        // Calculate the position of the cell center
        real posX = (x + 0.5) * cellSize;
        real posY = (y + 0.5) * cellSize;
        real posZ = (z + 0.5) * cellSize;

        Vec3 gpos(posX, posY, posZ);

        // Check if the cell has not been visited
        //if (!cellVisited[cellIndex] && world->ownsPoint( gpos ) ) {
        if (!cellVisited[cellIndex]) {
            // Mark the cell as visited
            cellVisited[cellIndex] = true;

            // Calculate the position of the cell center
            double posX = (x + 0.5) * cellSize;
            double posY = (y + 0.5) * cellSize;
            double posZ = (z + 0.5) * cellSize;

            // Create a Vec3 object for the position and add it to the positions vector
            positions.push_back(Vec3(posX, posY, posZ));
        }
    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
//    std::cout << MPISettings::rank() << ")local fraction:  " << (static_cast<real>(positions.size()) / totalCells) << " of " << volumeFraction << std::endl;
//    std::cout << MPISettings::rank() << ")local:  " << positions.size() << " of " << totalCells << std::endl;
    }

    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
    std::cout << MPISettings::rank() << ")Volume fraction:  " << solidFraction << std::endl;
    std::cout << MPISettings::rank() << ")local:  " << positions.size() << std::endl;
    return positions;
}
//=================================================================================================


//=================================================================================================
// Setup for the Kroupa Case
//=================================================================================================
void setupKroupa(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Re 1.5 configuration
  real simViscosity( 8.37e-5 );
  real simRho( 1.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );

  // Particle Bench Config 
  real slipLength( 0.001 );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication switch
  bool useLubrication(true);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real L( 0.1 );
  const real dx( L/processesX );
  const real dy( L/processesY );
  const real dz( L/processesZ );

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
  int periods[] = { true, true, true };
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

  real bx = 0.0;
  real by = 0.0;
  real bz = 0.0;

  // Size of the domain
  const real lx( L );
  const real ly( L );
  const real lz( L );

  decomposePeriodic3D(center, bx, by, bz, 
                              dx, dy, dz, 
                              lx, ly, lz, 
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
  MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  //======================================================================================== 
  // The way we atm include lubrication by increasing contact threshold
  // has problems: the particles get distributed to more domain bc the threshold AABB
  // is much larger than the particle actually is.
  // We can even run into the "registering distant domain" error when the AABB of the 
  // particle is close in size to the size of a domain part!
  //======================================================================================== 
  theCollisionSystem()->setLubrication(true);
  theCollisionSystem()->setSlipLength(slipLength);
  theCollisionSystem()->setMinEps(0.01);

  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  bool resume               = false;
  real epsilon              = 2e-4;
  real targetVolumeFraction = 0.35;
  real radius2              = 0.01 - epsilon;

  int idx = 0;
  real h = 0.0075;

  std::vector<Vec3> allPositions;
  int numPositions;

  //======================================================================================== 
  // The positions are created randomly on the root process and then bcasts 
  // to the other processes.
  //======================================================================================== 
  pe_EXCLUSIVE_SECTION(0) {
    allPositions = generateRandomPositions(0.1, 2.0 * radius2, targetVolumeFraction, epsilon); 
    numPositions = allPositions.size();
  }

  // Bcast the number of positions to other processes
  MPI_Bcast(&numPositions, 1, MPI_INT, 0, cartcomm);

  // Now all processes have the same value
  //std::cout << "Rank " << MPISettings::rank() << ": Received value " << numPositions << std::endl;

  //======================================================================================== 
  // For easier communication we create a double array that holds the positions 
  // in xyz, x1y1z1, and so on format 
  //======================================================================================== 
  std::vector<real> flatPositions(numPositions * 3);
  pe_EXCLUSIVE_SECTION(0) {
   for(std::size_t i(0); i < allPositions.size(); i++) {
      flatPositions[3 * i]     = allPositions[i][0];
      flatPositions[3 * i + 1] = allPositions[i][1];
      flatPositions[3 * i + 2] = allPositions[i][2];
   }
  }
  
  //======================================================================================== 
  // Bcast the flat array to the other processes 
  //======================================================================================== 
  // Broadcast the vector from the root process to all other processes
  MPI_Bcast(flatPositions.data(), numPositions * 3, MPI_DOUBLE, 0, cartcomm);

  pe_EXCLUSIVE_SECTION(0){
  
  }
  pe_EXCLUSIVE_ELSE {

   for(std::size_t i(0); i < numPositions * 3; i += 3) {
      Vec3 p(flatPositions[i], flatPositions[i + 1], flatPositions[i + 2]);
      allPositions.push_back(p);
   }
  
  }
  
  //=========================================================================================
  BodyID particle;
  Vec3 gpos(0.05 , 0.05, 0.1 - radius2 - epsilon);
  Vec3 gpos2(0.05 , 0.05, gpos[2] - (2. * radius2) - 6.0 * epsilon);
  //std::cout << "Separation dist:  " << gpos[2] - gpos2[2] << std::endl;

  //=========================================================================================
  if(!resume) {
    for (int i = 0; i < allPositions.size(); ++i) {
      Vec3 &position = allPositions[i];
      if( world->ownsPoint(position)) {
         SphereID sphere = createSphere(idx, position, radius2, elastic, true);
         ++idx;      
      }
    } 
  }
  else {
//    //checkpointer.read( "../start.1" );
//   if( world->ownsPoint( gpos ) ) {
//      createSphere( idx++, gpos, radius2, elastic );
//   }
//   gpos[2] -= 2. * (radius2 + epsilon);
//   if( world->ownsPoint( gpos  ) ) {
//      createSphere( idx++, gpos , radius2, elastic );
//   }
//   gpos[2] -= 2. * (radius2 + epsilon);
//   if( world->ownsPoint( gpos  ) ) {
//      createSphere( idx++, gpos , radius2, elastic );
//   }
//   gpos[2] -= 2. * (radius2 + epsilon);
//   if( world->ownsPoint( gpos  ) ) {
//      createSphere( idx++, gpos , radius2, elastic );
//   }
//   gpos[2] -= 2. * (radius2 + epsilon);
//   if( world->ownsPoint( gpos  ) ) {
//      createSphere( idx++, gpos , radius2, elastic );
//   }
  }
  //=========================================================================================

//  if( world->ownsPoint( gpos ) ) {
//    particle = createSphere( idx, gpos, radius2, elastic );
//    particle->setLinearVel( vel );
//  }
  //=========================================================================================  
  BodyID botPlane; 
  BodyID topPlane;

  pe_GLOBAL_SECTION
  {
     createPlane( 99999, 0.0, 0.0, 1.0, 0.0, granite, false ); // bottom border
     topPlane = createPlane( 88888, 0.0, 0.0,-1.0, -lz, granite, false ); // top border
  }

  pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "topPlaneID: "  << topPlane->getSystemID() << std::endl;
  }

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
      << " Lubrication h_c                         = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain volume                           = " << L * L * L << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Volume fraction[%]                      = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   

}

//=================================================================================================
// Cleanup for the Kroupa Case
//=================================================================================================
extern "C" void clean_world_() {

  World *w = world.get();
  MPISystem *m = mpisystem.get();
  delete w;
  delete m;

}
