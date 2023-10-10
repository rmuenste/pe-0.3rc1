#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>

using namespace pe::povray;

// Function to generate random positions within a cubic domain
std::vector<Vec3> generateRandomPositions(real L, real cellSize, real volumeFraction) {
    std::vector<Vec3> positions;

    // Calculate the number of cells along one side of the cubic grid
    int gridSize = static_cast<int>(L / cellSize);

    // Calculate the total number of cells in the grid
    int totalCells = gridSize * gridSize * gridSize;

    // Initialize a vector to keep track of visited cells
    std::vector<bool> cellVisited(totalCells, false);

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<real> dis(-cellSize / 2.0, cellSize / 2.0);

    // Generate random positions until the grid is full or the volume fraction is reached
    while (positions.size() < totalCells && (static_cast<real>(positions.size()) / totalCells) < volumeFraction) {
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
        if (!cellVisited[cellIndex] && world->ownsPoint( gpos ) ) {
            // Mark the cell as visited
            cellVisited[cellIndex] = true;

            // Calculate the position of the cell center
            double posX = (x + 0.5) * cellSize;
            double posY = (y + 0.5) * cellSize;
            double posZ = (z + 0.5) * cellSize;

            // Create a Vec3 object for the position and add it to the positions vector
            positions.push_back(Vec3(posX, posY, posZ));
        }
    }

    real partVol = 4./3. * M_PI * std::pow(cellSize, 3);
    real domainVol = L * L * L;
    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
    //std::cout << "Volume fraction:  " << solidFraction << std::endl;
    //std::cout << "local:  " << positions.size() << std::endl;
    return positions;
}

//int main() {
//    // Example usage
//    double cubeSideLength = 10.0;  // Length of the cubic domain
//    double cellSize = 1.0;         // Size of each cubic cell
//    double volumeFraction = 0.5;   // Desired volume fraction
//
//    std::vector<Vec3> randomPositions = generateRandomPositions(cubeSideLength, cellSize, volumeFraction);
//
//    // Print the generated positions
//    for (const Vec3& pos : randomPositions) {
//        std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
//    }
//
//    return 0;
//}

void setupKroupa(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );
  world->setLiquidSolid(true);
  world->setLiquidDensity(1.0);
  world->setViscosity( 8.37e-5 );
  world->setDamping( 1.0 );

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  // Resume Simulation
  bool resume ( true );


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

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
  }

  int idx = 0;

  // Create a custom material for the benchmark
  MaterialID myMaterial = createMaterial("Bench", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  real h = 0.0075;

  std::vector<Vec3> allPositions = generateRandomPositions(0.1, 0.01, 0.01); 
  //=========================================================================================
  BodyID particle;
  Vec3 gpos(0.02 , 0.02, 0.02);
  Vec3 vel(0.1, 0, 0.0);

  real radius2 = 0.005;
  MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

  //=========================================================================================
  if(!resume) {
    for (int i = 0; i < allPositions.size(); ++i) {
      Vec3 &position = allPositions[i];
      SphereID sphere = createSphere(idx, position, radius2, elastic, true);
      ++idx;      
    }
  }
  else {
    checkpointer.read( "../start.1" );
  }
  //=========================================================================================



//  if( world->ownsPoint( gpos ) ) {
//    particle = createSphere( idx, gpos, radius2, elastic );
//    particle->setLinearVel( vel );
//  }
  //=========================================================================================  
  
  pe_GLOBAL_SECTION
  {
     createPlane( 20000, 0.0, 0.0, 1.0, 0.0, granite, false ); // bottom border
     createPlane( 20004, 0.0, 0.0,-1.0, -lz, granite, false ); // top border
  }

  // Synchronization of the MPI processes
  world->synchronize();

  const real cylRad1 = 0.2;  
  const real cylRad2 = 0.4;  
  const real cylLength  = 0.4;

  real domainVol = L * L * L;
  //real domainVol = M_PI * std::pow(cylRad2, 2) * cylLength;
  //real cylVol = M_PI * std::pow(cylRad1, 2) * cylLength;
  //domainVol -= cylVol;
 
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

//  if(resume)
//    particlesTotal = primitivesTotal;

  real partVol = 4./3. * M_PI * std::pow(radius2, 3);

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Total particles          = " << particlesTotal << "\n"
      << " particle volume          = " << partVol << "\n"
      << " Domain volume            = " << L * L * L << "\n"
      << " Volume fraction[%]       = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
      << " Total objects            = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   

}
