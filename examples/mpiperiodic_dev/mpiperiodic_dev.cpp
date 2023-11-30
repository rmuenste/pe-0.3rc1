
//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
#include <pe/vtk.h>
#include <boost/filesystem.hpp>
#include <pe/util/Checkpointer.h>
#include <pe/interface/decompose.h>
#include <sstream>
#include <random>
#include <algorithm>
#include <pe/core/Types.h>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only the FFD solver or a hard contact solver is used since parameters are tuned for them.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig2;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactAndFluid>::Config TargetConfig3;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig2, TargetConfig3);

void outputDataToFile(const std::vector<double>& all_points_x,
                      const std::vector<double>& all_points_y,
                      const std::vector<double>& all_points_z,
                      const std::vector<pe::id_t>& allSystemIDs,
                      const std::vector<int>& globalWallContacts,
                      const std::vector<double>& globalWallDistances,
                      real L,
                      real phi,
                      real radius
                      ) {
    
    // Open the file
    std::ofstream outputFile("output.dat");
    if (!outputFile.is_open()) {
        std::cerr << "Error opening file: output.dat" << std::endl;
        return;
    }

    outputFile << L << std::endl;
    outputFile << phi << std::endl;
    outputFile << radius << std::endl;

    // Output all_points_x, all_points_y, all_points_z
    for (size_t i = 0; i < all_points_x.size(); ++i) {
        outputFile << all_points_x[i] << " " << all_points_y[i] << " " << all_points_z[i] << std::endl;
    }

    // Output globalWallContacts and globalWallDistances
    for (size_t i = 0; i < globalWallContacts.size(); ++i) {
        outputFile << globalWallContacts[i] << " " << globalWallDistances[i] << std::endl;
    }

    // Close the file
    outputFile.close();
}

int getMaxPositions(real L, real diameter, real eps) {

    real cellSize = diameter + eps;

    // Calculate the number of cells along one side of the cubic grid
    int gridSize = static_cast<int>(L / cellSize);

    // Calculate the total number of cells in the grid
    return gridSize * gridSize * gridSize;
}

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
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) / totalCells) < volumeFraction) {
    while (positions.size() < totalCells && (partVol * positions.size() / domainVol) < volumeFraction) {
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) * partVol) / domainVol < volumeFraction) {
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
    //std::cout << MPISettings::rank() << ")local:  " << positions.size() << std::endl;
    return positions;
}


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================



//*************************************************************************************************
/*!\brief Main function for the mpinano example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * Particles in a non-periodic box with non-zero initial velocities.
 */
int main( int argc, char** argv )
{

   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( true   );  // Switch between spheres and granular particles
   const real   radius  ( 0.005  );  // The radius of spheres of the granular media
   const real   spacing ( 0.001  );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 0 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.0005 );  // Size of a single time step

   // Process parameters
   const int processesX( 4 );  // Number of processes in x-direction
   const int processesZ( 1 );  // Number of processes in z-direction

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( false );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( false );  // Switches the POV-Ray visualization on and off
   bool   vtk( true );

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (   2 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (    51 );  // Number of particles in x-dimension with a specific color


   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );

   /////////////////////////////////////////////////////
   // Initial setups

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.0 );
   world->setViscosity( 373e-3 );

   // Configuration of the MPI system
   MPISystemID mpisystem = theMPISystem();

   // Checking the size of the particles
   if( radius > real(0.3) ) {
      std::cerr << pe_RED
                << "\n Invalid radius for particles! Choose a radius of at maximum 0.3!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the total number of MPI processes
   if( processesX < 2 || processesZ < 1 || processesX*processesZ != mpisystem->getSize() ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << processesX*processesZ << " : " << mpisystem->getSize()
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Parsing the command line arguments
   for( int i=1; i<argc; ++i ) {
      if( std::strncmp( argv[i], "-no-povray", 10 ) == 0 ) {
         povray = false;
      }
   }

   // Setup of the random number generation
   setSeed( seed );

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.04, 0.1, 0.1, 0.3, 300, 1e6, 1e5, 2e5 );

   // Fixed simulation parameters
   const real L(0.1);

   // Cube domain
   const real lx( L );                         // Size of the domain in x-direction
   const real ly( L );                         // Size of the domain in y-direction
   const real lz( L );                         // Size of the domain in z-direction
   const real space( real(2)*radius+spacing );  // Space initially required by a single particle


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: Periodic 2D Regular Domain Decomposition

   const real lpx( lx / processesX );  // Size of a process subdomain in x-direction
   const real lpz( lz / processesZ );  // Size of a process subdomain in z-direction

   int dims   [] = { processesX, processesZ };
   int periods[] = { true      , true       };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   int west     [] = { center[0]-1, center[1]  , center[2] };
   int east     [] = { center[0]+1, center[1]  , center[2] };
   int south    [] = { center[0]  , center[1]-1, center[2] };
   int north    [] = { center[0]  , center[1]+1, center[2] };
   int southwest[] = { center[0]-1, center[1]-1, center[2] };
   int southeast[] = { center[0]+1, center[1]-1, center[2] };
   int northwest[] = { center[0]-1, center[1]+1, center[2] };
   int northeast[] = { center[0]+1, center[1]+1, center[2] };

   int bottom    [] = { center[0]  , center[1]-1 };
   int bottomwest[] = { center[0]-1, center[1]-1 };
   int bottomeast[] = { center[0]+1, center[1]-1 };

   int top       [] = { center[0]  , center[1]+1 };
   int topwest   [] = { center[0]-1, center[1]+1 };
   int topeast   [] = { center[0]+1, center[1]+1 };

   // Specify local subdomain (Since the domain is periodic we do not have to remove intersections at the border)
   // A displacement of a plane in the +x direction is expressed by a negative displacement -d
   // Here the half-space with normal (-1, 0, 0) is displaced 6 units from the origin to +x direction 
   // HalfSpace( Vec3(-1,0,0), -6 ),
   defineLocalDomain( intersect(
      HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
      HalfSpace( Vec3(-1,0,0), -east[0]*lpx ),
      HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
      HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ) );

   pe_LOG_DEBUG_SECTION( log ) {
     log << "Rank: " << mpisystem->getRank() << " +1,0,0: " << +center[0]*lpx << "| -1,0,0: " << -east[0]*lpx << "| 0,0,+1: " << +center[1]*lpz << "| 0,0,-1: " << -top[1]*lpz << "\n";
   }


   // Connecting the west neighbor
   {
      MPI_Cart_rank( cartcomm, west, &rank );
      // If we are dealing with the west neighbor, then set the x-offset to lx 
      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );


   pe_LOG_DEBUG_SECTION( log ) {
      log << "west neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() << "| -1,0,0: " << -center[0]*lpx 
                            << "| 0,0,+1: " << +center[1]*lpz 
                            << "| 0,0,-1: " << -top[1]*lpz << " offset: " << offset << "\n";

      }

      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
                                HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ), offset );
   }

   // Connecting the east neighbor
   {
      MPI_Cart_rank( cartcomm, east, &rank );

      // If we are dealing with the east neighbor, then set the x-offset to -lx (which is -lx units to the left, in -x direction) 
      const Vec3 offset( ( ( east[0]==processesX )?( -lx ) : ( 0 ) ), 0, 0 );
      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
                                HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ), offset );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "east neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() << "| +1,0,0: " << +east[0]*lpx 
                               << "| 0,0,+1: " << +center[1]*lpz 
                               << "| 0,0,-1: " << -top[1]*lpz << " offset: " << offset << "\n";
      }
   }

//   // Connecting the bottom neighbor
//   {
//      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
//      std::cout << "bottom neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| 0,0,-1: " << -center[1]*lpz
//               << "| +1,0,0: " << +center[0]*lpx
//               << "| -1,0,0: " << -east[0]*lpx << " offset: " << offset << std::endl;
//      MPI_Cart_rank( cartcomm, bottom, &rank );
//
//      connect( rank, intersect( HalfSpace( Vec3(0,0,-1), -center[1]*lpz ),
//                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
//                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );
//
//      std::cout << "bottom neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| 0,0,-1: " << -center[1]*lpz
//               << "| +1,0,0: " << +center[0]*lpx
//               << "| -1,0,0: " << -east[0]*lpx << " offset: " << offset << std::endl;
//
//
//   }
//
//   // Connecting the top neighbor
//   {
//      MPI_Cart_rank( cartcomm, top, &rank );
//      const Vec3 offset( 0, 0, ( ( top[1]==processesZ )?( -lz ):( 0 ) ) );
//      connect( rank, intersect( HalfSpace( Vec3(0,0,+1), +top[1]*lpz ),
//                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
//                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );
//
//      std::cout << "top neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| 0,0,+1: " << +top[1]*lpz
//               << "| +1,0,0: " << +center[0]*lpx
//               << "| -1,0,0: " << -east[0]*lpx << " offset: " << offset << std::endl;
//
//
//   }
//
//   // Connecting the bottom-west neighbor
//   {
//      MPI_Cart_rank( cartcomm, bottomwest, &rank );
//      const Vec3 offset( ( ( west[0]<0 )?( lx ):( 0 ) ), 0, ( ( bottom[1]<0 )?( lz ):( 0 ) ) );
//      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
//                                HalfSpace( Vec3(0,0,-1), -center[1]*lpz ) ), offset );
//
//      std::cout << "bottom-west neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| -1,0,0: " << -center[0]*lpx
//               << "| 0,0,-1: " << -center[1]*lpz << " offset: " << offset << std::endl;
//   }

//   // Connecting the bottom-east neighbor
//   {
//      MPI_Cart_rank( cartcomm, bottomeast, &rank );
//      const Vec3 offset( ( ( east[0]==processesX )?( -lx ):( 0 ) ), 0, ( ( bottom[1]<0 )?( lz ):( 0 ) ) );
//      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
//                                HalfSpace( Vec3(0,0,-1), -center[1]*lpz ) ), offset );
//
//      std::cout << "bottom-east neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| +1,0,0: " << +east[0]*lpx
//               << "| 0,0,-1: " << -center[1]*lpz << " offset: " << offset << std::endl;
//   }
//
//   // Connecting the top-west neighbor
//   {
//      MPI_Cart_rank( cartcomm, topwest, &rank );
//      const Vec3 offset( ( ( west[0]<0 )?( lx ):( 0 ) ), 0, ( ( top[1]==processesZ )?( -lz ):( 0 ) ) );
//      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
//                                HalfSpace( Vec3(0,0,+1), +top[1]*lpz ) ), offset );
//
//      std::cout << "top-west neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| -1,0,0: " << -center[0]*lpx
//               << "| 0,0,+1: " << +top[1]*lpz << " offset: " << offset << std::endl;
//   }
//
//   // Connecting the top-east neighbor
//   {
//      MPI_Cart_rank( cartcomm, topeast, &rank );
//      const Vec3 offset( ( ( east[0]==processesX )?( -lx ):( 0 ) ), 0, ( ( top[1]==processesZ )?( -lz ):( 0 ) ) );
//      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
//                                HalfSpace( Vec3(0,0,+1), +top[1]*lpz ) ), offset );
//
//      std::cout << "top-east neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank()
//               << "| +1,0,0: " << +east[0]*lpx
//               << "| 0,0,+1: " << +top[1]*lpz << " offset: " << offset << std::endl;
//   }

   //Checking the process setup
   theMPISystem()->checkProcesses();

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   // Creates the material "myMaterial" with the following material properties:
   //  - material density               : 2.54
   //  - coefficient of restitution     : 0.8
   //  - coefficient of static friction : 0.1
   //  - coefficient of dynamic friction: 0.05
   //  - Poisson's ratio                : 0.2
   //  - Young's modulus                : 80
   //  - Contact stiffness              : 100
   //  - dampingN                       : 10
   //  - dampingT                       : 11
   //MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   theCollisionSystem()->setSlipLength(1.5);
   theCollisionSystem()->setMinEps(0.01);

  //======================================================================================== 
  // The way we atm include lubrication by increasing contact threshold
  // has problems: the particles get distributed to more domain bc the threshold AABB
  // is much larger than the particle actually is.
  // We can even run into the "registering distant domain" error when the AABB of the 
  // particle is close in size to the size of a domain part!
  //======================================================================================== 
  BodyID particle;
  Vec3 gpos (0.05 , 0.05, 0.05);
  Vec3 gpos2(0.05 + 3. * radius, 0.05, 0.05);
  Vec3 vel(0.025, 0.0, 0.0);
  int id = 0;


  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  bool resume = false;
  real radius2 = 0.0075;
  real targetVolumeFraction = 0.35 / 2.0;

  std::vector<Vec3> allPositions;
  int numPositions;
  
  pe_EXCLUSIVE_SECTION(0) {
   allPositions = generateRandomPositions(0.1, 2.0 * radius2, 0.16, 1e-4); 
   numPositions = allPositions.size();
  }

  MPI_Bcast(&numPositions, 1, MPI_INT, 0, cartcomm);
  // Now all processes have the same value
  std::cout << "Rank " << MPISettings::rank() << ": Received value " << numPositions << std::endl;


  std::vector<real> flatPositions(numPositions);
  for(std::size_t i(0); i < allPositions.size(); i++) {
    flatPositions[i][0];
    flatPositions[i][1];
    flatPositions[i][2];
  }

  
  // Gather the data from each process into the all_data array
  MPI_Gatherv(x.data(), x.size(), MPI_DOUBLE,
              all_points_x.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
              0, cartcomm);              


  if(!resume) {
    for (int i = 0; i < allPositions.size(); ++i) {
      Vec3 &position = allPositions[i];
      if( world->ownsPoint(position)) {
         SphereID sphere = createSphere(id, position, radius2, elastic, true);
         sphere->setLinearVel(0.01, 0.0, 0.0);
         ++id;      
      }
    }
  } else {
   if( world->ownsPoint( gpos ) ) {
      particle = createSphere( id++, gpos, radius, elastic );
      particle->setLinearVel( vel );
      particle->getID();
   }
   if( world->ownsPoint( gpos2 ) ) {
      particle = createSphere( id++, gpos2, radius, elastic );
   }
  }
  //======================================================================================== 

  // Here we add some planes
  pe_GLOBAL_SECTION
  {
     BodyID botPlane = createPlane( 99999, 0.0, 0.0, 1.0, 0.0, granite, false ); // bottom border
     BodyID topPlane = createPlane( 88888, 0.0, 0.0, -1.0, -L, granite, false ); // top border
     std::cout << "topPlaneID: "  << topPlane->getSystemID() << std::endl;
     std::cout << "botPlaneID: "  << botPlane->getSystemID() << std::endl;
  }


  // Synchronization of the MPI processes
  world->synchronize();

  //======================================================================================== 
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
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
  //======================================================================================== 

  real domainVol = L * L * L;
  real partVol = 4./3. * M_PI * std::pow(radius2, 3);
  real phi = (particlesTotal * partVol)/domainVol * 100.0;

  std::string resOut = (resume) ? " resuming " : " not resuming ";

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << processesX * processesZ << "\n"
      << " Total timesteps                         = " << timesteps << "\n"
      << " Timestep size                           = " << stepsize << "\n"
      << " Total particles                         = " << particlesTotal << "\n"
      << " particle volume                         = " << partVol << "\n"
      << " Domain volume                           = " << L * L * L << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Volume fraction[%]                      = " << phi << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
    std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }


  for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
    pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
//     std::cout << particle->getPosition() << std::endl;
//     std::cout << particle->getLinearVel() << std::endl;
    }
    world->simulationStep( stepsize );
#define OUTPUT_LVL666 
#ifdef OUTPUT_LVL666
    std::vector<Vec3> localPoints;
    std::vector<pe::id_t> systemIDs;
    std::vector<int> localWallContacts;
    std::vector<real> localWallDistances;
    for (unsigned int i(0); i < theCollisionSystem()->getBodyStorage().size(); i++) {
      World::SizeType widx = static_cast<World::SizeType>(i);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if(body->getType() == sphereType) { //} || body->getType() == capsuleType) {
        SphereID s = static_body_cast<Sphere>(body);
        Vec3 pos = s->getPosition();
        localPoints.push_back(pos);
        systemIDs.push_back(body->getSystemID());
        //std::cout << "Wall contact: " << body->wallContact_ << " distance: " << body->contactDistance_ << std::endl;
        localWallContacts.push_back(body->wallContact_);
        localWallDistances.push_back(body->contactDistance_);
//        std::cout << "Real System Id: " << body->getSystemID() << std::endl;
//        Vec3 vel = body->getLinearVel();
//        Vec3 ang = body->getAngularVel();
//        std::cout << "Position: " << body->getSystemID() << " " << body->getPosition() << std::endl;
//        std::cout << "Velocity: " << body->getSystemID() << " " << body->getLinearVel() << std::endl;
      }
    }
    int totalPoints(0);
    int maxPoints(0);
    int localSize = localPoints.size();
    MPI_Reduce(&localSize, &totalPoints, 1, MPI_INT, MPI_SUM, 0, mpisystem->getComm());

pe_EXCLUSIVE_SECTION(0) {
    std::cout << "\n--" << "Total points: " << totalPoints << std::endl;
}    

//======================================================================================================
// We perform MPI communication of the particle positions here
// Additionally we also want to communicate the particle systemIDs
// and information about boundary contacts
//
// Since the number of particles can be different on each process we use the MPI_Gatherv function
//======================================================================================================
   std::vector<double> x;
   std::vector<double> y;
   std::vector<double> z;
   for(auto point : localPoints) {
      x.push_back(point[0]);
      y.push_back(point[1]);
      z.push_back(point[2]);
   }


   int local_size = x.size();

   // Get the size of the mpi world
   int size = mpisystem->getSize();
   std::vector<int> recvcounts(size);

   // Get the id of the current process
   int myrank = mpisystem->getRank(); 

   // Gather the size of data from each process on the root
   MPI_Gather(&local_size, 1, MPI_INT, recvcounts.data(), 1, MPI_INT, 0, cartcomm); 

   std::vector<int> displs(size, 0); // Displacement array

   if (myrank == 0) {
        // Calculate the displacement array for the gathered data
        for (int i = 1; i < size; ++i) {
            displs[i] = displs[i - 1] + recvcounts[i - 1];
            //std::cout << "Displacement " << i << " " << displs[i] << std::endl;
        }
   }   

   std::vector<double> all_points_x;
   std::vector<double> all_points_y;
   std::vector<double> all_points_z;
   std::vector<pe::id_t> allSystemIDs;
   std::vector<int> globalWallContacts;
   std::vector<real> globalWallDistances;
   int total_size;
   if (myrank == 0) {
      // Calculate the total size of the gathered data
      total_size = 0;
      for (int i = 0; i < recvcounts.size(); ++i) {
          total_size += recvcounts[i];
      }

      // Allocate space for the gathered data
      all_points_x.resize(total_size);      
      all_points_y.resize(total_size);      
      all_points_z.resize(total_size);      
      //allSystemIDs.resize(total_size * sizeof(pe::id_t));      
      allSystemIDs.resize(total_size);      
      globalWallContacts.resize(total_size);      
      globalWallDistances.resize(total_size);      
   }

//===============================================================================
// Here we gather the x-y-z data on the root process
//===============================================================================
   // Gather the data from each process into the all_data array
   MPI_Gatherv(x.data(), x.size(), MPI_DOUBLE,
               all_points_x.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
               0, cartcomm);              
   // Gather the data from each process into the all_data array
   MPI_Gatherv(y.data(), y.size(), MPI_DOUBLE,
               all_points_y.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
               0, cartcomm);              
   // Gather the data from each process into the all_data array
   MPI_Gatherv(z.data(), z.size(), MPI_DOUBLE,
               all_points_z.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
               0, cartcomm);              
//===============================================================================
// Here we gather the system ids on the root process
//===============================================================================
   MPI_Gatherv(systemIDs.data(), systemIDs.size(), MPI_UNSIGNED_LONG_LONG,
               allSystemIDs.data(), recvcounts.data(), displs.data(), MPI_UNSIGNED_LONG_LONG,
               0, cartcomm);              
//===============================================================================
// Here we gather the wall contacts on the root process
//===============================================================================
   MPI_Gatherv(localWallDistances.data(), localWallDistances.size(), MPI_DOUBLE,
               globalWallDistances.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
               0, cartcomm);              
   MPI_Gatherv(localWallContacts.data(), localWallContacts.size(), MPI_INT,
               globalWallContacts.data(), recvcounts.data(), displs.data(), MPI_INT,
               0, cartcomm);              

pe_EXCLUSIVE_SECTION(0) {

   outputDataToFile(all_points_x,
                    all_points_y,
                    all_points_z,
                    allSystemIDs,
                    globalWallContacts,
                    globalWallDistances,
                    L,
                    phi,
                    radius2
                    );

}    

#endif

  }

   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Barrier(MPI_COMM_WORLD);
   //MPI_Finalize();

}
//*************************************************************************************************
