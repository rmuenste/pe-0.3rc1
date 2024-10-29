
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

//*************************************************************************************************


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
   const int processesX( 3 );  // Number of processes in x-direction
   const int processesZ( 2 );  // Number of processes in z-direction

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
   const size_t visspacing    (   1 );  // Number of time steps in-between two POV-Ray files
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

   // Checking the total number of MPI processes
   if( processesX < 2 || processesZ < 1 || processesX*processesZ != mpisystem->getSize() ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << processesX*processesZ << " : " << mpisystem->getSize()
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
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
   int periods[] = { false     , false      };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;

   loadPlanesAndCreateHalfSpaces("planes_div15.txt", halfSpaces);
   loadPlanesYAndCreateHalfSpaces("planesy_div15.txt", halfSpacesY);

   int west[] = {center[0] - 1, center[1]};
   int east[] = {center[0] + 1, center[1]};

   int south[] = {center[0], center[1] - 1};
   int north[] = {center[0], center[1] + 1};

   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };
   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   if (west[0] < 0)
   {
      HalfSpace hs_y = halfSpacesY[center[0]];  
      if(center[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

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
        HalfSpace hsy_temp = halfSpacesY[center[0]];
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
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,intersect(
              hs1,
              hs_y
              )
      );
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

   //===================================================================================
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

   //Checking the process setup
   theMPISystem()->checkProcesses();

   // Synchronization of the MPI processes
   world->synchronize();

   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Finalize();

}    

//*************************************************************************************************
