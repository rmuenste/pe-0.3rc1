
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
   const real   radius  ( 0.24  );  // The radius of spheres of the granular media
   const real   spacing ( 0.001  );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 3000 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.01 );  // Size of a single time step

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
   const size_t visspacing    (   50 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (    51 );  // Number of particles in x-dimension with a specific color


   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Initial setups

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.0 );

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
   const real lpz( lz );  // Size of a process subdomain in z-direction

   int dims   [] = { processesX, processesZ };
   int periods[] = { true      , false       };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   int west      [] = { center[0]-1, center[1]   };
   int east      [] = { center[0]+1, center[1]   };
   int bottom    [] = { center[0]  , center[1]-1 };
   int top       [] = { center[0]  , center[1]+1 };
   int bottomwest[] = { center[0]-1, center[1]-1 };
   int bottomeast[] = { center[0]+1, center[1]-1 };
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

   std::cout << "Rank: " << mpisystem->getRank() << " +1,0,0: " << +center[0]*lpx << "| -1,0,0: " << -east[0]*lpx << "| 0,0,+1: " << +center[1]*lpz << "| 0,0,-1: " << -top[1]*lpz << std::endl;


   // Connecting the west neighbor
   {
      MPI_Cart_rank( cartcomm, west, &rank );
      // If we are dealing with the west neighbor, then set the x-offset to lx 
      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );


      std::cout << "west neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() << "| -1,0,0: " << -center[0]*lpx 
                            << "| 0,0,+1: " << +center[1]*lpz 
                            << "| 0,0,-1: " << -top[1]*lpz << " offset: " << offset << std::endl;


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

      std::cout << "east neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() << "| +1,0,0: " << +east[0]*lpx 
                            << "| 0,0,+1: " << +center[1]*lpz 
                            << "| 0,0,-1: " << -top[1]*lpz << " offset: " << offset << std::endl;
   }

<<<<<<< HEAD
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
=======
   // Connecting the bottom neighbor
   {
      MPI_Cart_rank( cartcomm, bottom, &rank );
      const Vec3 offset( 0, 0, ( ( bottom[1]<0 )?( 48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(0,0,-1), -center[1]*lpz ),
                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );

      std::cout << "bottom neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() 
               << "| 0,0,-1: " << -center[1]*lpz 
               << "| +1,0,0: " << +center[0]*lpx 
               << "| -1,0,0: " << -east[0]*lpx << " offset: " << offset << std::endl;

   }

   // Connecting the top neighbor
   {
      MPI_Cart_rank( cartcomm, top, &rank );
      const Vec3 offset( 0, 0, ( ( top[1]==processesZ )?( -48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(0,0,+1), +top[1]*lpz ),
                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );

      std::cout << "top neighbor Rank/myrank: " << rank << "/" << mpisystem->getRank() 
               << "| 0,0,+1: " << +top[1]*lpz 
               << "| +1,0,0: " << +center[0]*lpx 
               << "| -1,0,0: " << -east[0]*lpx << " offset: " << offset << std::endl;                                
   }

>>>>>>> github/ff-integration
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

#ifndef NDEBUG
   //Checking the process setup
   theMPISystem()->checkProcesses();
#endif

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

  Vec3 gpos(0.5 , 0.5, 0.5);
  Vec3 vel(2, 0, 0);
  int id = 0;
  if( world->ownsPoint( gpos ) ) {
    BodyID particle;
    particle = createSphere( id, gpos, radius, elastic );
    particle->isFixed();
    particle->setLinearVel( vel );
  }

  // Synchronization of the MPI processes
  world->synchronize();

  for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
    pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
    }
    world->simulationStep( stepsize );
  }

   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Barrier(MPI_COMM_WORLD);
   MPI_Finalize();

}
//*************************************************************************************************
