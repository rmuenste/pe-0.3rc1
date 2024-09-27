
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


// Function to load planes from file and create HalfSpace instances
void loadPlanesAndCreateHalfSpaces(const std::string& filename, std::vector<HalfSpace> &halfSpaces) {
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << "\n";
        return;
    }
    int counter = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double px, py, pz, nx, ny, nz;
        
        // Read the plane's point and normal from the line
        if (!(iss >> px >> py >> pz >> nx >> ny >> nz)) {
            std::cerr << "Error: Malformed line: " << line << "\n";
            continue;
        }

        // Create a Vec3 for the normal vector
        Vec3 normal(nx, ny, nz);
        Vec3 point(px, py, pz);

        //std::cout << "Plane " << counter << " If " << trans(-point) * normal << " < 0 then the origin is outside." << std::endl;

        bool originOutside = (trans(-point) * normal < 0.0);
        //*  - > 0: The global origin is outside the half space\n
        //*  - < 0: The global origin is inside the half space\n
        //*  - = 0: The global origin is on the surface of the half space
        
        // Calculate the distance from the origin using the point-normal formula
        double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
        if (!originOutside) {
          dO = -dO; 
        } 
        
        // Create the HalfSpace instance
        halfSpaces.emplace_back(normal, dO);
        counter++;
    }
    
//    // Use the print function to print each HalfSpace to stdout
//    for (const auto& hs : halfSpaces) {
//        hs.print(std::cout, "\t");  // Passing std::cout for stdout and "\t" for tabbing
//    }
    
    file.close();
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
   //loadPlanesAndCreateHalfSpaces("my_planes.txt");

   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( true   );  // Switch between spheres and granular particles
   const real   radius  ( 0.05  );  // The radius of spheres of the granular media
   const real   spacing ( 0.001  );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 10 );  // Number of time steps for the flowing granular media
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
   const size_t visspacing    (   1 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (    51 );  // Number of particles in x-dimension with a specific color

//   std::vector<HalfSpace> halfSpaces1;
//   loadPlanesAndCreateHalfSpaces("planes.txt", halfSpaces1);
//   return EXIT_SUCCESS;

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
   if( processesX != 4 || processesZ < 1 || processesX*processesZ != mpisystem->getSize() ) {
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

   std::vector<HalfSpace> halfSpaces;
   loadPlanesAndCreateHalfSpaces("planes.txt", halfSpaces);


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

//   Vec3 p1 = Vec3(-1.4170880317687988, -2.5206289291381836, 0.4262872040271759);
//   Vec3 p2 = Vec3(-0.44539201259613037, -2.9075939655303955, 0.3683735132217407);
//   Vec3 p3 = Vec3(0.700677216053009, -2.8551599979400635, 0.3209492862224579);
//   Vec3 p4 = Vec3(1.7178499698638916, -2.3712100982666016, 0.27770888805389404);

   Vec3 p1 = Vec3(-1.1335910558700562, -2.659374952316284, 0.41303178668022156);
   Vec3 p2 = Vec3(-1.0874,-2.67839,0.410885);
   //Vec3 p2 = Vec3(-0.0776035264134407, -2.939215898513794, 0.3535797894001007);
   Vec3 p3 = Vec3(1.0964399576187134, -2.727055072784424, 0.30383288860321045);
   Vec3 p4 = Vec3(2.1420280933380127, -2.0990099906921387, 0.22060100734233856);

   pe_LOG_INFO_SECTION( log ) {
     log << "Rank: " << mpisystem->getRank() << " center " << Vec3(center[0], center[1], center[2])  << "\n";
   }

   if (west[0] < 0) {
      defineLocalDomain(
         halfSpaces[center[0]]
      );

      pe_LOG_INFO_SECTION( log ) {
      std::ostringstream oss;
      halfSpaces[center[0]].print(oss, "\t");
      
      log << "Rank: " << mpisystem->getRank() << " center " << Vec3(center[0], center[1], center[2])  << "\n";
      log << oss.str() << "\n";
      }
      pe_LOG_INFO_SECTION( log ) {
      
//      log << ") test point " << p1 << theWorld()->ownsPoint(p1)  << "\n";
      log << ") test point " << p2 << theWorld()->ownsPoint(p2)  << "\n";
//      log << ") test point " << p3 << theWorld()->ownsPoint(p3)  << "\n";
//      log << ") test point " << p4 << theWorld()->ownsPoint(p4)  << "\n";
      }
//   } else if (east[0] >= processesX) {
//      HalfSpace hs = halfSpaces[center[0]-1];
//      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), hs.getDisplacement());
//      defineLocalDomain(
//         hs_flip
//      );
//
//      pe_LOG_INFO_SECTION( log ) {
//      std::ostringstream oss;
//      hs_flip.print(oss, "\t");
//      
//      log << "Rank: " << mpisystem->getRank() << " center " << Vec3(center[0], center[1], center[2])  << "\n";
//      log << oss.str() << "\n";
//      }
//
//      pe_LOG_INFO_SECTION( log ) {
//      
//      log << ") test point " << p1 << theWorld()->ownsPoint(p1)  << "\n";
//      log << ") test point " << p2 << theWorld()->ownsPoint(p2)  << "\n";
//      log << ") test point " << p3 << theWorld()->ownsPoint(p3)  << "\n";
//      log << ") test point " << p4 << theWorld()->ownsPoint(p4)  << "\n";
//      } 
   } else {
      // -x of halfSpaces[mpisystem->getRank()] and +x of halfSpaces[mpisystem->getRank()-1]
      HalfSpace hs = halfSpaces[center[0]-1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), hs.getDisplacement());
      defineLocalDomain(intersect(
         halfSpaces[center[0]],
         hs_flip
         )
      );

      pe_LOG_INFO_SECTION( log ) {
      std::ostringstream oss;
      halfSpaces[center[0]].print(oss, "\t");
      hs_flip.print(oss, "\t");
      
      log << "Rank: " << mpisystem->getRank() << " center " << Vec3(center[0], center[1], center[2])  << "\n";
      log << oss.str() << "\n";
      }

      pe_LOG_INFO_SECTION( log ) {
      
//      log << ") test point " << p1 << theWorld()->ownsPoint(p1)  << "\n";
      log << ") test point " << p2 << theWorld()->ownsPoint(p2)  << "\n";
//      log << ") test point " << p3 << theWorld()->ownsPoint(p3)  << "\n";
//      log << ") test point " << p4 << theWorld()->ownsPoint(p4)  << "\n";

      HalfSpace hs = halfSpaces[center[0]-1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), hs.getDisplacement());
      HalfSpace mine = halfSpaces[center[0]];

      log << ") mine " << p2 << mine.containsPoint(p2)  << "\n";
      log << ") flip " << p2 << hs_flip.containsPoint(p2)  << "\n";
      }

   }

   // MPI Finalization
   MPI_Finalize();
   return EXIT_SUCCESS;

  // Connecting the west neighbor
  if( west[0] > 0 ) {
     MPI_Cart_rank( cartcomm, west, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs = halfSpaces[west[0]-1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), hs.getDisplacement());

      connect(rank, 
         intersect(
         hs1, 
         hs_flip)
      );
  }

  if( west[0] == 0 ) {
     MPI_Cart_rank( cartcomm, west, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];

      connect(rank, 
         hs1 
      );
  }

  // Connecting the east neighbor
  if( east[0] < processesX ) {
     MPI_Cart_rank( cartcomm, east, &rank );

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0]-1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), hs.getDisplacement());

      connect(rank, 
         intersect(
         hs1, 
         hs_flip)
      );

  }

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
  Vec3 gpos (p1);
  Vec3 vel(0.025, 0.0, 0.0);
  int id = 0;

  if( world->ownsPoint( gpos ) ) {
     particle = createSphere( id++, gpos, radius, elastic );
     particle->setLinearVel( -halfSpaces[0].getNormal() );
  }


  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  // const real   radius  ( 0.005  );
  //======================================================================================== 


  // Synchronization of the MPI processes
  world->synchronize();


  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << processesX * processesZ << "\n"
      << " Total timesteps                         = " << timesteps << "\n"
      << " Timestep size                           = " << stepsize << "\n" << std::endl;
    std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }


  for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
    pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
    }
    world->simulationStep( stepsize );
   for (int i=0; i < theCollisionSystem()->getBodyStorage().size(); i++) {
      World::SizeType widx = static_cast<World::SizeType>(i);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      std::cout << body->getPosition() << std::endl;
   }
  }

   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Finalize();

}
//*************************************************************************************************
