//=================================================================================================
/*!
 *  \file mpicapsules.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


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
#include <pe/core/rigidbody/Capsule.h>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;




// Assert statically that only the FFD solver or a hard contact solver is used since parameters are tuned for them.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::FFDSolver>::Config TargetConfig1;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig2;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1, TargetConfig2);

std::vector<HalfSpace> generateDomainHalfPlanes(int pX, int pY, int row, int col, int my_rank) {

  Vec3 normalsY[] = { Vec3(-1, 0, 0), Vec3( 1, 0, 0) };
  Vec3 normalsX[] = { Vec3( 0,-1, 0), Vec3( 0, 1, 0) };

  std::vector<real> distanceX;// = {0, 70, 140};
  distanceX.push_back(0);
  distanceX.push_back(70);
  distanceX.push_back(140);
  std::vector<real> distanceY;// = {0, 70, 140};
  distanceY.push_back(0);
  distanceY.push_back(70);
  distanceY.push_back(140);

  int processesY = pY;
  int processesX = pX;

  std::vector<HalfSpace> spaces;

  // First row
  if (row == 0) {

      // The first element in a row
      // Here we will have 2 normals
      if (col == 0) {
        std::cout << "Half-Space coordinates: (0, 0)" << std::endl;
        spaces.push_back(HalfSpace(normalsY[0], -distanceY[col]));
        spaces.push_back(HalfSpace(normalsX[1], distanceX[0]));
      }
      // The last element in a row
      // Here we will have 2 normals
      else if (col == processesY - 1) {
        std::cout << "Half-Space coordinates: (0, pY-1)" << std::endl;
        spaces.push_back(HalfSpace(normalsY[1], distanceY[col-1]));
        spaces.push_back(HalfSpace(normalsX[1], distanceX[0]));
      }
      // The inner elements in a row
      // Here we will have 3 normals (2Y + 1X)
      else {
        std::cout << "Half-Space coordinates: (0, 1:pY-2)" << std::endl;
        spaces.push_back(HalfSpace(normalsY[1], distanceY[col - 1]));
        spaces.push_back(HalfSpace(normalsY[0], -distanceY[col]));
        spaces.push_back(HalfSpace(normalsX[1], distanceX[0]));
      }

  }//========================================================================

  // Inner PX rows
  if( row >= 1 && row < processesX - 1) {

    // First element in row
    // This domain has 3 half-planes
    if (col == 0) {
      spaces.push_back(HalfSpace(normalsY[0],-distanceY[0]));
      spaces.push_back(HalfSpace(normalsX[0], distanceX[row - 1]));
      spaces.push_back(HalfSpace(normalsX[1],-distanceX[row]));
      std::cout << "Half-Space coordinates: (1:pX-1, 0)" << std::endl;
    }
    // Inner row elements
    // These domains have 4 half-planes
    else if (col >= 1 && col < processesY - 1) {
        std::cout << "Half-Space coordinates: (1:pX-1, 1:pY-1)" << std::endl;
        spaces.push_back(HalfSpace(normalsY[1], distanceY[col - 1]));
        spaces.push_back(HalfSpace(normalsY[0],-distanceY[col]));
        spaces.push_back(HalfSpace(normalsX[0], distanceX[row - 1]));
        spaces.push_back(HalfSpace(normalsX[1],-distanceX[row]));
    }
    // Last element in row
    // This domain has 3 half-planes
    else if (col == processesY - 1) {
      std::cout << "Half-Space coordinates: (1:pX-1, 1:pY-1)" << std::endl;
      spaces.push_back(HalfSpace(normalsY[1], distanceY[col - 1]));
      spaces.push_back(HalfSpace(normalsX[0], distanceX[row - 1]));
      spaces.push_back(HalfSpace(normalsX[1],-distanceX[row]));
    }
    else {
      std::cout << "Invalid column index: " << col << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  // last row
  if (row == processesX - 1) {

    // The first element in a row
    // Here we will have 2 normals
    if (col == 0) {
      std::cout << "Half-Space coordinates: (pX-1, 0)" << std::endl;
      spaces.push_back(HalfSpace(normalsY[0], -distanceY[col]));
      spaces.push_back(HalfSpace(normalsX[0], distanceX[processesX-2]));
    }
    // The last element in a row
    // Here we will have 2 normals
    else if (col == processesY - 1) {
      std::cout << "Half-Space coordinates: (pX-1, pY-1)" << std::endl;
      spaces.push_back(HalfSpace(normalsY[1], distanceY[col-1]));
      spaces.push_back(HalfSpace(normalsX[0], distanceX[processesX-2]));
    }
    // The inner elements in a row
    // Here we will have 3 normals (2Y + 1X)
    else {
      std::cout << "Half-Space coordinates: (pX-1, 1:pY-2)" << std::endl;
      spaces.push_back(HalfSpace(normalsY[1], distanceY[col - 1]));
      spaces.push_back(HalfSpace(normalsY[0], -distanceY[col]));
      spaces.push_back(HalfSpace(normalsX[0], distanceX[processesX-2]));
    }
  }

  return spaces;
}

void addDomainNeighbor(std::vector<HalfSpace> &spaces, int my_rank) {

  size_t size = spaces.size();
  switch (size) {
    case 2:
    connect( my_rank, intersect(
      spaces[0],
      spaces[1])
    );
    break;
    case 3:
    connect( my_rank, intersect(
      spaces[0],
      spaces[1],
      spaces[2])
    );
    break;
    case 4:
    connect( my_rank, intersect(
      spaces[0],
      spaces[1],
      spaces[2],
      spaces[3])
    );
    break;
    default:
      std::cout << "Invalid number of domain half-planes: " << size << std::endl;
      std::exit(EXIT_FAILURE);
      break;
  }

}

void defineLocalHalfPlanes(std::vector<HalfSpace> &spaces, int my_rank) {

  size_t size = spaces.size();
  switch (size) {
    case 2:
    defineLocalDomain(intersect(
      spaces[0],
      spaces[1])
    );
    break;
    case 3:
    defineLocalDomain(intersect(
      spaces[0],
      spaces[1],
      spaces[2])
    );
    break;
    case 4:
    defineLocalDomain(intersect(
      spaces[0],
      spaces[1],
      spaces[2],
      spaces[3])
    );
    break;
    default:
      std::cout << "Invalid number of domain half-planes: " << size << std::endl;
      std::exit(EXIT_FAILURE);
      break;
  }

}

void generateDomainNeighbors(int pX, int pY, int row, int col, int my_rank, MPI_Comm &cartcomm) {

  int center[] = { row, col };
  int west     [] = { center[0]  , center[1]-1 };
  int east     [] = { center[0]  , center[1]+1 };
  int south    [] = { center[0]+1, center[1]   };
  int north    [] = { center[0]-1, center[1]   };
  int southwest[] = { center[0]+1, center[1]-1 };
  int southeast[] = { center[0]+1, center[1]+1 };
  int northwest[] = { center[0]-1, center[1]-1 };
  int northeast[] = { center[0]-1, center[1]+1 };

  int processesY = pY;
  int processesX = pX;

  int rank = -1;

  std::cout << "Center: (" << row << "," << col << ")" << std::endl;
  // First row
  if (row == 0) {

      // The first element in a row
      // Here we will have 3 neighbors
      if (col == 0) {
        std::cout << "Half-Space coordinates: (0, 0), #n = 3" << std::endl;

        // east     
        std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
        MPI_Cart_rank( cartcomm, east, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // south    
        spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
        MPI_Cart_rank( cartcomm, south, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // southeast
        spaces = generateDomainHalfPlanes(pX, pY, southeast[0], southeast[1], my_rank);
        MPI_Cart_rank( cartcomm, southeast, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();
      }
      // The last element in a row
      // Here we will have 3 neighbors
      else if (col == processesY - 1) {
        std::cout << "Half-Space coordinates: (0, pY-1), #n = 3" << std::endl;
        // west     
        std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
        MPI_Cart_rank( cartcomm, west, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // south    
        spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
        MPI_Cart_rank( cartcomm, south, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // southwest
        spaces = generateDomainHalfPlanes(pX, pY, southwest[0], southwest[1], my_rank);
        MPI_Cart_rank( cartcomm, southwest, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();
      }
      // The inner elements in a row
      // Here we will have 5 neighbors
      else {
        std::cout << "Half-Space coordinates: (0, 1:pY-2), #n = 5" << std::endl;
        // west     
        std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
        MPI_Cart_rank( cartcomm, west, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // east     
        spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
        MPI_Cart_rank( cartcomm, east, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // south    
        spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
        MPI_Cart_rank( cartcomm, south, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // southwest
        spaces = generateDomainHalfPlanes(pX, pY, southwest[0], southwest[1], my_rank);
        MPI_Cart_rank( cartcomm, southwest, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();

        // southeast
        spaces = generateDomainHalfPlanes(pX, pY, southeast[0], southeast[1], my_rank);
        MPI_Cart_rank( cartcomm, southeast, &rank );
        addDomainNeighbor(spaces, rank);
        spaces.clear();
      }

  }//========================================================================

  // Inner PX rows
  if( row >= 1 && row < processesX - 1) {

    // First element in row
    // This domain has 5 neighbors
    if (col == 0) {
      std::cout << "Half-Space coordinates: (1:pX-1, 0), #n = 5" << std::endl;
      // east     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
      MPI_Cart_rank( cartcomm, east, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // south    
      spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
      MPI_Cart_rank( cartcomm, south, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // southeast
      spaces = generateDomainHalfPlanes(pX, pY, southeast[0], southeast[1], my_rank);
      MPI_Cart_rank( cartcomm, southeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northeast
      spaces = generateDomainHalfPlanes(pX, pY, northeast[0], northeast[1], my_rank);
      MPI_Cart_rank( cartcomm, northeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
    // Inner row elements
    // These domains have 8 neighbors
    else if (col >= 1 && col < processesY - 1) {
      std::cout << "Half-Space coordinates: (1:pX-1, 1:pY-1), #n = 5" << std::endl;
      // west     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
      MPI_Cart_rank( cartcomm, west, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
      // east     
      spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
      MPI_Cart_rank( cartcomm, east, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // south    
      spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
      MPI_Cart_rank( cartcomm, south, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // southwest
      spaces = generateDomainHalfPlanes(pX, pY, southwest[0], southwest[1], my_rank);
      MPI_Cart_rank( cartcomm, southwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // southeast
      spaces = generateDomainHalfPlanes(pX, pY, southeast[0], southeast[1], my_rank);
      MPI_Cart_rank( cartcomm, southeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northwest
      spaces = generateDomainHalfPlanes(pX, pY, northwest[0], northwest[1], my_rank);
      MPI_Cart_rank( cartcomm, northwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northeast
      spaces = generateDomainHalfPlanes(pX, pY, northeast[0], northeast[1], my_rank);
      MPI_Cart_rank( cartcomm, northeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
    // Last element in row
    // This domain has 5 neighbors
    else if (col == processesY - 1) {
      std::cout << "Half-Space coordinates: (1:pX-1, 1:pY-1), #n = 8" << std::endl;
      // west     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
      MPI_Cart_rank( cartcomm, west, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // south    
      spaces = generateDomainHalfPlanes(pX, pY, south[0], south[1], my_rank);
      MPI_Cart_rank( cartcomm, south, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // southwest
      spaces = generateDomainHalfPlanes(pX, pY, southwest[0], southwest[1], my_rank);
      MPI_Cart_rank( cartcomm, southwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northwest
      spaces = generateDomainHalfPlanes(pX, pY, northwest[0], northwest[1], my_rank);
      MPI_Cart_rank( cartcomm, northwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
  }

  // last row
  if (row == processesX - 1) {

    // The first element in a row
    // Here we will have 3 neighbors
    if (col == 0) {
      std::cout << "Half-Space coordinates: (pX-1, 0), #n = 3" << std::endl;
      // east     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
      MPI_Cart_rank( cartcomm, east, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northeast
      spaces = generateDomainHalfPlanes(pX, pY, northeast[0], northeast[1], my_rank);
      MPI_Cart_rank( cartcomm, northeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
    // The last element in a row
    // Here we will have 3 neighbors
    else if (col == processesY - 1) {
      std::cout << "Half-Space coordinates: (pX-1, pY-1), #n = 3" << std::endl;
      // west     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
      MPI_Cart_rank( cartcomm, west, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northwest
      spaces = generateDomainHalfPlanes(pX, pY, northwest[0], northwest[1], my_rank);
      MPI_Cart_rank( cartcomm, northwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
    // The inner elements in a row
    // Here we will have 5 neighbors
    else {
      std::cout << "Half-Space coordinates: (pX-1, 1:pY-2), #n = 5" << std::endl;
      // west     
      std::vector<HalfSpace> spaces = generateDomainHalfPlanes(pX, pY, west[0], west[1], my_rank);
      MPI_Cart_rank( cartcomm, west, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // east     
      spaces = generateDomainHalfPlanes(pX, pY, east[0], east[1], my_rank);
      MPI_Cart_rank( cartcomm, east, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // north    
      spaces = generateDomainHalfPlanes(pX, pY, north[0], north[1], my_rank);
      MPI_Cart_rank( cartcomm, north, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northwest
      spaces = generateDomainHalfPlanes(pX, pY, northwest[0], northwest[1], my_rank);
      MPI_Cart_rank( cartcomm, northwest, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();

      // northeast
      spaces = generateDomainHalfPlanes(pX, pY, northeast[0], northeast[1], my_rank);
      MPI_Cart_rank( cartcomm, northeast, &rank );
      addDomainNeighbor(spaces, rank);
      spaces.clear();
    }
  }
}

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a random angle between \f$ [-\frac{\pi}{20}..\frac{\pi}{20}] \f$.
 *
 * \return The random angle (radian measure).
 */
real angle()
{
   return rand<real>( -M_PI/real(20), M_PI/real(20) );
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
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   const real   velocity  (  0.02 );  // Initial maximum velocity of the spherical particles

   const size_t timesteps ( 60000 );  // Total number of time steps
   const real   stepsize  (  0.0004 );  // Size of a single time step

   const size_t seed      ( 12345 );  // Seed for the random number generation

   bool   povray    ( true );        // Switches the POV-Ray visualization on and off
   bool vtk( true );
   const size_t visspacing(    400 );  // Number of time steps inbetween two POV-Ray files

   const bool spheres     ( false );  // Switch between spheres and capsules particles
   const unsigned int H ( 24 );              // Height of the capsule stack



//   // Parsing the command line arguments
//   CommandLineInterface& cli = CommandLineInterface::getInstance();
//   cli.getDescription().add_options()
//      ("particles", value< std::vector<int> >()->multitoken()->required(), "number of particles in x-, y- and z-dimension")
//      ("processes", value< std::vector<int> >()->multitoken()->required(), "number of processes in x-, y- and z-dimension")
//   ;
//   cli.parse( argc, argv );
//   cli.evaluateOptions();
//   variables_map& vm = cli.getVariablesMap();
//   if( vm.count( "no-povray" ) > 0 )
//      povray = false;

   const int nx( 3 );
   const int ny( 3 );
   const int nz( 3 );
   const int px( 3 );
   const int py( 3 );
   const int pz( 3 );

   if( nx <= 0 ) {
      std::cerr << " Invalid number of particles in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( ny <= 0 ) {
      std::cerr << " Invalid number of particles in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( px <= 0 ) {
      std::cerr << " Invalid number of processes in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( py <= 0 ) {
      std::cerr << " Invalid number of processes in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( nx % px != 0 ) {
      std::cerr << " Bad ratio between number of particles and number of processes in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( ny % py != 0 ) {
      std::cerr << " Bad ratio between number of particles and number of processes in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }

   if( MPISettings::size() < px*py ) {
      std::cerr << " Number of available processes is smaller than the number of processes specified on the command line." << std::endl;
      pe::exit(EXIT_FAILURE);
   }

   const real   cradius   (  0.2  );  // The radius of the spherical cap of a capsule
   const real   clength   (  0.6  );  // The radius of the spherical cap of a capsule
   const real   radius    (  0.5  );  // The radius of spherical particles
   const real   spacing   (  2.0  );  // Initial spacing inbetween two spherical particles
   /////////////////////////////////////////////////////
   // Initial setups

   // Checking the ratio of the particle radius and the spacing
   if( real(2.1)*radius >= spacing ) {
      std::cerr << pe_RED << "\n Invalid particle/spacing ratio!\n\n" << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   const real lx( nx * spacing );  // Length of the simulation domain in x-dimension
   const real ly( ny * spacing );  // Length of the simulation domain in y-dimension
   const real lz( nz * spacing );  // Length of the simulation domain in z-dimension

   setSeed( seed );  // Setup of the random number generation

   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   WorldID     world     = theWorld();
   world->setGravity( 0.0, 0.0, -0.4 );
   MPISystemID mpisystem = theMPISystem();


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Regular Domain Decomposition

   const real dx( lx / px );
   const real dy( ly / py );
   const real dz( lz / pz );

   int dims   [] = { px   , py   , pz    };
   int periods[] = { false, false, false };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   if( cartcomm == MPI_COMM_NULL ) {
      MPI_Finalize();
      return 0;
   }

   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );
   rank = mpisystem->getRank();

  for (int i(0); i < px; ++i) {

    for (int j(0); j < py; ++j) {

      pe_EXCLUSIVE_SECTION(i * py + j) {
        std::vector<HalfSpace> spaces = generateDomainHalfPlanes(px, py, i, j, rank);
        std::cout << rank << ")Rank: " << i * py + j << " Spaces size: " << spaces.size() << std::endl;

        defineLocalHalfPlanes(spaces, rank);

      }
    }
  }

  generateDomainNeighbors(px, py, center[0], center[1], rank, cartcomm);


   /////////////////////////////////////////////////////
   // MPI Finalization


#ifndef NDEBUG
   // Checking the process setup
   mpisystem->checkProcesses();
#endif

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray )
   {
      const Vec3 location( real(0.5)*lx, -real(0.5)*lx/0.65, real(0.5)*lz );
      const Vec3 focus   ( real(0.5)*lx,  real(0.5)*ly     , real(0.5)*lz );

      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->setBackground( Color( 0, 0, 0 ) );
      pov->include( "settings.inc" );

      pov->addLightSource( PointLight( location, Color( 1, 1, 1 ) ) );

      CameraID camera = theCamera();
      camera->setLocation( location );
      camera->setFocus   ( focus    );

      std::ostringstream texture;
      texture << "Texture" << theMPISystem()->getRank()%13;
      pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
   }


   /////////////////////////////////////////////////////
   // Setup of the simulation domain

   pe_GLOBAL_SECTION
   {
      createPlane( 0,  0.0,  0.0,  1.0, 0.0, granite );
   }


   /////////////////////////////////////////////////////
   // Deterministic setup of the particles

   Vec3 gpos, vel;
   size_t id( 0 );

   const int nxpp( nx / px );
   const int nypp( ny / py );
   const int nzpp( nz / pz );

   // Starting the time measurement for the setup
   WcTimer setupTime;
   setupTime.start();

//   // Creating the iron capsule 1 with a radius of 0.9 and a length 2.5 of at the global
//   // position (2,3,4). Per default the capsule is visible in all visualizations. Note
//   // that the capsule is automatically added to the simulation world and is immediately
//   // part of the entire simulation. The function returns a handle to the newly created
//   // capsule, which can be used to for instance rotate the capsule around the global
//   // y-axis.
//   const real   cradius   (  0.2  );  // The radius of the spherical cap of a capsule
//   const real   clength   (  0.6  );  // The radius of the spherical cap of a capsule

//   capsule->rotate( 0.0, PI/3.0, 0.0 );
//   PE_PUBLIC CapsuleID createCapsule( id_t uid, const Vec3& gpos, real radius,
//                                   real length, MaterialID material, bool visible )
//

   real cylinderRadius = 0.25;
   real length = 8.0;
   real totalLength = length + 2. * cylinderRadius;
   real boxHeight = 2. * cylinderRadius;

   // Setup of the wooden box stack
   for (unsigned int k = 0; k < 48; ++k) {
     for (unsigned int i = H; i > 0; --i) {
       for (unsigned int j = 0; j < i; ++j)
       {
         Vec3 pos(0, 0, 0);
         if(k % 2 == 0)
           pos = Vec3( -2.5 * (i - 1) + j * totalLength - 27 + 50, -90 + k * 12 * cylinderRadius, 0.5 * boxHeight + (H - i) * boxHeight);
         else {
           pos = Vec3( -2.5 * (i - 1) + j * totalLength, -90 + k * 12 * cylinderRadius, 0.5 * boxHeight + (H - i) * boxHeight);
           pos[0] = -pos[0] + 53. - 27 + 70;
         }
         if (world->ownsPoint(pos)) {
           CapsuleID cap = createCapsule(++id, pos, cylinderRadius, length, oak);
           cap->rotate(0.0, 0.0, angle());
         }
       }
     }
   }

   Vec3 spos(0.0, -120.0, 7.5);
   if (world->ownsPoint(spos)) {
     // Setup of the metal sphere
     SphereID s = createSphere(++id, spos, 1.5, iron);
     s->setLinearVel(0.0, 5.5, 0.1);
   }

   // Synchronization of the MPI processes
   world->synchronize();

   // Ending the time measurement for the setup
   setupTime.end();

   unsigned long primitivesTotal(0);
   int numBodies = theCollisionSystem()->getBodyStorage().size();
   unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
   MPI_Reduce(&bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                      = (" << lx << "," << ly << "," << lz << ")\n"
                << " Number of MPI processes                 = (" << px << "," << py << "," << pz << ")\n"
                << " Total number of particles               = " << primitivesTotal << "\n"
                << " World gravity                           = " << world->getGravity() << "\n"
                << " Number of particles on each process     = " << nxpp*nypp*nzpp << "\n"
                << " Radius of a single particle             = " << radius << "\n"
                << " Initial spacing inbetween two particles = " << spacing << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "------------------------------------------------------------------------------" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Setup timing results

   double localTime( setupTime.total() );
   double globalMin( 0.0 );
   double globalMax( 0.0 );
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SETUP RESULTS" << pe_OLDCOLOR << "---------------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Simulation loop

   WcTimer simTime;
   simTime.start();
   world->run( timesteps, stepsize );
   simTime.end();


   /////////////////////////////////////////////////////
   // Simulation timing results

   localTime  = simTime.total();
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
