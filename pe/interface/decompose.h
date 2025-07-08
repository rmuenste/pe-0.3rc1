#ifndef _PE_DECOMPOSE_H_
#define _PE_DECOMPOSE_H_

//*************************************************************************************************
/*!\file
 * \brief Domain decomposition interface for MPI-based parallel simulations
 *
 * This header provides the complete interface for domain decomposition in the PE physics engine.
 * Domain decomposition is essential for distributing computational workload across multiple
 * MPI processes in parallel simulations. The functions in this file handle the creation of
 * local domain boundaries and establishment of inter-process communication patterns.
 *
 * \section Available Functions
 *
 * The interface provides five core decomposition functions:
 *
 * 1. **decomposeDomain()** - Universal non-periodic decomposition
 *    - Handles all axis configurations (1D, 2D, 3D)
 *    - Replaces previous axis-specific functions
 *    - Works with any process grid topology
 *
 * 2. **decomposePeriodic3D()** - Full 3D periodic decomposition
 *    - Periodic boundary conditions in all three dimensions
 *    - Supports wraparound connections at domain boundaries
 *
 * 3. **decomposePeriodicXY3D()** - XY-periodic decomposition
 *    - Periodic boundaries in X and Y directions
 *    - Non-periodic (fixed) boundaries in Z direction
 *
 * 4. **decomposePeriodicX3D()** - X-periodic decomposition
 *    - Periodic boundaries in X direction only
 *    - Non-periodic (fixed) boundaries in Y and Z directions
 *    - Ideal for extrusion, shear, or channeled flow simulations
 *
 * 5. **decomposeDomain2DArchimedes()** - Specialized 2D decomposition
 *    - Custom geometry support using HalfSpace definitions
 *    - Designed for Archimedes screw and complex 2D shapes
 *
 * \section Usage Requirements
 *
 * - MPI support must be enabled (HAVE_MPI)
 * - Proper MPI Cartesian communicator setup required
 * - Process coordinates must be obtained via MPI_Cart_coords
 * - Domain parameters must be consistent across all processes
 *
 * \section Design Principles
 *
 * - Each function establishes local domain boundaries using HalfSpace intersections
 * - Neighbor connectivity is determined by MPI Cartesian topology
 * - Up to 26 neighbors are supported in 3D (6 faces + 12 edges + 8 corners)
 * - Domain boundaries are defined to ensure no gaps or overlaps
 * - Communication patterns are optimized for nearest-neighbor interactions
 */
//*************************************************************************************************

//*************************************************************************************************
// Includes
//*************************************************************************************************


#if HAVE_MPI
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/Merging.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>

#else

#endif

namespace pe {

//*************************************************************************************************
/*!\brief Universal domain decomposition function for non-periodic boundaries.
 *
 * \param center Array containing the 3D coordinates [x,y,z] of the current process in the MPI Cartesian grid
 * \param bx Lower bound of the global domain in X direction
 * \param by Lower bound of the global domain in Y direction  
 * \param bz Lower bound of the global domain in Z direction
 * \param dx Size of each domain cell in X direction
 * \param dy Size of each domain cell in Y direction
 * \param dz Size of each domain cell in Z direction
 * \param px Total number of processes in X direction
 * \param py Total number of processes in Y direction
 * \param pz Total number of processes in Z direction
 * 
 * This function performs domain decomposition for MPI-based parallel simulations with non-periodic
 * boundary conditions. It automatically handles all decomposition patterns including:
 * 
 * - Single-axis decompositions (e.g., 12x1x1, 1x8x1, 1x1x4)
 * - Dual-axis decompositions (e.g., 4x3x1, 2x1x6, 1x4x2)  
 * - Full 3D decompositions (e.g., 2x2x2, 3x3x3)
 *
 * The function establishes the local domain boundaries using HalfSpace intersections and connects
 * to all valid neighboring processes (up to 26 neighbors in 3D). Neighbor connectivity is
 * determined by the MPI Cartesian topology.
 *
 * \note This function replaces the previously separate decomposeDomainX and decomposeDomainZ
 *       functions, providing universal coverage for all axis configurations.
 * \note Requires MPI support and proper MPI Cartesian communicator setup.
 */
void decomposeDomain(int center[], real bx, real by, real bz, real dx, real dy, real dz, int px, int py, int pz) {

  int west     [] = { center[0]-1, center[1]  , center[2] };
  int east     [] = { center[0]+1, center[1]  , center[2] };
  int south    [] = { center[0]  , center[1]-1, center[2] };
  int north    [] = { center[0]  , center[1]+1, center[2] };
  int southwest[] = { center[0]-1, center[1]-1, center[2] };
  int southeast[] = { center[0]+1, center[1]-1, center[2] };
  int northwest[] = { center[0]-1, center[1]+1, center[2] };
  int northeast[] = { center[0]+1, center[1]+1, center[2] };

  // bottom
  int bottom         [] = { center[0]  , center[1]  , center[2]-1 };
  int bottomwest     [] = { center[0]-1, center[1]  , center[2]-1 };
  int bottomeast     [] = { center[0]+1, center[1]  , center[2]-1 };
  int bottomsouth    [] = { center[0]  , center[1]-1, center[2]-1 };
  int bottomnorth    [] = { center[0]  , center[1]+1, center[2]-1 };
  int bottomsouthwest[] = { center[0]-1, center[1]-1, center[2]-1 };
  int bottomsoutheast[] = { center[0]+1, center[1]-1, center[2]-1 };
  int bottomnorthwest[] = { center[0]-1, center[1]+1, center[2]-1 };
  int bottomnortheast[] = { center[0]+1, center[1]+1, center[2]-1 };

  // top
  int top         [] = { center[0]  , center[1]  , center[2]+1 };
  int topwest     [] = { center[0]-1, center[1]  , center[2]+1 };
  int topeast     [] = { center[0]+1, center[1]  , center[2]+1 };
  int topsouth    [] = { center[0]  , center[1]-1, center[2]+1 };
  int topnorth    [] = { center[0]  , center[1]+1, center[2]+1 };
  int topsouthwest[] = { center[0]-1, center[1]-1, center[2]+1 };
  int topsoutheast[] = { center[0]+1, center[1]-1, center[2]+1 };
  int topnorthwest[] = { center[0]-1, center[1]+1, center[2]+1 };
  int topnortheast[] = { center[0]+1, center[1]+1, center[2]+1 };

  MPISystemID mpisystem = theMPISystem();

  MPI_Comm cartcomm = mpisystem->getComm();

  int rank = mpisystem->getRank();

  // Specify local domain
  defineLocalDomain( intersect(
     intersect(
     HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
     HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ),
     HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
     HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
     HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
     HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz ) ) ) );

  // Connecting the west neighbor
  if( west[0] >= 0 ) {
     MPI_Cart_rank( cartcomm, west, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the east neighbor
  if( east[0] < px ) {
     MPI_Cart_rank( cartcomm, east, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the south neighbor
  if( south[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, south, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the north neighbor
  if( north[1] < py ) {
     MPI_Cart_rank( cartcomm, north, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the bottom neighbor
  if( bottom[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottom, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the top neighbor
  if( top[2] < pz ) {
     MPI_Cart_rank( cartcomm, top, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,0,+1), +(bz+top[2]*dz   ) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the south-west neighbor
  if( southwest[0] >= 0 && southwest[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, southwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
        HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }


  // Connecting the south-east neighbor
  if( southeast[0] < px && southeast[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, southeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the north-west neighbor
  if( northwest[0] >= 0 && northwest[1] < py ) {
     MPI_Cart_rank( cartcomm, northwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
        HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the north-east neighbor
  if( northeast[0] < px && northeast[1] < py ) {
     MPI_Cart_rank( cartcomm, northeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the bottom-west neighbor
  if( bottomwest[0] >= 0 && bottomwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
        HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the bottom-east neighbor
  if( bottomeast[0] < px && bottomeast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the bottom-south neighbor
  if( bottomsouth[1] >= 0 && bottomsouth[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsouth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
        HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ) );
  }

  // Connecting the bottom-north neighbor
  if( bottomnorth[1] < py && bottomnorth[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnorth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ) );
  }

  // Connecting the bottom-south-west neighbor
  if( bottomsouthwest[0] >= 0 && bottomsouthwest[1] >= 0 && bottomsouthwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ) );
  }

  // Connecting the bottom-south-east neighbor
  if( bottomsoutheast[0] < px && bottomsoutheast[1] >= 0 && bottomsoutheast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ) );
  }

  // Connecting the bottom-north-west neighbor
  if( bottomnorthwest[0] >= 0 && bottomnorthwest[1] < py && bottomnorthwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ) );
  }

  // Connecting the bottom-north-east neighbor
  if( bottomnortheast[0] < px && bottomnortheast[1] < py && bottomnortheast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ) );
  }

  // Connecting the top-west neighbor
  if( topwest[0] >= 0 && topwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
        HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the top-east neighbor
  if( topeast[0] < px && topeast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
        HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
        HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
        HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ) );
  }

  // Connecting the top-south neighbor
  if( topsouth[1] >= 0 && topsouth[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsouth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
        HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ) );
  }

  // Connecting the top-north neighbor
  if( topnorth[1] < py && topnorth[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnorth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
        HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
        HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
        HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ) );
  }

  // Connecting the top-south-west neighbor
  if( topsouthwest[0] >= 0 && topsouthwest[1] >= 0 && topsouthwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsouthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the top-south-east neighbor
  if( topsoutheast[0] < px && topsoutheast[1] >= 0 && topsoutheast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsoutheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the top-north-west neighbor
  if( topnorthwest[0] >= 0 && topnorthwest[1] < py && topnorthwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnorthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ) );
  }

  // Connecting the top-north-east neighbor
  if( topnortheast[0] < px && topnortheast[1] < py && topnortheast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnortheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0), +(bx+east[0]*dx ) ),
                               HalfSpace( Vec3(0,1,0), +(by+north[1]*dy) ),
                               HalfSpace( Vec3(0,0,1), +(bz+top[2]*dz  ) ) ) );
  }

//===========================================================================================================

}

//*************************************************************************************************
/*!\brief Domain decomposition function with full 3D periodic boundary conditions.
 *
 * \param center Array containing the 3D coordinates [x,y,z] of the current process in the MPI Cartesian grid
 * \param bx Lower bound of the global domain in X direction
 * \param by Lower bound of the global domain in Y direction  
 * \param bz Lower bound of the global domain in Z direction
 * \param dx Size of each domain cell in X direction
 * \param dy Size of each domain cell in Y direction
 * \param dz Size of each domain cell in Z direction
 * \param lx Total length of the global domain in X direction
 * \param ly Total length of the global domain in Y direction
 * \param lz Total length of the global domain in Z direction
 * \param px Total number of processes in X direction
 * \param py Total number of processes in Y direction
 * \param pz Total number of processes in Z direction
 * 
 * This function performs domain decomposition for MPI-based parallel simulations with periodic
 * boundary conditions in all three spatial dimensions. Periodic boundaries allow particles or
 * other simulation entities to wrap around from one side of the domain to the opposite side,
 * effectively creating an infinite periodic lattice.
 *
 * Key features of 3D periodic decomposition:
 * - Establishes wraparound connections at domain boundaries
 * - Handles neighbor connectivity across periodic boundaries using modular arithmetic
 * - Maintains consistent domain topology for particles crossing boundaries
 * - Supports complex interaction patterns in periodic systems
 *
 * The function uses the domain lengths (lx, ly, lz) to calculate proper periodic offsets
 * and establish connections to neighboring processes across periodic boundaries.
 *
 * \note Requires MPI support and proper MPI Cartesian communicator setup.
 * \note Use this function for simulations requiring periodic boundary conditions in all directions.
 */
void decomposePeriodic3D(int center[], 
                         real bx, real by, real bz, 
                         real dx, real dy, real dz, 
                         real lx, real ly, real lz, 
                         int px, int py, int pz) {

   int west           [] = { center[0]-1, center[1]  , center[2]   };
   int east           [] = { center[0]+1, center[1]  , center[2]   };
   int south          [] = { center[0]  , center[1]-1, center[2]   };
   int north          [] = { center[0]  , center[1]+1, center[2]   };
   int southwest      [] = { center[0]-1, center[1]-1, center[2]   };
   int southeast      [] = { center[0]+1, center[1]-1, center[2]   };
   int northwest      [] = { center[0]-1, center[1]+1, center[2]   };
   int northeast      [] = { center[0]+1, center[1]+1, center[2]   };

   int bottom         [] = { center[0]  , center[1]  , center[2]-1 };
   int bottomwest     [] = { center[0]-1, center[1]  , center[2]-1 };
   int bottomeast     [] = { center[0]+1, center[1]  , center[2]-1 };
   int bottomsouth    [] = { center[0]  , center[1]-1, center[2]-1 };
   int bottomnorth    [] = { center[0]  , center[1]+1, center[2]-1 };
   int bottomsouthwest[] = { center[0]-1, center[1]-1, center[2]-1 };
   int bottomsoutheast[] = { center[0]+1, center[1]-1, center[2]-1 };
   int bottomnorthwest[] = { center[0]-1, center[1]+1, center[2]-1 };
   int bottomnortheast[] = { center[0]+1, center[1]+1, center[2]-1 };

   int top            [] = { center[0]  , center[1]  , center[2]+1 };
   int topwest        [] = { center[0]-1, center[1]  , center[2]+1 };
   int topeast        [] = { center[0]+1, center[1]  , center[2]+1 };
   int topsouth       [] = { center[0]  , center[1]-1, center[2]+1 };
   int topnorth       [] = { center[0]  , center[1]+1, center[2]+1 };
   int topsouthwest   [] = { center[0]-1, center[1]-1, center[2]+1 };
   int topsoutheast   [] = { center[0]+1, center[1]-1, center[2]+1 };
   int topnorthwest   [] = { center[0]-1, center[1]+1, center[2]+1 };
   int topnortheast   [] = { center[0]+1, center[1]+1, center[2]+1 };

   MPISystemID mpisystem = theMPISystem();

   MPI_Comm cartcomm = mpisystem->getComm();

   int rank = mpisystem->getRank();

   // Specify local domain
   defineLocalDomain( intersect(
      intersect(
      HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
      HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ),
      HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
      HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
      HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
      HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );

   // Connecting the west neighbor
   {
      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the east neighbor
   {
      const Vec3 offset( ( ( east[0]==px )?( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the south neighbor
   {
      const Vec3 offset( 0, ( ( south[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, south, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north neighbor
   {
      const Vec3 offset( 0, ( ( north[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, north, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the bottom neighbor
   {
      const Vec3 offset( 0, 0, ( ( bottom[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottom, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top neighbor
   {
      const Vec3 offset( 0, 0, ( ( top[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, top, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,+1), +top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the south-west neighbor
   {
      const Vec3 offset( ( ( southwest[0]<0 )?( lx ):( 0 ) ), ( ( southwest[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, southwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the south-east neighbor
   {
      const Vec3 offset( ( ( southeast[0]==px )?( -lx ):( 0 ) ), ( ( southeast[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, southeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north-west neighbor
   //if( northwest[0] >= 0 && northwest[1] < py ) {
   {
      const Vec3 offset( ( ( northwest[0]<0 )?( lx ):( 0 ) ), ( ( northwest[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, northwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north-east neighbor
   //if( northeast[0] < px && northeast[1] < py ) {
   {
      const Vec3 offset( ( ( northeast[0]==px )?( -lx ):( 0 ) ), ( ( northeast[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, northeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the bottom-west neighbor
   //if( bottomwest[0] >= 0 && bottomwest[2] >= 0 ) {
   {
      const Vec3 offset( ( ( bottomwest[0]<0 )?( lx ):( 0 ) ), 0, ( ( bottomwest[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the bottom-east neighbor
   {
      const Vec3 offset( ( ( bottomeast[0]==px )?( -lx ):( 0 ) ), 0, ( ( bottomeast[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the bottom-south neighbor
   //if( bottomsouth[1] >= 0 && bottomsouth[2] >= 0 ) {
   {
      const Vec3 offset( 0, ( ( bottomsouth[1]<0 )?( ly ):( 0 ) ), ( ( bottomsouth[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the bottom-north neighbor
   //if( bottomnorth[1] < py && bottomnorth[2] >= 0 ) {
   {
      const Vec3 offset( 0, ( ( bottomnorth[1]==py )?(-ly ):( 0 ) ), ( ( bottomnorth[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the bottom-south-west neighbor
   //if( bottomsouthwest[0] >= 0 && bottomsouthwest[1] >= 0 && bottomsouthwest[2] >= 0 ) {
   {
      const Vec3 offset( ( ( bottomsouthwest[0]<0 )?( lx ):( 0 ) ), ( ( bottomsouthwest[1]<0 )?( ly ):( 0 ) ), ( ( bottomsouthwest[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-south-east neighbor
   //if( bottomsoutheast[0] < px && bottomsoutheast[1] >= 0 && bottomsoutheast[2] >= 0 ) {
   {
      const Vec3 offset( ( ( bottomsoutheast[0]==px )?( -lx ):( 0 ) ), ( ( bottomsoutheast[1]<0 )?( ly ):( 0 ) ), ( ( bottomsoutheast[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-north-west neighbor
   //if( bottomnorthwest[0] >= 0 && bottomnorthwest[1] < py && bottomnorthwest[2] >= 0 ) {
   {
      const Vec3 offset( ( ( bottomnorthwest[0]<0 )?( lx ):( 0 ) ), ( ( bottomnorthwest[1]==py )?(-ly ):( 0 ) ), ( ( bottomnorthwest[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-north-east neighbor
   //if( bottomnortheast[0] < px && bottomnortheast[1] < py && bottomnortheast[2] >= 0 ) {
   {
      const Vec3 offset( ( ( bottomnortheast[0]==px )?( -lx ):( 0 ) ), ( ( bottomnortheast[1]==py )?(-ly ):( 0 ) ), ( ( bottomnortheast[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the top-west neighbor
   {
      const Vec3 offset( ( ( topwest[0]<0 )?( lx ):( 0 ) ), 0, ( ( topwest[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top-east neighbor
   {
      const Vec3 offset( ( ( topeast[0]==px )?( -lx ):( 0 ) ), 0, ( ( topeast[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(1,0,0), east[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top-south neighbor
   //if( topsouth[1] >= 0 && topsouth[2] < pz ) {
   {
      const Vec3 offset( 0, ( ( topsouth[1]<0 )?( ly ):( 0 ) ), ( ( topsouth[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the top-north neighbor
   //if( topnorth[1] < py && topnorth[2] < pz ) {
   {
      const Vec3 offset( 0, ( ( topnorth[1]==py )?(-ly ):( 0 ) ), ( ( topnorth[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,1,0), north[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the top-south-west neighbor
   //if( topsouthwest[0] >= 0 && topsouthwest[1] >= 0 && topsouthwest[2] < pz ) {
   {
      const Vec3 offset( ( ( topsouthwest[0]<0 )?( lx ):( 0 ) ), ( ( topsouthwest[1]==py )?(-ly ):( 0 ) ), ( ( topsouthwest[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-south-east neighbor
   //if( topsoutheast[0] < px && topsoutheast[1] >= 0 && topsoutheast[2] < pz ) {
   {
      const Vec3 offset( ( ( topsoutheast[0]==px )?( -lx ):( 0 ) ), ( ( topsoutheast[1]<0 )?( ly ):( 0 ) ), ( ( topsoutheast[2]==pz )?( -lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-north-west neighbor
   //if( topnorthwest[0] >= 0 && topnorthwest[1] < py && topnorthwest[2] < pz ) {
   {
      const Vec3 offset( ( ( topnorthwest[0]<0 )?( lx ):( 0 ) ), ( ( topnorthwest[1]==py )?(-ly ):( 0 ) ), ( ( topnorthwest[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-north-east neighbor
   //if( topnortheast[0] < px && topnortheast[1] < py && topnortheast[2] < pz ) {
   {
      const Vec3 offset( ( ( topnortheast[0]==px )?( -lx ):( 0 ) ), ( ( topnortheast[1]==py )?(-ly ):( 0 ) ), ( ( topnortheast[2]<0 )?( lz ):( 0 ) ) );
      MPI_Cart_rank( cartcomm, topnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }
}

//*************************************************************************************************
/*!\brief Domain decomposition function with periodic boundary conditions in XY plane only.
 *
 * \param center Array containing the 3D coordinates [x,y,z] of the current process in the MPI Cartesian grid
 * \param bx Lower bound of the global domain in X direction
 * \param by Lower bound of the global domain in Y direction  
 * \param bz Lower bound of the global domain in Z direction
 * \param dx Size of each domain cell in X direction
 * \param dy Size of each domain cell in Y direction
 * \param dz Size of each domain cell in Z direction
 * \param lx Total length of the global domain in X direction
 * \param ly Total length of the global domain in Y direction
 * \param lz Total length of the global domain in Z direction (used for verification)
 * \param px Total number of processes in X direction
 * \param py Total number of processes in Y direction
 * \param pz Total number of processes in Z direction
 * 
 * This function performs domain decomposition for MPI-based parallel simulations with periodic
 * boundary conditions in the XY plane and non-periodic (fixed) boundaries in the Z direction.
 * This configuration is common in simulations of:
 * 
 * - Thin films or layers where Z-direction has finite thickness
 * - 2D-like systems with limited Z-extent
 * - Systems with walls or boundaries in the Z direction but periodic in XY
 *
 * Key features of XY-periodic decomposition:
 * - X and Y directions: Periodic wraparound connections at boundaries
 * - Z direction: Standard non-periodic neighbor connections
 * - Proper handling of corner and edge neighbors with mixed boundary types
 * - Maintains domain continuity for particles crossing XY boundaries
 *
 * The function establishes periodic connections in X and Y directions using the domain lengths
 * (lx, ly) while treating Z as a standard non-periodic direction.
 *
 * \note Requires MPI support and proper MPI Cartesian communicator setup.
 * \note Use this function for simulations requiring periodic boundaries only in XY plane.
 */
void decomposePeriodicXY3D(int center[], 
                           real bx, real by, real bz, 
                           real dx, real dy, real dz, 
                           real lx, real ly, real lz, 
                           int px, int py, int pz) {

   int west           [] = { center[0]-1, center[1]  , center[2]   };
   int east           [] = { center[0]+1, center[1]  , center[2]   };
   int south          [] = { center[0]  , center[1]-1, center[2]   };
   int north          [] = { center[0]  , center[1]+1, center[2]   };
   int southwest      [] = { center[0]-1, center[1]-1, center[2]   };
   int southeast      [] = { center[0]+1, center[1]-1, center[2]   };
   int northwest      [] = { center[0]-1, center[1]+1, center[2]   };
   int northeast      [] = { center[0]+1, center[1]+1, center[2]   };

   int bottom         [] = { center[0]  , center[1]  , center[2]-1 };
   int bottomwest     [] = { center[0]-1, center[1]  , center[2]-1 };
   int bottomeast     [] = { center[0]+1, center[1]  , center[2]-1 };
   int bottomsouth    [] = { center[0]  , center[1]-1, center[2]-1 };
   int bottomnorth    [] = { center[0]  , center[1]+1, center[2]-1 };
   int bottomsouthwest[] = { center[0]-1, center[1]-1, center[2]-1 };
   int bottomsoutheast[] = { center[0]+1, center[1]-1, center[2]-1 };
   int bottomnorthwest[] = { center[0]-1, center[1]+1, center[2]-1 };
   int bottomnortheast[] = { center[0]+1, center[1]+1, center[2]-1 };

   int top            [] = { center[0]  , center[1]  , center[2]+1 };
   int topwest        [] = { center[0]-1, center[1]  , center[2]+1 };
   int topeast        [] = { center[0]+1, center[1]  , center[2]+1 };
   int topsouth       [] = { center[0]  , center[1]-1, center[2]+1 };
   int topnorth       [] = { center[0]  , center[1]+1, center[2]+1 };
   int topsouthwest   [] = { center[0]-1, center[1]-1, center[2]+1 };
   int topsoutheast   [] = { center[0]+1, center[1]-1, center[2]+1 };
   int topnorthwest   [] = { center[0]-1, center[1]+1, center[2]+1 };
   int topnortheast   [] = { center[0]+1, center[1]+1, center[2]+1 };

   MPISystemID mpisystem = theMPISystem();

   MPI_Comm cartcomm = mpisystem->getComm();

   int rank = mpisystem->getRank();

   // Specify local domain
   defineLocalDomain( intersect(
      intersect(
      HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
      HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ),
      HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
      HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
      HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
      HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );

   // Connecting the west neighbor
   {
      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the east neighbor
   {
      const Vec3 offset( ( ( east[0]==px )?( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the south neighbor
   {
      const Vec3 offset( 0, ( ( south[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, south, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north neighbor
   {
      const Vec3 offset( 0, ( ( north[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, north, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the bottom neighbor - skip if outside boundary (no periodicity in z)
   if (bottom[2] >= 0) {
      const Vec3 offset( 0, 0, 0 ); // No z offset as we're not periodic in z
      MPI_Cart_rank( cartcomm, bottom, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top neighbor - skip if outside boundary (no periodicity in z)
   if (top[2] < pz) {
      const Vec3 offset( 0, 0, 0 ); // No z offset as we're not periodic in z
      MPI_Cart_rank( cartcomm, top, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,+1), +top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the south-west neighbor
   {
      const Vec3 offset( ( ( southwest[0]<0 )?( lx ):( 0 ) ), ( ( southwest[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, southwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the south-east neighbor
   {
      const Vec3 offset( ( ( southeast[0]==px )?( -lx ):( 0 ) ), ( ( southeast[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, southeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north-west neighbor
   {
      const Vec3 offset( ( ( northwest[0]<0 )?( lx ):( 0 ) ), ( ( northwest[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, northwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the north-east neighbor
   {
      const Vec3 offset( ( ( northeast[0]==px )?( -lx ):( 0 ) ), ( ( northeast[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, northeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ), offset );
   }

   // Connecting the bottom-west neighbor - skip if outside z boundary
   if (bottomwest[2] >= 0) {
      const Vec3 offset( ( ( bottomwest[0]<0 )?( lx ):( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the bottom-east neighbor - skip if outside z boundary
   if (bottomeast[2] >= 0) {
      const Vec3 offset( ( ( bottomeast[0]==px )?( -lx ):( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the bottom-south neighbor - skip if outside z boundary
   if (bottomsouth[2] >= 0) {
      const Vec3 offset( 0, ( ( bottomsouth[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the bottom-north neighbor - skip if outside z boundary
   if (bottomnorth[2] >= 0) {
      const Vec3 offset( 0, ( ( bottomnorth[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the bottom-south-west neighbor - skip if outside z boundary
   if (bottomsouthwest[2] >= 0) {
      const Vec3 offset( ( ( bottomsouthwest[0]<0 )?( lx ):( 0 ) ), ( ( bottomsouthwest[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-south-east neighbor - skip if outside z boundary
   if (bottomsoutheast[2] >= 0) {
      const Vec3 offset( ( ( bottomsoutheast[0]==px )?( -lx ):( 0 ) ), ( ( bottomsoutheast[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-north-west neighbor - skip if outside z boundary
   if (bottomnorthwest[2] >= 0) {
      const Vec3 offset( ( ( bottomnorthwest[0]<0 )?( lx ):( 0 ) ), ( ( bottomnorthwest[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the bottom-north-east neighbor - skip if outside z boundary
   if (bottomnortheast[2] >= 0) {
      const Vec3 offset( ( ( bottomnortheast[0]==px )?( -lx ):( 0 ) ), ( ( bottomnortheast[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ), offset );
   }

   // Connecting the top-west neighbor - skip if outside z boundary
   if (topwest[2] < pz) {
      const Vec3 offset( ( ( topwest[0]<0 )?( lx ):( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top-east neighbor - skip if outside z boundary
   if (topeast[2] < pz) {
      const Vec3 offset( ( ( topeast[0]==px )?( -lx ):( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(1,0,0), east[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ), offset );
   }

   // Connecting the top-south neighbor - skip if outside z boundary
   if (topsouth[2] < pz) {
      const Vec3 offset( 0, ( ( topsouth[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the top-north neighbor - skip if outside z boundary
   if (topnorth[2] < pz) {
      const Vec3 offset( 0, ( ( topnorth[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,1,0), north[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ), offset );
   }

   // Connecting the top-south-west neighbor - skip if outside z boundary
   if (topsouthwest[2] < pz) {
      const Vec3 offset( ( ( topsouthwest[0]<0 )?( lx ):( 0 ) ), ( ( topsouthwest[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-south-east neighbor - skip if outside z boundary
   if (topsoutheast[2] < pz) {
      const Vec3 offset( ( ( topsoutheast[0]==px )?( -lx ):( 0 ) ), ( ( topsoutheast[1]<0 )?( ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-north-west neighbor - skip if outside z boundary
   if (topnorthwest[2] < pz) {
      const Vec3 offset( ( ( topnorthwest[0]<0 )?( lx ):( 0 ) ), ( ( topnorthwest[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }

   // Connecting the top-north-east neighbor - skip if outside z boundary
   if (topnortheast[2] < pz) {
      const Vec3 offset( ( ( topnortheast[0]==px )?( -lx ):( 0 ) ), ( ( topnortheast[1]==py )?(-ly ):( 0 ) ), 0 );
      MPI_Cart_rank( cartcomm, topnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ), offset );
   }
}

//*************************************************************************************************
/*!\brief Specialized 2D domain decomposition for Archimedes screw geometry.
 *
 * \param center Array containing the 2D coordinates [x,y] of the current process in the MPI Cartesian grid
 * \param cartcomm MPI Cartesian communicator for the process topology
 * \param halfSpaces Vector of HalfSpace objects defining the domain boundaries in X direction
 * \param halfSpacesY Vector of HalfSpace objects defining the domain boundaries in Y direction
 * \param px Total number of processes in X direction
 * \param py Total number of processes in Y direction
 * \param pz Total number of processes in Z direction (typically 1 for 2D)
 * 
 * This function performs domain decomposition for simulations involving Archimedes screw
 * geometries or other complex 2D shapes that cannot be adequately represented by simple
 * rectangular domains. Unlike the standard decomposition functions that use regular grid
 * patterns, this function accepts custom HalfSpace definitions to accommodate irregular
 * domain shapes.
 *
 * Key features of Archimedes-specific decomposition:
 * - Uses custom HalfSpace vectors to define irregular domain boundaries
 * - Supports complex geometries like spirals, curved boundaries, or arbitrary 2D shapes
 * - Separate handling of X and Y direction boundaries through distinct HalfSpace arrays
 * - 2D decomposition pattern suitable for thin or planar geometries
 * - Handles boundary conditions specific to Archimedes screw mechanics
 *
 * The function establishes domain boundaries based on the provided HalfSpace geometries
 * rather than regular rectangular grids, making it suitable for specialized applications
 * requiring complex domain shapes.
 *
 * \note This function is specifically designed for Archimedes screw simulations and may
 *       not be suitable for general-purpose domain decomposition.
 * \note Requires MPI support and pre-computed HalfSpace boundary definitions.
 * \note The HalfSpace vectors must be properly sized and ordered for the given process grid.
 */
void decomposeDomain2DArchimedes(int center[], MPI_Comm cartcomm, 
                                const std::vector<HalfSpace>& halfSpaces,
                                const std::vector<HalfSpace>& halfSpacesY,
                                int px, int py, int pz) {
   
   int west[] = {center[0] - 1, center[1]};
   int east[] = {center[0] + 1, center[1]};

   int south[] = {center[0], center[1] - 1};
   int north[] = {center[0], center[1] + 1};

   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };

   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   std::vector<HalfSpace> mySpaces;
   int rank;

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
   if (east[0] < px)
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
   if( north[1] < pz ) {
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
   if( southeast[0] < px && southeast[1] >= 0 ) {
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
   if( northwest[0] >= 0 && northwest[1] < pz ) {
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
   // Connecting the north-east neighbor
   if( northeast[0] < px && northeast[1] < pz ) {
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
}

//*************************************************************************************************
/*!\brief Domain decomposition with periodic boundaries in X direction only.
 *
 * \param center Array containing the 3D coordinates [x,y,z] of the current process in the MPI Cartesian grid
 * \param bx Lower bound of the global domain in X direction
 * \param by Lower bound of the global domain in Y direction  
 * \param bz Lower bound of the global domain in Z direction
 * \param dx Size of each domain cell in X direction
 * \param dy Size of each domain cell in Y direction
 * \param dz Size of each domain cell in Z direction
 * \param lx Total length of the global domain in X direction
 * \param ly Total length of the global domain in Y direction (used for verification)
 * \param lz Total length of the global domain in Z direction (used for verification)
 * \param px Total number of processes in X direction
 * \param py Total number of processes in Y direction
 * \param pz Total number of processes in Z direction
 * 
 * This function performs domain decomposition for simulations with periodic boundary conditions
 * in the X direction, and non-periodic (fixed) boundaries in the Y and Z directions. This is
 * useful for modeling systems that are continuous or repeating along one primary axis while
 * being confined in the other two. For example:
 * 
 * - Simulations of extrusion or continuous casting processes.
 * - Systems under shear in one direction.
 * - Modeling long structures confined within a channel.
 *
 * Key features of X-periodic decomposition:
 * - X direction: Periodic wraparound connections at boundaries.
 * - Y and Z directions: Standard non-periodic neighbor connections.
 * - Handles all corner and edge neighbors with the appropriate mixed boundary logic.
 *
 * The function establishes periodic connections in the X direction using the domain length (lx)
 * while treating Y and Z as standard non-periodic directions.
 *
 * \note Requires MPI support and proper MPI Cartesian communicator setup.
 */
void decomposePeriodicX3D(int center[], 
                          real bx, real by, real bz, 
                          real dx, real dy, real dz, 
                          real lx, real ly, real lz, 
                          int px, int py, int pz) {

   int west           [] = { center[0]-1, center[1]  , center[2]   };
   int east           [] = { center[0]+1, center[1]  , center[2]   };
   int south          [] = { center[0]  , center[1]-1, center[2]   };
   int north          [] = { center[0]  , center[1]+1, center[2]   };
   int southwest      [] = { center[0]-1, center[1]-1, center[2]   };
   int southeast      [] = { center[0]+1, center[1]-1, center[2]   };
   int northwest      [] = { center[0]-1, center[1]+1, center[2]   };
   int northeast      [] = { center[0]+1, center[1]+1, center[2]   };

   int bottom         [] = { center[0]  , center[1]  , center[2]-1 };
   int bottomwest     [] = { center[0]-1, center[1]  , center[2]-1 };
   int bottomeast     [] = { center[0]+1, center[1]  , center[2]-1 };
   int bottomsouth    [] = { center[0]  , center[1]-1, center[2]-1 };
   int bottomnorth    [] = { center[0]  , center[1]+1, center[2]-1 };
   int bottomsouthwest[] = { center[0]-1, center[1]-1, center[2]-1 };
   int bottomsoutheast[] = { center[0]+1, center[1]-1, center[2]-1 };
   int bottomnorthwest[] = { center[0]-1, center[1]+1, center[2]-1 };
   int bottomnortheast[] = { center[0]+1, center[1]+1, center[2]-1 };

   int top            [] = { center[0]  , center[1]  , center[2]+1 };
   int topwest        [] = { center[0]-1, center[1]  , center[2]+1 };
   int topeast        [] = { center[0]+1, center[1]  , center[2]+1 };
   int topsouth       [] = { center[0]  , center[1]-1, center[2]+1 };
   int topnorth       [] = { center[0]  , center[1]+1, center[2]+1 };
   int topsouthwest   [] = { center[0]-1, center[1]-1, center[2]+1 };
   int topsoutheast   [] = { center[0]+1, center[1]-1, center[2]+1 };
   int topnorthwest   [] = { center[0]-1, center[1]+1, center[2]+1 };
   int topnortheast   [] = { center[0]+1, center[1]+1, center[2]+1 };

   MPISystemID mpisystem = theMPISystem();
   MPI_Comm cartcomm = mpisystem->getComm();
   int rank = mpisystem->getRank();

   // Specify local domain. The definitions are based on absolute coordinates.
   // Periodicity is handled by offsets in the connect() calls.
   defineLocalDomain( intersect(
      intersect(
      HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
      HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ),
      HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
      HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
      HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
      HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ) );

   // Connecting the west neighbor (Periodic)
   {
      const Vec3 offset( ( ( west[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the east neighbor (Periodic)
   {
      const Vec3 offset( ( ( east[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the south neighbor (Non-periodic)
   if( south[1] >= 0 ) {
      const Vec3 offset(0, 0, 0);
      MPI_Cart_rank( cartcomm, south, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the north neighbor (Non-periodic)
   if( north[1] < py ) {
      const Vec3 offset(0, 0, 0);
      MPI_Cart_rank( cartcomm, north, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the bottom neighbor (Non-periodic)
   if( bottom[2] >= 0 ) {
      const Vec3 offset(0, 0, 0);
      MPI_Cart_rank( cartcomm, bottom, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting the top neighbor (Non-periodic)
   if( top[2] < pz ) {
      const Vec3 offset(0, 0, 0);
      MPI_Cart_rank( cartcomm, top, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,+1), +(bz+top[2]*dz   ) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting the south-west neighbor (Mixed: Periodic X, Non-periodic Y)
   if( southwest[1] >= 0 ) {
      const Vec3 offset( ( ( southwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, southwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
         HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the south-east neighbor (Mixed: Periodic X, Non-periodic Y)
   if( southeast[1] >= 0 ) {
      const Vec3 offset( ( ( southeast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, southeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the north-west neighbor (Mixed: Periodic X, Non-periodic Y)
   if( northwest[1] < py ) {
      const Vec3 offset( ( ( northwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, northwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
         HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the north-east neighbor (Mixed: Periodic X, Non-periodic Y)
   if( northeast[1] < py ) {
      const Vec3 offset( ( ( northeast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, northeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,+1), +(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,0,-1), -(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the bottom-west neighbor (Mixed: Periodic X, Non-periodic Z)
   if( bottomwest[2] >= 0 ) {
      const Vec3 offset( ( ( bottomwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
         HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting the bottom-east neighbor (Mixed: Periodic X, Non-periodic Z)
   if( bottomeast[2] >= 0 ) {
      const Vec3 offset( ( ( bottomeast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting the top-west neighbor (Mixed: Periodic X, Non-periodic Z)
   if( topwest[2] < pz ) {
      const Vec3 offset( ( ( topwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
         HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting the top-east neighbor (Mixed: Periodic X, Non-periodic Z)
   if( topeast[2] < pz ) {
      const Vec3 offset( ( ( topeast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
         HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
         HalfSpace( Vec3(0,+1,0), +(by+center[1]*dy) ),
         HalfSpace( Vec3(0,-1,0), -(by+north[1]*dy ) ) ), offset );
   }

   // Connecting bottom-south/north are covered by non-periodic Y and Z if's
   // Connecting the bottom-south neighbor
   if( bottomsouth[1] >= 0 && bottomsouth[2] >= 0 ) {
      const Vec3 offset( 0, 0, 0 );
      MPI_Cart_rank( cartcomm, bottomsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
         HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ), offset );
   }

   // Connecting the bottom-north neighbor
   if( bottomnorth[1] < py && bottomnorth[2] >= 0 ) {
      const Vec3 offset( 0, 0, 0 );
      MPI_Cart_rank( cartcomm, bottomnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ), offset );
   }

   // Connecting top-south/north are covered by non-periodic Y and Z if's
   // Connecting the top-south neighbor
   if( topsouth[1] >= 0 && topsouth[2] < pz ) {
      const Vec3 offset( 0, 0, 0 );
      MPI_Cart_rank( cartcomm, topsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
         HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ), offset );
   }

   // Connecting the top-north neighbor
   if( topnorth[1] < py && topnorth[2] < pz ) {
      const Vec3 offset( 0, 0, 0 );
      MPI_Cart_rank( cartcomm, topnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
         HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ),
         HalfSpace( Vec3(+1,0,0), +(bx+center[0]*dx) ),
         HalfSpace( Vec3(-1,0,0), -(bx+east[0]*dx  ) ) ), offset );
   }

   // Connecting the bottom-south-west neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( bottomsouthwest[1] >= 0 && bottomsouthwest[2] >= 0 ) {
      const Vec3 offset( ( ( bottomsouthwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ), offset );
   }

   // Connecting the bottom-south-east neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( bottomsoutheast[1] >= 0 && bottomsoutheast[2] >= 0 ) {
      const Vec3 offset( ( ( bottomsoutheast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ), offset );
   }

   // Connecting the bottom-north-west neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( bottomnorthwest[1] < py && bottomnorthwest[2] >= 0 ) {
      const Vec3 offset( ( ( bottomnorthwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ), offset );
   }

   // Connecting the bottom-north-east neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( bottomnortheast[1] < py && bottomnortheast[2] >= 0 ) {
      const Vec3 offset( ( ( bottomnortheast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,-1), -(bz+center[2]*dz) ) ), offset );
   }

   // Connecting the top-south-west neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( topsouthwest[1] >= 0 && topsouthwest[2] < pz ) {
      const Vec3 offset( ( ( topsouthwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the top-south-east neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( topsoutheast[1] >= 0 && topsoutheast[2] < pz ) {
      const Vec3 offset( ( ( topsoutheast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0),  +(bx+east[0]*dx  ) ),
                               HalfSpace( Vec3(0,-1,0), -(by+center[1]*dy) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the top-north-west neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( topnorthwest[1] < py && topnorthwest[2] < pz ) {
      const Vec3 offset( ( ( topnorthwest[0] < 0 ) ? ( lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -(bx+center[0]*dx) ),
                               HalfSpace( Vec3(0,1,0),  +(by+north[1]*dy ) ),
                               HalfSpace( Vec3(0,0,1),  +(bz+top[2]*dz   ) ) ), offset );
   }

   // Connecting the top-north-east neighbor (Mixed: Periodic X, Non-periodic YZ)
   if( topnortheast[1] < py && topnortheast[2] < pz ) {
      const Vec3 offset( ( ( topnortheast[0] == px ) ? ( -lx ) : ( 0 ) ), 0, 0 );
      MPI_Cart_rank( cartcomm, topnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), +(bx+east[0]*dx ) ),
                               HalfSpace( Vec3(0,1,0), +(by+north[1]*dy) ),
                               HalfSpace( Vec3(0,0,1), +(bz+top[2]*dz  ) ) ), offset );
   }
}

}


#endif
