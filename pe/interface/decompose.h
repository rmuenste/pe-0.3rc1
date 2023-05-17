#ifndef _PE_DECOMPOSE_H_
#define _PE_DECOMPOSE_H_

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
        HalfSpace( Vec3(-1,0,0), -(bz+center[0]*dx) ),
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

}


#endif