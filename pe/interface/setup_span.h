
#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <pe/core/Types.h>

using namespace pe::povray;
//=================================================================================================


//=================================================================================================
// Setup for the Kroupa Case
//=================================================================================================
void setupSpan(MPI_Comm ex0) {

  world = theWorld();
  //world->setGravity( 0.0, 0.0,-980.665 );
  world->setGravity( 0.0, 0.0, 0.0 );

  real simRho( 0.85 );
  world->setLiquidDensity( simRho );

  // Particle Bench Config 
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication switch
  bool useLubrication(false);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  unsigned int id( 0 );              // User-specific ID counter

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
  int periods[] = { false, false, false };
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

  mpisystem->setComm(cartcomm);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
  }

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

//===================================================================================================

  real bx = 0.0;
  real by = 0.0;
  real bz = 0.0;

  const real dx( 2.0  / processesX );
  const real dy( 0.02 / processesY );
  const real dz( 0.2 );

  decomposeDomain(center, bx, by, bz, dx, dy, dz, px, py, pz);

//===================================================================================================

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
  }

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

  std::string fileName = std::string("span_cm.obj");

  //=========================================================================================  
  // Creation and positioning of the span
  //=========================================================================================  
  // Create a custom material for the span
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
  real spanDensity = 8.19;
  MaterialID spanMat = createMaterial("span"    , spanDensity , 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);

  //Vec3 spanPos = Vec3(0.23, 0.01, 0.0374807);
  Vec3 spanPos = Vec3(0.22, 0.01, 0.0374807);
  //Vec3 spanPos = Vec3(0.800079 0.00976448,0.0399377);
  TriangleMeshID span;

  if(!resume) {
    if(world->ownsPoint(spanPos)) {
      span = createTriangleMesh(++id, spanPos, fileName, spanMat, true, true, Vec3(1.0,1.0,1.0), false, false);
      //AABB &aabb = span->getAABB();
      std::cout << "Span x:[" << span->getAABB()[3] << "," << span->getAABB()[0] << "]" << std::endl;
    }
  }
  else {
    checkpointer.read( "../start.1" );
  }

  //=========================================================================================  
  pe_GLOBAL_SECTION
  {
     // Setup of the ground plane
     PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, -0.0, granite );
     // +y
     createPlane( id++, 0.0, 1.0, 0.0,  0.0, granite );
     // -y
     createPlane( id++, 0.0,-1.0, 0.0,  -0.02, granite );
  }
  //=========================================================================================  

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
      << " Total number of particles               = " << 1 << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Density                           = " << simRho << " [g/cm**3]"  << "\n"
      << " Span  Density                           = " << spanDensity << " [g/cm**3]"  << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n" 
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain volume                           = " << 2. * 0.2 * 0.02 << " [cm**3]" << "\n"
      << " Span volume                             = " << 0.000103797 << " [cm**3]" << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   

}

