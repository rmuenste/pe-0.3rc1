#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>

using namespace pe::povray;

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
  bool resume ( false );


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
//  std::cout << "[Creating a plane] " << std::endl;
  pe_GLOBAL_SECTION
  {
//     // Creating the ground plane
//     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );
//     createPlane( 1778,+1.0, 0.0, 0.0, 0, granite, false ); // right border
//     createPlane( 1779,-1.0, 0.0, 0.0,-2.0, granite, false ); // left border
// 
//     createPlane( 1780, 0.0, 1.0, 0.0, 0, granite, false ); // back border
//     createPlane( 1781, 0.0,-1.0, 0.0,-2, granite, false ); // front border
//
     createPlane( 20000, 0.0, 0.0, 1.0, 0.0, granite, false ); // bottom border
     createPlane( 20004, 0.0, 0.0,-1.0, -lz, granite, false ); // top border
  }

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

  //=========================================================================================
  BodyID particle;
  Vec3 gpos(0.02 , 0.02, 0.02);
  Vec3 vel(0.1, 0, 0.0);

  real radius2 = 0.01;
  MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  if( world->ownsPoint( gpos ) ) {
    particle = createSphere( idx, gpos, radius2, elastic );
    particle->setLinearVel( vel );
  }
  //=========================================================================================  

  pe_GLOBAL_SECTION
  {
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
  int numBodies =  theCollisionSystem()->getBodyStorage().size();
  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  MPI_Reduce( &bla, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  particlesTotal = primitivesTotal;

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
