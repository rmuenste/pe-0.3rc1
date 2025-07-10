
#include <pe/interface/decompose.h>

void setupDraftKissTumbBench(MPI_Comm ex0) {

  auto& config = SimulationConfig::getInstance();
  world = theWorld();
  
  loadSimulationConfig("example.json");
  
  world->setGravity( config.getGravity() );
  world->setLiquidSolid(true);
  world->setLiquidDensity( config.getFluidDensity() );
  world->setViscosity( config.getFluidViscosity() );
  world->setDamping( 1.0 );

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real dx( 2.00 );
  const real dy( 2.00 );
  const real dz( 8.0 / config.getProcessesZ() );

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( config.getProcessesX()*config.getProcessesY()*config.getProcessesZ() != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << config.getProcessesX()*config.getProcessesY()*config.getProcessesZ() << "\n\n" << std::endl;
     std::exit(EXIT_FAILURE);
  }

  /////////////////////////////////////////////////////
  // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

  // Computing the Cartesian coordinates of the neighboring processes
  int dims   [] = { config.getProcessesX(), config.getProcessesY(), config.getProcessesZ() };
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
  int px = config.getProcessesX();
  int py = config.getProcessesY();
  int pz = config.getProcessesZ();

  real bx = 0.0;
  real by = 0.0;
  real bz = 0.0;

  // Setup domain decomposition using decomposeDomain function for non-periodic 3D
  decomposeDomain(center, bx, by, bz, dx, dy, dz, px, py, pz);

//===========================================================================================================


//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  std::cout << "[Creating a plane] " << std::endl;
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );
     createPlane( 1778,+1.0, 0.0, 0.0, 0, granite, false ); // right border
     createPlane( 1779,-1.0, 0.0, 0.0,-2.0, granite, false ); // left border
 
     createPlane( 1780, 0.0, 1.0, 0.0, 0, granite, false ); // back border
     createPlane( 1781, 0.0,-1.0, 0.0,-2, granite, false ); // front border

  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
  }

//  if(g_povray) {
//    povraySetup();
//  }

  const real lx = px * dx;
  const real ly = py * dy;
  const real lz = pz * dz;

  const int nx = 1;//  lx / space;
  const int ny = 1;//  ly / space;
  const int nz = 1;// (lz / space);


  pe_EXCLUSIVE_SECTION(0) {

    std::cout << "nx, ny, nz: " << (Vec3(nx, ny, nz)) << std::endl;

  }

  int idx = 0;

  // Create a custom material for the benchmark
  MaterialID myMaterial = createMaterial("Bench", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  Vec3 position( 0.99,  1.0, 6.9);
  
  real radBench = config.getBenchRadius();
  if (world->ownsPoint( position )) {
    SphereID spear = createSphere(idx, position, radBench, myMaterial, true);
    std::cout << "[Creating particle] at: " << (position) << " in domain: " << my_rank << std::endl;
    std::cout << "[particle mass]: " << spear->getMass()  << std::endl;
    std::cout << "[particle volume]: " << real(4.0)/real(3.0) * M_PI * radBench * radBench * radBench << std::endl;
    ++idx;
  }

  position = Vec3( 1.0,  1.0, 7.2);
  if (world->ownsPoint( position )) {
    SphereID spear = createSphere(idx, position, radBench, myMaterial, true);
    std::cout << "[Creating particle] at: " << (position) << " in domain: " << my_rank << std::endl;
    std::cout << "[particle mass]: " << spear->getMass()  << std::endl;
    std::cout << "[particle volume]: " << real(4.0)/real(3.0) * M_PI * radBench * radBench * radBench << std::endl;
    ++idx;
  }

  // Synchronization of the MPI processes
  world->synchronize();

  // Calculating the total number of particles and primitives
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  unsigned long bla = idx;
  int numBodies =  theCollisionSystem()->getBodyStorage().size();
  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  MPI_Reduce( &bla, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  TimeStep::stepsize( config.getStepsize() );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " particles x              = " << nx << "\n" 
      << " particles y              = " << ny << "\n" 
      << " particles z              = " << nz << "\n" 
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   
}

