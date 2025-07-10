
#include <pe/interface/decompose.h>

void setupParticleBench(MPI_Comm ex0) {

  auto& config = SimulationConfig::getInstance();

  world = theWorld();

  loadSimulationConfig("example.json");
  //world->setGravity( 0.0, 0.0, -9.807e-6 ); //9.807×10^-6 m/ms^2 (meters per millisecond squared)
  // Vec3 userGravity = config.getGravity();
  //world->setGravity( 0.0, 0.0, -9.807 ); //9.807×10^-6 m/ms^2 (meters per millisecond squared)
  world->setGravity( config.getGravity() ); //9.807×10^-6 m/ms^2 (meters per millisecond squared)

  // Re 1.5 configuration
  //real simViscosity( 373e-3 );
  //real simRho( 970 );

  // Re 4.1 configuration
  //real simViscosity( 212e-3 );
  //real simRho( 965 );

  // Configuration from config singleton
  real simViscosity( config.getFluidViscosity() );
  real simRho( config.getFluidDensity() );

  // New Benchmark Proposal
  //real simViscosity( 58e-3 );
  //real simRho( 1141.0 );

  real slipLength( 0.75 );
  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity( simViscosity );
  world->setDamping( 1.0 );
  bool useLubrication(false);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real dx( 0.05 );
  const real dy( 0.05 );
  //const real dz( 0.08 ) 2 subs;
  //const real dz( 0.08 ) 2 subs;
  //const real dx( -0.05 );
  //const real dy( -0.05 );
  const real dz( 0.16 / config.getProcessesZ() );

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( config.getProcessesX() * config.getProcessesY() * config.getProcessesZ() != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" 
               << config.getProcessesX() * config.getProcessesY() * config.getProcessesZ() << "\n\n" << std::endl;
     return;
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
    std::cout << Vec3(dims[0], dims[1], dims[2]) << std::endl;
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
    std::cout << "Rank:" << my_rank  << "->" << Vec3(center[0], center[1], center[2]) << std::endl;
  }

  // Setup domain decomposition using decomposePeriodic3D function
  const real lx = config.getPx() * dx;
  const real ly = config.getPy() * dy;
  const real lz = config.getPz() * dz;
  
  decomposeDomain(center, 0.0, 0.0, 0.0, dx, dy, dz, 
                  config.getPx(), config.getPy(), config.getPz());

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  //std::cout << "[Creating a plane] " << std::endl;
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );
  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
  }

  const int nx = 1;//  lx / space;
  const int ny = 1;//  ly / space;
  const int nz = 1;// (lz / space);


  pe_EXCLUSIVE_SECTION(0) {

    std::cout << "nx, ny, nz: " << Vec3(nx, ny, nz) << std::endl;

  }

  int idx = 0;

  //=========================================
  // New Bench configuration
  //real radBench = 0.011;
  //real rhoParticle( 1361.0 );
  // Vec3 position(-0.0, -0.0, 0.1571203);
  //MaterialID myMaterial = createMaterial("Bench", 1361.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  //=========================================
  
  real radBench = config.getBenchRadius();
  real rhoParticle( config.getParticleDensity() );
  Vec3 position(-0.0, -0.0, 0.1275);

  // Create a custom material for the benchmark
  theCollisionSystem()->setSlipLength(slipLength);
  MaterialID myMaterial = createMaterial("Bench", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  //==============================================================================================
  // Bench Configuration
  //==============================================================================================
  theCollisionSystem()->setMinEps(5e-6 / radBench);
  SphereID spear(nullptr);
  if (world->ownsPoint( position )) {
    spear = createSphere(idx, position, radBench, myMaterial, true);
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

  real sphereVol(0);
  real sphereMass(0);
  if (spear != nullptr) {
    sphereMass = spear->getMass();
    sphereVol = real(4.0)/real(3.0) * M_PI * radBench * radBench * radBench;
  }

  real totalMass(0);
  real totalVol(0);
  MPI_Reduce( &sphereMass, &totalMass, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &sphereVol, &totalVol, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm );

  TimeStep::stepsize( config.getStepsize() );
  std::string useLub = (useLubrication) ? "enabled" : "disabled";

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n" 
      << " Total number of MPI processes           = " << config.getPx() * config.getPy() * config.getPz() << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n" 
      << " Lubrication                             = " << useLub << "\n"
      << " Lubrication h_c                         = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Particle starting position              = " << position << "\n"  
      << " Particle mass                           = " << totalMass << "\n" 
      << " Particle volume                         = " << totalVol << "\n" 
      << " particles z                             = " << nz << "\n" << std::endl; 
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   
}

