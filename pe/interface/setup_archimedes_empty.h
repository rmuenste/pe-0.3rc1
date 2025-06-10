#pragma once


using namespace pe::povray;

//=================================================================================================
// Setup for the Empty case
//=================================================================================================
void setupArchimedesEmpty(MPI_Comm ex0) {
  auto& config = SimulationConfig::getInstance();

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Re 1.5 configuration
  real simViscosity( 1.0e0 );
  real simRho( 1.0 );
  world->setViscosity( simViscosity );
  world->setLiquidDensity( simRho );

  // Particle Bench Config 
  real slipLength( 0.01 );
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication switch
  bool useLubrication(false);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real L( 45.0 );
  const real LY( 25.0 );
  const real LZ( 0.5 );
  const real dx( L/config.getProcessesX() );
  const real dy( LY/config.getProcessesY() );
  const real dz( LZ/config.getProcessesZ() );

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
  //int periods[] = { true, true, false };
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


  real bx = -10.0;
  real by = -5.0;
  real bz =  0.0;

  // Size of the domain
  const real lx( L );
  const real ly( LY);
  const real lz( LZ);

  decomposeDomain(center, bx, by, bz, 
                   dx, dy, dz, 
                   px, py, pz);

//===========================================================================================================


//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
  }


  // Create a custom material for the benchmark
  MaterialID myMaterial = createMaterial("Bench", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID elastic = createMaterial( "elastic", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
  //======================================================================================== 
  // The way we atm include lubrication by increasing contact threshold
  // has problems: the particles get distributed to more domain bc the threshold AABB
  // is much larger than the particle actually is.
  // We can even run into the "registering distant domain" error when the AABB of the 
  // particle is close in size to the size of a domain part!
  //======================================================================================== 
  theCollisionSystem()->setLubrication(useLubrication);
  theCollisionSystem()->setSlipLength(slipLength);
  theCollisionSystem()->setMinEps(0.01);
  theCollisionSystem()->setMaxIterations(200);

  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  bool resume               = false;
  real epsilon              = 2e-4;
  real targetVolumeFraction = 0.35;
  real radius2              = 0.005 - epsilon;

  int idx = 0;
  real h = 0.0075;
  
  //=========================================================================================
  BodyID particle;
  Vec3 gpos(0.05 , 0.05, 0.1 - radius2 - epsilon);
  Vec3 gpos2(0.05 , 0.05, gpos[2]);

  //=========================================================================================  
  //                   We have a heavy throwing ball
  //=========================================================================================  
  MaterialID heavy = createMaterial( "heavy", 10.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
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
      << " Total number of particles               = " << particlesTotal << "\n"
      << " particle volume                         = " << partVol << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n" 
      << " Lubrication                             = " << useLub << "\n"
      << " Lubrication h_c (slip length)           = " << slipLength << "\n"
      << " Lubrication threshold                   = " << lubricationThreshold << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain cube side length                 = " << L << "\n"
      << " Domain volume                           = " << L * L * L << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Volume fraction[%]                      = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   MPI_Barrier(cartcomm);
}