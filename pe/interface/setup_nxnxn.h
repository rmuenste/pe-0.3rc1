
void setup2x2x2(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  world->setLiquidSolid(true);
  world->setLiquidDensity(1.0);


  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real dx( 3.0 );
  const real dy( 3.0 );
  const real dz( 3.0 );

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( processesX*processesY*processesZ != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX*processesY*processesZ << "\n\n" << std::endl;
     return;
  }

  /////////////////////////////////////////////////////
  // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

  std::stringstream ss;
  ss << "Bladomain" << my_rank << ".txt";
  std::cout << ss.str() << std::endl;

//  DomainLog dm(ss.str());

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

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "3D communicator created" << std::endl;
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
  }

  //decomposeDomain(center, dx, dy, dz, px, py, pz);

  MPI_Barrier(cartcomm);

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif


  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
  }

  const real lx = px * dx - 0.5 - space;
  const real ly = py * dy - 0.5 - space;
  const real lz = pz * dz - 0.5 - space;

  const int nx = (lx / space);
  const int ny = (ly / space);
  const int nz = 14; //lz / space;


  MaterialID myMaterial = createMaterial("Dcav", 1.05, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_EXCLUSIVE_SECTION(0) {

    std::cout << "nx, ny, nz: " << (Vec3(nx, ny, nz)) << std::endl;
    std::cout << "space: " << space << std::endl;
    std::cout << "MaterialID Dcav: " << myMaterial << std::endl;

  }

  //space = 1.0;
  //real   space( real(2)*radius+spacing );
  //real pos[] = { radius+spacing, radius+spacing, radius+spacing};
  //real pos[] = { space, 5.0, 0.5 * space};
  real pos[] = { space + 0.25, space + 0.25, space + 0.15 };
  //real pos[] = { space, 5.0, 5.0};


  int idx = 0;
  bool resume = true;
  if(!resume) {
    for (int z = 0; z < nz; z++)
    {
      for (int y = 0; y < ny; y++)
      {
        for (int x = 0; x < nx; x++)
        {
          real ddx = x*space;
          real ddy = y*space;
          real ddz = z*space;
          //Vec3 position(pos[0] + 0.5 * radius + dx, pos[1] + 0.5 * radius + dy, pos[2] + 0.5 * radius + dz);
          Vec3 position(pos[0] + ddx, pos[1] + ddy, pos[2] + ddz);

          if (world->ownsPoint( position )) {
            SphereID spear = createSphere(idx, position, radius, myMaterial, true);
            ++idx;
          }
        }
      }
    }
  }
  else {
    //checkpointer.read( "../start.n3" );
  }

  std::cout << "[Creating a plane] " << std::endl;
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
     g_ground = createPlane( 20000, 0.0, 0.0, 1.0,-0.0, granite, true );
     createPlane( 20001,+1.0, 0.0, 0.0,-0.0, granite, false ); // right border
     createPlane( 20002,-1.0, 0.0, 0.0,-dx * px, granite, false ); // left border
 
     createPlane( 20002, 0.0, 1.0, 0.0,-0.0, granite, false ); // back border
     createPlane( 20003, 0.0,-1.0, 0.0,-dy * py, granite, false ); // front border

     createPlane( 20004, 0.0, 0.0,-1.0,-dz * pz, granite, false ); // top border
  }

  // Synchronization of the MPI processes
  world->synchronize();
  MPI_Barrier(cartcomm);

  // Calculating the total number of particles and primitives
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  unsigned long bla = idx;
  int numBodies =  theCollisionSystem()->getBodyStorage().size();
  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  MPI_Reduce( &bla, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  
  // Particles total = 
  // Volume of the domain
  real domainVol = dx * processesX * processesY * dy * processesZ * dz;
  real partTotal = primitivesTotal; 
  real partVol = real(4.0)/real(3.0) * M_PI * radius * radius * radius; 
  real totalPartVol = partTotal * partVol;
  real volFrac = (totalPartVol / domainVol) * 100.0;

  //=================================================================================================
  //
  //    INFO SECTION
  //
  //=================================================================================================
  pe_EXCLUSIVE_SECTION( 0 ) {
    if(volFrac < 15.0) {
        std::cerr << "\n Low volume fraction < 15% warning" << volFrac << std::endl;
    }
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Checkpoint used          = " << resume << "\n" 
      << " Domain size x            = " << px * dx << "\n" 
      << " Domain size y            = " << py * dy << "\n" 
      << " Domain size z            = " << pz * dz << "\n" 
      << " particles x              = " << nx << "\n" 
      << " particles y              = " << ny << "\n" 
      << " particles z              = " << nz << "\n" 
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Particle volume fraction                = " << volFrac << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  // Configuring the time step
  TimeStep::stepsize(stepsize);
  MPI_Barrier(cartcomm);
   
   
}
