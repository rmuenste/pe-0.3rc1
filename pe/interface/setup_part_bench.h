
void setupParticleBench(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, -9.81 );

  // TODO: enable this!!
  world->setLiquidSolid(true);
  world->setLiquidDensity(962);

  world->setDamping( 1.0 );

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real dx( 0.05 );
  const real dy( 0.05 );
  //const real dz( 0.08 ) 2 subs;
  //const real dz( 0.08 ) 2 subs;
//  const real dx( -0.05 );
//  const real dy( -0.05 );
  const real dz( 0.013333333 );

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( processesX*processesY*processesZ != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << processesX*processesY*processesZ << "\n\n" << std::endl;
     return;
  }

  /////////////////////////////////////////////////////
  // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

  //std::stringstream ss;
  //ss << "domain" << my_rank << ".txt";
  //std::cout << ss.str() << std::endl;


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


  // Specify local domain
  defineLocalDomain( intersect(
     intersect(
     HalfSpace( Vec3(+1,0,0), +(center[0] - 1)*dx ),
     HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ),
     HalfSpace( Vec3(0,+1,0), +(center[1] - 1)*dy ),
     HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
     HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
     HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );

  // Connecting the west neighbor
  if( west[0] >= 0 ) {
     MPI_Cart_rank( cartcomm, west, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the east neighbor
  if( east[0] < px ) {
     MPI_Cart_rank( cartcomm, east, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the south neighbor
  if( south[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, south, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the north neighbor
  if( north[1] < py ) {
     MPI_Cart_rank( cartcomm, north, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the bottom neighbor
  if( bottom[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottom, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +(center[0] - 1)*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +(center[1] - 1)*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the top neighbor
  if( top[2] < pz ) {
     MPI_Cart_rank( cartcomm, top, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,0,+1), +top[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +(center[0] - 1)*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +(center[1] - 1)*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the south-west neighbor
  if( southwest[0] >= 0 && southwest[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, southwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
        HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }


  // Connecting the south-east neighbor
  if( southeast[0] < px && southeast[1] >= 0 ) {
     MPI_Cart_rank( cartcomm, southeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
        HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the north-west neighbor
  if( northwest[0] >= 0 && northwest[1] < py ) {
     MPI_Cart_rank( cartcomm, northwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the north-east neighbor
  if( northeast[0] < px && northeast[1] < py ) {
     MPI_Cart_rank( cartcomm, northeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
        HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
        HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
        HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
  }

  // Connecting the bottom-west neighbor
  if( bottomwest[0] >= 0 && bottomwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
        HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the bottom-east neighbor
  if( bottomeast[0] < px && bottomeast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
        HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the bottom-south neighbor
  if( bottomsouth[1] >= 0 && bottomsouth[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsouth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
        HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
  }

  // Connecting the bottom-north neighbor
  if( bottomnorth[1] < py && bottomnorth[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnorth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
        HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
  }

  // Connecting the bottom-south-west neighbor
  if( bottomsouthwest[0] >= 0 && bottomsouthwest[1] >= 0 && bottomsouthwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                               HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                               HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
  }

  // Connecting the bottom-south-east neighbor
  if( bottomsoutheast[0] < px && bottomsoutheast[1] >= 0 && bottomsoutheast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                               HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                               HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
  }

  // Connecting the bottom-north-west neighbor
  if( bottomnorthwest[0] >= 0 && bottomnorthwest[1] < py && bottomnorthwest[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                               HalfSpace( Vec3(0,1,0), north[1]*dy ),
                               HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
  }

  // Connecting the bottom-north-east neighbor
  if( bottomnortheast[0] < px && bottomnortheast[1] < py && bottomnortheast[2] >= 0 ) {
     MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                               HalfSpace( Vec3(0,1,0), north[1]*dy ),
                               HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
  }

  // Connecting the top-west neighbor
  if( topwest[0] >= 0 && topwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topwest, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
        HalfSpace( Vec3(0,0,1), top[2]*dz ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the top-east neighbor
  if( topeast[0] < px && topeast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topeast, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(1,0,0), east[0]*dx ),
        HalfSpace( Vec3(0,0,1), top[2]*dz ),
        HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
        HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
  }

  // Connecting the top-south neighbor
  if( topsouth[1] >= 0 && topsouth[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsouth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
        HalfSpace( Vec3(0,0,1), top[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
  }

  // Connecting the top-north neighbor
  if( topnorth[1] < py && topnorth[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnorth, &rank );
     connect( rank, intersect(
        HalfSpace( Vec3(0,1,0), north[1]*dy ),
        HalfSpace( Vec3(0,0,1), top[2]*dz ),
        HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
        HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
  }

  // Connecting the top-south-west neighbor
  if( topsouthwest[0] >= 0 && topsouthwest[1] >= 0 && topsouthwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsouthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                               HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                               HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
  }

  // Connecting the top-south-east neighbor
  if( topsoutheast[0] < px && topsoutheast[1] >= 0 && topsoutheast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topsoutheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                               HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                               HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
  }

  // Connecting the top-north-west neighbor
  if( topnorthwest[0] >= 0 && topnorthwest[1] < py && topnorthwest[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnorthwest, &rank );
     connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                               HalfSpace( Vec3(0,1,0), north[1]*dy ),
                               HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
  }

  // Connecting the top-north-east neighbor
  if( topnortheast[0] < px && topnortheast[1] < py && topnortheast[2] < pz ) {
     MPI_Cart_rank( cartcomm, topnortheast, &rank );
     connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                               HalfSpace( Vec3(0,1,0), north[1]*dy ),
                               HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
  }

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  std::cout << "[Creating a plane] " << std::endl;
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
     g_ground = createPlane( 777, 0.0, 0.0, 1.0, 0, gr, true );
//     createPlane( 1778,+1.0, 0.0, 0.0, 0, granite, false ); // right border
//     createPlane( 1779,-1.0, 0.0, 0.0,-10.0, granite, false ); // left border
// 
//     createPlane( 1780, 0.0, 1.0, 0.0, 0, granite, false ); // back border
//     createPlane( 1781, 0.0,-1.0, 0.0,-10, granite, false ); // front border

  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
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

    std::cout << "nx, ny, nz: " << Vec3(nx, ny, nz) << std::endl;

  }

  int idx = 0;

  // Create a custom material for the benchmark
  MaterialID myMaterial = createMaterial("Bench", 1120.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  Vec3 position(-0.0, -0.0, 0.1275);
//  0.123597

  //==============================================================================================
  // Bench Configuration
  //==============================================================================================
  real radBench = 0.0075;
  if (world->ownsPoint( position )) {
    SphereID spear = createSphere(idx, position, radBench, myMaterial, true);
    std::cout << "[Creating particle] at: " << position << " in domain: " << my_rank << std::endl;
    std::cout << "[particle mass]: " << spear->getMass()  << std::endl;
    std::cout << "[particle volume]: " << real(4.0)/real(3.0) * M_PI * radius * radius * radius << std::endl;
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

