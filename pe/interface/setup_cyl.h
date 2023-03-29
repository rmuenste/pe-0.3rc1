
using namespace pe::povray;

void setupCyl(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0,-1.0 );

  world->setLiquidSolid(true);
  world->setLiquidDensity(1.0);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real dx( 0.875 );
  const real dy( 0.875 );
  const real dz( 3.2 );

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

//===========================================================================================================

  real bx =-0.875;
  real by =-0.875;
  real bz = 0.0;

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
//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  PlaneID ground(0);
//  std::cout << "[Creating a plane] " << std::endl;
  CylinderID cyl(0);
  pe_GLOBAL_SECTION
  {
     // Creating the ground plane
//     ground = createPlane( 777, 0.0, 0.0, 1.0, 0.01, granite, true );
//     createPlane( 1778,+1.0, 0.0, 0.0, 0, granite, false ); // right border
//     createPlane( 1779,-1.0, 0.0, 0.0,-4.0, granite, false ); // left border
// 
//     createPlane( 1780, 0.0, 1.0, 0.0, 0, granite, false ); // back border
//     createPlane( 1781, 0.0,-1.0, 0.0,-6, granite, false ); // front border


     cyl = createCylinder(3, Vec3(0.0, 0.0, 5.0), 0.8, 10.0, iron, true);
     cyl->setFixed(true);
     real Rad = degreesToRadians(-90.0);
 
     cyl->rotate(Vec3(0,Rad,0));


  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
  }

  // Setup of the POV-Ray visualization
  if( g_povray ) {
     WriterID pov = activateWriter();
     pov->setSpacing( visspacing );
     pov->include( "colors.inc" );
     pov->include( "woods.inc" );
     pov->include( "metals.inc" );
     pov->include( "textures.inc" );
     pov->setFilename( "./video/box%.pov" );
     pov->setBackground( 1.0, 1.0, 1.0 );
     pov->addLightSource( PointLight( Vec3( 0.0, -10.0, 30.0 ), Color( 0.95, 0.95, 0.95 ) ) );

     std::vector<PlainTexture> simTextures;

     PlainTexture white(
        ColorPigment( 1.0, 1.0, 1.0 ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );

     real aqua_rgb[] = {59./255., 183./255., 201./255.};
     PlainTexture aqua(
        ColorPigment( aqua_rgb[0], aqua_rgb[1], aqua_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );

     real purple_rgb[] = {189./255., 38./255., 156./255.};
     PlainTexture purple(
        ColorPigment( purple_rgb[0], purple_rgb[1], purple_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );

     real pink_rgb[] = {224./255., 34./255., 87./255.};
     PlainTexture pink(
        ColorPigment( pink_rgb[0], pink_rgb[1], pink_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );

     real col1_rgb[] = {171./255., 128./255., 84./255.};
     real col2_rgb[] = {161./255., 161./255., 255./255.};
     real col3_rgb[] = {255./255., 241./255., 0./255.};
     real col4_rgb[] = {143./255., 3./255., 3./255.};
     real col5_rgb[] = {3./255., 143./255., 3./255.};

     real col6_rgb[] = {141./255., 84./255., 0./255.};
     real col7_rgb[] = {149./255., 123./255., 172./255.};
     real col8_rgb[] = {150./255., 150./255., 150./255.};

     PlainTexture col1(
        ColorPigment( col1_rgb[0], col1_rgb[1], col1_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col2(
        ColorPigment( col2_rgb[0], col2_rgb[1], col2_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col3(
        ColorPigment( col3_rgb[0], col3_rgb[1], col3_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col4(
        ColorPigment( col4_rgb[0], col4_rgb[1], col4_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col5(
        ColorPigment( col5_rgb[0], col5_rgb[1], col5_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col6(
        ColorPigment( col6_rgb[0], col6_rgb[1], col6_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col7(
        ColorPigment( col7_rgb[0], col7_rgb[1], col7_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );
     PlainTexture col8(
        ColorPigment( col8_rgb[0], col8_rgb[1], col8_rgb[2] ),
        Finish(
           Ambient( 0.1 ),
           Diffuse( 0.6 ),
           Phong( 0.9 ),
           PhongSize( 50 ),
           Reflection( 0.05 )
        )
       );

     simTextures.push_back(white);
     simTextures.push_back(aqua);
     simTextures.push_back(purple);
     simTextures.push_back(pink);

     simTextures.push_back(col1);
     simTextures.push_back(col2);
     simTextures.push_back(col3);
     simTextures.push_back(col4);
     simTextures.push_back(col5);
     simTextures.push_back(col6);
     simTextures.push_back(col7);
     simTextures.push_back(col8);


     // Configuring the POV-Ray camera
     CameraID camera = theCamera();
     camera->setLocation( 5.0,-20.0, 10.0 );
     camera->setFocus   ( 1.0,  0.0, 1.0 );

     // Setting the ground plane texture
     Finish grassFinish(
        Ambient( 0.2 )
     );
     PlainTexture grassTexture(
        ImagePigment( gif, "blackmarble.gif", planar, true ),
        grassFinish,
        Scale( 20.0 ),
        Rotation( M_PI/2.0, 0.0, 0.0 )
     );

//     pov->setTexture( ground, CustomTexture( "White_Marble" ) );

     pov->setTexture(cyl, CustomTexture("Glass"));
     //std::vector<PlainTexture> simTextures;
     // Configuration of the texture policy
     std::ostringstream texture;
     texture << "Texture" << theMPISystem()->getRank()%4;
     pov->setTexturePolicy( DefaultTexture( simTextures[my_rank]));

     // Setting the textures of the boxes
     std::ostringstream oss;
  }

  const real lx = px * dx;
  const real ly = py * dy;
  const real lz = pz * dz;

//  const int nx = lx / space;
//  const int ny = ly / space;
//  const int nz = lz / space;
  const int nx = 10;
  const int ny = 10;
  const int nz = 6;


  pe_EXCLUSIVE_SECTION(0) {

    std::cout << "nx, ny, nz: " << (Vec3(nx, ny, nz)) << std::endl;

  }

  real pbx = -3.0;
  real pby = -3.0;
  real pbz =  0.0;
  //space = 1.0;
  //real   space( real(2)*radius+spacing );
  real pos[] = { pbx + radius+spacing, pby + radius+spacing, pbz + radius+spacing};

  int idx = 2;
//  for (int z = 0; z < nz; z++)
//  {
//    for (int y = 0; y < ny; y++)
//    {
//      for (int x = 0; x < nx; x++)
//      {
//        real ddx = x*space;
//        real ddy = y*space;
//        real ddz = z*space;
//        //Vec3 position(pos[0] + 0.5 * radius + dx, pos[1] + 0.5 * radius + dy, pos[2] + 0.5 * radius + dz);
//        Vec3 position(pos[0] + ddx, pos[1] + ddy, pos[2] + ddz);
//
//        if (world->ownsPoint( position )) {
//          createSphere(idx, position, radius, iron, true);
////          std::cout << " pos[0] + dx = " << pos[0] + dx << std::endl;
////          pe_EXCLUSIVE_SECTION(4) {
////            std::cout << "[Creating particle] at: " << (position) << " in domain: " << my_rank << std::endl;
////          }
//          ++idx;
//        }
//        else {
//
////          pe_EXCLUSIVE_SECTION(4) {
////            std::cout << "[NOT Creating particle] at: " << (position) << " in domain: " << my_rank << std::endl;
////          }
//        }
//      }
//    }
//  }
    Vec3 position(0.0, 0.0, 5.0);
    SphereID particle(0);
    if (world->ownsPoint(position)) {
      particle = createSphere(9000, position, 0.2, iron, true);
      std::cout << "[Creating special particle] at: " << (position) << " In domain: " << my_rank << std::endl;
      ++idx;
    }
    else {
      std::cout << "[NOT Creating special particle]: " << (position) << "NOT in domain: " << my_rank << std::endl;
    }
    position = Vec3(0.0, 0.0, 5.5);
    if (world->ownsPoint(position)) {
      particle = createSphere(9001, position, 0.2, iron, true);
      std::cout << "[Creating special particle] at: " << (position) << " In domain: " << my_rank << std::endl;
      ++idx;
    }
//    else {
//      std::cout << "[NOT Creating special particle]: " << (position) << "NOT in domain: " << my_rank << std::endl;
//    }
//    position = Vec3(-1.0, 0.0, 5.0);
//    if (world->ownsPoint(position)) {
//      particle = createSphere(9002, position, 0.3, iron, true);
//      std::cout << "[Creating special particle] at: " << (position) << " In domain: " << my_rank << std::endl;
//      ++idx;
//    }
//    else {
//      std::cout << "[NOT Creating special particle]: " << (position) << "NOT in domain: " << my_rank << std::endl;
//    }
//    position = Vec3( 5.0, 0.0, 5.0);
//    if (world->ownsPoint(position)) {
//      particle = createSphere(9003, position, 0.2, iron, true);
//      std::cout << "[Creating special particle] at: " << (position) << " In domain: " << my_rank << std::endl;
//      ++idx;
//    }
//    else {
//      std::cout << "[NOT Creating special particle]: " << (position) << "NOT in domain: " << my_rank << std::endl;
//    }

  // Synchronization of the MPI processes
  world->synchronize();
  
  // Volume of the domain
  real domainVol = dx * processesX * processesY * dy * processesZ * dz;

  // Particles total = 
  real partTotal = nx * ny * nz; 
  real partVol = real(4.0)/real(3.0) * M_PI * radius * radius * radius; 
  real totalPartVol = partTotal * partVol;
  real volFrac = (totalPartVol / domainVol) * 100.0;

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
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Particle volume fraction                = " << volFrac << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
}