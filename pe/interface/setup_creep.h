#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <pe/core/Types.h>
#include <pe/math/Quaternion.h>

using namespace pe::povray;



//=================================================================================================
// Setup for the Creep Flow case
//=================================================================================================
void setupCreep(MPI_Comm ex0) {

  world = theWorld();
  world->setGravity( 0.0, 0.0, 0.0 );

  // Re 1.5 configuration
  real simViscosity( 1.0e-3 );
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

  const real L( 2.0 );
  const real LY( 1.0 );
  const real LZ( 0.05 );
  const real dx( L/processesX );
  const real dy( LY/processesY );
  const real dz( LZ/processesZ );

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

//  real bx = 0.0;
//  real by = 0.0;
//  real bz = 0.0;

  real bx = -1.0;
  real by = -0.5;
  real bz = 0.0;
//  const real L( 2.0 );
//  const real LY( 1.0 );
//  const real LZ( 0.05 );

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
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
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
  Vec3 ellipsoidPos = Vec3(0.0 * L, 0.0 * L, 0.0125); 
  //Vec3 ellipsoidPos = Vec3(0.05, 0.0 * L, 0.025); 
  TriangleMeshID ellipsoid;
  CapsuleID cap;

  if(!resume) {
  std::string fileName = std::string("ellipsoid.obj");
  if(world->ownsPoint(ellipsoidPos)) {

//    CylinderID cylinder = createCylinder( ++idx, ellipsoidPos, 0.1, 0.051, elastic );
//    cylinder->rotate( 0.0, 0.5 * M_PI, 0.0 );
//    Quaternion<real> q = Quaternion<real>( 0, 0.5 * M_PI, 0 );
//    cylinder->setOrientation(q);
    //real c=0.075, b=0.05, a=0.1;
    //real c=0.0500001, b=0.02, a=0.04;
    //real c=0.012500001, b=0.025, a=0.05;
    real c=0.012500001, b=0.0125, a=0.025;
    //real c=0.0500001, b=0.01, a=0.02;
    //real c=0.05005, b=0.05, a=0.05;
    std::cout << "Creating Ellipsoid in domain " << MPISettings::rank() << " size = (" << a << ", " << b << ", " << c << ")" << std::endl;
    //real a=0.25, b=0.25, c=0.25;
    EllipsoidID ell = createEllipsoid(++idx, ellipsoidPos, a, b, c, elastic);
    //ell->rotate( 0.0, 0, 0.5 * M_PI);
    //ell->setAngularVel( 0.0, 0, -0.4);
    std::cout << "I: " << ell->getBodyInertia() << std::endl;
    std::cout << "I^-1: " << ell->getInvBodyInertia() << std::endl;
    std::cout << "V: " << ell->getVolume() << std::endl;
    std::cout << "M: " << ell->getMass() << std::endl;

//    CapsuleID cap = createCapsule(++idx, ellipsoidPos, 0.05, 0.1, elastic);
//    ellipsoid = createTriangleMesh(++idx, ellipsoidPos, fileName,
//                                   elastic, true, true, Vec3(1.0, 1.0, 1.0),
//                                   false, false);
//    Quaternion<real> q = Quaternion<real>( 0, 0.5 * M_PI, 0 );
//    ellipsoid->setOrientation(q);

    // Is this is local coordinates?
    //ellipsoid->setAngularVel(Vec3(0, 0, 5));

//    std::cout << ellipsoid->getInertia() << std::endl;
//    std::cout << ellipsoid->getBodyInertia() << std::endl;
//    std::cout << ellipsoid->getMass() << std::endl;

//    std::cout << ellipsoid->getInertia() << std::endl;
//    std::cout << ellipsoid->getBodyInertia() << std::endl;
//    std::cout << ellipsoid->getMass() << std::endl;

/*
 * - pe::createCylinder( id_t uid, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCylinder( id_t uid, const Vec3 &gpos, real radius, real length, MaterialID material, bool visible )
 *
 * In order to destroy a specific cylinder primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a cylinder primitive:

   \code
   // Creates the iron cylinder 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4. Per default the cylinder is visible in all visualizations. Note that
   // the cylinder is automatically added to the simulation world and is immediately part
   // of the entire simulation. The function returns a handle to the newly created cylinder,
   // which can be used to for instance rotate the cylinder around the global y-axis.
   CylinderID cylinder = createCylinder( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   cylinder->rotate( 0.0, PI/3.0, 0.0 );

*/
    
    //Quaternion<real> q = Quaternion<real>( 0, 0.5 * M_PI, 0 );
    //ellipsoid->setOrientation(q);
    //ellipsoid->setAngularVel(Vec3(0, 5, 0));
    //setOrientation( const Quat& q ) = 0;
    //inline Quaternion<Type>::Quaternion( Type xangle, Type yangle, Type zangle )
    //{
    //   reset();
    //   rotateX( xangle );
    //   rotateY( yangle );
    //   rotateZ( zangle );
    //}

  }
  }
  else {
   //checkpointer.read( "../start.1" );
  }
  
  //=========================================================================================
  
//  BodyID botPlane; 
//  BodyID topPlane;
//
//  pe_GLOBAL_SECTION
//  {
//     createPlane( 99999, 0.0, 0.0, 1.0, 0.0, elastic, false ); // bottom border
//     topPlane = createPlane( 88888, 0.0, 0.0,-1.0, -lz, elastic, false ); // top border
//  }

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

