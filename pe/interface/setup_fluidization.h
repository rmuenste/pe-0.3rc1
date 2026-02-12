
#include <pe/interface/decompose.h>
#include <pe/config/SimulationConfig.h>
#include <algorithm>
#include <cmath>

void setupFluidization(MPI_Comm ex0) {

  auto& config = SimulationConfig::getInstance();

  world = theWorld();

  loadSimulationConfig("example.json");
  world->setGravity(config.getGravity());

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());

  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const real xMin = 0.0;
  const real xMax = 20.3;
  const real yMin = 0.0;
  const real yMax = 0.686;
  const real zMin = 0.0;
  const real zMax = 70.2;

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

  // Setup domain decomposition for the fixed fluidization column bounds.
  const real dx = (xMax - xMin) / config.getPx();
  const real dy = (yMax - yMin) / config.getPy();
  const real dz = (zMax - zMin) / config.getPz();

  decomposeDomain(center, xMin, yMin, zMin, dx, dy, dz,
                  config.getPx(), config.getPy(), config.getPz());

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_GLOBAL_SECTION
  {
     g_ground = createPlane(777, 0.0, 0.0, 1.0, 0.0, gr, true);
  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "#==================================================================================" << std::endl;
  }

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
  }

  const int targetParticles = 1204;
  const real radParticle = real(0.5) * real(0.635);  // Diameter 0.635 cm
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  const real spacing = spacingFactor * radParticle;
  const real pitch = real(2.0) * radParticle + spacing;
  const real zStart = std::max(real(4.0) * radParticle, zMin + radParticle);

  const real xSpan = (xMax - xMin) - real(2.0) * radParticle;
  const real ySpan = (yMax - yMin) - real(2.0) * radParticle;
  const real zSpan = (zMax - zStart) - radParticle;

  const int nxMax = static_cast<int>(std::floor(xSpan / pitch)) + 1;
  const int nyMax = static_cast<int>(std::floor(ySpan / pitch)) + 1;
  const int nzMax = static_cast<int>(std::floor(zSpan / pitch)) + 1;

  if (nxMax <= 0 || nyMax <= 0 || nzMax <= 0) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "ERROR: Fluidization column too small for requested particle diameter/spacing." << std::endl;
    }
    return;
  }

  const int maxCapacity = nxMax * nyMax * nzMax;
  if (maxCapacity < targetParticles) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "ERROR: Fluidization grid capacity (" << maxCapacity
                << ") is smaller than requested particles (" << targetParticles << ")." << std::endl;
    }
    return;
  }

  MaterialID myMaterial = createMaterial("FluidizationParticles", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  theCollisionSystem()->setMinEps(5e-6 / radParticle);

  unsigned long localParticles = 0;
  int usedNx = 0;
  int usedNy = 0;
  int usedNz = 0;

  int globalIndex = 0;
  for (int iz = 0; iz < nzMax && globalIndex < targetParticles; ++iz) {
    const real z = zStart + static_cast<real>(iz) * pitch;
    for (int iy = 0; iy < nyMax && globalIndex < targetParticles; ++iy) {
      const real y = (yMin + radParticle) + static_cast<real>(iy) * pitch;
      for (int ix = 0; ix < nxMax && globalIndex < targetParticles; ++ix) {
        const real x = (xMin + radParticle) + static_cast<real>(ix) * pitch;
        const Vec3 position(x, y, z);
        if (world->ownsPoint(position)) {
          createSphere(globalIndex, position, radParticle, myMaterial, true);
          ++localParticles;
        }
        ++globalIndex;
        usedNx = std::max(usedNx, ix + 1);
        usedNy = std::max(usedNy, iy + 1);
        usedNz = std::max(usedNz, iz + 1);
      }
    }
  }

  // Synchronization of the MPI processes
  world->synchronize();

  // Calculating the total number of particles and primitives
  unsigned long particlesTotal(0);
  unsigned long primitivesTotal(0);
  int numBodies =  theCollisionSystem()->getBodyStorage().size();
  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  MPI_Reduce(&localParticles, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

  const real sphereVol = (real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle;
  const real localVol = static_cast<real>(localParticles) * sphereVol;
  const real localMass = localVol * rhoParticle;
  real totalMass(0.0);
  real totalVol(0.0);
  MPI_Reduce(&localMass, &totalMass, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localVol, &totalVol, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);

  TimeStep::stepsize( config.getStepsize() );

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "FLUIDIZATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n" 
      << " Total number of MPI processes           = " << config.getPx() * config.getPy() * config.getPz() << "\n"
      << " Total number of particles               = " << particlesTotal << "\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Viscosity                         = " << simViscosity << "\n"
      << " Fluid Density                           = " << simRho << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Column bounds [x,y,z]                   = "
      << "[" << xMin << "," << xMax << "] x "
      << "[" << yMin << "," << yMax << "] x "
      << "[" << zMin << "," << zMax << "]\n"
      << " Particle radius                         = " << radParticle << "\n"
      << " Particle diameter                       = " << (real(2.0) * radParticle) << "\n"
      << " Spacing factor                          = " << spacingFactor << "\n"
      << " Gap (between surfaces)                  = " << spacing << "\n"
      << " Center-to-center pitch                  = " << pitch << "\n"
      << " Grid start z                            = " << zStart << "\n"
      << " Grid extents used (nx,ny,nz)            = "
      << usedNx << ", " << usedNy << ", " << usedNz << "\n"
      << " Particle mass                           = " << totalMass << "\n" 
      << " Particle volume                         = " << totalVol << "\n" 
      << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   
}
