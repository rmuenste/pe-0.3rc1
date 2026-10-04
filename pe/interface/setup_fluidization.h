#ifndef _PE_SETUP_FLUIDIZATION_H_
#define _PE_SETUP_FLUIDIZATION_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>

//*************************************************************************************************
/*!\brief PE setup for the fluidization column case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 *
 * Reached from Fortran through commf2c_fluidization_(). The case is a thin, fixed-size column
 * with a ground plane at z = 0. A fixed number of equal spheres is seeded on a regular lattice
 * that is filled x-first, then y, then z, starting above the ground plane.
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density, the process
 * layout (processesX_/Y_/Z_), the step size, the VTK switch and spacing, and the lattice
 * spacing factor (gap between sphere surfaces in units of the particle radius).
 *
 * Fixed in this file: the column bounds, the particle count and the particle diameter.
 *
 * Every error path aborts the run with a message. A setup that returned early would let the
 * CFD solver continue with an incomplete particle world.
 */
void setupFluidization(MPI_Comm ex0) {

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const real xMin =  0.0;
  const real xMax = 20.3;
  const real yMin =  0.0;
  const real yMax =  0.686;
  const real zMin =  0.0;
  const real zMax = 70.2;

  const int  targetParticles = 1204;
  const real radParticle     = real(0.5) * real(0.635);  // Diameter 0.635 cm

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto& config = SimulationConfig::getInstance();

  world = theWorld();

  loadSimulationConfig("example.json");

  const real simViscosity(config.getFluidViscosity());
  const real simRho(config.getFluidDensity());
  const real rhoParticle(config.getParticleDensity());

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(simRho);
  world->setViscosity(simViscosity);
  world->setDamping(1.0);

  TimeStep::stepsize(config.getStepsize());

  //===============================================================================================
  // MPI system and validation (nothing is created before all checks have passed)
  //===============================================================================================
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupFluidization: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();

  if (px * py * pz != mpisystem->getSize()) {
    abortSetup("invalid number of MPI processes: " + std::to_string(mpisystem->getSize()) +
               " != " + std::to_string(px * py * pz) + " (processesX_*Y_*Z_).");
  }

  // Seeding lattice: gap between sphere surfaces = spacingFactor * radius
  const real spacingFactor = std::max(real(0.0), config.getFluidizationSpacingFactor());
  const real spacing = spacingFactor * radParticle;
  const real pitch   = real(2.0) * radParticle + spacing;
  const real zStart  = std::max(real(4.0) * radParticle, zMin + radParticle);

  const real xSpan = (xMax - xMin) - real(2.0) * radParticle;
  const real ySpan = (yMax - yMin) - real(2.0) * radParticle;
  const real zSpan = (zMax - zStart) - radParticle;

  const int nxMax = static_cast<int>(std::floor(xSpan / pitch)) + 1;
  const int nyMax = static_cast<int>(std::floor(ySpan / pitch)) + 1;
  const int nzMax = static_cast<int>(std::floor(zSpan / pitch)) + 1;

  if (nxMax <= 0 || nyMax <= 0 || nzMax <= 0) {
    abortSetup("fluidization column too small for the requested particle diameter/spacing.");
  }

  const int maxCapacity = nxMax * nyMax * nzMax;
  if (maxCapacity < targetParticles) {
    abortSetup("fluidization grid capacity (" + std::to_string(maxCapacity) +
               ") is smaller than the requested particles (" +
               std::to_string(targetParticles) + ").");
  }

  //===============================================================================================
  // 3D rectilinear, non-periodic domain decomposition of the column
  //===============================================================================================
  int dims   [] = { px, py, pz };
  int periods[] = { false, false, false };
  int reorder   = false;
  MPI_Comm cartcomm;

  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if (cartcomm == MPI_COMM_NULL) {
    abortSetup("failed to create the cartesian communicator.");
  }
  mpisystem->setComm(cartcomm);

  // Cartesian coordinates of this process within the process grid
  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << Vec3(dims[0], dims[1], dims[2]) << std::endl;
    std::cout << "Rank:" << myRank << "->" << Vec3(center[0], center[1], center[2]) << std::endl;
  }

  const real dx = (xMax - xMin) / px;
  const real dy = (yMax - yMin) / py;
  const real dz = (zMax - zMin) / pz;

  decomposeDomain(center, xMin, yMin, zMin, dx, dy, dz, px, py, pz);

  // Checking the process setup
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Bodies: ground plane and the particle lattice
  //===============================================================================================
  MaterialID gr = createMaterial("ground", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  pe_GLOBAL_SECTION
  {
     g_ground = createPlane(777, 0.0, 0.0, 1.0, 0.0, gr, true);
  }

  MaterialID myMaterial = createMaterial("FluidizationParticles", rhoParticle, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  theCollisionSystem()->setMinEps(5e-6 / radParticle);

  // Every process walks the same global lattice and creates the spheres it owns, so the
  // user IDs (globalIndex) are consistent across processes.
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

  // Setup of the VTK visualization
  if (config.getVtk()) {
    vtk::activateWriter("./paraview", config.getVisspacing(), 0, config.getTimesteps(), false, true);
  }

  //===============================================================================================
  // Setup summary
  //===============================================================================================
  unsigned long particlesTotal(0);
  unsigned long primitivesTotal(0);
  unsigned long localBodies = static_cast<unsigned long>(theCollisionSystem()->getBodyStorage().size());
  MPI_Reduce(&localParticles, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localBodies, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

  const real sphereVol = (real(4.0) / real(3.0)) * M_PI * radParticle * radParticle * radParticle;
  const real localVol = static_cast<real>(localParticles) * sphereVol;
  const real localMass = localVol * rhoParticle;
  real totalMass(0.0);
  real totalVol(0.0);
  MPI_Reduce(&localMass, &totalMass, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localVol, &totalVol, 1, MPI_DOUBLE, MPI_SUM, 0, cartcomm);

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "FLUIDIZATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
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
//*************************************************************************************************

#endif
