//=================================================================================================
/*!\file pe/interface/setup/el_frozen_trace.h
 *  \brief Frozen-field Euler-Lagrange tracer scaffold.
 *
 *  Entry point: commf2c_el_frozen_trace_().
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_EL_FROZEN_TRACE_H_
#define _PE_INTERFACE_SETUP_EL_FROZEN_TRACE_H_

#if HAVE_MPI
#include <pe/interface/setup/parallel_common.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/support/decompose.h>
#include <pe/interface/support/geometry_utils.h>
#include <pe/interface/support/optional_collision_params.h>

namespace pe {

//*************************************************************************************************
/*!\brief PE setup for the Euler-Lagrange frozen-field tracer case (parallel PE mode).
 *
 * \param ex0 The CFD worker communicator (excludes the CFD master rank 0).
 * \param xmin,xmax,ymin,ymax,zmin,zmax Bounding box of the CFD domain.
 *
 * Reached from Fortran through commf2c_el_frozen_trace_(). Tracer spheres are released
 * upstream of a fixed obstacle cylinder in a channel. The channel is closed by planes at
 * x = xmin, y = ymin/ymax and z = zmin/zmax and is open at x = xmax (particle outflow).
 *
 * Decomposition: slabs along x only (processesY_ = processesZ_ = 1 is required). For
 * processesX_ = 4 the slab bounds are a fixed non-uniform split tuned for this channel;
 * for every other process count the split is uniform.
 *
 * Particles, depending on packingMethod_:
 *   - External: positions from the xyz file, radius benchRadius_.
 *   - otherwise: a regular lattice of fixed-diameter tracers between x = xmin and x = 0.3,
 *     with lattice sites inside the obstacle cylinder skipped.
 *
 * Read from example.json: gravity, fluid density and viscosity, particle density, material
 * coefficients, initial particle velocity, step size and substeps, the lubrication settings,
 * and the VTK switch and spacing.
 *
 * Fixed in this file: the obstacle cylinder, the lattice tracer diameter and gap, the
 * downstream end of the seeding region, and the 4-process slab bounds.
 *
 * Every error path aborts the run with a message.
 */
inline void setupELFrozenTrace(MPI_Comm ex0,
                        pe::real xmin, pe::real xmax,
                        pe::real ymin, pe::real ymax,
                        pe::real zmin, pe::real zmax) {
  using namespace pe;

  //===============================================================================================
  // Case constants
  //===============================================================================================
  const Vec3 cylinderCenter(real(0.5), real(0.2), real(0.205));  // Obstacle cylinder
  const real cylinderRadius(real(0.05));

  const real latticeDiameter(real(0.01));                        // Lattice tracers only
  const real latticeRadius(real(0.5) * latticeDiameter);
  const real latticeGap(real(1.5) * latticeDiameter);
  const real latticeXEnd(real(0.3));                             // Downstream end of seeding

  //===============================================================================================
  // Configuration, world and fluid properties
  //===============================================================================================
  auto &config = SimulationConfig::getInstance();
  WorldID world = theWorld();
  loadSimulationConfig("example.json");

  config.setCfdDomainMin(Vec3(xmin, ymin, zmin));
  config.setCfdDomainMax(Vec3(xmax, ymax, zmax));

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);
  world->setAutoForceReset(true);

  TimeStep::stepsize(config.getStepsize());

  //===============================================================================================
  // MPI system and validation
  //===============================================================================================
  MPISystemID mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  int myRank = 0;
  MPI_Comm_rank(ex0, &myRank);

  // Prints the message once and aborts the whole run
  const auto abortSetup = [&](const std::string& message) {
    if (myRank == 0) {
      std::cerr << "\nERROR in setupELFrozenTrace: " << message << "\n" << std::endl;
    }
    MPI_Abort(ex0, 1);
  };

  const int px = config.getProcessesX();
  const int commSize = mpisystem->getSize();

  if (!(xmin < xmax) || !(ymin < ymax) || !(zmin < zmax)) {
    abortSetup("the CFD domain bounds are empty or inverted.");
  }
  if (config.getProcessesY() != 1 || config.getProcessesZ() != 1) {
    abortSetup("the frozen trace setup requires processesY_ = 1 and processesZ_ = 1.");
  }
  if (px != commSize) {
    abortSetup("the frozen trace setup requires processesX_ == PE communicator size: " +
               std::to_string(px) + " != " + std::to_string(commSize) + ".");
  }

  const bool externalPacking =
    (config.getPackingMethod() == SimulationConfig::PackingMethod::External);
  const real externalRadius(config.getBenchRadius());
  if (externalPacking && externalRadius <= real(0)) {
    abortSetup("external packing requires benchRadius_ > 0.");
  }

  //===============================================================================================
  // Slab decomposition along x
  //===============================================================================================
  int dims[] = {px, 1, 1};
  int periods[] = {false, false, false};
  int reorder = false;
  MPI_Comm cartcomm;
  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if (cartcomm == MPI_COMM_NULL) {
    abortSetup("failed to create the cartesian communicator.");
  }
  mpisystem->setComm(cartcomm);

  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  std::vector<real> xBounds;
  if (px == 4) {
    xBounds = {xmin, real(0.58333333), real(1.05), real(1.7), xmax};
  } else {
    xBounds.resize(px + 1);
    const real dxUniform = (xmax - xmin) / static_cast<real>(px);
    for (int i = 0; i <= px; ++i) {
      xBounds[i] = xmin + static_cast<real>(i) * dxUniform;
    }
  }

  // The fixed 4-process bounds only fit a domain that contains them
  real minSlabExtent = std::numeric_limits<real>::max();
  for (int i = 0; i < px; ++i) {
    const real extent = xBounds[i + 1] - xBounds[i];
    if (!(extent > real(0))) {
      abortSetup("the x slab bounds are not strictly increasing; the fixed 4-process "
                 "bounds do not fit the CFD domain [" + std::to_string(xmin) + ", " +
                 std::to_string(xmax) + "].");
    }
    if (px > 1) {
      minSlabExtent = std::min(minSlabExtent, extent);
    }
  }

  // Pairwise lubrication: widen the shadow-copy overlap test (no-op when disabled). The
  // clamp uses the thinnest ACTUAL slab, which matters for the non-uniform split.
  applyLubricationShadowCopyMargin(config, minSlabExtent, myRank == 0);

  const real dy = ymax - ymin;
  const real dz = zmax - zmin;
  decomposeDomainNonuniformX(center, xBounds, ymin, zmin, dy, dz, px, 1, 1);
  theMPISystem()->checkProcesses();

  //===============================================================================================
  // Materials, walls and the obstacle cylinder
  //===============================================================================================
  MaterialID tracerMaterial = createMaterial("frozen_trace_particle", config.getParticleDensity(),
                                             config.getRestitution(),
                                             config.getStaticFriction(),
                                             config.getDynamicFriction(),
                                             0.2, 80, 100, 10, 11);
  MaterialID wallMaterial = createMaterial("frozen_trace_wall", real(1.0),
                                           config.getRestitution(),
                                           config.getStaticFriction(),
                                           config.getDynamicFriction(),
                                           0.2, 80, 100, 10, 11);
  MaterialID obstacleMaterial = createMaterial("frozen_trace_cylinder", real(1.0),
                                               config.getRestitution(),
                                               config.getStaticFriction(),
                                               config.getDynamicFriction(),
                                               0.2, 80, 100, 10, 11);

  pe_GLOBAL_SECTION {
    int globalIds = 12000;
    createPlane(globalIds++,  1.0,  0.0,  0.0,  xmin,  wallMaterial, true);
    createPlane(globalIds++,  0.0,  1.0,  0.0,  ymin,  wallMaterial, true);
    createPlane(globalIds++,  0.0, -1.0,  0.0, -ymax,  wallMaterial, true);
    createPlane(globalIds++,  0.0,  0.0,  1.0,  zmin,  wallMaterial, true);
    createPlane(globalIds++,  0.0,  0.0, -1.0, -zmax,  wallMaterial, true);

    CylinderID obstacleCylinder = createCylinder(globalIds++, cylinderCenter,
                                                 cylinderRadius, zmax - zmin, obstacleMaterial, true);
    obstacleCylinder->rotate(0, M_PI * 0.5, 0.0);
    obstacleCylinder->setFixed(true);
  }

  //===============================================================================================
  // Particles
  //===============================================================================================
  const Vec3 initialParticleVelocity(config.getInitialParticleVelocity());

  unsigned long localParticles = 0;   // Spheres created on this process
  unsigned long skippedLocal   = 0;   // Lattice sites inside the obstacle (lattice mode)
  unsigned long expectedTotal  = 0;   // Positions requested (external mode)
  int nx = 0, ny = 0, nz = 0;         // Lattice dimensions (lattice mode)

  if (externalPacking) {
    const std::vector<Vec3> spherePositions = readVectorsFromFile(config.getXyzFilePath().string());
    if (spherePositions.empty()) {
      abortSetup("external packing read no positions from " +
                 config.getXyzFilePath().string() + ".");
    }
    expectedTotal = static_cast<unsigned long>(spherePositions.size());

    for (std::size_t i = 0; i < spherePositions.size(); ++i) {
      const Vec3 pos = spherePositions[i];
      if (world->ownsPoint(pos)) {
        SphereID sphere = createSphere(static_cast<int>(i), pos,
                                       externalRadius, tracerMaterial, true);
        sphere->setLinearVel(initialParticleVelocity);
        ++localParticles;
      }
    }
  } else {
    const real pitch(latticeDiameter + latticeGap);
    const real wallPadding(latticeRadius + latticeGap);
    const real xSeedMin(xmin + wallPadding);
    const real xSeedMax(latticeXEnd - wallPadding);
    const real ySeedMin(ymin + wallPadding);
    const real ySeedMax(ymax - wallPadding);
    const real zSeedMin(zmin + wallPadding);
    const real zSeedMax(zmax - wallPadding);

    if (xSeedMin >= xSeedMax || ySeedMin >= ySeedMax || zSeedMin >= zSeedMax) {
      abortSetup("the CFD domain is too small for the lattice seeding region and its wall padding.");
    }

    nx = static_cast<int>(std::floor((xSeedMax - xSeedMin) / pitch)) + 1;
    ny = static_cast<int>(std::floor((ySeedMax - ySeedMin) / pitch)) + 1;
    nz = static_cast<int>(std::floor((zSeedMax - zSeedMin) / pitch)) + 1;

    // Every process walks the same global lattice, so the user IDs are consistent
    int globalLatticeId = 0;
    for (int iz = 0; iz < nz; ++iz) {
      const real z = zSeedMin + real(iz) * pitch;
      for (int iy = 0; iy < ny; ++iy) {
        const real y = ySeedMin + real(iy) * pitch;
        for (int ix = 0; ix < nx; ++ix, ++globalLatticeId) {
          const real x = xSeedMin + real(ix) * pitch;
          const Vec3 pos(x, y, z);
          if (!world->ownsPoint(pos)) {
            continue;
          }

          const real cx = x - cylinderCenter[0];
          const real cy = y - cylinderCenter[1];
          if (std::sqrt(cx * cx + cy * cy) <= cylinderRadius + latticeRadius) {
            ++skippedLocal;
            continue;
          }

          SphereID sphere = createSphere(globalLatticeId, pos, latticeRadius, tracerMaterial, true);
          sphere->setLinearVel(initialParticleVelocity);
          ++localParticles;
        }
      }
    }
  }

  world->synchronize();

  if (config.getVtk()) {
    vtk::activateWriter("./paraview", config.getVisspacing(), 0,
                        config.getTimesteps() * config.getSubsteps(), true, true);
  }

  //===============================================================================================
  // Setup summary and final checks
  //===============================================================================================
  unsigned long totalParticles = 0;
  unsigned long skippedTotal = 0;
  MPI_Reduce(&localParticles, &totalParticles, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&skippedLocal, &skippedTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "\n--Frozen-Field PE Parallel Initialization"
              << "--------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " PE decomposition                        = " << dims[0] << " x " << dims[1] << " x " << dims[2] << "\n"
              << " CFD domain xmin/xmax                    = " << xmin << " / " << xmax << "\n"
              << " CFD domain ymin/ymax                    = " << ymin << " / " << ymax << "\n"
              << " CFD domain zmin/zmax                    = " << zmin << " / " << zmax << "\n"
              << " PE slab bounds in x                     =";
    for (std::size_t i = 0; i < xBounds.size(); ++i) {
      std::cout << " " << xBounds[i];
    }
    std::cout << "\n"
              << " Closed PE wall planes                   = x = xmin, y = ymin/ymax, z = zmin/zmax\n"
              << " Open particle outflow                   = x = xmax\n";
    if (externalPacking) {
      std::cout << " Packing                                 = external ("
                << config.getXyzFilePath().string() << ")\n"
                << " Particle radius                         = " << externalRadius << "\n"
                << " Positions in file                       = " << expectedTotal << "\n";
    } else {
      std::cout << " Packing                                 = lattice\n"
                << " Particle diameter                       = " << latticeDiameter << "\n"
                << " Seed lattice                            = " << nx << " x " << ny << " x " << nz << "\n"
                << " Lattice sites skipped by cylinder       = " << skippedTotal << "\n";
    }
    std::cout << " Particles created                       = " << totalParticles << "\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;

    if (externalPacking && totalParticles != expectedTotal) {
      std::cerr << "WARNING in setupELFrozenTrace: " << expectedTotal
                << " positions were read but " << totalParticles
                << " spheres were created; positions outside the PE domain are dropped.\n";
    }
    if (totalParticles == 0) {
      std::cerr << "\nERROR in setupELFrozenTrace: no particles were created for the "
                   "configured PE slabs.\n" << std::endl;
      MPI_Abort(cartcomm, 1);
    }
  }

  MPI_Barrier(cartcomm);
}
//*************************************************************************************************

}  // namespace pe
#endif  // HAVE_MPI

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief Frozen-field E-L scaffold setup for serial mode
 *
 * Loads the JSON configuration and initializes only the general PE world state.
 * No benchmark geometry or particles are created here; the caller can use this
 * as a lightweight PE bootstrap for checkpoint-driven frozen-field applications.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupELFrozenTraceSerial(int cfd_rank,
                                     real xmin, real xmax,
                                     real ymin, real ymax,
                                     real zmin, real zmax) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  config.setCfdDomainMin(Vec3(xmin, ymin, zmin));
  config.setCfdDomainMax(Vec3(xmax, ymax, zmax));
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);
  world->setAutoForceReset(true);

  TimeStep::stepsize(config.getStepsize());

  const real rhoParticle(config.getParticleDensity());
  const real particleDiameter(real(0.01));
  const real particleRadius(real(0.5) * particleDiameter);
  const real cylinderCenterX(real(0.5));
  const real cylinderCenterY(real(0.2));
  const real cylinderCenterZ(real(0.205));
  const real cylinderRadius(real(0.05));
  const real particleGap(real(1.5) * particleDiameter);
  const real pitch(particleDiameter + particleGap);
  const real wallPadding(particleRadius + particleGap);
  const real xSeedMin(xmin + wallPadding);
  const real xSeedMax(real(0.3) - wallPadding);
  const real ySeedMin(ymin + wallPadding);
  const real ySeedMax(ymax - wallPadding);
  const real zSeedMin(zmin + wallPadding);
  const real zSeedMax(zmax - wallPadding);
  const bool hasOpenOutflowXMax(true);

  if (xSeedMin >= xSeedMax || ySeedMin >= ySeedMax || zSeedMin >= zSeedMax) {
    throw std::runtime_error("Frozen trace setup failed: CFD domain too small for requested particle padding.");
  }

  const int nx = std::max(1, static_cast<int>(std::floor((xSeedMax - xSeedMin) / pitch)) + 1);
  const int ny = std::max(1, static_cast<int>(std::floor((ySeedMax - ySeedMin) / pitch)) + 1);
  const int nz = std::max(1, static_cast<int>(std::floor((zSeedMax - zSeedMin) / pitch)) + 1);

  const auto latticeCoord = [pitch](int i, real minVal) -> real {
    return minVal + real(i) * pitch;
  };

  MaterialID tracerMaterial = createMaterial("frozen_trace_particle", rhoParticle,
                                             0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID wallMaterial = createMaterial("frozen_trace_wall", real(1.0),
                                           0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
  MaterialID obstacleMaterial = createMaterial("frozen_trace_cylinder", real(1.0),
                                               0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  int planeIds = 12000;
  createPlane(planeIds++,  1.0,  0.0,  0.0,  xmin,  wallMaterial, true);  // closed inlet plane
  createPlane(planeIds++,  0.0,  1.0,  0.0,  ymin,  wallMaterial, true);  // y = ymin wall
  createPlane(planeIds++,  0.0, -1.0,  0.0, -ymax,  wallMaterial, true);  // y = ymax wall
  createPlane(planeIds++,  0.0,  0.0,  1.0,  zmin,  wallMaterial, true);  // z = zmin wall
  createPlane(planeIds++,  0.0,  0.0, -1.0, -zmax,  wallMaterial, true);  // z = zmax wall

  CylinderID obstacleCylinder = createCylinder(planeIds++, Vec3(cylinderCenterX, cylinderCenterY, cylinderCenterZ),
                                               cylinderRadius, zmax - zmin, obstacleMaterial, true);
  obstacleCylinder->rotate(0, M_PI * 0.5, 0.0);
  obstacleCylinder->setFixed(true);

  int idx = 0;
  int particlesCreated = 0;
  int particlesSkippedByCylinder = 0;

  for (int iz = 0; iz < nz; ++iz) {
    const real z = latticeCoord(iz, zSeedMin);
    for (int iy = 0; iy < ny; ++iy) {
      const real y = latticeCoord(iy, ySeedMin);
      for (int ix = 0; ix < nx; ++ix) {
        const real x = latticeCoord(ix, xSeedMin);
        const real dx = x - cylinderCenterX;
        const real dy = y - cylinderCenterY;
        const real radialDistance = std::sqrt(dx * dx + dy * dy);

        // Exclude points inside the obstacle cylinder extended by one particle radius.
        if (radialDistance <= cylinderRadius + particleRadius) {
          ++particlesSkippedByCylinder;
          continue;
        }

        createSphere(idx++, Vec3(x, y, z), particleRadius, tracerMaterial, true);
        ++particlesCreated;
      }
    }
  }

  if (particlesCreated == 0) {
    throw std::runtime_error("Frozen trace setup failed: no particles remain after cylinder exclusion.");
  }

  if (isRepresentative && config.getVtk()) {
    vtk::WriterID vtk = vtk::activateWriter("./paraview", 1, 0, config.getTimesteps(), false);
    const Vec3 gravity = world->getGravity();
    const real initDumpDt = std::max(real(1e-12), TimeStep::size() * real(1e-9));

    // The VTK writer is trigger-based. Emit one initial snapshot without materially
    // changing the seeded particle cloud by stepping once with vanishing dt and no gravity.
    world->setGravity(0.0, 0.0, 0.0);
    world->simulationStep(initDumpDt);
    world->setGravity(gravity);
    TimeStep::stepsize(config.getStepsize());
  }

  if (isRepresentative) {
    std::cout << "\n--"
              << "Frozen-Field PE General Initialization"
              << "--------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << config.getFluidViscosity() << "\n"
              << " Fluid density                           = " << config.getFluidDensity() << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " CFD domain xmin/xmax                    = " << xmin << " / " << xmax << "\n"
              << " CFD domain ymin/ymax                    = " << ymin << " / " << ymax << "\n"
              << " CFD domain zmin/zmax                    = " << zmin << " / " << zmax << "\n"
              << " Closed PE wall planes                   = x = xmin, y = ymin/ymax, z = zmin/zmax\n"
              << " Open particle outflow                   = x = xmax"
              << (hasOpenOutflowXMax ? "" : " (disabled)") << "\n"
              << " Particle diameter                       = " << particleDiameter << "\n"
              << " Particle radius                         = " << particleRadius << "\n"
              << " Particle gap                            = " << particleGap << "\n"
              << " Center-to-center pitch                  = " << pitch << "\n"
              << " Wall padding                            = " << wallPadding << "\n"
              << " Seed lattice                            = " << nx << " x " << ny << " x " << nz << "\n"
              << " Seed box xmin/xmax                      = " << xSeedMin << " / " << xSeedMax << "\n"
              << " Seed box ymin/ymax                      = " << ySeedMin << " / " << ySeedMax << "\n"
              << " Seed box zmin/zmax                      = " << zSeedMin << " / " << zSeedMax << "\n"
              << " Excluded cylinder center(x,y,z), radius = "
              << cylinderCenterX << ", " << cylinderCenterY << ", " << cylinderCenterZ
              << ", " << cylinderRadius << "\n"
              << " Fixed PE cylinder length                = " << (zmax - zmin) << "\n"
              << " Particles skipped by cylinder           = " << particlesSkippedByCylinder << "\n"
              << " Particles created                       = " << particlesCreated << "\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_EL_FROZEN_TRACE_H_
