#ifndef _PE_SETUP_EL_FROZEN_TRACE_H_
#define _PE_SETUP_EL_FROZEN_TRACE_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/interface/setup_optional_collision_params.h>

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
void setupELFrozenTrace(MPI_Comm ex0,
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

#endif
