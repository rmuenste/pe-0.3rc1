#ifndef _PE_SETUP_EL_FROZEN_TRACE_H_
#define _PE_SETUP_EL_FROZEN_TRACE_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>

void setupELFrozenTrace(MPI_Comm ex0,
                        pe::real xmin, pe::real xmax,
                        pe::real ymin, pe::real ymax,
                        pe::real zmin, pe::real zmax) {
  using namespace pe;

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

  MPISystemID mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  const int commSize = mpisystem->getSize();
  if (config.getProcessesY() != 1 || config.getProcessesZ() != 1) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "Frozen trace PE parallel setup requires processesY_=1 and processesZ_=1.\n";
    }
    MPI_Abort(ex0, 1);
  }
  if (config.getProcessesX() != commSize) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "Frozen trace PE parallel setup requires processesX_ == PE communicator size: "
                << config.getProcessesX() << " != " << commSize << "\n";
    }
    MPI_Abort(ex0, 1);
  }

  int dims[] = {config.getProcessesX(), 1, 1};
  int periods[] = {false, false, false};
  int reorder = false;
  MPI_Comm cartcomm;
  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if (cartcomm == MPI_COMM_NULL) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "Frozen trace PE parallel setup failed to create cartesian communicator.\n";
    }
    MPI_Abort(ex0, 1);
  }
  mpisystem->setComm(cartcomm);

  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  std::vector<real> xBounds;
  if (config.getProcessesX() == 4) {
    xBounds = {xmin, real(0.58333333), real(1.05), real(1.7), xmax};
  } else {
    xBounds.resize(config.getProcessesX() + 1);
    const real dxUniform = (xmax - xmin) / static_cast<real>(config.getProcessesX());
    for (int i = 0; i <= config.getProcessesX(); ++i) {
      xBounds[i] = xmin + static_cast<real>(i) * dxUniform;
    }
  }
  const real dy = ymax - ymin;
  const real dz = zmax - zmin;
  decomposeDomainNonuniformX(center, xBounds, ymin, zmin, dy, dz, config.getProcessesX(), 1, 1);
  theMPISystem()->checkProcesses();

  const real localXMin = xBounds[center[0]];
  const real localXMax = xBounds[center[0] + 1];
  const real localYMin = ymin;
  const real localYMax = ymax;
  const real localZMin = zmin;
  const real localZMax = zmax;
  const real localDx = localXMax - localXMin;

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

  if (xSeedMin >= xSeedMax || ySeedMin >= ySeedMax || zSeedMin >= zSeedMax) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "Frozen trace setup failed: CFD domain too small for requested particle padding.\n";
    }
    MPI_Abort(cartcomm, 1);
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

  pe_GLOBAL_SECTION {
    int planeIds = 12000;
    createPlane(planeIds++,  1.0,  0.0,  0.0,  xmin,  wallMaterial, true);
    createPlane(planeIds++,  0.0,  1.0,  0.0,  ymin,  wallMaterial, true);
    createPlane(planeIds++,  0.0, -1.0,  0.0, -ymax,  wallMaterial, true);
    createPlane(planeIds++,  0.0,  0.0,  1.0,  zmin,  wallMaterial, true);
    createPlane(planeIds++,  0.0,  0.0, -1.0, -zmax,  wallMaterial, true);

    CylinderID obstacleCylinder = createCylinder(planeIds++, Vec3(cylinderCenterX, cylinderCenterY, cylinderCenterZ),
                                                 cylinderRadius, zmax - zmin, obstacleMaterial, true);
    obstacleCylinder->rotate(0, M_PI * 0.5, 0.0);
    obstacleCylinder->setFixed(true);
  }

  int globalLatticeId = 0;
  int particlesCreatedLocal = 0;
  int particlesSkippedByCylinderLocal = 0;

  for (int iz = 0; iz < nz; ++iz) {
    const real z = latticeCoord(iz, zSeedMin);
    for (int iy = 0; iy < ny; ++iy) {
      const real y = latticeCoord(iy, ySeedMin);
      for (int ix = 0; ix < nx; ++ix, ++globalLatticeId) {
        const real x = latticeCoord(ix, xSeedMin);
        const real cx = x - cylinderCenterX;
        const real cy = y - cylinderCenterY;
        const real radialDistance = std::sqrt(cx * cx + cy * cy);

        if (radialDistance <= cylinderRadius + particleRadius) {
          if (world->ownsPoint(Vec3(x, y, z))) ++particlesSkippedByCylinderLocal;
          continue;
        }

        if (world->ownsPoint(Vec3(x, y, z))) {
          createSphere(globalLatticeId, Vec3(x, y, z), particleRadius, tracerMaterial, true);
          ++particlesCreatedLocal;
        }
      }
    }
  }

  world->synchronize();

  unsigned long localParticles = static_cast<unsigned long>(particlesCreatedLocal);
  unsigned long totalParticles = 0;
  unsigned long skippedLocal = static_cast<unsigned long>(particlesSkippedByCylinderLocal);
  unsigned long skippedTotal = 0;
  MPI_Reduce(&localParticles, &totalParticles, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&skippedLocal, &skippedTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

  if (config.getVtk()) {
    vtk::WriterID vtk = vtk::activateWriter("./paraview", config.getVisspacing(), 0,
                                            config.getTimesteps() * config.getSubsteps(), true, true);
  }

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "\n--Frozen-Field PE Parallel Initialization"
              << "--------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " PE decomposition                         = " << dims[0] << " x " << dims[1] << " x " << dims[2] << "\n"
              << " CFD domain xmin/xmax                    = " << xmin << " / " << xmax << "\n"
              << " CFD domain ymin/ymax                    = " << ymin << " / " << ymax << "\n"
              << " CFD domain zmin/zmax                    = " << zmin << " / " << zmax << "\n"
              << " Local PE slab dx/dy/dz                  = " << localDx << " / " << dy << " / " << dz << "\n"
              << " Local PE slab xmin/xmax                 = " << localXMin << " / " << localXMax << "\n"
              << " Closed PE wall planes                   = x = xmin, y = ymin/ymax, z = zmin/zmax\n"
              << " Open particle outflow                   = x = xmax\n"
              << " Particle diameter                       = " << particleDiameter << "\n"
              << " Seed lattice                            = " << nx << " x " << ny << " x " << nz << "\n"
              << " Particles skipped by cylinder           = " << skippedTotal << "\n"
              << " Particles created                       = " << totalParticles << "\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;
  }

  MPI_Barrier(cartcomm);
  if (mpisystem->getRank() == 0 && totalParticles == 0) {
    std::cerr << "Frozen trace setup failed: no particles were created for the configured PE slabs.\n";
    MPI_Abort(cartcomm, 1);
  }
}

#endif
