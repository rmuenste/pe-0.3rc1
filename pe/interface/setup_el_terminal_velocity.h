#ifndef _PE_SETUP_EL_TERMINAL_VELOCITY_H_
#define _PE_SETUP_EL_TERMINAL_VELOCITY_H_

#include <cmath>
#include <iostream>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>

//*************************************************************************************************
/*!\brief PE setup for the unresolved Euler-Lagrange single-particle terminal-velocity case.
 *
 * Dedicated setup for the Tier-2 terminal-velocity verification (q2p1_el_pipeflow). Unlike
 * setupELFrozenTrace this routine:
 *   - creates exactly ONE free (mobile) sphere, read from particles.xyz (first position),
 *   - uses the GENERAL decomposeDomain so the PE decomposition follows whatever axis the CFD
 *     mesh is partitioned on (driven by processesX_/Y_/Z_ in example.json). For the z-split
 *     QBOX9Z3 mesh this means processesZ_=3, so PE and CFD own the same spatial slabs,
 *   - adds NO obstacle cylinder and NO boundary walls (the particle stays in the interior for
 *     a settling test; the enclosed-box pressure is handled CFD-side via NoOutflow).
 */
void setupELTerminalVelocity(MPI_Comm ex0,
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

  const int px = config.getProcessesX();
  const int py = config.getProcessesY();
  const int pz = config.getProcessesZ();
  if (px * py * pz != commSize) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "EL terminal-velocity PE setup requires processesX_*Y_*Z_ == PE communicator "
                << "size: " << px << "*" << py << "*" << pz << " != " << commSize << "\n";
    }
    MPI_Abort(ex0, 1);
  }

  int dims[]    = {px, py, pz};
  int periods[] = {false, false, false};
  int reorder   = false;
  MPI_Comm cartcomm;
  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if (cartcomm == MPI_COMM_NULL) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "EL terminal-velocity PE setup failed to create cartesian communicator.\n";
    }
    MPI_Abort(ex0, 1);
  }
  mpisystem->setComm(cartcomm);

  int center[3];
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  const real dx = (xmax - xmin) / static_cast<real>(px);
  const real dy = (ymax - ymin) / static_cast<real>(py);
  const real dz = (zmax - zmin) / static_cast<real>(pz);
  decomposeDomain(center, xmin, ymin, zmin, dx, dy, dz, px, py, pz);
  theMPISystem()->checkProcesses();

  const real sphereRadius = config.getBenchRadius();
  const real rhoParticle  = config.getParticleDensity();
  const Vec3 initialParticleVelocity(config.getInitialParticleVelocity());
  if (sphereRadius <= real(0)) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "EL terminal-velocity setup requires benchRadius_ > 0.\n";
    }
    MPI_Abort(cartcomm, 1);
  }

  MaterialID sphereMaterial = createMaterial("el_tv_particle", rhoParticle,
                                             0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  std::vector<Vec3> spherePositions = readVectorsFromFile(config.getXyzFilePath().string());
  if (spherePositions.empty()) {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "EL terminal-velocity setup read no positions from "
                << config.getXyzFilePath().string() << ".\n";
    }
    MPI_Abort(cartcomm, 1);
  }

  // Terminal-velocity case: exactly one free sphere (the first listed position).
  int createdLocal = 0;
  const Vec3 pos = spherePositions[0];
  if (world->ownsPoint(pos)) {
    SphereID sphere = createSphere(0, pos, sphereRadius, sphereMaterial, true);
    sphere->setLinearVel(initialParticleVelocity);
    ++createdLocal;
  }

  world->synchronize();

  unsigned long localParticles  = static_cast<unsigned long>(createdLocal);
  unsigned long globalParticles = 0;
  MPI_Reduce(&localParticles, &globalParticles, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  pe_EXCLUSIVE_SECTION(0) {
    if (globalParticles != 1) {
      std::cerr << "EL terminal-velocity setup: expected exactly one sphere, created "
                << globalParticles << " (check that particles.xyz position lies inside the domain).\n";
    } else {
      std::cout << "EL terminal-velocity setup: one free sphere created (r=" << sphereRadius
                << ", rho=" << rhoParticle << "), PE decomposition " << px << "x" << py << "x" << pz
                << ".\n";
    }
  }
}
//*************************************************************************************************

#endif
