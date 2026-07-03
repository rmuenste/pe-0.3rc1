#ifndef _PE_SETUP_EL_TERMINAL_VELOCITY_H_
#define _PE_SETUP_EL_TERMINAL_VELOCITY_H_

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>

namespace {

std::string elTerminalLower(std::string text) {
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] >= 'A' && text[i] <= 'Z') {
      text[i] = static_cast<char>(text[i] - 'A' + 'a');
    }
  }
  return text;
}

pe::real elTerminalMinPairGap(const std::vector<pe::Vec3>& positions, pe::real diameter) {
  if (positions.size() < 2) {
    return std::numeric_limits<pe::real>::max();
  }

  pe::real minGap = std::numeric_limits<pe::real>::max();
  for (std::size_t i = 0; i < positions.size(); ++i) {
    for (std::size_t j = i + 1; j < positions.size(); ++j) {
      const pe::Vec3 d = positions[i] - positions[j];
      minGap = std::min(minGap, d.length() - diameter);
    }
  }
  return minGap;
}

std::vector<pe::Vec3> elTerminalRandomSeeds(pe::real xmin, pe::real xmax,
                                            pe::real ymin, pe::real ymax,
                                            pe::real zmin, pe::real zmax,
                                            pe::real radius, pe::real seedMinGap,
                                            pe::real volumeFraction,
                                            std::size_t seed) {
  using namespace pe;

  const real diameter = real(2) * radius;
  const real spacing = diameter + seedMinGap;
  const real inset = radius + real(0.5) * seedMinGap;
  const real x0 = xmin + inset;
  const real x1 = xmax - inset;
  const real y0 = ymin + inset;
  const real y1 = ymax - inset;
  const real z0 = zmin + inset;
  const real z1 = zmax - inset;
  if (x0 >= x1 || y0 >= y1 || z0 >= z1) {
    throw std::runtime_error("random seeding domain is smaller than particle diameter plus seed gap");
  }

  const real domainVolume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
  const real particleVolume = real(4) * std::acos(real(-1)) * radius * radius * radius / real(3);
  const std::size_t targetCount = static_cast<std::size_t>(
    std::llround(std::max(real(0), volumeFraction) * domainVolume / particleVolume));
  if (targetCount == 0) {
    return std::vector<Vec3>();
  }

  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));
  std::uniform_real_distribution<real> rx(x0, x1);
  std::uniform_real_distribution<real> ry(y0, y1);
  std::uniform_real_distribution<real> rz(z0, z1);

  std::vector<Vec3> positions;
  positions.reserve(targetCount);
  const std::size_t maxAttempts = std::max<std::size_t>(100000, targetCount * 5000);
  std::size_t attempts = 0;
  while (positions.size() < targetCount && attempts < maxAttempts) {
    ++attempts;
    const Vec3 pos(rx(rng), ry(rng), rz(rng));
    bool accepted = true;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const Vec3 d = pos - positions[i];
      if (d.length() < spacing) {
        accepted = false;
        break;
      }
    }
    if (accepted) {
      positions.push_back(pos);
    }
  }

  if (positions.size() != targetCount) {
    throw std::runtime_error("random seeding could not place all requested particles");
  }
  return positions;
}

} // namespace

//*************************************************************************************************
/*!\brief PE setup for the unresolved Euler-Lagrange single-particle terminal-velocity case.
 *
 * Dedicated setup for the Tier-2 terminal-velocity verification (q2p1_el_pipeflow). Unlike
 * setupELFrozenTrace this routine:
 *   - creates free mobile spheres from particles.xyz or deterministic random seeding,
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

  const real seedMinGap = config.getSeedMinGap() >= real(0)
                            ? config.getSeedMinGap()
                            : real(0.2) * sphereRadius;
  const std::string seedMode = elTerminalLower(config.getSeedMode());
  const real diameter = real(2) * sphereRadius;

  MaterialID sphereMaterial = createMaterial("el_tv_particle", rhoParticle,
                                             config.getRestitution(),
                                             config.getStaticFriction(),
                                             config.getDynamicFriction(),
                                             0.2, 80, 100, 10, 11);

  std::vector<Vec3> spherePositions;
  if (seedMode == "file") {
    spherePositions = readVectorsFromFile(config.getXyzFilePath().string());
    if (spherePositions.empty()) {
      pe_EXCLUSIVE_SECTION(0) {
        std::cerr << "EL terminal-velocity setup read no positions from "
                  << config.getXyzFilePath().string() << ".\n";
      }
      MPI_Abort(cartcomm, 1);
    }
  } else if (seedMode == "random") {
    int cartRank = 0;
    MPI_Comm_rank(cartcomm, &cartRank);
    std::size_t count = 0;
    std::vector<double> packed;
    if (cartRank == 0) {
      try {
        spherePositions = elTerminalRandomSeeds(xmin, xmax, ymin, ymax, zmin, zmax,
                                                sphereRadius, seedMinGap,
                                                config.getVolumeFraction(),
                                                config.getSeed());
      } catch (const std::exception& ex) {
        std::cerr << "EL terminal-velocity random seeding failed: " << ex.what() << "\n";
        MPI_Abort(cartcomm, 1);
      }
      count = spherePositions.size();
    }
    unsigned long bcastCount = static_cast<unsigned long>(count);
    MPI_Bcast(&bcastCount, 1, MPI_UNSIGNED_LONG, 0, cartcomm);
    spherePositions.resize(static_cast<std::size_t>(bcastCount));
    packed.resize(3 * static_cast<std::size_t>(bcastCount));
    if (cartRank == 0) {
      for (std::size_t i = 0; i < spherePositions.size(); ++i) {
        packed[3*i + 0] = spherePositions[i][0];
        packed[3*i + 1] = spherePositions[i][1];
        packed[3*i + 2] = spherePositions[i][2];
      }
    }
    if (!packed.empty()) {
      MPI_Bcast(&packed[0], static_cast<int>(packed.size()), MPI_DOUBLE, 0, cartcomm);
    }
    if (cartRank != 0) {
      for (std::size_t i = 0; i < spherePositions.size(); ++i) {
        spherePositions[i] = Vec3(packed[3*i + 0], packed[3*i + 1], packed[3*i + 2]);
      }
    }
  } else {
    pe_EXCLUSIVE_SECTION(0) {
      std::cerr << "EL terminal-velocity setup invalid seedMode_: "
                << config.getSeedMode() << " (expected file or random).\n";
    }
    MPI_Abort(cartcomm, 1);
  }

  int createdLocal = 0;
  for (std::size_t i = 0; i < spherePositions.size(); ++i) {
    const Vec3 pos = spherePositions[i];
    if (world->ownsPoint(pos)) {
      SphereID sphere = createSphere(static_cast<int>(i), pos, sphereRadius, sphereMaterial, true);
      sphere->setLinearVel(initialParticleVelocity);
      ++createdLocal;
    }
  }

  world->synchronize();

  unsigned long localParticles  = static_cast<unsigned long>(createdLocal);
  unsigned long globalParticles = 0;
  MPI_Reduce(&localParticles, &globalParticles, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  const real minPairGap = elTerminalMinPairGap(spherePositions, diameter);
  const real domainVolume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
  const real particleVolume = real(4) * std::acos(real(-1)) * sphereRadius * sphereRadius *
                              sphereRadius / real(3);
  const real achievedPhi = spherePositions.empty()
                             ? real(0)
                             : static_cast<real>(spherePositions.size()) * particleVolume / domainVolume;

  pe_EXCLUSIVE_SECTION(0) {
    if (globalParticles != spherePositions.size()) {
      std::cerr << "EL terminal-velocity setup: expected " << spherePositions.size()
                << " spheres, created " << globalParticles
                << " (check that seed positions lie inside the domain).\n";
      MPI_Abort(cartcomm, 1);
    } else if (spherePositions.size() > 1 && minPairGap + real(1e-12) < seedMinGap) {
      std::cerr << "EL terminal-velocity setup: minimum seed gap " << minPairGap
                << " is below seedMinGap_ " << seedMinGap << ".\n";
      MPI_Abort(cartcomm, 1);
    } else if (seedMode == "random" &&
               std::abs(achievedPhi - config.getVolumeFraction()) >
                 real(0.02) * std::max(config.getVolumeFraction(), real(1e-30))) {
      std::cerr << "EL terminal-velocity setup: achieved phi " << achievedPhi
                << " differs from requested volumeFraction_ " << config.getVolumeFraction()
                << " by more than 2%.\n";
      MPI_Abort(cartcomm, 1);
    } else {
      std::cout << "EL terminal-velocity setup: " << globalParticles
                << " free sphere(s) created (r=" << sphereRadius
                << ", rho=" << rhoParticle << ", seedMode=" << seedMode
                << ", seedMinGap=" << seedMinGap << ", achieved phi=" << achievedPhi
                << "), PE decomposition " << px << "x" << py << "x" << pz << ".\n";
    }
  }
}
//*************************************************************************************************

#endif
