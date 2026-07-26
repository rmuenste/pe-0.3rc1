#ifndef _PE_SETUP_EL_TERMINAL_VELOCITY_H_
#define _PE_SETUP_EL_TERMINAL_VELOCITY_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <pe/config/SimulationConfig.h>
#include <pe/interface/decompose.h>
#include <pe/interface/geometry_utils.h>
#include <pe/core/lubrication/Params.h>

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

int elTerminalAxisIndex(const std::string& axis) {
  const std::string a = elTerminalLower(axis);
  if (a == "x") return 0;
  if (a == "y") return 1;
  if (a == "z") return 2;
  throw std::runtime_error("seedCylinderAxis_ must be x, y, or z");
}

pe::real elTerminalSeedDomainVolume(pe::real xmin, pe::real xmax,
                                    pe::real ymin, pe::real ymax,
                                    pe::real zmin, pe::real zmax,
                                    const std::string& seedDomain,
                                    const pe::Vec3& cylinderCenter,
                                    pe::real cylinderRadius,
                                    const std::string& cylinderAxis) {
  using namespace pe;

  const std::string domain = elTerminalLower(seedDomain);
  if (domain == "box") {
    return (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
  }
  if (domain != "cylinder") {
    throw std::runtime_error("seedDomain_ must be box or cylinder");
  }
  if (cylinderRadius <= real(0)) {
    throw std::runtime_error("seedCylinderRadius_ must be positive for seedDomain_=cylinder");
  }
  const int axis = elTerminalAxisIndex(cylinderAxis);
  const real length = axis == 0 ? xmax - xmin : axis == 1 ? ymax - ymin : zmax - zmin;
  (void)cylinderCenter;
  return std::acos(real(-1)) * cylinderRadius * cylinderRadius * length;
}

bool elTerminalInsideSeedDomain(const pe::Vec3& pos,
                                const std::string& seedDomain,
                                const pe::Vec3& cylinderCenter,
                                pe::real cylinderRadius,
                                const std::string& cylinderAxis,
                                pe::real inset) {
  using namespace pe;

  const std::string domain = elTerminalLower(seedDomain);
  if (domain == "box") {
    return true;
  }
  if (domain != "cylinder") {
    throw std::runtime_error("seedDomain_ must be box or cylinder");
  }

  const int axis = elTerminalAxisIndex(cylinderAxis);
  const int a = (axis + 1) % 3;
  const int b = (axis + 2) % 3;
  const real da = pos[a] - cylinderCenter[a];
  const real db = pos[b] - cylinderCenter[b];
  const real radial = std::sqrt(da * da + db * db);
  return radial <= cylinderRadius - inset;
}

std::vector<pe::Vec3> elTerminalRandomSeeds(pe::real xmin, pe::real xmax,
                                            pe::real ymin, pe::real ymax,
                                            pe::real zmin, pe::real zmax,
                                            pe::real radius, pe::real seedMinGap,
                                            pe::real volumeFraction,
                                            std::size_t seed,
                                            const std::string& seedDomain,
                                            const pe::Vec3& seedCylinderCenter,
                                            pe::real seedCylinderRadius,
                                            const std::string& seedCylinderAxis) {
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

  const real domainVolume = elTerminalSeedDomainVolume(xmin, xmax, ymin, ymax, zmin, zmax,
                                                       seedDomain, seedCylinderCenter,
                                                       seedCylinderRadius, seedCylinderAxis);
  const real particleVolume = real(4) * std::acos(real(-1)) * radius * radius * radius / real(3);
  const std::size_t targetCount = static_cast<std::size_t>(
    std::llround(std::max(real(0), volumeFraction) * domainVolume / particleVolume));
  if (targetCount == 0) {
    return std::vector<Vec3>();
  }

  const int nx = static_cast<int>(std::floor((x1 - x0) / spacing)) + 1;
  const int ny = static_cast<int>(std::floor((y1 - y0) / spacing)) + 1;
  const int nz = static_cast<int>(std::floor((z1 - z0) / spacing)) + 1;
  if (nx <= 0 || ny <= 0 || nz <= 0) {
    throw std::runtime_error("random seeding has no admissible grid cells");
  }

  std::vector<Vec3> candidates;
  candidates.reserve(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) *
                     static_cast<std::size_t>(nz));
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        const Vec3 pos(x0 + static_cast<real>(ix) * spacing,
                       y0 + static_cast<real>(iy) * spacing,
                       z0 + static_cast<real>(iz) * spacing);
        if (elTerminalInsideSeedDomain(pos, seedDomain, seedCylinderCenter,
                                       seedCylinderRadius, seedCylinderAxis, inset)) {
          candidates.push_back(pos);
        }
      }
    }
  }

  if (candidates.size() < targetCount) {
    const real maxPhi = static_cast<real>(candidates.size()) * particleVolume / domainVolume;
    throw std::runtime_error("random seeding grid cannot reach requested volumeFraction_; "
                             "maximum grid phi is " + std::to_string(maxPhi));
  }

  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));
  std::shuffle(candidates.begin(), candidates.end(), rng);
  std::vector<Vec3> positions(candidates.begin(), candidates.begin() + targetCount);
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
 *     QBOX9Z3 mesh this means processesZ_=3, so PE and CFD own the same spatial slabs.
 *     With periodicX_/periodicY_/periodicZ_ (json) the decomposition switches to the
 *     matching decomposePeriodic* variant (supported combinations: X, Z, XY, XYZ);
 *     wrap-around connections shift bodies by +-l_axis across the periodic planes
 *     (matching the CFD SimPar@PeriodicAxis setup); each periodic axis requires
 *     processes >= 3 on that axis and the MPI PE build (PE_SERIAL_MODE has no
 *     periodic connectivity),
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

  // Pairwise lubrication (EL solver): widen the shadow-copy overlap test so
  // cross-boundary pairs within the surface-gap cutoff are visible to the
  // designated treating rank. The full-visibility margin is
  // sphereRadius + cutoff: for a pair (A owned by r1, B owned by r2) with
  // gap < cutoff, r1 sees B iff dist(B_center, r1_box) <= R_B + margin, and
  // that distance can reach R_A + R_B + cutoff when A's center sits on r1's
  // boundary. The margin is CLAMPED so the total shadow reach
  // (radius + margin) stays below the thinnest decomposed subdomain extent:
  // pe cannot register shadow copies beyond direct neighbors ("Registering
  // distant processes is not yet implemented"). Under the designated-treater
  // relay a clamped margin is momentum-safe — a pair the treater cannot see
  // is skipped for a substep, never applied one-sided; only extremal
  // near-cutoff pairs are affected. No-op (margin 0) when disabled.
  if (config.getLubricationEnabled()) {
    real lubMargin = config.getLubricationCutoff() + config.getBenchRadius();
    real minExt = std::numeric_limits<real>::max();
    if (config.getProcessesX() > 1)
      minExt = std::min(minExt, (xmax - xmin) / config.getProcessesX());
    if (config.getProcessesY() > 1)
      minExt = std::min(minExt, (ymax - ymin) / config.getProcessesY());
    if (config.getProcessesZ() > 1)
      minExt = std::min(minExt, (zmax - zmin) / config.getProcessesZ());
    const real reachCap = real(0.99) * minExt - config.getBenchRadius();
    if (lubMargin > reachCap) {
      std::cout << "EL lubrication: shadow margin clamped " << lubMargin
                << " -> " << reachCap << " (subdomain extent " << minExt
                << "); near-cutoff cross-rank pairs may be skipped.\n";
      lubMargin = reachCap;
    }
    pe::lubrication::setShadowCopyMargin(std::max(lubMargin, real(0)));
  }
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

  // Per-axis periodic wrap (json periodicX_/periodicY_/periodicZ_). The
  // decompose.h library provides the combinations none / X / XY / XYZ / Z;
  // other combinations have no decomposition function yet.
  const bool periodicX = config.getPeriodicX();
  const bool periodicY = config.getPeriodicY();
  const bool periodicZ = config.getPeriodicZ();
  {
    const int  paxis[3] = {px, py, pz};
    const bool pflag[3] = {periodicX, periodicY, periodicZ};
    const char axname[3] = {'x', 'y', 'z'};
    for (int ax = 0; ax < 3; ++ax) {
      if (pflag[ax] && paxis[ax] < 3) {
        pe_EXCLUSIVE_SECTION(0) {
          std::cerr << "EL terminal-velocity setup: periodic" << axname[ax]
                    << "_ requires processes" << axname[ax]
                    << "_ >= 3 (distinct wrap neighbors), got " << paxis[ax] << ".\n";
        }
        MPI_Abort(ex0, 1);
      }
    }
    const bool supported =
      (!periodicX && !periodicY) ||                 // none or Z-only
      (periodicX && periodicY) ||                   // XY or XYZ
      (periodicX && !periodicY && !periodicZ);      // X-only
    if (!supported) {
      pe_EXCLUSIVE_SECTION(0) {
        std::cerr << "EL terminal-velocity setup: unsupported periodic axis "
                  << "combination (available: none, X, Z, XY, XYZ).\n";
      }
      MPI_Abort(ex0, 1);
    }
  }

  int dims[]    = {px, py, pz};
  int periods[] = {periodicX, periodicY, periodicZ};
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
  if (periodicX && periodicY && periodicZ) {
    decomposePeriodic3D(center, xmin, ymin, zmin, dx, dy, dz,
                        xmax - xmin, ymax - ymin, zmax - zmin, px, py, pz);
    pe_EXCLUSIVE_SECTION(0) {
      std::cout << "EL terminal-velocity setup: fully periodic PE decomposition, l=("
                << (xmax - xmin) << "," << (ymax - ymin) << "," << (zmax - zmin) << ")\n";
    }
  } else if (periodicX && periodicY) {
    decomposePeriodicXY3D(center, xmin, ymin, zmin, dx, dy, dz,
                          xmax - xmin, ymax - ymin, zmax - zmin, px, py, pz);
    pe_EXCLUSIVE_SECTION(0) {
      std::cout << "EL terminal-velocity setup: xy-periodic PE decomposition\n";
    }
  } else if (periodicX) {
    decomposePeriodicX3D(center, xmin, ymin, zmin, dx, dy, dz,
                         xmax - xmin, ymax - ymin, zmax - zmin, px, py, pz);
    pe_EXCLUSIVE_SECTION(0) {
      std::cout << "EL terminal-velocity setup: x-periodic PE decomposition, lx="
                << (xmax - xmin) << "\n";
    }
  } else if (periodicZ) {
    decomposePeriodicZ3D(center, xmin, ymin, zmin, dx, dy, dz,
                         xmax - xmin, ymax - ymin, zmax - zmin, px, py, pz);
    pe_EXCLUSIVE_SECTION(0) {
      std::cout << "EL terminal-velocity setup: z-periodic PE decomposition, lz="
                << (zmax - zmin) << "\n";
    }
  } else {
    decomposeDomain(center, xmin, ymin, zmin, dx, dy, dz, px, py, pz);
  }
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
                                                config.getSeed(),
                                                config.getSeedDomain(),
                                                config.getSeedCylinderCenter(),
                                                config.getSeedCylinderRadius(),
                                                config.getSeedCylinderAxis());
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

  if (g_vtk) {
    vtk::activateWriter("./paraview", config.getVisspacing(), 0,
                        config.getTimesteps(), false, true);
  }

  unsigned long localParticles  = static_cast<unsigned long>(createdLocal);
  unsigned long globalParticles = 0;
  MPI_Reduce(&localParticles, &globalParticles, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
  const real minPairGap = elTerminalMinPairGap(spherePositions, diameter);
  const real domainVolume = elTerminalSeedDomainVolume(xmin, xmax, ymin, ymax, zmin, zmax,
                                                       config.getSeedDomain(),
                                                       config.getSeedCylinderCenter(),
                                                       config.getSeedCylinderRadius(),
                                                       config.getSeedCylinderAxis());
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
    } else if (spherePositions.size() > 1 && !config.getSeedAllowContact() &&
               minPairGap + real(1e-12) < seedMinGap) {
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
      for (std::size_t i = 0; i < spherePositions.size(); ++i) {
        std::cout << std::setprecision(17)
                  << "EL_SEED_POS id= " << i
                  << " x= " << spherePositions[i][0]
                  << " y= " << spherePositions[i][1]
                  << " z= " << spherePositions[i][2] << "\n";
      }
      std::cout << "EL terminal-velocity setup: " << globalParticles
                << " free sphere(s) created (r=" << sphereRadius
                << ", rho=" << rhoParticle << ", seedMode=" << seedMode
                << ", seedDomain=" << elTerminalLower(config.getSeedDomain())
                << ", seedMinGap=" << seedMinGap << ", achieved phi=" << achievedPhi
                << "), PE decomposition " << px << "x" << py << "x" << pz << ".\n";
    }
  }
}
//*************************************************************************************************

#endif
