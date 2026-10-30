//=================================================================================================
/*!\file pe/interface/setup/dns_drag.h
 *  \brief DNS drag: fixed or free sphere/ellipsoid arrays in the unit cube (serial mode only).
 *
 *  Entry point: commf2c_dns_drag_(); the parallel variant aborts with a message.
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_DNS_DRAG_H_
#define _PE_INTERFACE_SETUP_DNS_DRAG_H_

// No parallel setup: the parallel entry point aborts with a message.

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief DNS drag setup for serial mode: periodic unit-cube sphere array
 *
 * Creates a monodisperse simple-cubic array of fixed spheres in a unit cube.
 * The number of spheres is derived from volume fraction and radius settings.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupDNSDragSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto& config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);

  TimeStep::stepsize(config.getStepsize());

  // D6.1: optional non-spherical body for the xyz-file path. The Kroupa
  // lubrication model is sphere-only (pair and wall resistances are built on
  // sphere radii), so a non-sphere body with lubrication enabled is refused
  // loudly rather than silently applying wrong-geometry corrections.
  const bool isEllipsoid = (config.getParticleShape() == "ellipsoid");
  if (isEllipsoid && config.getLubricationEnabled()) {
    throw std::runtime_error(
        "setupDNSDragSerial: lubricationEnabled_ is true but particleShape_ is 'ellipsoid' - "
        "the lubrication model is sphere-only; disable lubrication for non-spherical bodies");
  }

  // Optional per-axis angular lock (world frame) for the rotationOnly / free paths; refused
  // loudly under fixed and translationOnly so it can never be a silent no-op. Default (1,1,1).
  // particleMotion_: fixed | rotationOnly | translationOnly | free (D6.1/D6.2/D6.4).
  checkAngularDofMaskRequiresRotationOnly(config);
  const std::string& particleMotion = config.getParticleMotion();
  const Vec3 angularDofMask = config.getAngularDofMask();
  const Vec3 effectiveAngularMask = effectiveAngularDofMask(config);

  const real radius = config.getBenchRadius();
  if (radius <= 0.0 && !isEllipsoid) {
    throw std::runtime_error("setupDNSDragSerial: benchRadius must be > 0");
  }

  const real domainLength = 1.0;
  const real domainVolume = domainLength * domainLength * domainLength;
  const real diameter = 2.0 * radius;
  const real wallSurfaceGap = diameter;
  const real particleSurfaceGap = diameter;
  const real wallCenterOffset = radius + wallSurfaceGap;
  const real minCenterSpacing = diameter + particleSurfaceGap;

  const real targetVF = std::max(real(0.0), config.getVolumeFraction());
  const real sphereVol = (4.0 / 3.0) * M_PI * std::pow(radius, 3);

  // Opt-in random-array path (D3.1): when xyzFilePath_ is set, create one
  // FIXED sphere per line of the file and skip the legacy lattice below.
  // Overlap/periodic-image validity is the generator's responsibility.
  const std::string dragXyzPath = config.getXyzFilePath().string();
  if (!dragXyzPath.empty()) {
    // Material first, on both paths: checkpointed bodies index the material table.
    MaterialID arrayMaterial = createMaterial(
        "dns_drag_particle", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);
    if (config.getResume()) {
      // Segmented-run resume (D6.2 chain): the pe checkpoint written by the driver at its
      // dump instant carries position, orientation, linear and angular velocity of the body,
      // so the orbit continues instead of restarting from particleAxis_ with omega = 0.
      // DOF locks are setup state, not body state: re-apply them after the load.
      resumeFromConfiguredCheckpoint(config);
      int restored = 0;
      for (auto it = world->begin(); it != world->end(); ++it) {
        if (it->getType() != sphereType && it->getType() != ellipsoidType) continue;
        applyParticleMotionLocks(*it, particleMotion, angularDofMask);
        ++restored;
      }
      if (restored == 0) {
        throw std::runtime_error("setupDNSDragSerial: resume checkpoint '" +
                                 config.getResumeCheckpointFile() + "' holds no bodies");
      }
      if (isRepresentative) {
        std::cout << "\n--DNS DRAG SETUP (RESUMED from pe checkpoint)----------------\n"
                  << " Checkpoint                              = " << config.getResumeCheckpointFile() << "\n"
                  << " Bodies restored                         = " << restored << "\n"
                  << " Motion                                  = " << particleMotion << "\n"
                  << " Angular DOF mask                        = (" << effectiveAngularMask[0] << ", "
                  << effectiveAngularMask[1] << ", " << effectiveAngularMask[2] << ")\n"
                  << "-------------------------------------------------------------\n"
                  << std::endl;
      }
      return;
    }
    std::vector<Vec3> positions = readVectorsFromFile(dragXyzPath);
    if (positions.empty()) {
      throw std::runtime_error("setupDNSDragSerial: xyzFilePath_ set but no positions read from '" +
                               dragXyzPath + "'");
    }
    if (isEllipsoid) {
      // D6.1 Oberbeck path: one FIXED ellipsoid per xyz line, semi-axes from
      // semiAxes_ (a along body-frame x), a-axis rotated onto particleAxis_.
      const Vec3 axes = config.getSemiAxes();
      if (axes[0] <= 0.0 || axes[1] <= 0.0 || axes[2] <= 0.0) {
        throw std::runtime_error(
            "setupDNSDragSerial: particleShape_ 'ellipsoid' requires semiAxes_ = [a,b,c] > 0");
      }
      Vec3 dir = config.getParticleAxis();
      const real dirLen = dir.length();
      if (dirLen < real(1e-12)) {
        throw std::runtime_error("setupDNSDragSerial: particleAxis_ must be a nonzero vector");
      }
      dir /= dirLen;
      const Vec3 bodyX(1.0, 0.0, 0.0);
      const real cosang = trans(bodyX) * dir;
      int eidx = 0;
      for (const auto& pos : positions) {
        EllipsoidID ell =
            createEllipsoid(++eidx, pos, axes[0], axes[1], axes[2], arrayMaterial, true);
        if (cosang < real(1) - real(1e-12)) {
          if (cosang > real(-1) + real(1e-12)) {
            Vec3 rotAxis = bodyX % dir;
            rotAxis /= rotAxis.length();
            ell->rotate(rotAxis, std::acos(cosang));
          } else {
            ell->rotate(Vec3(0.0, 0.0, 1.0), M_PI);  // anti-parallel: any perpendicular axis
          }
        }
        // fixed (D6.1) | rotationOnly (D6.2: the FBM torque drives the Jeffery orbit) |
        // translationOnly (D6.4 tier S: Happel-Brenner settling at locked orientation) |
        // free (D6.4 G0 / tier I: six DOFs, buoyancy via the solver's ellipsoid branch).
        applyParticleMotionLocks(ell, particleMotion, angularDofMask);
        ell->setLinearVel(0.0, 0.0, 0.0);
        ell->setAngularVel(0.0, 0.0, 0.0);
      }
      const real ellVol = (4.0 / 3.0) * M_PI * axes[0] * axes[1] * axes[2];
      if (isRepresentative) {
        std::cout << "\n--DNS DRAG SETUP (ellipsoid array from file)-----------------\n"
                  << " Position file                           = " << dragXyzPath << "\n"
                  << " Number of ellipsoids                    = " << positions.size() << "\n"
                  << " Semi-axes (a,b,c)                       = " << axes << "\n"
                  << " a-axis direction (world)                = " << dir << "\n"
                  << " Achieved volume fraction                = "
                  << positions.size() * ellVol / domainVolume << "\n"
                  << " Motion                                  = " << particleMotion << "\n"
                  << " Angular DOF mask                        = (" << effectiveAngularMask[0] << ", "
                  << effectiveAngularMask[1] << ", " << effectiveAngularMask[2] << ")\n"
                  << "-------------------------------------------------------------\n"
                  << std::endl;
      }
      return;
    }
    int aidx = 0;
    for (const auto& pos : positions) {
      SphereID sphere = createSphere(++aidx, pos, radius, arrayMaterial, true);
      // Same four modes as the ellipsoid path (rotationOnly = D6.2 V0 spin control,
      // free = D6.4 sphere control).
      applyParticleMotionLocks(sphere, particleMotion, angularDofMask);
      sphere->setLinearVel(0.0, 0.0, 0.0);
      sphere->setAngularVel(0.0, 0.0, 0.0);
    }
    if (isRepresentative) {
      std::cout << "\n--DNS DRAG SETUP (random array from file)--------------------\n"
                << " Position file                           = " << dragXyzPath << "\n"
                << " Number of spheres                       = " << positions.size() << "\n"
                << " Radius                                  = " << radius << "\n"
                << " Achieved volume fraction                = "
                << positions.size() * sphereVol / domainVolume << "\n"
                << " Motion                                  = " << particleMotion << "\n"
                << " Angular DOF mask                        = (" << effectiveAngularMask[0] << ", "
                << effectiveAngularMask[1] << ", " << effectiveAngularMask[2] << ")\n"
                << "-------------------------------------------------------------\n"
                << std::endl;
    }
    return;
  }

  if (isEllipsoid) {
    throw std::runtime_error(
        "setupDNSDragSerial: particleShape_ 'ellipsoid' requires xyzFilePath_ "
        "(the legacy lattice path is sphere-only)");
  }

  int targetCount = 1;
  if (targetVF > 0.0 && sphereVol > 0.0) {
    targetCount = std::max(1, static_cast<int>(std::round(targetVF * domainVolume / sphereVol)));
  }

  int nPerDim = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(targetCount))));
  nPerDim = std::max(1, nPerDim);

  const real availableSpan = domainLength - 2.0 * wallCenterOffset;
  int nPerDimMax = 0;
  if (availableSpan >= 0.0) {
    nPerDimMax = 1 + static_cast<int>(std::floor(availableSpan / minCenterSpacing));
  }
  if (nPerDimMax < 1) {
    throw std::runtime_error(
        "setupDNSDragSerial: spheres do not fit into [0,1]^3 with one-diameter wall clearance");
  }
  nPerDim = std::min(nPerDim, nPerDimMax);

  const real spacing = (nPerDim > 1) ? availableSpan / static_cast<real>(nPerDim - 1) : 0.0;
  MaterialID particleMaterial = createMaterial(
      "dns_drag_particle", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  int idx = 0;
  int particlesCreated = 0;
  for (int k = 0; k < nPerDim; ++k) {
    for (int j = 0; j < nPerDim; ++j) {
      for (int i = 0; i < nPerDim; ++i) {
        Vec3 pos(wallCenterOffset + static_cast<real>(i) * spacing,
                 wallCenterOffset + static_cast<real>(j) * spacing,
                 wallCenterOffset + static_cast<real>(k) * spacing);
        SphereID sphere = createSphere(++idx, pos, radius, particleMaterial, true);
        sphere->setFixed(true);
        sphere->setLinearVel(0.0, 0.0, 0.0);
        sphere->setAngularVel(0.0, 0.0, 0.0);
        ++particlesCreated;
      }
    }
  }

  const real actualVF = particlesCreated * sphereVol / domainVolume;

  if (isRepresentative) {
    std::cout << "\n--DNS DRAG SETUP---------------------------------------------\n"
              << " Fluid viscosity                         = " << config.getFluidViscosity() << "\n"
              << " Fluid density                           = " << config.getFluidDensity() << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Domain length                           = " << domainLength << "\n"
              << " Radius                                  = " << radius << "\n"
              << " Target volume fraction                  = " << targetVF << "\n"
              << " Wall surface gap                        = " << wallSurfaceGap << "\n"
              << " Particle surface gap                    = " << particleSurfaceGap << "\n"
              << " Spheres per dimension                   = " << nPerDim << "\n"
              << " Center spacing                          = " << spacing << "\n"
              << " Number of spheres                       = " << particlesCreated << "\n"
              << " Achieved volume fraction                = " << actualVF << "\n"
              << "-------------------------------------------------------------\n"
              << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_DNS_DRAG_H_
