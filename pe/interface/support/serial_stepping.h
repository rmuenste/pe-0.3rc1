//=================================================================================================
/*!\file pe/interface/support/serial_stepping.h
 *  \brief Serial-mode (PE_SERIAL_MODE) time stepping: stepSimulationSerial() and
 *         stepELFrozenTraceSerial(), reached from Fortran through step_simulation_() and
 *         step_el_frozen_trace_() (src/interface/cfd_stepping.cpp).
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SUPPORT_SERIAL_STEPPING_H_
#define _PE_INTERFACE_SUPPORT_SERIAL_STEPPING_H_

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief Serial PE time stepping function with substepping support
 *
 * This function performs one time step of the PE simulation in serial mode with
 * configurable substepping. Substepping allows for more stable integration by
 * breaking each main timestep into multiple smaller substeps.
 *
 * Unlike the MPI version, this does not use MPI communications or domain decomposition.
 * Each CFD domain runs its own independent PE instance.
 *
 * Substepping configuration:
 * - Number of substeps is read from SimulationConfig::getSubsteps() (default: 1)
 * - VTK output is only triggered on the final substep to maintain output frequency
 * - Particle diagnostics are printed once per main timestep
 * - Checkpointer (if enabled) is triggered once per main timestep
 */
inline SerialStepContext makeSerialStepContext(WorldID world,
                                               SimulationConfig &config,
                                               const bool isRepresentative,
                                               const int timestep,
                                               const real fullStepSize,
                                               const int substeps) {
  SerialStepContext ctx = {world, config, isRepresentative, timestep, fullStepSize, substeps};
  return ctx;
}

inline void applySerialFluidForces(const SerialStepContext &serialStepCtx,
                                   const real alpha,
                                   const bool triggerFeatureHooks) {
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;

    Vec3 v_before = body->getLinearVel();
    Vec3 force = body->getForce();
    real forceMag = force.length();
    real mass = body->getMass();

    body->applyFluidForces(serialStepCtx.fullStepSize, alpha);

    if (triggerFeatureHooks) {
      Vec3 v_after = body->getLinearVel();
      Vec3 deltaV = v_after - v_before;
      real deltaVMag = deltaV.length();
      FluidForceApplicationContext forceCtx = {
          body, v_before, force, forceMag, mass, v_after, deltaV, deltaVMag};
      serialStepFeatureSet().afterFluidForceApplication(serialStepCtx, forceCtx);
    }
  }
}

inline void advanceSerialSubsteps(WorldID world,
                                  const real fullStepSize,
                                  const int substeps) {
  const real substepSize = fullStepSize / static_cast<real>(substeps);

  TimeStep::stepsize(substepSize);
  for (int istep = 0; istep < substeps; ++istep) {
    world->simulationStep(substepSize);
  }
  TimeStep::stepsize(fullStepSize);
}

inline void emitRepresentativeSerialParticleDiagnostics(WorldID world,
                                                        const int timestep,
                                                        const real fullStepSize,
                                                        const int substeps) {
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType ||
       body->getType() == capsuleType ||
       body->getType() == ellipsoidType ||
       body->getType() == cylinderType ||
       body->getType() == triangleMeshType) {

      const Vec3 vel = body->getLinearVel();
      const Vec3 ang = body->getAngularVel();
#ifdef PE_SERIAL_VERBOSE_PARTICLE_OUTPUT
      std::cout << "==Single Particle Data========================================================" << std::endl;

      std::cout << "Position: " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                   body->getPosition()[0] << " " <<
                                   body->getPosition()[1] << " " <<
                                   body->getPosition()[2] << std::endl;
      std::cout << "Velocity: " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                   vel[0] << " " <<
                                   vel[1] << " " <<
                                   vel[2] << std::endl;
      std::cout << "Omega:    " << body->getSystemID() << " " << timestep * fullStepSize << " " <<
                                   ang[0] << " " <<
                                   ang[1] << " " <<
                                   ang[2] << std::endl;
      std::cout << "We are using " << substeps << " substeps per CFD step." << std::endl;
#endif
    }
  }
}

inline int countELFrozenTraceParticles(WorldID world) {
  int particleCount = 0;
  for (auto it = world->begin(); it != world->end(); ++it) {
    if ((*it)->getType() == sphereType) {
      ++particleCount;
    }
  }
  return particleCount;
}

inline bool isFiniteVec3(const Vec3 &value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

inline int cullELFrozenTraceParticles(WorldID world,
                                      const Vec3 &domainMin,
                                      const Vec3 &domainMax) {
  int removed = 0;
  const real epsX = std::max(real(1e-12), std::abs(domainMax[0]) * real(1e-12));
  const real epsY = std::max(real(1e-12), std::abs(domainMax[1]) * real(1e-12));
  const real epsZ = std::max(real(1e-12), std::abs(domainMax[2]) * real(1e-12));
  for (auto it = world->begin(); it != world->end();) {
    BodyID body = *it;
    if (body->getType() != sphereType) {
      ++it;
      continue;
    }

    const Vec3 pos = body->getPosition();
    const Vec3 vel = body->getLinearVel();
    const Vec3 ang = body->getAngularVel();

    const bool invalidState = !isFiniteVec3(pos) || !isFiniteVec3(vel) || !isFiniteVec3(ang);
    const bool outsideClosedFaces =
        (pos[0] < domainMin[0] - epsX) ||
        (pos[1] < domainMin[1] - epsY) || (pos[1] > domainMax[1] + epsY) ||
        (pos[2] < domainMin[2] - epsZ) || (pos[2] > domainMax[2] + epsZ);
    const bool outsideOpenOutflow = (pos[0] > domainMax[0] + epsX);

    if (invalidState || outsideClosedFaces || outsideOpenOutflow) {
      it = world->destroy(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

inline void stepSimulationSerial() {
  WorldID world = theWorld();
  auto &config = SimulationConfig::getInstance();
  const bool isRepresentative = (config.getCfdRank() == 1);

  static int timestep = 0;

  if (timestep == 0) {
    pe_LOG_INFO_SECTION( log ) {
      log << "PE Logging initialized for CFD rank " << config.getCfdRank() << "\n";
      log << "This log file should be named: pe" << config.getCfdRank() << ".log\n";
    }
  }

  const int substeps = config.getSubsteps();
  const real fullStepSize = config.getStepsize();
  SerialStepContext serialStepCtx = makeSerialStepContext(
      world, config, isRepresentative, timestep, fullStepSize, substeps);

  const real alpha = real(0.35);
  applySerialFluidForces(serialStepCtx, alpha, true);
  advanceSerialSubsteps(world, fullStepSize, substeps);

  serialStepFeatureSet().afterMainStep(serialStepCtx);

  if (isRepresentative) {
    emitRepresentativeSerialParticleDiagnostics(world, timestep, fullStepSize, substeps);
  }

  timestep++;
}

inline void stepELFrozenTraceSerial() {
  WorldID world = theWorld();
  auto &config = SimulationConfig::getInstance();
  const bool isRepresentative = (config.getCfdRank() == 1);

  static int timestep = 0;

  const int substeps = config.getSubsteps();
  const real fullStepSize = config.getStepsize();
  const real alpha = real(0.35);
  const Vec3 domainMin = config.getCfdDomainMin();
  const Vec3 domainMax = config.getCfdDomainMax();
  const int particlesBefore = countELFrozenTraceParticles(world);

  SerialStepContext serialStepCtx = makeSerialStepContext(
      world, config, isRepresentative, timestep, fullStepSize, substeps);

  applySerialFluidForces(serialStepCtx, alpha, false);
  advanceSerialSubsteps(world, fullStepSize, substeps);

  const int particlesRemoved = cullELFrozenTraceParticles(world, domainMin, domainMax);
  const int particlesAfter = countELFrozenTraceParticles(world);

  if (isRepresentative) {
    std::cout << "Frozen-field PE step: timestep=" << timestep
              << " | particles before/after = " << particlesBefore
              << " / " << particlesAfter
              << " | outflow removed = " << particlesRemoved
              << " | substeps = " << substeps
              << std::endl;
  }

  timestep++;
}

}  // namespace pe

#endif  // _PE_INTERFACE_SUPPORT_SERIAL_STEPPING_H_
