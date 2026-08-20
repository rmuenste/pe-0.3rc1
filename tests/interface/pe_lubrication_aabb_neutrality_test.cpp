// AABB NEUTRALITY GATE: lubrication off must mean NO bounding-box inflation, silently or
// otherwise.
//
// This is a hard requirement, not a nice-to-have. Bounding-box size feeds the HashGrids
// broad phase: it selects the grid hierarchy level and therefore the candidate-pair
// enumeration order, which perturbs contact ordering in the sequential constraint solver.
// A stray padding term is thus a silent trajectory change in every DNS case, with no error
// message and no obviously wrong output.
//
// The history this guards against is real. Before the add-on refactor,
// SphereBase::calcBoundingBox added lubrication::getLubricationThreshold() unconditionally
// -- a hard-coded 1e-2 in ABSOLUTE units, applied to every solver, whether or not
// lubrication existed in the build. For a 1.5 mm benchmark sphere that is a 10 mm halo:
// padding several times the body radius, permanently on, invisible.
//
// What is asserted, in a dedicated process so the parameter store is pristine:
//   1. the shipped defaults really are neutral -- master switch off, security zone off;
//   2. aabbPadding() is exactly zero, for any radius, in that state;
//   3. a real sphere's AABB half-extent is EXACTLY radius + contactThreshold, and a real
//      plane's finite face is EXACTLY d + contactThreshold -- bitwise, not "close";
//   4. pushing a DEFAULT SimulationConfig through applyOptionalLubricationParams (the json
//      path every setup takes) does not change that;
//   5. the assertions are not vacuous: enabling lubrication does inflate, and disabling it
//      returns the box bitwise to the neutral value.
//
// Linked against pe_static_lubstage_fluid, i.e. pe_CONSTRAINT_SOLVER =
// pe::response::HardContactAndFluid -- the campaign solver, which is exactly the
// "HCAF + defaults + lubrication off" configuration the guarantee is about.

#include <pe/core.h>
#include <pe/core/Thresholds.h>
#include <pe/core/lubrication/Params.h>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/setup_optional_collision_params.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

using namespace pe;

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

const real R = real(0.0015);   // benchmark sphere radius: the case the 1e-2 halo ruined

// Half-extent of a sphere's AABB along x.
real halfExtent(ConstSphereID s) {
  return (s->getAABB()[3] - s->getAABB()[0]) * real(0.5);
}

}  // namespace

int main() {
  //---------------------------------------------------------------------------------
  // 1. Shipped defaults are neutral -- asserted BEFORE anything touches the store
  //---------------------------------------------------------------------------------
  expect(lubrication::isEnabled() == false,
         "default: lubrication master switch is OFF");
  expect(lubrication::getSecurityZone() == false,
         "default: no security-zone consumer is registered");
  expect(lubrication::contactGenerationEnabled() == false,
         "default: extended-range contact generation is OFF");

  // Zero padding for every radius, including the unregistered-plane case (radius 0)
  // and radii far outside the benchmark range.
  for (real r : {real(0), real(1e-6), R, real(1.0), real(1e3)}) {
    expect(lubrication::aabbPadding(r) == real(0),
           "default: aabbPadding is exactly zero for every radius");
  }

  std::printf("[defaults] isEnabled=%s securityZone=%s aabbPadding(R)=%.17g\n",
              lubrication::isEnabled() ? "true" : "false",
              lubrication::getSecurityZone() ? "true" : "false",
              (double)lubrication::aabbPadding(R));

  //---------------------------------------------------------------------------------
  // 2. Real bodies: AABBs carry contactThreshold and nothing else
  //---------------------------------------------------------------------------------
  WorldID world = theWorld();
  world->setGravity(0.0, 0.0, 0.0);
  world->setViscosity(0.1);
  world->setLiquidDensity(1.0);
  world->setDamping(1.0);

  MaterialID mat =
      createMaterial("neutrality", 2000.0, 0.0, 0.1, 0.05, 0.2, 300, 1e6, 1e3, 2e3);

  unsigned int id = 0;
  SphereID sphere = createSphere(id++, 0.0, 0.0, 0.0, R, mat);

  const real neutralSphere = R + contactThreshold;
  expect(halfExtent(sphere) == neutralSphere,
         "default: sphere AABB half-extent is exactly radius + contactThreshold");

  // The plane path pads by contactThreshold + aabbPadding(maxRegisteredSphereRadius).
  // The sphere above has already registered R, so a nonzero padding would show here.
  const real planeD = real(-0.05);
  PlaneID plane = createPlane(id++, 0.0, 0.0, 1.0, planeD, mat);
  plane->setFixed(true);
  const real neutralPlane = planeD + contactThreshold;
  expect(plane->getAABB()[5] == neutralPlane,
         "default: plane AABB finite face is exactly d + contactThreshold");

  std::printf("[defaults] sphere half-extent = %.17g (R + contactThreshold = %.17g)\n",
              (double)halfExtent(sphere), (double)neutralSphere);
  std::printf("[defaults] plane  face        = %.17g (d + contactThreshold = %.17g)\n",
              (double)plane->getAABB()[5], (double)neutralPlane);

  // Explicitly rule out the historical regression rather than only the general one.
  expect(std::fabs(halfExtent(sphere) - (R + contactThreshold + real(1e-2))) > real(1e-9),
         "default: the legacy unconditional 1e-2 halo is gone");

  //---------------------------------------------------------------------------------
  // 3. The json path is neutral too: a DEFAULT SimulationConfig must not inflate
  //---------------------------------------------------------------------------------
  {
    SimulationConfig& config = SimulationConfig::getInstance();
    expect(config.getLubricationEnabled() == false,
           "default SimulationConfig: lubricationEnabled_ is false");

    // This is the call every setup makes after loadFromFile. It must not arm anything.
    applyOptionalLubricationParams(*theCollisionSystem(), config);

    expect(lubrication::isEnabled() == false,
           "after applyOptionalLubricationParams: still OFF");
    expect(lubrication::aabbPadding(R) == real(0),
           "after applyOptionalLubricationParams: padding still exactly zero");

    // Force a recompute and confirm the box did not move.
    sphere->setPosition(0.0, 0.0, real(0.01));
    expect(halfExtent(sphere) == neutralSphere,
           "after applyOptionalLubricationParams: sphere AABB unchanged, bitwise");

    // The hysteresis half-widths must be zero with lubrication off, or the shared
    // detection path stops reducing to a plain `dist < contactThreshold` step.
    expect(lubrication::getContactHysteresisDelta() == real(0),
           "lubrication off: contact hysteresis half-width forced to zero");
    expect(lubrication::getLubricationHysteresisDelta() == real(0),
           "lubrication off: lubrication hysteresis half-width forced to zero");
  }

  //---------------------------------------------------------------------------------
  // 4. Non-vacuity: enabling really does inflate, disabling really does restore
  //---------------------------------------------------------------------------------
  {
    lubrication::setEnabled(true);
    lubrication::setAabbInflation(true);
    lubrication::setCutoffFactor(real(0.5));
    lubrication::setMeshClampFactor(real(0));

    expect(lubrication::aabbPadding(R) > real(0),
           "non-vacuity: padding becomes nonzero when lubrication is enabled");

    sphere->setPosition(0.0, 0.0, real(0.02));
    const real inflated = halfExtent(sphere);
    expect(inflated > neutralSphere,
           "non-vacuity: sphere AABB actually grows when lubrication is enabled");
    std::printf("[enabled ] sphere half-extent = %.17g (grew by %.17g = cutoffFactor*R)\n",
                (double)inflated, (double)(inflated - neutralSphere));

    // And the mesh clamp must bound it, not be ignored (D2.2 §3.4).
    lubrication::setMeshClampFactor(real(2));
    lubrication::setMeshDx(R / real(10));
    expect(lubrication::aabbPadding(R) == real(2) * (R / real(10)),
           "non-vacuity: mesh clamp bounds the padding when it is the smaller bound");
    lubrication::setMeshClampFactor(real(0));
    lubrication::setMeshDx(real(0));

    lubrication::setEnabled(false);
    sphere->setPosition(0.0, 0.0, real(0.03));
    expect(halfExtent(sphere) == neutralSphere,
           "non-vacuity: disabling restores the neutral AABB bitwise");
  }

  //---------------------------------------------------------------------------------
  // 5. The two padding consumers COMPOSE; neither may override the other
  //
  // A security-zone consumer (ShortRangeRepulsion) and lubrication are independent.
  // An early return for either one silently drops the other's pairs before fine
  // detection ever sees them -- under-inflation is not a safe failure here, it is a
  // missing interaction.
  //---------------------------------------------------------------------------------
  {
    const real rho = lubrication::getLubricationThreshold();   // SRR's absolute band

    // Security zone alone: absolute band, regardless of the master switch.
    lubrication::setEnabled(false);
    lubrication::setSecurityZone(true);
    expect(lubrication::aabbPadding(R) == rho,
           "compose: security zone alone gives the absolute band");

    // Security zone + lubrication with a LARGER relative band -> lubrication wins.
    lubrication::setEnabled(true);
    lubrication::setAabbInflation(true);
    lubrication::setCutoffFactor(real(0.5));
    const real bigR = real(10.0) * rho;             // 0.5 * bigR = 5*rho > rho
    expect(lubrication::aabbPadding(bigR) == real(0.5) * bigR,
           "compose: larger lubrication band is not clipped by the security zone");

    // Security zone + lubrication with a SMALLER relative band -> security zone wins.
    const real tinyR = rho / real(100);             // 0.5 * tinyR << rho
    expect(lubrication::aabbPadding(tinyR) == rho,
           "compose: security-zone band survives a smaller lubrication band");

    // And the mesh clamp must not be able to shrink the security-zone band either.
    lubrication::setMeshClampFactor(real(2));
    lubrication::setMeshDx(rho / real(1000));
    expect(lubrication::aabbPadding(bigR) == rho,
           "compose: mesh clamp bounds lubrication but never the security zone");

    lubrication::setMeshClampFactor(real(0));
    lubrication::setMeshDx(real(0));
    lubrication::setSecurityZone(false);
    lubrication::setEnabled(false);
    expect(lubrication::aabbPadding(R) == real(0),
           "compose: everything off is still exactly zero");
  }

  if (failures == 0) {
    std::cout << "pe-lubrication-aabb-neutrality: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-aabb-neutrality: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
