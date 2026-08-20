// SERIAL-MODE COVERAGE for the lubrication add-on.
//
// This test exists because an audit of the original lubrication PR found that ZERO tests
// exercised lubrication outside the isolated HardContactLubricated pipeline, and none at
// all under the campaign's serial (non-MPI) PE configuration. Everything shipped was
// either a pure-closure unit test or bound to a solver the campaign does not use.
//
// What it pins:
//   * the shared stage runs under pe::response::HardContactAndFluid -- the solver the DNS
//     campaign actually ships -- with lubrication switched on purely at RUNTIME, no
//     compile-time solver selection for lubrication anywhere;
//   * the force the full pipeline applies (detection -> AABB padding -> blend weight ->
//     stage -> impulse) equals the closure in LubricationModel.h evaluated directly.
//     The model is used as its own oracle: it is independently unit-tested by
//     pe_lubrication_model_test against hand-evaluated paper coefficients, so agreement
//     here isolates the PLUMBING rather than re-testing the physics;
//   * both pair geometries the stage supports: sphere-sphere and sphere-WALL. The wall
//     path is what a lubricated ten Cate settling case would depend on;
//   * the switch is honest in both directions -- off means exactly zero force, and off
//     means the AABB padding collapses to zero as well.
//
// This is a non-MPI build (PE_USE_MPI defaults OFF), which is the serial-PE arrangement
// the campaign runs. The CFD-coupled test belongs with the FeatFloWer wire (D2.2 §4) and
// is deliberately out of scope here.
//
// Linked against pe_static_lubstage_fluid: the same library built with
// pe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid. Runs under any shipped library
// default and never skips.

#include <pe/core.h>
#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdlib>
#include <cstdio>
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

const real radius = real(0.01);
const real dt     = real(1e-3);
const real eta    = real(0.1);

// The stage's semi-implicit normal mode integrates the linear drag exactly over the step:
//   v_after = v_before * exp(-K_n dt / m_eff)
// so one step at a prescribed approach velocity recovers K_n without any finite-difference
// error. m_eff = 1/(1/m1 + 1/m2); a fixed partner has infinite mass, so m_eff = m.
real resistanceFromStep(real vBefore, real vAfter, real mEff) {
  return -mEff * std::log(vAfter / vBefore) / dt;
}

lubrication::ModelConfig oracleConfig() {
  lubrication::ModelConfig cfg;
  cfg.enabled          = true;
  cfg.tangential       = lubrication::getTangential();
  cfg.twisting         = lubrication::getTwisting();
  cfg.slipCorrection   = lubrication::getSlipCorrection();
  cfg.resistSeparation = lubrication::getResistSeparation();
  cfg.wallTerms        = lubrication::getWallTerms();
  cfg.model            = lubrication::getModel();
  cfg.scheme           = lubrication::getScheme();
  cfg.epsCritical      = lubrication::getEpsCritical();
  cfg.cutoffFactor     = lubrication::getCutoffFactor();
  cfg.minGap           = lubrication::getMinGap();
  cfg.alphaImpulseCap  = lubrication::getAlphaImpulseCap();
  cfg.viscosity        = eta;
  return cfg;
}

}  // namespace

int main() {
  WorldID world = theWorld();
  world->setGravity(0.0, 0.0, 0.0);   // isolate lubrication: no gravity, no buoyancy drive
  world->setViscosity(eta);
  world->setLiquidDensity(1.0);
  world->setDamping(1.0);

  // Runtime configuration only -- this is the whole point of the add-on.
  lubrication::setEnabled(true);
  lubrication::setModel(lubrication::modelKroupa2016);
  lubrication::setScheme(lubrication::schemeSemiImplicit);
  lubrication::setContactHysteresisDelta(0.0);      // sharp band entry: blend == 1
  lubrication::setLubricationHysteresisDelta(0.0);  // sharp band exit
  lubrication::setAlphaImpulseCap(1.0);
  lubrication::setMinGap(1e-12);
  lubrication::setCutoffFactor(real(0.5));
  lubrication::setMeshClampFactor(real(0));
  lubrication::setSlipCorrection(false);
  lubrication::setEpsCritical(real(0));   // no saturation freeze: compare the raw resistance
  lubrication::setTangential(true);
  lubrication::setTwisting(true);
  lubrication::setWallTerms(true);
  lubrication::setAabbInflation(true);

  MaterialID mat =
      createMaterial("lub_serial", 2000.0, 0.0, 0.1, 0.05, 0.2, 300, 1e6, 1e3, 2e3);

  unsigned int id = 0;

  // Sphere-sphere pair: fixed partner at the origin, mobile sphere approaching from +z.
  SphereID fixedSphere = createSphere(id++, 0.0, 0.0, 0.0, radius, mat);
  fixedSphere->setFixed(true);
  SphereID mobile = createSphere(id++, 0.0, 0.0, 4.0 * radius, radius, mat);
  const real mass = mobile->getMass();

  // Wall for the sphere-plane leg, far enough below to stay outside every band used here.
  const real zWall = real(-50.0) * radius;
  createPlane(id++, 0.0, 0.0, 1.0, zWall, mat)->setFixed(true);

  // Mobile probe for the wall leg, parked far away while the pair leg runs.
  SphereID wallProbe =
      createSphere(id++, 0.0, 0.0, zWall + real(20.0) * radius, radius, mat);

  const real v0 = real(-0.01);   // approach (negative z)

  //---------------------------------------------------------------------------------
  // 1. Sphere-sphere: pipeline force == closure, across the band
  //---------------------------------------------------------------------------------
  {
    std::printf("\n[lubrication ENABLED, solver = HardContactAndFluid]\n");
    std::printf("  aabbPadding(R) = %.10g (cutoffFactor*R)\n",
                (double)lubrication::aabbPadding(radius));
    const real aRef = radius;   // 2*R*R/(R+R) for equal spheres
    bool anyNonzero = false;

    for (real hOverR : {real(0.2), real(0.1), real(0.05), real(0.02), real(0.01)}) {
      const real h = hOverR * radius;

      // Park the wall probe out of the way and reset it.
      wallProbe->setPosition(0.0, 0.0, zWall + real(20.0) * radius);
      wallProbe->setLinearVel(0.0, 0.0, 0.0);
      wallProbe->setAngularVel(0.0, 0.0, 0.0);

      mobile->setPosition(0.0, 0.0, 2.0 * radius + h);   // surface gap == h
      mobile->setLinearVel(0.0, 0.0, v0);
      mobile->setAngularVel(0.0, 0.0, 0.0);
      world->simulationStep(dt);

      const real vAfter = mobile->getLinearVel()[2];
      expect(vAfter > v0, "pair: lubrication decelerates the approach");
      if (std::fabs(vAfter - v0) > real(0)) anyNonzero = true;

      const real kMeasured = resistanceFromStep(v0, vAfter, mass);
      const real kModel = lubrication::normalResistance(h, aRef, false, oracleConfig());
      std::printf("  pair  h/R=%-5.3g  K_pipeline=%-14.8g K_closure=%-14.8g rel.diff=%.2e\n",
                  (double)hOverR, (double)kMeasured, (double)kModel,
                  (double)std::fabs(kMeasured / kModel - real(1)));
      expect(std::fabs(kMeasured / kModel - real(1)) < real(1e-6),
            "pair: pipeline resistance equals the closure evaluated directly");
    }
    expect(anyNonzero, "pair: stage applied a nonzero force under HardContactAndFluid");
  }

  //---------------------------------------------------------------------------------
  // 2. Sphere-wall: the geometry a lubricated ten Cate case needs
  //---------------------------------------------------------------------------------
  {
    const real aRef = radius;   // R_sphere for sphere-plane
    bool anyNonzero = false;

    // Park the pair's mobile sphere far from everything.
    mobile->setPosition(0.0, 0.0, 40.0 * radius);
    mobile->setLinearVel(0.0, 0.0, 0.0);
    mobile->setAngularVel(0.0, 0.0, 0.0);

    for (real hOverR : {real(0.2), real(0.1), real(0.05), real(0.02), real(0.01)}) {
      const real h = hOverR * radius;

      wallProbe->setPosition(0.0, 0.0, zWall + radius + h);   // surface gap == h
      wallProbe->setLinearVel(0.0, 0.0, v0);
      wallProbe->setAngularVel(0.0, 0.0, 0.0);
      world->simulationStep(dt);

      const real vAfter = wallProbe->getLinearVel()[2];
      expect(vAfter > v0, "wall: lubrication decelerates the approach");
      if (std::fabs(vAfter - v0) > real(0)) anyNonzero = true;

      // The plane is fixed (infinite mass), so m_eff is the sphere mass.
      const real kMeasured = resistanceFromStep(v0, vAfter, mass);
      const real kModel = lubrication::normalResistance(h, aRef, true, oracleConfig());
      std::printf("  wall  h/R=%-5.3g  K_pipeline=%-14.8g K_closure=%-14.8g rel.diff=%.2e\n",
                  (double)hOverR, (double)kMeasured, (double)kModel,
                  (double)std::fabs(kMeasured / kModel - real(1)));
      expect(std::fabs(kMeasured / kModel - real(1)) < real(1e-6),
            "wall: pipeline resistance equals the wall closure evaluated directly");
    }
    expect(anyNonzero, "wall: stage applied a nonzero force under HardContactAndFluid");

    // The wall resistance set (eq 16, leading 1/eps) must exceed the equal-sphere pair
    // set (eq 12, leading 1/(4 eps)) at the same gap -- a cheap guard that the wall
    // branch is genuinely taken rather than silently falling back to the pair branch.
    const real h = real(0.05) * radius;
    expect(lubrication::normalResistance(h, aRef, true, oracleConfig()) >
              lubrication::normalResistance(h, aRef, false, oracleConfig()),
          "wall resistance exceeds pair resistance at equal gap");
  }

  //---------------------------------------------------------------------------------
  // 3. The switch is honest: off means exactly nothing
  //---------------------------------------------------------------------------------
  {
    expect(lubrication::aabbPadding(radius) > real(0),
          "AABB padding is nonzero while lubrication is enabled");

    lubrication::setEnabled(false);

    std::printf("\n[lubrication DISABLED]\n");
    std::printf("  aabbPadding(R)             = %.10g\n",
                (double)lubrication::aabbPadding(radius));
    std::printf("  contactGenerationEnabled() = %s\n",
                lubrication::contactGenerationEnabled() ? "true" : "false");
    expect(lubrication::aabbPadding(radius) == real(0),
          "AABB padding collapses to zero when lubrication is disabled");

    mobile->setPosition(0.0, 0.0, 2.0 * radius + real(0.02) * radius);
    mobile->setLinearVel(0.0, 0.0, v0);
    mobile->setAngularVel(0.0, 0.0, 0.0);
    wallProbe->setPosition(0.0, 0.0, zWall + radius + real(0.02) * radius);
    wallProbe->setLinearVel(0.0, 0.0, v0);
    wallProbe->setAngularVel(0.0, 0.0, 0.0);
    world->simulationStep(dt);

    std::printf("  pair velocity delta        = %.17g\n",
                (double)(mobile->getLinearVel()[2] - v0));
    std::printf("  wall velocity delta        = %.17g\n\n",
                (double)(wallProbe->getLinearVel()[2] - v0));
    expect(mobile->getLinearVel()[2] == v0,
          "pair: velocity exactly unchanged with lubrication disabled");
    expect(wallProbe->getLinearVel()[2] == v0,
          "wall: velocity exactly unchanged with lubrication disabled");

    lubrication::setEnabled(true);
  }

  if (failures == 0) {
    std::cout << "pe-lubrication-serial-coverage: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-serial-coverage: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
