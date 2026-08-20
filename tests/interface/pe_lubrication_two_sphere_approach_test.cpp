// Level 1 validation: two-sphere constant-velocity approach (Kroupa et al. 2016, Fig. 7).
//
// A mobile sphere approaches a fixed equal sphere through the lubrication band. For each
// gap h a single solver step is taken from a prescribed approach velocity; the applied
// normal impulse gives the effective resistance K_meas = -m ln(v_after/v_before)/dt,
// which for the semi-implicit scheme must equal the model resistance K_n(h) exactly.
// This reproduces the paper's F/(v R eta) vs R/h diagnostic pointwise through the full
// solver pipeline (detection -> blend weights -> contact loop -> impulse application):
//   * no-slip curve matches the analytic resistance within tolerance
//   * Vinogradova slip correction lowers the curve
//   * forces plateau (saturate) below h_c, plateau monotone in eps_c
//   * separation branch: suction active with lubricationOnSeparation_, absent without
//
// Linked against pe_static_lubstage_plain: the same library built with
// pe_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers.
// The test therefore runs under any shipped library default and never skips.
// Set PE_LUB_CSV=1 to dump a curve CSV for plotting.

#include <pe/core.h>
#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <type_traits>

using namespace pe;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}


struct Scene {
  WorldID world;
  SphereID mobile;
  SphereID fixed;
  real radius;
  real mass;
  real dt;
  real eta;
};

// One measurement: place the mobile sphere at gap h with normal velocity vz (negative =
// approach), take one step, return the velocity ratio v_after/v_before along z.
real stepVelocityRatio(Scene& s, real h, real vz) {
  s.mobile->setPosition(0.0, 0.0, 2.0 * s.radius + h);  // fixed sphere center at z = R... see setup
  s.mobile->setLinearVel(0.0, 0.0, vz);
  s.mobile->setAngularVel(0.0, 0.0, 0.0);
  s.world->simulationStep(s.dt);
  return s.mobile->getLinearVel()[2] / vz;
}

// Effective normal resistance from the measured exponential decay factor.
real measuredResistance(Scene& s, real h, real vz) {
  const real ratio = stepVelocityRatio(s, h, vz);
  return -s.mass * std::log(ratio) / s.dt;
}

lubrication::ModelConfig currentConfig(real eta) {
  lubrication::ModelConfig cfg;
  cfg.enabled = true;
  cfg.tangential = lubrication::getTangential();
  cfg.twisting = lubrication::getTwisting();
  cfg.slipCorrection = lubrication::getSlipCorrection();
  cfg.resistSeparation = lubrication::getResistSeparation();
  cfg.wallTerms = true;
  cfg.model = lubrication::modelKroupa2016;
  cfg.scheme = lubrication::schemeSemiImplicit;
  cfg.epsCritical = lubrication::getEpsCritical();
  cfg.cutoffFactor = lubrication::getCutoffFactor();
  cfg.minGap = real(1e-12);
  cfg.alphaImpulseCap = real(1);
  cfg.viscosity = eta;
  return cfg;
}

template <typename CS>
int run(CS& cs) {
  (void)cs;   // the stage is configured through pe::lubrication::, not the solver
  Scene s;
  s.radius = real(0.01);
  s.dt = real(1e-3);
  s.eta = real(0.1);

  s.world = theWorld();
  s.world->setGravity(0.0, 0.0, 0.0);
  s.world->setViscosity(s.eta);
  s.world->setLiquidDensity(1.0);
  s.world->setDamping(1.0);

  lubrication::setContactHysteresisDelta(0.0);       // sharp entry: blend = 1 across the whole band
  lubrication::setLubricationHysteresisDelta(0.0);   // sharp exit
  lubrication::setAlphaImpulseCap(1.0);
  lubrication::setMinGap(1e-12);
  lubrication::setEnabled(true);
  lubrication::setModel(lubrication::modelKroupa2016);
  lubrication::setScheme(lubrication::schemeSemiImplicit);
  lubrication::setCutoffFactor(real(0.5));  // h_cut = 0.5 aRef = 0.005 m
  lubrication::setMeshClampFactor(real(0));
  lubrication::setTangential(true);
  lubrication::setTwisting(true);

  MaterialID mat = createMaterial("fig7_mat", 2000.0, 0.0, 0.1, 0.05, 0.2, 300, 1e6, 1e3, 2e3);
  unsigned int id = 0;
  // Fixed sphere centered at z = R (bottom); mobile sphere approaches from above along z.
  s.fixed = createSphere(id++, 0.0, 0.0, 0.0, s.radius, mat);
  s.fixed->setFixed(true);
  // Far-away wall for the sphere-plane (paper Fig. 7 / AFM) sweep: plane z = -50 R,
  // outside the lubrication cutoff of everything above.
  const real zWall = real(-50.0) * s.radius;
  createPlane(id++, 0.0, 0.0, 1.0, zWall, mat)->setFixed(true);
  s.mobile = createSphere(id++, 0.0, 0.0, 4.0 * s.radius, s.radius, mat);
  s.mass = s.mobile->getMass();

  // NOTE on stepVelocityRatio: mobile center is placed at z = 2R + h above the FIXED
  // sphere center at origin -> surface gap = h. aRef = R for equal spheres.
  const real aRef = s.radius;
  const real v0 = real(-0.01);
  const bool csv = (std::getenv("PE_LUB_CSV") != nullptr);

  // ---- 0. CSV mode: full Fig. 7 curves F/(v R eta) = K/(R eta) vs R/h ----
  // One no-slip curve plus slip-corrected/saturating curves for several eps_c,
  // measured through the full solver pipeline (one step per point).
  if (csv) {
    std::cout << "curve,R_over_h,F_over_vReta\n";
    const auto sweep = [&](const char* name) {
      for (real hOverR = real(0.2); hOverR >= real(1e-3); hOverR *= real(0.78)) {
        const real K = measuredResistance(s, hOverR * s.radius, v0);
        std::cout << name << "," << real(1) / hOverR << ","
                  << K / (s.radius * s.eta) << "\n";
      }
    };
    // sphere-plane variant: the paper's Fig. 7 compares against sphere-on-flat AFM
    // data and therefore uses the WALL resistance set (eq 16, leading eps^-1) -- a
    // factor ~4 above the equal-sphere pair curves (eq 12, leading eps^-1/4).
    const auto sweepWall = [&](const char* name) {
      for (real hOverR = real(0.2); hOverR >= real(1e-3); hOverR *= real(0.78)) {
        const real h = hOverR * s.radius;
        s.mobile->setPosition(0.5, 0.0, zWall + s.radius + h);
        s.mobile->setLinearVel(0.0, 0.0, v0);
        s.mobile->setAngularVel(0.0, 0.0, 0.0);
        s.world->simulationStep(s.dt);
        const real K = -s.mass * std::log(s.mobile->getLinearVel()[2] / v0) / s.dt;
        std::cout << name << "," << real(1) / hOverR << ","
                  << K / (s.radius * s.eta) << "\n";
      }
    };
    for (int wall = 0; wall <= 1; ++wall) {
      const auto sw = [&](const char* n) { if (wall) sweepWall(n); else sweep(n); };
      const char* pre = wall ? "wall " : "";
      char name[48];
      lubrication::setSlipCorrection(false);
      lubrication::setEpsCritical(real(0));
      std::snprintf(name, sizeof(name), "%sno-slip", pre);
      sw(name);
      lubrication::setSlipCorrection(true);
      for (real epsC : {real(0.02), real(0.01), real(0.005), real(0.0025)}) {
        lubrication::setEpsCritical(epsC);
        std::snprintf(name, sizeof(name), "%seps_c=%g", pre, static_cast<double>(epsC));
        sw(name);
      }
    }
  }

  // ---- 1. No-slip curve: K_meas(h) matches the analytic resistance ----
  lubrication::setSlipCorrection(false);
  lubrication::setEpsCritical(real(0));  // no saturation
  {
    const lubrication::ModelConfig cfg = currentConfig(s.eta);
    for (real hOverR = real(0.2); hOverR >= real(0.002); hOverR *= real(0.5)) {
      const real h = hOverR * s.radius;
      const real Kmeas = measuredResistance(s, h, v0);
      const real Kmodel = lubrication::normalResistance(h, aRef, false, cfg);
      check(std::fabs(Kmeas / Kmodel - real(1)) < real(0.01),
            "no-slip resistance matches model through the solver");
    }
  }

  // ---- 2. Slip correction lowers the curve ----
  {
    const real h = real(0.01) * s.radius;
    const real Knoslip = measuredResistance(s, h, v0);
    lubrication::setSlipCorrection(true);
    lubrication::setEpsCritical(real(0.01));  // h_c = 1e-4 m, h = 1e-4 m: at the knee
    const real Kslip = measuredResistance(s, real(0.02) * s.radius, v0);
    const real KnoslipSame = [&] {
      lubrication::setSlipCorrection(false);
      lubrication::setEpsCritical(real(0));
      const real k = measuredResistance(s, real(0.02) * s.radius, v0);
      lubrication::setSlipCorrection(true);
      lubrication::setEpsCritical(real(0.01));
      return k;
    }();
    (void)Knoslip;
    check(Kslip < KnoslipSame, "slip correction lowers the resistance");
  }

  // ---- 3. Saturation plateau below h_c, monotone in eps_c ----
  {
    lubrication::setSlipCorrection(true);
    real prevPlateau = real(-1);
    for (real epsC : {real(0.02), real(0.01), real(0.005)}) {
      lubrication::setEpsCritical(epsC);
      const real hc = epsC * aRef;
      const real Ka = measuredResistance(s, real(0.5) * hc, v0);
      const real Kb = measuredResistance(s, real(0.1) * hc, v0);
      check(std::fabs(Ka / Kb - real(1)) < real(1e-6),
            "resistance frozen (plateau) below h_c");
      if (prevPlateau > real(0)) {
        check(Ka > prevPlateau, "plateau resistance monotone: smaller eps_c -> higher plateau");
      }
      prevPlateau = Ka;
    }
  }

  // ---- 4. Separation branch (suction) switchable ----
  {
    lubrication::setSlipCorrection(false);
    lubrication::setEpsCritical(real(0));
    const real h = real(0.01) * s.radius;
    lubrication::setResistSeparation(true);
    const real ratioOn = stepVelocityRatio(s, h, real(+0.01));  // separating
    lubrication::setResistSeparation(false);
    const real ratioOff = stepVelocityRatio(s, h, real(+0.01));
    check(ratioOn < real(1) - real(1e-6), "suction decelerates separation when enabled");
    check(std::fabs(ratioOff - real(1)) < real(1e-9),
          "no normal force on separation when disabled");
    lubrication::setResistSeparation(true);
  }

  if (failures == 0) {
    std::cout << "pe-lubrication-two-sphere-approach: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-two-sphere-approach: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  // No solver gate: this target links the stage-capable library variant, so the
  // test runs (and must pass) under any shipped library default.
  return run(*theCollisionSystem());
}
