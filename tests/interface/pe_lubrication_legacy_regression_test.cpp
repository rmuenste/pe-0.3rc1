// Bit-for-bit regression gate for the LEGACY lubrication path of the shared lubrication stage.
//
// Reference trajectories below were captured from the pre-Kroupa-2016 solver (the original
// inline force block: F = 6*pi*mu*R_eff^2*(-vrn)/gap, approach-only, impulse-capped) with
// exactly this scene. After the solver upgrade, running with lubricationModel_="legacy" and
// lubricationCutoffFactor_=0 must reproduce these trajectories, protecting existing users
// who pin the legacy configuration.
//
// The constraint solver is compile-time configuration (pe/config/Collisions.h), so this
// target links pe_static_lubstage: the same library built with
// pe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid. The test therefore runs under
// any shipped library default and never skips.
//
// Scene: one sphere falling onto a plane through the lubrication band, plus a sphere pair
// approaching head-on inside the band (both far from each other). 500 steps, dt = 1e-4.
//
// Usage:
//   pe_lubrication_legacy_regression_test                   -> assert against references
//   pe_lubrication_legacy_regression_test --print-reference -> print reference table

#include <pe/core.h>
#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
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

bool close(real a, real b) {
  // Tight relative tolerance: identical code paths should agree to roundoff.
  return std::fabs(a - b) <= real(1e-12) * std::max(real(1), std::fabs(b));
}

// Checkpoints: after steps 100, 250, 500. Per checkpoint: pos + linvel of the three
// spheres (falling probe, pair left, pair right), 18 values per checkpoint.
const unsigned int checkpointSteps[3] = {100, 250, 500};

// Reference values captured from the pre-change solver (--print-reference output).
const real reference[3][18] = {
    {0, 0, 0.010500000000000001, 0, 0, 0, 0.50000275512485637, 0, 0.49957536714285716, 1.3087238618259216e-47, 0, -0.084085714285714305, 0.52049724487514359, 0, 0.49957536714285716, -1.3087238618259216e-47, 0, -0.084085714285714305},  // step 100
    {0, 0, 0.010500000000000001, 0, 0, 0, 0.50000275512485637, 0, 0.49736181071428576, 5.2869179891575046e-116, 0, -0.21021428571428577, 0.52049724487514359, 0, 0.49736181071428576, -5.2869179891575046e-116, 0, -0.21021428571428577},  // step 250
    {0, 0, 0.010500000000000001, 0, 0, 0, 0.50000275512485637, 0, 0.48946826428571433, 5.4174323263787033e-230, 0, -0.42042857142857154, 0.52049724487514359, 0, 0.48946826428571433, -5.4174323263787033e-230, 0, -0.42042857142857154},  // step 500
};

template <typename CS>
int runScene(CS& cs, bool printReference) {
  const real dt = real(1e-4);
  const real radius = real(0.01);
  const real gap0 = real(5e-4);  // initial surface gap, inside the lubrication band (1e-2)

  WorldID world = theWorld();
  world->setGravity(0.0, 0.0, -9.81);
  world->setViscosity(0.1);  // viscous enough that lubrication dominates the trajectories
  world->setLiquidDensity(1.0);
  world->setLiquidSolid(true);
  world->setDamping(1.0);

  // Pin every lubrication parameter explicitly: this test must be immune to default flips.
  lubrication::setContactHysteresisDelta(1e-9);
  lubrication::setLubricationHysteresisDelta(1e-3);
  lubrication::setLubricationThreshold(1e-2);
  lubrication::setAlphaImpulseCap(1.0);
  lubrication::setMinGap(1e-8);
  lubrication::setEnabled(true);
  lubrication::setModel(lubrication::modelLegacy);
  lubrication::setScheme(lubrication::schemeExplicitCapped);
  lubrication::setCutoffFactor(real(0));  // legacy absolute outer cutoff

  MaterialID planeMat = createMaterial("regr_plane", 2.5, 0.0, 0.5, 0.3, 0.25, 150, 1000, 10, 11);
  MaterialID sphereMat = createMaterial("regr_sphere", 7.0, 0.2, 0.45, 0.35, 0.22, 300, 5.0e4, 20, 25);

  unsigned int id = 0;
  PlaneID plane = createPlane(id++, 0.0, 0.0, 1.0, 0.0, planeMat);
  plane->setFixed(true);

  // Probe sphere falling onto the plane, starting inside the lubrication band
  SphereID probe = createSphere(id++, 0.0, 0.0, radius + gap0, radius, sphereMat);
  probe->setLinearVel(0.0, 0.0, -0.05);

  // Sphere pair approaching head-on along x, far from the plane and the probe
  SphereID left = createSphere(id++, 0.5, 0.0, 0.5, radius, sphereMat);
  SphereID right = createSphere(id++, 0.5 + 2.0 * radius + gap0, 0.0, 0.5, radius, sphereMat);
  left->setLinearVel(0.05, 0.0, 0.0);
  right->setLinearVel(-0.05, 0.0, 0.0);

  unsigned int nextCheckpoint = 0;
  for (unsigned int step = 1; step <= checkpointSteps[2]; ++step) {
    world->simulationStep(dt);
    if (nextCheckpoint < 3 && step == checkpointSteps[nextCheckpoint]) {
      real values[18];
      const SphereID bodies[3] = {probe, left, right};
      for (int b = 0; b < 3; ++b) {
        const Vec3 p = bodies[b]->getPosition();
        const Vec3 v = bodies[b]->getLinearVel();
        for (int c = 0; c < 3; ++c) {
          values[b * 6 + c] = p[c];
          values[b * 6 + 3 + c] = v[c];
        }
      }
      if (printReference) {
        std::cout << std::setprecision(17) << "    {";
        for (int i = 0; i < 18; ++i)
          std::cout << values[i] << (i < 17 ? ", " : "},");
        std::cout << "  // step " << step << "\n";
      } else {
        for (int i = 0; i < 18; ++i) {
          if (!close(values[i], reference[nextCheckpoint][i])) {
            std::cerr << "FAILED: checkpoint step " << step << " value " << i
                      << ": got " << std::setprecision(17) << values[i]
                      << " expected " << reference[nextCheckpoint][i] << "\n";
            ++failures;
          }
        }
      }
      ++nextCheckpoint;
    }
  }

  if (printReference) {
    std::cout << "(paste into the reference table)\n";
    return EXIT_SUCCESS;
  }

  // Sanity: the scene actually exercised lubrication (pair spheres were decelerated).
  check(std::fabs(left->getLinearVel()[0]) < real(0.05), "pair spheres were decelerated");

  if (failures == 0) {
    std::cout << "pe-lubrication-legacy-regression: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-legacy-regression: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
  const bool printReference =
      (argc > 1 && std::strcmp(argv[1], "--print-reference") == 0);

  // No solver gate: this target links the stage-capable library variant, so the
  // test runs (and must pass) under any shipped library default.
  return runScene(*theCollisionSystem(), printReference);
}
