// Level 1 validation: immersed wall impact (Gondret/Joseph-style scene) + dt robustness
// + tangential/twisting sanity for the wall resistance set.
//
// A sphere falls onto a plane under gravity with an explicit Stokes drag applied by the
// driver (PE-only stand-in for the fluid); lubrication and the hard contact come from the
// HardContactLubricated solver.
//
// IMPORTANT SCOPE NOTE: the canonical Gondret gate (restitution e vs Stokes number
// collapsing onto the experimental curve) requires an ELASTIC hard-contact rebound. The
// HardContactLubricated relaxation models are inelastic by design (relaxationModel_ =
// ApproximateInelasticCoulombContactByDecoupling; material restitution is not applied,
// see doc/technical-notes/relaxation-models.md) -- any upward velocity after impact is an
// error-reduction artifact, not physics. The full e(St) curve therefore remains a Level 2/3
// item pending a restitution-capable contact path; this test still emits the measured
// e(St) data as CSV (PE_LUB_CSV=1) for reference, but asserts only what the current
// solver stack guarantees:
//   * trajectories finite and dissipative (v_out <= v_in) across 3 decades of viscosity
//   * viscous limit: no rebound at low Stokes number
//   * near-wall impact velocity attenuated monotonically by viscosity, and reduced by
//     enabling lubrication at fixed viscosity (the quantity the model controls)
//   * dt robustness across 3 decades at fixed viscosity
//   * wall tangential force / rolling torque / twist decay signs and switches
//
// Needs pe_CONSTRAINT_SOLVER == pe::response::HardContactLubricated; exits 77 (CTest
// SKIPPED) otherwise.

#include <pe/core.h>
#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <type_traits>

using namespace pe;

namespace {

const int skipReturnCode = 77;
int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

template <typename CS, typename = void>
struct IsLubricatedSolver : std::false_type {};
template <typename CS>
struct IsLubricatedSolver<CS, std::void_t<
    decltype(std::declval<CS&>().setAlphaImpulseCap(real{})),
    decltype(std::declval<CS&>().setMinEpsLub(real{}))>> : std::true_type {};

struct DropResult {
  real vIn;    // max downward speed over the whole fall
  real vNear;  // max downward speed inside the near-wall band (gap < 0.1 R)
  real vOut;   // max upward speed after the first upward motion
  bool finite;
};

// Drop the sphere from rest at 10 R above the plane with explicit Stokes drag.
DropResult dropOntoWall(WorldID world, SphereID sphere, real radius, real eta,
                        real dt, real simTime) {
  sphere->setPosition(0.0, 0.0, radius + 10.0 * radius);
  sphere->setLinearVel(0.0, 0.0, 0.0);
  sphere->setAngularVel(0.0, 0.0, 0.0);

  DropResult r;
  r.vIn = r.vNear = r.vOut = real(0);
  r.finite = true;
  bool impacted = false;

  const unsigned int maxSteps = static_cast<unsigned int>(simTime / dt);
  for (unsigned int step = 0; step < maxSteps; ++step) {
    const Vec3 v = sphere->getLinearVel();
    sphere->addForce(-real(6) * M_PI * eta * radius * v);  // explicit Stokes drag
    world->simulationStep(dt);
    const real vz = sphere->getLinearVel()[2];
    const real gap = sphere->getPosition()[2] - radius;
    if (!std::isfinite(vz) || !std::isfinite(gap)) {
      r.finite = false;
      break;
    }
    if (!impacted) {
      if (vz < real(0)) {
        r.vIn = std::max(r.vIn, -vz);
        if (gap < real(0.1) * radius) r.vNear = std::max(r.vNear, -vz);
      }
      if (vz > real(0) && r.vIn > real(0)) impacted = true;
    }
    if (impacted) {
      r.vOut = std::max(r.vOut, vz);
      if (vz < real(0)) break;  // apex passed / second approach begins
    }
  }
  return r;
}

template <typename CS>
int run(CS& cs) {
  const real radius = real(3.175e-3);  // Gondret steel bead
  const real rhoP = real(7800.0);
  const real eDry = real(0.9);         // material value; NOT honored by the inelastic solver

  WorldID world = theWorld();
  world->setGravity(0.0, 0.0, -9.81);
  world->setLiquidDensity(1.0);
  world->setDamping(1.0);
  world->setAutoForceReset(true);

  cs.setContactHysteresisDelta(0.0);
  cs.setLubricationHysteresisDelta(0.0);
  cs.setAlphaImpulseCap(1.0);
  cs.setMinEpsLub(1e-12);
  lubrication::setEnabled(true);
  lubrication::setModel(lubrication::modelKroupa2016);
  lubrication::setScheme(lubrication::schemeSemiImplicit);
  lubrication::setCutoffFactor(real(0.5));
  lubrication::setEpsCritical(real(0.01));
  lubrication::setSlipCorrection(true);
  lubrication::setResistSeparation(true);
  lubrication::setTangential(true);
  lubrication::setTwisting(true);
  lubrication::setWallTerms(true);

  MaterialID sphereMat = createMaterial("bounce_sphere", rhoP, eDry, 0.0, 0.0, 0.2, 300, 1e6, 1e3, 2e3);
  MaterialID planeMat = createMaterial("bounce_plane", 2500.0, eDry, 0.0, 0.0, 0.2, 300, 1e6, 1e3, 2e3);

  unsigned int id = 0;
  PlaneID plane = createPlane(id++, 0.0, 0.0, 1.0, 0.0, planeMat);
  plane->setFixed(true);
  SphereID sphere = createSphere(id++, 0.0, 0.0, 20.0 * radius, radius, sphereMat);

  const bool csv = (std::getenv("PE_LUB_CSV") != nullptr);
  if (csv) std::cout << "eta,St,vIn,vNear,e\n";

  // ---- 1. Viscosity sweep: dissipative, finite, attenuation monotone ----
  {
    const real dt = real(2e-5);
    const real simTime = real(0.5);
    real prevVNear = real(-1);
    real firstE = real(-1);
    int attenuationViolations = 0;
    for (real eta : {real(1.0), real(0.3), real(0.1), real(0.03), real(0.01),
                     real(0.003), real(0.001)}) {
      world->setViscosity(eta);
      const DropResult r = dropOntoWall(world, sphere, radius, eta, dt, simTime);
      const real St = rhoP * real(2) * radius * r.vIn / (real(9) * eta);
      const real e = (r.vIn > real(0)) ? r.vOut / r.vIn : real(0);
      if (csv) std::cout << eta << "," << St << "," << r.vIn << "," << r.vNear << "," << e << "\n";
      check(r.finite, "viscosity sweep: trajectory stays finite");
      check(r.vOut <= r.vIn + real(1e-12), "no energy gain across the impact");
      if (firstE < real(0)) firstE = e;
      // Sweep runs from high to low viscosity: near-wall speed must not decrease.
      if (prevVNear >= real(0) && r.vNear < prevVNear - real(1e-9)) ++attenuationViolations;
      prevVNear = r.vNear;
    }
    check(firstE < real(0.05), "viscous limit: no rebound at low Stokes number");
    check(attenuationViolations == 0,
          "near-wall impact speed attenuates monotonically with viscosity");
  }

  // ---- 2. Lubrication reduces the near-wall impact speed at fixed viscosity ----
  {
    const real eta = real(0.1);
    const real dt = real(2e-5);
    world->setViscosity(eta);
    const DropResult on = dropOntoWall(world, sphere, radius, eta, dt, real(0.5));
    lubrication::setEnabled(false);
    const DropResult off = dropOntoWall(world, sphere, radius, eta, dt, real(0.5));
    lubrication::setEnabled(true);
    check(on.finite && off.finite, "lubrication on/off runs stay finite");
    check(on.vNear < off.vNear - real(1e-9),
          "lubrication reduces the near-wall impact speed");
    if (csv) std::cout << "lub_on,-," << on.vNear << ",-,-\nlub_off,-," << off.vNear << ",-,-\n";
  }

  // ---- 3. dt robustness across 3 decades at fixed viscosity ----
  {
    const real eta = real(0.03);
    world->setViscosity(eta);
    for (real dt : {real(2e-6), real(2e-5), real(2e-4)}) {
      const DropResult r = dropOntoWall(world, sphere, radius, eta, dt, real(0.5));
      check(r.finite, "dt sweep: trajectory stays finite");
      check(r.vOut <= r.vIn + real(1e-12), "dt sweep: no energy gain");
      if (csv) std::cout << "dt_" << dt << ",-," << r.vIn << "," << r.vNear << ",-\n";
    }
  }

  // ---- 4. Tangential and twisting sanity at fixed gap ----
  {
    world->setGravity(0.0, 0.0, 0.0);
    world->setViscosity(0.1);
    const real h = real(0.05) * radius;
    const real dt = real(1e-4);

    // Translation parallel to the wall: decelerating force + forward-rolling torque
    sphere->setPosition(0.0, 0.0, radius + h);
    sphere->setLinearVel(0.01, 0.0, 0.0);
    sphere->setAngularVel(0.0, 0.0, 0.0);
    world->simulationStep(dt);
    check(sphere->getLinearVel()[0] < real(0.01) - real(1e-12),
          "wall sliding force decelerates translation");
    check(sphere->getAngularVel()[1] > real(0),
          "wall sliding torque induces forward rolling");

    // Spin about the wall normal decays via the twisting torque
    sphere->setPosition(0.0, 0.0, radius + h);
    sphere->setLinearVel(0.0, 0.0, 0.0);
    sphere->setAngularVel(0.0, 0.0, 5.0);
    world->simulationStep(dt);
    const real wzAfter = sphere->getAngularVel()[2];
    check(wzAfter < real(5.0) - real(1e-12) && wzAfter > real(0),
          "twisting torque decays normal spin without reversal");

    // Switches: no tangential force / no spin decay when disabled
    lubrication::setTangential(false);
    lubrication::setTwisting(false);
    sphere->setPosition(0.0, 0.0, radius + h);
    sphere->setLinearVel(0.01, 0.0, 0.0);
    sphere->setAngularVel(0.0, 0.0, 5.0);
    world->simulationStep(dt);
    check(std::fabs(sphere->getLinearVel()[0] - real(0.01)) < real(1e-12),
          "tangential switch off: translation untouched");
    check(std::fabs(sphere->getAngularVel()[2] - real(5.0)) < real(1e-12),
          "twisting switch off: spin untouched");
    lubrication::setTangential(true);
    lubrication::setTwisting(true);
  }

  if (failures == 0) {
    std::cout << "pe-lubrication-bounce-stokes: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-bounce-stokes: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  using CollisionSystemType = std::remove_reference_t<decltype(*theCollisionSystem())>;
  if constexpr (IsLubricatedSolver<CollisionSystemType>::value) {
    return run(*theCollisionSystem());
  } else {
    std::cout << "pe-lubrication-bounce-stokes: SKIPPED (active pe_CONSTRAINT_SOLVER is "
                 "not pe::response::HardContactLubricated)\n";
    return skipReturnCode;
  }
}
