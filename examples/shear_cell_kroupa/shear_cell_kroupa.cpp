//=================================================================================================
/*!
 *  \file shear_cell_kroupa.cpp
 *  \brief Level-2 rheology skeleton: sheared monodisperse suspension (Kroupa et al. 2016)
 *
 *  Boundary-driven simple shear of N monodisperse spheres between two plane walls, using the
 *  HardContactLubricated solver with the Kroupa-2016 lubrication model. Shear is imposed by
 *  velocity-pinning thin particle layers at the top/bottom walls (PE plane walls carry no
 *  measurable reaction force, so the paper's wall force balance eqs 25-33 is replaced by the
 *  equivalent bulk virial): the lubrication contribution to the suspension shear stress is
 *  accumulated as
 *      sigma_xz = (1/V) sum_pairs r_ij,x * F_ij,z
 *  with F_ij recomputed from pe::lubrication::computeWrench for every close pair, and the
 *  lubrication viscosity estimate is eta_L = sigma_xz / gammaDot, written as CSV for
 *  evaluate_viscosity.py. This is the SKELETON for the eta(phi) validation campaign
 *  (compare against Krieger-Dougherty / Maron-Pierce, paper Fig. 3); production runs sweep
 *  phi and average over a longer steady-state window.
 *
 *  Build note: requires pe_CONSTRAINT_SOLVER == pe::response::HardContactLubricated
 *  (configure with -DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactLubricated").
 *
 *  Usage: shear_cell_kroupa [phi] [steps] [gammaDot]   (defaults: 0.3, 2000, 100 1/s)
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

using namespace pe;

int main(int argc, char* argv[]) {
  const real phi = (argc > 1) ? real(std::atof(argv[1])) : real(0.3);
  const size_t steps = (argc > 2) ? static_cast<size_t>(std::atol(argv[2])) : 2000;
  const real gammaDot = (argc > 3) ? real(std::atof(argv[3])) : real(100.0);

  const real R = real(1e-3);
  const real eta = real(1e-3);          // water-like
  const real rhoP = real(1000.0);
  const real dt = real(1e-5);

  // Box sized for N ~ 500 particles at the requested phi
  const size_t N = 500;
  const real Vp = real(4.0 / 3.0) * M_PI * R * R * R;
  const real L = std::cbrt(real(N) * Vp / phi);
  const real U = gammaDot * L;          // top-bottom velocity difference

  WorldID world = theWorld();
  world->setGravity(0.0, 0.0, 0.0);
  world->setViscosity(eta);
  world->setLiquidDensity(1000.0);
  world->setDamping(1.0);

  theCollisionSystem()->setContactHysteresisDelta(0.0);
  theCollisionSystem()->setLubricationHysteresisDelta(0.0);
  theCollisionSystem()->setMinEpsLub(1e-12);
  lubrication::setEnabled(true);
  lubrication::setModel(lubrication::modelKroupa2016);
  lubrication::setScheme(lubrication::schemeSemiImplicit);
  lubrication::setCutoffFactor(real(0.5));
  lubrication::setEpsCritical(real(0.1));  // paper's calibrated value
  lubrication::setSlipCorrection(true);

  MaterialID mat = createMaterial("shear_mat", rhoP, 0.0, 0.1, 0.05, 0.2, 300, 1e6, 1e3, 2e3);
  unsigned int id = 0;

  // Confining walls (x/y walls keep the skeleton simple in lieu of periodic boundaries)
  createPlane(id++, 0.0, 0.0, 1.0, 0.0, mat)->setFixed(true);   // bottom z = 0
  createPlane(id++, 0.0, 0.0, -1.0, -L, mat)->setFixed(true);   // top    z = L
  createPlane(id++, 1.0, 0.0, 0.0, 0.0, mat)->setFixed(true);
  createPlane(id++, -1.0, 0.0, 0.0, -L, mat)->setFixed(true);
  createPlane(id++, 0.0, 1.0, 0.0, 0.0, mat)->setFixed(true);
  createPlane(id++, 0.0, -1.0, 0.0, -L, mat)->setFixed(true);

  // Random non-overlapping seeding
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(1.05 * R, L - 1.05 * R);
  std::vector<SphereID> spheres;
  size_t attempts = 0;
  while (spheres.size() < N && attempts < 200000) {
    ++attempts;
    const Vec3 p(real(u(rng)), real(u(rng)), real(u(rng)));
    bool ok = true;
    for (SphereID s : spheres) {
      if ((s->getPosition() - p).length() < real(2.05) * R) { ok = false; break; }
    }
    if (!ok) continue;
    SphereID s = createSphere(id++, p[0], p[1], p[2], R, mat);
    // Seed with the linear shear profile
    s->setLinearVel(gammaDot * (p[2] - real(0.5) * L), 0.0, 0.0);
    spheres.push_back(s);
  }
  std::cout << "Seeded " << spheres.size() << "/" << N << " spheres, phi = "
            << real(spheres.size()) * Vp / (L * L * L) << ", box L = " << L << "\n";

  // Lubrication model snapshot for the stress virial (matches the solver configuration)
  lubrication::ModelConfig cfg;
  cfg.enabled = true;
  cfg.tangential = true;  cfg.twisting = true;
  cfg.slipCorrection = true;  cfg.resistSeparation = true;  cfg.wallTerms = true;
  cfg.model = lubrication::modelKroupa2016;
  cfg.scheme = lubrication::schemeSemiImplicit;
  cfg.epsCritical = lubrication::getEpsCritical();
  cfg.cutoffFactor = lubrication::getCutoffFactor();
  cfg.minGap = real(1e-12);
  cfg.alphaImpulseCap = real(1);
  cfg.viscosity = eta;

  std::ofstream csvOut("shear_cell_eta.csv");
  csvOut << "step,phi,gammaDot,sigma_xz_lub,eta_L\n";

  const real bandWidth = real(3.0) * R;  // velocity-pinned boundary layers
  const real V = L * L * L;

  for (size_t step = 0; step < steps; ++step) {
    // Drive: pin the boundary layers to the wall velocities
    for (SphereID s : spheres) {
      const real z = s->getPosition()[2];
      if (z > L - bandWidth) {
        s->setLinearVel(real(0.5) * U, 0.0, s->getLinearVel()[2]);
      } else if (z < bandWidth) {
        s->setLinearVel(-real(0.5) * U, 0.0, s->getLinearVel()[2]);
      }
    }

    world->simulationStep(dt);

    // Sample the lubrication stress virial every 20 steps
    if (step % 20 == 0) {
      real sigmaXZ = real(0);
      for (size_t a = 0; a < spheres.size(); ++a) {
        for (size_t b = a + 1; b < spheres.size(); ++b) {
          SphereID s1 = spheres[a];
          SphereID s2 = spheres[b];
          const Vec3 d = s1->getPosition() - s2->getPosition();
          const real dist = d.length();
          const real h = dist - real(2) * R;
          const real aRef = R;  // monodisperse: 2 R^2/(2R) = R
          if (h <= real(0) || h > lubrication::lubricationCutoff(aRef)) continue;

          lubrication::PairKinematics kin;
          kin.n = d / dist;
          kin.h = h;
          kin.r1 = -(R + real(0.5) * h) * kin.n;
          kin.r2 = (R + real(0.5) * h) * kin.n;
          kin.v1 = s1->getLinearVel();  kin.v2 = s2->getLinearVel();
          kin.w1 = s1->getAngularVel(); kin.w2 = s2->getAngularVel();
          kin.aRef = aRef;  kin.wall = false;
          const lubrication::PairWrench wr = lubrication::computeWrench(kin, cfg);
          const Vec3 F = wr.Fn + wr.Ft;  // force on s1 from s2
          sigmaXZ -= d[0] * F[2] / V;    // virial: sigma_ab = -(1/V) sum r_ij,a F_ij,b
        }
      }
      const real etaL = sigmaXZ / gammaDot;
      csvOut << step << "," << phi << "," << gammaDot << "," << sigmaXZ << "," << etaL << "\n";
      if (step % 200 == 0) {
        std::cout << "step " << step << "  sigma_xz(lub) = " << sigmaXZ
                  << "  eta_L = " << etaL << "\n";
      }
    }
  }

  std::cout << "Done. Stress samples written to shear_cell_eta.csv\n";
  return EXIT_SUCCESS;
}
