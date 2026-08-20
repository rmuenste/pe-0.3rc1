// Unit test for the pairwise lubrication model (Kroupa et al., Langmuir 2016).
//
// The physics lives in pe/core/lubrication/LubricationModel.h and is called from
// CollisionSystem<...HardContactLubricated>::resolveContacts. Exercising it through a
// real PE step would require standing up a world and the full collision pipeline;
// instead we test the pure model directly, which is exactly what the solver evaluates,
// so the force law is covered without that machinery.
//
// Covered (Level 0 of doc/technical-notes/lubrication-production-design.md §4):
//   * every resistance coefficient vs hand-evaluated references (pair + wall)
//   * Vinogradova slip factor: limits, monotonicity, value at h = h_c
//   * saturation freeze: force continuity across h = h_c
//   * dissipativity: per-term strict; full wrench power <= 0 on random states
//   * semi-implicit impulse formula: bounded, correct small-dt limit, no sign flip
//   * legacy model path: formula identity + approach-only gating
//   * feature switches: enabled/tangential/twisting/slipCorrection/wallTerms

#include <pe/core/lubrication/LubricationModel.h>
#include <pe/core/lubrication/Params.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

using namespace pe;
using namespace pe::lubrication;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

bool close(real a, real b, real tol) { return std::fabs(a - b) <= tol; }

bool vclose(const Vec3& a, const Vec3& b, real tol) {
  return close(a[0], b[0], tol) && close(a[1], b[1], tol) && close(a[2], b[2], tol);
}

bool vzero(const Vec3& a, real tol) { return vclose(a, Vec3(0, 0, 0), tol); }

// All switches on, no slip correction / saturation / floor unless a test sets them.
ModelConfig baseConfig() {
  ModelConfig cfg;
  cfg.enabled = true;
  cfg.tangential = true;
  cfg.twisting = true;
  cfg.slipCorrection = false;
  cfg.resistSeparation = true;
  cfg.wallTerms = true;
  cfg.model = modelKroupa2016;
  cfg.scheme = schemeSemiImplicit;
  cfg.epsCritical = real(0);
  cfg.cutoffFactor = real(0.5);
  cfg.minGap = real(0);
  cfg.alphaImpulseCap = real(1);
  cfg.viscosity = real(1);
  return cfg;
}

// Pair geometry: body 1 above body 2, n = +z (PE: from body 2 toward body 1).
PairKinematics pairGeometry(real R1, real R2, real h) {
  PairKinematics k;
  k.n = Vec3(0, 0, 1);
  k.h = h;
  k.r1 = Vec3(0, 0, -(R1 + real(0.5) * h));
  k.r2 = Vec3(0, 0, R2 + real(0.5) * h);
  k.v1 = k.v2 = k.w1 = k.w2 = Vec3(0, 0, 0);
  k.aRef = real(2) * R1 * R2 / (R1 + R2);
  k.wall = false;
  return k;
}

// Wall geometry: sphere (body 1) above a plane at z = 0, n = +z (plane normal).
PairKinematics wallGeometry(real R, real h) {
  PairKinematics k;
  k.n = Vec3(0, 0, 1);
  k.h = h;
  k.r1 = Vec3(0, 0, -(R + h));
  k.r2 = Vec3(0, 0, 0);  // plane reference at the contact point
  k.v1 = k.v2 = k.w1 = k.w2 = Vec3(0, 0, 0);
  k.aRef = R;
  k.wall = true;
  return k;
}

// Total dissipated power of a wrench: force part via F.g (covers the lever-arm
// torques), plus the pure lubrication torques against the angular velocities.
real wrenchPower(const PairKinematics& k, const PairWrench& wr) {
  const Vec3 g = k.v1 - k.v2 + k.w1 % k.r1 - k.w2 % k.r2;
  return trans(wr.Fn + wr.Ft) * g + trans(wr.M1) * k.w1 + trans(wr.M2) * k.w2;
}

// The exact impulse the semi-implicit solver branch applies to the normal mode.
real semiImplicitImpulse(real K, real mEff, real vrn, real dt) {
  return mEff * (real(1) - std::exp(-K * dt / mEff)) * std::fabs(vrn);
}

}  // namespace

int main() {
  const real tol = real(1e-9);

  // ---- 1. Resistance coefficients vs hand-evaluated references at eps = 0.1 ----
  // (independent transcription of paper eqs 12-19; L = ln 0.1 = -2.302585092994046)
  {
    ModelConfig cfg = baseConfig();
    const real eps = real(0.1);
    const real aRef = real(1);
    const real h = eps * aRef;
    const real pre = real(6) * M_PI;   // force prefactor, eta = aRef = 1
    const real preM = real(8) * M_PI;  // torque prefactor

    // pair: C_n = 0.25/eps - 9/40 L - 3/112 eps L
    check(close(normalResistance(h, aRef, false, cfg), pre * real(3.024249290),
                real(1e-6) * pre), "pair normal coefficient at eps=0.1");
    // wall: C_n = 1/eps - 1/5 L - 1/21 eps L
    check(close(normalResistance(h, aRef, true, cfg), pre * real(10.471481714),
                real(1e-6) * pre), "wall normal coefficient at eps=0.1");
    // pair: C_s(v_s) = -L/6
    check(close(slidingResistance(h, aRef, false, cfg), pre * real(0.383764182),
                real(1e-6) * pre), "pair sliding v_s coefficient at eps=0.1");
    // wall: C_s(v_s) = -8/15 L - 64/375 eps L
    check(close(slidingResistance(h, aRef, true, cfg), pre * real(1.267342835),
                real(1e-6) * pre), "wall sliding v_s coefficient at eps=0.1");
    // pair twist: 1/8 * (-eps L)
    check(close(twistingResistance(h, aRef, false, cfg), preM * real(0.028782314),
                real(1e-6) * preM), "pair twisting coefficient at eps=0.1");
    // wall twist: 1/2 * (-eps L)
    check(close(twistingResistance(h, aRef, true, cfg), preM * real(0.115129255),
                real(1e-6) * preM), "wall twisting coefficient at eps=0.1");

    // v_cs force / torque couplings via computeWrench (rotation only, vrn = 0):
    // pure rotation of body 1 about x with r1 = -(R + h/2) z gives
    // v_cs = w1 x r1 (already tangential), v_s = 0.
    {
      PairKinematics k = pairGeometry(real(1), real(1), h);  // aRef = 1
      k.w1 = Vec3(1, 0, 0);
      const Vec3 vcs = k.w1 % k.r1;  // = (0, -(1+h/2), 0)
      PairWrench wr = computeWrench(k, cfg);
      // Ft = -pre * C_cs * vcs ; pair C_cs = -L/6 - eps L/12 = 0.402952391
      check(vclose(wr.Ft, -pre * real(0.402952391) * vcs, real(1e-5)),
            "pair sliding v_cs force coefficient at eps=0.1");
      // M1 = preM * C_tvcs * (n x vcs) + twist(=0, spin.n=0)
      // pair C_tvcs = -L/5 - 47/250 eps L = 0.503805620
      check(vclose(wr.M1, preM * real(0.503805620) * (k.n % vcs), real(1e-5)),
            "pair sliding v_cs torque coefficient at eps=0.1");
      // pair sliding torque acts with the same vector on both bodies
      check(vclose(wr.M1, wr.M2, tol), "pair sliding torque equal on both bodies");
    }
    {
      PairKinematics k = wallGeometry(real(1), h);
      k.v1 = Vec3(1, 0, 0);  // translation parallel to the wall: v_s = x
      PairWrench wr = computeWrench(k, cfg);
      // wall C_ts(v_s) = -2/15 L - 86/375 eps L = 0.359817300
      check(vclose(wr.M1, preM * real(0.359817300) * (k.n % k.v1), real(1e-5)),
            "wall sliding v_s torque coefficient at eps=0.1");
      // wall: no pure lubrication torque on the plane
      check(vzero(wr.M2, tol), "wall M2 is zero");
      // wall C_cs(force) = -2/15 L - 86/375 eps L (same value) exercised via rotation:
      PairKinematics kr = wallGeometry(real(1), h);
      kr.w1 = Vec3(1, 0, 0);
      const Vec3 vcs = kr.w1 % kr.r1;
      PairWrench wrr = computeWrench(kr, cfg);
      const Vec3 expectedFt = -pre * real(0.359817300) * vcs;
      // wall C_tvcs = -2/5 L - 66/125 eps L = 1.042610528
      const Vec3 expectedM1 = preM * real(1.042610528) * (kr.n % vcs);
      check(vclose(wrr.Ft, expectedFt, real(1e-5)), "wall sliding v_cs force coefficient");
      check(vclose(wrr.M1, expectedM1, real(1e-5)), "wall sliding v_cs torque coefficient");
    }
  }

  // ---- 2. Leading-order limit: K_n -> 6 pi eta a^2 / h (a = aRef/2) as eps -> 0 ----
  {
    ModelConfig cfg = baseConfig();
    const real aRef = real(2e-3);
    for (real eps = real(1e-3); eps >= real(1e-7); eps *= real(1e-2)) {
      const real h = eps * aRef;
      const real Kn = normalResistance(h, aRef, false, cfg);
      const real lead = real(6) * M_PI * cfg.viscosity *
                        (real(0.5) * aRef) * (real(0.5) * aRef) / h;
      check(close(Kn / lead, real(1), real(50) * eps * std::fabs(std::log(eps))),
            "pair K_n approaches leading-order 6 pi eta a^2/h");
    }
  }

  // ---- 3. Slip factor: limits, monotonicity, value at h = h_c ----
  {
    const real hc = real(1e-3);
    check(close(slipFactor(real(1e6) * hc, hc), real(1), real(1e-5)),
          "slip factor -> 1 for h >> h_c");
    check(close(slipFactor(hc, hc), real(0.423409502), real(1e-8)),
          "slip factor at h = h_c is (1/3)((7/6)ln7 - 1)");
    check(close(slipFactor(real(1), real(0)), real(1), tol), "h_c = 0 disables slip");
    real prev = real(0);
    for (real h = real(1e-2) * hc; h <= real(1e4) * hc; h *= real(2)) {
      const real f = slipFactor(h, hc);
      check(f > prev && f <= real(1) + tol, "slip factor monotone in (0, 1]");
      prev = f;
    }
  }

  // ---- 4. Saturation freeze: force continuous across h = h_c, constant below ----
  {
    ModelConfig cfg = baseConfig();
    cfg.epsCritical = real(0.1);
    cfg.slipCorrection = true;
    const real aRef = real(1);
    const real hc = cfg.epsCritical * aRef;

    PairKinematics kAbove = pairGeometry(real(1), real(1), hc * (real(1) + real(1e-10)));
    PairKinematics kAt = pairGeometry(real(1), real(1), hc);
    PairKinematics kBelow = pairGeometry(real(1), real(1), hc * real(0.01));
    kAbove.v1 = kAt.v1 = kBelow.v1 = Vec3(0, 0, -1);  // approaching

    const PairWrench wAbove = computeWrench(kAbove, cfg);
    const PairWrench wAt = computeWrench(kAt, cfg);
    const PairWrench wBelow = computeWrench(kBelow, cfg);
    check(vclose(wAbove.Fn, wAt.Fn, real(1e-6) * wAt.Fn.length()),
          "force continuous across h = h_c");
    check(vclose(wBelow.Fn, wAt.Fn, tol), "force frozen (saturated) below h_c");
  }

  // ---- 5. Dissipativity ----
  {
    ModelConfig cfg = baseConfig();
    cfg.epsCritical = real(0.05);
    cfg.slipCorrection = true;

    // (a) normal term alone, strict: P = -K_n vrn^2
    {
      ModelConfig c = cfg;
      c.tangential = c.twisting = false;
      PairKinematics k = pairGeometry(real(1), real(2), real(0.05));
      k.v1 = Vec3(0, 0, -0.7);
      check(wrenchPower(k, computeWrench(k, c)) < real(0), "normal approach dissipates");
      k.v1 = Vec3(0, 0, 0.7);
      check(wrenchPower(k, computeWrench(k, c)) < real(0), "normal separation dissipates");
    }
    // (b) twist alone, strict (pair and wall)
    {
      ModelConfig c = cfg;
      c.tangential = false;
      PairKinematics k = pairGeometry(real(1), real(1), real(0.05));
      k.w1 = Vec3(0, 0, 3);
      k.w2 = Vec3(0, 0, -1);
      check(wrenchPower(k, computeWrench(k, c)) < real(0), "pair twist dissipates");
      PairKinematics kw = wallGeometry(real(1), real(0.05));
      kw.w1 = Vec3(0, 0, 2);
      check(wrenchPower(kw, computeWrench(kw, c)) < real(0), "wall twist dissipates");
    }
    // (c) pure translational sliding, strict
    {
      PairKinematics k = pairGeometry(real(1), real(1), real(0.05));
      k.v1 = Vec3(0.4, -0.3, 0);
      check(wrenchPower(k, computeWrench(k, cfg)) < real(0), "pair sliding dissipates");
      PairKinematics kw = wallGeometry(real(1), real(0.05));
      kw.v1 = Vec3(0.4, -0.3, 0);
      check(wrenchPower(kw, computeWrench(kw, cfg)) < real(0), "wall sliding dissipates");
    }
    // (d) full random states: total power <= 0 (small tolerance for FP noise)
    {
      std::mt19937 rng(12345);
      std::uniform_real_distribution<double> u(-1.0, 1.0);
      std::uniform_real_distribution<double> ug(1e-4, 0.4);
      int violations = 0;
      real worst = real(0);
      for (int i = 0; i < 2000; ++i) {
        const bool wall = (i % 4 == 0);
        const real R1 = real(0.5) + real(0.5) * real(std::fabs(u(rng)));
        const real R2 = real(0.5) + real(0.5) * real(std::fabs(u(rng)));
        const real aRefPair = real(2) * R1 * R2 / (R1 + R2);
        const real h = real(ug(rng)) * (wall ? R1 : aRefPair);
        PairKinematics k = wall ? wallGeometry(R1, h) : pairGeometry(R1, R2, h);
        k.v1 = Vec3(real(u(rng)), real(u(rng)), real(u(rng)));
        k.v2 = wall ? Vec3(0, 0, 0) : Vec3(real(u(rng)), real(u(rng)), real(u(rng)));
        k.w1 = Vec3(real(u(rng)), real(u(rng)), real(u(rng)));
        k.w2 = wall ? Vec3(0, 0, 0) : Vec3(real(u(rng)), real(u(rng)), real(u(rng)));
        const real P = wrenchPower(k, computeWrench(k, cfg));
        if (P > real(1e-12)) {
          ++violations;
          worst = std::max(worst, P);
        }
      }
      if (violations > 0) {
        std::cerr << "  dissipativity violations: " << violations
                  << " worst P = " << worst << "\n";
      }
      check(violations == 0, "full wrench power <= 0 on random states");
    }
  }

  // ---- 6. Semi-implicit impulse formula (as used by the solver branch) ----
  {
    const real mEff = real(2.3);
    const real vrn = real(-0.8);
    for (real Kdt = real(1e-8); Kdt <= real(1e6); Kdt *= real(100)) {
      const real J = semiImplicitImpulse(Kdt, mEff, vrn, real(1));
      check(J >= real(0) && J < mEff * std::fabs(vrn) + real(1e-15),
            "semi-implicit impulse bounded by approach momentum");
    }
    // small-argument limit: J -> K dt |vrn|
    const real Ksmall = real(1e-6);
    check(close(semiImplicitImpulse(Ksmall, mEff, vrn, real(1)),
                Ksmall * std::fabs(vrn), real(1e-12)),
          "semi-implicit impulse -> K dt |vrn| for K dt/m -> 0");
    // stiff limit: J -> m |vrn| (velocity fully arrested, never reversed)
    check(close(semiImplicitImpulse(real(1e12), mEff, vrn, real(1)),
                mEff * std::fabs(vrn), real(1e-12)),
          "semi-implicit impulse -> m |vrn| for K dt/m -> inf");
  }

  // ---- 7. Legacy model path ----
  {
    // formula identity with the historical solver expression
    const real gap = real(3.7e-4), Reff = real(1.2e-2), mu = real(8.9e-4), vrn = real(-0.25);
    const real expected = real(6) * M_PI * mu * Reff * Reff * (-vrn) / gap;
    check(close(legacyForceMagnitude(gap, Reff, mu, vrn), expected, real(0)),
          "legacy force magnitude identical to historical expression");

    ModelConfig cfg = baseConfig();
    cfg.model = modelLegacy;
    cfg.minGap = real(1e-8);
    cfg.viscosity = mu;

    // pair: legacy R_eff = aRef/2; normal-only; tangential/torques zero
    PairKinematics k = pairGeometry(real(1.2e-2) * real(2), real(1.2e-2) * real(2), gap);
    // R1 = R2 = 2 Reff -> aRef = 2 Reff -> legacy R_eff = Reff
    k.v1 = Vec3(0.3, 0, vrn);
    k.w1 = Vec3(1, 2, 3);
    PairWrench wr = computeWrench(k, cfg);
    check(close(wr.Fn[2], expected, real(1e-12) * expected), "legacy pair normal magnitude");
    check(vzero(wr.Ft, real(0)) && vzero(wr.M1, real(0)) && vzero(wr.M2, real(0)),
          "legacy path has no tangential force or torques");

    // approach-only gate, regardless of resistSeparation
    k.v1 = Vec3(0, 0, 0.25);
    check(vzero(computeWrench(k, cfg).Fn, real(0)), "legacy path skips separation");

    // wall: legacy R_eff = aRef = R_sphere
    PairKinematics kw = wallGeometry(Reff, gap);
    kw.v1 = Vec3(0, 0, vrn);
    check(close(computeWrench(kw, cfg).Fn[2], expected, real(1e-12) * expected),
          "legacy wall normal magnitude");
  }

  // ---- 8. Feature switches ----
  {
    PairKinematics k = pairGeometry(real(1), real(1), real(0.05));
    k.v1 = Vec3(0.4, 0.1, -0.6);
    k.w1 = Vec3(0.2, -0.5, 1.0);

    ModelConfig off = baseConfig();
    off.enabled = false;
    PairWrench wOff = computeWrench(k, off);
    check(vzero(wOff.Fn, real(0)) && vzero(wOff.Ft, real(0)) &&
          vzero(wOff.M1, real(0)) && vzero(wOff.M2, real(0)), "enabled=false zeroes all");

    ModelConfig noTan = baseConfig();
    noTan.tangential = false;
    noTan.twisting = false;
    PairWrench wNoTan = computeWrench(k, noTan);
    check(vzero(wNoTan.Ft, real(0)) && vzero(wNoTan.M1, real(0)) &&
          vzero(wNoTan.M2, real(0)), "tangential+twisting off leaves only Fn");
    check(!vzero(wNoTan.Fn, tol), "normal force still active");

    ModelConfig noSep = baseConfig();
    noSep.resistSeparation = false;
    PairKinematics ks = pairGeometry(real(1), real(1), real(0.05));
    ks.v1 = Vec3(0, 0, 0.5);  // separating
    check(vzero(computeWrench(ks, noSep).Fn, real(0)),
          "resistSeparation=false skips normal suction");
    ks.v1 = Vec3(0, 0, -0.5);
    check(!vzero(computeWrench(ks, noSep).Fn, tol),
          "resistSeparation=false keeps approach resistance");

    // wallTerms=false: wall pair evaluated with pair coefficients (same aRef)
    ModelConfig noWall = baseConfig();
    noWall.wallTerms = false;
    const real h = real(0.1);
    check(close(normalResistance(h, real(1), true, noWall),
                normalResistance(h, real(1), false, noWall), real(0)),
          "wallTerms=false falls back to pair coefficients");

    // slip correction reduces every resistance
    ModelConfig slip = baseConfig();
    slip.epsCritical = real(0.1);
    slip.slipCorrection = true;
    ModelConfig noSlip = slip;
    noSlip.slipCorrection = false;
    const real hs = real(0.2);
    check(normalResistance(hs, real(1), false, slip) <
              normalResistance(hs, real(1), false, noSlip),
          "slip correction reduces normal resistance");
  }

  // ---- 9. Params helpers: lubricationCutoff and aabbPadding ----
  {
    // legacy absolute mode (cutoffFactor = 0)
    setEnabled(true);
    setAabbInflation(true);
    setCutoffFactor(real(0));
    setLubricationThreshold(real(1e-2));
    check(close(lubricationCutoff(real(5)), real(1e-2), real(0)),
          "cutoff: legacy absolute fallback ignores aRef");
    check(close(aabbPadding(real(5)), real(1e-2), real(0)),
          "padding: legacy absolute fallback ignores radius");

    // relative mode
    setCutoffFactor(real(0.5));
    setMeshClampFactor(real(0));
    check(close(lubricationCutoff(real(2)), real(1), real(0)),
          "cutoff: relative mode = factor * aRef");
    check(close(aabbPadding(real(2)), real(1), real(0)),
          "padding: relative mode = factor * radius");

    // mesh clamp binds only when smaller and both factors set
    setMeshClampFactor(real(2));
    setMeshDx(real(0.1));
    check(close(lubricationCutoff(real(2)), real(0.2), real(0)),
          "cutoff: mesh clamp c*dx binds when smaller");
    check(close(lubricationCutoff(real(0.1)), real(0.05), real(0)),
          "cutoff: relative bound binds when smaller than clamp");
    // D2.2 §3.4: padding tracks the EFFECTIVE cutoff, mesh clamp included -- never
    // inflate by more than the band the model can act on. Pair detection keeps a 2x
    // margin regardless, because both partners carry the padding.
    check(close(aabbPadding(real(2)), real(0.2), real(0)),
          "padding: mesh clamp IS applied to padding (effective cutoff)");
    check(close(aabbPadding(real(0.1)), real(0.05), real(0)),
          "padding: relative bound still binds when smaller than the clamp");
    setMeshDx(real(0));
    check(close(aabbPadding(real(2)), real(1), real(0)),
          "padding: clamp inactive without dx -> relative band");
    check(close(lubricationCutoff(real(2)), real(1), real(0)),
          "cutoff: clamp inactive without dx");

    // switches
    setAabbInflation(false);
    check(close(aabbPadding(real(2)), real(0), real(0)), "padding: inflation off -> 0");
    setAabbInflation(true);
    setEnabled(false);
    check(close(aabbPadding(real(2)), real(0), real(0)), "padding: lubrication off -> 0");
    setEnabled(true);

    // max sphere radius registration (plane padding path)
    registerSphereRadius(real(0.3));
    registerSphereRadius(real(0.7));
    registerSphereRadius(real(0.5));
    check(close(getMaxSphereRadius(), real(0.7), real(0)),
          "registerSphereRadius keeps the maximum");
    check(close(aabbPadding(getMaxSphereRadius()), real(0.35), real(0)),
          "plane padding uses factor * R_max");
  }

  if (failures == 0) {
    std::cout << "pe-lubrication-model: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-model: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
