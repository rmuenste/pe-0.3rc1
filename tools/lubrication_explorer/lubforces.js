(function(){
//=================================================================================================
//  lubforces.js — faithful JS port of the PE Kroupa-2016 lubrication model
//
//  SYNC CONTRACT: this file mirrors, function-for-function, the C++ source of truth:
//    - pe/core/lubrication/LubricationModel.h        (coefficients, slip, saturation, wrench)
//    - pe/core/lubrication/LubricationStage.h        (per-step impulse: semi-implicit /
//                                                     explicit-capped, expClamp)
//    - pe/core/detection/fine/MaxContacts.h          (blend-weight ramp)
//    - src/core/lubrication/Params.cpp               (outer cutoff)
//  Every coefficient below carries its source line. If you change the model in C++, change it
//  here and re-run the self-check (see selfCheck() and README.md). Do NOT invent formulas.
//
//  This is a MODEL explorer, not a simulation: it evaluates the analytic pair/wall laws for
//  scalar kinematics, exactly as computeWrench() does for the orthogonal decomposition
//  (|n x v_s| = |v_s| since v_s ⟂ n, etc.).
//=================================================================================================

const PI = Math.PI;

// Enum values — LubricationModel.h:59-68
const MODEL_KROUPA = 0, MODEL_LEGACY = 1;
const SCHEME_SEMI_IMPLICIT = 0, SCHEME_EXPLICIT_CAPPED = 1;

// contactThreshold for double precision — pe/core/Thresholds.h:166
const CONTACT_THRESHOLD = 1e-8;

//=================================================================================================
//  Shared internals — LubricationModel.h:147-202
//=================================================================================================

// clampPos — :202. Coefficients are written non-negative for eps in (0,1); zeroed for eps >= 1.
const clampPos = (x) => (x > 0 ? x : 0);

// Vinogradova slip correction f* — :154-159 (paper eq 20). hc <= 0 -> 1 (disabled).
function slipFactor(h, hc) {
   if (hc <= 0 || h <= 0) return 1;
   const x = h / (3 * hc);
   return x * ((1 + h / (6 * hc)) * Math.log(1 + (6 * hc) / h) - 1);
}

// Effective gap after numerical floor and the hc saturation freeze — :169-172
function effectiveGap(h, hc, minGap) {
   return Math.max(Math.max(h, minGap), hc);
}

// Legacy normal force magnitude — :183-186
function legacyForceMagnitude(gap, Reff, mu, vrn) {
   return 6 * PI * mu * Reff * Reff * (-vrn) / gap;
}

//=================================================================================================
//  Resistance coefficients C(eps) — LubricationModel.h:204-256. L = ln(eps) (< 0 for eps<1).
//=================================================================================================

// Normal squeeze C_n — :205-211 (pair eq 12 / wall eq 16)
function coeffNormal(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(1 / eps - (1 / 5) * L - (1 / 21) * eps * L);
   return clampPos(0.25 / eps - (9 / 40) * L - (3 / 112) * eps * L);
}
// Sliding force coeff of v_s — :214-220 (pair eq 13 / wall eq 17)
function coeffSlideVs(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(-(8 / 15) * L - (64 / 375) * eps * L);
   return clampPos(-(1 / 6) * L);
}
// Sliding force coeff of v_cs — :223-229
function coeffSlideVcs(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(-(2 / 15) * L - (86 / 375) * eps * L);
   return clampPos(-(1 / 6) * L - (1 / 12) * eps * L);
}
// Sliding torque coeff of (n x v_s) — :232-238
function coeffTorqueVs(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(-(2 / 15) * L - (86 / 375) * eps * L);
   return clampPos(-(1 / 6) * L - (1 / 12) * eps * L);
}
// Sliding torque coeff of (n x v_cs) — :241-247
function coeffTorqueVcs(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(-(2 / 5) * L - (66 / 125) * eps * L);
   return clampPos(-(1 / 5) * L - (47 / 250) * eps * L);
}
// Twisting coeff — :250-256 (magnitude; sign fixed dissipative in computeWrench)
function coeffTwist(eps, wall) {
   const L = Math.log(eps);
   if (wall) return clampPos(-0.5 * eps * L);
   return clampPos(-0.125 * eps * L);
}

// gapAndEps — :259-264
function gapAndEps(h, aRef, cfg) {
   const hc = cfg.epsCritical * aRef;
   const hEff = effectiveGap(h, hc, cfg.minGap);
   return { hEff, eps: hEff / aRef };
}
// slip under config — :267-271
function slip(hEff, aRef, cfg) {
   if (!cfg.slipCorrection) return 1;
   return slipFactor(hEff, cfg.epsCritical * aRef);
}

//=================================================================================================
//  Scalar resistances (semi-implicit integrator) — LubricationModel.h:285-320
//=================================================================================================

function normalResistance(h, aRef, wall, cfg) {
   const { hEff, eps } = gapAndEps(h, aRef, cfg);
   const useWall = wall && cfg.wallTerms;
   return 6 * PI * cfg.viscosity * aRef * coeffNormal(eps, useWall) * slip(hEff, aRef, cfg);
}
function slidingResistance(h, aRef, wall, cfg) {
   const { hEff, eps } = gapAndEps(h, aRef, cfg);
   const useWall = wall && cfg.wallTerms;
   return 6 * PI * cfg.viscosity * aRef * coeffSlideVs(eps, useWall) * slip(hEff, aRef, cfg);
}
function twistingResistance(h, aRef, wall, cfg) {
   const { hEff, eps } = gapAndEps(h, aRef, cfg);
   const useWall = wall && cfg.wallTerms;
   return 8 * PI * cfg.viscosity * aRef * aRef * coeffTwist(eps, useWall) * slip(hEff, aRef, cfg);
}

//=================================================================================================
//  Continuous component magnitudes — scalar reduction of computeWrench() (LubricationModel.h:344)
//
//  Kinematic scalars (all signed, user inputs):
//    vn     approach speed (> 0 approaching). vrn = -vn (< 0 on approach), matches file header.
//    vs     translational sliding speed  (drives coeffSlide/TorqueVs)
//    vcs    rotational sliding surface speed (drives coeffSlide/TorqueVcs)
//    wtw    relative twist rate spin = (w1-w2).n
//  Returns magnitudes { normalF, slideF, slideM, twistM } acting on body 1, pre-blend.
//  Because v_s ⟂ n and v_cs ⟂ n, |n x v_s| = |v_s| and |n x v_cs| = |v_cs|.
//=================================================================================================

function components(h, aRef, wall, kin, cfg) {
   const out = { normalF: 0, slideF: 0, slideM: 0, twistM: 0 };
   if (!cfg.enabled || aRef <= 0) return out;

   const vrn = -kin.vn;

   // Legacy model: leading-order normal term, approach-only — :359-364
   if (cfg.model === MODEL_LEGACY) {
      if (vrn < 0) {
         const gap = Math.max(h, cfg.minGap);
         const Reff = wall ? aRef : 0.5 * aRef;
         out.normalF = legacyForceMagnitude(gap, Reff, cfg.viscosity, vrn);
      }
      return out;
   }

   // Kroupa 2016 — :368-404
   const { hEff, eps } = gapAndEps(h, aRef, cfg);
   const useWall = wall && cfg.wallTerms;
   const fs = slip(hEff, aRef, cfg);
   const preF = 6 * PI * cfg.viscosity * aRef * fs;
   const preM = 8 * PI * cfg.viscosity * aRef * aRef * fs;

   // Normal squeeze F_n = -K_n vrn n (approach, and separation iff resistSeparation) — :377-379
   if (vrn < 0 || cfg.resistSeparation) {
      out.normalF = -(preF * coeffNormal(eps, useWall)) * vrn; // = preF*Cn*vn
   }
   if (cfg.tangential) {
      // Sliding force — :387-388
      out.slideF = -preF * (coeffSlideVs(eps, useWall) * kin.vs + coeffSlideVcs(eps, useWall) * kin.vcs);
      // Sliding torque (|n x v| = |v|) — :392-393
      out.slideM = preM * (coeffTorqueVs(eps, useWall) * kin.vs + coeffTorqueVcs(eps, useWall) * kin.vcs);
   }
   if (cfg.twisting) {
      // Twist torque, dissipative sign — :400-401
      out.twistM = -(preM * coeffTwist(eps, useWall)) * kin.wtw; // magnitude tracks -spin
   }
   return out;
}

//=================================================================================================
//  Blend weight — MaxContacts.h:93-177 (rampUp/rampDown/computeLubricationWeight)
//  dist = signed gap h; threshold = contactThreshold; lubricationThreshold = h_cut.
//=================================================================================================

function rampUp(x, start, end) {
   if (end <= start) return x >= end ? 1 : 0;
   if (x <= start) return 0;
   if (x >= end) return 1;
   return (x - start) / (end - start);
}
function rampDown(x, start, end) {
   if (end <= start) return x <= start ? 1 : 0;
   if (x <= start) return 1;
   if (x >= end) return 0;
   return (end - x) / (end - start);
}
const clamp01 = (x) => Math.max(0, Math.min(1, x));

function lubricationWeight(h, hCut, cfg) {
   const threshold = CONTACT_THRESHOLD;
   const contactBlend = cfg.contactHysteresis;
   const lubBlend = cfg.lubricationHysteresis;
   const entryStart = threshold - contactBlend, entryEnd = threshold + contactBlend;
   const exitCenter = threshold + hCut, exitStart = exitCenter - lubBlend, exitEnd = exitCenter + lubBlend;
   const up = contactBlend > 0 ? rampUp(h, entryStart, entryEnd) : (h > threshold ? 1 : 0);
   const down = lubBlend > 0 ? rampDown(h, exitStart, exitEnd) : (h < exitCenter ? 1 : 0);
   return clamp01(up * down);
}

//=================================================================================================
//  Outer cutoff h_cut — Params.cpp:115-124
//=================================================================================================

function lubricationCutoff(aRef, cfg) {
   if (cfg.cutoffFactor <= 0) return cfg.legacyThreshold; // legacy absolute
   return cfg.cutoffFactor * aRef;                        // relative (mesh clamp omitted: off in tool)
}

//=================================================================================================
//  Per-step EFFECTIVE force = blend * impulse / dt — LubricationStage.h
//  Returns dimensional { normalF, slideF, slideM, twistM } as applied by the solver.
//=================================================================================================

const expClamp = (x) => (x > 1e-8 ? (1 - Math.exp(-x)) / x : 1); // :2244-2246

function effectiveComponents(h, aRef, wall, kin, cfg, step) {
   const out = { normalF: 0, slideF: 0, slideM: 0, twistM: 0 };
   if (!cfg.enabled || aRef <= 0) return out;

   const hCut = lubricationCutoff(aRef, cfg);
   const blend = lubricationWeight(h, hCut, cfg);
   if (blend <= 0) return out;

   const cont = components(h, aRef, wall, kin, cfg); // continuous magnitudes
   const { dt, mEff, iEff } = step;
   const vrn = -kin.vn;

   if (cfg.model === MODEL_LEGACY) {
      // Legacy path takes the explicit-capped branch structure; normal-only.
      const FnMag = cont.normalF;
      out.normalF = applyNormal(FnMag, vrn, blend, cfg, step);
      return out;
   }

   if (cfg.scheme === SCHEME_SEMI_IMPLICIT) {
      // Normal: exact exponential update, |Jn| < mEff|vrn| — :2408-2411
      if (vrn < 0 || cfg.resistSeparation) {
         const Kn = normalResistance(h, aRef, wall, cfg);
         const Jn = mEff * (1 - Math.exp(-Kn * dt / mEff)) * kin.vn; // = -mEff(1-e)vrn
         out.normalF = blend * Jn / dt;
      }
      // Tangential force with its own exp clamp — :2415-2416
      const Ks = slidingResistance(h, aRef, wall, cfg);
      out.slideF = blend * cont.slideF * expClamp(Ks * dt / mEff);
      // Sliding torque (cSlide) and twist (cTwist) clamps — :2421-2435
      const Kt = twistingResistance(h, aRef, wall, cfg);
      const cSlide = iEff > 0 ? expClamp(Ks * aRef * aRef * dt / iEff) : 1;
      const cTwist = iEff > 0 ? expClamp(Kt * dt / iEff) : 1;
      out.slideM = blend * cont.slideM * cSlide;
      out.twistM = blend * cont.twistM * cTwist;
   } else {
      // Explicit force x dt, normal capped — :2437-2448
      out.normalF = applyNormal(cont.normalF, vrn, blend, cfg, step);
      out.slideF = blend * cont.slideF;
      out.slideM = blend * cont.slideM;
      out.twistM = blend * cont.twistM;
   }
   return out;
}

// Explicit-capped normal: cap the proportional impulse at alpha*blend*mEff*|vrn| — :2440-2445
function applyNormal(FnMag, vrn, blend, cfg, step) {
   const { dt, mEff } = step;
   let Fn = FnMag;
   const JnProp = Math.abs(Fn) * dt;
   const Jcap = cfg.alphaImpulseCap * blend * mEff * Math.abs(vrn);
   if (JnProp > Jcap && JnProp > 0) Fn *= Jcap / JnProp;
   return blend * Fn;
}

//=================================================================================================
//  Reference radius / geometry
//=================================================================================================

// a_ref defines eps = h/aRef — LubricationStage.h
function refRadius(wall, r1, r2) {
   if (wall) return r1;                 // sphere-plane: a_ref = R_sphere
   return (2 * r1 * r2) / (r1 + r2);    // sphere-sphere: a_ref = 2 R1 R2/(R1+R2)
}

//=================================================================================================
//  Self-check — reproduces the Level-0 evidence (pe_lubrication_model_test): wall normal
//  plateau F/(v R eta) = 6*pi*Cn(eps_c)*f*(hc,hc) = 405/805/1605/3202 for eps_c
//  0.02/0.01/0.005/0.0025, and pair leading term ~ 1/4 of wall. Returns {pass, lines[]}.
//=================================================================================================

function selfCheck() {
   const lines = [];
   let pass = true;
   const expect = { 0.02: 405, 0.01: 805, 0.005: 1605, 0.0025: 3202 };
   for (const epsC of [0.02, 0.01, 0.005, 0.0025]) {
      // At saturation eps = eps_c, hEff = hc; dimensionless normal ordinate = 6*pi*Cn*f*.
      const fs = slipFactor(1, 1); // f*(hc,hc): depends only on ratio -> use h=hc=1
      const val = 6 * PI * coeffNormal(epsC, true) * fs;
      const ref = expect[epsC];
      const ok = Math.abs(val - ref) / ref < 0.01;
      pass = pass && ok;
      lines.push(`${ok ? "OK " : "FAIL"}  wall plateau eps_c=${epsC}: ${val.toFixed(1)} (paper ~${ref})`);
   }
   // Pair leading term is 1/4 of wall (0.25/eps vs 1/eps).
   const ratio = coeffNormal(1e-4, false) / coeffNormal(1e-4, true);
   const ok = Math.abs(ratio - 0.25) < 0.02;
   pass = pass && ok;
   lines.push(`${ok ? "OK " : "FAIL"}  pair/wall normal leading ratio: ${ratio.toFixed(3)} (~0.25)`);
   return { pass, lines };
}

//=================================================================================================
//  Global namespace (classic script; no ES modules so the page opens from file://).
//=================================================================================================
window.L = {
   MODEL_KROUPA, MODEL_LEGACY, SCHEME_SEMI_IMPLICIT, SCHEME_EXPLICIT_CAPPED, CONTACT_THRESHOLD,
   slipFactor, effectiveGap, legacyForceMagnitude,
   coeffNormal, coeffSlideVs, coeffSlideVcs, coeffTorqueVs, coeffTorqueVcs, coeffTwist,
   gapAndEps, normalResistance, slidingResistance, twistingResistance,
   components, effectiveComponents, lubricationWeight, lubricationCutoff, refRadius, selfCheck,
};

})();
