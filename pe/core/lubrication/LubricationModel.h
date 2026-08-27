//=================================================================================================
/*!
 *  \file pe/core/lubrication/LubricationModel.h
 *  \brief Pairwise lubrication force/torque model (Kroupa et al., Langmuir 2016)
 *
 *  This header isolates the complete pairwise lubrication closure so it can be unit-tested
 *  without standing up a full PE world (MPI, domain, body storage, collision pipeline).
 *  The shared lubrication stage (pe/core/lubrication/LubricationStage.h) calls into it
 *  from its contact loop, on behalf of whichever collision system invoked the stage.
 *
 *  Model reference:
 *    M. Kroupa, M. Vonka, M. Soos, J. Kosek, "Utilizing the Discrete Element Method for the
 *    Modeling of Viscosity in Concentrated Suspensions", Langmuir 2016, 32, 8451-8460
 *    (DOI 10.1021/acs.langmuir.6b02335). Pair resistances: eqs 12-15 (Dance & Maxey 2003);
 *    wall resistances: eqs 16-19; Vinogradova slip correction f*: eq 20.
 *
 *  Conventions (PE, not the paper):
 *    - The contact normal n points from body 2 toward body 1 (see
 *      doc/technical-notes/collision-detection-conventions.md). The paper's normal points
 *      from particle i to particle j; the transcription below absorbs the sign flip.
 *    - vrn = n . (v1 - v2 + w1 x r1 - w2 x r2) is NEGATIVE on approach.
 *    - All force/torque signs are fixed by dissipativity (power <= 0), not by the paper's
 *      printed signs; the twisting torque in particular is sign-corrected relative to eq 15
 *      (as printed it would pump energy into the relative spin).
 *    - For wall pairs (wall == true), body 1 MUST be the sphere and body 2 the plane; the
 *      caller canonicalizes the order.
 *
 *  Design note: PairWrench splits the normal force (Fn) from the tangential force (Ft) --
 *  deviating from the single-F wrench in doc/technical-notes/lubrication-production-design.md
 *  -- because the semi-implicit integrator treats the (stiff) normal mode exactly and only
 *  needs the soft tangential terms explicitly. M1/M2 are the PURE lubrication torques
 *  (sliding + twisting); the lever-arm torque r x F is added by the caller.
 */
//=================================================================================================

#ifndef _PE_CORE_LUBRICATION_LUBRICATIONMODEL_H_
#define _PE_CORE_LUBRICATION_LUBRICATIONMODEL_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>
#include <algorithm>
#include <cmath>


namespace pe {
namespace lubrication {

//=================================================================================================
//
//  MODEL SELECTION ENUMS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Selects the pairwise lubrication force model. */
enum ModelKind {
   modelKroupa2016   = 0,  //!< Full resistance set of Kroupa et al. 2016 (default)
   modelLegacy       = 1,  //!< Pre-2026 PE model: leading-order normal term, approach-only
   modelKroupaDeficit = 2  //!< Kroupa resistances MINUS their value at the pair's activation
                           //!< gap hCut (deficit form): every coefficient C(eps) becomes
                           //!< max(0, C(eps) - C(eps_cut)), so the force rises continuously
                           //!< from zero at activation and only the part of the film the
                           //!< resolved CFD flow does NOT already carry is added. Reduces to
                           //!< ten Cate (2002) eq. 10 in the leading normal term; keeps the
                           //!< h_c saturation. Decided by gate G2 (d22_g2_brenner): the full
                           //!< resistance set double-counts the FBM-resolved film by ~+75%
                           //!< in the 1-2h band, the deficit form lands at +7%.
};

/*!\brief Selects the time-integration scheme used by the solver for lubrication impulses. */
enum SchemeKind {
   schemeSemiImplicit  = 0,  //!< Exact exponential update per linear mode (default)
   schemeExplicitCapped = 1  //!< Explicit force x dt with impulse cap (legacy scheme)
};
//*************************************************************************************************


//=================================================================================================
//
//  CONFIG AND DATA STRUCTS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Immutable per-timestep snapshot of all lubrication model switches and parameters.
 *
 * Filled once per timestep by the solver (from lubrication::Params + solver members +
 * Settings); the per-contact loop reads only this local struct.
 */
struct ModelConfig {
   bool enabled;           //!< Master switch for lubrication forces
   bool tangential;        //!< Sliding force F_sl + sliding torque M_sl
   bool twisting;          //!< Twisting torque M_tl
   bool slipCorrection;    //!< Vinogradova slip correction f* (eq 20)
   bool resistSeparation;  //!< Apply the normal term for separating motion (suction) as well
   bool wallTerms;         //!< Use the wall resistance set (eqs 16-19) for sphere-plane pairs
   int  model;             //!< ModelKind
   int  scheme;            //!< SchemeKind
   real epsCritical;       //!< eps_c = h_c / a_ref: saturation distance / slip length scale
   real cutoffFactor;      //!< eps_cut = h_cut / a_ref outer cutoff; 0 = legacy absolute cutoff
   real minGap;            //!< Final numerical gap floor (legacy minEpsLub_)
   real alphaImpulseCap;   //!< Impulse cap factor (schemeExplicitCapped only)
   real viscosity;         //!< Fluid dynamic viscosity eta
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Kinematic and geometric state of one lubrication pair, PE conventions.
 *
 * \a n points from body 2 toward body 1; \a r1 / \a r2 are lever arms from the body centers
 * to the contact point; \a aRef is the reference radius defining eps = h / aRef:
 * 2*R1*R2/(R1+R2) for sphere-sphere (equals R for equal spheres, the paper's R_p) and
 * R_sphere for sphere-plane. For wall pairs body 1 must be the sphere.
 */
struct PairKinematics {
   Vec3 n;     //!< Contact normal (unit, body2 -> body1)
   real h;     //!< Surface gap (may be < minGap; the model applies floor/saturation itself)
   Vec3 r1;    //!< Lever arm of body 1 (contact point - center)
   Vec3 r2;    //!< Lever arm of body 2 (contact point - center)
   Vec3 v1;    //!< Linear velocity of body 1
   Vec3 v2;    //!< Linear velocity of body 2
   Vec3 w1;    //!< Angular velocity of body 1
   Vec3 w2;    //!< Angular velocity of body 2
   real aRef;  //!< Reference radius for eps (see above)
   bool wall;  //!< True for sphere-plane pairs (body 1 = sphere)
   real hCut = real(0);  //!< Pair's effective activation gap (lubricationCutoff(aRef), mesh
                         //!< clamp included), set by the stage. Consumed by modelKroupaDeficit
                         //!< as the subtraction reference; 0 = unknown -> deficit inactive.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Lubrication forces/torques acting on the pair, before blend-weight scaling.
 *
 * \a Fn and \a Ft act on body 1; body 2 receives -Fn - Ft. \a M1 / \a M2 are the pure
 * lubrication torques (sliding + twisting) on bodies 1 and 2; the lever-arm torques
 * r1 x F and r2 x (-F) are the caller's responsibility.
 */
struct PairWrench {
   Vec3 Fn;  //!< Normal squeeze force on body 1
   Vec3 Ft;  //!< Tangential sliding force on body 1
   Vec3 M1;  //!< Pure lubrication torque on body 1
   Vec3 M2;  //!< Pure lubrication torque on body 2
};
//*************************************************************************************************


//=================================================================================================
//
//  SHARED INTERNALS (exposed for unit testing)
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Vinogradova slip correction factor f* (paper eq 20).
 *
 * \param h  Separation distance (> 0).
 * \param hc Slip length h_c (> 0); hc <= 0 returns 1 (no slip correction).
 * \return f* in (0, 1]; f* -> 1 for h >> hc.
 */
inline real slipFactor( real h, real hc )
{
   if( hc <= real(0) || h <= real(0) ) return real(1);
   const real x = h / ( real(3) * hc );
   return x * ( ( real(1) + h / ( real(6) * hc ) ) * std::log( real(1) + real(6) * hc / h ) - real(1) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Effective gap after the numerical floor and the h_c saturation freeze.
 *
 * For h < hc all resistances (including f*) are evaluated at the frozen separation hc,
 * so forces saturate instead of diverging (paper, "Constraining Lubrication Forces").
 */
inline real effectiveGap( real h, real hc, real minGap )
{
   return std::max( std::max( h, minGap ), hc );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Legacy normal lubrication force magnitude.
 *
 * Byte-identical to the expression historically used in HardContactLubricated
 * (6*pi*mu*R_eff^2*(-vrn)/gap); kept as a single function so the legacy code path and its
 * regression test share the exact floating-point evaluation.
 */
inline real legacyForceMagnitude( real gap, real Reff, real mu, real vrn )
{
   return real(6) * M_PI * mu * Reff * Reff * (-vrn) / gap;
}
//*************************************************************************************************


//=================================================================================================
//
//  RESISTANCE COEFFICIENTS
//
//=================================================================================================

namespace detail {

// For eps in (0,1) the log is negative; every coefficient below is written so that it is
// non-negative there, and clamped at zero for eps >= 1 where the truncated asymptotics
// lose validity (can only be reached with extreme cutoff settings).

inline real clampPos( real x ) { return x > real(0) ? x : real(0); }

//! Normal squeeze coefficient C_n(eps): pair eq 12 / wall eq 16.
inline real coeffNormal( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( real(1) / eps - real(1) / real(5) * L - real(1) / real(21) * eps * L );
   return clampPos( real(0.25) / eps - real(9) / real(40) * L - real(3) / real(112) * eps * L );
}

//! Sliding force coefficient of v_s: pair eq 13 / wall eq 17.
inline real coeffSlideVs( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( -real(8) / real(15) * L - real(64) / real(375) * eps * L );
   return clampPos( -real(1) / real(6) * L );
}

//! Sliding force coefficient of v_cs: pair eq 13 / wall eq 17.
inline real coeffSlideVcs( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( -real(2) / real(15) * L - real(86) / real(375) * eps * L );
   return clampPos( -real(1) / real(6) * L - real(1) / real(12) * eps * L );
}

//! Sliding torque coefficient of (n x v_s): pair eq 14 / wall eq 18.
inline real coeffTorqueVs( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( -real(2) / real(15) * L - real(86) / real(375) * eps * L );
   return clampPos( -real(1) / real(6) * L - real(1) / real(12) * eps * L );
}

//! Sliding torque coefficient of (n x v_cs): pair eq 14 / wall eq 18.
inline real coeffTorqueVcs( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( -real(2) / real(5) * L - real(66) / real(125) * eps * L );
   return clampPos( -real(1) / real(5) * L - real(47) / real(250) * eps * L );
}

//! Twisting torque coefficient: pair eq 15 / wall eq 19 (magnitude; sign fixed by dissipativity).
inline real coeffTwist( real eps, bool wall )
{
   const real L = std::log( eps );
   if( wall )
      return clampPos( -real(0.5) * eps * L );
   return clampPos( -real(0.125) * eps * L );
}

//! Effective gap and eps for a pair under the current config.
inline void gapAndEps( real h, real aRef, const ModelConfig& cfg, real& hEff, real& eps )
{
   const real hc = cfg.epsCritical * aRef;
   hEff = effectiveGap( h, hc, cfg.minGap );
   eps  = hEff / aRef;
}

//! Combined slip factor under the current config (1 when disabled).
inline real slip( real hEff, real aRef, const ModelConfig& cfg )
{
   if( !cfg.slipCorrection ) return real(1);
   return slipFactor( hEff, cfg.epsCritical * aRef );
}

} // namespace detail


//=================================================================================================
//
//  SCALAR RESISTANCES (for the semi-implicit integrator)
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Normal squeeze resistance K_n >= 0 such that F_n = -K_n * vrn * n.
 */
inline real normalResistance( real h, real aRef, bool wall, const ModelConfig& cfg,
                              real hCut = real(0) )
{
   real hEff, eps;
   detail::gapAndEps( h, aRef, cfg, hEff, eps );
   const bool useWall = wall && cfg.wallTerms;
   real c = detail::coeffNormal( eps, useWall );
   if( cfg.model == modelKroupaDeficit && hCut > real(0) ) {
      real hEffCut, epsCut;
      detail::gapAndEps( hCut, aRef, cfg, hEffCut, epsCut );
      c = std::max( real(0), c - detail::coeffNormal( epsCut, useWall ) );
   }
   return real(6) * M_PI * cfg.viscosity * aRef * c
          * detail::slip( hEff, aRef, cfg );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sliding resistance K_s >= 0 (v_s self-coefficient) such that F_t ~ -K_s * v_s.
 */
inline real slidingResistance( real h, real aRef, bool wall, const ModelConfig& cfg,
                               real hCut = real(0) )
{
   real hEff, eps;
   detail::gapAndEps( h, aRef, cfg, hEff, eps );
   const bool useWall = wall && cfg.wallTerms;
   real c = detail::coeffSlideVs( eps, useWall );
   if( cfg.model == modelKroupaDeficit && hCut > real(0) ) {
      real hEffCut, epsCut;
      detail::gapAndEps( hCut, aRef, cfg, hEffCut, epsCut );
      c = std::max( real(0), c - detail::coeffSlideVs( epsCut, useWall ) );
   }
   return real(6) * M_PI * cfg.viscosity * aRef * c
          * detail::slip( hEff, aRef, cfg );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Twisting resistance K_t >= 0 such that M_t1 = -K_t * ((w1-w2).n) * n.
 */
inline real twistingResistance( real h, real aRef, bool wall, const ModelConfig& cfg,
                                real hCut = real(0) )
{
   real hEff, eps;
   detail::gapAndEps( h, aRef, cfg, hEff, eps );
   const bool useWall = wall && cfg.wallTerms;
   real c = detail::coeffTwist( eps, useWall );
   if( cfg.model == modelKroupaDeficit && hCut > real(0) ) {
      real hEffCut, epsCut;
      detail::gapAndEps( hCut, aRef, cfg, hEffCut, epsCut );
      c = std::max( real(0), c - detail::coeffTwist( epsCut, useWall ) );
   }
   return real(8) * M_PI * cfg.viscosity * aRef * aRef * c
          * detail::slip( hEff, aRef, cfg );
}
//*************************************************************************************************


//=================================================================================================
//
//  FULL PAIR WRENCH
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Complete lubrication wrench for one pair, before blend-weight scaling and integration.
 *
 * Kinematic decomposition (PE conventions, see file header):
 *   g    = v1 - v2 + w1 x r1 - w2 x r2      (relative surface velocity at the contact)
 *   vrn  = n . g                            (< 0 on approach)
 *   v_s  = (v1 - v2) - ((v1 - v2) . n) n    (translational sliding velocity)
 *   v_cs = tangential part of (w1 x r1 - w2 x r2)  (rotational sliding velocity)
 *   spin = (w1 - w2) . n                    (relative twist rate)
 *
 * Pair sliding torque acts with the SAME vector on both bodies (i <-> j symmetry of eq 14);
 * the twisting torque acts with opposite signs. For wall pairs M2 = 0 (an infinite plane
 * takes no pure lubrication torque; the reaction force -F still acts on it via the caller).
 */
inline PairWrench computeWrench( const PairKinematics& k, const ModelConfig& cfg )
{
   PairWrench wr;
   wr.Fn = Vec3();
   wr.Ft = Vec3();
   wr.M1 = Vec3();
   wr.M2 = Vec3();

   if( !cfg.enabled || k.aRef <= real(0) ) return wr;

   const Vec3 vRel = k.v1 - k.v2;
   const Vec3 g    = vRel + k.w1 % k.r1 - k.w2 % k.r2;
   const real vrn  = trans( k.n ) * g;

   // ---- Legacy model: leading-order normal term, approach-only ----
   if( cfg.model == modelLegacy ) {
      if( vrn < real(0) ) {
         const real gap  = std::max( k.h, cfg.minGap );
         const real Reff = k.wall ? k.aRef : real(0.5) * k.aRef;
         wr.Fn = legacyForceMagnitude( gap, Reff, cfg.viscosity, vrn ) * k.n;
      }
      return wr;
   }

   // ---- Kroupa 2016 model (full or deficit form) ----
   real hEff, eps;
   detail::gapAndEps( k.h, k.aRef, cfg, hEff, eps );
   const bool useWall = k.wall && cfg.wallTerms;
   const real fs      = detail::slip( hEff, k.aRef, cfg );

   // Deficit form: every coefficient is evaluated relative to its value at the pair's
   // activation gap, clamped at zero - continuous onset, and only the unresolved part
   // of the film is added (see modelKroupaDeficit). Slip stays evaluated at the pair's
   // own gap and multiplies the deficit coefficient.
   const bool deficit = ( cfg.model == modelKroupaDeficit && k.hCut > real(0) );
   real epsCut = real(0);
   if( deficit ) {
      real hEffCut;
      detail::gapAndEps( k.hCut, k.aRef, cfg, hEffCut, epsCut );
   }
   auto dc = [&]( real (*coeff)( real, bool ) ) {
      const real c = coeff( eps, useWall );
      return deficit ? std::max( real(0), c - coeff( epsCut, useWall ) ) : c;
   };
   const real preF    = real(6) * M_PI * cfg.viscosity * k.aRef * fs;
   const real preM    = real(8) * M_PI * cfg.viscosity * k.aRef * k.aRef * fs;

   // Normal squeeze force: F_n = -K_n vrn n (resists approach AND separation unless disabled)
   if( vrn < real(0) || cfg.resistSeparation ) {
      wr.Fn = -( preF * dc( detail::coeffNormal ) ) * vrn * k.n;
   }

   if( cfg.tangential ) {
      const Vec3 vs  = vRel - ( trans( vRel ) * k.n ) * k.n;
      const Vec3 vcsFull = k.w1 % k.r1 - k.w2 % k.r2;
      const Vec3 vcs = vcsFull - ( trans( vcsFull ) * k.n ) * k.n;

      // Sliding force on body 1 (paper eq 13/17; no n-dependence, transcribes directly)
      wr.Ft = -preF * ( dc( detail::coeffSlideVs ) * vs
                      + dc( detail::coeffSlideVcs ) * vcs );

      // Sliding torque (paper eq 14/18 uses the paper normal n_p = -n; the flip makes the
      // leading sign positive in PE convention). Same vector on both bodies for pairs.
      const Vec3 Msl = preM * ( dc( detail::coeffTorqueVs ) * ( k.n % vs )
                              + dc( detail::coeffTorqueVcs ) * ( k.n % vcs ) );
      wr.M1 += Msl;
      if( !k.wall ) wr.M2 += Msl;
   }

   if( cfg.twisting ) {
      // Twisting torque: dissipative form M_t1 = -K_t spin n (sign-corrected vs printed eq 15/19)
      const real spin = trans( k.n ) * ( k.w1 - k.w2 );
      const Vec3 Mt   = -( preM * dc( detail::coeffTwist ) ) * spin * k.n;
      wr.M1 += Mt;
      if( !k.wall ) wr.M2 -= Mt;
   }

   return wr;
}
//*************************************************************************************************

} // namespace lubrication
} // namespace pe

#endif
