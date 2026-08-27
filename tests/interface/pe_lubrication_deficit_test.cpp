//=================================================================================================
/*!
 *  \file tests/interface/pe_lubrication_deficit_test.cpp
 *  \brief Pure-arithmetic unit test for modelKroupaDeficit (gate G2 follow-up).
 *
 *  The deficit form subtracts every resistance coefficient's value at the pair's
 *  activation gap hCut: C_def(eps) = max(0, C(eps) - C(eps_cut)). Asserted here:
 *    1. exact identity K_def(h) = K_full(h) - K_full(hCut) for h < hCut (slip off,
 *       saturation off, so the coefficient subtraction is the whole story);
 *    2. continuous onset: K_def(hCut) = 0 and the wrench normal force vanishes at
 *       and above the activation gap;
 *    3. the full model is bit-unchanged when modelKroupa2016 is selected (deficit
 *       machinery inert);
 *    4. hCut = 0 (unknown) leaves the deficit model identical to the full model -
 *       the fail-safe when a caller never sets the activation gap;
 *    5. monotonicity: K_def > 0 strictly inside the band and grows as h shrinks.
 *
 *  No MPI, no world setup - exercises LubricationModel.h only.
 */
//=================================================================================================

#include <pe/core/lubrication/LubricationModel.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using pe::real;
using namespace pe::lubrication;

static int failures = 0;

static void check( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static void checkNear( real a, real b, real tol, const char* what )
{
   const real scale = std::max( real(1), std::max( std::fabs(a), std::fabs(b) ) );
   if( std::fabs( a - b ) > tol * scale ) {
      std::printf( "FAIL: %s  (%.17g vs %.17g)\n", what, double(a), double(b) );
      ++failures;
   }
}

int main()
{
   // Bare-resistance configuration (the G2 configuration): slip off, saturation off.
   ModelConfig cfg;
   cfg.enabled          = true;
   cfg.tangential       = true;
   cfg.twisting         = true;
   cfg.slipCorrection   = false;
   cfg.resistSeparation = true;
   cfg.wallTerms        = true;
   cfg.model            = modelKroupaDeficit;
   cfg.scheme           = schemeSemiImplicit;
   cfg.epsCritical      = real(0);
   cfg.cutoffFactor     = real(0.5);
   cfg.minGap           = real(1e-9);
   cfg.alphaImpulseCap  = real(1);
   cfg.viscosity        = real(0.373);

   ModelConfig full = cfg;
   full.model = modelKroupa2016;

   const real aRef = real(0.0075);
   const real hCut = real(2) * real(6.27151451e-4);   // the G2 activation gap 2h

   for( int wall = 0; wall <= 1; ++wall ) {
      const bool w = ( wall == 1 );

      // 1. exact subtraction identity inside the band
      for( real f : { real(0.1), real(0.3), real(0.6), real(0.9) } ) {
         const real h = f * hCut;
         checkNear( normalResistance( h, aRef, w, cfg, hCut ),
                    normalResistance( h, aRef, w, full ) - normalResistance( hCut, aRef, w, full ),
                    real(1e-12), "normal deficit identity" );
         checkNear( slidingResistance( h, aRef, w, cfg, hCut ),
                    slidingResistance( h, aRef, w, full ) - slidingResistance( hCut, aRef, w, full ),
                    real(1e-12), "sliding deficit identity" );
         // Twisting: the coefficient ~ eps*log(eps) DECREASES toward contact for
         // eps < 1/e, so its raw difference is negative and the deficit clamps to
         // zero - the resolved flow already carries a resistance that weakens near
         // contact. Assert the CLAMPED identity (which is the implementation's
         // contract for every coefficient).
         checkNear( twistingResistance( h, aRef, w, cfg, hCut ),
                    std::max( real(0),
                              twistingResistance( h, aRef, w, full )
                              - twistingResistance( hCut, aRef, w, full ) ),
                    real(1e-12), "twisting deficit clamped identity" );
      }

      // 2. continuous onset: zero at and above the activation gap
      check( normalResistance( hCut, aRef, w, cfg, hCut ) == real(0), "K_n(hCut) == 0" );
      check( normalResistance( real(1.5) * hCut, aRef, w, cfg, hCut ) == real(0),
             "K_n above band == 0" );

      // 4. hCut = 0 fail-safe: deficit model falls back to the full resistance
      checkNear( normalResistance( real(0.5) * hCut, aRef, w, cfg, real(0) ),
                 normalResistance( real(0.5) * hCut, aRef, w, full ),
                 real(1e-15), "hCut=0 fail-safe" );

      // 5. monotone growth toward contact
      const real k1 = normalResistance( real(0.8) * hCut, aRef, w, cfg, hCut );
      const real k2 = normalResistance( real(0.4) * hCut, aRef, w, cfg, hCut );
      const real k3 = normalResistance( real(0.1) * hCut, aRef, w, cfg, hCut );
      check( k1 > real(0) && k2 > k1 && k3 > k2, "monotone in-band growth" );
   }

   // 2b/3. wrench-level: normal force vanishes at activation under the deficit model
   // and is unchanged under the full model regardless of hCut.
   PairKinematics k;
   k.n    = pe::Vec3( 0, 0, 1 );
   k.r1   = pe::Vec3( 0, 0, -aRef );
   k.r2   = pe::Vec3( 0, 0, 0 );
   k.v1   = pe::Vec3( 0, 0, -real(0.02) );   // approaching the wall
   k.v2   = pe::Vec3( 0, 0, 0 );
   k.w1   = pe::Vec3( 0, 0, 0 );
   k.w2   = pe::Vec3( 0, 0, 0 );
   k.aRef = aRef;
   k.wall = true;

   k.h = hCut;  k.hCut = hCut;
   check( computeWrench( k, cfg ).Fn.sqrLength() == real(0), "wrench Fn == 0 at activation" );

   k.h = real(0.3) * hCut;
   const pe::Vec3 FnDef = computeWrench( k, cfg ).Fn;
   check( FnDef.sqrLength() > real(0), "wrench Fn > 0 inside band" );

   const pe::Vec3 FnFull      = computeWrench( k, full ).Fn;
   PairKinematics k0 = k;  k0.hCut = real(0);
   const pe::Vec3 FnFullNoCut = computeWrench( k0, full ).Fn;
   checkNear( FnFull[2], FnFullNoCut[2], real(1e-15), "full model ignores hCut" );
   check( std::fabs( FnDef[2] ) < std::fabs( FnFull[2] ), "deficit < full inside band" );

   if( failures == 0 ) {
      std::printf( "pe_lubrication_deficit_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_lubrication_deficit_test: %d FAILURES\n", failures );
   return EXIT_FAILURE;
}
