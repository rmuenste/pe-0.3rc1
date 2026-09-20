//=================================================================================================
/*!
 *  \file tests/interface/pe_ellipsoid_support_test.cpp
 *  \brief Pins the ellipsoid support mapping, bounding box and radial depth.
 *
 *  Before this test EllipsoidBase::support() returned gpos + A*d (a sphere of the semi-major
 *  radius), which made every ellipsoid behave as that sphere inside GJK/EPA, and
 *  EllipsoidBase::calcBoundingBox() spanned only the positive axis tips. Asserted here:
 *    1. placeholder gone: at identity orientation the support points along body x/y/z are at
 *       distance A/B/C (not A everywhere);
 *    2. for random rotations and random unit directions the support point lies on the surface
 *       ((x/A)^2+(y/B)^2+(z/C)^2 = 1 to 1e-12) and its outward normal (x/A^2, y/B^2, z/C^2) is
 *       parallel to the query direction (dot > 1-1e-10); it also dominates a cloud of other
 *       surface points in the query direction (support property);
 *    3. supportContactThreshold(d) == support(d) + d*contactThreshold;
 *    4. bounding box vs brute-force sampling of the rotated surface (20k points, then local
 *       refinement of the extremes): every sample is inside the box and the box is tight to
 *       1e-6 plus contactThreshold on all six faces;
 *    5. radial depth: zero on the surface, positive inside, negative outside, and the sphere
 *       formula r - |p| when A = B = C.
 *
 *  Serial world setup, no MPI - same harness class as pe_ellipsoid_inertia_test.
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool close( real x, real y, real tol )
{
   return std::fabs( x - y ) <= tol;
}

// Body-frame point of the ellipsoid surface at spherical parameters (theta, phi)
static Vec3 surfacePoint( real A, real B, real C, real theta, real phi )
{
   return Vec3( A * std::cos( theta ) * std::sin( phi ),
                B * std::sin( theta ) * std::sin( phi ),
                C * std::cos( phi ) );
}

int main()
{
   WorldID world = theWorld();
   (void)world;

   MaterialID mat = createMaterial( "support_test", real(1), real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );

   const real A = real(0.5), B = real(0.25), C = real(0.15);   // triaxial
   const Vec3 center( real(0.3), real(-0.2), real(0.7) );
   EllipsoidID ell = createEllipsoid( 1, center, A, B, C, mat );

   std::mt19937 rng( 12345u );
   std::uniform_real_distribution<real> U( real(-1), real(1) );
   auto randUnit = [&]() {
      Vec3 v;
      do { v = Vec3( U(rng), U(rng), U(rng) ); } while( v.sqrLength() < real(1e-3) || v.sqrLength() > real(1) );
      return v.getNormalized();
   };

   // --- 1. placeholder gone (identity orientation) --------------------------------------------
   expect( close( ( ell->support( Vec3( 1, 0, 0 ) ) - center ).length(), A, real(1e-12) ),
           "support along body x is at distance A" );
   expect( close( ( ell->support( Vec3( 0, 1, 0 ) ) - center ).length(), B, real(1e-12) ),
           "support along body y is at distance B (placeholder returned A)" );
   expect( close( ( ell->support( Vec3( 0, 0, 1 ) ) - center ).length(), C, real(1e-12) ),
           "support along body z is at distance C (placeholder returned A)" );
   expect( close( ( ell->support( Vec3( 0, -1, 0 ) ) - center ).length(), B, real(1e-12) ),
           "support along body -y is at distance B" );

   // --- 2./3. random rotations and directions -------------------------------------------------
   {
      bool onSurface = true, normalParallel = true, dominates = true, thresholdExact = true;
      real worstSurface = 0, worstDot = 1, worstDominance = 0, worstThreshold = 0;
      std::uniform_real_distribution<real> ANG( real(0), real(2) * M_PI );

      for( int r=0; r<50; ++r ) {
         ell->rotate( randUnit(), U(rng) * real(2) );
         const Rot3& R( ell->getRotation() );
         const Vec3 pos( ell->getPosition() );

         for( int k=0; k<200; ++k ) {
            const Vec3 d( randUnit() );
            const Vec3 p( ell->support( d ) );
            const Vec3 pb( trans( R ) * ( p - pos ) );
            const Vec3 db( trans( R ) * d );

            const real f( ( pb[0]*pb[0] )/( A*A ) + ( pb[1]*pb[1] )/( B*B ) + ( pb[2]*pb[2] )/( C*C ) );
            worstSurface = std::max( worstSurface, std::fabs( f - real(1) ) );
            if( std::fabs( f - real(1) ) > real(1e-12) ) onSurface = false;

            const Vec3 nb( Vec3( pb[0]/( A*A ), pb[1]/( B*B ), pb[2]/( C*C ) ).getNormalized() );
            const real dot( trans( nb ) * db );
            worstDot = std::min( worstDot, dot );
            if( dot <= real(1) - real(1e-10) ) normalParallel = false;

            // support property against a cloud of surface points
            for( int s=0; s<20; ++s ) {
               const Vec3 q( pos + R * surfacePoint( A, B, C, ANG(rng), real(0.5) * ANG(rng) ) );
               const real excess( trans( d ) * ( q - p ) );
               worstDominance = std::max( worstDominance, excess );
               if( excess > real(1e-12) ) dominates = false;
            }

            const Vec3 pt( ell->supportContactThreshold( d ) );
            const real err( ( pt - ( p + d * contactThreshold ) ).length() );
            worstThreshold = std::max( worstThreshold, err );
            if( err > real(1e-15) ) thresholdExact = false;
         }
      }
      std::printf( "support: max |f-1| = %.3e, min normal dot = %.16f, max dominance excess = %.3e, max threshold err = %.3e\n",
                   worstSurface, worstDot, worstDominance, worstThreshold );
      expect( onSurface,      "support point lies on the ellipsoid surface (1e-12)" );
      expect( normalParallel, "outward normal at the support point is parallel to d (dot > 1-1e-10)" );
      expect( dominates,      "support point maximises d.x over the surface" );
      expect( thresholdExact, "supportContactThreshold(d) == support(d) + d*contactThreshold" );
   }

   // --- 4. bounding box vs brute force --------------------------------------------------------
   {
      bool contained = true, tight = true;
      real worstOutside = 0, worstSlack = 0;
      const int NT = 200, NP = 100;   // 20k samples

      for( int r=0; r<20; ++r ) {
         ell->rotate( randUnit(), U(rng) * real(2) );
         ell->setPosition( Vec3( U(rng), U(rng), U(rng) ) );
         const Rot3& R( ell->getRotation() );
         const Vec3 pos( ell->getPosition() );
         const auto& box( ell->getAABB() );

         real lo[3] = {  real(1e300),  real(1e300),  real(1e300) };
         real hi[3] = { -real(1e300), -real(1e300), -real(1e300) };
         real loT[3], loP[3], hiT[3], hiP[3];

         for( int it=0; it<NT; ++it ) {
            const real theta( real(2) * M_PI * real(it) / real(NT) );
            for( int ip=0; ip<=NP; ++ip ) {
               const real phi( M_PI * real(ip) / real(NP) );
               const Vec3 q( pos + R * surfacePoint( A, B, C, theta, phi ) );
               for( int i=0; i<3; ++i ) {
                  if( q[i] < box[i] - real(1e-12) || q[i] > box[3+i] + real(1e-12) ) contained = false;
                  worstOutside = std::max( worstOutside, std::max( box[i] - q[i], q[i] - box[3+i] ) );
                  if( q[i] < lo[i] ) { lo[i] = q[i]; loT[i] = theta; loP[i] = phi; }
                  if( q[i] > hi[i] ) { hi[i] = q[i]; hiT[i] = theta; hiP[i] = phi; }
               }
            }
         }

         // Local refinement of the sampled extremes (still pure sampling, no support formula)
         for( int i=0; i<3; ++i ) {
            for( int side=0; side<2; ++side ) {
               real bt( side ? hiT[i] : loT[i] ), bp( side ? hiP[i] : loP[i] );
               real best( side ? hi[i] : lo[i] );
               real win( real(2) * M_PI / real(NT) );
               for( int round=0; round<8; ++round ) {
                  real nbt( bt ), nbp( bp );
                  for( int a=-20; a<=20; ++a ) {
                     for( int b=-20; b<=20; ++b ) {
                        const real t( bt + win * real(a) / real(20) );
                        const real ph( bp + win * real(b) / real(20) );
                        const real v( ( pos + R * surfacePoint( A, B, C, t, ph ) )[i] );
                        if( ( side && v > best ) || ( !side && v < best ) ) { best = v; nbt = t; nbp = ph; }
                     }
                  }
                  bt = nbt; bp = nbp; win *= real(0.1);
               }
               if( side ) hi[i] = best; else lo[i] = best;
            }
         }

         for( int i=0; i<3; ++i ) {
            const real slackLo( ( lo[i] - box[i] ) - contactThreshold );      // box face below the surface minimum
            const real slackHi( ( box[3+i] - hi[i] ) - contactThreshold );    // box face above the surface maximum
            worstSlack = std::max( worstSlack, std::max( std::fabs( slackLo ), std::fabs( slackHi ) ) );
            if( std::fabs( slackLo ) > real(1e-6) || std::fabs( slackHi ) > real(1e-6) ) tight = false;
         }
      }
      std::printf( "aabb: max outside = %.3e, max |slack - contactThreshold| = %.3e\n", worstOutside, worstSlack );
      expect( contained, "bounding box contains all sampled surface points" );
      expect( tight,     "bounding box is tight to 1e-6 plus contactThreshold on all faces" );
   }

   // --- 5. radial depth -----------------------------------------------------------------------
   {
      ell->setOrientation( Quat() );
      ell->setPosition( center );
      expect( close( ell->getRelDepth( A, 0, 0 ), real(0), real(1e-12) ), "depth zero at the a-tip" );
      expect( close( ell->getRelDepth( 0, B, 0 ), real(0), real(1e-12) ), "depth zero at the b-tip" );
      expect( close( ell->getRelDepth( 0, 0, C ), real(0), real(1e-12) ), "depth zero at the c-tip" );
      expect( close( ell->getRelDepth( 0, real(0.5)*B, 0 ), real(0.5)*B, real(1e-12) ), "depth along b is B - y" );
      expect( ell->getRelDepth( 0, real(1.5)*B, 0 ) < real(0), "negative depth outside along b" );
      expect( close( ell->getRelDepth( 0, 0, 0 ), C, real(1e-12) ), "depth at the center is the smallest semi-axis" );
      expect( close( ell->getDepth( center + Vec3( 0, 0, C ) ), real(0), real(1e-12) ), "global depth zero at the c-tip" );

      ell->rotate( Vec3( 0, 1, 0 ), real(-M_PI/2.0) );  // body x -> world z
      expect( close( ell->getDepth( center + Vec3( 0, 0, A ) ), real(0), real(1e-12) ), "rotated: global depth zero at the a-tip along world z" );
      expect( close( ell->getDepth( center + Vec3( 0, 0, real(0.5)*A ) ), real(0.5)*A, real(1e-12) ), "rotated: depth A - z along world z" );

      const real rr( real(0.4) );
      EllipsoidID esp = createEllipsoid( 2, Vec3( 10, 0, 0 ), rr, rr, rr, mat );
      const Vec3 q( real(0.1), real(0.2), real(-0.1) );
      expect( close( esp->getRelDepth( q ), rr - q.length(), real(1e-12) ), "degenerate ellipsoid depth equals the sphere formula r - |p|" );
   }

   if( failures == 0 ) {
      std::printf( "pe_ellipsoid_support_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_ellipsoid_support_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
