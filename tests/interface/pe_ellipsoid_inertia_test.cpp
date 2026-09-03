//=================================================================================================
/*!
 *  \file tests/interface/pe_ellipsoid_inertia_test.cpp
 *  \brief Pins the solid-ellipsoid inertia tensor (D6 torque-path review, defect D-1).
 *
 *  EllipsoidBase::calcInertia() carried a 1/4 factor on I_zz where the solid-ellipsoid
 *  value is 1/5 for all three principal axes: I_ii = (1/5) m (r_j^2 + r_k^2). The error
 *  was invisible to every sphere-based validation (Sphere has its own class) and would
 *  bias any rotating-ellipsoid run about the c-axis by 25%. Asserted here:
 *    1. all three principal values of a generic (a,b,c) ellipsoid against the closed form;
 *    2. off-diagonals exactly zero and Iinv consistent (I * Iinv = identity);
 *    3. sphere degeneracy: a = b = c = r reproduces the Sphere class value (2/5) m r^2
 *       on all axes.
 *
 *  Serial world setup, no MPI - mirrors the harness class of pe_lubrication_model_test.
 */
//=================================================================================================

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool close( real x, real y )
{
   const real scale = std::max( real(1), std::max( std::fabs(x), std::fabs(y) ) );
   return std::fabs( x - y ) <= real(1e-12) * scale;
}

int main()
{
   WorldID world = theWorld();
   (void)world;

   const real rho = real(1.25);
   MaterialID mat = createMaterial( "inertia_test", rho, real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );

   // --- 1./2. generic ellipsoid ---------------------------------------------------------------
   const real a = real(2), b = real(3), c = real(4);
   EllipsoidID ell = createEllipsoid( 1, Vec3( 0, 0, 0 ), a, b, c, mat );

   const real m = rho * ( real(4) / real(3) ) * M_PI * a * b * c;
   expect( close( ell->getVolume(), ( real(4) / real(3) ) * M_PI * a * b * c ),
           "ellipsoid volume (4/3)*pi*a*b*c" );
   expect( close( ell->getMass(), m ), "ellipsoid mass rho*(4/3)*pi*a*b*c" );

   const Mat3 I = ell->getInertia();   // identity orientation => body frame
   expect( close( I[0], real(0.2) * m * ( b*b + c*c ) ), "I_xx = (1/5) m (b^2+c^2)" );
   expect( close( I[4], real(0.2) * m * ( a*a + c*c ) ), "I_yy = (1/5) m (a^2+c^2)" );
   expect( close( I[8], real(0.2) * m * ( a*a + b*b ) ), "I_zz = (1/5) m (a^2+b^2)" );
   expect( I[1] == real(0) && I[2] == real(0) && I[3] == real(0) &&
          I[5] == real(0) && I[6] == real(0) && I[7] == real(0),
          "off-diagonal inertia entries are exactly zero" );

   const Mat3 P = I * ell->getInvInertia();
   expect( close( P[0], real(1) ) && close( P[4], real(1) ) && close( P[8], real(1) ) &&
          close( P[1], real(0) ) && close( P[2], real(0) ) && close( P[3], real(0) ) &&
          close( P[5], real(0) ) && close( P[6], real(0) ) && close( P[7], real(0) ),
          "I * Iinv = identity" );

   // --- 3. sphere degeneracy ------------------------------------------------------------------
   const real r = real(1.5);
   EllipsoidID esp = createEllipsoid( 2, Vec3( 20, 0, 0 ), r, r, r, mat );
   SphereID    sph = createSphere   ( 3, Vec3( 40, 0, 0 ), r, mat );

   const Mat3 Ie = esp->getInertia();
   const Mat3 Is = sph->getInertia();
   expect( close( Ie[0], Is[0] ) && close( Ie[4], Is[4] ) && close( Ie[8], Is[8] ),
          "degenerate ellipsoid (r,r,r) matches Sphere inertia (2/5) m r^2 on all axes" );
   expect( close( Ie[0], real(0.4) * esp->getMass() * r * r ),
          "degenerate ellipsoid I_xx = (2/5) m r^2" );

   if( failures == 0 ) {
      std::printf( "pe_ellipsoid_inertia_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_ellipsoid_inertia_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
