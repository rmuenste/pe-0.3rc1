//=================================================================================================
/*!
 *  \file tests/interface/pe_angular_dof_mask_test.cpp
 *  \brief Pins the per-axis angular DOF mask (RigidBody::setAngularDofMask).
 *
 *  The angular mask mirrors the linear one: a component-wise multiplication of the angular
 *  velocity in world axes, applied directly after the angular-velocity update in
 *  RigidBody::applyFluidForces() and in the sphere/ellipsoid move() routines - and nowhere
 *  else. Asserted here for a sphere AND an ellipsoid:
 *    1. mask (0,1,0), w = (1,1,1), nonzero torque: after applyFluidForces w_x = w_z = 0
 *       exactly and w_y equals the unmasked body's w_y bit-for-bit; same after move(dt);
 *    2. mask (1,1,1) is bit-identical to a body that never had the mask set;
 *    3. the linear mask is untouched: v of the angular-masked body equals v of the
 *       unmasked body bit-for-bit.
 *
 *  Serial world setup, no MPI - mirrors the harness class of pe_ellipsoid_inertia_test.
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

// Exact (bit-for-bit) comparison; no tolerance on purpose.
static bool same( const Vec3& a, const Vec3& b )
{
   return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

namespace {

template< typename ID >
struct Triple {
   ID masked;    // angular mask (0,1,0)
   ID plain;     // no mask call at all
   ID identity;  // angular mask (1,1,1)
};

const Vec3 w0( real(1), real(1), real(1) );
const Vec3 v0( real(0.3), real(-0.2), real(0.1) );
const Vec3 torque( real(0.7), real(-1.3), real(2.1) );
const Vec3 force( real(0.5), real(0.25), real(-0.125) );

template< typename ID >
void load( ID b )
{
   b->setLinearVel( v0 );
   b->setAngularVel( w0 );
   b->addForce( force );
   b->addTorque( torque );
}

template< typename ID >
void checkAfter( const Triple<ID>& t, const char* shape, const char* stage )
{
   char msg[256];
   const Vec3& wm = t.masked->getAngularVel();
   const Vec3& wp = t.plain->getAngularVel();
   const Vec3& wi = t.identity->getAngularVel();

   std::snprintf( msg, sizeof(msg), "%s/%s: unmasked angular velocity actually changed", shape, stage );
   expect( !same( wp, w0 ), msg );
   std::snprintf( msg, sizeof(msg), "%s/%s: unmasked w_y is nonzero", shape, stage );
   expect( wp[1] != real(0), msg );

   std::snprintf( msg, sizeof(msg), "%s/%s: masked w_x == 0 exactly", shape, stage );
   expect( wm[0] == real(0), msg );
   std::snprintf( msg, sizeof(msg), "%s/%s: masked w_z == 0 exactly", shape, stage );
   expect( wm[2] == real(0), msg );
   std::snprintf( msg, sizeof(msg), "%s/%s: masked w_y == unmasked w_y bit-for-bit", shape, stage );
   expect( wm[1] == wp[1], msg );

   std::snprintf( msg, sizeof(msg), "%s/%s: mask (1,1,1) bit-identical to no mask (w)", shape, stage );
   expect( same( wi, wp ), msg );

   std::snprintf( msg, sizeof(msg), "%s/%s: angular mask leaves v untouched (masked v == unmasked v)", shape, stage );
   expect( same( t.masked->getLinearVel(), t.plain->getLinearVel() ), msg );
   std::snprintf( msg, sizeof(msg), "%s/%s: unmasked linear velocity actually changed", shape, stage );
   expect( !same( t.plain->getLinearVel(), v0 ), msg );
   std::snprintf( msg, sizeof(msg), "%s/%s: mask (1,1,1) bit-identical to no mask (v)", shape, stage );
   expect( same( t.identity->getLinearVel(), t.plain->getLinearVel() ), msg );

   std::snprintf( msg, sizeof(msg), "%s/%s: linear mask still (1,1,1) on the masked body", shape, stage );
   expect( same( t.masked->getLinearDofMask(), Vec3( 1, 1, 1 ) ), msg );
}

template< typename ID >
void run( const Triple<ID>& t, const char* shape, real dt )
{
   // --- applyFluidForces --------------------------------------------------------------------
   load( t.masked ); load( t.plain ); load( t.identity );
   t.masked  ->applyFluidForces( dt );
   t.plain   ->applyFluidForces( dt );
   t.identity->applyFluidForces( dt );
   checkAfter( t, shape, "applyFluidForces" );

   // --- move --------------------------------------------------------------------------------
   load( t.masked ); load( t.plain ); load( t.identity );
   t.masked  ->move( dt );
   t.plain   ->move( dt );
   t.identity->move( dt );
   checkAfter( t, shape, "move" );
}

} // namespace

int main()
{
   WorldID world = theWorld();
   world->setGravity( real(0), real(0), real(0) );
   world->setDamping( real(1) );

   const real rho = real(1.25);
   MaterialID mat = createMaterial( "angmask_test", rho, real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );
   const real dt = real(0.01);

   // --- default state --------------------------------------------------------------------------
   SphereID probe = createSphere( 100, Vec3( -50, 0, 0 ), real(1), mat );
   expect( same( probe->getAngularDofMask(), Vec3( 1, 1, 1 ) ), "default angular mask is (1,1,1)" );
   expect( same( probe->getLinearDofMask(),  Vec3( 1, 1, 1 ) ), "default linear mask is (1,1,1)" );
   probe->setAngularDofMask( Vec3( 0, 1, 0 ) );
   expect( same( probe->getAngularDofMask(), Vec3( 0, 1, 0 ) ), "getAngularDofMask returns what was set" );
   expect( same( probe->getLinearDofMask(),  Vec3( 1, 1, 1 ) ), "setAngularDofMask leaves the linear mask alone" );

   // --- spheres ---------------------------------------------------------------------------------
   {
      const real r = real(1.5);
      Triple<SphereID> t;
      t.masked   = createSphere( 1, Vec3(  0, 0, 0 ), r, mat );
      t.plain    = createSphere( 2, Vec3( 20, 0, 0 ), r, mat );
      t.identity = createSphere( 3, Vec3( 40, 0, 0 ), r, mat );
      t.masked  ->setAngularDofMask( Vec3( 0, 1, 0 ) );
      t.identity->setAngularDofMask( Vec3( 1, 1, 1 ) );
      run( t, "sphere", dt );
   }

   // --- ellipsoids ------------------------------------------------------------------------------
   {
      const real a = real(2), b = real(3), c = real(4);
      Triple<EllipsoidID> t;
      t.masked   = createEllipsoid( 11, Vec3(  0, 50, 0 ), a, b, c, mat );
      t.plain    = createEllipsoid( 12, Vec3( 20, 50, 0 ), a, b, c, mat );
      t.identity = createEllipsoid( 13, Vec3( 40, 50, 0 ), a, b, c, mat );
      t.masked  ->setAngularDofMask( Vec3( 0, 1, 0 ) );
      t.identity->setAngularDofMask( Vec3( 1, 1, 1 ) );
      run( t, "ellipsoid", dt );
   }

   if( failures == 0 ) {
      std::printf( "pe-angular-dof-mask: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe-angular-dof-mask: %d check(s) failed\n", failures );
   return EXIT_FAILURE;
}
