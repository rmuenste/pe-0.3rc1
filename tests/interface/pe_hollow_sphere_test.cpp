//=================================================================================================
/*!
 *  \file tests/interface/pe_hollow_sphere_test.cpp
 *  \brief Hollow (shell) spheres: Sphere::setShellThickness(), createHollowSphere(), checkpoints.
 *
 *  A hollow sphere is an ordinary Sphere with a wall thickness: the collision geometry is the
 *  outer sphere, only the mass (wall volume times density) and the inertia change,
 *  I = (2/5) m (R^5 - r^5) / (R^3 - r^3), the thin-shell limit being (2/3) m R^2. Asserted:
 *    1. a 20 mm sphere with a 0.4 mm wall: mass and inertia against the closed forms, Iinv
 *       consistent, I / (m R^2) within 2 % of 2/3 and above the solid 2/5;
 *    2. setShellThickness( 0 ) restores the solid mass and inertia exactly; thickness == radius
 *       is solid; negative or larger than the radius throws;
 *    3. createHollowSphere() equals createSphere() + setShellThickness();
 *    4. a fixed sphere keeps zero inverse mass and inertia through the change;
 *    5. checkpoint round trip: the wall thickness, the mass and the inertia come back.
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/util/Checkpointer.h>
#include <pe/util/CheckpointMetadata.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool close( real x, real y, real tol = real(1e-12) )
{
   const real scale = std::max( real(1), std::max( std::fabs(x), std::fabs(y) ) );
   return std::fabs( x - y ) <= tol * scale;
}

int main()
{
   WorldID world = theWorld();
   const real rho( 1340 ), R( 0.02 ), t( 0.0004 ), r( R - t );
   MaterialID mat = createMaterial( "hs_shell", rho, real(0.9), real(0.1), real(0.1), real(0.25), real(300), real(1e5), real(10), real(10) );

   const real mSolid( rho * real(4) / real(3) * M_PI * R * R * R );
   const real mShell( rho * real(4) / real(3) * M_PI * ( R * R * R - r * r * r ) );
   const real iShell( real(0.4) * mShell * ( std::pow( R, 5 ) - std::pow( r, 5 ) ) / ( R * R * R - r * r * r ) );

   // 1. Shell mass and inertia.
   SphereID s = createSphere( 1, Vec3( 0, 0, 1 ), R, mat );
   expect( close( s->getMass(), mSolid ) && !s->isHollow() && s->getShellThickness() == real(0), "a new sphere is solid" );
   s->setShellThickness( t );
   expect( s->isHollow() && close( s->getShellThickness(), t ), "shell thickness stored" );
   expect( close( s->getMass(), mShell ), "shell mass = rho * 4/3 pi (R^3 - r^3)" );
   expect( close( s->getInvMass(), real(1) / mShell ), "inverse mass updated" );
   const Mat3 I = s->getInertia();
   expect( close( I[0], iShell ) && close( I[4], iShell ) && close( I[8], iShell ), "I = 2/5 m (R^5 - r^5)/(R^3 - r^3)" );
   expect( I[1] == real(0) && I[2] == real(0) && I[3] == real(0) && I[5] == real(0) && I[6] == real(0) && I[7] == real(0), "inertia diagonal" );
   const Mat3 P = I * s->getInvInertia();
   expect( close( P[0], real(1) ) && close( P[4], real(1) ) && close( P[8], real(1) ) && close( P[1], real(0) ), "I * Iinv = identity" );
   const real factor( I[0] / ( s->getMass() * R * R ) );
   std::printf( "shell: mass %.4e kg (solid %.4e), I/(m R^2) = %.4f\n", static_cast<double>( s->getMass() ), static_cast<double>( mSolid ), static_cast<double>( factor ) );
   expect( factor > real(0.4) && std::fabs( factor - real(2) / real(3) ) < real(0.02) * real(2) / real(3), "thin wall: I/(m R^2) within 2% of 2/3" );
   expect( close( s->getRadius(), R ) && close( s->getVolume(), real(4) / real(3) * M_PI * R * R * R ), "radius and outer volume unchanged" );

   // 2. Back to solid, degenerate and invalid values.
   s->setShellThickness( 0 );
   expect( !s->isHollow() && close( s->getMass(), mSolid ) && close( s->getInertia()[0], real(0.4) * mSolid * R * R ) && close( s->getInvMass(), real(1) / mSolid ), "thickness 0 restores the solid sphere" );
   s->setShellThickness( R );
   expect( !s->isHollow() && close( s->getMass(), mSolid ), "thickness == radius is solid" );
   bool threw = false;
   try { s->setShellThickness( -t ); } catch( const std::invalid_argument& ) { threw = true; }
   expect( threw, "negative thickness throws" );
   threw = false;
   try { s->setShellThickness( R * real(1.5) ); } catch( const std::invalid_argument& ) { threw = true; }
   expect( threw, "thickness above the radius throws" );
   expect( close( s->getMass(), mSolid ), "failed calls leave the sphere unchanged" );

   // 3. createHollowSphere.
   SphereID h = createHollowSphere( 2, Vec3( 1, 0, 1 ), R, t, mat );
   expect( h->isHollow() && close( h->getMass(), mShell ) && close( h->getInertia()[0], iShell ), "createHollowSphere = createSphere + setShellThickness" );
   threw = false;
   try { createHollowSphere( 3, Vec3( 2, 0, 1 ), R, -t, mat ); } catch( const std::invalid_argument& ) { threw = true; }
   expect( threw, "createHollowSphere rejects a negative thickness" );

   // 4. Fixed sphere.
   SphereID f = createSphere( 4, Vec3( 3, 0, 1 ), R, mat );
   f->setFixed( true );
   f->setShellThickness( t );
   expect( f->getInvMass() == real(0) && f->getInvInertia()[0] == real(0) && close( f->getMass(), mShell ), "fixed sphere keeps zero inverse mass / inertia" );
   f->setFixed( false );
   expect( close( f->getInvMass(), real(1) / mShell ) && close( f->getInvInertia()[0], real(1) / iShell ), "unfixing uses the shell mass and inertia" );

   // 5. Checkpoint round trip.
   destroy( f );
   const std::string dir = "./_ckpt_hollow_sphere_test", name = "hollow.1";
   setCheckpointIdentity( real(1), uint64_t(1), std::string( "hs" ) );
   writeCheckpoint( dir, name );
   for( World::Iterator it = world->begin(); it != world->end(); ) it = world->destroy( it );
   expect( world->size() == 0, "world empty before read" );
   readCheckpoint( dir, name );
   expect( world->size() == 2, "two spheres restored" );
   int hollow = 0, solid = 0;
   for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
      if( it->getType() != sphereType ) continue;
      SphereID b = static_body_cast<Sphere>( *it );
      if( b->getID() == 2 ) {
         ++hollow;
         expect( b->isHollow() && close( b->getShellThickness(), t ) && close( b->getMass(), mShell ) && close( b->getInertia()[0], iShell ), "restored hollow sphere: thickness, mass, inertia" );
      }
      else if( b->getID() == 1 ) {
         ++solid;
         expect( !b->isHollow() && close( b->getMass(), mSolid ), "restored solid sphere stays solid" );
      }
   }
   expect( hollow == 1 && solid == 1, "both spheres identified after the read" );

   if( failures == 0 ) { std::printf( "pe_hollow_sphere_test: all checks passed\n" ); return EXIT_SUCCESS; }
   std::printf( "pe_hollow_sphere_test: %d check(s) failed\n", failures );
   return EXIT_FAILURE;
}
