//=================================================================================================
/*!
 *  \file tests/interface/pe_cylinder_manifold_test.cpp
 *  \brief Multi-point contact manifolds for box-cylinder, cylinder-cylinder and capsule-cylinder.
 *
 *  These pairs go through GJK/EPA, which yields one contact: a cylinder standing on a box, two
 *  stacked cylinders or a capsule lying on a cylinder cap were supported by a single point. The
 *  routines now clip the touching features against each other (MaxContacts::addFeatureManifold).
 *  Asserted with a recording contact container (no solver), penetration delta = 0.01 unless
 *  stated otherwise; the box is the unit cube at the origin (top face z = 0.5):
 *
 *  box-cylinder
 *    1. cylinder (r 0.3) standing on the box top: 4 contacts, dist -delta, on the face, on the
 *       rim (radius r from the axis foot), spread over the cap (largest spacing 2r), normal from
 *       the cylinder to the box; both dispatch orders give the same set (the dispatch always
 *       calls collideBoxCylinder( box, cylinder ));
 *    2. the same cylinder overhanging the face edge (axis at x = 0.45): >= 2 contacts, all
 *       inside the face (x <= 0.5), dist -delta;
 *    3. cylinder lying on the box top: 2 contacts at the ends of the contact line; a lying
 *       cylinder longer than the box gets the line clipped to the face (x = +-0.5);
 *    4. box (edge 0.4) standing on the cap of a wide cylinder (r 1): 4 contacts at the box corners
 *       (on the cap or on the box bottom: for parallel faces either is the reference face);
 *    5. cylinder tilted 30 degrees on its rim: still exactly one contact;
 *    6. small tilts (random, <= 2 degrees) of the standing cylinder: the deepest manifold point
 *       equals the exact depth of the cylinder's support point below the face (1e-9);
 *  cylinder-cylinder
 *    7. coaxial stack: 4 contacts, dist -delta, on the lower cap's plane;
 *    8. parallel lying cylinders, one shifted by 0.3 along the axis: 2 contacts at the ends of
 *       the overlap (x = -0.2 and x = 0.5), dist -delta;
 *    9. crossed lying cylinders: one contact;
 *   10. upright cylinder standing on a lying one: 2 contacts across the cap, dist -delta;
 *  capsule-cylinder
 *   11. capsule lying on a cylinder cap, longer than the cap: 2 contacts, clipped to the cap;
 *   12. capsule parallel to a lying cylinder: 2 contacts at the ends of the overlap.
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/core/detection/fine/MaxContacts.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace pe;
using pe::detection::fine::MaxContacts;

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

// Recording contact container: the minimal interface MaxContacts expects.
struct ContactLog
{
   struct Entry { GeomID g1; GeomID g2; Vec3 pos; Vec3 normal; real dist; };
   std::vector<Entry> entries;

   void addVertexFaceContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist ) {
      entries.push_back( Entry{ g1, g2, gpos, normal, dist } );
   }
   void addLubricationContact( GeomID, GeomID, const Vec3&, const Vec3&, real, real = real(1) ) {}
   void addEdgeEdgeContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                            const Vec3&, const Vec3&, real dist ) {
      entries.push_back( Entry{ g1, g2, gpos, normal, dist } );
   }
   void clear() { entries.clear(); }
};

static ContactLog run( BodyID a, BodyID b )
{
   ContactLog log;
   MaxContacts::collide( a, b, log );
   return log;
}

static bool allDist( const ContactLog& log, real d, real tol )
{
   for( const ContactLog::Entry& e : log.entries )
      if( !close( e.dist, d, tol ) ) return false;
   return !log.entries.empty();
}

static bool allNormal( const ContactLog& log, const Vec3& n, real tol )
{
   for( const ContactLog::Entry& e : log.entries )
      if( ( e.normal - n ).length() > tol ) return false;
   return !log.entries.empty();
}

static real maxSpacing( const ContactLog& log )
{
   real m( 0 );
   for( const ContactLog::Entry& a : log.entries )
      for( const ContactLog::Entry& b : log.entries )
         m = std::max( m, ( a.pos - b.pos ).length() );
   return m;
}

//! The same contact set up to order (the dispatch calls collideBoxCylinder( box, cylinder ) for
//! both argument orders, so geom1 / geom2 and the normal are identical).
static bool sameSet( const ContactLog& a, const ContactLog& b, real tol )
{
   if( a.entries.size() != b.entries.size() ) return false;
   for( const ContactLog::Entry& e : a.entries ) {
      bool found = false;
      for( const ContactLog::Entry& f : b.entries )
         found = found || ( ( e.pos - f.pos ).length() <= tol && close( e.dist, f.dist, tol )
                            && ( e.normal - f.normal ).length() <= tol && e.g1 == f.g1 );
      if( !found ) return false;
   }
   return true;
}

int main()
{
   WorldID world = theWorld();

   MaterialID mat = createMaterial( "cylinder_manifold_test", real(1), real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );

   const real delta = real(0.01);
   const real pi    = real(3.14159265358979323846);
   const Quat upright( real(0), pi / 2, real(0) );   // cylinder axis (body x) onto -z
   const real tol   = real(1e-9);

   // 1. Cylinder standing on the box top.
   {
      world->clear();
      BoxID      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0.1, -0.05, 0.5 + 0.5 - delta ), real(0.3), real(1), mat );
      cy->setOrientation( upright );
      const ContactLog log = run( bx, cy );
      expect( log.entries.size() == 4, "box-cyl standing: four contacts" );
      expect( allDist( log, -delta, tol ), "box-cyl standing: every contact at dist -delta" );
      expect( allNormal( log, Vec3( 0, 0, -1 ), tol ), "box-cyl standing: normal from the cylinder to the box" );
      bool onFaceRim = true;
      for( const ContactLog::Entry& e : log.entries ) {
         onFaceRim &= close( e.pos[2], real(0.5), tol );
         onFaceRim &= close( ( e.pos - Vec3( 0.1, -0.05, 0.5 ) ).length(), real(0.3), tol );
      }
      expect( onFaceRim, "box-cyl standing: contacts on the box face, on the cylinder rim" );
      expect( close( maxSpacing( log ), real(0.6), real(1e-6) ), "box-cyl standing: contacts spread across the cap (2r)" );
      expect( sameSet( log, run( cy, bx ), tol ), "box-cyl standing: both dispatch orders give the same contacts" );
   }

   // 2. Overhanging the face edge.
   {
      world->clear();
      BoxID      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0.45, 0.0, 1.0 - delta ), real(0.3), real(1), mat );
      cy->setOrientation( upright );
      const ContactLog log = run( bx, cy );
      bool inside = true;
      for( const ContactLog::Entry& e : log.entries )
         inside &= ( e.pos[0] <= real(0.5) + tol );
      expect( log.entries.size() >= 2, "box-cyl overhang: at least two contacts" );
      expect( inside, "box-cyl overhang: every contact inside the box face" );
      expect( allDist( log, -delta, tol ), "box-cyl overhang: every contact at dist -delta" );
   }

   // 3. Lying on the box top; short and long.
   {
      world->clear();
      BoxID      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0.0, 0.1, 0.5 + 0.25 - delta ), real(0.25), real(0.8), mat );
      ContactLog log = run( bx, cy );
      bool ends = ( log.entries.size() == 2 );
      for( const ContactLog::Entry& e : log.entries )
         ends &= close( std::fabs( e.pos[0] ), real(0.4), tol ) && close( e.pos[2], real(0.5), tol );
      expect( ends, "box-cyl lying: two contacts at the ends of the contact line (x = +-0.4)" );
      expect( allDist( log, -delta, tol ), "box-cyl lying: dist -delta" );

      world->clear();
      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
      cy = createCylinder( 2, Vec3( 0.0, 0.1, 0.5 + 0.25 - delta ), real(0.25), real(2.0), mat );
      log = run( bx, cy );
      ends = ( log.entries.size() == 2 );
      for( const ContactLog::Entry& e : log.entries )
         ends &= close( std::fabs( e.pos[0] ), real(0.5), tol );
      expect( ends, "box-cyl lying, longer than the box: contact line clipped to the face (x = +-0.5)" );
   }

   // 4. Box standing on a wide cylinder cap.
   {
      world->clear();
      CylinderID cy = createCylinder( 1, Vec3( 0, 0, 0 ), real(1), real(1), mat );
      cy->setOrientation( upright );                                   // cap at z = +0.5
      BoxID bx = createBox( 2, Vec3( 0.1, 0.2, 0.5 + 0.2 - delta ), Vec3( 0.4, 0.4, 0.4 ), mat );
      const ContactLog log = run( bx, cy );
      bool corners = ( log.entries.size() == 4 );
      for( const ContactLog::Entry& e : log.entries )
         corners &= close( std::fabs( e.pos[0] - real(0.1) ), real(0.2), tol )
                 && close( std::fabs( e.pos[1] - real(0.2) ), real(0.2), tol )
                 && ( close( e.pos[2], real(0.5), tol ) || close( e.pos[2], real(0.5) - delta, tol ) );
      // Parallel faces: either may be the reference face, so the points lie on the cap
      // (z = 0.5) or on the box bottom (z = 0.5 - delta); both are flat body surfaces.
      expect( corners, "box on cylinder cap: four contacts at the box corners" );
      expect( allDist( log, -delta, tol ), "box on cylinder cap: dist -delta" );
      expect( allNormal( log, Vec3( 0, 0, 1 ), tol ), "box on cylinder cap: normal from the cylinder to the box" );
   }

   // 5. Tilted 30 degrees, on the rim: single contact.
   {
      world->clear();
      BoxID      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0, 0, 0 ), real(0.3), real(1), mat );
      cy->setOrientation( Quat( real(0), pi / 2 - pi / 6, real(0) ) );
      const real below( -cy->support( Vec3( 0, 0, -1 ) )[2] );       // centre to lowest rim point
      cy->setPosition( Vec3( 0, 0, 0.5 + below - delta ) );
      const ContactLog log = run( bx, cy );
      expect( log.entries.size() == 1, "box-cyl on rim: exactly one contact" );
      expect( allDist( log, -delta, real(1e-8) ), "box-cyl on rim: dist -delta" );
   }

   // 6. Small random tilts: the deepest manifold point is exact.
   {
      std::mt19937 rng( 4711 );
      std::uniform_real_distribution<double> uni( -1.0, 1.0 );
      bool exact = true, finite = true;
      int multi = 0;
      for( int k = 0; k < 100; ++k ) {
         world->clear();
         BoxID      bx = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
         CylinderID cy = createCylinder( 2, Vec3( 0, 0, 0 ), real(0.3), real(1), mat );
         const real tiltX( real(2) * pi / 180 * uni( rng ) ), tiltY( real(2) * pi / 180 * uni( rng ) );
         cy->setOrientation( Quat( tiltX, pi / 2 + tiltY, real(0) ) );
         const real target( -real(0.02) * std::fabs( uni( rng ) ) );
         const real below( -cy->support( Vec3( 0, 0, -1 ) )[2] );
         cy->setPosition( Vec3( 0.1 * uni( rng ), 0.1 * uni( rng ), 0.5 + below + target ) );
         const real exactDist( cy->support( Vec3( 0, 0, -1 ) )[2] - real(0.5) );

         const ContactLog log = run( bx, cy );
         real minD( 1e30 );
         for( const ContactLog::Entry& e : log.entries ) {
            minD = std::min( minD, e.dist );
            finite &= std::isfinite( e.dist ) && std::isfinite( e.pos[0] ) && std::isfinite( e.pos[2] );
         }
         exact &= !log.entries.empty() && close( minD, exactDist, tol );
         if( log.entries.size() >= 2 ) ++multi;
      }
      std::printf( "small-tilt sweep: 100 cases, %d with a multi-point manifold\n", multi );
      expect( exact,  "box-cyl small tilts: deepest manifold point equals the exact support depth (1e-9)" );
      expect( finite, "box-cyl small tilts: all contacts finite" );
   }

   // 7. Coaxial cylinder stack.
   {
      world->clear();
      CylinderID c1 = createCylinder( 1, Vec3( 0, 0, 0 ), real(0.5), real(1), mat );
      CylinderID c2 = createCylinder( 2, Vec3( 0.05, 0, 1 - delta ), real(0.5), real(1), mat );
      c1->setOrientation( upright );
      c2->setOrientation( upright );
      const ContactLog log = run( c1, c2 );
      bool onCap = ( log.entries.size() == 4 );
      for( const ContactLog::Entry& e : log.entries )
         onCap &= close( e.pos[2], real(0.5), tol ) || close( e.pos[2], real(0.5) - delta, tol );
      expect( onCap, "cyl-cyl coaxial: four contacts on a cap plane" );
      expect( allDist( log, -delta, tol ), "cyl-cyl coaxial: dist -delta" );
   }

   // 8. Parallel lying cylinders, shifted by 0.3 along the axis.
   {
      world->clear();
      CylinderID c1 = createCylinder( 1, Vec3( 0, 0, 0 ), real(0.5), real(1), mat );
      CylinderID c2 = createCylinder( 2, Vec3( 0.3, 0, 1 - delta ), real(0.5), real(1), mat );
      const ContactLog log = run( c1, c2 );
      bool ends = ( log.entries.size() == 2 );
      real xs[2] = { 0, 0 };
      for( std::size_t i = 0; i < log.entries.size() && i < 2; ++i ) xs[i] = log.entries[i].pos[0];
      ends &= close( std::min( xs[0], xs[1] ), real(-0.2), tol ) && close( std::max( xs[0], xs[1] ), real(0.5), tol );
      expect( ends, "cyl-cyl parallel: two contacts at the ends of the overlap (x = -0.2, 0.5)" );
      expect( allDist( log, -delta, tol ), "cyl-cyl parallel: dist -delta" );
   }

   // 9. Crossed lying cylinders.
   {
      world->clear();
      CylinderID c1 = createCylinder( 1, Vec3( 0, 0, 0 ), real(0.5), real(1), mat );
      CylinderID c2 = createCylinder( 2, Vec3( 0, 0, 1 - delta ), real(0.5), real(1), mat );
      c2->setOrientation( Quat( real(0), real(0), pi / 2 ) );
      const ContactLog log = run( c1, c2 );
      expect( log.entries.size() == 1, "cyl-cyl crossed: one contact" );
   }

   // 10. Upright cylinder standing on a lying one.
   {
      world->clear();
      CylinderID c1 = createCylinder( 1, Vec3( 0, 0, 0 ), real(0.5), real(2), mat );       // lying, axis x
      CylinderID c2 = createCylinder( 2, Vec3( 0.2, 0, 0.5 + 0.5 - delta ), real(0.4), real(1), mat );
      c2->setOrientation( upright );
      const ContactLog log = run( c1, c2 );
      expect( log.entries.size() == 2, "cyl on lying cyl: two contacts across the cap" );
      expect( allDist( log, -delta, tol ), "cyl on lying cyl: dist -delta" );
      expect( maxSpacing( log ) > real(0.75), "cyl on lying cyl: contacts span (nearly) the cap diameter" );
   }

   // 11. Capsule lying on a cylinder cap, longer than the cap.
   {
      world->clear();
      CapsuleID  ca = createCapsule( 1, Vec3( 0, 0.1, 0.5 + 0.2 - delta ), real(0.2), real(3), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0, 0, 0 ), real(0.5), real(1), mat );
      cy->setOrientation( upright );
      const ContactLog log = run( ca, cy );
      bool clipped = ( log.entries.size() == 2 );
      for( const ContactLog::Entry& e : log.entries )
         clipped &= ( e.pos[0] * e.pos[0] + e.pos[1] * e.pos[1] <= real(0.25) + tol ) && close( e.pos[2], real(0.5), tol );
      expect( clipped, "capsule on cylinder cap: two contacts on the cap, inside the rim" );
      expect( allDist( log, -delta, tol ), "capsule on cylinder cap: dist -delta" );
   }

   // 12. Capsule parallel to a lying cylinder.
   {
      world->clear();
      CapsuleID  ca = createCapsule( 1, Vec3( 0.2, 0, 0.5 + 0.2 - delta ), real(0.2), real(1), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0, 0, 0 ), real(0.5), real(1), mat );
      const ContactLog log = run( ca, cy );
      bool ends = ( log.entries.size() == 2 );
      real xs[2] = { 0, 0 };
      for( std::size_t i = 0; i < log.entries.size() && i < 2; ++i ) xs[i] = log.entries[i].pos[0];
      ends &= close( std::min( xs[0], xs[1] ), real(-0.3), tol ) && close( std::max( xs[0], xs[1] ), real(0.5), tol );
      expect( ends, "capsule parallel to lying cylinder: two contacts at the ends of the overlap" );
      expect( allDist( log, -delta, tol ), "capsule parallel to lying cylinder: dist -delta" );
   }

   if( failures == 0 ) {
      std::printf( "pe_cylinder_manifold_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_cylinder_manifold_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
