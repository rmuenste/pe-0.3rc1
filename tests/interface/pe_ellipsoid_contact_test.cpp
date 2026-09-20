//=================================================================================================
/*!
 *  \file tests/interface/pe_ellipsoid_contact_test.cpp
 *  \brief Narrow-phase contact generation for ellipsoids through MaxContacts::collide().
 *
 *  Before this test MaxContacts::collide() had no ellipsoidType case at all, so ellipsoid pairs
 *  generated no contacts. The dispatch now routes ellipsoid-{ellipsoid, sphere, box, capsule,
 *  cylinder, mesh} through the hybrid GJK/EPA helper on the exact support mapping, and
 *  ellipsoid-plane through the analytic support(-n) formula. Asserted here with a recording
 *  contact container (no solver involved):
 *    1. two axis-aligned prolate spheroids (A=0.5, B=C=0.25) on the x axis at separation
 *       2A - delta: exactly one contact, penetration -delta, normal along x, contact point at
 *       the overlap midpoint; both dispatch orders agree;
 *    2. the same pair rotated by 30 degrees about z with the separation along the rotated axis;
 *    3. a separated pair (2A + 1e-3 and 2A + 1e-6): no contact;
 *    4. ellipsoid-sphere and ellipsoid-box penetration along x;
 *    5. ellipsoid-plane: spheroid tilted 40 degrees above a horizontal plane; penetration equals
 *       the analytic plane height minus lowest surface point, which for a spheroid with axis at
 *       angle theta from the normal is sqrt(A^2 cos^2 theta + B^2 sin^2 theta) below the center;
 *       both dispatch orders agree; a spheroid above the threshold yields no contact;
 *    6. a random sweep of triaxial pairs: contacts are finite, at most one per pair, and pairs
 *       whose bounding spheres do not overlap produce none;
 *    7. reviewer case: an oblique, exactly touching (0.5, 0.25, 0.15) pair (EPA reported a
 *       -7.07e-5 penetration with a wrong normal): depth 0 to 1e-12, geometric normal to 1e-6;
 *    8. reviewer case: the same shapes 5e-9 apart (contactThreshold 1e-8), dropped before by
 *       GJK's unconverged first-separating-plane distance: contact with dist +5e-9 and the
 *       geometric normal; 1.5 * contactThreshold apart: none;
 *    9. reviewer case: ellipsoid (0.2, 0.930, 0.917) penetrating the wall of an inner cylinder
 *       of radius 0.93932 by 1.9485e-5, missed by the unconverged radial search: one wall
 *       contact whose depth matches a 10^6-sample brute-force extent to 1e-10;
 *   10. an oblique random-orientation sweep (100 pairs, independent random rotations, exact
 *       touch along a random direction through the support functions): depth 0 to 1e-10,
 *       geometric normal and touch point to 1e-8;
 *   11. reviewer case: the (0.5, 0.25, 0.15) ellipsoid at the origin against a unit box centred
 *       at (0.99, 0.1, 0) (the contact point was (0.495, -0.2, 0), the midpoint with a box
 *       CORNER picked by the rounding-level tangential components of the normal): dist -0.01,
 *       normal (-1, 0, 0), contact point ON THE BOX FACE, (0.49, 0, 0) to 1e-8; also at y = 0
 *       and y = 0.3;
 *   12. off-centre box-face sweep: 20 lateral (y, z) offsets of the box within the face
 *       (|offset| < 0.4) for each of three small rotations of the ellipsoid: the contact point
 *       lies in the box face plane x = 0.49 (1e-12), equals the projection of the ellipsoid's
 *       deepest point onto that plane (1e-8) and lies inside the face, never on an edge or
 *       corner; and the ellipsoid-plane path (analytic, its point lies on the plane) reports
 *       the same distance, normal and the SAME point (1e-8, no shift);
 *   13. an ellipsoid with equal semi-axes against the box, unrotated and rotated, gives the same
 *       distance, normal and the same contact point as the analytic collideSphereBox() for a
 *       sphere of that radius (1e-10, no shift);
 *   14. strictly convex pairs are untouched by the flat-body placement: ellipsoid-ellipsoid
 *       (head-on, tilted, reviewer's oblique touching pair) and ellipsoid-sphere (head-on,
 *       oblique) contact points match values captured before the change to 1e-12 (tolerance,
 *       not bit equality: CI builds with other compilers/flags) and equal
 *       0.5 * (support_1(-normal) + support_2(normal)) to 1e-13.
 *
 *  Placement convention (pe's, split by geometry): two curved bodies -> overlap midpoint
 *  (collideSphereSphere); a curved body against a flat one -> the point on the flat body's
 *  surface (collideSpherePlane, collideSphereBox, collideEllipsoidPlane). The GJK/EPA path
 *  follows it through MaxContacts::compatibleContactPoint().
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

static bool finite3( const Vec3& v )
{
   return std::isfinite( v[0] ) && std::isfinite( v[1] ) && std::isfinite( v[2] );
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

// Signed component of the contact normal along 'axis' as seen from body 'ref' (normal must
// point from the other body towards 'ref', i.e. from body2 to body1).
static real normalTowards( const ContactLog::Entry& c, BodyID ref, const Vec3& axisFromOtherToRef )
{
   const Vec3 n( c.g1 == ref ? c.normal : -c.normal );
   return trans( n ) * axisFromOtherToRef;
}

int main()
{
   WorldID world = theWorld();
   (void)world;

   MaterialID mat = createMaterial( "contact_test", real(1), real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );

   const real A = real(0.5), B = real(0.25), C = real(0.25);
   const real delta = real(0.01);
   const real tolDepth = real(1e-8);
   ContactLog log;

   // Bodies far apart by default; each case positions what it needs.
   EllipsoidID e1 = createEllipsoid( 1, Vec3( 0, 0, 0 ), A, B, C, mat );
   EllipsoidID e2 = createEllipsoid( 2, Vec3( 100, 0, 0 ), A, B, C, mat );
   SphereID    sp = createSphere   ( 3, Vec3( 200, 0, 0 ), real(0.2), mat );
   BoxID       bx = createBox      ( 4, Vec3( 300, 0, 0 ), Vec3( 1, 1, 1 ), mat );
   PlaneID     pl = createPlane    ( 5, Vec3( 0, 0, 1 ), real(0), mat );

   // --- 1. axis-aligned spheroids ---------------------------------------------------------------
   {
      e1->setPosition( Vec3( 0, 0, 0 ) );
      e2->setPosition( Vec3( real(2)*A - delta, 0, 0 ) );

      log.clear();
      MaxContacts::collide( e1, e2, log );
      expect( log.entries.size() == 1, "axis-aligned pair: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "axis-aligned pair: dist=%.12e (expected %.12e), normal=(%.10f,%.10f,%.10f), point=(%.10f,%.10f,%.10f)\n",
                      c.dist, -delta, c.normal[0], c.normal[1], c.normal[2], c.pos[0], c.pos[1], c.pos[2] );
         expect( close( c.dist, -delta, tolDepth ), "axis-aligned pair: penetration depth = -delta (1e-8)" );
         expect( normalTowards( c, e1, Vec3( -1, 0, 0 ) ) > real(1) - real(1e-8), "axis-aligned pair: normal along x (from e2 to e1)" );
         expect( close( c.pos[0], A - real(0.5)*delta, real(1e-6) ) && close( c.pos[1], 0, real(1e-8) ) && close( c.pos[2], 0, real(1e-8) ),
                 "axis-aligned pair: contact point at the overlap midpoint" );
      }

      log.clear();
      MaxContacts::collide( e2, e1, log );   // reversed dispatch order
      expect( log.entries.size() == 1, "axis-aligned pair (reversed order): exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         expect( close( c.dist, -delta, tolDepth ), "axis-aligned pair (reversed order): penetration depth = -delta" );
         expect( normalTowards( c, e1, Vec3( -1, 0, 0 ) ) > real(1) - real(1e-8), "axis-aligned pair (reversed order): normal along x" );
      }

      // Touching within the threshold band: contact with |dist| <= contactThreshold
      e2->setPosition( Vec3( real(2)*A, 0, 0 ) );
      log.clear();
      MaxContacts::collide( e1, e2, log );
      expect( log.entries.size() == 1, "touching pair: one contact" );
      if( log.entries.size() == 1 )
         expect( std::fabs( log.entries[0].dist ) <= real(2) * contactThreshold + real(1e-12), "touching pair: |dist| within the threshold" );

      // Shallow head-on overlaps: EPA cannot start from the sliver simplex GJK returns here, the
      // support-descent fallback of gjkEPAcollideHybrid must deliver the exact depth (mirrors
      // the analytic sphere-sphere behaviour row for row).
      {
         const real offsets[] = { real(-1e-4), real(-1e-5), real(-1e-6), real(-1e-7), real(-1e-8), real(0), real(5e-9), real(9e-9) };
         bool allFound = true, allExact = true, allAlongX = true;
         for( real o : offsets ) {
            e2->setPosition( Vec3( real(2)*A + o, 0, 0 ) );
            log.clear();
            MaxContacts::collide( e1, e2, log );
            if( log.entries.size() != 1 ) { allFound = false; std::printf( "  head-on offset %+.1e: %zu contacts\n", o, log.entries.size() ); continue; }
            const ContactLog::Entry& c( log.entries[0] );
            if( !close( c.dist, o, real(1e-10) ) ) { allExact = false; std::printf( "  head-on offset %+.1e: dist %+.3e\n", o, c.dist ); }
            if( normalTowards( c, e1, Vec3( -1, 0, 0 ) ) <= real(1) - real(1e-8) ) allAlongX = false;
         }
         expect( allFound,   "shallow head-on overlaps down to exact touch: one contact each" );
         expect( allExact,   "shallow head-on overlaps: dist equals the offset (1e-10)" );
         expect( allAlongX,  "shallow head-on overlaps: normal along x" );

         e2->setPosition( Vec3( real(2)*A + real(1.1) * contactThreshold, 0, 0 ) );
         log.clear();
         MaxContacts::collide( e1, e2, log );
         expect( log.entries.empty(), "head-on gap of 1.1*contactThreshold: no contact" );
      }
   }

   // --- 2. tilted spheroids ---------------------------------------------------------------------
   {
      const real ang( real(30) * M_PI / real(180) );
      const Vec3 axis( std::cos( ang ), std::sin( ang ), 0 );
      e1->setOrientation( Quat() );
      e2->setOrientation( Quat() );
      e1->rotate( Vec3( 0, 0, 1 ), ang );
      e2->rotate( Vec3( 0, 0, 1 ), ang );
      e1->setPosition( Vec3( 0, 0, 0 ) );
      e2->setPosition( axis * ( real(2)*A - delta ) );

      log.clear();
      MaxContacts::collide( e1, e2, log );
      expect( log.entries.size() == 1, "tilted pair: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "tilted pair: dist=%.12e (expected %.12e), normal.axis=%.12f\n",
                      c.dist, -delta, normalTowards( c, e1, -axis ) );
         expect( close( c.dist, -delta, tolDepth ), "tilted pair: penetration depth = -delta (1e-8)" );
         expect( normalTowards( c, e1, -axis ) > real(1) - real(1e-8), "tilted pair: normal along the rotated axis" );
         const Vec3 expectedPoint( axis * ( A - real(0.5)*delta ) );
         expect( ( c.pos - expectedPoint ).length() < real(1e-6), "tilted pair: contact point at the overlap midpoint" );
      }
      e1->setOrientation( Quat() );
      e2->setOrientation( Quat() );
   }

   // --- 3. separated pairs ----------------------------------------------------------------------
   {
      e1->setPosition( Vec3( 0, 0, 0 ) );
      e2->setPosition( Vec3( real(2)*A + real(1e-3), 0, 0 ) );
      log.clear();
      MaxContacts::collide( e1, e2, log );
      expect( log.entries.empty(), "separated pair (gap 1e-3): no contact" );

      e2->setPosition( Vec3( real(2)*A + real(1e-6), 0, 0 ) );
      log.clear();
      MaxContacts::collide( e1, e2, log );
      expect( log.entries.empty(), "separated pair (gap 1e-6 > contactThreshold): no contact" );

      e2->setPosition( Vec3( 100, 0, 0 ) );
   }

   // --- 4. ellipsoid-sphere and ellipsoid-box ---------------------------------------------------
   {
      e1->setPosition( Vec3( 0, 0, 0 ) );
      sp->setPosition( Vec3( A + sp->getRadius() - delta, 0, 0 ) );
      log.clear();
      MaxContacts::collide( e1, sp, log );
      expect( log.entries.size() == 1, "ellipsoid-sphere: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "ellipsoid-sphere: dist=%.12e (expected %.12e)\n", c.dist, -delta );
         expect( close( c.dist, -delta, tolDepth ), "ellipsoid-sphere: penetration depth = -delta (1e-8)" );
         expect( normalTowards( c, e1, Vec3( -1, 0, 0 ) ) > real(1) - real(1e-8), "ellipsoid-sphere: normal along x" );
      }
      log.clear();
      MaxContacts::collide( sp, e1, log );   // sphere first
      expect( log.entries.size() == 1, "sphere-ellipsoid (reversed order): exactly one contact" );
      if( log.entries.size() == 1 )
         expect( close( log.entries[0].dist, -delta, tolDepth ), "sphere-ellipsoid (reversed order): penetration depth = -delta" );
      sp->setPosition( Vec3( 200, 0, 0 ) );

      bx->setPosition( Vec3( A + real(0.5) - delta, 0, 0 ) );
      log.clear();
      MaxContacts::collide( e1, bx, log );
      expect( log.entries.size() == 1, "ellipsoid-box: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "ellipsoid-box: dist=%.12e (expected %.12e)\n", c.dist, -delta );
         expect( close( c.dist, -delta, tolDepth ), "ellipsoid-box: penetration depth = -delta (1e-8)" );
         expect( normalTowards( c, e1, Vec3( -1, 0, 0 ) ) > real(1) - real(1e-8), "ellipsoid-box: normal along x" );
      }
      log.clear();
      MaxContacts::collide( bx, e1, log );
      expect( log.entries.size() == 1, "box-ellipsoid (reversed order): exactly one contact" );
      bx->setPosition( Vec3( 300, 0, 0 ) );
   }

   // --- 5. ellipsoid-plane ----------------------------------------------------------------------
   {
      const real tilt( real(40) * M_PI / real(180) );   // above the horizontal
      e1->setOrientation( Quat() );
      e1->rotate( Vec3( 0, 1, 0 ), -tilt );             // body x tilted towards +z
      const Rot3& R( e1->getRotation() );
      const Vec3 u( R[0], R[3], R[6] );                  // world direction of the a-axis
      const real theta( std::acos( std::fabs( u[2] ) ) );  // angle between axis and plane normal
      expect( close( theta, M_PI/real(2) - tilt, real(1e-12) ), "ellipsoid-plane: axis is 50 degrees from the normal" );
      const real h( std::sqrt( A*A * std::cos( theta )*std::cos( theta ) + B*B * std::sin( theta )*std::sin( theta ) ) );

      e1->setPosition( Vec3( 0, 0, h - delta ) );
      log.clear();
      MaxContacts::collide( e1, pl, log );
      expect( log.entries.size() == 1, "ellipsoid-plane: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "ellipsoid-plane: dist=%.12e (expected %.12e), h=%.12f, point z=%.3e\n", c.dist, -delta, h, c.pos[2] );
         expect( close( c.dist, -delta, real(1e-12) ), "ellipsoid-plane: penetration = plane height minus lowest surface point" );
         expect( c.g1 == e1 && c.g2 == pl, "ellipsoid-plane: ellipsoid is body 1" );
         expect( close( c.normal[2], real(1), real(1e-15) ), "ellipsoid-plane: normal is the plane normal" );
         expect( close( c.pos[2], real(0), real(1e-12) ), "ellipsoid-plane: contact point on the plane surface" );
         // the deepest point is the support in -n; its horizontal position is the contact point
         const Vec3 deepest( e1->support( Vec3( 0, 0, -1 ) ) );
         expect( close( deepest[2], -delta, real(1e-12) ), "ellipsoid-plane: support(-n) is delta below the plane" );
         expect( close( c.pos[0], deepest[0], real(1e-12) ) && close( c.pos[1], deepest[1], real(1e-12) ), "ellipsoid-plane: contact point below the deepest point" );
      }
      log.clear();
      MaxContacts::collide( pl, e1, log );
      expect( log.entries.size() == 1 && log.entries[0].g1 == e1 && close( log.entries[0].dist, -delta, real(1e-12) ),
              "plane-ellipsoid (reversed order): same contact" );

      e1->setPosition( Vec3( 0, 0, h + real(1e-6) ) );
      log.clear();
      MaxContacts::collide( e1, pl, log );
      expect( log.entries.empty(), "ellipsoid-plane: no contact when 1e-6 above the plane" );

      e1->setPosition( Vec3( 0, 0, h + real(0.5) * contactThreshold ) );
      log.clear();
      MaxContacts::collide( e1, pl, log );
      expect( log.entries.size() == 1 && log.entries[0].dist > real(0), "ellipsoid-plane: positive-distance contact inside the threshold band" );

      e1->setOrientation( Quat() );
      e1->setPosition( Vec3( 0, 0, 50 ) );
   }

   // --- 6. random sweep -------------------------------------------------------------------------
   {
      std::mt19937 rng( 4711u );
      std::uniform_real_distribution<real> U( real(-1), real(1) );
      std::uniform_real_distribution<real> S( real(0.1), real(0.6) );
      auto randUnit = [&]() {
         Vec3 v;
         do { v = Vec3( U(rng), U(rng), U(rng) ); } while( v.sqrLength() < real(1e-3) || v.sqrLength() > real(1) );
         return v.getNormalized();
      };

      bool allFinite = true, atMostOne = true, noneWhenApart = true, negativeWhenOverlappingCenters = true;
      int contacts = 0, pairs = 0;
      for( int k=0; k<300; ++k ) {
         const real a1( S(rng) ), b1( S(rng) ), c1( S(rng) );
         const real a2( S(rng) ), b2( S(rng) ), c2( S(rng) );
         EllipsoidID x1 = createEllipsoid( 100 + 2*k,     Vec3( 0, 0, 0 ), a1, b1, c1, mat );
         EllipsoidID x2 = createEllipsoid( 100 + 2*k + 1, Vec3( 0, 0, 0 ), a2, b2, c2, mat );
         x1->rotate( randUnit(), U(rng) * real(3) );
         x2->rotate( randUnit(), U(rng) * real(3) );
         const real rmax1( std::max( a1, std::max( b1, c1 ) ) ), rmax2( std::max( a2, std::max( b2, c2 ) ) );
         const real rmin1( std::min( a1, std::min( b1, c1 ) ) ), rmin2( std::min( a2, std::min( b2, c2 ) ) );
         const real sep( ( real(0.2) + real(0.7) * ( U(rng) + real(1) ) ) * ( rmax1 + rmax2 ) );  // 0.2 .. 1.6 x
         x2->setPosition( randUnit() * sep );
         ++pairs;

         log.clear();
         MaxContacts::collide( x1, x2, log );
         if( log.entries.size() > 1 ) atMostOne = false;
         for( const ContactLog::Entry& c : log.entries ) {
            ++contacts;
            if( !finite3( c.pos ) || !finite3( c.normal ) || !std::isfinite( c.dist ) ) allFinite = false;
            if( c.dist > contactThreshold ) allFinite = false;
         }
         if( sep > rmax1 + rmax2 + real(1e-6) && !log.entries.empty() ) noneWhenApart = false;
         if( sep < rmin1 + rmin2 && ( log.entries.size() != 1 || log.entries[0].dist >= real(0) ) ) negativeWhenOverlappingCenters = false;

         destroy( x1 );
         destroy( x2 );
      }
      std::printf( "random sweep: %d pairs, %d contacts\n", pairs, contacts );
      expect( allFinite, "random sweep: all contact data finite and dist <= contactThreshold" );
      expect( atMostOne, "random sweep: at most one contact per pair" );
      expect( noneWhenApart, "random sweep: no contact when the bounding spheres are apart" );
      expect( negativeWhenOverlappingCenters, "random sweep: penetrating contact when the inscribed spheres overlap" );
   }

   // --- 7. reviewer: oblique exactly touching pair (EPA false penetration) ----------------------
   // Two triaxial (0.5, 0.25, 0.15) ellipsoids, b = a shifted by 2 support(n): they touch at
   // support(n) with the geometric normal -n (from b to a). EPA used to report a penetration of
   // -7.07e-5 along (-0.194, 0.282, 0.940); accepted unchecked that gave spurious impulses.
   {
      EllipsoidID a = createEllipsoid( 1001, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      EllipsoidID b = createEllipsoid( 1002, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      const int k( 23 );
      Vec3 n( real(1), real(0.13) + k * real(0.013), real(0.07) + k * real(0.007) );
      n.normalize();
      b->setPosition( real(2) * a->support( n ) );

      log.clear();
      MaxContacts::collide( a, b, log );
      expect( log.entries.size() == 1, "reviewer touching pair: exactly one contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         const Vec3 nn( c.g1 == a ? c.normal : -c.normal );
         std::printf( "reviewer touching pair: dist=%.3e, normal error=%.3e\n", c.dist, ( nn + n ).length() );
         expect( close( c.dist, real(0), real(1e-12) ), "reviewer touching pair: depth 0 (1e-12)" );
         expect( ( nn + n ).length() <= real(1e-6), "reviewer touching pair: geometric normal (1e-6)" );
         expect( ( c.pos - a->support( n ) ).length() <= real(1e-6), "reviewer touching pair: contact point at the touch point" );
      }
      log.clear();
      MaxContacts::collide( b, a, log );
      expect( log.entries.size() == 1 && close( log.entries[0].dist, real(0), real(1e-12) ), "reviewer touching pair (reversed order): same depth" );

      // --- 8. reviewer: sub-threshold gap rejected by the unconverged GJK distance -----------
      // Gap 5e-9 (threshold 1e-8): GJK's first separating plane reported 1.85e-7 and the
      // contact was dropped; the threshold-grown test is exact and must keep it.
      {
         const int k2( 2 );
         const real gap( real(5e-9) );
         Vec3 n2( real(1), real(0.13) + k2 * real(0.013), real(0.07) + k2 * real(0.007) );
         n2.normalize();
         b->setPosition( real(2) * a->support( n2 ) + gap * n2 );

         log.clear();
         MaxContacts::collide( a, b, log );
         expect( log.entries.size() == 1, "reviewer 5e-9 gap: exactly one contact" );
         if( log.entries.size() == 1 ) {
            const ContactLog::Entry& c( log.entries[0] );
            const Vec3 nn( c.g1 == a ? c.normal : -c.normal );
            std::printf( "reviewer 5e-9 gap: dist=%.12e, normal error=%.3e\n", c.dist, ( nn + n2 ).length() );
            expect( close( c.dist, gap, real(1e-12) ), "reviewer 5e-9 gap: dist = +5e-9 (positive = separation)" );
            expect( ( nn + n2 ).length() <= real(1e-6), "reviewer 5e-9 gap: geometric normal (1e-6)" );
         }

         // and the same direction just outside the band: no contact
         b->setPosition( real(2) * a->support( n2 ) + real(1.5) * contactThreshold * n2 );
         log.clear();
         MaxContacts::collide( a, b, log );
         expect( log.entries.empty(), "reviewer direction, gap 1.5*contactThreshold: no contact" );
      }
      destroy( a );
      destroy( b );
   }

   // --- 9. reviewer: inner-cylinder wall penetration missed by the unconverged search ---------
   // Ellipsoid (0.2, 0.930, 0.917) tilted about x inside a cylinder of radius 0.93932 (axis x):
   // the surface reaches 0.9393395 from the axis, a penetration of 1.9485e-5 that the 64-step
   // fixed-point iteration missed. Reference: brute-force radial extent over 10^6 azimuths.
   {
      InnerCylinderID cyl = createInnerCylinder( 1003, Vec3( 0, 0, 0 ), real(0.93932), real(10), mat );
      EllipsoidID e = createEllipsoid( 1004, Vec3( 0, real(-0.010616516792026215), real(-0.0020341444415746396) ),
                                       real(0.2), real(0.9300185014917651), real(0.9168575207622052), mat );
      e->rotate( Vec3( 1, 0, 0 ), real(-0.43934643468388357) );

      real rmax( 0 );
      real tbest( 0 );
      const int M( 1000000 );
      for( int i=0; i<M; ++i ) {
         const real t( real(2) * M_PI * real(i) / real(M) );
         const Vec3 p( e->support( Vec3( 0, std::cos( t ), std::sin( t ) ) ) );
         const real r( std::sqrt( p[1]*p[1] + p[2]*p[2] ) );
         if( r > rmax ) { rmax = r; tbest = t; }
      }
      const real expectedDist( real(0.93932) - rmax );

      log.clear();
      MaxContacts::collide( e, cyl, log );
      expect( log.entries.size() == 1, "reviewer inner cylinder: exactly one (wall) contact" );
      if( log.entries.size() == 1 ) {
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "reviewer inner cylinder: dist=%.12e (brute force %.12e), normal=(%.6f,%.6f,%.6f)\n",
                      c.dist, expectedDist, c.normal[0], c.normal[1], c.normal[2] );
         expect( c.g1 == e && c.g2 == cyl, "reviewer inner cylinder: ellipsoid is body 1" );
         expect( close( c.dist, real(-1.9485495e-5), real(1e-8) ), "reviewer inner cylinder: depth = -1.9485e-5 (1e-8)" );
         expect( close( c.dist, expectedDist, real(1e-10) ), "reviewer inner cylinder: depth matches the 10^6-sample brute force (1e-10)" );
         const Vec3 nExp( 0, -std::cos( tbest ), -std::sin( tbest ) );   // inward radial at the extremal azimuth
         expect( ( c.normal - nExp ).length() <= real(1e-4), "reviewer inner cylinder: inward normal at the extremal azimuth" );
         expect( close( std::sqrt( c.pos[1]*c.pos[1] + c.pos[2]*c.pos[2] ), rmax + real(0.5) * c.dist, real(1e-9) ),
                 "reviewer inner cylinder: contact point midway between surface and wall" );
      }
      log.clear();
      MaxContacts::collide( cyl, e, log );
      expect( log.entries.size() == 1 && log.entries[0].g1 == e, "reviewer inner cylinder (reversed order): same contact" );

      // Same ellipsoid with clearance: a cylinder 2*contactThreshold wider than the extent
      InnerCylinderID cyl2 = createInnerCylinder( 1005, Vec3( 0, 0, 0 ), rmax + real(2) * contactThreshold, real(10), mat );
      log.clear();
      MaxContacts::collide( e, cyl2, log );
      expect( log.entries.empty(), "reviewer inner cylinder widened by 2*contactThreshold: no contact" );

      destroy( cyl2 );
      destroy( e );
      destroy( cyl );
   }

   // --- 10. oblique random-orientation sweep at exact touch -----------------------------------
   // Pairs with independent random orientations, placed at exact touch along a random
   // direction through the support functions: b = a.support(n) - b0.support(-n) with b0 the
   // second body at the origin, so the touch point is a.support(n) and the normal (b to a) is
   // -n. Half of the sweep uses the reviewer's shape for both bodies, half random triaxial
   // shapes. Head-on-only coverage is gone with this.
   {
      std::mt19937 rng( 20260919u );
      std::uniform_real_distribution<real> U( real(-1), real(1) );
      std::uniform_real_distribution<real> S( real(0.1), real(0.8) );
      auto randUnit = [&]() {
         Vec3 v;
         do { v = Vec3( U(rng), U(rng), U(rng) ); } while( v.sqrLength() < real(1e-3) || v.sqrLength() > real(1) );
         return v.getNormalized();
      };

      int found( 0 );
      real worstDepth( 0 ), worstNormal( 0 ), worstPoint( 0 );
      const int sweep( 100 );
      for( int k=0; k<sweep; ++k ) {
         real a1( real(0.5) ), b1( real(0.25) ), c1( real(0.15) ), a2( a1 ), b2( b1 ), c2( c1 );
         if( k >= sweep/2 ) { a1 = S(rng); b1 = S(rng); c1 = S(rng); a2 = S(rng); b2 = S(rng); c2 = S(rng); }
         EllipsoidID x1 = createEllipsoid( 2000 + 2*k,     Vec3( 0, 0, 0 ), a1, b1, c1, mat );
         EllipsoidID x2 = createEllipsoid( 2000 + 2*k + 1, Vec3( 0, 0, 0 ), a2, b2, c2, mat );
         x1->rotate( randUnit(), U(rng) * real(3) );
         x2->rotate( randUnit(), U(rng) * real(3) );
         const Vec3 n( randUnit() );
         const Vec3 touch( x1->support( n ) );
         x2->setPosition( touch - x2->support( -n ) );

         log.clear();
         MaxContacts::collide( x1, x2, log );
         if( log.entries.size() == 1 ) {
            ++found;
            const ContactLog::Entry& c( log.entries[0] );
            const Vec3 nn( c.g1 == x1 ? c.normal : -c.normal );
            worstDepth  = std::max( worstDepth,  std::fabs( c.dist ) );
            worstNormal = std::max( worstNormal, ( nn + n ).length() );
            worstPoint  = std::max( worstPoint,  ( c.pos - touch ).length() );
         }
         destroy( x1 );
         destroy( x2 );
      }
      std::printf( "oblique touch sweep: %d/%d contacts, worst |depth|=%.3e, worst normal error=%.3e, worst point error=%.3e\n",
                   found, sweep, worstDepth, worstNormal, worstPoint );
      expect( found == sweep, "oblique touch sweep: one contact for every pair" );
      expect( worstDepth <= real(1e-10), "oblique touch sweep: depth 0 (1e-10)" );
      expect( worstNormal <= real(1e-8), "oblique touch sweep: geometric normal (1e-8)" );
      expect( worstPoint <= real(1e-8), "oblique touch sweep: contact point at the touch point (1e-8)" );
   }

   // --- 11. reviewer: off-centre box-face contact ---------------------------------------------
   // The support point of a box face is not unique; the midpoint of two independently selected
   // support points inherited half the offset of the corner picked by the 1e-18 tangential
   // components of the normal. The contact point must lie on the common contact patch, and by
   // pe's curved-vs-flat convention on the flat body's surface: the box face x = 0.49.
   {
      EllipsoidID e = createEllipsoid( 3000, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      BoxID       b = createBox( 3001, Vec3( real(0.99), real(0.1), 0 ), Vec3( 1, 1, 1 ), mat );
      const real shifts[] = { real(0.1), real(0), real(0.3) };
      for( real shift : shifts ) {
         b->setPosition( Vec3( real(0.99), shift, 0 ) );
         for( int order=0; order<2; ++order ) {
            log.clear();
            if( order == 0 ) MaxContacts::collide( e, b, log );
            else             MaxContacts::collide( b, e, log );
            char what[160];
            std::snprintf( what, sizeof what, "reviewer box face (box y=%.1f, %s): exactly one contact", shift, order == 0 ? "e,b" : "b,e" );
            expect( log.entries.size() == 1, what );
            if( log.entries.size() != 1 ) continue;
            const ContactLog::Entry& c( log.entries[0] );
            const Vec3 nn( c.g1 == e ? c.normal : -c.normal );   // from the box to the ellipsoid
            if( order == 0 )
               std::printf( "reviewer box face (box y=%.1f): dist=%.15g normal=(%.3g,%.3g,%.3g) point=(%.15g,%.15g,%.15g)\n",
                            shift, c.dist, nn[0], nn[1], nn[2], c.pos[0], c.pos[1], c.pos[2] );
            std::snprintf( what, sizeof what, "reviewer box face (box y=%.1f, %s): dist = -0.01 (1e-12)", shift, order == 0 ? "e,b" : "b,e" );
            expect( close( c.dist, real(-0.01), real(1e-12) ), what );
            std::snprintf( what, sizeof what, "reviewer box face (box y=%.1f, %s): normal (-1,0,0) (1e-12)", shift, order == 0 ? "e,b" : "b,e" );
            expect( ( nn - Vec3( -1, 0, 0 ) ).length() <= real(1e-12), what );
            std::snprintf( what, sizeof what, "reviewer box face (box y=%.1f, %s): contact point on the box face, (0.49,0,0) (1e-8)", shift, order == 0 ? "e,b" : "b,e" );
            expect( ( c.pos - Vec3( real(0.49), 0, 0 ) ).length() <= real(1e-8), what );
         }
      }
      destroy( e );
      destroy( b );
   }

   // --- 12. off-centre box-face sweep with the ellipsoid-plane cross-check ---------------------
   // The ellipsoid is placed so that its +x extreme is at x = 0.5 for each rotation; the unit
   // box at x = 0.99 (face at x = 0.49, penetration 0.01) is offset laterally within the face.
   // Expected contact point: on the box face, the projection of the ellipsoid's deepest point
   // onto the face plane (curved-vs-flat convention). The plane x = 0.49 (normal -x) is the
   // analytic reference for the same geometry and must report the same point.
   {
      EllipsoidID e    = createEllipsoid( 3010, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      BoxID       b    = createBox( 3011, Vec3( real(0.99), 0, 0 ), Vec3( 1, 1, 1 ), mat );
      PlaneID     face = createPlane( 3012, Vec3( -1, 0, 0 ), real(-0.49), mat );   // { x : -x = -0.49 }

      std::mt19937 rng( 20260920u );
      std::uniform_real_distribution<real> U( real(-0.4), real(0.4) );
      const Vec3 axes[3]   = { Vec3( 0, 0, 1 ), Vec3( 0, 1, 0 ), Vec3( 1, 1, 0 ).getNormalized() };
      const real angles[3] = { real(0.02), real(-0.05), real(0.04) };
      const Vec3 ex( 1, 0, 0 );

      int found( 0 ), planeFound( 0 ), configs( 0 );
      real worstDepth( 0 ), worstNormal( 0 ), worstPoint( 0 ), worstFacePlane( 0 ), worstPlaneDepth( 0 ), worstPlaneNormal( 0 ), worstPlanePoint( 0 );
      bool insideFace( true );
      for( int r=0; r<3; ++r ) {
         e->setOrientation( Quat() );
         e->setPosition( Vec3( 0, 0, 0 ) );
         e->rotate( axes[r], angles[r] );
         const Vec3 sx( e->support( ex ) );                 // +x extreme with the centre at the origin
         e->setPosition( Vec3( real(0.5) - sx[0], 0, 0 ) );
         const Vec3 deepest( e->support( ex ) );            // now at x = 0.5

         for( int k=0; k<20; ++k ) {
            const real oy( U(rng) ), oz( U(rng) );
            b->setPosition( Vec3( real(0.99), oy, oz ) );
            ++configs;

            log.clear();
            MaxContacts::collide( e, b, log );
            if( log.entries.size() == 1 ) {
               ++found;
               const ContactLog::Entry c( log.entries[0] );   // copy: the log is reused below
               const Vec3 nn( c.g1 == e ? c.normal : -c.normal );
               const Vec3 expectedPoint( real(0.49), deepest[1], deepest[2] );   // deepest point projected onto the face plane
               worstDepth     = std::max( worstDepth,     std::fabs( c.dist + real(0.01) ) );
               worstNormal    = std::max( worstNormal,    ( nn - Vec3( -1, 0, 0 ) ).length() );
               worstFacePlane = std::max( worstFacePlane, std::fabs( c.pos[0] - real(0.49) ) );
               worstPoint     = std::max( worstPoint,     ( c.pos - expectedPoint ).length() );
               if( std::fabs( c.pos[1] - oy ) > real(0.45) || std::fabs( c.pos[2] - oz ) > real(0.45) )
                  insideFace = false;

               // Plane reference for the same geometry: same point, no shift
               log.clear();
               MaxContacts::collide( e, face, log );
               if( log.entries.size() == 1 ) {
                  ++planeFound;
                  const ContactLog::Entry& p( log.entries[0] );
                  const Vec3 np( p.g1 == e ? p.normal : -p.normal );
                  worstPlaneDepth  = std::max( worstPlaneDepth,  std::fabs( p.dist - c.dist ) );
                  worstPlaneNormal = std::max( worstPlaneNormal, ( np - nn ).length() );
                  worstPlanePoint  = std::max( worstPlanePoint,  ( p.pos - c.pos ).length() );
               }
            }
            else {
               std::printf( "  box-face sweep rotation %d offset (%.3f,%.3f): %zu contacts\n", r, oy, oz, log.entries.size() );
            }
         }
      }
      std::printf( "box-face sweep: %d/%d contacts, worst |dist+0.01|=%.3e, worst normal error=%.3e, worst |x-0.49|=%.3e, worst point error=%.3e; "
                   "plane cross-check %d/%d: dist %.3e, normal %.3e, point %.3e\n",
                   found, configs, worstDepth, worstNormal, worstFacePlane, worstPoint, planeFound, configs, worstPlaneDepth, worstPlaneNormal, worstPlanePoint );
      expect( found == configs,                "box-face sweep: one contact for every configuration" );
      expect( worstDepth     <= real(1e-10),   "box-face sweep: dist = -0.01 (1e-10)" );
      expect( worstNormal    <= real(1e-10),   "box-face sweep: normal (-1,0,0) (1e-10)" );
      expect( worstFacePlane <= real(1e-12),   "box-face sweep: contact point lies in the box face plane x = 0.49 (1e-12)" );
      expect( worstPoint     <= real(1e-8),    "box-face sweep: contact point = projection of the ellipsoid's deepest point onto the face (1e-8)" );
      expect( insideFace,                      "box-face sweep: contact point inside the face, never on an edge or corner" );
      expect( planeFound == configs,           "box-face sweep: plane reference contact for every configuration" );
      expect( worstPlaneDepth  <= real(1e-10), "box-face sweep: plane dist equals the box dist (1e-10)" );
      expect( worstPlaneNormal <= real(1e-10), "box-face sweep: plane normal equals the box normal (1e-10)" );
      expect( worstPlanePoint  <= real(1e-8),  "box-face sweep: plane point equals the box point directly (1e-8, no shift)" );

      destroy( e );
      destroy( b );
      destroy( face );
   }

   // --- 13. equal semi-axes against the box vs the analytic sphere-box ------------------------
   // Unrotated and rotated boxes whose face nearest to the origin is at distance 0.19 with the
   // face centre offset laterally: the ellipsoid-box path must agree with collideSphereBox()
   // for a sphere of radius 0.2 in distance, normal and contact point (both on the box surface).
   {
      const real r( real(0.2) );
      EllipsoidID ball = createEllipsoid( 3020, Vec3( 0, 0, 0 ), r, r, r, mat );
      SphereID    ref  = createSphere( 3021, Vec3( 0, 0, 0 ), r, mat );
      BoxID       b    = createBox( 3022, Vec3( 10, 0, 0 ), Vec3( 1, 1, 1 ), mat );

      struct Config { real theta; Vec3 axis; real offT; real offZ; };
      const Config cfgs[] = {
         { real(0),    Vec3( 0, 0, 1 ), real(0.1),  real(-0.05) },
         { real(0),    Vec3( 0, 0, 1 ), real(-0.35), real(0.3) },
         { real(0.1),  Vec3( 0, 0, 1 ), real(0.2),  real(0.15) },
         { real(-0.7), Vec3( 0, 0, 1 ), real(-0.1), real(0.25) },
         { real(0.3),  Vec3( 0, 1, 0 ), real(0.3),  real(-0.2) },
         { real(0.5),  Vec3( 1, 2, 3 ).getNormalized(), real(-0.25), real(0.1) },
      };
      real worstDist( 0 ), worstNormal( 0 ), worstPoint( 0 );
      int found( 0 );
      const int n( sizeof( cfgs ) / sizeof( cfgs[0] ) );
      for( int k=0; k<n; ++k ) {
         b->setOrientation( Quat() );
         b->rotate( cfgs[k].axis, cfgs[k].theta );
         const Vec3 m( b->vectorFromBFtoWF( Vec3( -1, 0, 0 ) ) );   // outward normal of the -x face
         const Vec3 t( b->vectorFromBFtoWF( Vec3( 0, 1, 0 ) ) );
         const Vec3 u( b->vectorFromBFtoWF( Vec3( 0, 0, 1 ) ) );
         // face centre at distance 0.19 from the origin along -m, offset laterally in the face
         b->setPosition( -real(0.69) * m + cfgs[k].offT * t + cfgs[k].offZ * u );

         log.clear();
         MaxContacts::collide( ball, b, log );
         ContactLog refLog;
         MaxContacts::collide( ref, b, refLog );
         if( log.entries.size() != 1 || refLog.entries.size() != 1 ) {
            std::printf( "  sphere-box comparison %d: %zu ellipsoid-box, %zu sphere-box contacts\n", k, log.entries.size(), refLog.entries.size() );
            continue;
         }
         ++found;
         const ContactLog::Entry& c( log.entries[0] );
         const ContactLog::Entry& s( refLog.entries[0] );
         const Vec3 nc( c.g1 == ball ? c.normal : -c.normal );
         const Vec3 ns( s.g1 == ref  ? s.normal : -s.normal );
         worstDist   = std::max( worstDist,   std::fabs( c.dist - s.dist ) );
         worstNormal = std::max( worstNormal, ( nc - ns ).length() );
         worstPoint  = std::max( worstPoint,  ( c.pos - s.pos ).length() );
      }
      std::printf( "sphere-box comparison: %d/%d configurations, worst dist diff=%.3e, normal diff=%.3e, point diff=%.3e\n",
                   found, n, worstDist, worstNormal, worstPoint );
      expect( found == n,                 "sphere-box comparison: one contact from both paths for every configuration" );
      expect( worstDist   <= real(1e-10), "sphere-box comparison: same distance as collideSphereBox (1e-10)" );
      expect( worstNormal <= real(1e-10), "sphere-box comparison: same normal as collideSphereBox (1e-10)" );
      expect( worstPoint  <= real(1e-10), "sphere-box comparison: same contact point as collideSphereBox (1e-10, no shift)" );

      destroy( ball );
      destroy( ref );
      destroy( b );
   }

   // --- 14. strictly convex pairs: witness midpoint, pinned to the pre-convention values --------
   // The flat-body placement must not touch curved-vs-curved pairs. Reference values were
   // printed at %.17g by the library as of 619093f (midpoint convention for every GJK/EPA pair)
   // with gcc 13.2.0 -O2. They are compared with a tight tolerance rather than exactly, because
   // CI builds with other compilers and flags and the minimiser's last ulp may move; a genuine
   // convention change would shift the point by half the depth (5e-3 here), 1e10 times the
   // tolerance. The point is also re-derived as 0.5 * (support_1(-normal) + support_2(normal)).
   {
      const real pinTol( real(1e-12) );
      struct Pin { const char* tag; real dist; Vec3 normal; Vec3 point; };
      const Pin pins[] = {
         { "ellipsoid-ellipsoid head-on",  real(-0.010000000000000009),  Vec3( real(-1), real(-0), real(-0) ),
           Vec3( real(0.495), real(0), real(0) ) },
         { "ellipsoid-sphere head-on",     real(-0.010000000000000064),  Vec3( real(-1), real(-0), real(-0) ),
           Vec3( real(0.495), real(0), real(0) ) },
         { "ellipsoid-ellipsoid tilted",   real(-0.010000000000000037),
           Vec3( real(-0.8660254037844386), real(-0.5), real(3.910340272454317e-18) ),
           Vec3( real(0.42868257487329714), real(0.24749999999999997), real(0) ) },
         { "ellipsoid-ellipsoid oblique",  real(-9.7644165450583405e-17),
           Vec3( real(-0.89896874618588918), real(-0.38565759211374645), real(-0.20766178036894029) ),
           Vec3( real(0.48776130007567176), real(0.052312399433115796), real(0.010140557428573217) ) },
         { "ellipsoid-sphere oblique",     real(-0.010000000000000007),
           Vec3( real(-0.89896874618588896), real(-0.38565759211374623), real(-0.20766178036894034) ),
           Vec3( real(0.4832664563447423), real(0.050384111472547066), real(0.0091022485267285118) ) },
      };

      EllipsoidID p1 = createEllipsoid( 3030, Vec3( 0, 0, 0 ), A, B, C, mat );
      EllipsoidID p2 = createEllipsoid( 3031, Vec3( real(2)*A - delta, 0, 0 ), A, B, C, mat );
      SphereID    ps = createSphere( 3032, Vec3( A + real(0.2) - delta, 0, 0 ), real(0.2), mat );
      EllipsoidID q1 = createEllipsoid( 3033, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      EllipsoidID q2 = createEllipsoid( 3034, Vec3( 0, 0, 0 ), real(0.5), real(0.25), real(0.15), mat );
      Vec3 nq( real(1), real(0.13) + 23 * real(0.013), real(0.07) + 23 * real(0.007) );
      nq.normalize();
      q2->setPosition( real(2) * q1->support( nq ) );
      SphereID qs = createSphere( 3035, q1->support( nq ) + ( real(0.2) - delta ) * nq, real(0.2), mat );

      const real ang( real(30) * M_PI / real(180) );
      const Vec3 axis( std::cos( ang ), std::sin( ang ), 0 );

      for( int k=0; k<5; ++k ) {
         GeomID g1 = 0, g2 = 0;
         switch( k ) {
            case 0: g1 = p1; g2 = p2; break;
            case 1: g1 = p1; g2 = ps; break;
            case 2: p1->rotate( Vec3( 0, 0, 1 ), ang ); p2->rotate( Vec3( 0, 0, 1 ), ang );
                    p2->setPosition( axis * ( real(2)*A - delta ) ); g1 = p1; g2 = p2; break;
            case 3: g1 = q1; g2 = q2; break;
            case 4: g1 = q1; g2 = qs; break;
         }
         log.clear();
         MaxContacts::collide( g1, g2, log );
         char what[160];
         std::snprintf( what, sizeof what, "strictly convex pin (%s): exactly one contact", pins[k].tag );
         expect( log.entries.size() == 1, what );
         if( log.entries.size() != 1 ) continue;
         const ContactLog::Entry& c( log.entries[0] );
         std::printf( "strictly convex pin (%s): dist=%.17g point=(%.17g,%.17g,%.17g)\n",
                      pins[k].tag, c.dist, c.pos[0], c.pos[1], c.pos[2] );
         std::snprintf( what, sizeof what, "strictly convex pin (%s): bodies in dispatch order", pins[k].tag );
         expect( c.g1 == g1 && c.g2 == g2, what );
         std::snprintf( what, sizeof what, "strictly convex pin (%s): dist matches the pre-convention value (1e-12)", pins[k].tag );
         expect( std::fabs( c.dist - pins[k].dist ) <= pinTol, what );
         std::snprintf( what, sizeof what, "strictly convex pin (%s): normal matches the pre-convention value (1e-12)", pins[k].tag );
         expect( ( c.normal - pins[k].normal ).length() <= pinTol, what );
         std::snprintf( what, sizeof what, "strictly convex pin (%s): contact point matches the pre-convention value (1e-12)", pins[k].tag );
         expect( ( c.pos - pins[k].point ).length() <= pinTol, what );
         const Vec3 mid( real(0.5) * ( g1->support( -c.normal ) + g2->support( c.normal ) ) );
         std::snprintf( what, sizeof what, "strictly convex pin (%s): contact point is the witness midpoint (1e-13)", pins[k].tag );
         expect( ( c.pos - mid ).length() <= real(1e-13), what );
      }

      destroy( qs );
      destroy( q2 );
      destroy( q1 );
      destroy( ps );
      destroy( p2 );
      destroy( p1 );
   }

   if( failures == 0 ) {
      std::printf( "pe_ellipsoid_contact_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_ellipsoid_contact_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
