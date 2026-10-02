//=================================================================================================
/*!
 *  \file tests/interface/pe_primitive_mesh_distancemap_test.cpp
 *  \brief Primitive-mesh contact generation on a DistanceMap (sphere, box, capsule, cylinder,
 *         ellipsoid against a non-convex triangle mesh). Requires CGAL.
 *
 *  Before, box-, capsule-, cylinder-, ellipsoid- and sphere-mesh contacts came from GJK/EPA
 *  (one contact, the mesh treated as convex) or a brute-force closest triangle (sphere), even
 *  when the mesh had a DistanceMap. MaxContacts::collideTMeshWithDistanceMap() now samples the
 *  primitive's surface against the mesh's signed distance field whenever a DistanceMap is
 *  present (no GJK/EPA fallback then, the mesh may be non-convex).
 *
 *  The mesh is a torus (major radius R = 1, minor radius r = 0.3, 64 x 32 segments) with a
 *  DistanceMap (resolution 60, tolerance 5). Its ideal signed distance is analytic,
 *  d(p) = sqrt( ( sqrt( x^2 + y^2 ) - R )^2 + z^2 ) - r, so the contacts can be checked without
 *  CGAL queries; the polygonal mesh and the trilinear interpolation deviate from it by a few 1e-3,
 *  hence the 1e-2 tolerances. Asserted with a recording contact container (no solver):
 *    1. each primitive resting on top of the tube at x = R, penetrating by 0.02: contacts, the
 *       deepest within 1e-2 of -0.02, normal within 5 degrees of the analytic torus normal at the
 *       contact point (from the mesh towards the primitive), contact point on the tube crest
 *       (z within 1e-2 of r), both dispatch orders;
 *    2. each primitive separated from the tube by 0.1: no contact;
 *    3. each primitive inside the hole of the torus, clear of the tube (the convex hull of the
 *       mesh would contain it): no contact. This is what the DistanceMap path adds over GJK/EPA;
 *    4. the torus translated and rotated (hole axis along world y): a sphere on the tube in the
 *       mesh's frame gives the same depth and the rotated normal (world/body transforms);
 *    5. with the DistanceMap disabled the pairs still produce contacts for the resting
 *       configuration (the GJK/EPA path is untouched);
 *    6. a closed slab mesh (2 x 2 x 0.5, 12 triangles) with a DistanceMap (resolution 40):
 *       - a 0.4 cube resting flat on it, penetrating 0.01: at least four contacts whose points
 *         spread over at least 0.3 in both face directions (deepest plus four extremal points of
 *         one cluster; a lopsided manifold would let the box rock), all with normal +z and depth
 *         within 1e-2 of -0.01;
 *       - a capsule of length 6 lying across the slab (its ends far outside the grid): contacts
 *         only from samples inside the grid, every distance in (-0.1, 0) (no 1e6 sentinel leaks
 *         through), the deepest on the top face with normal +z (contacts at the slab's edges
 *         carry the side face's normal);
 *       - a cylinder of length 3 standing on the slab (its centre 1.5 above the top, outside the
 *         grid padding of 5 cells): the clamped centre query still yields the exact deepest
 *         point, depth within 1e-2 of -0.01.
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
};

static const real kR = real(1.0), kr = real(0.3);
static const real kPi = real(3.14159265358979323846);

//! Torus with outward-oriented triangles, centred at the origin, hole axis z.
static void makeTorus( real R, real r, int nMajor, int nMinor, Vertices& vertices, IndicesLists& faces )
{
   vertices.clear();
   faces.clear();
   for( int i = 0; i < nMajor; ++i ) {
      const real u( real(2) * kPi * i / nMajor );
      for( int j = 0; j < nMinor; ++j ) {
         const real v( real(2) * kPi * j / nMinor );
         vertices.push_back( Vec3( ( R + r * std::cos( v ) ) * std::cos( u ),
                                   ( R + r * std::cos( v ) ) * std::sin( u ),
                                   r * std::sin( v ) ) );
      }
   }
   const auto at = [nMajor, nMinor]( int i, int j ) {
      return static_cast<size_t>( ( i % nMajor ) * nMinor + ( j % nMinor ) );
   };
   for( int i = 0; i < nMajor; ++i )
      for( int j = 0; j < nMinor; ++j ) {
         faces.push_back( Vector3<size_t>( at( i, j ), at( i + 1, j ), at( i + 1, j + 1 ) ) );
         faces.push_back( Vector3<size_t>( at( i, j ), at( i + 1, j + 1 ), at( i, j + 1 ) ) );
      }
}

//! Closed axis-aligned slab (lx x ly x lz) centred at the origin, outward-oriented triangles.
static void makeSlab( real lx, real ly, real lz, Vertices& vertices, IndicesLists& faces )
{
   vertices.clear();
   faces.clear();
   const real hx( real(0.5) * lx ), hy( real(0.5) * ly ), hz( real(0.5) * lz );
   for( int i = 0; i < 8; ++i )
      vertices.push_back( Vec3( ( i & 1 ) ? hx : -hx, ( i & 2 ) ? hy : -hy, ( i & 4 ) ? hz : -hz ) );
   const size_t quads[6][4] = { { 0, 2, 3, 1 }, { 4, 5, 7, 6 }, { 0, 1, 5, 4 }, { 2, 6, 7, 3 }, { 0, 4, 6, 2 }, { 1, 3, 7, 5 } };
   for( const auto& q : quads ) {
      faces.push_back( Vector3<size_t>( q[0], q[1], q[2] ) );
      faces.push_back( Vector3<size_t>( q[0], q[2], q[3] ) );
   }
}

static TriangleMeshID makeSlabBody( pe::id_t uid, MaterialID mat )
{
   Vertices vertices;
   IndicesLists faces;
   makeSlab( real(2), real(2), real(0.5), vertices, faces );
   TriangleMeshID m = createTriangleMesh( uid, Vec3( 0, 0, 0 ), vertices, faces, mat, /*convex=*/true );
#ifdef PE_USE_CGAL
   m->enableDistanceMapAcceleration( 40, 5 );
#endif
   return m;
}

static TriangleMeshID makeTorusBody( pe::id_t uid, const Vec3& pos, MaterialID mat, bool distanceMap )
{
   Vertices vertices;
   IndicesLists faces;
   makeTorus( kR, kr, 64, 32, vertices, faces );
   TriangleMeshID m = createTriangleMesh( uid, pos, vertices, faces, mat, /*convex=*/false );
#ifdef PE_USE_CGAL
   if( distanceMap )
      m->enableDistanceMapAcceleration( 60, 5 );
#else
   (void)distanceMap;
#endif
   return m;
}

static ContactLog run( BodyID a, BodyID b )
{
   ContactLog log;
   MaxContacts::collide( a, b, log );
   return log;
}

struct Deepest { bool any; real dist; Vec3 normal; Vec3 pos; };

//! Deepest contact with the normal oriented from the mesh towards the primitive.
static Deepest deepest( const ContactLog& log, BodyID prim )
{
   Deepest d{ false, real(0), Vec3(), Vec3() };
   for( const ContactLog::Entry& e : log.entries ) {
      if( !d.any || e.dist < d.dist ) {
         d.any    = true;
         d.dist   = e.dist;
         d.normal = ( e.g1 == prim ) ? e.normal : -e.normal;
         d.pos    = e.pos;
      }
   }
   return d;
}

enum Prim { kSphere, kBox, kCapsule, kCylinder, kEllipsoid, kNumPrims };
static const char* const kPrimNames[kNumPrims] = { "sphere", "box", "capsule", "cylinder", "ellipsoid" };

//! A primitive whose lowest point (along body -z) is \a bottomOffset below \a centreTop... the
//! primitives are built so that their lowest point is 'extent' below the centre:
//! sphere r 0.2, box 0.4 cube, capsule r 0.15 lying along y, cylinder r 0.2 upright (axis z),
//! ellipsoid (0.3, 0.2, 0.15).
static BodyID makePrim( Prim p, pe::id_t uid, MaterialID mat, real& extent )
{
   const Vec3 o( 0, 0, 0 );
   switch( p ) {
      case kSphere:    extent = real(0.2);  return createSphere( uid, o, real(0.2), mat );
      case kBox:       extent = real(0.2);  return createBox( uid, o, Vec3( 0.4, 0.4, 0.4 ), mat );
      case kCapsule: { extent = real(0.15); CapsuleID c = createCapsule( uid, o, real(0.15), real(0.5), mat );
                       c->setOrientation( Quat( real(0), real(0), kPi / 2 ) ); return c; }   // axis along y
      case kCylinder:{ extent = real(0.3);  CylinderID c = createCylinder( uid, o, real(0.2), real(0.6), mat );
                       c->setOrientation( Quat( real(0), kPi / 2, real(0) ) ); return c; }    // axis along z
      default:         extent = real(0.15); return createEllipsoid( uid, o, real(0.3), real(0.2), real(0.15), mat );
   }
}

int main()
{
   WorldID world = theWorld();
   MaterialID mat = createMaterial( "primitive_mesh_dm_test", real(1), real(0), real(0.3), real(0.3),
                                    real(0.25), real(200), real(1000), real(10), real(11) );

#ifndef PE_USE_CGAL
   std::printf( "pe_primitive_mesh_distancemap_test: built without CGAL, nothing to test\n" );
   return EXIT_SUCCESS;
#endif

   const real delta( real(0.02) );
   char what[160];

   for( int p = 0; p < kNumPrims; ++p ) {
      // 1. Resting on top of the tube, penetrating by delta.
      world->clear();
      TriangleMeshID torus = makeTorusBody( 1, Vec3( 0, 0, 0 ), mat, true );
      expect( torus->hasDistanceMap(), "torus has a DistanceMap" );
      real extent( 0 );
      BodyID prim = makePrim( static_cast<Prim>( p ), 2, mat, extent );
      prim->setPosition( Vec3( kR, 0, kr + extent - delta ) );

      ContactLog log = run( torus, prim );
      Deepest d = deepest( log, prim );
      std::snprintf( what, sizeof what, "%s on the tube: contacts generated", kPrimNames[p] );
      expect( d.any, what );
      if( d.any ) {
         std::printf( "%-9s on tube: %d contact(s), deepest dist %+.4f, normal (%+.3f %+.3f %+.3f), point z %.4f\n",
                      kPrimNames[p], static_cast<int>( log.entries.size() ), static_cast<double>( d.dist ),
                      static_cast<double>( d.normal[0] ), static_cast<double>( d.normal[1] ), static_cast<double>( d.normal[2] ),
                      static_cast<double>( d.pos[2] ) );
         std::snprintf( what, sizeof what, "%s on the tube: deepest dist within 1e-2 of -delta", kPrimNames[p] );
         expect( std::fabs( d.dist + delta ) < real(1e-2), what );
         // The analytic torus normal at the contact point (the tube curves away from the crest
         // along the primitive, so a contact off the crest line has a tilted normal).
         const real rho( std::sqrt( d.pos[0] * d.pos[0] + d.pos[1] * d.pos[1] ) );
         const Vec3 core( kR * d.pos[0] / rho, kR * d.pos[1] / rho, real(0) );
         const Vec3 nAnalytic( ( d.pos - core ).getNormalized() );
         std::snprintf( what, sizeof what, "%s on the tube: normal within 5 degrees of the torus normal at the contact point", kPrimNames[p] );
         expect( trans( d.normal ) * nAnalytic > std::cos( real(5) * kPi / 180 ) && d.normal[2] > real(0.9), what );
         std::snprintf( what, sizeof what, "%s on the tube: contact point on the tube crest", kPrimNames[p] );
         expect( std::fabs( d.pos[2] - kr ) < real(1e-2) && std::fabs( d.pos[0] - kR ) < real(0.1), what );
      }
      ContactLog swapped = run( prim, torus );
      std::snprintf( what, sizeof what, "%s on the tube: both dispatch orders agree", kPrimNames[p] );
      expect( swapped.entries.size() == log.entries.size()
              && ( !d.any || std::fabs( deepest( swapped, prim ).dist - d.dist ) < real(1e-12) ), what );

      // 2. Separated by 0.1.
      prim->setPosition( Vec3( kR, 0, kr + extent + real(0.1) ) );
      std::snprintf( what, sizeof what, "%s 0.1 above the tube: no contact", kPrimNames[p] );
      expect( run( torus, prim ).entries.empty(), what );

      // 3. Inside the hole, clear of the tube (hole radius R - r = 0.7; every primitive here is
      // at most 0.42 from the axis).
      prim->setPosition( Vec3( 0, 0, 0 ) );
      std::snprintf( what, sizeof what, "%s in the torus hole: no contact (non-convex mesh)", kPrimNames[p] );
      expect( run( torus, prim ).entries.empty(), what );

      // 5. Without the DistanceMap the GJK/EPA path still finds the resting contact.
      torus->disableDistanceMapAcceleration();
      expect( !torus->hasDistanceMap(), "DistanceMap disabled" );
      prim->setPosition( Vec3( kR, 0, kr + extent - delta ) );
      std::snprintf( what, sizeof what, "%s on the tube without DistanceMap: GJK/EPA contact", kPrimNames[p] );
      expect( !run( torus, prim ).entries.empty(), what );
   }

   // 4. Transformed torus: at (1, 2, 0.5), hole axis along world y (rotated 90 degrees about x).
   {
      world->clear();
      TriangleMeshID torus = makeTorusBody( 1, Vec3( 1, 2, 0.5 ), mat, true );
      torus->setOrientation( Quat( kPi / 2, real(0), real(0) ) );
      SphereID s = createSphere( 2, Vec3( 0, 0, 0 ), real(0.2), mat );
      s->setPosition( torus->pointFromBFtoWF( Vec3( kR, 0, kr + real(0.2) - delta ) ) );
      const Vec3 up( torus->vectorFromBFtoWF( Vec3( 0, 0, 1 ) ) );

      ContactLog log = run( torus, s );
      Deepest d = deepest( log, s );
      std::printf( "sphere on the transformed torus: %d contact(s), deepest dist %+.4f, normal . body-up %.4f\n",
                   static_cast<int>( log.entries.size() ), static_cast<double>( d.dist ),
                   static_cast<double>( trans( d.normal ) * up ) );
      expect( d.any && std::fabs( d.dist + delta ) < real(1e-2), "transformed torus: deepest dist within 1e-2 of -delta" );
      expect( d.any && trans( d.normal ) * up > std::cos( real(5) * kPi / 180 ), "transformed torus: normal follows the mesh orientation" );
      expect( d.any && std::fabs( trans( d.pos - torus->getPosition() ) * up - kr ) < real(1e-2 ),
              "transformed torus: contact point on the tube crest" );
      s->setPosition( torus->getPosition() );
      expect( run( torus, s ).entries.empty(), "transformed torus: sphere in the hole, no contact" );
   }

   // 6. Slab mesh: flat resting manifold, samples outside the grid, centre outside the grid.
   {
      world->clear();
      TriangleMeshID slab = makeSlabBody( 1, mat );
      expect( slab->hasDistanceMap(), "slab has a DistanceMap" );
      const real top( real(0.25) );

      BoxID box = createBox( 2, Vec3( 0.1, -0.1, top + real(0.2) - real(0.01) ), Vec3( 0.4, 0.4, 0.4 ), mat );
      ContactLog log = run( slab, box );
      real minX( 1e30 ), maxX( -1e30 ), minY( 1e30 ), maxY( -1e30 );
      bool flatOk = log.entries.size() >= 4;
      for( const ContactLog::Entry& e : log.entries ) {
         minX = std::min( minX, e.pos[0] ); maxX = std::max( maxX, e.pos[0] );
         minY = std::min( minY, e.pos[1] ); maxY = std::max( maxY, e.pos[1] );
         const Vec3 n( e.g1 == box ? e.normal : -e.normal );
         flatOk = flatOk && n[2] > real(0.99) && std::fabs( e.dist + real(0.01) ) < real(1e-2);
      }
      std::printf( "box flat on the slab: %d contact(s), spread x %.3f, y %.3f\n", static_cast<int>( log.entries.size() ),
                   static_cast<double>( maxX - minX ), static_cast<double>( maxY - minY ) );
      expect( flatOk, "box flat on the slab: >= 4 contacts, normal +z, depth within 1e-2 of -0.01" );
      expect( maxX - minX > real(0.3) && maxY - minY > real(0.3), "box flat on the slab: contacts spread over the face in both directions" );
      destroy( box );

      CapsuleID cap = createCapsule( 3, Vec3( 0, 0, top + real(0.15) - real(0.01) ), real(0.15), real(6), mat );
      log = run( slab, cap );
      // Contacts at the slab's edges (x = +-1) see the side face as nearest surface, so their
      // normals turn sideways; only the deepest (on the top face) must be +z.
      bool straddleOk = !log.entries.empty();
      for( const ContactLog::Entry& e : log.entries ) {
         const Vec3 n( e.g1 == cap ? e.normal : -e.normal );
         std::printf( "   capsule contact: dist %+.4f pos (%+.3f %+.3f %+.3f) n (%+.3f %+.3f %+.3f)\n", static_cast<double>( e.dist ),
                      static_cast<double>( e.pos[0] ), static_cast<double>( e.pos[1] ), static_cast<double>( e.pos[2] ),
                      static_cast<double>( n[0] ), static_cast<double>( n[1] ), static_cast<double>( n[2] ) );
         // Samples within contactThreshold above the surface get a penetration of 0 (dist -0.0).
         straddleOk = straddleOk && e.dist > real(-0.1) && e.dist <= real(0)
                      && std::fabs( e.pos[0] ) < real(1.01) && std::fabs( e.pos[1] ) < real(1.01);
      }
      const Deepest dc = deepest( log, cap );
      straddleOk = straddleOk && dc.any && std::fabs( dc.dist + real(0.01) ) < real(1e-2) && dc.normal[2] > real(0.99);
      std::printf( "capsule across the slab (ends outside the grid): %d contact(s), deepest %+.4f\n",
                   static_cast<int>( log.entries.size() ), static_cast<double>( dc.dist ) );
      expect( straddleOk, "capsule across the slab: contacts only inside the grid, depths in (-0.1, 0), deepest on the top with normal +z" );
      destroy( cap );

      CylinderID cyl = createCylinder( 4, Vec3( 0, 0, 0 ), real(0.2), real(3), mat );
      cyl->setOrientation( Quat( real(0), kPi / 2, real(0) ) );          // axis along z
      cyl->setPosition( Vec3( 0.2, 0.1, top + real(1.5) - real(0.01) ) );   // centre 1.49 above the top
      log = run( slab, cyl );
      Deepest d = deepest( log, cyl );
      std::printf( "tall cylinder on the slab (centre outside the grid): %d contact(s), deepest %+.4f\n",
                   static_cast<int>( log.entries.size() ), static_cast<double>( d.dist ) );
      expect( d.any && std::fabs( d.dist + real(0.01) ) < real(1e-2) && d.normal[2] > real(0.99),
              "tall cylinder on the slab: exact deepest point although the centre is outside the grid" );
   }

   if( failures == 0 ) {
      std::printf( "pe_primitive_mesh_distancemap_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_primitive_mesh_distancemap_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
