//=================================================================================================
/*!
 *  \file tests/interface/pe_cylinder_plane_contact_test.cpp
 *  \brief Narrow-phase contact generation for cylinder-plane pairs through MaxContacts::collide().
 *
 *  Before this test MaxContacts::collideCylinderPlane() was an empty stub: a cylinder never got
 *  a contact with a plane and fell through the ground. The routine now tests four rim points
 *  per end cap (deepest along -n, opposite, and the two in between). Asserted here with a
 *  recording contact container (no solver involved), cylinder radius r = 0.5, length L = 1:
 *    1. standing flat on an end cap, penetration delta: four contacts, all at dist -delta, on
 *       the plane, at distance r from the axis foot, forming a square (sides r sqrt(2));
 *    2. lying on its side: two contacts at dist -delta, the end points of the contact line
 *       (x = -L/2 and x = +L/2 below the axis);
 *    3. standing on its rim, tilted 30 degrees: exactly one contact whose dist matches the
 *       analytic lowest point  z_c - (L/2) |a_z| - r sqrt(1 - a_z^2), placed on the plane below
 *       that point;
 *    4. contactThreshold: 0.5 * contactThreshold above the plane -> contact, 2 * contactThreshold
 *       above -> none;
 *    5. both dispatch orders (cylinder first / plane first) give the same contacts;
 *    6. random sweep, 200 cases: random orientation, random plane normal and offset, signed
 *       distance of the cylinder's support point in [-0.05, 0.05]: the deepest contact equals
 *       the independent CylinderBase::support() distance (1e-12) and point (1e-9), every
 *       contact lies on the plane with the plane normal, at most 8 contacts, and a separated
 *       cylinder gets none.
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

//! Every contact: cylinder is g1, normal is the plane normal, point on the plane surface.
static bool consistent( const ContactLog& log, CylinderID c, PlaneID p, real tol )
{
   for( const ContactLog::Entry& e : log.entries ) {
      if( e.g1 != c || e.g2 != p )                                  return false;
      if( ( e.normal - p->getNormal() ).length() > tol )             return false;
      if( !close( trans( p->getNormal() ) * e.pos, p->getDisplacement(), tol ) ) return false;
   }
   return true;
}

static real minDist( const ContactLog& log )
{
   real m = std::numeric_limits<real>::max();
   for( const ContactLog::Entry& e : log.entries )
      m = std::min( m, e.dist );
   return m;
}

int main()
{
   WorldID world = theWorld();

   MaterialID mat = createMaterial( "cylinder_plane_test", real(1), real(0.1), real(0.05), real(0.05),
                                    real(0.2), real(80), real(100), real(10), real(11) );

   const real r     = real(0.5);
   const real L     = real(1.0);
   const real delta = real(0.01);
   const real pi    = real(3.14159265358979323846);
   ContactLog log;

   // 1. Standing flat on an end cap: the axis (body x) rotated onto -z, lower cap at z = -delta.
   {
      world->clear();
      PlaneID    pl = createPlane( 1, Vec3( 0, 0, 1 ), real(0), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0.2, -0.1, 0.5 * L - delta ), r, L, mat );
      cy->setOrientation( Quat( real(0), pi / 2, real(0) ) );

      log.clear();
      MaxContacts::collide( cy, pl, log );
      expect( log.entries.size() == 4, "flat on cap: four contacts" );
      expect( consistent( log, cy, pl, real(1e-12) ), "flat on cap: g1 = cylinder, plane normal, points on the plane" );
      bool depthOk = true, radiusOk = true;
      for( const ContactLog::Entry& e : log.entries ) {
         depthOk  &= close( e.dist, -delta, real(1e-12) );
         radiusOk &= close( ( e.pos - Vec3( 0.2, -0.1, 0.0 ) ).length(), r, real(1e-12) );
      }
      expect( depthOk,  "flat on cap: every contact at dist -delta" );
      expect( radiusOk, "flat on cap: every contact on the rim, radius r from the axis foot" );
      if( log.entries.size() == 4 ) {
         // Square: each point has two neighbours at r sqrt(2) and one opposite at 2r.
         bool squareOk = true;
         for( std::size_t i = 0; i < 4; ++i ) {
            int nNeighbour = 0, nOpposite = 0;
            for( std::size_t j = 0; j < 4; ++j ) {
               if( i == j ) continue;
               const real d = ( log.entries[i].pos - log.entries[j].pos ).length();
               if( close( d, r * std::sqrt( real(2) ), real(1e-12) ) ) ++nNeighbour;
               if( close( d, real(2) * r, real(1e-12) ) )            ++nOpposite;
            }
            squareOk &= ( nNeighbour == 2 && nOpposite == 1 );
         }
         expect( squareOk, "flat on cap: the four support points form a square" );
      }
   }

   // 2. Lying on its side: axis along x, line contact at z = -delta from x = -L/2 to x = +L/2.
   {
      world->clear();
      PlaneID    pl = createPlane( 1, Vec3( 0, 0, 1 ), real(0), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0.0, 0.3, r - delta ), r, L, mat );

      log.clear();
      MaxContacts::collide( cy, pl, log );
      expect( log.entries.size() == 2, "lying: two contacts" );
      expect( consistent( log, cy, pl, real(1e-12) ), "lying: g1 = cylinder, plane normal, points on the plane" );
      bool endsOk = ( log.entries.size() == 2 );
      for( const ContactLog::Entry& e : log.entries ) {
         endsOk &= close( e.dist, -delta, real(1e-12) );
         endsOk &= close( std::fabs( e.pos[0] ), real(0.5) * L, real(1e-12) );
         endsOk &= close( e.pos[1], real(0.3), real(1e-12) );
      }
      if( log.entries.size() == 2 )
         endsOk &= ( log.entries[0].pos[0] * log.entries[1].pos[0] < real(0) );
      expect( endsOk, "lying: contacts at dist -delta at both ends of the contact line" );
   }

   // 3. Standing on its rim, tilted 30 degrees from upright.
   {
      world->clear();
      PlaneID    pl = createPlane( 1, Vec3( 0, 0, 1 ), real(0), mat );
      const real tilt = pi / 6;
      const Quat q( real(0), pi / 2 - tilt, real(0) );
      const Vec3 axis( q.toRotationMatrix() * Vec3( 1, 0, 0 ) );
      const real az   = std::fabs( axis[2] );
      // Analytic depth of the lowest rim point below the centre: (L/2) |a_z| + r sqrt(1 - a_z^2).
      const real below = real(0.5) * L * az + r * std::sqrt( real(1) - az * az );
      CylinderID cy = createCylinder( 2, Vec3( 0, 0, below - delta ), r, L, mat );
      cy->setOrientation( q );   // lowest rim point at z = -delta

      log.clear();
      MaxContacts::collide( cy, pl, log );
      expect( log.entries.size() == 1, "rim: exactly one contact" );
      expect( consistent( log, cy, pl, real(1e-12) ), "rim: g1 = cylinder, plane normal, point on the plane" );
      if( !log.entries.empty() ) {
         expect( close( log.entries[0].dist, -delta, real(1e-12) ), "rim: dist matches the analytic lowest rim point" );
         const Vec3 deepest( cy->support( Vec3( 0, 0, -1 ) ) );
         expect( ( log.entries[0].pos - ( deepest + delta * Vec3( 0, 0, 1 ) ) ).length() <= real(1e-12),
                 "rim: contact point is the lowest rim point projected onto the plane" );
      }
   }

   // 4. contactThreshold boundary, lying cylinder.
   {
      world->clear();
      PlaneID    pl = createPlane( 1, Vec3( 0, 0, 1 ), real(0), mat );
      CylinderID cy = createCylinder( 2, Vec3( 0, 0, r + real(0.5) * contactThreshold ), r, L, mat );
      log.clear();
      MaxContacts::collide( cy, pl, log );
      expect( log.entries.size() == 2, "threshold: 0.5 * contactThreshold above the plane -> contacts" );

      cy->setPosition( Vec3( 0, 0, r + real(2) * contactThreshold ) );
      log.clear();
      MaxContacts::collide( cy, pl, log );
      expect( log.entries.empty(), "threshold: 2 * contactThreshold above the plane -> no contact" );
   }

   // 5 + 6. Random sweep, both dispatch orders.
   {
      std::mt19937 rng( 12345 );
      std::uniform_real_distribution<double> uni( -1.0, 1.0 );
      int cases = 0, withContacts = 0;
      bool orderOk = true, depthOk = true, pointOk = true, consistentOk = true, countOk = true, separatedOk = true;

      for( int k = 0; k < 200; ++k ) {
         world->clear();
         Vec3 n;
         do { n = Vec3( uni( rng ), uni( rng ), uni( rng ) ); } while( n.length() < real(0.1) );
         n.normalize();
         const real d = real(0.3) * uni( rng );
         PlaneID pl = createPlane( 1, n, d, mat );

         const real radius = real(0.2) + real(0.3) * std::fabs( uni( rng ) );
         const real length = real(0.2) + real(0.8) * std::fabs( uni( rng ) );
         CylinderID cy = createCylinder( 2, Vec3( 0, 0, 0 ), radius, length, mat );
         Quat q( uni( rng ), uni( rng ), uni( rng ), uni( rng ) );
         q = q.getNormalized();
         cy->setOrientation( q );

         // Shift along n so that the support point has signed distance 'target'.
         const real target = real(0.05) * uni( rng );
         const real dist0  = trans( n ) * cy->support( -n ) - d;
         cy->setPosition( ( target - dist0 ) * n );
         const Vec3 deepest( cy->support( -n ) );
         ++cases;

         log.clear();
         MaxContacts::collide( cy, pl, log );
         ContactLog swapped;
         MaxContacts::collide( pl, cy, swapped );

         orderOk      &= ( swapped.entries.size() == log.entries.size() );
         for( std::size_t i = 0; i < std::min( log.entries.size(), swapped.entries.size() ); ++i )
            orderOk &= ( log.entries[i].pos - swapped.entries[i].pos ).length() <= real(1e-14)
                    && log.entries[i].dist == swapped.entries[i].dist;
         consistentOk &= consistent( log, cy, pl, real(1e-12) );
         countOk      &= ( log.entries.size() <= 8 );

         if( target < contactThreshold ) {
            ++withContacts;
            depthOk &= !log.entries.empty() && close( minDist( log ), target, real(1e-12) );
            for( const ContactLog::Entry& e : log.entries )
               if( e.dist == minDist( log ) )
                  pointOk &= ( e.pos - ( deepest - target * n ) ).length() <= real(1e-9);
         }
         else {
            separatedOk &= log.entries.empty();
         }
      }
      std::printf( "random sweep: %d cases, %d penetrating or touching\n", cases, withContacts );
      expect( orderOk,      "sweep: both dispatch orders give identical contacts" );
      expect( depthOk,      "sweep: deepest contact dist equals the CylinderBase::support() distance (1e-12)" );
      expect( pointOk,      "sweep: deepest contact point is the support point projected onto the plane (1e-9)" );
      expect( consistentOk, "sweep: every contact on the plane with the plane normal, g1 = cylinder" );
      expect( countOk,      "sweep: at most 8 contacts" );
      expect( separatedOk,  "sweep: separated cylinders get no contact" );
   }

   if( failures == 0 ) {
      std::printf( "pe_cylinder_plane_contact_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_cylinder_plane_contact_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
