//=================================================================================================
/*!
 *  \file tests/interface/pe_box_box_parallel_edges_test.cpp
 *  \brief Box-box contacts with (nearly) parallel edges, checked against GJK.
 *
 *  MaxContacts::collideBoxBox() is a separating-axis test whose nine edge-pair axes are the cross
 *  products of one edge of each box. For edges parallel up to rounding the cross product is
 *  ~1e-16 long; it used to pass the machine-epsilon guard, and the separation along it (rounding
 *  noise divided by that length) came out as an O(1) value of random sign. A positive one won the
 *  axis selection and replaced the real contact of two OVERLAPPING boxes by one edge/edge contact
 *  with a positive distance: the solver saw a gap and let the boxes pass into each other. Now
 *  edge pairs within 1e-6 of parallel are skipped and the rejection test is normalised.
 *
 *  Asserted with a recording contact container (no solver):
 *    1. random near-parallel configurations: A arbitrarily oriented, B = A * (random quarter-turn
 *       symmetry) * (perturbation of 1e-16 ... 1e-4 rad, half of them with a random yaw), B placed
 *       anywhere around A with overlapping bounding boxes. Against GJK:
 *       - no contact has a distance above contactThreshold (before: 1.7 % of the overlapping
 *         configurations, up to +0.24),
 *       - no contact for boxes more than 1e-6 apart,
 *       - every overlap with a corner more than 1e-6 inside the other box yields contacts, and the
 *         deepest reported contact is at least as deep as that corner;
 *    2. clean edge-versus-face approach (B turned 45 degrees about z, a vertical edge leading
 *       towards A's -x face, tilts from 0 to 1e-2): no contact while apart, and while penetrating
 *       vertex/face contacts with A's face normal at the penetration depth.
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/core/detection/fine/GJK.h>
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

// Recording contact container: the minimal interface MaxContacts expects.
struct ContactLog
{
   struct Entry { Vec3 pos; Vec3 normal; real dist; bool edgeEdge; };
   std::vector<Entry> entries;

   void addVertexFaceContact( GeomID, GeomID, const Vec3& gpos, const Vec3& normal, real dist ) {
      entries.push_back( Entry{ gpos, normal, dist, false } );
   }
   void addLubricationContact( GeomID, GeomID, const Vec3&, const Vec3&, real, real = real(1) ) {}
   void addEdgeEdgeContact( GeomID, GeomID, const Vec3& gpos, const Vec3& normal,
                            const Vec3&, const Vec3&, real dist ) {
      entries.push_back( Entry{ gpos, normal, dist, true } );
   }
};

//! Deepest penetration of a corner of \a p into box \a q (0 if no corner is inside).
static real cornerDepth( BoxID p, BoxID q )
{
   real worst( 0 );
   const Vec3 h( real(0.5) * p->getLengths() ), hq( real(0.5) * q->getLengths() );
   for( int i = 0; i < 8; ++i ) {
      const Vec3 c( p->pointFromBFtoWF( Vec3( ( i & 1 ) ? h[0] : -h[0], ( i & 2 ) ? h[1] : -h[1], ( i & 4 ) ? h[2] : -h[2] ) ) );
      const Vec3 l( q->pointFromWFtoBF( c ) );
      worst = std::max( worst, std::min( { hq[0] - std::fabs( l[0] ), hq[1] - std::fabs( l[1] ), hq[2] - std::fabs( l[2] ) } ) );
   }
   return worst;
}

int main()
{
   WorldID world = theWorld();
   MaterialID mat = createMaterial( "box_box_parallel_test", real(1), real(0), real(0.3), real(0.3),
                                    real(0.25), real(200), real(1000), real(10), real(11) );
   const real pi( real(3.14159265358979323846) );

   // 1. Random near-parallel configurations against GJK.
   {
      std::mt19937 rng( 11 );
      std::uniform_real_distribution<double> u( -1.0, 1.0 );
      std::uniform_int_distribution<int> quarter( 0, 3 );
      const real turns[4] = { real(0), real(0.5) * pi, pi, real(1.5) * pi };

      long tested = 0, overlapping = 0;
      int positive = 0, apartWithContact = 0, missed = 0, shallow = 0;
      real worstPositive( 0 ), worstShallow( 0 );

      for( int k = 0; k < 40000; ++k ) {
         world->clear();
         BoxID a = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
         BoxID b = createBox( 2, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
         const Quat qa( pi * u( rng ), pi * u( rng ), pi * u( rng ) );
         const Quat sym( turns[quarter( rng )], turns[quarter( rng )], turns[quarter( rng )] );
         const double eps( std::pow( 10.0, -16.0 + 12.0 * 0.5 * ( u( rng ) + 1.0 ) ) );
         const Quat pert( eps * u( rng ), eps * u( rng ), ( k % 2 ) ? 0.6 * u( rng ) : eps * u( rng ) );
         a->setOrientation( qa );
         b->setOrientation( qa * sym * pert );

         for( int j = 0; j < 50; ++j ) {
            b->setPosition( Vec3( 1.5 * u( rng ), 1.5 * u( rng ), 1.5 * u( rng ) ) );
            if( !a->getAABB().overlaps( b->getAABB() ) )
               continue;
            ++tested;

            ContactLog log;
            MaxContacts::collide( a, b, log );
            real dmin( std::numeric_limits<real>::max() );
            for( const ContactLog::Entry& c : log.entries ) {
               dmin = std::min( dmin, c.dist );
               if( c.dist > contactThreshold ) {
                  ++positive;
                  worstPositive = std::max( worstPositive, c.dist );
               }
            }

            pe::detection::fine::GJK gjk;
            Vec3 n, p;
            const real gap( gjk.doGJK( static_cast<BoxID>( a ), static_cast<BoxID>( b ), n, p ) );
            const real depth( std::max( cornerDepth( a, b ), cornerDepth( b, a ) ) );
            if( gap <= real(0) ) ++overlapping;
            if( !log.entries.empty() && gap > real(1e-6) ) ++apartWithContact;
            if( log.entries.empty() && gap <= real(0) && depth > real(1e-6) ) ++missed;
            if( !log.entries.empty() && depth > real(1e-6) && dmin > -depth + real(1e-9) ) {
               ++shallow;
               worstShallow = std::max( worstShallow, dmin + depth );
            }
         }
      }

      std::printf( "near-parallel search: %ld configurations (%ld overlapping): %d positive-distance contacts (worst %+.3e), "
                   "%d contacts for separated boxes, %d missed overlaps, %d too shallow (worst %.3e)\n",
                   tested, overlapping, positive, static_cast<double>( worstPositive ), apartWithContact, missed,
                   shallow, static_cast<double>( worstShallow ) );
      expect( overlapping > 10000, "near-parallel search: enough overlapping configurations to be meaningful" );
      expect( positive == 0, "near-parallel search: no contact with dist > contactThreshold" );
      expect( apartWithContact == 0, "near-parallel search: no contact for boxes more than 1e-6 apart" );
      expect( missed == 0, "near-parallel search: every overlap yields contacts" );
      expect( shallow == 0, "near-parallel search: deepest contact at least as deep as the deepest corner" );
   }

   // 2. Clean edge-versus-face approach.
   {
      bool apartOk = true, contactOk = true;
      for( const double tilt : { 0.0, 1e-15, 1e-9, 1e-5, 1e-2 } ) {
         for( const double gap : { 0.3, 0.01, -0.001, -0.01, -0.05 } ) {
            world->clear();
            BoxID a = createBox( 1, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
            BoxID b = createBox( 2, Vec3( 0, 0, 0 ), Vec3( 1, 1, 1 ), mat );
            b->setOrientation( Quat( tilt, real(0), pi / 4 ) );
            const real lead( b->support( Vec3( 1, 0, 0 ) )[0] );
            b->setPosition( Vec3( real(-0.5) - lead - gap, real(0.1), real(0) ) );

            ContactLog log;
            MaxContacts::collide( a, b, log );
            if( gap > 0.0 ) {
               apartOk = apartOk && log.entries.empty();
               continue;
            }
            real dmin( std::numeric_limits<real>::max() );
            bool ok = !log.entries.empty();
            for( const ContactLog::Entry& c : log.entries ) {
               dmin = std::min( dmin, c.dist );
               ok = ok && !c.edgeEdge && std::fabs( std::fabs( c.normal[0] ) - real(1) ) < real(1e-6);
            }
            contactOk = contactOk && ok && std::fabs( dmin - gap ) < 1e-9;   // gap < 0: the contact distance
         }
      }
      expect( apartOk,   "edge-versus-face: no contact while apart" );
      expect( contactOk, "edge-versus-face: vertex/face contacts along the face normal at the penetration depth" );
   }

   if( failures == 0 ) {
      std::printf( "pe_box_box_parallel_edges_test: all checks passed\n" );
      return EXIT_SUCCESS;
   }
   std::printf( "pe_box_box_parallel_edges_test: %d check(s) FAILED\n", failures );
   return EXIT_FAILURE;
}
