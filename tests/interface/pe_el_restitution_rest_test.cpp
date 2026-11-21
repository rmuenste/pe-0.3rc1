//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_rest_test.cpp
 *  \brief Restitution at the scale of the case-11 bed: a defluidised pile comes to rest with no
 *         delivered rebound, impacts in the fluidised range keep their rebound
 *         (HardContactEulerLagrange, restitutionThreshold_ = v_rest = 0.01 m/s, default stop rule).
 *
 *  0.87 mm spheres (rho 860, e = 0.9, friction 0.15 per material), 10 PE sub-steps of 2.5e-5 s
 *  per fluid step, the EL hydro state re-armed per fluid step (drag B = m / tau, tau = 0.01 s,
 *  carrier velocity u_f upward, weight as the other force).
 *
 *    1. a pile of 48 spheres (4 x 4 x 3 on a jittered lattice) in a box of four walls on the floor
 *       with the drag carrying f = 20 %, 50 % and 90 % of the weight (u_f = f g tau): after 0.8 s
 *       of settling, a late window of 0.1 s has ZERO delivered rebounds and no candidate, the
 *       spheres are slower than 1e-3 m/s; the largest pre-step relative normal approach speed at a
 *       contact (both windows) is printed for the calibration of v_rest;
 *    2. single spheres hitting the floor at 0.05, 0.1 and 0.2 m/s under the same drag (f = 50 %):
 *       one rebound delivered, rebound ratio above 0.85 (e = 0.9 less the drag of the sub-step);
 *    3. a sphere dropped at 0.1 m/s onto the settled pile (f = 50 %): at least one rebound
 *       delivered.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/support/el_optional_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>

using namespace pe;

typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

static int failures = 0;
static void expect( bool ok, const std::string& what )
{
   if( !ok ) { std::printf( "FAIL: %s\n", what.c_str() ); ++failures; }
}

static const real kR( 0.435e-3 ), kRho( 860 ), kG( 9.81 ), kTau( 0.01 ), kDt( 2.5e-5 ), kVRest( 0.01 );
static const int kSub( 10 );

struct Window { size_t delivered = 0, candidates = 0; real approach = 0, resting = 0, vmax = 0; };

template< typename Handle > void configure( Handle& cs )
{
   cs->setSplitImpulse( false );
   cs->setRestitution( true );
   cs->setRestitutionPostPass( true );
   cs->setRestitutionThreshold( kVRest );
   cs->setRestitutionIterations( 4 );
   cs->setRestitutionMaxIterations( 24 );
   cs->setRestitutionCheckInterval( 4 );
   cs->setRestitutionTolerance( real(1e-4), real(1e-3) );
}

//! One fluid step: re-arm the hydro state and the free-flight reference, 10 sub-steps; adds the
//! restitution statistics to w.
template< typename Handle > void fluidStep( Handle& cs, const std::vector<BodyID>& bodies, real f, Window& w )
{
   for( BodyID b : bodies ) {
      elSetHydroState( b, b->getMass() / kTau, Vec3( 0, 0, f * kG * kTau ), Vec3( 0, 0, -b->getMass() * kG ), true );
      elSetRefVelocity( b, b->getLinearVel() );
   }
   for( int k = 0; k < kSub; ++k ) {
      theWorld()->simulationStep( kDt );
      const RestitutionPassStats& st( cs->getRestitutionPassStats() );
      w.delivered += st.delivered;
      w.candidates += st.candidates;
      w.approach = std::max( w.approach, st.approachMax );
      w.resting = std::max( w.resting, st.restingApproachMax );
   }
   for( BodyID b : bodies ) w.vmax = std::max( w.vmax, b->getLinearVel().length() );
}

static MaterialID material()
{
   return createMaterial( kRho, real(0.9), real(0.15), real(0.15), real(0.25), real(300), real(1e5), real(10), real(10) );
}

static std::vector<BodyID> buildPile( MaterialID m )
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const real half( real(4.2) * kR );
   createPlane( 1,  0,  0, 1, 0, m );
   createPlane( 2,  1,  0, 0, -half, m );
   createPlane( 3, -1,  0, 0, -half, m );
   createPlane( 4,  0,  1, 0, -half, m );
   createPlane( 5,  0, -1, 0, -half, m );
   std::vector<BodyID> bodies;
   unsigned int seed( 4321 );
   const auto jitter = [&seed]() { seed = seed * 1103515245u + 12345u; return ( ( seed >> 8 ) % 2001 ) / real(1000) - real(1); };
   pe::id_t id( 10 );
   for( int iz = 0; iz < 3; ++iz )
      for( int iy = 0; iy < 4; ++iy )
         for( int ix = 0; ix < 4; ++ix )
            bodies.push_back( createSphere( id++, Vec3( ( ix - real(1.5) ) * real(2.04) * kR + real(0.03) * kR * jitter(),
                                                        ( iy - real(1.5) ) * real(2.04) * kR + real(0.03) * kR * jitter(),
                                                        kR + iz * real(2.04) * kR + real(0.05) * kR ), kR, m ) );
   return bodies;
}

template< typename Handle > void run( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasElHydroState<RigidBody>::value && std::is_class<CS>::value ) {
      configure( cs );
      const MaterialID m( material() );

      // 1. defluidised pile
      std::vector<BodyID> settled;
      for( real f : { real(0.2), real(0.5), real(0.9) } ) {
         std::vector<BodyID> bodies( buildPile( m ) );
         Window settle, late;
         for( int s = 0; s < 3200; ++s ) fluidStep( cs, bodies, f, settle );
         std::vector<Vec3> x0;
         for( BodyID b : bodies ) x0.push_back( b->getPosition() );
         for( int s = 0; s < 400; ++s ) fluidStep( cs, bodies, f, late );
         real drift( 0 ), vEnd( 0 );
         for( size_t k = 0; k < bodies.size(); ++k ) {
            drift = std::max( drift, ( bodies[k]->getPosition() - x0[k] ).length() );
            vEnd = std::max( vEnd, bodies[k]->getLinearVel().length() );
         }
         std::printf( "pile, drag carries %.0f %% of the weight: settling 0.8 s: delivered %zu, candidates %zu, largest contact approach %.3e "
                      "(armed) / %.3e (below v_rest) m/s, max |v| %.3e | late 0.1 s: delivered %zu, candidates %zu, approach %.3e / %.3e m/s, "
                      "max |v| %.3e, drift %.3e m, |v| at the end %.3e\n",
                      static_cast<double>( 100 * f ), settle.delivered, settle.candidates, static_cast<double>( settle.approach ),
                      static_cast<double>( settle.resting ), static_cast<double>( settle.vmax ), late.delivered, late.candidates,
                      static_cast<double>( late.approach ), static_cast<double>( late.resting ), static_cast<double>( late.vmax ),
                      static_cast<double>( drift ), static_cast<double>( vEnd ) );
         expect( late.delivered == 0 && late.candidates == 0, "pile: no rebound (no candidate) in the late window" );
         expect( late.vmax < real(1e-3), "pile: sustained rest (every sphere slower than 1e-3 m/s in the late window)" );
         if( f == real(0.5) ) {
            // 3. a sphere dropped onto the settled pile
            real top( 0 );
            for( BodyID b : bodies ) top = std::max( top, b->getPosition()[2] );
            SphereID ball = createSphere( 99, Vec3( 0, 0, top + real(2) * kR + real(2e-5) ), kR, m );
            ball->setLinearVel( Vec3( 0, 0, real(-0.1) ) );
            bodies.push_back( ball );
            Window hit;
            for( int s = 0; s < 20; ++s ) fluidStep( cs, bodies, f, hit );
            std::printf( "sphere dropped at 0.1 m/s onto the pile: delivered %zu, largest approach %.3e m/s\n",
                         hit.delivered, static_cast<double>( hit.approach ) );
            expect( hit.delivered >= 1, "pile: an impact at 0.1 m/s onto the pile delivers a rebound" );
         }
      }

      // 2. impacts on the floor in the fluidised range
      for( real v : { real(0.05), real(0.1), real(0.2) } ) {
         WorldID world = theWorld();
         world->clear();
         createPlane( 1, 0, 0, 1, 0, m );
         SphereID s = createSphere( 2, Vec3( 0, 0, kR + v * kDt * real(3.9) ), kR, m );
         s->setLinearVel( Vec3( 0, 0, -v ) );
         std::vector<BodyID> one( 1, BodyID( s ) );
         Window w;
         real vPre( 0 ), vPost( 0 );
         for( int fs = 0; fs < 3 && vPost == 0; ++fs ) {
            elSetHydroState( one[0], s->getMass() / kTau, Vec3( 0, 0, real(0.5) * kG * kTau ), Vec3( 0, 0, -s->getMass() * kG ), true );
            elSetRefVelocity( one[0], s->getLinearVel() );
            for( int k = 0; k < kSub && vPost == 0; ++k ) {
               const real before( s->getLinearVel()[2] );
               world->simulationStep( kDt );
               w.delivered += cs->getRestitutionPassStats().delivered;
               if( before < 0 && s->getLinearVel()[2] > 0 ) { vPre = before; vPost = s->getLinearVel()[2]; }
            }
         }
         std::printf( "floor impact at %.2f m/s under drag: delivered %zu, rebound %.6f m/s, ratio %.4f\n",
                      static_cast<double>( v ), w.delivered, static_cast<double>( vPost ), static_cast<double>( -vPost / vPre ) );
         expect( w.delivered == 1 && -vPost / vPre > real(0.85), "floor impact in the fluidised range keeps its rebound" );
      }
      cs->setRestitution( false );
   }
   else { (void)cs; }
}

int main()
{
   if( !HasElHydroState<RigidBody>::value ) {
      std::printf( "pe_el_restitution_rest_test: the configured collision system is not HardContactEulerLagrange, skipped\n" );
      return 77;
   }
   CollisionSystemID cs = theCollisionSystem();
   run( cs );
   if( failures == 0 ) std::printf( "pe_el_restitution_rest_test: all checks passed\n" );
   else std::printf( "pe_el_restitution_rest_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
