//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_test.cpp
 *  \brief Restitution of HardContactEulerLagrange (setRestitution(), port of the HCSITS switch).
 *
 *  The Euler-Lagrange solver leaves body forces to the driver, so gravity is either a force per
 *  step (addForce) or the "other force" of the EL hydro state. Asserted (serial, no MPI):
 *
 *    0. defaults: off, capped post-pass, four sweeps, threshold 0;
 *    1. two spheres head-on, e = 0.9 / 0.5 (Baumgarte, the EL default, and split impulse) and
 *       e = 0 (split impulse), equal and unequal masses: relative velocity after the impact
 *       -e times before (Newton), momentum conserved, kinetic energy of the relative motion
 *       e^2 times before, each to 1e-12 relative; e = 0 with the switch on and Baumgarte gives
 *       bit for bit the result with the switch off; the in-solve scheme gives the same single
 *       impact;
 *    2. a sphere dropped on the floor plane (gravity as a force, e = 0.9): the apex of each of the
 *       first three rebounds is e^2 times the previous one within 2 %, the energy at the apexes
 *       decreases and never exceeds the start value, and the sphere comes to rest (|v| < 2 g dt,
 *       bottom within 1e-6 of the plane after 4 s);
 *    3. the same with the EL drag active (hydro state with drag, carrier velocity 0, gravity as
 *       the other force, 10 sub-steps per armed step as in FeatFloWer, drag decelerating the
 *       approach), Baumgarte and split impulse: at the impact sub-step the rebound is
 *       min( e |v_pre|, e |v_free| + (1 + e) p ) to 1e-9 (Newton target on the pre-step velocity,
 *       Poisson ceiling on the free-flight velocity, p the Baumgarte push-out, 0 with split);
 *    4. oblique impact with friction (mu = 0.3 per pair from 0.15 per material, e = 0.9): sliding
 *       case v = (3, 0, -1): normal impulse (1 + e) m |v_z|, tangential impulse mu times the full
 *       normal impulse (1e-9); sticking case v = (0.2, 0, -1): the contact point stops sliding and
 *       the tangential impulse stays below mu times the normal impulse;
 *    5. a resting pile (square pyramid 2 x 2 + 1 in a box of four walls touching the base, three
 *       more spheres stacked on the apex), gravity as a force, 5000 steps of 1e-4 s with the
 *       switch on (e = 0.9): every sphere stays slower than the arming threshold 2 g dt and moves
 *       less than 1e-6; with the switch off the run is bit
 *       for bit the same before and after an "on" run (hash printed for the comparison with the
 *       solver before the switch);
 *    6. simultaneous contacts: a sphere into a touching pair (three-sphere chain) and a sphere into
 *       a corner of two planes: no kinetic energy is created, momentum (chain) is conserved, the
 *       corner rebound is e times the incoming velocity in both components.
 *
 *    7. a disordered bed (36 spheres settled in a box of four walls) under active EL drag with step
 *       changes of the carrier forcing between fluid steps: drag cancelling 98 % of the weight,
 *       released, cancelled again (40 fluid steps x 10 sub-steps each), Baumgarte and split: the
 *       contacts create no kinetic energy beyond the work of the hydro forces and the bed moves
 *       less than 1 % of r; the number of contacts that arm (micro-bounce) is printed, not
 *       asserted;
 *    8. a finite-mass global body is refused while the switch is on (std::runtime_error), also
 *       one unfixed after its registration; a global body fixed and destroyed, or removed by
 *       World::clear(), does not refuse (or crash) the next step (the refusal reads the live
 *       body storage).
 *
 *  Under a configured solver other than HardContactEulerLagrange the test returns 77 (skipped).
 *  Against the EL solver without the switch (origin/master 6003b00) it fails: cases 0-4 and 6
 *  see inelastic contacts; against 1d030e7 (before review round 1) case 8 fails.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/support/el_optional_api.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using namespace pe;

typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitution( true ) ),
                                                              decltype( std::declval<T&>().setRestitutionPostPass( true ) ),
                                                              decltype( std::declval<T&>().setRestitutionIterations( size_t(4) ) ),
                                                              decltype( std::declval<T&>().setRestitutionThreshold( real(0) ) ) > > : std::true_type {};

static const bool kHasSwitch = HasRestitution<CollisionSystemType>::value;

//! Switches the restitution of the configured collision system (no-op without the switch).
template< typename T, typename = void > struct HasStopRule : std::false_type {};
template< typename T > struct HasStopRule< T, std::void_t< decltype( std::declval<T&>().setRestitutionCheckInterval( size_t(1) ) ) > > : std::true_type {};

template< typename Handle > void applySwitches( Handle& cs, bool restitution, bool split, bool postPass = true )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   cs->setSplitImpulse( split );
   if constexpr ( HasRestitution<CS>::value ) {
      cs->setRestitution( restitution );
      cs->setRestitutionPostPass( postPass );
      cs->setRestitutionIterations( 4 );
      cs->setRestitutionThreshold( real(0) );
      if constexpr ( HasStopRule<CS>::value ) {   // the regression setting of the stop rule
         cs->setRestitutionMaxIterations( 100 );
         cs->setRestitutionCheckInterval( 1 );
         cs->setRestitutionTolerance( real(0), real(1e-9) );
      }
   }
   else { (void)restitution; (void)postPass; }
}

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static bool relClose( real a, real b, real tol )
{
   return std::fabs( a - b ) <= tol * std::max( std::fabs( b ), real(1e-300) );
}

static const real kR( 0.01 );
static const real kG( 9.81 );

//! density, cor, csf (per material; the pair value is the sum), cdf, poisson, young, stiffness, dampingN, dampingT
static MaterialID material( real density, real cor, real mu )
{
   return createMaterial( density, cor, mu, mu, real(0.25), real(300), real(1e5), real(10), real(10) );
}

static real kinetic( BodyID b )
{
   const Vec3& v = b->getLinearVel();
   const Vec3& w = b->getAngularVel();
   return real(0.5) * b->getMass() * ( trans( v ) * v ) + real(0.5) * ( trans( w ) * ( b->getInertia() * w ) );
}

//! FNV-1a over the bits of positions and velocities (bitwise comparison of two runs).
static std::uint64_t stateHash( const std::vector<BodyID>& bodies )
{
   std::uint64_t h( 1469598103934665603ULL );
   for( BodyID b : bodies ) {
      const Vec3 vals[3] = { b->getPosition(), b->getLinearVel(), b->getAngularVel() };
      for( const Vec3& v : vals )
         for( size_t k = 0; k < 3; ++k ) {
            const double d( v[k] );
            unsigned char bytes[sizeof( double )];
            std::memcpy( bytes, &d, sizeof( double ) );
            for( unsigned char c : bytes ) { h ^= c; h *= 1099511628211ULL; }
         }
   }
   return h;
}

//=================================================================================================
// 1. Head-on impact of two spheres
//=================================================================================================

struct HeadOn { Vec3 va, vb; real ma, mb; real e; };

//! Two spheres on the x axis approach at +u / -u; the gap closes after 5.9 steps, so the impact
//! step starts with an overlap of 0.1 of a step's approach. Runs until the spheres are apart.
static HeadOn headOn( CollisionSystemID cs, real cor, real rhoA, real rhoB, bool restitution, bool split, bool postPass = true )
{
   applySwitches( cs, restitution, split, postPass );
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const real u( 0.5 ), dt( 1e-4 );
   const real gap( real(2) * u * dt * real(5.9) );
   const MaterialID ma( material( rhoA, cor, real(0) ) ), mb( material( rhoB, cor, real(0) ) );
   SphereID a = createSphere( 1, Vec3( -( kR + real(0.5) * gap ), 0, 0 ), kR, ma );
   SphereID b = createSphere( 2, Vec3(  ( kR + real(0.5) * gap ), 0, 0 ), kR, mb );
   a->setLinearVel( Vec3(  u, 0, 0 ) );
   b->setLinearVel( Vec3( -u, 0, 0 ) );
   for( int i = 0; i < 40; ++i )
      world->simulationStep( dt );
   HeadOn r;
   r.va = a->getLinearVel(); r.vb = b->getLinearVel();
   r.ma = a->getMass(); r.mb = b->getMass();
   r.e = Material::getRestitution( ma, mb );
   return r;
}

static void caseHeadOn( CollisionSystemID cs )
{
   const real u( 0.5 );
   char what[256];
   struct Variant { real cor, rhoA, rhoB; bool split; };
   const Variant variants[] = {
      { real(0.9), real(1000), real(1000), false }, { real(0.5), real(1000), real(1000), false },
      { real(0.9), real(1000), real(1000), true  }, { real(0.5), real(1000), real(1000), true  },
      { real(0.0), real(1000), real(1000), true  },
      { real(0.9), real(1000), real(3000), false }, { real(0.5), real(2500), real(800),  true  } };
   for( const Variant& var : variants ) {
      const HeadOn r( headOn( cs, var.cor, var.rhoA, var.rhoB, true, var.split ) );
      const real vrelBefore( -u - u );                  // (v_b - v_a)_x before
      const real vrelAfter( r.vb[0] - r.va[0] );
      const real pBefore( r.ma * u - r.mb * u ), pAfter( r.ma * r.va[0] + r.mb * r.vb[0] );
      const real mu( r.ma * r.mb / ( r.ma + r.mb ) );
      const real eRelBefore( real(0.5) * mu * vrelBefore * vrelBefore ), eRelAfter( real(0.5) * mu * vrelAfter * vrelAfter );
      std::printf( "head-on e=%.2f m_b/m_a=%.2f %s: v_rel' / v_rel = %.15f (expected %.2f), p' - p = %.3e, E_rel'/E_rel = %.15f\n",
                   static_cast<double>( r.e ), static_cast<double>( r.mb / r.ma ), var.split ? "split    " : "Baumgarte",
                   static_cast<double>( vrelAfter / vrelBefore ), static_cast<double>( -r.e ),
                   static_cast<double>( pAfter - pBefore ), static_cast<double>( eRelAfter / eRelBefore ) );
      std::snprintf( what, sizeof( what ), "head-on e=%.2f m_b/m_a=%.2f %s: Newton law v_rel' = -e v_rel",
                     static_cast<double>( r.e ), static_cast<double>( r.mb / r.ma ), var.split ? "split" : "Baumgarte" );
      if( r.e > real(0) ) expect( relClose( vrelAfter, -r.e * vrelBefore, real(1e-12) ), what );
      else                expect( std::fabs( vrelAfter ) < real(1e-12) * std::fabs( vrelBefore ), what );
      std::snprintf( what, sizeof( what ), "head-on e=%.2f m_b/m_a=%.2f: momentum conserved",
                     static_cast<double>( r.e ), static_cast<double>( r.mb / r.ma ) );
      expect( std::fabs( pAfter - pBefore ) <= real(1e-12) * r.ma * u, what );
      std::snprintf( what, sizeof( what ), "head-on e=%.2f m_b/m_a=%.2f: relative kinetic energy ratio e^2",
                     static_cast<double>( r.e ), static_cast<double>( r.mb / r.ma ) );
      expect( std::fabs( eRelAfter / eRelBefore - r.e * r.e ) <= real(1e-12), what );
      expect( std::fabs( r.va[1] ) + std::fabs( r.va[2] ) + std::fabs( r.vb[1] ) + std::fabs( r.vb[2] ) == real(0),
              "head-on: the impact stays on the axis" );
   }

   // e = 0 with Baumgarte: the switch on is the switch off, bit for bit (the Baumgarte push-out
   // of the one-step overlap is the same artificial separation velocity in both).
   {
      const HeadOn on( headOn( cs, real(0), real(1000), real(1000), true, false ) );
      const HeadOn off( headOn( cs, real(0), real(1000), real(1000), false, false ) );
      std::printf( "head-on e=0 Baumgarte: v_a on %.17g off %.17g\n", static_cast<double>( on.va[0] ), static_cast<double>( off.va[0] ) );
      expect( on.va[0] == off.va[0] && on.vb[0] == off.vb[0] && on.va[1] == off.va[1] && on.vb[2] == off.vb[2],
              "head-on e=0: switch on gives bit for bit the inelastic result" );
   }

   // The in-solve Newton target gives the same single impact.
   {
      const HeadOn r( headOn( cs, real(0.9), real(1000), real(3000), true, false, false ) );
      const real vrelAfter( r.vb[0] - r.va[0] );
      std::printf( "head-on e=0.9 in-solve: v_rel' / v_rel = %.15f\n", static_cast<double>( vrelAfter / ( -u - u ) ) );
      expect( relClose( vrelAfter, -r.e * ( -u - u ), real(1e-9) ), "head-on in-solve scheme: Newton law" );
   }
}

//=================================================================================================
// 2. Drop on the floor, gravity as a force
//=================================================================================================

static void caseDrop( CollisionSystemID cs )
{
   applySwitches( cs, true, false );
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const real e( 0.9 ), h0( 0.1 ), dt( 1e-4 );
   const MaterialID m( material( real(1000), e, real(0.15) ) );
   createPlane( 1, 0, 0, 1, 0, m );
   SphereID s = createSphere( 2, Vec3( 0, 0, kR + h0 ), kR, m );
   const real mass( s->getMass() );
   const real E0( mass * kG * h0 );

   std::vector<real> apex;
   real Emax( 0 ), top( -1 );
   bool rising( false );
   for( int i = 0; i < 40000; ++i ) {
      s->addForce( Vec3( 0, 0, -mass * kG ) );
      const real vzBefore( s->getLinearVel()[2] );
      world->simulationStep( dt );
      const real vz( s->getLinearVel()[2] );
      const real z( s->getPosition()[2] - kR );
      Emax = std::max( Emax, kinetic( s ) + mass * kG * z );
      if( vzBefore < 0 && vz > 0 ) { rising = true; top = z; }
      if( rising ) {
         top = std::max( top, z );
         if( vz <= 0 ) { apex.push_back( top ); rising = false; }
      }
   }
   const real vEnd( s->getLinearVel().length() ), zEnd( s->getPosition()[2] - kR );
   std::printf( "drop e=0.9: %zu rebounds, apexes", apex.size() );
   for( size_t k = 0; k < std::min( apex.size(), size_t(4) ); ++k ) std::printf( " %.5f", static_cast<double>( apex[k] ) );
   std::printf( " m; E_max / E_0 = %.6f; at 4 s |v| = %.3e, bottom %.3e\n",
                static_cast<double>( Emax / E0 ), static_cast<double>( vEnd ), static_cast<double>( zEnd ) );
   expect( apex.size() >= 3, "drop: at least three rebounds" );
   if( apex.size() >= 3 ) {
      real prev( h0 );
      for( size_t k = 0; k < 3; ++k ) {
         std::printf( "   rebound %zu: apex ratio %.4f (e^2 = %.4f)\n", k + 1, static_cast<double>( apex[k] / prev ), static_cast<double>( e * e ) );
         expect( relClose( apex[k] / prev, e * e, real(0.02) ), "drop: apex ratio e^2 within 2 %" );
         prev = apex[k];
      }
   }
   for( size_t k = 1; k < apex.size(); ++k )
      if( !( apex[k] < apex[k-1] ) ) { expect( false, "drop: the apexes decrease" ); break; }
   expect( Emax <= E0 * real(1.0001), "drop: the energy never exceeds the start value" );
   expect( vEnd < real(2) * kG * dt, "drop: the sphere comes to rest (|v| below the arming threshold 2 g dt)" );
   expect( std::fabs( zEnd ) < real(1e-6), "drop: the sphere rests on the plane" );
}

//=================================================================================================
// 3. Drop with the EL drag active
//=================================================================================================

//! Free-flight reference velocity of the EL drag sub-cycling (zero vector without the EL trait).
template< typename B > Vec3 refVelocity( B body )
{
   if constexpr ( HasElHydroStateQuery< typename std::remove_pointer<B>::type >::value ) return body->getEulerLagrangeRefVelocity();
   else { (void)body; return Vec3(); }
}

static void caseDropWithDrag( CollisionSystemID cs )
{
   const real e( 0.9 ), dtSub( 1e-4 ), erp( 0.7 );
   const int subSteps( 10 );
   for( int split = 0; split < 2; ++split ) {
      applySwitches( cs, true, split == 1 );
      cs->setErrorReductionParameter( erp );
      WorldID world = theWorld();
      world->clear();
      world->setGravity( 0, 0, 0 );
      const MaterialID m( material( real(2500), e, real(0.15) ) );
      createPlane( 1, 0, 0, 1, 0, m );
      SphereID s = createSphere( 2, Vec3( 0, 0, kR + real(0.02) ), kR, m );
      BodyID body( s );
      const real mass( s->getMass() );
      const real tau( 20 * subSteps * dtSub );           // particle relaxation time m / B = 20 armed steps
      const real B( mass / tau );
      const Vec3 weight( 0, 0, -mass * kG );             // submerged weight as the "other force"
      s->setLinearVel( Vec3( 0, 0, real(-1.5) ) );        // faster than terminal (g tau = 0.196): the drag decelerates

      bool found( false );
      real vPre( 0 ), vPost( 0 ), vFree( 0 ), depth( 0 );
      for( int step = 0; step < 200 && !found; ++step ) {
         // armed once per fluid step, as the FeatFloWer driver does (10 PE sub-steps)
         elSetHydroState( body, B, Vec3(), weight, true );
         elSetRefVelocity( body, s->getLinearVel() );
         for( int k = 0; k < subSteps && !found; ++k ) {
            const real before( s->getLinearVel()[2] );
            const real uref( refVelocity( body )[2] );
            const real d( std::max( kR - s->getPosition()[2], real(0) ) );
            world->simulationStep( dtSub );
            const real after( s->getLinearVel()[2] );
            if( before < 0 && after > 0 ) {
               found = true; vPre = before; vPost = after; depth = d;
               vFree = before + ( refVelocity( body )[2] - uref );   // velocity the sub-step gives without the contact
            }
         }
      }
      // Single impact: Newton target e |v_pre|, Poisson ceiling e lambda_c with
      // lambda_c = m ( |v_free| + p ), p = erp depth / dt the Baumgarte push-out (0 with split impulse).
      const real p( split ? real(0) : erp * depth / dtSub );
      const real expected( std::min( e * std::fabs( vPre ), e * std::fabs( vFree ) + ( 1 + e ) * p ) );
      std::printf( "drop with EL drag (%s): impact sub-step v_n before %.6f, free %.6f, after %.9f; expected min( e|v_pre|, e|v_free| + (1+e) p ) = %.9f (p = %.3e)\n",
                   split ? "split    " : "Baumgarte", static_cast<double>( vPre ), static_cast<double>( vFree ), static_cast<double>( vPost ),
                   static_cast<double>( expected ), static_cast<double>( p ) );
      expect( found, "drop with EL drag: the sphere bounces" );
      expect( std::fabs( vFree ) < std::fabs( vPre ), "drop with EL drag: the drag decelerates the approach in the impact sub-step" );
      expect( relClose( vPost, expected, real(1e-9) ), "drop with EL drag: impact law at the impact sub-step (Newton target, Poisson ceiling)" );
   }
   cs->setErrorReductionParameter( erp );
}

//=================================================================================================
// 4. Oblique impact with friction
//=================================================================================================

static void caseOblique( CollisionSystemID cs )
{
   const real e( 0.9 ), dt( 1e-4 );
   for( int sticking = 0; sticking < 2; ++sticking ) {
      applySwitches( cs, true, false );
      WorldID world = theWorld();
      world->clear();
      world->setGravity( 0, 0, 0 );
      const MaterialID m( material( real(1000), e, real(0.15) ) );   // pair: mu = 0.3
      createPlane( 1, 0, 0, 1, 0, m );
      const Vec3 v0( sticking ? real(0.2) : real(3), 0, -1 );
      SphereID s = createSphere( 2, Vec3( 0, 0, kR + real(0.95) * dt ), kR, m );   // touches after 0.95 steps
      s->setLinearVel( v0 );
      const real mu( Material::getDynamicFriction( m, m ) );
      real depth( 0 );   // overlap at the start of the impact step: the contact point lies on the plane, r - depth below the centre
      for( int i = 0; i < 20; ++i ) {
         const real vzBefore( s->getLinearVel()[2] ), d( kR - s->getPosition()[2] );
         world->simulationStep( dt );
         if( vzBefore < 0 && s->getLinearVel()[2] > 0 ) depth = d;
      }
      const Vec3 v( s->getLinearVel() ), w( s->getAngularVel() );
      const real mass( s->getMass() );
      const real pn( mass * ( v[2] - v0[2] ) ), pt( mass * std::fabs( v[0] - v0[0] ) );
      const real slip( v[0] - w[1] * ( kR - depth ) );   // velocity of the contact point along x: v + omega x ( -( r - depth ) e_z )
      std::printf( "oblique %s: v' = (%.6f, %.3e, %.6f), omega_y' = %.3f, p_t / p_n = %.12f (mu = %.2f), contact slip %.3e\n",
                   sticking ? "sticking" : "sliding ", static_cast<double>( v[0] ), static_cast<double>( v[1] ), static_cast<double>( v[2] ),
                   static_cast<double>( w[1] ), static_cast<double>( pt / pn ), static_cast<double>( mu ), static_cast<double>( slip ) );
      expect( relClose( pn, mass * ( real(1) + e ) * std::fabs( v0[2] ), real(1e-9) ), "oblique: normal impulse (1 + e) m |v_z|" );
      expect( pt <= mu * pn * ( real(1) + real(1e-9) ), "oblique: tangential impulse bounded by mu times the full normal impulse" );
      if( !sticking )
         expect( relClose( pt, mu * pn, real(1e-9) ), "oblique sliding: tangential impulse = mu times the full normal impulse" );
      else
         expect( std::fabs( slip ) < real(1e-9) && pt < mu * pn, "oblique sticking: the contact point stops sliding inside the cone" );
   }
}

//=================================================================================================
// 5. Resting pile
//=================================================================================================

//! A square pyramid (2 x 2 + 1) in a box of four walls that touch the base, with a column of
//! three more spheres on the apex; everything exactly touching, on the floor plane.
static std::vector<BodyID> buildPile( real e )
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const MaterialID m( material( real(1000), e, real(0.15) ) );
   createPlane( 1,  0,  0, 1, 0, m );
   createPlane( 2,  1,  0, 0, -real(2) * kR, m );   // x = -2r, normal +x
   createPlane( 3, -1,  0, 0, -real(2) * kR, m );   // x = +2r, normal -x
   createPlane( 4,  0,  1, 0, -real(2) * kR, m );
   createPlane( 5,  0, -1, 0, -real(2) * kR, m );
   std::vector<BodyID> bodies;
   pe::id_t id( 10 );
   for( int ix = -1; ix <= 1; ix += 2 )
      for( int iy = -1; iy <= 1; iy += 2 )
         bodies.push_back( createSphere( id++, Vec3( ix * kR, iy * kR, kR ), kR, m ) );
   const real apex( kR + std::sqrt( real(2) ) * kR );
   for( int k = 0; k < 4; ++k )
      bodies.push_back( createSphere( id++, Vec3( 0, 0, apex + real(2) * k * kR ), kR, m ) );
   return bodies;
}

struct PileResult { std::uint64_t hash; real vmax; real drift; };

static PileResult runPile( CollisionSystemID cs, bool restitution )
{
   applySwitches( cs, restitution, false );
   std::vector<BodyID> bodies( buildPile( real(0.9) ) );
   std::vector<Vec3> start;
   for( BodyID b : bodies ) start.push_back( b->getPosition() );
   const real dt( 1e-4 );
   PileResult r = { 0, 0, 0 };
   for( int i = 0; i < 5000; ++i ) {
      for( BodyID b : bodies ) b->addForce( Vec3( 0, 0, -b->getMass() * kG ) );
      theWorld()->simulationStep( dt );
      for( size_t k = 0; k < bodies.size(); ++k ) {
         r.vmax  = std::max( r.vmax, bodies[k]->getLinearVel().length() );
         r.drift = std::max( r.drift, ( bodies[k]->getPosition() - start[k] ).length() );
      }
   }
   r.hash = stateHash( bodies );
   return r;
}

static void casePile( CollisionSystemID cs )
{
   const real dt( 1e-4 );
   const PileResult off1( runPile( cs, false ) );
   const PileResult on( runPile( cs, true ) );
   const PileResult off2( runPile( cs, false ) );
   std::printf( "pile: switch on: max |v| %.3e (threshold 2 g dt = %.3e), max drift %.3e; switch off: max |v| %.3e, drift %.3e\n",
                static_cast<double>( on.vmax ), static_cast<double>( 2 * kG * dt ), static_cast<double>( on.drift ),
                static_cast<double>( off1.vmax ), static_cast<double>( off1.drift ) );
   std::printf( "pile: state hash switch off %016llx (before and after an on run: %016llx), switch on %016llx\n",
                static_cast<unsigned long long>( off1.hash ), static_cast<unsigned long long>( off2.hash ),
                static_cast<unsigned long long>( on.hash ) );
   expect( on.vmax < real(2) * kG * dt, "pile: with restitution on every sphere stays below the arming threshold 2 g dt" );
   expect( on.drift < real(1e-6), "pile: with restitution on the pile does not move (drift < 1e-6)" );
   expect( off1.hash == off2.hash, "pile: switch off is bit for bit the same before and after a run with the switch on" );
   expect( on.hash == off1.hash, "pile: at rest no contact is armed, the run with the switch on is bit for bit the run with it off" );
}

//=================================================================================================
// 6. Several simultaneous contacts
//=================================================================================================

static void caseSimultaneous( CollisionSystemID cs )
{
   const real e( 0.9 ), u( 0.5 ), dt( 1e-4 );
   // Three-sphere chain: a into the touching pair b-c.
   {
      applySwitches( cs, true, false );
      WorldID world = theWorld();
      world->clear();
      world->setGravity( 0, 0, 0 );
      const MaterialID m( material( real(1000), e, real(0) ) );
      const real gap( u * dt * real(5.9) );
      SphereID a = createSphere( 1, Vec3( -real(2) * kR - gap, 0, 0 ), kR, m );
      SphereID b = createSphere( 2, Vec3( 0, 0, 0 ), kR, m );
      SphereID c = createSphere( 3, Vec3( real(2) * kR, 0, 0 ), kR, m );
      a->setLinearVel( Vec3( u, 0, 0 ) );
      const real E0( kinetic( a ) ), p0( a->getMass() * u );
      real Emax( 0 );
      for( int i = 0; i < 60; ++i ) {
         world->simulationStep( dt );
         Emax = std::max( Emax, kinetic( a ) + kinetic( b ) + kinetic( c ) );
      }
      const real p1( a->getMass() * a->getLinearVel()[0] + b->getMass() * b->getLinearVel()[0] + c->getMass() * c->getLinearVel()[0] );
      std::printf( "chain: v = %.6f %.6f %.6f, E_max / E_0 = %.12f, p' - p = %.3e\n",
                   static_cast<double>( a->getLinearVel()[0] ), static_cast<double>( b->getLinearVel()[0] ), static_cast<double>( c->getLinearVel()[0] ),
                   static_cast<double>( Emax / E0 ), static_cast<double>( p1 - p0 ) );
      expect( Emax <= E0 * ( real(1) + real(1e-12) ), "chain: no kinetic energy is created" );
      expect( std::fabs( p1 - p0 ) <= real(1e-12) * p0, "chain: momentum conserved" );
      expect( a->getLinearVel()[0] < real(0.5) * u, "chain: the impact is resolved (a no longer moves at u)" );
   }
   // Corner of two planes: the sphere hits both at once.
   {
      applySwitches( cs, true, false );
      WorldID world = theWorld();
      world->clear();
      world->setGravity( 0, 0, 0 );
      const MaterialID m( material( real(1000), e, real(0) ) );
      createPlane( 1, 0, 0, 1, 0, m );
      createPlane( 2, 1, 0, 0, 0, m );
      const real gap( u * dt * real(5.9) );
      SphereID s = createSphere( 3, Vec3( kR + gap, 0, kR + gap ), kR, m );
      s->setLinearVel( Vec3( -u, 0, -u ) );
      const real E0( kinetic( s ) );
      real Emax( 0 );
      for( int i = 0; i < 40; ++i ) {
         world->simulationStep( dt );
         Emax = std::max( Emax, kinetic( s ) );
      }
      const Vec3 v( s->getLinearVel() );
      std::printf( "corner: v' = (%.12f, %.3e, %.12f) expected (%.2f, 0, %.2f), E_max / E_0 = %.12f\n",
                   static_cast<double>( v[0] ), static_cast<double>( v[1] ), static_cast<double>( v[2] ),
                   static_cast<double>( e * u ), static_cast<double>( e * u ), static_cast<double>( Emax / E0 ) );
      expect( Emax <= E0 * ( real(1) + real(1e-12) ), "corner: no kinetic energy is created" );
      expect( relClose( v[0], e * u, real(1e-9) ) && relClose( v[2], e * u, real(1e-9) ), "corner: both components rebound at e u" );
   }
}

//=================================================================================================
// 7. Resting pile under active EL drag with step changes of the carrier forcing
//=================================================================================================

template< typename T, typename = void > struct HasArmingCounters : std::false_type {};
template< typename T > struct HasArmingCounters< T, std::void_t< decltype( std::declval<const T&>().candidates ),
   decltype( std::declval<const T&>().armedRows ), decltype( std::declval<const T&>().delivered ) > > : std::true_type {};

//! Arming counters of the last step: candidates (pre-step approach faster than the threshold),
//! armed rows (candidates with compression impulse) and delivered rebounds (armed rows ending with a
//! booked restitution impulse). A solver with the switch must have them (compile error otherwise;
//! the round-1 counter `impacting` no longer exists). Zero without the switch.
struct ArmingCounts { size_t candidates = 0, armedRows = 0, delivered = 0; };
template< typename Handle > ArmingCounts armingCounts( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   ArmingCounts c;
   if constexpr ( HasRestitution<CS>::value ) {
      const auto& st( cs->getRestitutionPassStats() );
      typedef typename std::remove_cv<typename std::remove_reference<decltype( st )>::type>::type Stats;
      static_assert( HasArmingCounters<Stats>::value, "RestitutionPassStats lacks candidates / armedRows / delivered" );
      c.candidates = st.candidates; c.armedRows = st.armedRows; c.delivered = st.delivered;
   }
   else { (void)cs; }
   return c;
}

//! Armed visits of the post-pass in the last step (0 without the switch).
template< typename Handle > size_t armedVisits( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasRestitution<CS>::value ) return cs->getRestitutionPassStats().armed;
   else { (void)cs; return 0; }
}

struct DragPileResult { size_t armed[3]; size_t candidates[3]; size_t armedRows[3]; size_t delivered[3]; real created[3]; real drift[3]; real vmax[3]; };

struct SavedBody { Vec3 x, v, w; };

//! A disordered bed: 3 x 3 x 4 spheres on a jittered lattice (spacing 2.04 r, jitter 0.03 r) in a
//! box of four walls 6.4 r wide, dropped and settled under gravity (as a force, switch off, 24000
//! steps of 2.5e-5 s). The settled state is computed once and every run starts from it.
static std::vector<BodyID> buildDisorderedBed( CollisionSystemID cs, std::vector<SavedBody>& saved )
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const MaterialID m( material( real(1000), real(0.9), real(0.15) ) );
   const real half( real(3.2) * kR );
   createPlane( 1,  0,  0, 1, 0, m );
   createPlane( 2,  1,  0, 0, -half, m );
   createPlane( 3, -1,  0, 0, -half, m );
   createPlane( 4,  0,  1, 0, -half, m );
   createPlane( 5,  0, -1, 0, -half, m );
   std::vector<BodyID> bodies;
   pe::id_t id( 10 );
   if( saved.empty() ) {
      applySwitches( cs, false, false );
      unsigned int seed( 12345 );
      const auto jitter = [&seed]() { seed = seed * 1103515245u + 12345u; return ( ( seed >> 8 ) % 2001 ) / real(1000) - real(1); };
      for( int iz = 0; iz < 4; ++iz )
         for( int iy = -1; iy <= 1; ++iy )
            for( int ix = -1; ix <= 1; ++ix )
               bodies.push_back( createSphere( id++, Vec3( ix * real(2.04) * kR + real(0.03) * kR * jitter(),
                                                           iy * real(2.04) * kR + real(0.03) * kR * jitter(),
                                                           kR + iz * real(2.04) * kR + real(0.05) * kR ), kR, m ) );
      for( int i = 0; i < 24000; ++i ) {
         for( BodyID b : bodies ) b->addForce( Vec3( 0, 0, -b->getMass() * kG ) );
         world->simulationStep( real(2.5e-5) );
      }
      real vmax( 0 ), zmax( 0 );
      for( BodyID b : bodies ) { vmax = std::max( vmax, b->getLinearVel().length() ); zmax = std::max( zmax, b->getPosition()[2] ); }
      std::printf( "disordered bed settled (0.6 s): max |v| %.3e, top centre at %.3f r\n", static_cast<double>( vmax ), static_cast<double>( zmax / kR ) );
      for( BodyID b : bodies ) saved.push_back( SavedBody{ b->getPosition(), b->getLinearVel(), b->getAngularVel() } );
   }
   else {
      for( const SavedBody& sb : saved ) {
         SphereID sp = createSphere( id++, sb.x, kR, m );
         sp->setLinearVel( sb.v );
         sp->setAngularVel( sb.w );
         bodies.push_back( sp );
      }
   }
   return bodies;
}

//! The settled bed of buildDisorderedBed() with an EL hydro state on every sphere: weight as the
//! other force, drag B = m / tau with tau = 20 fluid steps, carrier velocity u_f upward. Phase 0:
//! drag cancels 98 % of the weight (u_f = 0.98 g tau); phase 1: released (u_f = 0); phase 2:
//! cancelled again. Each phase 40 fluid steps of 10 sub-steps of 2.5e-5 s (the FeatFloWer case-11
//! sub-step); the hydro state and the free-flight reference are re-armed per fluid step, as the
//! driver does. Per sub-step the work of the free (hydro) increment,
//! W = sum m dv_free . ( v_pre + dv_free / 2 ), is the exact kinetic-energy change without
//! contacts, so E_c = KE_after - KE_pre - W is what the contact solve did; the sum of its positive
//! parts is the energy the contacts created.
static DragPileResult runDragPile( CollisionSystemID cs, std::vector<SavedBody>& saved, bool restitution, bool split )
{
   std::vector<BodyID> bodies( buildDisorderedBed( cs, saved ) );
   applySwitches( cs, restitution, split );
   std::vector<Vec3> start;
   for( BodyID b : bodies ) start.push_back( b->getPosition() );
   const real dtSub( 2.5e-5 );
   const int subSteps( 10 ), fluidSteps( 40 );
   const real tau( 20 * subSteps * dtSub );
   DragPileResult r;
   for( int ph = 0; ph < 3; ++ph ) {
      r.armed[ph] = 0; r.candidates[ph] = 0; r.armedRows[ph] = 0; r.delivered[ph] = 0; r.created[ph] = 0; r.drift[ph] = 0; r.vmax[ph] = 0;
      const real uf( ph == 1 ? real(0) : real(0.98) * kG * tau );
      for( int fs = 0; fs < fluidSteps; ++fs ) {
         for( BodyID b : bodies ) {
            elSetHydroState( b, b->getMass() / tau, Vec3( 0, 0, uf ), Vec3( 0, 0, -b->getMass() * kG ), true );
            elSetRefVelocity( b, b->getLinearVel() );
         }
         for( int k = 0; k < subSteps; ++k ) {
            std::vector<Vec3> vPre, uref;
            real kePre( 0 );
            for( BodyID b : bodies ) { vPre.push_back( b->getLinearVel() ); uref.push_back( refVelocity( b ) ); kePre += kinetic( b ); }
            theWorld()->simulationStep( dtSub );
            r.armed[ph] += armedVisits( cs );
            const ArmingCounts ac( armingCounts( cs ) );
            r.candidates[ph] += ac.candidates; r.armedRows[ph] += ac.armedRows; r.delivered[ph] += ac.delivered;
            real keAfter( 0 ), work( 0 );
            for( size_t j = 0; j < bodies.size(); ++j ) {
               const Vec3 dvf( refVelocity( bodies[j] ) - uref[j] );
               work += bodies[j]->getMass() * ( trans( dvf ) * ( vPre[j] + real(0.5) * dvf ) );
               keAfter += kinetic( bodies[j] );
               r.drift[ph] = std::max( r.drift[ph], ( bodies[j]->getPosition() - start[j] ).length() );
               r.vmax[ph] = std::max( r.vmax[ph], bodies[j]->getLinearVel().length() );
            }
            r.created[ph] += std::max( keAfter - kePre - work, real(0) );
         }
      }
   }
   for( BodyID b : bodies ) elSetHydroState( b, real(0), Vec3(), Vec3(), false );
   return r;
}

static void caseDragPile( CollisionSystemID cs )
{
   const real mgr( real(1000) * real(4.0 / 3.0) * M_PI * kR * kR * kR * kG * kR );   // m g r of one sphere
   std::vector<SavedBody> saved;
   buildDisorderedBed( cs, saved );
   for( int split = 0; split < 2; ++split ) {
      const DragPileResult off( runDragPile( cs, saved, false, split == 1 ) );
      const DragPileResult on( runDragPile( cs, saved, true, split == 1 ) );
      for( int ph = 0; ph < 3; ++ph ) {
         const char* name[3] = { "drag cancels 98 % of the weight", "released", "cancelled again" };
         std::printf( "bed under EL drag (%s), phase %d (%s): switch on: candidates %zu, armed rows %zu, delivered rebounds %zu (armed visits %zu), "
                      "energy created by contacts %.3e m g r, max drift %.3e, max |v| %.3e | switch off: created %.3e m g r, drift %.3e, max |v| %.3e\n",
                      split ? "split" : "Baumgarte", ph, name[ph], on.candidates[ph], on.armedRows[ph], on.delivered[ph], on.armed[ph],
                      static_cast<double>( on.created[ph] / mgr ),
                      static_cast<double>( on.drift[ph] ), static_cast<double>( on.vmax[ph] ),
                      static_cast<double>( off.created[ph] / mgr ), static_cast<double>( off.drift[ph] ), static_cast<double>( off.vmax[ph] ) );
         // Asserted: the contacts create no kinetic energy beyond the work of the hydro forces
         // (switch on, either correction scheme), and the bed stays put (drift below 1 % of r).
         // NOT asserted: the numbers of candidates, armed rows and delivered rebounds. The current-step threshold
         // 2 |dv_free| lets contacts of a settling bed arm (micro-bounce); the counts are printed
         // and discussed in doc/technical-notes/el-restitution.md.
         expect( on.created[ph] <= off.created[ph] + real(1e-12) * mgr,
                 "bed under EL drag: the contacts create no energy beyond the external work (switch on)" );
         expect( on.drift[ph] < real(0.01) * kR, "bed under EL drag: the bed stays put (drift < 1 % of r)" );
      }
   }
}

//=================================================================================================
// 8. Refusal: a finite-mass global body
//=================================================================================================

static bool stepRefused( WorldID world, std::string& message )
{
   try { world->simulationStep( real(1e-4) ); }
   catch( const std::runtime_error& ex ) { message = ex.what(); return true; }
   return false;
}

static void caseGlobalMobileRefused( CollisionSystemID cs )
{
   WorldID world = theWorld();
   const MaterialID m( material( real(1000), real(0.9), real(0) ) );
   std::string message;

   // (a) a mobile global body: refused
   applySwitches( cs, true, false );
   world->clear();
   world->setGravity( 0, 0, 0 );
   pe_GLOBAL_SECTION { createSphere( 1, Vec3( 0, 0, 0 ), kR, m ); }
   const bool a( stepRefused( world, message ) );
   std::printf( "mobile global body with restitution on: %s%s\n", a ? "refused: " : "NOT refused", message.c_str() );
   expect( a && message.find( "global" ) != std::string::npos, "a finite-mass global body is refused while restitution is on" );

   // (b) registered mobile, then fixed and destroyed: the next step neither reads it nor refuses
   world->clear();
   SphereID g( nullptr );
   pe_GLOBAL_SECTION { g = createSphere( 2, Vec3( 0, 0, 0 ), kR, m ); }
   g->setFixed( true );
   destroy( g );
   createSphere( 3, Vec3( real(0.1), 0, 0 ), kR, m );
   message.clear();
   const bool b( stepRefused( world, message ) );
   std::printf( "global body fixed and destroyed, then a step: %s %s\n", b ? "refused:" : "not refused", message.c_str() );
   expect( !b, "a destroyed global body does not refuse (or crash) the next step" );

   // (c) clear and repopulate: a step with ordinary bodies is not refused
   world->clear();
   pe_GLOBAL_SECTION { createSphere( 4, Vec3( 0, 0, 0 ), kR, m ); }
   world->clear();
   createSphere( 5, Vec3( 0, 0, 0 ), kR, m );
   message.clear();
   const bool c( stepRefused( world, message ) );
   std::printf( "clear and repopulate, then a step: %s %s\n", c ? "refused:" : "not refused", message.c_str() );
   expect( !c, "after clear() a world without global bodies is not refused" );

   // (d) fixed, then unfixed again: refused (the live storage, not the registry, decides)
   world->clear();
   SphereID f( nullptr );
   pe_GLOBAL_SECTION { f = createSphere( 7, Vec3( 0, 0, 0 ), kR, m ); }
   f->setFixed( true );
   f->setFixed( false );
   message.clear();
   const bool d( stepRefused( world, message ) );
   std::printf( "global body unfixed after registration: %s%s\n", d ? "refused: " : "NOT refused", message.c_str() );
   expect( d, "a global body unfixed after its registration is refused" );

   applySwitches( cs, false, false );
   world->clear();
}

//! 0. Factory defaults of the switch (template, so that the body compiles without the switch).
template< typename Handle > void checkDefaults( Handle& cs )
{
   typedef typename std::remove_reference<decltype( *cs )>::type CS;
   if constexpr ( HasRestitution<CS>::value ) {
      std::printf( "defaults: restitution %s, post-pass %s, %zu sweeps, threshold %g\n",
                   cs->getRestitution() ? "on" : "off", cs->getRestitutionPostPass() ? "on" : "off",
                   cs->getRestitutionIterations(), static_cast<double>( cs->getRestitutionThreshold() ) );
      expect( !cs->getRestitution() && cs->getRestitutionPostPass() && cs->getRestitutionIterations() == 4 &&
              cs->getRestitutionThreshold() == real(0), "defaults: off, capped post-pass, four sweeps, threshold 0" );
   }
   else { (void)cs; }
}

int main()
{
   if( !HasElHydroStateQuery<RigidBody>::value ) {
      std::printf( "pe_el_restitution_test: the configured collision system is not HardContactEulerLagrange, skipped\n" );
      return 77;
   }
   if( !kHasSwitch ) {
      std::printf( "FAIL: HardContactEulerLagrange has no restitution switch (setRestitution); the cases below run inelastic\n" );
      ++failures;
   }
   CollisionSystemID cs = theCollisionSystem();

   // 0. Defaults.
   checkDefaults( cs );

   caseHeadOn( cs );
   caseDrop( cs );
   caseDropWithDrag( cs );
   caseOblique( cs );
   casePile( cs );
   caseSimultaneous( cs );
   caseDragPile( cs );
   caseGlobalMobileRefused( cs );

   applySwitches( cs, false, false );
   if( failures == 0 ) std::printf( "pe_el_restitution_test: all checks passed\n" );
   else std::printf( "pe_el_restitution_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
