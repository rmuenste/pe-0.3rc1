//=================================================================================================
/*!
 *  \file tests/interface/pe_contact_friction_coefficient_test.cpp
 *  \brief Selection of the static or the dynamic coefficient of friction per contact.
 *
 *  Contact::Contact() picks the coefficient of friction of a contact from the tangential relative
 *  speed at the contact point: below frictionThreshold the static coefficient of the material
 *  pair, above it the dynamic one. The tangential speed used to be computed as the normal
 *  component of the tangential relative velocity, which is zero, so every contact got the static
 *  coefficient and Material::setDynamicFriction() had no effect.
 *
 *  Materials with mu_s = 0.5 and mu_d = 0.1, so that the two are far apart, g = 9.81, dt = 1e-3:
 *    1. a box sliding on a horizontal plane at 1 m/s decelerates with mu_d g: after 0.2 s
 *       v = 1 - 0.1 * 9.81 * 0.2 = 0.804 m/s (with the static coefficient: 0.019 m/s);
 *    2. a box resting on an incline of slope tan(theta) = 0.3, between mu_d and mu_s, stays put
 *       (static friction holds; the dynamic coefficient alone could not);
 *    3. the same box pushed down that incline at 0.5 m/s accelerates with
 *       g ( sin(theta) - mu_d cos(theta) ) = 1.879 m/s^2: after 0.2 s v = 0.876 m/s (with the
 *       static coefficient it decelerates and would stop after 0.27 s);
 *    4. a box sliding on top of a fixed box (box/box contacts) decelerates like case 1.
 *  The incline is a tilted gravity vector on a horizontal plane.
 *
 *  Every case runs under two relaxation models of the semi-implicit solver. With
 *  InelasticCoulombContactByDecoupling the sliding speeds match the analytical values to four
 *  digits and are asserted within 2 %. The default model,
 *  ApproximateInelasticCoulombContactByDecoupling, relaxes the normal and the tangential
 *  components of a sliding contact one after the other and under-delivers the friction of a
 *  box on four corners by about 10 % (deceleration 0.090 g for mu_d = 0.1, the same with
 *  mu_s = mu_d, so unrelated to the selection); its speeds are asserted within 15 %, which still
 *  separates mu_d from mu_s by a wide margin. The resting box is asserted below 1e-3 m/s under
 *  both.
 *
 *  Serial world setup, no MPI. Linked against the semi-implicit solver variant.
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

using namespace pe;

typedef std::remove_reference<decltype( *theCollisionSystem() )>::type CollisionSystemType;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static const real kG( 9.81 );
static const real kMuS( 0.5 );
static const real kMuD( 0.1 );
static const real kDt( 1e-3 );
static const Vec3 kLengths( 0.2, 0.2, 0.1 );

static MaterialID material()
{
   // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
   MaterialID mat = createMaterial( real(1000), real(0), kMuS, kMuD, real(0.25), real(300), real(1e5), real(10), real(10) );
   // The pair table of a new material defaults to the sum of the two coefficients.
   Material::setStaticFriction( mat, mat, kMuS );
   Material::setDynamicFriction( mat, mat, kMuD );
   return mat;
}

//! A box on the plane z = 0 (or on a fixed box three times its size), gravity g, initial velocity v0.
static BoxID build( const Vec3& gravity, const Vec3& v0, bool onFixedBox )
{
   WorldID world = theWorld();
   world->clear();
   world->setGravity( gravity );
   MaterialID mat = material();
   createPlane( 1, 0, 0, 1, 0, mat );
   real base( 0 );
   if( onFixedBox ) {
      const Vec3 groundLengths( kLengths * real(3) );
      const real gap( 1e-3 );   // keep the fixed box off the (fixed) plane
      BoxID ground = createBox( 2, Vec3( 0, 0, gap + real(0.5) * groundLengths[2] ), groundLengths, mat );
      ground->setFixed( true );
      base = gap + groundLengths[2];
   }
   BoxID box = createBox( 3, Vec3( 0, 0, base + real(0.5) * kLengths[2] ), kLengths, mat );
   box->setLinearVel( v0 );
   return box;
}

static void run( int steps )
{
   for( int i = 0; i < steps; ++i ) theWorld()->simulationStep( kDt );
}

struct Model {
   CollisionSystemType::RelaxationModel model;
   const char* name;
   real tolerance;   // relative, on the sliding speeds
};

int main()
{
   char what[256];
   const int steps( 200 );
   const real t( steps * kDt );
   const real slope( 0.3 ), theta( std::atan( slope ) );
   const Vec3 gTilted( kG * std::sin( theta ), 0, -kG * std::cos( theta ) );   // downhill is +x

   const Model models[2] = {
      { CollisionSystemType::InelasticCoulombContactByDecoupling,            "InelasticCoulombContactByDecoupling           ", real(0.02) },
      { CollisionSystemType::ApproximateInelasticCoulombContactByDecoupling, "ApproximateInelasticCoulombContactByDecoupling", real(0.15) },
   };

   for( const Model& m : models ) {
      theCollisionSystem()->setRelaxationModel( m.model );
      std::printf( "relaxation model %s (tolerance %.0f %%)\n", m.name, static_cast<double>( 100 * m.tolerance ) );

      // 1. Sliding on a horizontal plane: deceleration mu_d g.
      {
         BoxID box = build( Vec3( 0, 0, -kG ), Vec3( 1, 0, 0 ), false );
         run( steps );
         const real v( box->getLinearVel()[0] ), expected( real(1) - kMuD * kG * t ), withStatic( real(1) - kMuS * kG * t );
         std::printf( "  plane:           v_x = %.4f m/s after %.2f s (mu_d: %.4f, mu_s: %.4f), mu_eff = %.4f, |omega| = %.1e\n",
                      static_cast<double>( v ), static_cast<double>( t ), static_cast<double>( expected ), static_cast<double>( withStatic ),
                      static_cast<double>( ( real(1) - v ) / ( kG * t ) ), static_cast<double>( box->getAngularVel().length() ) );
         std::snprintf( what, sizeof( what ), "%s plane: the sliding box decelerates with mu_d g (v_x = %.4f, expected %.4f)",
                        m.name, static_cast<double>( v ), static_cast<double>( expected ) );
         expect( std::fabs( v - expected ) < m.tolerance * expected, what );
         expect( box->getAngularVel().length() < real(0.05), "plane: the sliding box does not tip" );
      }

      // 2. Incline with mu_d < tan(theta) < mu_s, box at rest: static friction holds.
      {
         BoxID box = build( gTilted, Vec3(), false );
         real maxV( 0 );
         for( int i = 0; i < 500; ++i ) {
            theWorld()->simulationStep( kDt );
            maxV = std::max( maxV, box->getLinearVel().length() );
         }
         std::printf( "  incline at rest: max |v| = %.1e m/s over 0.5 s (mu_s = %.1f holds slope %.1f, mu_d = %.1f would not)\n",
                      static_cast<double>( maxV ), static_cast<double>( kMuS ), static_cast<double>( slope ), static_cast<double>( kMuD ) );
         std::snprintf( what, sizeof( what ), "%s incline: the resting box stays put under static friction", m.name );
         expect( maxV < real(1e-3), what );
      }

      // 3. The same incline, box pushed downhill: slides with mu_d.
      {
         BoxID box = build( gTilted, Vec3( real(0.5), 0, 0 ), false );
         run( steps );
         const real a( kG * ( std::sin( theta ) - kMuD * std::cos( theta ) ) );
         const real v( box->getLinearVel()[0] ), expected( real(0.5) + a * t );
         const real withStatic( real(0.5) + kG * ( std::sin( theta ) - kMuS * std::cos( theta ) ) * t );
         std::printf( "  incline sliding: v_x = %.4f m/s after %.2f s (mu_d: %.4f, mu_s: %.4f)\n",
                      static_cast<double>( v ), static_cast<double>( t ), static_cast<double>( expected ), static_cast<double>( withStatic ) );
         std::snprintf( what, sizeof( what ), "%s incline: the pushed box accelerates with mu_d (v_x = %.4f, expected %.4f)",
                        m.name, static_cast<double>( v ), static_cast<double>( expected ) );
         expect( std::fabs( v - expected ) < m.tolerance * expected, what );
      }

      // 4. Box sliding on a fixed box.
      {
         BoxID box = build( Vec3( 0, 0, -kG ), Vec3( 1, 0, 0 ), true );
         run( steps );
         const real v( box->getLinearVel()[0] ), expected( real(1) - kMuD * kG * t );
         std::printf( "  box on box:      v_x = %.4f m/s after %.2f s (mu_d: %.4f), mu_eff = %.4f\n",
                      static_cast<double>( v ), static_cast<double>( t ), static_cast<double>( expected ),
                      static_cast<double>( ( real(1) - v ) / ( kG * t ) ) );
         std::snprintf( what, sizeof( what ), "%s box on box: the sliding box decelerates with mu_d g (v_x = %.4f, expected %.4f)",
                        m.name, static_cast<double>( v ), static_cast<double>( expected ) );
         expect( std::fabs( v - expected ) < m.tolerance * expected, what );
      }
   }
   theCollisionSystem()->setRelaxationModel( CollisionSystemType::ApproximateInelasticCoulombContactByDecoupling );

   if( failures == 0 ) std::printf( "pe_contact_friction_coefficient_test: all checks passed\n" );
   else std::printf( "pe_contact_friction_coefficient_test: %d check(s) failed\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
