//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_setup_test.cpp
 *  \brief Setups that do not support the restitution switch refuse an explicit
 *         restitutionEnabled_ = true (pe/interface/support/optional_collision_params.h,
 *         rejectRestitutionRequest()).
 *
 *  usage: pe_el_restitution_setup_test frozen | dnsdrag | inherit
 *    frozen   setupELFrozenTraceSerial (its materials have e = 0 hard-coded)
 *    dnsdrag  setupDNSDragSerial (DNS setup, no restitution wiring)
 *    inherit  a supporting setup switched restitution on (emulated: deck with the key + the
 *             applier), the world is cleared, then an unsupported setup loads a deck WITHOUT the
 *             key: the solver must be off afterwards and a head-on impact inelastic
 *  The deck (example.json in the working directory) asks for restitution; the setup must throw a
 *  std::runtime_error naming restitutionEnabled_ before it creates any body. Registered against
 *  the default library (EL solver) and against HardContactAndFluid (pe_static_lubstage_fluid),
 *  the DNS solver without the switch. Serial setups; no MPI.
 */
//=================================================================================================

#include <pe/interface/setup/dns_drag.h>
#include <pe/interface/setup/el_frozen_trace.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>

template< typename T, typename = void > struct HasSwitch : std::false_type {};
template< typename T > struct HasSwitch< T, std::void_t< decltype( std::declval<const T&>().getRestitution() ) > > : std::true_type {};

//! Mode inherit: whether the solver is still on (false without a switch).
template< typename CS > bool solverOn( const CS& cs )
{
   if constexpr ( HasSwitch<CS>::value ) return cs.getRestitution();
   else { (void)cs; return false; }
}

static int runInherit()
{
   {
      std::ofstream deck( "example.json" );
      deck << "{ \"stepsize_\": 0.001, \"restitution_\": 0.9, \"restitutionEnabled_\": true }\n";
   }
   pe::SimulationConfig::loadFromFile( "example.json" );
   bool supported( true );
   try { pe::applyOptionalRestitutionParams( *pe::theCollisionSystem(), pe::SimulationConfig::getInstance() ); }
   catch( const std::runtime_error& ) { supported = false; }   // HardContactAndFluid: nothing to inherit
   const bool onBefore( solverOn( *pe::theCollisionSystem() ) );
   pe::theWorld()->clear();
   {
      std::ofstream deck( "example.json" );
      deck << "{ \"stepsize_\": 0.001, \"timesteps_\": 1, \"substeps_\": 1, \"benchRadius_\": 0.01,\n"
              "  \"particleDensity_\": 1100.0, \"fluidDensity_\": 1000.0, \"fluidViscosity_\": 0.001,\n"
              "  \"gravity_\": [0.0, 0.0, 0.0], \"vtk_\": false }\n";
   }
   pe::setupELFrozenTraceSerial( 1, 0.0, 2.2, 0.0, 0.41, 0.0, 0.41 );
   const bool onAfter( solverOn( *pe::theCollisionSystem() ) );
   // head-on impact with e = 0.9 materials after the unsupported setup
   pe::WorldID world = pe::theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const pe::MaterialID m( pe::createMaterial( 1000.0, 0.9, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 ) );
   const double r( 0.01 ), u( 0.5 ), dt( 1e-4 ), gap( 2 * u * dt * 5.9 );
   pe::SphereID a = pe::createSphere( 1, pe::Vec3( -r - 0.5 * gap, 0, 0 ), r, m );
   pe::SphereID b = pe::createSphere( 2, pe::Vec3(  r + 0.5 * gap, 0, 0 ), r, m );
   a->setLinearVel( pe::Vec3( u, 0, 0 ) ); b->setLinearVel( pe::Vec3( -u, 0, 0 ) );
   for( int i = 0; i < 40; ++i ) world->simulationStep( dt );
   const double ratio( ( b->getLinearVel()[0] - a->getLinearVel()[0] ) / ( 2 * u ) );
   std::printf( "inherit: solver on after the supporting setup %d, after the unsupported setup %d; head-on v_rel'/v_rel %.4f\n",
                onBefore ? 1 : 0, onAfter ? 1 : 0, ratio );
   const bool ok( ( !supported || onBefore ) && !onAfter && ratio < 0.2 );
   std::printf( "pe_el_restitution_setup_test inherit: %s\n", ok ? "all checks passed" : "FAILED" );
   return ok ? 0 : 1;
}

int main( int argc, char** argv )
{
   const std::string which( argc > 1 ? argv[1] : "" );
   if( which == "inherit" ) return runInherit();
   if( which != "frozen" && which != "dnsdrag" ) {
      std::printf( "usage: pe_el_restitution_setup_test frozen | dnsdrag | inherit\n" );
      return 2;
   }
   {
      std::ofstream deck( "example.json" );
      deck << "{ \"stepsize_\": 0.001, \"timesteps_\": 1, \"substeps_\": 1, \"benchRadius_\": 0.01,\n"
              "  \"particleDensity_\": 1100.0, \"fluidDensity_\": 1000.0, \"fluidViscosity_\": 0.001,\n"
              "  \"volumeFraction_\": 0.01, \"gravity_\": [0.0, 0.0, 0.0], \"vtk_\": false,\n"
              "  \"restitution_\": 0.9, \"restitutionEnabled_\": true }\n";
   }
   const size_t bodiesBefore( pe::theWorld()->size() );
   bool refused( false );
   std::string message;
   try {
      if( which == "frozen" ) pe::setupELFrozenTraceSerial( 1, 0.0, 2.2, 0.0, 0.41, 0.0, 0.41 );
      else                    pe::setupDNSDragSerial( 1 );
   }
   catch( const std::runtime_error& ex ) { refused = true; message = ex.what(); }
   const size_t bodiesAfter( pe::theWorld()->size() );
   std::printf( "%s: %s%s; bodies created %zu\n", which.c_str(), refused ? "refused: " : "NOT refused", message.c_str(),
                bodiesAfter - bodiesBefore );
   const bool ok( refused && message.find( "restitutionEnabled_" ) != std::string::npos && bodiesAfter == bodiesBefore );
   std::printf( "pe_el_restitution_setup_test %s: %s\n", which.c_str(), ok ? "all checks passed" : "FAILED" );
   return ok ? 0 : 1;
}
