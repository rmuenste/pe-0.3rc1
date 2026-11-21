//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_setup_test.cpp
 *  \brief Setups that do not support the restitution switch refuse an explicit
 *         restitutionEnabled_ = true (pe/interface/support/optional_collision_params.h,
 *         rejectRestitutionRequest()).
 *
 *  usage: pe_el_restitution_setup_test frozen | dnsdrag
 *    frozen   setupELFrozenTraceSerial (its materials have e = 0 hard-coded)
 *    dnsdrag  setupDNSDragSerial (DNS setup, no restitution wiring)
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

int main( int argc, char** argv )
{
   const std::string which( argc > 1 ? argv[1] : "" );
   if( which != "frozen" && which != "dnsdrag" ) {
      std::printf( "usage: pe_el_restitution_setup_test frozen | dnsdrag\n" );
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
