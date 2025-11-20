#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include <pe/core.h>
#include <pe/support.h>
#include <pe/util.h>
#include <pe/core/Settings.h>
#include <pe/math/Constants.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/logging/InfoSection.h>

#include "LubricationDemoCommon.h"

using namespace pe;
using namespace pe::examples::lubrication;

namespace {

DemoConfig parseCommandLine( int argc, char* argv[] )
{
   DemoConfig cfg;

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "steps", value<std::size_t>()->default_value( cfg.totalSteps ), "Total simulation steps" )
      ( "dt", value<double>()->default_value( static_cast<double>( cfg.dt ) ), "Time step [s]" )
      ( "approach-steps", value<std::size_t>()->default_value( cfg.approachSteps ), "Steps with prescribed approach velocity" )
      ( "hold-steps", value<std::size_t>()->default_value( cfg.holdSteps ), "Steps to hold at minimum gap" )
      ( "log-every", value<std::size_t>()->default_value( cfg.logEvery ), "Log every N steps" )
      ( "approach-speed", value<double>()->default_value( static_cast<double>( cfg.approachSpeed ) ), "Approach speed of each sphere [m/s]" )
      ( "retreat-speed", value<double>()->default_value( static_cast<double>( cfg.retreatSpeed ) ), "Retreat speed of each sphere [m/s]" )
      ( "initial-gap", value<double>()->default_value( static_cast<double>( cfg.initialGap ) ), "Initial surface gap between spheres [m]" )
      ( "radius-a", value<double>()->default_value( static_cast<double>( cfg.radiusA ) ), "Radius of sphere A [m]" )
      ( "radius-b", value<double>()->default_value( static_cast<double>( cfg.radiusB ) ), "Radius of sphere B [m]" )
      ( "contact-blend", value<double>()->default_value( static_cast<double>( cfg.contactBlend ) ), "Half-width of hard-contact blend band" )
      ( "lubrication-blend", value<double>()->default_value( static_cast<double>( cfg.lubricationBlend ) ), "Half-width of lubrication blend band" )
      ( "min-gap", value<double>()->default_value( static_cast<double>( cfg.minGapRegularization ) ), "Gap regularisation epsilon used for force estimates" )
      ( "disable-info", "Silence info-level stdout logging" );

   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   cfg.totalSteps         = vm["steps"].as<std::size_t>();
   cfg.dt                 = static_cast<real>( vm["dt"].as<double>() );
   cfg.approachSteps      = vm["approach-steps"].as<std::size_t>();
   cfg.holdSteps          = vm["hold-steps"].as<std::size_t>();
   cfg.logEvery           = std::max<std::size_t>( std::size_t(1), vm["log-every"].as<std::size_t>() );
   cfg.approachSpeed      = static_cast<real>( vm["approach-speed"].as<double>() );
   cfg.retreatSpeed       = static_cast<real>( vm["retreat-speed"].as<double>() );
   cfg.initialGap         = static_cast<real>( vm["initial-gap"].as<double>() );
   cfg.radiusA            = static_cast<real>( vm["radius-a"].as<double>() );
   cfg.radiusB            = static_cast<real>( vm["radius-b"].as<double>() );
   cfg.contactBlend       = static_cast<real>( vm["contact-blend"].as<double>() );
   cfg.lubricationBlend   = static_cast<real>( vm["lubrication-blend"].as<double>() );
   cfg.minGapRegularization = static_cast<real>( vm["min-gap"].as<double>() );
   cfg.verbose            = ( vm.count( "disable-info" ) == 0 );

   return cfg;
}

void logConfiguration( const DemoConfig& cfg )
{
   if( !cfg.verbose ) return;

   const real contactThresholdValue     = contactThreshold;
   const real lubricationThresholdValue = lubricationThreshold;

   pe_LOG_INFO_SECTION( log ) {
      log << "Sphere-sphere lubrication demo configuration\n"
          << "   total steps          : " << cfg.totalSteps << "\n"
          << "   time step [s]        : " << cfg.dt << "\n"
          << "   approach steps       : " << cfg.approachSteps << "\n"
          << "   hold steps           : " << cfg.holdSteps << "\n"
          << "   logging stride       : " << cfg.logEvery << "\n"
          << "   radius A [m]         : " << cfg.radiusA << "\n"
          << "   radius B [m]         : " << cfg.radiusB << "\n"
          << "   initial gap [m]      : " << cfg.initialGap << "\n"
          << "   approach speed [m/s] : " << cfg.approachSpeed << "\n"
          << "   retreat speed [m/s]  : " << cfg.retreatSpeed << "\n"
          << "   contact threshold    : " << contactThresholdValue << "\n"
          << "   lubrication threshold: " << lubricationThresholdValue << "\n"
          << "   contact blend        : " << cfg.contactBlend << "\n"
          << "   lubrication blend    : " << cfg.lubricationBlend << "\n"
          << "   min gap epsilon      : " << cfg.minGapRegularization << "\n";
   }
}

} // namespace

int main( int argc, char* argv[] )
{
   DemoConfig cfg = parseCommandLine( argc, argv );

   if( cfg.approachSteps + cfg.holdSteps >= cfg.totalSteps ) {
      std::cerr << "The sum of approach-steps and hold-steps must be smaller than the total number of steps.\n";
      return EXIT_FAILURE;
   }

   if( cfg.initialGap <= real(0) ) {
      std::cerr << "Initial gap must be positive.\n";
      return EXIT_FAILURE;
   }

   applyBlendParameters( cfg.contactBlend, cfg.lubricationBlend );
   logConfiguration( cfg );

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );

   pe::id_t id( 0 );
   MaterialID material = granite;

   const real centerDistance = cfg.radiusA + cfg.radiusB + cfg.initialGap;
   const Vec3 posA( 0.0, 0.0, 0.5 * centerDistance );
   const Vec3 posB( 0.0, 0.0, -0.5 * centerDistance );

   SphereID sphereA = createSphere( ++id, posA, cfg.radiusA, material, true );
   SphereID sphereB = createSphere( ++id, posB, cfg.radiusB, material, true );

   sphereA->setLinearVel( 0.0, 0.0, -cfg.approachSpeed );
   sphereB->setLinearVel( 0.0, 0.0,  cfg.approachSpeed );

   const real contactThresholdValue     = contactThreshold;
   const real lubricationThresholdValue = lubricationThreshold;
   const real effectiveRadius           = effectiveRadiusSphereSphere( cfg.radiusA, cfg.radiusB );

   pe_LOG_INFO_SECTION( log ) {
      log << "Starting integration ...\n";
   }

   for( std::size_t step = 0; step < cfg.totalSteps; ++step )
   {
      if( step == cfg.approachSteps ) {
         sphereA->setLinearVel( 0.0, 0.0, 0.0 );
         sphereB->setLinearVel( 0.0, 0.0, 0.0 );
      }
      else if( step == cfg.approachSteps + cfg.holdSteps ) {
         sphereA->setLinearVel( 0.0, 0.0, cfg.retreatSpeed );
         sphereB->setLinearVel( 0.0, 0.0, -cfg.retreatSpeed );
      }

      world->simulationStep( cfg.dt );

      const Vec3 posAcur = sphereA->getPosition();
      const Vec3 posBcur = sphereB->getPosition();
      const Vec3 velA    = sphereA->getLinearVel();
      const Vec3 velB    = sphereB->getLinearVel();
      const Vec3 angA    = sphereA->getAngularVel();
      const Vec3 angB    = sphereB->getAngularVel();

      Vec3 delta = posAcur - posBcur;
      real distance = delta.length();
      Vec3 normal = distance > real(0) ? ( delta / distance ) : Vec3( 0.0, 0.0, 1.0 );
      const real gap = distance - ( cfg.radiusA + cfg.radiusB );

      const Vec3 relVel = velA - velB;
      const real vn = trans( normal ) * relVel;

      const real hardWeight = computeHardWeight( gap, contactThresholdValue, cfg.contactBlend );
      const real lubWeight  = computeLubricationWeight( gap, contactThresholdValue, lubricationThresholdValue,
                                                        cfg.contactBlend, cfg.lubricationBlend );

      if( step % cfg.logEvery == 0 ) {
         const real mu = Settings::liquidViscosity();
         const real denomGap = std::max( gap, cfg.minGapRegularization );
         const real baseForce = ( vn < real(0) )
                              ? real(6) * real(M_PI) * mu * effectiveRadius * effectiveRadius * (-vn) / denomGap
                              : real(0);
         const real weightedForce = baseForce * lubWeight;

         pe_LOG_DEBUG_SECTION( log ) {
            log << std::scientific << std::setprecision(6)
                << "[sphere-sphere step " << step
                << " phase=" << phaseName( step, cfg )
                << "] gap=" << gap
                << " vn=" << vn
                << " hard_w=" << hardWeight
                << " lub_w=" << lubWeight
                << " F_est=" << weightedForce
                << " posA=" << posAcur
                << " posB=" << posBcur
                << " velA=" << velA
                << " velB=" << velB
                << " omegaA=" << angA
                << " omegaB=" << angB
                << "\n";
         }
      }
   }

   pe_LOG_INFO_SECTION( log ) {
      log << "Integration finished after " << cfg.totalSteps << " steps.\n";
   }

   return EXIT_SUCCESS;
}
