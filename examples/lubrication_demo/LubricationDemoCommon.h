#pragma once

#include <algorithm>
#include <cstddef>
#include <string_view>
#include <type_traits>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Configuration.h>
#include <pe/core/collisionsystem/HardContactLubricated.h>
#include <pe/core/Types.h>

namespace pe::examples::lubrication {

namespace detail {

using LubricatedConfig = pe::Configuration<
   pe_COARSE_COLLISION_DETECTOR,
   pe_FINE_COLLISION_DETECTOR,
   pe_BATCH_GENERATOR,
   pe::response::HardContactLubricated
>::Config;

static_assert( std::is_same<LubricatedConfig, pe::Config>::value,
   "examples/lubrication_demo requires pe::response::HardContactLubricated as pe_CONSTRAINT_SOLVER.\n"
   "Define PE_USE_LEGACY_HARD_CONTACT only if you intentionally disable lubrication support." );

} // namespace detail

struct DemoConfig {
   pe::real dt{ pe::real(1e-4) };
   std::size_t totalSteps{ 2000 };
   std::size_t approachSteps{ 400 };
   std::size_t holdSteps{ 200 };
   std::size_t logEvery{ 10 };
   pe::real approachSpeed{ pe::real(0.02) };
   pe::real retreatSpeed{ pe::real(0.02) };
   pe::real initialGap{ pe::real(5e-5) };
   pe::real minGapRegularization{ pe::real(1e-8) };
   pe::real contactBlend{ pe::real(1e-9) };
   pe::real lubricationBlend{ pe::real(1e-9) };
   pe::real radiusA{ pe::real(0.01) };
   pe::real radiusB{ pe::real(0.01) };
   bool verbose{ true };
};

inline void applyBlendParameters( pe::real contactBlend, pe::real lubricationBlend )
{
   CollisionSystemID collisionSystem = theCollisionSystem();
   collisionSystem->setContactHysteresisDelta( contactBlend );
   collisionSystem->setLubricationHysteresisDelta( lubricationBlend );
}

inline std::string_view phaseName( std::size_t step, const DemoConfig& cfg )
{
   if( step < cfg.approachSteps ) {
      return "approach";
   }
   if( step < cfg.approachSteps + cfg.holdSteps ) {
      return "hold";
   }
   return "retreat";
}

inline pe::real clamp01( pe::real value )
{
   if( value <= pe::real(0) ) return pe::real(0);
   if( value >= pe::real(1) ) return pe::real(1);
   return value;
}

inline pe::real rampUp( pe::real x, pe::real start, pe::real end )
{
   if( end <= start ) {
      return x >= end ? pe::real(1) : pe::real(0);
   }
   if( x <= start ) return pe::real(0);
   if( x >= end )   return pe::real(1);
   return ( x - start ) / ( end - start );
}

inline pe::real rampDown( pe::real x, pe::real start, pe::real end )
{
   if( end <= start ) {
      return x <= start ? pe::real(1) : pe::real(0);
   }
   if( x <= start ) return pe::real(1);
   if( x >= end )   return pe::real(0);
   return ( end - x ) / ( end - start );
}

inline pe::real computeHardWeight( pe::real dist,
                                   pe::real contactThresholdValue,
                                   pe::real blendHalfWidth )
{
   if( blendHalfWidth <= pe::real(0) ) {
      return dist < contactThresholdValue ? pe::real(1) : pe::real(0);
   }

   const pe::real lower = contactThresholdValue - blendHalfWidth;
   const pe::real upper = contactThresholdValue + blendHalfWidth;
   return rampDown( dist, lower, upper );
}

inline pe::real computeLubricationWeight( pe::real dist,
                                          pe::real contactThresholdValue,
                                          pe::real lubricationThresholdValue,
                                          pe::real contactBlend,
                                          pe::real lubricationBlend )
{
   const pe::real entryStart = contactThresholdValue - contactBlend;
   const pe::real entryEnd   = contactThresholdValue + contactBlend;
   const pe::real exitCenter = contactThresholdValue + lubricationThresholdValue;
   const pe::real exitStart  = exitCenter - lubricationBlend;
   const pe::real exitEnd    = exitCenter + lubricationBlend;

   const pe::real weightUp = ( contactBlend > pe::real(0) )
                             ? rampUp( dist, entryStart, entryEnd )
                             : ( dist > contactThresholdValue ? pe::real(1) : pe::real(0) );

   const pe::real weightDown = ( lubricationBlend > pe::real(0) )
                               ? rampDown( dist, exitStart, exitEnd )
                               : ( dist < exitCenter ? pe::real(1) : pe::real(0) );

   return clamp01( weightUp * weightDown );
}

inline pe::real effectiveRadiusSphereSphere( pe::real r1, pe::real r2 )
{
   return ( r1 * r2 ) / ( r1 + r2 );
}

} // namespace pe::examples::lubrication
