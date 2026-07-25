//=================================================================================================
/*!
 *  \file src/core/lubrication/Params.cpp
 *  \brief Runtime storage for lubrication/contact hysteresis parameters
 */
//=================================================================================================

#include <pe/core/lubrication/Params.h>

namespace pe {
namespace lubrication {

namespace {
real contactHystDelta  = real(0);
real lubricationHystDelta = real(0);
real lubricationThresh = real(1E-2);  // Default matches Thresholds.h
real shadowCopyMargin  = real(0);     // Off unless lubrication setup enables it
}  // namespace

real getContactHysteresisDelta()
{
   return contactHystDelta;
}

void setContactHysteresisDelta( real delta )
{
   contactHystDelta = delta;
}

real getLubricationHysteresisDelta()
{
   return lubricationHystDelta;
}

void setLubricationHysteresisDelta( real delta )
{
   lubricationHystDelta = delta;
}

real getLubricationThreshold()
{
   return lubricationThresh;
}

void setLubricationThreshold( real threshold )
{
   lubricationThresh = threshold;
}

real getShadowCopyMargin()
{
   return shadowCopyMargin;
}

void setShadowCopyMargin( real margin )
{
   shadowCopyMargin = margin;
}

} // namespace lubrication
} // namespace pe
