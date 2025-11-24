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

} // namespace lubrication
} // namespace pe
