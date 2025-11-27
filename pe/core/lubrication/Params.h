//=================================================================================================
/*!
 *  \file pe/core/lubrication/Params.h
 *  \brief Lightweight access to lubrication/contact hysteresis parameters
 */
//=================================================================================================

#ifndef _PE_CORE_LUBRICATION_PARAMS_H_
#define _PE_CORE_LUBRICATION_PARAMS_H_

#include <pe/system/Precision.h>

namespace pe {
namespace lubrication {

// Getter/setter pair for contact hysteresis half-width
real getContactHysteresisDelta();
void setContactHysteresisDelta( real delta );

// Getter/setter pair for lubrication hysteresis half-width
real getLubricationHysteresisDelta();
void setLubricationHysteresisDelta( real delta );

// Getter/setter pair for lubrication threshold distance
real getLubricationThreshold();
void setLubricationThreshold( real threshold );

} // namespace lubrication
} // namespace pe

#endif
