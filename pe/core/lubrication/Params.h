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

// Getter/setter pair for the domain-decomposition shadow-copy margin.
// Default 0 (no behavior change). When pairwise lubrication is enabled the
// EL setup sets this to the lubrication cutoff so that any pair with a
// surface gap below the cutoff has both partners shadow-copied across the
// process boundary (HalfSpace::intersectsWith adds it to the overlap test).
real getShadowCopyMargin();
void setShadowCopyMargin( real margin );

} // namespace lubrication
} // namespace pe

#endif
