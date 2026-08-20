//=================================================================================================
/*!
 *  \file pe/core/lubrication/Params.h
 *  \brief Lightweight access to runtime lubrication parameters and switches
 *
 *  Free-function global store so detection code (MaxContacts), bounding-box code
 *  (SphereBase/PlaneBase) and the HardContactLubricated solver can read runtime-configured
 *  lubrication parameters without depending on the CollisionSystem or SimulationConfig
 *  headers. Values are pushed in from the interface layer (applyOptionalLubricationParams)
 *  after SimulationConfig::loadFromFile, or set programmatically.
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

// Getter/setter pair for lubrication threshold distance (legacy absolute outer cutoff)
real getLubricationThreshold();
void setLubricationThreshold( real threshold );

//=================================================================================================
// Model switches and parameters (see doc/technical-notes/lubrication-production-design.md).
// Enum values for model/scheme are ModelKind/SchemeKind from LubricationModel.h.
//=================================================================================================

// Master switch for lubrication contacts/forces
bool isEnabled();
void setEnabled( bool enabled );

// Force model: 0 = Kroupa 2016, 1 = legacy (ModelKind)
int  getModel();
void setModel( int model );

// Integration scheme: 0 = semi-implicit, 1 = explicit-capped (SchemeKind)
int  getScheme();
void setScheme( int scheme );

// Tangential terms (sliding force + sliding torque)
bool getTangential();
void setTangential( bool on );

// Twisting torque
bool getTwisting();
void setTwisting( bool on );

// Vinogradova slip correction f*
bool getSlipCorrection();
void setSlipCorrection( bool on );

// Resist separating motion (normal suction) as well as approach
bool getResistSeparation();
void setResistSeparation( bool on );

// Use the dedicated wall resistance set for sphere-plane pairs
bool getWallTerms();
void setWallTerms( bool on );

// eps_c = h_c / a_ref: saturation distance / slip length scale
real getEpsCritical();
void setEpsCritical( real eps );

// eps_cut = h_cut / a_ref relative outer cutoff; 0 = legacy absolute threshold
real getCutoffFactor();
void setCutoffFactor( real factor );

// Mesh clamp factor c: caps h_cut at c * dx in resolved CFD-coupled runs; 0 = off
real getMeshClampFactor();
void setMeshClampFactor( real factor );

// CFD mesh width dx used by the mesh clamp; 0 = unknown/off (set by the interface layer)
real getMeshDx();
void setMeshDx( real dx );

// Grow AABBs by the lubrication cutoff so coarse detection sees pre-contact pairs
bool getAabbInflation();
void setAabbInflation( bool on );

// Largest sphere radius registered so far (used for plane AABB padding)
real getMaxSphereRadius();
void registerSphereRadius( real radius );

//=================================================================================================
// Derived helpers
//=================================================================================================

// Effective outer cutoff distance for a pair with reference radius aRef:
//   cutoffFactor > 0 : min( cutoffFactor*aRef, meshClampFactor*meshDx if both > 0 )
//   cutoffFactor == 0: getLubricationThreshold()  (legacy absolute)
real lubricationCutoff( real aRef );

// AABB padding for a body of the given radius:
//   0                          if lubrication disabled or inflation switched off
//   cutoffFactor * bodyRadius  in relative-cutoff mode (mesh clamp deliberately NOT
//                              applied: padding must stay a superset of the force band)
//   getLubricationThreshold()  in legacy absolute mode (radius ignored)
real aabbPadding( real bodyRadius );
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
