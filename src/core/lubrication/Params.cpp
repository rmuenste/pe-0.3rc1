//=================================================================================================
/*!
 *  \file src/core/lubrication/Params.cpp
 *  \brief Runtime storage for lubrication parameters and switches
 */
//=================================================================================================

#include <pe/core/lubrication/Params.h>

#include <algorithm>

namespace pe {
namespace lubrication {

namespace {
real contactHystDelta  = real(0);
real lubricationHystDelta = real(0);
real lubricationThresh = real(1E-2);  // Default matches Thresholds.h

// Model switches/parameters. Production defaults: Kroupa-2016 model, semi-implicit
// scheme, relative outer cutoff 0.5. Must stay in sync with the SimulationConfig
// constructor defaults (src/config/SimulationConfig.cpp).
bool enabled          = true;
int  model            = 0;        // ModelKind: 0 = kroupa2016
int  scheme           = 0;        // SchemeKind: 0 = semi-implicit
bool tangential       = true;
bool twisting         = true;
bool slipCorrection   = true;
bool resistSeparation = true;
bool wallTerms        = true;
real epsCritical      = real(0.1);
real cutoffFactor     = real(0.5);
real meshClampFactor  = real(0);
real meshDx           = real(0);
bool aabbInflation    = true;
real maxSphereRadius  = real(0);
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

bool isEnabled()              { return enabled; }
void setEnabled( bool on )    { enabled = on; }

int  getModel()               { return model; }
void setModel( int m )        { model = m; }

int  getScheme()              { return scheme; }
void setScheme( int s )       { scheme = s; }

bool getTangential()          { return tangential; }
void setTangential( bool on ) { tangential = on; }

bool getTwisting()            { return twisting; }
void setTwisting( bool on )   { twisting = on; }

bool getSlipCorrection()          { return slipCorrection; }
void setSlipCorrection( bool on ) { slipCorrection = on; }

bool getResistSeparation()          { return resistSeparation; }
void setResistSeparation( bool on ) { resistSeparation = on; }

bool getWallTerms()           { return wallTerms; }
void setWallTerms( bool on )  { wallTerms = on; }

real getEpsCritical()         { return epsCritical; }
void setEpsCritical( real e ) { epsCritical = e; }

real getCutoffFactor()          { return cutoffFactor; }
void setCutoffFactor( real f )  { cutoffFactor = f; }

real getMeshClampFactor()         { return meshClampFactor; }
void setMeshClampFactor( real f ) { meshClampFactor = f; }

real getMeshDx()              { return meshDx; }
void setMeshDx( real dx )     { meshDx = dx; }

bool getAabbInflation()           { return aabbInflation; }
void setAabbInflation( bool on )  { aabbInflation = on; }

real getMaxSphereRadius()     { return maxSphereRadius; }

void registerSphereRadius( real radius )
{
   maxSphereRadius = std::max( maxSphereRadius, radius );
}

real lubricationCutoff( real aRef )
{
   if( cutoffFactor <= real(0) )
      return lubricationThresh;

   real cutoff = cutoffFactor * aRef;
   if( meshClampFactor > real(0) && meshDx > real(0) )
      cutoff = std::min( cutoff, meshClampFactor * meshDx );
   return cutoff;
}

real aabbPadding( real bodyRadius )
{
   if( !enabled || !aabbInflation )
      return real(0);
   if( cutoffFactor <= real(0) )
      return lubricationThresh;
   return cutoffFactor * bodyRadius;
}

} // namespace lubrication
} // namespace pe
