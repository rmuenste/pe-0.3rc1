//=================================================================================================
/*!
 *  \file src/interface/coupling/BuildInfo.cpp
 *  \brief Build configuration queries of the coupling API
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <limits>
#include <string>
#include <pe/core.h>
#include <pe/interface/coupling/BuildInfo.h>
#include <pe/system/SleepMode.h>
#include <src/interface/coupling/SolverFeatures.h>


//*************************************************************************************************
// Local macros
//*************************************************************************************************

#define pe_COUPLING_STRINGIFY2( x ) #x
#define pe_COUPLING_STRINGIFY( x ) pe_COUPLING_STRINGIFY2( x )


namespace pe {

namespace coupling {

//=================================================================================================
//
//  BUILD INFORMATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns pe_CONSTRAINT_SOLVER as it was configured for this library.
 *
 * \return The solver name, e.g. "pe::response::HardContactAndFluid".
 */
std::string detail::constraintSolverName()
{
   return std::string( pe_COUPLING_STRINGIFY( pe_CONSTRAINT_SOLVER ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the build configuration of the linked pe library.
 *
 * \return The build information.
 *
 * Needs no engine and touches no pe singleton.
 */
BuildInfo buildInfo()
{
   typedef detail::ActiveCollisionSystem CS;

   BuildInfo info;
   info.mpi                   = ( HAVE_MPI != 0 );
   info.constraintSolver      = detail::constraintSolverName();
   info.gravity               = detail::gravitySemantics<CS>();
   info.buoyancy              = detail::buoyancySemantics<CS>();
   info.realSize              = sizeof( real );
#ifdef PE_USE_CGAL
   info.cgal                  = true;
#else
   info.cgal                  = false;
#endif
#if defined( MOBILE_INFINITE ) && MOBILE_INFINITE
   info.mobileInfinite        = true;
#else
   info.mobileInfinite        = false;
#endif
   info.sleepMode             = ( sleepThreshold != real( 0 ) );
   info.realDigits            = std::numeric_limits<real>::digits;
   info.checkpointsSupported  = !info.sleepMode &&
                                info.realDigits <= std::numeric_limits<double>::digits;
   return info;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the compiled constraint solver supports a world setting.
 *
 * \param setting The setting.
 * \return \a true if the corresponding Engine setter works, \a false if it throws.
 */
bool isSupported( Setting setting )
{
   return detail::settingSupported( setting );
}
//*************************************************************************************************

} // namespace coupling

} // namespace pe
