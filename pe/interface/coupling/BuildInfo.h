//=================================================================================================
/*!
 *  \file pe/interface/coupling/BuildInfo.h
 *  \brief Build configuration of the linked pe library, queryable without an engine
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

#ifndef _PE_INTERFACE_COUPLING_BUILDINFO_H_
#define _PE_INTERFACE_COUPLING_BUILDINFO_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstddef>
#include <string>
#include <pe/interface/coupling/Export.h>
#include <pe/interface/coupling/Types.h>


namespace pe {

namespace coupling {

//=================================================================================================
//
//  STRUCTURE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief What the linked pe library was built with.
 * \ingroup coupling
 *
 * The values describe the pe library the program is linked against, as compiled, not the
 * headers of the calling translation unit. A CFD code uses them to refuse a configuration it
 * cannot work with, e.g.

   \code
   const pe::coupling::BuildInfo info( pe::coupling::buildInfo() );
   if( info.constraintSolver.find( "HardContactAndFluid" ) == std::string::npos )
      throw std::runtime_error( "pe must be built with HardContactAndFluid" );
   \endcode

 * \b Gravity and buoyancy. Who owns the body forces depends on the solver:
 * - \c HardContactAndFluid applies gravity and buoyancy: every step it adds
 *   \f$ V (\rho_p - \rho_f)\, m^{-1}\, g\,\Delta t \f$ to the velocity of each awake movable
 *   sphere, capsule, cylinder, box and ellipsoid, with \f$ V \f$ the volume enclosed by the
 *   surface, \f$ \rho_p \f$ the material density and \f$ m \f$ the body's mass (bare gravity,
 *   without buoyancy, to every other shape). For a solid body \f$ m = \rho_p V \f$ and this is
 *   \f$ (1 - \rho_f/\rho_p)\,g\,\Delta t \f$; a hollow sphere has the shell mass, so its
 *   increment is larger. The hydrodynamic load handed to pe must then NOT contain the submerged
 *   weight or the hydrostatic pressure force.
 * - \c HardContactSemiImplicitTimesteppingSolvers applies gravity but no buoyancy.
 * - \c HardContactEulerLagrange applies neither; the driver must include the submerged weight
 *   \f$ (m - \rho_f V) g \f$ in the load, with \f$ m \f$ the body's mass and \f$ V \f$ its
 *   displaced volume (BodyState::mass, BodyState::volume); for a solid body this simplifies to
 *   \f$ (\rho_p - \rho_f) V g \f$.
 * - A solver that does not declare its body-force semantics (the collision-system members
 *   \c appliesGravity / \c appliesBuoyancy) is reported as BodyForce::unknown: nobody has
 *   classified it, so the driver cannot tell which side owns the force.
 *
 * \b Checkpoints. With a nonzero pe::sleepThreshold (PE_SLEEP_THRESHOLD) pe puts slow bodies to
 * sleep; the checkpoint cannot restore the accumulated motion measure. A pe::real wider than
 * double cannot round-trip through the checkpoint's 17-digit text. In both cases
 * Engine::writeCheckpoint() and Engine::readCheckpoint() refuse (see \c checkpointsSupported).
 */
struct BuildInfo
{
   bool        mpi;                    //!< Built with MPI (HAVE_MPI).
   std::string constraintSolver;       //!< pe_CONSTRAINT_SOLVER as configured (full name).
   BodyForce   gravity;                //!< Who applies gravity.
   BodyForce   buoyancy;               //!< Who applies the buoyancy of the fluid density.
   std::size_t realSize;               //!< sizeof(pe::real) in bytes (8: double, 4: float).
   bool        cgal;                   //!< Built with CGAL (PE_USE_CGAL).
   bool        mobileInfinite;         //!< Fixed bodies may carry a velocity (MOBILE_INFINITE).
   bool        sleepMode;              //!< Nonzero sleep threshold (PE_SLEEP_THRESHOLD).
   int         realDigits;             //!< Mantissa digits of pe::real (53: double, 24: float).
   bool        checkpointsSupported;   //!< The build can write and read bitwise checkpoints:
                                       //!< no sleep mode and realDigits <= 53 (on one process).
};
//*************************************************************************************************




//=================================================================================================
//
//  BUILD INFORMATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the build configuration of the linked pe library.
 * \ingroup coupling
 *
 * \return The build information.
 *
 * Needs no engine and touches no pe singleton, so it can be called before MPI is initialized.
 */
PE_COUPLING_API BuildInfo buildInfo();
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the compiled constraint solver supports a world setting.
 * \ingroup coupling
 *
 * \param setting The setting.
 * \return \a true if the corresponding Engine setter works, \a false if it throws.
 *
 * Contact parameters and split impulses are detected from the collision system's member
 * functions; gravity and fluid density are available when the solver declares that it applies
 * gravity or buoyancy itself (BodyForce::appliedBySolver, see BuildInfo); the fluid viscosity
 * is always available, although only lubrication models read it.
 */
PE_COUPLING_API bool isSupported( Setting setting );
//*************************************************************************************************

} // namespace coupling

} // namespace pe

#endif
