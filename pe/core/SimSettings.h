
//=================================================================================================
/*!
 *  \file pe/core/Settings.h
 *  \brief Header file for the world settings
 *
 *  Copyright (C) 2023 Raphael Muenster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_CORE_SIMSETTINGS_H_
#define _PE_CORE_SIMSETTINGS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/NonCreatable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Settings of the rigid body simulation world.
 * \ingroup core
 *
 * The Settings class represents the current settings of the rigid body simulation world.
 * It offers the functionality to acquire the current settings (as for instance the damping
 * factor and the gravity) for all modules of the physics engine. Although it is closely
 * coupled to the World class, it is implemented as a separate class to minimize coupling
 * for other classes.
 */
class PE_PROTECTED SimSettings : private NonCreatable
{
public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline int         px();
   static inline int         py();
   static inline int         pz();
   static inline int         visspacing();
   static inline real        stepsize();
//   static inline const Vec3& gravity();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static int px_;  //!< The number of processes in x
   static int py_;  //!< The number of processes in y
   static int pz_;  //!< The number of processes in z
   static int visspacing_;  //!< The visualization spacing
   static real stepSize_; //!< The time step size of the simulation
//   static Vec3 gravity_;       //!< Gravity of the simulated system.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class World;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the time step size
 *
 * \return \a The time step size
 */
inline real SimSettings::stepsize()
{
   return stepSize_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of processes in x
 *
 * \return \a The number of processes in x
 */
inline int SimSettings::px()
{
   return px_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of processes in y
 *
 * \return \a The number of processes in y
 */
inline int SimSettings::py()
{
   return py_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of processes in z
 *
 * \return \a The number of processes in z
 */
inline int SimSettings::pz()
{
   return pz_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the visualization spacing
 *
 * \return \a The visualization spacing
 */
inline int SimSettings::visspacing()
{
   return visspacing_;
}
//*************************************************************************************************


//*************************************************************************************************
} // namespace pe

#endif
