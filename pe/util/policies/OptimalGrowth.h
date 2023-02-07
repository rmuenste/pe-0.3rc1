//=================================================================================================
/*!
 *  \file pe/util/policies/OptimalGrowth.h
 *  \brief Header file for the OptimalGrowth policy classes.
 *
 *  Copyright (C) 2009 Klaus Iglberger
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

#ifndef _PE_UTIL_POLICIES_OPTIMALGROWTH_H_
#define _PE_UTIL_POLICIES_OPTIMALGROWTH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Functions.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Optimal growth policy class.
 * \ingroup util
 *
 * The OptimalGrowth policy class implements the optimal growth strategy suggested by Andrew
 * Koenig for the std::vector class (see Andrew Koenig's column in the September 1998 issue of
 * JOOP (Journal of Object-Oriented Programming), or the Dr. Dobb's article 'C++ Made Easier:
 * How Vectors Grow', 2001). It applies an exponential growth strategy using a factor of 1.5
 * and additionally ensures that the sizes returns are always multiples of four.
 */
struct OptimalGrowth
{
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline size_t operator()( size_t oldSize, size_t minSize ) const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a new size depending on the given old size and the required minimum size.
 *
 * \param old The old size.
 * \param minimum The required minimum size.
 * \return The new size (at least the required minimum size).
 */
inline size_t OptimalGrowth::operator()( size_t old, size_t minimum ) const
{
   const size_t needed( max( static_cast<size_t>( old*1.5 ), minimum ) );
   return ( ( needed )?( 4 * ( (needed-1)/4 + 1 ) ):( 0 ) );
}
//*************************************************************************************************

} // namespace pe

#endif
