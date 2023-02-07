//=================================================================================================
/*!
 *  \file pe/math/shims/Invert.h
 *  \brief Header file for the invert shim
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

#ifndef _PE_MATH_SHIMS_INVERT_H_
#define _PE_MATH_SHIMS_INVERT_H_


namespace pe {

//=================================================================================================
//
//  INVERT SHIMS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Inverting the given single precision value.
 * \ingroup math_shims
 *
 * \param a The single precision value to be inverted.
 * \return The inverse of the given value.
 *
 * The invert shim represents an abstract interface for inverting a value/object of any given
 * data type. For single precision floating point values this results in \f$ \frac{1}{a} \f$.
 */
inline float inv( float a )
{
   return ( 1.0F / a );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the given double precision value.
 * \ingroup math_shims
 *
 * \param a The double precision value to be inverted.
 * \return The inverse of the given value.
 *
 * The invert shim represents an abstract interface for inverting a value/object of any given
 * data type. For double precision floating point values this results in \f$ \frac{1}{a} \f$.
 */
inline double inv( double a )
{
   return ( 1.0 / a );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the given long double value.
 * \ingroup math_shims
 *
 * \param a The long double value to be inverted.
 * \return The inverse of the given value.
 *
 * The invert shim represents an abstract interface for inverting a value/object of any given
 * data type. For long double floating point values this results in \f$ \frac{1}{a} \f$.
 */
inline long double inv( long double a )
{
   return ( 1.0L / a );
}
//*************************************************************************************************

} // namespace pe

#endif
