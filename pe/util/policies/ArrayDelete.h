//=================================================================================================
/*!
 *  \file pe/util/policies/ArrayDelete.h
 *  \brief Header file for the ArrayDelete policy classes.
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

#ifndef _PE_UTIL_POLICIES_ARRAYDELETE_H_
#define _PE_UTIL_POLICIES_ARRAYDELETE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/checked_delete.hpp>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Array-delete policy class.
 * \ingroup util
 *
 * The ArrayDelete policy functor class applies an array delete operation to the given argument.
 * Note that the array delete operation is NOT permitted for inclomplete types (i.e. declared
 * but undefined data types). The attempt to apply an ArrayDelete functor to a pointer to an
 * array of objects of incomplete type results in a compile time error!
 */
struct ArrayDelete
{
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Type >
   inline void operator()( Type ptr ) const;
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
/*!\brief Implementation of the array-delete policy.
 *
 * \param ptr The pointer to the array to be deleted.
 * \return void
 *
 * This function applies an array delete operation to the given argument. Note that the array
 * delete operation is NOT permitted for inclomplete types (i.e. declared but undefined data
 * types). The attempt to use this function for a pointer to an array of objects of incomplete
 * type results in a compile time error!
 */
template< typename Type >
inline void ArrayDelete::operator()( Type ptr ) const
{
   boost::checked_array_delete( ptr );
}
//*************************************************************************************************

} // namespace pe

#endif
