//=================================================================================================
/*!
 *  \file pe/util/NonCopyable.h
 *  \brief Base class for non-copyable class instances
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

#ifndef _PE_UTIL_NONCOPYABLE_H_
#define _PE_UTIL_NONCOPYABLE_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for non-copyable class instances.
 * \ingroup util
 *
 * The NonCopyable class is intended to work as a base class for non-copyable classes. Both the
 * copy constructor and the copy assignment operator are declared private and left undefined in
 * order to prohibit copy operations of the derived classes.\n
 *
 * \b Note: It is not necessary to publicly derive from this class. It is sufficient to derive
 * privately to prevent copy operations on the derived class.

   \code
   class A : private NonCopyable
   { ... };
   \endcode
 */
class NonCopyable
{
protected:
   //**Constructor and destructor******************************************************************
   /*!\name Constructor and destructor */
   //@{
   inline NonCopyable()  {}  //!< Default constructor for the NonCopyable class.
   inline ~NonCopyable() {}  //!< Destructor of the NonCopyable class.
   //@}
   //**********************************************************************************************

private:
   //**Copy constructor and copy assignment operator***********************************************
   /*!\name Copy constructor and copy assignment operator */
   //@{
   NonCopyable( const NonCopyable& );             //!< Copy constructor (private & undefined)
   NonCopyable& operator=( const NonCopyable& );  //!< Copy assignment operator (private & undefined)
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
