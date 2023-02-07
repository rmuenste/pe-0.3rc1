//=================================================================================================
/*!
 *  \file pe/util/NullType.h
 *  \brief Utility type for generic codes
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

#ifndef _PE_UTIL_NULLTYPE_H_
#define _PE_UTIL_NULLTYPE_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Utility type for generic codes.
 * \ingroup util
 *
 * The NullType class represents an invalid or terminating data type for generic codes. For
 * instance, the TypeList class uses the NullType as terminating data type for the type list.
 */
class NullType
{};
//*************************************************************************************************

} // namespace pe

#endif
