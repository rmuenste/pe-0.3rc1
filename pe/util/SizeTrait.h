//=================================================================================================
/*!
 *  \file pe/util/SizeTrait.h
 *  \brief Header file for the size trait
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

#ifndef _PE_UTIL_SIZETRAIT_H_
#define _PE_UTIL_SIZETRAIT_H_


namespace pe {

//=================================================================================================
//
//  SIZETRAITHELPER CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary class for the SizeTrait class template.
 * \ingroup util
 */
template< typename T1, typename T2, bool >
struct SizeTraitHelper
{
   //**Type definitions****************************************************************************
   typedef T1  Large;
   typedef T2  Small;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the auxiliary SizeTraitHelper class template.
 * \ingroup util
 */
template< typename T1, typename T2 >
struct SizeTraitHelper<T1,T2,true>
{
   //**Type definitions****************************************************************************
   typedef T2  Large;
   typedef T1  Small;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SIZETRAIT CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base template for the SizeTrait class.
 * \ingroup util
 *
 * The SizeTrait class template evaluates the larger of the two given data types by use of the
 * sizeof operator. SizeTrait defines the data types \a Large for the larger of the two given
 * data types and \a Small for the smaller data type. In case both data types have the same
 * size, the first given data type \a T1 is chosen as the large and \a T2 as the small data
 * type.
 */
template< typename T1, typename T2 >
struct SizeTrait
{
private:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   //! Instantiation of the SizeTraitHelper class template.
   typedef SizeTraitHelper<T1,T2,( sizeof(T2) > sizeof(T1) )>  Helper;
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef typename Helper::Large  Large;  //!< The larger of the two given data types.
   typedef typename Helper::Small  Small;  //!< The smaller of the two given data types.
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
