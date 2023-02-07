//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsPod.h
 *  \brief Header file for the IsPod type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISPOD_H_
#define _PE_UTIL_TYPETRAITS_ISPOD_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_pod.hpp>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsPod type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsPodHelper
{
   //**********************************************************************************************
   enum { value = boost::is_pod<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for pod data types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a POD (Plain Old Data)
 * or not. In case the type is a POD, the \a value member enumeration is set o 1, the nested
 * type definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   class A {
      int i_;
      double d_;
   };

   class B {
      virtual ~B() {}
   }

   class C {
      std::string s_;
   };

   pe::IsPod<int>::value                 // Evaluates to 1
   pe::IsPod<const double>::Type         // Results in TrueType
   pe::IsPod<volatile A>                 // Is derived from TrueType
   pe::IsPod< std::vector<int> >::value  // Evaluates to 0
   pe::IsPod<B>::Type                    // Results in FalseType
   pe::IsPod<C>                          // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsPod : public IsPodHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsPodHelper<T>::value };
   typedef typename IsPodHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
