//=================================================================================================
/*!
 *  \file pe/core/response/HardContactAndFluidWithLubrication.h
 *  \brief Collision response solvers based on a timestepping approach for hard contact problems with a semi-implicit time integrator
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_RESPONSE_HARDCONTACTANDFLUIDWITHLUBRICATION_H_
#define _PE_CORE_RESPONSE_HARDCONTACTANDFLUIDWITHLUBRICATION_H_

// DEPRECATION: This response will be removed in the next version.
// Prefer pe::response::HardContactLubricated instead.
#if !defined(PE_SUPPRESS_DEPRECATION_WARNINGS)
#  if defined(__clang__) || defined(__GNUC__) || defined(_MSC_VER)
#    pragma message("pe: HardContactAndFluidWithLubrication response is deprecated; use HardContactLubricated")
#  endif
#endif


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/NonCopyable.h>
#include <pe/util/NullType.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Collision response solvers based on a timestepping approach for hard contact problems with a semi-implicit time integrator.
 * \ingroup collision_response
 *
 * TODO
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class HardContactAndFluidWithLubrication : private NonCopyable
{
public:

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U2, NullType );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
