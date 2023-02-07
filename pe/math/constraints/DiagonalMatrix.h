//=================================================================================================
/*!
 *  \file pe/math/constraints/DiagonalMatrix.h
 *  \brief Constraint on the data type
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

#ifndef _PE_MATH_CONSTRAINTS_DIAGONALMATRIX_H_
#define _PE_MATH_CONSTRAINTS_DIAGONALMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/typetraits/IsDiagonalMatrix.h>
#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>


namespace pe {

//=================================================================================================
//
//  MUST_BE_DIAGONAL_MATRIX_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is not a diagonal, N-dimensional matrix type, a compilation
 * error is created.
 */
#define pe_CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE_FAILED< IsDiagonalMatrix<T>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_DIAGONAL_MATRIX_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_NOT_BE_DIAGONAL_MATRIX_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_DIAGONAL_MATRIX_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is a diagonal, N-dimensional matrix type, a compilation
 * error is created.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_DIAGONAL_MATRIX_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_DIAGONAL_MATRIX_TYPE_FAILED< !IsDiagonalMatrix<T>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_DIAGONAL_MATRIX_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
