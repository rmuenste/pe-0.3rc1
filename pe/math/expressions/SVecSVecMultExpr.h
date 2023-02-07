//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecSVecMultExpr.h
 *  \brief Header file for the sparse vector/sparse vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECSVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS SVECSVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for sparse vector-sparse vector multiplications.
 * \ingroup sparse_vector_expression
 *
 * The SVecSVecMultExpr class represents the compile time expression for componentwise
 * multiplications between sparse vectors.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF >     // Transposition flag
class SVecSVecMultExpr : public SparseVector< SVecSVecMultExpr<VT1,VT2,TF>, TF >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT1::ResultType     RT1;  //!< Result type of the left-hand side sparse vector expression.
   typedef typename VT2::ResultType     RT2;  //!< Result type of the right-hand side sparse vector expression.
   typedef typename VT1::TransposeType  TT1;  //!< Transpose type of the left-hand side sparse vector expression.
   typedef typename VT2::TransposeType  TT2;  //!< Transpose type of the right-hand side sparse vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SVecSVecMultExpr<VT1,VT2,TF>           This;           //!< Type of this SVecSVecMultExpr instance.
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef SVecSVecMultExpr<TT1,TT2,!TF>          TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the left-hand side sparse vector expression.
   typedef typename VT1::CompositeType  Lhs;

   //! Composite type of the right-hand side sparse vector expression.
   typedef typename VT2::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the SVecSVecMultExpr class.
   */
   explicit inline SVecSVecMultExpr( const VT1& lhs, const VT2& rhs )
      : lhs_     ( lhs )  // Left-hand side dense vector of the multiplication expression
      , rhs_     ( rhs )  // Right-hand side sparse vector of the multiplication expression
      , nonzeros_( 0 )    // The number of non-zero elements in the sparse vector
   {
      using boost::remove_reference;

      pe_INTERNAL_ASSERT( lhs.size() == rhs.size(), "Invalid vector sizes" );

      typedef typename remove_reference<Lhs>::type  X1;             // Auxiliary type for the left-hand side composite type
      typedef typename remove_reference<Rhs>::type  X2;             // Auxiliary type for the right-hand side composite type
      typedef typename X1::ConstIterator            LeftIterator;   // Iterator type of the left-hand sparse vector expression
      typedef typename X2::ConstIterator            RightIterator;  // Iterator type of the right-hand sparse vector expression

      const LeftIterator  lend( lhs_.end() );
      const RightIterator rend( rhs_.end() );

      // Estimating the number of non-zero elements
      LeftIterator  l( lhs_.begin() );
      RightIterator r( rhs_.begin() );

      for( ; l!=lend && r!=rend; ++l ) {
         while( r->index() < l->index() && ++r != rend ) {}
         if( r!=rend && l->index() == r->index() ) {
            ++nonzeros_;
            ++r;
         }
      }
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return lhs_.size();
   }
   //**********************************************************************************************

   //**NonZeros function***************************************************************************
   /*!\brief Returns the number of non-zero elements in the sparse vector.
   //
   // \return The number of non-zero elements in the sparse vector.
   */
   inline size_t nonZeros() const {
      return nonzeros_;
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression is aliased with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case an alias effect is detected, \a false otherwise.
   */
   template< typename T >
   inline bool isAliased( const T* alias ) const {
      return ( lhs_.isAliased( alias ) || rhs_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs    lhs_;       //!< Left-hand side dense vector of the multiplication expression.
   Rhs    rhs_;       //!< Right-hand side sparse vector of the multiplication expression.
   size_t nonzeros_;  //!< The number of non-zero elements in the sparse vector.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a sparse vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a sparse vector-sparse
   // vector multiplication expression to a dense vector.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline void assign( DenseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  LeftIterator;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  RightIterator;

      const LeftIterator  lend( rhs.lhs_.end() );
      const RightIterator rend( rhs.rhs_.end() );

      LeftIterator  l( rhs.lhs_.begin()  );
      RightIterator r( rhs.rhs_.begin() );

      for( ; l!=lend; ++l ) {
         while( r!=rend && r->index() < l->index() ) ++r;
         if( r==rend ) break;
         if( l->index() == r->index() ) {
            (~lhs)[l->index()] = l->value() * r->value();
            ++r;
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a sparse vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a sparse vector-sparse
   // vector multiplication expression to a sparse vector.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline void assign( SparseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  LeftIterator;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  RightIterator;

      const LeftIterator  lend( rhs.lhs_.end() );
      const RightIterator rend( rhs.rhs_.end() );

      LeftIterator  l( rhs.lhs_.begin()  );
      RightIterator r( rhs.rhs_.begin() );

      for( ; l!=lend; ++l ) {
         while( r!=rend && r->index() < l->index() ) ++r;
         if( r==rend ) break;
         if( l->index() == r->index() ) {
            (~lhs).append( l->index(), l->value() * r->value() );
            ++r;
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a sparse vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a sparse vector-
   // sparse vector multiplication expression to a dense vector.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline void addAssign( DenseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  LeftIterator;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  RightIterator;

      const LeftIterator  lend( rhs.lhs_.end() );
      const RightIterator rend( rhs.rhs_.end() );

      LeftIterator  l( rhs.lhs_.begin() );
      RightIterator r( rhs.rhs_.begin() );

      for( ; l!=lend; ++l ) {
         while( r!=rend && r->index() < l->index() ) ++r;
         if( r==rend ) break;
         if( l->index() == r->index() ) {
            (~lhs)[l->index()] += l->value() * r->value();
            ++r;
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse vectors*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a sparse vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a sparse vector-
   // sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline void addAssign( SparseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      addAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to dense vectors*****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a sparse vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a sparse vector-
   // sparse vector multiplication expression to a dense vector.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline void subAssign( DenseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  LeftIterator;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  RightIterator;

      const LeftIterator  lend( rhs.lhs_.end() );
      const RightIterator rend( rhs.rhs_.end() );

      LeftIterator  l( rhs.lhs_.begin()  );
      RightIterator r( rhs.rhs_.begin() );

      for( ; l!=lend; ++l ) {
         while( r!=rend && r->index() < l->index() ) ++r;
         if( r==rend ) break;
         if( l->index() == r->index() ) {
            (~lhs)[l->index()] -= l->value() * r->value();
            ++r;
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse vectors****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a sparse vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a sparse vector-
   // sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline void subAssign( SparseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      subAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to dense vectors**************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a sparse vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a sparse
   // vector-sparse vector multiplication expression to a dense vector.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline void multAssign( DenseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  LeftIterator;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  RightIterator;

      const LeftIterator  lend( rhs.lhs_.end() );
      const RightIterator rend( rhs.rhs_.end() );

      LeftIterator  l( rhs.lhs_.begin()  );
      RightIterator r( rhs.rhs_.begin() );

      size_t i( 0 );

      for( ; l!=lend; ++l ) {
         while( r!=rend && r->index() < l->index() ) ++r;
         if( r==rend ) break;
         if( l->index() == r->index() ) {
            for( ; i<r->index(); ++i )
               reset( (~lhs)[i] );
            (~lhs)[l->index()] *= l->value() * r->value();
            ++r;
            ++i;
         }
      }

      for( ; i<rhs.size(); ++i )
         reset( (~lhs)[i] );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a sparse vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a sparse
   // vector-sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline void multAssign( SparseVector<VT,TF>& lhs, const SVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename VT::ConstIterator                                  Iterator1;
      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  Iterator2;
      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  Iterator3;

      VT tmp( rhs.size(), rhs.nonZeros() );

      const Iterator1 end1( (~lhs).end()   );
      const Iterator2 end2( rhs.lhs_.end() );
      const Iterator3 end3( rhs.rhs_.end() );

      Iterator1 i1( (~lhs).begin()   );
      Iterator2 i2( rhs.lhs_.begin() );
      Iterator3 i3( rhs.rhs_.begin() );

      for( ; i1!=end1; ++i1 ) {
         while( i2!=end2 && i2->index() < i1->index() ) ++i2;
         if( i2==end2 ) break;
         while( i3!=end3 && i3->index() < i1->index() ) ++i3;
         if( i3==end3 ) break;
         if( i1->index() == i2->index() && i1->index() == i3->index() ) {
            tmp.append( i1->index(), i1->value() * i2->value() * i3->value() );
            ++i2;
            ++i3;
         }
      }

      swap( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT2 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT1, TF );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT2, TF );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the componentwise multiplication of two sparse vectors
 *        (\f$ \vec{a}=\vec{b}*\vec{c} \f$).
 * \ingroup sparse_vector
 *
 * \param lhs The left-hand side sparse vector for the component product.
 * \param rhs The right-hand side sparse vector for the component product.
 * \return The product of the two sparse vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the componentwise multiplication of two sparse vectors:

   \code
   pe::SVecN a, b, c;
   // ... Resizing and initialization
   c = a * b;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the two involved
 * vector element types \a T1::ElementType and \a T2::ElementType. Both vector types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline const SVecSVecMultExpr<T1,T2,TF>
   operator*( const SparseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   return SVecSVecMultExpr<T1,T2,TF>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
