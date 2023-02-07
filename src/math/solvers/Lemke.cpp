//*************************************************************************************************
/*!
 *  \file src/math/solvers/Lemke.cpp
 *  \brief The Lemke pivoting algorithm for solving LCPs
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
//*************************************************************************************************


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <ostream>
#include <boost/format.hpp>
#include <pe/math/Accuracy.h>
#include <pe/math/Infinity.h>
#include <pe/math/solvers/Lemke.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/Random.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the Lemke class.
 */
Lemke::Lemke()
   : Solver()      // Initialization of the base class
   , basics_()     // TODO
   , nonbasics_()  // TODO
   , M_()          // TODO
   , Q_()          // TODO
{}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * \param lcp TODO
 * \return void
 *
 * TODO
 */
bool Lemke::solve( LCP& lcp )
{
   bool converged( false );
   VecN coverVector( lcp.size(), 1 );

   size_t it( 0 );
   for( ; !converged && it<maxIterations_; ++it )
   {
      bool solved( solve( lcp, coverVector ) );

      lastPrecision_ = lcp.residual();

      if( lastPrecision_ < threshold_ )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            if( !solved )
               log << pe_YELLOW << "      WARNING: The LCP was only solved approximately." << pe_OLDCOLOR;
         }

         converged = true;
      }
      else
      {
         pe_LOG_DEBUG_SECTION( log ) {
            if( solved )
               log << pe_YELLOW << "      WARNING: The LCP was supposedly solved but not accurately enough (" << lastPrecision_ << ")." << pe_OLDCOLOR;
            else
               log << pe_YELLOW << "      WARNING: The LCP was NOT solved." << pe_OLDCOLOR;
         }

         // Retrying with a different randomly chosen cover vector
         for( size_t i=0; i<coverVector.size(); ++i )
            coverVector[i] = rand<real>( static_cast<real>( 0.1 ), static_cast<real>( 9.91 ) );
      }
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( converged && it == 1 )
         log << "      Solved the LCP on first try.";
      else if( converged && it > 1 )
         log << pe_YELLOW << "      WARNING: Solved the LCP in " << it << " tries." << pe_OLDCOLOR;
      else
         log << pe_YELLOW << "      WARNING: Did not solve the LCP within accuracy. (" << lastPrecision_ << ")" << pe_OLDCOLOR;
   }

   lastIterations_ = it;

   return converged;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param lcp TODO
 * \param d TODO
 * \return TODO
 *
 * TODO
 */
bool Lemke::solve( LCP& lcp, const VecN& d )
{
   const size_t n( lcp.size() );

   const SMatN& A( lcp.A_ );
   const VecN&  b( lcp.b_ );

   basics_.resize( n, false );
   nonbasics_.resize( n+1, false );
   M_.resize( n, n+1, false );
   Q_.resize( n, n+1, false );

   // Merging q into Q' = [q; Q]
   for( size_t i=0; i<n; ++i )
   {
      Q_(i,0) = b[i];

      // Constructing a lexicographically positive identity matrix Q
      for( size_t j=1; j<Q_.columns(); ++j ) {
         if( j == i+1 )
            Q_(i,j) = 1;
         else
            Q_(i,j) = 0;
      }
   }

   // Preparing the augmented LCP with M' = [d; M]
   M_ = real( 0 );
   for( size_t i=0; i<n; ++i ) {
      M_(i,0) = d[i];

      for( SMatN::ConstIterator element=A.begin(i); element!=A.end(i); ++element ) {
         M_(i,element->index()+1) = element->value();
      }
   }

   // Annotating the tableau
   nonbasics_[0] = 0;
   for( size_t i=1; i<=n; ++i ) {
      basics_[i-1]  = -static_cast<ptrdiff_t>(i);
      nonbasics_[i] =  static_cast<ptrdiff_t>(i);
   }

   // Printing the annotated tableau
   //pe_LOG_DEBUG_SECTION( log ) {
   //   printTableau( log );
   //}

   // Determination of the lexicographically smallest blocking variable for the initial pivot step
   size_t r = inf;

   for( size_t i=0; i<n; ++i ) {
      if( Q_(i,0) < -accuracy ) {  // < 0
         r = i;
         break;
      }
   }

   // We are finished if q >= 0 since z = 0 solves the LCP
   if( r == inf ) {
      lcp.x_ = real( 0 );
      return true;
   }

   for( size_t i=r+1; i<n; ++i ) {
      if( Q_(i,0) > -accuracy ) {  // >= 0
         pe_INTERNAL_ASSERT( d[i] > -accuracy, "Negative value found" );  // >= 0
         continue;
      }

      pe_INTERNAL_ASSERT( d[i] > real( 0 ), "Non-positive value found" );
      if( isLexicographicallyGreater( i, real(-1)/d[i], r, real(-1)/d[r]) )
         r = i;
   }

   size_t s = 0;
   size_t pivot_steps = 0;

   while( true )
   {
      // Perform pivot step
      pivot( r, s );

      // Printing the annotated tableau
      //pe_LOG_DEBUG_SECTION( log ) {
      //   printTableau( log );
      //}

      // Finish if z0 blocked the driving variable
      if( nonbasics_[s] == 0 ) {
         lcp.x_ = real( 0 );
         for( size_t i=0; i<n; ++i ) {
            if( basics_[i] > 0 )
               lcp.x_[basics_[i]-1] = Q_(i,0);
         }

         return true;
      }

      // Find complement which will be driven next
      ptrdiff_t variable = -nonbasics_[s];
      s = inf;
      for( size_t i=0; i<=n; ++i ) {
         if( nonbasics_[i] == variable ) {
            s = i;
            break;
         }
      }

      pe_INTERNAL_ASSERT( s != inf, "No complement found" );

      pe_LOG_DEBUG_SECTION( log ) {
         if( nonbasics_[s] < 0 )
            log << "         w" << -nonbasics_[s] << " is new driving variable in column " << s;
         else
            log << "         z" <<  nonbasics_[s] << " is new driving variable in column " << s;
      }

      // Determination of the lexicographically smallest blocking variable
      r = inf;
      for( size_t i=0; i<n; ++i ) {
         if( M_(i,s) < -accuracy ) {  // < 0
            r = i;
            break;
         }
      }

      if( r == inf ) {
         // Driving variable is unblocked
         lcp.x_ = real( 0 );
         for( size_t i=0; i<n; ++i ) {
            if( basics_[i] > 0 )
               lcp.x_[basics_[i]-1] = Q_(i,0);
         }

         return false;
      }

      for( size_t i=r+1; i<n; ++i ) {
         if( M_(i,s) > -accuracy )  // >= 0
            continue;

         if( isLexicographicallyLess( i, real( -1 )/M_(i,s), r, real( -1 )/M_(r,s) ) )
            r = i;
      }

      ++pivot_steps;

      if( pivot_steps > 10*n ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << pe_YELLOW << "      WARNING: The LCP could not be solved with 10n pivot steps." << pe_OLDCOLOR;
         }
         return false;
      }
   }

   return false;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * \param v TODO
 * \return TODO
 *
 * TODO < 0
 */
bool Lemke::isComponentwiseNonnegative( const VecN& v ) const
{
   for( size_t i=0; i<v.size(); ++i ) {
      if( v[i] < -accuracy )  // < 0
         return false;
   }
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param v TODO
 * \return TODO
 *
 * TODO <= 0
 */
bool Lemke::isComponentwisePositive( const VecN& v ) const
{
   for( size_t i=0; i<v.size(); ++i ) {
      if( v[i] < accuracy )  // <= 0
         return false;
   }
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i1 TODO
 * \param f1 TODO
 * \param i2 TODO
 * \param f2 TODO
 * \return TODO
 *
 * TODO
 */
bool Lemke::isLexicographicallyLess( size_t i1, real f1, size_t i2, real f2 ) const
{
   for( size_t j=0; j<Q_.columns(); ++j )
   {
      if( Q_(i1,j) * f1 < Q_(i2,j) * f2 - real(accuracy) )
         return true;
      else if( Q_(i1,j) * f1 > Q_(i2,j) * f2 + real(accuracy) )
         return false;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i1 TODO
 * \param f1 TODO
 * \param i2 TODO
 * \param f2 TODO
 * \return TODO
 *
 * TODO
 */
bool Lemke::isLexicographicallyGreater( size_t i1, real f1, size_t i2, real f2 ) const
{
   for( size_t j = 0; j < Q_.columns(); ++j )
   {
      if( Q_(i1,j) * f1 > Q_(i2,j) * f2 + real(accuracy) )
         return true;
      else if( Q_(i1,j) * f1 < Q_(i2,j) * f2 - real(accuracy) )
         return false;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param block TODO
 * \param drive TODO
 * \return void
 *
 * TODO
 */
void Lemke::pivot( size_t block, size_t drive )
{
   using boost::format;

   size_t r( block );
   size_t s( drive );

   pe_INTERNAL_ASSERT( r < basics_.size() && s <= basics_.size(), "Invalid preconditions for pivot step" );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "         pivot step <"
          << ( basics_[r] < 0 ? "w" : "z" )
          << ( basics_[r] < 0 ? -basics_[r] : basics_[r] ) << ", "
          << ( nonbasics_[s] < 0 ? "w" : "z" )
          << ( nonbasics_[s] < 0 ? -nonbasics_[s] : nonbasics_[s] ) << ">\n";
   }

   real invPivot = real( 1 ) / M_(r,s);

   for( size_t i=0; i<M_.rows(); ++i )
   {
      if( i == r ) continue;

      for( size_t j=0; j<Q_.columns(); ++j ) {
         Q_(i,j) -= Q_(r,j) * M_(i,s) * invPivot;
      }

      for( size_t j=0; j<M_.columns(); ++j )
      {
         if( j == s ) continue;
         M_(i,j) -= M_(i,s) * ( M_(r,j) * invPivot );
      }

      M_(i,s) *= invPivot;
   }

   for( size_t j=0; j<Q_.columns(); ++j ) {
      Q_(r,j) = -Q_(r,j) * invPivot;
   }

   for( size_t j=0; j<M_.columns(); ++j )
   {
      if( j == s )
         M_(r,j) = invPivot;
      else
         M_(r,j) = -M_(r,j) * invPivot;
   }

   // Swap the blocking and driving variables
   std::swap( basics_[r], nonbasics_[s] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \return TODO
 *
 * TODO
 */
void Lemke::printTableau( std::ostream& os ) const
{
   using boost::format;

   const size_t n( basics_.size() );

   // Printing the header
   os << "      ";
   for( size_t j=0; j<Q_.columns(); ++j ) {
      if( j == 0 )
         os << format( " %-7d " ) % 1;
      else
         os << format( " x%-6d " ) % j;
   }
   os << "  ";
   for( size_t j=0; j<M_.columns(); ++j ) {
      if( nonbasics_[j] < 0 )
         os << format( " w%-6d " ) % -nonbasics_[j];
      else
         os << format( " z%-6d " ) % nonbasics_[j];
   }
   os << "\n";

   // Printing the table border
   os << "     +-";
   for( size_t j=0; j<Q_.columns(); ++j ) {
      os << "---------";
   }
   os << "+-";
   for( size_t j=0; j<M_.columns(); ++j ) {
      os << "---------";
   }
   os << "+\n";

   for( size_t i=0; i<n; ++i ) {
      if( basics_[i] < 0 )
         os << format( " w%-2d | " ) % -basics_[i];
      else
         os << format( " z%-2d | " ) % basics_[i];

      for( size_t j=0; j<Q_.columns(); ++j ) {
         os << format( "%-8.2d " ) % Q_(i, j);
      }
      os << "| ";

      for( size_t j=0; j<M_.columns(); ++j ) {
         os << format( "%-8.2d " ) % M_(i, j);
      }
      os << "|\n";
   }

   os << "     +-";
   for( size_t j=0; j<Q_.columns(); ++j ) {
      os << "---------";
   }
   os << "+-";
   for( size_t j=0; j<M_.columns(); ++j ) {
      os << "---------";
   }
   os << "+\n\n";
}
//*************************************************************************************************

} // namespace solvers

} // namespace pe
