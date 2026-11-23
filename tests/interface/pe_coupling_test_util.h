//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_test_util.h
 *  \brief Minimal check helpers shared by the pe::coupling tests
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_TESTS_INTERFACE_PE_COUPLING_TEST_UTIL_H_
#define _PE_TESTS_INTERFACE_PE_COUPLING_TEST_UTIL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <pe/interface/coupling/Types.h>


namespace coupling_test {

//*************************************************************************************************
/*!\brief Number of failed checks of the running test.
 */
inline int& failures()
{
   static int count( 0 );
   return count;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Records a failed check.
 */
inline void expect( bool ok, const std::string& what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what.c_str() );
      ++failures();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks that a callable throws an exception of type E whose message contains all of
 *        the given fragments.
 */
template< typename E, typename F >
inline void expectThrows( F f, const std::string& what, const char* fragment1 = nullptr,
                          const char* fragment2 = nullptr )
{
   try {
      f();
   }
   catch( const E& ex ) {
      const std::string msg( ex.what() );
      if( fragment1 && msg.find( fragment1 ) == std::string::npos )
         expect( false, what + ": message lacks '" + fragment1 + "': " + msg );
      if( fragment2 && msg.find( fragment2 ) == std::string::npos )
         expect( false, what + ": message lacks '" + fragment2 + "': " + msg );
      return;
   }
   catch( const std::exception& ex ) {
      expect( false, what + ": threw an exception of the wrong type: " + ex.what() );
      return;
   }
   expect( false, what + ": did not throw" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The bit pattern of a double.
 */
inline std::uint64_t bits( double x )
{
   std::uint64_t b( 0 );
   std::memcpy( &b, &x, sizeof(b) );
   return b;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether two arrays of doubles are bitwise identical.
 */
template< typename Array >
inline bool sameBits( const Array& a, const Array& b )
{
   for( std::size_t i=0; i<a.size(); ++i )
      if( bits( a[i] ) != bits( b[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prints the verdict and returns the process exit code.
 */
inline int report( const char* name )
{
   if( failures() )
      std::printf( "%s: FAILED (%d)\n", name, failures() );
   else
      std::printf( "%s: PASSED\n", name );
   return failures() ? 1 : 0;
}
//*************************************************************************************************

} // namespace coupling_test

#endif
