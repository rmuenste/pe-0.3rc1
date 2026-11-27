//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_gather_layout_test.cpp
 *  \brief The layout of the coupling API's variable-length gathers (64-bit sizes, MPI int counts)
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  detail::gatherLayout() computes counts and displacements in 64 bits and refuses a layout
 *  that MPI's int counts cannot address (a count or the total above INT_MAX), before anything
 *  is allocated or sent. Only sizes are computed here; nothing is allocated.
 */
//=================================================================================================

#include <climits>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <src/interface/coupling/GatherLayout.h>

#include "pe_coupling_test_util.h"

using coupling_test::expect;

int main()
{
   namespace pd = pe::coupling::detail;
   std::vector<int> counts, displs;

   // small layouts: prefix sums
   expect( pd::gatherLayout( { 3, 0, 5, 7 }, counts, displs ).empty() &&
           counts == std::vector<int>{ 3, 0, 5, 7 } && displs == std::vector<int>{ 0, 3, 3, 8 },
           "counts and displacements of a small layout" );
   expect( pd::gatherLayout( {}, counts, displs ).empty() && counts.empty(), "an empty layout" );

   // the review case: six processes with 768 MiB each (each count fits, the total does not)
   const std::uint64_t mib768( 768ull << 20 );
   const std::string six( pd::gatherLayout( std::vector<std::uint64_t>( 6, mib768 ), counts, displs ) );
   expect( !six.empty() && six.find( "exceed" ) != std::string::npos,
           "six times 768 MiB is refused: " + six );

   // the boundaries: a total of exactly INT_MAX fits, one byte more does not
   const std::uint64_t max( static_cast<std::uint64_t>( INT_MAX ) );
   expect( pd::gatherLayout( { max }, counts, displs ).empty() && counts[0] == INT_MAX,
           "a single count of INT_MAX fits" );
   expect( !pd::gatherLayout( { max + 1 }, counts, displs ).empty(),
           "a single count above INT_MAX is refused" );
   expect( pd::gatherLayout( { max - 10, 10 }, counts, displs ).empty() &&
           displs[1] == INT_MAX - 10, "a total of exactly INT_MAX fits" );
   expect( !pd::gatherLayout( { max - 10, 11 }, counts, displs ).empty(),
           "a total of INT_MAX + 1 is refused" );
   expect( !pd::gatherLayout( { 1, ~std::uint64_t( 0 ) }, counts, displs ).empty(),
           "a count that would wrap a 64-bit sum is refused" );

   const int failures( coupling_test::failures() );
   std::printf( "pe_coupling_gather_layout_test: %s (%d failed checks)\n",
                failures ? "FAILED" : "PASSED", failures );
   return failures ? 1 : 0;
}
