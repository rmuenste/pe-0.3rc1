//=================================================================================================
/*!
 *  \file src/interface/coupling/GatherLayout.h
 *  \brief The layout of a variable-length gather in MPI's int counts (coupling API, internal)
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 */
//=================================================================================================

#ifndef _PE_SRC_INTERFACE_COUPLING_GATHERLAYOUT_H_
#define _PE_SRC_INTERFACE_COUPLING_GATHERLAYOUT_H_

#include <climits>
#include <cstdint>
#include <string>
#include <vector>

namespace pe {

namespace coupling {

namespace detail {

//*************************************************************************************************
/*!\brief Computes the counts and displacements of a variable-length gather (MPI_Gatherv,
 *        MPI_Allgatherv) from the byte counts of all processes, in 64 bits.
 *
 * \param bytes The number of bytes each process contributes (the same vector on every process).
 * \param counts Output: the counts as MPI ints.
 * \param displs Output: the displacements as MPI ints.
 * \return An empty string, or why the layout does not fit MPI's int counts and displacements
 *         (a count, or the total, above INT_MAX): then nothing may be allocated or transferred.
 *         The result depends only on \a bytes, so every process decides the same.
 */
inline std::string gatherLayout( const std::vector<std::uint64_t>& bytes, std::vector<int>& counts,
                                 std::vector<int>& displs )
{
   counts.assign( bytes.size(), 0 );
   displs.assign( bytes.size(), 0 );
   std::uint64_t total( 0 );
   const std::uint64_t limit( static_cast<std::uint64_t>( INT_MAX ) );
   for( std::size_t r=0; r<bytes.size(); ++r ) {
      if( bytes[r] > limit || total > limit - bytes[r] )
         return "the gathered data (" + std::to_string( total ) + " bytes and " +
                std::to_string( bytes[r] ) + " more from process " + std::to_string( r ) +
                ") exceed what an MPI gather with int counts can address (" +
                std::to_string( limit ) + " bytes)";
      counts[r] = static_cast<int>( bytes[r] );
      displs[r] = static_cast<int>( total );
      total += bytes[r];
   }
   return std::string();
}
//*************************************************************************************************

} // namespace detail

} // namespace coupling

} // namespace pe

#endif
