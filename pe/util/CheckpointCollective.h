#ifndef _PE_UTIL_CHECKPOINTCOLLECTIVE_H_
#define _PE_UTIL_CHECKPOINTCOLLECTIVE_H_

#include <pe/core/MPISettings.h>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

namespace pe {
namespace checkpoint_detail {

// Each phase contains either local work or one collective, never local work followed by a
// collective. Agreement must precede the next collective, including MPI_File_close on failure.
// Keep the diagnostic buffer bounded so capturing/broadcasting an error needs no allocation.
template<typename Operation>
void phase( const char* path, Operation operation )
{
#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      std::exception_ptr failure;
      char message[2048] = {};
      try { operation(); }
      catch( const std::exception& error ) {
         failure = std::current_exception();
         std::snprintf( message, sizeof(message), "%s", error.what() );
      }
      catch( ... ) {
         failure = std::current_exception();
         std::snprintf( message, sizeof(message), "Unknown exception" );
      }
      const int sentinel = MPISettings::size();
      int localRank = failure ? MPISettings::rank() : sentinel;
      int failingRank = sentinel;
      MPI_Allreduce( &localRank, &failingRank, 1, MPI_INT, MPI_MIN, MPISettings::comm() );
      if( failingRank != sentinel ) {
         MPI_Bcast( message, sizeof(message), MPI_CHAR, failingRank, MPISettings::comm() );
         // Broadcast the path too: it can differ across ranks during a failed local preflight.
         char failingPath[2048] = {};
         if( MPISettings::rank() == failingRank )
            std::snprintf( failingPath, sizeof(failingPath), "%s", path );
         MPI_Bcast( failingPath, sizeof(failingPath), MPI_CHAR, failingRank, MPISettings::comm() );
         throw std::runtime_error( "Checkpoint I/O failed on rank " + std::to_string(failingRank) +
                                   " for '" + failingPath + "': " + message );
      }
      return;
   }
#else
   (void)path;
#endif
   // Preserve the original exception type and avoid collective overhead for serial callers.
   operation();
}

#if HAVE_MPI
inline void mpiCheck( int result, const char* operation )
{
   if( result != MPI_SUCCESS ) {
      char message[MPI_MAX_ERROR_STRING];
      int length = 0;
      MPI_Error_string( result, message, &length );
      throw std::runtime_error( std::string(operation) + ": " + std::string(message, length) );
   }
}

inline void requireTransfer( int result, MPI_Status& status, size_t bytes,
                             const char* operation = "MPI file read" )
{
   mpiCheck( result, operation );
   int count = 0;
   mpiCheck( MPI_Get_count( &status, MPI_BYTE, &count ), "MPI_Get_count" );
   if( count < 0 || static_cast<size_t>(count) != bytes )
      throw std::runtime_error( "Incomplete MPI file transfer." );
}
#endif

} // namespace checkpoint_detail
} // namespace pe
#endif
