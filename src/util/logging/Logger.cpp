//=================================================================================================
/*!
 *  \file src/util/logging/Logger.cpp
 *  \brief Source file for the Logger class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#if HAVE_MPI
#  include <mpi.h>
#endif
#include <pe/util/logging/Logger.h>


namespace pe {

namespace logging {

//=================================================================================================
//
//  STATIC MEMBER INITIALIZATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Static initialization of customRank_.
 *
 * The default value of -1 indicates that no custom rank has been set, and the logger will
 * use the standard MPI rank detection (if MPI is available) or create a single log file
 * for serial execution.
 */
int Logger::customRank_ = -1;
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Logger class.
 */
Logger::Logger()
   : Singleton<Logger,SystemClock>()  // Initialization of the Singleton base object
   , mutex_()                         // Synchronization mutex for thread-parallel logging
   , log_  ()                         // The log file
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Logger class.
 *
 * The destructor of the Logger class writes the bottom line of the log file and closes
 * the file.
 */
Logger::~Logger()
{
   if( log_.is_open() )
   {
      const std::time_t t = theSystemClock()->now();
      std::tm* localTime;
      char c[100];

      // Calculation of the local time
      localTime = std::localtime( &t );

      // Construction of the filename and time string
      std::strftime( c, 100, "%A, %d.%B %Y, %H:%M:%S", localTime );

      // Writing the bottom line
      const size_t length( std::strlen( c ) );
      log_ << "\n--LOG END, " << std::setw(length) << c
         << std::setw(90-length) << std::setfill('-') << "\n\n" << std::endl;

      // Closing the log file
      log_.close();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  CONFIGURATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets a custom rank for the log filename.
 *
 * \param rank The custom rank to use in the log filename.
 * \return void
 *
 * This function allows setting a custom rank that will be used in the log filename instead of
 * the MPI rank from MPI_COMM_WORLD. This is particularly useful in serial PE mode where each
 * CFD domain runs an independent serial PE instance but needs unique log files.
 *
 * The custom rank should be set BEFORE any logging occurs (i.e., before the first log message
 * is written). Once the log file is opened, changing the rank has no effect.
 *
 * Example usage in serial PE mode:
 * \code
 * // In setupParticleBenchSerial(), called with CFD rank from Fortran
 * pe::logging::Logger::setCustomRank(cfd_rank);
 * \endcode
 *
 * Setting rank to -1 disables the custom rank and reverts to standard behavior.
 */
void Logger::setCustomRank( int rank )
{
   customRank_ = rank;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Opens and initializes the log file.
 *
 * \return void
 *
 * This function is responsible for opening the log file and writing the header line. In case
 * of a non-MPI-parallel simulation the function creates the log file 'pe.log', which contains
 * all logging information from all logging levels. In case of a MPI parallel simulation, each
 * process creates his own individual log file called 'peX.log', where 'X' is replaced by the
 * according rank the process has in the MPI_COMM_WORLD communicator.
 */
void Logger::openLogFile()
{
   // Creating the filename
   std::ostringstream filename;
   filename << "pe";

   // Check for custom rank first (used in serial PE mode with CFD MPI rank)
   if( customRank_ >= 0 ) {
      filename << customRank_;
   }
#if HAVE_MPI
   else {
      // Standard MPI mode - use MPI_COMM_WORLD rank
      int initialized( 0 );
      MPI_Initialized( &initialized );

      if( initialized ) {
         int rank( 0 );
         MPI_Comm_rank( MPI_COMM_WORLD, &rank );  // Estimating the rank of this process
         filename << rank;
      }
   }
#endif

   filename << ".log";

   // Creating the time and date string for the header line
   const std::time_t t = theSystemClock()->start();
   std::tm* localTime;
   char c[100];

   localTime = std::localtime( &t );
   strftime( c, 100, "%A, %d.%B %Y, %H:%M:%S", localTime );

   // Opening the log file
   log_.open( filename.str().c_str(), std::ofstream::out | std::ofstream::trunc );
   if( !log_.is_open() ) {
      std::ostringstream oss;
      oss << " Error opening log file '" << filename.str() << "' !\n";
      throw std::runtime_error( oss.str() );
   }

   // Writing the header line
   const size_t length( std::strlen( c ) );
   log_ << "\n\n--LOG BEGIN, " << std::setw(length) << c
        << std::setw(88-length) << std::setfill('-') << "\n\n" << std::endl;
}
//*************************************************************************************************

} // namespace logging

} // namespace pe
