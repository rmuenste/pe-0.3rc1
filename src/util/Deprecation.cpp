//=================================================================================================
/*!
 *  \file src/util/Deprecation.cpp
 *  \brief Reporting of deprecated input and behaviour at run time
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
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <set>
#include <string>
#include <pe/util/Deprecation.h>
#include <pe/util/logging/WarningSection.h>


namespace pe {

namespace {

//*************************************************************************************************
/*!\brief Whether reportDeprecated() treats a deprecation as an error.
 */
bool asErrors_( false );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The IDs reportDeprecated() has already warned about in this process.
 */
std::set<std::string>& reportedIds()
{
   static std::set<std::string> ids;
   return ids;
}
//*************************************************************************************************

} // namespace




//=================================================================================================
//
//  DEPRECATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets whether reportDeprecated() treats a deprecation as an error.
 *
 * \param asErrors \a true to treat every deprecation as an error, \a false to warn (default).
 * \return void
 *
 * SimulationConfig calls this when it reads the json key \c deprecationsAsErrors_.
 */
void setDeprecationsAsErrors( bool asErrors )
{
   asErrors_ = asErrors;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether reportDeprecated() treats a deprecation as an error.
 *
 * \return \a true if deprecations are errors, \a false if they are warnings.
 */
bool deprecationsAsErrors()
{
   return asErrors_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reports that deprecated input or behaviour is used.
 *
 * \param id Short, stable ID of the deprecation, e.g. \c "archimedes.position"; it is listed in
 *           doc/technical-notes/deprecations.md.
 * \param message What is deprecated and what replaces it.
 * \param console \a true to also print to \c std::cerr. Pass \a false on all but one process
 *                (e.g. rank 0), so the message appears once in the console output.
 * \return \a true if the caller may go on with the deprecated input or behaviour, \a false if
 *         deprecations are errors and the caller has to stop.
 *
 * The message goes to the pe log file of this process and, if \a console is \a true, to
 * \c std::cerr, with the prefix "PE DEPRECATED [<id>]". As a warning it is printed once per ID
 * and process; as an error it is printed every time. Not thread-safe: call it from setup code.
 *
 * \b Example:
   \code
   if( !reportDeprecated( "archimedes.position", "set domainBoundary_.position", rank == 0 ) )
      abortSetup( "deprecated input, see the PE DEPRECATED message" );
   \endcode
 */
bool reportDeprecated( const std::string& id, const std::string& message, bool console )
{
   if( asErrors_ ) {
      const std::string text( "PE DEPRECATED [" + id + "] (an error, because deprecations are "
                              "errors): " + message );
      pe_LOG_WARNING_SECTION( log ) {
         log << text << "\n";
      }
      if( console ) std::cerr << text << std::endl;
      return false;
   }

   if( !reportedIds().insert( id ).second )
      return true;

   const std::string text( "PE DEPRECATED [" + id + "]: " + message );
   pe_LOG_WARNING_SECTION( log ) {
      log << text << "\n";
   }
   if( console ) std::cerr << text << std::endl;
   return true;
}
//*************************************************************************************************

} // namespace pe
