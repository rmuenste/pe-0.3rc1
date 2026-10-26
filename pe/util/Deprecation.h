//=================================================================================================
/*!
 *  \file pe/util/Deprecation.h
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
 *
 *  A deprecation in pe goes through two stages (see doc/technical-notes/deprecations.md):
 *
 *  1. The old input or behaviour still works, and reportDeprecated() prints a warning that
 *     names the replacement.
 *  2. The old input or behaviour is removed; the code that called reportDeprecated() stops
 *     with an error instead.
 *
 *  With setDeprecationsAsErrors( true ) (json key deprecationsAsErrors_), stage 2 can be tried
 *  out before the old behaviour is removed: reportDeprecated() then returns false and the caller
 *  stops.
 */
//=================================================================================================

#ifndef _PE_UTIL_DEPRECATION_H_
#define _PE_UTIL_DEPRECATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>


namespace pe {

//=================================================================================================
//
//  DEPRECATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Deprecation functions */
//@{
void setDeprecationsAsErrors( bool asErrors );
bool deprecationsAsErrors();
bool reportDeprecated( const std::string& id, const std::string& message, bool console=true );
//@}
//*************************************************************************************************

} // namespace pe

#endif
