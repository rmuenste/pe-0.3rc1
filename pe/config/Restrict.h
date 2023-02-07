//=================================================================================================
/*!
 *  \file pe/config/Restrict.h
 *  \brief Configuration of the restrict policy of the physics engine
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
/*!\brief Compilation switch for C99 restrict keyword.
 * \ingroup config
 *
 * This compilation switch enables/disables the C99 restrict keyword.
 *
 * Possible settings for the C99 restrict switch:
 *  - Deactivated: \b 0
 *  - Activated  : \b 1
 */
#define pe_USE_RESTRICT 1
//*************************************************************************************************
