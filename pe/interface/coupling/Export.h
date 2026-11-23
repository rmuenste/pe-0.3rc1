//=================================================================================================
/*!
 *  \file pe/interface/coupling/Export.h
 *  \brief Symbol visibility of the coupling API (PE_COUPLING_API)
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
 *
 *  The one macro the coupling headers define besides their include guards. pe's shared library
 *  is built with -fvisibility=hidden, so the coupling API marks its own symbols visible. The
 *  macro depends on no pe header and not on pe's config.h, so the coupling headers stay usable
 *  in any translation unit. On Windows it is empty: pe's shared-library build is GCC/Clang only
 *  (experimental), and the static library needs no attribute.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_COUPLING_EXPORT_H_
#define _PE_INTERFACE_COUPLING_EXPORT_H_


//*************************************************************************************************
/*!\brief Marks a coupling API symbol as exported from pe's shared library.
 * \ingroup coupling
 */
#if defined( _WIN32 ) || defined( __CYGWIN__ )
#  define PE_COUPLING_API
#elif defined( __GNUC__ ) || defined( __clang__ )
#  define PE_COUPLING_API __attribute__(( visibility( "default" ) ))
#else
#  define PE_COUPLING_API
#endif
//*************************************************************************************************

#endif
