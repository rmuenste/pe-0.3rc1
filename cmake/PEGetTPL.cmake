#==================================================================================================
#
#  pe_get_tpl(): third-party libraries from the system or from a verified download
#
#  Copyright (C) 2026 Raphael Münster
#
#  Adapted from FEAT3, build_system/cmake/FEATGetTPL.cmake and FindPackageOverrideHelper.cmake,
#  Copyright (C) the FEAT group, GPL-3.
#
#  This file is part of pe.
#
#  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
#  General Public License as published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with pe. If not,
#  see <http://www.gnu.org/licenses/>.
#
#==================================================================================================
#
#  pe_get_tpl(
#     PACKAGE_NAME  <name>                 find_package() name, also the FetchContent name
#     VERSION       <version>              version of the pinned archive; minimum version for
#                                          find_package() unless FIND_ANY_VERSION is given
#     URLS          <url> [<url>...]       archive URLs, upstream first, then mirrors
#     URL_HASH      <ALGO>=<hex>           hash of the archive, e.g. SHA256=...
#     [TARGET       <target>]              target the package provides (e.g. Eigen3::Eigen)
#     [SOURCE_SUBDIR <dir>]                as for FetchContent_Declare(); a directory that does
#                                          not exist keeps the TPL's own CMakeLists from running
#     [CONFIG]                             find_package() in config mode
#     [FIND_ANY_VERSION]                   find_package() without a version
#     [COMPONENTS   <component>...]        forwarded to find_package()
#     [FIND_PACKAGE_ARGS <arg>...]         further find_package() arguments, e.g. HINTS
#     [OVERRIDE_FIND_PACKAGE]              later find_package(<name>) calls use the download
#  )
#
#  Order of the sources (see doc/technical-notes/build-with-cmake.md, "Third-party libraries"):
#
#  1. TARGET already exists (e.g. defined by a parent project): it is used, nothing else is done.
#  2. If PE_PREFER_EXTERNAL_TPL or PE_NO_EXTERNAL_DOWNLOAD is ON, or <name>_DIR is set (other
#     than to FetchContent's redirect directory): find_package(). If TARGET is given, the
#     package is only used if it defines TARGET.
#  3. FETCHCONTENT_SOURCE_DIR_<NAME> (upper case) set: that local source tree is used.
#  4. The archive <name>-<version>-<file of the first URL> in PE_TPL_CACHE_DIRECTORY (or, if
#     that is empty, in <build>/_deps/pe-tpl-archives) is used if its hash matches. Otherwise
#     it is downloaded there first: the URLs are tried in order, and a download that fails or
#     whose hash does not match is discarded and the next URL is tried (a wrong hash with a
#     warning). A file with a wrong hash is never used. If no URL delivers the pinned file,
#     configuration stops. FetchContent then extracts the verified archive and checks its hash
#     again.
#
#  The archives are downloaded with file(DOWNLOAD) and checked with file(<ALGO>) rather than
#  with file(DOWNLOAD ... EXPECTED_HASH): a mismatch there is a configure error that leaves the
#  file in place, so the next URL could not be tried. FetchContent's own URL list is not used
#  either: it reports a hash mismatch only as "Each download failed!" and rejects file:// URLs
#  in a list.
#
#  With PE_NO_EXTERNAL_DOWNLOAD ON, steps 1-4 work as above, but if they do not provide the
#  package, configuration stops instead of downloading.
#
#  After the call, <name>_FOUND is TRUE in the caller's scope if the package came from step 1 or
#  2 (also <name>_VERSION and <name>_INCLUDE_DIRS if find_package() set them). Otherwise the
#  source tree is in <lowercase name>_SOURCE_DIR, and the caller creates TARGET from it if
#  SOURCE_SUBDIR kept the TPL's CMakeLists from doing so.
#
#  A TPL whose own CMakeLists does run is configured without warnings (-w, /W0 for MSVC) and,
#  with CMake >= 3.25, with SYSTEM include directories. The flags are only changed inside this
#  function, so they never reach pe's own targets.
#
#==================================================================================================

include_guard( GLOBAL )
include( FetchContent )


#**************************************************************************************************
# Writes the config-version file that find_package(<package> <version>) checks once
# OVERRIDE_FIND_PACKAGE redirects it to the downloaded copy. Without it, FetchContent writes a
# version file that accepts any requested version.
#
# Adapted from FEAT3's find_package_override_helper(); FEAT's <name>-extra.cmake files are not
# needed, because pe creates the targets of downloaded TPLs as global targets.
#**************************************************************************************************
function( pe_tpl_write_redirect_version_file package version compat )
   include( CMakePackageConfigHelpers )
   string( TOLOWER "${package}" lower )
   write_basic_package_version_file(
      "${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/${lower}-config-version.cmake"
      VERSION ${version} COMPATIBILITY ${compat} )
endfunction()
#**************************************************************************************************




#**************************************************************************************************
# Downloads the archive from the first URL in ARGN that delivers a file with the expected hash and
# moves it to <archive>. The file is downloaded to a temporary name and only renamed after the
# hash check, so an interrupted or wrong download never ends up in the cache.
#**************************************************************************************************
function( pe_tpl_download name archive algo hash )
   set( failures "" )
   foreach( url IN LISTS ARGN )
      string( RANDOM LENGTH 8 suffix )
      set( part "${archive}.part-${suffix}" )
      message( STATUS "pe TPL ${name}: downloading ${url}" )
      file( DOWNLOAD "${url}" "${part}" STATUS status INACTIVITY_TIMEOUT 60 TIMEOUT 900 )
      list( GET status 0 code )
      list( GET status 1 text )
      if( NOT code EQUAL 0 )
         file( REMOVE "${part}" )
         string( APPEND failures "\n   ${url}: ${text} (status ${code})" )
         message( STATUS "pe TPL ${name}: download failed: ${text} (status ${code})" )
         continue()
      endif()
      file( ${algo} "${part}" actual )
      if( NOT actual STREQUAL hash )
         file( REMOVE "${part}" )
         string( APPEND failures "\n   ${url}: ${algo} mismatch, got ${actual}" )
         message( WARNING
            "pe TPL ${name}: ${url} delivered a file with ${algo}=${actual}, but ${algo}=${hash} "
            "is pinned. The file was deleted and is not used. If the next URL fails as well, "
            "the archive has changed upstream: verify the new archive and update the pin with "
            "tools/bump_tpl_pin.py." )
         continue()
      endif()
      file( RENAME "${part}" "${archive}" )
      message( STATUS "pe TPL ${name}: ${algo} verified, stored ${archive}" )
      return()
   endforeach()
   message( FATAL_ERROR
      "pe TPL ${name}: no URL delivered the pinned archive (${algo}=${hash}):${failures}\n"
      "See \"Third-party libraries\" in doc/technical-notes/build-with-cmake.md for builds "
      "without network access and for moving a pin." )
endfunction()
#**************************************************************************************************




#**************************************************************************************************
# Provides a third-party library; see the description at the top of this file.
#**************************************************************************************************
function( pe_get_tpl )
   cmake_parse_arguments( PARSE_ARGV 0 TPL
      "CONFIG;FIND_ANY_VERSION;OVERRIDE_FIND_PACKAGE"
      "PACKAGE_NAME;VERSION;URL_HASH;SOURCE_SUBDIR;TARGET"
      "URLS;COMPONENTS;FIND_PACKAGE_ARGS" )

   if( TPL_UNPARSED_ARGUMENTS )
      message( FATAL_ERROR "pe_get_tpl: unknown arguments ${TPL_UNPARSED_ARGUMENTS}" )
   endif()
   foreach( required PACKAGE_NAME VERSION URLS URL_HASH )
      if( NOT TPL_${required} )
         message( FATAL_ERROR "pe_get_tpl: ${required} is missing" )
      endif()
   endforeach()

   set( name "${TPL_PACKAGE_NAME}" )
   string( TOLOWER "${name}" lower )
   string( TOUPPER "${name}" upper )

   # The hash algorithm is taken from the URL_HASH prefix (FEAT3 hard-codes MD5 here).
   if( NOT TPL_URL_HASH MATCHES "^([A-Za-z0-9_]+)=([0-9A-Fa-f]+)$" )
      message( FATAL_ERROR
         "pe_get_tpl(${name}): URL_HASH must be <ALGO>=<hex>, not ${TPL_URL_HASH}" )
   endif()
   string( TOUPPER "${CMAKE_MATCH_1}" algo )
   string( TOLOWER "${CMAKE_MATCH_2}" hash )
   set( algos MD5 SHA1 SHA224 SHA256 SHA384 SHA512 SHA3_224 SHA3_256 SHA3_384 SHA3_512 )
   if( NOT algo IN_LIST algos )
      message( FATAL_ERROR "pe_get_tpl(${name}): unsupported hash algorithm ${algo}" )
   endif()

   # 1. A target provided by someone else, e.g. a parent project that includes pe.
   if( TPL_TARGET AND TARGET ${TPL_TARGET} )
      message( STATUS "pe TPL ${name}: using the existing target ${TPL_TARGET}" )
      set( ${name}_FOUND TRUE PARENT_SCOPE )
      return()
   endif()

   # 2. An installed package. A <name>_DIR that points to FetchContent's redirect directory was
   # set by an earlier OVERRIDE_FIND_PACKAGE download, not by the user: it must not switch a
   # reconfigure with PE_PREFER_EXTERNAL_TPL=OFF over to an installed package.
   set( user_dir FALSE )
   if( ${name}_DIR )
      get_filename_component( dir "${${name}_DIR}" REALPATH )
      get_filename_component( redirects "${CMAKE_BINARY_DIR}/CMakeFiles/pkgRedirects" REALPATH )
      if( CMAKE_FIND_PACKAGE_REDIRECTS_DIR )
         get_filename_component( redirects "${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}" REALPATH )
      endif()
      if( NOT dir STREQUAL redirects )
         set( user_dir TRUE )
      endif()
   endif()
   if( PE_PREFER_EXTERNAL_TPL OR PE_NO_EXTERNAL_DOWNLOAD OR user_dir )
      set( args ${name} )
      if( NOT TPL_FIND_ANY_VERSION )
         list( APPEND args ${TPL_VERSION} )
      endif()
      list( APPEND args QUIET )
      if( TPL_CONFIG )
         list( APPEND args CONFIG )
      endif()
      if( TPL_COMPONENTS )
         list( APPEND args COMPONENTS ${TPL_COMPONENTS} )
      endif()
      list( APPEND args ${TPL_FIND_PACKAGE_ARGS} )
      find_package( ${args} )

      if( ${name}_FOUND AND TPL_TARGET AND NOT TARGET ${TPL_TARGET} )
         message( WARNING "pe TPL ${name}: the installed package in ${${name}_DIR} does not "
                          "define ${TPL_TARGET}; it is not used" )
         set( ${name}_FOUND FALSE )
      endif()
      if( ${name}_FOUND )
         message( STATUS "pe TPL ${name}: found version ${${name}_VERSION} in ${${name}_DIR}" )
         foreach( var FOUND VERSION INCLUDE_DIRS )
            set( ${name}_${var} "${${name}_${var}}" PARENT_SCOPE )
         endforeach()
         return()
      endif()
      if( TPL_FIND_ANY_VERSION )
         message( STATUS "pe TPL ${name}: not found on the system" )
      else()
         message( STATUS "pe TPL ${name}: version ${TPL_VERSION} or a compatible newer one "
                         "not found on the system" )
      endif()
   endif()

   # 3. and 4. The pinned archive.
   if( TPL_SOURCE_SUBDIR AND CMAKE_VERSION VERSION_LESS 3.18 )
      message( FATAL_ERROR "pe TPL ${name}: not installed, and using the downloaded archive needs "
                           "CMake 3.18 or newer (FetchContent SOURCE_SUBDIR)" )
   endif()
   set( urls ${TPL_URLS} )
   list( GET urls 0 first_url )
   get_filename_component( file "${first_url}" NAME )
   set( offline_hint
      "The pinned archive is ${first_url} (${algo}=${hash}). Without network access, either "
      "install ${name} ${TPL_VERSION} and set ${name}_DIR or CMAKE_PREFIX_PATH, or copy the "
      "archive to PE_TPL_CACHE_DIRECTORY as ${name}-${TPL_VERSION}-${file}, or set "
      "FETCHCONTENT_SOURCE_DIR_${upper} to an extracted source tree, or turn off the pe option "
      "that needs ${name}." )

   if( FETCHCONTENT_SOURCE_DIR_${upper} )
      message( STATUS "pe TPL ${name}: using the source tree "
                      "FETCHCONTENT_SOURCE_DIR_${upper}=${FETCHCONTENT_SOURCE_DIR_${upper}}" )
   else()
      # Without the shared cache, the archive is kept in the build directory, so the download
      # and its hash check work the same way.
      if( PE_TPL_CACHE_DIRECTORY )
         set( dir "${PE_TPL_CACHE_DIRECTORY}" )
      else()
         set( dir "${CMAKE_BINARY_DIR}/_deps/pe-tpl-archives" )
      endif()
      set( archive "${dir}/${name}-${TPL_VERSION}-${file}" )
      set( cached FALSE )
      if( EXISTS "${archive}" )
         file( ${algo} "${archive}" actual )
         if( actual STREQUAL hash )
            set( cached TRUE )
            message( STATUS "pe TPL ${name}: using the cached archive ${archive}" )
         else()
            message( WARNING "pe TPL ${name}: the cached ${archive} has ${algo}=${actual}, not "
                             "the pinned ${algo}=${hash}; it is not used" )
         endif()
      endif()
      if( NOT cached )
         if( PE_NO_EXTERNAL_DOWNLOAD )
            message( FATAL_ERROR "pe TPL ${name}: not found and PE_NO_EXTERNAL_DOWNLOAD is ON. "
                                 ${offline_hint} " The archive would be used from ${archive}." )
         endif()
         file( MAKE_DIRECTORY "${dir}" )
         pe_tpl_download( ${name} "${archive}" ${algo} ${hash} ${urls} )
      endif()
      set( urls "${archive}" )
   endif()

   cmake_policy( PUSH )
   if( POLICY CMP0135 )
      cmake_policy( SET CMP0135 NEW )   # extracted files get the extraction time stamp
   endif()
   if( POLICY CMP0168 )
      cmake_policy( SET CMP0168 NEW )   # populate directly, without a sub-build
   endif()

   set( args ${name} URL ${urls} URL_HASH ${algo}=${hash} UPDATE_DISCONNECTED ON )
   if( TPL_SOURCE_SUBDIR )
      list( APPEND args SOURCE_SUBDIR ${TPL_SOURCE_SUBDIR} )
   endif()
   if( NOT CMAKE_VERSION VERSION_LESS 3.25 )
      list( APPEND args SYSTEM )
   endif()
   if( TPL_OVERRIDE_FIND_PACKAGE AND NOT CMAKE_VERSION VERSION_LESS 3.24 )
      list( APPEND args OVERRIDE_FIND_PACKAGE )
      pe_tpl_write_redirect_version_file( ${name} ${TPL_VERSION} SameMajorVersion )
   endif()
   FetchContent_Declare( ${args} )

   # No warnings from a TPL's own CMake targets. These variables are local to this function and
   # are inherited by the TPL's directory scope only.
   if( MSVC )
      string( REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}"   )
      string( REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
      string( APPEND CMAKE_C_FLAGS   " /W0" )
      string( APPEND CMAKE_CXX_FLAGS " /W0" )
   else()
      string( APPEND CMAKE_C_FLAGS   " -w" )
      string( APPEND CMAKE_CXX_FLAGS " -w" )
   endif()

   FetchContent_MakeAvailable( ${name} )
   cmake_policy( POP )

   foreach( var SOURCE_DIR BINARY_DIR POPULATED )
      set( ${lower}_${var} "${${lower}_${var}}" PARENT_SCOPE )
   endforeach()
   set( ${name}_FOUND FALSE PARENT_SCOPE )
   message( STATUS "pe TPL ${name}: version ${TPL_VERSION} in ${${lower}_SOURCE_DIR}" )
endfunction()
#**************************************************************************************************
