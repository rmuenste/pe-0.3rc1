# FindGMP.cmake
# Try to find the GMP library
# https://gmplib.org/
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(GMP 6.0.0)
# to require version 6.0.0 or newer of GMP.
#
# Once done this will define
#  GMP_FOUND - System has GMP
#  GMP_INCLUDE_DIRS - The GMP include directory
#  GMP_LIBRARIES - The libraries needed to use GMP
#  GMP_DEFINITIONS - Compiler switches required for using GMP
#  GMP_VERSION - GMP version

find_path(GMP_INCLUDE_DIR NAMES gmp.h
          PATHS ${GMP_ROOT_DIR}/include
                /usr/include
                /usr/local/include
                /opt/local/include
                $ENV{GMP_DIR}/include)

find_library(GMP_LIBRARY NAMES gmp libgmp
             PATHS ${GMP_ROOT_DIR}/lib
                   /usr/lib
                   /usr/local/lib
                   /opt/local/lib
                   $ENV{GMP_DIR}/lib)

if(GMP_INCLUDE_DIR AND EXISTS "${GMP_INCLUDE_DIR}/gmp.h")
  file(STRINGS "${GMP_INCLUDE_DIR}/gmp.h" gmp_version_str 
       REGEX "^#define[\t ]+__GNU_MP_VERSION[\t ]+([0-9]+)")
  file(STRINGS "${GMP_INCLUDE_DIR}/gmp.h" gmp_version_str_minor 
       REGEX "^#define[\t ]+__GNU_MP_VERSION_MINOR[\t ]+([0-9]+)")
  file(STRINGS "${GMP_INCLUDE_DIR}/gmp.h" gmp_version_str_patchlevel 
       REGEX "^#define[\t ]+__GNU_MP_VERSION_PATCHLEVEL[\t ]+([0-9]+)")
  string(REGEX REPLACE "^#define[\t ]+__GNU_MP_VERSION[\t ]+([0-9]+)" "\\1" GMP_VERSION_MAJOR "${gmp_version_str}")
  string(REGEX REPLACE "^#define[\t ]+__GNU_MP_VERSION_MINOR[\t ]+([0-9]+)" "\\1" GMP_VERSION_MINOR "${gmp_version_str_minor}")
  string(REGEX REPLACE "^#define[\t ]+__GNU_MP_VERSION_PATCHLEVEL[\t ]+([0-9]+)" "\\1" GMP_VERSION_PATCH "${gmp_version_str_patchlevel}")
  set(GMP_VERSION "${GMP_VERSION_MAJOR}.${GMP_VERSION_MINOR}.${GMP_VERSION_PATCH}")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set GMP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(GMP
                                  REQUIRED_VARS GMP_LIBRARY GMP_INCLUDE_DIR
                                  VERSION_VAR GMP_VERSION)

mark_as_advanced(GMP_INCLUDE_DIR GMP_LIBRARY)

if(GMP_FOUND)
  set(GMP_LIBRARIES ${GMP_LIBRARY})
  set(GMP_INCLUDE_DIRS ${GMP_INCLUDE_DIR})
  set(GMP_DEFINITIONS "")
endif()