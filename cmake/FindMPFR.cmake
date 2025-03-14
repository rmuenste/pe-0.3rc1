# FindMPFR.cmake
# Try to find the MPFR library
# https://www.mpfr.org/
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(MPFR 4.0.0)
# to require version 4.0.0 or newer of MPFR.
#
# Once done this will define
#  MPFR_FOUND - System has MPFR
#  MPFR_INCLUDE_DIRS - The MPFR include directory
#  MPFR_LIBRARIES - The libraries needed to use MPFR
#  MPFR_DEFINITIONS - Compiler switches required for using MPFR
#  MPFR_VERSION - MPFR version

# MPFR needs GMP
find_package(GMP REQUIRED)

find_path(MPFR_INCLUDE_DIR NAMES mpfr.h
          PATHS ${MPFR_ROOT_DIR}/include
                /usr/include
                /usr/local/include
                /opt/local/include
                $ENV{MPFR_DIR}/include)

find_library(MPFR_LIBRARY NAMES mpfr libmpfr
             PATHS ${MPFR_ROOT_DIR}/lib
                   /usr/lib
                   /usr/local/lib
                   /opt/local/lib
                   $ENV{MPFR_DIR}/lib)

if(MPFR_INCLUDE_DIR AND EXISTS "${MPFR_INCLUDE_DIR}/mpfr.h")
  file(STRINGS "${MPFR_INCLUDE_DIR}/mpfr.h" mpfr_version_str 
       REGEX "^#define[\t ]+MPFR_VERSION_MAJOR[\t ]+([0-9]+)")
  file(STRINGS "${MPFR_INCLUDE_DIR}/mpfr.h" mpfr_version_str_minor 
       REGEX "^#define[\t ]+MPFR_VERSION_MINOR[\t ]+([0-9]+)")
  file(STRINGS "${MPFR_INCLUDE_DIR}/mpfr.h" mpfr_version_str_patchlevel 
       REGEX "^#define[\t ]+MPFR_VERSION_PATCHLEVEL[\t ]+([0-9]+)")
  string(REGEX REPLACE "^#define[\t ]+MPFR_VERSION_MAJOR[\t ]+([0-9]+)" "\\1" MPFR_VERSION_MAJOR "${mpfr_version_str}")
  string(REGEX REPLACE "^#define[\t ]+MPFR_VERSION_MINOR[\t ]+([0-9]+)" "\\1" MPFR_VERSION_MINOR "${mpfr_version_str_minor}")
  string(REGEX REPLACE "^#define[\t ]+MPFR_VERSION_PATCHLEVEL[\t ]+([0-9]+)" "\\1" MPFR_VERSION_PATCH "${mpfr_version_str_patchlevel}")
  set(MPFR_VERSION "${MPFR_VERSION_MAJOR}.${MPFR_VERSION_MINOR}.${MPFR_VERSION_PATCH}")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MPFR_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MPFR
                                  REQUIRED_VARS MPFR_LIBRARY MPFR_INCLUDE_DIR
                                  VERSION_VAR MPFR_VERSION)

mark_as_advanced(MPFR_INCLUDE_DIR MPFR_LIBRARY)

if(MPFR_FOUND)
  set(MPFR_LIBRARIES ${MPFR_LIBRARY} ${GMP_LIBRARIES})
  set(MPFR_INCLUDE_DIRS ${MPFR_INCLUDE_DIR} ${GMP_INCLUDE_DIRS})
  set(MPFR_DEFINITIONS ${GMP_DEFINITIONS})
endif()