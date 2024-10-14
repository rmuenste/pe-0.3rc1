#==================================================================================================
#
#  CMakeLists for the pe physics engine
#
#  Copyright (C) 2009 Klaus Iglberger
#                2012 Tobias Preclik
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

cmake_minimum_required( VERSION 3.5 )
project( pe )

if( POLICY CMP0015 )
   cmake_policy( SET CMP0015 NEW )
endif()
cmake_policy(SET CMP0057 NEW)
#==================================================================================================
#
# Definition of global options
#
#==================================================================================================

set( CMAKE_BUILD_TYPE RELEASE CACHE STRING   "The default build type for the whole project is RELEASE." )
set( LIBRARY_TYPE     STATIC  CACHE STRING   "Build type of library (STATIC/SHARED/BOTH)."              )
set( BLAS             OFF     CACHE BOOL     "Build support for BLAS."                                  )
set( MPI              OFF     CACHE BOOL     "Build support for MPI."                                   )
set( MPI_PREINCLUDE   OFF     CACHE BOOL     "Include mpi.h ahead of all other headers."                )
set( OPENCL           OFF     CACHE BOOL     "Build support for OpenCL."                                )
set( IRRLICHT         OFF     CACHE BOOL     "Build support for Irrlicht engine."                       )
set( EXAMPLES         OFF     CACHE BOOL     "Build examples."                                          )
set( PACKAGE_NAME     pe                                                                                )
set( PACKAGE_VERSION  trunk                                                                             )
set( BOOST_INCLUDEDIR ""      CACHE PATH     "The boost include directory."                             )
set( BOOST_LIBRARYDIR ""      CACHE PATH     "The boost library directory."                             )
set( PREPROCESSOR_FLAGS ""    CACHE STRING   "Additional preprocessor flags."                           )
set( COMPILER_FLAGS   ""      CACHE STRING   "Additional compiler flags."                               )
set( LINKER_FLAGS     ""      CACHE STRING   "Additional linker flags when linking the examples."       )
#OPTION(USE_CGAL       "Build the project using the CGAL library" OFF                                  )

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

# Guide selection of boost libraries by FindBoost.cmake
set( Boost_USE_STATIC_LIBS OFF CACHE BOOL "Force static linking of Boost libraries." )

# Boost threads is only available as a multithreaded version so force FindBoost.cmake to use the multithreaded versions.
set( Boost_USE_MULTITHREADED ON )

# Setting the build type (default=Release)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Validate build type
set(VALID_BUILD_TYPES Debug Release RelWithDebInfo MinSizeRel)
if(NOT CMAKE_BUILD_TYPE IN_LIST VALID_BUILD_TYPES)
  message(FATAL_ERROR "Unknown build type: ${CMAKE_BUILD_TYPE}")
endif()

#==================================================================================================
#
# Compiler Flag generation based on the used compiler
#
#==================================================================================================

# Start with empty compiler flags
set(PE_CXX_FLAGS)
# Setting compiler flags analogously to the configure script (and clear default compiler flags set
# by cmake for all compilers supported by the configure script to ensure compatibility)
# Set the required C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Initialize compiler flag variables
# Check if the compiler supports C++17
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_STANDARD 17)
  message(STATUS "C++17 support enabled.")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  message(STATUS "C++14 support enabled.")
else()
  message(FATAL_ERROR "The compiler does not support C++14 or C++17.")
endif()

# Detect compiler and set flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(LRG_FUNC1-"SHELL:--param large-function-growth=600")
 
  message(STATUS "We found a GNU compiler")
  list(APPEND PE_CXX_FLAGS
    -DMOBILE_INFINITE=1
    -Wall
    -Wextra
    -Wshadow
    -Wfatal-errors
    -Woverloaded-virtual
    -Wno-unused-local-typedefs
    -ansi
    "SHELL:--param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600"
  )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  message(STATUS "We found an Intel compiler")
  list(APPEND PE_CXX_FLAGS
    -Wall
    -Wshadow
    -Woverloaded-virtual
    -ansi
  )
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(STATUS "We found a Clang compiler")
  list(APPEND PE_CXX_FLAGS
    -Wall
    -Wextra
    -Wshadow
    -Woverloaded-virtual
    -ansi
  )
else()
  message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()

#========================================================================================================================================
#  OLD CODE FOR COMPILER SETTINGS
#========================================================================================================================================
#if( CMAKE_COMPILER_IS_GNUCXX )
#   # Check if the compiler supports C++17
#   include(CheckCXXCompilerFlag)
#   check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
#   check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
#
#   if(COMPILER_SUPPORTS_CXX17)
#      set(CMAKE_CXX_STANDARD 17)
#      set(CMAKE_CXX_STANDARD_REQUIRED ON)
#      message(STATUS "C++17 support enabled.")
#   elseif(COMPILER_SUPPORTS_CXX14)
#      set(CMAKE_CXX_STANDARD 14)
#      set(CMAKE_CXX_STANDARD_REQUIRED ON)
#      message(STATUS "C++14 support enabled.")
#   else()
#      message(FATAL_ERROR "The compiler does not support C++14 or C++17.")
#   endif()
#   set( CMAKE_CXX_FLAGS "-DMOBILE_INFINITE=1 -Wall -Wextra -Wshadow -Wfatal-errors -Woverloaded-virtual -Wno-unused-local-typedefs -ansi --param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600" CACHE STRING "Compiler flags for GNU C++ compiler." FORCE )
#   set( PE_CXX_FLAGS -DMOBILE_INFINITE=1 -Wall -Wextra -Wshadow -Wfatal-errors -Woverloaded-virtual -Wno-unused-local-typedefs -ansi --param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600)
#endif()
#
#if( ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
#   set( CMAKE_CXX_FLAGS "-Wall -Wshadow -Woverloaded-virtual -ansi" CACHE STRING "Compiler flags for Intel C++ compiler." FORCE )
#endif()
#
#if( ${CMAKE_CXX_COMPILER} MATCHES "clang\\+\\+" )
#   set( CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Woverloaded-virtual -ansi" CACHE STRING "Compiler flags for Intel C++ compiler." FORCE )
#endif()
#========================================================================================================================================

# Setting user compiler and linker flags
if(PREPROCESSOR_FLAGS)
  separate_arguments(PREPROCESSOR_FLAGS)
  list(APPEND PE_CXX_FLAGS ${PREPROCESSOR_FLAGS})
endif()

if(COMPILER_FLAGS)
  separate_arguments(COMPILER_FLAGS)
  list(APPEND PE_CXX_FLAGS ${COMPILER_FLAGS})
endif()

# Assert that static library is built
if ( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   set( PE_SHARED_LIB_BUILD BOOL ON )
   set( DEFINES "${DEFINES} PE_SHARED_LIB_BUILD=1" )
   message( WARNING ": Shared library builds are currently experimental." )
endif()

#==================================================================================================
#
# Visual Studio Setup
#
#==================================================================================================

if ( MSVC )
   ADD_DEFINITIONS ( "-DNOMINMAX" )                # Disable Min/Max-Macros
   ADD_DEFINITIONS ( "-D_CRT_SECURE_NO_WARNINGS" ) # disable warnings promoting Microsoft's security enhanced CRT
   ADD_DEFINITIONS ( "-D_SCL_SECURE_NO_WARNINGS" ) # disable warnings triggered by Microsoft's checked iterators
   set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -MP" ) # enable multi-threaded compiling
endif ( MSVC )




#==================================================================================================
#
# 3rd party software
#
#==================================================================================================

set( CMAKE_MODULE_PATH ${pe_SOURCE_DIR}/cmake )

# Configuration of the Boost library
find_package( Boost COMPONENTS thread system filesystem program_options random REQUIRED)

if( NOT Boost_FOUND )
   message( ERROR ": Boost library not found! " )
endif()

INCLUDE_DIRECTORIES( SYSTEM ${Boost_INCLUDE_DIRS} )
LINK_DIRECTORIES   ( ${Boost_LIBRARY_DIRS} )

if(USE_CGAL)
  LINK_LIBRARIES     ( ${Boost_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES}   )
else()
  LINK_LIBRARIES     ( ${Boost_LIBRARIES} )
endif()

# Disable Boost auto-linking since it conflicts with cmake:
ADD_DEFINITIONS    ( -DBOOST_ALL_NO_LIB    )

# Configuration of the BLAS library
if( BLAS )
   set( BLAS_INCLUDE_DIR "" CACHE STRING "BLAS include path." )
   set( BLAS_LIBRARY_1   "" CACHE STRING "BLAS library 1."    )
   set( BLAS_LIBRARY_2   "" CACHE STRING "BLAS library 2."    )
   set( BLAS_LIBRARY_3   "" CACHE STRING "BLAS library 3."    )
   INCLUDE_DIRECTORIES( SYSTEM ${BLAS_INCLUDE_DIR} )
   LINK_LIBRARIES     ( ${BLAS_LIBRARY_1} ${BLAS_LIBRARY_2} ${BLAS_LIBRARY_3} )
   set( HAVE_BLAS BOOL ON )
   set( DEFINES "${DEFINES} HAVE_BLAS=1" )
else()
   set( DEFINES "${DEFINES} HAVE_BLAS=0" )
endif()

# Configuration of the Irrlicht library
if( IRRLICHT )
   find_package( Irrlicht REQUIRED )

   if( NOT IRRLICHT_FOUND )
      message( ERROR ": Irrlicht engine not found!" )
   endif()

   INCLUDE_DIRECTORIES( SYSTEM ${IRRLICHT_INCLUDE_DIR} )
   LINK_DIRECTORIES   ( ${IRRLICHT_LIBRARY_DIR} )
   LINK_LIBRARIES     ( ${IRRLICHT_LIBRARY}     )

   set( MEDIA_PATH "${PROJECT_SOURCE_DIR}/media/" )
   set( HAVE_IRRLICHT BOOL ON )
   set( DEFINES "${DEFINES} PE_MEDIA_PATH=${MEDIA_PATH} HAVE_IRRLICHT=1" )
else()
   set( DEFINES "${DEFINES} PE_MEDIA_PATH= HAVE_IRRLICHT=0" )
endif()

# Configuration of the CGAL library
# Check if USE_CGAL is enabled
if (USE_CGAL)
    message(STATUS "CGAL support is enabled.")
    # Add a compiler define if USE_CGAL is true
    set( DEFINES "${DEFINES} HAVE_CGAL=1" )
    INCLUDE_DIRECTORIES( SYSTEM ${CGAL_INCLUDE_DIRS} )
else()
    message(STATUS "CGAL support is disabled.")
    # Add a compiler define if USE_CGAL is false (optional)
    set( DEFINES "${DEFINES} HAVE_CGAL=0" )
endif()

# Configuration of the MPI library
if( MPI )
   find_package( MPI )
   if( MPI_FOUND )
      INCLUDE_DIRECTORIES( SYSTEM ${MPI_CXX_INCLUDE_PATH} )
      LINK_DIRECTORIES( ${MPI_CXX_LIBRARY_PATH} )
      LINK_LIBRARIES( ${MPI_CXX_LIBRARIES} )
      set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}" )
      set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MPI_CXX_LINK_FLAGS}" )

      message(FATAL_ERROR "MPI_INCLUDE: ${MPI_CXX_INCLUDE_PATH}  MPI_CXX_LIB_PATH: ${MPI_CXX_LIBRARY_PATH} ${MPI_CXX_LIBRARIES")

      set( HAVE_MPI BOOL ON )
      set( DEFINES "${DEFINES} HAVE_MPI=1" )
   else()
      set( DEFINES "${DEFINES} HAVE_MPI=0" )
   endif( MPI_FOUND )
else()
   set( DEFINES "${DEFINES} HAVE_MPI=0" )
endif()

# Configuration of the OpenCL library
if( OPENCL )
   set( OPENCL_KERNEL_PATH "${PROJECT_SOURCE_DIR}/src/core/opencl/" )
   set( HAVE_OPENCL BOOL ON )
   set( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH=${OPENCL_KERNEL_PATH} HAVE_OPENCL=1" )
else()
   set( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH= HAVE_OPENCL=0" )
endif()

# Create config.h from template and force inclusion
configure_file( ${pe_SOURCE_DIR}/cmake/config.h.in ${PROJECT_BINARY_DIR}/config.h )
if( MPI AND MPI_PREINCLUDE )
	file( APPEND ${PROJECT_BINARY_DIR}/config.h "#include <mpi.h>" )
endif()
if ( MSVC )
   set( CMAKE_CXX_FLAGS "\"/FI ${PROJECT_BINARY_DIR}/config.h\" ${CMAKE_CXX_FLAGS}" )
else()
   set( CMAKE_CXX_FLAGS "-include ${PROJECT_BINARY_DIR}/config.h ${CMAKE_CXX_FLAGS}" )
endif()

#==================================================================================================
#
# Build of the physics engine modules
#
#==================================================================================================

file( GLOB         PE_HEADERS         pe/*.h                 )
file( GLOB_RECURSE CONFIG_HEADERS     pe/config/*.h          )
file( GLOB_RECURSE CORE_HEADERS       pe/core/*.h            )
file( GLOB_RECURSE IRRLICHT_HEADERS   pe/irrlicht/*.h        )
file( GLOB_RECURSE MATH_HEADERS       pe/math/*.h            )
file( GLOB_RECURSE OPENDX_HEADERS     pe/opendx/*.h          )
file( GLOB_RECURSE POVRAY_HEADERS     pe/povray/*.h          )
file( GLOB_RECURSE SUPPORT_HEADERS    pe/support/*.h         )
file( GLOB_RECURSE SYSTEM_HEADERS     pe/system/*.h          )
file( GLOB_RECURSE UTIL_HEADERS       pe/util/*.h            )
file( GLOB_RECURSE VTK_HEADERS        pe/vtk/*.h             )
file( GLOB_RECURSE INTF_HEADERS       pe/interface/*.h       )

file( GLOB_RECURSE CORE_SOURCES       src/core/*.cpp         )
file( GLOB_RECURSE IRRLICHT_SOURCES   src/irrlicht/*.cpp     )
file( GLOB_RECURSE MATH_SOURCES       src/math/solvers/*.cpp )
file( GLOB_RECURSE OPENDX_SOURCES     src/opendx/*.cpp       )
file( GLOB_RECURSE POVRAY_SOURCES     src/povray/*.cpp       )
file( GLOB_RECURSE SUPPORT_SOURCES    src/support/*.cpp      )
file( GLOB_RECURSE UTIL_SOURCES       src/util/*.cpp         )
file( GLOB_RECURSE VTK_SOURCES        src/vtk/*.cpp          )
file( GLOB_RECURSE INTF_SOURCES       src/interface/*.cpp    )

if( MSVC )
   SOURCE_GROUP( "pe"                 FILES ${PE_HEADERS}         )
   SOURCE_GROUP( "pe\\config"         FILES ${CONFIG_HEADERS}     )
   SOURCE_GROUP( "pe\\core"           FILES ${CORE_HEADERS}       )
   SOURCE_GROUP( "pe\\irrlicht"       FILES ${IRRLICHT_HEADERS}   )
   SOURCE_GROUP( "pe\\math"           FILES ${MATH_HEADERS}       )
   SOURCE_GROUP( "pe\\opendx"         FILES ${OPENDX_HEADERS}     )
   SOURCE_GROUP( "pe\\povray"         FILES ${POVRAY_HEADERS}     )
   SOURCE_GROUP( "pe\\support"        FILES ${SUPPORT_HEADERS}    )
   SOURCE_GROUP( "pe\\system"         FILES ${SYSTEM_HEADERS}     )
   SOURCE_GROUP( "pe\\util"           FILES ${UTIL_HEADERS}       )
   SOURCE_GROUP( "pe\\vtk"            FILES ${VTK_HEADERS}        )
   SOURCE_GROUP( "pe\\config"         FILES ${CONFIG_HEADERS}     )
   SOURCE_GROUP( "pe\\interface"      FILES ${INTF_HEADERS}       )

   SOURCE_GROUP( "src\\core"          FILES ${CORE_SOURCES}       )
   SOURCE_GROUP( "src\\irrlicht"      FILES ${IRRLICHT_SOURCES}   )
   SOURCE_GROUP( "src\\math\\solvers" FILES ${MATH_SOURCES}       )
   SOURCE_GROUP( "src\\opendx"        FILES ${OPENDX_SOURCES}     )
   SOURCE_GROUP( "src\\povray"        FILES ${POVRAY_SOURCES}     )
   SOURCE_GROUP( "src\\support"       FILES ${SUPPORT_SOURCES}    )
   SOURCE_GROUP( "src\\util"          FILES ${UTIL_SOURCES}       )
   SOURCE_GROUP( "src\\vtk"           FILES ${VTK_SOURCES}        )
   SOURCE_GROUP( "src\\interface"     FILES ${INTF_SOURCES}       )
endif()

set( PE_SOURCES ${CORE_SOURCES} ${MATH_SOURCES} ${OPENDX_SOURCES} ${POVRAY_SOURCES} ${UTIL_SOURCES} ${SUPPORT_SOURCES} ${VTK_SOURCES} ${INTF_SOURCES})
set( PE_HEADERS ${PE_HEADERS} ${CONFIG_HEADERS} ${CORE_HEADERS} ${MATH_HEADERS} ${OPENDX_HEADERS} ${POVRAY_HEADERS} ${UTIL_HEADERS} ${SUPPORT_HEADERS} ${SYSTEM_HEADERS} ${VTK_HEADERS} ${INTF_HEADERS})

if( IRRLICHT )
   set( PE_SOURCES ${PE_SOURCES} ${IRRLICHT_SOURCES} )
   set( PE_HEADERS ${PE_HEADERS} ${IRRLICHT_HEADERS} )
endif()

INCLUDE_DIRECTORIES( . )

set( LIBRARY_OUTPUT_PATH lib )

set( PE_ALL ${PE_SOURCES} ${PE_HEADERS} )

#==================================================================================================
#
# CONFIGURATION OF THE pe_static AND pe_shared LIBRARIES
#
#=================================================================================================
if( LIBRARY_TYPE STREQUAL STATIC OR LIBRARY_TYPE STREQUAL BOTH )
   add_library( pe_static STATIC ${PE_ALL} )
   message(STATUS "cmake: ${CMAKE_CXX_FLAGS}")
   #message(FATAL_ERROR "cmake: ${PE_CXX_FLAGS}")
   target_compile_options(pe_static PRIVATE ${PE_CXX_FLAGS})
   get_target_property(COMP_OPTIONS pe_static COMPILE_OPTIONS)
   #message(FATAL_ERROR "flags: ${COMP_OPTIONS}")
   set_target_properties( pe_static PROPERTIES OUTPUT_NAME pe )
   # The library target "pe_static" will be named "pe.lib" with Microsoft tools.
   # This conflicts with the "pe.lib" import library corresponding to "pe.dll",
   # so we add a "lib" prefix (which is default on other platforms anyway):
   set_target_properties( pe_static PROPERTIES PREFIX "lib" )
endif()

if( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   add_library( pe_shared SHARED ${PE_ALL} )
   target_compile_options(pe_shared PRIVATE ${PE_CXX_FLAGS})
   if( CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
      # CXXFLAGS automatically also used for linking
      target_compile_options(pe_shared PRIVATE 
        -fPIC 
        -fvisibility=hidden 
        -fvisibility-inlines-hidden
      )
   endif()
   set_target_properties( pe_shared PROPERTIES OUTPUT_NAME pe )
   target_compile_definitions(pe_shared PRIVATE PE_SHARED_LIB_SELECT_EXPORTS)
endif()




#==================================================================================================
#
# Build of the examples
#
#=================================================================================================
if( EXAMPLES )
   message(STATUS "Building PE examples")
   # General settings for all examples
   INCLUDE_DIRECTORIES( . )
   LINK_DIRECTORIES( "${PROJECT_BINARY_DIR}" )
   LINK_LIBRARIES( ${Boost_LIBRARIES} )

   if( LIBRARY_TYPE STREQUAL STATIC )
      LINK_LIBRARIES( pe_static )
   else()
      LINK_LIBRARIES( pe_shared )
   endif()

   if( IRRLICHT )
      LINK_LIBRARIES( Irrlicht GLU Xxf86vm Xext X11 )
   endif()

   # Boxstack example
   add_executable( boxstack examples/boxstack/BoxStack.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/boxstack )
   set_target_properties( boxstack PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/boxstack/ )
   set_target_properties( boxstack PROPERTIES OUTPUT_NAME boxstack )

   # Chain example
   add_executable( chain examples/chain/Chain.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/chain )
   set_target_properties( chain PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/chain/ )
   set_target_properties( chain PROPERTIES OUTPUT_NAME chain )

   # Newton's cradle example
   add_executable( cradle examples/cradle/Cradle.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/cradle )
   set_target_properties( cradle PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/cradle/ )
   set_target_properties( cradle PROPERTIES OUTPUT_NAME cradle )

   # Domino example
   add_executable( domino examples/domino/Domino.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/domino )
   set_target_properties( domino PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/domino/ )
   set_target_properties( domino PROPERTIES OUTPUT_NAME domino )

   # Granular example
   add_executable( granular examples/granular/Granular.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/granular )
   set_target_properties( granular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/granular/ )
   set_target_properties( granular PROPERTIES OUTPUT_NAME granular )

   # Nano example
   add_executable( nano examples/nano/Nano.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/nano )
   set_target_properties( nano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/nano/ )
   set_target_properties( nano PROPERTIES OUTPUT_NAME nano )

   # Well example
   add_executable( well examples/well/Well.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/well )
   set_target_properties( well PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/well/ )
   set_target_properties( well PROPERTIES OUTPUT_NAME well )

   # Shaker example
   add_executable( shaker examples/shaker/Shaker.cpp )
   file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/shaker )
   set_target_properties( shaker PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/shaker/ )
   set_target_properties( shaker PROPERTIES OUTPUT_NAME shaker )

   #dependencies
   if( LIBRARY_TYPE STREQUAL STATIC )
      ADD_DEPENDENCIES( boxstack pe_static )
      ADD_DEPENDENCIES( chain    pe_static )
      ADD_DEPENDENCIES( cradle   pe_static )
      ADD_DEPENDENCIES( domino   pe_static )
      ADD_DEPENDENCIES( granular pe_static )
      ADD_DEPENDENCIES( nano     pe_static )
      ADD_DEPENDENCIES( well     pe_static )
      ADD_DEPENDENCIES( shaker   pe_static )
   else()
      ADD_DEPENDENCIES( boxstack pe_shared )
      ADD_DEPENDENCIES( chain    pe_shared )
      ADD_DEPENDENCIES( cradle   pe_shared )
      ADD_DEPENDENCIES( domino   pe_shared )
      ADD_DEPENDENCIES( granular pe_shared )
      ADD_DEPENDENCIES( nano     pe_shared )
      ADD_DEPENDENCIES( well     pe_shared )
      ADD_DEPENDENCIES( shaker   pe_shared )
   endif()

   if( MPI AND MPI_CXX_FOUND )
      # MPI-Cube example
      add_executable( mpicube examples/mpicube/MPICube.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpicube )
      set_target_properties( mpicube PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpicube/ )
      set_target_properties( mpicube PROPERTIES OUTPUT_NAME mpicube )

      # MPI-Granular example
      add_executable( mpigranular examples/mpigranular/MPIGranular.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpigranular )
      set_target_properties( mpigranular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpigranular/ )
      set_target_properties( mpigranular PROPERTIES OUTPUT_NAME mpigranular )

      # MPI-Hourglass example
      add_executable( mpihourglass examples/mpihourglass/MPIHourglass.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpihourglass )
      set_target_properties( mpihourglass PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpihourglass/ )
      set_target_properties( mpihourglass PROPERTIES OUTPUT_NAME mpihourglass )

      # MPI-Labyrinth example
      add_executable( mpilabyrinth examples/mpilabyrinth/MPILabyrinth.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilabyrinth )
      set_target_properties( mpilabyrinth PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilabyrinth/ )
      set_target_properties( mpilabyrinth PROPERTIES OUTPUT_NAME mpilabyrinth )

      # MPI-LSS example
      add_executable( mpilss examples/mpilss/MPILSS.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilss )
      set_target_properties( mpilss PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilss/ )
      set_target_properties( mpilss PROPERTIES OUTPUT_NAME mpilss )

      # MPI-Nano example
      add_executable( mpinano examples/mpinano/MPINano.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpinano )
      set_target_properties( mpinano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpinano/ )
      set_target_properties( mpinano PROPERTIES OUTPUT_NAME mpinano )

      # MPI-Stair example
      add_executable( mpistair examples/mpistair/MPIStair.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpistair )
      set_target_properties( mpistair PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpistair/ )
      set_target_properties( mpistair PROPERTIES OUTPUT_NAME mpistair )

      # MPI-Impact example
      add_executable( mpiimpact examples/mpiimpact/MPIImpact.cpp )
      file( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpiimpact )
      set_target_properties( mpiimpact PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpiimpact/ )
      set_target_properties( mpiimpact PROPERTIES OUTPUT_NAME mpiimpact )

      # Dependencies
      if( LIBRARY_TYPE STREQUAL STATIC )
         ADD_DEPENDENCIES( mpicube      pe_static )
         ADD_DEPENDENCIES( mpigranular  pe_static )
         ADD_DEPENDENCIES( mpihourglass pe_static )
         ADD_DEPENDENCIES( mpilabyrinth pe_static )
         ADD_DEPENDENCIES( mpilss       pe_static )
         ADD_DEPENDENCIES( mpinano      pe_static )
         ADD_DEPENDENCIES( mpistair     pe_static )
         ADD_DEPENDENCIES( mpiimpact    pe_static )
      else()
         ADD_DEPENDENCIES( mpicube      pe_shared )
         ADD_DEPENDENCIES( mpigranular  pe_shared )
         ADD_DEPENDENCIES( mpihourglass pe_shared )
         ADD_DEPENDENCIES( mpilabyrinth pe_shared )
         ADD_DEPENDENCIES( mpilss       pe_shared )
         ADD_DEPENDENCIES( mpinano      pe_shared )
         ADD_DEPENDENCIES( mpistair     pe_shared )
         ADD_DEPENDENCIES( mpiimpact    pe_shared )
      endif()

   endif()

   ADD_SUBDIRECTORY(examples)
endif()



#==================================================================================================
#
# Doxygen documentation
#
#==================================================================================================

find_package( Doxygen )
if( DOXYGEN_EXECUTABLE )
   file( WRITE ${PROJECT_BINARY_DIR}/doc/Doxyfile.inject "
PROJECT_NAME     = ${PACKAGE_NAME}
PROJECT_NUMBER   = ${PACKAGE_VERSION}
OUTPUT_DIRECTORY = ${PROJECT_BINARY_DIR}/doc
INPUT            = ${CMAKE_SOURCE_DIR}/pe ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tutorial
IMAGE_PATH       = ${CMAKE_SOURCE_DIR}/doc/pics/jpg ${CMAKE_SOURCE_DIR}/doc/pics/png ${CMAKE_SOURCE_DIR}/doc/pics/eps
EXTRA_PACKAGES   = ${CMAKE_SOURCE_DIR}/doc/pe
PREDEFINED       = ${DEFINES} PE_PUBLIC= PE_PRIVATE= PE_PROTECTED= PE_NOTHROW=" )
   ADD_CUSTOM_TARGET( doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/Doxyfile WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/doc COMMENT "Generating documentation" )
endif()

