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

CMAKE_MINIMUM_REQUIRED( VERSION 3.1 )
PROJECT( pe )

IF( POLICY CMP0015 )
   CMAKE_POLICY( SET CMP0015 NEW )
ENDIF()

#==================================================================================================
#
# Definition of global options
#
#==================================================================================================

SET( CMAKE_BUILD_TYPE RELEASE CACHE STRING "The default build type for the whole project is RELEASE." )
SET( LIBRARY_TYPE     STATIC  CACHE STRING "Build type of library (STATIC/SHARED/BOTH)."              )
SET( BLAS             OFF     CACHE BOOL   "Build support for BLAS."                                  )
SET( MPI              OFF     CACHE BOOL   "Build support for MPI."                                   )
SET( MPI_PREINCLUDE   OFF     CACHE BOOL   "Include mpi.h ahead of all other headers."                )
SET( OPENCL           OFF     CACHE BOOL   "Build support for OpenCL."                                )
SET( IRRLICHT         OFF     CACHE BOOL   "Build support for Irrlicht engine."                       )
SET( EXAMPLES         OFF     CACHE BOOL   "Build examples."                                          )
SET( PACKAGE_NAME     pe                                                                              )
SET( PACKAGE_VERSION  trunk                                                                           )
SET( BOOST_INCLUDEDIR ""      CACHE PATH   "The boost include directory."                             )
SET( BOOST_LIBRARYDIR ""      CACHE PATH   "The boost library directory."                             )
SET( PREPROCESSOR_FLAGS ""    CACHE PATH   "Additional preprocessor flags."                           )
SET( COMPILER_FLAGS   ""      CACHE PATH   "Additional compiler flags."                               )
SET( LINKER_FLAGS     ""      CACHE PATH   "Additional linker flags when linking the examples."       )

# Guide selection of boost libraries by FindBoost.cmake
SET( Boost_USE_STATIC_LIBS OFF CACHE BOOL "Force static linking of Boost libraries." )

# Boost threads is only available as a multithreaded version so force FindBoost.cmake to use the multithreaded versions.
SET( Boost_USE_MULTITHREADED ON )

# Setting the build type (default=release)
IF( NOT CMAKE_BUILD_TYPE )
   SET ( CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "The default build type is RELEASE." FORCE )
ENDIF()

# Setting proper options for the compiler
IF( NOT CMAKE_BUILD_TYPE STREQUAL RELEASE AND NOT CMAKE_BUILD_TYPE STREQUAL DEBUG )
   MESSAGE( ERROR ": Unknown build type!" )
ENDIF()

# Setting compiler flags analogously to the configure script (and clear default compiler flags set
# by cmake for all compilers supported by the configure script to ensure compatibility)
IF( CMAKE_COMPILER_IS_GNUCXX )
   SET( CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Woverloaded-virtual -Wno-unused-local-typedefs -ansi --param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600" CACHE STRING "Compiler flags for GNU C++ compiler." FORCE )
ENDIF()

IF( ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
   SET( CMAKE_CXX_FLAGS "-Wall -Wshadow -Woverloaded-virtual -ansi" CACHE STRING "Compiler flags for Intel C++ compiler." FORCE )
ENDIF()

IF( ${CMAKE_CXX_COMPILER} MATCHES "clang\\+\\+" )
   SET( CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Woverloaded-virtual -ansi" CACHE STRING "Compiler flags for Intel C++ compiler." FORCE )
ENDIF()

# Setting user compiler and linker flags
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PREPROCESSOR_FLAGS} ${COMPILER_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_FLAGS}" )

# Assert that static library is built
IF ( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   SET( PE_SHARED_LIB_BUILD BOOL ON )
   SET( DEFINES "${DEFINES} PE_SHARED_LIB_BUILD=1" )
   MESSAGE( WARNING ": Shared library builds are currently experimental." )
ENDIF()




#==================================================================================================
#
# Visual Studio Setup
#
#==================================================================================================

IF ( MSVC )
   ADD_DEFINITIONS ( "-DNOMINMAX" )                # Disable Min/Max-Macros
   ADD_DEFINITIONS ( "-D_CRT_SECURE_NO_WARNINGS" ) # disable warnings promoting Microsoft's security enhanced CRT
   ADD_DEFINITIONS ( "-D_SCL_SECURE_NO_WARNINGS" ) # disable warnings triggered by Microsoft's checked iterators
   SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -MP" ) # enable multi-threaded compiling
ENDIF ( MSVC )




#==================================================================================================
#
# 3rd party software
#
#==================================================================================================

SET( CMAKE_MODULE_PATH ${pe_SOURCE_DIR}/cmake )

# Configuration of the Boost library
FIND_PACKAGE( Boost COMPONENTS thread system filesystem program_options REQUIRED )

IF( NOT Boost_FOUND )
   MESSAGE( ERROR ": Boost library not found! " )
ENDIF()

INCLUDE_DIRECTORIES( SYSTEM ${Boost_INCLUDE_DIRS} )
LINK_DIRECTORIES   ( ${Boost_LIBRARY_DIRS} )
LINK_LIBRARIES     ( ${Boost_LIBRARIES}    )
# Disable Boost auto-linking since it conflicts with cmake:
ADD_DEFINITIONS    ( -DBOOST_ALL_NO_LIB    )

# Configuration of the BLAS library
IF( BLAS )
   SET( BLAS_INCLUDE_DIR "" CACHE STRING "BLAS include path." )
   SET( BLAS_LIBRARY_1   "" CACHE STRING "BLAS library 1."    )
   SET( BLAS_LIBRARY_2   "" CACHE STRING "BLAS library 2."    )
   SET( BLAS_LIBRARY_3   "" CACHE STRING "BLAS library 3."    )
   INCLUDE_DIRECTORIES( SYSTEM ${BLAS_INCLUDE_DIR} )
   LINK_LIBRARIES     ( ${BLAS_LIBRARY_1} ${BLAS_LIBRARY_2} ${BLAS_LIBRARY_3} )
   SET( HAVE_BLAS BOOL ON )
   SET( DEFINES "${DEFINES} HAVE_BLAS=1" )
ELSE()
   SET( DEFINES "${DEFINES} HAVE_BLAS=0" )
ENDIF()

# Configuration of the Irrlicht library
IF( IRRLICHT )
   FIND_PACKAGE( Irrlicht REQUIRED )

   IF( NOT IRRLICHT_FOUND )
      MESSAGE( ERROR ": Irrlicht engine not found!" )
   ENDIF()

   INCLUDE_DIRECTORIES( SYSTEM ${IRRLICHT_INCLUDE_DIR} )
   LINK_DIRECTORIES   ( ${IRRLICHT_LIBRARY_DIR} )
   LINK_LIBRARIES     ( ${IRRLICHT_LIBRARY}     )

   SET( MEDIA_PATH "${PROJECT_SOURCE_DIR}/media/" )
   SET( HAVE_IRRLICHT BOOL ON )
   SET( DEFINES "${DEFINES} PE_MEDIA_PATH=${MEDIA_PATH} HAVE_IRRLICHT=1" )
ELSE()
   SET( DEFINES "${DEFINES} PE_MEDIA_PATH= HAVE_IRRLICHT=0" )
ENDIF()

# Configuration of the MPI library
IF( MPI )
   FIND_PACKAGE( MPI )
   IF( MPI_FOUND )
      INCLUDE_DIRECTORIES( SYSTEM ${MPI_CXX_INCLUDE_PATH} )
      LINK_DIRECTORIES( ${MPI_CXX_LIBRARY_PATH} )
      LINK_LIBRARIES( ${MPI_CXX_LIBRARIES} )
      SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}" )
      SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MPI_CXX_LINK_FLAGS}" )
      SET( HAVE_MPI BOOL ON )
      SET( DEFINES "${DEFINES} HAVE_MPI=1" )
   ELSE()
      SET( DEFINES "${DEFINES} HAVE_MPI=0" )
   ENDIF( MPI_FOUND )
ELSE()
   SET( DEFINES "${DEFINES} HAVE_MPI=0" )
ENDIF()

# Configuration of the OpenCL library
IF( OPENCL )
   SET( OPENCL_KERNEL_PATH "${PROJECT_SOURCE_DIR}/src/core/opencl/" )
   SET( HAVE_OPENCL BOOL ON )
   SET( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH=${OPENCL_KERNEL_PATH} HAVE_OPENCL=1" )
ELSE()
   SET( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH= HAVE_OPENCL=0" )
ENDIF()

# Create config.h from template and force inclusion
CONFIGURE_FILE( ${pe_SOURCE_DIR}/cmake/config.h.in ${PROJECT_BINARY_DIR}/config.h )
IF( MPI AND MPI_PREINCLUDE )
	FILE( APPEND ${PROJECT_BINARY_DIR}/config.h "#include <mpi.h>" )
ENDIF()
IF ( MSVC )
   SET( CMAKE_CXX_FLAGS "\"/FI ${PROJECT_BINARY_DIR}/config.h\" ${CMAKE_CXX_FLAGS}" )
ELSE()
   SET( CMAKE_CXX_FLAGS "-include ${PROJECT_BINARY_DIR}/config.h ${CMAKE_CXX_FLAGS}" )
ENDIF()

#==================================================================================================
#
# Build of the physics engine modules
#
#==================================================================================================

FILE( GLOB         PE_HEADERS         pe/*.h                 )
FILE( GLOB_RECURSE CONFIG_HEADERS     pe/config/*.h          )
FILE( GLOB_RECURSE CORE_HEADERS       pe/core/*.h            )
FILE( GLOB_RECURSE IRRLICHT_HEADERS   pe/irrlicht/*.h        )
FILE( GLOB_RECURSE MATH_HEADERS       pe/math/*.h            )
FILE( GLOB_RECURSE OPENDX_HEADERS     pe/opendx/*.h          )
FILE( GLOB_RECURSE POVRAY_HEADERS     pe/povray/*.h          )
FILE( GLOB_RECURSE SUPPORT_HEADERS    pe/support/*.h         )
FILE( GLOB_RECURSE SYSTEM_HEADERS     pe/system/*.h          )
FILE( GLOB_RECURSE UTIL_HEADERS       pe/util/*.h            )
FILE( GLOB_RECURSE VTK_HEADERS        pe/vtk/*.h             )

FILE( GLOB_RECURSE CORE_SOURCES       src/core/*.cpp         )
FILE( GLOB_RECURSE IRRLICHT_SOURCES   src/irrlicht/*.cpp     )
FILE( GLOB_RECURSE MATH_SOURCES       src/math/solvers/*.cpp )
FILE( GLOB_RECURSE OPENDX_SOURCES     src/opendx/*.cpp       )
FILE( GLOB_RECURSE POVRAY_SOURCES     src/povray/*.cpp       )
FILE( GLOB_RECURSE SUPPORT_SOURCES    src/support/*.cpp      )
FILE( GLOB_RECURSE UTIL_SOURCES       src/util/*.cpp         )
FILE( GLOB_RECURSE VTK_SOURCES        src/vtk/*.cpp          )

IF( MSVC )
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

   SOURCE_GROUP( "src\\core"          FILES ${CORE_SOURCES}       )
   SOURCE_GROUP( "src\\irrlicht"      FILES ${IRRLICHT_SOURCES}   )
   SOURCE_GROUP( "src\\math\\solvers" FILES ${MATH_SOURCES}       )
   SOURCE_GROUP( "src\\opendx"        FILES ${OPENDX_SOURCES}     )
   SOURCE_GROUP( "src\\povray"        FILES ${POVRAY_SOURCES}     )
   SOURCE_GROUP( "src\\support"       FILES ${SUPPORT_SOURCES}    )
   SOURCE_GROUP( "src\\util"          FILES ${UTIL_SOURCES}       )
   SOURCE_GROUP( "src\\vtk"           FILES ${VTK_SOURCES}        )
ENDIF()

SET( PE_SOURCES ${CORE_SOURCES} ${MATH_SOURCES} ${OPENDX_SOURCES} ${POVRAY_SOURCES} ${UTIL_SOURCES} ${SUPPORT_SOURCES} ${VTK_SOURCES} )
SET( PE_HEADERS ${PE_HEADERS} ${CONFIG_HEADERS} ${CORE_HEADERS} ${MATH_HEADERS} ${OPENDX_HEADERS} ${POVRAY_HEADERS} ${UTIL_HEADERS} ${SUPPORT_HEADERS} ${SYSTEM_HEADERS} ${VTK_HEADERS} )

IF( IRRLICHT )
   SET( PE_SOURCES ${PE_SOURCES} ${IRRLICHT_SOURCES} )
   SET( PE_HEADERS ${PE_HEADERS} ${IRRLICHT_HEADERS} )
ENDIF()

INCLUDE_DIRECTORIES( . )

SET( LIBRARY_OUTPUT_PATH lib )

SET( PE_ALL ${PE_SOURCES} ${PE_HEADERS} )

IF( LIBRARY_TYPE STREQUAL STATIC OR LIBRARY_TYPE STREQUAL BOTH )
   ADD_LIBRARY( pe_static STATIC ${PE_ALL} )
   SET_TARGET_PROPERTIES( pe_static PROPERTIES OUTPUT_NAME pe )
   # The library target "pe_static" will be named "pe.lib" with Microsoft tools.
   # This conflicts with the "pe.lib" import library corresponding to "pe.dll",
   # so we add a "lib" prefix (which is default on other platforms anyway):
   SET_TARGET_PROPERTIES( pe_static PROPERTIES PREFIX "lib" )
ENDIF()

IF( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   ADD_LIBRARY( pe_shared SHARED ${PE_ALL} )
   IF( CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
      # CXXFLAGS automatically also used for linking
      SET_TARGET_PROPERTIES( pe_shared PROPERTIES COMPILE_FLAGS "-fPIC -fvisibility=hidden -fvisibility-inlines-hidden" )
   ENDIF()
   SET_TARGET_PROPERTIES( pe_shared PROPERTIES OUTPUT_NAME pe )
   SET_TARGET_PROPERTIES( pe_shared PROPERTIES COMPILE_DEFINITIONS PE_SHARED_LIB_SELECT_EXPORTS )
ENDIF()




#==================================================================================================
#
# Build of the examples
#
#=================================================================================================

IF( EXAMPLES )

   # General settings for all examples
   INCLUDE_DIRECTORIES( . )
   LINK_DIRECTORIES( "${PROJECT_BINARY_DIR}" )
   LINK_LIBRARIES( ${Boost_LIBRARIES} )

   IF( LIBRARY_TYPE STREQUAL STATIC )
      LINK_LIBRARIES( pe_static )
   ELSE()
      LINK_LIBRARIES( pe_shared )
   ENDIF()

   IF( IRRLICHT )
      LINK_LIBRARIES( Irrlicht GLU Xxf86vm Xext X11 )
   ENDIF()

   # Boxstack example
   ADD_EXECUTABLE( boxstack examples/boxstack/BoxStack.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/boxstack )
   SET_TARGET_PROPERTIES( boxstack PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/boxstack/ )
   SET_TARGET_PROPERTIES( boxstack PROPERTIES OUTPUT_NAME boxstack )

   # Chain example
   ADD_EXECUTABLE( chain examples/chain/Chain.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/chain )
   SET_TARGET_PROPERTIES( chain PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/chain/ )
   SET_TARGET_PROPERTIES( chain PROPERTIES OUTPUT_NAME chain )

   # Newton's cradle example
   ADD_EXECUTABLE( cradle examples/cradle/Cradle.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/cradle )
   SET_TARGET_PROPERTIES( cradle PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/cradle/ )
   SET_TARGET_PROPERTIES( cradle PROPERTIES OUTPUT_NAME cradle )

   # Domino example
   ADD_EXECUTABLE( domino examples/domino/Domino.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/domino )
   SET_TARGET_PROPERTIES( domino PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/domino/ )
   SET_TARGET_PROPERTIES( domino PROPERTIES OUTPUT_NAME domino )

   # Granular example
   ADD_EXECUTABLE( granular examples/granular/Granular.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/granular )
   SET_TARGET_PROPERTIES( granular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/granular/ )
   SET_TARGET_PROPERTIES( granular PROPERTIES OUTPUT_NAME granular )

   # Nano example
   ADD_EXECUTABLE( nano examples/nano/Nano.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/nano )
   SET_TARGET_PROPERTIES( nano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/nano/ )
   SET_TARGET_PROPERTIES( nano PROPERTIES OUTPUT_NAME nano )

   # Well example
   ADD_EXECUTABLE( well examples/well/Well.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/well )
   SET_TARGET_PROPERTIES( well PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/well/ )
   SET_TARGET_PROPERTIES( well PROPERTIES OUTPUT_NAME well )

   # Shaker example
   ADD_EXECUTABLE( shaker examples/shaker/Shaker.cpp )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/shaker )
   SET_TARGET_PROPERTIES( shaker PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/shaker/ )
   SET_TARGET_PROPERTIES( shaker PROPERTIES OUTPUT_NAME shaker )

   #dependencies
   IF( LIBRARY_TYPE STREQUAL STATIC )
      ADD_DEPENDENCIES( boxstack pe_static )
      ADD_DEPENDENCIES( chain    pe_static )
      ADD_DEPENDENCIES( cradle   pe_static )
      ADD_DEPENDENCIES( domino   pe_static )
      ADD_DEPENDENCIES( granular pe_static )
      ADD_DEPENDENCIES( nano     pe_static )
      ADD_DEPENDENCIES( well     pe_static )
      ADD_DEPENDENCIES( shaker   pe_static )
   ELSE()
      ADD_DEPENDENCIES( boxstack pe_shared )
      ADD_DEPENDENCIES( chain    pe_shared )
      ADD_DEPENDENCIES( cradle   pe_shared )
      ADD_DEPENDENCIES( domino   pe_shared )
      ADD_DEPENDENCIES( granular pe_shared )
      ADD_DEPENDENCIES( nano     pe_shared )
      ADD_DEPENDENCIES( well     pe_shared )
      ADD_DEPENDENCIES( shaker   pe_shared )
   ENDIF()

   IF( MPI AND MPI_CXX_FOUND )
      # MPI-Cube example
      ADD_EXECUTABLE( mpicube examples/mpicube/MPICube.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpicube )
      SET_TARGET_PROPERTIES( mpicube PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpicube/ )
      SET_TARGET_PROPERTIES( mpicube PROPERTIES OUTPUT_NAME mpicube )

      # MPI-Granular example
      ADD_EXECUTABLE( mpigranular examples/mpigranular/MPIGranular.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpigranular )
      SET_TARGET_PROPERTIES( mpigranular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpigranular/ )
      SET_TARGET_PROPERTIES( mpigranular PROPERTIES OUTPUT_NAME mpigranular )

      # MPI-Hourglass example
      ADD_EXECUTABLE( mpihourglass examples/mpihourglass/MPIHourglass.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpihourglass )
      SET_TARGET_PROPERTIES( mpihourglass PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpihourglass/ )
      SET_TARGET_PROPERTIES( mpihourglass PROPERTIES OUTPUT_NAME mpihourglass )

      # MPI-Labyrinth example
      ADD_EXECUTABLE( mpilabyrinth examples/mpilabyrinth/MPILabyrinth.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilabyrinth )
      SET_TARGET_PROPERTIES( mpilabyrinth PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilabyrinth/ )
      SET_TARGET_PROPERTIES( mpilabyrinth PROPERTIES OUTPUT_NAME mpilabyrinth )

      # MPI-LSS example
      ADD_EXECUTABLE( mpilss examples/mpilss/MPILSS.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilss )
      SET_TARGET_PROPERTIES( mpilss PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilss/ )
      SET_TARGET_PROPERTIES( mpilss PROPERTIES OUTPUT_NAME mpilss )

      # MPI-Nano example
      ADD_EXECUTABLE( mpinano examples/mpinano/MPINano.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpinano )
      SET_TARGET_PROPERTIES( mpinano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpinano/ )
      SET_TARGET_PROPERTIES( mpinano PROPERTIES OUTPUT_NAME mpinano )

      # MPI-Stair example
      ADD_EXECUTABLE( mpistair examples/mpistair/MPIStair.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpistair )
      SET_TARGET_PROPERTIES( mpistair PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpistair/ )
      SET_TARGET_PROPERTIES( mpistair PROPERTIES OUTPUT_NAME mpistair )

      # MPI-Well example
      ADD_EXECUTABLE( mpiwell examples/mpiwell/MPIWell.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpiwell )
      SET_TARGET_PROPERTIES( mpiwell PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpiwell/ )
      SET_TARGET_PROPERTIES( mpiwell PROPERTIES OUTPUT_NAME mpiwell )

      # MPI-Impact example
      ADD_EXECUTABLE( mpiimpact examples/mpiimpact/MPIImpact.cpp )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpiimpact )
      SET_TARGET_PROPERTIES( mpiimpact PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpiimpact/ )
      SET_TARGET_PROPERTIES( mpiimpact PROPERTIES OUTPUT_NAME mpiimpact )

      # Dependencies
      IF( LIBRARY_TYPE STREQUAL STATIC )
         ADD_DEPENDENCIES( mpicube      pe_static )
         ADD_DEPENDENCIES( mpigranular  pe_static )
         ADD_DEPENDENCIES( mpihourglass pe_static )
         ADD_DEPENDENCIES( mpilabyrinth pe_static )
         ADD_DEPENDENCIES( mpilss       pe_static )
         ADD_DEPENDENCIES( mpinano      pe_static )
         ADD_DEPENDENCIES( mpistair     pe_static )
         ADD_DEPENDENCIES( mpiwell      pe_static )
         ADD_DEPENDENCIES( mpiimpact    pe_static )
      ELSE()
         ADD_DEPENDENCIES( mpicube      pe_shared )
         ADD_DEPENDENCIES( mpigranular  pe_shared )
         ADD_DEPENDENCIES( mpihourglass pe_shared )
         ADD_DEPENDENCIES( mpilabyrinth pe_shared )
         ADD_DEPENDENCIES( mpilss       pe_shared )
         ADD_DEPENDENCIES( mpinano      pe_shared )
         ADD_DEPENDENCIES( mpistair     pe_shared )
         ADD_DEPENDENCIES( mpiimpact    pe_shared )
      ENDIF()

   ENDIF()

ENDIF()



#==================================================================================================
#
# Doxygen documentation
#
#==================================================================================================

FIND_PACKAGE( Doxygen )
IF( DOXYGEN_EXECUTABLE )
   FILE( WRITE ${PROJECT_BINARY_DIR}/doc/Doxyfile.inject "
PROJECT_NAME     = ${PACKAGE_NAME}
PROJECT_NUMBER   = ${PACKAGE_VERSION}
OUTPUT_DIRECTORY = ${PROJECT_BINARY_DIR}/doc
INPUT            = ${CMAKE_SOURCE_DIR}/pe ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tutorial
IMAGE_PATH       = ${CMAKE_SOURCE_DIR}/doc/pics/jpg ${CMAKE_SOURCE_DIR}/doc/pics/png ${CMAKE_SOURCE_DIR}/doc/pics/eps
EXTRA_PACKAGES   = ${CMAKE_SOURCE_DIR}/doc/pe
PREDEFINED       = ${DEFINES} PE_PUBLIC= PE_PRIVATE= PE_PROTECTED= PE_NOTHROW=" )
   ADD_CUSTOM_TARGET( doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/Doxyfile WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/doc COMMENT "Generating documentation" )
ENDIF()

