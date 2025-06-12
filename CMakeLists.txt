#==================================================================================================
#
#  CMakeLists for the pe physics engine
#
#  Copyright (C) 2009 Klaus Iglberger
#                2012 Tobias Preclik
#                2023 Raphael Münster
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

cmake_minimum_required(VERSION 3.15)  # or higher, if possible.  3.15 introduced INTERFACE_LIBRARY
project(pe)

# Enable ExternalProject module
include(ExternalProject)
include(FetchContent)
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
set( EIGEN            ON      CACHE BOOL     "Build with Eigen library support."                        )
set( PACKAGE_NAME     pe                                                                                )
set( PACKAGE_VERSION  trunk                                                                             )
set( BOOST_INCLUDEDIR ""      CACHE PATH     "The boost include directory."                             )
set( BOOST_LIBRARYDIR ""      CACHE PATH     "The boost library directory."                             )
set( PREPROCESSOR_FLAGS ""    CACHE STRING   "Additional preprocessor flags."                           )
set( COMPILER_FLAGS   ""      CACHE STRING   "Additional compiler flags."                               )
set( LINKER_FLAGS     ""      CACHE STRING   "Additional linker flags when linking the examples."       )
set( CGAL             OFF     CACHE BOOL     "Build with CGAL library support."                         )

# Option to enable/disable JSON support
option(USE_JSON "Use nlohmann/json library" ON)

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

# Detect compiler and set flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
 
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
# C++ Standard Handling
#==================================================================================================
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
  list(APPEND PE_CXX_FLAGS
  -std=c++17
  )
  message(STATUS "C++17 support enabled.")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  message(STATUS "C++14 support enabled.")
  list(APPEND PE_CXX_FLAGS
  -std=c++14
  )
else()
  message(FATAL_ERROR "The compiler does not support C++14 or C++17.")
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
set(PE_THIRD_PARTY_LIBS)
set(PE_THIRD_PARTY_INCLUDE_DIRS)

set( CMAKE_MODULE_PATH ${pe_SOURCE_DIR}/cmake )

# Configuration of the Boost library
find_package(Boost REQUIRED COMPONENTS thread system filesystem program_options random)

if( NOT Boost_FOUND )
   message( ERROR ": Boost library not found! " )
endif()

list(APPEND PE_THIRD_PARTY_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND PE_THIRD_PARTY_LIBS
  Boost::thread
  Boost::system
  Boost::filesystem
  Boost::program_options
  Boost::random
)

if(USE_CGAL)
  LINK_LIBRARIES(
    Boost::thread Boost::system Boost::filesystem Boost::program_options Boost::random
    ${GMP_LIBRARIES} ${MPFR_LIBRARIES}
  )
else()
  LINK_LIBRARIES(
    Boost::thread Boost::system Boost::filesystem Boost::program_options Boost::random
  )
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

# Configuration of the Eigen library
#============================================================================
if( EIGEN )
  include(FetchContent)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
  )
  FetchContent_MakeAvailable(eigen)
  list(APPEND PE_THIRD_PARTY_LIBS Eigen3::Eigen)
  add_definitions(-DPE_USE_EIGEN)
  set( DEFINES "${DEFINES} PE_USE_EIGEN=1" )
  message(STATUS "Eigen support enabled")
else()
  set( DEFINES "${DEFINES} PE_USE_EIGEN=0" )
endif()

#============================================================================
# NEW APPROACH TO ADD CGAL
#============================================================================
# In subproject's CMakeLists.txt
if(CGAL)
  # Try to find an existing CGAL installation first
  find_package(CGAL QUIET HINTS 
    ${CGAL_DIR}                               # If passed from parent
    ${CMAKE_BINARY_DIR}/extern/cgal/install/lib/cmake/CGAL  # Default install location from main project
  )
  
  if(CGAL_FOUND)
    #message(STATUS "Found existing CGAL installation")
    # Use the found CGAL
    include(${CGAL_USE_FILE})
    
  else()
    # message(STATUS "Building CGAL from source")
    # Your existing ExternalProject_Add code for CGAL
    set(CGAL_INSTALL_DIR ${CMAKE_BINARY_DIR}/extern/cgal/install)
    set(CGAL_STAMP_DIR ${CMAKE_BINARY_DIR}/extern/cgal/src/cgal-stamp)
    
    # Check if the install stamp file exists
    if(EXISTS "${CGAL_STAMP_DIR}/cgal-install")
      add_custom_target(cgal)
    else()
      ExternalProject_Add(
        cgal
        GIT_REPOSITORY https://github.com/CGAL/cgal.git
        GIT_TAG v5.3.2
        PREFIX ${CMAKE_BINARY_DIR}/extern/cgal
        CMAKE_ARGS 
          -DCMAKE_INSTALL_PREFIX=${CGAL_INSTALL_DIR}
          -DWITH_CGAL_ImageIO=OFF
          -DWITH_CGAL_Qt5=OFF
        TEST_COMMAND ""
      )
    endif()
    
    # Set CGAL include directory
    set(CGAL_INCLUDE_DIR ${CMAKE_BINARY_DIR}/extern/cgal/install/include)
    include_directories(SYSTEM ${CGAL_INCLUDE_DIR})
  endif()
  
  # Add definition to enable CGAL
  add_definitions(-DPE_USE_CGAL)
  set(DEFINES "${DEFINES} PE_USE_CGAL=1")
  # message(STATUS "CGAL support enabled")
else()
  set(DEFINES "${DEFINES} PE_USE_CGAL=0")
endif()
#============================================================================


#============================================================================
# Update library and linking settings for CGAL
if( CGAL )
    message(STATUS "CGAL support is enabled.")
    # Add a compiler define if CGAL is true
    set( DEFINES "${DEFINES} HAVE_CGAL=1" )
    
    # Add CGAL-related libraries to the third-party libraries
    find_package(GMP QUIET)
    find_package(MPFR QUIET)
    
    if(GMP_FOUND)
        list(APPEND PE_THIRD_PARTY_LIBS ${GMP_LIBRARIES})
    endif()
    
    if(MPFR_FOUND)
        list(APPEND PE_THIRD_PARTY_LIBS ${MPFR_LIBRARIES})
    endif()
    
    # Add CGAL include directories
    list(APPEND PE_THIRD_PARTY_INCLUDE_DIRS ${CGAL_INCLUDE_DIR})
else()
    message(STATUS "CGAL support is disabled.")
    # Add a compiler define if CGAL is false
    set( DEFINES "${DEFINES} HAVE_CGAL=0" )
endif()
#============================================================================


# Configuration of the Irrlicht library
#==============================================================================
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

# Configuration of the Json library
#==============================================================================
if(USE_JSON)
    
    # Set custom download directory
    set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/external" CACHE PATH "Directory for downloaded dependencies")
    
    # Configure nlohmann/json
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2  # You can specify a specific version/tag
    )
    
    # Make json available for the project
    FetchContent_MakeAvailable(json)
    
    # Get the source directory for the downloaded library
    FetchContent_GetProperties(json SOURCE_DIR json_SOURCE_DIR)
    
    # Add compile definition to indicate JSON is available
    add_compile_definitions(HAVE_JSON)
    
    # Add the JSON include directory explicitly
    list(APPEND PE_THIRD_PARTY_INCLUDE_DIRS ${json_SOURCE_DIR}/include)
    
    message(STATUS "nlohmann/json will be downloaded to: ${FETCHCONTENT_BASE_DIR}")
    message(STATUS "nlohmann/json include directory: ${json_SOURCE_DIR}/include")
else()
    message(STATUS "nlohmann/json will NOT be used")
endif()


# Configuration of the MPI library
if( MPI )
   find_package( MPI REQUIRED)
   if( MPI_FOUND )

      list(APPEND PE_THIRD_PARTY_LIBS MPI::MPI_CXX)
      get_target_property(MPI_CXX_INCLUDE_DIRS MPI::MPI_CXX INTERFACE_INCLUDE_DIRECTORIES)
      list(APPEND PE_THIRD_PARTY_INCLUDE_DIRS ${MPI_CXX_INCLUDE_DIRS})
      #message(FATAL_ERROR "3rd: ${PE_THIRD_PARTY_INCLUDE_DIRS}")
      message(STATUS "User-defined MPI_COMPILER and MPI_LINKER flags" )
      set( HAVE_MPI BOOL ON )

      set( DEFINES "${DEFINES} HAVE_MPI=1" )
   else()
      set( DEFINES "${DEFINES} HAVE_MPI=0" )
   endif()
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
   target_compile_options(pe_static PUBLIC ${PE_CXX_FLAGS})
   get_target_property(COMP_OPTIONS pe_static COMPILE_OPTIONS)
   set_target_properties( pe_static PROPERTIES OUTPUT_NAME pe )
   # The library target "pe_static" will be named "pe.lib" with Microsoft tools.
   # This conflicts with the "pe.lib" import library corresponding to "pe.dll",
   # so we add a "lib" prefix (which is default on other platforms anyway):
   set_target_properties( pe_static PROPERTIES PREFIX "lib" )
   target_link_libraries(pe_static PRIVATE ${PE_THIRD_PARTY_LIBS})
   target_include_directories(pe_static PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PE_THIRD_PARTY_INCLUDE_DIRS})
   add_dependencies(pe_static eigen)
endif()

if( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   add_library( pe_shared SHARED ${PE_ALL} )
   target_compile_options(pe_shared PUBLIC ${PE_CXX_FLAGS})
   if( CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
      # CXXFLAGS automatically also used for linking
      target_compile_options(pe_shared PUBLIC 
        -fPIC 
        -fvisibility=hidden 
        -fvisibility-inlines-hidden
      )
   endif()
   set_target_properties( pe_shared PROPERTIES OUTPUT_NAME pe )
   target_link_libraries(pe_shared PRIVATE ${PE_THIRD_PARTY_LIBS})
   target_include_directories(pe_shared PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PE_THIRD_PARTY_INCLUDE_DIRS})
   target_compile_definitions(pe_shared PUBLIC PE_SHARED_LIB_SELECT_EXPORTS)
   add_dependencies(pe_static eigen)
endif()


#==================================================================================================
#
# Build of the examples
#
#=================================================================================================
set(PE_EXAMPLES_LIBS)
set(PE_EXAMPLES_INCLUDE_DIRS)

if( EXAMPLES )
   message(STATUS "Building PE examples")

   list(APPEND PE_EXAMPLES_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
   list(APPEND PE_EXAMPLES_INCLUDE_DIRS ${MPI_CXX_INCLUDE_DIRS})

   if( IRRLICHT )
      LINK_LIBRARIES( Irrlicht GLU Xxf86vm Xext X11 )
   endif()

   function(pe_add_example target src)
      add_executable(${target} ${src})
      set_target_properties(${target} PROPERTIES
         RUNTIME_OUTPUT_DIRECTORY examples/${target}/
         OUTPUT_NAME ${target})
      target_include_directories(${target} PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
      target_link_libraries(${target} PRIVATE
         $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
         $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
         $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
      )
   endfunction()

   set(PE_SERIAL_EXAMPLES
      "boxstack;examples/boxstack/BoxStack.cpp"
      "chain;examples/chain/Chain.cpp"
      "cradle;examples/cradle/Cradle.cpp"
      "domino;examples/domino/Domino.cpp"
      "granular;examples/granular/Granular.cpp"
      "nano;examples/nano/Nano.cpp"
      "well;examples/well/Well.cpp"
      "shaker;examples/shaker/Shaker.cpp"
   )

   foreach(example_pair IN LISTS PE_SERIAL_EXAMPLES)
      list(GET example_pair 0 exe_name)
      list(GET example_pair 1 src)
      pe_add_example(${exe_name} ${src})
   endforeach()

   if( MPI AND MPI_CXX_FOUND )
      set(PE_MPI_EXAMPLES
         "mpicube;examples/mpicube/MPICube.cpp"
         "mpigranular;examples/mpigranular/MPIGranular.cpp"
         "mpihourglass;examples/mpihourglass/MPIHourglass.cpp"
         "mpilabyrinth;examples/mpilabyrinth/MPILabyrinth.cpp"
         "mpilss;examples/mpilss/MPILSS.cpp"
         "mpinano;examples/mpinano/MPINano.cpp"
         "mpistair;examples/mpistair/MPIStair.cpp"
         "mpiimpact;examples/mpiimpact/MPIImpact.cpp"
      )
      foreach(example_pair IN LISTS PE_MPI_EXAMPLES)
         list(GET example_pair 0 exe_name)
         list(GET example_pair 1 src)
         pe_add_example(${exe_name} ${src})
      endforeach()
   endif()


   add_subdirectory(examples)
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

