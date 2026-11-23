//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_header_test.cpp
 *  \brief The coupling API header is self-contained and can sit next to <mpi.h>
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
 *  This translation unit includes the coupling API header and nothing else from pe (the test
 *  harness helpers below use the C++ standard library only). With
 *  PE_COUPLING_TEST_WITH_MPI_H=1 it includes <mpi.h> before and after the header: that is the
 *  situation of a CFD code such as FEAT3, which uses MPI itself while pe may be built without
 *  it (pe's own headers then define MPI stand-in types that clash with <mpi.h>). The program
 *  links against pe and queries the build information without creating an engine.
 *
 *  With PE_COUPLING_TEST_ENGINE=1 it also drives an Engine (the shared-library smoke test).
 *
 *  The compile-only CTest cases pe-coupling-header-standalone-* additionally compile each
 *  coupling header on its own, with the bare compiler and without pe's forced config.h.
 */
//=================================================================================================

#if PE_COUPLING_TEST_WITH_MPI_H
#  include <mpi.h>
#endif

#include <pe/interface/coupling/Engine.h>

#if PE_COUPLING_TEST_WITH_MPI_H
#  include <mpi.h>
#endif

#include <cstdio>
#include <string>
#include <type_traits>


//*************************************************************************************************
// The header must not drag in pe's core headers (and with them pe's MPI stand-ins)
//*************************************************************************************************

#if defined( _PE_CORE_MPI_H_ ) || defined( _PE_CORE_WORLD_H_ ) || \
    defined( _PE_SYSTEM_PRECISION_H_ ) || defined( _PE_UTIL_TYPES_H_ ) || \
    defined( _PE_CORE_COLLISIONSYSTEM_H_ )
#  error "pe/interface/coupling/Engine.h includes pe core headers"
#endif

#if !defined( _PE_INTERFACE_COUPLING_ENGINE_H_ ) || !defined( _PE_INTERFACE_COUPLING_TYPES_H_ ) \
    || !defined( _PE_INTERFACE_COUPLING_BUILDINFO_H_ )                                        \
    || !defined( _PE_INTERFACE_COUPLING_EXPORT_H_ )
#  error "the coupling headers were not included"
#endif

// The one macro besides the include guards: the documented symbol-visibility exception
#ifndef PE_COUPLING_API
#  error "PE_COUPLING_API is not defined"
#endif


//*************************************************************************************************
// Compile-time properties of the public types
//*************************************************************************************************

static_assert( !std::is_copy_constructible<pe::coupling::Engine>::value,
               "Engine must not be copyable" );
static_assert( !std::is_copy_assignable<pe::coupling::Engine>::value,
               "Engine must not be copy assignable" );
static_assert( std::is_same<pe::coupling::BodyId,std::uint64_t>::value,
               "BodyId is pe's 64-bit system id" );
static_assert( pe::coupling::noBody == ~std::uint64_t( 0 ), "noBody is the maximum id" );


int main()
{
   int failures( 0 );

   const pe::coupling::BuildInfo info( pe::coupling::buildInfo() );
   std::printf( "pe build: mpi=%d solver=%s gravity=%d buoyancy=%d real=%zu cgal=%d "
                "mobileInfinite=%d sleepMode=%d\n", int( info.mpi ), info.constraintSolver.c_str(),
                int( info.gravity ), int( info.buoyancy ), info.realSize, int( info.cgal ),
                int( info.mobileInfinite ), int( info.sleepMode ) );

   if( info.constraintSolver.find( "pe::response::" ) != 0 ) {
      std::printf( "FAIL: unexpected solver name '%s'\n", info.constraintSolver.c_str() );
      ++failures;
   }
   if( info.realSize != sizeof(float) && info.realSize != sizeof(double) &&
       info.realSize != sizeof(long double) ) {
      std::printf( "FAIL: implausible real size %zu\n", info.realSize );
      ++failures;
   }
   if( pe::coupling::isSupported( pe::coupling::Setting::fluidViscosity ) != true ) {
      std::printf( "FAIL: the fluid viscosity must always be supported\n" );
      ++failures;
   }

#if PE_COUPLING_TEST_ENGINE
   // Use the Engine through the library only (the shared-library smoke test links libpe.so,
   // where every symbol but the exported coupling API is hidden)
   {
      pe::coupling::Engine engine;
      const pe::coupling::MaterialId mat( engine.createMaterial( "headerTestMat", 1000.0, 0.0,
                                                                 0.1, 0.1 ) );
      const pe::coupling::BodyId id( engine.createSphere( { { 0.0, 0.0, 1.0 } }, 0.25, mat ) );
      engine.addLoad( id, { { 0.0, 0.0, 1.0 } }, { { 0.0, 0.0, 0.0 } } );
      engine.applyLoads( 1.0, 1.0e-3 );
      engine.advance( 1.0e-3 );
      const pe::coupling::BodyState s( engine.state( id ) );
      std::printf( "engine: sphere %llu at z = %.6g, v_z = %.6g\n", (unsigned long long)id,
                   s.position[2], s.linearVelocity[2] );
      if( !( s.linearVelocity[2] > 0.0 ) || engine.owner( { { 0.0, 0.0, 1.0 } },
             pe::coupling::QueryMethod::exhaustive ) != id ) {
         std::printf( "FAIL: the engine does not work through the library\n" );
         ++failures;
      }
   }
#endif

#if PE_COUPLING_TEST_WITH_MPI_H
   // Use the MPI header for real: its types and the pe::coupling types side by side
   const MPI_Comm comm( MPI_COMM_WORLD );
   (void)comm;
   std::printf( "compiled next to <mpi.h> (MPI %d.%d)\n", MPI_VERSION, MPI_SUBVERSION );
#endif

   std::printf( failures ? "FAILED (%d)\n" : "PASSED\n", failures );
   return failures ? 1 : 0;
}
