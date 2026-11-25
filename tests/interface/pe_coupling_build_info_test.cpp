//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_build_info_test.cpp
 *  \brief pe::coupling build information, feature support and the one-engine rule
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
 *  The same source is linked against every library variant of the test build (the configured
 *  pe_static and the HardContactAndFluid / HardContactSemiImplicitTimesteppingSolvers
 *  variants). CMake passes the solver the variant was built with as PE_COUPLING_EXPECTED_SOLVER;
 *  the expected body-force semantics come from the table below, written from the solvers'
 *  velocity prediction (doc/technical-notes/euler-lagrange-solver-conventions.md), not from
 *  the trait the implementation reads. Checked:
 *    1. buildInfo() against the solver name and the table, and against what this translation
 *       unit sees of pe's configuration (HAVE_MPI, sizeof(real), PE_USE_CGAL, MOBILE_INFINITE);
 *    2. isSupported() for every setting, and every Engine setter: a supported one takes effect
 *       in pe's world (read back, or for the contact parameters through a contact scenario
 *       whose outcome changes with the error reduction and the iteration limit), an
 *       unsupported one throws std::runtime_error naming the setting and the solver (under
 *       HardContactEulerLagrange: gravity and fluid density; under
 *       HardContactSemiImplicitTimesteppingSolvers: fluid density); values outside pe's
 *       preconditions throw std::invalid_argument;
 *    3. a second Engine in the process throws std::logic_error; after the first is destroyed a
 *       new one can be created, and the world (with its bodies) survives the first.
 *  Serial, no MPI_Init: in an MPI build the test is not registered.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
namespace pc = pe::coupling;

#ifndef PE_COUPLING_EXPECTED_SOLVER
#  error "PE_COUPLING_EXPECTED_SOLVER must name the solver of the linked library"
#endif


//*************************************************************************************************
/*!\brief Expected body-force semantics per solver.
 */
struct Expected
{
   const char*   solver;
   pc::BodyForce gravity;
   pc::BodyForce buoyancy;
};

static const pc::BodyForce bySolver( pc::BodyForce::appliedBySolver );
static const pc::BodyForce byDriver( pc::BodyForce::appliedByDriver );

static const Expected expectedTable[] = {
   { "pe::response::HardContactAndFluid",                        bySolver, bySolver },
   { "pe::response::HardContactSemiImplicitTimesteppingSolvers", bySolver, byDriver },
   { "pe::response::HardContactEulerLagrange",                   byDriver, byDriver },
};
//*************************************************************************************************


static void checkBuildInfo( const Expected& e )
{
   const pc::BuildInfo info( pc::buildInfo() );
   std::printf( "solver %s: gravity=%d buoyancy=%d mpi=%d real=%zu (%d digits) cgal=%d "
                "mobileInfinite=%d sleepMode=%d checkpoints=%d\n", info.constraintSolver.c_str(),
                int( info.gravity ), int( info.buoyancy ), int( info.mpi ), info.realSize,
                info.realDigits, int( info.cgal ), int( info.mobileInfinite ),
                int( info.sleepMode ), int( info.checkpointsSupported ) );

   expect( info.constraintSolver == e.solver, "constraintSolver is " + std::string( e.solver ) );
   expect( info.gravity == e.gravity, "gravity semantics" );
   expect( info.buoyancy == e.buoyancy, "buoyancy semantics" );
   expect( info.sleepMode == ( pe::sleepThreshold != pe::real( 0 ) ), "sleepMode" );
   expect( info.realDigits == std::numeric_limits<pe::real>::digits, "realDigits" );
   expect( info.checkpointsSupported ==
           ( !info.sleepMode && info.realDigits <= std::numeric_limits<double>::digits ),
           "checkpointsSupported" );
   expect( info.mpi == ( HAVE_MPI != 0 ), "mpi matches HAVE_MPI" );
   expect( info.realSize == sizeof( pe::real ), "realSize matches sizeof(pe::real)" );
#ifdef PE_USE_CGAL
   expect( info.cgal, "cgal matches PE_USE_CGAL" );
#else
   expect( !info.cgal, "cgal matches PE_USE_CGAL" );
#endif
   expect( info.mobileInfinite == ( MOBILE_INFINITE != 0 ),
           "mobileInfinite matches MOBILE_INFINITE" );

   // The FEAT3 guard keeps working through the API
   const bool isFluid( info.constraintSolver.find( "HardContactAndFluid" ) != std::string::npos );
   expect( isFluid == ( std::string( e.solver ) == "pe::response::HardContactAndFluid" ),
           "HardContactAndFluid is recognisable by name" );
}


static void checkSettings( const Expected& e )
{
   expect( pc::isSupported( pc::Setting::gravity ) == ( e.gravity == bySolver ),
           "isSupported(gravity)" );
   expect( pc::isSupported( pc::Setting::fluidDensity ) == ( e.buoyancy == bySolver ),
           "isSupported(fluidDensity)" );
   expect( pc::isSupported( pc::Setting::fluidViscosity ), "isSupported(fluidViscosity)" );
   // All three hard-contact solvers have the contact parameters and split impulses
   expect( pc::isSupported( pc::Setting::errorReduction ), "isSupported(errorReduction)" );
   expect( pc::isSupported( pc::Setting::maxIterations ), "isSupported(maxIterations)" );
   expect( pc::isSupported( pc::Setting::contactRelaxation ), "isSupported(contactRelaxation)" );
   expect( pc::isSupported( pc::Setting::splitImpulse ), "isSupported(splitImpulse)" );

   pc::Engine engine;

   // gravity
   const pc::Vec3d g{ { 0.25, -0.5, -9.81 } };
   if( e.gravity == bySolver ) {
      expectThrows<std::invalid_argument>( [&]() {
         engine.setGravity( pc::Vec3d{ { 0.0, std::nan( "" ), 0.0 } } ); }, "NaN gravity" );
      engine.setGravity( g );
      expect( pe::theWorld()->getGravity() == pe::Vec3( 0.25, -0.5, -9.81 ), "gravity reaches pe" );
      expect( coupling_test::sameBits( engine.gravity(), g ), "gravity() reads it back" );
   }
   else {
      expectThrows<std::runtime_error>( [&]() { engine.setGravity( g ); }, "setGravity throws",
                                        "'gravity'", e.solver );
      expect( pe::theWorld()->getGravity() == pe::Vec3( 0, 0, 0 ), "rejected gravity not stored" );
   }

   // fluid density
   if( e.buoyancy == bySolver ) {
      engine.setFluidDensity( 1000.0 );
      expect( pe::Settings::liquidDensity() == pe::real( 1000 ), "fluid density reaches pe" );
      expect( pe::Settings::liquidSolid(), "fluid density switches liquid-solid mode on" );
   }
   else {
      expectThrows<std::runtime_error>( [&]() { engine.setFluidDensity( 1000.0 ); },
                                        "setFluidDensity throws", "'fluidDensity'", e.solver );
      expect( pe::Settings::liquidDensity() == pe::real( 0 ), "rejected density not stored" );
   }
   expectThrows<std::invalid_argument>( [&]() { engine.setFluidViscosity( -1.0 ); },
                                        "negative viscosity" );

   engine.setFluidViscosity( 1.0e-3 );
   expect( pe::theWorld()->getViscosity() == pe::real( 1.0e-3 ), "viscosity reaches pe" );

   // Range checks: pe's preconditions (erp in [0, 1], relaxation > 0, at least one iteration)
   for( const double bad : { -0.1, 1.5, std::nan( "" ) } )
      expectThrows<std::invalid_argument>( [&]() { engine.setErrorReduction( bad ); },
                                           "invalid error reduction", "[0, 1]" );
   for( const double bad : { 0.0, -1.0, std::nan( "" ), HUGE_VAL } )
      expectThrows<std::invalid_argument>( [&]() { engine.setContactRelaxation( bad ); },
                                           "invalid contact relaxation", "positive" );
   expectThrows<std::invalid_argument>( [&]() { engine.setMaxIterations( 0 ); },
                                        "zero iterations", "at least one" );
   engine.setErrorReduction( 0.0 );
   engine.setErrorReduction( 1.0 );

   engine.setSplitImpulse( true );
   expect( engine.splitImpulse(), "split impulse on" );
   expect( pe::theCollisionSystem()->getSplitImpulse(), "split impulse reaches pe" );
   engine.setSplitImpulse( false );
   expect( !engine.splitImpulse(), "split impulse off" );
}


//*************************************************************************************************
/*!\brief A scenario sensitive to the contact parameters; returns the final state bits.
 *
 * Two overlapping spheres at rest (the penetration is removed at a rate set by the error
 * reduction parameter, split impulses off) and a column of four touching spheres on a plane
 * hit from above (the impulse travels down the column only as far as the solver iterates). No
 * gravity is needed, so it works under every solver.
 */
static std::string contactScenario( pc::Engine& engine, pc::MaterialId mat, double erp,
                                    std::size_t iterations, double relaxation )
{
   pe::theWorld()->clear();
   engine.setSplitImpulse( false );
   engine.setErrorReduction( erp );
   engine.setMaxIterations( iterations );
   engine.setContactRelaxation( relaxation );

   engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, mat );
   engine.createSphere( pc::Vec3d{ { 10.0, 0.0, 5.0 } }, 0.5, mat );
   engine.createSphere( pc::Vec3d{ { 10.9, 0.0, 5.0 } }, 0.5, mat );
   pc::BodyId top( pc::noBody );
   for( int k=0; k<4; ++k )
      top = engine.createSphere( pc::Vec3d{ { 0.0, 0.0, 0.5 + 1.0*k } }, 0.5, mat );
   const pc::BodyState s( engine.state( top ) );
   engine.setKinematicState( top, pc::KinematicState{ s.position, s.orientation,
                                                      { { 0.0, 0.0, -1.0 } }, { { 0, 0, 0 } } } );
   for( int i=0; i<3; ++i ) engine.advance( 1.0e-3 );

   std::vector<pc::BodyState> states;
   engine.states( pc::Selection::movable, states );
   std::string bitsText;
   for( const pc::BodyState& b : states ) {
      for( double x : b.position ) bitsText += std::to_string( coupling_test::bits( x ) ) + " ";
      for( double x : b.linearVelocity )
         bitsText += std::to_string( coupling_test::bits( x ) ) + " ";
   }
   return bitsText;
}
//*************************************************************************************************


static void checkContactParameterEffects()
{
   pc::Engine engine;
   const pc::MaterialId mat( engine.createMaterial( "contactEffectMat", 1000.0, 0.0, 0.2, 0.2 ) );
   const std::string base( contactScenario( engine, mat, 0.7, 100, 0.9 ) );
   expect( contactScenario( engine, mat, 0.7, 100, 0.9 ) == base,
           "the contact scenario is deterministic" );
   expect( contactScenario( engine, mat, 0.2, 100, 0.9 ) != base,
           "setErrorReduction changes the response" );
   expect( contactScenario( engine, mat, 0.7, 1, 0.9 ) != base,
           "setMaxIterations changes the response" );
   // pe applies the contact relaxation only to the velocity corrections exchanged between
   // processes (synchronizeVelocities() returns early on one process), so on one process it
   // has no effect in any of the three solvers; pe-coupling-mpi checks its effect across a
   // subdomain cut. Pinned here so a change of that pe behaviour is noticed.
   expect( contactScenario( engine, mat, 0.7, 100, 0.3 ) == base,
           "setContactRelaxation has no effect on one process (pe behaviour)" );
   pe::theWorld()->clear();
}


static void checkOneEngine()
{
   pc::BodyId sphere( pc::noBody );
   {
      pc::Engine first;
      expectThrows<std::logic_error>( []() { pc::Engine second; }, "second Engine throws",
                                      "already exists" );
      const pc::MaterialId mat( first.createMaterial( "oneEngineMat", 2000.0, 0.0, 0.1, 0.1 ) );
      sphere = first.createSphere( pc::Vec3d{ { 0.0, 0.0, 5.0 } }, 0.5, mat, 7 );
   }
   pc::Engine again;
   expect( again.hasBody( sphere ), "the world and its bodies survive the Engine" );
   expect( again.state( sphere ).userId == 7, "the new Engine sees the body" );
}


int main()
{
   const std::string expectedSolver( PE_COUPLING_EXPECTED_SOLVER );
   const Expected* e( nullptr );
   for( const Expected& candidate : expectedTable )
      if( expectedSolver == candidate.solver ) e = &candidate;
   if( e == nullptr ) {
      std::printf( "FAIL: no expectation for solver %s\n", expectedSolver.c_str() );
      return 1;
   }

   checkBuildInfo( *e );
   checkSettings( *e );
   checkContactParameterEffects();
   checkOneEngine();
   return coupling_test::report( "pe_coupling_build_info_test" );
}
