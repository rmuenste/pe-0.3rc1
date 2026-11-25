//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_fluid_test.cpp
 *  \brief pe::coupling under HardContactAndFluid: buoyant first step, settling, substeps, quiet
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
 *  Linked against pe_static_lubstage_fluid (pe_CONSTRAINT_SOLVER = HardContactAndFluid, the
 *  solver of the particle-resolved FBM coupling). Checked:
 *    1. the first step of a sphere at rest gives v = (1 - rho_f/rho_p) g dt (pe applies
 *       gravity and buoyancy itself);
 *    2. a sphere settling under gravity, with a Stokes-drag load handed over every step
 *       (relaxation 0.5), comes to rest on the bottom plane: centre at z = radius, velocity
 *       zero;
 *    3. advance( dt, n ) equals n calls of advance( dt / n ) bit for bit (no load applied),
 *       through an impact on the plane;
 *    4. advance() prints nothing although SimulationConfig's CFD rank is 1 (on which the
 *       solver prints a per-step diagnostic block): it switches only that block off
 *       (setPredictionDiagnostics), so a pe trigger running inside the step still sees rank 1,
 *       and the switch is back on afterwards; the control (a native World::simulationStep on
 *       rank 1) still prints, which shows the capture works and the default is unchanged.
 *  Serial world, no MPI.
 */
//=================================================================================================

#include <pe/config/SimulationConfig.h>
#include <pe/core.h>
#include <pe/core/Trigger.h>
#include <pe/interface/coupling/Engine.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::sameBits;
namespace pc = pe::coupling;

static const double rhoP( 1120.0 ), rhoF( 970.0 ), muF( 0.373 ), radius( 0.0075 );
static const pc::Vec3d gravity{ { 0.0, 0.0, -9.81 } };


//*************************************************************************************************
/*!\brief Builds the settling column: bottom plane z = 0 and one sphere.
 */
static pc::BodyId buildColumn( pc::Engine& engine, pc::MaterialId sphereMat, pc::MaterialId wallMat,
                               double z0 )
{
   engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, wallMat );
   return engine.createSphere( pc::Vec3d{ { 0.0, 0.0, z0 } }, radius, sphereMat );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Runs \a call with stdout redirected into a file; returns the number of bytes printed.
 */
template< typename F >
static long capturedStdout( F call )
{
   const char* path( "pe_coupling_fluid_test.stdout" );
   std::fflush( stdout );
   const int saved( dup( 1 ) );
   const int fd( open( path, O_CREAT | O_TRUNC | O_WRONLY, 0644 ) );
   dup2( fd, 1 );
   close( fd );
   call();
   std::fflush( stdout );
   dup2( saved, 1 );
   close( saved );
   struct stat st;
   return ( stat( path, &st ) == 0 ) ? long( st.st_size ) : -1L;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A pe trigger (run by World::simulationStep() at the end of every step) that records
 *        the CFD rank it sees.
 */
class RankProbe : public pe::Trigger
{
public:
   RankProbe() : calls( 0 ), rank( -1 ) {}
   virtual void trigger() { ++calls; rank = pe::SimulationConfig::getInstance().getCfdRank(); }
   int calls;
   int rank;
};
//*************************************************************************************************


static void checkFirstStepAndQuiet( pc::Engine& engine, pc::MaterialId sphereMat,
                                    pc::MaterialId wallMat )
{
   pe::SimulationConfig::getInstance().setCfdRank( 1 );
   const pc::BodyId s( buildColumn( engine, sphereMat, wallMat, 0.1 ) );
   const double dt( 1.0e-3 );

   RankProbe probe;
   const long printed( capturedStdout( [&]() { engine.advance( dt ); } ) );
   expect( printed == 0, "advance() prints nothing on CFD rank 1 (" + std::to_string( printed ) +
                         " bytes)" );
   expect( probe.calls == 1 && probe.rank == 1,
           "a trigger inside advance() sees the caller's CFD rank (1), got " +
           std::to_string( probe.rank ) );
   expect( pe::SimulationConfig::getInstance().getCfdRank() == 1, "the rank stays 1" );
   expect( pe::theCollisionSystem()->getPredictionDiagnostics(),
           "advance() restores the solver's diagnostics switch (default on)" );

   const pc::Vec3d v( engine.state( s ).linearVelocity );
   const double expected( ( 1.0 - rhoF / rhoP ) * gravity[2] * dt );
   std::printf( "first step: v_z = %.17g, expected (1 - rho_f/rho_p) g dt = %.17g\n",
                v[2], expected );
   expect( v[0] == 0.0 && v[1] == 0.0, "first step: no lateral velocity" );
   expect( std::fabs( v[2] - expected ) <= 1e-12 * std::fabs( expected ),
           "first step: buoyant velocity" );

   // Control: a native step on rank 1 does print (the default behaviour of existing drivers is
   // unchanged), so the capture above would have seen it
   const long control( capturedStdout( []() { pe::theWorld()->simulationStep( 1.0e-3 ); } ) );
   expect( control > 0, "control: the native step prints on rank 1 (capture works)" );
   expect( probe.calls == 2, "the trigger ran in the native step as well" );
   pe::SimulationConfig::getInstance().setCfdRank( 0 );
}


static void checkSettling( pc::Engine& engine, pc::MaterialId sphereMat, pc::MaterialId wallMat )
{
   const pc::BodyId s( buildColumn( engine, sphereMat, wallMat, 0.04 ) );
   const double dt( 1.0e-3 );
   const double c( 6.0 * pe::M_PI * muF * radius );   // Stokes drag coefficient

   double maxSpeed( 0.0 );
   for( int step=0; step<3000; ++step ) {
      const pc::BodyState st( engine.state( s ) );
      const pc::Vec3d drag{ { -c * st.linearVelocity[0], -c * st.linearVelocity[1],
                              -c * st.linearVelocity[2] } };
      engine.addLoad( s, drag, pc::Vec3d{ { 0.0, 0.0, 0.0 } } );
      engine.applyLoads( 0.5, dt );
      engine.advance( dt );
      maxSpeed = std::max( maxSpeed, std::fabs( st.linearVelocity[2] ) );
   }
   const pc::BodyState st( engine.state( s ) );
   std::printf( "settled: z = %.10g (radius %.10g), v = (%.3g, %.3g, %.3g), max |v_z| = %.4g\n",
                st.position[2], radius, st.linearVelocity[0], st.linearVelocity[1],
                st.linearVelocity[2], maxSpeed );
   expect( maxSpeed > 1.0e-3, "settling: the sphere actually fell" );
   expect( std::fabs( st.position[2] - radius ) < 1.0e-3 * radius,
           "settling: resting on the plane" );
   expect( std::fabs( st.linearVelocity[2] ) < 1.0e-6, "settling: at rest" );
   expect( std::fabs( st.position[0] ) < 1.0e-12 && std::fabs( st.position[1] ) < 1.0e-12,
           "settling: no lateral drift" );
}


static std::vector<pc::BodyState> runSubsteps( pc::Engine& engine, pc::MaterialId sphereMat,
                                               pc::MaterialId wallMat, unsigned int substeps,
                                               bool split )
{
   pe::theWorld()->clear();
   engine.setGravity( gravity );
   const pc::BodyId s( buildColumn( engine, sphereMat, wallMat, 0.012 ) );
   pc::KinematicState k{ { { 0.0, 0.0, 0.012 } }, { { 1.0, 0.0, 0.0, 0.0 } },
                         { { 0.02, -0.01, -0.2 } }, { { 0.0, 3.0, 0.0 } } };
   engine.setKinematicState( s, k );
   const double dt( 2.0e-3 );
   std::vector<pc::BodyState> trace;
   for( int step=0; step<100; ++step ) {
      if( split ) {
         const double h( dt / double( substeps ) );
         for( unsigned int i=0; i<substeps; ++i ) engine.advance( h );
      }
      else {
         engine.advance( dt, substeps );
      }
      trace.push_back( engine.state( s ) );
   }
   return trace;
}


static void checkSubsteps( pc::Engine& engine, pc::MaterialId sphereMat, pc::MaterialId wallMat )
{
   const std::vector<pc::BodyState> a( runSubsteps( engine, sphereMat, wallMat, 4, false ) );
   const std::vector<pc::BodyState> b( runSubsteps( engine, sphereMat, wallMat, 4, true ) );
   const std::vector<pc::BodyState> one( runSubsteps( engine, sphereMat, wallMat, 1, false ) );
   int diff( 0 );
   for( std::size_t i=0; i<a.size(); ++i )
      if( !sameBits( a[i].position, b[i].position ) ||
          !sameBits( a[i].linearVelocity, b[i].linearVelocity ) ||
          !sameBits( a[i].angularVelocity, b[i].angularVelocity ) ||
          !sameBits( a[i].orientation, b[i].orientation ) )
         ++diff;
   std::printf( "substeps: %d of %zu steps differ; final z %.10g (4 substeps) vs %.10g "
                "(no substeps)\n", diff, a.size(), a.back().position[2], one.back().position[2] );
   double minZ( 1.0 );
   for( const pc::BodyState& s : a ) minZ = std::min( minZ, s.position[2] );
   expect( diff == 0, "advance( dt, 4 ) == 4 x advance( dt/4 ), bit for bit" );
   expect( minZ < 1.01 * radius, "substeps: the run includes the impact on the plane" );
   expect( !sameBits( a[10].position, one[10].position ),
           "substeps: 4 substeps differ from 1 step" );
}


int main()
{
   pc::Engine engine;
   engine.setGravity( gravity );
   engine.setFluidDensity( rhoF );
   engine.setFluidViscosity( muF );
   const pc::MaterialId sphereMat( engine.createMaterial( "fluidTestSphere", rhoP, 0.0, 0.2,
                                                          0.2 ) );
   const pc::MaterialId wallMat( engine.createMaterial( "fluidTestWall", 2500.0, 0.0, 0.2, 0.2 ) );

   checkFirstStepAndQuiet( engine, sphereMat, wallMat );
   pe::theWorld()->clear();
   engine.setGravity( gravity );
   checkSettling( engine, sphereMat, wallMat );
   checkSubsteps( engine, sphereMat, wallMat );
   return coupling_test::report( "pe_coupling_fluid_test" );
}
