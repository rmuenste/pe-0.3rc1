//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_mpi.cpp
 *  \brief Restitution of HardContactEulerLagrange across a process boundary (two ranks).
 *
 *  The EL terminal-velocity setup (pe/interface/setup/el_terminal_velocity.h) builds the world
 *  from a deck written by the test: processesX_ = 2 on x in [-0.1, 0.1] (cut at x = 0), four
 *  spheres of radius 0.01 from the xyz file, gravity 0, restitution_ 0.9, friction 0, and
 *  restitutionEnabled_ true. Pair 1 straddles the cut (sphere 0 owned by rank 0, sphere 1 by
 *  rank 1 with a shadow copy on rank 0, the contact treated on rank 0); pair 2 is the same
 *  configuration inside the domain of rank 0. Both pairs approach at +-u and bounce.
 *
 *  Checked: pair 1 ends with the velocities of pair 2 to rounding (1e-14 relative), the
 *  relative velocity after the impact is -e times before within 2e-4 (in a multi-process run
 *  every synchronizeVelocities() folds the corrections with relaxationParam_ = 0.9, so four
 *  post-pass sweeps deliver 1 - 0.1^4 of the Newton bounce), momentum of each pair is
 *  conserved (1e-12), and the setup switched the solver on from the deck.
 */
//=================================================================================================

#include <pe/interface/setup/el_terminal_velocity.h>

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <mpi.h>

namespace {

void writeFile( const std::string& name, const std::string& body )
{
   std::ofstream out( name );
   out << body;
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );

   int rank = 0, size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 ) {
      if( rank == 0 ) std::cerr << "pe_el_restitution_mpi requires exactly two MPI ranks\n";
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }

   const double r = 0.01, u = 0.5, dt = 1e-4, e = 0.9;
   const double gap = 2.0 * u * dt * 5.9;   // closes after 5.9 steps: the impact step starts with a small overlap
   const double xa1 = -0.015, xb1 = xa1 + 2.0 * r + gap;   // pair 1: across x = 0
   const double xa2 = -0.07,  xb2 = xa2 + 2.0 * r + gap;   // pair 2: inside rank 0
   if( rank == 0 ) {
      writeFile( "example.json",
         "{\n"
         "  \"processesX_\": 2, \"processesY_\": 1, \"processesZ_\": 1,\n"
         "  \"stepsize_\": 0.0001, \"timesteps_\": 40, \"substeps_\": 1,\n"
         "  \"benchRadius_\": 0.01, \"particleDensity_\": 1000.0,\n"
         "  \"restitution_\": 0.9, \"staticFriction_\": 0.0, \"dynamicFriction_\": 0.0,\n"
         "  \"restitutionEnabled_\": true,\n"
         "  \"fluidDensity_\": 1.0, \"fluidViscosity_\": 1e-5,\n"
         "  \"gravity_\": [0.0, 0.0, 0.0],\n"
         "  \"seedMode_\": \"file\", \"seedMinGap_\": 1e-6, \"xyzFilePath_\": \"particles.xyz\",\n"
         "  \"vtk_\": false, \"useCheckpointer_\": false, \"resume_\": false\n"
         "}\n" );
      char xyz[512];
      std::snprintf( xyz, sizeof( xyz ), "%.17g 0.0 0.0\n%.17g 0.0 0.0\n%.17g 0.03 0.0\n%.17g 0.03 0.0\n", xa1, xb1, xa2, xb2 );
      writeFile( "particles.xyz", xyz );
   }
   MPI_Barrier( MPI_COMM_WORLD );

   pe::setupELTerminalVelocity( MPI_COMM_WORLD, -0.1, 0.1, -0.05, 0.05, -0.05, 0.05 );

   bool valid = true;
   const auto check = [&valid, rank]( bool ok, const char* what ) {
      if( !ok ) {
         std::cerr << "rank " << rank << " FAILED: " << what << "\n";
         valid = false;
      }
   };

   pe::CollisionSystemID cs = pe::theCollisionSystem();
   check( cs->getRestitution(), "the setup switched restitution on from the deck" );

   // Approach velocities on the owners, then one synchronisation for the shadow copies
   pe::WorldID world = pe::theWorld();
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      pe::BodyID b = *it;
      if( b->getType() != pe::sphereType || b->isRemote() ) continue;
      b->setLinearVel( pe::Vec3( ( b->getID() % 2 == 0 ) ? u : -u, 0, 0 ) );
   }
   world->synchronize();
   const size_t shadows = cs->getBodyShadowCopyStorage().size();
   std::printf( "rank %d: %zu shadow cop%s\n", rank, shadows, shadows == 1 ? "y" : "ies" );
   check( rank != 0 || shadows == 1, "rank 0 holds the shadow copy of sphere 1 (the pair straddles the cut)" );

   for( int i = 0; i < 40; ++i )
      world->simulationStep( dt );

   // Final velocities and masses by user id, reduced from the owners
   double v[4] = { 0, 0, 0, 0 }, m[4] = { 0, 0, 0, 0 }, owner[4] = { 0, 0, 0, 0 };
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      pe::BodyID b = *it;
      if( b->getType() != pe::sphereType || b->isRemote() ) continue;
      const int id = static_cast<int>( b->getID() );
      v[id] = b->getLinearVel()[0];
      m[id] = b->getMass();
      owner[id] = rank;
   }
   MPI_Allreduce( MPI_IN_PLACE, v, 4, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, m, 4, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, owner, 4, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );

   if( rank == 0 ) {
      std::printf( "owners: sphere 0 rank %.0f, 1 rank %.0f, 2 rank %.0f, 3 rank %.0f\n", owner[0], owner[1], owner[2], owner[3] );
      std::printf( "pair 1 (across the cut): v = %.17g %.17g\n", v[0], v[1] );
      std::printf( "pair 2 (inside rank 0):  v = %.17g %.17g\n", v[2], v[3] );
      const double rel1 = ( v[1] - v[0] ) / ( -2.0 * u ), rel2 = ( v[3] - v[2] ) / ( -2.0 * u );
      std::printf( "v_rel' / v_rel: pair 1 %.15f, pair 2 %.15f (Newton -e = %.2f; 4 sweeps under relaxation 0.9: %.6f)\n",
                   rel1, rel2, -e, -e * ( 1.0 - 1e-4 ) );
      std::printf( "pair 1 - pair 2: %.3e %.3e\n", v[0] - v[2], v[1] - v[3] );
      check( owner[0] == 0 && owner[1] == 1 && owner[2] == 0 && owner[3] == 0, "pair 1 straddles the cut, pair 2 is inside rank 0" );
      check( std::fabs( v[0] - v[2] ) <= 1e-14 * u && std::fabs( v[1] - v[3] ) <= 1e-14 * u,
             "the impact across the process boundary gives the result inside one domain" );
      check( std::fabs( rel1 + e ) <= 2e-4 * e, "Newton law across the boundary (within the relaxation deficit 1e-4)" );
      check( std::fabs( m[0] * v[0] + m[1] * v[1] - ( m[0] - m[1] ) * u ) <= 1e-12 * m[0] * u, "pair 1 momentum conserved" );
      check( std::fabs( m[2] * v[2] + m[3] * v[3] - ( m[2] - m[3] ) * u ) <= 1e-12 * m[2] * u, "pair 2 momentum conserved" );
   }

   const int localFailure = valid ? 0 : 1;
   int failures = 0;
   MPI_Allreduce( &localFailure, &failures, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) {
      if( failures == 0 ) std::cout << "pe_el_restitution_mpi: all checks passed\n";
      else std::cerr << "pe_el_restitution_mpi: " << failures << " rank(s) reported failures\n";
   }

   MPI_Finalize();
   return failures == 0 ? 0 : 1;
}
