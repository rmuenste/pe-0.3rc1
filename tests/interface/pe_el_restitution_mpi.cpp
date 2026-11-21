//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_mpi.cpp
 *  \brief Restitution of HardContactEulerLagrange across a process boundary (two ranks).
 *
 *  The EL terminal-velocity setup (pe/interface/setup/el_terminal_velocity.h) builds the world
 *  from a deck written by the test: processesX_ = 2 on x in [-0.1, 0.1] (cut at x = 0), ten
 *  spheres of radius 0.01 from the xyz file, gravity 0, restitution_ 0.9, friction 0, and
 *  restitutionEnabled_ true:
 *
 *    pair 1   spheres 0, 1 straddle the cut (0 on rank 0, 1 on rank 1 with a shadow on rank 0)
 *    pair 2   spheres 2, 3, the same configuration inside rank 0
 *    chain 1  sphere 4 into the touching pair 5-6; the 5-6 contact straddles the cut
 *    chain 2  spheres 7, 8-9, the same configuration inside rank 0
 *
 *  Checked (on rank 0, from positions and velocities reduced from the owners every step):
 *    - pair 1 ends with the velocities of pair 2 to rounding (1e-14 relative);
 *    - pair 1's relative velocity after the impact is the value the relaxation recurrence
 *      predicts: in a multi-process run every synchronizeVelocities() folds the corrections with
 *      alpha = relaxationParam_ = 0.9, so after n sweeps g_n - T = (1 - alpha)^n (g_0 - T), with T the
 *      Newton target e |v_rel| and g_0 the velocity after the compression solve (here the
 *      Baumgarte push-out of the impact step's overlap, 0.07 |v_rel|). The pairs impact with the
 *      pass held to its minimum of n = 4 sweeps (tolerance 1): -0.899917 |v_rel| (1e-10). The
 *      chains impact later with the default tolerance (sweeps until converged);
 *    - momentum of each pair and chain conserved (1e-12);
 *    - the support contact of each chain never closes (normal velocity >= -1e-9 u after every
 *      step);
 *    - chain 1 against chain 2 within 1e-6 u: not bitwise, because Gauss-Seidel updates inside a
 *      process are seen at once and those from the other process only at the synchronisation;
 *    - a finite-mass global sphere is refused on both ranks while restitution is on.
 *
 *  Under a configured solver other than HardContactEulerLagrange the test returns 77 (skipped).
 */
//=================================================================================================

#include <pe/interface/setup/el_terminal_velocity.h>
#include <pe/interface/support/el_optional_api.h>

#include <pe/core.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <type_traits>

#include <mpi.h>

namespace {

void writeFile( const std::string& name, const std::string& body )
{
   std::ofstream out( name );
   out << body;
}

const int N = 10;
const bool kIsEulerLagrange = pe::HasElHydroStateQuery<pe::RigidBody>::value;

//! Positions (x) and velocities (x) of all spheres by user id, reduced from the owners.
void gather( double* x, double* v, double* m, double* owner, int rank )
{
   for( int k = 0; k < N; ++k ) { x[k] = 0; v[k] = 0; m[k] = 0; owner[k] = 0; }
   pe::WorldID world = pe::theWorld();
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      pe::BodyID b = *it;
      if( b->getType() != pe::sphereType || b->isRemote() || b->isGlobal() ) continue;
      const int id = static_cast<int>( b->getID() );
      if( id < 0 || id >= N ) continue;
      x[id] = b->getPosition()[0];
      v[id] = b->getLinearVel()[0];
      m[id] = b->getMass();
      owner[id] = rank;
   }
   MPI_Allreduce( MPI_IN_PLACE, x, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, v, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, m, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, owner, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
}

template< typename CS >
int run( CS& cs, int rank )
{
   bool valid = true;
   const auto check = [&valid, rank]( bool ok, const char* what ) {
      if( !ok ) {
         std::cerr << "rank " << rank << " FAILED: " << what << "\n";
         valid = false;
      }
   };
   const double r = 0.01, u = 0.5, dt = 1e-4, e = 0.9, alpha = 0.9, erp = 0.7;

   check( cs.getRestitution(), "the setup switched restitution on from the deck" );
   const size_t shadows = cs.getBodyShadowCopyStorage().size();
   std::printf( "rank %d: %zu shadow copies\n", rank, shadows );
   check( rank != 0 || shadows == 2, "rank 0 holds the shadow copies of spheres 1 and 6" );

   pe::WorldID world = pe::theWorld();
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      pe::BodyID b = *it;
      if( b->getType() != pe::sphereType || b->isRemote() ) continue;
      const int id = static_cast<int>( b->getID() );
      const double vx = ( id == 0 || id == 2 || id == 4 || id == 7 ) ? u : ( id == 1 || id == 3 ) ? -u : 0.0;
      b->setLinearVel( pe::Vec3( vx, 0, 0 ) );
   }
   world->synchronize();

   double x[N], v[N], m[N], owner[N];
   gather( x, v, m, owner, rank );
   const double p0[4] = { m[0] * v[0] + m[1] * v[1], m[2] * v[2] + m[3] * v[3],
                          m[4] * v[4] + m[5] * v[5] + m[6] * v[6], m[7] * v[7] + m[8] * v[8] + m[9] * v[9] };
   // Pairs impact in step 6 with the post-pass stopped at its minimum of four sweeps (tolerance 1:
   // the relaxation recurrence below applies to exactly n = 4); the chains impact in step 16 with
   // the default tolerance (sweeps until converged).
   const pe::real defaultTolerance = cs.getRestitutionTolerance();
   cs.setRestitutionTolerance( 1.0 );
   double worstSupport = 0.0;
   size_t pairSweeps = 0, chainSweeps = 0;
   for( int i = 0; i < 40; ++i ) {
      if( i == 10 ) cs.setRestitutionTolerance( defaultTolerance );
      world->simulationStep( dt );
      if( i < 10 ) pairSweeps = std::max( pairSweeps, cs.getRestitutionPassStats().sweeps );
      else chainSweeps = std::max( chainSweeps, cs.getRestitutionPassStats().sweeps );
      gather( x, v, m, owner, rank );
      worstSupport = std::min( worstSupport, std::min( v[6] - v[5], v[9] - v[8] ) );
   }

   if( rank == 0 ) {
      std::printf( "owners:" );
      for( int k = 0; k < N; ++k ) std::printf( " %d:%.0f", k, owner[k] );
      std::printf( "\npair 1 (across the cut): v = %.17g %.17g\n", v[0], v[1] );
      std::printf( "pair 2 (inside rank 0):  v = %.17g %.17g\n", v[2], v[3] );
      const double rel1 = ( v[1] - v[0] ) / ( -2.0 * u ), rel2 = ( v[3] - v[2] ) / ( -2.0 * u );
      const double g0 = erp * 0.1;   // push-out of the overlap 0.1 * 2u dt, relative to |v_rel| = 2u
      const double predicted = -( e - std::pow( 1.0 - alpha, 4 ) * ( e - g0 ) );
      std::printf( "v_rel' / v_rel: pair 1 %.15f, pair 2 %.15f, recurrence (1 - alpha)^4 prediction %.15f (sweeps %zu)\n", rel1, rel2, predicted, pairSweeps );
      std::printf( "chains: %zu sweeps at the impact step (converged to the default tolerance)\n", chainSweeps );
      std::printf( "chain 1 (support across the cut): v = %.12f %.12f %.12f\n", v[4], v[5], v[6] );
      std::printf( "chain 2 (inside rank 0):           v = %.12f %.12f %.12f\n", v[7], v[8], v[9] );
      std::printf( "chain 1 - chain 2: %.3e %.3e %.3e; most closing support contact over the run %.3e\n",
                   v[4] - v[7], v[5] - v[8], v[6] - v[9], worstSupport );
      check( owner[0] == 0 && owner[1] == 1 && owner[2] == 0 && owner[3] == 0, "pair 1 straddles the cut, pair 2 is inside rank 0" );
      check( owner[5] == 0 && owner[6] == 1 && owner[8] == 0 && owner[9] == 0, "chain 1's support contact straddles the cut, chain 2 is inside rank 0" );
      check( std::fabs( v[0] - v[2] ) <= 1e-14 * u && std::fabs( v[1] - v[3] ) <= 1e-14 * u,
             "the impact across the process boundary gives the result inside one domain" );
      check( pairSweeps == 4 && std::fabs( rel1 - predicted ) <= 1e-10, "relaxation recurrence: v_rel' = -( e - (1 - alpha)^4 ( e - g_0 ) ) v_rel after 4 sweeps" );
      const double p1[4] = { m[0] * v[0] + m[1] * v[1], m[2] * v[2] + m[3] * v[3],
                             m[4] * v[4] + m[5] * v[5] + m[6] * v[6], m[7] * v[7] + m[8] * v[8] + m[9] * v[9] };
      for( int k = 0; k < 4; ++k ) check( std::fabs( p1[k] - p0[k] ) <= 1e-12 * m[0] * u, "momentum of each pair and chain conserved" );
      check( worstSupport >= -1e-9 * u, "the support contact of a chain never closes" );
      check( std::fabs( v[4] - v[7] ) <= 1e-6 * u && std::fabs( v[5] - v[8] ) <= 1e-6 * u && std::fabs( v[6] - v[9] ) <= 1e-6 * u,
             "chain across the cut against the chain inside one domain within 1e-6 u" );
   }

   // A finite-mass global body: refused on every rank while restitution is on.
   const pe::MaterialID mat = pe::createMaterial( 1000.0, 0.9, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 );
   pe_GLOBAL_SECTION {
      pe::createSphere( 100, pe::Vec3( 0.0, -0.04, -0.04 ), r, mat );
   }
   int refused = 0;
   try { world->simulationStep( dt ); }
   catch( const std::runtime_error& ex ) {
      refused = std::string( ex.what() ).find( "global" ) != std::string::npos ? 1 : 0;
      if( rank == 0 ) std::printf( "mobile global body: refused: %s\n", ex.what() );
   }
   int refusedAll = 0;
   MPI_Allreduce( &refused, &refusedAll, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) check( refusedAll == 2, "a finite-mass global body is refused on both ranks" );

   const int localFailure = valid ? 0 : 1;
   int failures = 0;
   MPI_Allreduce( &localFailure, &failures, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) {
      if( failures == 0 ) std::cout << "pe_el_restitution_mpi: all checks passed\n";
      else std::cerr << "pe_el_restitution_mpi: " << failures << " rank(s) reported failures\n";
   }
   return failures == 0 ? 0 : 1;
}

//! Runs the checks only for the EL solver (the body is not even instantiated for another one).
template< typename CS >
int dispatch( CS& cs, int rank )
{
   if constexpr ( pe::HasElHydroStateQuery<pe::RigidBody>::value && std::is_class<CS>::value ) return run( cs, rank );
   else { (void)cs; (void)rank; return 77; }
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );

   int rank = 0, size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );

   // The compiled solver decides, not the CMake cache entry (PE_PREPROCESSOR_FLAGS or
   // CMAKE_CXX_FLAGS can select another one).
   if( !kIsEulerLagrange ) {
      if( rank == 0 ) std::printf( "pe_el_restitution_mpi: the configured collision system is not HardContactEulerLagrange, skipped\n" );
      MPI_Finalize();
      return 77;
   }
   if( size != 2 ) {
      if( rank == 0 ) std::cerr << "pe_el_restitution_mpi requires exactly two MPI ranks\n";
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }

   const double r = 0.01, u = 0.5, dt = 1e-4;
   const double gap2 = 2.0 * u * dt * 5.9;   // pairs: both move, the gap closes after 5.9 steps
   const double gap1 = u * dt * 15.9;        // chains: only the first sphere moves; impact in step 16
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
         "  \"seedMode_\": \"file\", \"seedMinGap_\": 0.0, \"seedAllowContact_\": true, \"xyzFilePath_\": \"particles.xyz\",\n"
         "  \"vtk_\": false, \"useCheckpointer_\": false, \"resume_\": false\n"
         "}\n" );
      const double xa1 = -0.015, xa2 = -0.07;
      const double xb1 = -0.012, xb2 = -0.062;   // chains: first sphere of the touching pair
      char xyz[2048];
      std::snprintf( xyz, sizeof( xyz ),
         "%.17g 0.0 0.0\n%.17g 0.0 0.0\n%.17g 0.03 0.0\n%.17g 0.03 0.0\n"
         "%.17g 0.0 0.03\n%.17g 0.0 0.03\n%.17g 0.0 0.03\n"
         "%.17g 0.03 0.03\n%.17g 0.03 0.03\n%.17g 0.03 0.03\n",
         xa1, xa1 + 2.0 * r + gap2, xa2, xa2 + 2.0 * r + gap2,
         xb1 - 2.0 * r - gap1, xb1, xb1 + 2.0 * r,
         xb2 - 2.0 * r - gap1, xb2, xb2 + 2.0 * r );
      writeFile( "particles.xyz", xyz );
   }
   MPI_Barrier( MPI_COMM_WORLD );

   pe::setupELTerminalVelocity( MPI_COMM_WORLD, -0.1, 0.1, -0.05, 0.05, -0.05, 0.05 );

   const int rc = dispatch( *pe::theCollisionSystem(), rank );
   MPI_Finalize();
   return rc;
}
