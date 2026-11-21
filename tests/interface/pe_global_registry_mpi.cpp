//=================================================================================================
/*!
 *  \file tests/interface/pe_global_registry_mpi.cpp
 *  \brief Registry of non-fixed global bodies (globalNonfixedBodies_) of the hard-contact solvers
 *         on two ranks (cut at x = 0), restitution OFF (the default path). Review round 3,
 *         finding 1: the set followed the fixedness at add() / remove() time and survived clear().
 *
 *  usage: pe_global_registry_mpi fixdestroy | clear | unfix
 *    fixdestroy  a global sphere G registered mobile, then fixed and destroyed; a local pair keeps
 *                colliding: its trajectory must equal the run without G (a stale set entry was
 *                dereferenced by the velocity synchronisation);
 *    clear       a world with a mobile global sphere is cleared and repopulated with the pair
 *                only: the pair's trajectory must equal the run in a fresh world;
 *    unfix       G created fixed (not registered) and unfixed before the first step, hit by a
 *                local sphere: the trajectory (sphere and both replicas of G) must equal the run
 *                with G created mobile (registered).
 *  The references run where they cannot inherit a stale entry (before the scenario, or after a
 *  scenario that registered nothing). Comparison bitwise. Built against both solvers.
 */
//=================================================================================================

#include <pe/core.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <mpi.h>

using namespace pe;

namespace {

int rank_ = 0;
const double R = 0.01, U = 0.5, DT = 1e-4;

MaterialID material() { return createMaterial( 1000.0, 0.0, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 ); }

//! Velocities of the local spheres with user ids 1..2 (summed over owners) and of the global
//! sphere 100 on each rank: v[0..5] pair, v[6..8] G on rank 0, v[9..11] G on rank 1.
std::vector<double> state()
{
   std::vector<double> v( 12, 0.0 );
   double g[3] = { 0, 0, 0 };
   for( World::Iterator it = theWorld()->begin(); it != theWorld()->end(); ++it ) {
      BodyID b = *it;
      if( b->getType() != sphereType ) continue;
      const int id = static_cast<int>( b->getID() );
      if( b->isGlobal() && id == 100 ) { for( int c = 0; c < 3; ++c ) g[c] = b->getLinearVel()[c]; continue; }
      if( b->isRemote() || id < 1 || id > 2 ) continue;
      for( int c = 0; c < 3; ++c ) v[3 * ( id - 1 ) + c] = b->getLinearVel()[c];
   }
   MPI_Allreduce( MPI_IN_PLACE, v.data(), 6, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   double all[6];
   MPI_Allgather( g, 3, MPI_DOUBLE, all, 3, MPI_DOUBLE, MPI_COMM_WORLD );
   for( int k = 0; k < 6; ++k ) v[6 + k] = all[k];
   return v;
}

//! The colliding local pair on rank 0 (head-on, inelastic, repeated contact after a while).
void pair( MaterialID m )
{
   const Vec3 a( -0.06, 0, 0 ), b( -0.06 + 2 * R + 2 * U * DT * 3.9, 0, 0 );
   if( theWorld()->ownsPoint( a ) ) { SphereID s = createSphere( 1, a, R, m ); s->setLinearVel( Vec3(  U, 0.01, 0 ) ); }
   if( theWorld()->ownsPoint( b ) ) { SphereID s = createSphere( 2, b, R, m ); s->setLinearVel( Vec3( -U, 0, 0 ) ); }
}

std::vector<double> steps( int n )
{
   std::vector<double> traj;
   for( int i = 0; i < n; ++i ) {
      theWorld()->simulationStep( DT );
      const std::vector<double> s( state() );
      traj.insert( traj.end(), s.begin(), s.end() );
   }
   return traj;
}

void fresh()
{
   theWorld()->clear();
   theWorld()->setGravity( 0, 0, 0 );
   theWorld()->setDamping( 1 );
}

int compare( const std::vector<double>& a, const std::vector<double>& b, const char* what )
{
   size_t diff = 0, first = a.size();
   for( size_t k = 0; k < a.size() && k < b.size(); ++k )
      if( std::memcmp( &a[k], &b[k], sizeof( double ) ) != 0 ) { ++diff; if( first == a.size() ) first = k; }
   if( a.size() != b.size() ) diff += 1;
   if( rank_ == 0 ) {
      if( diff == 0 ) std::printf( "%s: bitwise identical (%zu values)\n", what, a.size() );
      else std::printf( "%s: %zu of %zu values differ (first at step %zu, entry %zu: %.17g vs %.17g)\n", what, diff, a.size(),
                        first / 12, first % 12, first < a.size() ? a[first] : 0.0, first < b.size() ? b[first] : 0.0 );
   }
   return diff == 0 ? 0 : 1;
}

int runFixDestroy()
{
   fresh(); MaterialID m( material() ); pair( m ); theWorld()->synchronize();
   std::vector<double> ref( steps( 3 ) ), r2( steps( 20 ) ); ref.insert( ref.end(), r2.begin(), r2.end() );
   fresh(); m = material(); pair( m );
   SphereID g( nullptr );
   pe_GLOBAL_SECTION { g = createSphere( 100, Vec3( 0, 0.2, 0 ), R, m ); }   // mobile global, far away
   theWorld()->synchronize();
   std::vector<double> sc( steps( 3 ) );
   g->setFixed( true );
   destroy( g );
   std::vector<double> s2( steps( 20 ) ); sc.insert( sc.end(), s2.begin(), s2.end() );
   for( size_t k = 0; k < 3 * 12; k += 12 ) for( int c = 6; c < 12; ++c ) { ref[k + c] = 0; sc[k + c] = 0; }   // G only exists in the scenario
   return compare( ref, sc, "fix + destroy, then steps: pair trajectory against the run without the global body" );
}

int runClear()
{
   fresh(); MaterialID m( material() ); pair( m ); theWorld()->synchronize();
   const std::vector<double> ref( steps( 20 ) );
   fresh(); m = material();
   pe_GLOBAL_SECTION { createSphere( 100, Vec3( 0, 0.2, 0 ), R, m ); }
   theWorld()->synchronize();
   steps( 2 );
   fresh(); m = material(); pair( m ); theWorld()->synchronize();
   const std::vector<double> sc( steps( 20 ) );
   return compare( ref, sc, "clear + repopulate, then steps: pair trajectory against a fresh world" );
}

int runUnfix()
{
   const Vec3 a( -2 * R - U * DT * 3.9, 0.05, 0 );   // touches G after about 4 steps
   // scenario first: G registered fixed, unfixed before the first step
   fresh(); MaterialID m( material() );
   SphereID g( nullptr );
   pe_GLOBAL_SECTION { g = createSphere( 100, Vec3( 0, 0.05, 0 ), R, m ); }
   g->setFixed( true );
   g->setFixed( false );
   if( theWorld()->ownsPoint( a ) ) { SphereID s = createSphere( 1, a, R, m ); s->setLinearVel( Vec3( U, 0, 0 ) ); }
   theWorld()->synchronize();
   const std::vector<double> sc( steps( 30 ) );
   // reference: G created mobile
   fresh(); m = material();
   pe_GLOBAL_SECTION { createSphere( 100, Vec3( 0, 0.05, 0 ), R, m ); }
   if( theWorld()->ownsPoint( a ) ) { SphereID s = createSphere( 1, a, R, m ); s->setLinearVel( Vec3( U, 0, 0 ) ); }
   theWorld()->synchronize();
   const std::vector<double> ref( steps( 30 ) );
   const size_t last( ref.size() - 12 );
   if( rank_ == 0 )
      std::printf( "unfix: G after the impact, registered mobile: rank 0 %.9f, rank 1 %.9f; registered fixed then unfixed: rank 0 %.9f, rank 1 %.9f\n",
                   ref[last + 6], ref[last + 9], sc[last + 6], sc[last + 9] );
   return compare( ref, sc, "registered fixed then unfixed against registered mobile (sphere and both replicas of G)" );
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank_ );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 || argc < 2 ) { if( rank_ == 0 ) std::cerr << "usage (two ranks): pe_global_registry_mpi fixdestroy | clear | unfix\n"; MPI_Abort( MPI_COMM_WORLD, 2 ); }
   if( rank_ == 0 ) { defineLocalDomain( HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); connect( 1, HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); }
   else             { defineLocalDomain( HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); connect( 0, HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); }
   const std::string mode( argv[1] );
   int rc = 2;
   if( mode == "fixdestroy" ) rc = runFixDestroy();
   else if( mode == "clear" ) rc = runClear();
   else if( mode == "unfix" ) rc = runUnfix();
   int any = 0;
   MPI_Allreduce( &rc, &any, 1, MPI_INT, MPI_MAX, MPI_COMM_WORLD );
   if( rank_ == 0 ) std::printf( "pe_global_registry_mpi %s: %s\n", mode.c_str(), any == 0 ? "all checks passed" : "FAILED" );
   MPI_Finalize();
   return any == 0 ? 0 : 1;
}
