//=================================================================================================
/*!
 *  \file tests/interface/pe_global_registry_mpi.cpp
 *  \brief Registry of non-fixed global bodies (globalNonfixedBodies_) of the hard-contact solvers
 *         on two ranks (cut at x = 0), restitution OFF (the default path). Review round 3,
 *         finding 1: the set followed the fixedness at add() / remove() time and survived clear().
 *
 *  usage: pe_global_registry_mpi fixdestroy | clear | fixstepunfix | fixstepunfix-iter | unfixref | unfix
 *    fixdestroy         a global sphere G registered mobile, then fixed and destroyed; a local pair
 *                       keeps colliding: its trajectory must equal the run without G;
 *    clear              a world with a mobile global sphere is cleared and repopulated with the pair
 *                       only: the pair's trajectory must equal the run in a fresh world;
 *    fixstepunfix[-iter] G fixed on every rank, a step (the rebuilt set omits G), unfixed and
 *                       destroyed before the next step, through destroy( BodyID ) or
 *                       World::destroy( Iterator ): no assertion (the assertion-enabled build is
 *                       the test), the pair's trajectory equals the run without G;
 *    unfixref / unfix   G REGISTERED WHILE FIXED (taken out of the world, fixed, added again), then
 *                       unfixed before the first step and hit by a local sphere, against G registered
 *                       mobile (unfixref, a separate process writing the reference to a file, so the
 *                       reference never follows the scenario's clear()); a force on G from rank 0
 *                       only must reach both replicas through synchronizeForces(); the trajectory
 *                       (sphere and both replicas of G) must be bitwise the reference.
 *  The in-process references run before their scenario in a fresh world. Comparison bitwise. Built
 *  against both solvers.
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

//! Fix on all ranks -> step (the rebuild omits G) -> unfix -> destroy before the next step,
//! through destroy( BodyID ) (iter = false) or World::destroy( Iterator ) (iter = true). With
//! assertions enabled the removal of a body that is mobile but not in the cached set aborted.
int runFixStepUnfix( bool iter )
{
   fresh(); MaterialID m( material() ); pair( m ); theWorld()->synchronize();
   std::vector<double> ref( steps( 6 ) ), r2( steps( 20 ) ); ref.insert( ref.end(), r2.begin(), r2.end() );
   fresh(); m = material(); pair( m );
   SphereID g( nullptr );
   pe_GLOBAL_SECTION { g = createSphere( 100, Vec3( 0, 0.2, 0 ), R, m ); }
   theWorld()->synchronize();
   std::vector<double> sc( steps( 3 ) );
   g->setFixed( true );                                    // on every rank
   const std::vector<double> s1( steps( 3 ) );             // the rebuild omits G
   sc.insert( sc.end(), s1.begin(), s1.end() );
   g->setFixed( false );
   if( iter ) {
      for( World::Iterator it = theWorld()->begin(); it != theWorld()->end(); ++it )
         if( *it == g ) { theWorld()->destroy( it ); break; }
   }
   else destroy( g );
   std::vector<double> s2( steps( 20 ) ); sc.insert( sc.end(), s2.begin(), s2.end() );
   for( size_t k = 0; k < 6 * 12; k += 12 ) for( int c = 6; c < 12; ++c ) { ref[k + c] = 0; sc[k + c] = 0; }   // G only in the scenario
   return compare( ref, sc, iter ? "fix, step, unfix, World::destroy( Iterator ): pair trajectory against the run without the global body"
                                 : "fix, step, unfix, destroy( BodyID ): pair trajectory against the run without the global body" );
}

//! Registered-fixed scenario and its reference, in separate processes (modes unfixref, unfix; the
//! reference trajectory goes through a file so that it never follows the scenario's clear()).
//! Scenario: G is created (registered mobile), taken out of the world, fixed, added again (so it
//! is REGISTERED WHILE FIXED), then unfixed before the first step. Reference: G registered mobile.
//! Both: a force on G from rank 0 only, synchronizeForces() (both replicas must see it), then the
//! impact of a local sphere and 30 steps.
std::vector<double> unfixRun( bool registeredFixed, double* force )
{
   const Vec3 a( -2 * R - U * DT * 3.9, 0.05, 0 );   // touches G after about 4 steps
   fresh(); MaterialID m( material() );
   SphereID g( nullptr );
   pe_GLOBAL_SECTION { g = createSphere( 100, Vec3( 0, 0.05, 0 ), R, m ); }
   if( registeredFixed ) {
      theWorld()->remove( g );   // deregistered (mobile)
      g->setFixed( true );
      theWorld()->add( g );      // registered while fixed: add() leaves it out of the set
      g->setFixed( false );
   }
   if( theWorld()->ownsPoint( a ) ) { SphereID s = createSphere( 1, a, R, m ); s->setLinearVel( Vec3( U, 0, 0 ) ); }
   theWorld()->synchronize();
   if( rank_ == 0 ) g->addForce( Vec3( 0, 0, 1e-3 ) );
   theCollisionSystem()->synchronizeForces();
   double f[2] = { 0, 0 }, mine( g->getForce()[2] );
   MPI_Allgather( &mine, 1, MPI_DOUBLE, f, 1, MPI_DOUBLE, MPI_COMM_WORLD );
   force[0] = f[0]; force[1] = f[1];
   return steps( 30 );
}

int runUnfixRef()
{
   double force[2];
   const std::vector<double> ref( unfixRun( false, force ) );
   if( rank_ == 0 ) {
      FILE* fp( std::fopen( "unfix_reference.bin", "wb" ) );
      if( !fp ) { std::printf( "cannot write unfix_reference.bin\n" ); return 1; }
      std::fwrite( force, sizeof( double ), 2, fp );
      std::fwrite( ref.data(), sizeof( double ), ref.size(), fp );
      std::fclose( fp );
      std::printf( "unfix reference (G registered mobile): force on G after synchronizeForces() rank 0 %.3e, rank 1 %.3e; %zu values written\n",
                   force[0], force[1], ref.size() );
   }
   return ( force[0] == 1e-3 && force[1] == 1e-3 ) ? 0 : 1;
}

int runUnfix()
{
   double force[2];
   const std::vector<double> sc( unfixRun( true, force ) );
   std::vector<double> ref( sc.size() + 2, 0.0 );
   int ok( 1 );
   if( rank_ == 0 ) {
      FILE* fp( std::fopen( "unfix_reference.bin", "rb" ) );
      ok = fp && std::fread( ref.data(), sizeof( double ), ref.size(), fp ) == ref.size();
      if( fp ) std::fclose( fp );
   }
   MPI_Bcast( &ok, 1, MPI_INT, 0, MPI_COMM_WORLD );
   if( !ok ) { if( rank_ == 0 ) std::printf( "no reference file (run mode unfixref first)\n" ); return 1; }
   int bad( 0 );
   if( rank_ == 0 ) {
      std::printf( "unfix (G registered fixed, then unfixed): force on G after synchronizeForces() rank 0 %.3e, rank 1 %.3e (reference %.3e, %.3e)\n",
                   force[0], force[1], ref[0], ref[1] );
      if( !( force[0] == ref[0] && force[1] == ref[1] ) ) { std::printf( "FAIL: synchronizeForces() reduces the force of the unfixed global body\n" ); bad = 1; }
   }
   const std::vector<double> r( ref.begin() + 2, ref.end() );
   bad |= compare( r, sc, "registered fixed then unfixed against registered mobile (sphere and both replicas of G)" );
   MPI_Bcast( &bad, 1, MPI_INT, 0, MPI_COMM_WORLD );
   return bad;
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank_ );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 || argc < 2 ) { if( rank_ == 0 ) std::cerr << "usage (two ranks): pe_global_registry_mpi fixdestroy | clear | fixstepunfix | fixstepunfix-iter | unfixref | unfix\n"; MPI_Abort( MPI_COMM_WORLD, 2 ); }
   if( rank_ == 0 ) { defineLocalDomain( HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); connect( 1, HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); }
   else             { defineLocalDomain( HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); connect( 0, HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); }
   const std::string mode( argv[1] );
   int rc = 2;
   if( mode == "fixdestroy" ) rc = runFixDestroy();
   else if( mode == "clear" ) rc = runClear();
   else if( mode == "unfix" ) rc = runUnfix();
   else if( mode == "unfixref" ) rc = runUnfixRef();
   else if( mode == "fixstepunfix" ) rc = runFixStepUnfix( false );
   else if( mode == "fixstepunfix-iter" ) rc = runFixStepUnfix( true );
   int any = 0;
   MPI_Allreduce( &rc, &any, 1, MPI_INT, MPI_MAX, MPI_COMM_WORLD );
   if( rank_ == 0 ) std::printf( "pe_global_registry_mpi %s: %s\n", mode.c_str(), any == 0 ? "all checks passed" : "FAILED" );
   MPI_Finalize();
   return any == 0 ? 0 : 1;
}
