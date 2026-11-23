//=================================================================================================
/*!
 *  \file tests/interface/pe_restitution_trace_comm_mpi.cpp
 *  \brief Review round 4, finding 3: the trace settings of the restitution post-pass belong to one
 *         PE communicator lifetime. Two ranks: rank 0 first runs restitution steps on its own
 *         single-process communicator (MPI_COMM_SELF, the trace settings are read there), then
 *         both ranks install MPI_COMM_WORLD and run a head-on impact across the cut. Rank 1 reads
 *         the settings for the first time; rank 0 must read them again on the new communicator,
 *         or the two ranks enter different collectives (hang or MPI error; ctest TIMEOUT 60).
 *
 *  Checked: the common phase completes and the pair rebounds at e. Exit 77 without the switch.
 */
//=================================================================================================

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <type_traits>

#include <mpi.h>

using namespace pe;

namespace {

template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitutionPostPass( true ) ) > > : std::true_type {};

template< typename CS > int run( CS& cs, int rank )
{
   const double r = 0.01, u = 0.5, dt = 1e-4, e = 0.9, gap = 2 * u * dt * 5.9;
   WorldID world = theWorld();
   world->setGravity( 0, 0, 0 );
   world->setDamping( 1 );
   cs.setRestitution( true );
   cs.setRestitutionPostPass( true );
   cs.setRestitutionThreshold( 0 );
   const MaterialID mat = createMaterial( 1000.0, e, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 );

   // 1. subgroup: rank 0 alone, a pair impact on its own communicator
   theMPISystem()->setComm( MPI_COMM_SELF );
   if( rank == 0 ) {
      SphereID a = createSphere( 1, Vec3( -r - 0.5 * gap, 0, 0 ), r, mat ); a->setLinearVel( Vec3(  u, 0, 0 ) );
      SphereID b = createSphere( 2, Vec3(  r + 0.5 * gap, 0, 0 ), r, mat ); b->setLinearVel( Vec3( -u, 0, 0 ) );
      for( int i = 0; i < 20; ++i ) world->simulationStep( dt );
      std::printf( "subgroup phase (rank 0 alone): v_rel'/v_rel %.6f\n", ( b->getLinearVel()[0] - a->getLinearVel()[0] ) / ( 2 * u ) );
      world->clear();
   }

   // 2. common communicator, two subdomains, a pair across the cut
   theMPISystem()->setComm( MPI_COMM_WORLD );
   if( rank == 0 ) { defineLocalDomain( HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); connect( 1, HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); }
   else            { defineLocalDomain( HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); connect( 0, HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); }
   const Vec3 pa( -r - 0.5 * gap, 0.05, 0 ), pb( r + 0.5 * gap, 0.05, 0 );
   if( world->ownsPoint( pa ) ) { SphereID s = createSphere( 3, pa, r, mat ); s->setLinearVel( Vec3(  u, 0, 0 ) ); }
   if( world->ownsPoint( pb ) ) { SphereID s = createSphere( 4, pb, r, mat ); s->setLinearVel( Vec3( -u, 0, 0 ) ); }
   world->synchronize();
   for( int i = 0; i < 20; ++i ) world->simulationStep( dt );
   double v[2] = { 0, 0 };
   for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
      BodyID b = *it;
      if( b->isRemote() ) continue;
      if( b->getID() == 3 ) v[0] = b->getLinearVel()[0];
      if( b->getID() == 4 ) v[1] = b->getLinearVel()[0];
   }
   MPI_Allreduce( MPI_IN_PLACE, v, 2, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   const double ratio = ( v[1] - v[0] ) / ( 2 * u );
   int bad = 0;
   if( rank == 0 ) {
      std::printf( "common phase (both ranks, MPI_COMM_WORLD): v_rel'/v_rel %.6f\n", ratio );
      if( !( std::fabs( ratio - e ) < 1e-3 ) ) { std::printf( "FAIL: the pair across the cut rebounds at e\n" ); bad = 1; }
      std::printf( "pe_restitution_trace_comm_mpi: %s\n", bad == 0 ? "all checks passed" : "FAILED" );
   }
   MPI_Bcast( &bad, 1, MPI_INT, 0, MPI_COMM_WORLD );
   return bad;
}

template< typename CS > int dispatch( CS& cs, int rank )
{
   if constexpr ( HasRestitution<CS>::value ) return run( cs, rank );
   else { (void)cs; (void)rank; return 77; }
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );
   int rank = 0, size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 ) { if( rank == 0 ) std::cerr << "requires two ranks\n"; MPI_Abort( MPI_COMM_WORLD, 2 ); }
   const int rc = dispatch( *theCollisionSystem(), rank );
   MPI_Finalize();
   return rc;
}
