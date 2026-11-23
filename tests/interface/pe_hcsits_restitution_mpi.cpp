//=================================================================================================
/*!
 *  \file tests/interface/pe_hcsits_restitution_mpi.cpp
 *  \brief Restitution of HardContactSemiImplicitTimesteppingSolvers on two ranks (cut at x = 0).
 *
 *  A pair straddling the cut and a three-sphere chain whose A-B row is treated on rank 0 and B-C
 *  row on rank 1, e = 0.9, no forces, the regression stop rule (up to 100 sweeps, tested every
 *  sweep, relative tolerance 1e-9: the convergence tail runs many sweeps whose corrections are far
 *  below 1e-8). Checked: momentum of the pair and of the chain conserved to 1e-14 of the impact
 *  momentum over 40 steps (the shadow-correction send filter is exact with restitution on; the
 *  approximate Vec3 comparison dropped corrections below 1e-8), the pair rebounds at e, and a
 *  finite-mass global body is refused on both ranks.
 */
//=================================================================================================

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <type_traits>

#include <mpi.h>

using namespace pe;

namespace {

const int N = 5;

template< typename T, typename = void > struct HasStopRule : std::false_type {};
template< typename T > struct HasStopRule< T, std::void_t< decltype( std::declval<T&>().setRestitutionCheckInterval( size_t(1) ) ) > > : std::true_type {};
template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitutionPostPass( true ) ) > > : std::true_type {};

void gather( double* x, double* v, double* m, int rank )
{
   (void)rank;
   for( int k = 0; k < N; ++k ) { x[k] = 0; v[k] = 0; m[k] = 0; }
   WorldID world = theWorld();
   for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
      BodyID b = *it;
      if( b->getType() != sphereType || b->isRemote() || b->isGlobal() ) continue;
      const int id = static_cast<int>( b->getID() );
      if( id < 0 || id >= N ) continue;
      x[id] = b->getPosition()[0]; v[id] = b->getLinearVel()[0]; m[id] = b->getMass();
   }
   MPI_Allreduce( MPI_IN_PLACE, x, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, v, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( MPI_IN_PLACE, m, N, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
}

template< typename CS > int run( CS& cs, int rank )
{
   const double r = 0.01, u = 0.5, dt = 1e-4, e = 0.9;
   WorldID world = theWorld();
   world->setGravity( 0, 0, 0 );
   world->setDamping( 1 );
   if( rank == 0 ) { defineLocalDomain( HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); connect( 1, HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); }
   else            { defineLocalDomain( HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); connect( 0, HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); }
   cs.setRestitution( true );
   cs.setRestitutionPostPass( true );
   cs.setRestitutionThreshold( 0 );
   cs.setRestitutionIterations( 4 );
   if constexpr ( HasStopRule<CS>::value ) {   // the regression setting (the build before the configurable rule ran so)
      cs.setRestitutionMaxIterations( 100 );
      cs.setRestitutionCheckInterval( 1 );
      cs.setRestitutionTolerance( 0.0, 1e-9 );
   }

   const MaterialID mat = createMaterial( 1000.0, e, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 );
   const double gap2 = 2.0 * u * dt * 5.9, gap1 = u * dt * 5.9;
   const double xa = -0.015, xb3 = -0.002;
   const Vec3 pos[N] = { Vec3( xa, 0, 0 ), Vec3( xa + 2 * r + gap2, 0, 0 ),
                         Vec3( xb3 - 2 * r - gap1, 0.03, 0 ), Vec3( xb3, 0.03, 0 ), Vec3( xb3 + 2 * r, 0.03, 0 ) };
   const double vel[N] = { u, -u, u, 0, 0 };
   for( int k = 0; k < N; ++k )
      if( world->ownsPoint( pos[k] ) ) { SphereID s = createSphere( k, pos[k], r, mat ); s->setLinearVel( Vec3( vel[k], 0, 0 ) ); }
   world->synchronize();

   double x[N], v[N], m[N];
   gather( x, v, m, rank );
   const double p0[2] = { m[0] * v[0] + m[1] * v[1], m[2] * v[2] + m[3] * v[3] + m[4] * v[4] };
   double leak = 0;
   for( int i = 0; i < 40; ++i ) {
      world->simulationStep( dt );
      gather( x, v, m, rank );
      leak = std::max( leak, std::max( std::fabs( m[0] * v[0] + m[1] * v[1] - p0[0] ),
                                       std::fabs( m[2] * v[2] + m[3] * v[3] + m[4] * v[4] - p0[1] ) ) );
   }
   int bad = 0;
   if( rank == 0 ) {
      const double scale = m[0] * u;
      std::printf( "HCSITS two ranks: pair v = %.12f %.12f (v_rel'/v_rel %.9f), chain v = %.12f %.12f %.12f, largest momentum change %.3e (%.3e of m u)\n",
                   v[0], v[1], ( v[1] - v[0] ) / ( -2 * u ), v[2], v[3], v[4], leak, leak / scale );
      if( !( leak <= 1e-14 * scale ) ) { std::printf( "FAIL: momentum conserved through the convergence tail\n" ); ++bad; }
      if( !( std::fabs( ( v[1] - v[0] ) / ( -2 * u ) + e ) <= 1e-6 ) ) { std::printf( "FAIL: the pair rebounds at e\n" ); ++bad; }
   }
   // a finite-mass global body: refused on both ranks
   pe_GLOBAL_SECTION { createSphere( 100, Vec3( 0.0, -0.04, -0.04 ), r, mat ); }
   int refused = 0;
   try { world->simulationStep( dt ); }
   catch( const std::runtime_error& ex ) { refused = std::string( ex.what() ).find( "global" ) != std::string::npos ? 1 : 0; }
   int refusedAll = 0;
   MPI_Allreduce( &refused, &refusedAll, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 && refusedAll != 2 ) { std::printf( "FAIL: a finite-mass global body is refused on both ranks\n" ); ++bad; }
   MPI_Allreduce( MPI_IN_PLACE, &bad, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) std::printf( "pe_hcsits_restitution_mpi: %s\n", bad == 0 ? "all checks passed" : "FAILED" );
   return bad == 0 ? 0 : 1;
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
