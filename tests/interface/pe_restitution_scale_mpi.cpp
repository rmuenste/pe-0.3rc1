//=================================================================================================
/*!
 *  \file tests/interface/pe_restitution_scale_mpi.cpp
 *  \brief Restitution post-pass on two ranks (cut at x = 0) under the PRODUCTION stop rule
 *         (min 4, max 24, check every 4, 1e-4 + 1e-3 of the local impact scale): a three-sphere
 *         chain A -> (B, C touching) treated on one rank against the same chain split across the
 *         cut (A-B row on rank 0, B-C row on rank 1). Review round 3, finding 5: the impact scale
 *         of B must be the same on every rank that holds B, so the B-C support row gets the same
 *         tolerance in both layouts.
 *
 *  Checked: the largest support-row scale (RestitutionPassStats::supportScaleMax, where the
 *  build has it) equals the impact speed e u in both layouts; both layouts converge (stop reason
 *  2) within the production budget; the final velocities agree within the tolerance. The sweep
 *  counts are printed, not compared: the split rows couple only at the synchronisations. Built against both
 *  solvers (EL: pe-restitution-scale-mpi, HCSITS: -plain). Exit 77 without the restitution switch.
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

template< typename T, typename = void > struct HasRestitution : std::false_type {};
template< typename T > struct HasRestitution< T, std::void_t< decltype( std::declval<T&>().setRestitutionPostPass( true ) ) > > : std::true_type {};
template< typename T, typename = void > struct HasStopRule : std::false_type {};
template< typename T > struct HasStopRule< T, std::void_t< decltype( std::declval<T&>().setRestitutionCheckInterval( size_t(1) ) ) > > : std::true_type {};
template< typename S, typename = void > struct HasScaleStat : std::false_type {};
template< typename S > struct HasScaleStat< S, std::void_t< decltype( std::declval<const S&>().supportScaleMax ) > > : std::true_type {};

template< typename S > double supportScale( const S& st )
{
   if constexpr ( HasScaleStat<S>::value ) return st.supportScaleMax;
   else { (void)st; return -1.0; }
}

struct Result { int stop = -1; size_t sweeps = 0; double scale = -1; double v[3] = { 0, 0, 0 }; };

void gather( double* v )
{
   for( int k = 0; k < 3; ++k ) v[k] = 0;
   for( World::Iterator it = theWorld()->begin(); it != theWorld()->end(); ++it ) {
      BodyID b = *it;
      if( b->getType() != sphereType || b->isRemote() || b->isGlobal() ) continue;
      const int id = static_cast<int>( b->getID() );
      if( id >= 0 && id < 3 ) v[id] = b->getLinearVel()[0];
   }
   MPI_Allreduce( MPI_IN_PLACE, v, 3, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
}

template< typename CS > Result chain( CS& cs, double xb )
{
   const double r = 0.01, u = 0.5, dt = 1e-4, e = 0.9, gap = u * dt * 5.9;
   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0, 0, 0 );
   const MaterialID mat = createMaterial( 1000.0, e, 0.0, 0.0, 0.25, 300.0, 1e5, 10.0, 10.0 );
   const Vec3 pos[3] = { Vec3( xb - 2 * r - gap, 0, 0 ), Vec3( xb, 0, 0 ), Vec3( xb + 2 * r, 0, 0 ) };
   for( int k = 0; k < 3; ++k )
      if( world->ownsPoint( pos[k] ) ) { SphereID s = createSphere( k, pos[k], r, mat ); s->setLinearVel( Vec3( k == 0 ? u : 0.0, 0, 0 ) ); }
   world->synchronize();
   Result res;
   double v[3];
   for( int i = 0; i < 40; ++i ) {
      world->simulationStep( dt );
      const RestitutionPassStats& st( cs.getRestitutionPassStats() );
      if( res.stop < 0 && st.stop >= 2 ) {
         res.stop = st.stop; res.sweeps = st.sweeps;
         res.scale = supportScale( st );
      }
   }
   gather( v );
   for( int k = 0; k < 3; ++k ) res.v[k] = v[k];
   return res;
}

template< typename CS > int run( CS& cs, int rank )
{
   theWorld()->setDamping( 1 );
   if( rank == 0 ) { defineLocalDomain( HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); connect( 1, HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); }
   else            { defineLocalDomain( HalfSpace( Vec3( +1, 0, 0 ), 0 ) ); connect( 0, HalfSpace( Vec3( -1, 0, 0 ), 0 ) ); }
   cs.setRestitution( true );
   cs.setRestitutionPostPass( true );
   cs.setRestitutionThreshold( 0 );
   cs.setRestitutionIterations( 4 );
   if constexpr ( HasStopRule<CS>::value ) {   // the production rule (also the round-2 default)
      cs.setRestitutionMaxIterations( 24 );
      cs.setRestitutionCheckInterval( 4 );
      cs.setRestitutionTolerance( 1e-4, 1e-3 );
   }
   const double e = 0.9, u = 0.5;
   const Result one( chain( cs, -0.06 ) );     // B at -0.06, C at -0.04: all rows on rank 0
   const Result split( chain( cs, -0.002 ) );  // A-B midpoint -0.012 (rank 0), B-C midpoint 0.008 (rank 1)
   int bad = 0;
   if( rank == 0 ) {
      const double tol = 1e-4 + 1e-3 * e * u;
      std::printf( "one treater: stop %d after %zu sweeps, support scale %.6f, v = %.9f %.9f %.9f\n",
                   one.stop, one.sweeps, one.scale, one.v[0], one.v[1], one.v[2] );
      std::printf( "split chain: stop %d after %zu sweeps, support scale %.6f, v = %.9f %.9f %.9f\n",
                   split.stop, split.sweeps, split.scale, split.v[0], split.v[1], split.v[2] );
      if( HasScaleStat<RestitutionPassStats>::value && !( one.scale == e * u && split.scale == e * u ) ) {
         std::printf( "FAIL: the B-C support row has the impact scale e u in both layouts\n" ); ++bad;
      }
      // The stop REASON must agree (both converge within the budget). The sweep count need not: the
      // two rows of the split chain see each other's updates only at the synchronisations (block
      // Jacobi across the cut) instead of within the sweep (Gauss-Seidel), whatever the tolerance.
      if( !( one.stop == 2 && split.stop == 2 ) ) {
         std::printf( "FAIL: both layouts converge within the production budget\n" ); ++bad;
      }
      for( int k = 0; k < 3; ++k )
         if( !( std::fabs( one.v[k] - split.v[k] ) <= tol ) ) { std::printf( "FAIL: velocity %d agrees within the tolerance\n", k ); ++bad; }
   }
   MPI_Allreduce( MPI_IN_PLACE, &bad, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) std::printf( "pe_restitution_scale_mpi: %s\n", bad == 0 ? "all checks passed" : "FAILED" );
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
