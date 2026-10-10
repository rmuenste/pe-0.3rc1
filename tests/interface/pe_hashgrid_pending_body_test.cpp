//=================================================================================================
/*!
 *  \file tests/interface/pe_hashgrid_pending_body_test.cpp
 *  \brief HashGrids::getBodiesNearPoint must return bodies added since the last findContacts().
 *
 *  A body added to the coarse detector waits in HashGrids::bodiesToAdd_ until the next
 *  findContacts() inserts it into a grid (or into nonGridBodies_ while the grid is inactive).
 *  getBodiesNearPoint() used to search only the grids and nonGridBodies_, so a body created
 *  between simulation steps was invisible to point queries for one step. FeatFloWer's
 *  accelerated FBM classification (pointInsideParticlesAccelerated) then classified the interior
 *  of a freshly inserted particle as fluid, and in parallel mode a shadow copy received in
 *  synchronize() after findContacts() had the same problem.
 *
 *  Asserted, once with the grid inactive (few bodies) and once with the grid active (more than
 *  gridActivationThreshold bodies, after one simulation step):
 *    1. a sphere created after the step is returned for a query at its centre before the next
 *       step;
 *    2. after the next step it is still returned, exactly once (it moved from bodiesToAdd_ into
 *       the data structure);
 *    3. a sphere that existed before the step is still returned for a query at its centre.
 *
 *  Serial world setup, no MPI.
 */
//=================================================================================================

#include <pe/core.h>

#include <algorithm>
#include <cstdio>
#include <vector>

using namespace pe;

static int failures = 0;

static void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

static size_t countNear( const Vec3& p, ConstBodyID body )
{
   std::vector<BodyID> candidates;
   theCollisionSystem()->getCoarseDetector().getBodiesNearPoint( p[0], p[1], p[2], candidates );
   return static_cast<size_t>( std::count( candidates.begin(), candidates.end(), body ) );
}

//! \a lattice spheres on a cubic lattice, one step, then a new sphere away from all of them.
static void runCase( const char* name, int lattice )
{
   theWorld()->clear();
   theWorld()->setGravity( 0, 0, 0 );

   pe::id_t uid( 1 );
   SphereID first( nullptr );
   for( int i = 0; i < lattice; ++i )
      for( int j = 0; j < lattice; ++j )
         for( int k = 0; k < lattice; ++k ) {
            SphereID s = createSphere( uid++, Vec3( 2*i, 2*j, 2*k ), real(0.4), granite );
            if( !first ) first = s;
         }

   theWorld()->simulationStep( real(1e-3) );
   const bool gridActive = theCollisionSystem()->getCoarseDetector().isGridActive();
   std::printf( "%s: %zu bodies, grid %s\n", name, theWorld()->size(),
                gridActive ? "active" : "inactive" );

   const Vec3 pos( -10, -10, -10 );
   SphereID added = createSphere( uid++, pos, real(0.4), granite );

   char what[160];
   std::snprintf( what, sizeof(what), "%s: body created after the step is returned before the next step", name );
   expect( countNear( pos, added ) == 1, what );
   std::snprintf( what, sizeof(what), "%s: existing body still returned", name );
   expect( countNear( first->getPosition(), first ) == 1, what );

   theWorld()->simulationStep( real(1e-3) );
   std::snprintf( what, sizeof(what), "%s: body returned exactly once after the next step", name );
   expect( countNear( added->getPosition(), added ) == 1, what );
}

int main()
{
   // 2^3 = 8 bodies keep the grid inactive; 4^3 = 64 exceed gridActivationThreshold (32).
   runCase( "grid inactive", 2 );
   runCase( "grid active", 4 );

   if( !theCollisionSystem()->getCoarseDetector().isGridActive() ) {
      std::printf( "FAIL: the grid-active case did not activate the grid\n" );
      ++failures;
   }

   std::printf( failures ? "FAILED (%d)\n" : "PASSED\n", failures );
   return failures ? 1 : 0;
}
