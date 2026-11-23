//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_classification_test.cpp
 *  \brief pe::coupling point classification: exhaustive, spatial index, ownership rule
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
 *  Stages (owner IDENTITIES are compared everywhere, not just inside/outside):
 *    A  analytic: a few separated spheres; the exhaustive owner of random points equals the
 *       sphere that analytically contains the point (points within 1e-9 of a surface skipped);
 *       the spatial-index method agrees (grid inactive: the fallback); batch == single queries.
 *    B  grid ACTIVE (asserted): 70+ bodies of several shapes including overlapping fixed
 *       spheres, a fixed sphere, a fixed and a movable cylinder and a plane; after
 *       refreshSpatialIndex() the spatial-index owner and owner list equal the exhaustive ones
 *       for every point; the legacy extern "C" queries pointInsideParticles() /
 *       pointOwners() / their accelerated forms return the same owners, and both legacy solid
 *       flags are 1 on hits and 0 on misses.
 *    O  ownership rule: in the overlap of two bodies the owner is the minimum id and the
 *       owner list holds both ids ascending; contains() agrees with the owner list; fixed
 *       cylinders are never owners, fixed spheres are, planes never.
 *    K  containsPoints() for every body over a point batch equals contains() point by point;
 *    E  non-finite query points throw (all methods), a coordinate beyond the hash grid's range
 *       throws for the spatial index, a refused batch writes nothing;
 *    C  after motion (several steps with random velocities): with refreshSpatialIndex() the
 *       methods agree again (verdict); without it the disagreement count is only reported
 *       (control, documents the stale filing).
 *  Serial world, no MPI; linked against the configured pe_static.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>
#include <pe/interface/object_queries.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
namespace pc = pe::coupling;


//*************************************************************************************************
/*!\brief Random points in a box, plus points close to (inside and outside) the bounding
 *        spheres of the given bodies, so surfaces are probed densely.
 */
static std::vector<pc::Vec3d> makePoints( const pc::Vec3d& lo, const pc::Vec3d& hi, int n,
                                          unsigned int seed )
{
   std::mt19937 rng( seed );
   std::uniform_real_distribution<double> u( 0.0, 1.0 );
   std::vector<pc::Vec3d> pts;
   for( int i=0; i<n; ++i )
      pts.push_back( pc::Vec3d{ { lo[0] + u( rng ) * ( hi[0] - lo[0] ),
                                  lo[1] + u( rng ) * ( hi[1] - lo[1] ),
                                  lo[2] + u( rng ) * ( hi[2] - lo[2] ) } } );
   return pts;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds points on shells around a centre, just inside and just outside radius r.
 */
static void addShellPoints( std::vector<pc::Vec3d>& pts, const pc::Vec3d& c, double r,
                            unsigned int seed )
{
   std::mt19937 rng( seed );
   std::normal_distribution<double> nd( 0.0, 1.0 );
   for( int i=0; i<60; ++i ) {
      double d[3] = { nd( rng ), nd( rng ), nd( rng ) };
      const double len( std::sqrt( d[0]*d[0] + d[1]*d[1] + d[2]*d[2] ) );
      for( const double f : { 0.999, 1.001 } )
         pts.push_back( pc::Vec3d{ { c[0] + f*r*d[0]/len, c[1] + f*r*d[1]/len,
                                     c[2] + f*r*d[2]/len } } );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compares both methods on all points; returns the number of disagreements.
 */
static int compareMethods( const pc::Engine& engine, const std::vector<pc::Vec3d>& pts,
                           int& inside )
{
   int bad( 0 );
   inside = 0;
   std::vector<pc::BodyId> listE, listS;
   for( const pc::Vec3d& p : pts ) {
      const pc::BodyId e( engine.owner( p, pc::QueryMethod::exhaustive ) );
      const pc::BodyId s( engine.owner( p, pc::QueryMethod::spatialIndex ) );
      engine.owners( p, listE, pc::QueryMethod::exhaustive );
      engine.owners( p, listS, pc::QueryMethod::spatialIndex );
      if( e != pc::noBody ) ++inside;
      if( e != s || listE != listS ) ++bad;
   }
   return bad;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stage A: exhaustive classification against analytic sphere containment.
 */
static void stageAnalytic( pc::Engine& engine, pc::MaterialId mat )
{
   struct Ball { pc::Vec3d c; double r; pc::BodyId id; };
   std::vector<Ball> balls = { { { { 0.0, 0.0, 0.0 } }, 1.0, 0 },
                               { { { 3.0, 0.5, 0.0 } }, 0.75, 0 },
                               { { { -2.5, 2.0, 1.0 } }, 1.2, 0 },
                               { { { 0.5, -3.0, -1.0 } }, 0.4, 0 },
                               { { { 2.0, 2.5, 2.0 } }, 0.9, 0 } };
   for( Ball& b : balls ) b.id = engine.createSphere( b.c, b.r, mat );

   std::vector<pc::Vec3d> pts( makePoints( pc::Vec3d{ { -4.5, -4.0, -3.0 } },
                                           pc::Vec3d{ { 4.5, 4.5, 3.5 } }, 20000, 11u ) );
   for( std::size_t i=0; i<balls.size(); ++i )
      addShellPoints( pts, balls[i].c, balls[i].r, 100u+unsigned( i ) );

   int checked( 0 ), inside( 0 ), wrong( 0 ), wrongSpatial( 0 );
   std::vector<double> xyz;
   std::vector<pc::BodyId> expectedOwners;
   for( const pc::Vec3d& p : pts ) {
      pc::BodyId analytic( pc::noBody );
      bool nearSurface( false );
      for( const Ball& b : balls ) {
         const double d( std::sqrt( ( p[0]-b.c[0] )*( p[0]-b.c[0] ) +
                                    ( p[1]-b.c[1] )*( p[1]-b.c[1] ) +
                                    ( p[2]-b.c[2] )*( p[2]-b.c[2] ) ) );
         if( std::fabs( d - b.r ) < 1e-9 ) nearSurface = true;
         if( d < b.r ) analytic = b.id;
      }
      if( nearSurface ) continue;
      ++checked;
      if( analytic != pc::noBody ) ++inside;
      if( engine.owner( p, pc::QueryMethod::exhaustive ) != analytic ) ++wrong;
      if( engine.owner( p, pc::QueryMethod::spatialIndex ) != analytic ) ++wrongSpatial;
      xyz.insert( xyz.end(), p.begin(), p.end() );
      expectedOwners.push_back( analytic );
   }
   std::printf( "A analytic: %d points, %d inside, %d exhaustive and %d spatial-index mismatches "
                "(grid %s)\n", checked, inside, wrong, wrongSpatial,
                engine.spatialIndexActive() ? "active" : "inactive" );
   expect( inside > 500, "A: enough points inside spheres" );
   expect( wrong == 0, "A: exhaustive owner equals the analytic owner" );
   expect( wrongSpatial == 0, "A: spatial-index owner equals the analytic owner" );

   for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive, pc::QueryMethod::spatialIndex } ) {
      std::vector<pc::BodyId> batch( expectedOwners.size(), 12345u );
      engine.classify( xyz.data(), expectedOwners.size(), batch.data(), m );
      expect( batch == expectedOwners, "A: batch classify equals the analytic owners" );
   }
   engine.classify( nullptr, 0, nullptr, pc::QueryMethod::exhaustive );   // empty batch is fine
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Builds the stage B world: more than 32 bodies of mixed shapes.
 */
static void buildCrowd( pc::Engine& engine, pc::MaterialId mat, std::vector<pc::Vec3d>& probes )
{
   using namespace pe;
   std::mt19937 rng( 5u );
   std::uniform_real_distribution<double> radius( 0.3, 0.9 );

   for( int i=0; i<4; ++i )
      for( int j=0; j<4; ++j )
         for( int k=0; k<4; ++k ) {
            const pc::Vec3d c{ { 2.0*i, 2.0*j, 2.0*k } };
            const double r( radius( rng ) );
            engine.createSphere( c, r, mat );
            addShellPoints( probes, c, r, 1000u + 16*i + 4*j + k );
         }

   // Other shapes between the lattice spheres
   createBox( 1, Vec3( 1, 1, 1 ), Vec3( 0.6, 0.8, 0.5 ), granite )->rotate( 0.2, 0.4, 0.1 );
   createCapsule( 2, Vec3( 3, 1, 1 ), 0.3, 0.8, granite );
   createEllipsoid( 3, Vec3( 1, 3, 1 ), 0.6, 0.4, 0.3, granite );
   createCylinder( 4, Vec3( 3, 3, 1 ), 0.3, 0.8, granite );                    // movable cylinder
   createCylinder( 5, Vec3( 5, 5, 5 ), 0.4, 1.0, granite )->setFixed( true );  // never an owner
   createSphere( 6, Vec3( 1, 1, 5 ), 0.5, granite )->setFixed( true );         // fixed: an owner
   // Two overlapping fixed spheres (fixed bodies do not collide, so they stay overlapping)
   createSphere( 7, Vec3( 5, 1, 3 ), 0.6, granite )->setFixed( true );
   createSphere( 8, Vec3( 5.5, 1, 3 ), 0.6, granite )->setFixed( true );
   createPlane( 9, 0, 0, 1, -1.5, granite );                                   // never an owner

   const std::vector<pc::Vec3d> centres = {
      pc::Vec3d{ { 1, 1, 1 } }, pc::Vec3d{ { 3, 1, 1 } }, pc::Vec3d{ { 1, 3, 1 } },
      pc::Vec3d{ { 3, 3, 1 } }, pc::Vec3d{ { 5, 5, 5 } }, pc::Vec3d{ { 1, 1, 5 } },
      pc::Vec3d{ { 5.25, 1, 3 } } };
   for( const pc::Vec3d& c : centres )
      addShellPoints( probes, c, 0.5, 7u );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stage B: spatial index against exhaustive with an active grid; legacy queries agree.
 */
static void stageGrid( pc::Engine& engine )
{
   expect( engine.spatialIndexActive(), "B: the hash grid is active" );
   engine.refreshSpatialIndex();

   std::vector<pc::Vec3d> pts( makePoints( pc::Vec3d{ { -1.5, -1.5, -2.0 } },
                                           pc::Vec3d{ { 7.5, 7.5, 7.5 } }, 40000, 21u ) );
   int inside( 0 );
   const int bad( compareMethods( engine, pts, inside ) );
   std::printf( "B grid active=%d: %zu points, %d inside, %d disagreements\n",
                int( engine.spatialIndexActive() ), pts.size(), inside, bad );
   expect( inside > 2000, "B: enough points inside bodies" );
   expect( bad == 0, "B: spatial index agrees with exhaustive (owner and owner list)" );

   // The legacy extern "C" queries share the logic and must give the same owners
   int legacyBad( 0 ), legacyFlagBad( 0 ), legacyHits( 0 ), legacyMisses( 0 );
   std::vector<pc::BodyId> list;
   for( std::size_t i=0; i<pts.size(); i+=7 ) {
      double pos[3] = { pts[i][0], pts[i][1], pts[i][2] };
      const pc::BodyId e( engine.owner( pts[i], pc::QueryMethod::exhaustive ) );
      engine.owners( pts[i], list, pc::QueryMethod::exhaustive );

      int inpr( -1 );
      short int bytes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
      const bool found( pointInsideParticles( 0, &inpr, pos, bytes ) );
      const pc::BodyId legacy( found ? ByteArrayToUint64( bytes ) : pc::noBody );
      int inprA( -1 );
      short int bytesA[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
      const bool foundA( pointInsideParticlesAccelerated( 0, &inprA, pos, bytesA ) );
      const pc::BodyId legacyA( foundA ? ByteArrayToUint64( bytesA ) : pc::noBody );

      unsigned long long ids[8];
      const int maxOwners( 8 );
      int n( -1 ), nA( -1 );
      pointOwners( pos, &maxOwners, ids, &n );
      std::vector<pc::BodyId> legacyList( ids, ids + std::min( n, maxOwners ) );
      unsigned long long idsA[8];
      pointOwnersAccelerated( pos, &maxOwners, idsA, &nA );
      std::vector<pc::BodyId> legacyListA( idsA, idsA + std::min( nA, maxOwners ) );

      if( legacy != e || legacyA != e || legacyList != list || legacyListA != list )
         ++legacyBad;
      // The solid flags: 1 on a hit, 0 on a miss, for both legacy queries
      if( inpr != ( found ? 1 : 0 ) || inprA != ( foundA ? 1 : 0 ) ||
          found != ( e != pc::noBody ) || foundA != ( e != pc::noBody ) )
         ++legacyFlagBad;
      if( e != pc::noBody ) ++legacyHits; else ++legacyMisses;
   }
   expect( legacyBad == 0, "B: legacy extern \"C\" queries give the same owners" );
   expect( legacyFlagBad == 0, "B: legacy exhaustive and accelerated flags (hits and misses)" );
   expect( legacyHits > 100 && legacyMisses > 100, "B: the legacy check saw hits and misses" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stage O: the ownership rule and the fixed-body rule.
 */
static void stageOwnership( pc::Engine& engine )
{
   // The two overlapping fixed spheres (user ids 7 and 8); 7 was created first
   pc::BodyId a( pc::noBody ), b( pc::noBody ), fixedCyl( pc::noBody ), movCyl( pc::noBody );
   pc::BodyId fixedSph( pc::noBody ), plane( pc::noBody );
   std::vector<pc::BodyState> all;
   engine.states( pc::Selection::all, all );
   for( const pc::BodyState& s : all ) {
      if( s.userId == 7 ) a = s.id;
      if( s.userId == 8 ) b = s.id;
      if( s.userId == 5 ) fixedCyl = s.id;
      if( s.userId == 4 ) movCyl = s.id;
      if( s.userId == 6 ) fixedSph = s.id;
      if( s.userId == 9 ) plane = s.id;
   }
   expect( a < b, "O: the first overlapping sphere has the smaller id" );

   const pc::Vec3d overlap{ { 5.25, 1.0, 3.0 } };
   std::vector<pc::BodyId> list;
   for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive, pc::QueryMethod::spatialIndex } ) {
      expect( engine.owner( overlap, m ) == a, "O: the minimum id owns the overlap" );
      engine.owners( overlap, list, m );
      expect( list == std::vector<pc::BodyId>( { a, b } ), "O: owner list ascending, both ids" );
      expect( engine.owner( pc::Vec3d{ { 5.9, 1.0, 3.0 } }, m ) == b, "O: only b" );
      expect( engine.owner( pc::Vec3d{ { 5.0, 5.0, 5.0 } }, m ) == pc::noBody,
              "O: fixed cylinder" );
      expect( engine.owner( pc::Vec3d{ { 3.0, 3.0, 1.0 } }, m ) == movCyl, "O: movable cylinder" );
      expect( engine.owner( pc::Vec3d{ { 1.0, 1.0, 5.0 } }, m ) == fixedSph, "O: fixed sphere" );
      expect( engine.owner( pc::Vec3d{ { 7.0, 7.0, -1.7 } }, m ) == pc::noBody, "O: behind plane" );
   }
   expect( engine.contains( a, overlap ) && engine.contains( b, overlap ), "O: contains both" );
   expect( !engine.contains( fixedCyl, pc::Vec3d{ { 5.0, 5.0, 5.0 } } ), "O: contains(fixed cyl)" );
   expect( !engine.contains( plane, pc::Vec3d{ { 7.0, 7.0, -1.7 } } ), "O: contains(plane)" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stage C: after motion, with and without refreshSpatialIndex().
 */
static void stageMotion( pc::Engine& engine )
{
   std::mt19937 rng( 9u );
   std::uniform_real_distribution<double> u( -1.0, 1.0 );
   std::vector<pc::BodyState> movable;
   engine.states( pc::Selection::movable, movable );
   for( const pc::BodyState& s : movable ) {
      const pc::Vec3d v{ { 4.0*u( rng ), 4.0*u( rng ), 4.0*u( rng ) } };
      const pc::Vec3d w{ { u( rng ), u( rng ), u( rng ) } };
      const pc::KinematicState k{ s.position, s.orientation, v, w };
      engine.setKinematicState( s.id, k );
   }
   for( int i=0; i<5; ++i ) engine.advance( 0.05 );
   expect( engine.spatialIndexActive(), "C: the hash grid is active" );

   // One body is also moved far by a state override: the grid still files it where it was
   const pc::BodyState jumper( engine.state( movable.front().id ) );
   const pc::Vec3d landing{ { jumper.position[0] + 12.0, jumper.position[1], jumper.position[2] } };
   engine.setKinematicState( jumper.id, pc::KinematicState{ landing, jumper.orientation,
                                                            jumper.linearVelocity,
                                                            jumper.angularVelocity } );

   std::vector<pc::Vec3d> pts( makePoints( pc::Vec3d{ { -2.5, -2.5, -2.0 } },
                                           pc::Vec3d{ { 8.5, 8.5, 8.5 } }, 40000, 31u ) );
   engine.states( pc::Selection::all, movable );
   for( std::size_t i=0; i<movable.size(); ++i )
      if( movable[i].shape != pc::ShapeKind::plane )
         addShellPoints( pts, movable[i].position, 0.5, 3000u + unsigned( i ) );
   pts.push_back( landing );

   int inside( 0 );
   const int stale( compareMethods( engine, pts, inside ) );
   std::printf( "C [control, no refresh]: %d disagreements of %zu points\n", stale, pts.size() );
   expect( engine.owner( landing, pc::QueryMethod::exhaustive ) == jumper.id,
           "C: the moved body owns its new centre (exhaustive)" );
   expect( engine.owner( landing, pc::QueryMethod::spatialIndex ) != jumper.id,
           "C [control]: without a refresh the spatial index misses the moved body" );

   engine.refreshSpatialIndex();
   const int bad( compareMethods( engine, pts, inside ) );
   std::printf( "C [refreshed]: %d inside, %d disagreements\n", inside, bad );
   expect( inside > 2000, "C: enough points inside bodies" );
   expect( bad == 0, "C: after motion and refresh the methods agree" );
   expect( engine.owner( landing, pc::QueryMethod::spatialIndex ) == jumper.id,
           "C: after the refresh the spatial index finds the moved body" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stage K/E: batch containment for one body against contains(); invalid query points.
 */
static void stageBatchAndValidation( pc::Engine& engine )
{
   std::vector<pc::BodyState> all;
   engine.states( pc::Selection::all, all );
   std::vector<pc::Vec3d> pts( makePoints( pc::Vec3d{ { -1.5, -1.5, -2.0 } },
                                           pc::Vec3d{ { 7.5, 7.5, 7.5 } }, 20000, 41u ) );
   std::vector<double> xyz;
   for( const pc::Vec3d& p : pts ) xyz.insert( xyz.end(), p.begin(), p.end() );

   int bad( 0 ), hits( 0 );
   std::vector<unsigned char> flags( pts.size(), 7u );
   for( const pc::BodyState& b : all ) {
      engine.containsPoints( b.id, xyz.data(), pts.size(), flags.data() );
      for( std::size_t i=0; i<pts.size(); ++i ) {
         const bool expected( engine.contains( b.id, pts[i] ) );
         if( flags[i] != ( expected ? 1u : 0u ) ) ++bad;
         if( expected ) ++hits;
      }
   }
   std::printf( "K containsPoints: %zu bodies x %zu points, %d hits, %d mismatches\n",
                all.size(), pts.size(), hits, bad );
   expect( bad == 0, "K: containsPoints equals contains() for every body and point" );
   expect( hits > 1000, "K: containsPoints saw hits" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.containsPoints( pc::noBody, xyz.data(), 1, flags.data() ); }, "K: unknown body" );
   engine.containsPoints( all.front().id, nullptr, 0, nullptr );   // empty batch is fine

   // Invalid query points (E): non-finite everywhere, out of the hash grid's range for the
   // spatial index; nothing is written for a refused batch
   const double inf( std::numeric_limits<double>::infinity() );
   const double nan( std::numeric_limits<double>::quiet_NaN() );
   for( const pc::QueryMethod m : { pc::QueryMethod::exhaustive, pc::QueryMethod::spatialIndex } ) {
      expectThrows<std::invalid_argument>( [&]() {
         engine.owner( pc::Vec3d{ { nan, 0.0, 0.0 } }, m ); }, "E: NaN point", "not finite" );
      std::vector<pc::BodyId> list;
      expectThrows<std::invalid_argument>( [&]() {
         engine.owners( pc::Vec3d{ { 0.0, inf, 0.0 } }, list, m ); }, "E: infinite point" );
      std::vector<double> batch = { 1.0, 1.0, 1.0, 0.0, 0.0, nan };
      std::vector<pc::BodyId> out( 2, 12345u );
      expectThrows<std::invalid_argument>( [&]() {
         engine.classify( batch.data(), 2, out.data(), m ); }, "E: NaN in a batch" );
      expect( out[0] == 12345u, "E: a refused batch writes nothing" );
   }
   expect( engine.spatialIndexActive(), "E: the grid is active (the range check applies)" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.owner( pc::Vec3d{ { 1.0e300, 0.0, 0.0 } }, pc::QueryMethod::spatialIndex ); },
      "E: a coordinate beyond the hash grid's range", "range" );
   expect( engine.owner( pc::Vec3d{ { 1.0e300, 0.0, 0.0 } }, pc::QueryMethod::exhaustive ) ==
           pc::noBody, "E: the exhaustive method handles huge finite coordinates" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.contains( all.front().id, pc::Vec3d{ { nan, 0.0, 0.0 } } ); }, "E: contains(NaN)" );
   expectThrows<std::invalid_argument>( [&]() {
      engine.containsPoints( all.front().id, std::vector<double>{ 0.0, inf, 0.0 }.data(), 1,
                             flags.data() ); }, "E: containsPoints(inf)" );
}
//*************************************************************************************************


int main()
{
   pc::Engine engine;
   const pc::MaterialId mat( engine.createMaterial( "classifyMat", 1500.0, 0.0, 0.1, 0.1 ) );

   stageAnalytic( engine, mat );

   pe::theWorld()->clear();
   std::vector<pc::Vec3d> probes;
   buildCrowd( engine, mat, probes );
   engine.advance( 1.0e-6 );   // the first contact search files the bodies and activates the grid
   stageGrid( engine );
   stageOwnership( engine );

   // probes on the bodies' surfaces, in the refreshed state after the tiny step
   int inside( 0 );
   engine.refreshSpatialIndex();
   expect( compareMethods( engine, probes, inside ) == 0, "B: surface probes agree" );
   stageBatchAndValidation( engine );

   stageMotion( engine );
   return coupling_test::report( "pe_coupling_classification_test" );
}
