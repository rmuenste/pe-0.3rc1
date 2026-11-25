//=================================================================================================
/*!
 *  \file tests/interface/pe_reload_bounding_box_test.cpp
 *  \brief The bounding box of a rotated body restored by pe's binary checkpoint reader
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
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 *
 *  The instantiation constructors of Box, Capsule, Cylinder and Ellipsoid (used by the binary
 *  reader and for MPI shadow copies) set the orientation after the parent constructor had
 *  computed the bounding box for the identity orientation, and did not recompute it: a rotated
 *  body came back with the box of the unrotated one until it moved. This test writes rotated
 *  bodies of all four shapes, reads them back and requires the bounding box, also as reported
 *  by the coupling API, to be bit for bit the one before the checkpoint.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/interface/coupling/Engine.h>
#include <pe/util/Checkpointer.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
namespace pc = pe::coupling;


//*************************************************************************************************
/*!\brief Whether two bounding boxes are the same bit for bit.
 */
static bool sameBox( const pe::RigidBody::AABB& a, const pe::RigidBody::AABB& b )
{
   for( std::size_t i=0; i<6; ++i ) {
      const pe::real x( a[i] ), y( b[i] );
      if( std::memcmp( &x, &y, sizeof( pe::real ) ) != 0 )
         return false;
   }
   return true;
}
//*************************************************************************************************


int main()
{
   pe::WorldID world( pe::theWorld() );
   world->clear();

   // A rotation that is not a symmetry of any of the shapes: the long axes leave the x axis.
   const pe::Quat q( pe::Vec3( 1.0, 2.0, 3.0 ), 0.7 );
   const pe::Vec3 c( 0.5, -1.0, 2.0 );

   std::vector<pe::BodyID> bodies;
   bodies.push_back( pe::createBox( 1, c, pe::Vec3( 20.0, 2.0, 1.0 ), pe::granite ) );
   bodies.push_back( pe::createCapsule( 2, c + pe::Vec3( 30.0, 0.0, 0.0 ), 0.5, 10.0, pe::granite ) );
   bodies.push_back( pe::createCylinder( 3, c + pe::Vec3( 60.0, 0.0, 0.0 ), 0.5, 10.0, pe::granite ) );
   bodies.push_back( pe::createEllipsoid( 4, c + pe::Vec3( 90.0, 0.0, 0.0 ), 8.0, 2.0, 1.0,
                                          pe::granite, true ) );

   std::vector<pe::id_t> ids;
   std::vector<pe::RigidBody::AABB> before;
   for( pe::BodyID b : bodies ) {
      b->setOrientation( q );   // recomputes the box
      ids.push_back( b->getSystemID() );
      before.push_back( b->getAABB() );
   }

   const std::string dir( "./_reload_bounding_box" ), name( "rotated" );
   pe::writeCheckpoint( dir, name );
   pe::readCheckpoint( dir, name );   // destroys the bodies and instantiates them from the file

   const char* shape[] = { "box", "capsule", "cylinder", "ellipsoid" };
   pc::Engine engine;
   for( std::size_t k=0; k<ids.size(); ++k ) {
      const auto it( pe::theCollisionSystem()->getBodyStorage().find( ids[k] ) );
      expect( it != pe::theCollisionSystem()->getBodyStorage().end(),
              std::string( shape[k] ) + ": restored" );
      const pe::ConstBodyID b( *it );
      expect( sameBox( b->getAABB(), before[k] ),
              std::string( shape[k] ) + ": the restored bounding box is the one of the rotated body" );
      const pc::BodyState s( engine.state( ids[k] ) );
      bool same( true );
      for( std::size_t i=0; i<3; ++i )
         same = same && s.aabbMin[i] == before[k][i] && s.aabbMax[i] == before[k][i+3];
      expect( same, std::string( shape[k] ) + ": Engine::state() reports that bounding box" );
   }

   world->clear();
   std::cout << "pe_reload_bounding_box_test: all checks passed\n";
   return 0;
}
