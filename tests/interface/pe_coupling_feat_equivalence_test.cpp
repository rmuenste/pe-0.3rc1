//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_feat_equivalence_test.cpp
 *  \brief pe::coupling reproduces the FEAT3 wrapper (pe_world.cpp) bit for bit
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
 *  FEAT3's ten Cate sedimentation benchmark was validated with its own wrapper class
 *  FEAT::Control::RigidBody::PhysicsEngine (control/rigid_body/pe_world.cpp), which drives pe
 *  directly. The coupling API replaces it, so for one sphere and substeps = 1 it must do the
 *  same arithmetic. FeatWrapperReplica below copies the wrapper's pe calls statement by
 *  statement (constructor, set_gravity, set_fluid, set_contact_parameters, set_split_impulse,
 *  add_material, add_plane, add_sphere, body_state, apply_hydrodynamic_load, advance); the same
 *  scenario then runs through pe::coupling::Engine, and position, velocities and orientation
 *  are compared bit for bit after every one of 1500 steps. The scenario is the benchmark's
 *  shape: a sphere settling onto the bottom plane under gravity and buoyancy, a state-dependent
 *  load (drag, a lateral force and a torque) handed over every step, through the impact and
 *  the resting contact. It runs for two configurations: relaxation 0.6 without split impulses,
 *  relaxation 1 with split impulses.
 *  Linked against pe_static_lubstage_fluid (HardContactAndFluid, the FEAT3 build's solver).
 */
//=================================================================================================

#include <pe/config/SimulationConfig.h>
#include <pe/core.h>
#include <pe/core/lubrication/Params.h>
#include <pe/interface/coupling/Engine.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::sameBits;
namespace pc = pe::coupling;


//*************************************************************************************************
/*!\brief The scenario parameters.
 */
struct Scenario
{
   const char* name;
   double      relaxation;
   bool        splitImpulse;
};
//*************************************************************************************************

static const double rhoP( 1120.0 ), rhoF( 970.0 ), muF( 0.373 ), radius( 0.0075 ), friction( 0.25 );
static const double dt( 5.0e-4 );
static const int    numSteps( 1500 );


//*************************************************************************************************
/*!\brief One step's state, as both paths report it.
 */
struct Sample
{
   pc::Vec3d position, velocity, angularVelocity;
   pc::Quatd orientation;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The synthetic hydrodynamic load: depends on the state, so a divergence would grow.
 */
static void syntheticLoad( const Sample& s, int step, pc::Vec3d& force, pc::Vec3d& torque )
{
   const double c( 6.0 * pe::M_PI * muF * radius );
   const double ct( 8.0 * pe::M_PI * muF * radius * radius * radius );
   force  = pc::Vec3d{ { -c * s.velocity[0] + 1.0e-6 * ( step % 7 ),
                         -c * s.velocity[1] - 2.0e-7,
                         -c * s.velocity[2] + 3.0e-6 * s.angularVelocity[1] } };
   torque = pc::Vec3d{ { -ct * s.angularVelocity[0] + 1.0e-9,
                         -ct * s.angularVelocity[1] - 1.0e-6 * s.velocity[0],
                         -ct * s.angularVelocity[2] } };
}
//*************************************************************************************************




//=================================================================================================
//
//  THE FEAT3 WRAPPER, REPLICATED (control/rigid_body/pe_world.cpp, same calls in the same order)
//
//=================================================================================================

class FeatWrapperReplica
{
public:
   struct RawLoad { pc::Vec3d force; pc::Vec3d torque; };

   FeatWrapperReplica()
   {
      world = pe::theWorld();
      pe::SimulationConfig::getInstance().setCfdRank( 0 );
      pe::lubrication::setEnabled( false );
      world->setAutoForceReset( true );
      world->setDamping( pe::real( 1 ) );
   }

   static pe::Vec3 to_pe( const pc::Vec3d& v )
   {
      return pe::Vec3( pe::real( v[0] ), pe::real( v[1] ), pe::real( v[2] ) );
   }

   static pc::Vec3d from_pe( const pe::Vec3& v )
   {
      return pc::Vec3d{ { double( v[0] ), double( v[1] ), double( v[2] ) } };
   }

   void set_gravity( const pc::Vec3d& gravity ) { world->setGravity( to_pe( gravity ) ); }

   void set_fluid( double density, double dynamic_viscosity )
   {
      world->setLiquidDensity( pe::real( density ) );
      world->setViscosity( pe::real( dynamic_viscosity ) );
      world->setLiquidSolid( true );
   }

   void set_contact_parameters( double erp, std::size_t max_iter, double relaxation )
   {
      pe::CollisionSystemID cs = pe::theCollisionSystem();
      cs->setErrorReductionParameter( pe::real( erp ) );
      cs->setMaxIterations( max_iter );
      cs->setRelaxationParameter( pe::real( relaxation ) );
   }

   void set_split_impulse( bool enable ) { pe::theCollisionSystem()->setSplitImpulse( enable ); }

   std::size_t add_material( const std::string& name, double density, double restitution,
                             double static_friction, double dynamic_friction )
   {
      pe::MaterialID mat = pe::createMaterial( name, pe::real( density ), pe::real( restitution ),
         pe::real( static_friction ), pe::real( dynamic_friction ), pe::real( 0.3 ),
         pe::real( 2.0e6 ), pe::real( 1.0e6 ), pe::real( 1.0e3 ), pe::real( 2.0e3 ) );
      materials.push_back( mat );
      return materials.size() - 1u;
   }

   std::size_t add_sphere( const pc::Vec3d& centre, double r, std::size_t material )
   {
      pe::SphereID sphere = pe::createSphere( next_id++, to_pe( centre ), pe::real( r ),
                                              materials[material] );
      bodies.push_back( sphere );
      prev_loads.push_back( RawLoad{ pc::Vec3d{ { 0.0, 0.0, 0.0 } },
                                     pc::Vec3d{ { 0.0, 0.0, 0.0 } } } );
      has_prev_load.push_back( char( 0 ) );
      return bodies.size() - 1u;
   }

   void add_plane( const pc::Vec3d& normal, double offset, std::size_t material )
   {
      pe::PlaneID plane = pe::createPlane( next_id++, pe::real( normal[0] ), pe::real( normal[1] ),
         pe::real( normal[2] ), pe::real( offset ), materials[material] );
      plane->setFixed( true );
   }

   Sample body_state( std::size_t body ) const
   {
      const pe::BodyID b = bodies[body];
      Sample s;
      s.position = from_pe( b->getPosition() );
      s.velocity = from_pe( b->getLinearVel() );
      s.angularVelocity = from_pe( b->getAngularVel() );
      const pe::Quat& q = b->getQuaternion();
      s.orientation = pc::Quatd{ { double( q[0] ), double( q[1] ), double( q[2] ),
                                   double( q[3] ) } };
      return s;
   }

   void apply_hydrodynamic_load( std::size_t body, const RawLoad& load, double relaxation,
                                 double step )
   {
      pe::BodyID b = bodies[body];
      RawLoad eff( load );
      if( has_prev_load[body] != 0 ) {
         const RawLoad& prev = prev_loads[body];
         for( int k( 0 ); k < 3; ++k ) {
            eff.force[k] = relaxation * load.force[k] + ( 1.0 - relaxation ) * prev.force[k];
            eff.torque[k] = relaxation * load.torque[k] + ( 1.0 - relaxation ) * prev.torque[k];
         }
      }
      prev_loads[body] = load;
      has_prev_load[body] = char( 1 );
      b->setForce( to_pe( eff.force ) );
      b->setTorque( to_pe( eff.torque ) );
      b->applyFluidForces( pe::real( step ), pe::real( 1 ) );
   }

   void advance( double step ) { world->simulationStep( pe::real( step ) ); }

private:
   pe::WorldID                 world;
   std::vector<pe::BodyID>     bodies;
   std::vector<RawLoad>        prev_loads;
   std::vector<char>           has_prev_load;
   std::vector<pe::MaterialID> materials;
   pe::id_t                    next_id = 1u;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The scenario through the replicated FEAT3 wrapper (as ccnd_sedimentation drives it).
 */
static std::vector<Sample> runReplica( const Scenario& sc )
{
   FeatWrapperReplica engine;
   engine.set_gravity( pc::Vec3d{ { 0.0, 0.0, -9.81 } } );
   engine.set_fluid( rhoF, muF );
   engine.set_contact_parameters( 0.8, 150, 0.9 );
   engine.set_split_impulse( sc.splitImpulse );
   const std::size_t matSphere( engine.add_material( std::string( "featSphere_" ) + sc.name,
                                                     rhoP, 0.0, friction, friction ) );
   const std::size_t matWall( engine.add_material( std::string( "featWall_" ) + sc.name,
                                                   2500.0, 0.0, friction, friction ) );
   engine.add_plane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, matWall );
   const std::size_t body( engine.add_sphere( pc::Vec3d{ { 1.0e-3, -5.0e-4, 0.02 } }, radius,
                                              matSphere ) );

   std::vector<Sample> trace;
   for( int step=0; step<numSteps; ++step ) {
      const Sample s( engine.body_state( body ) );
      FeatWrapperReplica::RawLoad load;
      syntheticLoad( s, step, load.force, load.torque );
      engine.apply_hydrodynamic_load( body, load, sc.relaxation, dt );
      engine.advance( dt );
      trace.push_back( engine.body_state( body ) );
   }
   return trace;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The same scenario through pe::coupling::Engine.
 */
static std::vector<Sample> runApi( const Scenario& sc )
{
   pc::Engine engine;
   engine.setGravity( pc::Vec3d{ { 0.0, 0.0, -9.81 } } );
   engine.setFluidDensity( rhoF );
   engine.setFluidViscosity( muF );
   engine.setErrorReduction( 0.8 );
   engine.setMaxIterations( 150 );
   engine.setContactRelaxation( 0.9 );
   engine.setSplitImpulse( sc.splitImpulse );
   const pc::MaterialId matSphere( engine.createMaterial( std::string( "apiSphere_" ) + sc.name,
                                                          rhoP, 0.0, friction, friction ) );
   const pc::MaterialId matWall( engine.createMaterial( std::string( "apiWall_" ) + sc.name,
                                                        2500.0, 0.0, friction, friction ) );
   engine.createPlane( pc::Vec3d{ { 0.0, 0.0, 1.0 } }, 0.0, matWall, 1 );
   const pc::BodyId body( engine.createSphere( pc::Vec3d{ { 1.0e-3, -5.0e-4, 0.02 } }, radius,
                                               matSphere, 2 ) );

   auto sample = [&]() {
      const pc::BodyState st( engine.state( body ) );
      return Sample{ st.position, st.linearVelocity, st.angularVelocity, st.orientation };
   };

   std::vector<Sample> trace;
   for( int step=0; step<numSteps; ++step ) {
      pc::Vec3d force, torque;
      syntheticLoad( sample(), step, force, torque );
      engine.addLoad( body, force, torque );
      engine.applyLoads( sc.relaxation, dt );
      engine.advance( dt, 1 );
      trace.push_back( sample() );
   }
   return trace;
}
//*************************************************************************************************


int main()
{
   const Scenario scenarios[] = { { "relaxed", 0.6, false }, { "split", 1.0, true } };

   for( const Scenario& sc : scenarios ) {
      pe::theWorld()->clear();
      const std::vector<Sample> ref( runReplica( sc ) );
      pe::theWorld()->clear();
      const std::vector<Sample> api( runApi( sc ) );

      int firstDiff( -1 );
      for( std::size_t i=0; i<ref.size() && firstDiff<0; ++i ) {
         if( !sameBits( ref[i].position, api[i].position ) ||
             !sameBits( ref[i].velocity, api[i].velocity ) ||
             !sameBits( ref[i].angularVelocity, api[i].angularVelocity ) ||
             !sameBits( ref[i].orientation, api[i].orientation ) )
            firstDiff = int( i );
      }
      double minZ( 1.0 );
      for( const Sample& s : ref ) minZ = std::min( minZ, s.position[2] );
      std::printf( "%s: %d steps, final z = %.17g, w_y = %.6g, min z = %.10g, "
                   "first difference: %d\n", sc.name, numSteps, ref.back().position[2],
                   ref.back().angularVelocity[1], minZ, firstDiff );
      expect( firstDiff < 0, std::string( sc.name ) + ": API trajectory equals the FEAT3 wrapper "
                             "trajectory bit for bit" );
      expect( minZ < radius * 1.01, std::string( sc.name ) + ": the sphere reached the plane" );
      expect( ref.back().angularVelocity[1] != 0.0, std::string( sc.name ) + ": the torque acts" );
   }
   return coupling_test::report( "pe_coupling_feat_equivalence_test" );
}
