//=================================================================================================
/*!
 *  \file multisphere.cpp
 *  \brief pe port of LIGGGHTS Tutorials_public/multisphere_stone_restitution (in.multisphere).
 *
 *  LIGGGHTS: two 50-sphere clumps ("stones", data/stone1.multisphere, scale 0.001) dropped with
 *  v = (0,0,-1) onto the plane z = 0; density 2500, restitution 0.3, friction 0.5, Hertz,
 *  dt 1e-5, 40000 steps. The clump positions are random (insert/pack), so this port reads the
 *  sphere positions of the LIGGGHTS dump at step 200 (stones_step200.txt: mol x y z r) and starts
 *  from there with the free-fall velocity of that instant.
 *  pe: one Union of 50 spheres per stone, hard contacts, dt 1e-4. The union mass, centre of mass
 *  and inertia are recomputed from the real clump volume (UnionBase::recomputeMassFromVolume,
 *  1e6 Monte-Carlo samples) unless --no-mc is given.
 */
//=================================================================================================
#include <pe/system/WarningDisable.h>
#include "liggghts_common.h"
#include <map>
using namespace pe;
using namespace lp;

int main( int argc, char* argv[] )
{
   std::vector<char*> filtered; for( int i=0; i<argc; ++i ) if( std::string( argv[i] ) != "--no-mc" ) filtered.push_back( argv[i] );
   Args args = parseArgs( static_cast<int>( filtered.size() ), &filtered[0], "usage: multisphere [--dt 1e-4] [--tend 0.398] [--friction 0.25] [--out dir] [--no-vtk] [--no-mc] [--erp 0.5]  (reads stones_step200.txt)" );
   const real dt      ( args.dt   > 0 ? args.dt   : 1.0e-4 );
   const real tEnd    ( args.tEnd > 0 ? args.tEnd : 0.398 );
   const real friction( args.friction >= 0 ? args.friction : 0.25 );
   const real density ( 2500.0 );
   const Vec3 vel( 0.0, 0.0, -1.0196 );   // free-fall velocity at LIGGGHTS step 200
   bool noMC( false ); const size_t mcSamples( 1000000 );
   for( int i=1; i<argc; ++i ) if( std::string( argv[i] ) == "--no-mc" ) noMC = true;
   const unsigned int steps( stepsFor( tEnd, dt ) ), outSteps( stepsFor( 0.002, dt ) ), thermoSteps( stepsFor( 0.01, dt ) );

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 1.0 );
   theCollisionSystem()->setErrorReductionParameter( args.erp );
   if( args.vtk ) vtk::activateWriter( args.out, outSteps, 0, steps, false, true );

   MaterialID stone = createMaterial( "stone", density, 0.3, friction, friction, 0.45, 1.0e7, 1e6, 1e5, 2e5 );
   MaterialID wall  = createMaterial( "wall" , density, 0.3, friction, friction, 0.45, 1.0e7, 1e6, 1e5, 2e5 );

   unsigned int id( 0 );
   createPlane( ++id, 0.0, 0.0, 1.0, 0.0, wall, false );

   // read "mol x y z r" lines
   std::ifstream in( "stones_step200.txt" );
   if( !in ) { std::cerr << "stones_step200.txt not found\n"; return 1; }
   std::map< int, std::vector<Candidate> > stones;
   int mol; real x, y, z, r;
   while( in >> mol >> x >> y >> z >> r ) { Candidate c; c.pos = Vec3(x,y,z); c.r = r; stones[mol].push_back( c ); }

   std::vector<UnionID> unions;
   for( std::map< int, std::vector<Candidate> >::const_iterator s=stones.begin(); s!=stones.end(); ++s ) {
      UnionID u = createUnion( ++id );
      real sumMass( 0 );
      for( size_t i=0; i<s->second.size(); ++i ) {
         SphereID sp = createSphere( ++id, s->second[i].pos, s->second[i].r, stone );
         sumMass += sp->getMass();
         u->add( sp );
      }
      const real sumMassUnion( u->getMass() );
      const Vec3 sumCentre( u->getPosition() );
      real volume( 0 );
      if( !noMC ) volume = u->recomputeMassFromVolume( mcSamples );   // pe: UnionBase::recomputeMassFromVolume
      u->setLinearVel( vel );
      unions.push_back( u );
      std::cout << " stone " << s->first << ": " << s->second.size() << " spheres\n"
                << "    sum of member masses " << sumMassUnion << " kg (= " << sumMass << "), centre " << sumCentre << "\n"
                << "    Monte-Carlo volume " << volume << " m^3 -> mass " << u->getMass() << " kg, centre " << u->getPosition()
                << "   (LIGGGHTS: 0.0460 kg)\n"
                << "    inertia diag " << u->getInertia()[0] << " " << u->getInertia()[4] << " " << u->getInertia()[8] << "\n";
   }

   std::cout << "\n--MULTISPHERE (pe port)----------------------------------------------------------\n"
             << " dt = " << dt << " s, " << steps << " steps, " << unions.size() << " stones, friction per material " << friction << ", erp " << args.erp << "\n"
             << "--------------------------------------------------------------------------------\n";

   Thermo th; Thermo::header();
   timing::WcTimer timer; timer.start();
   for( unsigned int step=0; step<steps; ++step ) {
      world->simulationStep( dt );
      if( (step+1) % thermoSteps == 0 || step+1 == steps ) { th.measure( world ); th.print( step+1, (step+1)*dt ); }
   }
   timer.end();
   for( size_t i=0; i<unions.size(); ++i )
      std::cout << " stone " << i+1 << " final centre " << unions[i]->getPosition() << " v " << unions[i]->getLinearVel()
                << " w " << unions[i]->getAngularVel() << "\n";
   std::cout << " wall-clock " << timer.total() << " s\n";
   return 0;
}
