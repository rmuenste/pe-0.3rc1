//=================================================================================================
/*!
 *  \file tests/interface/pe_atc_arc_containment.cpp
 *  \brief Containment of spheres inside the boundary surface of the ATC arc, in both PE modes.
 *
 *  The case is the liquid segment of the Archimedes tube crystalliser: a 90 degree arc of a
 *  coiled tube closed by two spherical caps (the 90 deg mesh M273). Its boundary surface is a
 *  fixed triangle mesh with an inverted DistanceMap, the spheres start inside it.
 *
 *  One source, two programs:
 *   - built with MPI it calls the parallel setup setupATC() and needs one process per slab of
 *     single_plane.txt (the planes the CFD mesh is partitioned with),
 *   - built without MPI it calls the serial-mode setup setupATCSerial().
 *
 *  Both then drive the engine the way an Euler-Lagrange CFD driver does (the driver owns the
 *  body forces): every step each sphere gets its submerged weight, optionally a Stokes drag
 *  towards the fluid at rest, and the world is advanced by one step. The spheres fall onto the
 *  wall, slide along it and pile up at the lowest point of the arc.
 *
 *  After every step the centre of every owned sphere is compared with the exact surface
 *  (clearance(), the same formula the generator of the inputs uses):
 *
 *     penetration = radius - clearance(centre)     > 0: the sphere reaches through the surface
 *     escaped     = clearance(centre) < 0          the centre itself is outside
 *
 *  and the number of owned spheres is summed over the processes: it has to stay the number of
 *  start positions (nothing lost, nothing owned twice).
 *
 *  Usage: pe_atc_arc_containment --inputs DIR --run-dir DIR [options]
 *     --inputs DIR                directory with example.json, the boundary OBJ, particles.xyz,
 *                                 sorted_vertices_by_x_world.txt and single_plane.txt
 *     --run-dir DIR               working directory of the run (created, inputs are copied)
 *     --steps N                   number of steps (default: timesteps_ of the deck)
 *     --gravity GX GY GZ          gravity (default: gravity_ of the deck)
 *     --drag                      add the Stokes drag of the deck's fluid
 *     --report-every N            print a status line every N steps (default 0 = never)
 *     --dump FILE                 write "id x y z" of every sphere after the last step
 *     --max-penetration-radii X   fail above this penetration, in sphere radii (default 1)
 *
 *  Exit code 0 if no centre ever left the domain, the number of spheres was conserved and the
 *  penetration stayed below the limit.
 */
//=================================================================================================

#include <pe/interface/setup/atc.h>

#include <pe/core.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#if HAVE_MPI
#include <mpi.h>
#endif

namespace {

//=================================================================================================
//  Exact geometry of the arc [cm]; must match fixtures/atc_arc/gen_atc_arc_inputs.py
//=================================================================================================
const double kPi   = 3.14159265358979323846;
const double R_C   = 2.5;      // coil centreline radius (coil axis = z)
const double R_T   = 0.25;     // tube inner radius
const double PITCH = 0.7;      // axial advance per turn
const double PHI_A = -45.0;    // arc start, z = 0 (the +x end)
const double PHI_B = -135.0;   // arc end (the -x end)
const double R_SPH = 0.5;      // radius of the cap spheres

struct Cap {
   double c[3];   // sphere centre
   double n[3];   // outward normal of the end plane
};

Cap makeCap( double phiDeg, double outwardSign )
{
   const double p( phiDeg * kPi / 180.0 );
   const double back( std::sqrt( R_SPH * R_SPH - R_T * R_T ) );
   Cap cap;
   cap.n[0] = outwardSign * -std::sin( p );
   cap.n[1] = outwardSign *  std::cos( p );
   cap.n[2] = 0.0;
   cap.c[0] = R_C * std::cos( p ) - back * cap.n[0];
   cap.c[1] = R_C * std::sin( p ) - back * cap.n[1];
   cap.c[2] = -PITCH * ( phiDeg - PHI_A ) / 360.0;
   return cap;
}

//! Distance of a point to the boundary of the fluid domain, negative outside.
double clearance( const pe::Vec3& pos )
{
   static const double sgn( PHI_B > PHI_A ? 1.0 : -1.0 );
   static const Cap caps[2] = { makeCap( PHI_A, -sgn ), makeCap( PHI_B, sgn ) };
   static const double back( std::sqrt( R_SPH * R_SPH - R_T * R_T ) );
   static const double cosPsi( back / R_SPH );

   const double x( pos[0] ), y( pos[1] ), z( pos[2] );
   const double phi( std::atan2( y, x ) * 180.0 / kPi );
   const double zc( -PITCH * ( phi - PHI_A ) / 360.0 );
   const double wall( R_T - std::hypot( std::hypot( x, y ) - R_C, z - zc ) );

   double out( ( phi >= std::min( PHI_A, PHI_B ) && phi <= std::max( PHI_A, PHI_B ) )
               ? wall : -std::numeric_limits<double>::infinity() );
   for( const Cap& cap : caps ) {
      const double d[3] = { x - cap.c[0], y - cap.c[1], z - cap.c[2] };
      const double dist( std::sqrt( d[0]*d[0] + d[1]*d[1] + d[2]*d[2] ) );
      const double along( d[0]*cap.n[0] + d[1]*cap.n[1] + d[2]*cap.n[2] );
      if( along > back )                     // beyond the end plane: only the cap bounds
         out = R_SPH - dist;
      else if( along >= dist * cosPsi )      // the nearest point of the sphere is on the cap
         out = std::min( out, R_SPH - dist );
   }
   return out;
}

struct Options {
   std::string inputs;
   std::string runDir;
   std::string dump;
   long   steps = -1;
   bool   hasGravity = false;
   double gravity[3] = { 0.0, 0.0, 0.0 };
   bool   drag = false;
   long   reportEvery = 0;
   double maxPenetrationRadii = 1.0;
};

bool parse( int argc, char** argv, Options& o )
{
   for( int i = 1; i < argc; ++i ) {
      const std::string a( argv[i] );
      const auto need = [&]( int n ) { return i + n < argc; };
      if( a == "--inputs" && need(1) )                        o.inputs = argv[++i];
      else if( a == "--run-dir" && need(1) )                  o.runDir = argv[++i];
      else if( a == "--dump" && need(1) )                     o.dump = argv[++i];
      else if( a == "--steps" && need(1) )                    o.steps = std::atol( argv[++i] );
      else if( a == "--report-every" && need(1) )             o.reportEvery = std::atol( argv[++i] );
      else if( a == "--max-penetration-radii" && need(1) )    o.maxPenetrationRadii = std::atof( argv[++i] );
      else if( a == "--drag" )                                o.drag = true;
      else if( a == "--gravity" && need(3) ) {
         o.hasGravity = true;
         for( int k = 0; k < 3; ++k ) o.gravity[k] = std::atof( argv[++i] );
      }
      else return false;
   }
   return !o.inputs.empty() && !o.runDir.empty();
}

//! Statistics of one process over one step, reduced over the processes afterwards.
struct StepStats {
   long   owned = 0;
   long   escaped = 0;                // centre outside the domain
   long   penetrating = 0;            // penetration above 1 % of the radius
   double maxPenetration = -std::numeric_limits<double>::infinity();
   double maxSpeed = 0.0;
   double kinetic = 0.0;
};

void reduce( StepStats& s )
{
#if HAVE_MPI
   long   li[3] = { s.owned, s.escaped, s.penetrating }, lo[3];
   double di[2] = { s.maxPenetration, s.maxSpeed }, dout[2];
   double ke( 0.0 );
   MPI_Allreduce( li, lo, 3, MPI_LONG, MPI_SUM, MPI_COMM_WORLD );
   MPI_Allreduce( di, dout, 2, MPI_DOUBLE, MPI_MAX, MPI_COMM_WORLD );
   MPI_Allreduce( &s.kinetic, &ke, 1, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD );
   s.owned = lo[0]; s.escaped = lo[1]; s.penetrating = lo[2];
   s.maxPenetration = dout[0]; s.maxSpeed = dout[1]; s.kinetic = ke;
#else
   (void)s;
#endif
}

}  // namespace


int main( int argc, char** argv )
{
   int rank( 0 );
   int size( 1 );
#if HAVE_MPI
   MPI_Init( &argc, &argv );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
#endif
   const bool root( rank == 0 );

   const auto fail = [&]( const std::string& message ) {
      if( root ) std::cerr << "pe_atc_arc_containment: " << message << "\n";
#if HAVE_MPI
      MPI_Abort( MPI_COMM_WORLD, 2 );
#endif
      std::exit( 2 );
   };

   Options opt;
   if( !parse( argc, argv, opt ) )
      fail( "usage: pe_atc_arc_containment --inputs DIR --run-dir DIR [--steps N] "
            "[--gravity GX GY GZ] [--drag] [--report-every N] [--dump FILE] "
            "[--max-penetration-radii X]" );

   // --- run directory: the root copies the inputs, then everybody changes into it ------------
   namespace fs = boost::filesystem;
   const fs::path dumpPath( opt.dump.empty() ? fs::path() : fs::absolute( opt.dump ) );
   if( root ) {
      try {
         fs::create_directories( opt.runDir );
         for( fs::directory_iterator it( opt.inputs ); it != fs::directory_iterator(); ++it ) {
            if( fs::is_regular_file( it->path() ) )
               fs::copy_file( it->path(), fs::path( opt.runDir ) / it->path().filename(),
                              fs::copy_options::overwrite_existing );
         }
      }
      catch( const std::exception& e ) {
         fail( std::string( "cannot prepare the run directory: " ) + e.what() );
      }
   }
#if HAVE_MPI
   MPI_Barrier( MPI_COMM_WORLD );
#endif
   fs::current_path( opt.runDir );

   // --- setup ---------------------------------------------------------------------------------
#if HAVE_MPI
   const char* const mode( "parallel (setupATC)" );
   pe::setupATC( MPI_COMM_WORLD );
#else
   const char* const mode( "serial (setupATCSerial)" );
   pe::setupATCSerial( 1 );
#endif

   pe::WorldID world( pe::theWorld() );
   const pe::SimulationConfig& config( pe::SimulationConfig::getInstance() );

   if( opt.hasGravity )
      world->setGravity( opt.gravity[0], opt.gravity[1], opt.gravity[2] );
   const pe::Vec3 gravity( world->getGravity() );
   const long   steps( opt.steps >= 0 ? opt.steps : static_cast<long>( config.getTimesteps() ) );
   const double dt( config.getStepsize() );
   const double radius( config.getBenchRadius() );
   const double rhoP( config.getParticleDensity() );
   const double rhoF( config.getFluidDensity() );
   const double mu( config.getFluidViscosity() );
   const double volume( 4.0 / 3.0 * kPi * radius * radius * radius );
   const double mass( rhoP * volume );
   const pe::Vec3 weight( ( rhoP - rhoF ) * volume * gravity );   // submerged weight
   const double dragB( 6.0 * kPi * mu * radius );                 // Stokes drag coefficient

   // --- initial state -------------------------------------------------------------------------
   long   spheresLocal( 0 );
   double minStartClearance( std::numeric_limits<double>::infinity() );
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      if( it->getType() != pe::sphereType || it->isRemote() ) continue;
      ++spheresLocal;
      minStartClearance = std::min( minStartClearance, clearance( it->getPosition() ) );
   }
   long spheresTotal( spheresLocal );
#if HAVE_MPI
   MPI_Allreduce( &spheresLocal, &spheresTotal, 1, MPI_LONG, MPI_SUM, MPI_COMM_WORLD );
   {
      double tmp( minStartClearance );
      MPI_Allreduce( &tmp, &minStartClearance, 1, MPI_DOUBLE, MPI_MIN, MPI_COMM_WORLD );
   }
#endif

   // --- accuracy of the DistanceMap against the exact surface (root only) ----------------------
   // Probe points: where the centre of a sphere that touches the surface would be, and up to
   // three more radii into the domain; along the whole wall and inside both caps.
   double dmErrorWall( -1.0 );   // more than 5 degrees away from both ends
   double dmErrorEnds( -1.0 );   // near the ends and inside the caps
   double dmSpacing( 0.0 );
   long   dmSamples( 0 );
#ifdef PE_USE_CGAL
   if( root ) {
      for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
         if( it->getType() != pe::triangleMeshType ) continue;
         pe::TriangleMeshID mesh( pe::static_body_cast<pe::TriangleMesh>( *it ) );
         if( !mesh->hasDistanceMap() ) continue;
         const pe::DistanceMap* dm( mesh->getDistanceMap() );
         dmSpacing = dm->getSpacing();
         dmErrorWall = dmErrorEnds = 0.0;

         const auto probe = [&]( const pe::Vec3& p, bool nearEnd ) {
            const double exact( clearance( p ) );
            if( exact < 0.25 * radius ) return;
            const pe::Vec3 local( mesh->pointFromWFtoBF( p ) );
            const double d( dm->interpolateDistance( local[0], local[1], local[2] ) );
            double& worst( nearEnd ? dmErrorEnds : dmErrorWall );
            worst = std::max( worst, std::abs( d - exact ) );
            ++dmSamples;
         };

         for( int i = 0; i <= 360; ++i ) {                       // tube
            const double phiDeg( PHI_A + ( PHI_B - PHI_A ) * i / 360.0 );
            const double phi( phiDeg * kPi / 180.0 );
            const double zc( -PITCH * ( phiDeg - PHI_A ) / 360.0 );
            const bool nearEnd( std::abs( phiDeg - PHI_A ) < 5.0 || std::abs( phiDeg - PHI_B ) < 5.0 );
            for( int j = 0; j < 24; ++j ) {
               const double th( 2.0 * kPi * j / 24.0 );
               for( int k = 0; k < 4; ++k ) {
                  const double rr( R_T - radius * ( 1.0 + k ) );
                  const double rho( R_C + rr * std::cos( th ) );
                  probe( pe::Vec3( rho * std::cos( phi ), rho * std::sin( phi ), zc + rr * std::sin( th ) ), nearEnd );
               }
            }
         }
         const double sgn( PHI_B > PHI_A ? 1.0 : -1.0 );
         const double psiMax( std::asin( R_T / R_SPH ) );
         for( int e = 0; e < 2; ++e ) {                          // caps
            const Cap cap( makeCap( e == 0 ? PHI_A : PHI_B, e == 0 ? -sgn : sgn ) );
            const double pe_( ( e == 0 ? PHI_A : PHI_B ) * kPi / 180.0 );
            const double er[3] = { std::cos( pe_ ), std::sin( pe_ ), 0.0 };
            for( int i = 0; i <= 12; ++i ) {
               const double psi( psiMax * i / 12.0 );
               for( int j = 0; j < 24; ++j ) {
                  const double th( 2.0 * kPi * j / 24.0 );
                  for( int k = 0; k < 4; ++k ) {
                     const double rr( R_SPH - radius * ( 1.0 + k ) );
                     const double a( rr * std::cos( psi ) ), b( rr * std::sin( psi ) );
                     probe( pe::Vec3( cap.c[0] + a * cap.n[0] + b * std::cos( th ) * er[0],
                                      cap.c[1] + a * cap.n[1] + b * std::cos( th ) * er[1],
                                      cap.c[2] + b * std::sin( th ) ), true );
                  }
               }
            }
         }
      }
   }
#endif

   if( root ) {
      std::cout << "\n== pe_atc_arc_containment =====================================================\n"
                << " mode                      = " << mode << ", " << size << " process(es)\n"
                << " spheres                   = " << spheresTotal << ", radius " << radius << "\n"
                << " min start clearance       = " << minStartClearance / radius << " radii\n"
                << " steps, dt                 = " << steps << ", " << dt << "\n"
                << " gravity                   = " << gravity << "\n"
                << " submerged acceleration    = " << ( weight.length() / mass ) << "\n"
                << " Stokes drag               = " << ( opt.drag ? "on" : "off" );
      if( opt.drag )
         std::cout << " (terminal velocity " << weight.length() / dragB << ")";
      std::cout << "\n";
      if( dmErrorWall >= 0.0 )
         std::cout << " DistanceMap spacing       = " << dmSpacing << " (" << dmSpacing / radius << " radii)\n"
                   << " DistanceMap error, wall   = " << dmErrorWall << " (" << dmErrorWall / radius << " radii)\n"
                   << " DistanceMap error, ends   = " << dmErrorEnds << " (" << dmErrorEnds / radius
                   << " radii); max over " << dmSamples << " points\n";
      std::cout << std::flush;
   }

   // --- time loop -----------------------------------------------------------------------------
   double worstPenetration( -std::numeric_limits<double>::infinity() );
   long   worstStep( -1 );
   long   maxEscaped( 0 );
   long   firstEscapeStep( -1 );
   long   stepsWithPenetration( 0 );
   long   maxPenetrating( 0 );
   long   countViolations( 0 );
   double maxSpeedEver( 0.0 );
   StepStats last;

   const std::chrono::steady_clock::time_point wallStart( std::chrono::steady_clock::now() );

   for( long step = 1; step <= steps; ++step ) {
      // body forces of the driver: submerged weight, optional Stokes drag towards fluid at rest
      // (implicit in the velocity, so it is stable for any step size)
      for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
         if( it->getType() != pe::sphereType || it->isRemote() ) continue;
         pe::Vec3 force( weight );
         if( opt.drag ) {
            const pe::Vec3 v( it->getLinearVel() );
            const pe::Vec3 vNew( ( v + ( dt / mass ) * weight ) / ( 1.0 + dt * dragB / mass ) );
            force = ( mass / dt ) * ( vNew - v );
         }
         it->addForce( force );
      }

      world->simulationStep( dt );

      StepStats s;
      for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
         if( it->getType() != pe::sphereType || it->isRemote() ) continue;
         ++s.owned;
         const double c( clearance( it->getPosition() ) );
         const double pen( radius - c );
         if( c < 0.0 ) ++s.escaped;
         if( pen > 0.01 * radius ) ++s.penetrating;
         s.maxPenetration = std::max( s.maxPenetration, pen );
         const double speed( it->getLinearVel().length() );
         s.maxSpeed = std::max( s.maxSpeed, speed );
         s.kinetic += 0.5 * mass * speed * speed;
      }
      reduce( s );

      if( s.maxPenetration > worstPenetration ) { worstPenetration = s.maxPenetration; worstStep = step; }
      if( s.escaped > 0 && firstEscapeStep < 0 ) firstEscapeStep = step;
      maxEscaped = std::max( maxEscaped, s.escaped );
      maxPenetrating = std::max( maxPenetrating, s.penetrating );
      if( s.penetrating > 0 ) ++stepsWithPenetration;
      if( s.owned != spheresTotal ) ++countViolations;
      maxSpeedEver = std::max( maxSpeedEver, s.maxSpeed );
      last = s;

      if( root && opt.reportEvery > 0 && ( step % opt.reportEvery == 0 || step == steps ) ) {
         std::cout << " step " << std::setw(6) << step
                   << "  owned " << s.owned
                   << "  max penetration " << std::setw(10) << s.maxPenetration / radius << " r"
                   << "  penetrating " << std::setw(5) << s.penetrating
                   << "  escaped " << s.escaped
                   << "  max speed " << s.maxSpeed
                   << "  Ekin " << s.kinetic << "\n" << std::flush;
      }
   }

   const double wallTime(
      std::chrono::duration<double>( std::chrono::steady_clock::now() - wallStart ).count() );

   // --- final state ---------------------------------------------------------------------------
   // owned spheres per process, and the dump of the final positions
   std::vector<long> ownedPerRank( static_cast<std::size_t>( size ), 0 );
   {
      long ownedLocal( 0 );
      std::vector<double> rows;   // id x y z
      for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
         if( it->getType() != pe::sphereType || it->isRemote() ) continue;
         ++ownedLocal;
         const pe::Vec3 p( it->getPosition() );
         rows.push_back( static_cast<double>( it->getID() ) );
         rows.push_back( p[0] ); rows.push_back( p[1] ); rows.push_back( p[2] );
      }
#if HAVE_MPI
      MPI_Gather( &ownedLocal, 1, MPI_LONG, &ownedPerRank[0], 1, MPI_LONG, 0, MPI_COMM_WORLD );
      std::vector<int> counts( static_cast<std::size_t>( size ), 0 ), offsets( static_cast<std::size_t>( size ), 0 );
      int n( static_cast<int>( rows.size() ) );
      MPI_Gather( &n, 1, MPI_INT, &counts[0], 1, MPI_INT, 0, MPI_COMM_WORLD );
      std::vector<double> all;
      if( root ) {
         int total( 0 );
         for( int r = 0; r < size; ++r ) { offsets[r] = total; total += counts[r]; }
         all.resize( static_cast<std::size_t>( total ) + 1 );
      }
      double dummy( 0.0 );
      MPI_Gatherv( rows.empty() ? &dummy : &rows[0], n, MPI_DOUBLE,
                   root ? &all[0] : &dummy, &counts[0], &offsets[0], MPI_DOUBLE, 0, MPI_COMM_WORLD );
      if( root ) { all.pop_back(); rows.swap( all ); }
#else
      ownedPerRank[0] = ownedLocal;
#endif
      if( root && !dumpPath.empty() ) {
         std::map<long, std::size_t> byId;
         for( std::size_t i = 0; i < rows.size(); i += 4 ) byId[static_cast<long>( rows[i] )] = i;
         std::ofstream out( dumpPath.string().c_str() );
         out << std::setprecision( 15 );
         for( const auto& entry : byId )
            out << entry.first << " " << rows[entry.second+1] << " " << rows[entry.second+2] << " "
                << rows[entry.second+3] << "\n";
      }
   }

   const bool conserved( countViolations == 0 && last.owned == spheresTotal );
   const bool contained( maxEscaped == 0 );
   const bool shallow( worstPenetration <= opt.maxPenetrationRadii * radius );
   const bool ok( steps == 0 || ( conserved && contained && shallow ) );

   if( root ) {
      std::cout << " -- result ----------------------------------------------------------------------\n"
                << " owned spheres at the end   = " << last.owned << " of " << spheresTotal
                << ( conserved ? "" : "   <-- NOT CONSERVED" ) << "\n"
                << " steps with a wrong count   = " << countViolations << "\n"
                << " owned per process          =";
      for( long n : ownedPerRank ) std::cout << " " << n;
      std::cout << "\n"
                << " max penetration, any step  = " << worstPenetration << " (" << worstPenetration / radius
                << " radii) at step " << worstStep << "\n"
                << " max penetration, last step = " << last.maxPenetration << " ("
                << last.maxPenetration / radius << " radii)\n"
                << " spheres penetrating > 1 %  = " << last.penetrating << " at the end, at most "
                << maxPenetrating << ", in " << stepsWithPenetration << " of " << steps << " steps\n"
                << " centres outside the domain = " << last.escaped << " at the end, at most " << maxEscaped;
      if( firstEscapeStep >= 0 ) std::cout << ", first at step " << firstEscapeStep;
      std::cout << "\n"
                << " max speed, any step        = " << maxSpeedEver << " (" << maxSpeedEver * dt / radius
                << " radii per step)\n"
                << " max speed, last step       = " << last.maxSpeed << "\n"
                << " kinetic energy, last step  = " << last.kinetic << "\n";
      std::cout << " wall time of the steps     = " << wallTime << " s\n";
      std::cout << " verdict                    = " << ( ok ? "PASS" : "FAIL" ) << "\n"
                << "===============================================================================\n"
                << std::flush;
   }

#if HAVE_MPI
   MPI_Finalize();
#endif
   return ok ? 0 : 1;
}
