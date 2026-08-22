//=================================================================================================
/*!
 *  \file examples/mpiperiodic/mpiperiodic.cpp
 *  \brief Periodic transport test: one sphere crossing a periodic box at least twice
 *
 *  A minimal, self-verifying MPI example. The domain is a box of length \a lx in x, cut into
 *  \a px slabs along x, one per process, and PERIODIC in x. A single sphere is launched along
 *  +x with no gravity and nothing to collide with, so it must migrate from slab to slab, wrap
 *  around the periodic boundary, and keep going. The example asserts that it completes at least
 *  two full laps and that every process owned it at least twice.
 *
 *  HOW PERIODICITY WORKS IN PE
 *  ---------------------------
 *  Periodicity is expressed by the \a offset argument of connect(), not by defineLocalDomain().
 *  The local domain is an ordinary slab in unwrapped coordinates. A neighbour that is physically
 *  on the far side of the box is described in ITS OWN coordinates and then translated into place
 *  by the offset:
 *
 *     west neighbour wrapped round the low  end (west[0] <  0 ) -> offset = ( +lx, 0, 0 )
 *     east neighbour wrapped round the high end (east[0] == px) -> offset = ( -lx, 0, 0 )
 *     otherwise                                                 -> offset = (   0, 0, 0 )
 *
 *  MPI_Cart_create() is given periods = { true }, which is what lets MPI_Cart_rank() accept an
 *  out-of-range coordinate and wrap it to the process on the opposite face.
 *
 *  MINIMUM PROCESS COUNT
 *  ---------------------
 *  A periodic axis needs AT LEAST 3 processes, and pe does not check this for you -- it surfaces
 *  as an exception from inside connect() (see pe/core/domaindecomp/DomainDecomposition.h):
 *
 *     px == 1 : center-1 and center+1 both wrap to this rank
 *               -> "Invalid MPI rank self-connect"        (std::invalid_argument)
 *     px == 2 : the west and east neighbours are the SAME rank, connected twice
 *               -> "Remote process is already connected"  (std::invalid_argument)
 *
 *  This example therefore validates px >= 3 up front and reports it in plain language.
 *
 *  Usage:  mpirun -np <N> ./mpiperiodic [--laps L] [--verbose]
 *          N must be >= 3. Every available process is used, so px = N.
 */
//=================================================================================================


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/engine.h>
#include <pe/support.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace pe;


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // Simulation parameters

   const real   lx       ( 1.0   );  // Length of the periodic box in x
   const real   radius   ( 0.02  );  // Radius of the travelling sphere
   const real   speed    ( 0.5   );  // Speed of the sphere along +x
   const real   stepsize ( 0.001 );  // Size of a single time step

   size_t requiredLaps( 2 );         // Laps the sphere must complete (>= 2 by the task)
   bool   verbose     ( false );

   for( int i = 1; i < argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--verbose" ) verbose = true;
      else if( arg == "--laps" && i+1 < argc ) requiredLaps = static_cast<size_t>( std::atoi( argv[++i] ) );
   }

   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );

   WorldID     world     = theWorld();
   MPISystemID mpisystem = theMPISystem();

   const int px( mpisystem->getSize() );  // One slab per process

   // A periodic axis needs at least three processes -- see the header comment. Checked here so
   // the failure is a clear message instead of an exception from inside connect().
   if( px < 3 ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cerr << "\n Invalid number of MPI processes: " << px << "\n"
                   << " A periodic axis needs at least 3 processes (with 1 a process would have to\n"
                   << " connect to itself, with 2 its west and east neighbour are the same rank).\n"
                   << " Re-run with e.g.  mpirun -np 3 ./mpiperiodic\n" << std::endl;
      }
      MPI_Finalize();
      return EXIT_FAILURE;
   }

   const real dx( lx / px );  // Width of one process slab

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 1D periodic domain decomposition along x

   int dims   [] = { px   };
   int periods[] = { true };

   int rank;           // Rank of the neighbouring process
   int center[1];      // Cartesian coordinate of this process (1D topology -> ONE entry)
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 1, dims, periods, false, &cartcomm );
   if( cartcomm == MPI_COMM_NULL ) {
      MPI_Finalize();
      return EXIT_SUCCESS;
   }

   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 1, center );

   int west[] = { center[0]-1 };
   int east[] = { center[0]+1 };

   // The local slab, in unwrapped coordinates: x in [ center*dx, east*dx ]. For the last process
   // east[0] == px, so the upper bound is px*dx == lx -- exactly the box edge. Unbounded in y
   // and z: the sphere only ever moves along x.
   defineLocalDomain( intersect(
      HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
      HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );

   // Connecting the west neighbour. Its geometry is everything left of our lower face, expressed
   // in its own coordinates; the offset carries it across the periodic seam when it wraps.
   {
      const Vec3 offset( ( west[0] < 0 ) ? ( lx ) : ( 0 ), 0, 0 );
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, HalfSpace( Vec3(-1,0,0), -center[0]*dx ), offset );
   }

   // Connecting the east neighbour.
   {
      const Vec3 offset( ( east[0] == px ) ? ( -lx ) : ( 0 ), 0, 0 );
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, HalfSpace( Vec3(+1,0,0), +east[0]*dx ), offset );
   }

   /////////////////////////////////////////////////////
   // Setup of the simulation

   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   world->setGravity( 0.0, 0.0, 0.0 );  // Pure ballistic transport: no gravity, nothing to hit

   // Launch the sphere from the middle of the first slab, so it starts well inside one process.
   const Vec3 start( real(0.5)*dx, 0.0, 0.0 );

   if( world->ownsPoint( start ) ) {
      SphereID sphere = createSphere( 1, start, radius, elastic );
      sphere->setLinearVel( Vec3( speed, 0.0, 0.0 ) );
   }

   world->synchronize();

   // Size the run so the sphere completes the required laps with margin.
   const real    distance ( real(requiredLaps) * lx + real(0.5) * lx );
   const size_t  timesteps( static_cast<size_t>( distance / ( speed * stepsize ) ) + 1 );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--PERIODIC TRANSPORT TEST-------------------------------------------------------\n"
                << " MPI processes (px)                      = " << px        << "\n"
                << " Box length lx                           = " << lx        << "\n"
                << " Slab width dx                           = " << dx        << "\n"
                << " Sphere radius                           = " << radius    << "\n"
                << " Sphere speed                            = " << speed     << "\n"
                << " Time step size                          = " << stepsize  << "\n"
                << " Number of time steps                    = " << timesteps << "\n"
                << " Required laps                           = " << requiredLaps << "\n"
                << "--------------------------------------------------------------------------------\n"
                << std::endl;
   }

   /////////////////////////////////////////////////////
   // Simulation loop with periodic-transport bookkeeping
   //
   // Ownership of the sphere moves between processes, so the position is gathered globally each
   // step: the owning rank contributes x, everyone else contributes a sentinel, and MPI_Allreduce
   // gives every rank the same value. That doubles as a migration check -- exactly one process
   // must own the sphere at any time; zero means it was lost, more than one means it was
   // duplicated instead of migrated.

   const double sentinel( -1.0e300 );

   double xPrev      ( 0.0 );   // Previous global x, for wrap-corrected delta
   double travelled  ( 0.0 );   // Unwrapped distance travelled
   bool   haveFirst  ( false );
   size_t crossings  ( 0 );     // Slab-boundary crossings
   int    slabPrev   ( -1 );
   size_t lostSteps  ( 0 );     // Steps with no owner
   size_t dupSteps   ( 0 );     // Steps with more than one owner

   std::vector<size_t> visits( static_cast<size_t>(px), 0 );  // Times each slab was entered

   for( size_t timestep = 0; timestep < timesteps; ++timestep )
   {
      world->simulationStep( stepsize );

      // Find the sphere among the LOCAL bodies (shadow copies are excluded via isRemote()).
      double xLocal( sentinel );
      int    owners( 0 );

      for( World::SizeType i = 0; i < world->size(); ++i ) {
         BodyID body = world->getBody( static_cast<unsigned int>( i ) );
         if( body->getType() == sphereType && !body->isRemote() ) {
            xLocal = body->getPosition()[0];
            ++owners;
         }
      }

      double xGlobal( sentinel );
      int    ownersTotal( 0 );
      MPI_Allreduce( &xLocal, &xGlobal,     1, MPI_DOUBLE, MPI_MAX, cartcomm );
      MPI_Allreduce( &owners, &ownersTotal, 1, MPI_INT,    MPI_SUM, cartcomm );

      if( ownersTotal == 0 ) { ++lostSteps; continue; }
      if( ownersTotal >  1 ) { ++dupSteps; }

      // Wrap-corrected displacement: a jump larger than half the box is a periodic wrap, not
      // motion. Accumulating this gives the true unwrapped distance travelled.
      if( haveFirst ) {
         double delta = xGlobal - xPrev;
         if     ( delta >  0.5*lx ) delta -= lx;
         else if( delta < -0.5*lx ) delta += lx;
         travelled += delta;
      }
      xPrev = xGlobal;
      haveFirst = true;

      // Which slab is it in, and did it just change?
      int slab = static_cast<int>( xGlobal / dx );
      if( slab <   0 ) slab = 0;
      if( slab >= px ) slab = px - 1;

      if( slab != slabPrev ) {
         if( slabPrev >= 0 ) ++crossings;
         ++visits[ static_cast<size_t>( slab ) ];
         slabPrev = slab;
      }

      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << " step " << std::setw(6) << timestep
                      << "  x = "         << std::fixed << std::setprecision(4) << xGlobal
                      << "  travelled = " << travelled
                      << "  slab = "      << slab
                      << "  owners = "    << ownersTotal << "\n";
         }
      }
   }

   /////////////////////////////////////////////////////
   // Verdict

   const double laps( travelled / lx );

   size_t minVisits( visits.empty() ? 0 : visits[0] );
   for( size_t i = 0; i < visits.size(); ++i )
      if( visits[i] < minVisits ) minVisits = visits[i];

   const bool okLaps    ( laps >= static_cast<double>( requiredLaps ) );
   const bool okVisits  ( minVisits >= requiredLaps );
   const bool okOwners  ( lostSteps == 0 && dupSteps == 0 );
   const bool okCrossing( crossings >= requiredLaps * static_cast<size_t>( px ) );
   const bool passed    ( okLaps && okVisits && okOwners && okCrossing );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--RESULT------------------------------------------------------------------------\n"
                << " Distance travelled (unwrapped)          = " << travelled << "\n"
                << " Laps completed                          = " << laps
                << "   [" << ( okLaps ? "PASS" : "FAIL" ) << ", need >= " << requiredLaps << "]\n"
                << " Slab-boundary crossings                 = " << crossings
                << "   [" << ( okCrossing ? "PASS" : "FAIL" ) << ", need >= " << requiredLaps*px << "]\n"
                << " Least-visited slab entered              = " << minVisits << " times"
                << "   [" << ( okVisits ? "PASS" : "FAIL" ) << ", need >= " << requiredLaps << "]\n"
                << " Steps with no owner / duplicated        = " << lostSteps << " / " << dupSteps
                << "   [" << ( okOwners ? "PASS" : "FAIL" ) << ", need 0 / 0]\n"
                << "--------------------------------------------------------------------------------\n"
                << ( passed ? " PERIODIC TRANSPORT TEST PASSED" : " PERIODIC TRANSPORT TEST FAILED" )
                << "\n--------------------------------------------------------------------------------\n"
                << std::endl;
   }

   MPI_Finalize();

   return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
//*************************************************************************************************
