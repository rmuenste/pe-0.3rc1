//=================================================================================================
/*!
 *  \file tests/interface/pe_checkpoint_parallel_file_serial_read.cpp
 *  \brief A checkpoint written by two MPI ranks is read by a serial program of the same MPI build.
 *
 *  The checkpoint writer and reader select MPI-IO at run time, when MPI is initialised, and the
 *  stream path otherwise. This test covers the second case in an MPI build with a file that has
 *  more than one body chunk:
 *
 *    --write <dir>   under mpiexec -n 2: each rank creates three spheres, a collective
 *                    writeCheckpoint( dir, "two_ranks" ) follows (MPI-IO path);
 *    --read <dir>    without mpiexec and without MPI_Init: readCheckpoint() must take the stream
 *                    path, see that the file was written by 2 processes while it runs on 1, read
 *                    both chunks and restore all six spheres with their positions.
 *
 *  CTest chains the two through a fixture (pe-checkpoint-two-rank-write is the setup of
 *  pe-checkpoint-two-rank-serial-read). Before the run-time selection, the read aborted with
 *  "MPI_File_open() function was called before MPI_INIT was invoked".
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/core/Materials.h>
#include <pe/core/TimeStep.h>
#include <pe/util/CheckpointMetadata.h>
#include <pe/util/Checkpointer.h>

#include <boost/filesystem.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#if HAVE_MPI
#include <mpi.h>
#endif

namespace fs = boost::filesystem;
using namespace pe;

static const unsigned spheresPerRank = 3;
static const int      ranks          = 2;

static real expectedX( int rank, unsigned i ) { return ( rank == 0 ? real(-1) : real(1) ) * ( real(0.5) + real(i) ); }

static MaterialID particleMaterial()
{
   return createMaterial( "two-rank-particle", real(1.1), real(0.1), real(0.05), real(0.05),
                          real(0.3), real(300), real(1e6), real(1e5), real(2e5) );
}

static int write( int argc, char** argv, const fs::path& dir )
{
#if HAVE_MPI
   MPI_Init( &argc, &argv );
   int rank( 0 ), size( 0 );
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != ranks ) {
      if( rank == 0 ) std::fprintf( stderr, "--write needs exactly %d MPI ranks\n", ranks );
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );
   TimeStep::stepsize( real(0.004) );
   setCheckpointIdentity( real(1.0), 250u, "two-rank file" );
   const MaterialID mat = particleMaterial();
   for( unsigned i = 0; i < spheresPerRank; ++i )
      createSphere( static_cast<pe::id_t>( rank * 100 + i ), Vec3( expectedX( rank, i ), 0.0, 0.0 ), real(0.1), mat );
   if( rank == 0 ) fs::create_directories( dir );
   MPI_Barrier( MPI_COMM_WORLD );
   writeCheckpoint( dir, "two_ranks" );
   MPI_Barrier( MPI_COMM_WORLD );
   if( rank == 0 ) {
      const bool ok = fs::exists( dir / "two_ranks.peb" );
      std::printf( "two-rank write: %s\n", ok ? "two_ranks.peb written" : "FAILED, no .peb" );
      MPI_Finalize();
      return ok ? EXIT_SUCCESS : EXIT_FAILURE;
   }
   MPI_Finalize();
   return EXIT_SUCCESS;
#else
   (void)argc; (void)argv; (void)dir;
   std::printf( "--write needs an MPI build, skipped\n" );
   return 77;
#endif
}

static int read( const fs::path& dir )
{
   // Deliberately no MPI_Init: a serial program of an MPI build.
   if( !fs::exists( dir / "two_ranks.peb" ) ) {
      std::printf( "two-rank serial read: %s missing (the write step did not run), skipped\n", ( dir / "two_ranks.peb" ).string().c_str() );
      return 77;
   }
   WorldID world = theWorld();
   particleMaterial();   // the recorded material table is reinstated by readCheckpoint(); the name must exist
   int failures( 0 );
   try {
      const CheckpointMetadata meta = readCheckpoint( dir, "two_ranks" );
      if( !meta.present )                           { std::printf( "FAIL: sidecar missing\n" ); ++failures; }
      if( meta.bodyCount != ranks * spheresPerRank ) { std::printf( "FAIL: sidecar bodyCount %u\n", static_cast<unsigned>( meta.bodyCount ) ); ++failures; }
      unsigned found( 0 );
      for( int rank = 0; rank < ranks; ++rank )
         for( unsigned i = 0; i < spheresPerRank; ++i )
            for( World::Bodies::CastIterator<Sphere> s = world->begin<Sphere>(); s != world->end<Sphere>(); ++s )
               if( std::fabs( static_cast<double>( s->getPosition()[0] - expectedX( rank, i ) ) ) < 1e-9 ) { ++found; break; }
      std::printf( "two-rank serial read: %d bodies in the world, %u of %u expected sphere positions found\n",
                   static_cast<int>( world->size() ), found, ranks * spheresPerRank );
      if( world->size() != ranks * spheresPerRank || found != ranks * spheresPerRank ) ++failures;
   }
   catch( const std::exception& e ) {
      std::printf( "FAIL: readCheckpoint threw: %s\n", e.what() );
      ++failures;
   }
   std::printf( "pe_checkpoint_parallel_file_serial_read: %s\n", failures == 0 ? "all checks passed" : "FAILED" );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main( int argc, char** argv )
{
   if( argc == 3 && std::string( argv[1] ) == "--write" ) return write( argc, argv, fs::path( argv[2] ) );
   if( argc == 3 && std::string( argv[1] ) == "--read" )  return read( fs::path( argv[2] ) );
   std::fprintf( stderr, "usage: %s --write <dir> (under mpiexec -n 2) | --read <dir>\n", argv[0] );
   return EXIT_FAILURE;
}
