#include <pe/interface/frozen_field_trace.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <mpi.h>

namespace {

const char* cubeOff()
{
   return
      "OFF\n"
      "8 12 0\n"
      "0 0 0\n"
      "1 0 0\n"
      "1 1 0\n"
      "0 1 0\n"
      "0 0 1\n"
      "1 0 1\n"
      "1 1 1\n"
      "0 1 1\n"
      "3 0 2 1\n"
      "3 0 3 2\n"
      "3 4 5 6\n"
      "3 4 6 7\n"
      "3 0 4 7\n"
      "3 0 7 3\n"
      "3 1 2 6\n"
      "3 1 6 5\n"
      "3 0 1 5\n"
      "3 0 5 4\n"
      "3 3 7 6\n"
      "3 3 6 2\n";
}

bool near( double lhs, double rhs )
{
   return std::abs( lhs - rhs ) < 1.0e-8;
}

} // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );

   int rank = 0;
   int size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 ) {
      if( rank == 0 )
         std::cerr << "test requires exactly two MPI ranks\n";
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }
   if( argc != 3 ) {
      if( rank == 0 )
         std::cerr << "usage: pe_frozen_field_trace_mpi <off|obj> <obj-path>\n";
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }

   pe::SurfaceMeshInput surface;
   if( std::string( argv[1] ) == "off" )
      surface = pe::OffMeshData{ cubeOff() };
   else
      surface = pe::ObjMeshFile{ argv[2] };

   std::vector<std::array<double, 3> > seeds;
   if( rank == 0 )
      seeds.push_back( {{ 0.25, 0.25, 0.25 }} );
   else
      seeds.push_back( {{ 0.75, 0.75, 0.75 }} );

   int callbackCalls = 0;
   const auto callback = [&callbackCalls]( const double* vertices, double* velocities,
                                           std::size_t count ) -> int {
      ++callbackCalls;
      for( std::size_t i = 0; i < count; ++i ) {
         velocities[3 * i] = vertices[3 * i + 1] < 0.5 ? 1.0 : 0.0;
         velocities[3 * i + 1] = 0.0;
         velocities[3 * i + 2] = 0.0;
      }
      return 0;
   };

   const pe::FrozenFieldTraceResult result =
      pe::runFrozenFieldTrace( seeds, surface, callback );

   bool valid = callbackCalls == 6;
   if( rank == 0 ) {
      valid = valid && result.exits.empty() && result.survivors.empty();
   }
   else {
      valid = valid && result.exits.size() == 1 && result.survivors.size() == 1;
      if( result.exits.size() == 1 ) {
         const pe::TracerExit& exit = result.exits.front();
         valid = valid &&
            exit.tracer.originRank == 0 &&
            exit.tracer.originLocalIndex == 0 &&
            exit.exitRank == 1 &&
            exit.timestep == 4 &&
            exit.face == pe::CubeFace::XMax &&
            near( exit.time, 0.75 ) &&
            near( exit.position[0], 1.0 ) &&
            near( exit.position[1], 0.25 ) &&
            near( exit.position[2], 0.25 ) &&
            near( exit.velocity[0], 1.0 );
      }
      if( result.survivors.size() == 1 ) {
         const pe::TracerSurvivor& survivor = result.survivors.front();
         valid = valid &&
            survivor.tracer.originRank == 1 &&
            survivor.ownerRank == 1 &&
            near( survivor.position[0], 0.75 ) &&
            near( survivor.position[1], 0.75 ) &&
            near( survivor.position[2], 0.75 );
      }
   }

   const int localFailure = valid ? 0 : 1;
   int failures = 0;
   MPI_Allreduce( &localFailure, &failures, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( failures && rank == 0 )
      std::cerr << "frozen-field trace result validation failed\n";

   MPI_Finalize();
   return failures == 0 ? 0 : 1;
}
