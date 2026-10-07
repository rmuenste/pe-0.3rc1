//=================================================================================================
/*!
 *  \file src/core/BodyBinaryWriter.cpp
 *  \brief Writer for rigid body binary parameter files
 *
 *  Copyright (C) 2011-2012 Tobias Preclik
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/BodyBinaryWriter.h>
#include <pe/core/Marshalling.h>
#include <pe/core/MPITrait.h>
#include <pe/util/CheckpointCollective.h>
#include <pe/core/rigidbody/Ellipsoid.h>


namespace pe {

//*************************************************************************************************
/*!\brief Writes out a binary rigid body parameter file and returns immediately.
 * \param filename The filename of the parameter file to write to.
 * \return void
 */
void BodyBinaryWriter::writeFileAsync( const char* filename ) {
   timing::WcTimer timeAll, timeWait, timeExscan, timeOpen, timeGather, timeWrite;
   size_t sizeWrite( 0 ), bodies( 0 ), sizeWriteAll( 0 );

   using boost::numeric_cast;
#if HAVE_MPI
   const bool parallel = MPISettings::isParallel();
#endif

   pe_PROFILING_SECTION {
      timeAll.start();
      timeWait.start();
   }

   // wait until we can reuse buffers
   wait();

   pe_PROFILING_SECTION {
      timeWait.end();
   }

   size_t localSize = 0, headerSize = 0;
   std::vector<size_t> offsets;
   checkpoint_detail::phase( filename, [&] {
      filename_ = filename;
#if HAVE_MPI
      requests_.clear();
#endif
      buffer_.clear();
      header_.clear();
      globals_.clear();

      buffer_.setFloatingPointSize( fpSize_ );
      header_.setFloatingPointSize( fpSize_ );
      globals_.setFloatingPointSize( fpSize_ );

      ConstWorldID world = theWorld();

      marshal( buffer_, UniqueID<RigidBody>::counter_ );
      bodies += marshalAllPrimitives<Sphere>      ( buffer_, world );
      bodies += marshalAllPrimitives<Ellipsoid>   ( buffer_, world );
      bodies += marshalAllPrimitives<Box>         ( buffer_, world );
      bodies += marshalAllPrimitives<Capsule>     ( buffer_, world );
      bodies += marshalAllPrimitives<Cylinder>    ( buffer_, world );
      //bodies += marshalAllPrimitives<Plane>       ( buffer_, world );
      bodies += marshalAllPrimitives<TriangleMesh>( buffer_, world );
      bodies += marshalAllPrimitives<Union>       ( buffer_, world );

      localSize = buffer_.size();

      pe_EXCLUSIVE_SECTION( 0 ) {
         // marshal global bodies
         marshal( globals_, UniqueID<RigidBody>::globalCounter_ );
         bodies += marshalAllPrimitives<Sphere>      ( globals_, world, true );
         bodies += marshalAllPrimitives<Ellipsoid>   ( globals_, world, true );
         bodies += marshalAllPrimitives<Box>         ( globals_, world, true );
         bodies += marshalAllPrimitives<Capsule>     ( globals_, world, true );
         bodies += marshalAllPrimitives<Cylinder>    ( globals_, world, true );
         bodies += marshalAllPrimitives<Plane>       ( globals_, world, true );
         bodies += marshalAllPrimitives<TriangleMesh>( globals_, world, true );
         bodies += marshalAllPrimitives<Union>       ( globals_, world, true );

         // write header
         const byte fileFormatVersionMajor( 0 );
         const byte fileFormatVersionMinor( 2 );
         header_ << static_cast<byte>('P') << static_cast<byte>('E') << fileFormatVersionMajor << fileFormatVersionMinor;

         // table of data type sizes
         if( fpSize_ == 0 )
            header_ << static_cast<byte>( sizeof(real) );
         else
            header_ << static_cast<byte>( 1 << fpSize_ );
         header_ << static_cast<byte>( sizeof(int) )
                 << static_cast<byte>( sizeof(size_t) )
                 << static_cast<byte>( sizeof(id_t) )
                 << static_cast<byte>( sizeof(bool) );

         header_ << numeric_cast<uint32_t>( MPISettings::size() );

         headerSize = header_.size() + ( MPISettings::size() + 2 ) * sizeof( uint32_t );
         localSize = headerSize + globals_.size() + buffer_.size();
      }

      // Captured before the profiling reduction below, which would otherwise turn this rank's count
      // into a global sum only when profiling happens to be enabled. Stays per-rank by design; see
      // getMarshalledBodyCount().
      bodies_ = bodies;
      // Allocate all MPI request slots before opening the collective file. Failed submissions
      // leave MPI_REQUEST_NULL slots, which can safely be skipped during collective cleanup.
#if HAVE_MPI
      if( parallel ) {
         requests_.push_back( PendingWrite{ MPI_REQUEST_NULL, numeric_cast<int>(buffer_.size()) } );
         if( MPISettings::rank() == 0 ) {
            requests_.push_back( PendingWrite{ MPI_REQUEST_NULL, 0 } );
            requests_.push_back( PendingWrite{ MPI_REQUEST_NULL, numeric_cast<int>(globals_.size()) } );
         }
      }
#endif
      if( MPISettings::rank() == 0 ) offsets.resize( MPISettings::size() );
   });

   // determine offset of chunk for local body descriptions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " size of local bodies chunk is " << localSize << "\n";
   }

   pe_PROFILING_SECTION {
      timeExscan.start();
   }

   size_t offset = 0;
   checkpoint_detail::phase( filename, [&] {
#if HAVE_MPI
      if( parallel )
         checkpoint_detail::mpiCheck( MPI_Exscan( &localSize, &offset, 1, MPITrait<size_t>::getType(), MPI_SUM, MPISettings::comm() ), "MPI_Exscan" );
      else
#endif
         offset = headerSize + globals_.size();   // the only chunk follows the global bodies
   });

   pe_PROFILING_SECTION {
      timeExscan.end();
   }

   pe_EXCLUSIVE_SECTION( 0 ) {
      offset = headerSize + globals_.size();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " offset of local bodies chunk is " << offset << "\n";
   }

   const size_t end = offset + buffer_.size();
   pe_PROFILING_SECTION { timeGather.start(); }
   checkpoint_detail::phase( filename, [&] {
#if HAVE_MPI
      if( parallel )
         checkpoint_detail::mpiCheck( MPI_Gather( &end, 1, MPITrait<size_t>::getType(),
            MPISettings::rank() == 0 ? offsets.data() : 0, 1, MPITrait<size_t>::getType(),
            0, MPISettings::comm() ), "MPI_Gather" );
      else
#endif
         offsets[0] = end;
   });
   pe_PROFILING_SECTION { timeGather.end(); }
   checkpoint_detail::phase( filename, [&] {
      // The file format uses 32-bit offsets; reject overflow on every rank before opening.
      numeric_cast<uint32_t>( end );
      if( MPISettings::rank() == 0 ) {
         header_ << numeric_cast<uint32_t>( headerSize ) << numeric_cast<uint32_t>( offset );
         for( size_t i = 0; i < offsets.size(); ++i ) header_ << numeric_cast<uint32_t>( offsets[i] );
#if HAVE_MPI
         if( parallel ) {
            auto it = requests_.begin();
            ++it;
            it->bytes = numeric_cast<int>(header_.size());
         }
#endif
         pe_PROFILING_SECTION {
            sizeWriteAll = offsets.back();
         }
      }
   });

   try {
      pe_PROFILING_SECTION { timeOpen.start(); }
      checkpoint_detail::phase( filename, [&] {
#if HAVE_MPI
         fhParallel_ = parallel;
         if( parallel ) {
            checkpoint_detail::mpiCheck( MPI_File_open( MPISettings::comm(), &filename_[0],
               MPI_MODE_WRONLY | MPI_MODE_CREATE, MPI_INFO_NULL, &fh_ ), "MPI_File_open" );
            fhOpen_ = true;
         }
         else
#endif
         {
            sfh_.clear();
            sfh_.open( filename_.c_str(), std::ofstream::binary );
            if( !sfh_ ) throw std::runtime_error( "Cannot open file." );
            fhOpen_ = true;
         }
      });
#if HAVE_MPI
      // File error handlers return errors rather than terminating the communicator.
      checkpoint_detail::phase( filename, [&] {
         if( parallel ) checkpoint_detail::mpiCheck(
            MPI_File_set_errhandler( fh_, MPI_ERRORS_RETURN ), "MPI_File_set_errhandler" );
      });
#endif
      pe_PROFILING_SECTION { timeOpen.end(); timeWrite.start(); }
      checkpoint_detail::phase( filename, [&] {
#if HAVE_MPI
         if( parallel ) {
            auto it = requests_.begin();
            auto submit = [&]( MPI_Offset at, const void* data, const char* label ) {
               MPI_Request request = MPI_REQUEST_NULL;
               checkpoint_detail::mpiCheck( MPI_File_iwrite_at( fh_, at, data,
                  it->bytes, MPI_BYTE, &request ), label );
               // An unsuccessful MPI call need not leave a valid output request.
               it->request = request;
               ++it;
            };
            submit( offset, buffer_.ptr(), "MPI_File_iwrite_at (local bodies)" );
            if( MPISettings::rank() == 0 ) {
               submit( 0, header_.ptr(), "MPI_File_iwrite_at (header)" );
               submit( headerSize, globals_.ptr(), "MPI_File_iwrite_at (global bodies)" );
            }
         }
         else
#endif
         {
            sfh_.seekp( offset );
            sfh_.write( reinterpret_cast<const char*>(buffer_.ptr()), buffer_.size() );
            sfh_.seekp( 0 );
            sfh_.write( reinterpret_cast<const char*>(header_.ptr()), header_.size() );
            sfh_.seekp( headerSize );
            sfh_.write( reinterpret_cast<const char*>(globals_.ptr()), globals_.size() );
            if( !sfh_ ) throw std::runtime_error( "Failed while writing rigid body parameter file." );
         }
      });
   }
   catch( ... ) {
      const std::exception_ptr error = std::current_exception();
      try { wait(); } catch( ... ) {}
#if HAVE_MPI
      requests_.clear();
#endif
      std::rethrow_exception( error );
   }
   pe_PROFILING_SECTION {
      sizeWrite = buffer_.size();
      if( MPISettings::rank() == 0 ) sizeWrite += header_.size() + globals_.size();
   }

   pe_PROFILING_SECTION {
      timeWrite.end();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " rigid body counter is " << UniqueID<RigidBody>::counter_ << "\n";
      log << "On rank " << MPISettings::rank() << " marshaled " << bodies << " rigid bodies out of " << theWorld()->size() << " in the world\n";
   }

   pe_PROFILING_SECTION {
      timeAll.end();
      if( logging::loglevel >= logging::info ) {
         std::vector<timing::WcTimer*> timers;
         timers.push_back( &timeAll );
         timers.push_back( &timeWait );
         timers.push_back( &timeExscan );
         timers.push_back( &timeOpen );
         timers.push_back( &timeWrite );
         timers.push_back( &timeGather );

         // Store the minimum time measurement of each timer over all time steps
         std::vector<double> minValues( timers.size() );
         for( std::size_t i = 0; i < minValues.size(); ++i )
            minValues[i] = timers[i]->min();

         // Store the maximum time measurement of each timer over all time steps
         std::vector<double> maxValues( timers.size() );
         for( std::size_t i = 0; i < maxValues.size(); ++i )
            maxValues[i] = timers[i]->max();

         // Store the total time measured of each timer over all time steps
         std::vector<double> totalValues( timers.size() );
         for( std::size_t i = 0; i < totalValues.size(); ++i )
            totalValues[i] = timers[i]->total();

         // Store the total number of measurements of each timer over all time steps
         std::vector<size_t> numValues( timers.size() );
         for( std::size_t i = 0; i < numValues.size(); ++i )
            numValues[i] = timers[i]->getCounter();

         pe_LOG_INFO_SECTION( log ) {
            log << "Timing results of BodyBinaryWriter::writeFileAsync() on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min time     max time     avg time     total time   executions   bytes\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "total                  "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << sizeWrite << "\n"
                << "  wait                 "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                << "  exscan               "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << 0 << "\n"
                << "  open                 "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << 0 << "\n"
                << "  write                "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "   " << std::setw(10) << sizeWrite << "\n"
                << "    gather             "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "   " << std::setw(10) << 0 << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "Number of bodies marshaled on current process: " << bodies << " (out of " << theWorld()->size() << " in the world)\n";
         }

         // Logging the profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total time measurement and number of measurements of each timer over all time steps and all ranks
            if( rank == 0 ) {
               MPI_Reduce( MPI_IN_PLACE, &minValues[0],   static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxValues[0],   static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalValues[0], static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &numValues[0],   static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &bodies,         1,                                      MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
            }
            else {
               MPI_Reduce( &minValues[0],   0, static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, 0, comm );
               MPI_Reduce( &maxValues[0],   0, static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, 0, comm );
               MPI_Reduce( &totalValues[0], 0, static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( &numValues[0],   0, static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( &bodies,         0, 1,                                      MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
            }


            pe_EXCLUSIVE_SECTION( 0 ) {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Timing results of BodyBinaryWriter::writeFileAsync() reduced over all ranks:\n" << std::fixed << std::setprecision(4)
                      << "code part              min time     max time     avg time     total time   executions   bytes\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                      << "total:                 "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << sizeWriteAll << "\n"
                      << "  wait                 "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                      << "  exscan               "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << 0 << "\n"
                      << "  open                 "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << 0 << "\n"
                      << "  write                "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "   " << std::setw(10) << sizeWriteAll << "\n"
                      << "    gather             "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "   " << std::setw(10) << 0 << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                      << "Number of bodies marshaled on all processes: " << bodies << "\n";
               }
            }
         }
      }
   }
}
//*************************************************************************************************

// Draining requests and closing the MPI file must finish on every rank even if a local write
// failed. Only then may agreement throw, so destructors cannot strand peers in MPI_File_close.
void BodyBinaryWriter::wait()
{
   if( !fhOpen_ ) return;
   std::exception_ptr error;
   auto record = [&]( auto operation ) {
      try { operation(); }
      catch( ... ) { if( !error ) error = std::current_exception(); }
   };
#if HAVE_MPI
   if( fhParallel_ ) {
      while( !requests_.empty() ) {
         PendingWrite& pending = requests_.front();
         if( pending.request != MPI_REQUEST_NULL ) record( [&] {
            MPI_Status status;
            checkpoint_detail::requireTransfer( MPI_Wait( &pending.request, &status ), status, pending.bytes, "MPI_Wait (file write)" );
         });
         requests_.pop_front();
      }
      record( [&] { checkpoint_detail::mpiCheck( MPI_File_close( &fh_ ), "MPI_File_close" ); } );
   }
   else
#endif
   {
      record( [&] {
         sfh_.flush();
         const bool failed = !sfh_;
         sfh_.close();
         if( failed || !sfh_ ) throw std::runtime_error( "Failed while closing rigid body parameter file." );
      });
   }
   fhOpen_ = false;
   checkpoint_detail::phase( filename_.c_str(), [&] { if( error ) std::rethrow_exception(error); } );
}

} // namespace pe
