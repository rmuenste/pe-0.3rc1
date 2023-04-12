#include <pe/util/Checkpointer.h>

namespace pe {

//*************************************************************************************************
void Checkpointer::read( std::string name ) {
   bbreader_.readFile( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
}
//*************************************************************************************************



//*************************************************************************************************
void Checkpointer::trigger()
{
  // Skipping the visualization for intermediate time steps
  if(( ++steps_ < tspacing_ ) && (!counter_ == 0)) return;

  // Adjusing the counters
  steps_ = 0;
  std::ostringstream bodyFile;
  bodyFile << "checkpoint." << counter_;
  setPath( "checkpoints/" );
  size_t               mt( 0 );
  write(bodyFile.str());
  ++counter_;
}
//*************************************************************************************************



//*************************************************************************************************
void Checkpointer::write( std::string name ) {
   boost::filesystem::create_directories( checkpointsPath_ );
   bbwriter_.writeFileAsync( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
   pe_PROFILING_SECTION {
      timing::WcTimer timeWait;
      timeWait.start();
      bbwriter_.wait();
      timeWait.end();
#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
      pe_LOG_INFO_SECTION( log ) {
         log << "BodyBinaryWriter::wait() took " << timeWait.total() << "s on rank " << MPISettings::rank() << ".\n";
      }
   }

}
//*************************************************************************************************

} // namespace pe



