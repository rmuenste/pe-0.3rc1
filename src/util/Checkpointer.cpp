#include <pe/util/Checkpointer.h>

namespace pe {

//*************************************************************************************************
// Static member definitions
//*************************************************************************************************
bool Checkpointer::active_ = false;
boost::mutex Checkpointer::instanceMutex_;


//*************************************************************************************************
void Checkpointer::read( const std::string &name ) {
   bbreader_.readFile( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
}
//*************************************************************************************************

Checkpointer::~Checkpointer() {
}


//*************************************************************************************************
void Checkpointer::trigger()
{
  // Skipping the visualization for intermediate time steps
  if(( ++steps_ < tspacing_ ) && (!counter_ == 0)) {
  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "Checkpoint in:" << tspacing_ - steps_ << std::endl;
  }
    return;
  }

  // Adjusting the counters
  steps_ = 0;
  std::ostringstream bodyFile;
  bodyFile << "checkpoint." << counter_;
  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "Checkpoint:" << bodyFile.str() << std::endl;
  }
  write(bodyFile.str());
  ++counter_;

  // Flush so the checkpoint is self-contained for auto-triggering
  flush();
}
//*************************************************************************************************



//*************************************************************************************************
void Checkpointer::write( const std::string &name ) {
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


//*************************************************************************************************
CheckpointerID activateCheckpointer(const path& checkpointsPath,
                                     unsigned int spacing,
                                     unsigned int start,
                                     unsigned int end)
{
   boost::mutex::scoped_lock lock( Checkpointer::instanceMutex_ );
   static CheckpointerID cp( new Checkpointer(checkpointsPath, spacing, start, end) );
   Checkpointer::active_ = true;
   return cp;
}
//*************************************************************************************************

} // namespace pe
