#include <pe/util/Checkpointer.h>

#include <pe/core/MPISettings.h>
#include <pe/core/World.h>

#include <sstream>
#include <stdexcept>

namespace pe {

//*************************************************************************************************
// Static member definitions
//*************************************************************************************************
bool Checkpointer::active_ = false;
boost::mutex Checkpointer::instanceMutex_;


namespace {

//*************************************************************************************************
/*!\brief Rank that owns the single-writer parts of a checkpoint write.
 *
 * The `.peb` body chunks are written collectively by all ranks, but the sidecar and the atomic
 * renames are whole-file operations and must be performed exactly once.
 */
bool isCheckpointFileOwner()
{
   return MPISettings::rank() == 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Blocks until every rank has finished its part of the collective `.peb` write.
 */
void synchronizeCheckpointWriters()
{
#if HAVE_MPI
   if( MPISettings::size() > 1 )
      MPI_Barrier( MPISettings::comm() );
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Cross-checks the `.peb` on disk against what the sidecar says it should be.
 *
 * BodyBinaryReader documents that an invalid file yields assertions in debug and undefined
 * behaviour in release, so a truncated checkpoint must be rejected before it is opened.
 */
void requirePebIntact( const boost::filesystem::path& pebPath, const CheckpointMetadata& metadata )
{
   if( !boost::filesystem::exists( pebPath ) )
      throw std::runtime_error( "Checkpoint '" + pebPath.string() + "' does not exist, although "
                                "its metadata sidecar does. The pair is incomplete." );

   const uint64_t actualBytes = static_cast<uint64_t>( boost::filesystem::file_size( pebPath ) );
   if( actualBytes != metadata.pebBytes ) {
      std::ostringstream message;
      message << "Checkpoint '" << pebPath.string() << "' is " << actualBytes
              << " bytes, but its metadata sidecar records " << metadata.pebBytes
              << " bytes. The file was truncated by a killed job, or the sidecar belongs to a "
                 "different checkpoint. Refusing to resume from it.";
      throw std::runtime_error( message.str() );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Loads \a pebPath after restoring the recorded material table.
 *
 * \return The metadata that was used, so callers can validate their own expectations against it.
 */
CheckpointMetadata loadCheckpoint( const boost::filesystem::path& pebPath,
                                   const boost::filesystem::path& sidecarPath,
                                   BodyBinaryReader& reader )
{
   const CheckpointMetadata metadata = readCheckpointMetadata( sidecarPath );

   if( metadata.present ) {
      requirePebIntact( pebPath, metadata );
      restoreCheckpointMaterials( metadata, pebPath.string() );
   }
   else {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cerr << "WARNING: checkpoint '" << pebPath.string() << "' has no metadata sidecar ("
                   << sidecarPath.filename().string() << "). It was written by a pe build without "
                      "checkpoint metadata, so its simulation time, step index and material table "
                      "are unknown: it cannot be paired with driver state, truncation cannot be "
                      "detected, and body material indices are only valid if this process "
                      "registers exactly the same materials, in the same order, before the "
                      "checkpoint is read." << std::endl;
      }
   }

   reader.readFile( pebPath.string().c_str() );

   return metadata;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes the world to `<basePath>.peb` plus its sidecar, publishing both atomically.
 *
 * Publication order is: drop any stale sidecar, write the `.peb` under a scratch name, rename it
 * into place, then write the sidecar. Every intermediate state is therefore either the previous
 * checkpoint or a sidecar-less checkpoint, and a sidecar-less checkpoint is reported loudly on
 * read. A mismatched (`.peb`, sidecar) pair is never published.
 */
void storeCheckpoint( const boost::filesystem::path& pebPath,
                      const boost::filesystem::path& sidecarPath,
                      BodyBinaryWriter& writer )
{
   boost::filesystem::create_directories( pebPath.parent_path() );

   if( isCheckpointFileOwner() ) {
      boost::system::error_code ignored;
      boost::filesystem::remove( sidecarPath, ignored );
   }
   synchronizeCheckpointWriters();

   const boost::filesystem::path pebTemp = checkpointTempPath( pebPath );
   writer.writeFile( pebTemp.string().c_str() );
   writer.wait();
   synchronizeCheckpointWriters();

   if( !isCheckpointFileOwner() )
      return;

   CheckpointMetadata metadata = currentCheckpointMetadata();
   metadata.pebBytes  = static_cast<uint64_t>( boost::filesystem::file_size( pebTemp ) );
   metadata.bodyCount = static_cast<uint64_t>( writer.getMarshalledBodyCount() );

   commitCheckpointTempFile( pebTemp, pebPath );
   writeCheckpointMetadata( sidecarPath, metadata );
}
//*************************************************************************************************

} // namespace


//*************************************************************************************************
CheckpointMetadata Checkpointer::read( const std::string &name ) {
   return loadCheckpoint( checkpointsPath_ / ( name + ".peb" ),
                          checkpointMetadataPath( checkpointsPath_, name ),
                          bbreader_ );
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
}
//*************************************************************************************************



//*************************************************************************************************
void Checkpointer::write( const std::string &name ) {
   storeCheckpoint( checkpointsPath_ / ( name + ".peb" ),
                    checkpointMetadataPath( checkpointsPath_, name ),
                    bbwriter_ );
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

//*************************************************************************************************
CheckpointMetadata readCheckpoint(const path& checkpointsPath, const std::string& name)
{
   BodyBinaryReader reader;
   return loadCheckpoint( checkpointsPath / ( name + ".peb" ),
                          checkpointMetadataPath( checkpointsPath, name ),
                          reader );
}
//*************************************************************************************************

//*************************************************************************************************
void writeCheckpoint(const path& checkpointsPath, const std::string& name)
{
   BodyBinaryWriter writer;
   storeCheckpoint( checkpointsPath / ( name + ".peb" ),
                    checkpointMetadataPath( checkpointsPath, name ),
                    writer );
}
//*************************************************************************************************

} // namespace pe
