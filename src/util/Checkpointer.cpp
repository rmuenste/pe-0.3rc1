#include <pe/util/Checkpointer.h>

#include <pe/core/MPISettings.h>
#include <pe/core/MPITrait.h>
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
      // Deliberately std::cerr rather than pe_LOG_WARNING_SECTION: this warning says that the
      // integrity and pairing guarantees do not hold for this run, and it must reach the operator
      // even when logging is off or below warning level, which is the default in coupled runs.
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
/*!\brief Path the previous sidecar is preserved under while the new one is assembled.
 *
 * Must NOT be checkpointTempPath( sidecarPath, ... ): writeCheckpointMetadata() builds the new
 * sidecar under exactly that name, and on the writing rank the tokens coincide, so sharing the
 * marker would truncate the very copy being preserved.
 */
boost::filesystem::path preservedSidecarPath( const boost::filesystem::path& sidecarPath,
                                              long token )
{
   std::ostringstream name;
   name << sidecarPath.filename().string() << ".prev-" << token;
   return sidecarPath.parent_path() / name.str();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Total body records across all ranks. COLLECTIVE: every rank must call it.
 *
 * BodyBinaryWriter counts what the calling rank marshalled. The sidecar describes the whole
 * file, so the per-rank counts are summed onto the rank that writes it.
 *
 * \return The total, valid on rank 0 only -- MPI_Reduce leaves every other rank's buffer
 *         untouched, so the value other ranks see is their own count. That is sufficient here
 *         because only rank 0 writes the sidecar, and it is why the caller must not use the
 *         result for anything else.
 */
uint64_t totalMarshalledBodyCount( const BodyBinaryWriter& writer )
{
   size_t count = writer.getMarshalledBodyCount();

#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      size_t local = count;
      MPI_Reduce( &local, &count, 1, MPITrait<size_t>::getType(), MPI_SUM, 0, MPISettings::comm() );
   }
#endif

   return static_cast<uint64_t>( count );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes the world to `<basePath>.peb` plus its sidecar, publishing both atomically.
 *
 * Publication order is: move any stale sidecar aside under a `.prev-` marker, write the `.peb`
 * under a scratch name, rename it into place, write the new sidecar, then drop the preserved one.
 * Every intermediate state is either the previous checkpoint or a sidecar-less checkpoint, and a
 * sidecar-less checkpoint is reported loudly on read -- a mismatched (`.peb`, sidecar) pair is
 * never published.
 *
 * The preserved copy survives a kill throughout the `.peb` write, which is the long part. It does
 * not survive a kill during the sidecar write itself: that is a truncate-and-rewrite of the
 * sidecar path with no third copy to fall back on. The outcome there is a sidecar-less checkpoint,
 * which is loud, not a wrong pairing.
 *
 * The stale sidecar has to go before the new `.peb` appears rather than after: checkpoint names
 * are reused across runs (Checkpointer's counter restarts at zero), and a steady-state `.peb`
 * can be byte-identical to its predecessor, so pebBytes alone cannot detect a leftover sidecar
 * describing a different instant.
 *
 * COLLECTIVE. Every rank must call this: the `.peb` write, the scratch token and the body count
 * reduction are all collective operations.
 */
void storeCheckpoint( const boost::filesystem::path& pebPath,
                      const boost::filesystem::path& sidecarPath,
                      BodyBinaryWriter& writer )
{
   boost::filesystem::create_directories( pebPath.parent_path() );

   // Collective: MPI_File_open below requires an identical filename on every rank.
   const long token = collectiveCheckpointScratchToken();
   const boost::filesystem::path staleSidecar = preservedSidecarPath( sidecarPath, token );

   if( isCheckpointFileOwner() && boost::filesystem::exists( sidecarPath ) ) {
      boost::system::error_code ignored;
      boost::filesystem::rename( sidecarPath, staleSidecar, ignored );
   }
   synchronizeCheckpointWriters();

   // writeFile() already waits for the asynchronous chunk writes to complete.
   const boost::filesystem::path pebTemp = checkpointTempPath( pebPath, token );
   writer.writeFile( pebTemp.string().c_str() );
   synchronizeCheckpointWriters();

   const uint64_t bodyCount = totalMarshalledBodyCount( writer );

   if( !isCheckpointFileOwner() )
      return;

   CheckpointMetadata metadata = currentCheckpointMetadata();
   metadata.pebBytes  = static_cast<uint64_t>( boost::filesystem::file_size( pebTemp ) );
   metadata.bodyCount = bodyCount;

   commitCheckpointTempFile( pebTemp, pebPath );
   writeCheckpointMetadata( sidecarPath, metadata );

   boost::system::error_code ignored;
   boost::filesystem::remove( staleSidecar, ignored );
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
  // Skip intermediate time steps. The counter_ term forces the very first trigger to write
  // regardless of spacing, so a run always has a checkpoint at its start; it was previously
  // spelled `!counter_ == 0`, which parses as `(!counter_) == 0` and happens to mean the same
  // thing by accident.
  if(( ++steps_ < tspacing_ ) && ( counter_ != 0 )) {
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
