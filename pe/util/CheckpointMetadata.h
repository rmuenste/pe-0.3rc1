//=================================================================================================
/*!
 *  \file pe/util/CheckpointMetadata.h
 *  \brief Identity metadata for rigid body checkpoints
 *
 *  A `.peb` checkpoint (see BodyBinaryWriter) stores body state only: it carries no simulation
 *  time, no step index, no material table and no length information. That makes a checkpoint
 *  impossible to pair with the driver state (e.g. a FeatFloWer `_dump`) it belongs to, and makes
 *  a truncated checkpoint indistinguishable from a complete one.
 *
 *  This module adds a human-readable sidecar file `<name>.peinfo` next to `<name>.peb`. The
 *  sidecar is additive: the `.peb` format is untouched, so checkpoints written by this version
 *  remain readable by older builds, and checkpoints written by older builds ("legacy
 *  checkpoints") remain readable here -- with a loud warning, because none of the guarantees
 *  below can be enforced for them.
 */
//=================================================================================================

#ifndef _PE_UTIL_CHECKPOINTMETADATA_H_
#define _PE_UTIL_CHECKPOINTMETADATA_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/filesystem.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  SIDECAR FORMAT CONSTANTS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Highest sidecar layout this build can parse.
 *
 * A sidecar declaring a higher version is rejected rather than partially interpreted: a future
 * version may reinterpret an existing key, so "ignore what you do not know" is not safe here.
 */
const unsigned int checkpointMetadataVersion = 1u;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Filename extension of the metadata sidecar accompanying `<name>.peb`.
 */
extern const char* const checkpointMetadataExtension;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default width of the resume time window, in units of the checkpoint's step size.
 *
 * A driver and pe accumulate simulation time by repeated addition of the same dt, so their
 * values agree to within rounding, never exactly. Half a step is the widest window that still
 * identifies a unique step -- which is precisely what the pairing check must establish.
 */
const real defaultResumeTimeToleranceSteps = real( 0.5 );
//*************************************************************************************************


//=================================================================================================
//
//  CLASS DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief One entry of the material table a checkpoint was written with.
 *
 * Bodies serialise their material as a bare index into the process-global material table
 * (see Marshalling.h, `buffer << obj.getMaterial()`). The index is only meaningful together with
 * the table that produced it, so the table travels with the checkpoint.
 */
struct CheckpointMaterialRecord
{
   std::string name;       //!< Material name; unique within the table, may contain spaces.
   real density;
   real cor;               //!< Coefficient of restitution.
   real csf;               //!< Coefficient of static friction.
   real cdf;               //!< Coefficient of dynamic friction.
   real poisson;
   real young;
   real stiffness;
   real dampingN;
   real dampingT;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Origin of the time/step pair stored in a checkpoint.
 */
enum CheckpointTimeSource
{
   checkpointTimeFromDriver,    //!< Supplied by the driver via setCheckpointIdentity().
   checkpointTimeFromTimeStep   //!< Derived from pe's own TimeStep counter.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Identity of a checkpoint: everything needed to pair it with driver state.
 */
struct CheckpointMetadata
{
   CheckpointMetadata();

   bool present;                  //!< \a false for a legacy checkpoint that has no sidecar.
   unsigned int version;          //!< Sidecar layout version the file declared.
   CheckpointTimeSource timeSource;
   real simulationTime;
   uint64_t timeStep;
   real stepSize;                 //!< TimeStep::size() at write time.
   uint64_t bodyCount;            //!< Body records the `.peb` contains (not the world size).
   uint64_t pebBytes;             //!< Exact size of the companion `.peb`; detects truncation.
   std::string pairingTag;        //!< Free-form driver tag, e.g. the CFD step. May be empty.
   std::vector<CheckpointMaterialRecord> materials;  //!< Full table, in index order.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Expectation a driver places on the checkpoint it is about to resume from.
 *
 * Every field is opt-in. An unset field is not checked; a set field that cannot be checked
 * (legacy checkpoint) is an error rather than a silent pass -- an expectation that is quietly
 * dropped is worse than no expectation at all.
 */
struct ResumeExpectation
{
   ResumeExpectation();

   bool hasTime;
   real time;
   real toleranceSteps;   //!< Allowed |time| deviation, in units of the checkpoint's stepSize.

   bool hasStep;
   uint64_t step;

   bool hasTag;
   std::string tag;

   bool any() const { return hasTime || hasStep || hasTag; }
};
//*************************************************************************************************


//=================================================================================================
//
//  SIDECAR I/O
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the sidecar path belonging to `<checkpointsPath>/<name>.peb`.
 */
boost::filesystem::path checkpointMetadataPath( const boost::filesystem::path& checkpointsPath,
                                                const std::string& name );
//*************************************************************************************************


//=================================================================================================
//
//  ATOMIC PUBLICATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the scratch path a checkpoint file is assembled under before publication.
 *
 * \param token Disambiguates concurrent writers to one directory. It is a parameter rather than
 *        something this function samples itself because a file written collectively must be
 *        opened under an identical name on every rank -- see collectiveCheckpointScratchToken().
 */
boost::filesystem::path checkpointTempPath( const boost::filesystem::path& finalPath, long token );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scratch token for a file written by one rank on its own.
 */
long localCheckpointScratchToken();
//*************************************************************************************************


//*************************************************************************************************
/*!\brief COLLECTIVE. Scratch token agreed by every rank, for a collectively written file.
 *
 * MPI_File_open is collective and requires the same filename on all ranks, so a per-rank pid in
 * the scratch name is an error, not merely untidy. Every rank must call this.
 */
long collectiveCheckpointScratchToken();
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Moves a fully written scratch file onto its final path.
 *
 * rename(2) within one directory is atomic, so a reader observes either the previous checkpoint
 * or the complete new one -- never a partially written file. A job killed mid-write leaves only
 * the scratch file behind.
 *
 * \exception std::runtime_error The scratch file is missing or the rename failed.
 */
void commitCheckpointTempFile( const boost::filesystem::path& tempPath,
                               const boost::filesystem::path& finalPath );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Snapshots the current world into a CheckpointMetadata.
 *
 * Time and step come from setCheckpointIdentity() when the driver has supplied them, otherwise
 * from pe's own TimeStep counter. The material table is read from the live process state.
 * bodyCount and pebBytes are left at zero: only the writer knows them, and it fills them in.
 */
CheckpointMetadata currentCheckpointMetadata();
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Records the driver's authoritative time/step and pairing tag for subsequent writes.
 *
 * Drivers that own the clock (a CFD solver stepping pe) must call this before each checkpoint
 * write, otherwise the checkpoint records pe's substep counter, which does not pair with the
 * driver's step index.
 */
void setCheckpointIdentity( real simulationTime, uint64_t timeStep, const std::string& pairingTag );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Serialises \a metadata to \a filename. Overwrites atomically.
 */
void writeCheckpointMetadata( const boost::filesystem::path& filename,
                              const CheckpointMetadata& metadata );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Parses the sidecar at \a filename.
 *
 * \return The parsed metadata, or a metadata with \a present == \a false if the file does not
 *         exist (legacy checkpoint).
 * \exception std::runtime_error The file exists but is malformed, truncated or declares an
 *            unsupported version.
 */
CheckpointMetadata readCheckpointMetadata( const boost::filesystem::path& filename );
//*************************************************************************************************


//=================================================================================================
//
//  RESUME VALIDATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Enforces \a expectation against \a metadata.
 *
 * \param label Human-readable identification of the checkpoint, used in error messages.
 * \exception std::runtime_error Any set expectation is violated, or an expectation is set but
 *            \a metadata is not present.
 */
void validateResumeExpectation( const CheckpointMetadata& metadata,
                                const ResumeExpectation& expectation,
                                const std::string& label );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reinstates the checkpoint's material table in the live process.
 *
 * Body material indices are only valid against the table that produced them, so the table must
 * be rebuilt -- at identical indices -- before any body is instantiated. Materials already
 * registered under the same name are reused after verifying their properties are unchanged;
 * missing ones are appended.
 *
 * \exception std::runtime_error The live table cannot be brought into agreement with the
 *            recorded one (name collision at a different index, or diverging properties).
 */
void restoreCheckpointMaterials( const CheckpointMetadata& metadata, const std::string& label );
//*************************************************************************************************

} // namespace pe

#endif // _PE_UTIL_CHECKPOINTMETADATA_H_
