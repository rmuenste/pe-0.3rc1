//=================================================================================================
/*!
 *  \file src/util/CheckpointMetadata.cpp
 *  \brief Implementation of the rigid body checkpoint metadata sidecar
 *
 *  Sidecar layout (version 1), one directive per line, `#` starts a comment line:
 *
 *      metadataVersion  <uint>
 *      timeSource       driver|pe-timestep
 *      simulationTime   <real>
 *      timeStep         <uint64>
 *      stepSize         <real>
 *      bodyCount        <uint64>
 *      pebBytes         <uint64>
 *      pairingTag       <rest of line, may be empty>
 *      materialCount    <uint64>
 *      material <index> <density> <cor> <csf> <cdf> <poisson> <young> <stiffness>
 *               <dampingN> <dampingT> <name, rest of line>
 *
 *  Reals are written with max_digits10 so the text form roundtrips exactly; the equality checks
 *  in restoreCheckpointMaterials() rely on that. Free-form fields (pairingTag, material name)
 *  are always last on their line so they may contain spaces.
 */
//=================================================================================================


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/CheckpointMetadata.h>

#include <pe/core/Materials.h>
#include <pe/core/World.h>
#include <pe/core/TimeStep.h>

#include <boost/filesystem/fstream.hpp>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unistd.h>


namespace pe {

//*************************************************************************************************
const char* const checkpointMetadataExtension = ".peinfo";
//*************************************************************************************************


namespace {

//*************************************************************************************************
/*!\brief Driver-supplied identity, valid until the driver overwrites it.
 *
 * Process-global rather than a Checkpointer member because the periodic writer (Checkpointer's
 * Trigger) and manual writeCheckpoint() calls must agree on the same identity, and a driver
 * calling through the C interface holds no CheckpointerID.
 */
struct DriverIdentity
{
   DriverIdentity() : valid( false ), simulationTime( 0 ), timeStep( 0 ) {}

   bool valid;
   real simulationTime;
   uint64_t timeStep;
   std::string pairingTag;
};

DriverIdentity& driverIdentity()
{
   static DriverIdentity identity;
   return identity;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Stream manipulator setup for exact real roundtripping.
 */
void configureRealOutput( std::ostream& os )
{
   os << std::scientific << std::setprecision( std::numeric_limits<real>::max_digits10 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads the remainder of \a stream as a free-form field, without leading blank.
 */
std::string readRestOfLine( std::istringstream& stream )
{
   std::string rest;
   std::getline( stream, rest );
   std::string::size_type begin = rest.find_first_not_of( " \t" );
   if( begin == std::string::npos )
      return std::string();
   std::string::size_type end = rest.find_last_not_of( " \t\r" );
   return rest.substr( begin, end - begin + 1 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extracts one value, failing loudly with the offending directive named.
 */
template< typename T >
T requireValue( std::istringstream& stream, const std::string& key, const std::string& file )
{
   T value = T();
   if( !( stream >> value ) )
      throw std::runtime_error( "Checkpoint metadata '" + file + "': directive '" + key +
                                "' is missing its value or the value is malformed." );
   return value;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Fails with a message naming the material property, its recorded and its live value.
 */
void reportPropertyMismatch( const std::string& materialName, const std::string& property,
                             real recorded, real live, const std::string& label )
{
   std::ostringstream message;
   configureRealOutput( message );
   message << "Checkpoint resume refused for " << label << ": material '" << materialName
           << "' is already registered in this process with a different " << property
           << " (checkpoint: " << recorded << ", process: " << live << "). Body masses were "
              "derived from the checkpoint's values, so resuming would silently change the "
              "physics. Align the setup with the checkpoint or start fresh.";
   throw std::runtime_error( message.str() );
}
//*************************************************************************************************

} // namespace


//=================================================================================================
//
//  STRUCT CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
CheckpointMetadata::CheckpointMetadata()
   : present( false )
   , version( 0 )
   , timeSource( checkpointTimeFromTimeStep )
   , simulationTime( 0 )
   , timeStep( 0 )
   , stepSize( 0 )
   , bodyCount( 0 )
   , pebBytes( 0 )
{}
//*************************************************************************************************


//*************************************************************************************************
ResumeExpectation::ResumeExpectation()
   : hasTime( false )
   , time( 0 )
   , toleranceSteps( defaultResumeTimeToleranceSteps )
   , hasStep( false )
   , step( 0 )
   , hasTag( false )
{}
//*************************************************************************************************


//=================================================================================================
//
//  PATHS AND ATOMIC PUBLICATION
//
//=================================================================================================

//*************************************************************************************************
boost::filesystem::path checkpointMetadataPath( const boost::filesystem::path& checkpointsPath,
                                                const std::string& name )
{
   return checkpointsPath / ( name + checkpointMetadataExtension );
}
//*************************************************************************************************


//*************************************************************************************************
boost::filesystem::path checkpointTempPath( const boost::filesystem::path& finalPath )
{
   std::ostringstream suffix;
   suffix << finalPath.filename().string() << ".tmp-" << static_cast<long>( ::getpid() );
   return finalPath.parent_path() / suffix.str();
}
//*************************************************************************************************


//*************************************************************************************************
void commitCheckpointTempFile( const boost::filesystem::path& tempPath,
                               const boost::filesystem::path& finalPath )
{
   if( !boost::filesystem::exists( tempPath ) )
      throw std::runtime_error( "Cannot publish checkpoint file '" + finalPath.string() +
                                "': the scratch file '" + tempPath.string() + "' does not exist." );

   boost::system::error_code error;
   boost::filesystem::rename( tempPath, finalPath, error );
   if( error ) {
      boost::filesystem::remove( tempPath, error );
      throw std::runtime_error( "Cannot publish checkpoint file '" + finalPath.string() +
                                "': rename from '" + tempPath.string() + "' failed." );
   }
}
//*************************************************************************************************


//=================================================================================================
//
//  IDENTITY
//
//=================================================================================================

//*************************************************************************************************
void setCheckpointIdentity( real simulationTime, uint64_t timeStep, const std::string& pairingTag )
{
   DriverIdentity& identity = driverIdentity();
   identity.valid          = true;
   identity.simulationTime = simulationTime;
   identity.timeStep       = timeStep;
   identity.pairingTag     = pairingTag;
}
//*************************************************************************************************


//*************************************************************************************************
CheckpointMetadata currentCheckpointMetadata()
{
   CheckpointMetadata metadata;
   metadata.present  = true;
   metadata.version  = checkpointMetadataVersion;
   metadata.stepSize = TimeStep::size();

   const DriverIdentity& identity = driverIdentity();
   if( identity.valid ) {
      metadata.timeSource     = checkpointTimeFromDriver;
      metadata.simulationTime = identity.simulationTime;
      metadata.timeStep       = identity.timeStep;
      metadata.pairingTag     = identity.pairingTag;
   }
   else {
      metadata.timeSource     = checkpointTimeFromTimeStep;
      metadata.timeStep       = TimeStep::step();
      metadata.simulationTime = static_cast<real>( TimeStep::step() ) * TimeStep::size();
   }

   // bodyCount is filled in by the writer, which knows how many records actually went into the
   // `.peb`; World::size() is not that number (non-global planes are not persisted at all).

   const size_t materialCount = Material::count();
   metadata.materials.reserve( materialCount );
   for( size_t i = 0; i < materialCount; ++i ) {
      const MaterialID id( i );
      CheckpointMaterialRecord record;
      record.name      = Material::getName( id );
      record.density   = Material::getDensity( id );
      record.cor       = Material::getRestitution( id );
      record.csf       = Material::getStaticFriction( id );
      record.cdf       = Material::getDynamicFriction( id );
      record.poisson   = Material::getPoissonRatio( id );
      record.young     = Material::getYoungModulus( id );
      record.stiffness = Material::getStiffness( id );
      record.dampingN  = Material::getDampingN( id );
      record.dampingT  = Material::getDampingT( id );
      metadata.materials.push_back( record );
   }

   return metadata;
}
//*************************************************************************************************


//=================================================================================================
//
//  SIDECAR I/O
//
//=================================================================================================

//*************************************************************************************************
void writeCheckpointMetadata( const boost::filesystem::path& filename,
                              const CheckpointMetadata& metadata )
{
   const boost::filesystem::path tempPath = checkpointTempPath( filename );

   {
      boost::filesystem::ofstream out( tempPath, std::ios::out | std::ios::trunc );
      if( !out )
         throw std::runtime_error( "Cannot write checkpoint metadata '" + tempPath.string() + "'." );

      out << "# pe checkpoint metadata -- sidecar of the companion .peb file\n";
      out << "metadataVersion " << checkpointMetadataVersion << "\n";
      out << "timeSource "
          << ( metadata.timeSource == checkpointTimeFromDriver ? "driver" : "pe-timestep" ) << "\n";

      configureRealOutput( out );
      out << "simulationTime " << metadata.simulationTime << "\n";
      out << "timeStep " << metadata.timeStep << "\n";
      out << "stepSize " << metadata.stepSize << "\n";
      out << "bodyCount " << metadata.bodyCount << "\n";
      out << "pebBytes " << metadata.pebBytes << "\n";
      out << "pairingTag " << metadata.pairingTag << "\n";
      out << "materialCount " << metadata.materials.size() << "\n";

      for( size_t i = 0; i < metadata.materials.size(); ++i ) {
         const CheckpointMaterialRecord& record = metadata.materials[i];
         out << "material " << i
             << " " << record.density
             << " " << record.cor
             << " " << record.csf
             << " " << record.cdf
             << " " << record.poisson
             << " " << record.young
             << " " << record.stiffness
             << " " << record.dampingN
             << " " << record.dampingT
             << " " << record.name << "\n";
      }

      out.flush();
      if( !out )
         throw std::runtime_error( "Failed while writing checkpoint metadata '" +
                                   tempPath.string() + "'." );
   }

   commitCheckpointTempFile( tempPath, filename );
}
//*************************************************************************************************


//*************************************************************************************************
CheckpointMetadata readCheckpointMetadata( const boost::filesystem::path& filename )
{
   CheckpointMetadata metadata;

   if( !boost::filesystem::exists( filename ) )
      return metadata;  // Legacy checkpoint: absence is reported, not an error.

   boost::filesystem::ifstream in( filename );
   if( !in )
      throw std::runtime_error( "Cannot read checkpoint metadata '" + filename.string() + "'." );

   const std::string file = filename.string();
   size_t declaredMaterialCount = 0;
   bool sawVersion = false, sawTimeSource = false, sawSimulationTime = false, sawTimeStep = false;
   bool sawStepSize = false, sawBodyCount = false, sawPebBytes = false, sawPairingTag = false;
   bool sawMaterialCount = false;

   std::string line;
   while( std::getline( in, line ) ) {
      if( line.empty() || line[0] == '#' )
         continue;

      std::istringstream stream( line );
      std::string key;
      stream >> key;
      if( key.empty() )
         continue;

      if( key == "metadataVersion" ) {
         metadata.version = requireValue<unsigned int>( stream, key, file );
         if( metadata.version > checkpointMetadataVersion ) {
            std::ostringstream message;
            message << "Checkpoint metadata '" << file << "' declares layout version "
                    << metadata.version << ", but this build understands at most version "
                    << checkpointMetadataVersion << ". Refusing to guess at its contents.";
            throw std::runtime_error( message.str() );
         }
         sawVersion = true;
      }
      else if( key == "timeSource" ) {
         const std::string value = requireValue<std::string>( stream, key, file );
         if( value == "driver" )
            metadata.timeSource = checkpointTimeFromDriver;
         else if( value == "pe-timestep" )
            metadata.timeSource = checkpointTimeFromTimeStep;
         else
            throw std::runtime_error( "Checkpoint metadata '" + file + "': unknown timeSource '" +
                                      value + "'." );
         sawTimeSource = true;
      }
      else if( key == "simulationTime" ) {
         metadata.simulationTime = requireValue<real>( stream, key, file );
         sawSimulationTime = true;
      }
      else if( key == "timeStep" ) {
         metadata.timeStep = requireValue<uint64_t>( stream, key, file );
         sawTimeStep = true;
      }
      else if( key == "stepSize" ) {
         metadata.stepSize = requireValue<real>( stream, key, file );
         sawStepSize = true;
      }
      else if( key == "bodyCount" ) {
         metadata.bodyCount = requireValue<uint64_t>( stream, key, file );
         sawBodyCount = true;
      }
      else if( key == "pebBytes" ) {
         metadata.pebBytes = requireValue<uint64_t>( stream, key, file );
         sawPebBytes = true;
      }
      else if( key == "pairingTag" ) {
         metadata.pairingTag = readRestOfLine( stream );  // May legitimately be empty.
         sawPairingTag = true;
      }
      else if( key == "materialCount" ) {
         declaredMaterialCount = requireValue<size_t>( stream, key, file );
         sawMaterialCount = true;
      }
      else if( key == "material" ) {
         const size_t index = requireValue<size_t>( stream, key, file );
         if( index != metadata.materials.size() ) {
            std::ostringstream message;
            message << "Checkpoint metadata '" << file << "': material entries must be in index "
                       "order; expected index " << metadata.materials.size() << ", found "
                    << index << ".";
            throw std::runtime_error( message.str() );
         }
         CheckpointMaterialRecord record;
         record.density   = requireValue<real>( stream, key, file );
         record.cor       = requireValue<real>( stream, key, file );
         record.csf       = requireValue<real>( stream, key, file );
         record.cdf       = requireValue<real>( stream, key, file );
         record.poisson   = requireValue<real>( stream, key, file );
         record.young     = requireValue<real>( stream, key, file );
         record.stiffness = requireValue<real>( stream, key, file );
         record.dampingN  = requireValue<real>( stream, key, file );
         record.dampingT  = requireValue<real>( stream, key, file );
         record.name      = readRestOfLine( stream );
         if( record.name.empty() )
            throw std::runtime_error( "Checkpoint metadata '" + file +
                                      "': material entry without a name." );
         metadata.materials.push_back( record );
      }
      else {
         throw std::runtime_error( "Checkpoint metadata '" + file + "': unknown directive '" +
                                   key + "'." );
      }
   }

   if( !sawVersion || !sawTimeSource || !sawSimulationTime || !sawTimeStep || !sawStepSize ||
       !sawBodyCount || !sawPebBytes || !sawPairingTag || !sawMaterialCount )
      throw std::runtime_error( "Checkpoint metadata '" + file + "' is incomplete; it was most "
                                "likely truncated by a killed job. Refusing to resume from it." );

   if( declaredMaterialCount != metadata.materials.size() ) {
      std::ostringstream message;
      message << "Checkpoint metadata '" << file << "' declares " << declaredMaterialCount
              << " materials but contains " << metadata.materials.size()
              << "; the file is truncated or corrupt.";
      throw std::runtime_error( message.str() );
   }

   metadata.present = true;
   return metadata;
}
//*************************************************************************************************


//=================================================================================================
//
//  RESUME VALIDATION
//
//=================================================================================================

//*************************************************************************************************
void validateResumeExpectation( const CheckpointMetadata& metadata,
                                const ResumeExpectation& expectation,
                                const std::string& label )
{
   if( !expectation.any() )
      return;

   if( !metadata.present )
      throw std::runtime_error( "Checkpoint resume refused for " + label + ": the driver supplied "
                                "an expected time/step/tag, but this is a legacy checkpoint "
                                "without a metadata sidecar, so the expectation cannot be "
                                "checked. Re-write the checkpoint with a current pe build or "
                                "clear the expectation keys." );

   if( expectation.hasStep && metadata.timeStep != expectation.step ) {
      std::ostringstream message;
      message << "Checkpoint resume refused for " << label << ": step index mismatch. The "
                 "checkpoint was written at step " << metadata.timeStep
              << ", the driver expects step " << expectation.step
              << ". Pair the checkpoint with the driver dump written at the same step.";
      throw std::runtime_error( message.str() );
   }

   if( expectation.hasTime ) {
      const real tolerance = expectation.toleranceSteps * metadata.stepSize;
      const real deviation = std::abs( metadata.simulationTime - expectation.time );
      if( !( deviation <= tolerance ) ) {
         std::ostringstream message;
         configureRealOutput( message );
         message << "Checkpoint resume refused for " << label << ": simulation time mismatch. "
                    "The checkpoint was written at t = " << metadata.simulationTime
                 << ", the driver expects t = " << expectation.time << " (deviation "
                 << deviation << " exceeds the tolerance " << tolerance << " = "
                 << expectation.toleranceSteps << " x stepSize " << metadata.stepSize << ").";
         throw std::runtime_error( message.str() );
      }
   }

   if( expectation.hasTag && metadata.pairingTag != expectation.tag )
      throw std::runtime_error( "Checkpoint resume refused for " + label + ": pairing tag "
                                "mismatch. The checkpoint carries '" + metadata.pairingTag +
                                "', the driver expects '" + expectation.tag + "'." );
}
//*************************************************************************************************


//=================================================================================================
//
//  MATERIAL TABLE RESTORE
//
//=================================================================================================

//*************************************************************************************************
void restoreCheckpointMaterials( const CheckpointMetadata& metadata, const std::string& label )
{
   if( !metadata.present )
      return;

   for( size_t index = 0; index < metadata.materials.size(); ++index ) {
      const CheckpointMaterialRecord& record = metadata.materials[index];

      const MaterialID existing = Material::find( record.name );
      if( existing != invalid_material ) {
         if( static_cast<size_t>( existing ) != index ) {
            std::ostringstream message;
            message << "Checkpoint resume refused for " << label << ": material '" << record.name
                    << "' is registered at index " << existing
                    << " in this process but was index " << index
                    << " in the checkpoint. Body material references would point at the wrong "
                       "material.";
            throw std::runtime_error( message.str() );
         }

         if( Material::getDensity( existing ) != record.density )
            reportPropertyMismatch( record.name, "density", record.density,
                                    Material::getDensity( existing ), label );
         if( Material::getRestitution( existing ) != record.cor )
            reportPropertyMismatch( record.name, "coefficient of restitution", record.cor,
                                    Material::getRestitution( existing ), label );
         if( Material::getStaticFriction( existing ) != record.csf )
            reportPropertyMismatch( record.name, "coefficient of static friction", record.csf,
                                    Material::getStaticFriction( existing ), label );
         if( Material::getDynamicFriction( existing ) != record.cdf )
            reportPropertyMismatch( record.name, "coefficient of dynamic friction", record.cdf,
                                    Material::getDynamicFriction( existing ), label );
         if( Material::getPoissonRatio( existing ) != record.poisson )
            reportPropertyMismatch( record.name, "Poisson ratio", record.poisson,
                                    Material::getPoissonRatio( existing ), label );
         if( Material::getYoungModulus( existing ) != record.young )
            reportPropertyMismatch( record.name, "Young modulus", record.young,
                                    Material::getYoungModulus( existing ), label );
         if( Material::getStiffness( existing ) != record.stiffness )
            reportPropertyMismatch( record.name, "stiffness", record.stiffness,
                                    Material::getStiffness( existing ), label );
         if( Material::getDampingN( existing ) != record.dampingN )
            reportPropertyMismatch( record.name, "normal damping", record.dampingN,
                                    Material::getDampingN( existing ), label );
         if( Material::getDampingT( existing ) != record.dampingT )
            reportPropertyMismatch( record.name, "tangential damping", record.dampingT,
                                    Material::getDampingT( existing ), label );
         continue;
      }

      if( Material::count() != index ) {
         std::ostringstream message;
         message << "Checkpoint resume refused for " << label << ": cannot reinstate material '"
                 << record.name << "' at index " << index << " because this process already has "
                 << Material::count() << " materials registered. The material table diverged "
                    "from the one the checkpoint was written with.";
         throw std::runtime_error( message.str() );
      }

      createMaterial( record.name, record.density, record.cor, record.csf, record.cdf,
                      record.poisson, record.young, record.stiffness, record.dampingN,
                      record.dampingT );
   }
}
//*************************************************************************************************

} // namespace pe
