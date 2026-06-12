#include <pe/interface/frozen_field_trace.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#if HAVE_MPI && PE_USE_CGAL
#include <pe/config/SimulationConfig.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/TimeStep.h>
#include <pe/core/World.h>
#include <pe/core/detection/fine/DistanceMap.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/engine.h>
#include <pe/interface/decompose.h>
#endif

namespace pe {

#if HAVE_MPI && PE_USE_CGAL

namespace {

using Point = std::array<double, 3>;
using Face = std::array<std::size_t, 3>;

struct ParsedMesh {
   std::vector<Point> vertices;
   std::vector<Face> faces;
};

struct Intersection {
   double fraction;
   std::size_t triangleIndex;
   Point position;
};

constexpr double kGeometryEpsilon = 1.0e-10;
constexpr id_t kBoundaryUserId = std::numeric_limits<id_t>::max();

[[noreturn]] void collectiveAbort( MPI_Comm comm, const std::string& message, int code = 1 )
{
   int rank = -1;
   MPI_Comm_rank( comm, &rank );
   if( !message.empty() )
      std::cerr << "[rank " << rank << "] frozen-field trace: " << message << std::endl;
   MPI_Abort( comm, code > 0 ? code : 1 );
   std::abort();
}

void collectiveCheck( MPI_Comm comm, const std::string& localError )
{
   const int localFailed = localError.empty() ? 0 : 1;
   int anyFailed = 0;
   MPI_Allreduce( &localFailed, &anyFailed, 1, MPI_INT, MPI_MAX, comm );
   if( anyFailed )
      collectiveAbort( comm, localError.empty() ? "another rank reported an error" : localError );
}

bool finitePoint( const Point& point )
{
   return std::isfinite( point[0] ) && std::isfinite( point[1] ) && std::isfinite( point[2] );
}

Point toPoint( const Vec3& value )
{
   return Point{{ value[0], value[1], value[2] }};
}

Vec3 toVec3( const Point& value )
{
   return Vec3( value[0], value[1], value[2] );
}

Point subtract( const Point& lhs, const Point& rhs )
{
   return Point{{ lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2] }};
}

Point addScaled( const Point& start, const Point& direction, double scale )
{
   return Point{{ start[0] + direction[0] * scale,
                  start[1] + direction[1] * scale,
                  start[2] + direction[2] * scale }};
}

Point cross( const Point& lhs, const Point& rhs )
{
   return Point{{ lhs[1] * rhs[2] - lhs[2] * rhs[1],
                  lhs[2] * rhs[0] - lhs[0] * rhs[2],
                  lhs[0] * rhs[1] - lhs[1] * rhs[0] }};
}

double dot( const Point& lhs, const Point& rhs )
{
   return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

ParsedMesh parseOff( const std::string& data )
{
   std::istringstream input( data );
   std::string header;
   input >> header;
   if( header != "OFF" )
      throw std::invalid_argument( "OFF input must begin with OFF" );

   std::size_t vertexCount = 0;
   std::size_t faceCount = 0;
   std::size_t edgeCount = 0;
   if( !( input >> vertexCount >> faceCount >> edgeCount ) || vertexCount == 0 || faceCount == 0 )
      throw std::invalid_argument( "invalid OFF counts" );
   (void)edgeCount;

   ParsedMesh mesh;
   mesh.vertices.resize( vertexCount );
   for( Point& point : mesh.vertices ) {
      if( !( input >> point[0] >> point[1] >> point[2] ) || !finitePoint( point ) )
         throw std::invalid_argument( "invalid OFF vertex" );
   }

   for( std::size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex ) {
      std::size_t count = 0;
      if( !( input >> count ) || count < 3 )
         throw std::invalid_argument( "invalid OFF face" );
      std::vector<std::size_t> indices( count );
      for( std::size_t& index : indices ) {
         if( !( input >> index ) || index >= mesh.vertices.size() )
            throw std::invalid_argument( "OFF face index is out of range" );
      }
      for( std::size_t i = 1; i + 1 < indices.size(); ++i )
         mesh.faces.push_back( Face{{ indices[0], indices[i], indices[i + 1] }} );
   }

   return mesh;
}

long parseObjIndex( const std::string& token, std::size_t vertexCount )
{
   const std::size_t separator = token.find( '/' );
   const std::string indexText = token.substr( 0, separator );
   if( indexText.empty() )
      throw std::invalid_argument( "OBJ face has an empty vertex index" );

   std::size_t consumed = 0;
   const long rawIndex = std::stol( indexText, &consumed );
   if( consumed != indexText.size() || rawIndex == 0 )
      throw std::invalid_argument( "OBJ face has an invalid vertex index" );

   const long index = rawIndex > 0 ? rawIndex - 1 : static_cast<long>( vertexCount ) + rawIndex;
   if( index < 0 || static_cast<std::size_t>( index ) >= vertexCount )
      throw std::invalid_argument( "OBJ face index is out of range" );
   return index;
}

ParsedMesh parseObj( const std::string& path )
{
   std::ifstream input( path.c_str() );
   if( !input )
      throw std::runtime_error( "could not open OBJ file: " + path );

   ParsedMesh mesh;
   std::string line;
   while( std::getline( input, line ) ) {
      std::istringstream fields( line );
      std::string kind;
      fields >> kind;
      if( kind == "v" ) {
         Point point;
         if( !( fields >> point[0] >> point[1] >> point[2] ) || !finitePoint( point ) )
            throw std::invalid_argument( "invalid OBJ vertex in " + path );
         mesh.vertices.push_back( point );
      }
      else if( kind == "f" ) {
         std::vector<std::size_t> indices;
         std::string token;
         while( fields >> token )
            indices.push_back( static_cast<std::size_t>( parseObjIndex( token, mesh.vertices.size() ) ) );
         if( indices.size() < 3 )
            throw std::invalid_argument( "invalid OBJ face in " + path );
         for( std::size_t i = 1; i + 1 < indices.size(); ++i )
            mesh.faces.push_back( Face{{ indices[0], indices[i], indices[i + 1] }} );
      }
   }

   if( mesh.vertices.empty() || mesh.faces.empty() )
      throw std::invalid_argument( "OBJ contains no usable triangle surface: " + path );
   return mesh;
}

ParsedMesh parseSurface( const SurfaceMeshInput& surface )
{
   if( const OffMeshData* off = std::get_if<OffMeshData>( &surface ) )
      return parseOff( off->data );
   return parseObj( std::get<ObjMeshFile>( surface ).path );
}

std::uint64_t hashMesh( const ParsedMesh& mesh )
{
   std::uint64_t hash = UINT64_C(1469598103934665603);
   const auto mix = [&hash]( std::uint64_t value ) {
      for( int byte = 0; byte < 8; ++byte ) {
         hash ^= ( value >> ( byte * 8 ) ) & UINT64_C(0xff);
         hash *= UINT64_C(1099511628211);
      }
   };

   mix( mesh.vertices.size() );
   mix( mesh.faces.size() );
   for( const Point& point : mesh.vertices ) {
      for( double coordinate : point ) {
         std::uint64_t bits = 0;
         std::memcpy( &bits, &coordinate, sizeof( bits ) );
         mix( bits );
      }
   }
   for( const Face& face : mesh.faces )
      for( std::size_t index : face )
         mix( index );
   return hash;
}

std::uint64_t hashPoints( const std::vector<Point>& points )
{
   std::uint64_t hash = UINT64_C(1469598103934665603);
   const auto mix = [&hash]( std::uint64_t value ) {
      for( int byte = 0; byte < 8; ++byte ) {
         hash ^= ( value >> ( byte * 8 ) ) & UINT64_C(0xff);
         hash *= UINT64_C(1099511628211);
      }
   };

   mix( points.size() );
   for( const Point& point : points ) {
      for( double coordinate : point ) {
         std::uint64_t bits = 0;
         std::memcpy( &bits, &coordinate, sizeof( bits ) );
         mix( bits );
      }
   }
   return hash;
}

double signedVolumeTimesSix( const ParsedMesh& mesh )
{
   double volume = 0.0;
   for( const Face& face : mesh.faces )
      volume += dot( mesh.vertices[face[0]], cross( mesh.vertices[face[1]], mesh.vertices[face[2]] ) );
   return volume;
}

void validateMesh( const ParsedMesh& mesh )
{
   if( mesh.vertices.empty() || mesh.faces.empty() )
      throw std::invalid_argument( "surface mesh is empty" );

   Point minimum{{ std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity() }};
   Point maximum{{ -std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity() }};
   for( const Point& point : mesh.vertices ) {
      for( std::size_t axis = 0; axis < 3; ++axis ) {
         minimum[axis] = std::min( minimum[axis], point[axis] );
         maximum[axis] = std::max( maximum[axis], point[axis] );
         if( point[axis] < -kGeometryEpsilon || point[axis] > 1.0 + kGeometryEpsilon )
            throw std::invalid_argument( "surface vertex lies outside the unit cube" );
      }
      const bool onBoundary =
         std::abs( point[0] ) <= kGeometryEpsilon || std::abs( point[0] - 1.0 ) <= kGeometryEpsilon ||
         std::abs( point[1] ) <= kGeometryEpsilon || std::abs( point[1] - 1.0 ) <= kGeometryEpsilon ||
         std::abs( point[2] ) <= kGeometryEpsilon || std::abs( point[2] - 1.0 ) <= kGeometryEpsilon;
      if( !onBoundary )
         throw std::invalid_argument( "surface vertex is not on the unit-cube boundary" );
   }
   for( std::size_t axis = 0; axis < 3; ++axis ) {
      if( std::abs( minimum[axis] ) > kGeometryEpsilon ||
          std::abs( maximum[axis] - 1.0 ) > kGeometryEpsilon )
         throw std::invalid_argument( "surface bounding box must be exactly [0,1]^3" );
   }

   using Edge = std::pair<std::size_t, std::size_t>;
   std::map<Edge, std::pair<int, int> > edgeCounts;
   for( const Face& face : mesh.faces ) {
      if( face[0] == face[1] || face[1] == face[2] || face[2] == face[0] )
         throw std::invalid_argument( "surface contains a degenerate triangle" );
      for( int edge = 0; edge < 3; ++edge ) {
         const std::size_t from = face[edge];
         const std::size_t to = face[( edge + 1 ) % 3];
         if( from >= mesh.vertices.size() || to >= mesh.vertices.size() )
            throw std::invalid_argument( "surface face index is out of range" );
         const Edge key = std::minmax( from, to );
         std::pair<int, int>& counts = edgeCounts[key];
         ++counts.first;
         counts.second += from < to ? 1 : -1;
      }
   }
   for( const auto& entry : edgeCounts ) {
      if( entry.second.first != 2 || entry.second.second != 0 )
         throw std::invalid_argument( "surface must be closed and consistently oriented" );
   }

   if( signedVolumeTimesSix( mesh ) <= kGeometryEpsilon )
      throw std::invalid_argument( "surface triangles must use outward counter-clockwise winding" );
}

void validateEquivalentMeshes( MPI_Comm comm, const ParsedMesh& mesh )
{
   const std::uint64_t localHash = hashMesh( mesh );
   std::uint64_t minimumHash = 0;
   std::uint64_t maximumHash = 0;
   MPI_Allreduce( &localHash, &minimumHash, 1, MPI_UINT64_T, MPI_MIN, comm );
   MPI_Allreduce( &localHash, &maximumHash, 1, MPI_UINT64_T, MPI_MAX, comm );
   if( minimumHash != maximumHash )
      collectiveAbort( comm, "surface mesh differs between ranks" );
}

void validateEquivalentSeeds( MPI_Comm comm, const std::vector<Point>& seeds )
{
   const std::uint64_t localHash = hashPoints( seeds );
   std::uint64_t minimumHash = 0;
   std::uint64_t maximumHash = 0;
   MPI_Allreduce( &localHash, &minimumHash, 1, MPI_UINT64_T, MPI_MIN, comm );
   MPI_Allreduce( &localHash, &maximumHash, 1, MPI_UINT64_T, MPI_MAX, comm );
   if( minimumHash != maximumHash )
      collectiveAbort( comm, "seed points differ between ranks" );
}

Point meshCenterOfMass( const ParsedMesh& mesh )
{
   Point numerator{{ 0.0, 0.0, 0.0 }};
   double total = 0.0;
   for( const Face& face : mesh.faces ) {
      const Point& a = mesh.vertices[face[0]];
      const Point& b = mesh.vertices[face[1]];
      const Point& c = mesh.vertices[face[2]];
      const double volume = dot( a, cross( b, c ) );
      total += volume;
      for( std::size_t axis = 0; axis < 3; ++axis )
         numerator[axis] += ( a[axis] + b[axis] + c[axis] ) * volume;
   }
   for( double& coordinate : numerator )
      coordinate /= 4.0 * total;
   return numerator;
}

TracerIdentity decodeIdentity( id_t id )
{
   const std::uint64_t globalId = static_cast<std::uint64_t>( id );
   TracerIdentity identity;
   identity.globalId = globalId;
   identity.originRank = static_cast<int>( globalId >> 32 );
   identity.originLocalIndex = static_cast<std::size_t>( globalId & UINT64_C(0xffffffff) );
   return identity;
}

std::vector<BodyID> localTracers()
{
   std::vector<BodyID> tracers;
   WorldID world = theWorld();
   for( auto it = world->begin(); it != world->end(); ++it ) {
      BodyID body = *it;
      if( body->getType() == sphereType && !body->isRemote() )
         tracers.push_back( body );
   }
   std::sort( tracers.begin(), tracers.end(),
              []( BodyID lhs, BodyID rhs ) { return lhs->getID() < rhs->getID(); } );
   return tracers;
}

void applyVelocityCallback( MPI_Comm comm, const VelocityCallback& callback )
{
   std::vector<BodyID> tracers = localTracers();
   std::vector<double> positions( 3 * tracers.size() );
   std::vector<double> velocities( 3 * tracers.size(), 0.0 );
   for( std::size_t i = 0; i < tracers.size(); ++i ) {
      const Vec3 position = tracers[i]->getPosition();
      positions[3 * i] = position[0];
      positions[3 * i + 1] = position[1];
      positions[3 * i + 2] = position[2];
   }

   int callbackCode = 0;
   try {
      callbackCode = callback( positions.data(), velocities.data(), tracers.size() );
   }
   catch( const std::exception& error ) {
      std::cerr << "frozen-field velocity callback threw: " << error.what() << std::endl;
      callbackCode = std::numeric_limits<int>::min() + 1;
   }
   catch( ... ) {
      std::cerr << "frozen-field velocity callback threw an unknown exception" << std::endl;
      callbackCode = std::numeric_limits<int>::min() + 2;
   }

   int anyFailure = 0;
   const int localFailure = callbackCode == 0 ? 0 : 1;
   MPI_Allreduce( &localFailure, &anyFailure, 1, MPI_INT, MPI_MAX, comm );
   if( anyFailure ) {
      const int candidate = callbackCode == 0 ? std::numeric_limits<int>::max() : callbackCode;
      int reportedCode = 0;
      MPI_Allreduce( &candidate, &reportedCode, 1, MPI_INT, MPI_MIN, comm );
      collectiveAbort( comm, "velocity callback returned error code " + std::to_string( reportedCode ),
                       reportedCode );
   }

   std::string localError;
   for( std::size_t i = 0; i < tracers.size(); ++i ) {
      const Point velocity{{ velocities[3 * i], velocities[3 * i + 1], velocities[3 * i + 2] }};
      if( !finitePoint( velocity ) ) {
         localError = "velocity callback produced a non-finite value";
         break;
      }
      tracers[i]->setLinearVel( toVec3( velocity ) );
      tracers[i]->setAngularVel( 0.0, 0.0, 0.0 );
   }
   collectiveCheck( comm, localError );
}

bool segmentTriangleIntersection( const Point& start, const Point& end,
                                  const Point& a, const Point& b, const Point& c,
                                  double& fraction )
{
   const Point direction = subtract( end, start );
   const Point edge1 = subtract( b, a );
   const Point edge2 = subtract( c, a );
   const Point p = cross( direction, edge2 );
   const double determinant = dot( edge1, p );
   if( std::abs( determinant ) <= kGeometryEpsilon )
      return false;

   const double inverse = 1.0 / determinant;
   const Point t = subtract( start, a );
   const double u = dot( t, p ) * inverse;
   if( u < -kGeometryEpsilon || u > 1.0 + kGeometryEpsilon )
      return false;

   const Point q = cross( t, edge1 );
   const double v = dot( direction, q ) * inverse;
   if( v < -kGeometryEpsilon || u + v > 1.0 + kGeometryEpsilon )
      return false;

   fraction = dot( edge2, q ) * inverse;
   return fraction >= -kGeometryEpsilon && fraction <= 1.0 + kGeometryEpsilon;
}

Intersection firstIntersection( const ParsedMesh& mesh, const Point& start, const Point& end )
{
   Intersection result;
   result.fraction = std::numeric_limits<double>::infinity();
   result.triangleIndex = std::numeric_limits<std::size_t>::max();
   const Point direction = subtract( end, start );

   for( std::size_t i = 0; i < mesh.faces.size(); ++i ) {
      const Face& face = mesh.faces[i];
      double fraction = 0.0;
      if( segmentTriangleIntersection( start, end, mesh.vertices[face[0]],
                                      mesh.vertices[face[1]], mesh.vertices[face[2]], fraction ) &&
          ( fraction < result.fraction - kGeometryEpsilon ||
            ( std::abs( fraction - result.fraction ) <= kGeometryEpsilon &&
              i < result.triangleIndex ) ) ) {
         result.fraction = std::max( 0.0, std::min( 1.0, fraction ) );
         result.triangleIndex = i;
         result.position = addScaled( start, direction, result.fraction );
      }
   }

   if( result.triangleIndex == std::numeric_limits<std::size_t>::max() )
      throw std::runtime_error( "outside tracer segment did not intersect the supplied surface" );
   return result;
}

CubeFace classifyCubeFace( const Point& point )
{
   const std::array<double, 6> distances{{
      std::abs( point[0] ), std::abs( point[0] - 1.0 ),
      std::abs( point[1] ), std::abs( point[1] - 1.0 ),
      std::abs( point[2] ), std::abs( point[2] - 1.0 )
   }};
   return static_cast<CubeFace>(
      std::distance( distances.begin(), std::min_element( distances.begin(), distances.end() ) ) );
}

bool outsideUnitCube( const Point& point )
{
   return point[0] < -kGeometryEpsilon || point[0] > 1.0 + kGeometryEpsilon ||
          point[1] < -kGeometryEpsilon || point[1] > 1.0 + kGeometryEpsilon ||
          point[2] < -kGeometryEpsilon || point[2] > 1.0 + kGeometryEpsilon;
}

bool removeExitingTracers( MPI_Comm comm, WorldID world, const ParsedMesh& mesh,
                           double stepSize, std::size_t timestep,
                           std::vector<TracerExit>& exits )
{
   const int rank = theMPISystem()->getRank();
   std::vector<id_t> exitingIds;
   std::string localError;

   for( BodyID tracer : localTracers() ) {
      const Point start = toPoint( tracer->getPosition() );
      const Point velocity = toPoint( tracer->getLinearVel() );
      const Point end = addScaled( start, velocity, stepSize );
      if( !outsideUnitCube( end ) )
         continue;

      try {
         const Intersection intersection = firstIntersection( mesh, start, end );
         TracerExit exit;
         exit.tracer = decodeIdentity( tracer->getID() );
         exit.exitRank = rank;
         exit.timestep = timestep;
         exit.time = ( static_cast<double>( timestep - 1 ) + intersection.fraction ) * stepSize;
         exit.triangleIndex = intersection.triangleIndex;
         exit.face = classifyCubeFace( intersection.position );
         exit.position = intersection.position;
         exit.velocity = velocity;
         exits.push_back( exit );
         exitingIds.push_back( tracer->getID() );
      }
      catch( const std::exception& error ) {
         localError = error.what();
         break;
      }
   }
   collectiveCheck( comm, localError );

   if( exitingIds.empty() )
      return false;
   std::sort( exitingIds.begin(), exitingIds.end() );
   for( auto it = world->begin(); it != world->end(); ) {
      if( std::binary_search( exitingIds.begin(), exitingIds.end(), it->getID() ) )
         it = world->destroy( it );
      else
         ++it;
   }
   return true;
}

} // namespace

FrozenFieldTraceResult runFrozenFieldTrace(
   const std::vector<std::array<double, 3> >& seeds,
   const SurfaceMeshInput& surface,
   const VelocityCallback& callback )
{
   int initialized = 0;
   MPI_Initialized( &initialized );
   if( !initialized )
      throw std::runtime_error( "runFrozenFieldTrace requires MPI to be initialized" );

   MPI_Comm comm = MPI_COMM_WORLD;
   int rank = 0;
   int size = 0;
   MPI_Comm_rank( comm, &rank );
   MPI_Comm_size( comm, &size );
   collectiveCheck( comm, callback ? "" : "runFrozenFieldTrace requires a velocity callback" );

   static bool called = false;
   collectiveCheck( comm, called ? "runFrozenFieldTrace may only be called once per process" : "" );
   called = true;

   std::string localError;
   try {
      SimulationConfig::loadFromFile( "example.json" );
   }
   catch( const std::exception& error ) {
      localError = error.what();
   }
   collectiveCheck( comm, localError );

   SimulationConfig& config = SimulationConfig::getInstance();
   const int px = config.getProcessesX();
   const int py = config.getProcessesY();
   const int pz = config.getProcessesZ();
   if( px <= 0 || py <= 0 || pz <= 0 || px * py * pz != size )
      collectiveAbort( comm, "process grid in example.json does not match MPI_COMM_WORLD" );
   if( config.getStepsize() <= 0.0 || config.getSubsteps() <= 0 || config.getBenchRadius() <= 0.0 )
      collectiveAbort( comm, "stepsize_, substeps_, and benchRadius_ must be positive" );

   int dims[3] = { px, py, pz };
   int periods[3] = { 0, 0, 0 };
   MPI_Comm cartComm = MPI_COMM_NULL;
   MPI_Cart_create( comm, 3, dims, periods, 0, &cartComm );
   if( cartComm == MPI_COMM_NULL )
      collectiveAbort( comm, "failed to create Cartesian communicator" );

   MPISystemID mpiSystem = theMPISystem();
   mpiSystem->setComm( cartComm );
   MPI_Comm_rank( cartComm, &rank );
   int coordinates[3] = { 0, 0, 0 };
   MPI_Cart_coords( cartComm, rank, 3, coordinates );
   decomposeDomain( coordinates, 0.0, 0.0, 0.0,
                    1.0 / px, 1.0 / py, 1.0 / pz, px, py, pz );
   mpiSystem->checkProcesses();

   ParsedMesh parsedMesh;
   localError.clear();
   try {
      parsedMesh = parseSurface( surface );
      validateMesh( parsedMesh );
   }
   catch( const std::exception& error ) {
      localError = error.what();
   }
   collectiveCheck( cartComm, localError );
   validateEquivalentMeshes( cartComm, parsedMesh );

   WorldID world = theWorld();
   world->setGravity( Vec3( 0.0, 0.0, 0.0 ) );
   world->setDamping( 1.0 );
   world->setAutoForceReset( true );
   TimeStep::stepsize( config.getStepsize() );

   MaterialID boundaryMaterial = createMaterial(
      "frozen_field_trace_boundary", 1.0, 0.0, 0.0, 0.0, 0.2, 1.0, 1.0, 0.0, 0.0 );
   MaterialID tracerMaterial = createMaterial(
      "frozen_field_trace_particle", config.getParticleDensity(),
      0.0, 0.0, 0.0, 0.2, 1.0, 1.0, 0.0, 0.0 );

   Vertices peVertices;
   peVertices.reserve( parsedMesh.vertices.size() );
   for( const Point& vertex : parsedMesh.vertices )
      peVertices.push_back( toVec3( vertex ) );
   IndicesLists peFaces;
   peFaces.reserve( parsedMesh.faces.size() );
   for( const Face& face : parsedMesh.faces )
      peFaces.push_back( Vector3<std::size_t>( face[0], face[1], face[2] ) );

   TriangleMeshID boundary = nullptr;
   const Point center = meshCenterOfMass( parsedMesh );
   pe_GLOBAL_SECTION {
      boundary = createTriangleMesh( kBoundaryUserId, toVec3( center ), peVertices, peFaces,
                                     boundaryMaterial, true, true );
      boundary->setFixed( true );
      boundary->setCollisionEnabled( false );
      boundary->enableDistanceMapAcceleration(
         config.getDomainBoundaryDistanceMapResolution(),
         config.getDomainBoundaryDistanceMapTolerance() );
      if( !boundary->hasDistanceMap() )
         collectiveAbort( cartComm, "failed to create DistanceMap for supplied surface" );
      boundary->getDistanceMap()->invertForDomainBoundary();
   }

   if( seeds.size() > UINT32_MAX )
      collectiveAbort( cartComm, "more than UINT32_MAX seed points were supplied" );
   localError.clear();
   std::vector<Point> parsedSeeds;
   parsedSeeds.reserve( seeds.size() );
   for( std::size_t i = 0; i < seeds.size(); ++i ) {
      const Point seed{{ seeds[i][0], seeds[i][1], seeds[i][2] }};
      if( !finitePoint( seed ) || outsideUnitCube( seed ) ) {
         localError = "seed point is non-finite or outside the unit cube";
         break;
      }
      parsedSeeds.push_back( seed );
   }
   collectiveCheck( cartComm, localError );
   validateEquivalentSeeds( cartComm, parsedSeeds );

   for( std::size_t i = 0; i < parsedSeeds.size(); ++i ) {
      if( !world->ownsPoint( toVec3( parsedSeeds[i] ) ) )
         continue;
      const std::uint64_t globalId =
         ( static_cast<std::uint64_t>( static_cast<std::uint32_t>( rank ) ) << 32 ) |
         static_cast<std::uint32_t>( i );
      SphereID tracer = createSphere( static_cast<id_t>( globalId ), toVec3( parsedSeeds[i] ),
                                      config.getBenchRadius(), tracerMaterial, true );
      tracer->setCollisionEnabled( false );
      tracer->setLinearVel( 0.0, 0.0, 0.0 );
      tracer->setAngularVel( 0.0, 0.0, 0.0 );
   }

   world->synchronize();

   FrozenFieldTraceResult result;
   applyVelocityCallback( cartComm, callback );

   const double mainStepSize = config.getStepsize();
   const double substepSize = mainStepSize / config.getSubsteps();
   for( std::size_t step = 1; step <= config.getTimesteps(); ++step ) {
      const bool removedLocal =
         removeExitingTracers( cartComm, world, parsedMesh, mainStepSize, step, result.exits );
      int removedAny = 0;
      const int removed = removedLocal ? 1 : 0;
      MPI_Allreduce( &removed, &removedAny, 1, MPI_INT, MPI_MAX, cartComm );
      if( removedAny )
         world->synchronize();

      for( int substep = 0; substep < config.getSubsteps(); ++substep ) {
         world->simulationStep( substepSize );
      }
      applyVelocityCallback( cartComm, callback );
   }
   TimeStep::stepsize( mainStepSize );

   for( BodyID tracer : localTracers() ) {
      TracerSurvivor survivor;
      survivor.tracer = decodeIdentity( tracer->getID() );
      survivor.ownerRank = rank;
      survivor.position = toPoint( tracer->getPosition() );
      survivor.velocity = toPoint( tracer->getLinearVel() );
      result.survivors.push_back( survivor );
   }
   std::sort( result.exits.begin(), result.exits.end(),
              []( const TracerExit& lhs, const TracerExit& rhs ) {
                 return lhs.tracer.globalId < rhs.tracer.globalId;
              } );

   return result;
}

#else

FrozenFieldTraceResult runFrozenFieldTrace(
   const std::vector<std::array<double, 3> >&,
   const SurfaceMeshInput&,
   const VelocityCallback& )
{
   throw std::runtime_error(
      "runFrozenFieldTrace requires a PE build with MPI and CGAL/DistanceMap support" );
}

#endif

} // namespace pe
