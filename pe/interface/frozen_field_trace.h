//=================================================================================================
/*!
 *  \file pe/interface/frozen_field_trace.h
 *  \brief Collective C++ interface for parallel frozen-field particle tracing.
 */
//=================================================================================================

#ifndef _PE_INTERFACE_FROZEN_FIELD_TRACE_H_
#define _PE_INTERFACE_FROZEN_FIELD_TRACE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <variant>
#include <vector>

namespace pe {

struct OffMeshData {
   std::string data;
};

struct ObjMeshFile {
   std::string path;
};

using SurfaceMeshInput = std::variant<OffMeshData, ObjMeshFile>;
using VelocityCallback =
   std::function<int( const double* vertices, double* velocities, std::size_t numVertices )>;

enum class CubeFace {
   XMin,
   XMax,
   YMin,
   YMax,
   ZMin,
   ZMax
};

struct TracerIdentity {
   std::uint64_t globalId;
   int originRank;
   std::size_t originLocalIndex;
};

struct TracerExit {
   TracerIdentity tracer;
   int exitRank;
   std::size_t timestep;
   double time;
   std::size_t triangleIndex;
   CubeFace face;
   std::array<double, 3> position;
   std::array<double, 3> velocity;
};

struct TracerSurvivor {
   TracerIdentity tracer;
   int ownerRank;
   std::array<double, 3> position;
   std::array<double, 3> velocity;
};

struct FrozenFieldTraceResult {
   std::vector<TracerExit> exits;
   std::vector<TracerSurvivor> survivors;
};

/*!
 * Collective operation on MPI_COMM_WORLD.
 *
 * MPI must already be initialized. Each rank supplies the same complete seed-point list and the
 * same surface mesh. The function loads example.json into SimulationConfig, decomposes the unit
 * cube, creates only the tracer spheres owned by each rank, creates a global fixed TriangleMesh
 * with an inverted DistanceMap, and advances passive tracer spheres for exactly
 * SimulationConfig::getTimesteps() main steps.
 *
 * The callback is invoked once before the first step and once after every main step. Its vertices
 * input contains 3*numVertices doubles, and it must write the same number of velocity components.
 * A nonzero return code aborts the collective operation with MPI_Abort.
 *
 * The returned exits and survivors are rank-local.
 */
PE_PUBLIC FrozenFieldTraceResult runFrozenFieldTrace(
   const std::vector<std::array<double, 3> >& seeds,
   const SurfaceMeshInput& surface,
   const VelocityCallback& callback );

} // namespace pe

#endif
