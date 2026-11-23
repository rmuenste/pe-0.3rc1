//=================================================================================================
/*!
 *  \file src/core/rigidbody/VertexCachePose.h
 *  \brief The exact pose test of the world-space vertex caches of triangle and inner meshes
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_SRC_CORE_RIGIDBODY_VERTEXCACHEPOSE_H_
#define _PE_SRC_CORE_RIGIDBODY_VERTEXCACHEPOSE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/math/Quaternion.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

namespace detail {

//*************************************************************************************************
/*!\brief Whether two reals are the same value, a zero including its sign.
 *
 * Compares values, not storage, so padding bytes (e.g. of a long double) play no role. NaN is
 * never the same, which only makes a cache recompute.
 */
inline bool sameValueAndSign( real a, real b )
{
   return a == b && std::signbit( a ) == std::signbit( b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the pose a vertex cache was filled for is exactly the current pose.
 *
 * \param lastGpos The position the cache was filled for.
 * \param lastQ The orientation the cache was filled for.
 * \param gpos The current position.
 * \param q The current orientation.
 * \return \a true if all seven components are the same (signs of zeros included).
 *
 * Exact on purpose: pe's tolerant vector and quaternion comparisons would keep the cached
 * vertices for a change of the pose below their tolerance.
 */
inline bool sameCachePose( const Vec3& lastGpos, const Quat& lastQ, const Vec3& gpos,
                           const Quat& q )
{
   return sameValueAndSign( lastGpos[0], gpos[0] ) && sameValueAndSign( lastGpos[1], gpos[1] ) &&
          sameValueAndSign( lastGpos[2], gpos[2] ) && sameValueAndSign( lastQ[0], q[0] ) &&
          sameValueAndSign( lastQ[1], q[1] ) && sameValueAndSign( lastQ[2], q[2] ) &&
          sameValueAndSign( lastQ[3], q[3] );
}
//*************************************************************************************************

} // namespace detail

} // namespace pe

#endif
