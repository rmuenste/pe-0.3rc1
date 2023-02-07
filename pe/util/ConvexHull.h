//=================================================================================================
/*!
 *  \file examples/mpibrazil/ConvexHull.h
 *  \brief Header file for computation of convex hulls
 *
 *  Copyright (C) 2014 Tobias Preclik
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

#ifndef _PE_UTIL_CONVEXHULL_H_
#define _PE_UTIL_CONVEXHULL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/TriangleMeshTypes.h>


namespace pe {

namespace chull {

void generateRock( int n, pe::Vertices& verticesOut, pe::IndicesLists& facesOut, pe::real radius );

} // namespace chull

} // namespace pe

#endif
