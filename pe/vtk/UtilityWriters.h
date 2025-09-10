//=================================================================================================
/*!
 *  \file pe/vtk/UtilityWriters.h
 *  \brief Convenience header for VTK utility writers
 *
 *  Copyright (C) 2024 pe Development Team
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

#ifndef _PE_VTK_UTILITYWRITERS_H_
#define _PE_VTK_UTILITYWRITERS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/vtk/DistanceMapWriter.h>
#include <pe/vtk/PointCloudWriter.h>
#include <pe/vtk/MeshDataWriter.h>


//*************************************************************************************************
/*!\defgroup VTK utility writers */
//*************************************************************************************************


//=================================================================================================
//
//  DOCUMENTATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Convenience header for VTK utility writers.
 * \ingroup vtk
 *
 * This header provides convenient access to all VTK utility writers in the pe::vtk namespace.
 * These utility writers complement the existing simulation-integrated Writer class by providing
 * direct-use tools for analysis, debugging, and specialized visualization tasks.
 *
 * ## Available Utility Writers
 *
 * ### DistanceMapWriter
 * Exports DistanceMap data as VTK structured grids (.vti files) for 3D visualization.
 * Supports signed distance fields, alpha values, surface normals, and contact points.
 * 
 * ### PointCloudWriter  
 * Exports point clouds with scalar and vector data as VTK polydata files.
 * Ideal for visualizing test results, containment analysis, and scattered data.
 *
 * ### MeshDataWriter
 * Exports triangle meshes with enhanced scalar and vector data support.
 * Supports both PE TriangleMesh objects and CGAL Surface_mesh objects.
 *
 * ## Design Philosophy
 *
 * These utility writers follow a **direct-use approach** where functions are called explicitly
 * when immediate output is needed, in contrast to the existing simulation-integrated Writer
 * class that runs automatically during time steps.
 *
 * **Key characteristics:**
 * - **Non-invasive**: Zero impact on existing PE simulation workflows  
 * - **Specialized**: Each writer focuses on specific data types and use cases
 * - **Immediate**: Write data immediately when called, no pipeline integration
 * - **Analysis-focused**: Perfect for debugging, testing, and post-processing
 * - **PE-compliant**: Follow PE naming conventions, types, and error handling
 *
 * ## Example Usage
 *
 * \code
 * #include <pe/vtk/UtilityWriters.h>
 *
 * // DistanceMap visualization
 * pe::vtk::DistanceMapWriter::writeVTI("distance_field.vti", distanceMap);
 * 
 * // Point cloud analysis results
 * pe::vtk::PointCloudWriter::writeContainmentTest("containment.vtk", points, 
 *                                                 dmResults, cgalResults);
 *
 * // Mesh with analysis data
 * std::map<std::string, std::vector<pe::real>> scalarFields;
 * scalarFields["stress"] = stressValues;
 * pe::vtk::MeshDataWriter::writeMesh("stress_analysis.vtk", mesh, scalarFields);
 * \endcode
 *
 * ## Integration with Existing PE VTK System
 *
 * The utility writers are designed to complement, not replace, the existing PE VTK system:
 *
 * **Existing Writer class**: Handles ongoing simulation visualization with time-series output,
 * automatic body registration, and PVD collection generation.
 *
 * **New Utility writers**: Handle immediate, specialized output for analysis and debugging
 * tasks that don't fit into the simulation pipeline paradigm.
 *
 * Both systems coexist peacefully and serve different but complementary purposes in the
 * PE visualization ecosystem.
 */
//*************************************************************************************************


#endif /* _PE_VTK_UTILITYWRITERS_H_ */