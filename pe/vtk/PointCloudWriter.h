//=================================================================================================
/*!
 *  \file pe/vtk/PointCloudWriter.h
 *  \brief VTK point cloud writer for visualization of point data
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

#ifndef _PE_VTK_POINTCLOUDWRITER_H_
#define _PE_VTK_POINTCLOUDWRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <vector>
#include <map>
#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>


namespace pe {

namespace vtk {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief VTK point cloud writer for visualization of scattered point data.
 * \ingroup vtk
 *
 * The PointCloudWriter class provides functionality to export point clouds with associated
 * scalar and vector data as VTK POLYDATA files for visualization in ParaView and other 
 * VTK-compatible tools. This is particularly useful for visualizing test results, particle
 * positions, containment analysis, and other scattered data.
 *
 * The class follows PE design patterns as a utility writer - it provides direct-use functions
 * that can be called explicitly when immediate output is needed, complementing the existing
 * simulation-integrated Writer class.
 *
 * Example usage:
 * \code
 * std::vector<pe::Vec3> points = generateTestPoints();
 * std::vector<bool> containmentResults = testContainment(points, mesh);
 * 
 * // Simple boolean result visualization
 * pe::vtk::PointCloudWriter::writeBooleanResults("containment_test.vtk", points, 
 *                                                "DistanceMap", containmentResults);
 *
 * // Multiple scalar fields
 * std::map<std::string, std::vector<pe::real>> scalarFields;
 * scalarFields["distance"] = distanceValues;
 * scalarFields["alpha"] = alphaValues;
 * pe::vtk::PointCloudWriter::writePoints("analysis.vtk", points, scalarFields);
 * \endcode
 */
class PointCloudWriter
{
public:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef std::map<std::string, std::vector<pe::real>>    ScalarFieldMap;   //!< Map of scalar field names to data
   typedef std::map<std::string, std::vector<pe::Vec3>>    VectorFieldMap;   //!< Map of vector field names to data
   typedef std::map<std::string, std::vector<bool>>        BooleanFieldMap;  //!< Map of boolean field names to data
   typedef std::map<std::string, std::vector<int>>         IntegerFieldMap;  //!< Map of integer field names to data
   /*! \endcond */
   //**********************************************************************************************

   //**Basic point cloud output functions*********************************************************
   /*!\name Basic point cloud output functions */
   //@{
   static void writePoints( const std::string& filename,
                           const std::vector<pe::Vec3>& points );

   static void writePoints( const std::string& filename,
                           const std::vector<pe::Vec3>& points,
                           const ScalarFieldMap& scalarFields );

   static void writePoints( const std::string& filename,
                           const std::vector<pe::Vec3>& points,
                           const ScalarFieldMap& scalarFields,
                           const VectorFieldMap& vectorFields );

   static void writePoints( const std::string& filename,
                           const std::vector<pe::Vec3>& points,
                           const ScalarFieldMap& scalarFields,
                           const VectorFieldMap& vectorFields,
                           const IntegerFieldMap& integerFields );
   //@}
   //**********************************************************************************************

   //**Specialized output functions****************************************************************
   /*!\name Specialized output functions */
   //@{
   static void writeBooleanResults( const std::string& filename,
                                   const std::vector<pe::Vec3>& points,
                                   const std::string& fieldName,
                                   const std::vector<bool>& results );

   static void writeBooleanResults( const std::string& filename,
                                   const std::vector<pe::Vec3>& points,
                                   const BooleanFieldMap& booleanFields );

   static void writeComparisonResults( const std::string& filename,
                                      const std::vector<pe::Vec3>& points,
                                      const std::vector<bool>& results1,
                                      const std::vector<bool>& results2,
                                      const std::string& field1Name,
                                      const std::string& field2Name );

   static void writeContainmentTest( const std::string& filename,
                                    const std::vector<pe::Vec3>& points,
                                    const std::vector<bool>& distanceMapResults,
                                    const std::vector<bool>& cgalResults,
                                    bool hasCgalResults = true );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static void writeVTKHeader( std::ostream& out, const std::string& description );
   static void writePointData( std::ostream& out, const std::vector<pe::Vec3>& points );
   static void writeScalarField( std::ostream& out, const std::string& name,
                                const std::vector<pe::real>& data );
   static void writeScalarField( std::ostream& out, const std::string& name,
                                const std::vector<int>& data );
   static void writeBooleanField( std::ostream& out, const std::string& name,
                                 const std::vector<bool>& data );
   static void writeVectorField( std::ostream& out, const std::string& name,
                                const std::vector<pe::Vec3>& data );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace vtk

} // namespace pe

#endif /* _PE_VTK_POINTCLOUDWRITER_H_ */