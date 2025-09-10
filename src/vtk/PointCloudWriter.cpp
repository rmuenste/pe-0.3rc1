//=================================================================================================
/*!
 *  \file src/vtk/PointCloudWriter.cpp
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


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <pe/vtk/PointCloudWriter.h>
#include <pe/util/Assert.h>


namespace pe {

namespace vtk {

//=================================================================================================
//
//  BASIC POINT CLOUD OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes a basic point cloud without additional data fields.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 *
 * This function creates a simple point cloud visualization with only position data.
 */
void PointCloudWriter::writePoints( const std::string& filename,
                                   const std::vector<pe::Vec3>& points )
{
   ScalarFieldMap emptyScalars;
   writePoints( filename, points, emptyScalars );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a point cloud with scalar data fields.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param scalarFields Map of scalar field names to data arrays.
 *
 * This function creates a point cloud with associated scalar data for each point.
 * Each scalar field array must contain exactly the same number of values as there are points.
 */
void PointCloudWriter::writePoints( const std::string& filename,
                                   const std::vector<pe::Vec3>& points,
                                   const ScalarFieldMap& scalarFields )
{
   VectorFieldMap emptyVectors;
   writePoints( filename, points, scalarFields, emptyVectors );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a point cloud with scalar and vector data fields.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param scalarFields Map of scalar field names to data arrays.
 * \param vectorFields Map of vector field names to data arrays.
 *
 * This function creates a point cloud with both scalar and vector data associated with each point.
 * All data field arrays must contain exactly the same number of values as there are points.
 */
void PointCloudWriter::writePoints( const std::string& filename,
                                   const std::vector<pe::Vec3>& points,
                                   const ScalarFieldMap& scalarFields,
                                   const VectorFieldMap& vectorFields )
{
   IntegerFieldMap emptyIntegers;
   writePoints( filename, points, scalarFields, vectorFields, emptyIntegers );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a point cloud with scalar, vector, and integer data fields.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param scalarFields Map of scalar field names to data arrays.
 * \param vectorFields Map of vector field names to data arrays.
 * \param integerFields Map of integer field names to data arrays.
 *
 * This is the most complete point cloud writing function, supporting scalar, vector, and integer
 * data fields. All data field arrays must contain exactly the same number of values as there are points.
 */
void PointCloudWriter::writePoints( const std::string& filename,
                                   const std::vector<pe::Vec3>& points,
                                   const ScalarFieldMap& scalarFields,
                                   const VectorFieldMap& vectorFields,
                                   const IntegerFieldMap& integerFields )
{
   pe_USER_ASSERT( !points.empty(), "Point array cannot be empty" );
   
   const size_t numPoints = points.size();

   // Validate field sizes
   for( const auto& field : scalarFields ) {
      pe_USER_ASSERT( field.second.size() == numPoints,
                      "Scalar field '" << field.first << "' has incorrect size" );
   }
   for( const auto& field : vectorFields ) {
      pe_USER_ASSERT( field.second.size() == numPoints,
                      "Vector field '" << field.first << "' has incorrect size" );
   }
   for( const auto& field : integerFields ) {
      pe_USER_ASSERT( field.second.size() == numPoints,
                      "Integer field '" << field.first << "' has incorrect size" );
   }

   std::ofstream out( filename );
   if( !out ) {
      pe_THROW( std::runtime_error, "Cannot open output file: " << filename );
   }

   out << std::fixed << std::setprecision(6);

   // Write VTK header
   writeVTKHeader( out, "Point cloud data" );

   // Write point coordinates
   writePointData( out, points );

   // Write point data fields if any exist
   const bool hasData = !scalarFields.empty() || !vectorFields.empty() || !integerFields.empty();
   if( hasData ) {
      out << "POINT_DATA " << points.size() << "\n";

      // Write scalar fields
      for( const auto& field : scalarFields ) {
         writeScalarField( out, field.first, field.second );
      }

      // Write integer fields
      for( const auto& field : integerFields ) {
         writeScalarField( out, field.first, field.second );
      }

      // Write vector fields
      for( const auto& field : vectorFields ) {
         writeVectorField( out, field.first, field.second );
      }
   }
}
//*************************************************************************************************


//=================================================================================================
//
//  SPECIALIZED OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes a point cloud with a single boolean result field.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param fieldName Name of the boolean field.
 * \param results Boolean results for each point.
 *
 * This function is useful for visualizing binary test results such as containment tests.
 */
void PointCloudWriter::writeBooleanResults( const std::string& filename,
                                           const std::vector<pe::Vec3>& points,
                                           const std::string& fieldName,
                                           const std::vector<bool>& results )
{
   BooleanFieldMap booleanFields;
   booleanFields[fieldName] = results;
   writeBooleanResults( filename, points, booleanFields );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a point cloud with multiple boolean result fields.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param booleanFields Map of boolean field names to data arrays.
 *
 * This function supports multiple boolean fields, useful for comparing different test results.
 */
void PointCloudWriter::writeBooleanResults( const std::string& filename,
                                           const std::vector<pe::Vec3>& points,
                                           const BooleanFieldMap& booleanFields )
{
   pe_USER_ASSERT( !points.empty(), "Point array cannot be empty" );
   
   const size_t numPoints = points.size();

   // Validate field sizes
   for( const auto& field : booleanFields ) {
      pe_USER_ASSERT( field.second.size() == numPoints,
                      "Boolean field '" << field.first << "' has incorrect size" );
   }

   std::ofstream out( filename );
   if( !out ) {
      pe_THROW( std::runtime_error, "Cannot open output file: " << filename );
   }

   out << std::fixed << std::setprecision(6);

   // Write VTK header
   writeVTKHeader( out, "Point cloud with boolean results" );

   // Write point coordinates
   writePointData( out, points );

   // Write boolean fields
   if( !booleanFields.empty() ) {
      out << "POINT_DATA " << points.size() << "\n";
      
      for( const auto& field : booleanFields ) {
         writeBooleanField( out, field.first, field.second );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a point cloud comparing two boolean result sets.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param results1 First boolean result set.
 * \param results2 Second boolean result set.
 * \param field1Name Name for the first result field.
 * \param field2Name Name for the second result field.
 *
 * This function creates a comparison visualization with both result sets and an automatic
 * disagreement field showing where the results differ.
 */
void PointCloudWriter::writeComparisonResults( const std::string& filename,
                                              const std::vector<pe::Vec3>& points,
                                              const std::vector<bool>& results1,
                                              const std::vector<bool>& results2,
                                              const std::string& field1Name,
                                              const std::string& field2Name )
{
   pe_USER_ASSERT( !points.empty(), "Point array cannot be empty" );
   pe_USER_ASSERT( results1.size() == points.size(), "Results1 array size mismatch" );
   pe_USER_ASSERT( results2.size() == points.size(), "Results2 array size mismatch" );

   std::ofstream out( filename );
   if( !out ) {
      pe_THROW( std::runtime_error, "Cannot open output file: " << filename );
   }

   out << std::fixed << std::setprecision(6);

   // Write VTK header
   writeVTKHeader( out, "Comparison of boolean results" );

   // Write point coordinates
   writePointData( out, points );

   // Write comparison fields
   out << "POINT_DATA " << points.size() << "\n";
   
   writeBooleanField( out, field1Name, results1 );
   writeBooleanField( out, field2Name, results2 );

   // Create and write disagreement field
   std::vector<bool> disagreement;
   disagreement.reserve( points.size() );
   for( size_t i = 0; i < points.size(); ++i ) {
      disagreement.push_back( results1[i] != results2[i] );
   }
   writeBooleanField( out, "Disagreement", disagreement );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes containment test results in the format used by the CGAL examples.
 *
 * \param filename The name of the output VTK file.
 * \param points Vector of 3D points to write.
 * \param distanceMapResults DistanceMap containment results.
 * \param cgalResults CGAL containment results.
 * \param hasCgalResults Whether CGAL results are available.
 *
 * This function replicates the functionality of the original write_vtk_points function
 * from the CGAL examples, providing compatibility with existing analysis workflows.
 */
void PointCloudWriter::writeContainmentTest( const std::string& filename,
                                            const std::vector<pe::Vec3>& points,
                                            const std::vector<bool>& distanceMapResults,
                                            const std::vector<bool>& cgalResults,
                                            bool hasCgalResults )
{
   if( hasCgalResults ) {
      writeComparisonResults( filename, points, distanceMapResults, cgalResults,
                             "DistanceMapResult", "CGALResult" );
   } else {
      writeBooleanResults( filename, points, "DistanceMapResult", distanceMapResults );
   }
}
//*************************************************************************************************


//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes the VTK file header.
 *
 * \param out Output stream.
 * \param description Description for the dataset.
 */
void PointCloudWriter::writeVTKHeader( std::ostream& out, const std::string& description )
{
   out << "# vtk DataFile Version 3.0\n";
   out << description << "\n";
   out << "ASCII\n";
   out << "DATASET POLYDATA\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes point coordinate data.
 *
 * \param out Output stream.
 * \param points Vector of 3D points.
 */
void PointCloudWriter::writePointData( std::ostream& out, const std::vector<pe::Vec3>& points )
{
   out << "POINTS " << points.size() << " double\n";
   for( const auto& point : points ) {
      out << point[0] << " " << point[1] << " " << point[2] << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a scalar field with real values.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data values.
 */
void PointCloudWriter::writeScalarField( std::ostream& out, const std::string& name,
                                        const std::vector<pe::real>& data )
{
   out << "SCALARS " << name << " double 1\n";
   out << "LOOKUP_TABLE default\n";
   for( pe::real value : data ) {
      out << value << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a scalar field with integer values.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data values.
 */
void PointCloudWriter::writeScalarField( std::ostream& out, const std::string& name,
                                        const std::vector<int>& data )
{
   out << "SCALARS " << name << " int 1\n";
   out << "LOOKUP_TABLE default\n";
   for( int value : data ) {
      out << value << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a boolean field (converted to integers).
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Boolean field data.
 */
void PointCloudWriter::writeBooleanField( std::ostream& out, const std::string& name,
                                         const std::vector<bool>& data )
{
   out << "SCALARS " << name << " int 1\n";
   out << "LOOKUP_TABLE default\n";
   for( bool value : data ) {
      out << (value ? 1 : 0) << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a vector field.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Vector field data.
 */
void PointCloudWriter::writeVectorField( std::ostream& out, const std::string& name,
                                        const std::vector<pe::Vec3>& data )
{
   out << "VECTORS " << name << " double\n";
   for( const auto& vector : data ) {
      out << vector[0] << " " << vector[1] << " " << vector[2] << "\n";
   }
}
//*************************************************************************************************

} // namespace vtk

} // namespace pe