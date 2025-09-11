//=================================================================================================
/*!
 *  \file src/vtk/DistanceMapWriter.cpp
 *  \brief VTK structured grid writer for DistanceMap data
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
#include <pe/vtk/DistanceMapWriter.h>
#include <pe/util/Assert.h>


namespace pe {

namespace vtk {

//=================================================================================================
//
//  DISTANCEMAP OUTPUT FUNCTIONS
//
//=================================================================================================

#ifdef PE_USE_CGAL
//*************************************************************************************************
/*!\brief Writes a complete DistanceMap to a VTK Image Data file.
 *
 * \param filename The name of the output VTI file.
 * \param distanceMap The DistanceMap to export.
 *
 * This function exports all available data fields from the DistanceMap including signed distance
 * field (SDF), alpha (containment) values, surface normals, and contact points.
 */
void DistanceMapWriter::writeVTI( const std::string& filename, const DistanceMap& distanceMap )
{
   writeVTI( filename, distanceMap, true, true, true, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes selected DistanceMap data fields to a VTK Image Data file.
 *
 * \param filename The name of the output VTI file.
 * \param distanceMap The DistanceMap to export.
 * \param includeSDF Include signed distance field data.
 * \param includeAlpha Include alpha (containment) data.
 * \param includeNormals Include surface normal data.
 * \param includeContactPoints Include contact point data.
 *
 * This function allows selective export of DistanceMap data fields, useful for reducing file
 * size when only specific data is needed for visualization or analysis.
 */
void DistanceMapWriter::writeVTI( const std::string& filename, const DistanceMap& distanceMap,
                                 bool includeSDF, bool includeAlpha, 
                                 bool includeNormals, bool includeContactPoints )
{
   // Get grid parameters
   int nx = distanceMap.getNx();
   int ny = distanceMap.getNy(); 
   int nz = distanceMap.getNz();
   pe::real spacing = distanceMap.getSpacing();
   pe::Vec3 origin = distanceMap.getOrigin();

   // Get data arrays
   const std::vector<pe::real>& sdfData = distanceMap.getSdfData();
   const std::vector<int>& alphaData = distanceMap.getAlphaData();
   const std::vector<pe::Vec3>& normalData = distanceMap.getNormalData();
   const std::vector<pe::Vec3>& contactPointData = distanceMap.getContactPointData();

   // Create empty vectors for fields that shouldn't be included
   std::vector<pe::real> emptySdf;
   std::vector<int> emptyAlpha;
   std::vector<pe::Vec3> emptyNormals;
   std::vector<pe::Vec3> emptyContactPoints;

   // Use original data or empty vectors based on inclusion flags
   const std::vector<pe::real>& sdf = includeSDF ? sdfData : emptySdf;
   const std::vector<int>& alpha = includeAlpha ? alphaData : emptyAlpha;
   const std::vector<pe::Vec3>& normals = includeNormals ? normalData : emptyNormals;
   const std::vector<pe::Vec3>& contactPoints = includeContactPoints ? contactPointData : emptyContactPoints;

   // Write to file using low-level function
   writeVTI( filename, sdf, alpha, normals, contactPoints, nx, ny, nz,
            spacing, spacing, spacing, origin[0], origin[1], origin[2] );
}
//*************************************************************************************************
#endif


//=================================================================================================
//
//  CUSTOM GRID OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes custom scalar field data to a VTK Image Data file.
 *
 * \param filename The name of the output VTI file.
 * \param origin Grid origin coordinates.
 * \param spacing Grid spacing in each dimension.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.
 * \param nz Number of grid points in Z direction.
 * \param scalarFields Map of scalar field names to data arrays.
 *
 * This function provides flexibility to export arbitrary scalar field data arranged on a
 * regular 3D grid. Each scalar field must contain exactly nx*ny*nz values.
 */
void DistanceMapWriter::writeCustomGrid( const std::string& filename,
                                        const pe::Vec3& origin,
                                        const pe::Vec3& spacing,
                                        int nx, int ny, int nz,
                                        const ScalarFieldMap& scalarFields )
{
   VectorFieldMap emptyVectorFields;
   writeCustomGrid( filename, origin, spacing, nx, ny, nz, scalarFields, emptyVectorFields );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes custom scalar and vector field data to a VTK Image Data file.
 *
 * \param filename The name of the output VTI file.
 * \param origin Grid origin coordinates.
 * \param spacing Grid spacing in each dimension.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.
 * \param nz Number of grid points in Z direction.
 * \param scalarFields Map of scalar field names to data arrays.
 * \param vectorFields Map of vector field names to data arrays.
 *
 * This function provides maximum flexibility to export arbitrary scalar and vector field data
 * arranged on a regular 3D grid. All field arrays must contain the correct number of values
 * (nx*ny*nz for scalar fields, nx*ny*nz for vector fields).
 */
void DistanceMapWriter::writeCustomGrid( const std::string& filename,
                                        const pe::Vec3& origin,
                                        const pe::Vec3& spacing,
                                        int nx, int ny, int nz,
                                        const ScalarFieldMap& scalarFields,
                                        const VectorFieldMap& vectorFields )
{
   pe_USER_ASSERT( nx > 0 && ny > 0 && nz > 0, "Grid dimensions must be positive" );
   pe_USER_ASSERT( spacing[0] > 0.0 && spacing[1] > 0.0 && spacing[2] > 0.0, 
                   "Grid spacing must be positive" );

   const size_t expectedSize = static_cast<size_t>(nx * ny * nz);

   // Validate scalar field sizes
   for( const auto& field : scalarFields ) {
      pe_USER_ASSERT( field.second.size() == expectedSize,
                      "Scalar field '" << field.first << "' has incorrect size" );
   }

   // Validate vector field sizes
   for( const auto& field : vectorFields ) {
      pe_USER_ASSERT( field.second.size() == expectedSize,
                      "Vector field '" << field.first << "' has incorrect size" );
   }

   std::ofstream out( filename );
   if( !out ) {
      std::ostringstream oss;
      oss << "Cannot open output file: " << filename;
      throw std::runtime_error(oss.str());
   }

   out << std::fixed << std::setprecision(6);

   // Write VTI header
   writeVTIHeader( out, origin, spacing, nx, ny, nz );

   // Start point data section
   out << "      <PointData";
   if( !scalarFields.empty() ) {
      out << " Scalars=\"" << scalarFields.begin()->first << "\"";
   }
   out << ">\n";

   // Write scalar fields
   for( const auto& field : scalarFields ) {
      writeScalarField( out, field.first, field.second );
   }

   // Write vector fields
   for( const auto& field : vectorFields ) {
      writeVectorField( out, field.first, field.second, nx, ny, nz );
   }

   out << "      </PointData>\n";
   out << "      <CellData/>\n";

   // Write VTI footer
   writeVTIFooter( out );
}
//*************************************************************************************************


//=================================================================================================
//
//  LOW-LEVEL OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes DistanceMap-style data to a VTK Image Data file (without face indices).
 *
 * \param filename The name of the output VTI file.
 * \param sdf Signed distance field values.
 * \param alpha Alpha (containment) values.
 * \param normals Surface normal vectors.
 * \param contactPoints Contact point vectors.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.
 * \param nz Number of grid points in Z direction.
 * \param dx Grid spacing in X direction.
 * \param dy Grid spacing in Y direction.
 * \param dz Grid spacing in Z direction.
 * \param x0 Grid origin X coordinate.
 * \param y0 Grid origin Y coordinate.
 * \param z0 Grid origin Z coordinate.
 *
 * This is the low-level function that performs the actual VTI file writing. It directly
 * corresponds to the original write_vti function from the examples but uses PE types and
 * error handling conventions.
 */
void DistanceMapWriter::writeVTI( const std::string& filename,
                                 const std::vector<pe::real>& sdf,
                                 const std::vector<int>& alpha,
                                 const std::vector<pe::Vec3>& normals,
                                 const std::vector<pe::Vec3>& contactPoints,
                                 int nx, int ny, int nz,
                                 pe::real dx, pe::real dy, pe::real dz,
                                 pe::real x0, pe::real y0, pe::real z0 )
{
   std::vector<int> emptyFaceIndex;
   writeVTI( filename, sdf, alpha, normals, contactPoints, emptyFaceIndex,
            nx, ny, nz, dx, dy, dz, x0, y0, z0 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes DistanceMap-style data with face indices to a VTK Image Data file.
 *
 * \param filename The name of the output VTI file.
 * \param sdf Signed distance field values.
 * \param alpha Alpha (containment) values.
 * \param normals Surface normal vectors.
 * \param contactPoints Contact point vectors.
 * \param faceIndex Face index values.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.
 * \param nz Number of grid points in Z direction.
 * \param dx Grid spacing in X direction.
 * \param dy Grid spacing in Y direction.
 * \param dz Grid spacing in Z direction.
 * \param x0 Grid origin X coordinate.
 * \param y0 Grid origin Y coordinate.
 * \param z0 Grid origin Z coordinate.
 *
 * This function includes face index data in addition to the standard DistanceMap fields.
 * This is the most complete low-level writing function that handles all possible data fields.
 */
void DistanceMapWriter::writeVTI( const std::string& filename,
                                 const std::vector<pe::real>& sdf,
                                 const std::vector<int>& alpha,
                                 const std::vector<pe::Vec3>& normals,
                                 const std::vector<pe::Vec3>& contactPoints,
                                 const std::vector<int>& faceIndex,
                                 int nx, int ny, int nz,
                                 pe::real dx, pe::real dy, pe::real dz,
                                 pe::real x0, pe::real y0, pe::real z0 )
{
   pe_USER_ASSERT( nx > 0 && ny > 0 && nz > 0, "Grid dimensions must be positive" );
   pe_USER_ASSERT( dx > 0.0 && dy > 0.0 && dz > 0.0, "Grid spacing must be positive" );
   
   const size_t expectedSize = static_cast<size_t>(nx * ny * nz);

   // Validate array sizes (allow empty arrays)
   if( !sdf.empty() ) {
      pe_USER_ASSERT( sdf.size() == expectedSize, "SDF array size mismatch" );
   }
   if( !alpha.empty() ) {
      pe_USER_ASSERT( alpha.size() == expectedSize, "Alpha array size mismatch" );
   }
   if( !normals.empty() ) {
      pe_USER_ASSERT( normals.size() == expectedSize, "Normals array size mismatch" );
   }
   if( !contactPoints.empty() ) {
      pe_USER_ASSERT( contactPoints.size() == expectedSize, "Contact points array size mismatch" );
   }
   if( !faceIndex.empty() ) {
      pe_USER_ASSERT( faceIndex.size() == expectedSize, "Face index array size mismatch" );
   }

   std::ofstream out( filename );
   if( !out ) {
      std::ostringstream oss;
      oss << "Cannot open output file: " << filename;
      throw std::runtime_error( oss.str() );
   }

   out << std::fixed << std::setprecision(6);

   // Write VTI header
   pe::Vec3 origin( x0, y0, z0 );
   pe::Vec3 spacing( dx, dy, dz );
   writeVTIHeader( out, origin, spacing, nx, ny, nz );

   // Start point data section
   out << "      <PointData";
   if( !sdf.empty() ) {
      out << " Scalars=\"sdf\"";
   }
   out << ">\n";

   // Write data fields
   if( !sdf.empty() ) {
      writeScalarField( out, "sdf", sdf );
   }
   
   if( !alpha.empty() ) {
      writeScalarField( out, "alpha", alpha );
   }

   if( !normals.empty() ) {
      writeVectorField( out, "normal", normals, nx, ny, nz );
   }

   if( !contactPoints.empty() ) {
      writeVectorField( out, "contact_point", contactPoints, nx, ny, nz );
   }

   if( !faceIndex.empty() ) {
      writeScalarField( out, "face", faceIndex );
   }

   out << "      </PointData>\n";
   out << "      <CellData/>\n";

   // Write VTI footer
   writeVTIFooter( out );
}
//*************************************************************************************************


//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes the VTI file header.
 *
 * \param out Output stream.
 * \param origin Grid origin coordinates.
 * \param spacing Grid spacing in each dimension.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.
 * \param nz Number of grid points in Z direction.
 */
void DistanceMapWriter::writeVTIHeader( std::ostream& out,
                                       const pe::Vec3& origin, const pe::Vec3& spacing,
                                       int nx, int ny, int nz )
{
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"ImageData\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << "  <ImageData Origin=\"" << origin[0] << " " << origin[1] << " " << origin[2] << "\" "
       << "Spacing=\"" << spacing[0] << " " << spacing[1] << " " << spacing[2] << "\" "
       << "WholeExtent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";
   out << "    <Piece Extent=\"0 " << nx - 1 << " 0 " << ny - 1 << " 0 " << nz - 1 << "\">\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes the VTI file footer.
 *
 * \param out Output stream.
 */
void DistanceMapWriter::writeVTIFooter( std::ostream& out )
{
   out << "    </Piece>\n";
   out << "  </ImageData>\n";
   out << "</VTKFile>\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a scalar field with real values.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data values.
 */
void DistanceMapWriter::writeScalarField( std::ostream& out, const std::string& name,
                                         const std::vector<pe::real>& data )
{
   out << "        <DataArray type=\"Float32\" Name=\"" << name << "\" format=\"ascii\">\n";
   for( pe::real v : data ) {
      out << v << " ";
   }
   out << "\n        </DataArray>\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a scalar field with integer values.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data values.
 */
void DistanceMapWriter::writeScalarField( std::ostream& out, const std::string& name,
                                         const std::vector<int>& data )
{
   out << "        <DataArray type=\"Int32\" Name=\"" << name << "\" format=\"ascii\">\n";
   for( int v : data ) {
      out << v << " ";
   }
   out << "\n        </DataArray>\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a vector field.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data vectors.
 * \param nx Number of grid points in X direction.
 * \param ny Number of grid points in Y direction.  
 * \param nz Number of grid points in Z direction.
 */
void DistanceMapWriter::writeVectorField( std::ostream& out, const std::string& name,
                                         const std::vector<pe::Vec3>& data, 
                                         int nx, int ny, int nz )
{
   out << "        <DataArray type=\"Float32\" Name=\"" << name 
       << "\" NumberOfComponents=\"3\" format=\"ascii\">\n";
   for( int z = 0; z < nz; ++z ) {
      for( int y = 0; y < ny; ++y ) {
         for( int x = 0; x < nx; ++x ) {
            int idx = x + y * nx + z * nx * ny;
            const auto& v = data[idx];
            out << v[0] << " " << v[1] << " " << v[2] << " \n";
         }
      }
   }
   out << "\n        </DataArray>\n";
}
//*************************************************************************************************

} // namespace vtk

} // namespace pe