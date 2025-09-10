//=================================================================================================
/*!
 *  \file src/vtk/MeshDataWriter.cpp
 *  \brief VTK mesh writer with enhanced scalar and vector data support
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
#include <pe/vtk/MeshDataWriter.h>
#include <pe/util/Assert.h>

#ifdef PE_USE_CGAL
#include <CGAL/boost/graph/helpers.h>
#endif


namespace pe {

namespace vtk {

//=================================================================================================
//
//  PE TRIANGLEMESH OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes a PE TriangleMesh without additional data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The PE TriangleMesh to export.
 *
 * This function creates a basic mesh visualization with only geometry data.
 */
void MeshDataWriter::writeMesh( const std::string& filename, const TriangleMeshID& mesh )
{
   ScalarFieldMap emptyScalars;
   writeMesh( filename, mesh, emptyScalars );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a PE TriangleMesh with scalar data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The PE TriangleMesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 *
 * This function creates a mesh visualization with associated scalar data for each vertex.
 * Each scalar field array must contain exactly the same number of values as there are vertices.
 */
void MeshDataWriter::writeMesh( const std::string& filename, 
                               const TriangleMeshID& mesh,
                               const ScalarFieldMap& scalarFields )
{
   VectorFieldMap emptyVectors;
   writeMesh( filename, mesh, scalarFields, emptyVectors );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a PE TriangleMesh with scalar and vector data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The PE TriangleMesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 *
 * This function creates a mesh visualization with both scalar and vector data associated 
 * with each vertex. All data field arrays must contain exactly the same number of values 
 * as there are vertices.
 */
void MeshDataWriter::writeMesh( const std::string& filename,
                               const TriangleMeshID& mesh,
                               const ScalarFieldMap& scalarFields,
                               const VectorFieldMap& vectorFields )
{
   IntegerFieldMap emptyIntegers;
   writeMesh( filename, mesh, scalarFields, vectorFields, emptyIntegers );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a PE TriangleMesh with scalar, vector, and integer data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The PE TriangleMesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 * \param integerFields Map of integer field names to vertex data arrays.
 *
 * This is the most complete PE TriangleMesh writing function, supporting scalar, vector, 
 * and integer data fields. All data field arrays must contain exactly the same number of 
 * values as there are vertices.
 */
void MeshDataWriter::writeMesh( const std::string& filename,
                               const TriangleMeshID& mesh,
                               const ScalarFieldMap& scalarFields,
                               const VectorFieldMap& vectorFields,
                               const IntegerFieldMap& integerFields )
{
   pe_USER_ASSERT( mesh, "TriangleMesh pointer cannot be null" );

   // Extract mesh geometry data
   std::vector<pe::Vec3> vertices;
   std::vector<std::vector<int>> faces;
   extractMeshData( mesh, vertices, faces );

   // Write using low-level function
   writeMeshData( filename, vertices, faces, scalarFields, vectorFields, integerFields,
                 "PE TriangleMesh with data fields" );
}
//*************************************************************************************************


//=================================================================================================
//
//  CGAL SURFACE_MESH OUTPUT FUNCTIONS
//
//=================================================================================================

#ifdef PE_USE_CGAL
//*************************************************************************************************
/*!\brief Writes a CGAL Surface_mesh without additional data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The CGAL Surface_mesh to export.
 *
 * This function creates a basic mesh visualization with only geometry data.
 */
void MeshDataWriter::writeCGALMesh( const std::string& filename, const CGALSurfaceMesh& mesh )
{
   ScalarFieldMap emptyScalars;
   writeCGALMesh( filename, mesh, emptyScalars );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a CGAL Surface_mesh with scalar data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The CGAL Surface_mesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 *
 * This function creates a mesh visualization with associated scalar data for each vertex.
 * Each scalar field array must contain exactly the same number of values as there are vertices.
 */
void MeshDataWriter::writeCGALMesh( const std::string& filename,
                                   const CGALSurfaceMesh& mesh,
                                   const ScalarFieldMap& scalarFields )
{
   VectorFieldMap emptyVectors;
   writeCGALMesh( filename, mesh, scalarFields, emptyVectors );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a CGAL Surface_mesh with scalar and vector data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The CGAL Surface_mesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 *
 * This function creates a mesh visualization with both scalar and vector data associated 
 * with each vertex. All data field arrays must contain exactly the same number of values 
 * as there are vertices.
 */
void MeshDataWriter::writeCGALMesh( const std::string& filename,
                                   const CGALSurfaceMesh& mesh,
                                   const ScalarFieldMap& scalarFields,
                                   const VectorFieldMap& vectorFields )
{
   IntegerFieldMap emptyIntegers;
   writeCGALMesh( filename, mesh, scalarFields, vectorFields, emptyIntegers );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a CGAL Surface_mesh with scalar, vector, and integer data fields.
 *
 * \param filename The name of the output VTK file.
 * \param mesh The CGAL Surface_mesh to export.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 * \param integerFields Map of integer field names to vertex data arrays.
 *
 * This is the most complete CGAL Surface_mesh writing function, supporting scalar, vector, 
 * and integer data fields. All data field arrays must contain exactly the same number of 
 * values as there are vertices.
 */
void MeshDataWriter::writeCGALMesh( const std::string& filename,
                                   const CGALSurfaceMesh& mesh,
                                   const ScalarFieldMap& scalarFields,
                                   const VectorFieldMap& vectorFields,
                                   const IntegerFieldMap& integerFields )
{
   // Extract mesh geometry data
   std::vector<pe::Vec3> vertices;
   std::vector<std::vector<int>> faces;
   extractCGALMeshData( mesh, vertices, faces );

   // Write using low-level function
   writeMeshData( filename, vertices, faces, scalarFields, vectorFields, integerFields,
                 "CGAL Surface_mesh with data fields" );
}
//*************************************************************************************************
#endif


//=================================================================================================
//
//  LOW-LEVEL MESH OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes mesh geometry and data fields to a VTK POLYDATA file.
 *
 * \param filename The name of the output VTK file.
 * \param vertices Vector of vertex coordinates.
 * \param faces Vector of face connectivity lists (each face contains vertex indices).
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 * \param integerFields Map of integer field names to vertex data arrays.
 * \param description Description string for the VTK file header.
 *
 * This is the low-level function that performs the actual mesh writing. It supports arbitrary
 * mesh connectivity and multiple data field types. All data field arrays must contain exactly
 * the same number of values as there are vertices.
 */
void MeshDataWriter::writeMeshData( const std::string& filename,
                                   const std::vector<pe::Vec3>& vertices,
                                   const std::vector<std::vector<int>>& faces,
                                   const ScalarFieldMap& scalarFields,
                                   const VectorFieldMap& vectorFields,
                                   const IntegerFieldMap& integerFields,
                                   const std::string& description )
{
   pe_USER_ASSERT( !vertices.empty(), "Vertex array cannot be empty" );
   pe_USER_ASSERT( !faces.empty(), "Face array cannot be empty" );

   const size_t numVertices = vertices.size();

   // Validate field sizes
   for( const auto& field : scalarFields ) {
      pe_USER_ASSERT( field.second.size() == numVertices,
                      "Scalar field '" << field.first << "' has incorrect size" );
   }
   for( const auto& field : vectorFields ) {
      pe_USER_ASSERT( field.second.size() == numVertices,
                      "Vector field '" << field.first << "' has incorrect size" );
   }
   for( const auto& field : integerFields ) {
      pe_USER_ASSERT( field.second.size() == numVertices,
                      "Integer field '" << field.first << "' has incorrect size" );
   }

   // Validate face connectivity
   for( size_t i = 0; i < faces.size(); ++i ) {
      const auto& face = faces[i];
      pe_USER_ASSERT( face.size() >= 3, "Face " << i << " has fewer than 3 vertices" );
      
      for( int vertexIndex : face ) {
         pe_USER_ASSERT( vertexIndex >= 0 && static_cast<size_t>(vertexIndex) < numVertices,
                         "Face " << i << " contains invalid vertex index: " << vertexIndex );
      }
   }

   std::ofstream out( filename );
   if( !out ) {
      pe_THROW( std::runtime_error, "Cannot open output file: " << filename );
   }

   out << std::fixed << std::setprecision(6);

   // Write VTK header
   writeVTKHeader( out, description );

   // Write mesh geometry
   writeVertices( out, vertices );
   writeFaces( out, faces );

   // Write vertex data fields
   writePointData( out, numVertices, scalarFields, vectorFields, integerFields );
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
void MeshDataWriter::writeVTKHeader( std::ostream& out, const std::string& description )
{
   out << "# vtk DataFile Version 3.0\n";
   out << description << "\n";
   out << "ASCII\n";
   out << "DATASET POLYDATA\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes vertex coordinate data.
 *
 * \param out Output stream.
 * \param vertices Vector of vertex coordinates.
 */
void MeshDataWriter::writeVertices( std::ostream& out, const std::vector<pe::Vec3>& vertices )
{
   out << "POINTS " << vertices.size() << " double\n";
   for( const auto& vertex : vertices ) {
      out << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes face connectivity data.
 *
 * \param out Output stream.
 * \param faces Vector of face connectivity lists.
 */
void MeshDataWriter::writeFaces( std::ostream& out, const std::vector<std::vector<int>>& faces )
{
   // Calculate total connectivity size
   size_t connectivitySize = 0;
   for( const auto& face : faces ) {
      connectivitySize += (1 + face.size());  // 1 for vertex count + vertex indices
   }

   out << "POLYGONS " << faces.size() << " " << connectivitySize << "\n";
   for( const auto& face : faces ) {
      out << face.size();
      for( int vertexIndex : face ) {
         out << " " << vertexIndex;
      }
      out << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes point data fields if any are present.
 *
 * \param out Output stream.
 * \param numVertices Number of vertices in the mesh.
 * \param scalarFields Map of scalar field names to vertex data arrays.
 * \param vectorFields Map of vector field names to vertex data arrays.
 * \param integerFields Map of integer field names to vertex data arrays.
 */
void MeshDataWriter::writePointData( std::ostream& out, size_t numVertices,
                                     const ScalarFieldMap& scalarFields,
                                     const VectorFieldMap& vectorFields,
                                     const IntegerFieldMap& integerFields )
{
   const bool hasData = !scalarFields.empty() || !vectorFields.empty() || !integerFields.empty();
   
   if( hasData ) {
      out << "POINT_DATA " << numVertices << "\n";

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


//*************************************************************************************************
/*!\brief Writes a scalar field with real values.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Field data values.
 */
void MeshDataWriter::writeScalarField( std::ostream& out, const std::string& name,
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
void MeshDataWriter::writeScalarField( std::ostream& out, const std::string& name,
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
/*!\brief Writes a vector field.
 *
 * \param out Output stream.
 * \param name Field name.
 * \param data Vector field data.
 */
void MeshDataWriter::writeVectorField( std::ostream& out, const std::string& name,
                                      const std::vector<pe::Vec3>& data )
{
   out << "VECTORS " << name << " double\n";
   for( const auto& vector : data ) {
      out << vector[0] << " " << vector[1] << " " << vector[2] << "\n";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extracts geometry data from a PE TriangleMesh.
 *
 * \param mesh The PE TriangleMesh to extract data from.
 * \param vertices Output vector for vertex coordinates.
 * \param faces Output vector for face connectivity.
 */
void MeshDataWriter::extractMeshData( const TriangleMeshID& mesh,
                                     std::vector<pe::Vec3>& vertices,
                                     std::vector<std::vector<int>>& faces )
{
   vertices.clear();
   faces.clear();

   // Extract vertices
   const size_t numVertices = mesh->getNumVertices();
   vertices.reserve( numVertices );
   
   for( size_t i = 0; i < numVertices; ++i ) {
      vertices.push_back( mesh->getVertex(i) );
   }

   // Extract faces
   const size_t numFaces = mesh->getNumFaces();
   faces.reserve( numFaces );

   for( size_t i = 0; i < numFaces; ++i ) {
      const auto& faceIndices = mesh->getFace(i);
      std::vector<int> face;
      face.reserve( faceIndices.size() );
      
      for( auto index : faceIndices ) {
         face.push_back( static_cast<int>(index) );
      }
      faces.push_back( std::move(face) );
   }
}
//*************************************************************************************************


#ifdef PE_USE_CGAL
//*************************************************************************************************
/*!\brief Extracts geometry data from a CGAL Surface_mesh.
 *
 * \param mesh The CGAL Surface_mesh to extract data from.
 * \param vertices Output vector for vertex coordinates.
 * \param faces Output vector for face connectivity.
 */
void MeshDataWriter::extractCGALMeshData( const CGALSurfaceMesh& mesh,
                                         std::vector<pe::Vec3>& vertices,
                                         std::vector<std::vector<int>>& faces )
{
   vertices.clear();
   faces.clear();

   // Extract vertices
   vertices.reserve( mesh.number_of_vertices() );
   for( auto v : mesh.vertices() ) {
      const auto& point = mesh.point(v);
      vertices.emplace_back( CGAL::to_double(point.x()),
                            CGAL::to_double(point.y()), 
                            CGAL::to_double(point.z()) );
   }

   // Extract faces
   faces.reserve( mesh.number_of_faces() );
   for( auto f : mesh.faces() ) {
      std::vector<int> face;
      
      auto vrange = CGAL::vertices_around_face( mesh.halfedge(f), mesh );
      for( auto v : vrange ) {
         face.push_back( static_cast<int>(v) );
      }
      faces.push_back( std::move(face) );
   }
}
//*************************************************************************************************
#endif

} // namespace vtk

} // namespace pe