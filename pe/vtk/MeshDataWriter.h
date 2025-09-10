//=================================================================================================
/*!
 *  \file pe/vtk/MeshDataWriter.h
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

#ifndef _PE_VTK_MESHDATAWRITER_H_
#define _PE_VTK_MESHDATAWRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <vector>
#include <map>
#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>
#include <pe/core/rigidbody/TriangleMesh.h>

#ifdef PE_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#endif


namespace pe {

namespace vtk {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief VTK mesh writer with enhanced data field support.
 * \ingroup vtk
 *
 * The MeshDataWriter class provides functionality to export triangle meshes with associated
 * scalar and vector data as VTK POLYDATA files for visualization in ParaView and other 
 * VTK-compatible tools. This writer supports both PE TriangleMesh objects and CGAL Surface_mesh
 * objects (when CGAL is available) and allows attachment of arbitrary data fields to vertices.
 *
 * The class follows PE design patterns as a utility writer - it provides direct-use functions
 * that can be called explicitly when immediate output is needed, complementing the existing
 * simulation-integrated Writer class.
 *
 * Example usage:
 * \code
 * // PE TriangleMesh with scalar data
 * std::map<std::string, std::vector<pe::real>> scalarFields;
 * scalarFields["distance"] = distanceValues;
 * scalarFields["alpha"] = alphaValues;
 * pe::vtk::MeshDataWriter::writeMesh("mesh_analysis.vtk", triangleMesh, scalarFields);
 *
 * // CGAL Surface_mesh with multiple data types (if CGAL available)
 * #ifdef PE_USE_CGAL
 * pe::vtk::MeshDataWriter::writeCGALMesh("cgal_mesh.vtk", surfaceMesh, 
 *                                       scalarFields, vectorFields);
 * #endif
 * \endcode
 */
class MeshDataWriter
{
public:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef std::map<std::string, std::vector<pe::real>>  ScalarFieldMap;   //!< Map of scalar field names to data
   typedef std::map<std::string, std::vector<pe::Vec3>>  VectorFieldMap;   //!< Map of vector field names to data  
   typedef std::map<std::string, std::vector<int>>       IntegerFieldMap;  //!< Map of integer field names to data
   /*! \endcond */
   //**********************************************************************************************

#ifdef PE_USE_CGAL
   /*! \cond PE_INTERNAL */
   typedef CGAL::Exact_predicates_inexact_constructions_kernel  CGALKernel;    //!< CGAL kernel type
   typedef CGALKernel::Point_3                                  CGALPoint;     //!< CGAL point type
   typedef CGAL::Surface_mesh<CGALPoint>                        CGALSurfaceMesh;  //!< CGAL surface mesh type
   /*! \endcond */
#endif

   //**PE TriangleMesh output functions************************************************************
   /*!\name PE TriangleMesh output functions */
   //@{
   static void writeMesh( const std::string& filename,
                         const TriangleMeshID& mesh );

   static void writeMesh( const std::string& filename,
                         const TriangleMeshID& mesh,
                         const ScalarFieldMap& scalarFields );

   static void writeMesh( const std::string& filename,
                         const TriangleMeshID& mesh,
                         const ScalarFieldMap& scalarFields,
                         const VectorFieldMap& vectorFields );

   static void writeMesh( const std::string& filename,
                         const TriangleMeshID& mesh,
                         const ScalarFieldMap& scalarFields,
                         const VectorFieldMap& vectorFields,
                         const IntegerFieldMap& integerFields );
   //@}
   //**********************************************************************************************

#ifdef PE_USE_CGAL
   //**CGAL Surface_mesh output functions**********************************************************
   /*!\name CGAL Surface_mesh output functions */
   //@{
   static void writeCGALMesh( const std::string& filename,
                             const CGALSurfaceMesh& mesh );

   static void writeCGALMesh( const std::string& filename,
                             const CGALSurfaceMesh& mesh,
                             const ScalarFieldMap& scalarFields );

   static void writeCGALMesh( const std::string& filename,
                             const CGALSurfaceMesh& mesh,
                             const ScalarFieldMap& scalarFields,
                             const VectorFieldMap& vectorFields );

   static void writeCGALMesh( const std::string& filename,
                             const CGALSurfaceMesh& mesh,
                             const ScalarFieldMap& scalarFields,
                             const VectorFieldMap& vectorFields,
                             const IntegerFieldMap& integerFields );
   //@}
   //**********************************************************************************************
#endif

   //**Low-level mesh output functions*************************************************************
   /*!\name Low-level mesh output functions */
   //@{
   static void writeMeshData( const std::string& filename,
                             const std::vector<pe::Vec3>& vertices,
                             const std::vector<std::vector<int>>& faces,
                             const ScalarFieldMap& scalarFields,
                             const VectorFieldMap& vectorFields,
                             const IntegerFieldMap& integerFields,
                             const std::string& description = "Mesh data" );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static void writeVTKHeader( std::ostream& out, const std::string& description );
   static void writeVertices( std::ostream& out, const std::vector<pe::Vec3>& vertices );
   static void writeFaces( std::ostream& out, const std::vector<std::vector<int>>& faces );
   static void writePointData( std::ostream& out, size_t numVertices,
                              const ScalarFieldMap& scalarFields,
                              const VectorFieldMap& vectorFields,
                              const IntegerFieldMap& integerFields );
   static void writeScalarField( std::ostream& out, const std::string& name,
                                const std::vector<pe::real>& data );
   static void writeScalarField( std::ostream& out, const std::string& name,
                                const std::vector<int>& data );
   static void writeVectorField( std::ostream& out, const std::string& name,
                                const std::vector<pe::Vec3>& data );
   
   // PE TriangleMesh extraction utilities
   static void extractMeshData( const TriangleMeshID& mesh,
                               std::vector<pe::Vec3>& vertices,
                               std::vector<std::vector<int>>& faces );

#ifdef PE_USE_CGAL
   // CGAL Surface_mesh extraction utilities
   static void extractCGALMeshData( const CGALSurfaceMesh& mesh,
                                   std::vector<pe::Vec3>& vertices,
                                   std::vector<std::vector<int>>& faces );
#endif
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace vtk

} // namespace pe

#endif /* _PE_VTK_MESHDATAWRITER_H_ */