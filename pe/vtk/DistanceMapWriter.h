//=================================================================================================
/*!
 *  \file pe/vtk/DistanceMapWriter.h
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

#ifndef _PE_VTK_DISTANCEMAPWRITER_H_
#define _PE_VTK_DISTANCEMAPWRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <vector>
#include <map>
#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>

#ifdef PE_USE_CGAL
#include <pe/core/detection/fine/DistanceMap.h>
#endif


namespace pe {

namespace vtk {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief VTK structured grid writer for DistanceMap visualization.
 * \ingroup vtk
 *
 * The DistanceMapWriter class provides functionality to export DistanceMap data as VTK Image Data
 * (.vti) files for visualization in ParaView and other VTK-compatible tools. This writer creates
 * structured 3D grids with scalar and vector fields representing signed distance values, alpha
 * (containment) values, surface normals, and contact points.
 *
 * The class follows PE design patterns as a utility writer - it provides direct-use functions
 * that can be called explicitly when immediate output is needed, complementing the existing
 * simulation-integrated Writer class.
 *
 * Example usage:
 * \code
 * // Write complete DistanceMap data to VTI file
 * pe::vtk::DistanceMapWriter::writeVTI("distance_field.vti", distanceMap);
 *
 * // Write custom grid data with specific fields
 * std::map<std::string, std::vector<pe::real>> scalarFields;
 * scalarFields["sdf"] = sdfData;
 * scalarFields["alpha"] = alphaData;
 * pe::vtk::DistanceMapWriter::writeCustomGrid("custom.vti", origin, spacing, 
 *                                            nx, ny, nz, scalarFields);
 * \endcode
 */
class DistanceMapWriter
{
public:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef std::map<std::string, std::vector<pe::real>>  ScalarFieldMap;   //!< Map of scalar field names to data
   typedef std::map<std::string, std::vector<pe::Vec3>>  VectorFieldMap;   //!< Map of vector field names to data
   /*! \endcond */
   //**********************************************************************************************

#ifdef PE_USE_CGAL
   //**DistanceMap output functions****************************************************************
   /*!\name DistanceMap output functions */
   //@{
   static void writeVTI( const std::string& filename, const DistanceMap& distanceMap );
   static void writeVTI( const std::string& filename, const DistanceMap& distanceMap, 
                         bool includeSDF, bool includeAlpha, bool includeNormals, bool includeContactPoints );
   //@}
   //**********************************************************************************************
#endif

   //**Custom grid output functions****************************************************************
   /*!\name Custom grid output functions */
   //@{
   static void writeCustomGrid( const std::string& filename,
                               const pe::Vec3& origin,
                               const pe::Vec3& spacing,
                               int nx, int ny, int nz,
                               const ScalarFieldMap& scalarFields );

   static void writeCustomGrid( const std::string& filename,
                               const pe::Vec3& origin,
                               const pe::Vec3& spacing, 
                               int nx, int ny, int nz,
                               const ScalarFieldMap& scalarFields,
                               const VectorFieldMap& vectorFields );
   //@}
   //**********************************************************************************************

   //**Low-level output functions******************************************************************
   /*!\name Low-level output functions */
   //@{
   static void writeVTI( const std::string& filename,
                        const std::vector<pe::real>& sdf,
                        const std::vector<int>& alpha,
                        const std::vector<pe::Vec3>& normals,
                        const std::vector<pe::Vec3>& contactPoints,
                        int nx, int ny, int nz,
                        pe::real dx, pe::real dy, pe::real dz,
                        pe::real x0, pe::real y0, pe::real z0 );

   static void writeVTI( const std::string& filename,
                        const std::vector<pe::real>& sdf,
                        const std::vector<int>& alpha,
                        const std::vector<pe::Vec3>& normals,
                        const std::vector<pe::Vec3>& contactPoints,
                        const std::vector<int>& faceIndex,
                        int nx, int ny, int nz,
                        pe::real dx, pe::real dy, pe::real dz,
                        pe::real x0, pe::real y0, pe::real z0 );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static void writeVTIHeader( std::ostream& out, 
                              const pe::Vec3& origin, const pe::Vec3& spacing,
                              int nx, int ny, int nz );
   static void writeVTIFooter( std::ostream& out );
   static void writeScalarField( std::ostream& out, const std::string& name, 
                                const std::vector<pe::real>& data );
   static void writeScalarField( std::ostream& out, const std::string& name,
                                const std::vector<int>& data );
   static void writeVectorField( std::ostream& out, const std::string& name,
                                const std::vector<pe::Vec3>& data, int nx, int ny, int nz );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace vtk

} // namespace pe

#endif /* _PE_VTK_DISTANCEMAPWRITER_H_ */