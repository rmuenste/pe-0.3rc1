//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/HalfSpace.h
 *  \brief Header file for the HalfSpace class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
 *                2025 Raphael Münster
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

#ifndef _PE_CORE_DOMAINDECOMP_TRIMESHBOUNDARY_H_
#define _PE_CORE_DOMAINDECOMP_TRIMESHBOUNDARY_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <vector>
#include <array>

namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!
 * \brief Implementation of a closed triangle mesh for specifying subdomains.
 * 
 * The ClosedSurfaceTriangulation class represents a closed surface triangulation that can perform various intersection tests with spheres, boxes, capsules, and cylinders. It also provides functionality to 
determine the distance from a point to the center of these objects and check if a given point is inside the surface triangulation.
 * 
 * \ingroup domaindecomp
 */

class TriMeshBoundary : public ProcessGeometry
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   TriMeshBoundary(const std::vector<Vec3>& vertices, const std::vector<std::array<int, 3>>& triangles);
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

    // Implement the pure virtual functions from ProcessGeometry
    //**Utility functions***************************************************************************
    /*!\name Utility functions */
    //@{
    bool intersectsWith(ConstBodyID b) const override;
    bool intersectsWith(ConstSphereID s) const override;
    bool intersectsWith(ConstBoxID b) const override;
    bool intersectsWith(ConstCapsuleID c) const override;
    bool intersectsWith(ConstCylinderID c) const override;
    bool intersectsWith(ConstUnionID u) const override;
    bool containsPoint(const Vec3& gpos) const override;
    bool containsPointStrictly(const Vec3& gpos) const override;
    //@}
    //**********************************************************************************************

    //**Output functions****************************************************************************
    /*!\name Output functions */
    //@{
    void print( std::ostream& os                  ) const;
    void print( std::ostream& os, const char* tab ) const;
    //@}
    //**********************************************************************************************

    // Additional utility functions specific to TriMeshBoundary
    std::vector<Vec3> getVertices() const;
    std::vector<std::array<int, 3>> getTriangles() const;

private:
    std::vector<Vec3> vertices_;
    std::vector<std::array<int, 3>> triangles_;
};
//*************************************************************************************************

} // namespace pe

#endif
