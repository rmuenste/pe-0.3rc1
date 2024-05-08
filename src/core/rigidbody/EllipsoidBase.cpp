//=================================================================================================
/*!
 *  \file src/core/rigidbody/EllipsoidBase.cpp
 *  \brief Base class for the sphere geometry
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/EllipsoidBase.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/RotationMatrix.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the EllipsoidBase class.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 */
EllipsoidBase::EllipsoidBase( id_t sid, id_t uid, const Vec3& gpos,
                        real a, real b, real c, MaterialID material, bool visible )
   : GeomPrimitive( ellipsoidType, true, visible, sid, uid, material )  // Initializing the base object
   , radiusA_( a )                                               // Radius of the sphere
   , radiusB_( b )                                               // Radius of the sphere
   , radiusC_( c )                                               // Radius of the sphere
{
   // Checking the radius
   // Since the sphere constructor is never directly called but only used in a small number
   // of functions that already check the sphere arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( a > real(0), "Invalid a radius" );
   pe_INTERNAL_ASSERT( b > real(0), "Invalid b radius" );
   pe_INTERNAL_ASSERT( c > real(0), "Invalid c radius" );

   // Setting the center of the sphere
   gpos_ = gpos;

   // Calculating the sphere mass
   mass_ = calcMass( radiusA_, Material::getDensity( material ) );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   EllipsoidBase::calcBoundingBox();
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the EllipsoidBase class.
 */
EllipsoidBase::~EllipsoidBase()
{}
//*************************************************************************************************

} // namespace pe
