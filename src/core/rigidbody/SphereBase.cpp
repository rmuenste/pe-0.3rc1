//=================================================================================================
/*!
 *  \file src/core/rigidbody/SphereBase.cpp
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
#include <stdexcept>
#include <pe/core/rigidbody/SphereBase.h>
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
/*!\brief Constructor for the SphereBase class.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 */
SphereBase::SphereBase( id_t sid, id_t uid, const Vec3& gpos,
                        real radius, MaterialID material, bool visible )
   : GeomPrimitive( sphereType, true, visible, sid, uid, material )  // Initializing the base object
   , radius_( radius )                                               // Radius of the sphere
   , shellThickness_( 0 )                                            // Solid sphere
{
   // Checking the radius
   // Since the sphere constructor is never directly called but only used in a small number
   // of functions that already check the sphere arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid sphere radius" );

   // Setting the center of the sphere
   gpos_ = gpos;

   // Calculating the sphere mass
   mass_ = calcMass( radius, Material::getDensity( material ) );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Registering the radius for lubrication AABB padding of radius-less bodies (planes)
   lubrication::registerSphereRadius( radius );

   // Setting the axis-aligned bounding box
   SphereBase::calcBoundingBox();
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the SphereBase class.
 */
SphereBase::~SphereBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Makes the sphere a hollow shell of the given wall thickness (0: solid).
 *
 * \param thickness The wall thickness \f$ [0..radius] \f$; 0 restores the solid sphere.
 * \return void
 * \exception std::invalid_argument Thickness negative or larger than the radius.
 *
 * Only the mass and the moment of inertia change: the mass is the wall volume times the
 * material density and the inertia (2/5) m (R^5 - r^5) / (R^3 - r^3), which tends to the
 * thin-shell value (2/3) m R^2. The collision geometry, the contact generation and the
 * bounding box are those of the solid sphere with the same outer radius. A thickness equal
 * to the radius is a solid sphere.
 *
 * Fixed spheres keep their infinite inverse mass and inertia. Call this before adding the
 * sphere to a union: a union does not re-read the mass and inertia of its parts afterwards.
 * The thickness is marshalled with the sphere, so remote (shadow) copies and checkpoints carry
 * it; checkpoint files written before this attribute (format 0.2) read back as solid spheres.
 */
void SphereBase::setShellThickness( real thickness )
{
   if( thickness < real(0) || thickness > radius_ )
      throw std::invalid_argument( "Invalid sphere shell thickness" );

   shellThickness_ = ( thickness >= radius_ ) ? real(0) : thickness;

   const bool translationFree( invMass_ != real(0) );
   const bool rotationFree( Iinv_[0] != real(0) );

   mass_ = calcMass( radius_, shellThickness_, Material::getDensity( material_ ) );
   calcInertia();   // also sets Iinv_ = inverse( I_ )

   invMass_ = translationFree ? real(1) / mass_ : real(0);
   if( !rotationFree )
      Iinv_ = real(0);
}
//*************************************************************************************************

} // namespace pe
