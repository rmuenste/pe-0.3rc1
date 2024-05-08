//=================================================================================================
/*!
 *  \file pe/core/rigidbody/EllipsoidBase.h
 *  \brief Base class for the sphere geometry
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_RIGIDBODY_ELLIPSOIDBASE_H_
#define _PE_CORE_RIGIDBODY_ELLIPSOIDBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/math/Constants.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for the sphere geometry.
 * \ingroup sphere
 *
 * The EllipsoidBase class represents the base class for the sphere geometry. It provides
 * the basic functionality of a sphere. For a full description of the sphere geometry,
 * see the Ellipsoid class description.
 */
class EllipsoidBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit EllipsoidBase( id_t sid, id_t uid, const Vec3& gpos,
                        real a, real b, real c,
                        MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~EllipsoidBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline Vec3 getRadius() const;
   inline real getVolume() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline virtual Vec3 support( const Vec3& d ) const;
   inline virtual Vec3 supportContactThreshold( const Vec3& d ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Volume, mass and density functions**********************************************************
   /*!\name Volume, mass and density functions */
   //@{
   static inline real calcVolume( real A, real B, real C );
   static inline real calcMass( real radius, real density );
   static inline real calcDensity( real radius, real mass );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline virtual void calcBoundingBox();  // Calculation of the axis-aligned bounding box
   void inline calcInertia();      // Calculation of the moment of inertia
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   real radiusA_;  //!< Radius A of the ellipsoid.
   real radiusB_;  //!< Radius B of the ellipsoid.
   real radiusC_;  //!< Radius C of the ellipsoid.
                  /*!< The radius is constrained to values larger than 0.0. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the radius of the sphere.
 * This code defines a method getRadius in the EllipsoidBase class 
 * that returns the radius of an ellipsoid 
 * as a Vec3 object containing the values of radiusA_, radiusB_, and radiusC_.
 *
 * \return The radius of the sphere.
 */
inline Vec3 EllipsoidBase::getRadius() const
{
   return Vec3(radiusA_, radiusB_, radiusC_);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the radius of the sphere.
 *
 * \return The radius of the sphere.
 */
inline real EllipsoidBase::getVolume() const
{
   return real(4.0)/real(3.0) * M_PI * radiusA_ * radiusB_ * radiusC_;
}
//*************************************************************************************************


//=================================================================================================
//
//  VOLUME, MASS AND DENSITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the volume of a sphere for a given radius.
 *
 * \param radius The radius of the sphere.
 * \return The volume of the sphere.
 */
inline real EllipsoidBase::calcVolume( real A, real B, real C )
{
   return real(4.0)/real(3.0) * M_PI * A * B * C;

}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a sphere for a given radius and density.
 *
 * \param radius The radius of the sphere.
 * \param density The density of the sphere.
 * \return The total mass of the sphere.
 */
inline real EllipsoidBase::calcMass( real radius, real density )
{
   return real(4.0)/real(3.0) * M_PI * radius * radius * radius * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a sphere for a given radius and mass.
 *
 * \param radius The radius of the sphere.
 * \param mass The total mass of the sphere.
 * \return The density of the sphere.
 */
inline real EllipsoidBase::calcDensity( real radius, real mass )
{
   return real(0.75) * mass / ( M_PI * radius * radius * radius );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the bounding box of the sphere.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the sphere primitive according to the
 * current position and orientation of the sphere. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity of
 * the sphere are also considered during the collision detection process.
 */
inline void EllipsoidBase::calcBoundingBox()
{

   // Create the axis-aligned bounding box in body frame
   Vec3 aabb[6];
   aabb[0] = Vec3(radiusA_ + contactThreshold, 0, 0);
   aabb[1] = Vec3(0, radiusB_ + contactThreshold, 0);
   aabb[2] = Vec3(0, 0, radiusC_ + contactThreshold);
   aabb[3] = Vec3(radiusA_ + contactThreshold, 0, 0);
   aabb[4] = Vec3(0, radiusB_ + contactThreshold, 0);
   aabb[5] = Vec3(0, 0, radiusC_ + contactThreshold);

   real minX = std::numeric_limits<real>::max();
   real minY = std::numeric_limits<real>::max();
   real minZ = std::numeric_limits<real>::max();
   real maxX = std::numeric_limits<real>::min();
   real maxY = std::numeric_limits<real>::min();
   real maxZ = std::numeric_limits<real>::min();

   // Transform the axis-aligned bounding box to WF
   for(int i(0); i < 6; ++i) {
      Vec3 v( aabb[i] );
      v = gpos_ + (R_ * v );
      if(v[0] < minX) {minX = v[0];}
      if(v[1] < minY) {minY = v[1];}
      if(v[2] < minZ) {minZ = v[2];}

      if(v[0] > maxX) {maxX = v[0];}
      if(v[1] > maxY) {maxY = v[1];}
      if(v[2] > maxZ) {maxZ = v[2];}
   }

   aabb_[0] = minX;
   aabb_[1] = minY;
   aabb_[2] = minZ;
   aabb_[3] = maxX;
   aabb_[4] = maxY;
   aabb_[5] = maxZ;

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the sphere.
 *
 * \return void
 */
inline void EllipsoidBase::calcInertia()
{
   I_[0] = real(0.2) * mass_ *(radiusB_ * radiusB_ + radiusC_ * radiusC_);
   I_[4] = real(0.2) * mass_ *(radiusA_ * radiusA_ + radiusC_ * radiusC_);
   I_[8] = real(0.2) * mass_ *(radiusB_ * radiusB_ + radiusA_ * radiusA_);
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
inline Vec3 EllipsoidBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return gpos_ + radiusA_*d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
inline Vec3 EllipsoidBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return gpos_ + d*(radiusA_ + contactThreshold);
}
//*************************************************************************************************

} // namespace pe

#endif
