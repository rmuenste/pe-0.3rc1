//=================================================================================================
/*!
 *  \file pe/core/rigidbody/SphereBase.h
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

#ifndef _PE_CORE_RIGIDBODY_SPHEREBASE_H_
#define _PE_CORE_RIGIDBODY_SPHEREBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/Thresholds.h>
#include <pe/core/lubrication/Params.h>
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
 * The SphereBase class represents the base class for the sphere geometry. It provides
 * the basic functionality of a sphere. For a full description of the sphere geometry,
 * see the Sphere class description.
 */
class SphereBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SphereBase( id_t sid, id_t uid, const Vec3& gpos,
                        real radius, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~SphereBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline real getRadius() const;
   inline real getVolume() const;
   inline real getShellThickness() const;
   inline bool isHollow() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setShellThickness( real thickness );
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
   static inline real calcVolume( real radius );
   static inline real calcVolume( real radius, real shellThickness );
   static inline real calcMass( real radius, real density );
   static inline real calcMass( real radius, real shellThickness, real density );
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
   real radius_;  //!< Radius of the sphere.
                  /*!< The radius is constrained to values larger than 0.0. */
   real shellThickness_;  //!< Wall thickness of a hollow sphere, 0 for a solid sphere.
                          /*!< Only mass and inertia depend on it; the collision geometry is
                               the outer radius in both cases. Constrained to [0, radius]. */
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
 *
 * \return The radius of the sphere.
 */
inline real SphereBase::getRadius() const
{
   return radius_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the radius of the sphere.
 *
 * \return The radius of the sphere.
 */
inline real SphereBase::getVolume() const
{
   return real(4.0)/real(3.0) * M_PI * radius_ * radius_ * radius_;
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
inline real SphereBase::calcVolume( real radius )
{
   return real(4.0)/real(3.0) * M_PI * radius * radius * radius;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the material volume of a (hollow) sphere.
 *
 * \param radius The outer radius of the sphere \f$ (0..\infty) \f$.
 * \param shellThickness The wall thickness \f$ [0..radius] \f$; 0 means solid.
 * \return The volume of the shell, or of the whole ball for a solid sphere.
 */
inline real SphereBase::calcVolume( real radius, real shellThickness )
{
   if( shellThickness <= real(0) || shellThickness >= radius )
      return calcVolume( radius );
   const real inner( radius - shellThickness );
   return real(4.0)/real(3.0) * M_PI * ( radius * radius * radius - inner * inner * inner );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a (hollow) sphere for a given radius, wall thickness and density.
 *
 * \param radius The outer radius of the sphere \f$ (0..\infty) \f$.
 * \param shellThickness The wall thickness \f$ [0..radius] \f$; 0 means solid.
 * \param density The density of the wall material.
 * \return The mass of the shell, or of the whole ball for a solid sphere.
 */
inline real SphereBase::calcMass( real radius, real shellThickness, real density )
{
   return calcVolume( radius, shellThickness ) * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the wall thickness of a hollow sphere (0 for a solid sphere).
 *
 * \return The wall thickness.
 */
inline real SphereBase::getShellThickness() const
{
   return shellThickness_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the sphere is a hollow shell.
 *
 * \return \a true if the wall thickness is positive and smaller than the radius.
 */
inline bool SphereBase::isHollow() const
{
   return shellThickness_ > real(0) && shellThickness_ < radius_;
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Calculates the mass of a sphere for a given radius and density.
 *
 * \param radius The radius of the sphere.
 * \param density The density of the sphere.
 * \return The total mass of the sphere.
 */
inline real SphereBase::calcMass( real radius, real density )
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
inline real SphereBase::calcDensity( real radius, real mass )
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
inline void SphereBase::calcBoundingBox()
{
   const real length( radius_ + contactThreshold + lubrication::aabbPadding( radius_ ) );

   aabb_[0] = gpos_[0] - length;
   aabb_[1] = gpos_[1] - length;
   aabb_[2] = gpos_[2] - length;
   aabb_[3] = gpos_[0] + length;
   aabb_[4] = gpos_[1] + length;
   aabb_[5] = gpos_[2] + length;

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the sphere.
 *
 * \return void
 */
inline void SphereBase::calcInertia()
{
   // Solid ball: (2/5) m R^2. Shell with inner radius r: (2/5) m (R^5 - r^5) / (R^3 - r^3),
   // which tends to (2/3) m R^2 for a thin wall.
   real factor( radius_ * radius_ );
   if( isHollow() ) {
      const real R( radius_ ), r( radius_ - shellThickness_ );
      const real R3( R * R * R ), r3( r * r * r );
      factor = ( R3 * R * R - r3 * r * r ) / ( R3 - r3 );
   }
   I_[0] = I_[4] = I_[8] = real(0.4) * mass_ * factor;
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
inline Vec3 SphereBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return gpos_ + radius_*d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
inline Vec3 SphereBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return gpos_ + d*(radius_ + contactThreshold);
}
//*************************************************************************************************

} // namespace pe

#endif
