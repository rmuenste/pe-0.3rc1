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

#include <cmath>
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
   inline real getMass() const;
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
   static inline real calcMass( real A, real B, real C, real density );
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


//*************************************************************************************************
/*!\brief Returns the mass of the ellipsoid.
 *
 * \return The radius of the sphere.
 */
inline real EllipsoidBase::getMass() const
{
   return mass_;
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
inline real EllipsoidBase::calcMass( real A, real B, real C, real density )
{
   return real(4.0)/real(3.0) * M_PI * A * B * C * density;
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
/*!\brief Calculation of the bounding box of the ellipsoid.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the ellipsoid primitive according to
 * the current position and orientation of the ellipsoid. The half-extent of a rotated ellipsoid
 * along world axis \f$ i \f$ is exact:
 *
 * \f[ e_i = \sqrt{ \sum_j R_{ij}^2 \, r_j^2 } \f]
 *
 * with \f$ r = (A, B, C) \f$ the semi-axes and \f$ R \f$ the body rotation matrix (the extent
 * is the support distance of the ellipsoid in direction \f$ e_i \f$, i.e. the length of the
 * vector \f$ (A R_{i0}, B R_{i1}, C R_{i2}) \f$). Note that the bounding box is increased in all
 * dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity of the
 * ellipsoid are also considered during the collision detection process.
 */
inline void EllipsoidBase::calcBoundingBox()
{
   Vec3 extent;
   for( size_t i=0; i<3; ++i ) {
      const real ex( R_(i,0) * radiusA_ );
      const real ey( R_(i,1) * radiusB_ );
      const real ez( R_(i,2) * radiusC_ );
      extent[i] = std::sqrt( ex*ex + ey*ey + ez*ez ) + contactThreshold;
   }

   aabb_[0] = gpos_[0] - extent[0];
   aabb_[1] = gpos_[1] - extent[1];
   aabb_[2] = gpos_[2] - extent[2];
   aabb_[3] = gpos_[0] + extent[0];
   aabb_[4] = gpos_[1] + extent[1];
   aabb_[5] = gpos_[2] + extent[2];

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
   // Solid ellipsoid: I_ii = (1/5) m (r_j^2 + r_k^2) for all three principal axes.
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
 *
 * The support point of an ellipsoid with semi-axes \f$ (A,B,C) \f$ along its body axes is
 * computed in the body frame from \f$ d_b = R^T d \f$:
 *
 * \f[ v = ( A^2 d_{b,x}, B^2 d_{b,y}, C^2 d_{b,z} ), \quad
 *     p_b = \frac{v}{\sqrt{ A^2 d_{b,x}^2 + B^2 d_{b,y}^2 + C^2 d_{b,z}^2 }} \f]
 *
 * and returned as \f$ g + R\,p_b \f$. The point lies exactly on the surface and its outward
 * normal \f$ (p_x/A^2, p_y/B^2, p_z/C^2) \f$ is parallel to \f$ d_b \f$. A degenerate zero
 * direction yields the center of mass, consistent with the other primitives.
 */
inline Vec3 EllipsoidBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );

   const Vec3 bfD( vectorFromWFtoBF( d ) );  // d in body frame coordinates

   const Vec3 v( radiusA_ * radiusA_ * bfD[0],
                 radiusB_ * radiusB_ * bfD[1],
                 radiusC_ * radiusC_ * bfD[2] );
   const real denom( std::sqrt( trans( v ) * bfD ) );  // sqrt( A^2 dx^2 + B^2 dy^2 + C^2 dz^2 )

   if( denom <= real(0) )
      return gpos_;

   return gpos_ + vectorFromBFtoWF( v / denom );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 *
 * This is the exact support point of the Minkowski sum of the ellipsoid with a ball of radius
 * pe::contactThreshold.
 */
inline Vec3 EllipsoidBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return support( d ) + d * contactThreshold;
}
//*************************************************************************************************

} // namespace pe

#endif
