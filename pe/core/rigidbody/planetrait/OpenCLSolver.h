//=================================================================================================
/*!
 *  \file pe/core/rigidbody/planetrait/OpenCLSolver.h
 *  \brief Specialization of the PlaneTrait class template for the OpenCL solver
 *
 *  Copyright (C) 2009 Klaus Iglberger
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

#ifndef _PE_CORE_RIGIDBODY_PLANETRAIT_OPENCLSOLVER_H_
#define _PE_CORE_RIGIDBODY_PLANETRAIT_OPENCLSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/PlaneBase.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the PlaneTrait class template for the OpenCL solver.
 * \ingroup plane
 *
 * This specialization of the PlaneTrait class template adapts the plane geometry to the
 * requirements of the OpenCL solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class PlaneTrait< C<CD,FD,BG,response::OpenCLSolver> > : public PlaneBase
{
protected:
   //**Type definitions****************************************************************************
   typedef PlaneBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit PlaneTrait( size_t sid, size_t uid, const Vec3& gpos, const Vec3& normal,
                        real d, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~PlaneTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void integrateVelocity( real dt );
   virtual void integratePosition( real dt );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the OpenCLSolver specialization of the PlaneTrait class template.
 *
 * \param sid Unique system-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param normal The plane's normal in reference to the global world frame, \f$ |n| = 1 \f$.
 * \param d The displacement of the plane.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
PlaneTrait< C<CD,FD,BG,response::OpenCLSolver> >::PlaneTrait( size_t sid, size_t uid, const Vec3& gpos, const Vec3& normal,
                           real d, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, normal, d, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the OpenCLSolver specialization of the PlaneTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
PlaneTrait< C<CD,FD,BG,response::OpenCLSolver> >::~PlaneTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Integrate the body velocity by a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * The linear and angular velocity is updated according to the current acting forces and
 * gravity. The forces are reset afterwards. Note that forces and torques never have an
 * influence on the velocity integration of planes due to their infinite nature.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void PlaneTrait< C<CD,FD,BG,response::OpenCLSolver> >::integrateVelocity( real /* dt */ )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Note: Applied forces, torques or gravity never have an influence on planes

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Integrate the body position by a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * The global position and the orientation of the body is updated according to the current
 * body velocity. Additionally the contact graph data structures are cleared, velocity
 * damping is performed, bounding boxes are updated and motion decay is detected.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void PlaneTrait< C<CD,FD,BG,response::OpenCLSolver> >::integratePosition( real dt )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Note: Resetting the contact nodes and removing all attached contacts is not necessary here

   // Move the plane
   if( awake_ )
   {
      // Calculating the translational displacement
      gpos_ += v_ * dt;

      // Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Updating secondary plane description (normal and displacement)
      normal_ = q_.rotate( Vec3( 0, 0, 1 ) );
      d_ = trans(normal_) * gpos_;

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v_ *= drag;
         w_ *= drag;
      }

      // Setting the axis-aligned bounding box
      PlaneBase::calcBoundingBox();

      // Calculating the current motion of the box
      calcMotion();
   }

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
