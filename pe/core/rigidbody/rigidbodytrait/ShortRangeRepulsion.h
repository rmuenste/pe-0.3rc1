//=================================================================================================
/*!
 *  \file pe/core/rigidbody/rigidbodytrait/ShortRangeRepulsion.h
 *  \brief Specialization of the RigidBodyTrait class template for the ShortRangeRepulsion solver.
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_SHORTRANGEREPULSION_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_SHORTRANGEREPULSION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/MPIRigidBodyTrait.h>
#include <pe/core/response/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the RigidBodyTrait class template for the ShortRangeRepulsion solver.
// \ingroup rigid_body
//
// This specialization inherits from MPIRigidBodyTrait to provide the MPI ownership/shadow-copy
// bookkeeping needed by the ShortRangeRepulsion collision system specialization.
*/
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class RigidBodyTrait< C<CD,FD,BG,response::ShortRangeRepulsion> > : public MPIRigidBodyTrait
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBodyTrait();
   //@}
   //**********************************************************************************************

public:
   // Diagnostic flag required by the PE-FeatFloWer interface (object_queries.cpp).
   // Always false for ShortRangeRepulsion (no stuck-particle detection in this solver).
   bool isStuck_ = false;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for RigidBodyTrait<ShortRangeRepulsion>.
//
// \param body The body ID of this rigid body.
*/
template< template<typename> class CD
        , typename FD
        , template<typename> class BG
        , template< template<typename> class
                  , typename
                  , template<typename> class
                  , template<typename,typename,typename> class
                  > class C >
RigidBodyTrait< C<CD,FD,BG,response::ShortRangeRepulsion> >::RigidBodyTrait( BodyID body )
   : MPIRigidBodyTrait( body )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destructor for RigidBodyTrait<ShortRangeRepulsion>.
*/
template< template<typename> class CD
        , typename FD
        , template<typename> class BG
        , template< template<typename> class
                  , typename
                  , template<typename> class
                  , template<typename,typename,typename> class
                  > class C >
RigidBodyTrait< C<CD,FD,BG,response::ShortRangeRepulsion> >::~RigidBodyTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
