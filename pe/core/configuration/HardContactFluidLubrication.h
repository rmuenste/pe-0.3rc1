//=================================================================================================
/*!
 *  \file pe/core/configuration/HardContactFluidLubrication.h
 *  \brief Configuration for the hard contact solver with lubrication forces
 *
 *  Copyright (C) 2023 Raphael Münster
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

#ifndef _PE_CORE_CONFIGURATION_HARDCONTACTFLUIDLUBRICATION_H_
#define _PE_CORE_CONFIGURATION_HARDCONTACTFLUIDLUBRICATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/config/Collisions.h>
#include <pe/config/HashGrids.h>
#include <pe/config/Precision.h>
#include <pe/config/SleepMode.h>
#include <pe/core/attachable/AttachableType.h>
#include <pe/core/batches/Batches.h>
#include <pe/core/batches/Types.h>
#include <pe/core/contact/ContactBase.h>
#include <pe/core/ContactType.h>
#include <pe/core/detection/Detection.h>
#include <pe/core/joint/JointType.h>
#include <pe/core/response/BodyTrait.h>
#include <pe/core/response/ContactTrait.h>
#include <pe/core/response/HardContactFluidLubrication.h>
#include <pe/core/response/JointTrait.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/MPIRigidBodyTrait.h>



namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Configuration template for the HardContactFluidLubrication collision response solver.
 * \ingroup collision_response
 *
 * The HardContactFluidLubricationConfig class template provides the setup and configuration for
 * the HardContactFluidLubrication collision response solver. It provides the necessary functions
 * for the handling of contacts, is equipped with a coarse collision detection algorithm, a fine
 * collision detection algorithm and a batch generator, and provides the coupling with the
 * available body functionality.
 */
template< template<typename> class CD                      // Type of the coarse collision detection algorithm
        , typename FD                                      // Type of the fine collision detection algorithm
        , template<typename> class BG                      // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR >  // Type of the collision response algorithm
struct HardContactFluidLubricationConfig
{
   //**Type definitions****************************************************************************
   //! Type of the configuration (Curiously recurring template pattern).
   typedef HardContactFluidLubricationConfig<CD,FD,BG,CR>  ThisType;

   //! Type of the body traits.
   /*! For the HardContactFluidLubrication collision response solver the BodyTraits class from the
       response module is chosen, which provides all necessary functionality for the relaxation
       of hard contacts. */
   typedef response::BodyTraits<ThisType>  BodyTraits;

   //! Type of the contact traits.
   /*! For the HardContactFluidLubrication algorithm the ContactTraits class from the response module is
       chosen, which provides all necessary functionality of contacts for the relaxation of
       hard contacts. */
   typedef response::ContactTraits<ThisType>  ContactTraits;

   //! Type of the joint traits.
   /*! For the Lemke algorithm the JointTraits class from the response module is chosen,
       which provides all necessary functionality of joints for the relaxation of non-interpenetration
       constraints and joints. */
   typedef response::JointTraits<ThisType>  JointTraits;

   //! Type of the body type.
   /*! This definition represents the base type of all rigid bodies handled by this configuration. */
   typedef typename BodyTraits::BodyType  BodyType;

   //! Type of a body handle.
   /*! This definition represents a handle to a rigid body of type BodyType. */
   typedef typename BodyTraits::BodyID  BodyID;

   //! Type of a constant body handle.
   /*! This definition represents a constant handle to a rigid body of type BodyType. */
   typedef typename BodyTraits::ConstBodyID  ConstBodyID;

   //! Type of a handle to a remote rigid body.
   /*! This definition represents a handle to a remote rigid body. The concrete shadow copy
       is directly accessible via this handle. */
   typedef MPIRigidBodyTrait<BodyTraits>  MPIBodyTrait;

   //! Type of the attachable type.
   /*! This definition represents the base type of all attachables handled by this configuration. */
   typedef AttachableType<ThisType>  AttachableType;

   //! Type of an attachable handle.
   /*! This definition represents a handle to an attachable of type AttachableType. */
   typedef typename AttachableType::AttachableID  AttachableID;

   //! Type of a constant attachable handle.
   /*! This definition represents a constant handle to an attachable of type AttachableType. */
   typedef typename AttachableType::ConstAttachableID  ConstAttachableID;

   //! Type of the joint type.
   /*! This definition represents the base type of all joints handled by this configuration. */
   typedef JointType<ThisType>  JointType;

   //! Type of a joint handle.
   /*! This definition represents a handle to a joint of type JointType. */
   typedef typename JointType::JointID  JointID;

   //! Type of a constant joint handle.
   /*! This definition represents a constant handle to a joint of type JointType. */
   typedef typename JointType::ConstJointID  ConstJointID;

   //! Type of the contact type.
   /*! This definition represents the base type of all contacts handled by this configuration. */
   typedef ContactType<ThisType>  ContactType;

   //! Type of a contact handle.
   /*! This definition represents a handle to a contact of type ContactType. */
   typedef typename ContactType::ContactID  ContactID;

   //! Type of a constant contact handle.
   /*! This definition represents a constant handle to a contact of type ContactType. */
   typedef typename ContactType::ConstContactID  ConstContactID;

   //! Type of the process type.
   /*! This definition represents the base type of all remote processes handled by this
       configuration. */
   typedef Process  ProcessType;

   //! Type of a process handle.
   /*! This definition represents a handle to a remote process of type ProcessType. */
   typedef typename ProcessType::ProcessID  ProcessID;

   //! Type of a constant process handle.
   /*! This definition represents a constant handle to a remote process of type ProcessType. */
   typedef typename ProcessType::ConstProcessID  ConstProcessID;

   //! Type of the batch type.
   /*! This definition represents the base type of all batches handled by this configuration. */
   typedef BatchType<ThisType>  BatchType;

   //! Type of a batch handle.
   /*! This definition represents a handle to a batch of type BatchType. */
   typedef typename BatchType::BatchID  BatchID;

   //! Type of a constant batch handle.
   /*! This definition represents a constant handle to a batch of type BatchType. */
   typedef typename BatchType::ConstBatchID  ConstBatchID;
   //**********************************************************************************************
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Specialization of HardContactFluidLubricationConfig for the HashGrids coarse collision detection.
 */
typedef HardContactFluidLubricationConfig< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactFluidLubrication >  HCFLConfig;
//*************************************************************************************************

} // namespace pe

#endif