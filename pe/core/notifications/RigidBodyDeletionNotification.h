//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyDeletionNotification.h
 *  \brief Header file for the RigidBodyDeletionNotification class
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYDELETIONNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYDELETIONNOTIFICATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBody.h>



namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Wrapper class for rigid body deletion notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyDeletionNotification class is a wrapper class for marshaling and unmarshaling rigid body
 * deletion notifications. When receiving a deletion notification this indicates that the body
 * was deleted (e.g. due to outflow or manual deletion). Thus the shadow copy is no longer valid
 * and must be removed from the receiving process. A deletion notice must only be sent by the owner
 * to a receiver having a shadow copy of the concerned body. On deletion attachments to other
 * bodies must be detached.
 */
class RigidBodyDeletionNotification {
public:
   struct Parameters {
      id_t sid_;
   };

   inline RigidBodyDeletionNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Marshaling rigid body deletion notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The deletion notifications just consists of the system ID of the body to delete.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyDeletionNotification& obj ) {
   buffer << obj.body_.getSystemID();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body deletion notifications.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The deletion notifications just consists of the system ID of the body to delete.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyDeletionNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
}
//*************************************************************************************************

} // namespace pe

#endif
