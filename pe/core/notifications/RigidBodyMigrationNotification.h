//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyMigrationNotification.h
 *  \brief Header file for the RigidBodyMigrationNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYMIGRATIONNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYMIGRATIONNOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body migration notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyMigrationNotification class is a wrapper class for marshaling and unmarshaling
 * rigid body migration notifications. When receiving a migration notification this indicates that
 * a body migrates from the sending neighbor to the local process and the local process takes over
 * ownership of the body. Migration notices may only be sent if the new owner already obtained a
 * shadow copy previously. They may also only be sent by a neighboring process.
 */
class RigidBodyMigrationNotification {
public:
   struct Parameters {
      id_t sid_;
      std::vector<int> reglist_;
   };

   inline RigidBodyMigrationNotification( const RigidBody& b, const std::vector<int>& reglist ) : body_(b), reglist_(reglist) {}
   const RigidBody& body_;
   const std::vector<int>& reglist_;
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Marshaling rigid body migration notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The migration notifications just consists of the system ID of the body migrating and the
 * registration list which contains ranks of all processes currently having a shadow copy of the
 * migrating body.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyMigrationNotification& obj ) {
   // TODO fixed sizes

   buffer << obj.body_.getSystemID();
   buffer << obj.reglist_.size();
   for( std::size_t i = 0; i < obj.reglist_.size(); ++i )
      buffer << obj.reglist_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body migration notifications.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The migration notifications just consists of the system ID of the body migrating and the
 * registration list which contains ranks of all processes currently having a shadow copy of the
 * migrating body.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyMigrationNotification::Parameters& objparam ) {
   // TODO fixed sizes

   buffer >> objparam.sid_;
   std::size_t n;
   buffer >> n;
   objparam.reglist_.resize( n );
   for( std::size_t i = 0; i < n; ++i )
      buffer >> objparam.reglist_[i];
}
//*************************************************************************************************


} // namespace pe

#endif
