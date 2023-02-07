//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyCopyNotification.h
 *  \brief Header file for the RigidBodyCopyNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYCOPYNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYCOPYNOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body copies.
 * \ingroup rigid_body
 *
 * The RigidBodyCopyNotification class is a wrapper class for marshaling and unmarshaling rigid body
 * copies. When receiving a copy notification a new shadow copy of the encapsulated body should be
 * created. Copy notification must originate from the owner.
 */
class RigidBodyCopyNotification {
public:
   struct Parameters {
      GeomType geomType_;
   };

   inline RigidBodyCopyNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshaling rigid body copies.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * TODO
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyCopyNotification& obj ) {
   marshalDynamically( buffer, obj.body_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body copies.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * TODO
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyCopyNotification::Parameters& objparam ) {
   unmarshal( buffer, objparam.geomType_ );
}
//*************************************************************************************************

} // namespace pe

#endif
