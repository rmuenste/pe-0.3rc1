/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) <2011>  <Raphael Muenster>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/
#ifndef COLLIDERCOMPOUNDBODYBOUNDARYBOX_H
#define COLLIDERCOMPOUNDBODYBOUNDARYBOX_H



//===================================================
//                     INCLUDES
//===================================================
#include <rigidbody.h>
#include <contact.h>
#include <boundarybox.h>

#include "collider.h"

namespace i3d {

	/**
	* @brief A Collider for a cylinder and a box-shaped boundary
	*
	*
	*/
	class ColliderCompoundBodyBoundaryBox : public Collider {

	public:

		ColliderCompoundBodyBoundaryBox();

		~ColliderCompoundBodyBoundaryBox();

		/**
		* @see CCollider::Collide
		*
		*/
		void collide(std::vector<Contact> &vContacts);

	};

}
#endif