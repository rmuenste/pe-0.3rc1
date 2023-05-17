//=================================================================================================
/*!
 *  \file pe/core/rigidbody/InnerCylinderVector.h
 *  \brief Implementation of a vector for cylinder handles
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

#ifndef _PE_CORE_RIGIDBODY_INNERCYLINDERVECTOR_H_
#define _PE_CORE_RIGIDBODY_INNERCYLINDERVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

class InnerCylinder;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for cylinder handles.
 * \ingroup core
 *
 * The InnerCylinderVector class is a special PtrVector for cylinder handles (pe::InnerCylinderID). It
 * offers easy and fast access to the contained cylinders:

   \code
   pe::InnerCylinderVector cylinders;

   // Calculating the total number of cylinders contained in the cylinder vector.
   pe::InnerCylinderVector::SizeType num = cylinders.size();

   // Loop over all cylinders contained in the cylinder vector.
   pe::InnerCylinderVector::Iterator begin = cylinders.begin();
   pe::InnerCylinderVector::Iterator end   = cylinders.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<InnerCylinder,NoDelete>  InnerCylinderVector;
//*************************************************************************************************

} // namespace pe

#endif
