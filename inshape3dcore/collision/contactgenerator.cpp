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


//===================================================
//                     INCLUDES
//===================================================
#include "contactgenerator.h"

namespace i3d {

template <class T>
CContactGenerator<T>::CContactGenerator() 
{

}

template <class T>
CContactGenerator<T>::~CContactGenerator() 
{

}

template<class T>
void CContactGenerator<T>::GenerateContactPoints(const Shape<T> &shape0, const Shape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                 const Transformation<T> &transform0, const Transformation<T> &transform1,
                                                 const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                                 CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts)
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CContactGenerator<Real>;

//----------------------------------------------------------------------------
}
