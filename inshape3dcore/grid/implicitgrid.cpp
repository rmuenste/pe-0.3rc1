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


#include "implicitgrid.h"

namespace i3d {

CImplicitGrid::CImplicitGrid() 
{

}

CImplicitGrid::CImplicitGrid(CBasicSpatialHash *pSpatialHash)
{

  m_pSpatialHash=pSpatialHash;
  
}

CImplicitGrid::CImplicitGrid(CBasicSpatialHash *pSpatialHash, Real cellSize)
{
  m_pSpatialHash = pSpatialHash;
  m_dCellSize    = cellSize;
}

CImplicitGrid::~CImplicitGrid() 
{
  delete m_pSpatialHash;
}

void CImplicitGrid::Insert(RigidBody *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CCellCoords cell;
  VECTOR3 center = body->com_;

  CSpatialHashEntry entry(body,cell);

  m_pSpatialHash->Insert(entry);

}

void CImplicitGrid::Insert(CompoundBody *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CCellCoords cell;

  for(int i=0;i<body->GetNumComponents();i++)
  {
    RigidBody *pBody = body->GetComponent(i);      
    VECTOR3 center    = pBody->com_;
    CSpatialHashEntry entry(pBody,cell);
    m_pSpatialHash->Insert(entry);
  }

}

void CImplicitGrid::Insert(SubdomainBoundary *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CCellCoords cell;

  for(int i=0;i<body->m_pBodies.size();i++)
  {
    RigidBody *pBody = body->m_pBodies[i];
    VECTOR3 center    = pBody->com_;
    CSpatialHashEntry entry(pBody,cell,CSpatialHashEntry::SUBDOMAIN);
    m_pSpatialHash->Insert(entry);
  }

}

void CImplicitGrid::Remove(RigidBody *body)
{

}

void CImplicitGrid::Clear()
{
  m_pSpatialHash->Clear();
}

}
