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
#ifndef HSPATIALHASH_H
#define HSPATIALHASH_H

#define MAX_LEVELS_HGRID 5

//===================================================
//                     INCLUDES
//===================================================
#include<spatialhash.h>
#include <list>
#include <unstructuredgrid.h>

namespace i3d {
  
/**
 * @brief The class implements the hierarchical spatial hash data structure
 * 
 */
class CHSpatialHash : public CSpatialHash {

public: 

  CHSpatialHash(); 

  ~CHSpatialHash();

  void InitHGrid();
  
  CHSpatialHash(int ncells); 

  CHSpatialHash(int ncells, Real dim[6], std::vector<CRigidBody*> &vRigidBodies);

/**
 * @brief Insert a new element in the CSpatialHashEntry into the grid
 *
 * Insert a new element in the CSpatialHashEntry into the grid
 * 
 */  
  void Insert(CSpatialHashEntry &e);
  
/**
 * @brief Remove a certain cell from the grid
 * 
 * Remove a certain cell from the grid 
 *
 */  
  void Remove(const CCellCoords &cell);

/**
 * @brief Remove all entries from the grid so it can be rebuild with new parameters
 *
 * Remove all entries from the grid so it can be rebuild with new parameters 
 * 
 */  
  void Clear();

/**
 * @brief Returns whether a cell is empty or not
 * 
 * Returns whether a cell is empty or not 
 *
 */  
  bool IsEmpty(const CCellCoords &cell);

/**
 * @brief Returns a vector of all entries that are contained in a cell
 *  
 * Returns a vector of all entries that are contained in a cell
 * 
 */
  std::vector<CSpatialHashEntry> *GetCellEntries(CCellCoords &cell);  
  
/**
 * @brief Convert the grid to a vtk unstructuredgrid for output purposes 
 *
 * Convert the grid to a vtk unstructuredgrid for output purposes 
 * 
 */  
  void ConvertToUnstructuredGrid(CUnstrGridr &ugrid);
  
/**
 * @brief Returns the number of cells that are not empty
 *
 *  Returns the number of cells that are not empty
 *
 */  
  int GetUsedCells(int level) {return m_iUsedCells[level];};

/**
 * @brief Returns the total number of cells
 *
 *  Returns the total number of cells
 *
 */    
  int GetNCells() {return m_iNCells;};

/**
 * @brief Returns the grid size of the level
 *
 * Returns the grid size of the level
 *
 */      
  Real GetGridSize(int level) {return m_pGridSize[level];};

/**
 * @brief Returns the index of the maximum level
 *
 * Returns the index of the maximum level
 *
 */        
  int GetMaxLevel(){return (m_iMaxLevel-1);}

/**
 * @brief Returns the index of the minimum level
 *
 * Returns the index of the minimum level
 *
 */          
  int GetMinLevel(){return 0;}

/**
 * @brief Returns a CCellCoords object on the given level for the given position
 *
 * Returns a CCellCoords object on the given level for the given position. 
 * The CCellCoords object stores information about the cell location of the query point
 * in the grid.
 * 
 */          

  inline CCellCoords GetCell(const VECTOR3 &p, int level)
  {
    //calculate the cell indices
    Real invCellSize = (Real)1.0/m_pGridSize[level];
    
    CCellCoords cell;
    
    //calculate the cell indices
    cell.x = (int)(p.x * invCellSize);
    cell.y = (int)(p.y * invCellSize);
    cell.z = (int)(p.z * invCellSize);
    cell.level = level;    
    return cell;
  }
  
  bool IsBoundaryLevel(int iLevel)
  {
    if(iLevel < MAX_LEVELS_HGRID)
    {
      return m_bIsBoundaryLevel[iLevel];
    }
    else
      return false;
  }
  
  inline bool IsBoundary(const CCellCoords &cell)
  {
    return (cell.x==0 || cell.y==0 || cell.z==0 || cell.x==m_iMaxIndices[cell.level][0] || cell.y==m_iMaxIndices[cell.level][1] || cell.z==m_iMaxIndices[cell.level][2]);
  }
  

/**
 * @brief An iterator that iterates over the entries of the hierarchical hash on a given level
 */
  class hashiterator
  {
  public:
  	
	  typedef std::vector<CSpatialHashEntry> value_type;
	  typedef std::vector<CSpatialHashEntry>* pointer;
	  typedef std::vector<CSpatialHashEntry>& reference;
    hashiterator(std::vector<int>::iterator iter, int level, CHSpatialHash *pHash) : _pHash(pHash), _iter(iter), _level(level)
    {
      if(_iter!=pHash->m_vUsedCells[_level].end())
        _curpos = &pHash->m_pCells[_level][(*iter)];
      else
      {
        //error
      }
    };

	  reference operator*() {return *_curpos;}

	  pointer Get() {return _curpos;}

		int GetPos() {return (*_iter);};

    //prefix operator
    hashiterator& operator++()
    {
      _iter++;
      if(_iter!=_pHash->m_vUsedCells[_level].end())
        _curpos = &_pHash->m_pCells[_level][(*_iter)];
      return *this;
	  }

    //postfix operator
	  hashiterator operator++(int)
	  {
		  hashiterator old_it = *this;
		  ++(*this);
		  return old_it;
	  }

    bool operator !=(hashiterator rhs){return _iter != rhs._iter;};

  protected:
    std::vector<CSpatialHashEntry>* _curpos;
    std::vector<int>::iterator _iter;
    CHSpatialHash *_pHash;
    int _level;
  };

  hashiterator begin(int ilevel) {return hashiterator(m_vUsedCells[ilevel].begin(),ilevel,this);};
  hashiterator end(int ilevel) {return hashiterator(m_vUsedCells[ilevel].end(),ilevel,this);};

  friend class CHSpatialHash::hashiterator;

private:

  int hash(int x, int y, int z);
  void EstimateCellSize(std::vector<CRigidBody*> &vRigidBodies);

  const static int m_iPrime1 = 73856093;
  const static int m_iPrime2 = 19349663;
  const static int m_iPrime3 = 83492791;

  std::vector<CSpatialHashEntry> *m_pCells[MAX_LEVELS_HGRID];

  int              m_iNCells;
  int              m_iMaxIndices[MAX_LEVELS_HGRID][3];
  std::vector<int> m_vUsedCells[MAX_LEVELS_HGRID];
  bool             *m_bUsedCells[MAX_LEVELS_HGRID];
  int              m_iUsedCells[MAX_LEVELS_HGRID];
  Real             m_dDimension[6];

  bool m_bSizeInitialized;
  std::list<Real> lSizes;

  bool m_bIsBoundaryLevel[MAX_LEVELS_HGRID];
  
  Real m_pGridSize[MAX_LEVELS_HGRID];
  int  m_iMaxLevel;
  
};

}
#endif
