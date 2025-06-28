# HashGrid Collision Detection Algorithm

This document summarizes the hierarchical hash grid (HHG) coarse collision detection implemented in `pe/core/detection/coarse/HashGrids.h`.

---

## **1. Overview**
The HHG algorithm partitions 3D space using several uniform grids of increasing cell size. Every rigid body is assigned to exactly one grid and exactly one cell: the smallest grid whose cubic cell spans are larger than the body's longest AABB edge. Contacts are only generated among bodies that share the same cell or belong to neighboring cells. The hierarchy automatically adapts at runtime by growing grids or creating new levels when required.

---

## **2. Grid Structure**
- Each **HashGrid** is a 3D array of cells allocated as a single linear array. The cell counts per axis are powers of two so that modulo operations can be replaced with bitwise masks during hashing.
- A **Cell** lazily allocates a vector of body handles (`BodyVector`) when at least one body is assigned. It also stores a table of offsets to all 26 neighbors plus itself, enabling fast access to adjacent cells.
- The array of neighbor offsets is shared by all inner cells (`stdNeighborOffset_`), while border cells allocate their own tables to support wrap-around indexing.
- Each grid keeps a list of all cells currently containing bodies (`occupiedCells_`) to avoid iterating over empty cells when generating contacts.

---

## **3. Body Insertion and Removal**
- Adding a body computes its cell hash from the AABB center coordinates using the cell span and bit masks.
  * `minimalGridDensity` is a configuration constant defining the minimum ratio of cells to bodies that must be maintained. If adding a body would drop the grid below this density, the grid must grow.
  * Each grid stores an `enlargementThreshold_` equal to the current total cell count divided by `minimalGridDensity`. Once the number of inserted bodies equals this threshold, `enlarge()` doubles the cell count along every axis (increasing the total cell count eightfold) and recomputes the threshold for the new grid size.
- Bodies store their hash and index within the cell container so that removal occurs in constant time. When the last body leaves a cell, the cell's container is deleted and the cell is removed from `occupiedCells_`.
- During updates a body's hash is recomputed; if it changes, the body is removed from its old cell and inserted into the new one.

---

## **4. Contact Generation**
- For each occupied cell, the algorithm performs pairwise checks among bodies within the cell and against bodies in the first half of its neighboring cells to avoid duplicate tests. Afterwards the list of processed bodies is returned so other grids can check them against bodies stored in larger cells.
- To test bodies from one grid against another grid with larger cell size, each body is hashed in the larger grid and checked against all bodies in the corresponding cell and its neighbors.
- Bodies not assigned to any grid (e.g., infinite size or before activation) are kept in `nonGridBodies_` and tested pairwise as well as against bodies from every grid.

---

## **5. Hierarchy Management**
- The hierarchy of grids (`gridList_`) is sorted by cell size. The factor between successive grid levels is controlled by the configuration constant `hierarchyFactor`.
- The algorithm remains inactive until the number of bodies exceeds `gridActivationThreshold` (default 32). Before activation all bodies reside in `nonGridBodies_` and naive pairwise checking is used.
- When grids become too densely populated, `enlarge()` doubles the cell count in each axis, reinitializes neighbor offsets, and reinserts all bodies.

---

## **6. Advantages**
- **Average O(N)** time for broad-phase detection due to spatial partitioning.
- Constant-time insertion/removal using cached hash and cell indices.
- Adaptable grid sizes keep memory usage low and permit large-scale simulations with many bodies.

