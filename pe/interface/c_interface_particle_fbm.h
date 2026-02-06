//=================================================================================================
/*
 *!\brief The function returns true if the point is inside the object else false
 * \param idx The index of the object to check against
 * \param pos The coordinates of the point
 */
// Bound to Fortran function pointInsideObject(idx, pos) in
// source/src_particles/dem_query.f90 line 204
extern "C" bool pointInsideObject(int idx, double pos[3]) {
  int i = idx;
  return isInsideObject(i, pos);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns true if the point is inside the remote object else false
 * \param idx The index of the remote object to check against
 * \param pos The coordinates of the point
 */
// Bound to Fortran function pointInsideRemObject(idx, pos) in
// source/src_particles/dem_query.f90 line 215
extern "C" bool pointInsideRemObject(int idx, double pos[3]) {
  return isInsideRemObject(idx, pos);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns true if the point is inside the object else false
 * \param vidx The index of the in the cfd grid
 * \param idx The index of the object to check against
 * \param pos The coordinates of the point
 *
 * Uses original O(N) linear search by default. Enable USE_ACCELERATED_POINT_QUERY=ON
 * to use HashGrid-based spatial hashing acceleration (requires validation).
 */
extern "C" bool checkAllParticles(int vidx, int *inpr, double pos[3], short int bytes[8]) {
#if !defined(USE_ACCELERATED_POINT_QUERY) || USE_ACCELERATED_POINT_QUERY
  return pointInsideParticlesAccelerated(vidx, inpr, pos, bytes);
#else
  return pointInsideParticles(vidx, inpr, pos, bytes);
#endif
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns true if the point is inside the object else false
 * \param vidx The index of the in the cfd grid
 * \param idx The index of the object to check against
 * \param pos The coordinates of the point
 *
 * Uses original O(N) linear search by default. Enable USE_ACCELERATED_POINT_QUERY=ON
 * to use HashGrid-based spatial hashing acceleration (requires validation).
 */
extern "C" bool verifyAllParticles(int vidx, int *inpr, double pos[3], short int bytes[8]) {
  return pointInsideParticles(vidx, inpr, pos, bytes);
}
//=================================================================================================

//=================================================================================================
/*
 *!\brief Explicit accelerated point query (always uses HashGrid optimization)
 * \param vidx The index of the in the cfd grid
 * \param inpr Output: particle index if inside, 0 otherwise
 * \param pos The coordinates of the point
 * \param bytes Output: 8-byte system ID array
 */
extern "C" bool checkAllParticlesAccelerated(int vidx, int *inpr, double pos[3], short int bytes[8]) {
  return pointInsideParticlesAccelerated(vidx, inpr, pos, bytes);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Synchronize the forces acting on the remote particles
 */
extern "C"  
// Bound to Fortran function rem_particles_index_map(idxMap) in
// source/src_particles/dem_query.f90 line 234
void rem_particles_index_map(int *idxMap) {
  getRemoteParticlesIndexMap(idxMap);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Synchronize the forces acting on the remote particles
 */
extern "C"  
void map_particles_() {
  particleMapping(); 
} 
//=================================================================================================


//=================================================================================================
/*
 *!\brief Synchronize the forces acting on the remote particles
 */
extern "C"  
void rem_map_particles_() {
  remoteParticleMapping(); 
} 
//=================================================================================================


//=================================================================================================
/*
 *!\brief Synchronize the forces acting on the remote particles
 */
extern "C"  
// Bound to Fortran function particles_index_map(idxMap) in
// source/src_particles/dem_query.f90 line 241
void particles_index_map(int *idxMap) {
  getParticlesIndexMap(idxMap);
}
//=================================================================================================
