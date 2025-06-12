

// Bound to Fortran function check_rem_id(fbmid, id) in
// source/src_particles/dem_query.f90 line 226
extern "C"
bool check_rem_id(int fbmid, int id) {

  return checkRemoteFBM(fbmid, id); 
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns the total number of particles
 */
//=================================================================================================
// Bound to Fortran function getTotalParticles() in
// source/src_particles/dem_query.f90 line 35
extern "C" int getTotalParticles() {
   return getTotalParts();
}


//=================================================================================================
/*
 *!\brief The function returns the number of particles in the domain
 */
// Bound to Fortran function getNumParticles() in
// source/src_particles/dem_query.f90 line 46
extern "C" int getNumParticles() {
  return getNumParts();
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns the number of remote particles in the domain
 */
// Bound to Fortran function getNumRemParticles() in
// source/src_particles/dem_query.f90 line 56
extern "C" int getNumRemParticles() {
  return getNumRemParts();
}

//=================================================================================================

/*
 *!\brief The function returns the radius the particle idx
 * \param idx The index of the particle
 */
// Bound to Fortran function getParticleRadius(idx) in
// source/src_particles/dem_query.f90 line 194
extern "C" double getParticleRadius(int *idx) {
  int ridx = *idx;
  return getObjRadius(ridx);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief We return whether the object with id idx is a sphere
 *
 * \param idx The id of the local particle
 */
// Bound to Fortran function isSphere(idx) via isTypeSphere in
// source/src_particles/dem_query.f90 line 184
extern "C" bool isTypeSphere(int *idx) {

  int ridx = *idx;
  return isSphereType(ridx); 
}
//=================================================================================================
