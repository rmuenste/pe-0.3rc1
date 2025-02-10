

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
extern "C" int getTotalParticles() {
   return getTotalParts();
}


//=================================================================================================
/*
 *!\brief The function returns the number of particles in the domain
 */
extern "C" int getNumParticles() {
  return getNumParts();
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns the number of remote particles in the domain
 */
extern "C" int getNumRemParticles() {
  return getNumRemParts();
}

//=================================================================================================

/*
 *!\brief The function returns the radius the particle idx
 * \param idx The index of the particle
 */
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
extern "C" bool isTypeSphere(int *idx) {

  int ridx = *idx;
  return isSphereType(ridx); 
}
//=================================================================================================
