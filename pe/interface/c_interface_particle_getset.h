//=================================================================================================
/*
 *!\brief Synchronize the forces acting on the remote particles
 */
extern "C" void sync_forces_() {
  synchronizeForces();
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Returns a copy of the object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param particle The particle structure
 */
// Bound to Fortran function getParticle2(idx, particle) in
// source/src_particles/dem_query.f90 line 80
extern "C" void getParticle2(int idx, particleData_t *particle) {
  getPartStructByIdx(idx, particle);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle The particle structure
 */
// Bound to Fortran function setParticle2(particle) in
// source/src_particles/dem_query.f90 line 92
extern "C" void setParticle2(particleData_t *particle) {
  setPartStruct(particle); 
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle The particle structure
 */
// Bound to Fortran function setRemoteParticle2(particle) in
// source/src_particles/dem_query.f90 line 130
extern "C" void setRemoteParticle2(particleData_t *particle) {
  setRemPartStruct(particle); 
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Returns a copy of the object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param particle The particle structure
 */
// Bound to Fortran function getRemoteParticle2(idx, particle) in
// source/src_particles/dem_query.f90 line 118
extern "C" void getRemoteParticle2(int idx, particleData_t *particle) {
  getRemPartStructByIdx(idx, particle);
}
//=================================================================================================
