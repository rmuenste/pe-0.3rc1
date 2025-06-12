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
 *!\brief Set the force and torque for q remote particle
 *
 * \param idx The id of the remote particle
 */
// Bound to Fortran function setForces(idx, lidx, uidx, force, torque) in
// source/src_particles/dem_query.f90 line 156
extern "C" void setForces(int *idx,
                          int *lidx,
                          int *uidx,
                          double force[3],
                          double torque[3]) {

  int i = *idx;
  setForcesByIdx(i, lidx, uidx, force, torque);

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Set the force and torque for q remote particle
 *
 * \param idx The id of the remote particle
 */
// Bound to Fortran function setRemoteForces(idx, lidx, uidx, force, torque) in
// source/src_particles/dem_query.f90 line 170
extern "C" void setRemoteForces(int *idx,
                                int *lidx,
                                int *uidx,
                                double force[3],
                                double torque[3]) {

  int i = *idx;
  setRemoteForcesByIdx(i, lidx, uidx, force, torque);

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Returns a copy of the object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
// Bound to Fortran function getParticle(idx, lidx, uidx, time, pos, vel) in
// source/src_particles/dem_query.f90 line 65
extern "C" void getParticle(int idx,
                            int *lidx,
                            int *uidx,
                            double *time,
                            double pos[3],
                            double vel[3]) {

  getObjByIdx(idx, lidx, uidx, time, pos, vel);

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
 *!\brief Returns a copy of the remote object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
// Bound to Fortran function getRemoteParticle(idx, lidx, uidx, time, pos, vel) in
// source/src_particles/dem_query.f90 line 103
extern "C" void getRemoteParticle(int *idx,
                                  int *lidx,
                                  int *uidx,
                                  double *time,
                                  double pos[3],
                                  double vel[3]) {

  int i = *idx;
  getRemoteObjByIdx(i, lidx, uidx, time, pos, vel);
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
