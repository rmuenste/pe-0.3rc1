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
extern "C" void getParticle2(int idx, particleData_t *particle) { 
  getPartStructByIdx(idx, particle);
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle The particle structure
 */
extern "C" void setParticle2(particleData_t *particle) { 
  setPartStruct(particle); 
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle The particle structure
 */
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
extern "C" void getRemoteParticle2(int idx, particleData_t *particle) { 
  getRemPartStructByIdx(idx, particle);
}
//=================================================================================================
