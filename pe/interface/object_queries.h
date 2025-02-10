#ifndef OBJECT_QUERIES_HPP
#define OBJECT_QUERIES_HPP
#include <stdio.h>
#include <pe/core.h>
#include <pe/engine.h>
#include <pe/support.h>
#include <pe/config/Precision.h>

typedef struct {
  real position[3]; 
  real velocity[3]; 
  real angvel[3]; 
  real force[3]; 
  real torque[3]; 
  real time;
  int localIdx;
  int uniqueIdx;
  int systemIdx;
  short int bytes[8];
}particleData_t;

/*
 *!\brief Converts a byte array to a 8-byte uint
 * \param byte_array The input byte array of length 8 
 */
boost::uint64_t ByteArrayToUint64(short int byte_array[8]);

/*
 *!\brief The function checks if an id corresponds to a remote particle on the local domain
 * \param fbmid The id used in the fbm context
 * \param id An index from 0,..,numRemParticles-1 that we map & compare with fbmid
 */
bool checkRemoteFBM(int fbmid, int id); 

/*
 *!\brief The function synchronizes (by addition) the forces on a distributed particle 
 */
void synchronizeForces(); 

/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 */
void particleMapping(); 

/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 */
void remoteParticleMapping(); 

/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getParticlesIndexMap(int *idxMap); 

/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getRemoteParticlesIndexMap(int *idxMap); 

/*
 *!\brief The function checks if the point pos in inside the particles 
 * \param vidx The index of the in the cfd grid 
 * \param inpr An output parameter that is set to the idx of that particle that contains pos 
 * \param pos An array that cointans the 3d point
 */
bool pointInsideParticles(int vidx, int* inpr, double pos[3], short int bytes[8]); 


/*
 *!\brief The function checks if an id corresponds to a remote particle on the local domain
 * \param fbmid The id used in the fbm context
 * \param id An index from 0,..,numRemParticles-1 that we map & compare with fbmid
 */
bool checkRemoteFBM(int fbmid, int id); 

/*
 *!\brief The function synchronizes (by addition) the forces on a distributed particle 
 */
void synchronizeForces(); 

/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 */
void particleMapping(); 

/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 */
void remoteParticleMapping(); 

/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getParticlesIndexMap(int *idxMap); 

/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getRemoteParticlesIndexMap(int *idxMap); 

/*
 *!\brief The function checks if the point pos in inside the particles 
 * \param inpr An output parameter that is set to the idx of that particle that contains pos 
 * \param pos An array that cointans the 3d point
 */
bool pointInsideParticles(int* inpr, double pos[3]); 

/*
 *!\brief The function returns the radius the particle idx
 * \param idx The index of the particle
 */
double getObjRadius(int idx);

/*
 *!\brief The function returns if the object is a sphere
 */
bool isSphereType(int idx);


/*
 *!\brief The function returns if the object is a plane
 */
bool isPlaneType(int idx);


/*
 *!\brief The function returns if the object is a plane
 */
bool isCapsuleType(int idx);
/*
 *!\brief We return whether the object with id idx is a sphere
 *
 * \param idx The id of the local particle
 */
int getNumParts();

/*
 *!\brief We return whether the object with id idx is a sphere
 *
 * \param idx The id of the local particle
 */
int getNumRemParts();

int getTotalParts();
/*
 *!\brief Returns a copy of the object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void getObjByIdx(int idx,
                 int *lidx,
                 int *uidx,
                 double *time,
                 double pos[3],
                 double vel[3]); 

void getPartStructByIdx(int idx, particleData_t *particle);

/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param idx The system id
 * \param particle The particle structure
 */
void setPartStruct(particleData_t *particle); 

void getRemPartStructByIdx(int idx, particleData_t *particle);

/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle A pointer to the particle structure 
 */
void setRemPartStruct(particleData_t *particle); 
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param idx The system id
 * \param particle The particle structure
 */
void setPartStruct(particleData_t *particle); 

void getRemPartStructByIdx(int idx, particleData_t *particle);

/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle A pointer to the particle structure 
 */
void setRemPartStruct(particleData_t *particle); 
/*
 *!\brief Returns a copy of the remote object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void getRemoteObjByIdx(int idx,
                       int *lidx,
                       int *uidx,
                       double *time,
                       double pos[3],
                       double vel[3]); 

/*
 *!\brief Returns a copy of the remote object main parameters. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void getRemoteObjByIdx(int idx,
                       int *lidx,
                       int *uidx,
                       double *time,
                       double pos[3],
                       double vel[3]); 

/*
 *!\brief Set updated parameters for the object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setObjByIdx(int idx,
                 int *lidx,
                 int *uidx,
                 double *time,
                 double pos[3],
                 double vel[3]); 
/*
 *!\brief Set updated parameters for the remote object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setRemoteObjByIdx(int idx,
                       int *lidx,
                       int *uidx,
                       double *time,
                       double pos[3],
                       double vel[3]); 

/*
 *!\brief Set updated parameters for the object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setForcesByIdx(int idx,
                    int *lidx,
                    int *uidx,
                    double force[3],
                    double torque[3]); 
/*
 *!\brief Set updated parameters for the remote object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setRemoteForcesByIdx(int idx,
                          int *lidx,
                          int *uidx,
                          double force[3],
                          double torque[3]); 

/*
 *!\brief Set updated parameters for the object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setForcesByIdx(int idx,
                    int *lidx,
                    int *uidx,
                    double force[3],
                    double torque[3]); 
/*
 *!\brief Set updated parameters for the remote object. The function is callable from Fortran
 * \param idx The system id
 * \param lidx The local id
 * \param uidx The unique id
 * \param time The contact time 
 * \param pos The global position of the object
 * \param vel The global velocity of the object
 */
void setRemoteForcesByIdx(int idx,
                          int *lidx,
                          int *uidx,
                          double force[3],
                          double torque[3]); 

/*
 *!\brief The function returns true if the point is inside the object else false
 * \param idx The index of the object to check against
 * \param pos The coordinates of the point
 */
bool isInsideObject(int idx, double pos[3]);

/*
 *!\brief The function returns true if the point is inside the remote object else false
 * \param idx The index of the remote object to check against
 * \param pos The coordinates of the point
 */
bool isInsideRemObject(int idx, double pos[3]);

/*
 *!\brief Converts a 8-byte uint to a byte array
 * \param id The 8-byte unsigned int to be converted
 * \param byte_array The byte array of length 8 that will hold the result
 */
void uint64toByteArray(boost::uint64_t id, short int byte_array[8]);
#endif
