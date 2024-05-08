#include <pe/interface/object_queries.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <map>

#define ONLY_ROTATION 
using namespace pe;
std::map<int, boost::uint64_t> particleMap;
std::map<int, boost::uint64_t> remoteParticleMap;
std::map<int, boost::uint64_t> fbmMap;


extern "C"
void uint64_test(uint64_t* value) {
  *value = 18446744073709551615;
  //*value = 123456789123457890;
}


/*
 * This is is build in the pointInsideParticles function. It serves to 
 * convert the fbm-Id to the systemId of the remote particle for the 
 * current time step
 */
std::map<int, boost::uint64_t> fbmMapRemote; // The key is the local fbmId, the value is the systemId

/*
 *!\brief The function synchronizes (by addition) the forces on a distributed particle 
 * Fortran interface: sync_forces()
 */
void synchronizeForces() {

  // Get the current time step size
  real stepsize = TimeStep::size();

  WorldID world = theWorld();
  MPI_Comm cartcomm = theMPISystem()->getComm();
  int rank = theMPISystem()->getRank();
  MPI_Barrier(cartcomm);
  theCollisionSystem()->synchronizeForces();


  for (pe::World::SizeType i=0; i < world->size(); i++) {
    BodyID body = world->getBody(i);
#ifdef ONLY_ROTATION
    body->setForce(Vec3(0,0,0));
#endif
    if (body->getType() == sphereType) {
      std::cout << "Force: "  <<  body->getForce()[2] << std::endl;
      std::cout << "Torque: "  <<  body->getTorque() << std::endl;
    }
    else if(body->getType() == triangleMeshType) {
      Vec3 tau = body->getTorque();
      tau[0] = 0;
      tau[2] = 0;
      body->setTorque(tau);
      std::cout << "Force: "  << std::setprecision(8)  << body->getForce()[0] << " " << body->getForce()[1] << " " << body->getForce()[2] << std::endl;
      std::cout << "Torque: "  << std::setprecision(8)  << body->getTorque()[0] << " " << body->getTorque()[1] << " " << body->getTorque()[2] << std::endl;
//      std::cout << "Force: "  <<  body->getForce().toString() << std::endl;
//      std::cout << "Torque: "  <<  body->getTorque().toString() << std::endl;
    }
    if (body->getType() == capsuleType) {
      Vec3 tau = body->getTorque();
      tau[0] = 0;
      tau[1] = 0;
      body->setTorque(tau);
      std::cout << "Force: "  << std::setprecision(8)  << body->getForce()[0] << " " << body->getForce()[1] << " " << body->getForce()[2] << std::endl;
      std::cout << "Torque: "  << std::setprecision(8)  << body->getTorque()[0] << " " << body->getTorque()[1] << " " << body->getTorque()[2] << std::endl;
    }
    body->applyFluidForces(stepsize);
    if(body->getType() == triangleMeshType) {
      std::cout << "Omega_a: "  <<  body->getAngularVel().toString() << std::endl;
    }
  }

  for (std::size_t i=0; i < theCollisionSystem()->getBodyShadowCopyStorage().size(); i++) {
    BodyID body = world->getShadowBody(i);
    if (body->getType() == sphereType) {
//      std::cout << rank << ")" << body << std::endl;
    }
    body->applyFluidForces(stepsize);
  }

  world->synchronize();

}
//=================================================================================================


//=================================================================================================
//
//  Particle Mapping Function between Fortran and C++
//
//=================================================================================================
//=================================================================================================
/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getParticlesIndexMap(int *idxMap) {

  WorldID world = theWorld();
  int count = 0;
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      idxMap[count] = i;
      count++;
    }
    else if(body->getType() == capsuleType) {
      idxMap[count] = i;
      count++;
    }
    else if(body->getType() == ellipsoidType) {
      idxMap[count] = i;
      count++;
    }
    else if(body->getType() == triangleMeshType) {
      idxMap[count] = i;
      count++;
    }
  }

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 * \param idxMap Integer array that contains the mapping
 */
void getRemoteParticlesIndexMap(int *idxMap) {

  WorldID world = theWorld();
  MPISystemID mpisys = theMPISystem();
  int rank = mpisys->getRank();
  int count = 0;
  unsigned int i(0);
//  std::cout << rank << ")";
  for (; i < theCollisionSystem()->getBodyShadowCopyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    ConstBodyID body;
    body = theCollisionSystem()->getBodyShadowCopyStorage().at(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      idxMap[count] = i;
//      std::cout << " " << i << " " << body->getSystemID();
      count++;
    }
    else if(body->getType() == capsuleType) {
      idxMap[count] = i;
//      std::cout << " " << i << " " << body->getSystemID();
      count++;
    }
    else if(body->getType() == ellipsoidType) {
      idxMap[count] = i;
//      std::cout << " " << i << " " << body->getSystemID();
      count++;
    }
    else if(body->getType() == triangleMeshType) {
      idxMap[count] = i;
//      std::cout << " " << i << " " << body->getSystemID();
      count++;
    }
  }
//  std::cout  << std::endl;
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns a mapping from 0,..,numParticles-1 to the bodystorage corr. indices
 */
void particleMapping() {

  WorldID world = theWorld();
  MPISystemID mpisys = theMPISystem();

  particleMap.clear();
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      particleMap[static_cast<unsigned int>(i)] = body->getSystemID();
    }
    else if(body->getType() == capsuleType) {
      particleMap[static_cast<unsigned int>(i)] = body->getSystemID();
    }
  }

} 
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns a mapping from 0,..,numRemParticles-1 to the shadow copy corr. indices
 */
void remoteParticleMapping() {

  WorldID world = theWorld();
  MPISystemID mpisys = theMPISystem();

  remoteParticleMap.clear();
  unsigned int i(0);
  for (int j(0); j < theCollisionSystem()->getBodyShadowCopyStorage().size(); j++) {

    World::SizeType widx = static_cast<World::SizeType>(j);
    World::SizeType size = theCollisionSystem()->getBodyShadowCopyStorage().size();

    ConstBodyID body;
    body = theCollisionSystem()->getBodyShadowCopyStorage().at(static_cast<unsigned int>(widx));
    if (body->getType() != sphereType) {
      continue;
    }

    remoteParticleMap[static_cast<unsigned int>(i)] = body->getSystemID();
    i++;

  }

} 
//=================================================================================================


//=================================================================================================
//
//  Utility functions
//
//=================================================================================================
/*
 *!\brief The function checks if the point pos in inside the particles 
 * \param inpr An output parameter that is set to the idx of that particle that contains pos 
 * \param pos An array that cointans the 3d point
 */
extern "C"
void debug_output_force_() {


  WorldID world = theWorld();

  MPISystemID mpisys = theMPISystem();
  int rank = mpisys->getRank();
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      Sphere* s = static_cast<Sphere*>(body); 
//      std::cout << rank << ")force: " << vec3ToString(s->getForce())<< std::endl;
//      std::cout << rank << ")VelUp: " << vec3ToString(s->getLinVel())<< std::endl;
    }
    else if(body->getType() == capsuleType) {
      Capsule* s = static_cast<Capsule*>(body); 
      std::cout << rank << ")force(" << s->getSystemID() << ", " << s->getForce()[0] << ", " << s->getForce()[1] << ", " << s->getForce()[2] << std::endl;
//      std::cout << rank << ")VelUp(" << s->getSystemID() << ", " << (s->getLinearVel())<< std::endl;
    }
  }
  for (int j(0); j < theCollisionSystem()->getBodyShadowCopyStorage().size(); j++, i++) {

    World::SizeType widx = static_cast<World::SizeType>(j);
    World::SizeType size = theCollisionSystem()->getBodyShadowCopyStorage().size();

    BodyID body;
    body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));

    if (body->getType() == sphereType) {
      Sphere* s = static_cast<Sphere*>(body); 
//      std::cout << rank << ")Remforce: " << vec3ToString(s->getForce())<<  std::endl;
//      std::cout << rank << ")RemVelUp: " << vec3ToString(s->getLinVel())<< std::endl;
    }
    else if(body->getType() == capsuleType) {
      Capsule* s = static_cast<Capsule*>(body); 
      std::cout << rank << ")force: " << (s->getForce())<< std::endl;
      std::cout << rank << ")VelUp: " << (s->getLinearVel())<< std::endl;
    }
  }
}
//=================================================================================================


//=================================================================================================
extern "C"
void debug_output_particles_() {


  WorldID world = theWorld();

  MPISystemID mpisys = theMPISystem();
  int rank = mpisys->getRank();
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      Sphere* s = static_cast<Sphere*>(body); 
      std::cout << rank << ")pos: " << s->getPosition() << "vel: " << s->getLinearVel() << std::endl;
    }
  }
  for (int j(0); j < theCollisionSystem()->getBodyShadowCopyStorage().size(); j++, i++) {

    World::SizeType widx = static_cast<World::SizeType>(j);
    World::SizeType size = theCollisionSystem()->getBodyShadowCopyStorage().size();

    BodyID body;
    body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));

    if (body->getType() == sphereType) {
      Sphere* s = static_cast<Sphere*>(body); 
      std::cout << rank << ")pos: " << s->getPosition()<< "vel: " << s->getLinearVel() << std::endl;
    }
  }
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function checks if the point pos in inside the particles (C++ end point for checkAllParticles) 
 * \param inpr An output parameter that is set to the idx of that particle that contains pos 
 * \param pos An array that cointans the 3d point
 */
bool pointInsideParticles(int vidx, int* inpr, double pos[3], short int bytes[8]) {

  MPISystemID mpisys = theMPISystem();
  int rank = mpisys->getRank();

  WorldID world = theWorld();
  unsigned int i(0);
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));

    if(body->getType() == sphereType) {
      if(static_cast<Sphere*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
    else if(body->getType() == capsuleType) {
      if(static_cast<Capsule*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
    else if(body->getType() == ellipsoidType) {
      if(static_cast<Ellipsoid*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
    else if(body->getType() == triangleMeshType) {
      if(static_cast<TriangleMesh*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
  }
  for (int j(0); j < theCollisionSystem()->getBodyShadowCopyStorage().size(); j++) {

    World::SizeType widx = static_cast<World::SizeType>(j);
    World::SizeType size = theCollisionSystem()->getBodyShadowCopyStorage().size();

    ConstBodyID body;
    body = theCollisionSystem()->getBodyShadowCopyStorage().at(static_cast<unsigned int>(widx));

    if (body->getType() == sphereType) {
      if(static_cast<const Sphere*>(body)->containsPoint(pos[0], pos[1], pos[2])) {
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;

        *inpr = val;
        i++;
        return true;
      }
    }
    else if(body->getType() == capsuleType) {
      if(static_cast<const Capsule*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
    else if(body->getType() == ellipsoidType) {
      if(static_cast<const Ellipsoid*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
    else if(body->getType() == triangleMeshType) {
      if(static_cast<const TriangleMesh*>(body)->containsPoint(pos[0], pos[1], pos[2])){
        uint64toByteArray(body->getSystemID(), bytes); 
        int val = bytes[0] + 1;
        *inpr = val;
        return true;
      }
    }
  }

  *inpr = 0;
  return false;

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function checks if an id corresponds to a remote particle on the local domain
 * \param fbmid The id used in the fbm context
 * \param id An index from 0,..,numRemParticles-1 that we map & compare with fbmid
 */
bool checkRemoteFBM(int fbmid, int id) {
  if (fbmid == 0)
    return false;

  std::map<int, boost::uint64_t>::iterator it = fbmMapRemote.find(fbmid);
  if (it != fbmMapRemote.end() && (it->second == id))  
    return true;
  // Something is wrong when we try to find an fbmid that is not on the domain
  else if(it == fbmMapRemote.end())  {
    MPISystemID mpisys = theMPISystem();
    std::stringstream msg;
//    msg << "FBM index: " << fbmid << " not found in local domain(" << mpisys->getRank() << ")" << "\n";
//    throw std::logic_error(msg.str());
    return false;
  }
  else
    return false;

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief Outputs the properties of the remoteMap
 */
extern "C"
void check_remote_status_() {

//  auto it = fbmMapRemote.find(fbmid);

  MPISystemID mpisys = theMPISystem();
  if (fbmMapRemote.size() > 0) {
    std::cout << mpisys->getRank() <<  ")Len: " << fbmMapRemote.size() << std::endl;
    std::cout << mpisys->getRank() <<  ")[key, value]: [";

    for(std::map<int, boost::uint64_t>::iterator it = fbmMapRemote.begin(); it != fbmMapRemote.end(); ++it) {
      std::cout << it->first << ", " << it->second << "] "; 
    }
    std::cout << std::endl;
  }


//  if (it != fbmMapRemote.end() && (it->second == id))  
//    return true;
//  // Something is wrong when we try to find an fbmid that is not on the domain
//  else if(it == fbmMapRemote.end())  {
//    std::stringstream msg;
//    msg << "FBM index: " << fbmid << " not found in local domain(" << mpisys->getRank() << ")" << "\n";
//    throw std::logic_error(msg.str());
//  }
//  else
//    return false;

}
//=================================================================================================



//=================================================================================================
/*
 *!\brief The function returns true if the point is inside the object else false
 * \param idx The index of the object to check against
 * \param pos The coordinates of the point
 */
bool isInsideObject(int idx, double pos[3]) {
  if(isSphereType(idx)) {

    WorldID world = theWorld();
    World::SizeType widx = static_cast<World::SizeType>(idx);
    BodyID body;
    body = world->getBody(static_cast<unsigned int>(widx));
    return static_cast<Sphere*>(body)->containsPoint(pos[0], pos[1], pos[2]);
  } 
  else if(isCapsuleType(idx)) {

    WorldID world = theWorld();
    World::SizeType widx = static_cast<World::SizeType>(idx);
    BodyID body;
    body = world->getBody(static_cast<unsigned int>(widx));
    return static_cast<Capsule*>(body)->containsPoint(pos[0], pos[1], pos[2]);
  } else {
    return false;
  } 
}

//=================================================================================================

/*
 *!\brief The function returns true if the point is inside the remote object else false
 * \param idx The index of the remote object to check against
 * \param pos The coordinates of the point
 */
bool isInsideRemObject(int idx, double pos[3]) {

  WorldID world = theWorld();

  World::SizeType widx = static_cast<World::SizeType>(idx);
  World::SizeType size = theCollisionSystem()->getBodyShadowCopyStorage().size();

  ConstBodyID body;
  if ( widx < size ) {
    body = theCollisionSystem()->getBodyShadowCopyStorage().at(static_cast<unsigned int>(widx));
    if (body->getType() != sphereType) {
      return false;
    }
  }
  else {
    std::stringstream msg;
    msg << "Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }


  return static_cast<const Sphere*>(body)->containsPoint(pos[0], pos[1], pos[2]);
}

//=================================================================================================

/*
 *!\brief The function returns the number of particles in the domain
 */
int getNumParts() {

  WorldID world = theWorld();
  unsigned int i(0);
  int numBodies = 0;

  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) {
      numBodies++;
    }
    else if(body->getType() == capsuleType) {
      numBodies++;
    }
    else if(body->getType() == triangleMeshType) {
      numBodies++;
    }
    else if(body->getType() == ellipsoidType) {
      numBodies++;
    }
  }

  return numBodies;
}
//=================================================================================================

/*
 *!\brief The function returns the number of particles in the domain
 */
int getNumRemParts() {

  int numBodies =  theCollisionSystem()->getBodyShadowCopyStorage().size();
  return numBodies;
}

//=================================================================================================


//=================================================================================================
/*
 *!\brief We return whether the object with id idx is a plane
 *
 * \param idx The id of the local particle
 */
bool isPlaneType(int idx) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    return (body->getType() == planeType) ? true : false;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 
//=================================================================================================



//=================================================================================================
/*
 *!\brief We return whether the object with id idx is a sphere
 *
 * \param idx The id of the local particle
 */
bool isSphereType(int idx) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    return (body->getType() == sphereType) ? true : false;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 
//=================================================================================================


//=================================================================================================
/*
 *!\brief We return whether the object with id idx is a capsule
 *
 * \param idx The id of the local particle
 */
bool isCapsuleType(int idx) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    return (body->getType() == capsuleType) ? true : false;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 
//=================================================================================================


//=================================================================================================
//
//  Getter/Setter for Objects aka Particles 
//
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
void getObjByIdx(int idx,
                 int *lidx,
                 int *uidx,
                 double *time,
                 double pos[3],
                 double vel[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    Vec3 v = body->getLinearVel();
    vel[0] = v[0];
    vel[1] = v[1];
    vel[2] = v[2];
    Vec3 p = body->getPosition();
    pos[0] = p[0];
    pos[1] = p[1];
    pos[2] = p[2];
    // TODO: this is a dubious conversion to a smaller type -> fix
    *lidx = body->getSystemID();
    *uidx = idx;
//    std::cout << idx << "pos: " << vec3ToString(p)<< "vel: " << vec3ToString(v) << std::endl;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

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
void getRemoteObjByIdx(int idx,
                       int *lidx,
                       int *uidx,
                       double *time,
                       double pos[3],
                       double vel[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));
    Vec3 v = body->getLinearVel();
    vel[0] = v[0];
    vel[1] = v[1];
    vel[2] = v[2];
    Vec3 p = body->getPosition();
    pos[0] = p[0];
    pos[1] = p[1];
    pos[2] = p[2];
    // TODO: this is a dubious conversion to a smaller type -> fix
    *lidx = body->getSystemID();
    *uidx = idx;
//    std::cout << idx << "pos: " << vec3ToString(p)<< "vel: " << vec3ToString(v) << std::endl;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 
//=================================================================================================


//=================================================================================================
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
                 double vel[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    Vec3 v(vel[0], vel[1], vel[2]);
    body->setLinearVel(v);
    Vec3 p(pos[0], pos[1], pos[2]);
    body->setPosition(p);
//    std::cout << idx << "pos: " << vec3ToString(p)<< "vel: " << vec3ToString(v) << std::endl;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 
//=================================================================================================

//=================================================================================================
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
                       double vel[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));
    Vec3 v(vel[0], vel[1], vel[2]);
    body->setLinearVel(v);
    Vec3 p(pos[0], pos[1], pos[2]);
    body->setPosition(p);
//    std::cout << idx << "pos: " << vec3ToString(p)<< "vel: " << vec3ToString(v) << std::endl;
  }
  else {
    std::stringstream msg;
    msg << "Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

} 

//=================================================================================================

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
                    double torque[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  body = world->getBody(static_cast<unsigned int>(widx));

  Vec3 f(force[0], force[1], force[2]);
  body->setForce(1.025 * f);
  Vec3 t(0.0, 0.0, 0.0);
  body->setTorque(t);

} 
//=================================================================================================

//=================================================================================================
/*
 *!\brief Set updated parameters for the object. The function is callable from Fortran
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
                          double torque[3]) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));

  Vec3 f(force[0], force[1], force[2]);
  body->setForce(f);
  Vec3 t(0.0, 0.0, 0.0);
  body->setTorque(t);

} 

//=================================================================================================

/*
 *!\brief The function returns the radius the particle idx
 * \param idx The index of the particle
 */
double getObjRadius(int idx) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);
  double rad(0.0);

  BodyID body;
  if ( widx < world->size() ) {

    body = world->getBody(static_cast<unsigned int>(widx));

    if(!isSphereType(idx)) {
      std::stringstream msg;
      msg << "Radius queried for non-sphere object " << idx << "." << "\n";
      throw std::logic_error(msg.str());
    }

    Sphere *s = static_cast<Sphere*>(body);
    rad = (double)s->getRadius();

    std::cout << "Particle: " << idx << " Radius: " <<  rad << std::endl;
  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }

  return rad;
}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns the particle idx as a struct
 * \param idx The index of the particle
 * \param particle A pointer to the particle structure 
 */
void getPartStructByIdx(int idx, particleData_t *particle) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);

  BodyID body;
  if ( widx < world->size() ) {
    body = world->getBody(static_cast<unsigned int>(widx));
    Vec3 v = body->getLinearVel();

    particle->velocity[0] = v[0];
    particle->velocity[1] = v[1];
    particle->velocity[2] = v[2];

    Vec3 omega = body->getAngularVel();

    particle->angvel[0] = omega[0];
    particle->angvel[1] = omega[1];
    particle->angvel[2] = omega[2];

    Vec3 p = body->getPosition();

    particle->position[0] = p[0];
    particle->position[1] = p[1];
    particle->position[2] = p[2];

    // TODO: this is a dubious conversion to a smaller type -> fix
    particle->uniqueIdx = body->getSystemID();
    particle->localIdx  = idx;
    particle->systemIdx = -1;
    particle->time = -1.0;
    
    uint64toByteArray(body->getSystemID(), particle->bytes); 

  }
  else {
    std::stringstream msg;
    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
    throw std::out_of_range(msg.str());
  }
}
//=================================================================================================

/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran by SetForcesMapped
 * \param idx The system id
 * \param particle The particle structure
 */
void setPartStruct(particleData_t *particle) { 

  boost::uint64_t id = ByteArrayToUint64(particle->bytes);

  BodyID body;

  MPISystemID mpisys = theMPISystem();
  pe::World::Iterator fid = theCollisionSystem()->getBodyStorage().find(id);
  if( fid != theCollisionSystem()->getBodyStorage().end()) {
    body = *fid;
    std::stringstream msg;
    msg << "Setting local forces for system id: " << id << " in domain " << mpisys->getRank() << ".\n";
//    std::cout << msg.str() << std::endl;
  } else {

    std::stringstream msg;
    msg << "Could not find system id: " << id << " in domain " << mpisys->getRank() << ".\n";
    throw std::logic_error(msg.str());
  }

  Vec3 f(
    particle->force[0], 
    particle->force[1],
    particle->force[2]
  );

  body->setForce(1.0 * f);

  Vec3 t(
    particle->torque[0], 
    particle->torque[1],
    particle->torque[2]
  );

  body->setTorque(t);

}
//=================================================================================================
/*
 *!\brief Sets values of the fortran struct to the c++ class. The function is callable from Fortran
 * \param particle A pointer to the particle structure 
 */
void setRemPartStruct(particleData_t *particle) {

  boost::uint64_t id = ByteArrayToUint64(particle->bytes);

  BodyID body;

  MPISystemID mpisys = theMPISystem();

  //pe::World::Iterator fid = theCollisionSystem()->getBodyStorage().find(id);

  pe::World::Iterator fid = theCollisionSystem()->getBodyShadowCopyStorage().find(id);
  if( fid != theCollisionSystem()->getBodyShadowCopyStorage().end()) {
    body = *fid;
    std::stringstream msg;
    msg << "Setting remote forces for system id: " << id << " in domain " << mpisys->getRank() << ".\n";
//    std::cout << msg.str() << std::endl;
  } else {

    std::stringstream msg;
    msg << "Could not find system id: " << id << " in domain " << mpisys->getRank() << ".\n";
    throw std::logic_error(msg.str());
  }

  Vec3 f(
    particle->force[0], 
    particle->force[1],
    particle->force[2]
  );

  body->setForce(1.0 * f);

  Vec3 t(
    particle->torque[0], 
    particle->torque[1],
    particle->torque[2]
  );

  body->setTorque(t);

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function returns the particle idx as a struct
 * \param idx The index of the particle
 * \param particle A pointer to the particle structure 
 */
void getRemPartStructByIdx(int idx, particleData_t *particle) {

  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(idx);
  MPISystemID mpisys = theMPISystem();
  int rank = mpisys->getRank();

  BodyID body;
  if ( widx < theCollisionSystem()->getBodyShadowCopyStorage().size() ) {
    body = theWorld()->getShadowBody(static_cast<unsigned int>(widx));

    Vec3 v = body->getLinearVel();

    particle->velocity[0] = v[0];
    particle->velocity[1] = v[1];
    particle->velocity[2] = v[2];

    Vec3 p = body->getPosition();

    particle->position[0] = p[0];
    particle->position[1] = p[1];
    particle->position[2] = p[2];

    Vec3 omega = body->getAngularVel();

    particle->angvel[0] = omega[0];
    particle->angvel[1] = omega[1];
    particle->angvel[2] = omega[2];

    // TODO: this is a dubious conversion to a smaller type -> fix
    particle->uniqueIdx = body->getSystemID();
    particle->localIdx  = idx;
    particle->systemIdx = -1;
    particle->time = -1.0;
    
    uint64toByteArray(body->getSystemID(), particle->bytes); 
  }
  else {
    unsigned int i(0);
    int count = 0;
    std::cout << rank << ")";
    for (; i < theCollisionSystem()->getBodyShadowCopyStorage().size(); i++) {
      World::SizeType widx = static_cast<World::SizeType>(i);
      ConstBodyID body;
      body = theCollisionSystem()->getBodyShadowCopyStorage().at(static_cast<unsigned int>(widx));
      if(body->getType() == sphereType) {
        std::cout << count << " -> " << i << " / " << body->getSystemID() << "||";
        count++;
      }
    }
    std::stringstream msg;
    msg << "Line - " << __LINE__ <<  ": Body index: " << idx << " out of range on domain: " << rank << "\n";
    throw std::out_of_range(msg.str());
  }
}
//=================================================================================================



//=================================================================================================
/*
 *!\brief Converts a 8-byte uint to a byte array
 * \param id The 8-byte unsigned int to be converted
 * \param byte_array The byte array of length 8 that will hold the result
 */
void uint64toByteArray(boost::uint64_t id, short int byte_array[8]) {
  
//  MPISystemID mpisys = theMPISystem();
//  int rank = mpisys->getRank();
//  if(rank==3)
//    std::cout << rank <<") sys=" << id << " -> ";

  for( int i = 0; i < 8; ++i ) {
    byte_array[i] = ( id >> ( 7 - i ) * 8 ) & 0xff;
  }
}

//=================================================================================================
/*
 *!\brief Converts a byte array to a 8-byte uint
 * \param byte_array The input byte array of length 8 
 */
boost::uint64_t ByteArrayToUint64(short int byte_array[8]) {

 boost::uint64_t result = 0; 

 for(int i = 0; i < 8; ++i) {
   result <<= 8;
   result |= byte_array[i];
 }

 return result;

}

/*
Fortran code for converting byte array to integer(kind=16)

integer(c_short), dimension(8) :: byte_array

integer(kind=16) :: result, temp

result = 0
do i = 1, 8
  result = ishft(result, 8)
  temp = int(byte_array(i), 16)
  result = ior(result, temp)
end do

*/
//=================================================================================================



//=================================================================================================
extern "C"
void clear_fbm_maps_() {

//  fbmMap.clear();
  fbmMapRemote.clear();

}
//=================================================================================================



//=================================================================================================
bool mapLocalToSystem(int lidx, int vidx) {
  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(lidx);
  
  int *idxMap = new int[theCollisionSystem()->getBodyShadowCopyStorage().size()]; 

  getRemoteParticlesIndexMap(idxMap); 
  int mappedIdx = static_cast<unsigned int>(idxMap[widx]);

  MPISystemID mpisys = theMPISystem();

  BodyID body = world->getShadowBody(mappedIdx);
  if (fbmMapRemote[vidx] == body->getSystemID()) {
    std::stringstream msg;
    msg << "local particle: " << lidx << " = " << body->getSystemID() << " contains vertex " << vidx << " in local domain(" << mpisys->getRank() << ")" << "\n";
//    throw std::logic_error(msg.str());
    return true;
  } else {
 //!   std::stringstream msg;
 //!   msg << "local particle: " << lidx << " = " << body->getSystemID() << " does not contains vertex " << vidx << " in local domain(" << mpisys->getRank() << ")" << "\n";
 //!   std::cout << msg.str() << std::endl;
    return false;
  }
}

bool mapLocalToSystem2(int lidx, int vidx) {
  WorldID world = theWorld();
  World::SizeType widx = static_cast<World::SizeType>(lidx);
  BodyID body = world->getBody(static_cast<unsigned int>(widx));
  MPISystemID mpisys = theMPISystem();
  if (fbmMapRemote[vidx] == body->getSystemID()) {
    std::stringstream msg;
    msg << "local particle: " << lidx << " = " << body->getSystemID() << " contains vertex " << vidx << " in local domain(" << mpisys->getRank() << ")" << "\n";
//    throw std::logic_error(msg.str());
    return true;
  } else {
    std::stringstream msg;
    msg << "local particle: " << lidx << " = " << body->getSystemID() << " does not contains vertex " << vidx << " in local domain(" << mpisys->getRank() << ")" << "\n";
    std::cout << msg.str() << std::endl;
    return false;
  }

}
//=================================================================================================


//=================================================================================================
/*
 *!\brief The function checks if the point pos in inside the particles (C++ end point for checkAllParticles) 
 * \param inpr An output parameter that is set to the idx of that particle that contains pos 
 * \param pos An array that cointans the 3d point
 */
//void getPartStructByIdx(int idx, particleData_t *particle) {
//
//  WorldID world = theWorld();
//  World::SizeType widx = static_cast<World::SizeType>(idx);
//
//  BodyID body;
//  if ( widx < world->size() ) {
//    body = world->getBody(static_cast<unsigned int>(widx));
//
//    // TODO: this is a dubious conversion to a smaller type -> fix
//    particle->uniqueIdx = body->getSystemID();
//    particle->localIdx  = idx;
//    particle->systemIdx = -1;
//    particle->time = -1.0;
//    
//    uint64toByteArray(body->getSystemID(), particle->bytes); 
//
//  }
//  else {
//    std::stringstream msg;
//    msg << "Line- " << __LINE__ <<  ": Body index: " << idx << " out of range." << "\n";
//    throw std::out_of_range(msg.str());
//  }
//}
//=================================================================================================

extern "C"
bool map_local_to_system(int lidx, int vidx) {
  return mapLocalToSystem(lidx, vidx);
}

extern "C"
bool map_local_to_system2(int lidx, int vidx) {
  return mapLocalToSystem(lidx, vidx);
}

