//=================================================================================================
/*!\file pe/interface/cfd_queries.h
 *  \brief The C query and get/set functions FeatFloWer binds to.
 *
 *  Together with cfd_entry_points.h (setup and stepping) and the extern "C" functions declared
 *  in object_queries.h and fbm_gate_probe.h, this is the complete ABI the CFD side uses: the
 *  Fortran interface blocks in source/src_particles/dem_query.f90 and friends bind to these
 *  names. Keep names and signatures stable; the implementations are thin wrappers in
 *  src/interface/cfd_queries.cpp over the C++ query API of object_queries.h.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_CFD_QUERIES_H_
#define _PE_INTERFACE_CFD_QUERIES_H_

#include <pe/interface/object_queries.h>

//=================================================================================================
//  Lubrication add-on
//=================================================================================================
extern "C" void   set_lubrication_threshold(double *threshold);
extern "C" int    get_lubrication_enabled();
extern "C" void   get_lubrication_stage_diag(double *totalForce, double *dissipation,
                                             double *maxNormalImpulse, int *numContacts,
                                             int *numSaturated);
extern "C" void   set_lubrication_mesh_dx(double *dx);

//=================================================================================================
//  Particle counts and per-particle properties
//=================================================================================================
extern "C" int    getTotalParticles();
extern "C" int    getNumParticles();
extern "C" int    getNumRemParticles();
extern "C" double getParticleRadius(int *idx);
extern "C" bool   isTypeSphere(int *idx);
extern "C" bool   isTypeEllipsoid(int *idx);
extern "C" void   getParticleOrientation(int *idx, double axis[3]);
extern "C" bool   check_rem_id(int fbmid, int id);

//=================================================================================================
//  Euler-Lagrange coupling diagnostics
//=================================================================================================
extern "C" int    getElSubsteps();
extern "C" double getElStepsize();
extern "C" int    getElContactCount();
extern "C" double getElMaxPenetration();
extern "C" void   getElLubricationVirial(double *sigma);
extern "C" int    getElLubricationPairs();
extern "C" void   getElLubricationImpulse(double *dp);
extern "C" void   getElContactVirial(double *sigma);
extern "C" int    getElContactVirialPairs();
extern "C" void   getElWallLubImpulse(int wall, double *dp);
extern "C" void   getElWallContactImpulse(int wall, double *dp);
extern "C" int    getElWallLubPairs();

//=================================================================================================
//  Point containment (FBM alpha field) and index maps
//=================================================================================================
extern "C" bool   pointInsideObject(int idx, double pos[3]);
extern "C" bool   pointInsideRemObject(int idx, double pos[3]);
extern "C" bool   checkAllParticles(int vidx, int *inpr, double pos[3], short int bytes[8]);
extern "C" bool   verifyAllParticles(int vidx, int *inpr, double pos[3], short int bytes[8]);
extern "C" bool   checkAllParticlesAccelerated(int vidx, int *inpr, double pos[3], short int bytes[8]);
extern "C" void   rem_particles_index_map(int *idxMap);
extern "C" void   particles_index_map(int *idxMap);

//=================================================================================================
//  Particle state get/set and force synchronization
//=================================================================================================
extern "C" void   sync_forces_();
extern "C" void   getParticle2(int idx, particleData_t *particle);
extern "C" void   setParticle2(particleData_t *particle);
extern "C" void   setRemoteParticle2(particleData_t *particle);
extern "C" void   getRemoteParticle2(int idx, particleData_t *particle);

//=================================================================================================
//  DistanceMap debugging
//=================================================================================================
#ifdef PE_USE_CGAL
extern "C" void   exportOwnedDistanceMaps();
#endif
extern "C" void   debugDistanceMapBBoxes();

#endif  // _PE_INTERFACE_CFD_QUERIES_H_
