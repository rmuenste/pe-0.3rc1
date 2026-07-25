
#include <pe/config/SimulationConfig.h>
#include <pe/core/CollisionSystem.h>

// Bound to Fortran function check_rem_id(fbmid, id) in
// source/src_particles/dem_query.f90 line 226
extern "C"
bool check_rem_id(int fbmid, int id) {

  return checkRemoteFBM(fbmid, id); 
}
//=================================================================================================


//=================================================================================================
/*!\brief Sets the lubrication threshold distance from Fortran.
 *
 * \param threshold Pointer to the lubrication threshold distance value.
 * \return void
 *
 * This function allows Fortran code to set the runtime lubrication threshold distance,
 * which determines the maximum separation at which lubrication forces are active.
 */
extern "C" void set_lubrication_threshold(double *threshold)
{
   pe::lubrication::setLubricationThreshold( static_cast<pe::real>(*threshold) );
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


//=================================================================================================
/*
 *!\brief Returns the configured PE sub-step count (n_sub = max(1, substeps_)).
 *
 * Single source of truth for the Euler-Lagrange feedback: the Fortran side
 * reproduces PE's sub-cycled semi-implicit drag (n_sub steps of dt/n_sub on the
 * frozen hydro state, see sim_setup.cpp) to spread the drag impulse PE actually
 * applies. Exporting n_sub here keeps the two sides from drifting apart.
 */
// Bound to Fortran function getElSubsteps() in source/src_particles/dem_query.f90
extern "C" int getElSubsteps() {
  const int s = pe::SimulationConfig::getInstance().getSubsteps();
  return s > 1 ? s : 1;
}

//=================================================================================================

/*
 *!\brief Returns the configured PE macro timestep.
 *
 * The Euler-Lagrange CFD step must match this macro step exactly; substeps only
 * subdivide PE integration internally.
 */
extern "C" double getElStepsize() {
  return static_cast<double>(pe::SimulationConfig::getInstance().getStepsize());
}

//=================================================================================================

/*
 *!\brief Returns the number of PE contacts from the last collision response.
 */
extern "C" int getElContactCount() {
  return static_cast<int>(pe::theCollisionSystem()->getNumberOfContacts());
}

//=================================================================================================

/*
 *!\brief Returns the maximum penetration from the last PE collision response.
 */
extern "C" double getElMaxPenetration() {
  return static_cast<double>(pe::theCollisionSystem()->getMaximumPenetration());
}

//=================================================================================================

/*
 *!\brief Rank-local lubrication impulse virial of the current macro step
 * (9 components, row-major): Sum w*dt*F (x) r12 over pairs and substeps,
 * weight 0.5 per locally-owned pair member so the MPI sum counts each pair
 * once. Particle-phase stress: sigma = -virial/(dt_macro*V).
 */
extern "C" void getElLubricationVirial(double *sigma) {
  pe::theCollisionSystem()->getElLubricationVirial(sigma);
}

//=================================================================================================

/*
 *!\brief Rank-local count of lubrication pair evaluations this macro step.
 */
extern "C" int getElLubricationPairs() {
  return static_cast<int>(pe::theCollisionSystem()->getElLubricationPairs());
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


#ifdef PE_USE_CGAL
//=================================================================================================
/*
 *!\brief Export DistanceMap data from all owned TriangleMesh objects to VTI files
 *
 * This function iterates through all rigid bodies owned by the current domain/world,
 * identifies TriangleMesh objects that have DistanceMap acceleration enabled,
 * and exports their DistanceMap data to VTI files for visualization.
 *
 * Output files are named: "distancemap_rank{rank}_body{bodyid}.vti"
 * where {rank} is the MPI process rank and {bodyid} is the body ID.
 */
//=================================================================================================
// Bound to Fortran function exportOwnedDistanceMaps() in
// source/src_particles/dem_query.f90 (to be added)
extern "C" void exportOwnedDistanceMaps() {
  exportDistanceMapsFromBodies();
}
//=================================================================================================
#endif


//=================================================================================================
/*
 *!\brief Debug bounding boxes of TriangleMesh objects with DistanceMap
 * 
 * This function iterates through all rigid bodies owned by the current process,
 * identifies TriangleMesh objects that have DistanceMap acceleration enabled,
 * and outputs detailed bounding box information to the console for debugging.
 */
//=================================================================================================
// Bound to Fortran function debugDistanceMapBBoxes() in
// source/src_particles/dem_query.f90 (to be added)
extern "C" void debugDistanceMapBBoxes() {
  debugDistanceMapBoundingBoxes();
}
//=================================================================================================
