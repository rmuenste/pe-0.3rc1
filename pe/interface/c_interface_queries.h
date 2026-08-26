
#include <pe/config/SimulationConfig.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/lubrication/Params.h>
#include <pe/interface/el_optional_api.h>

#include <iostream>
#include <type_traits>
#include <utility>

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
/*!\brief Pushes the resolved CFD mesh width into the lubrication mesh clamp from Fortran.
 *
 * \param dx Pointer to the finest resolved CFD element width (global h_min).
 * \return void
 *
 * Arms the mesh clamp: with lubricationMeshClampFactor_ = c > 0 the effective outer
 * cutoff becomes min(cutoffFactor*aRef, c*dx), so lubrication activates exactly where
 * the CFD mesh stops resolving the squeeze film instead of at a hand-computed per-case
 * cutoff. Called from the CFD layer (ComputeCFL) once the global h_min is known; the
 * value is identical on every rank, so per-rank PE instances stay deterministic.
 * Idempotent; a no-op while lubrication is disabled or the clamp factor is 0.
 */
//=================================================================================================
/*!\brief Reports whether the lubrication add-on master switch is on (1) or off (0).
 *
 * Lets the CFD layer gate its per-step DNS_LUB diagnostic print without touching json
 * parsing on the Fortran side; with the switch off the print is skipped entirely, so
 * lubrication-off runs keep byte-identical stdout.
 */
// Bound to Fortran function get_lubrication_enabled() in
// source/src_particles/dem_query.f90
extern "C" int get_lubrication_enabled()
{
   return pe::lubrication::isEnabled() ? 1 : 0;
}
//=================================================================================================


//=================================================================================================
// Accessor detection for the per-step lubrication stage diagnostics: only collision
// systems that RETAIN the diagnostics (HardContactAndFluid, HCSITS) expose
// lubricationStageDiag(). Detected structurally so this translation unit still compiles
// when the active pe_CONSTRAINT_SOLVER is any other solver (e.g. the EL family) - the
// query then reports zeros.
namespace pe_query_detail {

template <typename CS, typename = void>
struct HasLubStageDiag : std::false_type {};

template <typename CS>
struct HasLubStageDiag<CS, std::void_t<decltype(std::declval<const CS&>().lubricationStageDiag())>>
    : std::true_type {};

// Template so the if-constexpr false branch is genuinely discarded (in a plain
// function both branches would be instantiated and an EL-solver build would fail
// to compile the accessor call).
template <typename CS>
inline void readLubStageDiag(const CS& cs, double *totalForce, double *dissipation,
                             double *maxNormalImpulse, int *numContacts, int *numSaturated)
{
   if constexpr( HasLubStageDiag<CS>::value ) {
      const auto& d = cs.lubricationStageDiag();
      *totalForce       = static_cast<double>( d.totalForce );
      *dissipation      = static_cast<double>( d.dissipation );
      *maxNormalImpulse = static_cast<double>( d.maxNormalImpulse );
      *numContacts      = static_cast<int>( d.numContacts );
      *numSaturated     = static_cast<int>( d.numSaturated );
   }
   else {
      (void)cs;
      (void)totalForce; (void)dissipation; (void)maxNormalImpulse;
      (void)numContacts; (void)numSaturated;
   }
}

}  // namespace pe_query_detail


/*!\brief Reads the diagnostics of the most recent lubrication stage invocation.
 *
 * \param totalForce Sum of applied lubrication force magnitudes over the last macro step.
 * \param dissipation Sum of J.g + L.w over the step (must be <= 0; sign-convention check).
 * \param maxNormalImpulse Largest normal lubrication impulse applied.
 * \param numContacts Lubrication contacts acted on.
 * \param numSaturated Contacts inside the h_c saturation freeze.
 *
 * For a single sphere-wall pair (the G2 Brenner benchmark) totalForce IS the applied
 * lubrication force and maxNormalImpulse/dt its normal component. Zeros while the
 * add-on is off or the active solver does not run the stage.
 */
// Bound to Fortran subroutine get_lubrication_stage_diag(...) in
// source/src_particles/dem_query.f90
extern "C" void get_lubrication_stage_diag(double *totalForce, double *dissipation,
                                           double *maxNormalImpulse, int *numContacts,
                                           int *numSaturated)
{
   *totalForce       = 0.0;
   *dissipation      = 0.0;
   *maxNormalImpulse = 0.0;
   *numContacts      = 0;
   *numSaturated     = 0;

   pe_query_detail::readLubStageDiag( *pe::theCollisionSystem(), totalForce, dissipation,
                                      maxNormalImpulse, numContacts, numSaturated );
}
//=================================================================================================


// Bound to Fortran subroutine set_lubrication_mesh_dx(dx) in
// source/src_particles/dem_query.f90
extern "C" void set_lubrication_mesh_dx(double *dx)
{
   pe::lubrication::setMeshDx( static_cast<pe::real>(*dx) );

   // One-time arming announcement on the representative rank only (in PE serial mode
   // every CFD rank runs this on its own PE instance): grep-able run evidence that the
   // clamp is live. The opposite failure - clamp configured but dx never pushed - warns
   // loudly in applyLubricationStage.
   if( pe::lubrication::isEnabled() &&
       pe::lubrication::getMeshClampFactor() > pe::real(0) && *dx > 0.0 &&
       pe::SimulationConfig::getInstance().getCfdRank() == 1 ) {
      static bool announced = false;
      if( !announced ) {
         announced = true;
         std::cout << "PE_LUB_MESHDX armed: dx= " << *dx
                   << " clamp_factor= " << pe::lubrication::getMeshClampFactor()
                   << " activation_gap= "
                   << pe::lubrication::getMeshClampFactor() * (*dx) << std::endl;
      }
   }
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
  pe::elLubricationVirial(*pe::theCollisionSystem(), sigma);
}

//=================================================================================================

/*
 *!\brief Rank-local count of lubrication pair evaluations this macro step.
 */
extern "C" int getElLubricationPairs() {
  return static_cast<int>(pe::elLubricationPairs(*pe::theCollisionSystem()));
}

//=================================================================================================

/*
 *!\brief Rank-local NET momentum folded by the lubrication sweep this macro
 * step; the MPI sum across ranks measures pair one-sidedness (must be ~0).
 */
extern "C" void getElLubricationImpulse(double *dp) {
  pe::elLubricationImpulse(*pe::theCollisionSystem(), dp);
}

//=================================================================================================

/*
 *!\brief Rank-local impulse virial of the converged sphere-sphere PGS contact
 * impulses this macro step, Sum p (x) r12 (row-major 3x3); same normalization
 * as the lubrication virial (stress = -virial/(dt_macro*V) after MPI sum).
 */
extern "C" void getElContactVirial(double *sigma) {
  pe::elContactVirial(*pe::theCollisionSystem(), sigma);
}

//=================================================================================================

/*
 *!\brief Rank-local count of sphere-sphere contacts accumulated into the
 * contact virial this macro step.
 */
extern "C" int getElContactVirialPairs() {
  return static_cast<int>(pe::elContactVirialPairs(*pe::theCollisionSystem()));
}

//=================================================================================================

/*
 *!\brief Rank-local lubrication impulse the spheres exert on z-wall `wall`
 * (0 = bottom, 1 = top) this macro step; MPI-SUM counts each sphere once
 * (owner-only evaluation). Wall shear stress: tau = Sum(dp_x)/(dt_macro*A).
 */
extern "C" void getElWallLubImpulse(int wall, double *dp) {
  pe::elWallLubImpulse(*pe::theCollisionSystem(), wall, dp);
}

//=================================================================================================

/*
 *!\brief Rank-local converged PGS contact impulse ON z-wall `wall`
 * (0 = bottom, 1 = top) this macro step; each contact counted once.
 */
extern "C" void getElWallContactImpulse(int wall, double *dp) {
  pe::elWallContactImpulse(*pe::theCollisionSystem(), wall, dp);
}

//=================================================================================================

/*
 *!\brief Rank-local count of sphere-wall lubrication evaluations this macro step.
 */
extern "C" int getElWallLubPairs() {
  return static_cast<int>(pe::elWallLubPairs(*pe::theCollisionSystem()));
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
